from argparse import ArgumentParser
from pathlib import Path
import csv
import json
import random

import numpy as np
import tensorflow as tf


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATASET_ROOT = PROJECT_ROOT / "dataset" / "raw"
MODELS_DIR = PROJECT_ROOT / "assets" / "models"
OUTPUTS_DIR = PROJECT_ROOT / "training_outputs"

CLASS_NAMES = ("black", "other", "silver")
CLASS_TO_INDEX = {name: index for index, name in enumerate(CLASS_NAMES)}

INPUT_WIDTH = 128
INPUT_HEIGHT = 128
BATCH_SIZE = 32

TRAIN_RATIO = 0.70
VALIDATION_RATIO = 0.15
TEST_RATIO = 0.15

DEFAULT_SEED = 42

PHASE1_EPOCHS = 20
PHASE1_LEARNING_RATE = 1e-3

FINETUNE_EPOCHS = 15
FINETUNE_LEARNING_RATE = 1e-5
FINETUNE_LAYERS = 30

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png"}


def parse_arguments():
    parser = ArgumentParser(description="Treina o classificador de faixa prata da OBR.")
    parser.add_argument("--camera", choices=("forward", "down"), required=True, help="Câmera cujo modelo será treinado.")
    parser.add_argument("--dataset-root", type=Path, default=DATASET_ROOT, help="Diretório contendo forward/ e down/.")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED, help="Seed usada para dividir as sessões.")
    parser.add_argument("--fine-tune", action="store_true", help="Executa uma segunda fase descongelando parte da MobileNet.")
    parser.add_argument(
        "--roi",
        nargs=4,
        type=float,
        metavar=("LEFT", "TOP", "RIGHT", "BOTTOM"),
        default=(0.0, 0.0, 1.0, 1.0),
        help="ROI normalizada. Exemplo: --roi 0.1 0.3 0.9 1.0",
    )
    return parser.parse_args()


def validate_roi(roi):
    left, top, right, bottom = roi

    if not (0.0 <= left < right <= 1.0 and 0.0 <= top < bottom <= 1.0):
        raise ValueError("ROI inválida. Use LEFT TOP RIGHT BOTTOM normalizados entre 0 e 1.")


def discover_sessions(camera_dir):
    if not camera_dir.is_dir():
        raise FileNotFoundError(f"Dataset da câmera não encontrado: {camera_dir}")

    sessions = sorted(path for path in camera_dir.iterdir() if path.is_dir())

    if len(sessions) < 3:
        raise RuntimeError("São necessárias pelo menos 3 sessões independentes para treino, validação e teste.")

    return sessions


def split_sessions(sessions, seed):
    sessions = list(sessions)
    random.Random(seed).shuffle(sessions)

    total = len(sessions)
    test_count = max(1, round(total * TEST_RATIO))
    validation_count = max(1, round(total * VALIDATION_RATIO))

    while total - test_count - validation_count < 1:
        if test_count > validation_count:
            test_count -= 1
        else:
            validation_count -= 1

    test_sessions = sessions[:test_count]
    validation_sessions = sessions[test_count:test_count + validation_count]
    train_sessions = sessions[test_count + validation_count:]

    return train_sessions, validation_sessions, test_sessions


def collect_samples(sessions):
    samples = []

    for session in sessions:
        for class_name in CLASS_NAMES:
            class_dir = session / class_name

            if not class_dir.is_dir():
                continue

            for path in sorted(class_dir.iterdir()):
                if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS:
                    samples.append((str(path), CLASS_TO_INDEX[class_name]))

    return samples


def count_classes(samples):
    counts = {class_name: 0 for class_name in CLASS_NAMES}

    for _, label in samples:
        counts[CLASS_NAMES[label]] += 1

    return counts


def validate_samples(samples, split_name):
    if not samples:
        raise RuntimeError(f"O conjunto {split_name} não possui imagens.")

    counts = count_classes(samples)
    missing = [class_name for class_name, count in counts.items() if count == 0]

    if missing:
        raise RuntimeError(f"O conjunto {split_name} não possui imagens das classes: {', '.join(missing)}")


def decode_image(path, label, roi):
    data = tf.io.read_file(path)
    image = tf.io.decode_image(data, channels=3, expand_animations=False)

    image.set_shape([None, None, 3])
    image = tf.cast(image, tf.float32)

    height = tf.shape(image)[0]
    width = tf.shape(image)[1]
    left, top, right, bottom = roi

    x0 = tf.cast(tf.round(left * tf.cast(width, tf.float32)), tf.int32)
    x1 = tf.cast(tf.round(right * tf.cast(width, tf.float32)), tf.int32)
    y0 = tf.cast(tf.round(top * tf.cast(height, tf.float32)), tf.int32)
    y1 = tf.cast(tf.round(bottom * tf.cast(height, tf.float32)), tf.int32)

    image = image[y0:y1, x0:x1]
    image = tf.image.resize(image, (INPUT_HEIGHT, INPUT_WIDTH), antialias=True)
    image = tf.clip_by_value(image, 0.0, 255.0)

    return image, label


def augment_image(image, label):
    image = tf.image.random_brightness(image, max_delta=18.0)
    image = tf.image.random_contrast(image, lower=0.88, upper=1.12)
    image = tf.clip_by_value(image, 0.0, 255.0)

    return image, label


def create_dataset(samples, roi, training=False):
    paths = [sample[0] for sample in samples]
    labels = [sample[1] for sample in samples]

    dataset = tf.data.Dataset.from_tensor_slices((paths, labels))

    if training:
        dataset = dataset.shuffle(buffer_size=len(samples), seed=DEFAULT_SEED, reshuffle_each_iteration=True)

    dataset = dataset.map(
        lambda path, label: decode_image(path, label, roi),
        num_parallel_calls=tf.data.AUTOTUNE,
    )

    if training:
        dataset = dataset.map(augment_image, num_parallel_calls=tf.data.AUTOTUNE)

    return dataset.batch(BATCH_SIZE).prefetch(tf.data.AUTOTUNE)


def create_model():
    inputs = tf.keras.Input(shape=(INPUT_HEIGHT, INPUT_WIDTH, 3), dtype=tf.float32, name="image")

    backbone = tf.keras.applications.MobileNetV3Small(
        input_shape=(INPUT_HEIGHT, INPUT_WIDTH, 3),
        include_top=False,
        weights="imagenet",
        include_preprocessing=True,
        pooling="avg",
    )

    backbone.trainable = False

    x = backbone(inputs, training=False)
    x = tf.keras.layers.Dropout(0.20, name="dropout")(x)
    outputs = tf.keras.layers.Dense(len(CLASS_NAMES), activation="softmax", name="classes")(x)

    model = tf.keras.Model(inputs=inputs, outputs=outputs, name="silver_classifier")

    return model, backbone


def compile_model(model, learning_rate):
    model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=learning_rate),
        loss=tf.keras.losses.SparseCategoricalCrossentropy(),
        metrics=[tf.keras.metrics.SparseCategoricalAccuracy(name="accuracy")],
    )


def create_callbacks():
    return [
        tf.keras.callbacks.EarlyStopping(monitor="val_loss", patience=5, restore_best_weights=True),
        tf.keras.callbacks.ReduceLROnPlateau(
            monitor="val_loss",
            factor=0.5,
            patience=2,
            min_lr=1e-7,
            verbose=1,
        ),
    ]


def fine_tune_backbone(backbone, layers_to_unfreeze):
    backbone.trainable = True
    cutoff = max(0, len(backbone.layers) - layers_to_unfreeze)

    for index, layer in enumerate(backbone.layers):
        if index < cutoff:
            layer.trainable = False
        elif isinstance(layer, tf.keras.layers.BatchNormalization):
            layer.trainable = False
        else:
            layer.trainable = True


def confusion_matrix(true_labels, predicted_labels):
    matrix = np.zeros((len(CLASS_NAMES), len(CLASS_NAMES)), dtype=np.int64)

    for true_label, predicted_label in zip(true_labels, predicted_labels):
        matrix[int(true_label), int(predicted_label)] += 1

    return matrix


def calculate_metrics(matrix):
    metrics = {}

    total = int(matrix.sum())
    correct = int(np.trace(matrix))
    metrics["accuracy"] = correct / total if total > 0 else 0.0

    for index, class_name in enumerate(CLASS_NAMES):
        true_positive = int(matrix[index, index])
        false_positive = int(matrix[:, index].sum() - true_positive)
        false_negative = int(matrix[index, :].sum() - true_positive)

        precision_denominator = true_positive + false_positive
        recall_denominator = true_positive + false_negative

        precision = true_positive / precision_denominator if precision_denominator else 0.0
        recall = true_positive / recall_denominator if recall_denominator else 0.0
        f1 = 2.0 * precision * recall / (precision + recall) if precision + recall else 0.0

        metrics[class_name] = {
            "precision": precision,
            "recall": recall,
            "f1": f1,
            "truePositive": true_positive,
            "falsePositive": false_positive,
            "falseNegative": false_negative,
        }

    return metrics


def print_confusion_matrix(matrix):
    print("\nMATRIZ DE CONFUSÃO\n")
    print(f"{'REAL / PRED':>13}{'black':>10}{'other':>10}{'silver':>10}")

    for index, class_name in enumerate(CLASS_NAMES):
        print(f"{class_name:>13}{matrix[index, 0]:>10}{matrix[index, 1]:>10}{matrix[index, 2]:>10}")


def save_false_positive_silver(samples, probabilities, output_path):
    rows = []
    silver_index = CLASS_TO_INDEX["silver"]

    for (path, true_label), prediction in zip(samples, probabilities):
        predicted_label = int(np.argmax(prediction))

        if predicted_label == silver_index and true_label != silver_index:
            rows.append(
                {
                    "path": path,
                    "real": CLASS_NAMES[true_label],
                    "silverProbability": float(prediction[silver_index]),
                    "blackProbability": float(prediction[CLASS_TO_INDEX["black"]]),
                    "otherProbability": float(prediction[CLASS_TO_INDEX["other"]]),
                }
            )

    rows.sort(key=lambda row: row["silverProbability"], reverse=True)

    with output_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=["path", "real", "silverProbability", "blackProbability", "otherProbability"],
        )
        writer.writeheader()
        writer.writerows(rows)


def convert_to_tflite(model, output_path):
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    tflite_model = converter.convert()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(tflite_model)

    return len(tflite_model)


def save_split_manifest(output_path, train_sessions, validation_sessions, test_sessions, roi, seed):
    manifest = {
        "seed": seed,
        "classes": CLASS_NAMES,
        "inputSize": [INPUT_WIDTH, INPUT_HEIGHT],
        "roi": list(roi),
        "trainSessions": [session.name for session in train_sessions],
        "validationSessions": [session.name for session in validation_sessions],
        "testSessions": [session.name for session in test_sessions],
    }

    output_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")


def print_split(name, sessions, samples):
    counts = count_classes(samples)

    print(f"\n{name}")
    print("  sessões:", ", ".join(session.name for session in sessions))
    print("  imagens:", len(samples))

    for class_name in CLASS_NAMES:
        print(f"    {class_name}: {counts[class_name]}")


def main():
    args = parse_arguments()

    roi = tuple(args.roi)
    validate_roi(roi)

    camera_dir = args.dataset_root / args.camera
    output_dir = OUTPUTS_DIR / f"silver_{args.camera}"
    output_dir.mkdir(parents=True, exist_ok=True)

    sessions = discover_sessions(camera_dir)
    train_sessions, validation_sessions, test_sessions = split_sessions(sessions, args.seed)

    train_samples = collect_samples(train_sessions)
    validation_samples = collect_samples(validation_sessions)
    test_samples = collect_samples(test_sessions)

    validate_samples(train_samples, "treino")
    validate_samples(validation_samples, "validação")
    validate_samples(test_samples, "teste")

    print_split("TREINO", train_sessions, train_samples)
    print_split("VALIDAÇÃO", validation_sessions, validation_samples)
    print_split("TESTE", test_sessions, test_samples)

    save_split_manifest(
        output_dir / "split.json",
        train_sessions,
        validation_sessions,
        test_sessions,
        roi,
        args.seed,
    )

    train_dataset = create_dataset(train_samples, roi, training=True)
    validation_dataset = create_dataset(validation_samples, roi)
    test_dataset = create_dataset(test_samples, roi)

    model, backbone = create_model()

    print()
    model.summary()

    print("\nFASE 1 — TRANSFER LEARNING")
    print("MobileNet congelada; treinando apenas o classificador.")

    compile_model(model, PHASE1_LEARNING_RATE)

    history_phase1 = model.fit(
        train_dataset,
        validation_data=validation_dataset,
        epochs=PHASE1_EPOCHS,
        callbacks=create_callbacks(),
    )

    history_phase2 = None

    if args.fine_tune:
        print("\nFASE 2 — FINE TUNING")

        fine_tune_backbone(backbone, FINETUNE_LAYERS)
        compile_model(model, FINETUNE_LEARNING_RATE)

        history_phase2 = model.fit(
            train_dataset,
            validation_data=validation_dataset,
            epochs=FINETUNE_EPOCHS,
            callbacks=create_callbacks(),
        )

    print("\nAVALIANDO TESTE INDEPENDENTE...")

    probabilities = model.predict(test_dataset, verbose=1)
    true_labels = np.asarray([label for _, label in test_samples], dtype=np.int64)
    predicted_labels = np.argmax(probabilities, axis=1)

    matrix = confusion_matrix(true_labels, predicted_labels)
    metrics = calculate_metrics(matrix)

    print_confusion_matrix(matrix)
    print(f"\nAccuracy total: {metrics['accuracy']:.4f}\n")
    print("MÉTRICAS")

    for class_name in CLASS_NAMES:
        class_metrics = metrics[class_name]
        print(
            f"{class_name}: precision={class_metrics['precision']:.4f} "
            f"recall={class_metrics['recall']:.4f} "
            f"f1={class_metrics['f1']:.4f}"
        )

    silver_metrics = metrics["silver"]

    print(f"\nSILVER FALSE POSITIVES: {silver_metrics['falsePositive']}")
    print(f"SILVER FALSE NEGATIVES: {silver_metrics['falseNegative']}")

    model_path = output_dir / "silver_classifier.keras"
    model.save(model_path)

    metrics_path = output_dir / "metrics.json"
    metrics_path.write_text(
        json.dumps(
            {
                "camera": args.camera,
                "metrics": metrics,
                "confusionMatrix": matrix.tolist(),
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    save_false_positive_silver(
        test_samples,
        probabilities,
        output_dir / "false_positive_silver.csv",
    )

    history = {
        "phase1": history_phase1.history,
        "phase2": history_phase2.history if history_phase2 else None,
    }

    (output_dir / "history.json").write_text(json.dumps(history, indent=2), encoding="utf-8")

    tflite_path = MODELS_DIR / f"silver_{args.camera}.tflite"
    model_size = convert_to_tflite(model, tflite_path)

    print("\nTREINAMENTO CONCLUÍDO")
    print(f"Modelo Keras: {model_path}")
    print(f"Modelo TFLite: {tflite_path}")
    print(f"Tamanho TFLite: {model_size / 1024 / 1024:.2f} MB")
    print(f"Relatório: {metrics_path}")


if __name__ == "__main__":
    main()