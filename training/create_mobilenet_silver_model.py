from pathlib import Path

import tensorflow as tf


PROJECT_ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = PROJECT_ROOT / "assets" / "models"
OUTPUT_PATH = OUTPUT_DIR / "silver_classifier.tflite"

INPUT_WIDTH = 128
INPUT_HEIGHT = 128
CLASS_COUNT = 3


def create_model():
    inputs = tf.keras.Input(
        shape=(INPUT_HEIGHT, INPUT_WIDTH, 3),
        dtype=tf.float32,
        name="image",
    )

    backbone = tf.keras.applications.MobileNetV3Small(
        input_shape=(INPUT_HEIGHT, INPUT_WIDTH, 3),
        include_top=False,
        weights="imagenet",
        include_preprocessing=True,
        pooling="avg",
    )

    backbone.trainable = False

    x = backbone(inputs, training=False)

    outputs = tf.keras.layers.Dense(
        CLASS_COUNT,
        activation="softmax",
        name="classes",
    )(x)

    return tf.keras.Model(
        inputs=inputs,
        outputs=outputs,
        name="silver_classifier",
    )


def convert_to_tflite(model):
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    return converter.convert()


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    model = create_model()

    print()
    model.summary()
    print()

    tflite_model = convert_to_tflite(model)
    OUTPUT_PATH.write_bytes(tflite_model)

    print(f"Modelo criado: {OUTPUT_PATH}")
    print(f"Tamanho: {len(tflite_model) / 1024 / 1024:.2f} MB")


if __name__ == "__main__":
    main()