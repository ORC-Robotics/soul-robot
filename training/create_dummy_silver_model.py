from pathlib import Path

import numpy as np
import tensorflow as tf


PROJECT_ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = PROJECT_ROOT / "assets" / "models"
OUTPUT_PATH = OUTPUT_DIR / "silver_classifier.tflite"

INPUT_WIDTH = 128
INPUT_HEIGHT = 128
CLASS_NAMES = ("black", "other", "silver")


def create_model():
    inputs = tf.keras.Input(
        shape=(INPUT_HEIGHT, INPUT_WIDTH, 3),
        dtype=tf.float32,
        name="image",
    )

    x = tf.keras.layers.Rescaling(
        1.0 / 255.0,
        name="normalize",
    )(inputs)

    x = tf.keras.layers.GlobalAveragePooling2D(
        name="global_average",
    )(x)

    outputs = tf.keras.layers.Dense(
        len(CLASS_NAMES),
        activation="softmax",
        name="classes",
    )(x)

    model = tf.keras.Model(
        inputs=inputs,
        outputs=outputs,
        name="dummy_silver_classifier",
    )

    kernel, bias = model.get_layer("classes").get_weights()

    kernel[:] = 0.0
    bias[:] = np.array(
        [0.0, 0.5, 1.0],
        dtype=np.float32,
    )

    model.get_layer("classes").set_weights(
        [kernel, bias]
    )

    return model


def convert_to_tflite(model):
    converter = tf.lite.TFLiteConverter.from_keras_model(
        model
    )

    return converter.convert()


def main():
    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    model = create_model()
    tflite_model = convert_to_tflite(model)

    OUTPUT_PATH.write_bytes(tflite_model)

    print(f"Modelo criado: {OUTPUT_PATH}")
    print(f"Tamanho: {len(tflite_model)} bytes")
    print(f"Classes: {CLASS_NAMES}")


if __name__ == "__main__":
    main()