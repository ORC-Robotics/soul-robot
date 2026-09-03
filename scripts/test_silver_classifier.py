from pathlib import Path

import numpy as np

from silver_classifier import SilverClassifier


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = (
    PROJECT_ROOT
    / "assets"
    / "models"
    / "silver_classifier.tflite"
)


def main():
    classifier = SilverClassifier(
        MODEL_PATH,
        num_threads=2,
    )

    print(
        "Entrada do modelo:",
        classifier.input_size,
    )

    test_frame = np.full(
        (480, 640, 3),
        127,
        dtype=np.uint8,
    )

    result = classifier.classify(test_frame)

    print()
    print("Resultado:")
    print(f"  black : {result.black:.4f}")
    print(f"  other : {result.other:.4f}")
    print(f"  silver: {result.silver:.4f}")
    print()
    print(f"Classe:     {result.label}")
    print(f"Confiança:  {result.confidence:.4f}")
    print()
    print(
        f"Preprocess: {result.preprocess_ms:.3f} ms"
    )
    print(
        f"Inferência: {result.inference_ms:.3f} ms"
    )
    print(
        f"Total:      {result.total_ms:.3f} ms"
    )


if __name__ == "__main__":
    main()