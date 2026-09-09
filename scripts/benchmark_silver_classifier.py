from pathlib import Path
import statistics

import numpy as np

from silver_classifier import SilverClassifier


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = (
    PROJECT_ROOT
    / "assets"
    / "models"
    / "silver_down.tflite"
)

WARMUP_RUNS = 10
BENCHMARK_RUNS = 100


def percentile(values, percentile_value):
    ordered = sorted(values)
    index = round(
        (percentile_value / 100.0)
        * (len(ordered) - 1)
    )
    return ordered[index]


def main():
    classifier = SilverClassifier(
        MODEL_PATH,
        num_threads=2,
    )

    frame = np.random.randint(
        0,
        256,
        (480, 640, 3),
        dtype=np.uint8,
    )

    print(
        f"Modelo: {classifier.input_width}x"
        f"{classifier.input_height}"
    )

    print(
        f"Aquecendo com {WARMUP_RUNS} execuções..."
    )

    for _ in range(WARMUP_RUNS):
        classifier.classify(frame)

    preprocess_times = []
    inference_times = []
    total_times = []

    print(
        f"Medindo {BENCHMARK_RUNS} execuções..."
    )

    for _ in range(BENCHMARK_RUNS):
        result = classifier.classify(frame)

        preprocess_times.append(
            result.preprocess_ms
        )
        inference_times.append(
            result.inference_ms
        )
        total_times.append(
            result.total_ms
        )

    print()
    print("PREPROCESSAMENTO")
    print(
        f"média: {statistics.mean(preprocess_times):.3f} ms"
    )
    print(
        f"mín:   {min(preprocess_times):.3f} ms"
    )
    print(
        f"máx:   {max(preprocess_times):.3f} ms"
    )
    print(
        f"p95:   {percentile(preprocess_times, 95):.3f} ms"
    )

    print()
    print("INFERÊNCIA")
    print(
        f"média: {statistics.mean(inference_times):.3f} ms"
    )
    print(
        f"mín:   {min(inference_times):.3f} ms"
    )
    print(
        f"máx:   {max(inference_times):.3f} ms"
    )
    print(
        f"p95:   {percentile(inference_times, 95):.3f} ms"
    )

    print()
    print("TOTAL")
    print(
        f"média: {statistics.mean(total_times):.3f} ms"
    )
    print(
        f"mín:   {min(total_times):.3f} ms"
    )
    print(
        f"máx:   {max(total_times):.3f} ms"
    )
    print(
        f"p95:   {percentile(total_times, 95):.3f} ms"
    )


if __name__ == "__main__":
    main()
