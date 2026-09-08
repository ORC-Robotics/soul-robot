"""Testes da decisão de presença da faixa prata."""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from silver_classifier import SilverClassification
from vision.silver_detection import SilverLineDetector


class FixedClassifier:
    def __init__(self, classification):
        self.classification = classification
        self.received_frame = None

    def classify(self, frame):
        self.received_frame = frame
        return self.classification


def classification(black, other, silver):
    return SilverClassification(
        black=black,
        other=other,
        silver=silver,
        preprocess_ms=1.0,
        inference_ms=2.0,
        total_ms=3.0,
    )


class SilverLineDetectorTest(unittest.TestCase):
    def test_detects_silver_with_confidence_and_margin(self):
        detector = SilverLineDetector(FixedClassifier(classification(0.05, 0.10, 0.85)))

        result = detector.detect(np.zeros((10, 20, 3), dtype=np.uint8))

        self.assertTrue(result.detected)
        self.assertEqual(result.label, "silver")
        self.assertAlmostEqual(result.silver_margin, 0.75)

    def test_rejects_silver_below_minimum_confidence(self):
        detector = SilverLineDetector(FixedClassifier(classification(0.20, 0.21, 0.59)))

        result = detector.detect(np.zeros((10, 20, 3), dtype=np.uint8))

        self.assertFalse(result.detected)
        self.assertEqual(result.label, "silver")

    def test_rejects_silver_without_margin_over_other(self):
        detector = SilverLineDetector(
            FixedClassifier(classification(0.02, 0.14, 0.84)),
            minimum_silver_margin=0.75,
        )

        result = detector.detect(np.zeros((10, 20, 3), dtype=np.uint8))

        self.assertFalse(result.detected)
        self.assertAlmostEqual(result.competing_probability, 0.14)

    def test_rejects_non_silver_top_class(self):
        detector = SilverLineDetector(FixedClassifier(classification(0.80, 0.05, 0.15)))

        result = detector.detect(np.zeros((10, 20, 3), dtype=np.uint8))

        self.assertFalse(result.detected)
        self.assertEqual(result.label, "black")

    def test_applies_same_normalized_roi_expected_by_training(self):
        classifier = FixedClassifier(classification(0.05, 0.10, 0.85))
        detector = SilverLineDetector(classifier, roi=(0.25, 0.50, 0.75, 1.0))
        frame = np.zeros((10, 20, 3), dtype=np.uint8)

        detector.detect(frame)

        self.assertEqual(classifier.received_frame.shape, (5, 10, 3))
        self.assertTrue(classifier.received_frame.flags["C_CONTIGUOUS"])

    def test_rejects_invalid_roi_and_thresholds(self):
        classifier = FixedClassifier(classification(0.05, 0.10, 0.85))

        with self.assertRaises(ValueError):
            SilverLineDetector(classifier, roi=(0.8, 0.0, 0.2, 1.0))
        with self.assertRaises(ValueError):
            SilverLineDetector(classifier, minimum_silver_confidence=1.1)


if __name__ == "__main__":
    unittest.main()
