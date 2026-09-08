"""Testes da decisão de presença da faixa prata."""

from pathlib import Path
import sys
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from silver_classifier import SilverClassification
from vision.silver_detection import SilverLineDetector, SilverShadowMonitor


class FixedClassifier:
    def __init__(self, classification):
        self.classification = classification
        self.received_frame = None

    def classify(self, frame):
        self.received_frame = frame
        return self.classification


class FailingDetector:
    def detect(self, frame):
        raise RuntimeError("falha simulada")


class SequenceDetector:
    def __init__(self, results):
        self.results = iter(results)
        self.calls = 0

    def detect(self, frame):
        self.calls += 1
        return next(self.results)


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

    def test_shadow_publishes_diagnostics_without_changing_detection(self):
        detector = SilverLineDetector(
            FixedClassifier(classification(0.05, 0.10, 0.85))
        )
        monitor = SilverShadowMonitor(detector)

        with mock.patch("builtins.print"):
            status = monitor.process(
                np.zeros((10, 20, 3), dtype=np.uint8),
                sequence=7,
                timestamp=123.5,
            )

        self.assertTrue(status["silverShadowAvailable"])
        self.assertTrue(status["silverShadowDetected"])
        self.assertEqual(status["silverShadowLabel"], "silver")
        self.assertAlmostEqual(status["silverShadowProbability"], 0.85)
        self.assertAlmostEqual(status["silverShadowMargin"], 0.75)
        self.assertEqual(status["silverShadowSequence"], 7)
        self.assertEqual(status["silverShadowTimestamp"], 123.5)

    def test_shadow_failure_disables_only_optional_detector(self):
        monitor = SilverShadowMonitor(FailingDetector())
        frame = np.zeros((10, 20, 3), dtype=np.uint8)

        with mock.patch("builtins.print") as print_mock:
            first_status = monitor.process(frame, 1, 10.0)
            second_status = monitor.process(frame, 2, 11.0)

        self.assertFalse(first_status["silverShadowAvailable"])
        self.assertFalse(second_status["silverShadowAvailable"])
        self.assertIn("falha simulada", first_status["silverShadowError"])
        self.assertIsNone(monitor.detector)
        print_mock.assert_called_once()

    def test_shadow_uses_fast_cadence_and_confirms_four_positive_frames(self):
        positive_detector = SilverLineDetector(
            FixedClassifier(classification(0.05, 0.10, 0.85))
        )
        positive_result = positive_detector.detect(
            np.zeros((10, 20, 3), dtype=np.uint8)
        )
        detector = SequenceDetector([positive_result] * 4)
        monitor = SilverShadowMonitor(detector)
        frame = np.zeros((10, 20, 3), dtype=np.uint8)

        with mock.patch("builtins.print"):
            first = monitor.process(frame, 1, 10.0, monotonic_time=0.0)
            skipped = monitor.process(frame, 2, 10.05, monotonic_time=0.05)
            second = monitor.process(frame, 3, 10.11, monotonic_time=0.11)
            third = monitor.process(frame, 4, 10.22, monotonic_time=0.22)
            confirmed = monitor.process(frame, 5, 10.33, monotonic_time=0.33)

        self.assertEqual(first["silverConfirmationFrames"], 1)
        self.assertEqual(first["silverInferenceTargetFps"], 15.0)
        self.assertIs(skipped, first)
        self.assertEqual(second["silverConfirmationFrames"], 2)
        self.assertEqual(third["silverConfirmationFrames"], 3)
        self.assertTrue(confirmed["courseMarkerConfirmed"])
        self.assertEqual(confirmed["courseMarker"], "GRAY")
        self.assertEqual(confirmed["silverInferenceTargetFps"], 6.0)
        self.assertEqual(detector.calls, 4)

    def test_shadow_negative_resets_confirmation_and_search_cadence(self):
        fixed = FixedClassifier(classification(0.05, 0.10, 0.85))
        silver_detector = SilverLineDetector(fixed)
        positive = silver_detector.detect(np.zeros((10, 20, 3), dtype=np.uint8))
        fixed.classification = classification(0.80, 0.05, 0.15)
        negative = silver_detector.detect(np.zeros((10, 20, 3), dtype=np.uint8))
        monitor = SilverShadowMonitor(SequenceDetector([positive, negative]))
        frame = np.zeros((10, 20, 3), dtype=np.uint8)

        with mock.patch("builtins.print"):
            monitor.process(frame, 1, 10.0, monotonic_time=0.0)
            reset = monitor.process(frame, 2, 10.11, monotonic_time=0.11)

        self.assertEqual(reset["silverConfirmationFrames"], 0)
        self.assertFalse(reset["courseMarkerConfirmed"])
        self.assertEqual(reset["courseMarker"], "NONE")
        self.assertEqual(reset["silverInferenceTargetFps"], 6.0)


if __name__ == "__main__":
    unittest.main()
