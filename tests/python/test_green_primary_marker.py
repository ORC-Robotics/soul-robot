"""Regressões do marcador lateral interrompido por outro verde no quadro."""

from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from vision.green_detection import (
    GreenObservationTracker,
    analyze_green_marker_contours,
    build_green_status,
    confirmed_green_path_black_valid,
    green_geometry_reason,
    green_marker_roi_geometry,
)


def rectangle(x, y, width=50, height=50):
    return np.array([[[x, y]], [[x + width, y]],
                     [[x + width, y + height]], [[x, y + height]]], np.int32)


def primary_mask(contour, direction):
    mask = np.zeros((360, 480), np.uint8)
    geometry = green_marker_roi_geometry(contour, 480)
    x1, y1, x2, y2 = geometry["upper_roi"]
    mask[max(0, y1):y2, x1:x2] = 255
    left, right = geometry["marker_horizontal_bounds"]
    track_left = right + 5 if direction == "ESQUERDA" else left - 25
    mask[int(np.min(geometry["box"][:, 1])):,
         track_left:track_left + 20] = 255
    return mask


class GreenPrimaryMarkerTest(unittest.TestCase):
    def observe(self, tracker, sequence, contours, mask, at=None):
        global_result = analyze_green_marker_contours(contours, mask)
        result = tracker.resolve_primary_marker(global_result, mask)
        publication = tracker.update(sequence, result["interpretation"],
                                     sequence / 10 if at is None else at)
        return global_result, result, publication

    def test_second_marker_arriving_before_first_side_does_not_block_confirmation(self):
        for direction in ("ESQUERDA", "DIREITA"):
            for reverse_order in (False, True):
                with self.subTest(direction=direction, reverse_order=reverse_order):
                    tracker = GreenObservationTracker()
                    first, other = rectangle(220, 190), rectangle(80, 0)
                    initial_mask = primary_mask(first, direction)
                    x1, y1, x2, y2 = green_marker_roi_geometry(first, 480)["upper_roi"]
                    initial_mask[y1:y2, x1:x2] = 0
                    _, result, publication = self.observe(tracker, 1, [first], initial_mask)
                    self.assertEqual(result["interpretation"], "VERDE_FALSO")
                    self.assertFalse(publication[1])
                    self.assertIsNotNone(tracker.primary_marker_bounds)
                    self.assertEqual(tracker.primary_marker_side, "SEM_DECISAO")
                    contours = [other, first] if reverse_order else [first, other]
                    mask = primary_mask(first, direction)
                    global_result, result, publication = self.observe(tracker, 2, contours, mask)
                    self.assertEqual(global_result["interpretation"], "AMBIGUO")
                    self.assertEqual(result["interpretation"], direction)
                    self.assertFalse(result["pair_compatible"])
                    self.assertFalse(publication[1])
                    _, _, publication = self.observe(tracker, 3, contours, mask)
                    self.assertEqual(publication[:2], (direction, True))

    def test_early_identity_without_upper_black_never_confirms_a_side(self):
        tracker = GreenObservationTracker()
        first, other = rectangle(220, 190), rectangle(80, 0)
        mask = np.zeros((360, 480), np.uint8)
        self.observe(tracker, 1, [first], mask)
        for sequence in range(2, 7):
            _, result, publication = self.observe(tracker, sequence, [other, first], mask)
            self.assertFalse(result["path_black_valid"])
            self.assertNotIn(publication[0], ("ESQUERDA", "DIREITA", "RETORNO_180"))
        self.assertEqual(tracker.primary_marker_side, "SEM_DECISAO")

    def test_size_change_recovers_primary_before_side_is_known(self):
        for direction in ("ESQUERDA", "DIREITA"):
            with self.subTest(direction=direction):
                tracker = GreenObservationTracker()
                initial = rectangle(220, 190)
                self.observe(tracker, 1, [initial], np.zeros((360, 480), np.uint8))
                grown, other = rectangle(195, 140, 100, 100), rectangle(80, 0)
                mask = primary_mask(grown, direction)
                _, result, publication = self.observe(tracker, 2, [other, grown], mask)
                self.assertEqual(result["primary_match_method"], "size_change")
                self.assertLess(result["primary_best_iou"], 0.35)
                self.assertEqual(result["primary_match_count"], 1)
                self.assertEqual(result["interpretation"], direction)
                self.assertFalse(publication[1])
                _, result, publication = self.observe(tracker, 3, [grown, other], mask)
                self.assertEqual(result["primary_match_method"], "iou")
                self.assertEqual(publication[:2], (direction, True))

    def test_size_change_does_not_choose_between_two_matching_markers(self):
        tracker = GreenObservationTracker()
        first, grown = rectangle(220, 190), rectangle(195, 140, 100, 100)
        self.observe(tracker, 1, [first], np.zeros((360, 480), np.uint8))
        mask = primary_mask(first, "ESQUERDA") | primary_mask(grown, "ESQUERDA")
        with patch("vision.green_detection.black_roi_orientation_degrees", side_effect=(0.0, 90.0)):
            _, result, publication = self.observe(tracker, 2, [first, grown], mask)
        self.assertEqual(result["primary_match_count"], 2)
        self.assertEqual(result["primary_marker_reason"], "primary_marker_multiple_matches")
        self.assertFalse(publication[1])

    def test_logged_left_frame_then_second_unmeasured_marker_confirms_same_green(self):
        for direction in ("ESQUERDA", "DIREITA"):
            for reverse_order in (False, True):
                with self.subTest(direction=direction, reverse_order=reverse_order):
                    tracker = GreenObservationTracker()
                    first = rectangle(220, 190)
                    _, _, pending = self.observe(tracker, 1, [first], primary_mask(first, direction))
                    self.assertFalse(pending[1])
                    moved = rectangle(222, 192)
                    other = rectangle(80, 0)
                    contours = [other, moved] if reverse_order else [moved, other]
                    mask = primary_mask(moved, direction)
                    global_result, result, confirmed = self.observe(tracker, 2, contours, mask)
                    self.assertEqual(global_result["interpretation"], "AMBIGUO")
                    self.assertEqual(confirmed[:2], (direction, True))
                    self.assertFalse(result["pair_compatible"])
                    self.assertTrue(result["path_black_valid"])
                    self.assertTrue(green_geometry_reason(result).startswith("tracked_primary_"))
                    candidates = [{"centroid": (0, 0), "area": 2500} for _ in contours]
                    status = build_green_status(candidates, 0, result, confirmed, 0)
                    self.assertEqual(status["greenCandidateCount"], 2)
                    self.assertTrue(status["greenFrontRoiValid"])
                    for sequence in range(3, 10):
                        _, _, publication = self.observe(tracker, sequence, contours, mask)
                        self.assertEqual(publication[:2], (direction, True))

    def test_separate_upper_valid_marker_does_not_block_original_side(self):
        first, other = rectangle(220, 190), rectangle(80, 80)
        tracker = GreenObservationTracker()
        self.observe(tracker, 1, [first], primary_mask(first, "ESQUERDA"))
        mask = primary_mask(first, "ESQUERDA")
        x1, y1, x2, y2 = green_marker_roi_geometry(other, 480)["upper_roi"]
        mask[y1:y2, x1:x2] = 255
        global_result, result, publication = self.observe(tracker, 2, [other, first], mask)
        self.assertFalse(global_result["pair_compatible"])
        self.assertEqual(publication[:2], ("ESQUERDA", True))
        self.assertFalse(result["pair_compatible"])

    def test_other_marker_cannot_complete_missing_primary_evidence(self):
        first, other = rectangle(220, 190), rectangle(80, 190)
        tracker = GreenObservationTracker()
        self.observe(tracker, 1, [first], primary_mask(first, "ESQUERDA"))
        _, result, publication = self.observe(tracker, 2, [other], primary_mask(other, "ESQUERDA"))
        self.assertEqual(result["interpretation"], "AMBIGUO")
        self.assertFalse(publication[1])
        self.assertEqual(publication[2], 1)
        self.assertEqual(result["primary_marker_reason"], "primary_marker_missing")

    def test_primary_still_requires_current_upper_black(self):
        first, other = rectangle(220, 190), rectangle(80, 0)
        tracker = GreenObservationTracker()
        self.observe(tracker, 1, [first], primary_mask(first, "ESQUERDA"))
        mask = primary_mask(first, "ESQUERDA")
        x1, y1, x2, y2 = green_marker_roi_geometry(first, 480)["upper_roi"]
        mask[y1:y2, x1:x2] = 0
        _, result, publication = self.observe(tracker, 2, [first, other], mask)
        self.assertEqual(result["interpretation"], "VERDE_FALSO")
        self.assertFalse(publication[1])
        self.assertFalse(result["path_black_valid"])

    def test_two_overlapping_matches_cannot_select_a_marker_by_order(self):
        first = rectangle(220, 190)
        tracker = GreenObservationTracker()
        mask = primary_mask(first, "ESQUERDA")
        self.observe(tracker, 1, [first], mask)
        with patch("vision.green_detection.black_roi_orientation_degrees",
                   side_effect=(0.0, 90.0)):
            _, result, publication = self.observe(
                tracker, 2, [first, rectangle(222, 192)], mask,
            )
        self.assertEqual(result["interpretation"], "AMBIGUO")
        self.assertFalse(publication[1])
        self.assertEqual(green_geometry_reason(result), "primary_marker_multiple_matches")

    def test_opposite_reading_of_primary_cannot_change_first_side(self):
        first = rectangle(220, 190)
        tracker = GreenObservationTracker()
        self.observe(tracker, 1, [first], primary_mask(first, "ESQUERDA"))
        _, result, publication = self.observe(tracker, 2, [first], primary_mask(first, "DIREITA"))
        self.assertEqual(result["interpretation"], "AMBIGUO")
        self.assertFalse(publication[1])
        _, _, publication = self.observe(tracker, 3, [first], primary_mask(first, "ESQUERDA"))
        self.assertEqual(publication[:2], ("ESQUERDA", True))

    def test_compatible_pair_keeps_priority_and_original_temporal_confirmation(self):
        first, other = rectangle(220, 190), rectangle(80, 190)
        tracker = GreenObservationTracker()
        mask = primary_mask(first, "ESQUERDA")
        self.observe(tracker, 1, [first], mask)
        self.observe(tracker, 2, [first], mask)
        x1, y1, x2, y2 = green_marker_roi_geometry(other, 480)["upper_roi"]
        mask[y1:y2, x1:x2] = 255
        for sequence in (3, 4, 5):
            _, result, publication = self.observe(tracker, sequence, [first, other], mask)
            self.assertTrue(result["pair_compatible"])
            self.assertEqual(result["interpretation"], "RETORNO_180")
            self.assertEqual(publication[:2],
                             ("RETORNO_180" if sequence == 5 else "ESQUERDA", True))

    def test_confirmed_primary_survives_false_and_rearms_only_after_clear(self):
        first = rectangle(220, 190)
        tracker = GreenObservationTracker()
        mask = primary_mask(first, "ESQUERDA")
        self.observe(tracker, 1, [first], mask)
        self.observe(tracker, 2, [first], mask)
        _, result, publication = self.observe(tracker, 3, [first], np.zeros_like(mask))
        self.assertEqual(publication[:2], ("ESQUERDA", True))
        self.assertTrue(confirmed_green_path_black_valid(result, publication))
        self.observe(tracker, 4, [], np.zeros_like(mask), 1.0)
        self.observe(tracker, 5, [], np.zeros_like(mask), 1.1)
        self.assertIsNone(tracker.primary_marker_bounds)
        other = rectangle(80, 190)
        self.observe(tracker, 6, [other], primary_mask(other, "DIREITA"), 1.2)
        _, _, publication = self.observe(tracker, 7, [other], primary_mask(other, "DIREITA"), 1.3)
        self.assertEqual(publication[:2], ("DIREITA", True))


if __name__ == "__main__":
    unittest.main()
