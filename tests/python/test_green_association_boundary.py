"""Regressões da associação preta próxima ao limite da detecção verde."""

from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np
import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from vision.green_detection import (
    analyze_green_marker_contours,
    create_green_association_mask,
    GreenObservationTracker,
    green_geometry_reason,
    green_marker_roi_geometry,
    green_local_track_reference,
    build_green_status,
    black_roi_orientation_degrees,
)


class GreenAssociationBoundaryTest(unittest.TestCase):
    def test_orientation_diagnostics_preserve_angles_and_report_sample_shape(self):
        roi = (20, 20, 92, 92)
        for height in (0, 1, 20, 40, 69, 72):
            with self.subTest(height=height):
                mask = np.zeros((120, 120), np.uint8)
                mask[92 - height:92, 20:92] = 255
                without_diagnostics = black_roi_orientation_degrees(mask, roi)
                diagnostics = {}
                with_diagnostics = black_roi_orientation_degrees(mask, roi, diagnostics)
                self.assertEqual(without_diagnostics, with_diagnostics)
                self.assertEqual(diagnostics["black_pixels"], 72 * height)
                self.assertEqual(diagnostics["fully_black"], height == 72)
                self.assertGreaterEqual(diagnostics["confidence"], 0.0)
                self.assertLessEqual(diagnostics["confidence"], 1.0)
                if height in (0, 72):
                    self.assertEqual(diagnostics["confidence"], 0.0)
                elif height > 0:
                    self.assertGreater(diagnostics["confidence"], 0.0)

    def test_pair_diagnostics_distinguish_orientation_and_height_rejection(self):
        first = np.array([[[90, 180]], [[140, 180]],
                          [[140, 230]], [[90, 230]]], dtype=np.int32)
        for second_y, second_fraction, expected_vertical, expected_orientation in (
                (180, 1.0, True, True), (180, 0.95, True, True),
                (180, 0.639, True, True), (280, 1.0, False, True)):
            with self.subTest(second_y=second_y, second_fraction=second_fraction):
                second = np.array([[[320, second_y]], [[370, second_y]],
                                   [[370, second_y + 50]], [[320, second_y + 50]]], dtype=np.int32)
                mask = np.zeros((360, 480), dtype=np.uint8)
                for index, contour in enumerate((first, second)):
                    x1, y1, x2, y2 = green_marker_roi_geometry(contour, 480)["upper_roi"]
                    height = y2 - y1 if index == 0 else round((y2 - y1) * second_fraction)
                    mask[y2 - height:y2, x1:x2] = 255
                result = analyze_green_marker_contours([first, second], mask)
                diagnostics = result["pair_diagnostics"]
                self.assertEqual(diagnostics["vertical_compatible"], expected_vertical)
                self.assertEqual(diagnostics["orientation_compatible"], expected_orientation)
                self.assertEqual(result["pair_compatible"], expected_vertical and expected_orientation)
                self.assertEqual(result["interpretation"],
                                 "RETORNO_180" if expected_vertical and expected_orientation else "AMBIGUO")
                self.assertEqual(diagnostics["first_confidence"], 0.0)
                self.assertFalse(diagnostics["first_orientation_defined"])
                self.assertFalse(diagnostics["orientation_comparison_available"])

    def test_defined_incompatible_orientations_still_reject_pair(self):
        contours = [np.array([[[90, 180]], [[140, 180]], [[140, 230]],
                              [[90, 230]]], np.int32),
                    np.array([[[320, 180]], [[370, 180]], [[370, 230]],
                              [[320, 230]]], np.int32)]
        mask = np.zeros((360, 480), np.uint8)
        for index, contour in enumerate(contours):
            x1, y1, x2, y2 = green_marker_roi_geometry(contour, 480)["upper_roi"]
            if index == 0:
                mask[y2 - 36:y2, x1:x2] = 255
            else:
                mask[y1:y2, x1 + 27:x1 + 45] = 255
        result = analyze_green_marker_contours(contours, mask)
        self.assertTrue(all(marker["upper"]["valid"] for marker in result["markers"]))
        self.assertTrue(result["pair_diagnostics"]["orientation_comparison_available"])
        self.assertEqual(result["pair_diagnostics"]["angle_delta_degrees"], 90.0)
        self.assertEqual(result["interpretation"], "AMBIGUO")
        self.assertFalse(result["pair_compatible"])

    def test_full_roi_does_not_bypass_three_frame_turnaround_confirmation(self):
        contours = [np.array([[[90, 180]], [[140, 180]], [[140, 230]],
                              [[90, 230]]], np.int32),
                    np.array([[[320, 214]], [[370, 214]], [[370, 264]],
                              [[320, 264]]], np.int32)]
        mask = np.zeros((360, 480), np.uint8)
        for index, contour in enumerate(contours):
            x1, y1, x2, y2 = green_marker_roi_geometry(contour, 480)["upper_roi"]
            if index == 0:
                mask[y2 - 32:y2, x1:x2] = 255
            else:
                mask[y1:y2, x1:x2] = 255
        result = analyze_green_marker_contours(contours, mask)
        self.assertEqual(result["interpretation"], "RETORNO_180")
        diagnostics = result["pair_diagnostics"]
        self.assertEqual(diagnostics["first_angle_degrees"], 0.0)
        self.assertEqual(diagnostics["second_angle_degrees"], 90.0)
        self.assertTrue(diagnostics["first_orientation_defined"])
        self.assertFalse(diagnostics["second_orientation_defined"])
        tracker = GreenObservationTracker()
        for sequence in (1, 2):
            self.assertFalse(tracker.update(sequence, result["interpretation"], sequence * 0.033)[1])
        published, confirmed, samples = tracker.update(3, result["interpretation"], 0.099)
        self.assertEqual((published, confirmed, samples), ("RETORNO_180", True, 3))

    def test_nearly_full_roi_with_defined_axis_keeps_original_angle_gate(self):
        contours = [np.array([[[90, 180]], [[140, 180]], [[140, 230]],
                              [[90, 230]]], np.int32),
                    np.array([[[320, 180]], [[370, 180]], [[370, 230]],
                              [[320, 230]]], np.int32)]
        mask = np.zeros((360, 480), np.uint8)
        for index, contour in enumerate(contours):
            x1, y1, x2, y2 = green_marker_roi_geometry(contour, 480)["upper_roi"]
            if index == 0:
                mask[y2 - 32:y2, x1:x2] = 255
            else:
                mask[y1:y2, x1 + 1:x2 - 1] = 255
        result = analyze_green_marker_contours(contours, mask)
        self.assertTrue(result["pair_diagnostics"]["orientation_comparison_available"])
        self.assertEqual(result["interpretation"], "AMBIGUO")

    def test_single_marker_does_not_create_pair_diagnostics(self):
        contour = np.array([[[200, 180]], [[250, 180]],
                            [[250, 230]], [[200, 230]]], dtype=np.int32)
        mask = np.full((360, 480), 255, dtype=np.uint8)
        self.assertNotIn("pair_diagnostics", analyze_green_marker_contours([contour], mask))

    def test_local_track_resolves_both_logged_ambiguous_right_cases_and_mirrors(self):
        contour = np.array([[[200, 180]], [[250, 180]],
                            [[250, 230]], [[200, 230]]], dtype=np.int32)
        for direction in ("DIREITA", "ESQUERDA"):
            for opposite_ratio in (0.285, 0.418):
                with self.subTest(direction=direction, opposite_ratio=opposite_ratio):
                    mask = np.zeros((360, 480), dtype=np.uint8)
                    geometry = green_marker_roi_geometry(contour, 480, local_l=True)
                    ux1, uy1, ux2, uy2 = geometry["upper_roi"]
                    mask[uy1:uy2, ux1:ux2] = 255
                    x1, y1, x2, y2 = geometry["horizontal_roi"]
                    mx1, mx2 = geometry["marker_horizontal_bounds"]
                    if direction == "DIREITA":
                        mask[y1:y2, x1:mx1] = 255
                        mask[y1:y2, mx2:mx2 + int(round((x2 - mx2) * opposite_ratio))] = 255
                        mask[231:, 160:200] = 255
                    else:
                        mask[y1:y2, mx2:x2] = 255
                        mask[y1:y2, mx1 - int(round((mx1 - x1) * opposite_ratio)):mx1] = 255
                        mask[231:, 250:290] = 255
                    original = mask.copy()
                    result = analyze_green_marker_contours([contour], mask)
                    self.assertEqual(result["interpretation"], direction)
                    self.assertTrue(result["path_black_valid"])
                    self.assertFalse(result["pair_compatible"])
                    self.assertEqual(green_geometry_reason(result), "local_track_reference")
                    np.testing.assert_array_equal(mask, original)
                    tracker = GreenObservationTracker()
                    for sequence in range(1, 20):
                        observation = tracker.update(sequence, result["interpretation"], sequence * 0.03)
                    status = build_green_status([], 0, result, observation, 0.0)
                    self.assertTrue(status["greenConfirmed"])
                    self.assertTrue(status["greenLocalReferenceValid"])
                    self.assertEqual(status["greenInterpretation"], direction)

    def test_local_track_prevents_opposite_lateral_roi_from_inverting_side(self):
        contour = np.array([[[200, 180]], [[250, 180]],
                            [[250, 230]], [[200, 230]]], dtype=np.int32)
        for direction in ("DIREITA", "ESQUERDA"):
            with self.subTest(direction=direction):
                mask = np.zeros((360, 480), dtype=np.uint8)
                geometry = green_marker_roi_geometry(contour, 480, local_l=True)
                ux1, uy1, ux2, uy2 = geometry["upper_roi"]
                mask[uy1:uy2, ux1:ux2] = 255
                x1, y1, x2, y2 = geometry["horizontal_roi"]
                if direction == "DIREITA":
                    mask[y1:y2, 250:x2] = 255
                    mask[231:, 160:200] = 255
                else:
                    mask[y1:y2, x1:200] = 255
                    mask[231:, 250:290] = 255
                result = analyze_green_marker_contours([contour], mask)
                self.assertEqual(result["interpretation"], direction)
                self.assertEqual(green_geometry_reason(result), "local_track_reference")

    def test_local_track_projects_inclined_lane_for_both_sides_and_screen_positions(self):
        for direction in ("DIREITA", "ESQUERDA"):
            for angle in (-40, -25, 0, 25, 40):
                for shift in (-70, 0, 70):
                    with self.subTest(direction=direction, angle=angle, shift=shift):
                        contour = np.array([[[200, 170]], [[250, 170]],
                                            [[250, 220]], [[200, 220]]], dtype=np.int32)
                        mask = np.zeros((360, 480), dtype=np.uint8)
                        transform = cv2.getRotationMatrix2D((225, 195), angle, 1.0)
                        transform[0, 2] += shift
                        left, right = (160, 200) if direction == "DIREITA" else (250, 290)
                        # Desenha uma faixa longa depois da transformação para
                        # não criar um fim de pista artificial na borda original.
                        for points in (((left, -360), (right, -360), (right, 720), (left, 720)),
                                       ((80, 100), (370, 100), (370, 170), (80, 170))):
                            polygon = cv2.transform(np.array(points, np.float32).reshape(-1, 1, 2), transform)
                            cv2.fillPoly(mask, [np.round(polygon).astype(np.int32)], 255)
                        contour = cv2.transform(contour.astype(np.float32), transform)
                        geometry = green_marker_roi_geometry(contour, 480, local_l=True)
                        reference = green_local_track_reference(geometry, mask)
                        self.assertTrue(reference["valid"], reference)
                        result = analyze_green_marker_contours([contour], mask)
                        self.assertEqual(result["interpretation"], direction)

    def test_local_track_does_not_bypass_missing_upper_black(self):
        contour = np.array([[[200, 180]], [[250, 180]],
                            [[250, 230]], [[200, 230]]], dtype=np.int32)
        mask = np.zeros((360, 480), dtype=np.uint8)
        mask[231:, 160:200] = 255
        result = analyze_green_marker_contours([contour], mask)
        self.assertEqual(result["interpretation"], "VERDE_FALSO")
        self.assertFalse(result["path_black_valid"])

    def test_local_track_rejects_two_equally_near_lanes_and_short_fragments(self):
        contour = np.array([[[200, 180]], [[250, 180]],
                            [[250, 230]], [[200, 230]]], dtype=np.int32)
        geometry = green_marker_roi_geometry(contour, 480, local_l=True)
        mask = np.zeros((360, 480), dtype=np.uint8)
        mask[231:, 160:200] = 255
        mask[231:, 250:290] = 255
        self.assertFalse(green_local_track_reference(geometry, mask)["valid"])

    def test_conflicting_reference_cannot_fall_back_to_opposite_lateral_side(self):
        contour = np.array([[[200, 180]], [[250, 180]],
                            [[250, 230]], [[200, 230]]], dtype=np.int32)
        geometry = green_marker_roi_geometry(contour, 480, local_l=True)
        mask = np.zeros((360, 480), dtype=np.uint8)
        ux1, uy1, ux2, uy2 = geometry["upper_roi"]
        mask[uy1:uy2, ux1:ux2] = 255
        x1, y1, x2, y2 = geometry["horizontal_roi"]
        mask[y1:y2, 250:x2] = 255
        mask[231:256, 160:200] = 255
        mask[256:282, 260:300] = 255
        result = analyze_green_marker_contours([contour], mask)
        self.assertEqual(result["interpretation"], "AMBIGUO")
        self.assertEqual(green_geometry_reason(result), "conflicting_local_track")
        self.assertFalse(result["path_black_valid"])
        mask[:] = 0
        mask[231:240, 160:200] = 255
        self.assertFalse(green_local_track_reference(geometry, mask)["valid"])

    def test_both_lateral_sides_allow_confirmation_relative_to_upper_black(self):
        for direction, side in (("DIREITA", "left"), ("ESQUERDA", "right")):
            with self.subTest(direction=direction):
                contour = np.array([[[200, 180]], [[250, 180]],
                                    [[250, 230]], [[200, 230]]], dtype=np.int32)
                mask = np.zeros((360, 480), dtype=np.uint8)
                geometry = green_marker_roi_geometry(contour, 480)
                x1, y1, x2, y2 = geometry["horizontal_roi"]
                mx1, mx2 = geometry["marker_horizontal_bounds"]
                mask[y1:y2, x1:mx1] = 255
                mask[y1:y2, mx2:x2] = 255
                ux1, uy1, ux2, uy2 = geometry["upper_roi"]
                center = (mx1 + mx2) // 2
                if side == "left":
                    mask[uy1:uy2, ux1:center] = 255
                else:
                    mask[uy1:uy2, center:ux2] = 255
                with patch("vision.green_detection.GREEN_LOCAL_L_CONFIRMATION_ENABLED", False):
                    result = analyze_green_marker_contours([contour], mask)
                horizontal = result["markers"][0]["horizontal"]
                self.assertTrue(horizontal["left_valid"])
                self.assertTrue(horizontal["right_valid"])
                self.assertEqual(result["interpretation"], direction)
                self.assertTrue(result["path_black_valid"])
                self.assertFalse(result["pair_compatible"])
                tracker = GreenObservationTracker()
                for sequence in range(1, 20):
                    published, confirmed, _samples = tracker.update(
                        sequence, result["interpretation"], sequence * 0.03,
                    )
                    if confirmed:
                        break
                self.assertTrue(confirmed)
                self.assertEqual(published, direction)

    def test_turnaround_still_accepts_original_upper_only_compatible_pair(self):
        contours = [np.array([[[90, 180]], [[140, 180]], [[140, 230]],
                              [[90, 230]]], dtype=np.int32),
                    np.array([[[320, 182]], [[370, 182]], [[370, 232]],
                              [[320, 232]]], dtype=np.int32)]
        mask = np.zeros((360, 480), dtype=np.uint8)
        for contour in contours:
            x1, y1, x2, y2 = green_marker_roi_geometry(contour, 480)["upper_roi"]
            mask[y1:y2, x1:x2] = 255
        result = analyze_green_marker_contours(contours, mask)
        self.assertEqual(result["interpretation"], "RETORNO_180")
        self.assertTrue(result["pair_compatible"])

    def test_visible_side_black_below_green_limit_preserves_both_directions(self):
        for direction, side in (("DIREITA", "left"), ("ESQUERDA", "right")):
            with self.subTest(direction=direction):
                contour = np.array([[[200, 280]], [[250, 280]],
                                    [[250, 300]], [[200, 300]]], dtype=np.int32)
                structural = np.zeros((360, 480), dtype=np.uint8)
                geometry = green_marker_roi_geometry(contour, 480)
                ux1, uy1, ux2, uy2 = geometry["upper_roi"]
                structural[uy1:uy2, ux1:ux2] = 255
                x1, y1, x2, y2 = geometry["horizontal_roi"]
                mx1, mx2 = geometry["marker_horizontal_bounds"]
                # A faixa lateral é real e visível abaixo do limite verde.
                if side == "left":
                    structural[300:y2, x1:mx1] = 255
                else:
                    structural[300:y2, mx2:x2] = 255
                original = structural.copy()
                green = np.zeros((300, 480), dtype=np.uint8)
                green[280:300, 200:250] = 255
                association = create_green_association_mask(structural, green, 0)
                result = analyze_green_marker_contours([contour], association)
                self.assertEqual(result["interpretation"], direction)
                self.assertEqual(green_geometry_reason(result), "experimental_local_l")
                self.assertTrue(result["path_black_valid"])
                np.testing.assert_array_equal(structural, original)
                old_mask = association.copy()
                old_mask[300:] = 0
                old_result = analyze_green_marker_contours([contour], old_mask)
                self.assertEqual(old_result["interpretation"], "AMBIGUO")
                self.assertEqual(green_geometry_reason(old_result), "side_black_missing")

    def test_green_and_dead_zone_are_still_excluded(self):
        structural = np.full((360, 480), 255, dtype=np.uint8)
        green = np.zeros((300, 480), dtype=np.uint8)
        green[200:230, 200:250] = 255
        association = create_green_association_mask(structural, green, 45)
        self.assertFalse(association[:45].any())
        self.assertFalse(association[200:230, 200:250].any())
        self.assertTrue(association[300:].all())
        self.assertTrue(structural.all())

    def test_local_l_ignores_distant_opposite_black_for_both_directions(self):
        contour = np.array([[[200, 180]], [[250, 180]],
                            [[250, 230]], [[200, 230]]], dtype=np.int32)
        for direction in ("DIREITA", "ESQUERDA"):
            with self.subTest(direction=direction):
                mask = np.zeros((360, 480), dtype=np.uint8)
                original = green_marker_roi_geometry(contour, 480)
                local = green_marker_roi_geometry(contour, 480, local_l=True)
                ux1, uy1, ux2, uy2 = local["upper_roi"]
                mask[uy1:uy2, ux1:ux2] = 255
                ox1, y1, ox2, y2 = original["horizontal_roi"]
                lx1, _, lx2, _ = local["horizontal_roi"]
                mx1, mx2 = local["marker_horizontal_bounds"]
                if direction == "DIREITA":
                    mask[y1:y2, lx1:mx1] = 255
                    mask[y1:y2, lx2:ox2] = 255
                else:
                    mask[y1:y2, mx2:lx2] = 255
                    mask[y1:y2, ox1:lx1] = 255
                with patch("vision.green_detection.GREEN_LOCAL_L_CONFIRMATION_ENABLED", False):
                    self.assertEqual(analyze_green_marker_contours([contour], mask)["interpretation"], "AMBIGUO")
                result = analyze_green_marker_contours([contour], mask)
                self.assertEqual(result["interpretation"], direction)
                self.assertTrue(result["path_black_valid"])
                self.assertFalse(result["pair_compatible"])

    def test_both_local_hypotheses_remain_ambiguous_without_false_turnaround(self):
        contour = np.array([[[200, 180]], [[250, 180]],
                            [[250, 230]], [[200, 230]]], dtype=np.int32)
        mask = np.full((360, 480), 255, dtype=np.uint8)
        result = analyze_green_marker_contours([contour], mask)
        self.assertEqual(result["interpretation"], "AMBIGUO")
        self.assertFalse(result["pair_compatible"])
        self.assertFalse(result["path_black_valid"])
        self.assertEqual(green_geometry_reason(result), "both_local_l_hypotheses_valid")
        tracker = GreenObservationTracker()
        for sequence in range(1, 20):
            tracker.update(sequence, "DIREITA", sequence * 0.03)
        published, confirmed, _samples = tracker.update(20, result["interpretation"], 0.6)
        self.assertTrue(confirmed)
        self.assertEqual(published, "DIREITA")

    def test_local_l_does_not_require_opposite_roi_visibility(self):
        for direction, left, right in (("DIREITA", 445, 480),
                                       ("ESQUERDA", 0, 35)):
            with self.subTest(direction=direction):
                contour = np.array([[[left, 180]], [[right, 180]],
                                    [[right, 230]], [[left, 230]]], dtype=np.int32)
                mask = np.full((360, 480), 255, dtype=np.uint8)
                result = analyze_green_marker_contours([contour], mask)
                self.assertEqual(result["interpretation"], direction)

    def test_visible_black_without_upper_association_still_rejects_false_green(self):
        contour = np.array([[[200, 280]], [[250, 280]],
                            [[250, 300]], [[200, 300]]], dtype=np.int32)
        structural = np.zeros((360, 480), dtype=np.uint8)
        structural[300:314, 152:200] = 255
        association = create_green_association_mask(
            structural, np.zeros((300, 480), dtype=np.uint8), 0,
        )
        result = analyze_green_marker_contours([contour], association)
        self.assertEqual(result["interpretation"], "VERDE_FALSO")
        self.assertEqual(green_geometry_reason(result), "upper_black_missing")
        self.assertFalse(result["path_black_valid"])
        self.assertFalse(result["pair_compatible"])


if __name__ == "__main__":
    unittest.main()
