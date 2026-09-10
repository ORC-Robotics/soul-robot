import sys
import unittest
from pathlib import Path
from unittest import mock

import cv2  # type: ignore
import numpy as np


SCRIPT_DIRECTORY = Path(__file__).resolve().parents[2] / "scripts"
if str(SCRIPT_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIRECTORY))

from vision import rescue_zone
from vision.rescue_zone import analyze_rescue_zones, serializable_results


FRAME_HEIGHT = 240
FRAME_WIDTH = 320
GREEN = (0, 200, 0)
RED = (0, 0, 220)
WHITE = (255, 255, 255)


def blank_frame():
    return np.full((FRAME_HEIGHT, FRAME_WIDTH, 3), WHITE, dtype=np.uint8)


def draw_complete_zone(frame, color, horizontal_offset=0):
    points = np.asarray(
        [
            (70 + horizontal_offset, 220),
            (105 + horizontal_offset, 55),
            (215 + horizontal_offset, 55),
            (250 + horizontal_offset, 220),
        ],
        dtype=np.int32,
    )
    cv2.fillConvexPoly(frame, points, color)


def frame_with_rgb_zone(rgb):
    frame = blank_frame()
    draw_complete_zone(frame, tuple(reversed(rgb)))
    return frame


def frame_with_hsv_zone(hsv):
    hsv_color = np.asarray([[hsv]], dtype=np.uint8)
    bgr = cv2.cvtColor(hsv_color, cv2.COLOR_HSV2BGR)[0, 0]
    frame = blank_frame()
    draw_complete_zone(frame, tuple(int(channel) for channel in bgr))
    return frame


class RescueZoneDetectionTest(unittest.TestCase):
    def test_one_positive_frame_does_not_confirm_zone(self):
        tracker = rescue_zone.RescueZoneTemporalFilter()
        candidate = analyze_rescue_zones(frame_with_rgb_zone((0, 152, 119)))

        result = tracker.update(candidate)["green"]

        self.assertTrue(result["candidateDetected"])
        self.assertTrue(result["candidateAimValid"])
        self.assertFalse(result["detected"])
        self.assertEqual(result["confirmationFrames"], 1)
        self.assertFalse(result["aimValid"])

    def test_two_positive_frames_do_not_confirm_zone(self):
        tracker = rescue_zone.RescueZoneTemporalFilter()
        candidate = analyze_rescue_zones(frame_with_rgb_zone((0, 152, 119)))

        tracker.update(candidate)
        result = tracker.update(candidate)["green"]

        self.assertFalse(result["detected"])
        self.assertEqual(result["confirmationFrames"], 2)
        self.assertFalse(result["aimValid"])

    def test_three_positive_frames_confirm_zone_and_release_full_bounds_aim(self):
        tracker = rescue_zone.RescueZoneTemporalFilter()
        candidate = analyze_rescue_zones(frame_with_rgb_zone((0, 152, 119)))

        tracker.update(candidate)
        tracker.update(candidate)
        result = tracker.update(candidate)["green"]

        self.assertTrue(result["detected"])
        self.assertEqual(result["confirmationFrames"], 3)
        self.assertEqual(result["confirmationRequiredFrames"], 3)
        self.assertEqual(result["geometryState"], "FULL_BOUNDS")
        self.assertTrue(result["aimValid"])
        self.assertIsNotNone(result["aimX"])

    def test_confirmed_zone_survives_one_negative_without_reusing_aim(self):
        tracker = rescue_zone.RescueZoneTemporalFilter()
        candidate = analyze_rescue_zones(frame_with_rgb_zone((0, 152, 119)))
        missing = analyze_rescue_zones(blank_frame())
        for _ in range(3):
            tracker.update(candidate)

        result = tracker.update(missing)["green"]

        self.assertFalse(result["candidateDetected"])
        self.assertTrue(result["detected"])
        self.assertEqual(result["lossFrames"], 1)
        self.assertFalse(result["aimValid"])
        self.assertIsNone(result["aimX"])
        self.assertEqual(result["geometryState"], "NOT_DETECTED")

    def test_confirmed_zone_is_removed_after_two_negative_frames(self):
        tracker = rescue_zone.RescueZoneTemporalFilter()
        candidate = analyze_rescue_zones(frame_with_rgb_zone((0, 152, 119)))
        missing = analyze_rescue_zones(blank_frame())
        for _ in range(3):
            tracker.update(candidate)

        tracker.update(missing)
        result = tracker.update(missing)["green"]

        self.assertFalse(result["candidateDetected"])
        self.assertFalse(result["detected"])
        self.assertFalse(result["aimValid"])

    def test_green_and_red_temporal_counters_are_independent(self):
        tracker = rescue_zone.RescueZoneTemporalFilter()
        green = analyze_rescue_zones(frame_with_rgb_zone((0, 152, 119)))
        red = analyze_rescue_zones(frame_with_rgb_zone((220, 0, 0)))
        mixed = {
            "green": green["green"],
            "red": red["red"],
        }

        tracker.update({"green": green["green"], "red": green["red"]})
        tracker.update(mixed)
        result = tracker.update(mixed)

        self.assertTrue(result["green"]["detected"])
        self.assertFalse(result["red"]["detected"])
        self.assertEqual(result["green"]["confirmationFrames"], 3)
        self.assertEqual(result["red"]["confirmationFrames"], 2)

    def test_overlay_draws_zone_only_after_temporal_confirmation(self):
        tracker = rescue_zone.RescueZoneTemporalFilter()
        candidate = analyze_rescue_zones(frame_with_rgb_zone((0, 152, 119)))
        first = tracker.update(candidate)
        tracker.update(candidate)
        confirmed = tracker.update(candidate)

        with mock.patch.object(
            rescue_zone.cv2, "putText", wraps=rescue_zone.cv2.putText
        ) as put_text:
            rescue_zone.draw_rescue_zone_overlay(blank_frame(), first)
            first_labels = [call.args[1] for call in put_text.call_args_list]
        with mock.patch.object(
            rescue_zone.cv2, "putText", wraps=rescue_zone.cv2.putText
        ) as put_text:
            rescue_zone.draw_rescue_zone_overlay(blank_frame(), confirmed)
            confirmed_labels = [call.args[1] for call in put_text.call_args_list]

        self.assertNotIn("AREA VERDE  |  ULTRA --", first_labels)
        self.assertIn("ULTRA --", first_labels)
        self.assertIn("AREA VERDE  |  ULTRA --", confirmed_labels)

    def test_observed_real_green_samples_are_accepted(self):
        real_green_samples = (
            (0, 152, 119),
            (0, 147, 110),
            (0, 141, 105),
            (0, 156, 124),
            (0, 156, 119),
        )

        for rgb in real_green_samples:
            with self.subTest(rgb=rgb):
                result = analyze_rescue_zones(frame_with_rgb_zone(rgb))["green"]
                self.assertTrue(result["detected"])
                self.assertTrue(result["aimValid"])
                self.assertEqual(result["strongGreenFraction"], 1.0)

    def test_observed_cyan_floor_samples_are_rejected(self):
        cyan_floor_samples = (
            (129, 237, 235),
            (126, 193, 192),
            (123, 186, 185),
            (115, 229, 226),
        )

        for rgb in cyan_floor_samples:
            with self.subTest(rgb=rgb):
                result = analyze_rescue_zones(frame_with_rgb_zone(rgb))["green"]
                self.assertFalse(result["detected"])
                self.assertIsNone(result["_hull"])

    def test_neutral_and_tinted_white_are_not_green(self):
        non_green_samples = (
            (235, 235, 235),
            (160, 225, 225),
            (190, 215, 205),
        )

        for rgb in non_green_samples:
            with self.subTest(rgb=rgb):
                result = analyze_rescue_zones(frame_with_rgb_zone(rgb))["green"]
                self.assertFalse(result["detected"])

    def test_observed_olive_and_mdf_samples_are_rejected(self):
        olive_mdf_samples = (
            ((94, 126, 94), (54, 83, 126)),
            ((58, 75, 51), (67, 101, 75)),
            ((97, 109, 65), (38, 102, 109)),
            ((75, 101, 59), (53, 108, 101)),
            ((117, 142, 97), (47, 80, 142)),
            ((134, 167, 116), (49, 78, 167)),
        )

        for rgb, hsv in olive_mdf_samples:
            for source, frame in (
                ("rgb", frame_with_rgb_zone(rgb)),
                ("hsv", frame_with_hsv_zone(hsv)),
            ):
                with self.subTest(rgb=rgb, hsv=hsv, source=source):
                    result = analyze_rescue_zones(frame)["green"]
                    self.assertFalse(result["detected"])
                    self.assertIsNone(result["_hull"])

    def test_wood_olive_and_brown_surfaces_are_rejected(self):
        non_green_samples = (
            (190, 170, 125),
            (82, 68, 44),
            (110, 120, 85),
            (155, 130, 48),
        )

        for rgb in non_green_samples:
            with self.subTest(rgb=rgb):
                result = analyze_rescue_zones(frame_with_rgb_zone(rgb))["green"]
                self.assertFalse(result["detected"])

    def test_candidate_component_requires_twenty_percent_strong_green(self):
        zone_mask = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype=np.uint8)
        draw_complete_zone(zone_mask, 255)
        candidate_pixels = np.flatnonzero(zone_mask)

        accepted_frame = blank_frame()
        weak_hsv = np.asarray([[[54, 83, 126]]], dtype=np.uint8)
        weak_bgr = cv2.cvtColor(weak_hsv, cv2.COLOR_HSV2BGR)[0, 0]
        accepted_frame[zone_mask > 0] = weak_bgr
        accepted_strong_pixels = candidate_pixels[: int(candidate_pixels.size * 0.30)]
        accepted_frame.reshape(-1, 3)[accepted_strong_pixels] = GREEN
        accepted = analyze_rescue_zones(accepted_frame)["green"]

        rejected_frame = blank_frame()
        rejected_frame[zone_mask > 0] = weak_bgr
        rejected_strong_pixels = candidate_pixels[: int(candidate_pixels.size * 0.10)]
        rejected_frame.reshape(-1, 3)[rejected_strong_pixels] = GREEN
        rejected = analyze_rescue_zones(rejected_frame)["green"]

        self.assertTrue(accepted["detected"])
        self.assertGreaterEqual(
            accepted["strongGreenFraction"],
            rescue_zone.MIN_STRONG_GREEN_FRACTION,
        )
        self.assertFalse(rejected["detected"])

    def test_plausible_competition_green_variations_are_accepted(self):
        green_samples = (
            (0, 70, 30),
            (100, 210, 135),
            (90, 170, 30),
            (10, 150, 110),
            (0, 76, 60),
            (0, 213, 167),
        )

        for rgb in green_samples:
            with self.subTest(rgb=rgb):
                result = analyze_rescue_zones(frame_with_rgb_zone(rgb))["green"]
                self.assertTrue(result["detected"])

    def test_complete_green_zone_has_full_bounds(self):
        frame = blank_frame()
        draw_complete_zone(frame, GREEN)

        result = analyze_rescue_zones(frame)["green"]

        self.assertTrue(result["detected"])
        self.assertEqual(result["geometryState"], "FULL_BOUNDS")
        self.assertTrue(result["aimValid"])
        self.assertAlmostEqual(result["aimX"], 160.0, delta=3.0)

    def test_complete_red_zone_has_full_bounds(self):
        frame = blank_frame()
        draw_complete_zone(frame, RED)

        result = analyze_rescue_zones(frame)["red"]

        self.assertTrue(result["detected"])
        self.assertEqual(result["geometryState"], "FULL_BOUNDS")
        self.assertTrue(result["aimValid"])

    def test_red_hue_wrap_is_detected(self):
        hsv_frame = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
        hsv_frame[:, :] = (0, 0, 255)
        points = np.asarray([(70, 220), (105, 55), (215, 55), (250, 220)])
        cv2.fillConvexPoly(hsv_frame, points, (175, 220, 220))
        frame = cv2.cvtColor(hsv_frame, cv2.COLOR_HSV2BGR)

        result = analyze_rescue_zones(frame)["red"]

        self.assertTrue(result["detected"])
        self.assertTrue(result["aimValid"])

    def test_red_remains_detected_in_shadow_and_warm_flash(self):
        red_samples = (
            (85, 10, 10),
            (240, 95, 90),
            (220, 25, 35),
        )

        for rgb in red_samples:
            with self.subTest(rgb=rgb):
                result = analyze_rescue_zones(frame_with_rgb_zone(rgb))["red"]
                self.assertTrue(result["detected"])

    def test_almost_full_green_frame_has_unknown_bounds(self):
        frame = blank_frame()
        cv2.rectangle(frame, (2, 3), (317, 239), GREEN, -1)

        result = analyze_rescue_zones(frame)["green"]

        self.assertTrue(result["detected"])
        self.assertEqual(result["geometryState"], "BOUNDS_UNKNOWN")
        self.assertFalse(result["aimValid"])

    def test_almost_full_red_frame_has_unknown_bounds(self):
        frame = blank_frame()
        cv2.rectangle(frame, (1, 2), (318, 239), RED, -1)

        result = analyze_rescue_zones(frame)["red"]

        self.assertTrue(result["detected"])
        self.assertEqual(result["geometryState"], "BOUNDS_UNKNOWN")
        self.assertFalse(result["aimValid"])

    def test_only_left_boundary_is_visible(self):
        frame = blank_frame()
        points = np.asarray([(55, 220), (95, 55), (319, 55), (319, 220)])
        cv2.fillConvexPoly(frame, points, GREEN)

        result = analyze_rescue_zones(frame)["green"]

        self.assertEqual(result["geometryState"], "LEFT_BOUND_ONLY")
        self.assertIsNotNone(result["leftEdgeX"])
        self.assertIsNone(result["rightEdgeX"])
        self.assertFalse(result["aimValid"])

    def test_only_right_boundary_is_visible(self):
        frame = blank_frame()
        points = np.asarray([(0, 220), (0, 55), (225, 55), (265, 220)])
        cv2.fillConvexPoly(frame, points, RED)

        result = analyze_rescue_zones(frame)["red"]

        self.assertEqual(result["geometryState"], "RIGHT_BOUND_ONLY")
        self.assertIsNone(result["leftEdgeX"])
        self.assertIsNotNone(result["rightEdgeX"])
        self.assertFalse(result["aimValid"])

    def test_partially_outside_frame_remains_detected(self):
        frame = blank_frame()
        points = np.asarray([(-80, 220), (-40, 60), (130, 60), (190, 220)])
        cv2.fillConvexPoly(frame, points, GREEN)

        result = analyze_rescue_zones(frame)["green"]

        self.assertTrue(result["detected"])
        self.assertEqual(result["geometryState"], "RIGHT_BOUND_ONLY")
        self.assertFalse(result["aimValid"])

    def test_large_central_occlusion_is_reconstructed(self):
        frame = blank_frame()
        draw_complete_zone(frame, GREEN)
        cv2.rectangle(frame, (135, 45), (185, 225), WHITE, -1)

        result = analyze_rescue_zones(frame)["green"]

        self.assertTrue(result["detected"])
        self.assertEqual(result["geometryState"], "FULL_BOUNDS")
        self.assertTrue(result["aimValid"])
        self.assertAlmostEqual(result["aimX"], 160.0, delta=5.0)

    def test_small_colored_noise_is_rejected(self):
        frame = blank_frame()
        for center in ((20, 20), (100, 80), (250, 190), (300, 30)):
            cv2.circle(frame, center, 2, GREEN, -1)
            cv2.circle(frame, (center[0], min(239, center[1] + 8)), 2, RED, -1)

        results = analyze_rescue_zones(frame)

        self.assertFalse(results["green"]["detected"])
        self.assertFalse(results["red"]["detected"])

    def test_frame_without_zone_returns_not_detected(self):
        results = analyze_rescue_zones(blank_frame())

        self.assertEqual(results["green"]["geometryState"], "NOT_DETECTED")
        self.assertEqual(results["red"]["geometryState"], "NOT_DETECTED")

    def test_green_and_red_are_reported_independently(self):
        frame = blank_frame()
        cv2.fillConvexPoly(
            frame,
            np.asarray([(20, 220), (35, 80), (125, 80), (145, 220)]),
            GREEN,
        )
        cv2.fillConvexPoly(
            frame,
            np.asarray([(175, 220), (195, 80), (285, 80), (305, 220)]),
            RED,
        )

        results = analyze_rescue_zones(frame)

        self.assertTrue(results["green"]["detected"])
        self.assertTrue(results["red"]["detected"])
        self.assertTrue(results["green"]["aimValid"])
        self.assertTrue(results["red"]["aimValid"])

    def test_aim_x_is_stable_during_central_occlusion(self):
        complete = blank_frame()
        draw_complete_zone(complete, GREEN, horizontal_offset=20)
        occluded = complete.copy()
        cv2.rectangle(occluded, (155, 45), (205, 225), WHITE, -1)

        complete_result = analyze_rescue_zones(complete)["green"]
        occluded_result = analyze_rescue_zones(occluded)["green"]

        self.assertTrue(complete_result["aimValid"])
        self.assertTrue(occluded_result["aimValid"])
        self.assertAlmostEqual(
            complete_result["aimX"], occluded_result["aimX"], delta=5.0
        )

    def test_serializable_result_does_not_expose_opencv_hull(self):
        frame = blank_frame()
        draw_complete_zone(frame, GREEN)

        status = serializable_results(analyze_rescue_zones(frame))

        self.assertNotIn("_hull", status["green"])
        self.assertIn("boundsCoverage", status["green"])

    def test_green_and_nearby_background_medians_use_rgb_display_order(self):
        frame = blank_frame()
        draw_complete_zone(frame, GREEN)

        status = serializable_results(analyze_rescue_zones(frame))

        self.assertEqual(status["greenMedianRgb"], [0, 200, 0])
        self.assertEqual(status["greenMedianHsv"], [60, 255, 200])
        self.assertEqual(status["backgroundMedianRgb"], [255, 255, 255])
        self.assertEqual(status["backgroundMedianHsv"], [0, 0, 255])
        self.assertEqual(
            status["green"]["greenMedianRgb"],
            status["greenMedianRgb"],
        )

    def test_red_and_nearby_background_medians_use_rgb_display_order(self):
        frame = blank_frame()
        draw_complete_zone(frame, RED)

        status = serializable_results(analyze_rescue_zones(frame))

        self.assertEqual(status["redMedianRgb"], [220, 0, 0])
        self.assertEqual(status["redMedianHsv"], [0, 255, 220])
        self.assertEqual(status["redBackgroundMedianRgb"], [255, 255, 255])
        self.assertEqual(status["redBackgroundMedianHsv"], [0, 0, 255])
        self.assertEqual(status["red"]["redMedianRgb"], status["redMedianRgb"])

    def test_missing_green_sample_publishes_none_and_draws_dashes(self):
        frame = blank_frame()
        results = analyze_rescue_zones(frame)
        status = serializable_results(results)

        self.assertIsNone(status["greenMedianRgb"])
        self.assertIsNone(status["greenMedianHsv"])
        self.assertIsNone(status["backgroundMedianRgb"])
        self.assertIsNone(status["backgroundMedianHsv"])
        self.assertIsNone(status["redMedianRgb"])
        self.assertIsNone(status["redMedianHsv"])
        self.assertIsNone(status["redBackgroundMedianRgb"])
        self.assertIsNone(status["redBackgroundMedianHsv"])

        with mock.patch.object(
            rescue_zone.cv2, "putText", wraps=rescue_zone.cv2.putText
        ) as put_text:
            rescue_zone.draw_rescue_zone_overlay(
                frame, results, show_color_diagnostics=True
            )

        labels = [call.args[1] for call in put_text.call_args_list]
        self.assertIn("ULTRA --", labels)
        self.assertIn("GREEN RGB -- | HSV --", labels)
        self.assertIn("BG    RGB -- | HSV --", labels)
        self.assertIn("RED RGB -- | HSV --", labels)
        self.assertIn("BG  RGB -- | HSV --", labels)

    def test_red_diagnostics_are_shown_only_when_requested(self):
        frame = blank_frame()
        draw_complete_zone(frame, RED)
        results = analyze_rescue_zones(frame)

        with mock.patch.object(
            rescue_zone.cv2, "putText", wraps=rescue_zone.cv2.putText
        ) as put_text:
            rescue_zone.draw_rescue_zone_overlay(
                frame,
                results,
                show_color_diagnostics=True,
            )

        labels = [call.args[1] for call in put_text.call_args_list]
        self.assertIn("RED RGB 220 0 0 | HSV 0 255 220", labels)
        self.assertIn("BG  RGB 255 255 255 | HSV 0 0 255", labels)

    def test_ultrasonic_remains_visible_without_detected_zone(self):
        frame = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
        results = analyze_rescue_zones(frame)
        ultrasonic = {
            "ultrasonicFresh": True,
            "ultrasonicValid": True,
            "ultrasonicDistanceCm": 7.4,
        }

        with mock.patch.object(
            rescue_zone.cv2, "putText", wraps=rescue_zone.cv2.putText
        ) as put_text:
            rescue_zone.draw_rescue_zone_overlay(
                frame,
                results,
                ultrasonic,
                show_color_diagnostics=True,
            )

        labels = [call.args[1] for call in put_text.call_args_list]
        self.assertIn("ULTRA 7.4 cm", labels)
        self.assertFalse(any(label.startswith("AREA ") for label in labels))

    def test_overlay_places_valid_ultrasonic_beside_area_name(self):
        frame = blank_frame()
        draw_complete_zone(frame, GREEN)
        results = analyze_rescue_zones(frame)
        ultrasonic = {
            "ultrasonicFresh": True,
            "ultrasonicValid": True,
            "ultrasonicDistanceCm": 43.7,
        }

        with mock.patch.object(
            rescue_zone.cv2, "putText", wraps=rescue_zone.cv2.putText
        ) as put_text:
            rescue_zone.draw_rescue_zone_overlay(
                frame,
                results,
                ultrasonic,
                show_color_diagnostics=True,
            )

        labels = [call.args[1] for call in put_text.call_args_list]
        self.assertIn("AREA VERDE  |  ULTRA 43.7 cm", labels)
        self.assertIn("GREEN RGB 0 200 0 | HSV 60 255 200", labels)
        self.assertIn("BG    RGB 255 255 255 | HSV 0 0 255", labels)
        self.assertLess(
            labels.index("GREEN RGB 0 200 0 | HSV 60 255 200"),
            labels.index("AREA VERDE  |  ULTRA 43.7 cm"),
        )

    def test_color_diagnostics_are_hidden_by_default(self):
        frame = blank_frame()
        draw_complete_zone(frame, GREEN)
        results = analyze_rescue_zones(frame)

        with mock.patch.object(
            rescue_zone.cv2, "putText", wraps=rescue_zone.cv2.putText
        ) as put_text:
            rescue_zone.draw_rescue_zone_overlay(frame, results)

        labels = [call.args[1] for call in put_text.call_args_list]
        self.assertIn("AREA VERDE  |  ULTRA --", labels)
        self.assertFalse(any(label.startswith("GREEN RGB") for label in labels))
        self.assertFalse(any(label.startswith("BG    RGB") for label in labels))
        self.assertFalse(any(label.startswith("RED RGB") for label in labels))
        self.assertFalse(any(label.startswith("BG  RGB") for label in labels))

    def test_zone_labels_do_not_overlap_diagnostic_or_each_other(self):
        diagnostic = (10, 181, 300, 230)
        first = rescue_zone._place_zone_label(
            (FRAME_HEIGHT, FRAME_WIDTH, 3),
            (0, 180, 300, 60),
            (245, 14),
            [diagnostic],
        )
        second = rescue_zone._place_zone_label(
            (FRAME_HEIGHT, FRAME_WIDTH, 3),
            (20, 175, 280, 65),
            (250, 14),
            [diagnostic, first],
        )

        self.assertFalse(rescue_zone._rectangles_overlap(first, diagnostic))
        self.assertFalse(rescue_zone._rectangles_overlap(second, diagnostic))
        self.assertFalse(rescue_zone._rectangles_overlap(first, second))

    def test_overlay_hides_invalid_or_stale_ultrasonic(self):
        frame = blank_frame()
        draw_complete_zone(frame, RED)
        results = analyze_rescue_zones(frame)
        readings = (
            {
                "ultrasonicFresh": False,
                "ultrasonicValid": True,
                "ultrasonicDistanceCm": 43.7,
            },
            {
                "ultrasonicFresh": True,
                "ultrasonicValid": True,
                "ultrasonicDistanceCm": 500.0,
            },
        )

        for ultrasonic in readings:
            with self.subTest(ultrasonic=ultrasonic), mock.patch.object(
                rescue_zone.cv2, "putText", wraps=rescue_zone.cv2.putText
            ) as put_text:
                rescue_zone.draw_rescue_zone_overlay(frame, results, ultrasonic)
                labels = [call.args[1] for call in put_text.call_args_list]
                self.assertIn("AREA VERMELHA  |  ULTRA --", labels)


if __name__ == "__main__":
    unittest.main()
