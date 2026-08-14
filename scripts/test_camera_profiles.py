import importlib.util
import os
import sys
import unittest
from unittest import mock

import numpy as np


SCRIPT_PATH = os.path.join(os.path.dirname(__file__), "camera_line_frame.py")
try:
    import cv2  # type: ignore[import]
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    sys.modules.setdefault("cv2", mock.MagicMock())
SPEC = importlib.util.spec_from_file_location("camera_line_frame", SCRIPT_PATH)
camera_line_frame = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(camera_line_frame)


def synthetic_line_axis(near=(320, 260), far=(320, 160)):
    return camera_line_frame.build_line_axis(near, far)


def synthetic_line_mask(
    left_branch=False,
    right_branch=False,
    forward_branch=False,
):
    mask = np.zeros((425, 640), dtype=np.uint8)
    mask[140:401, 310:331] = 255
    if forward_branch:
        mask[40:140, 310:331] = 255
    if left_branch:
        mask[130:151, 100:321] = 255
    if right_branch:
        mask[130:151, 320:541] = 255
    return mask


def synthetic_topology(**branches):
    return camera_line_frame.analyze_line_topology(
        synthetic_line_mask(**branches),
        synthetic_line_axis(),
        21.0,
    )


def synthetic_candidate(
    centroid,
    line_axis=None,
    partial=False,
    associated=True,
):
    line_axis = line_axis or synthetic_line_axis()
    longitudinal, lateral = camera_line_frame.project_point_on_line_axis(
        centroid, line_axis
    )
    return {
        "area": 625.0,
        "centroid": tuple(float(value) for value in centroid),
        "partial": partial,
        "associated_with_line": associated,
        "local_line_width_px": 21.0,
        "longitudinal": longitudinal,
        "lateral": lateral,
        "side": "DIREITA" if lateral > 0.0 else "ESQUERDA",
    }


class CameraProfilesTest(unittest.TestCase):
    def assert_control_preview(
        self,
        profile_role,
        near_valid,
        near_error,
        far_valid,
        far_error,
        expected,
    ):
        vision_profile = camera_line_frame.CAMERA_PROFILES[profile_role]["vision"]
        actual = camera_line_frame.calculate_control_preview(
            vision_profile,
            near_valid,
            near_error,
            far_valid,
            far_error,
        )

        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value, places=6)

    def test_forward_profile_preserves_original_capture(self):
        profile = camera_line_frame.CAMERA_PROFILES["forward"]

        self.assertEqual(profile["main_size"], (960, 540))
        self.assertEqual(profile["sensor_size"], (1920, 1080))
        self.assertEqual(profile["sensor_bit_depth"], 10)
        self.assertEqual(profile["target_fps"], 30)

    def test_down_profile_uses_full_fov_sensor_mode(self):
        profile = camera_line_frame.CAMERA_PROFILES["down"]

        self.assertEqual(profile["main_size"], (640, 480))
        self.assertEqual(profile["sensor_size"], (1640, 1232))
        self.assertEqual(profile["sensor_bit_depth"], 10)
        self.assertEqual(profile["target_fps"], 30)

    def test_argument_has_priority_over_environment(self):
        with mock.patch.dict(os.environ, {"OBR_CAMERA_ROLE": "forward"}):
            profile = camera_line_frame.parse_camera_profile(
                ["--camera-role", "down"]
            )

        self.assertEqual(profile["role"], "down")

    def test_environment_selects_profile_without_argument(self):
        with mock.patch.dict(os.environ, {"OBR_CAMERA_ROLE": "down"}):
            profile = camera_line_frame.parse_camera_profile([])

        self.assertEqual(profile["role"], "down")

    def test_scaler_crop_tuple_is_serialized_for_status(self):
        values = camera_line_frame.rectangle_values((0, 2, 3280, 2460))

        self.assertEqual(
            values,
            {"x": 0, "y": 2, "width": 3280, "height": 2460},
        )

    def test_down_guidance_on_centered_straight_line(self):
        self.assert_control_preview(
            "down", True, 0.0, True, 0.0,
            (0.0, 0.0, 0.0, 0.70, 0.70),
        )

    def test_down_guidance_on_parallel_offset_straight_line(self):
        self.assert_control_preview(
            "down", True, 0.30, True, 0.30,
            (0.30, 2.0 / 9.0, 1.0 / 15.0, 11.0 / 15.0, 2.0 / 3.0),
        )

    def test_down_guidance_anticipates_left_curve(self):
        self.assert_control_preview(
            "down", True, 0.036, True, -0.356,
            (-0.3168, -0.2408888889, -0.0722666667, 0.6638666667, 0.7361333333),
        )

    def test_down_guidance_anticipates_right_curve(self):
        self.assert_control_preview(
            "down", True, -0.036, True, 0.356,
            (0.3168, 0.2408888889, 0.0722666667, 0.7361333333, 0.6638666667),
        )

    def test_down_guidance_preserves_near_when_far_is_invalid(self):
        self.assert_control_preview(
            "down", True, -0.30, False, 0.0,
            (-0.30, -2.0 / 9.0, -1.0 / 15.0, 2.0 / 3.0, 11.0 / 15.0),
        )

    def test_down_guidance_stays_zero_when_near_is_invalid(self):
        self.assert_control_preview(
            "down", False, 0.0, True, -0.80,
            (0.0, 0.0, 0.0, 0.0, 0.0),
        )

    def test_down_guidance_clamps_combined_error(self):
        for near_error, far_error, expected_guidance in (
            (1.0, 3.0, 1.0),
            (-1.0, -3.0, -1.0),
        ):
            with self.subTest(expected_guidance=expected_guidance):
                result = camera_line_frame.calculate_control_preview(
                    camera_line_frame.CAMERA_PROFILES["down"]["vision"],
                    True,
                    near_error,
                    True,
                    far_error,
                )
                self.assertEqual(result[0], expected_guidance)
                self.assertGreaterEqual(result[1], -1.0)
                self.assertLessEqual(result[1], 1.0)

    def test_down_balanced_differential_mixer(self):
        vision_profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]
        cases = (
            (0.0, 0.0, 0.700, 0.700),
            (0.28, 0.06, 0.730, 0.670),
            (-0.28, -0.06, 0.670, 0.730),
            (0.55, 0.15, 0.750, 0.650),
            (-0.55, -0.15, 0.650, 0.750),
        )

        self.assertTrue(vision_profile["balanced_differential_mixing"])
        self.assertEqual(vision_profile["minimum_tracking_power"], 0.65)
        for guidance_error, correction, left, right in cases:
            with self.subTest(guidance_error=guidance_error):
                result = camera_line_frame.calculate_control_preview(
                    vision_profile, True, guidance_error, False, 0.0
                )
                self.assertAlmostEqual(result[2], correction, places=6)
                self.assertAlmostEqual(result[3], left, places=6)
                self.assertAlmostEqual(result[4], right, places=6)
                self.assertAlmostEqual((result[3] + result[4]) / 2.0, 0.70)
                self.assertGreaterEqual(result[3], 0.65)
                self.assertGreaterEqual(result[4], 0.65)
                self.assertLessEqual(result[3], 1.0)
                self.assertLessEqual(result[4], 1.0)

    def test_forward_profile_ignores_far_for_control(self):
        vision_profile = camera_line_frame.CAMERA_PROFILES["forward"]["vision"]
        with_far = camera_line_frame.calculate_control_preview(
            vision_profile, True, 0.036, True, -0.356
        )
        without_far = camera_line_frame.calculate_control_preview(
            vision_profile, True, 0.036, False, 0.0
        )

        self.assertEqual(with_far, without_far)
        self.assertEqual(with_far, (0.036, 0.0, 0.0, 0.65, 0.65))

    def test_forward_profile_preserves_one_sided_mixer(self):
        self.assert_control_preview(
            "forward", True, 0.28, False, 0.0,
            (0.28, 0.20, 0.06, 0.71, 0.65),
        )

    def test_down_deadzone_uses_ten_percent(self):
        vision_profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]

        self.assertEqual(vision_profile["near_deadzone_ratio"], 0.10)
        for error in (0.08, -0.08):
            with self.subTest(error=error):
                result = camera_line_frame.calculate_control_preview(
                    vision_profile, True, error, False, 0.0
                )
                self.assertEqual(result[1], 0.0)
                self.assertEqual(result[3:], (0.70, 0.70))
        for error in (0.11, -0.11):
            with self.subTest(error=error):
                result = camera_line_frame.calculate_control_preview(
                    vision_profile, True, error, False, 0.0
                )
                self.assertNotEqual(result[1], 0.0)
                self.assertEqual(result[1] > 0.0, error > 0.0)

    def test_forward_deadzone_remains_ten_percent(self):
        vision_profile = camera_line_frame.CAMERA_PROFILES["forward"]["vision"]

        self.assertEqual(vision_profile["near_deadzone_ratio"], 0.10)
        for error in (0.08, -0.08):
            with self.subTest(error=error):
                result = camera_line_frame.calculate_control_preview(
                    vision_profile, True, error, False, 0.0
                )
                self.assertEqual(result[1], 0.0)

    def test_profile_deadzone_pixel_limits_match_overlay_geometry(self):
        down_geometry = camera_line_frame.resolve_horizontal_deadzone(
            640,
            camera_line_frame.CAMERA_PROFILES["down"]["vision"],
        )
        forward_geometry = camera_line_frame.resolve_horizontal_deadzone(
            960,
            camera_line_frame.CAMERA_PROFILES["forward"]["vision"],
        )

        self.assertEqual(down_geometry, (32, 288, 352))
        self.assertEqual(forward_geometry, (48, 432, 528))

    def test_green_rgb888_array_uses_bgr_before_hsv(self):
        green_pixel = camera_line_frame.rgb_pixel_to_camera_array(0, 255, 0)
        red_pixel = camera_line_frame.rgb_pixel_to_camera_array(255, 0, 0)
        synthetic_frame = np.array([[green_pixel, red_pixel]], dtype=np.uint8)
        converted = object()

        with mock.patch.object(
            camera_line_frame.cv2,
            "cvtColor",
            return_value=converted,
        ) as convert:
            result = camera_line_frame.frame_to_hsv(synthetic_frame)

        self.assertIs(result, converted)
        self.assertEqual(green_pixel, (0, 255, 0))
        self.assertEqual(red_pixel, (0, 0, 255))
        convert.assert_called_once_with(
            synthetic_frame,
            camera_line_frame.cv2.COLOR_BGR2HSV,
        )

    def test_green_hsv_rejects_white_black_red_and_yellow(self):
        self.assertTrue(camera_line_frame.is_hsv_green(60, 255, 255))
        for name, hsv in (
            ("white", (0, 0, 255)),
            ("black", (0, 0, 0)),
            ("red", (0, 255, 255)),
            ("yellow", (30, 255, 255)),
        ):
            with self.subTest(color=name):
                self.assertFalse(camera_line_frame.is_hsv_green(*hsv))

    def test_green_small_spot_is_rejected_as_noise(self):
        self.assertFalse(camera_line_frame.green_geometry_is_valid(
            area=12.0,
            short_side=4.0,
            aspect_ratio=0.90,
            extent=0.80,
            partial=False,
        ))

    def test_green_square_is_geometrically_accepted(self):
        self.assertTrue(camera_line_frame.green_geometry_is_valid(
            area=576.0,
            short_side=24.0,
            aspect_ratio=1.0,
            extent=0.92,
            partial=False,
        ))
        if CV2_AVAILABLE:
            frame = np.full((480, 640, 3), 255, dtype=np.uint8)
            frame[190:216, 260:286] = (0, 255, 0)
            line_mask = synthetic_line_mask(left_branch=True)
            _, candidates, rejected = camera_line_frame.find_green_candidates(
                frame,
                425,
                line_mask,
                synthetic_line_axis(),
            )
            self.assertEqual(len(candidates), 1)
            self.assertEqual(len(rejected), 0)

    def test_green_partial_square_stays_candidate_without_confirmation(self):
        self.assertTrue(camera_line_frame.green_geometry_is_valid(
            area=40.0,
            short_side=4.0,
            aspect_ratio=0.50,
            extent=0.30,
            partial=True,
        ))
        candidate = synthetic_candidate((270, 200), partial=True)
        result = camera_line_frame.interpret_green_candidates(
            [candidate], synthetic_line_axis(), synthetic_topology(left_branch=True)
        )
        tracker = camera_line_frame.GreenObservationTracker()
        tracker_result = None
        for sequence in (1, 2, 3):
            tracker_result = tracker.update(sequence, result["interpretation"])
        self.assertEqual(result["interpretation"], "AMBIGUO")
        self.assertFalse(tracker_result[1])

    def test_green_fragmented_square_is_counted_once(self):
        groups = camera_line_frame.group_fragment_boxes((
            (100, 100, 10, 20),
            (117, 101, 10, 19),
        ))
        self.assertEqual(len(groups), 1)
        self.assertEqual(sorted(groups[0]), [0, 1])

    def test_green_left_before_left_branch(self):
        result = camera_line_frame.interpret_green_candidates(
            [synthetic_candidate((270, 200))],
            synthetic_line_axis(),
            synthetic_topology(left_branch=True),
        )
        self.assertEqual(result["interpretation"], "ESQUERDA")
        self.assertTrue(result["left_seen"])

    def test_green_right_before_right_branch(self):
        result = camera_line_frame.interpret_green_candidates(
            [synthetic_candidate((370, 200))],
            synthetic_line_axis(),
            synthetic_topology(right_branch=True),
        )
        self.assertEqual(result["interpretation"], "DIREITA")
        self.assertTrue(result["right_seen"])

    def test_green_diagonal_line_preserves_left_side(self):
        axis = camera_line_frame.build_line_axis((360, 280), (280, 180))
        marker = camera_line_frame.point_from_line_axis(axis, 60.0, -40.0)
        topology = synthetic_topology(left_branch=True)
        topology["junction_longitudinal"] = 120.0
        result = camera_line_frame.interpret_green_candidates(
            [synthetic_candidate(marker, axis)], axis, topology
        )
        self.assertEqual(result["interpretation"], "ESQUERDA")

    def test_green_diagonal_line_preserves_right_side(self):
        axis = camera_line_frame.build_line_axis((360, 280), (280, 180))
        marker = camera_line_frame.point_from_line_axis(axis, 60.0, 40.0)
        topology = synthetic_topology(right_branch=True)
        topology["junction_longitudinal"] = 120.0
        result = camera_line_frame.interpret_green_candidates(
            [synthetic_candidate(marker, axis)], axis, topology
        )
        self.assertEqual(result["interpretation"], "DIREITA")

    def test_green_after_junction_is_false_for_current_direction(self):
        result = camera_line_frame.interpret_green_candidates(
            [synthetic_candidate((270, 100))],
            synthetic_line_axis(),
            synthetic_topology(left_branch=True),
        )
        self.assertEqual(
            result["interpretation"],
            "VERDE_FALSO_NO_SENTIDO_ATUAL",
        )

    def test_green_before_junction_without_matching_branch_is_false(self):
        result = camera_line_frame.interpret_green_candidates(
            [synthetic_candidate((270, 200))],
            synthetic_line_axis(),
            synthetic_topology(),
        )
        self.assertEqual(
            result["interpretation"],
            "VERDE_FALSO_NO_SENTIDO_ATUAL",
        )

    def test_green_opposite_pair_before_line_end_is_return(self):
        result = camera_line_frame.interpret_green_candidates(
            [
                synthetic_candidate((270, 200)),
                synthetic_candidate((370, 202)),
            ],
            synthetic_line_axis(),
            synthetic_topology(),
        )
        self.assertEqual(result["interpretation"], "RETORNO_180")
        self.assertTrue(result["pair_compatible"])

    def test_green_return_pair_is_never_individually_false(self):
        result = camera_line_frame.interpret_green_candidates(
            [
                synthetic_candidate((270, 200)),
                synthetic_candidate((370, 200)),
            ],
            synthetic_line_axis(),
            synthetic_topology(),
        )
        self.assertNotEqual(
            result["interpretation"],
            "VERDE_FALSO_NO_SENTIDO_ATUAL",
        )
        self.assertEqual(result["interpretation"], "RETORNO_180")

    def test_green_two_fragments_cannot_produce_return(self):
        groups = camera_line_frame.group_fragment_boxes((
            (250, 190, 12, 22),
            (268, 191, 12, 21),
        ))
        candidates = [synthetic_candidate((270, 200)) for _ in groups]
        result = camera_line_frame.interpret_green_candidates(
            candidates,
            synthetic_line_axis(),
            synthetic_topology(left_branch=True),
        )
        self.assertEqual(len(candidates), 1)
        self.assertNotEqual(result["interpretation"], "RETORNO_180")

    def test_green_intersection_without_marker_has_no_decision(self):
        result = camera_line_frame.interpret_green_candidates(
            [],
            synthetic_line_axis(),
            synthetic_topology(left_branch=True, right_branch=True),
        )
        self.assertEqual(result["observation_state"], "SEM_VERDE")
        self.assertEqual(result["interpretation"], "SEM_DECISAO")

    def test_green_insufficient_topology_is_ambiguous(self):
        result = camera_line_frame.interpret_green_candidates(
            [synthetic_candidate((270, 200))],
            synthetic_line_axis(),
            {"junction_valid": False, "confidence": 0.0},
        )
        self.assertEqual(result["interpretation"], "AMBIGUO")

    def test_green_three_new_frames_confirm_but_repeated_frame_does_not(self):
        tracker = camera_line_frame.GreenObservationTracker()
        first = tracker.update(10, "ESQUERDA")
        repeated = tracker.update(10, "ESQUERDA")
        second = tracker.update(11, "ESQUERDA")
        third = tracker.update(12, "ESQUERDA")

        self.assertEqual(first[2], 1)
        self.assertEqual(first[0], "SEM_DECISAO")
        self.assertEqual(repeated[2], 1)
        self.assertEqual(repeated[0], "SEM_DECISAO")
        self.assertEqual(second[2], 2)
        self.assertEqual(second[0], "SEM_DECISAO")
        self.assertFalse(second[1])
        self.assertEqual(third, ("ESQUERDA", True, 3))

    def test_green_disappearance_clears_after_hysteresis(self):
        tracker = camera_line_frame.GreenObservationTracker()
        for sequence in (1, 2, 3):
            tracker.update(sequence, "DIREITA")

        first_missing = tracker.update(4, "SEM_DECISAO")
        second_missing = tracker.update(5, "SEM_DECISAO")

        self.assertEqual(first_missing[0], "DIREITA")
        self.assertTrue(first_missing[1])
        self.assertEqual(second_missing, ("SEM_DECISAO", False, 0))

    def test_green_changes_do_not_modify_line_preview_calculation(self):
        self.assert_control_preview(
            "down", True, 0.036, True, -0.356,
            (-0.3168, -0.2408888889, -0.0722666667, 0.6638666667, 0.7361333333),
        )
        self.assert_control_preview(
            "forward", True, 0.28, False, 0.0,
            (0.28, 0.20, 0.06, 0.71, 0.65),
        )

    def test_green_fast_status_contains_only_finite_numbers(self):
        candidate = synthetic_candidate((270, 200))
        candidate["centroid"] = (float("nan"), float("inf"))
        candidate["area"] = float("nan")
        interpretation = {
            "observation_state": "UM_CANDIDATO",
            "interpretation": "AMBIGUO",
            "left_seen": True,
            "right_seen": False,
            "pair_compatible": False,
        }
        status = camera_line_frame.build_green_status(
            [candidate],
            0,
            interpretation,
            {"junction_valid": False},
            ("AMBIGUO", False, 1),
            float("nan"),
        )

        for value in status.values():
            if isinstance(value, float):
                self.assertTrue(np.isfinite(value))


if __name__ == "__main__":
    unittest.main()
