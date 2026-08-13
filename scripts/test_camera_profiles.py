import importlib.util
import os
import sys
import unittest
from unittest import mock


SCRIPT_PATH = os.path.join(os.path.dirname(__file__), "camera_line_frame.py")
sys.modules.setdefault("cv2", mock.MagicMock())
SPEC = importlib.util.spec_from_file_location("camera_line_frame", SCRIPT_PATH)
camera_line_frame = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(camera_line_frame)


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


if __name__ == "__main__":
    unittest.main()
