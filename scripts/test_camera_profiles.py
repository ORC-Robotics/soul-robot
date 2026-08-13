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


if __name__ == "__main__":
    unittest.main()
