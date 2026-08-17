import importlib.util
import json
import os
import sys
import tempfile
import unittest
from unittest import mock


SCRIPT_DIRECTORY = os.path.dirname(__file__)
CAMERA_SCRIPT_PATH = os.path.join(SCRIPT_DIRECTORY, "camera_line_frame.py")
FORWARD_SCRIPT_PATH = os.path.join(SCRIPT_DIRECTORY, "forward_camera_stream.py")

CV2_ALREADY_LOADED = "cv2" in sys.modules
try:
    import cv2  # type: ignore[import]
except ImportError:
    sys.modules.setdefault("cv2", mock.MagicMock())

CAMERA_SPEC = importlib.util.spec_from_file_location(
    "camera_line_frame",
    CAMERA_SCRIPT_PATH,
)
camera_line_frame = importlib.util.module_from_spec(CAMERA_SPEC)
CAMERA_SPEC.loader.exec_module(camera_line_frame)
sys.modules["camera_line_frame"] = camera_line_frame
if not CV2_ALREADY_LOADED:
    # O módulo de câmera já guardou sua referência; não contamine outros testes.
    sys.modules.pop("cv2", None)

FORWARD_SPEC = importlib.util.spec_from_file_location(
    "forward_camera_stream",
    FORWARD_SCRIPT_PATH,
)
forward_camera_stream = importlib.util.module_from_spec(FORWARD_SPEC)
FORWARD_SPEC.loader.exec_module(forward_camera_stream)


class ForwardCameraStreamTest(unittest.TestCase):
    def test_environment_defaults_to_disabled(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertFalse(forward_camera_stream.environment_enabled())

    def test_environment_can_enable_camera_at_startup(self):
        with mock.patch.dict(
            os.environ,
            {"OBR_FORWARD_CAMERA_ENABLED": "true"},
            clear=True,
        ):
            self.assertTrue(forward_camera_stream.environment_enabled())

    def test_control_file_round_trip(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            control_path = os.path.join(temporary_directory, "enabled")
            temporary_path = os.path.join(temporary_directory, "enabled.tmp")
            with mock.patch.object(
                forward_camera_stream,
                "CONTROL_PATH",
                control_path,
            ), mock.patch.object(
                forward_camera_stream,
                "TEMP_CONTROL_PATH",
                temporary_path,
            ):
                forward_camera_stream.write_requested_enabled(True)
                self.assertTrue(forward_camera_stream.requested_enabled())
                forward_camera_stream.write_requested_enabled(False)
                self.assertFalse(forward_camera_stream.requested_enabled())

    def test_disabled_status_keeps_forward_profile_without_line_data(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            status_path = os.path.join(temporary_directory, "status.json")
            temporary_path = os.path.join(temporary_directory, "status.tmp.json")
            with mock.patch.object(
                forward_camera_stream,
                "STATUS_PATH",
                status_path,
            ), mock.patch.object(
                forward_camera_stream,
                "TEMP_STATUS_PATH",
                temporary_path,
            ), mock.patch.dict(os.environ, {}, clear=True):
                forward_camera_stream.save_status(False, False, "disabled")

            with open(status_path, "r", encoding="utf-8") as status_file:
                status = json.load(status_file)

        self.assertFalse(status["enabled"])
        self.assertFalse(status["active"])
        self.assertEqual(status["state"], "disabled")
        self.assertEqual(status["cameraRole"], "forward")
        self.assertEqual(status["mainResolution"], {"width": 960, "height": 540})
        self.assertEqual(status["sensorMode"]["width"], 1920)
        self.assertEqual(status["sensorMode"]["height"], 1080)
        self.assertNotIn("lineSequence", status)
        self.assertNotIn("nearError", status)


if __name__ == "__main__":
    unittest.main()
