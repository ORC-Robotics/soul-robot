import importlib.util
import json
import os
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np


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
    def setUp(self):
        forward_camera_stream.ball_tracker.reset()
        forward_camera_stream.active_stream_clients = 0

    @staticmethod
    def silver_ball_frame():
        """Gera uma bola clara e texturizada sem depender da câmera real."""

        frame = np.full((540, 960, 3), (205, 220, 205), dtype=np.uint8)
        center = (300, 180)
        radius = 75
        cv2.circle(frame, center, radius, (145, 155, 150), -1, cv2.LINE_AA)
        cv2.circle(frame, center, radius, (75, 90, 80), 4, cv2.LINE_AA)
        random = np.random.default_rng(321)
        for _ in range(75):
            angle = float(random.uniform(0.0, 2.0 * np.pi))
            start_radius = float(random.uniform(0.0, radius * 0.65))
            length = float(random.uniform(radius * 0.25, radius * 0.90))
            start = (
                int(center[0] + np.cos(angle) * start_radius),
                int(center[1] + np.sin(angle) * start_radius),
            )
            end_angle = angle + float(random.uniform(-1.0, 1.0))
            end = (
                int(start[0] + np.cos(end_angle) * length),
                int(start[1] + np.sin(end_angle) * length),
            )
            value = int(random.integers(70, 230))
            cv2.line(frame, start, end, (value, value, value), 2, cv2.LINE_AA)
        return frame

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

    def test_ball_detection_control_file_round_trip(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            control_path = os.path.join(temporary_directory, "ball_enabled")
            temporary_path = os.path.join(
                temporary_directory,
                "ball_enabled.tmp",
            )
            with mock.patch.object(
                forward_camera_stream,
                "BALL_DETECTION_CONTROL_PATH",
                control_path,
            ), mock.patch.object(
                forward_camera_stream,
                "TEMP_BALL_DETECTION_CONTROL_PATH",
                temporary_path,
            ):
                forward_camera_stream.write_requested_ball_detection_enabled(True)
                self.assertTrue(
                    forward_camera_stream.requested_ball_detection_enabled()
                )
                forward_camera_stream.write_requested_ball_detection_enabled(False)
                self.assertFalse(
                    forward_camera_stream.requested_ball_detection_enabled()
                )

    def test_disabled_ball_detection_skips_opencv_pipeline(self):
        frame = np.full((540, 960, 3), 255, dtype=np.uint8)
        with mock.patch.object(forward_camera_stream, "analyze_frame") as detector:
            observation, candidates, status = (
                forward_camera_stream.analyze_requested_ball_frame(frame, False)
            )

        detector.assert_not_called()
        self.assertIsNone(observation)
        self.assertEqual(candidates, [])
        self.assertFalse(status["ballDetectionEnabled"])
        self.assertFalse(status["ballDetected"])
        self.assertIsNone(status["ballTxDegrees"])

    def test_fast_ball_status_contains_control_fields(self):
        ball_status = forward_camera_stream.empty_ball_status()
        ball_status.update({
            "ballDetected": True,
            "ballType": "black_ball",
            "ballTxDegrees": 7.5,
            "ballDistanceCm": 42.0,
            "ballRadiusPixels": 70.0,
        })
        with tempfile.TemporaryDirectory() as temporary_directory:
            status_path = os.path.join(temporary_directory, "ball.json")
            temporary_path = os.path.join(temporary_directory, "ball.tmp.json")
            with mock.patch.object(
                forward_camera_stream,
                "BALL_STATUS_PATH",
                status_path,
            ), mock.patch.object(
                forward_camera_stream,
                "TEMP_BALL_STATUS_PATH",
                temporary_path,
            ):
                forward_camera_stream.save_ball_control_status(True, ball_status)

            with open(status_path, "r", encoding="utf-8") as status_file:
                status = json.load(status_file)

        self.assertTrue(status["active"])
        self.assertTrue(status["ballDetected"])
        self.assertEqual(status["ballType"], "black_ball")
        self.assertEqual(status["ballTxDegrees"], 7.5)
        self.assertNotIn("ballPayload", status)

    def test_stream_encoding_requires_connected_client_and_respects_fps(self):
        self.assertFalse(forward_camera_stream.stream_frame_is_due(10.0, 0.0))

        forward_camera_stream.register_stream_client()
        interval = 1.0 / forward_camera_stream.STREAM_FPS
        self.assertFalse(
            forward_camera_stream.stream_frame_is_due(10.0, 10.0 - interval / 2.0)
        )
        self.assertTrue(
            forward_camera_stream.stream_frame_is_due(10.0, 10.0 - interval)
        )

        forward_camera_stream.unregister_stream_client()
        self.assertFalse(forward_camera_stream.stream_frame_is_due(10.0, 0.0))

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
        self.assertEqual(status["targetCameraFps"], 15)
        self.assertEqual(status["streamFps"], 12)
        self.assertEqual(status["opencvThreads"], 2)
        self.assertEqual(status["silverProcessingScale"], 0.5)
        self.assertNotIn("lineSequence", status)
        self.assertFalse(status["ballDetectionEnabled"])
        self.assertFalse(status["ballDetected"])
        self.assertIsNone(status["ballPayload"])

    def test_ball_frame_is_annotated_and_published_in_status(self):
        frame = np.full((540, 960, 3), 255, dtype=np.uint8)
        cv2.circle(frame, (700, 270), 72, (0, 0, 0), -1)

        display, status = forward_camera_stream.process_ball_frame(frame)

        self.assertEqual(display.shape, frame.shape)
        self.assertTrue(status["ballDetected"])
        self.assertEqual(status["ballType"], "black_ball")
        self.assertEqual(status["ballPosition"], "direita")
        self.assertAlmostEqual(status["ballCenterX"], 700.0, delta=1.0)
        self.assertAlmostEqual(status["ballCenterY"], 270.0, delta=1.0)
        self.assertGreater(status["ballRadiusPixels"], 70.0)
        self.assertFalse(status["ballTopClipped"])
        self.assertEqual(status["ballDetectionMethod"], "contour")
        self.assertAlmostEqual(
            status["ballTxDegrees"],
            status["ballAngleDegrees"],
        )
        self.assertEqual(status["ballPayload"]["type"], "black_ball")

    def test_silver_ball_is_annotated_and_published_in_status(self):
        display, status = forward_camera_stream.process_ball_frame(
            self.silver_ball_frame()
        )

        self.assertEqual(display.shape, (540, 960, 3))
        self.assertTrue(status["ballDetected"])
        self.assertEqual(status["ballType"], "silver_ball")
        self.assertEqual(status["ballDetectionMethod"], "hough")
        self.assertEqual(status["ballPosition"], "esquerda")
        self.assertEqual(status["ballPayload"]["type"], "silver_ball")

    def test_empty_frame_clears_previous_ball_values(self):
        frame = np.full((540, 960, 3), 255, dtype=np.uint8)

        _, status = forward_camera_stream.process_ball_frame(frame)

        self.assertFalse(status["ballDetected"])
        self.assertIsNone(status["ballCenterX"])
        self.assertIsNone(status["ballDistanceCm"])
        self.assertIsNone(status["ballPayload"])


if __name__ == "__main__":
    unittest.main()
