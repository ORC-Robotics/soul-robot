import importlib.util
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np


SCRIPT_DIRECTORY = Path(__file__).resolve().parents[2] / "scripts"
CAMERA_SCRIPT_PATH = SCRIPT_DIRECTORY / "camera_line_frame.py"
FORWARD_SCRIPT_PATH = SCRIPT_DIRECTORY / "forward_camera_stream.py"
sys.path.insert(0, str(SCRIPT_DIRECTORY))

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
    def test_environment_defaults_to_enabled(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertTrue(forward_camera_stream.environment_enabled())

    def test_environment_can_disable_camera_at_startup(self):
        with mock.patch.dict(
            os.environ,
            {"OBR_FORWARD_CAMERA_ENABLED": "false"},
            clear=True,
        ):
            self.assertFalse(forward_camera_stream.environment_enabled())

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

    def test_missing_control_file_defaults_to_enabled(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            missing_path = os.path.join(temporary_directory, "missing")
            with mock.patch.object(
                forward_camera_stream,
                "CONTROL_PATH",
                missing_path,
            ):
                self.assertTrue(forward_camera_stream.requested_enabled())

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

    def test_disabled_ball_detection_skips_heavy_pipeline(self):
        frame = np.full((540, 960, 3), 255, dtype=np.uint8)
        with mock.patch.object(
            forward_camera_stream.ball_vision_pipeline,
            "analyze",
        ) as detector:
            observation, candidates, status = (
                forward_camera_stream.analyze_requested_ball_frame(frame, False)
            )

        detector.assert_not_called()
        self.assertIsNone(observation)
        self.assertEqual(candidates, ())
        self.assertFalse(status["ballDetectionEnabled"])
        self.assertFalse(status["ballDetected"])
        self.assertIsNone(status["ballTxDegrees"])

    def test_ball_detection_has_independent_rate_limit(self):
        interval = 1.0 / forward_camera_stream.BALL_DETECTION_FPS

        self.assertTrue(forward_camera_stream.ball_analysis_due(10.0, 0.0))
        self.assertFalse(
            forward_camera_stream.ball_analysis_due(
                10.0 + interval * 0.5,
                10.0,
            )
        )
        self.assertTrue(
            forward_camera_stream.ball_analysis_due(
                10.0 + interval * 1.1,
                10.0,
            )
        )

    def test_fast_ball_status_contains_only_control_fields(self):
        ball_status = forward_camera_stream.empty_ball_status()
        ball_status.update({
            "ballDetected": True,
            "ballType": "black_ball",
            "ballTxDegrees": 7.5,
            "ballDistanceCm": 42.0,
            "ballRadiusPixels": 70.0,
            "visibleAreaPixels": 15000.0,
            "targetSequence": 8,
            "targetLocked": True,
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
        self.assertEqual(status["visibleAreaPixels"], 15000.0)
        self.assertEqual(status["targetSequence"], 8)
        self.assertTrue(status["targetLocked"])
        self.assertNotIn("ballPayload", status)

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
        self.assertFalse(status["processingActive"])
        self.assertEqual(status["state"], "disabled")
        self.assertEqual(status["cameraRole"], "forward")
        self.assertEqual(status["mainResolution"], {"width": 960, "height": 540})
        self.assertEqual(status["sensorMode"]["width"], 1920)
        self.assertEqual(status["sensorMode"]["height"], 1080)
        self.assertEqual(status["targetCameraFps"], 30)
        self.assertEqual(status["rotationDegrees"], 180)
        self.assertEqual(status["captureTransform"], "identity")
        self.assertEqual(status["transform"], "opencv-rotate-180")
        self.assertNotIn("lineSequence", status)

    def test_forward_frame_is_rotated_before_processing(self):
        frame = np.arange(2 * 3 * 3, dtype=np.uint8).reshape((2, 3, 3))

        oriented = forward_camera_stream.orient_forward_frame(frame)

        np.testing.assert_array_equal(oriented, frame[::-1, ::-1])

    def test_downward_process_rejects_forward_role(self):
        with mock.patch("sys.stderr"), self.assertRaises(SystemExit):
            camera_line_frame.parse_camera_profile(["--camera-role", "forward"])

    def test_forward_processing_reuses_only_forward_black_segmentation(self):
        frame = np.zeros((100, 200, 3), dtype=np.uint8)
        filtered_mask = np.zeros((100, 200), dtype=np.uint8)
        filtered_mask[60:95, 95:105] = 255
        with mock.patch.object(
            camera_line_frame,
            "create_filtered_line_mask",
            return_value=(filtered_mask, 0),
        ) as create_mask:
            reading = forward_camera_stream.process_forward_frame(frame, "RGB888")

        create_mask.assert_called_once_with(
            frame,
            camera_line_frame.CAMERA_PROFILES["forward"]["vision"],
            "RGB888",
        )
        self.assertTrue(reading["forwardLineVisible"])
        self.assertAlmostEqual(reading["forwardLinePosition"], 0.0, places=6)

    def test_forward_line_on_left_has_negative_position(self):
        mask = np.zeros((100, 200), dtype=np.uint8)
        mask[60:95, 20:30] = 255

        reading = forward_camera_stream.calculate_forward_line_assist(mask)

        self.assertTrue(reading["forwardLineVisible"])
        self.assertLess(reading["forwardLinePosition"], 0.0)
        self.assertGreater(reading["forwardLineConfidence"], 0.0)

    def test_forward_line_in_center_has_position_near_zero(self):
        mask = np.zeros((100, 200), dtype=np.uint8)
        mask[60:95, 95:105] = 255

        reading = forward_camera_stream.calculate_forward_line_assist(mask)

        self.assertTrue(reading["forwardLineVisible"])
        self.assertAlmostEqual(reading["forwardLinePosition"], 0.0, places=6)

    def test_forward_line_on_right_has_positive_position(self):
        mask = np.zeros((100, 200), dtype=np.uint8)
        mask[60:95, 170:180] = 255

        reading = forward_camera_stream.calculate_forward_line_assist(mask)

        self.assertTrue(reading["forwardLineVisible"])
        self.assertGreater(reading["forwardLinePosition"], 0.0)

    def test_empty_forward_roi_is_not_visible(self):
        mask = np.zeros((100, 200), dtype=np.uint8)

        reading = forward_camera_stream.calculate_forward_line_assist(mask)

        self.assertFalse(reading["forwardLineVisible"])
        self.assertIsNone(reading["forwardLinePosition"])
        self.assertEqual(reading["forwardLineConfidence"], 0.0)

    def test_forward_line_status_uses_dedicated_atomic_ipc(self):
        reading = {
            "forwardLineVisible": True,
            "forwardLinePosition": -0.25,
            "forwardLineConfidence": 0.04,
        }
        with tempfile.TemporaryDirectory() as temporary_directory:
            status_path = os.path.join(temporary_directory, "forward.json")
            temporary_path = os.path.join(temporary_directory, "forward.tmp.json")
            with mock.patch.object(
                forward_camera_stream,
                "FORWARD_LINE_STATUS_PATH",
                status_path,
            ), mock.patch.object(
                forward_camera_stream,
                "TEMP_FORWARD_LINE_STATUS_PATH",
                temporary_path,
            ):
                saved = forward_camera_stream.save_forward_line_status(
                    reading,
                    timestamp=123.5,
                    sequence=7,
                )

            with open(status_path, "r", encoding="utf-8") as status_file:
                status = json.load(status_file)

        self.assertTrue(saved)
        self.assertEqual(
            set(status),
            {
                "forwardLineVisible",
                "forwardLinePosition",
                "forwardLineConfidence",
                "forwardLineSequence",
                "forwardLineTimestamp",
                "forwardLineNormalLeftPower",
                "forwardLineNormalRightPower",
                "forwardPathVersion",
                "forwardLinePresent",
                "forwardPathState",
                "forwardPathConfidence",
                "forwardPathReferenceValid",
                "forwardPathReferenceSequence",
                "forwardPathReferenceTimestamp",
                "forwardPathComponents",
            },
        )
        self.assertEqual(status["forwardLineSequence"], 7)
        self.assertEqual(status["forwardLineTimestamp"], 123.5)
        self.assertIsNone(status["forwardLineNormalRightPower"])
        self.assertIsNone(status["forwardLineNormalLeftPower"])
        self.assertEqual(status["forwardPathVersion"], 2)
        self.assertFalse(status["forwardPathReferenceValid"])

    def test_paired_ipc_uses_recent_bottom_reference_and_confirms_native_gap(self):
        from vision import status_publisher
        from vision.fusion_guidance import extract_fusion_style_line
        from vision.gap_validation import GapValidator, read_json_snapshot
        from vision.line_control import LineFollowerController
        from vision.maneuver_state import LineManeuverState

        bottom = np.zeros((360, 480), np.uint8)
        bottom[:, 220:260] = 255
        fusion = extract_fusion_style_line(bottom)
        controller, validator = LineFollowerController(), GapValidator()
        command = controller.calculate(bottom, {}, fusion_style_line=fusion)
        validator.remember(bottom, fusion, command, 100.0, 10)
        command.update(validator.diagnostics())
        front = np.full((540, 960, 3), 255, np.uint8)
        front[:, 462:498] = 0
        maneuver = LineManeuverState()
        tracker = forward_camera_stream.ForwardPathTracker()
        with tempfile.TemporaryDirectory() as folder:
            bottom_path, front_path = Path(folder) / "bottom.json", Path(folder) / "front.json"
            with mock.patch.object(status_publisher, "LINE_STATUS_PATH", bottom_path), \
                    mock.patch.object(status_publisher, "TEMP_LINE_STATUS_PATH", str(bottom_path) + ".tmp"), \
                    mock.patch.object(forward_camera_stream, "FORWARD_LINE_STATUS_PATH", front_path), \
                    mock.patch.object(forward_camera_stream, "TEMP_FORWARD_LINE_STATUS_PATH", str(front_path) + ".tmp"):
                status_publisher.save_line_status(command, 100.0, 10, {})
                observed_reference = read_json_snapshot(bottom_path)["bottomPathReference"]
                for sequence, seconds in ((1, 0.02), (2, 0.06)):
                    observed = forward_camera_stream.process_forward_frame(
                        front, "RGB888", observed_reference, tracker, 100 + seconds)
                    self.assertTrue(forward_camera_stream.save_forward_line_status(observed, 100 + seconds, sequence))
                    snapshot = read_json_snapshot(front_path)
                    validator.apply(maneuver, controller, True, False, False,
                                    snapshot, seconds, 100 + seconds)
                self.assertEqual(validator.decision, "GAP")
                self.assertTrue(maneuver.gap_forward_active)
                self.assertIsNone(snapshot["forwardLineNormalLeftPower"])
                empty = np.zeros_like(bottom)
                crossing = controller.calculate(empty, {}, gap_forward_active=maneuver.gap_forward_active)
                self.assertEqual(crossing["controlSource"], "gap-forward")

    def test_bottom_mapper_remains_inside_original_normal_range(self):
        for position in (-1.0, -0.2, 0.0, 0.2, 1.0):
            command = camera_line_frame.map_normal_steering_error(position)

            self.assertIsNotNone(command)
            self.assertGreaterEqual(
                command["left_power"],
                camera_line_frame.NORMAL_INNER_MIN_POWER,
            )
            self.assertGreaterEqual(
                command["right_power"],
                camera_line_frame.NORMAL_INNER_MIN_POWER,
            )
            self.assertLessEqual(
                command["left_power"],
                camera_line_frame.NORMAL_MAX_POWER,
            )
            self.assertLessEqual(
                command["right_power"],
                camera_line_frame.NORMAL_MAX_POWER,
            )


if __name__ == "__main__":
    unittest.main()
