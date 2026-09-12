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
    def test_yolo_mode_skips_forward_assist_processing(self):
        frame = np.zeros((20, 30, 3), dtype=np.uint8)
        with mock.patch.object(
            forward_camera_stream,
            "process_forward_frame",
        ) as process_forward_frame:
            reading = forward_camera_stream.process_forward_frame_for_mode(
                frame,
                "RGB888",
                True,
            )

        process_forward_frame.assert_not_called()
        self.assertFalse(reading["forwardLineVisible"])
        self.assertEqual(reading["source"], "YOLO_EXCLUSIVE")

    def test_rescue_zone_frame_obstruction_covers_real_closeup_patterns(self):
        dark_frame = np.full((120, 160, 3), (18, 14, 51), dtype=np.uint8)
        cyan_frame = np.full((120, 160, 3), (220, 190, 20), dtype=np.uint8)
        textured_frame = np.zeros((120, 160, 3), dtype=np.uint8)
        textured_frame[:, ::2] = 255

        self.assertTrue(
            forward_camera_stream.measure_rescue_zone_frame_obstruction(
                dark_frame
            )["obscured"]
        )
        self.assertTrue(
            forward_camera_stream.measure_rescue_zone_frame_obstruction(
                cyan_frame
            )["obscured"]
        )
        self.assertFalse(
            forward_camera_stream.measure_rescue_zone_frame_obstruction(
                textured_frame
            )["obscured"]
        )

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

    def test_ball_detection_gate_defaults_to_disabled(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            missing_path = os.path.join(temporary_directory, "missing")
            with mock.patch.object(
                forward_camera_stream,
                "BALL_DETECTION_CONTROL_PATH",
                missing_path,
            ):
                self.assertFalse(
                    forward_camera_stream.requested_ball_detection_enabled()
                )

    def test_disabled_ball_gate_never_calls_yolo(self):
        fake_pipeline = mock.Mock()
        fake_pipeline.target_locked = False
        frame = np.zeros((20, 30, 3), dtype=np.uint8)

        with mock.patch.object(
            forward_camera_stream,
            "ball_vision_pipeline",
            fake_pipeline,
        ):
            observation, candidates, status = (
                forward_camera_stream.analyze_requested_ball_frame(
                    frame,
                    False,
                )
            )

        fake_pipeline.analyze.assert_not_called()
        self.assertIsNone(observation)
        self.assertEqual(candidates, ())
        self.assertFalse(status["ballDetectionEnabled"])
        self.assertFalse(status["ballDetected"])

    def test_target_sequence_change_is_detected_only_once(self):
        previous_sequence = forward_camera_stream.active_ball_target_sequence
        previous_type = forward_camera_stream.active_ball_target_type
        try:
            forward_camera_stream.active_ball_target_sequence = 0
            forward_camera_stream.active_ball_target_type = "any"
            with mock.patch.object(
                forward_camera_stream.ball_vision_pipeline,
                "set_target_type",
            ) as set_target_type:
                self.assertTrue(
                    forward_camera_stream.synchronize_ball_target(
                        17,
                        "silver_ball",
                    )
                )
                self.assertFalse(
                    forward_camera_stream.synchronize_ball_target(
                        17,
                        "silver_ball",
                    )
                )
                set_target_type.assert_not_called()
            self.assertEqual(
                forward_camera_stream.active_ball_target_sequence,
                17,
            )
            self.assertEqual(
                forward_camera_stream.active_ball_target_type,
                "silver_ball",
            )
        finally:
            forward_camera_stream.active_ball_target_sequence = previous_sequence
            forward_camera_stream.active_ball_target_type = previous_type

    def test_ball_target_control_reads_sequence_and_requested_type(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            target_path = os.path.join(temporary_directory, "target.json")
            with open(target_path, "w", encoding="utf-8") as target_file:
                json.dump(
                    {
                        "targetSequence": 42,
                        "targetType": "black_ball",
                    },
                    target_file,
                )
            with mock.patch.object(
                forward_camera_stream,
                "BALL_TARGET_SEQUENCE_CONTROL_PATH",
                target_path,
            ):
                self.assertEqual(
                    forward_camera_stream.requested_ball_target(),
                    (42, "black_ball"),
                )

    def test_cam1_loop_starts_with_ball_detector_disabled(self):
        class FakeCamera:
            def __init__(self):
                self.stopped = False
                self.closed = False

            def capture_array(self, stream_name):
                self.stream_name = stream_name
                forward_camera_stream.running = False
                return np.zeros((20, 30, 3), dtype=np.uint8)

            def stop(self):
                self.stopped = True

            def close(self):
                self.closed = True

        class FakeServer:
            def __init__(self):
                self.shutdown_called = False
                self.closed = False

            def shutdown(self):
                self.shutdown_called = True

            def server_close(self):
                self.closed = True

        fake_camera = FakeCamera()
        fake_server = FakeServer()
        fake_pipeline = mock.Mock()
        fake_pipeline.target_locked = False
        fake_pipeline.silver_processing_scale = 1.0
        fake_pipeline.analyze.side_effect = AssertionError(
            "YOLO não deve executar com o gate desligado."
        )
        empty_reading = {
            "forwardLineVisible": False,
            "forwardLinePosition": None,
            "forwardLineConfidence": 0.0,
        }

        with tempfile.TemporaryDirectory() as temporary_directory:
            path = lambda name: os.path.join(temporary_directory, name)
            patches = (
                mock.patch.object(forward_camera_stream, "CONTROL_PATH", path("camera")),
                mock.patch.object(forward_camera_stream, "TEMP_CONTROL_PATH", path("camera.tmp")),
                mock.patch.object(forward_camera_stream, "STATUS_PATH", path("status.json")),
                mock.patch.object(forward_camera_stream, "TEMP_STATUS_PATH", path("status.tmp")),
                mock.patch.object(forward_camera_stream, "BALL_DETECTION_CONTROL_PATH", path("ball-gate")),
                mock.patch.object(forward_camera_stream, "TEMP_BALL_DETECTION_CONTROL_PATH", path("ball-gate.tmp")),
                mock.patch.object(forward_camera_stream, "BALL_TARGET_SEQUENCE_CONTROL_PATH", path("sequence")),
                mock.patch.object(forward_camera_stream, "BALL_STATUS_PATH", path("ball.json")),
                mock.patch.object(forward_camera_stream, "TEMP_BALL_STATUS_PATH", path("ball.tmp")),
                mock.patch.object(forward_camera_stream, "SilverDatasetRecorder", None),
                mock.patch.object(forward_camera_stream, "ball_vision_pipeline", fake_pipeline),
                mock.patch.object(forward_camera_stream, "start_stream_server", return_value=fake_server),
                mock.patch.object(
                    forward_camera_stream,
                    "open_forward_camera",
                    return_value=(fake_camera, "RGB888", {}),
                ),
                mock.patch.object(
                    forward_camera_stream,
                    "process_forward_frame",
                    return_value=empty_reading,
                ),
                mock.patch.object(
                    forward_camera_stream,
                    "save_forward_line_status",
                    return_value=True,
                ),
                mock.patch.object(
                    forward_camera_stream,
                    "read_json_snapshot",
                    return_value={},
                ),
                mock.patch.object(
                    forward_camera_stream,
                    "read_rescue_zone_detection_input",
                    return_value={"enabled": False},
                ),
                mock.patch.object(
                    forward_camera_stream,
                    "clear_forward_line_status",
                ),
                mock.patch.object(
                    forward_camera_stream,
                    "clear_rescue_zone_status",
                ),
                mock.patch.object(
                    forward_camera_stream,
                    "stream_has_clients",
                    return_value=False,
                ),
                mock.patch.object(forward_camera_stream.signal, "signal"),
                mock.patch.object(
                    forward_camera_stream.camera_line_frame,
                    "Picamera2",
                    object(),
                ),
            )
            forward_camera_stream.running = True
            try:
                for patcher in patches:
                    patcher.start()
                result = forward_camera_stream.main()
            finally:
                for patcher in reversed(patches):
                    patcher.stop()
                forward_camera_stream.running = True

        self.assertEqual(result, 0)
        fake_pipeline.analyze.assert_not_called()
        self.assertTrue(fake_camera.stopped)
        self.assertTrue(fake_camera.closed)
        self.assertTrue(fake_server.shutdown_called)
        self.assertTrue(fake_server.closed)

    def test_rescue_zone_gate_defaults_to_disabled(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            missing_path = os.path.join(temporary_directory, "missing")
            with mock.patch.object(
                forward_camera_stream,
                "RESCUE_ZONE_CONTROL_PATH",
                missing_path,
            ):
                self.assertFalse(
                    forward_camera_stream.requested_rescue_zone_detection_enabled()
                )

    def test_rescue_zone_input_validates_freshness_and_ultrasonic_range(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            input_path = os.path.join(temporary_directory, "zones_input.json")
            with open(input_path, "w", encoding="utf-8") as input_file:
                json.dump(
                    {
                        "enabled": True,
                        "ultrasonicFresh": True,
                        "ultrasonicValid": True,
                        "ultrasonicDistanceCm": 43.7,
                        "timestamp": 100.0,
                    },
                    input_file,
                )
            with mock.patch.object(
                forward_camera_stream,
                "RESCUE_ZONE_CONTROL_PATH",
                input_path,
            ):
                current = forward_camera_stream.read_rescue_zone_detection_input(
                    now=100.1
                )
                stale = forward_camera_stream.read_rescue_zone_detection_input(
                    now=100.4
                )
                with open(input_path, "w", encoding="utf-8") as input_file:
                    json.dump(
                        {
                            "enabled": True,
                            "ultrasonicFresh": True,
                            "ultrasonicValid": True,
                            "ultrasonicDistanceCm": 401.0,
                            "timestamp": 100.0,
                        },
                        input_file,
                    )
                outside_range = (
                    forward_camera_stream.read_rescue_zone_detection_input(
                        now=100.1
                    )
                )

        self.assertTrue(current["enabled"])
        self.assertTrue(current["ultrasonicFresh"])
        self.assertTrue(current["ultrasonicValid"])
        self.assertEqual(current["ultrasonicDistanceCm"], 43.7)
        self.assertFalse(stale["enabled"])
        self.assertFalse(stale["ultrasonicFresh"])
        self.assertFalse(outside_range["ultrasonicValid"])
        self.assertIsNone(outside_range["ultrasonicDistanceCm"])

    def test_rescue_zone_status_is_atomic_and_independent_by_color(self):
        frame = np.full((240, 320, 3), 255, dtype=np.uint8)
        cv2.rectangle(frame, (30, 70), (130, 220), (0, 200, 0), -1)
        cv2.rectangle(frame, (190, 70), (290, 220), (0, 0, 220), -1)
        candidates = forward_camera_stream.analyze_rescue_zones(
            frame,
            collect_color_diagnostics=True,
        )
        temporal_filter = forward_camera_stream.RescueZoneTemporalFilter()
        temporal_filter.update(candidates)
        temporal_filter.update(candidates)
        results = temporal_filter.update(candidates)

        with tempfile.TemporaryDirectory() as temporary_directory:
            status_path = os.path.join(temporary_directory, "zones.json")
            temporary_path = os.path.join(temporary_directory, "zones.tmp.json")
            with mock.patch.object(
                forward_camera_stream,
                "RESCUE_ZONE_STATUS_PATH",
                status_path,
            ), mock.patch.object(
                forward_camera_stream,
                "TEMP_RESCUE_ZONE_STATUS_PATH",
                temporary_path,
            ):
                forward_camera_stream.save_rescue_zone_status(
                    results,
                    123.5,
                    7,
                    {
                        "ultrasonicFresh": True,
                        "ultrasonicValid": True,
                        "ultrasonicDistanceCm": 43.7,
                    },
                )
                with open(status_path, "r", encoding="utf-8") as status_file:
                    status = json.load(status_file)
                forward_camera_stream.clear_rescue_zone_status()
                self.assertFalse(os.path.exists(status_path))

        self.assertTrue(status["active"])
        self.assertEqual(status["timestamp"], 123.5)
        self.assertEqual(status["sequence"], 7)
        self.assertTrue(status["green"]["detected"])
        self.assertTrue(status["red"]["detected"])
        self.assertTrue(status["green"]["candidateDetected"])
        self.assertTrue(status["red"]["candidateDetected"])
        self.assertEqual(status["green"]["confirmationFrames"], 3)
        self.assertEqual(status["red"]["confirmationFrames"], 3)
        self.assertEqual(status["green"]["confirmationRequiredFrames"], 3)
        self.assertEqual(status["red"]["confirmationRequiredFrames"], 3)
        self.assertEqual(status["green"]["strongGreenFraction"], 1.0)
        self.assertNotIn("_hull", status["green"])
        self.assertTrue(status["ultrasonic"]["fresh"])
        self.assertTrue(status["ultrasonic"]["valid"])
        self.assertEqual(status["ultrasonic"]["distanceCm"], 43.7)
        self.assertEqual(status["greenMedianRgb"], [0, 200, 0])
        self.assertEqual(status["greenMedianHsv"], [60, 255, 200])
        self.assertEqual(status["backgroundMedianRgb"], [255, 255, 255])
        self.assertEqual(status["backgroundMedianHsv"], [0, 0, 255])
        self.assertEqual(status["redMedianRgb"], [220, 0, 0])
        self.assertEqual(status["redMedianHsv"], [0, 255, 220])
        self.assertEqual(status["redBackgroundMedianRgb"], [255, 255, 255])
        self.assertEqual(status["redBackgroundMedianHsv"], [0, 0, 255])

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
                "obstacleBlackPixelCount",
                "obstacleBlackRatio",
                "obstacleBlackLargestComponent",
                "obstacleBlackSequence",
                "obstacleBlackVisible",
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

    def test_rescue_zone_profiler_separates_client_conditions_and_reports_p95(self):
        profiler = forward_camera_stream.RescueZoneCycleProfiler(
            enabled=True,
            window_frames=2,
        )
        first = {
            stage: 1.0
            for stage in forward_camera_stream.RescueZoneCycleProfiler.STAGES
        }
        second = dict(first)
        first["totalCycleMs"] = 10.0
        second["totalCycleMs"] = 30.0

        with mock.patch("builtins.print") as print_mock:
            profiler.record(False, first, {"ExposureTime": 8000})
            profiler.record(True, first, {"FrameDuration": 33333})
            self.assertEqual(print_mock.call_count, 0)
            profiler.record(False, second, {"ExposureTime": 12000})

        self.assertEqual(print_mock.call_count, 1)
        report_text = print_mock.call_args.args[0]
        self.assertTrue(report_text.startswith("CAM1_PROFILE "))
        report = json.loads(report_text.split(" ", 1)[1])
        self.assertEqual(report["condition"], "withoutStreamClient")
        self.assertEqual(report["frames"], 2)
        self.assertEqual(report["effectiveFps"], 50.0)
        self.assertEqual(report["stagesMs"]["totalCycleMs"]["average"], 20.0)
        self.assertEqual(report["stagesMs"]["totalCycleMs"]["p95"], 30.0)
        self.assertEqual(len(profiler.samples["withStreamClient"]), 1)


if __name__ == "__main__":
    unittest.main()
