import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import cv2
import numpy as np

SCRIPTS_DIRECTORY = Path(__file__).resolve().parents[2] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIRECTORY))

from ball_vision import BallVisionPipeline
from ball_vision.ball_detector import (
    BallCandidate,
    BallDetector,
    BallDetectorConfig,
    BlackBallDetector,
    SilverBallDetector,
)
from ball_vision.ball_tracker import BallTracker, BallTrackerConfig, TargetState
from ball_vision.camera import CameraConfig, FrontCamera
import ball_vision.camera as camera_module
from ball_vision.distance_calibration import DistanceCalibration
from ball_vision.main import (
    analyze_frame,
    build_esp32_payload,
    calculate_horizontal_angle,
    classify_position,
)
from ball_vision.yolo_detector import (
    YoloBallDetector,
    YoloBallDetectorConfig,
)


class DistanceCalibrationTest(unittest.TestCase):
    def setUp(self):
        self.calibration = DistanceCalibration()

    def test_calibration_points_are_reproduced_exactly(self):
        expected_points = (
            (45.5, 60.0),
            (71.7, 40.0),
            (149.0, 20.0),
            (207.6, 15.0),
            (268.8, 5.0),
        )
        for radius, expected_distance in expected_points:
            estimate = self.calibration.estimate(radius)
            self.assertAlmostEqual(estimate.distance_cm, expected_distance)
            self.assertFalse(estimate.extrapolated)

    def test_interpolation_stays_between_neighboring_points(self):
        estimate = self.calibration.estimate((71.7 + 149.0) * 0.5)
        self.assertAlmostEqual(estimate.distance_cm, 30.0)
        self.assertFalse(estimate.extrapolated)

    def test_extrapolation_is_continuous_and_positive(self):
        far_edge = self.calibration.estimate(45.5)
        just_farther = self.calibration.estimate(45.5 - 1e-6)
        near_edge = self.calibration.estimate(268.8)
        just_nearer = self.calibration.estimate(268.8 + 1e-6)

        self.assertAlmostEqual(
            far_edge.distance_cm,
            just_farther.distance_cm,
            places=4,
        )
        self.assertAlmostEqual(
            near_edge.distance_cm,
            just_nearer.distance_cm,
            places=4,
        )
        self.assertGreater(self.calibration.estimate(20.0).distance_cm, 0.0)
        self.assertGreater(self.calibration.estimate(400.0).distance_cm, 0.0)
        self.assertTrue(just_farther.extrapolated)
        self.assertTrue(just_nearer.extrapolated)

    def test_invalid_radius_is_rejected(self):
        for radius in (0.0, -1.0, math.inf, math.nan):
            with self.assertRaises(ValueError):
                self.calibration.estimate(radius)


class FrontCameraTest(unittest.TestCase):
    def test_rejects_downward_camera_index(self):
        with self.assertRaises(ValueError):
            FrontCamera(CameraConfig(camera_index=0, downward_camera_index=0))

    def test_uses_forward_camera_and_closes_simulated_picamera(self):
        class FakePicamera2:
            latest = None

            def __init__(self, camera_index):
                self.camera_index = camera_index
                self.started = False
                self.stopped = False
                self.closed = False
                self.configuration = None
                FakePicamera2.latest = self

            def create_video_configuration(self, **configuration):
                self.configuration = configuration
                return configuration

            def configure(self, configuration):
                self.configuration = configuration

            def start(self):
                self.started = True

            def capture_array(self, stream_name):
                self.stream_name = stream_name
                return np.zeros((540, 960, 3), dtype=np.uint8)

            def stop(self):
                self.stopped = True

            def close(self):
                self.closed = True

        with mock.patch.object(camera_module, "Picamera2", FakePicamera2), mock.patch.object(
            camera_module,
            "Transform",
            lambda **values: values,
        ):
            camera = FrontCamera(CameraConfig())
            camera.open()
            frame = camera.capture_frame()
            camera.close()

        fake = FakePicamera2.latest
        self.assertEqual(fake.camera_index, 1)
        self.assertTrue(fake.started)
        self.assertEqual(fake.stream_name, "main")
        self.assertEqual(frame.shape, (540, 960, 3))
        self.assertTrue(fake.stopped)
        self.assertTrue(fake.closed)


class BlackBallDetectorTest(unittest.TestCase):
    def setUp(self):
        self.detector = BlackBallDetector()

    @staticmethod
    def white_frame():
        return np.full((540, 960, 3), 255, dtype=np.uint8)

    def test_detects_center_and_radius_of_black_circle(self):
        frame = self.white_frame()
        cv2.circle(frame, (600, 260), 50, (5, 5, 5), -1)

        candidates = self.detector.detect(frame)

        self.assertEqual(len(candidates), 1)
        candidate = candidates[0]
        self.assertEqual(candidate.ball_type, "black_ball")
        self.assertAlmostEqual(candidate.center_x, 600.0, delta=1.0)
        self.assertAlmostEqual(candidate.center_y, 260.0, delta=1.0)
        self.assertAlmostEqual(candidate.radius_pixels, 50.0, delta=2.0)
        self.assertAlmostEqual(candidate.diameter_pixels, 100.0, delta=4.0)

    def test_rejects_small_noise_and_non_circular_rectangle(self):
        frame = self.white_frame()
        cv2.circle(frame, (100, 100), 3, (0, 0, 0), -1)
        cv2.rectangle(frame, (250, 200), (500, 240), (0, 0, 0), -1)

        self.assertEqual(self.detector.detect(frame), [])

    def test_rejects_square_using_circle_fill_ratio(self):
        frame = self.white_frame()
        cv2.rectangle(frame, (300, 160), (400, 260), (0, 0, 0), -1)

        self.assertEqual(self.detector.detect(frame), [])

    def test_largest_radius_is_first(self):
        frame = self.white_frame()
        cv2.circle(frame, (200, 250), 35, (0, 0, 0), -1)
        cv2.circle(frame, (700, 250), 70, (0, 0, 0), -1)

        candidates = self.detector.detect(frame)

        self.assertEqual(len(candidates), 2)
        self.assertGreater(candidates[0].radius_pixels, candidates[1].radius_pixels)
        self.assertAlmostEqual(candidates[0].center_x, 700.0, delta=1.0)

    def test_detects_circle_clipped_by_top_border(self):
        frame = self.white_frame()
        cv2.circle(frame, (480, 18), 90, (20, 20, 20), -1)

        candidates = self.detector.detect(frame)

        self.assertEqual(len(candidates), 1)
        self.assertTrue(candidates[0].top_clipped)
        self.assertAlmostEqual(candidates[0].center_x, 480.0, delta=2.0)
        self.assertAlmostEqual(candidates[0].radius_pixels, 90.0, delta=3.0)

    def test_rejects_dark_bar_clipped_by_top_border(self):
        frame = self.white_frame()
        cv2.rectangle(frame, (250, 0), (710, 45), (20, 20, 20), -1)

        self.assertEqual(self.detector.detect(frame), [])

    def test_brightness_threshold_is_configurable(self):
        frame = self.white_frame()
        cv2.circle(frame, (480, 270), 50, (90, 90, 90), -1)
        strict_detector = BlackBallDetector(BallDetectorConfig(maximum_value=70))
        permissive_detector = BlackBallDetector(
            BallDetectorConfig(maximum_value=100)
        )

        self.assertEqual(strict_detector.detect(frame), [])
        self.assertEqual(len(permissive_detector.detect(frame)), 1)


class SilverBallDetectorTest(unittest.TestCase):
    def setUp(self):
        self.detector = SilverBallDetector()

    @staticmethod
    def frame_with_silver_ball(center=(600, 180), radius=75):
        """Cria um círculo claro com borda e dobras semelhantes ao alumínio."""

        frame = np.full((540, 960, 3), (205, 220, 205), dtype=np.uint8)
        cv2.circle(frame, center, radius, (145, 155, 150), -1, cv2.LINE_AA)
        cv2.circle(frame, center, radius, (75, 90, 80), 4, cv2.LINE_AA)
        random = np.random.default_rng(123)
        for _ in range(75):
            angle = float(random.uniform(0.0, 2.0 * math.pi))
            start_radius = float(random.uniform(0.0, radius * 0.65))
            line_length = float(random.uniform(radius * 0.25, radius * 0.90))
            start_x = int(center[0] + math.cos(angle) * start_radius)
            start_y = int(center[1] + math.sin(angle) * start_radius)
            end_angle = angle + float(random.uniform(-1.0, 1.0))
            end_x = int(start_x + math.cos(end_angle) * line_length)
            end_y = int(start_y + math.sin(end_angle) * line_length)
            brightness = int(random.integers(70, 230))
            cv2.line(
                frame,
                (start_x, start_y),
                (end_x, end_y),
                (brightness, brightness, brightness),
                2,
                cv2.LINE_AA,
            )
        return frame

    def test_detects_center_radius_and_type_of_silver_ball(self):
        candidates = self.detector.detect(self.frame_with_silver_ball())

        self.assertGreaterEqual(len(candidates), 1)
        candidate = candidates[0]
        self.assertEqual(candidate.ball_type, "silver_ball")
        self.assertEqual(candidate.detection_method, "hough")
        self.assertAlmostEqual(candidate.center_x, 600.0, delta=5.0)
        self.assertAlmostEqual(candidate.center_y, 180.0, delta=5.0)
        self.assertAlmostEqual(candidate.radius_pixels, 75.0, delta=6.0)

    def test_rejects_smooth_bright_floor(self):
        frame = np.full((540, 960, 3), (205, 220, 205), dtype=np.uint8)

        self.assertEqual(self.detector.detect(frame), [])

    def test_detects_silver_ball_clipped_by_top_border(self):
        candidates = self.detector.detect(
            self.frame_with_silver_ball(center=(480, 20), radius=75)
        )

        self.assertGreaterEqual(len(candidates), 1)
        self.assertEqual(candidates[0].ball_type, "silver_ball")
        self.assertTrue(candidates[0].top_clipped)
        self.assertAlmostEqual(candidates[0].center_x, 480.0, delta=5.0)
        self.assertAlmostEqual(candidates[0].center_y, 20.0, delta=5.0)
        self.assertLess(
            candidates[0].visible_area_pixels,
            math.pi * candidates[0].radius_pixels ** 2,
        )

    def test_combined_detector_does_not_duplicate_black_ball_as_silver(self):
        frame = np.full((540, 960, 3), 255, dtype=np.uint8)
        cv2.circle(frame, (480, 270), 70, (10, 10, 10), -1, cv2.LINE_AA)

        candidates = BallDetector().detect(frame)

        self.assertEqual(len(candidates), 1)
        self.assertEqual(candidates[0].ball_type, "black_ball")


class BallTrackerTest(unittest.TestCase):
    @staticmethod
    def candidate(
        center_x,
        center_y,
        radius,
        ball_type="silver_ball",
        visible_area_pixels=None,
    ):
        if visible_area_pixels is None:
            visible_area_pixels = math.pi * radius * radius
        return BallCandidate(
            ball_type=ball_type,
            center_x=float(center_x),
            center_y=float(center_y),
            radius_pixels=float(radius),
            diameter_pixels=float(radius * 2.0),
            contour_area_pixels=math.pi * radius * radius,
            circularity=0.9,
            circle_fill_ratio=0.08,
            top_clipped=True,
            detection_method="hough",
            visible_area_pixels=float(visible_area_pixels),
        )

    @staticmethod
    def yolo_candidate(
        center_x,
        radius,
        ball_type="black_ball",
        confidence=0.70,
    ):
        return BallCandidate(
            ball_type=ball_type,
            center_x=float(center_x),
            center_y=250.0,
            radius_pixels=float(radius),
            diameter_pixels=float(radius * 2.0),
            contour_area_pixels=float(radius * radius * 4.0),
            circularity=float(confidence),
            circle_fill_ratio=float(confidence),
            top_clipped=False,
            detection_method="yolo",
            visible_area_pixels=float(radius * radius * 4.0),
            bounding_box=(
                float(center_x - radius),
                float(250.0 - radius),
                float(radius * 2.0),
                float(radius * 2.0),
            ),
        )

    def test_yolo_acquisition_prioritizes_silver_over_black(self):
        tracker = BallTracker(BallTrackerConfig(acquisition_frames=1))
        black = self.yolo_candidate(300, 70, "black_ball", 0.95)
        silver = self.yolo_candidate(650, 50, "silver_ball", 0.60)

        selected = tracker.update([black, silver], frame_width=960)

        self.assertEqual(selected[0].ball_type, "silver_ball")
        self.assertTrue(tracker.locked)

    def test_target_switch_requires_all_confirmation_frames(self):
        tracker = BallTracker(BallTrackerConfig(acquisition_frames=1))
        locked = self.yolo_candidate(250, 40, "black_ball", 0.70)
        tracker.update([locked], frame_width=960)

        for frame_index in range(
            tracker.config.switch_confirmation_frames
        ):
            current = self.yolo_candidate(
                250 + frame_index,
                40,
                "black_ball",
                0.70,
            )
            replacement = self.yolo_candidate(
                320,
                55,
                "silver_ball",
                0.80,
            )
            selected = tracker.update(
                [current, replacement],
                frame_width=960,
            )
            if frame_index + 1 < tracker.config.switch_confirmation_frames:
                self.assertEqual(selected[0].ball_type, "black_ball")

        self.assertEqual(selected[0].ball_type, "silver_ball")

    def test_reacquisition_requires_consecutive_compatible_frames(self):
        tracker = BallTracker(BallTrackerConfig(acquisition_frames=1))
        tracker.update(
            [self.yolo_candidate(180, 45, "silver_ball")],
            frame_width=960,
        )
        self.assertEqual(tracker.update([], frame_width=960), [])

        reacquired = self.yolo_candidate(600, 45, "silver_ball")
        self.assertEqual(
            tracker.update([reacquired], frame_width=960),
            [],
        )
        selected = tracker.update([reacquired], frame_width=960)

        self.assertEqual(selected[0].ball_type, "silver_ball")
        self.assertAlmostEqual(selected[0].center_x, 600.0)

    def test_single_high_confidence_candidate_reacquires_locked_target(self):
        tracker = BallTracker(BallTrackerConfig(acquisition_frames=1))
        tracker.update(
            [self.yolo_candidate(180, 45, "silver_ball", 0.91)],
            frame_width=960,
        )
        self.assertEqual(tracker.update([], frame_width=960), [])

        reacquired = tracker.update(
            [self.yolo_candidate(600, 45, "silver_ball", 0.91)],
            frame_width=960,
        )

        self.assertEqual(reacquired[0].ball_type, "silver_ball")
        self.assertAlmostEqual(reacquired[0].center_x, 600.0)

    def test_visible_jump_enters_reacquisition_without_empty_frame(self):
        tracker = BallTracker(BallTrackerConfig(acquisition_frames=1))
        tracker.update(
            [self.yolo_candidate(180, 45, "silver_ball", 0.91)],
            frame_width=960,
        )

        reacquired = tracker.update(
            [self.yolo_candidate(600, 45, "silver_ball", 0.91)],
            frame_width=960,
        )

        self.assertEqual(tracker.state, TargetState.TRACKING)
        self.assertTrue(tracker.locked)
        self.assertEqual(reacquired[0].ball_type, "silver_ball")
        self.assertAlmostEqual(reacquired[0].center_x, 600.0)

    def test_high_confidence_does_not_skip_confirmation_with_two_candidates(self):
        tracker = BallTracker(BallTrackerConfig(acquisition_frames=1))
        tracker.update(
            [self.yolo_candidate(180, 45, "silver_ball", 0.91)],
            frame_width=960,
        )
        self.assertEqual(tracker.update([], frame_width=960), [])

        candidates = [
            self.yolo_candidate(600, 45, "silver_ball", 0.91),
            self.yolo_candidate(760, 40, "silver_ball", 0.89),
        ]

        self.assertEqual(tracker.update(candidates, frame_width=960), [])

    def test_reacquisition_tolerates_one_empty_inference(self):
        tracker = BallTracker(BallTrackerConfig(acquisition_frames=1))
        tracker.update(
            [self.yolo_candidate(180, 45, "silver_ball")],
            frame_width=960,
        )
        self.assertEqual(tracker.update([], frame_width=960), [])

        reacquired = self.yolo_candidate(600, 45, "silver_ball")
        self.assertEqual(
            tracker.update([reacquired], frame_width=960),
            [],
        )
        self.assertEqual(tracker.update([], frame_width=960), [])
        selected = tracker.update([reacquired], frame_width=960)

        self.assertEqual(selected[0].ball_type, "silver_ball")
        self.assertAlmostEqual(selected[0].center_x, 600.0)

    def test_two_empty_inferences_clear_partial_reacquisition(self):
        tracker = BallTracker(BallTrackerConfig(acquisition_frames=1))
        tracker.update(
            [self.yolo_candidate(180, 45, "silver_ball")],
            frame_width=960,
        )
        tracker.update([], frame_width=960)

        reacquired = self.yolo_candidate(600, 45, "silver_ball")
        self.assertEqual(
            tracker.update([reacquired], frame_width=960),
            [],
        )
        self.assertEqual(tracker.update([], frame_width=960), [])
        self.assertEqual(tracker.update([], frame_width=960), [])
        self.assertEqual(
            tracker.update([reacquired], frame_width=960),
            [],
        )
        selected = tracker.update([reacquired], frame_width=960)

        self.assertEqual(selected[0].ball_type, "silver_ball")

    def test_reduces_radius_and_center_variation_for_stationary_ball(self):
        tracker = BallTracker(BallTrackerConfig(acquisition_frames=1))
        measurements = (
            (466, 9, 39),
            (501, 4, 52),
            (470, 8, 40),
            (498, 5, 51),
            (472, 7, 41),
        )

        filtered = []
        for measurement in measurements * 3:
            filtered.append(tracker.update([self.candidate(*measurement)])[0])

        raw_radius_range = max(value[2] for value in measurements) - min(
            value[2] for value in measurements
        )
        stable_filtered = filtered[-5:]
        filtered_radius_range = max(
            candidate.radius_pixels for candidate in stable_filtered
        ) - min(candidate.radius_pixels for candidate in stable_filtered)
        raw_center_range = max(value[0] for value in measurements) - min(
            value[0] for value in measurements
        )
        filtered_center_range = max(
            candidate.center_x for candidate in stable_filtered
        ) - min(
            candidate.center_x for candidate in stable_filtered
        )

        self.assertLess(filtered_radius_range, raw_radius_range * 0.35)
        self.assertLess(filtered_center_range, raw_center_range * 0.35)
        self.assertGreater(stable_filtered[-1].radius_pixels, 48.0)

    def test_does_not_publish_previous_ball_when_current_frame_is_empty(self):
        tracker = BallTracker(BallTrackerConfig(acquisition_frames=1))
        tracker.update([self.candidate(480, 20, 50)])

        self.assertEqual(tracker.update([]), [])

    def test_keeps_target_when_radius_ranking_changes_during_turn(self):
        tracker = BallTracker(BallTrackerConfig(acquisition_frames=1))
        current_target = self.candidate(220, 240, 80)
        other_ball = self.candidate(700, 240, 50)
        tracker.update([current_target, other_ball])

        enlarged_other_ball = self.candidate(300, 240, 80)
        changed_current_target = self.candidate(235, 240, 30)
        tracked = tracker.update([
            enlarged_other_ball,
            changed_current_target,
        ])

        self.assertLess(tracked[0].center_x, 300.0)
        self.assertEqual(tracked[0].ball_type, changed_current_target.ball_type)
        self.assertIs(tracked[1], enlarged_other_ball)

    def test_never_accepts_an_incompatible_target_until_reset(self):
        tracker = BallTracker(BallTrackerConfig(acquisition_frames=1))
        tracker.update([self.candidate(200, 240, 50)])
        new_target = self.candidate(700, 240, 90)

        for _ in range(10):
            self.assertEqual(tracker.update([new_target]), [])

        tracker.reset()
        selected = tracker.update([new_target])

        self.assertEqual(len(selected), 1)
        self.assertEqual(selected[0].center_x, new_target.center_x)

    def test_waits_for_three_stable_frames_of_largest_visible_area(self):
        tracker = BallTracker(BallTrackerConfig(acquisition_frames=3))
        larger_radius = self.candidate(
            650, 240, 85, visible_area_pixels=4000
        )
        larger_visible_area = self.candidate(
            280, 200, 60, visible_area_pixels=7000
        )

        self.assertEqual(
            tracker.update([larger_radius, larger_visible_area]), []
        )
        self.assertEqual(
            tracker.update([larger_radius, larger_visible_area]), []
        )
        selected = tracker.update([larger_radius, larger_visible_area])

        self.assertEqual(len(selected), 2)
        self.assertLess(selected[0].center_x, 400.0)
        self.assertEqual(selected[0].visible_area_pixels, 7000)
        self.assertIs(selected[1], larger_radius)
        self.assertTrue(tracker.locked)

    def test_largest_visible_area_wins_even_at_frame_edge(self):
        tracker = BallTracker(BallTrackerConfig(acquisition_frames=1))
        edge_ball = self.candidate(
            -8, 180, 90, visible_area_pixels=9000
        )
        central_ball = self.candidate(
            480, 180, 100, visible_area_pixels=8000
        )

        selected = tracker.update([central_ball, edge_ball])

        self.assertEqual(selected[0].center_x, edge_ball.center_x)
        self.assertEqual(selected[0].visible_area_pixels, 9000)

    def test_larger_ball_does_not_replace_locked_target(self):
        tracker = BallTracker(BallTrackerConfig(acquisition_frames=1))
        locked = self.candidate(200, 240, 50, visible_area_pixels=6000)
        tracker.update([locked])

        same_target = self.candidate(215, 240, 30, visible_area_pixels=2500)
        new_larger_ball = self.candidate(
            700, 240, 100, visible_area_pixels=25000
        )
        selected = tracker.update([new_larger_ball, same_target])

        self.assertLess(selected[0].center_x, 300.0)
        self.assertIs(selected[1], new_larger_ball)

    def test_combined_detector_selects_larger_distinct_ball_as_closest(self):
        class FixedDetector:
            def __init__(self, candidates):
                self.candidates = candidates

            def detect(self, frame):
                del frame
                return self.candidates

        black = self.candidate(220, 250, 40, "black_ball")
        silver = self.candidate(700, 230, 75)
        detector = BallDetector(
            black_detector=FixedDetector([black]),
            silver_detector=FixedDetector([silver]),
        )

        candidates = detector.detect(np.zeros((540, 960, 3), dtype=np.uint8))

        self.assertEqual(candidates[0].ball_type, "silver_ball")
        self.assertEqual(candidates[0].radius_pixels, 75.0)


class BallObservationTest(unittest.TestCase):
    def setUp(self):
        self.detector = BlackBallDetector()
        self.calibration = DistanceCalibration()

    @staticmethod
    def frame_with_ball(center_x=480, radius=45):
        frame = np.full((540, 960, 3), 255, dtype=np.uint8)
        cv2.circle(frame, (center_x, 270), radius, (0, 0, 0), -1)
        return frame

    def test_angle_and_position_use_positive_sign_on_right(self):
        self.assertAlmostEqual(calculate_horizontal_angle(480, 960), 0.0)
        self.assertAlmostEqual(calculate_horizontal_angle(960, 960), 31.0)
        self.assertAlmostEqual(calculate_horizontal_angle(0, 960), -31.0)
        self.assertEqual(classify_position(-6.0), "esquerda")
        self.assertEqual(classify_position(5.0), "centro")
        self.assertEqual(classify_position(6.0), "direita")

    def test_frame_without_ball_does_not_reuse_previous_result(self):
        observation, _ = analyze_frame(
            self.frame_with_ball(),
            self.detector,
            self.calibration,
        )
        self.assertIsNotNone(observation)

        blank = np.full((540, 960, 3), 255, dtype=np.uint8)
        observation, candidates = analyze_frame(
            blank,
            self.detector,
            self.calibration,
        )
        self.assertIsNone(observation)
        self.assertEqual(candidates, [])

    def test_payload_contains_only_simple_esp32_fields(self):
        payload = build_esp32_payload("black_ball", 42.4, 14.6)
        self.assertEqual(payload, {
            "type": "black_ball",
            "distance_cm": 42,
            "angle": 15,
        })


class PublicBallVisionPipelineTest(unittest.TestCase):
    def test_default_pipeline_uses_yolo_without_legacy_fallback(self):
        pipeline = BallVisionPipeline()

        self.assertIsInstance(pipeline.detector, YoloBallDetector)

    def test_missing_yolo_model_fails_instead_of_using_legacy_detector(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            missing_model = Path(temporary_directory) / "missing.onnx"
            with self.assertRaises(FileNotFoundError):
                YoloBallDetector(
                    YoloBallDetectorConfig(model_path=missing_model)
                )

    def test_public_pipeline_can_be_imported_from_package(self):
        pipeline = BallVisionPipeline(
            tracker=BallTracker(BallTrackerConfig(acquisition_frames=1))
        )
        frame = np.full((540, 960, 3), 255, dtype=np.uint8)
        cv2.circle(frame, (480, 270), 70, (0, 0, 0), -1)

        result = pipeline.analyze(frame)

        self.assertIsNotNone(result.observation)
        self.assertAlmostEqual(result.observation.angle_degrees, 0.0, delta=1.0)
        self.assertGreater(result.observation.candidate.radius_pixels, 65.0)

    def test_target_type_filters_before_temporal_lock(self):
        class FixedDetector:
            processing_scale = 1.0

            def detect(self, frame):
                del frame
                return [
                    BallCandidate(
                        "black_ball", 480.0, 270.0, 80.0, 160.0,
                        10000.0, 0.9, 0.9, False, "yolo", 10000.0,
                    ),
                    BallCandidate(
                        "silver_ball", 600.0, 270.0, 60.0, 120.0,
                        8000.0, 0.9, 0.9, False, "yolo", 8000.0,
                    ),
                ]

        pipeline = BallVisionPipeline(
            detector=FixedDetector(),
            tracker=BallTracker(BallTrackerConfig(acquisition_frames=1)),
        )
        pipeline.set_target_type("silver_ball")
        result = pipeline.analyze(np.zeros((540, 960, 3), dtype=np.uint8))

        self.assertIsNotNone(result.observation)
        self.assertEqual(result.observation.candidate.ball_type, "silver_ball")
        self.assertEqual(
            {candidate.ball_type for candidate in result.candidates},
            {"silver_ball"},
        )


if __name__ == "__main__":
    unittest.main()
