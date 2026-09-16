"""Testes determinísticos do tracker virtual sem acessar câmera ou motores."""

import importlib.util
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np


SCRIPTS_DIRECTORY = Path(__file__).resolve().parents[2] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIRECTORY))
MODULE_PATH = SCRIPTS_DIRECTORY / "camera_line_frame.py"
MODULE_SPEC = importlib.util.spec_from_file_location(
    "camera_line_frame_virtual_turn_test",
    MODULE_PATH,
)
camera_line_frame = importlib.util.module_from_spec(MODULE_SPEC)
MODULE_SPEC.loader.exec_module(camera_line_frame)


def sensor_values(
    steering_error=0.0,
    medium_position=0.0,
    far_band_position=0.0,
    near_fine_position=None,
):
    """Cria uma leitura completa sem depender da câmera."""

    return {
        "farLeft": 0.10,
        "farCenter": 0.40,
        "farRight": 0.10,
        "controlFarLeft": 0.10,
        "controlFarCenter": 0.40,
        "controlFarRight": 0.10,
        "farPosition": 0.0,
        "rawFarPosition": 0.0,
        "farTrusted": True,
        "farBandLeft": 0.10,
        "farBandCenter": 0.10,
        "farBandRight": 0.10,
        "controlFarBandLeft": 0.10,
        "controlFarBandCenter": 0.10,
        "controlFarBandRight": 0.10,
        "farBandPosition": far_band_position,
        "rawFarBandPosition": far_band_position,
        "mediumLeft": 0.10,
        "mediumCenter": 0.20,
        "mediumRight": 0.10,
        "controlMediumLeft": 0.10,
        "controlMediumCenter": 0.20,
        "controlMediumRight": 0.10,
        "mediumPosition": medium_position,
        "rawMediumPosition": medium_position,
        "mediumTrusted": True,
        "nearCenter": 0.0 if steering_error is None else 0.20,
        "nearFinePosition": near_fine_position,
        "headingAngle": 0.0,
        "steeringError": steering_error,
    }


def calculate_command(
    sensors,
    tracker=None,
    pivot_state_tracker=None,
    medium_spin_tracker=None,
    green_direction="NENHUMA",
    gap_active=False,
    line_search_tracker=None,
    blind_search_requested=False,
    sensor_recovery_requested=False,
    mask=None,
    fusion_style_line=None,
    blind_search_preferred_direction=None,
    gap_fusion_reacquire_active=False,
    green_active_frames=0,
):
    """Executa somente o controle virtual com leituras determinísticas."""

    if mask is None:
        mask = np.zeros((100, 100), dtype=np.uint8)
    with patch.object(
        camera_line_frame,
        "read_virtual_line_sensors",
        return_value=sensors,
    ):
        return camera_line_frame.calculate_line_follower_command(
            mask,
            {},
            direcao_verde_ativa=green_direction,
            gap_forward_active=gap_active,
            virtual_turn_tracker=tracker,
            pivot_state_tracker=pivot_state_tracker,
            medium_spin_tracker=medium_spin_tracker,
            line_search_tracker=line_search_tracker,
            blind_search_requested=blind_search_requested,
            sensor_recovery_requested=sensor_recovery_requested,
            fusion_style_line=fusion_style_line,
            blind_search_preferred_direction=(
                blind_search_preferred_direction
            ),
            gap_fusion_reacquire_active=gap_fusion_reacquire_active,
            green_active_frames=green_active_frames,
        )


def expected_normal_motor_powers(steering_error, strong_enabled=False):
    """Replica o mapper normal com STRONG autorizado somente pelo MEDIUM."""

    if steering_error is None:
        return 0.0, 0.0

    if not strong_enabled:
        steering_error = max(
            -camera_line_frame.NORMAL_FULL_STEERING_ERROR,
            min(
                camera_line_frame.NORMAL_FULL_STEERING_ERROR,
                steering_error,
            ),
        )

    steering_magnitude = abs(steering_error)
    if (
        strong_enabled
        and steering_magnitude
        >= camera_line_frame.NORMAL_FULL_STEERING_ERROR
    ):
        transition_progress = (
            steering_magnitude
            - camera_line_frame.NORMAL_FULL_STEERING_ERROR
        ) / (
            camera_line_frame.PIVOT_ENTER_THRESHOLD
            - camera_line_frame.NORMAL_FULL_STEERING_ERROR
        )
        transition_progress = max(0.0, min(1.0, transition_progress))
        transition_progress *= transition_progress
        outer_power = 0.82 + transition_progress * (0.85 - 0.82)
        inner_power = 0.66 - transition_progress * (0.66 - 0.61)
    else:
        steering_strength = (
            steering_magnitude
            / camera_line_frame.NORMAL_FULL_STEERING_ERROR
        )
        outer_power = 0.75 + steering_strength * (0.82 - 0.75)
        inner_power = 0.75 - steering_strength * (0.75 - 0.66)
    if steering_error > 0.0:
        return outer_power, inner_power
    return inner_power, outer_power


class CameraStreamRegressionTests(unittest.TestCase):
    def setUp(self):
        with camera_line_frame.frame_condition:
            self.original_client_count = (
                camera_line_frame.active_stream_clients
            )
            camera_line_frame.active_stream_clients = 0

    def tearDown(self):
        with camera_line_frame.frame_condition:
            camera_line_frame.active_stream_clients = (
                self.original_client_count
            )

    def test_stream_frame_is_due_only_with_connected_client(self):
        self.assertFalse(camera_line_frame.stream_frame_is_due(10.0, 0.0))

        camera_line_frame.register_stream_client()
        self.assertTrue(camera_line_frame.stream_frame_is_due(10.0, 0.0))

        camera_line_frame.unregister_stream_client()
        self.assertFalse(camera_line_frame.stream_frame_is_due(10.0, 0.0))

    def test_stream_waits_for_fps_interval_with_connected_client(self):
        camera_line_frame.register_stream_client()
        interval = 1.0 / camera_line_frame.MJPEG_STREAM_FPS

        self.assertFalse(camera_line_frame.stream_frame_is_due(
            interval * 0.99,
            0.0,
        ))
        self.assertTrue(camera_line_frame.stream_frame_is_due(
            interval,
            0.0,
        ))


class NormalTrajectoryExtractionTests(unittest.TestCase):
    def test_continuity_keeps_right_path_despite_left_distractor(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        right_path = np.asarray(
            [(240, 359), (260, 290), (300, 220), (350, 150), (390, 47)],
            dtype=np.int32,
        )
        camera_line_frame.cv2.polylines(
            mask,
            [right_path],
            False,
            255,
            22,
        )
        # O ramo isolado à esquerda ocupa FAR/MEDIUM, mas não continua o NEAR.
        camera_line_frame.cv2.line(mask, (80, 280), (80, 47), 255, 40)

        virtual_sensors = camera_line_frame.read_virtual_line_sensors(mask)
        trajectory = camera_line_frame.extract_normal_line_trajectory(mask)
        points = trajectory["points"]

        self.assertLess(virtual_sensors["rawFarPosition"], 0.0)
        self.assertLess(virtual_sensors["rawMediumPosition"], 0.0)
        self.assertTrue(trajectory["valid"])
        self.assertGreater(trajectory["ambiguousScanlineCount"], 0)
        self.assertGreater(points[-1]["x"], points[0]["x"] + 100.0)
        self.assertTrue(all(point["x"] > 200.0 for point in points))
        self.assertTrue(all(
            scanline["selectedSegment"] is not None
            and scanline["acceptedPoint"] is not None
            for scanline in trajectory["scanlines"]
        ))
        self.assertTrue(any(
            scanline["rejectedSegments"]
            for scanline in trajectory["scanlines"]
        ))
        self.assertTrue(all(
            first["y"] > second["y"]
            for first, second in zip(points, points[1:])
        ))

    def test_path_without_near_anchor_is_not_claimed_as_normal(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(mask, (330, 260), (390, 47), 255, 22)

        trajectory = camera_line_frame.extract_normal_line_trajectory(mask)

        self.assertFalse(trajectory["valid"])
        self.assertEqual(trajectory["pointCount"], 0)
        self.assertGreater(trajectory["evaluatedScanlineCount"], 0)
        self.assertTrue(all(
            scanline["status"] == "noPoint"
            for scanline in trajectory["scanlines"]
        ))

    def test_scanline_rejects_thin_segment_and_keeps_tape_width(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        envelope = camera_line_frame.resolve_normal_trajectory_envelope(
            mask.shape
        )
        y = 180
        mask[y - 1:y + 2, 100:102] = 255
        mask[y - 1:y + 2, 300:324] = 255

        segments, rejected_segments = (
            camera_line_frame.find_normal_trajectory_segments(
                mask,
                envelope,
                y,
            )
        )

        self.assertEqual(len(rejected_segments), 1)
        self.assertEqual(rejected_segments[0]["rejectionReason"], "width")
        self.assertEqual(len(segments), 1)
        self.assertAlmostEqual(segments[0]["centerX"], 311.5)

    def test_overlay_draws_extracted_polyline_without_changing_result(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(mask, (240, 359), (330, 47), 255, 22)
        trajectory = camera_line_frame.extract_normal_line_trajectory(mask)
        original_trajectory = {
            key: value[:] if isinstance(value, list) else value
            for key, value in trajectory.items()
        }
        frame = np.zeros((360, 480, 3), dtype=np.uint8)

        with patch.object(
            camera_line_frame.cv2,
            "putText",
            wraps=camera_line_frame.cv2.putText,
        ) as put_text:
            camera_line_frame.draw_normal_trajectory_overlay(frame, trajectory)

        self.assertGreater(np.count_nonzero(frame), 0)
        self.assertEqual(trajectory, original_trajectory)
        metric_texts = {call.args[1] for call in put_text.call_args_list}
        self.assertIn(
            f"trajectoryPoints {trajectory['pointCount']}",
            metric_texts,
        )
        self.assertIn(
            f"coverage {trajectory['verticalCoverage']:.3f}",
            metric_texts,
        )
        self.assertIn(
            f"ambiguities {trajectory['ambiguousScanlineCount']}",
            metric_texts,
        )
        self.assertIn(
            f"trajectoryMs {trajectory['processingMs']:.2f}",
            metric_texts,
        )

    def test_overlay_marks_tolerated_scanline_without_point(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(mask, (240, 359), (300, 47), 255, 22)
        envelope = camera_line_frame.resolve_normal_trajectory_envelope(
            mask.shape
        )
        scanline_ys = camera_line_frame.normal_trajectory_scanline_ys(
            envelope,
            mask.shape[0],
        )
        missing_y = scanline_ys[12]
        mask[missing_y - 2:missing_y + 3, :] = 0

        trajectory = camera_line_frame.extract_normal_line_trajectory(mask)
        missing_scanline = next(
            scanline
            for scanline in trajectory["scanlines"]
            if scanline["y"] == missing_y
        )
        frame = np.zeros((360, 480, 3), dtype=np.uint8)
        camera_line_frame.draw_normal_trajectory_overlay(frame, trajectory)

        self.assertTrue(trajectory["valid"])
        self.assertEqual(missing_scanline["status"], "noPoint")
        self.assertIsNone(missing_scanline["acceptedPoint"])
        marker_x = missing_scanline["x0"] + 7
        red_region = frame[
            missing_y - 5:missing_y + 6,
            marker_x - 5:marker_x + 6,
            2,
        ]
        self.assertGreater(int(red_region.max()), 200)


class FusionStyleLineExtractionTests(unittest.TestCase):
    @staticmethod
    def directional_intersection_mask(direction):
        """Cria uma interseção com continuação frontal e um ramo lateral."""

        mask = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(mask, (240, 359), (240, 47), 255, 22)
        edge_x = 0 if direction == "LEFT" else mask.shape[1] - 1
        camera_line_frame.cv2.line(mask, (240, 180), (edge_x, 180), 255, 22)
        return mask

    @staticmethod
    def lateral_only_mask(direction):
        """Cria um ramo lateral sem trecho que alcance a topBand."""

        mask = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(mask, (240, 359), (240, 180), 255, 22)
        edge_x = 0 if direction == "LEFT" else mask.shape[1] - 1
        camera_line_frame.cv2.line(mask, (240, 180), (edge_x, 180), 255, 22)
        return mask

    def test_straight_contour_produces_ninety_degree_angle(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(mask, (240, 359), (240, 47), 255, 22)

        fusion_line = camera_line_frame.extract_fusion_style_line(mask)

        self.assertTrue(fusion_line["valid"])
        self.assertEqual(fusion_line["selection"], "nearCenter")
        self.assertAlmostEqual(fusion_line["angleDeg"], 90.0, delta=1.0)
        self.assertEqual(fusion_line["nearPoint"], {"x": 240, "y": 359})
        self.assertLess(fusion_line["farPoint"]["y"], 70)

    def test_green_left_prefers_valid_left_edge_over_straight_top_band(self):
        mask = self.directional_intersection_mask("LEFT")

        normal = camera_line_frame.extract_fusion_style_line(mask)
        preferred = camera_line_frame.extract_fusion_style_line(
            mask,
            preferred_direction="LEFT",
        )

        self.assertEqual(normal["referenceSource"], "topBand")
        self.assertEqual(preferred["referenceSource"], "leftEdge")
        self.assertEqual(preferred["preferredDirection"], "LEFT")
        self.assertLess(preferred["angleDeg"], 90.0)

    def test_green_right_prefers_valid_right_edge_over_straight_top_band(self):
        mask = self.directional_intersection_mask("RIGHT")

        normal = camera_line_frame.extract_fusion_style_line(mask)
        preferred = camera_line_frame.extract_fusion_style_line(
            mask,
            preferred_direction="RIGHT",
        )

        self.assertEqual(normal["referenceSource"], "topBand")
        self.assertEqual(preferred["referenceSource"], "rightEdge")
        self.assertEqual(preferred["preferredDirection"], "RIGHT")
        self.assertGreater(preferred["angleDeg"], 90.0)

    def test_missing_preferred_edge_uses_existing_fusion_fallback(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(mask, (240, 359), (240, 47), 255, 22)

        normal = camera_line_frame.extract_fusion_style_line(mask)
        preferred = camera_line_frame.extract_fusion_style_line(
            mask,
            preferred_direction="LEFT",
        )

        self.assertEqual(preferred["referenceSource"], normal["referenceSource"])
        self.assertEqual(preferred["farPoint"], normal["farPoint"])
        self.assertEqual(preferred["angleDeg"], normal["angleDeg"])

    def test_green_right_rejects_opposite_top_band_fallback(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(mask, (240, 359), (140, 47), 255, 22)

        preferred = camera_line_frame.extract_fusion_style_line(
            mask,
            preferred_direction="RIGHT",
        )

        self.assertFalse(preferred["valid"])
        self.assertEqual(preferred["referenceSource"], "none")

    def test_green_left_rejects_opposite_top_band_fallback(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(mask, (240, 359), (340, 47), 255, 22)

        preferred = camera_line_frame.extract_fusion_style_line(
            mask,
            preferred_direction="LEFT",
        )

        self.assertFalse(preferred["valid"])
        self.assertEqual(preferred["referenceSource"], "none")

    def test_green_right_never_falls_back_to_left_edge(self):
        mask = self.lateral_only_mask("LEFT")

        normal = camera_line_frame.extract_fusion_style_line(mask)
        preferred = camera_line_frame.extract_fusion_style_line(
            mask,
            preferred_direction="RIGHT",
        )

        self.assertTrue(normal["valid"])
        self.assertEqual(normal["referenceSource"], "leftEdge")
        self.assertFalse(preferred["valid"])
        self.assertEqual(preferred["referenceSource"], "none")

    def test_green_left_never_falls_back_to_right_edge(self):
        mask = self.lateral_only_mask("RIGHT")

        normal = camera_line_frame.extract_fusion_style_line(mask)
        preferred = camera_line_frame.extract_fusion_style_line(
            mask,
            preferred_direction="LEFT",
        )

        self.assertTrue(normal["valid"])
        self.assertEqual(normal["referenceSource"], "rightEdge")
        self.assertFalse(preferred["valid"])
        self.assertEqual(preferred["referenceSource"], "none")

    def test_none_preference_preserves_current_fusion_selection(self):
        mask = self.directional_intersection_mask("LEFT")

        implicit = camera_line_frame.extract_fusion_style_line(mask)
        explicit = camera_line_frame.extract_fusion_style_line(
            mask,
            preferred_direction=None,
        )

        self.assertEqual(explicit["referenceSource"], implicit["referenceSource"])
        self.assertEqual(explicit["farPoint"], implicit["farPoint"])
        self.assertEqual(explicit["angleDeg"], implicit["angleDeg"])

    def test_angle_convention_distinguishes_left_and_right(self):
        left_mask = np.zeros((360, 480), dtype=np.uint8)
        right_mask = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(
            left_mask,
            (240, 359),
            (100, 47),
            255,
            22,
        )
        camera_line_frame.cv2.line(
            right_mask,
            (240, 359),
            (380, 47),
            255,
            22,
        )

        left_line = camera_line_frame.extract_fusion_style_line(left_mask)
        right_line = camera_line_frame.extract_fusion_style_line(right_mask)

        self.assertLess(left_line["angleDeg"], 90.0)
        self.assertGreater(right_line["angleDeg"], 90.0)

    def test_top_band_uses_midpoint_of_widest_transverse_section(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        asymmetric_segment = np.asarray(
            [(350, 100), (300, 114), (340, 114)],
            dtype=np.int32,
        )
        camera_line_frame.cv2.fillPoly(
            mask,
            [asymmetric_segment],
            255,
        )
        contours, _ = camera_line_frame.cv2.findContours(
            mask,
            camera_line_frame.cv2.RETR_EXTERNAL,
            camera_line_frame.cv2.CHAIN_APPROX_SIMPLE,
        )

        top_point, top_y, band_end_y = (
            camera_line_frame.calculate_fusion_style_top_contour(
                mask,
                max(contours, key=camera_line_frame.cv2.contourArea),
            )
        )

        self.assertEqual((top_y, band_end_y), (100, 115))
        self.assertEqual(top_point, (320, 114))

    def test_diagonal_top_target_is_centered_between_row_limits(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(mask, (220, 359), (350, 100), 255, 34)
        contours, _ = camera_line_frame.cv2.findContours(
            mask,
            camera_line_frame.cv2.RETR_EXTERNAL,
            camera_line_frame.cv2.CHAIN_APPROX_SIMPLE,
        )

        top_point, _, _ = (
            camera_line_frame.calculate_fusion_style_top_contour(
                mask,
                max(contours, key=camera_line_frame.cv2.contourArea),
            )
        )
        row_xs = np.flatnonzero(mask[top_point[1]] > 0)

        self.assertGreater(row_xs.size, 0)
        self.assertEqual(
            top_point[0],
            (int(row_xs[0]) + int(row_xs[-1])) // 2,
        )

    def test_top_band_first_stable_target_is_centered_and_farther(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(mask, (220, 359), (350, 100), 255, 34)
        contours, _ = camera_line_frame.cv2.findContours(
            mask,
            camera_line_frame.cv2.RETR_EXTERNAL,
            camera_line_frame.cv2.CHAIN_APPROX_SIMPLE,
        )

        top_point, band_start_y, band_end_y = (
            camera_line_frame.calculate_fusion_style_top_contour(
                mask,
                max(contours, key=camera_line_frame.cv2.contourArea),
            )
        )
        target_row_xs = np.flatnonzero(mask[top_point[1]] > 0)
        previous_target_y = band_end_y - 1
        previous_row_xs = np.flatnonzero(mask[previous_target_y] > 0)
        previous_target = (
            (int(previous_row_xs[0]) + int(previous_row_xs[-1])) // 2,
            previous_target_y,
        )
        robot_reference = (mask.shape[1] / 2.0, mask.shape[0] - 1.0)
        target_distance = math.hypot(
            top_point[0] - robot_reference[0],
            top_point[1] - robot_reference[1],
        )
        previous_target_distance = math.hypot(
            previous_target[0] - robot_reference[0],
            previous_target[1] - robot_reference[1],
        )

        self.assertEqual(top_point[1], band_start_y)
        self.assertEqual(
            top_point[0],
            (int(target_row_xs[0]) + int(target_row_xs[-1])) // 2,
        )
        self.assertLess(top_point[1], previous_target[1])
        self.assertGreater(target_distance, previous_target_distance)

    def test_near_component_wins_over_larger_disconnected_distractor(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(mask, (240, 359), (300, 100), 255, 22)
        camera_line_frame.cv2.rectangle(mask, (30, 60), (190, 250), 255, -1)

        fusion_line = camera_line_frame.extract_fusion_style_line(mask)

        self.assertTrue(fusion_line["valid"])
        self.assertEqual(fusion_line["candidateContourCount"], 2)
        self.assertEqual(fusion_line["selection"], "nearCenter")
        self.assertGreater(fusion_line["farPoint"]["x"], 250)

    def test_connected_ninety_degree_curve_uses_top_contour_band(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        curve = np.asarray(
            [(240, 359), (240, 180), (479, 180)],
            dtype=np.int32,
        )
        camera_line_frame.cv2.polylines(mask, [curve], False, 255, 22)

        fusion_line = camera_line_frame.extract_fusion_style_line(mask)
        envelope = camera_line_frame.resolve_normal_trajectory_envelope(
            mask.shape
        )
        _, right_x = camera_line_frame.normal_trajectory_horizontal_bounds(
            envelope,
            fusion_line["farPoint"]["y"],
        )

        self.assertTrue(fusion_line["valid"])
        self.assertEqual(fusion_line["referenceSource"], "rightEdge")
        self.assertLess(fusion_line["farPoint"]["x"], right_x - 1)
        self.assertGreaterEqual(
            fusion_line["farPoint"]["x"],
            right_x - 1 - mask.shape[1] // 16,
        )
        self.assertAlmostEqual(fusion_line["farPoint"]["y"], 180, delta=2)
        self.assertGreater(fusion_line["topBandPoint"]["x"], 300)
        self.assertGreater(fusion_line["angleDeg"], 90.0)
        self.assertEqual(
            fusion_line["topBand"]["y1"] - fusion_line["topBand"]["y0"],
            15,
        )

    def test_first_stable_top_target_preserves_anticipation(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        curve = np.asarray(
            [
                (240, 359),
                (240, 250),
                (255, 200),
                (300, 150),
                (370, 100),
                (430, 47),
            ],
            dtype=np.int32,
        )
        camera_line_frame.cv2.polylines(mask, [curve], False, 255, 22)

        anticipated = camera_line_frame.extract_fusion_style_line(mask)
        with patch.object(
            camera_line_frame,
            "FUSION_STYLE_TARGET_BAND_HEIGHT_RATIO",
            1.0 / 20.0,
        ):
            previous_band = camera_line_frame.extract_fusion_style_line(mask)

        self.assertEqual(anticipated["farPoint"], previous_band["farPoint"])
        self.assertEqual(anticipated["angleDeg"], previous_band["angleDeg"])

    def test_connected_left_ninety_degree_curve_uses_physical_edge(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        curve = np.asarray(
            [(240, 359), (240, 180), (0, 180)],
            dtype=np.int32,
        )
        camera_line_frame.cv2.polylines(mask, [curve], False, 255, 22)

        fusion_line = camera_line_frame.extract_fusion_style_line(mask)
        envelope = camera_line_frame.resolve_normal_trajectory_envelope(
            mask.shape
        )
        left_x, _ = camera_line_frame.normal_trajectory_horizontal_bounds(
            envelope,
            fusion_line["farPoint"]["y"],
        )

        self.assertTrue(fusion_line["valid"])
        self.assertEqual(fusion_line["referenceSource"], "leftEdge")
        self.assertGreater(fusion_line["farPoint"]["x"], left_x)
        self.assertLessEqual(
            fusion_line["farPoint"]["x"],
            left_x + mask.shape[1] // 16,
        )
        self.assertAlmostEqual(fusion_line["farPoint"]["y"], 180, delta=2)
        self.assertLess(fusion_line["angleDeg"], 90.0)

    def test_edge_center_requires_one_transverse_segment(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        connected_double_crossing = np.asarray(
            [(0, 130), (200, 130), (200, 230), (0, 230)],
            dtype=np.int32,
        )
        camera_line_frame.cv2.polylines(
            mask,
            [connected_double_crossing],
            False,
            255,
            22,
        )
        contours, _ = camera_line_frame.cv2.findContours(
            mask,
            camera_line_frame.cv2.RETR_EXTERNAL,
            camera_line_frame.cv2.CHAIN_APPROX_SIMPLE,
        )
        envelope = camera_line_frame.resolve_normal_trajectory_envelope(
            mask.shape
        )

        centered_point = (
            camera_line_frame.calculate_fusion_style_edge_segment_center(
                mask,
                max(contours, key=camera_line_frame.cv2.contourArea),
                envelope,
                "leftEdge",
            )
        )

        self.assertIsNone(centered_point)

    def test_short_last_visible_segment_keeps_center_target(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(mask, (240, 359), (240, 270), 255, 22)

        fusion_line = camera_line_frame.extract_fusion_style_line(mask)

        self.assertTrue(fusion_line["valid"])
        self.assertEqual(fusion_line["referenceSource"], "topBand")
        self.assertEqual(fusion_line["farPoint"]["x"], 240)
        self.assertAlmostEqual(fusion_line["angleDeg"], 90.0, delta=0.1)

    def test_previous_target_does_not_change_current_geometry_selection(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        connected_path = np.asarray(
            [(240, 359), (240, 210), (30, 120), (450, 80)],
            dtype=np.int32,
        )
        camera_line_frame.cv2.polylines(
            mask,
            [connected_path],
            False,
            255,
            22,
        )
        instantaneous = camera_line_frame.extract_fusion_style_line(mask)
        previous = {
            "valid": True,
            "angleDeg": 35.0,
            "farPoint": {"x": 30, "y": 120},
            "missedFrames": 0,
        }
        with_previous_target = camera_line_frame.extract_fusion_style_line(
            mask,
            previous,
        )

        self.assertEqual(
            with_previous_target["referenceSource"],
            instantaneous["referenceSource"],
        )
        self.assertEqual(
            with_previous_target["farPoint"],
            instantaneous["farPoint"],
        )
        self.assertEqual(
            with_previous_target["angleDeg"],
            instantaneous["angleDeg"],
        )

    def test_extreme_pivot_guard_blocks_only_opposite_jump(self):
        envelope = camera_line_frame.resolve_normal_trajectory_envelope(
            (360, 480)
        )
        extreme_right = camera_line_frame.empty_fusion_style_line()
        extreme_right.update({
            "valid": True,
            "angleDeg": 166.4,
            "farPoint": {"x": 450, "y": 300},
            "referenceSource": "rightEdge",
        })
        camera_line_frame.apply_fusion_extreme_pivot_direction_guard(
            extreme_right,
            None,
            envelope,
        )

        self.assertTrue(extreme_right["pivotDirectionGuardActive"])
        self.assertEqual(
            extreme_right["pivotDirectionGuard"],
            "RIGHT",
        )

        opposite_jump = camera_line_frame.empty_fusion_style_line()
        opposite_jump.update({
            "valid": True,
            "angleDeg": 21.3,
            "farPoint": {"x": 30, "y": 300},
            "referenceSource": "leftEdge",
        })
        camera_line_frame.apply_fusion_extreme_pivot_direction_guard(
            opposite_jump,
            extreme_right,
            envelope,
        )

        self.assertTrue(
            opposite_jump["pivotDirectionGuardRejectedOpposite"]
        )
        self.assertEqual(opposite_jump["angleDeg"], 166.4)
        self.assertEqual(opposite_jump["farPoint"], {"x": 450, "y": 300})

        forward_again = camera_line_frame.empty_fusion_style_line()
        forward_again.update({
            "valid": True,
            "angleDeg": 90.0,
            "farPoint": {"x": 240, "y": 55},
            "referenceSource": "topBand",
        })
        camera_line_frame.apply_fusion_extreme_pivot_direction_guard(
            forward_again,
            opposite_jump,
            envelope,
        )

        self.assertFalse(forward_again["pivotDirectionGuardActive"])
        self.assertEqual(forward_again["pivotDirectionGuard"], "NONE")
        self.assertEqual(forward_again["angleDeg"], 90.0)

    def test_extreme_pivot_guard_releases_and_rearms_without_top_band(self):
        envelope = camera_line_frame.resolve_normal_trajectory_envelope(
            (360, 480)
        )
        extreme_right = camera_line_frame.empty_fusion_style_line()
        extreme_right.update({
            "valid": True,
            "angleDeg": 166.4,
            "farPoint": {"x": 450, "y": 300},
            "referenceSource": "rightEdge",
        })
        camera_line_frame.apply_fusion_extreme_pivot_direction_guard(
            extreme_right,
            None,
            envelope,
        )

        near_forward = camera_line_frame.empty_fusion_style_line()
        near_forward.update({
            "valid": True,
            "angleDeg": 96.6,
            "farPoint": {"x": 265, "y": 143},
            "referenceSource": "rightEdge",
        })
        camera_line_frame.apply_fusion_extreme_pivot_direction_guard(
            near_forward,
            extreme_right,
            envelope,
        )

        self.assertFalse(near_forward["pivotDirectionGuardActive"])
        self.assertEqual(near_forward["pivotDirectionGuard"], "NONE")
        self.assertFalse(
            near_forward["pivotDirectionGuardRejectedOpposite"]
        )
        self.assertEqual(near_forward["angleDeg"], 96.6)
        self.assertEqual(near_forward["farPoint"], {"x": 265, "y": 143})

        valid_left = camera_line_frame.empty_fusion_style_line()
        valid_left.update({
            "valid": True,
            "angleDeg": 21.3,
            "farPoint": {"x": 30, "y": 300},
            "referenceSource": "leftEdge",
        })
        camera_line_frame.apply_fusion_extreme_pivot_direction_guard(
            valid_left,
            near_forward,
            envelope,
        )

        self.assertTrue(valid_left["pivotDirectionGuardActive"])
        self.assertEqual(valid_left["pivotDirectionGuard"], "LEFT")
        self.assertFalse(
            valid_left["pivotDirectionGuardRejectedOpposite"]
        )
        self.assertEqual(valid_left["angleDeg"], 21.3)
        self.assertEqual(valid_left["farPoint"], {"x": 30, "y": 300})

    def test_new_green_can_start_without_previous_extreme_pivot_guard(self):
        mask = self.directional_intersection_mask("LEFT")
        previous_extreme_right = camera_line_frame.empty_fusion_style_line()
        previous_extreme_right.update({
            "valid": True,
            "angleDeg": 166.4,
            "nearPoint": {"x": 240, "y": 359},
            "farPoint": {"x": 450, "y": 300},
            "referenceSource": "rightEdge",
            "selection": "nearCenter",
            "pivotDirectionGuard": "RIGHT",
            "pivotDirectionGuardActive": True,
            "missedFrames": 0,
        })

        blocked = camera_line_frame.extract_fusion_style_line(
            mask,
            previous_fusion_line=previous_extreme_right,
            preferred_direction="LEFT",
        )
        neutralized_history = dict(previous_extreme_right)
        neutralized_history["pivotDirectionGuard"] = "NONE"
        neutralized_history["pivotDirectionGuardActive"] = False
        neutralized_history["pivotDirectionGuardRejectedOpposite"] = False
        accepted_after_neutralizing_guard = (
            camera_line_frame.extract_fusion_style_line(
                mask,
                previous_fusion_line=neutralized_history,
                preferred_direction="LEFT",
            )
        )

        self.assertTrue(blocked["pivotDirectionGuardRejectedOpposite"])
        self.assertEqual(blocked["angleDeg"], 166.4)
        self.assertFalse(
            accepted_after_neutralizing_guard[
                "pivotDirectionGuardRejectedOpposite"
            ]
        )
        self.assertEqual(
            accepted_after_neutralizing_guard["referenceSource"],
            "leftEdge",
        )
        self.assertLess(
            accepted_after_neutralizing_guard["angleDeg"],
            90.0,
        )

    def test_empty_mask_keeps_experimental_result_invalid(self):
        mask = np.zeros((360, 480), dtype=np.uint8)

        fusion_line = camera_line_frame.extract_fusion_style_line(mask)

        self.assertTrue(fusion_line["enabled"])
        self.assertFalse(fusion_line["valid"])
        self.assertIsNone(fusion_line["angleDeg"])
        self.assertIsNone(fusion_line["farPoint"])


    def test_hot_path_skips_scanline_extractor_without_legacy_flag(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(mask, (240, 359), (360, 47), 255, 22)

        with patch.object(
            camera_line_frame,
            "extract_normal_line_trajectory",
        ) as scanline_extractor:
            normal_trajectory, fusion_line = (
                camera_line_frame.extract_line_diagnostics(
                    mask,
                    "down",
                    legacy_debug_enabled=False,
                )
            )

        scanline_extractor.assert_not_called()
        self.assertFalse(normal_trajectory["enabled"])
        self.assertEqual(normal_trajectory["processingMs"], 0.0)
        self.assertTrue(fusion_line["valid"])

    def test_legacy_flag_restores_scanline_extractor(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(mask, (240, 359), (330, 47), 255, 22)

        with patch.object(
            camera_line_frame,
            "extract_normal_line_trajectory",
            wraps=camera_line_frame.extract_normal_line_trajectory,
        ) as scanline_extractor:
            normal_trajectory, fusion_line = (
                camera_line_frame.extract_line_diagnostics(
                    mask,
                    "down",
                    legacy_debug_enabled=True,
                )
            )

        scanline_extractor.assert_called_once_with(mask)
        self.assertTrue(normal_trajectory["enabled"])
        self.assertTrue(fusion_line["valid"])

    def test_overlay_draws_magenta_ray_and_angle_text(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(mask, (240, 359), (360, 47), 255, 22)
        fusion_line = camera_line_frame.extract_fusion_style_line(mask)
        frame = np.zeros((360, 480, 3), dtype=np.uint8)

        with patch.object(
            camera_line_frame.cv2,
            "putText",
            wraps=camera_line_frame.cv2.putText,
        ) as put_text:
            camera_line_frame.draw_fusion_style_line_overlay(
                frame,
                fusion_line,
                {
                    "controlSource": "legacy_normal",
                    "left_power": 0.42,
                    "right_power": 0.31,
                },
            )

        self.assertGreater(np.count_nonzero(frame), 0)
        texts = {call.args[1] for call in put_text.call_args_list}
        self.assertIn(
            f"FusionAngle {fusion_line['angleDeg']:.1f} deg",
            texts,
        )
        self.assertIn("F TARGET", texts)
        timing_text = f"{fusion_line['processingMs']:.2f} ms"
        self.assertIn(timing_text, texts)
        timing_calls = [
            call for call in put_text.call_args_list
            if call.args[1] == timing_text
        ]
        self.assertEqual(len(timing_calls), 2)
        self.assertTrue(all(
            call.args[2][0] > frame.shape[1] // 2 and call.args[2][1] == 20
            for call in timing_calls
        ))
        self.assertIn("Source legacy_normal", texts)
        self.assertIn("Powers L +0.42 R +0.31", texts)


class FusionNormalSteeringControlTests(unittest.TestCase):
    @staticmethod
    def valid_fusion_line(angle_deg):
        return {
            "valid": True,
            "angleDeg": angle_deg,
            "selection": "nearCenter",
            "nearPoint": {"x": 240, "y": 359},
            "farPoint": {"x": 360, "y": 47},
        }

    def test_angle_mapping_uses_small_deadband_and_safe_clamp(self):
        expected_magnitudes = (
            (2.0, 0.0),
            (5.0, 0.04866636322257624),
            (10.0, camera_line_frame.NORMAL_FULL_STEERING_ERROR),
            (15.0, 0.3725925925925926),
            (20.0, 0.4074074074074074),
            (25.0, 0.46),
            (40.0, 0.68),
            (55.0, 0.9),
            (70.0, 1.0),
        )
        for angle_error_deg, expected_magnitude in expected_magnitudes:
            with self.subTest(angle_error_deg=angle_error_deg):
                right = (
                    camera_line_frame.map_fusion_angle_to_steering_error(
                        90.0 + angle_error_deg
                    )
                )
                left = (
                    camera_line_frame.map_fusion_angle_to_steering_error(
                        90.0 - angle_error_deg
                    )
                )
                self.assertTrue(math.isclose(right, expected_magnitude))
                self.assertTrue(math.isclose(left, -expected_magnitude))

        self.assertLess(
            camera_line_frame.map_fusion_angle_to_steering_error(93.0),
            0.02,
        )
        self.assertEqual(
            camera_line_frame.map_fusion_angle_to_steering_error(0.0),
            -1.0,
        )
        self.assertEqual(
            camera_line_frame.map_fusion_angle_to_steering_error(180.0),
            1.0,
        )
        for invalid_angle in (None, float("nan"), -0.1, 180.1):
            with self.subTest(invalid_angle=invalid_angle):
                self.assertIsNone(
                    camera_line_frame.map_fusion_angle_to_steering_error(
                        invalid_angle
                    )
                )

    def test_fusion_power_curve_separates_strong_normal_from_pivot(self):
        expected_right_turn_powers = (
            (10.0, 0.82, 0.66),
            (15.0, 0.8277777777777777, 0.5666666666666667),
            (20.0, 0.8422222222222222, 0.39333333333333337),
            (25.0, 0.85, 0.30),
            (40.0, 0.83, 0.02),
            (45.0, 0.8248148148148148, -0.037037037037037035),
            (55.0, 0.81, -0.20),
            (70.0, 0.78, -0.72),
        )
        for angle_error_deg, expected_outer, expected_inner in (
            expected_right_turn_powers
        ):
            with self.subTest(angle_error_deg=angle_error_deg):
                right_command = (
                    camera_line_frame.map_fusion_angle_to_motor_powers(
                        90.0 + angle_error_deg
                    )
                )
                left_command = (
                    camera_line_frame.map_fusion_angle_to_motor_powers(
                        90.0 - angle_error_deg
                    )
                )

                self.assertTrue(
                    math.isclose(
                        right_command["left_power"],
                        expected_outer,
                    )
                )
                self.assertTrue(
                    math.isclose(
                        right_command["right_power"],
                        expected_inner,
                    )
                )
                self.assertTrue(
                    math.isclose(
                        left_command["left_power"],
                        expected_inner,
                    )
                )
                self.assertTrue(
                    math.isclose(
                        left_command["right_power"],
                        expected_outer,
                    )
                )

        for invalid_angle in (None, float("nan"), -0.1, 180.1):
            with self.subTest(invalid_power_angle=invalid_angle):
                self.assertIsNone(
                    camera_line_frame.map_fusion_angle_to_motor_powers(
                        invalid_angle
                    )
                )

    def test_green_uses_directed_fusion_angle_and_unchanged_power_mapper(self):
        for green_direction, preferred_direction, fusion_angle in (
            ("ESQUERDA", "LEFT", 75.0),
            ("DIREITA", "RIGHT", 105.0),
        ):
            with self.subTest(green_direction=green_direction):
                fusion_line = self.valid_fusion_line(fusion_angle)
                fusion_line["preferredDirection"] = preferred_direction
                fusion_line["fusionSpeedScale"] = 1.0
                sensors = sensor_values(
                    steering_error=(
                        0.55 if green_direction == "ESQUERDA" else -0.55
                    )
                )
                sensors["headingAngle"] = (
                    30.0 if green_direction == "ESQUERDA" else -30.0
                )

                result = calculate_command(
                    sensors,
                    green_direction=green_direction,
                    fusion_style_line=fusion_line,
                )
                expected_powers = (
                    camera_line_frame.map_fusion_angle_to_motor_powers(
                        fusion_angle
                    )
                )

                self.assertEqual(result["controlSource"], "fusion-green")
                self.assertTrue(result["fusionControlActive"])
                self.assertEqual(
                    result["fusionPreferredDirection"],
                    preferred_direction,
                )
                self.assertTrue(math.isclose(
                    result["finalSteering"],
                    camera_line_frame.map_fusion_angle_to_steering_error(
                        fusion_angle
                    ),
                ))
                self.assertTrue(math.isclose(
                    result["left_power"],
                    expected_powers["left_power"],
                ))
                self.assertTrue(math.isclose(
                    result["right_power"],
                    expected_powers["right_power"],
                ))

    def test_fusion_power_curve_is_continuous_at_calibration_points(self):
        boundary_errors_deg = (2.0, 10.0, 25.0, 40.0, 55.0, 70.0)
        for boundary_error_deg in boundary_errors_deg:
            with self.subTest(boundary_error_deg=boundary_error_deg):
                before = (
                    camera_line_frame.map_fusion_angle_to_motor_powers(
                        90.0 + boundary_error_deg - 0.000001
                    )
                )
                after = (
                    camera_line_frame.map_fusion_angle_to_motor_powers(
                        90.0 + boundary_error_deg + 0.000001
                    )
                )
                self.assertLess(
                    abs(before["left_power"] - after["left_power"]),
                    0.000001,
                )
                self.assertLess(
                    abs(before["right_power"] - after["right_power"]),
                    0.000001,
                )

    def test_straight_speed_recovers_progressively_with_stable_target(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(mask, (240, 359), (240, 47), 255, 22)
        history = None
        observed_powers = []
        observed_scales = []

        for _frame_index in range(
            camera_line_frame.FUSION_TARGET_STABLE_FRAMES_FOR_FULL_SPEED
        ):
            fusion_line = camera_line_frame.extract_fusion_style_line(
                mask,
                history,
            )
            result = calculate_command(
                sensor_values(steering_error=0.0),
                fusion_style_line=fusion_line,
            )
            observed_powers.append(result["left_power"])
            observed_scales.append(result["fusionSpeedScale"])
            history = camera_line_frame.update_fusion_style_history(
                history,
                fusion_line,
            )

        self.assertTrue(math.isclose(observed_powers[0], 0.69))
        self.assertTrue(math.isclose(observed_powers[-1], 0.75))
        self.assertEqual(observed_powers, sorted(observed_powers))
        self.assertEqual(observed_scales, sorted(observed_scales))

    def test_speed_limit_preserves_mapper_yaw_differential(self):
        raw_command = camera_line_frame.map_fusion_angle_to_motor_powers(100.0)
        limited_command = camera_line_frame.apply_fusion_forward_speed_limit(
            raw_command,
            camera_line_frame.FUSION_MIN_FORWARD_SPEED_SCALE,
        )

        self.assertTrue(math.isclose(
            raw_command["left_power"] - raw_command["right_power"],
            limited_command["left_power"] - limited_command["right_power"],
        ))
        self.assertTrue(math.isclose(
            (
                limited_command["left_power"]
                + limited_command["right_power"]
            ) / 2.0,
            camera_line_frame.NORMAL_BASE_POWER
            * camera_line_frame.FUSION_MIN_FORWARD_SPEED_SCALE,
        ))

    def test_strong_correction_delays_full_speed_on_curve_exit(self):
        strong_mask = np.zeros((360, 480), dtype=np.uint8)
        straight_mask = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(
            strong_mask,
            (240, 359),
            (0, 47),
            255,
            22,
        )
        camera_line_frame.cv2.line(
            straight_mask,
            (240, 359),
            (240, 47),
            255,
            22,
        )
        strong_line = camera_line_frame.extract_fusion_style_line(strong_mask)
        history = camera_line_frame.update_fusion_style_history(
            None,
            strong_line,
        )

        exit_powers = []
        for _frame_index in range(
            camera_line_frame.FUSION_TARGET_STABLE_FRAMES_FOR_FULL_SPEED
        ):
            straight_line = camera_line_frame.extract_fusion_style_line(
                straight_mask,
                history,
            )
            result = calculate_command(
                sensor_values(steering_error=0.0),
                fusion_style_line=straight_line,
            )
            exit_powers.append(result["left_power"])
            history = camera_line_frame.update_fusion_style_history(
                history,
                straight_line,
            )

        self.assertLess(exit_powers[0], 0.75)
        self.assertTrue(math.isclose(exit_powers[-1], 0.75))
        self.assertEqual(exit_powers, sorted(exit_powers))

    def test_short_or_reacquired_target_keeps_moderate_speed(self):
        short_mask = np.zeros((360, 480), dtype=np.uint8)
        full_mask = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(
            short_mask,
            (240, 359),
            (240, 270),
            255,
            22,
        )
        camera_line_frame.cv2.line(
            full_mask,
            (240, 359),
            (240, 47),
            255,
            22,
        )
        short_line = camera_line_frame.extract_fusion_style_line(short_mask)
        short_result = calculate_command(
            sensor_values(steering_error=0.0),
            fusion_style_line=short_line,
        )
        self.assertLess(
            short_line["targetLengthRatio"],
            camera_line_frame.FUSION_TARGET_SHORT_LENGTH_RATIO,
        )
        self.assertTrue(math.isclose(short_result["left_power"], 0.69))

        established = camera_line_frame.extract_fusion_style_line(full_mask)
        history = camera_line_frame.update_fusion_style_history(
            None,
            established,
        )
        history = camera_line_frame.update_fusion_style_history(
            history,
            camera_line_frame.empty_fusion_style_line(),
        )
        reacquired = camera_line_frame.extract_fusion_style_line(
            full_mask,
            history,
        )
        reacquired_result = calculate_command(
            sensor_values(steering_error=0.0),
            fusion_style_line=reacquired,
        )

        self.assertTrue(reacquired["targetReacquired"])
        self.assertEqual(reacquired["targetConsistency"], 0.0)
        self.assertTrue(math.isclose(reacquired_result["left_power"], 0.69))

    def test_fusion_is_primary_only_in_normal_following(self):
        sensors = sensor_values(steering_error=-0.20)
        fusion_line = self.valid_fusion_line(110.0)

        result = calculate_command(
            sensors,
            fusion_style_line=fusion_line,
        )

        self.assertEqual(result["controlSource"], "fusion")
        self.assertTrue(result["fusionControlActive"])
        self.assertEqual(result["fusionAngle"], 110.0)
        self.assertEqual(result["filteredFusionAngle"], 110.0)
        self.assertEqual(
            result["fusionSteeringError"],
            0.4074074074074074,
        )
        self.assertTrue(math.isclose(
            result["left_power"],
            0.8422222222222222,
        ))
        self.assertTrue(math.isclose(
            result["right_power"],
            0.39333333333333337,
        ))
        self.assertGreater(result["left_power"], 0.0)
        self.assertGreater(result["right_power"], 0.0)

    def test_fusion_final_command_preserves_small_negative_power(self):
        fusion_angle = 140.0
        raw_command = camera_line_frame.map_fusion_angle_to_motor_powers(
            fusion_angle
        )
        result = calculate_command(
            sensor_values(steering_error=0.0),
            fusion_style_line=self.valid_fusion_line(fusion_angle),
        )

        self.assertTrue(math.isclose(
            result["left_power"],
            raw_command["left_power"],
        ))
        self.assertTrue(math.isclose(
            result["right_power"],
            raw_command["right_power"],
        ))
        self.assertGreater(result["right_power"], -0.69)
        self.assertLess(result["right_power"], 0.0)

    def test_current_fusion_target_prevents_new_blind_search(self):
        search_tracker = camera_line_frame.VirtualLineSearchTracker()
        search_tracker.last_direction = "LEFT"

        result = calculate_command(
            sensor_values(steering_error=None),
            line_search_tracker=search_tracker,
            fusion_style_line=self.valid_fusion_line(18.0),
        )

        self.assertEqual(result["controlSource"], "fusion")
        self.assertTrue(result["fusionControlActive"])
        self.assertFalse(search_tracker.active)
        self.assertTrue(math.isclose(result["left_power"], -0.72))
        self.assertTrue(math.isclose(result["right_power"], 0.78))

    def test_active_blind_search_keeps_priority_over_fusion(self):
        search_tracker = camera_line_frame.VirtualLineSearchTracker()
        search_tracker.start("LEFT")
        sensors = sensor_values(None, None, None)
        sensors["farTrusted"] = False
        sensors["mediumTrusted"] = False
        sensors["farPosition"] = None

        result = calculate_command(
            sensors,
            line_search_tracker=search_tracker,
            fusion_style_line=self.valid_fusion_line(130.0),
        )

        self.assertEqual(result["controlSource"], "virtual-blind-search")
        self.assertFalse(result["fusionControlActive"])
        self.assertTrue(search_tracker.active)

    def test_automatic_search_requires_two_lost_frames(self):
        search_tracker = camera_line_frame.VirtualLineSearchTracker()
        search_tracker.last_direction = "LEFT"
        sensors = sensor_values(None, None, None)
        sensors["farTrusted"] = False
        sensors["mediumTrusted"] = False
        sensors["farPosition"] = None

        first = calculate_command(
            sensors,
            line_search_tracker=search_tracker,
        )
        second = calculate_command(
            sensors,
            line_search_tracker=search_tracker,
        )

        self.assertEqual(first["controlSource"], "virtual-search-wait")
        self.assertEqual((first["left_power"], first["right_power"]), (0.0, 0.0))
        self.assertEqual(
            second["controlSource"],
            "virtual-blind-search-backup",
        )
        self.assertTrue(search_tracker.active)
        self.assertEqual(
            (second["left_power"], second["right_power"]),
            (
                camera_line_frame.VIRTUAL_BLIND_SEARCH_BACKUP_POWER,
                camera_line_frame.VIRTUAL_BLIND_SEARCH_BACKUP_POWER,
            ),
        )

        backup_results = [second]
        for _ in range(
            camera_line_frame.VIRTUAL_BLIND_SEARCH_BACKUP_FRAMES - 1
        ):
            backup_results.append(
                calculate_command(
                    sensors,
                    line_search_tracker=search_tracker,
                )
            )
        self.assertTrue(
            all(
                result["controlSource"]
                == "virtual-blind-search-backup"
                for result in backup_results
            )
        )

        search = calculate_command(
            sensors,
            line_search_tracker=search_tracker,
        )
        self.assertEqual(search["controlSource"], "virtual-blind-search")
        self.assertLess(search["left_power"], search["right_power"])

    def test_recent_fusion_direction_overrides_stale_virtual_search_side(self):
        fusion_history = self.valid_fusion_line(150.0)
        fusion_history["missedFrames"] = 2
        preferred_direction = (
            camera_line_frame.fusion_style_blind_search_direction(
                fusion_history
            )
        )
        self.assertEqual(preferred_direction, "RIGHT")

        search_tracker = camera_line_frame.VirtualLineSearchTracker()
        search_tracker.last_direction = "LEFT"
        sensors = sensor_values(None, None, None)
        sensors["farTrusted"] = False
        sensors["mediumTrusted"] = False
        sensors["farPosition"] = None

        first = calculate_command(
            sensors,
            line_search_tracker=search_tracker,
            blind_search_preferred_direction=preferred_direction,
        )
        second = calculate_command(
            sensors,
            line_search_tracker=search_tracker,
            blind_search_preferred_direction=preferred_direction,
        )

        self.assertEqual(first["controlSource"], "virtual-search-wait")
        self.assertEqual(
            second["controlSource"],
            "virtual-blind-search-backup",
        )
        self.assertEqual(search_tracker.initial_direction, "RIGHT")

        for _ in range(
            camera_line_frame.VIRTUAL_BLIND_SEARCH_BACKUP_FRAMES - 1
        ):
            calculate_command(
                sensors,
                line_search_tracker=search_tracker,
                blind_search_preferred_direction=preferred_direction,
            )
        search = calculate_command(
            sensors,
            line_search_tracker=search_tracker,
            blind_search_preferred_direction=preferred_direction,
        )

        self.assertEqual(search["controlSource"], "virtual-blind-search")
        self.assertGreater(search["left_power"], search["right_power"])

    def test_straight_or_stale_fusion_does_not_override_virtual_side(self):
        straight_history = self.valid_fusion_line(
            90.0 + camera_line_frame.FUSION_STEERING_DEADBAND_DEG
        )
        stale_history = self.valid_fusion_line(150.0)
        stale_history["missedFrames"] = (
            camera_line_frame.FUSION_TARGET_HISTORY_MAX_MISSED_FRAMES + 1
        )

        self.assertIsNone(
            camera_line_frame.fusion_style_blind_search_direction(
                straight_history
            )
        )
        self.assertIsNone(
            camera_line_frame.fusion_style_blind_search_direction(
                stale_history
            )
        )

    def test_line_reacquisition_interrupts_automatic_search_backup(self):
        search_tracker = camera_line_frame.VirtualLineSearchTracker()
        missing_sensors = sensor_values(None, None, None)
        missing_sensors["farTrusted"] = False
        missing_sensors["mediumTrusted"] = False

        calculate_command(
            missing_sensors,
            line_search_tracker=search_tracker,
        )
        backup = calculate_command(
            missing_sensors,
            line_search_tracker=search_tracker,
        )
        self.assertEqual(
            backup["controlSource"],
            "virtual-blind-search-backup",
        )

        recovered = calculate_command(
            sensor_values(None, 0.0, None),
            line_search_tracker=search_tracker,
        )
        self.assertFalse(search_tracker.active)
        self.assertNotEqual(
            recovered["controlSource"],
            "virtual-blind-search-backup",
        )
        self.assertGreaterEqual(recovered["left_power"], 0.0)
        self.assertGreaterEqual(recovered["right_power"], 0.0)

    def test_missing_target_uses_virtual_fallback(self):
        sensors = sensor_values(steering_error=-0.20)
        fusion_line = self.valid_fusion_line(110.0)
        fusion_line["farPoint"] = None

        result = calculate_command(
            sensors,
            fusion_style_line=fusion_line,
        )

        self.assertEqual(result["controlSource"], "virtual")
        self.assertFalse(result["fusionControlActive"])
        self.assertEqual(result["steeringError"], -0.20)
        self.assertIsNone(result["fusionSteeringError"])

    def test_unanchored_component_uses_virtual_fallback(self):
        sensors = sensor_values(steering_error=0.14)
        fusion_line = self.valid_fusion_line(70.0)
        fusion_line["selection"] = "deepestFallback"

        result = calculate_command(
            sensors,
            fusion_style_line=fusion_line,
        )

        self.assertEqual(result["controlSource"], "virtual")
        self.assertFalse(result["fusionControlActive"])
        self.assertEqual(result["steeringError"], 0.14)

    def test_fusion_target_does_not_replace_existing_recovery(self):
        sensors = sensor_values(steering_error=None)
        tracker = camera_line_frame.VirtualTurnStateTracker()
        tracker.state = camera_line_frame.VIRTUAL_STATE_REORIENT_LEFT

        result = calculate_command(
            sensors,
            tracker=tracker,
            fusion_style_line=self.valid_fusion_line(130.0),
        )

        self.assertEqual(
            result["virtualState"],
            camera_line_frame.VIRTUAL_STATE_REORIENT_LEFT,
        )
        self.assertEqual(result["controlSource"], "virtual-reorient")
        self.assertFalse(result["fusionControlActive"])
        self.assertEqual(result["steeringError"], -1.0)

    def test_gap_ignores_valid_fusion_control(self):
        sensors = sensor_values(steering_error=0.12)
        fusion_line = self.valid_fusion_line(130.0)

        baseline = calculate_command(sensors, gap_active=True)
        result = calculate_command(
            sensors,
            fusion_style_line=fusion_line,
            gap_active=True,
        )

        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (baseline["left_power"], baseline["right_power"]),
        )
        self.assertEqual(
            result["controlSource"],
            baseline["controlSource"],
        )
        self.assertFalse(result["fusionControlActive"])

    def test_existing_hard_corner_keeps_priority_over_fusion(self):
        tracker = camera_line_frame.VirtualMediumSpinTracker()
        entry_sensors = sensor_values(
            0.0,
            -0.72,
            None,
            near_fine_position=-0.38,
        )
        entry_sensors["farPosition"] = None
        calculate_command(entry_sensors, medium_spin_tracker=tracker)

        fusion_line = self.valid_fusion_line(130.0)
        held = calculate_command(
            entry_sensors,
            medium_spin_tracker=tracker,
            fusion_style_line=fusion_line,
        )

        self.assertEqual(
            tracker.hard_corner_state,
            camera_line_frame.PIVOT_STATE_LEFT,
        )
        self.assertEqual(
            (held["left_power"], held["right_power"]),
            (-0.72, 0.72),
        )
        self.assertEqual(held["controlSource"], "virtual")
        self.assertFalse(held["fusionControlActive"])


class VirtualSensorRegressionTests(unittest.TestCase):
    @staticmethod
    def read_directional_green_stripe(
        green_direction,
        far_center_x,
        medium_center_x,
    ):
        """Cria uma faixa trusted dentro do lado selecionado pelo GREEN."""

        mask = np.zeros((360, 480), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(mask.shape)
        selected_side = (
            "left" if green_direction == "ESQUERDA" else "right"
        )
        for row_name, center_x in (
            ("far", far_center_x),
            ("medium", medium_center_x),
        ):
            for region in camera_line_frame.virtual_sensor_regions(
                geometry[row_name][selected_side]
            ):
                stripe_x0 = max(region["x0"], center_x - 15)
                stripe_x1 = min(region["x1"], center_x + 15)
                mask[
                    region["y0"]:region["y1"],
                    stripe_x0:stripe_x1,
                ] = 255

        return camera_line_frame.read_virtual_line_sensors(
            mask,
            green_direction,
            curva_verde_iniciada=False,
        )

    def test_near_fine_position_is_centered(self):
        mask = np.zeros((101, 101), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            mask.shape
        )
        near = geometry["near"]
        mask[near["center"]["y0"]:near["center"]["y1"], 50] = 255

        position = camera_line_frame.calculate_virtual_near_fine_position(
            mask,
            near,
        )

        self.assertTrue(math.isclose(position, 0.0, abs_tol=1e-9))

    def test_near_fine_position_reaches_left(self):
        mask = np.zeros((101, 101), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            mask.shape
        )
        near = geometry["near"]
        mask[near["center"]["y0"]:near["center"]["y1"], 0] = 255

        position = camera_line_frame.calculate_virtual_near_fine_position(
            mask,
            near,
        )

        self.assertTrue(math.isclose(position, -1.0, abs_tol=1e-9))

    def test_near_fine_position_reaches_right(self):
        mask = np.zeros((101, 101), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            mask.shape
        )
        near = geometry["near"]
        mask[near["center"]["y0"]:near["center"]["y1"], 100] = 255

        position = camera_line_frame.calculate_virtual_near_fine_position(
            mask,
            near,
        )

        self.assertTrue(math.isclose(position, 1.0, abs_tol=1e-9))

    def test_near_fine_position_is_invalid_without_line(self):
        mask = np.zeros((101, 101), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            mask.shape
        )

        position = camera_line_frame.calculate_virtual_near_fine_position(
            mask,
            geometry["near"],
        )

        self.assertIsNone(position)

    def test_near_fine_is_published_when_near_center_is_active(self):
        mask = np.zeros((101, 101), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            mask.shape
        )
        near = geometry["near"]
        mask[near["center"]["y0"]:near["center"]["y1"], 57:60] = 255

        sensors = camera_line_frame.read_virtual_line_sensors(mask)

        self.assertGreater(sensors["nearCenter"], 0.0)
        self.assertGreater(sensors["nearFinePosition"], 0.10)

    def test_pixels_outside_near_center_do_not_create_local_detection(self):
        mask = np.zeros((101, 101), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            mask.shape
        )
        near_position = geometry["near"]["position"]
        center_x0 = geometry["near"]["center"]["x0"]
        mask[
            near_position["y0"]:near_position["y1"],
            near_position["x0"]:center_x0,
        ] = 255

        sensors = camera_line_frame.read_virtual_line_sensors(mask)

        self.assertEqual(sensors["nearCenter"], 0.0)
        self.assertIsNone(sensors["nearFinePosition"])
        self.assertIsNone(sensors["steeringError"])

    def test_heading_keeps_far_as_primary_lookahead(self):
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            (360, 480)
        )
        expected = camera_line_frame.calculate_virtual_heading_angle(
            far_position=-0.40,
            near_fine_position=0.20,
            geometry=geometry,
        )

        actual = camera_line_frame.calculate_virtual_heading_angle(
            far_position=-0.40,
            near_fine_position=0.20,
            geometry=geometry,
            medium_position=1.0,
        )

        self.assertTrue(math.isclose(actual, expected))

    def test_heading_uses_physical_medium_geometry_when_far_is_invalid(self):
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            (360, 480)
        )
        medium_position = 1.0
        near_fine_position = -1.0
        medium_point = camera_line_frame.virtual_row_position_to_point(
            medium_position,
            geometry["medium"],
        )
        near_point = camera_line_frame.virtual_fine_position_to_point(
            near_fine_position,
            geometry["near"]["position"],
        )
        expected = math.degrees(math.atan2(
            medium_point[0] - near_point[0],
            near_point[1] - medium_point[1],
        ))

        actual = camera_line_frame.calculate_virtual_heading_angle(
            far_position=None,
            near_fine_position=near_fine_position,
            geometry=geometry,
            medium_position=medium_position,
        )

        self.assertTrue(math.isclose(actual, expected))

    def test_sensor_reading_uses_medium_heading_fallback(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            mask.shape
        )
        medium_right = geometry["medium"]["right"]
        near_center = geometry["near"]["center"]
        mask[
            medium_right["y0"]:medium_right["y1"],
            medium_right["x0"]:medium_right["x1"],
        ] = 255
        mask[
            near_center["y0"]:near_center["y1"],
            near_center["x0"]:near_center["x1"],
        ] = 255

        sensors = camera_line_frame.read_virtual_line_sensors(mask)
        expected = camera_line_frame.calculate_virtual_heading_angle(
            far_position=None,
            near_fine_position=sensors["nearFinePosition"],
            geometry=geometry,
            medium_position=sensors["mediumPosition"],
        )

        self.assertIsNone(sensors["farPosition"])
        self.assertIsNotNone(sensors["mediumPosition"])
        self.assertGreater(sensors["nearCenter"], 0.0)
        self.assertIsNotNone(sensors["nearFinePosition"])
        self.assertTrue(math.isclose(sensors["headingAngle"], expected))

    def test_far_medium_diagonal_steers_without_near_center(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            mask.shape
        )
        far_right = geometry["far"]["right"]
        medium_left = geometry["medium"]["left"]
        mask[
            far_right["y0"]:far_right["y1"],
            far_right["x0"]:far_right["x1"],
        ] = 255
        mask[
            medium_left["y0"]:medium_left["y1"],
            medium_left["x0"]:medium_left["x1"],
        ] = 255

        sensors = camera_line_frame.read_virtual_line_sensors(mask)
        command = calculate_command(sensors, mask=mask)

        self.assertEqual(sensors["nearCenter"], 0.0)
        self.assertIsNone(sensors["nearFinePosition"])
        self.assertGreater(sensors["farPosition"], 0.0)
        self.assertLess(sensors["mediumPosition"], 0.0)
        self.assertGreater(sensors["headingAngle"], 0.0)
        self.assertGreater(sensors["steeringError"], 0.0)
        self.assertNotEqual(
            (command["left_power"], command["right_power"]),
            (0.0, 0.0),
        )

    def test_far_medium_left_offset_uses_medium_lateral_correction(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            mask.shape
        )
        far_left = geometry["far"]["left"]
        medium_left = geometry["medium"]["left"]
        mask[
            far_left["y0"]:far_left["y1"],
            far_left["x0"]:far_left["x1"],
        ] = 255
        mask[
            medium_left["y0"]:medium_left["y1"],
            medium_left["x0"]:medium_left["x1"],
        ] = 255

        sensors = camera_line_frame.read_virtual_line_sensors(mask)
        command = calculate_command(sensors, mask=mask)
        heading_normalized = max(
            -1.0,
            min(
                1.0,
                sensors["headingAngle"]
                / camera_line_frame.VIRTUAL_HEADING_FULL_SCALE_DEG,
            ),
        )
        expected_steering = max(
            -1.0,
            min(
                1.0,
                0.50 * sensors["mediumPosition"]
                + camera_line_frame.VIRTUAL_HEADING_GAIN
                * heading_normalized,
            ),
        )

        self.assertIsNone(sensors["nearFinePosition"])
        self.assertTrue(math.isclose(sensors["farPosition"], -1.0))
        self.assertTrue(math.isclose(sensors["mediumPosition"], -1.0))
        self.assertTrue(math.isclose(
            sensors["steeringError"],
            expected_steering,
        ))
        self.assertLess(sensors["steeringError"], -0.60)
        self.assertLess(command["finalSteering"], -0.60)
        self.assertNotEqual(
            (command["left_power"], command["right_power"]),
            (0.0, 0.0),
        )

    def test_medium_left_keeps_authority_without_near_or_far(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            mask.shape
        )
        medium_left = geometry["medium"]["left"]
        mask[
            medium_left["y0"]:medium_left["y1"],
            medium_left["x0"]:medium_left["x1"],
        ] = 255

        sensors = camera_line_frame.read_virtual_line_sensors(mask)
        command = calculate_command(sensors, mask=mask)

        self.assertIsNone(sensors["farPosition"])
        self.assertTrue(math.isclose(sensors["mediumPosition"], -1.0))
        self.assertIsNone(sensors["nearFinePosition"])
        self.assertIsNone(sensors["headingAngle"])
        self.assertTrue(math.isclose(sensors["steeringError"], -0.50))
        self.assertEqual(command["controlSource"], "virtual")
        self.assertNotEqual(
            (command["left_power"], command["right_power"]),
            (0.0, 0.0),
        )

    def test_far_right_keeps_authority_without_near_or_medium(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            mask.shape
        )
        far_right = geometry["far"]["right"]
        mask[
            far_right["y0"]:far_right["y1"],
            far_right["x0"]:far_right["x1"],
        ] = 255

        sensors = camera_line_frame.read_virtual_line_sensors(mask)
        command = calculate_command(sensors, mask=mask)

        self.assertTrue(math.isclose(sensors["farPosition"], 1.0))
        self.assertIsNone(sensors["mediumPosition"])
        self.assertIsNone(sensors["nearFinePosition"])
        self.assertIsNone(sensors["headingAngle"])
        self.assertTrue(math.isclose(
            sensors["steeringError"],
            camera_line_frame.VIRTUAL_HEADING_GAIN,
        ))
        self.assertEqual(command["controlSource"], "virtual")
        self.assertNotEqual(
            (command["left_power"], command["right_power"]),
            (0.0, 0.0),
        )

    def test_green_does_not_enable_medium_only_authority_gate(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            mask.shape
        )
        medium_left = geometry["medium"]["left"]
        mask[
            medium_left["y0"]:medium_left["y1"],
            medium_left["x0"]:medium_left["x1"],
        ] = 255

        sensors = camera_line_frame.read_virtual_line_sensors(
            mask,
            "DIREITA",
        )

        self.assertIsNone(sensors["steeringError"])

    def test_single_forward_sensor_does_not_create_gate_heading(self):
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            (360, 480)
        )
        cases = (
            (1.0, None),
            (None, -1.0),
        )
        for far_position, medium_position in cases:
            with self.subTest(
                far_position=far_position,
                medium_position=medium_position,
            ):
                heading_angle = (
                    camera_line_frame.calculate_virtual_heading_angle(
                        far_position=far_position,
                        near_fine_position=None,
                        geometry=geometry,
                        medium_position=medium_position,
                    )
                )
                self.assertIsNone(heading_angle)

    def test_near_center_without_lookahead_keeps_straight_steering(self):
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            (360, 480)
        )
        heading_angle = camera_line_frame.calculate_virtual_heading_angle(
            far_position=None,
            near_fine_position=0.35,
            geometry=geometry,
            medium_position=None,
        )

        steering_error = camera_line_frame.calculate_virtual_steering_error(
            near_fine_position_valid=True,
            heading_angle=heading_angle,
        )

        self.assertIsNone(heading_angle)
        self.assertEqual(steering_error, 0.0)

    def test_fine_centering_deadband_zeros_small_center_error(self):
        deadband = camera_line_frame.VIRTUAL_FINE_CENTER_DEADBAND
        for near_fine_position in (-deadband, -0.02, 0.0, 0.02, deadband):
            with self.subTest(near_fine_position=near_fine_position):
                sensors = sensor_values(
                    steering_error=0.0,
                    near_fine_position=near_fine_position,
                )

                result = calculate_command(sensors)

                self.assertEqual(result["fineCorrection"], 0.0)
                self.assertEqual(result["finalSteering"], 0.0)

    def test_fine_centering_deadband_is_continuous_and_preserves_range(self):
        deadband = camera_line_frame.VIRTUAL_FINE_CENTER_DEADBAND
        epsilon = 1e-6
        expected_fine = epsilon / (1.0 - deadband)
        for sign in (-1.0, 1.0):
            near_fine_position = sign * (deadband + epsilon)
            with self.subTest(near_fine_position=near_fine_position):
                sensors = sensor_values(
                    steering_error=0.0,
                    near_fine_position=near_fine_position,
                )

                result = calculate_command(sensors)

                self.assertTrue(math.isclose(
                    result["fineCorrection"],
                    sign * expected_fine
                    * camera_line_frame.VIRTUAL_FINE_CENTER_GAIN,
                    rel_tol=1e-9,
                    abs_tol=1e-12,
                ))

        self.assertEqual(
            camera_line_frame.apply_virtual_fine_center_deadband(-1.0),
            -1.0,
        )
        self.assertEqual(
            camera_line_frame.apply_virtual_fine_center_deadband(1.0),
            1.0,
        )

    def test_fine_centering_gain_is_applied_before_normal_clamp(self):
        sensors = sensor_values(
            steering_error=0.25,
            medium_position=None,
            far_band_position=None,
            near_fine_position=1.0,
        )

        result = calculate_command(
            sensors,
            pivot_state_tracker=camera_line_frame.VirtualPivotStateTracker(),
        )

        self.assertTrue(math.isclose(
            result["fineCorrection"],
            camera_line_frame.VIRTUAL_FINE_CENTER_GAIN,
        ))
        self.assertTrue(math.isclose(
            result["finalSteering"],
            camera_line_frame.NORMAL_FULL_STEERING_ERROR,
        ))
        self.assertEqual(
            (result["left_power"], result["right_power"]),
            expected_normal_motor_powers(
                camera_line_frame.NORMAL_FULL_STEERING_ERROR
            ),
        )
        self.assertGreater(result["right_power"], 0.0)

    def test_fine_centering_requires_near_center_and_normal_steering(self):
        cases = (
            (0.20, 0.0, 0.0, 0.20),
            (
                camera_line_frame.NORMAL_FULL_STEERING_ERROR + 0.01,
                0.20,
                0.0,
                camera_line_frame.NORMAL_FULL_STEERING_ERROR,
            ),
        )
        for (
            protected_steering,
            near_center,
            expected_correction,
            expected_steering,
        ) in cases:
            with self.subTest(
                protected_steering=protected_steering,
                near_center=near_center,
            ):
                sensors = sensor_values(
                    steering_error=protected_steering,
                    near_fine_position=1.0,
                )
                sensors["nearCenter"] = near_center
                result = calculate_command(sensors)
                self.assertTrue(math.isclose(
                    result["fineCorrection"],
                    expected_correction,
                ))
                self.assertTrue(math.isclose(
                    result["finalSteering"],
                    expected_steering,
                ))

    def test_fine_centering_does_not_change_green_or_gap(self):
        sensors = sensor_values(
            steering_error=0.10,
            medium_position=None,
            far_band_position=None,
            near_fine_position=1.0,
        )
        for green_direction, gap_active, expected_source in (
            ("DIREITA", False, "green-entry-pivot"),
            ("NENHUMA", True, "virtual-gap-far"),
        ):
            with self.subTest(expected_source=expected_source):
                result = calculate_command(
                    sensors,
                    green_direction=green_direction,
                    gap_active=gap_active,
                )
                self.assertEqual(result["fineCorrection"], 0.0)
                self.assertEqual(result["controlSource"], expected_source)

    def test_virtual_overlay_draws_only_near_far_direction(self):
        command = calculate_command(sensor_values(
            steering_error=0.10,
            near_fine_position=0.50,
        ))
        frame = np.zeros((360, 480, 3), dtype=np.uint8)
        with (
            patch.object(camera_line_frame.cv2, "putText") as put_text,
            patch.object(camera_line_frame.cv2, "rectangle") as rectangle,
            patch.object(camera_line_frame.cv2, "line") as line,
            patch.object(camera_line_frame.cv2, "circle") as circle,
        ):
            camera_line_frame.draw_virtual_sensor_geometry(
                frame,
                command,
                show_debug_details=False,
            )

        put_text.assert_not_called()
        rectangle.assert_not_called()
        line.assert_called_once()
        self.assertEqual(circle.call_count, 2)

    def test_virtual_debug_overlay_draws_only_near_center(self):
        command = calculate_command(sensor_values(
            steering_error=0.10,
            near_fine_position=0.0,
        ))
        frame = np.zeros((360, 480, 3), dtype=np.uint8)
        with (
            patch.object(camera_line_frame.cv2, "putText") as put_text,
            patch.object(camera_line_frame.cv2, "rectangle"),
            patch.object(camera_line_frame.cv2, "line"),
            patch.object(camera_line_frame.cv2, "circle"),
        ):
            camera_line_frame.draw_virtual_sensor_geometry(frame, command)

        near_texts = [
            call.args[1]
            for call in put_text.call_args_list
            if call.args[1].startswith("NEAR")
        ]
        self.assertEqual(len(near_texts), 2)
        self.assertTrue(near_texts[0].startswith("NEAR-C "))
        self.assertTrue(near_texts[1].startswith("NEAR FINE POS "))

    def test_down_overlay_omits_temporary_confidence_diagnostics(self):
        command = calculate_command(sensor_values(
            steering_error=0.10,
            medium_position=0.20,
            far_band_position=-0.10,
            near_fine_position=0.05,
        ))
        command["lineProcessingMs"] = 12.5
        frame = np.zeros((360, 480, 3), dtype=np.uint8)
        with (
            patch.object(camera_line_frame.cv2, "putText") as put_text,
            patch.object(camera_line_frame.cv2, "rectangle"),
            patch.object(camera_line_frame.cv2, "line"),
            patch.object(camera_line_frame.cv2, "circle"),
        ):
            camera_line_frame.draw_line_control_overlay(frame, command)
            camera_line_frame.draw_virtual_sensor_geometry(frame, command)

        rendered_texts = [
            call.args[1]
            for call in put_text.call_args_list
        ]
        preserved_prefixes = (
            "LINE ",
            "L ",
            "FAR-L ",
            "FAR-C ",
            "FAR-R ",
            "FAR BAND POS ",
            "MEDIUM-L ",
            "MEDIUM-C ",
            "MEDIUM-R ",
            "MEDIUM POS ",
            "NEAR-C ",
            "NEAR FINE POS ",
        )
        for prefix in preserved_prefixes:
            self.assertTrue(
                any(text.startswith(prefix) for text in rendered_texts),
                prefix,
            )

        removed_fragments = (
            "FAR CONF",
            "FAR THICK",
            "FAR CONS",
            "FAR TRUST",
            "MED CONF",
            "MED THICK",
            "MED CONS",
            "MED TRUST",
            "RAW POS",
            "CTRL POS",
        )
        for fragment in removed_fragments:
            self.assertFalse(
                any(fragment in text for text in rendered_texts),
                fragment,
            )

        origins_by_prefix = {
            prefix: next(
                call.args[2]
                for call in put_text.call_args_list
                if call.args[1].startswith(prefix)
            )
            for prefix in preserved_prefixes
        }
        self.assertEqual(
            origins_by_prefix["MEDIUM-L "][1],
            origins_by_prefix["MEDIUM-R "][1],
        )
        self.assertGreater(
            origins_by_prefix["MEDIUM-C "][1],
            origins_by_prefix["MEDIUM-L "][1],
        )
        self.assertGreater(
            origins_by_prefix["MEDIUM POS "][1],
            origins_by_prefix["MEDIUM-C "][1],
        )
        for text_x, text_y in origins_by_prefix.values():
            self.assertGreaterEqual(text_x, 0)
            self.assertLess(text_x, frame.shape[1])
            self.assertGreaterEqual(text_y, 0)
            self.assertLess(text_y, frame.shape[0])

    def test_line_confidence_does_not_change_control_decisions(self):
        low_confidence_sensors = sensor_values(
            steering_error=0.18,
            medium_position=0.10,
            far_band_position=-0.05,
            near_fine_position=0.08,
        )
        low_confidence_sensors.update({
            "farLineConfidence": 0.0,
            "mediumLineConfidence": 0.0,
            "farThicknessConsistency": 0.0,
            "mediumThicknessConsistency": 0.0,
        })
        high_confidence_sensors = dict(low_confidence_sensors)
        high_confidence_sensors.update({
            "farLineConfidence": 1.0,
            "mediumLineConfidence": 1.0,
            "farThicknessConsistency": 1.0,
            "mediumThicknessConsistency": 1.0,
        })

        low_result = calculate_command(low_confidence_sensors)
        high_result = calculate_command(high_confidence_sensors)

        decision_fields = (
            "left_power",
            "right_power",
            "farPosition",
            "mediumPosition",
            "steeringError",
            "finalSteering",
            "virtualState",
            "lineState",
            "controlSource",
        )
        for field in decision_fields:
            self.assertEqual(low_result[field], high_result[field])

    def test_untrusted_forward_candidates_cannot_feed_control(self):
        sensors = sensor_values(
            steering_error=0.80,
            medium_position=0.90,
            far_band_position=0.90,
        )
        sensors["farPosition"] = 0.90
        sensors["rawFarPosition"] = 0.90
        sensors["rawFarBandPosition"] = 0.90
        sensors["rawMediumPosition"] = 0.90
        sensors["farTrusted"] = False
        sensors["mediumTrusted"] = False

        result = calculate_command(sensors)

        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (0.0, 0.0),
        )
        self.assertEqual(result["rawFarPosition"], 0.90)
        self.assertEqual(result["rawFarBandPosition"], 0.90)
        self.assertEqual(result["rawMediumPosition"], 0.90)
        self.assertIsNone(result["farPosition"])
        self.assertIsNone(result["farBandPosition"])
        self.assertIsNone(result["mediumPosition"])
        self.assertEqual(
            (
                result["controlFarLeft"],
                result["controlFarCenter"],
                result["controlFarRight"],
                result["controlFarBandLeft"],
                result["controlFarBandCenter"],
                result["controlFarBandRight"],
                result["controlMediumLeft"],
                result["controlMediumCenter"],
                result["controlMediumRight"],
            ),
            (0.0,) * 9,
        )
        self.assertIsNone(result["headingAngle"])
        self.assertIsNone(result["steeringError"])
        self.assertEqual(result["controlSource"], "virtual-no-line")

    def test_untrusted_medium_candidate_does_not_start_pivot_or_spin(self):
        sensors = sensor_values(
            steering_error=0.90,
            medium_position=0.90,
            far_band_position=0.90,
            near_fine_position=0.60,
        )
        sensors["farPosition"] = 0.90
        sensors["farTrusted"] = False
        sensors["mediumTrusted"] = False
        pivot_tracker = camera_line_frame.VirtualPivotStateTracker()
        spin_tracker = camera_line_frame.VirtualMediumSpinTracker()

        result = calculate_command(
            sensors,
            pivot_state_tracker=pivot_tracker,
            medium_spin_tracker=spin_tracker,
        )

        self.assertEqual(
            pivot_tracker.state,
            camera_line_frame.PIVOT_STATE_NONE,
        )
        self.assertEqual(
            spin_tracker.state,
            camera_line_frame.PIVOT_STATE_NONE,
        )
        self.assertEqual(
            spin_tracker.critical_state,
            camera_line_frame.PIVOT_STATE_NONE,
        )
        self.assertGreaterEqual(result["left_power"], 0.0)
        self.assertGreaterEqual(result["right_power"], 0.0)

    def test_untrusted_candidate_does_not_update_memorized_direction(self):
        sensors = sensor_values(
            steering_error=None,
            medium_position=0.90,
            far_band_position=0.90,
        )
        sensors["farTrusted"] = False
        sensors["mediumTrusted"] = False
        search_tracker = camera_line_frame.VirtualLineSearchTracker()
        search_tracker.last_direction = "LEFT"

        calculate_command(
            sensors,
            gap_active=True,
            line_search_tracker=search_tracker,
        )

        self.assertEqual(search_tracker.last_direction, "LEFT")

    def test_untrusted_candidates_do_not_confirm_reorient(self):
        sensors = sensor_values(
            steering_error=None,
            medium_position=0.90,
            far_band_position=0.90,
        )
        sensors["farTrusted"] = False
        sensors["mediumTrusted"] = False
        turn_tracker = camera_line_frame.VirtualTurnStateTracker()

        for _ in range(
            camera_line_frame.VIRTUAL_REORIENT_CONFIRMATION_FRAMES
        ):
            calculate_command(sensors, tracker=turn_tracker)

        self.assertEqual(
            turn_tracker.state,
            camera_line_frame.VIRTUAL_STATE_NORMAL,
        )
        self.assertIsNone(turn_tracker.reorient_candidate)
        self.assertEqual(turn_tracker.reorient_frames, 0)

    def test_untrusted_forward_candidates_do_not_interrupt_gap(self):
        sensors = sensor_values(
            steering_error=None,
            medium_position=-0.90,
            far_band_position=-0.90,
        )
        sensors["farTrusted"] = False
        sensors["mediumTrusted"] = False

        result = calculate_command(sensors, gap_active=True)

        self.assertFalse(
            camera_line_frame.virtual_raw_line_is_visible(sensors)
        )
        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (0.75, 0.75),
        )
        self.assertEqual(result["controlSource"], "gap-forward")

    def test_untrusted_forward_geometry_does_not_block_gap_entry(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        mask[200:216, 20:460] = 255
        sensors = camera_line_frame.read_virtual_line_sensors(mask)
        forward_control_trusted = (
            sensors["farTrusted"] or sensors["mediumTrusted"]
        )

        guidance = camera_line_frame.extract_gap_geometric_guidance(
            mask,
            gap_forward_active=False,
            green_direction="NENHUMA",
            near_center_visible=False,
            forward_control_trusted=forward_control_trusted,
        )

        self.assertFalse(forward_control_trusted)
        self.assertIsNone(guidance["nearPoint"])
        self.assertIsNone(guidance["virtualNearPoint"])
        self.assertIsNone(guidance["lateralExitTarget"])
        self.assertTrue(camera_line_frame.gap_entry_is_required(
            gap_forward_active=False,
            green_direction="NENHUMA",
            recent_near_frames=1,
            near_center_visible=False,
            real_near_point=guidance["nearPoint"],
            virtual_near_point=guidance["virtualNearPoint"],
            lateral_exit_target=guidance["lateralExitTarget"],
        ))

    def test_virtual_debug_overlay_draws_composite_medium_side_wings(self):
        command = calculate_command(sensor_values(steering_error=0.10))
        frame = np.zeros((100, 200, 3), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            frame.shape
        )
        with (
            patch.object(camera_line_frame.cv2, "putText"),
            patch.object(camera_line_frame.cv2, "rectangle") as rectangle,
            patch.object(camera_line_frame.cv2, "line"),
            patch.object(camera_line_frame.cv2, "circle"),
        ):
            camera_line_frame.draw_virtual_sensor_geometry(frame, command)

        rectangles = [
            (call.args[1], call.args[2])
            for call in rectangle.call_args_list
        ]
        for sensor_name in ("left", "center", "right"):
            sensor = geometry["medium"][sensor_name]
            for region in camera_line_frame.virtual_sensor_regions(sensor):
                self.assertIn(
                    (
                        (region["x0"], region["y0"]),
                        (region["x1"], region["y1"]),
                    ),
                    rectangles,
                )

    def test_forward_heading_combines_with_fine_centering_before_mapper(self):
        forward_steering = -0.26
        sensors = sensor_values(forward_steering, -0.06, 0.0)
        sensors["nearFinePosition"] = 0.23
        sensors["headingAngle"] = -14.7

        result = calculate_command(
            sensors,
            camera_line_frame.VirtualTurnStateTracker(),
        )
        expected_steering = (
            forward_steering
            + camera_line_frame.apply_virtual_fine_center_deadband(0.23)
            * camera_line_frame.VIRTUAL_FINE_CENTER_GAIN
        )
        expected_left, expected_right = expected_normal_motor_powers(
            expected_steering
        )
        self.assertTrue(math.isclose(
            result["finalSteering"],
            expected_steering,
        ))
        self.assertTrue(math.isclose(result["left_power"], expected_left))
        self.assertTrue(math.isclose(result["right_power"], expected_right))
        self.assertEqual(result["controlSource"], "virtual")

    def test_control_overlay_is_compact_and_anchored_at_top(self):
        frame = np.zeros((120, 200, 3), dtype=np.uint8)
        command = {
            "left_power": 0.69,
            "right_power": 0.66,
            "lineProcessingMs": 4.25,
        }
        for line_state, virtual_state, expected_state in (
            ("LINE", "NORMAL", "LINE"),
            ("GAP", "NORMAL", "GAP"),
            ("GREEN", "NORMAL", "GREEN"),
            ("LINE", "REORIENT_LEFT", "REORIENT"),
            ("LINE", "REORIENT_RIGHT", "REORIENT"),
        ):
            with self.subTest(
                line_state=line_state,
                virtual_state=virtual_state,
            ):
                command["lineState"] = line_state
                command["virtualState"] = virtual_state
                with patch.object(
                    camera_line_frame.cv2,
                    "putText",
                ) as put_text:
                    camera_line_frame.draw_line_control_overlay(frame, command)

                texts = [call.args[1] for call in put_text.call_args_list]
                positions = [call.args[2] for call in put_text.call_args_list]
                self.assertEqual(texts, [
                    f"{expected_state} 4.2ms",
                    "L 0.69  R 0.66",
                ])
                self.assertEqual(positions, [(8, 22), (8, 44)])

    def test_virtual_geometry_preserves_far_near_and_composes_medium(self):
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            (101, 203)
        )
        self.assertEqual(
            geometry["far"]["left"]["y0"],
            round(101 * camera_line_frame.VIRTUAL_FAR_Y0),
        )
        self.assertEqual(
            geometry["far"]["left"]["y1"],
            round(101 * camera_line_frame.VIRTUAL_FAR_Y1),
        )
        self.assertEqual(
            geometry["near"]["center"]["y0"],
            round(101 * camera_line_frame.VIRTUAL_NEAR_Y0),
        )
        self.assertEqual(
            geometry["near"]["center"]["y1"],
            round(101 * camera_line_frame.VIRTUAL_NEAR_Y1),
        )
        self.assertNotIn("left", geometry["near"])
        self.assertNotIn("right", geometry["near"])
        self.assertEqual(geometry["farBand"]["left"]["y1"], round(101 * 0.27))
        self.assertEqual(
            geometry["medium"]["left"]["y0"],
            round(101 * camera_line_frame.VIRTUAL_MEDIUM_Y0),
        )
        self.assertEqual(
            geometry["medium"]["center"]["y0"],
            geometry["medium"]["left"]["y0"],
        )
        self.assertEqual(
            geometry["medium"]["right"]["y0"],
            geometry["medium"]["left"]["y0"],
        )
        self.assertEqual(
            geometry["medium"]["center"]["y1"],
            round(101 * camera_line_frame.VIRTUAL_MEDIUM_Y1),
        )
        self.assertEqual(geometry["medium"]["left"]["x0"], round(203 * 0.04))
        self.assertEqual(geometry["medium"]["left"]["x1"], round(203 * 0.43))
        self.assertEqual(geometry["medium"]["center"]["x0"], round(203 * 0.43))
        self.assertEqual(geometry["medium"]["center"]["x1"], round(203 * 0.57))
        self.assertEqual(geometry["medium"]["right"]["x0"], round(203 * 0.57))
        self.assertEqual(geometry["medium"]["right"]["x1"], round(203 * 0.96))
        self.assertEqual(
            geometry["medium"]["left"]["x1"],
            geometry["medium"]["center"]["x0"],
        )
        self.assertEqual(
            geometry["medium"]["center"]["x1"],
            geometry["medium"]["right"]["x0"],
        )
        self.assertEqual(
            geometry["medium"]["left"]["y1"],
            geometry["medium"]["center"]["y1"],
        )
        self.assertEqual(
            geometry["medium"]["right"]["y1"],
            geometry["medium"]["center"]["y1"],
        )

        left_wing = geometry["medium"]["left"]["regions"][1]
        right_wing = geometry["medium"]["right"]["regions"][1]
        self.assertEqual(
            left_wing["y0"],
            geometry["medium"]["left"]["y1"],
        )
        self.assertEqual(
            right_wing["y0"],
            geometry["medium"]["right"]["y1"],
        )
        self.assertEqual(
            left_wing["y1"],
            round(101 * camera_line_frame.VIRTUAL_MEDIUM_WING_Y1),
        )
        self.assertEqual(right_wing["y1"], left_wing["y1"])
        self.assertEqual(
            left_wing["x1"],
            geometry["near"]["center"]["x0"],
        )
        self.assertEqual(left_wing["x0"], round(203 * 0.04))
        self.assertEqual(
            right_wing["x0"],
            geometry["near"]["center"]["x1"],
        )
        self.assertEqual(right_wing["x1"], round(203 * 0.96))

    def test_composite_medium_reading_has_one_normalized_occupancy(self):
        mask = np.zeros((100, 200), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(mask.shape)
        medium_left = geometry["medium"]["left"]
        upper, wing = camera_line_frame.virtual_sensor_regions(medium_left)

        mask[
            upper["y0"]:upper["y1"],
            upper["x0"]:upper["x1"],
        ] = 255
        upper_area = (
            (upper["y1"] - upper["y0"])
            * (upper["x1"] - upper["x0"])
        )
        wing_area = (
            (wing["y1"] - wing["y0"])
            * (wing["x1"] - wing["x0"])
        )
        expected_upper_occupancy = upper_area / (upper_area + wing_area)
        self.assertTrue(math.isclose(
            camera_line_frame.read_virtual_sensor(mask, medium_left),
            expected_upper_occupancy,
        ))

        mask[
            wing["y0"]:wing["y1"],
            wing["x0"]:wing["x1"],
        ] = 255
        self.assertTrue(math.isclose(
            camera_line_frame.read_virtual_sensor(mask, medium_left),
            1.0,
        ))

    def test_sensor_outputs_match_direct_near_center_reference(self):
        random_generator = np.random.default_rng(2026)
        mask = (
            random_generator.random((101, 203)) > 0.72
        ).astype(np.uint8) * 255
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(mask.shape)

        # O GREEN possui testes próprios para os pontos físicos. Esta
        # referência preserva a equivalência do caminho virtual NORMAL.
        for green_direction in ("NENHUMA",):
            actual = camera_line_frame.read_virtual_line_sensors(
                mask,
                green_direction,
            )
            far_left = camera_line_frame.read_virtual_sensor(
                mask, geometry["far"]["left"]
            )
            far_center = camera_line_frame.read_virtual_sensor(
                mask, geometry["far"]["center"]
            )
            far_right = camera_line_frame.read_virtual_sensor(
                mask, geometry["far"]["right"]
            )
            medium_left = camera_line_frame.read_virtual_sensor(
                mask, geometry["medium"]["left"]
            )
            medium_center = camera_line_frame.read_virtual_sensor(
                mask, geometry["medium"]["center"]
            )
            medium_right = camera_line_frame.read_virtual_sensor(
                mask, geometry["medium"]["right"]
            )
            near_center = camera_line_frame.read_virtual_sensor(
                mask, geometry["near"]["center"]
            )
            raw_far = (far_left, far_center, far_right)
            raw_medium = (medium_left, medium_center, medium_right)
            raw_far_position = (
                camera_line_frame.calculate_virtual_row_position(*raw_far)
            )
            raw_medium_position = (
                camera_line_frame.calculate_virtual_row_position(*raw_medium)
            )
            if green_direction == "ESQUERDA":
                far_center = 0.0
                far_right = 0.0
                medium_center = 0.0
                medium_right = 0.0
            elif green_direction == "DIREITA":
                far_left = 0.0
                far_center = 0.0
                medium_left = 0.0
                medium_center = 0.0

            far_position = camera_line_frame.calculate_virtual_row_position(
                far_left, far_center, far_right
            )
            near_center_visible = camera_line_frame.virtual_sensor_is_active(
                near_center
            )
            near_fine_position = None
            if near_center_visible:
                near_fine_position = (
                    camera_line_frame.calculate_virtual_near_fine_position(
                        mask,
                        geometry["near"],
                    )
                )
            medium_position = camera_line_frame.calculate_virtual_row_position(
                medium_left, medium_center, medium_right
            )
            trusted_far_position = (
                far_position if actual["farTrusted"] else None
            )
            trusted_medium_position = (
                medium_position if actual["mediumTrusted"] else None
            )
            heading_angle = camera_line_frame.calculate_virtual_heading_angle(
                trusted_far_position,
                near_fine_position,
                geometry,
                medium_position=trusted_medium_position,
            )
            steering_error = camera_line_frame.calculate_virtual_steering_error(
                near_fine_position is not None,
                heading_angle,
                fallback_medium_position=(
                    trusted_medium_position
                    if near_fine_position is None
                    else None
                ),
                fallback_far_position=(
                    trusted_far_position
                    if near_fine_position is None
                    and green_direction == "NENHUMA"
                    else None
                ),
            )
            expected = {
                "farLeft": raw_far[0],
                "farCenter": raw_far[1],
                "farRight": raw_far[2],
                "controlFarLeft": far_left if actual["farTrusted"] else 0.0,
                "controlFarCenter": (
                    far_center if actual["farTrusted"] else 0.0
                ),
                "controlFarRight": (
                    far_right if actual["farTrusted"] else 0.0
                ),
                "rawFarPosition": raw_far_position,
                "farPosition": trusted_far_position,
                "mediumLeft": raw_medium[0],
                "mediumCenter": raw_medium[1],
                "mediumRight": raw_medium[2],
                "controlMediumLeft": (
                    medium_left if actual["mediumTrusted"] else 0.0
                ),
                "controlMediumCenter": (
                    medium_center if actual["mediumTrusted"] else 0.0
                ),
                "controlMediumRight": (
                    medium_right if actual["mediumTrusted"] else 0.0
                ),
                "rawMediumPosition": raw_medium_position,
                "mediumPosition": trusted_medium_position,
                "nearCenter": near_center,
                "nearFinePosition": near_fine_position,
                "headingAngle": heading_angle,
                "steeringError": steering_error,
            }
            for field_name, expected_value in expected.items():
                actual_value = actual[field_name]
                if expected_value is None:
                    self.assertIsNone(actual_value, field_name)
                else:
                    self.assertTrue(
                        math.isclose(actual_value, expected_value),
                        field_name,
                    )

    def test_green_with_only_medium_does_not_replace_missing_fusion(self):
        mask = np.zeros((100, 200), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(mask.shape)
        for side in ("left", "right"):
            for region in camera_line_frame.virtual_sensor_regions(
                geometry["medium"][side]
            ):
                mask[
                    region["y0"]:region["y1"],
                    region["x0"]:region["x1"],
                ] = 255

        for green_direction, expected_sign in (
            ("ESQUERDA", -1.0),
            ("DIREITA", 1.0),
        ):
            with self.subTest(green_direction=green_direction):
                sensors = camera_line_frame.read_virtual_line_sensors(
                    mask,
                    green_direction,
                )
                result = calculate_command(
                    sensors,
                    green_direction=green_direction,
                )

                self.assertGreater(
                    expected_sign * sensors["mediumPosition"],
                    0.0,
                )
                self.assertLess(abs(sensors["mediumPosition"]), 1.0)
                self.assertIsNone(sensors["farPosition"])
                self.assertIsNone(sensors["headingAngle"])
                self.assertEqual(result["finalSteering"], expected_sign)
                self.assertEqual(
                    result["controlSource"],
                    "green-entry-pivot",
                )
                self.assertEqual(
                    (result["left_power"], result["right_power"]),
                    (
                        camera_line_frame.PIVOT_INNER_POWER
                        if expected_sign < 0.0
                        else camera_line_frame.PIVOT_OUTER_POWER,
                        camera_line_frame.PIVOT_OUTER_POWER
                        if expected_sign < 0.0
                        else camera_line_frame.PIVOT_INNER_POWER,
                    ),
                )

    def test_green_trust_is_measured_only_on_selected_right_branch(self):
        mask = np.zeros((100, 200), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(mask.shape)
        for row_name in ("far", "medium"):
            for region in camera_line_frame.virtual_sensor_regions(
                geometry[row_name]["right"]
            ):
                mask[
                    region["y0"]:region["y1"],
                    region["x0"]:region["x1"],
                ] = 255

        measured_geometries = []
        original_measurement = (
            camera_line_frame.measure_virtual_row_line_confidence
        )

        def measure_selected_branch(processed_mask, row_geometry):
            measured_geometries.append(tuple(row_geometry))
            return original_measurement(processed_mask, row_geometry)

        with patch.object(
            camera_line_frame,
            "measure_virtual_row_line_confidence",
            side_effect=measure_selected_branch,
        ):
            sensors = camera_line_frame.read_virtual_line_sensors(
                mask,
                "DIREITA",
            )

        self.assertEqual(measured_geometries, [("right",), ("right",)])
        self.assertTrue(sensors["farTrusted"])
        self.assertTrue(sensors["mediumTrusted"])
        self.assertGreater(sensors["farPosition"], 0.0)
        self.assertGreater(sensors["mediumPosition"], 0.0)
        self.assertLess(sensors["farPosition"], 1.0)
        self.assertLess(sensors["mediumPosition"], 1.0)
        self.assertIsNotNone(sensors["greenFarPoint"])
        self.assertIsNotNone(sensors["greenMediumPoint"])

    def test_green_left_expands_trust_and_positions_after_curve_starts(self):
        mask = np.zeros((100, 200), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(mask.shape)
        for row_name in ("far", "medium"):
            for side in ("left", "center", "right"):
                for region in camera_line_frame.virtual_sensor_regions(
                    geometry[row_name][side]
                ):
                    mask[
                        region["y0"]:region["y1"],
                        region["x0"]:region["x1"],
                    ] = 255

        measured_geometries = []
        trusted_measurement = {
            "lineConfidence": 1.0,
            "robustThicknessPx": 30.0,
            "thicknessScore": 1.0,
            "thicknessConsistency": 1.0,
            "continuityScore": 1.0,
            "areaScore": 1.0,
            "componentCenterX": 60.0,
        }

        def measure_selected_branch(_processed_mask, row_geometry):
            measured_geometries.append(tuple(row_geometry))
            return trusted_measurement

        with patch.object(
            camera_line_frame,
            "measure_virtual_row_line_confidence",
            side_effect=measure_selected_branch,
        ):
            before = camera_line_frame.read_virtual_line_sensors(
                mask,
                "ESQUERDA",
                curva_verde_iniciada=False,
            )
            after = camera_line_frame.read_virtual_line_sensors(
                mask,
                "ESQUERDA",
                curva_verde_iniciada=True,
            )

        self.assertEqual(
            measured_geometries,
            [("left",), ("left",), ("left", "center"), ("left", "center")],
        )
        self.assertEqual(
            (
                before["controlFarLeft"],
                before["controlFarCenter"],
                before["controlFarRight"],
            ),
            (1.0, 0.0, 0.0),
        )
        self.assertEqual(
            (
                after["controlFarLeft"],
                after["controlFarCenter"],
                after["controlFarRight"],
            ),
            (1.0, 1.0, 0.0),
        )
        self.assertEqual(
            (
                before["controlMediumLeft"],
                before["controlMediumCenter"],
                before["controlMediumRight"],
            ),
            (1.0, 0.0, 0.0),
        )
        self.assertEqual(
            (
                after["controlMediumLeft"],
                after["controlMediumCenter"],
                after["controlMediumRight"],
            ),
            (1.0, 1.0, 0.0),
        )
        self.assertEqual(before["greenFarPoint"][0], 60.0)
        self.assertEqual(before["greenMediumPoint"][0], 60.0)
        self.assertEqual(after["greenFarPoint"][0], 60.0)
        self.assertEqual(after["greenMediumPoint"][0], 60.0)

    def test_green_right_expands_trust_and_positions_after_curve_starts(self):
        mask = np.zeros((100, 200), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(mask.shape)
        for row_name in ("far", "medium"):
            for side in ("left", "center", "right"):
                for region in camera_line_frame.virtual_sensor_regions(
                    geometry[row_name][side]
                ):
                    mask[
                        region["y0"]:region["y1"],
                        region["x0"]:region["x1"],
                    ] = 255

        measured_geometries = []
        trusted_measurement = {
            "lineConfidence": 1.0,
            "robustThicknessPx": 30.0,
            "thicknessScore": 1.0,
            "thicknessConsistency": 1.0,
            "continuityScore": 1.0,
            "areaScore": 1.0,
            "componentCenterX": 140.0,
        }

        def measure_selected_branch(_processed_mask, row_geometry):
            measured_geometries.append(tuple(row_geometry))
            return trusted_measurement

        with patch.object(
            camera_line_frame,
            "measure_virtual_row_line_confidence",
            side_effect=measure_selected_branch,
        ):
            before = camera_line_frame.read_virtual_line_sensors(
                mask,
                "DIREITA",
                curva_verde_iniciada=False,
            )
            after = camera_line_frame.read_virtual_line_sensors(
                mask,
                "DIREITA",
                curva_verde_iniciada=True,
            )

        self.assertEqual(
            measured_geometries,
            [("right",), ("right",), ("center", "right"), ("center", "right")],
        )
        self.assertEqual(
            (
                before["controlFarLeft"],
                before["controlFarCenter"],
                before["controlFarRight"],
            ),
            (0.0, 0.0, 1.0),
        )
        self.assertEqual(
            (
                after["controlFarLeft"],
                after["controlFarCenter"],
                after["controlFarRight"],
            ),
            (0.0, 1.0, 1.0),
        )
        self.assertEqual(
            (
                before["controlMediumLeft"],
                before["controlMediumCenter"],
                before["controlMediumRight"],
            ),
            (0.0, 0.0, 1.0),
        )
        self.assertEqual(
            (
                after["controlMediumLeft"],
                after["controlMediumCenter"],
                after["controlMediumRight"],
            ),
            (0.0, 1.0, 1.0),
        )
        self.assertEqual(before["greenFarPoint"][0], 140.0)
        self.assertEqual(before["greenMediumPoint"][0], 140.0)
        self.assertEqual(after["greenFarPoint"][0], 140.0)
        self.assertEqual(after["greenMediumPoint"][0], 140.0)

    def test_green_left_uses_real_component_x_and_heading(self):
        strong_curve = self.read_directional_green_stripe(
            "ESQUERDA",
            far_center_x=70,
            medium_center_x=150,
        )
        mild_curve = self.read_directional_green_stripe(
            "ESQUERDA",
            far_center_x=125,
            medium_center_x=155,
        )

        self.assertTrue(strong_curve["farTrusted"])
        self.assertTrue(strong_curve["mediumTrusted"])
        self.assertLess(strong_curve["farPosition"], 0.0)
        self.assertGreater(strong_curve["farPosition"], -1.0)
        self.assertTrue(math.isclose(
            strong_curve["greenFarPoint"][0],
            69.5,
        ))
        self.assertTrue(math.isclose(
            strong_curve["greenMediumPoint"][0],
            149.5,
        ))
        self.assertNotEqual(
            strong_curve["greenFarPoint"][0],
            mild_curve["greenFarPoint"][0],
        )
        self.assertLess(strong_curve["headingAngle"], 0.0)
        self.assertLess(mild_curve["headingAngle"], 0.0)
        self.assertGreater(
            abs(strong_curve["headingAngle"]),
            abs(mild_curve["headingAngle"]),
        )

    def test_green_right_mirrors_real_component_x_and_heading(self):
        left = self.read_directional_green_stripe(
            "ESQUERDA",
            far_center_x=70,
            medium_center_x=150,
        )
        right = self.read_directional_green_stripe(
            "DIREITA",
            far_center_x=410,
            medium_center_x=330,
        )

        self.assertTrue(right["farTrusted"])
        self.assertTrue(right["mediumTrusted"])
        self.assertGreater(right["farPosition"], 0.0)
        self.assertLess(right["farPosition"], 1.0)
        self.assertTrue(math.isclose(
            right["greenFarPoint"][0],
            409.5,
        ))
        self.assertTrue(math.isclose(
            right["greenMediumPoint"][0],
            329.5,
        ))
        self.assertGreater(right["headingAngle"], 0.0)
        self.assertTrue(math.isclose(
            right["headingAngle"],
            -left["headingAngle"],
            rel_tol=0.02,
        ))

    def test_green_heading_uses_real_points_in_required_priority(self):
        near_point = (240.0, 330.0)
        medium_point = (200.0, 220.0)
        far_point = (160.0, 100.0)

        with_near_and_far = camera_line_frame.calculate_green_heading_angle(
            near_point,
            medium_point,
            far_point,
        )
        with_near_and_medium = camera_line_frame.calculate_green_heading_angle(
            near_point,
            medium_point,
            None,
        )
        without_near = camera_line_frame.calculate_green_heading_angle(
            None,
            medium_point,
            far_point,
        )

        self.assertTrue(math.isclose(
            with_near_and_far,
            math.degrees(math.atan2(160.0 - 240.0, 330.0 - 100.0)),
        ))
        self.assertTrue(math.isclose(
            with_near_and_medium,
            math.degrees(math.atan2(200.0 - 240.0, 330.0 - 220.0)),
        ))
        self.assertTrue(math.isclose(
            without_near,
            math.degrees(math.atan2(160.0 - 200.0, 220.0 - 100.0)),
        ))

    def test_green_direction_has_priority_without_fusion(self):
        for green_direction, steering_error in (
            ("ESQUERDA", -0.55),
            ("DIREITA", 0.55),
        ):
            with self.subTest(green_direction=green_direction):
                sensors = sensor_values(
                    steering_error,
                    medium_position=(
                        -0.50 if steering_error < 0.0 else 0.50
                    ),
                    far_band_position=None,
                )
                sensors["headingAngle"] = (
                    -30.0 if steering_error < 0.0 else 30.0
                )
                result = calculate_command(
                    sensors,
                    green_direction=green_direction,
                )

                self.assertEqual(
                    result["controlSource"],
                    "green-entry-pivot",
                )
                expected_steering = (
                    -1.0 if green_direction == "ESQUERDA" else 1.0
                )
                self.assertEqual(result["finalSteering"], expected_steering)
                self.assertFalse(result["fusionControlActive"])
                self.assertEqual(
                    (result["left_power"], result["right_power"]),
                    (
                        camera_line_frame.PIVOT_INNER_POWER
                        if green_direction == "ESQUERDA"
                        else camera_line_frame.PIVOT_OUTER_POWER,
                        camera_line_frame.PIVOT_OUTER_POWER
                        if green_direction == "ESQUERDA"
                        else camera_line_frame.PIVOT_INNER_POWER,
                    ),
                )

    def test_green_without_trusted_branch_starts_confirmed_turn(self):
        for green_direction in ("ESQUERDA", "DIREITA"):
            with self.subTest(green_direction=green_direction):
                sensors = sensor_values(None, None, None)
                sensors["farTrusted"] = False
                sensors["mediumTrusted"] = False
                result = calculate_command(
                    sensors,
                    green_direction=green_direction,
                )

                self.assertEqual(
                    result["controlSource"],
                    "green-entry-pivot",
                )
                self.assertEqual(result["lineState"], "GREEN")
                expected_steering = (
                    -1.0 if green_direction == "ESQUERDA" else 1.0
                )
                self.assertEqual(result["finalSteering"], expected_steering)
                self.assertEqual(
                    (result["left_power"], result["right_power"]),
                    (
                        camera_line_frame.PIVOT_INNER_POWER
                        if green_direction == "ESQUERDA"
                        else camera_line_frame.PIVOT_OUTER_POWER,
                        camera_line_frame.PIVOT_OUTER_POWER
                        if green_direction == "ESQUERDA"
                        else camera_line_frame.PIVOT_INNER_POWER,
                    ),
                )

    def test_green_with_only_far_branch_starts_confirmed_turn(self):
        for green_direction, far_position in (
            ("ESQUERDA", -0.70),
            ("DIREITA", 0.70),
        ):
            with self.subTest(green_direction=green_direction):
                sensors = sensor_values(None, None, None)
                sensors["farPosition"] = far_position
                sensors["mediumTrusted"] = False
                result = calculate_command(
                    sensors,
                    green_direction=green_direction,
                )

                self.assertEqual(
                    result["controlSource"],
                    "green-entry-pivot",
                )
                expected_steering = (
                    -1.0 if green_direction == "ESQUERDA" else 1.0
                )
                self.assertEqual(result["finalSteering"], expected_steering)
                self.assertEqual(
                    (result["left_power"], result["right_power"]),
                    (
                        camera_line_frame.PIVOT_INNER_POWER
                        if green_direction == "ESQUERDA"
                        else camera_line_frame.PIVOT_OUTER_POWER,
                        camera_line_frame.PIVOT_OUTER_POWER
                        if green_direction == "ESQUERDA"
                        else camera_line_frame.PIVOT_INNER_POWER,
                    ),
                )


class VirtualRecoveryTests(unittest.TestCase):
    def test_active_near_center_skips_gap_geometry(self):
        mask = np.zeros((100, 100), dtype=np.uint8)
        with patch.object(
            camera_line_frame,
            "extract_geometric_line_path",
        ) as extract_path:
            guidance = camera_line_frame.extract_gap_geometric_guidance(
                mask,
                gap_forward_active=False,
                green_direction="NENHUMA",
                near_center_visible=True,
            )

        extract_path.assert_not_called()
        self.assertEqual(guidance, {
            "nearPoint": None,
            "farHeadingDeg": None,
            "virtualNearPoint": None,
            "lateralExitTarget": None,
            "processingMs": 0.0,
        })

    def test_active_green_without_gap_skips_gap_geometry(self):
        mask = np.zeros((100, 100), dtype=np.uint8)
        with patch.object(
            camera_line_frame,
            "extract_geometric_line_path",
        ) as extract_path:
            guidance = camera_line_frame.extract_gap_geometric_guidance(
                mask,
                gap_forward_active=False,
                green_direction="ESQUERDA",
                near_center_visible=False,
            )

        extract_path.assert_not_called()
        self.assertEqual(guidance["processingMs"], 0.0)

    def test_gap_geometry_result_is_preserved_when_required(self):
        mask = np.zeros((100, 100), dtype=np.uint8)
        expected_guidance = {
            "nearPoint": (49.0, 83.0),
            "farHeadingDeg": -12.5,
            "virtualNearPoint": None,
            "lateralExitTarget": (0.0, 45.0),
        }
        required_cases = (
            (False, "NENHUMA", False),
            (True, "DIREITA", True),
        )
        for gap_active, green_direction, near_center_visible in required_cases:
            with self.subTest(
                gap_active=gap_active,
                green_direction=green_direction,
                near_center_visible=near_center_visible,
            ):
                with patch.object(
                    camera_line_frame,
                    "extract_geometric_line_path",
                    return_value=expected_guidance.copy(),
                ) as extract_path:
                    guidance = (
                        camera_line_frame.extract_gap_geometric_guidance(
                            mask,
                            gap_forward_active=gap_active,
                            green_direction=green_direction,
                            near_center_visible=near_center_visible,
                        )
                    )

                extract_path.assert_called_once_with(mask)
                for field_name, expected_value in expected_guidance.items():
                    self.assertEqual(guidance[field_name], expected_value)
                self.assertGreaterEqual(guidance["processingMs"], 0.0)

    def test_gap_geometry_preserves_real_near_path(self):
        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[:, 47:53] = 255

        guidance = camera_line_frame.extract_geometric_line_path(mask)

        self.assertIsNotNone(guidance["nearPoint"])
        self.assertIsNone(guidance["virtualNearPoint"])
        self.assertLess(abs(guidance["farHeadingDeg"]), 1.0)

    def test_gap_geometry_projects_reacquired_path_to_near(self):
        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[:76, 47:53] = 255

        guidance = camera_line_frame.extract_geometric_line_path(mask)

        self.assertIsNone(guidance["nearPoint"])
        self.assertIsNotNone(guidance["virtualNearPoint"])
        self.assertLess(abs(guidance["farHeadingDeg"]), 1.0)

    def test_valid_near_center_then_lost_near_requests_gap(self):
        recent_near_frames = camera_line_frame.update_gap_recent_near_frames(
            0,
            near_center_visible=True,
        )
        self.assertEqual(
            recent_near_frames,
            camera_line_frame.GAP_NEAR_HISTORY_FRAMES,
        )

        recent_near_frames = camera_line_frame.update_gap_recent_near_frames(
            recent_near_frames,
            near_center_visible=False,
        )

        self.assertEqual(
            recent_near_frames,
            camera_line_frame.GAP_NEAR_HISTORY_FRAMES - 1,
        )
        self.assertTrue(camera_line_frame.gap_entry_is_required(
            gap_forward_active=False,
            green_direction="NENHUMA",
            recent_near_frames=recent_near_frames,
            near_center_visible=False,
            real_near_point=None,
            virtual_near_point=None,
            lateral_exit_target=None,
        ))

    def test_near_connected_fusion_contour_blocks_false_gap_entry(self):
        connected_curve = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(
            connected_curve,
            (240, 359),
            (40, 170),
            255,
            22,
        )
        connected_fusion = camera_line_frame.extract_fusion_style_line(
            connected_curve
        )
        fusion_near_connected = (
            connected_fusion["valid"]
            and connected_fusion["selection"] == "nearCenter"
        )

        self.assertTrue(fusion_near_connected)
        self.assertFalse(camera_line_frame.gap_entry_is_required(
            gap_forward_active=False,
            green_direction="NENHUMA",
            recent_near_frames=1,
            near_center_visible=False,
            real_near_point=None,
            virtual_near_point=None,
            lateral_exit_target=None,
            fusion_near_connected=fusion_near_connected,
        ))

        distant_segment = np.zeros((360, 480), dtype=np.uint8)
        camera_line_frame.cv2.line(
            distant_segment,
            (220, 220),
            (120, 120),
            255,
            22,
        )
        distant_fusion = camera_line_frame.extract_fusion_style_line(
            distant_segment
        )
        distant_fusion_near_connected = (
            distant_fusion["valid"]
            and distant_fusion["selection"] == "nearCenter"
        )

        self.assertFalse(distant_fusion_near_connected)
        self.assertTrue(camera_line_frame.gap_entry_is_required(
            gap_forward_active=False,
            green_direction="NENHUMA",
            recent_near_frames=1,
            near_center_visible=False,
            real_near_point=None,
            virtual_near_point=None,
            lateral_exit_target=None,
            fusion_near_connected=distant_fusion_near_connected,
        ))

    def test_recent_near_loss_with_far_left_guides_forward(self):
        recent_near_frames = camera_line_frame.update_gap_recent_near_frames(
            0,
            near_center_visible=True,
        )
        recent_near_frames = camera_line_frame.update_gap_recent_near_frames(
            recent_near_frames,
            near_center_visible=False,
        )
        gap_active = camera_line_frame.gap_entry_is_required(
            False,
            "NENHUMA",
            recent_near_frames,
            False,
            None,
            None,
            None,
        )
        sensors = sensor_values(None, None, -0.80)
        sensors["farPosition"] = -0.80
        result = calculate_command(sensors, gap_active=gap_active)

        self.assertEqual(result["lineState"], "GAP")
        self.assertEqual(result["controlSource"], "virtual-gap-far")
        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (camera_line_frame.NORMAL_INNER_MIN_POWER,
             camera_line_frame.NORMAL_MAX_POWER),
        )

    def test_recent_near_loss_with_far_right_guides_forward(self):
        recent_near_frames = camera_line_frame.update_gap_recent_near_frames(
            0,
            near_center_visible=True,
        )
        recent_near_frames = camera_line_frame.update_gap_recent_near_frames(
            recent_near_frames,
            near_center_visible=False,
        )
        gap_active = camera_line_frame.gap_entry_is_required(
            False,
            "NENHUMA",
            recent_near_frames,
            False,
            None,
            None,
            None,
        )
        sensors = sensor_values(None, None, 0.80)
        sensors["farPosition"] = 0.80
        result = calculate_command(sensors, gap_active=gap_active)

        self.assertEqual(result["lineState"], "GAP")
        self.assertEqual(result["controlSource"], "virtual-gap-far")
        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (camera_line_frame.NORMAL_MAX_POWER,
             camera_line_frame.NORMAL_INNER_MIN_POWER),
        )

    def test_gap_near_history_expires_without_being_renewed(self):
        recent_near_frames = camera_line_frame.update_gap_recent_near_frames(
            0,
            near_center_visible=True,
        )
        observed_history = []
        for _ in range(camera_line_frame.GAP_NEAR_HISTORY_FRAMES):
            recent_near_frames = camera_line_frame.update_gap_recent_near_frames(
                recent_near_frames,
                near_center_visible=False,
            )
            observed_history.append(recent_near_frames)

        self.assertEqual(observed_history, [2, 1, 0])
        self.assertFalse(camera_line_frame.gap_entry_is_required(
            gap_forward_active=False,
            green_direction="NENHUMA",
            recent_near_frames=recent_near_frames,
            near_center_visible=False,
            real_near_point=None,
            virtual_near_point=None,
            lateral_exit_target=None,
        ))

        sensors = sensor_values(None, None, -0.80)
        result = calculate_command(sensors, gap_active=False)
        self.assertEqual(result["lineState"], "LINE")
        self.assertEqual(result["controlSource"], "virtual-no-line")
        self.assertEqual((result["left_power"], result["right_power"]), (0.0, 0.0))

    def test_green_keeps_priority_over_recent_near_loss(self):
        recent_near_frames = camera_line_frame.GAP_NEAR_HISTORY_FRAMES - 1
        gap_active = camera_line_frame.gap_entry_is_required(
            gap_forward_active=False,
            green_direction="DIREITA",
            recent_near_frames=recent_near_frames,
            near_center_visible=False,
            real_near_point=None,
            virtual_near_point=None,
            lateral_exit_target=None,
        )
        self.assertFalse(gap_active)

        sensors = sensor_values(None, None, -0.80)
        result = calculate_command(
            sensors,
            green_direction="DIREITA",
            gap_active=gap_active,
        )
        self.assertEqual(result["lineState"], "GREEN")
        self.assertEqual(
            result["controlSource"],
            "green-entry-pivot",
        )
        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (
                camera_line_frame.PIVOT_OUTER_POWER,
                camera_line_frame.PIVOT_INNER_POWER,
            ),
        )

    def enter_reorient(self, direction):
        sign = -1.0 if direction == "LEFT" else 1.0
        tracker = camera_line_frame.VirtualTurnStateTracker()
        sensors = sensor_values(
            steering_error=None,
            medium_position=sign * 0.80,
            far_band_position=sign * 0.80,
        )
        result = None
        for _ in range(camera_line_frame.VIRTUAL_REORIENT_CONFIRMATION_FRAMES):
            result = calculate_command(sensors, tracker)
        return tracker, result

    def test_valid_normal_steering_never_enters_reorient(self):
        tracker = camera_line_frame.VirtualTurnStateTracker()
        sensors = sensor_values(0.25, 0.90, 0.90)
        for _ in range(camera_line_frame.VIRTUAL_REORIENT_CONFIRMATION_FRAMES + 2):
            result = calculate_command(sensors, tracker)
        self.assertEqual(tracker.state, camera_line_frame.VIRTUAL_STATE_NORMAL)
        self.assertEqual(result["finalSteering"], 0.25)

    def test_valid_normal_steering_never_receives_medium_scan(self):
        result = calculate_command(sensor_values(0.12, -0.90, -0.90))
        self.assertEqual(result["finalSteering"], 0.12)
        self.assertEqual(result["controlSource"], "virtual")

    def test_invalid_steering_uses_left_medium_scan(self):
        tracker = camera_line_frame.VirtualTurnStateTracker()
        result = calculate_command(sensor_values(None, -0.80, None), tracker)
        self.assertIsNone(result["finalSteering"])
        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (0.0, camera_line_frame.NORMAL_BASE_POWER),
        )
        self.assertEqual(result["controlSource"], "virtual-medium-scan")

    def test_invalid_steering_uses_right_medium_scan(self):
        tracker = camera_line_frame.VirtualTurnStateTracker()
        result = calculate_command(sensor_values(None, 0.80, None), tracker)
        self.assertIsNone(result["finalSteering"])
        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (camera_line_frame.NORMAL_BASE_POWER, 0.0),
        )
        self.assertEqual(result["controlSource"], "virtual-medium-scan")

    def test_medium_scan_disappears_when_normal_steering_returns(self):
        tracker = camera_line_frame.VirtualTurnStateTracker()
        calculate_command(sensor_values(None, 0.80, None), tracker)
        result = calculate_command(sensor_values(0.18, -0.80, -0.80), tracker)
        self.assertEqual(result["finalSteering"], 0.18)
        self.assertEqual(result["controlSource"], "virtual")

    def test_medium_scan_stops_after_three_frames(self):
        tracker = camera_line_frame.VirtualTurnStateTracker()
        sensors = sensor_values(None, 0.80, None)
        for _ in range(camera_line_frame.VIRTUAL_MEDIUM_SCAN_MAX_FRAMES):
            result = calculate_command(sensors, tracker)
            self.assertEqual(result["controlSource"], "virtual-medium-scan")
            self.assertEqual(
                (result["left_power"], result["right_power"]),
                (camera_line_frame.NORMAL_BASE_POWER, 0.0),
            )

        result = calculate_command(sensors, tracker)
        self.assertEqual(result["controlSource"], "virtual-no-line")
        self.assertEqual((result["left_power"], result["right_power"]), (0.0, 0.0))

    def test_matching_left_recovery_enters_reorient(self):
        tracker, result = self.enter_reorient("LEFT")
        self.assertEqual(tracker.state, camera_line_frame.VIRTUAL_STATE_REORIENT_LEFT)
        self.assertEqual(result["finalSteering"], -1.0)
        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (
                camera_line_frame.PIVOT_INNER_POWER,
                camera_line_frame.PIVOT_OUTER_POWER,
            ),
        )

    def test_matching_right_recovery_enters_reorient(self):
        tracker, result = self.enter_reorient("RIGHT")
        self.assertEqual(tracker.state, camera_line_frame.VIRTUAL_STATE_REORIENT_RIGHT)
        self.assertEqual(result["finalSteering"], 1.0)
        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (
                camera_line_frame.PIVOT_OUTER_POWER,
                camera_line_frame.PIVOT_INNER_POWER,
            ),
        )

    def test_medium_far_disagreement_never_enters_reorient(self):
        tracker = camera_line_frame.VirtualTurnStateTracker()
        sensors = sensor_values(None, 0.80, -0.80)
        for _ in range(camera_line_frame.VIRTUAL_REORIENT_CONFIRMATION_FRAMES + 2):
            result = calculate_command(sensors, tracker)
        self.assertEqual(tracker.state, camera_line_frame.VIRTUAL_STATE_NORMAL)
        self.assertEqual(result["controlSource"], "virtual-no-line")

    def test_only_far_band_valid_does_not_move(self):
        result = calculate_command(sensor_values(None, None, 0.80))
        self.assertIsNone(result["finalSteering"])
        self.assertEqual((result["left_power"], result["right_power"]), (0.0, 0.0))

    def test_no_valid_reading_stops_motors(self):
        result = calculate_command(sensor_values(None, None, None))
        self.assertIsNone(result["steeringError"])
        self.assertEqual(result["controlSource"], "virtual-no-line")
        self.assertEqual((result["left_power"], result["right_power"]), (0.0, 0.0))

    def test_green_blocks_medium_scan_and_reorient(self):
        tracker = camera_line_frame.VirtualTurnStateTracker()
        tracker.state = camera_line_frame.VIRTUAL_STATE_REORIENT_RIGHT
        result = calculate_command(
            sensor_values(None, 0.90, 0.90),
            tracker,
            green_direction="DIREITA",
        )
        self.assertEqual(tracker.state, camera_line_frame.VIRTUAL_STATE_NORMAL)
        self.assertEqual(result["finalSteering"], 1.0)
        self.assertEqual(
            result["controlSource"],
            "green-entry-pivot",
        )
        self.assertEqual(result["greenDirection"], "DIREITA")
        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (
                camera_line_frame.PIVOT_OUTER_POWER,
                camera_line_frame.PIVOT_INNER_POWER,
            ),
        )

    def test_gap_ignores_medium_alone_and_uses_far_with_both_wheels_forward(self):
        medium_only = sensor_values(None, -0.80, None)
        medium_only["farTrusted"] = False
        medium_only["farPosition"] = None
        far_right = sensor_values(None, None, 0.80)
        far_right["farPosition"] = 0.80
        cases = (
            (
                medium_only,
                "MEDIUM_ONLY",
                "gap-forward",
                (camera_line_frame.NORMAL_BASE_POWER,
                 camera_line_frame.NORMAL_BASE_POWER),
            ),
            (
                far_right,
                "FAR_RIGHT",
                "virtual-gap-far",
                (camera_line_frame.NORMAL_MAX_POWER,
                 camera_line_frame.NORMAL_INNER_MIN_POWER),
            ),
        )
        for sensors, _direction, expected_source, expected_powers in cases:
            with self.subTest(direction=_direction):
                tracker = camera_line_frame.VirtualTurnStateTracker()
                tracker.state = camera_line_frame.VIRTUAL_STATE_REORIENT_LEFT
                result = calculate_command(
                    sensors,
                    tracker,
                    gap_active=True,
                )
                self.assertEqual(
                    tracker.state,
                    camera_line_frame.VIRTUAL_STATE_NORMAL,
                )
                self.assertEqual(
                    result["controlSource"],
                    expected_source,
                )
                self.assertEqual(
                    (result["left_power"], result["right_power"]),
                    expected_powers,
                )

    def test_gap_far_guidance_ignores_near_and_legacy_timeout(self):
        sensors = sensor_values(None, 0.80, 0.80)
        sensors["farPosition"] = 0.80
        sensors["nearCenter"] = 0.20
        sensors["nearFinePosition"] = -0.80
        result = calculate_command(
            sensors,
            gap_active=True,
            blind_search_requested=True,
        )
        self.assertEqual(result["controlSource"], "virtual-gap-far")
        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (camera_line_frame.NORMAL_MAX_POWER,
             camera_line_frame.NORMAL_INNER_MIN_POWER),
        )

    def test_distant_fusion_only_controls_during_confirmed_gap_reacquisition(self):
        fusion_line = camera_line_frame.empty_fusion_style_line()
        fusion_line.update({
            "valid": True,
            "selection": "deepestFallback",
            "angleDeg": 110.0,
            "nearPoint": {"x": 50, "y": 99},
            "farPoint": {"x": 70, "y": 30},
            "targetStableFrames": 2,
            "targetConsistency": 1.0,
            "fusionSpeedScale": 1.0,
        })
        sensors = sensor_values(None, 0.25, 0.30)

        normal = calculate_command(sensors, fusion_style_line=fusion_line)
        reacquiring = calculate_command(
            sensors,
            fusion_style_line=fusion_line,
            gap_fusion_reacquire_active=True,
        )

        self.assertFalse(normal["fusionControlActive"])
        self.assertNotEqual(normal["controlSource"], "fusion")
        self.assertEqual(reacquiring["lineState"], "GAP")
        self.assertEqual(
            reacquiring["controlSource"],
            "fusion-gap-reacquire",
        )
        self.assertTrue(reacquiring["fusionControlActive"])
        self.assertGreater(reacquiring["left_power"], 0.0)
        self.assertGreater(reacquiring["right_power"], 0.0)
        self.assertNotEqual(
            reacquiring["left_power"],
            reacquiring["right_power"],
        )

    def test_extreme_gap_fusion_keeps_both_wheels_forward(self):
        fusion_line = camera_line_frame.empty_fusion_style_line()
        fusion_line.update({
            "valid": True,
            "selection": "deepestFallback",
            "angleDeg": 160.0,
            "nearPoint": {"x": 50, "y": 99},
            "farPoint": {"x": 70, "y": 30},
            "targetStableFrames": 2,
            "targetConsistency": 1.0,
            "fusionSpeedScale": 1.0,
        })
        result = calculate_command(
            sensor_values(None, 0.90, 0.90),
            fusion_style_line=fusion_line,
            gap_fusion_reacquire_active=True,
        )

        self.assertEqual(result["controlSource"], "fusion-gap-reacquire")
        self.assertGreater(result["left_power"], 0.0)
        self.assertGreater(result["right_power"], 0.0)
        self.assertEqual(
            min(result["left_power"], result["right_power"]),
            camera_line_frame.NORMAL_INNER_MIN_POWER,
        )

    def test_gap_only_ends_after_confirmed_near_reacquisition(self):
        self.assertEqual(camera_line_frame.GEOMETRIC_GAP_REACQUIRE_FRAMES, 2)
        state = {
            "active": True,
            "forwardFrames": 0,
            "reacquireFrames": 0,
            "lineLostSeen": False,
        }
        for _ in range(camera_line_frame.GEOMETRIC_GAP_FORWARD_MAX_FRAMES + 2):
            state = camera_line_frame.update_gap_forward_recovery(
                state["active"],
                state["forwardFrames"],
                state["reacquireFrames"],
                state["lineLostSeen"],
                near_reacquired=False,
            )
        self.assertTrue(state["active"])
        self.assertTrue(state["blindSearchRequested"])

        for frame_index in range(
            camera_line_frame.GEOMETRIC_GAP_REACQUIRE_FRAMES
        ):
            state = camera_line_frame.update_gap_forward_recovery(
                state["active"],
                state["forwardFrames"],
                state["reacquireFrames"],
                state["lineLostSeen"],
                near_reacquired=True,
            )
            if frame_index < camera_line_frame.GEOMETRIC_GAP_REACQUIRE_FRAMES - 1:
                self.assertTrue(state["active"])
        self.assertFalse(state["active"])

    def test_reorient_returns_to_normal_after_configured_valid_frames(self):
        tracker, _result = self.enter_reorient("LEFT")
        for frame_index in range(camera_line_frame.VIRTUAL_REORIENT_RECOVERY_FRAMES):
            result = calculate_command(sensor_values(0.20, None, None), tracker)
            self.assertEqual(result["finalSteering"], 0.20)
            self.assertEqual(result["controlSource"], "virtual")
            if frame_index < camera_line_frame.VIRTUAL_REORIENT_RECOVERY_FRAMES - 1:
                self.assertEqual(
                    tracker.state,
                    camera_line_frame.VIRTUAL_STATE_REORIENT_LEFT,
                )
        self.assertEqual(tracker.state, camera_line_frame.VIRTUAL_STATE_NORMAL)

    def test_missing_roi_positions_never_compare_none_with_float(self):
        self.assertIsNone(camera_line_frame.virtual_reorient_direction({}))
        self.assertIsNone(camera_line_frame.virtual_medium_scan_direction({}))
        for medium_position, far_band_position in (
            (None, None),
            (None, 0.80),
            (0.80, None),
        ):
            calculate_command(
                sensor_values(None, medium_position, far_band_position),
                camera_line_frame.VirtualTurnStateTracker(),
            )

    def test_green_keeps_only_near_center(self):
        mask = np.zeros((100, 100), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(mask.shape)
        near_center = geometry["near"]["center"]
        mask[
            near_center["y0"]:near_center["y1"],
            near_center["x0"]:near_center["x1"],
        ] = 255
        for green_direction in ("ESQUERDA", "DIREITA"):
            with self.subTest(green_direction=green_direction):
                sensors = camera_line_frame.read_virtual_line_sensors(
                    mask,
                    green_direction,
                )
                self.assertGreater(sensors["nearCenter"], 0.0)

    def test_normal_mapper_below_strong_transition_matches_expected_curve(self):
        for steering_error in (0.20, -0.20):
            result = calculate_command(
                sensor_values(steering_error, None, None),
                camera_line_frame.VirtualTurnStateTracker(),
            )
            expected_powers = expected_normal_motor_powers(steering_error)
            self.assertTrue(math.isclose(
                result["left_power"], expected_powers[0]
            ))
            self.assertTrue(math.isclose(
                result["right_power"], expected_powers[1]
            ))

    def test_far_mapper_is_continuous_at_normal_limit(self):
        transition = camera_line_frame.NORMAL_FULL_STEERING_ERROR
        before_transition = calculate_command(
            sensor_values(transition - 1e-9, None, None),
            camera_line_frame.VirtualTurnStateTracker(),
        )
        at_transition = calculate_command(
            sensor_values(transition + 1e-9, None, None),
            camera_line_frame.VirtualTurnStateTracker(),
        )
        self.assertTrue(math.isclose(at_transition["left_power"], 0.82))
        self.assertTrue(math.isclose(at_transition["right_power"], 0.66))
        self.assertTrue(math.isclose(
            before_transition["left_power"],
            at_transition["left_power"],
            abs_tol=1e-8,
        ))
        self.assertTrue(math.isclose(
            before_transition["right_power"],
            at_transition["right_power"],
            abs_tol=1e-8,
        ))

    def test_normal_mapper_uses_quadratic_progress_in_strong_range(self):
        steering_error = (
            camera_line_frame.NORMAL_FULL_STEERING_ERROR
            + camera_line_frame.PIVOT_ENTER_THRESHOLD
        ) * 0.5
        result = calculate_command(
            sensor_values(steering_error, 0.30, None),
            camera_line_frame.VirtualTurnStateTracker(),
        )
        expected_left, expected_right = expected_normal_motor_powers(
            steering_error,
            strong_enabled=True,
        )
        self.assertTrue(math.isclose(result["left_power"], expected_left))
        self.assertTrue(math.isclose(result["right_power"], expected_right))

    def test_normal_mapper_approaches_strong_limits_before_pivot(self):
        steering_error = camera_line_frame.PIVOT_ENTER_THRESHOLD - 1e-9
        result = calculate_command(
            sensor_values(steering_error, 0.30, None),
            camera_line_frame.VirtualTurnStateTracker(),
        )
        self.assertTrue(math.isclose(
            result["left_power"], 0.85, abs_tol=1e-8
        ))
        self.assertTrue(math.isclose(
            result["right_power"], 0.61, abs_tol=1e-8
        ))

    def test_far_level_steering_at_pivot_threshold_stays_normal(self):
        result = calculate_command(
            sensor_values(camera_line_frame.PIVOT_ENTER_THRESHOLD, None, None),
            camera_line_frame.VirtualTurnStateTracker(),
        )
        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (0.82, 0.66),
        )

    def test_normal_mapper_transition_is_symmetric(self):
        right = calculate_command(
            sensor_values(0.40, None, None),
            camera_line_frame.VirtualTurnStateTracker(),
        )
        left = calculate_command(
            sensor_values(-0.40, None, None),
            camera_line_frame.VirtualTurnStateTracker(),
        )
        self.assertTrue(math.isclose(
            right["left_power"], left["right_power"]
        ))
        self.assertTrue(math.isclose(
            right["right_power"], left["left_power"]
        ))

    def test_normal_mapper_matches_expected_curve(self):
        for steering_error in (
            -1.0,
            -0.50,
            -0.40,
            -0.20,
            0.0,
            0.20,
            0.40,
            0.50,
            1.0,
        ):
            result = calculate_command(
                sensor_values(steering_error, None, None),
                camera_line_frame.VirtualTurnStateTracker(),
            )
            expected_left, expected_right = expected_normal_motor_powers(
                steering_error
            )
            self.assertTrue(math.isclose(result["left_power"], expected_left))
            self.assertTrue(math.isclose(result["right_power"], expected_right))

    def test_far_steering_never_activates_pivot_tracker(self):
        pivot_tracker = camera_line_frame.VirtualPivotStateTracker()
        for steering_error in (0.47, 1.0, -1.0, 0.40):
            result = calculate_command(
                sensor_values(steering_error, None, None),
                camera_line_frame.VirtualTurnStateTracker(),
                pivot_state_tracker=pivot_tracker,
            )
            self.assertEqual(
                pivot_tracker.state,
                camera_line_frame.PIVOT_STATE_NONE,
            )
            self.assertGreater(result["left_power"], 0.0)
            self.assertGreater(result["right_power"], 0.0)

    def test_far_sign_inversion_remains_in_normal_mapper(self):
        pivot_tracker = camera_line_frame.VirtualPivotStateTracker()
        left = calculate_command(
            sensor_values(-1.0, 0.0, None),
            camera_line_frame.VirtualTurnStateTracker(),
            pivot_state_tracker=pivot_tracker,
        )
        right = calculate_command(
            sensor_values(1.0, 0.0, None),
            camera_line_frame.VirtualTurnStateTracker(),
            pivot_state_tracker=pivot_tracker,
        )
        self.assertEqual(
            pivot_tracker.state,
            camera_line_frame.PIVOT_STATE_NONE,
        )
        self.assertEqual(
            (left["left_power"], left["right_power"]),
            (0.66, 0.82),
        )
        self.assertEqual(
            (right["left_power"], right["right_power"]),
            (0.82, 0.66),
        )

    def test_green_keeps_direction_without_fusion_target(self):
        result = calculate_command(
            sensor_values(0.30, None, None),
            green_direction="DIREITA",
        )

        self.assertEqual(result["controlSource"], "green-entry-pivot")
        self.assertEqual(result["finalSteering"], 1.0)
        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (
                camera_line_frame.PIVOT_OUTER_POWER,
                camera_line_frame.PIVOT_INNER_POWER,
            ),
        )


class VirtualMediumUrgencyTests(unittest.TestCase):
    def test_local_hard_corner_with_missing_far_uses_existing_spin(self):
        sensors = sensor_values(
            0.0,
            -0.60,
            None,
            near_fine_position=-0.38,
        )
        sensors["farPosition"] = None

        result = calculate_command(
            sensors,
            pivot_state_tracker=camera_line_frame.VirtualPivotStateTracker(),
            medium_spin_tracker=camera_line_frame.VirtualMediumSpinTracker(),
        )

        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (-0.72, 0.72),
        )

    def test_hard_corner_persists_until_two_aligned_frames(self):
        tracker = camera_line_frame.VirtualMediumSpinTracker()
        entry_sensors = sensor_values(
            0.0,
            -0.72,
            None,
            near_fine_position=-0.38,
        )
        entry_sensors["farPosition"] = None

        entered = calculate_command(
            entry_sensors,
            medium_spin_tracker=tracker,
        )

        self.assertEqual(
            tracker.hard_corner_state,
            camera_line_frame.PIVOT_STATE_LEFT,
        )
        self.assertEqual(
            (entered["left_power"], entered["right_power"]),
            (-0.72, 0.72),
        )

        transient_sensors = (
            sensor_values(0.0, -0.05, None, near_fine_position=-0.38),
            sensor_values(None, None, None),
            sensor_values(0.0, 0.60, None, near_fine_position=0.38),
        )
        for transient_sensor in transient_sensors:
            transient_sensor["farPosition"] = None
        for sensors in transient_sensors:
            held = calculate_command(
                sensors,
                medium_spin_tracker=tracker,
            )
            self.assertEqual(
                tracker.hard_corner_state,
                camera_line_frame.PIVOT_STATE_LEFT,
            )
            self.assertEqual(
                (held["left_power"], held["right_power"]),
                (-0.72, 0.72),
            )

        aligned_sensors = sensor_values(
            0.04,
            0.08,
            None,
            near_fine_position=0.05,
        )
        first_aligned = calculate_command(
            aligned_sensors,
            medium_spin_tracker=tracker,
        )
        recovered = calculate_command(
            aligned_sensors,
            medium_spin_tracker=tracker,
        )

        self.assertEqual(
            (first_aligned["left_power"], first_aligned["right_power"]),
            (-0.72, 0.72),
        )
        self.assertEqual(
            tracker.hard_corner_state,
            camera_line_frame.PIVOT_STATE_NONE,
        )
        self.assertEqual(
            (recovered["left_power"], recovered["right_power"]),
            expected_normal_motor_powers(0.04),
        )

    def test_hard_corner_requires_two_consecutive_far_frames_to_exit(self):
        tracker = camera_line_frame.VirtualMediumSpinTracker()
        entry_sensors = sensor_values(
            0.0,
            -0.72,
            None,
            near_fine_position=-0.38,
        )
        entry_sensors["farPosition"] = None
        far_sensors = dict(entry_sensors)
        far_sensors["farPosition"] = -0.40

        calculate_command(entry_sensors, medium_spin_tracker=tracker)
        isolated_far = calculate_command(
            far_sensors,
            medium_spin_tracker=tracker,
        )
        far_lost_again = calculate_command(
            entry_sensors,
            medium_spin_tracker=tracker,
        )
        first_consecutive_far = calculate_command(
            far_sensors,
            medium_spin_tracker=tracker,
        )
        recovered = calculate_command(
            far_sensors,
            medium_spin_tracker=tracker,
        )

        for held in (
            isolated_far,
            far_lost_again,
            first_consecutive_far,
        ):
            self.assertEqual(
                (held["left_power"], held["right_power"]),
                (-0.72, 0.72),
            )
        self.assertEqual(
            tracker.hard_corner_state,
            camera_line_frame.PIVOT_STATE_NONE,
        )
        self.assertEqual(
            (recovered["left_power"], recovered["right_power"]),
            (-0.72, 0.78),
        )

    def test_hard_corner_timeout_returns_to_existing_recovery(self):
        tracker = camera_line_frame.VirtualMediumSpinTracker()
        entry_sensors = sensor_values(
            0.0,
            -0.72,
            None,
            near_fine_position=-0.38,
        )
        entry_sensors["farPosition"] = None
        calculate_command(entry_sensors, medium_spin_tracker=tracker)

        for _ in range(
            camera_line_frame.VIRTUAL_HARD_CORNER_MAX_FRAMES - 2
        ):
            held = calculate_command(
                entry_sensors,
                medium_spin_tracker=tracker,
            )
            self.assertEqual(
                (held["left_power"], held["right_power"]),
                (-0.72, 0.72),
            )

        timed_out = calculate_command(
            entry_sensors,
            medium_spin_tracker=tracker,
        )
        still_blocked = calculate_command(
            entry_sensors,
            medium_spin_tracker=tracker,
        )

        self.assertEqual(
            tracker.hard_corner_state,
            camera_line_frame.PIVOT_STATE_NONE,
        )
        self.assertTrue(tracker.hard_corner_entry_blocked)
        self.assertEqual(
            (timed_out["left_power"], timed_out["right_power"]),
            (-0.72, 0.78),
        )
        self.assertEqual(
            (still_blocked["left_power"], still_blocked["right_power"]),
            (-0.72, 0.78),
        )

    def test_green_and_gap_cancel_persistent_hard_corner(self):
        tracker = camera_line_frame.VirtualMediumSpinTracker()
        entry_sensors = sensor_values(
            0.0,
            -0.72,
            None,
            near_fine_position=-0.38,
        )
        entry_sensors["farPosition"] = None

        calculate_command(entry_sensors, medium_spin_tracker=tracker)
        green = calculate_command(
            entry_sensors,
            medium_spin_tracker=tracker,
            green_direction="DIREITA",
        )
        self.assertEqual(
            tracker.hard_corner_state,
            camera_line_frame.PIVOT_STATE_NONE,
        )
        self.assertNotEqual(
            (green["left_power"], green["right_power"]),
            (-0.72, 0.72),
        )

        calculate_command(entry_sensors, medium_spin_tracker=tracker)
        gap = calculate_command(
            entry_sensors,
            medium_spin_tracker=tracker,
            gap_active=True,
        )
        self.assertEqual(
            tracker.hard_corner_state,
            camera_line_frame.PIVOT_STATE_NONE,
        )
        self.assertNotEqual(
            (gap["left_power"], gap["right_power"]),
            (-0.72, 0.72),
        )

    def test_hard_corner_requires_large_near_offset(self):
        sensors = sensor_values(
            0.0,
            -0.60,
            None,
            near_fine_position=-0.10,
        )
        sensors["farPosition"] = None

        result = calculate_command(
            sensors,
            pivot_state_tracker=camera_line_frame.VirtualPivotStateTracker(),
            medium_spin_tracker=camera_line_frame.VirtualMediumSpinTracker(),
        )

        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (-0.72, 0.78),
        )

    def test_hard_corner_requires_missing_far(self):
        result = calculate_command(
            sensor_values(
                0.0,
                -0.60,
                None,
                near_fine_position=-0.38,
            ),
            pivot_state_tracker=camera_line_frame.VirtualPivotStateTracker(),
            medium_spin_tracker=camera_line_frame.VirtualMediumSpinTracker(),
        )

        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (-0.72, 0.78),
        )

    def test_hard_corner_requires_matching_medium_and_near_sides(self):
        sensors = sensor_values(
            0.0,
            -0.60,
            None,
            near_fine_position=0.38,
        )
        sensors["farPosition"] = None

        result = calculate_command(
            sensors,
            pivot_state_tracker=camera_line_frame.VirtualPivotStateTracker(),
            medium_spin_tracker=camera_line_frame.VirtualMediumSpinTracker(),
        )

        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (-0.72, 0.78),
        )

    def test_medium_below_strong_threshold_does_not_promote(self):
        tracker = camera_line_frame.VirtualMediumSpinTracker()
        result = calculate_command(
            sensor_values(0.12, 0.20, None),
            medium_spin_tracker=tracker,
        )
        reference = calculate_command(
            sensor_values(0.12, None, None),
            medium_spin_tracker=camera_line_frame.VirtualMediumSpinTracker(),
        )

        self.assertEqual(result["finalSteering"], 0.12)
        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (reference["left_power"], reference["right_power"]),
        )

    def test_medium_guarantees_strong_action_in_its_direction(self):
        for medium_position, expected_sign in ((-0.30, -1.0), (0.30, 1.0)):
            with self.subTest(medium_position=medium_position):
                result = calculate_command(
                    sensor_values(-expected_sign * 0.10, medium_position, None),
                    medium_spin_tracker=(
                        camera_line_frame.VirtualMediumSpinTracker()
                    ),
                )

                self.assertEqual(
                    result["finalSteering"],
                    expected_sign
                    * camera_line_frame.NORMAL_FULL_STEERING_ERROR,
                )
                self.assertGreater(result["left_power"], 0.0)
                self.assertGreater(result["right_power"], 0.0)
                if expected_sign < 0.0:
                    self.assertLess(
                        result["left_power"], result["right_power"]
                    )
                else:
                    self.assertGreater(
                        result["left_power"], result["right_power"]
                    )

    def test_medium_critical_without_near_enters_spin(self):
        for medium_position, expected_powers in (
            (-0.50, (-0.72, 0.72)),
            (0.50, (0.72, -0.72)),
        ):
            with self.subTest(medium_position=medium_position):
                result = calculate_command(
                    sensor_values(0.0, medium_position, None),
                    medium_spin_tracker=(
                        camera_line_frame.VirtualMediumSpinTracker()
                    ),
                )

                self.assertEqual(
                    (result["left_power"], result["right_power"]),
                    expected_powers,
                )

    def test_far_extreme_with_central_medium_never_pivots_or_spins(self):
        for medium_position in (None, -0.20, 0.0, 0.20):
            for near_fine_position in (None, 0.0):
                with self.subTest(
                    medium_position=medium_position,
                    near_fine_position=near_fine_position,
                ):
                    pivot_tracker = (
                        camera_line_frame.VirtualPivotStateTracker()
                    )
                    result = calculate_command(
                        sensor_values(
                            1.0,
                            medium_position,
                            None,
                            near_fine_position=near_fine_position,
                        ),
                        pivot_state_tracker=pivot_tracker,
                        medium_spin_tracker=(
                            camera_line_frame.VirtualMediumSpinTracker()
                        ),
                    )

                    self.assertEqual(
                        result["finalSteering"],
                        camera_line_frame.NORMAL_FULL_STEERING_ERROR,
                    )
                    self.assertEqual(
                        pivot_tracker.state,
                        camera_line_frame.PIVOT_STATE_NONE,
                    )
                    self.assertEqual(
                        (result["left_power"], result["right_power"]),
                        (0.82, 0.66),
                    )

    def test_medium_critical_with_valid_near_uses_pivot(self):
        result = calculate_command(
            sensor_values(0.0, 0.50, None, near_fine_position=0.0),
            pivot_state_tracker=camera_line_frame.VirtualPivotStateTracker(),
            medium_spin_tracker=camera_line_frame.VirtualMediumSpinTracker(),
        )

        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (0.78, -0.72),
        )

    def test_spin_exits_at_medium_exit_threshold_in_same_frame(self):
        tracker = camera_line_frame.VirtualMediumSpinTracker()
        entered = calculate_command(
            sensor_values(0.0, 0.48, None),
            medium_spin_tracker=tracker,
        )
        held = calculate_command(
            sensor_values(0.0, 0.31, None),
            medium_spin_tracker=tracker,
        )
        released = calculate_command(
            sensor_values(0.0, 0.30, None),
            medium_spin_tracker=tracker,
        )

        self.assertEqual(
            (entered["left_power"], entered["right_power"]),
            (0.72, -0.72),
        )
        self.assertEqual(
            (held["left_power"], held["right_power"]),
            (0.72, -0.72),
        )
        self.assertEqual(tracker.state, camera_line_frame.PIVOT_STATE_NONE)
        self.assertEqual(
            released["finalSteering"],
            camera_line_frame.NORMAL_FULL_STEERING_ERROR,
        )
        self.assertGreater(released["left_power"], 0.0)
        self.assertGreater(released["right_power"], 0.0)

    def test_spin_cancels_immediately_on_side_change(self):
        side_tracker = camera_line_frame.VirtualMediumSpinTracker()
        calculate_command(
            sensor_values(0.0, 0.48, None),
            medium_spin_tracker=side_tracker,
        )
        changed_side = calculate_command(
            sensor_values(0.0, -0.48, None),
            medium_spin_tracker=side_tracker,
        )

        self.assertEqual(
            side_tracker.state,
            camera_line_frame.PIVOT_STATE_NONE,
        )
        self.assertEqual(
            changed_side["finalSteering"],
            -camera_line_frame.NORMAL_FULL_STEERING_ERROR,
        )
        self.assertGreater(changed_side["left_power"], 0.0)
        self.assertGreater(changed_side["right_power"], 0.0)

    def test_critical_tolerance_holds_three_invalid_medium_frames(self):
        tracker = camera_line_frame.VirtualMediumSpinTracker()
        calculate_command(
            sensor_values(0.0, 0.48, None),
            medium_spin_tracker=tracker,
        )

        for invalid_frame in (1, 2, 3):
            result = calculate_command(
                sensor_values(None, None, None),
                medium_spin_tracker=tracker,
            )
            self.assertEqual(tracker.invalid_frames, invalid_frame)
            self.assertEqual(
                (result["left_power"], result["right_power"]),
                (0.72, -0.72),
            )

    def test_critical_tolerance_expires_after_configured_invalid_frames(self):
        tracker = camera_line_frame.VirtualMediumSpinTracker()
        calculate_command(
            sensor_values(0.0, 0.48, None),
            medium_spin_tracker=tracker,
        )
        for _ in range(
            camera_line_frame.VIRTUAL_MEDIUM_CRITICAL_INVALID_MAX_FRAMES
        ):
            calculate_command(
                sensor_values(None, None, None),
                medium_spin_tracker=tracker,
            )

        expired = calculate_command(
            sensor_values(None, None, None),
            medium_spin_tracker=tracker,
        )

        self.assertEqual(tracker.state, camera_line_frame.PIVOT_STATE_NONE)
        self.assertEqual(
            tracker.critical_state,
            camera_line_frame.PIVOT_STATE_NONE,
        )
        self.assertEqual(tracker.invalid_frames, 0)
        self.assertEqual(
            (expired["left_power"], expired["right_power"]),
            (0.0, 0.0),
        )

    def test_neutral_medium_keeps_previous_critical_direction(self):
        tracker = camera_line_frame.VirtualMediumSpinTracker()
        calculate_command(
            sensor_values(0.0, -0.50, None),
            medium_spin_tracker=tracker,
        )
        direction_lost = calculate_command(
            sensor_values(0.0, -0.05, None),
            medium_spin_tracker=tracker,
        )

        self.assertEqual(tracker.invalid_frames, 1)
        self.assertEqual(
            tracker.critical_state,
            camera_line_frame.PIVOT_STATE_LEFT,
        )
        self.assertEqual(
            (direction_lost["left_power"], direction_lost["right_power"]),
            (-0.72, 0.72),
        )

    def test_valid_medium_ends_tolerance_in_current_frame(self):
        tracker = camera_line_frame.VirtualMediumSpinTracker()
        calculate_command(
            sensor_values(0.0, 0.48, None),
            medium_spin_tracker=tracker,
        )
        calculate_command(
            sensor_values(None, None, None),
            medium_spin_tracker=tracker,
        )
        recovered = calculate_command(
            sensor_values(0.12, 0.20, None),
            medium_spin_tracker=tracker,
        )

        self.assertEqual(tracker.state, camera_line_frame.PIVOT_STATE_NONE)
        self.assertEqual(
            tracker.critical_state,
            camera_line_frame.PIVOT_STATE_NONE,
        )
        self.assertEqual(tracker.invalid_frames, 0)
        self.assertEqual(recovered["finalSteering"], 0.12)
        self.assertGreater(recovered["left_power"], 0.0)
        self.assertGreater(recovered["right_power"], 0.0)

    def test_near_reappearance_ends_tolerance_in_current_frame(self):
        tracker = camera_line_frame.VirtualMediumSpinTracker()
        calculate_command(
            sensor_values(0.0, 0.48, None),
            medium_spin_tracker=tracker,
        )
        calculate_command(
            sensor_values(None, None, None),
            medium_spin_tracker=tracker,
        )
        near_reacquired = calculate_command(
            sensor_values(0.12, None, None, near_fine_position=0.0),
            medium_spin_tracker=tracker,
        )

        self.assertEqual(tracker.state, camera_line_frame.PIVOT_STATE_NONE)
        self.assertEqual(
            tracker.critical_state,
            camera_line_frame.PIVOT_STATE_NONE,
        )
        self.assertEqual(tracker.invalid_frames, 0)
        self.assertEqual(near_reacquired["finalSteering"], 0.12)
        self.assertGreater(near_reacquired["left_power"], 0.0)
        self.assertGreater(near_reacquired["right_power"], 0.0)

    def test_pivot_side_is_tolerated_if_medium_and_near_are_lost(self):
        tracker = camera_line_frame.VirtualMediumSpinTracker()
        pivot = calculate_command(
            sensor_values(0.0, -0.50, None, near_fine_position=0.0),
            medium_spin_tracker=tracker,
        )
        lost = calculate_command(
            sensor_values(None, None, None),
            medium_spin_tracker=tracker,
        )

        self.assertEqual(
            (pivot["left_power"], pivot["right_power"]),
            (-0.72, 0.78),
        )
        self.assertEqual(
            (lost["left_power"], lost["right_power"]),
            (-0.72, 0.72),
        )

    def test_spin_exits_to_pivot_when_near_reappears(self):
        medium_tracker = camera_line_frame.VirtualMediumSpinTracker()
        spinning = calculate_command(
            sensor_values(0.0, 0.50, None),
            medium_spin_tracker=medium_tracker,
        )
        near_reacquired = calculate_command(
            sensor_values(0.0, 0.50, None, near_fine_position=0.0),
            medium_spin_tracker=medium_tracker,
        )

        self.assertEqual(
            (spinning["left_power"], spinning["right_power"]),
            (0.72, -0.72),
        )
        self.assertEqual(
            medium_tracker.state,
            camera_line_frame.PIVOT_STATE_NONE,
        )
        self.assertEqual(
            (near_reacquired["left_power"], near_reacquired["right_power"]),
            (0.78, -0.72),
        )


class GreenRearmTests(unittest.TestCase):
    def test_normal_fusion_rearms_immediately_with_next_candidate_visible(self):
        armed, frames = camera_line_frame.update_green_rearm_state(
            False, 0, "NENHUMA", candidate_count=1,
            normal_fusion_valid=True,
        )
        self.assertTrue(armed)
        self.assertEqual(frames, 0)
        for direction in ("ESQUERDA", "DIREITA"):
            self.assertEqual(camera_line_frame.select_confirmed_green_direction(
                armed, "NENHUMA",
                {"greenConfirmed": True, "greenInterpretation": direction},
                marker_consumed=False,
            ), direction)

    def test_fusion_cannot_rearm_before_active_green_finishes(self):
        self.assertEqual(camera_line_frame.update_green_rearm_state(
            False, 0, "DIREITA", candidate_count=1,
            normal_fusion_valid=True,
        ), (False, 0))

    def test_consumed_confirmation_does_not_repeat_after_fusion_rearms(self):
        self.assertIsNone(camera_line_frame.select_confirmed_green_direction(
            True, "NENHUMA",
            {"greenConfirmed": True, "greenInterpretation": "DIREITA"},
            marker_consumed=True,
        ))
        consumed, frames = camera_line_frame.update_green_consumed_marker_state(
            True, 0, candidate_count=0,
        )
        self.assertTrue(consumed)
        consumed, frames = camera_line_frame.update_green_consumed_marker_state(
            consumed, frames, candidate_count=0,
        )
        self.assertFalse(consumed)
        self.assertEqual(frames, 0)
        self.assertEqual(camera_line_frame.select_confirmed_green_direction(
            True, "NENHUMA",
            {"greenConfirmed": True, "greenInterpretation": "DIREITA"},
            marker_consumed=consumed,
        ), "DIREITA")

    def test_green_absence_during_active_maneuver_does_not_rearm(self):
        armed = False
        clear_frames = 0

        for _frame_index in range(10):
            armed, clear_frames = (
                camera_line_frame.update_green_rearm_state(
                    armed,
                    clear_frames,
                    active_direction="ESQUERDA",
                    candidate_count=0,
                )
            )

        self.assertFalse(armed)
        self.assertEqual(clear_frames, 0)

    def test_green_rearms_only_on_fifth_clean_frame_after_maneuver(self):
        armed = False
        clear_frames = 0

        for expected_frames in range(1, 5):
            armed, clear_frames = (
                camera_line_frame.update_green_rearm_state(
                    armed,
                    clear_frames,
                    active_direction="NENHUMA",
                    candidate_count=0,
                )
            )
            self.assertFalse(armed)
            self.assertEqual(clear_frames, expected_frames)

        armed, clear_frames = camera_line_frame.update_green_rearm_state(
            armed,
            clear_frames,
            active_direction="NENHUMA",
            candidate_count=0,
        )

        self.assertTrue(armed)
        self.assertEqual(clear_frames, 0)
        self.assertEqual(camera_line_frame.QUADROS_PARA_REARMAR_VERDE, 5)

    def test_any_green_candidate_resets_rearm_window(self):
        armed = False
        clear_frames = 0

        for _frame_index in range(3):
            armed, clear_frames = (
                camera_line_frame.update_green_rearm_state(
                    armed,
                    clear_frames,
                    active_direction="NENHUMA",
                    candidate_count=0,
                )
            )
        self.assertEqual(clear_frames, 3)

        armed, clear_frames = camera_line_frame.update_green_rearm_state(
            armed,
            clear_frames,
            active_direction="NENHUMA",
            candidate_count=1,
        )
        self.assertFalse(armed)
        self.assertEqual(clear_frames, 0)

        for expected_frames in range(1, 5):
            armed, clear_frames = (
                camera_line_frame.update_green_rearm_state(
                    armed,
                    clear_frames,
                    active_direction="NENHUMA",
                    candidate_count=0,
                )
            )
            self.assertFalse(armed)
            self.assertEqual(clear_frames, expected_frames)

    def test_rearmed_control_accepts_new_confirmed_green(self):
        armed = False
        clear_frames = 0
        for _frame_index in range(
            camera_line_frame.QUADROS_PARA_REARMAR_VERDE
        ):
            armed, clear_frames = (
                camera_line_frame.update_green_rearm_state(
                    armed,
                    clear_frames,
                    active_direction="NENHUMA",
                    candidate_count=0,
                )
            )

        green_status = camera_line_frame.empty_green_status()
        green_status["greenConfirmed"] = True
        green_status["greenInterpretation"] = "ESQUERDA"

        self.assertEqual(
            camera_line_frame.select_confirmed_green_direction(
                armed,
                "NENHUMA",
                green_status,
            ),
            "ESQUERDA",
        )

    def test_green_telemetry_distinguishes_detected_from_active(self):
        green_status = camera_line_frame.empty_green_status()
        green_status["greenConfirmed"] = True
        green_status["greenInterpretation"] = "ESQUERDA"
        green_status["greenRawInterpretation"] = "ESQUERDA"

        camera_line_frame.update_green_control_telemetry(
            green_status,
            active_direction="NENHUMA",
            armed=False,
            rearm_clear_frames=2,
        )
        self.assertEqual(green_status["greenDecisionState"], "detected")
        self.assertFalse(green_status["greenControlActive"])
        self.assertEqual(green_status["greenControlDirection"], "NENHUMA")
        self.assertFalse(green_status["greenArmed"])
        self.assertEqual(green_status["greenRearmClearFrames"], 2)

        camera_line_frame.update_green_control_telemetry(
            green_status,
            active_direction="ESQUERDA",
            armed=False,
            rearm_clear_frames=0,
        )
        self.assertEqual(green_status["greenDecisionState"], "active")
        self.assertTrue(green_status["greenControlActive"])
        self.assertEqual(green_status["greenControlDirection"], "ESQUERDA")


class GreenCandidateHoldTests(unittest.TestCase):
    @staticmethod
    def fusion_command(left_power, right_power, angle):
        steering_error = (
            camera_line_frame.map_fusion_angle_to_steering_error(angle)
        )
        return {
            "left_power": left_power,
            "right_power": right_power,
            "controlSource": "fusion",
            "fusionControlActive": True,
            "fusionAngle": angle,
            "filteredFusionAngle": angle,
            "fusionSteeringError": steering_error,
            "steeringError": steering_error,
            "finalSteering": steering_error,
            "fusionSpeedScale": 1.0,
        }

    @staticmethod
    def pending_status():
        status = camera_line_frame.empty_green_status()
        status["greenCandidateCount"] = 1
        status["greenRawInterpretation"] = "ESQUERDA"
        status["greenObservationState"] = "UM_CANDIDATO"
        status["greenPathBlackValid"] = True
        return status

    def test_candidate_first_frame_holds_previous_fusion_command(self):
        previous = camera_line_frame.capture_valid_fusion_command(
            self.fusion_command(0.72, 0.78, 86.0)
        )
        current = self.fusion_command(0.85, -0.40, 145.0)

        result = camera_line_frame.apply_green_candidate_fusion_hold(
            current,
            previous,
            armed=True,
            active_direction="NENHUMA",
            green_status=self.pending_status(),
            hold_frames=0,
            hold_blocked=False,
        )

        self.assertTrue(result["active"])
        self.assertEqual(result["frames"], 1)
        self.assertEqual(
            (result["command"]["left_power"], result["command"]["right_power"]),
            (0.72, 0.78),
        )
        self.assertEqual(result["command"]["fusionAngle"], 86.0)

    def test_confirmation_on_second_frame_bypasses_hold_immediately(self):
        previous = camera_line_frame.capture_valid_fusion_command(
            self.fusion_command(0.72, 0.78, 86.0)
        )
        first = camera_line_frame.apply_green_candidate_fusion_hold(
            self.fusion_command(0.85, -0.40, 145.0),
            previous,
            armed=True,
            active_direction="NENHUMA",
            green_status=self.pending_status(),
            hold_frames=0,
            hold_blocked=False,
        )
        confirmed_status = self.pending_status()
        confirmed_status["greenConfirmed"] = True
        green_command = self.fusion_command(-0.30, 0.85, 155.0)
        green_command["controlSource"] = "fusion-green"

        result = camera_line_frame.apply_green_candidate_fusion_hold(
            green_command,
            previous,
            armed=False,
            active_direction="ESQUERDA",
            green_status=confirmed_status,
            hold_frames=first["frames"],
            hold_blocked=first["blocked"],
        )

        self.assertFalse(result["active"])
        self.assertEqual(result["frames"], 0)
        self.assertEqual(result["command"], green_command)
        self.assertEqual(result["command"]["controlSource"], "fusion-green")

    def test_candidate_disappearance_restores_current_fusion_immediately(self):
        previous = camera_line_frame.capture_valid_fusion_command(
            self.fusion_command(0.72, 0.78, 86.0)
        )
        first = camera_line_frame.apply_green_candidate_fusion_hold(
            self.fusion_command(0.85, -0.40, 145.0),
            previous,
            armed=True,
            active_direction="NENHUMA",
            green_status=self.pending_status(),
            hold_frames=0,
            hold_blocked=False,
        )
        resumed_command = self.fusion_command(0.82, 0.66, 105.0)

        result = camera_line_frame.apply_green_candidate_fusion_hold(
            resumed_command,
            previous,
            armed=True,
            active_direction="NENHUMA",
            green_status=camera_line_frame.empty_green_status(),
            hold_frames=first["frames"],
            hold_blocked=first["blocked"],
        )

        self.assertFalse(result["active"])
        self.assertFalse(result["blocked"])
        self.assertEqual(result["frames"], 0)
        self.assertEqual(result["command"], resumed_command)

    def test_explicitly_rejected_candidate_exits_hold_immediately(self):
        previous = camera_line_frame.capture_valid_fusion_command(
            self.fusion_command(0.72, 0.78, 86.0)
        )
        first = camera_line_frame.apply_green_candidate_fusion_hold(
            self.fusion_command(0.85, -0.40, 145.0),
            previous,
            armed=True,
            active_direction="NENHUMA",
            green_status=self.pending_status(),
            hold_frames=0,
            hold_blocked=False,
        )
        rejected_status = self.pending_status()
        rejected_status["greenRawInterpretation"] = "VERDE_FALSO"
        resumed_command = self.fusion_command(0.82, 0.66, 105.0)

        result = camera_line_frame.apply_green_candidate_fusion_hold(
            resumed_command,
            previous,
            armed=True,
            active_direction="NENHUMA",
            green_status=rejected_status,
            hold_frames=first["frames"],
            hold_blocked=first["blocked"],
        )

        self.assertFalse(result["active"])
        self.assertFalse(result["blocked"])
        self.assertEqual(result["frames"], 0)
        self.assertEqual(result["command"], resumed_command)

    def test_persistent_candidate_expires_after_two_frames_without_restart(self):
        previous = camera_line_frame.capture_valid_fusion_command(
            self.fusion_command(0.72, 0.78, 86.0)
        )
        hold_frames = 0
        hold_blocked = False
        activity = []

        for frame_index in range(1, 5):
            current = self.fusion_command(
                0.80 + frame_index * 0.01,
                0.70 - frame_index * 0.01,
                90.0 + frame_index,
            )
            result = camera_line_frame.apply_green_candidate_fusion_hold(
                current,
                previous,
                armed=True,
                active_direction="NENHUMA",
                green_status=self.pending_status(),
                hold_frames=hold_frames,
                hold_blocked=hold_blocked,
            )
            activity.append(result["active"])
            hold_frames = result["frames"]
            hold_blocked = result["blocked"]
            if frame_index >= 3:
                self.assertEqual(result["command"], current)

        self.assertEqual(activity, [True, True, False, False])
        self.assertEqual(hold_frames, 2)
        self.assertTrue(hold_blocked)

    def test_without_candidate_or_with_active_green_command_is_unchanged(self):
        previous = camera_line_frame.capture_valid_fusion_command(
            self.fusion_command(0.72, 0.78, 86.0)
        )
        normal_command = self.fusion_command(0.82, 0.66, 105.0)
        without_candidate = camera_line_frame.apply_green_candidate_fusion_hold(
            normal_command,
            previous,
            armed=True,
            active_direction="NENHUMA",
            green_status=camera_line_frame.empty_green_status(),
            hold_frames=0,
            hold_blocked=False,
        )
        confirmed_status = self.pending_status()
        confirmed_status["greenConfirmed"] = True
        green_command = self.fusion_command(-0.30, 0.85, 155.0)
        green_command["controlSource"] = "fusion-green"
        active_green = camera_line_frame.apply_green_candidate_fusion_hold(
            green_command,
            previous,
            armed=False,
            active_direction="ESQUERDA",
            green_status=confirmed_status,
            hold_frames=0,
            hold_blocked=False,
        )

        self.assertEqual(without_candidate["command"], normal_command)
        self.assertFalse(without_candidate["active"])
        self.assertEqual(active_green["command"], green_command)
        self.assertFalse(active_green["active"])


class GreenFusionTargetHoldTests(unittest.TestCase):
    @staticmethod
    def valid_fusion_line(direction):
        line = camera_line_frame.empty_fusion_style_line()
        line.update({
            "valid": True,
            "selection": "nearCenter",
            "referenceSource": (
                "leftEdge" if direction == "ESQUERDA" else "rightEdge"
            ),
            "preferredDirection": (
                "LEFT" if direction == "ESQUERDA" else "RIGHT"
            ),
            "angleDeg": 30.0 if direction == "ESQUERDA" else 150.0,
            "nearPoint": {"x": 240, "y": 359},
            "farPoint": {
                "x": 0 if direction == "ESQUERDA" else 479,
                "y": 180,
            },
            "fusionSpeedScale": 1.0,
        })
        return line

    def test_last_green_target_is_held_during_short_missing_window(self):
        valid_line = self.valid_fusion_line("DIREITA")
        state = camera_line_frame.update_green_fusion_target_hold(
            "DIREITA",
            valid_line,
            previous_valid_fusion_line=None,
            missing_frames=0,
        )
        self.assertFalse(state["holdActive"])

        expected_command = calculate_command(
            sensor_values(),
            green_direction="DIREITA",
            fusion_style_line=valid_line,
        )
        for expected_missing_frames in range(
            1,
            camera_line_frame.GREEN_FUSION_TARGET_HOLD_MAX_FRAMES + 1,
        ):
            state = camera_line_frame.update_green_fusion_target_hold(
                "DIREITA",
                camera_line_frame.empty_fusion_style_line(),
                state["previousValidFusionLine"],
                state["missingFrames"],
            )
            self.assertTrue(state["holdActive"])
            self.assertEqual(
                state["missingFrames"],
                expected_missing_frames,
            )
            held_command = calculate_command(
                sensor_values(),
                green_direction="DIREITA",
                fusion_style_line=state["fusionLine"],
            )
            self.assertEqual(
                (held_command["left_power"], held_command["right_power"]),
                (
                    expected_command["left_power"],
                    expected_command["right_power"],
                ),
            )

        state = camera_line_frame.update_green_fusion_target_hold(
            "DIREITA",
            camera_line_frame.empty_fusion_style_line(),
            state["previousValidFusionLine"],
            state["missingFrames"],
        )
        self.assertFalse(state["holdActive"])
        self.assertIsNone(state["recoveryDirection"])
        self.assertIsNone(state["previousValidFusionLine"])
        self.assertEqual(
            state["missingFrames"],
            camera_line_frame.GREEN_FUSION_TARGET_HOLD_MAX_FRAMES + 1,
        )

        directional_hold = calculate_command(
            sensor_values(None, None, None),
            green_direction="DIREITA",
            fusion_style_line=state["fusionLine"],
            green_active_frames=24,
        )
        self.assertEqual(
            directional_hold["controlSource"],
            "green-direction-hold",
        )
        self.assertEqual(
            (
                directional_hold["left_power"],
                directional_hold["right_power"],
            ),
            (
                camera_line_frame.PIVOT_OUTER_POWER,
                camera_line_frame.PIVOT_INNER_POWER,
            ),
        )

    def test_missing_green_target_keeps_visual_turn_until_angle_release(self):
        for direction in ("ESQUERDA", "DIREITA"):
            with self.subTest(direction=direction):
                tracker = camera_line_frame.VirtualLineSearchTracker()
                tracker.start("LEFT" if direction == "DIREITA" else "RIGHT")
                result = calculate_command(
                    sensor_values(None, -0.90, -0.90),
                    green_direction=direction,
                    green_active_frames=24,
                    gap_active=True,
                    line_search_tracker=tracker,
                )
                self.assertEqual(result["controlSource"], "green-direction-hold")
                self.assertEqual(result["lineState"], "GREEN")
                self.assertEqual(result["greenDirection"], direction)
                self.assertFalse(tracker.active)
                self.assertEqual(
                    (result["left_power"], result["right_power"]),
                    (
                        camera_line_frame.PIVOT_OUTER_POWER
                        if direction == "DIREITA" else camera_line_frame.PIVOT_INNER_POWER,
                        camera_line_frame.PIVOT_INNER_POWER
                        if direction == "DIREITA" else camera_line_frame.PIVOT_OUTER_POWER,
                    ),
                )

                resumed = calculate_command(
                    sensor_values(),
                    green_direction=direction,
                    green_active_frames=25,
                    fusion_style_line=self.valid_fusion_line(direction),
                )
                self.assertEqual(resumed["controlSource"], "fusion-green")
                self.assertEqual(resumed["greenDirection"], direction)
                self.assertNotEqual(
                    (resumed["left_power"], resumed["right_power"]),
                    (0.0, 0.0),
                )

    def test_target_recovery_resets_short_loss_window(self):
        previous = self.valid_fusion_line("ESQUERDA")
        missing = camera_line_frame.update_green_fusion_target_hold(
            "ESQUERDA",
            camera_line_frame.empty_fusion_style_line(),
            previous,
            missing_frames=0,
        )
        recovered = camera_line_frame.update_green_fusion_target_hold(
            "ESQUERDA",
            previous,
            missing["previousValidFusionLine"],
            missing["missingFrames"],
        )

        self.assertFalse(recovered["holdActive"])
        self.assertEqual(recovered["missingFrames"], 0)
        self.assertIsNone(recovered["recoveryDirection"])

    def test_inactive_green_clears_held_target(self):
        result = camera_line_frame.update_green_fusion_target_hold(
            "NENHUMA",
            camera_line_frame.empty_fusion_style_line(),
            self.valid_fusion_line("DIREITA"),
            missing_frames=2,
        )

        self.assertIsNone(result["previousValidFusionLine"])
        self.assertEqual(result["missingFrames"], 0)
        self.assertIsNone(result["recoveryDirection"])


class GreenAngleLimitTests(unittest.TestCase):
    @staticmethod
    def control(yaw, gyro=0.0):
        return {
            "greenYawValid": True,
            "greenYawDegrees": yaw,
            "greenGyroDegreesPerSecond": gyro,
            "greenYawAgeMs": 0.0,
            "greenMaximumTurnDegrees": 45.0,
            "greenAnglePredictionSeconds": 0.10,
            "timestamp": 100.0,
        }

    def test_both_sides_release_at_45_degrees_including_yaw_wrap(self):
        for start, yaw in ((0, 45), (0, -45), (170, -145), (-170, 145)):
            with self.subTest(start=start, yaw=yaw):
                self.assertTrue(camera_line_frame.green_maneuver_angle_limit_reached(
                    start, self.control(yaw), now=100.0,
                ))
        self.assertFalse(camera_line_frame.green_maneuver_angle_limit_reached(
            170, self.control(-146), now=100.0,
        ))

    def test_angular_rate_anticipates_release_instead_of_overshooting(self):
        self.assertTrue(camera_line_frame.green_maneuver_angle_limit_reached(
            0, self.control(35, gyro=100.0), now=100.0,
        ))

    def test_invalid_or_stale_yaw_releases_instead_of_latching_stop(self):
        for control in ({}, self.control(float("nan")), self.control(10)):
            with self.subTest(control=control):
                self.assertTrue(camera_line_frame.green_maneuver_angle_limit_reached(
                    0, control, now=100.3,
                ))

    def test_angle_release_resumes_normal_fusion_without_repeating_marker(self):
        released = camera_line_frame.update_green_maneuver_state(
            "DIREITA", 12, raw_line_visible=True,
            completed=camera_line_frame.green_maneuver_angle_limit_reached(
                0, self.control(45), now=100.0,
            ),
        )
        result = calculate_command(
            sensor_values(), green_direction=released["direction"],
            fusion_style_line=GreenFusionTargetHoldTests.valid_fusion_line("DIREITA"),
        )
        self.assertEqual(result["controlSource"], "fusion")
        self.assertEqual(result["lineState"], "LINE")
        self.assertNotEqual((result["left_power"], result["right_power"]), (0.0, 0.0))
        self.assertIsNone(camera_line_frame.select_confirmed_green_direction(
            False, released["direction"],
            {"greenConfirmed": True, "greenInterpretation": "DIREITA"},
        ))


class GreenTimeoutAndBlindSearchTests(unittest.TestCase):
    def test_green_completion_waits_for_minimum_active_window(self):
        self.assertEqual(
            camera_line_frame.GREEN_MIN_ACTIVE_FRAMES_BEFORE_COMPLETION,
            24,
        )

    def test_green_entry_cannot_start_inside_initial_pivot_window(self):
        self.assertFalse(
            camera_line_frame.green_maneuver_entry_is_confirmed(
                "DIREITA",
                active_frames=8,
                near_fine_position=0.80,
            )
        )
        self.assertTrue(
            camera_line_frame.green_maneuver_entry_is_confirmed(
                "DIREITA",
                active_frames=9,
                near_fine_position=0.80,
            )
        )

    def test_green_geometric_completion_requires_three_consecutive_frames(self):
        completion_frames = 0
        for _frame_index in range(
            camera_line_frame.QUADROS_CENTRALIZADO_PARA_CONCLUIR
        ):
            geometrically_complete = (
                camera_line_frame.green_maneuver_is_geometrically_complete(
                    True,
                    near_fine_position=0.10,
                    medium_trusted=True,
                    medium_position=0.20,
                    far_trusted=False,
                    far_position=None,
                )
            )
            completion_frames = (
                completion_frames + 1
                if geometrically_complete
                else 0
            )

        self.assertEqual(
            camera_line_frame.QUADROS_CENTRALIZADO_PARA_CONCLUIR,
            3,
        )
        self.assertEqual(completion_frames, 3)
        completed = camera_line_frame.update_green_maneuver_state(
            "ESQUERDA",
            active_frames=10,
            raw_line_visible=True,
            completed=(
                completion_frames
                >= camera_line_frame.QUADROS_CENTRALIZADO_PARA_CONCLUIR
            ),
        )
        self.assertEqual(completed["direction"], "NENHUMA")
        self.assertFalse(completed["timedOut"])

    def test_green_geometric_completion_accepts_far_when_medium_is_untrusted(self):
        self.assertTrue(
            camera_line_frame.green_maneuver_is_geometrically_complete(
                True,
                near_fine_position=-0.10,
                medium_trusted=False,
                medium_position=None,
                far_trusted=True,
                far_position=-0.30,
            )
        )

    def test_green_near_alone_does_not_complete_maneuver(self):
        self.assertFalse(
            camera_line_frame.green_maneuver_is_geometrically_complete(
                True,
                near_fine_position=0.0,
                medium_trusted=False,
                medium_position=None,
                far_trusted=False,
                far_position=None,
            )
        )

    def test_green_still_finishes_normally(self):
        result = camera_line_frame.update_green_maneuver_state(
            "ESQUERDA",
            12,
            raw_line_visible=False,
            completed=True,
        )
        self.assertEqual(result["direction"], "NENHUMA")
        self.assertFalse(result["timedOut"])
        self.assertIsNone(result["searchDirection"])

    def test_green_timeout_releases_direction_at_75_frame_safety_limit(self):
        self.assertEqual(
            camera_line_frame.GREEN_MANEUVER_TIMEOUT_FRAMES,
            75,
        )
        active = camera_line_frame.update_green_maneuver_state(
            "DIREITA",
            active_frames=73,
            raw_line_visible=False,
        )
        self.assertEqual(active["direction"], "DIREITA")
        self.assertEqual(active["activeFrames"], 74)
        self.assertFalse(active["timedOut"])

        timed_out = camera_line_frame.update_green_maneuver_state(
            active["direction"],
            active_frames=active["activeFrames"],
            raw_line_visible=False,
        )
        self.assertEqual(timed_out["direction"], "NENHUMA")
        self.assertEqual(timed_out["activeFrames"], 0)
        self.assertTrue(timed_out["timedOut"])
        self.assertIsNone(timed_out["searchDirection"])

    def test_green_timeout_returns_control_to_normal_fusion(self):
        timeout = camera_line_frame.update_green_maneuver_state(
            "DIREITA",
            camera_line_frame.GREEN_MANEUVER_TIMEOUT_FRAMES - 1,
            raw_line_visible=True,
        )
        self.assertEqual(timeout["direction"], "NENHUMA")
        self.assertTrue(timeout["timedOut"])
        self.assertIsNone(timeout["searchDirection"])

        result = calculate_command(
            sensor_values(),
            green_direction=timeout["direction"],
            green_active_frames=timeout["activeFrames"],
            fusion_style_line=GreenFusionTargetHoldTests.valid_fusion_line("DIREITA"),
        )
        self.assertEqual(result["controlSource"], "fusion")
        self.assertNotEqual(
            (result["left_power"], result["right_power"]),
            (0.0, 0.0),
        )

    def test_green_timeout_without_line_releases_both_directions(self):
        for green_direction in ("ESQUERDA", "DIREITA"):
            with self.subTest(green_direction=green_direction):
                timeout = camera_line_frame.update_green_maneuver_state(
                    green_direction,
                    camera_line_frame.GREEN_MANEUVER_TIMEOUT_FRAMES - 1,
                    raw_line_visible=False,
                )
                self.assertEqual(timeout["direction"], "NENHUMA")
                self.assertIsNone(timeout["searchDirection"])
                result = calculate_command(
                    sensor_values(None, None, None),
                    green_direction=timeout["direction"],
                    green_active_frames=timeout["activeFrames"],
                )
                self.assertEqual(result["lineState"], "LINE")
                self.assertEqual(
                    (result["left_power"], result["right_power"]),
                    (0.0, 0.0),
                )

    def test_blind_search_reverses_after_short_initial_window(self):
        self.assertEqual(
            camera_line_frame.VIRTUAL_BLIND_SEARCH_INITIAL_FRAMES,
            35,
        )
        self.assertEqual(
            camera_line_frame.VIRTUAL_BLIND_SEARCH_REVERSE_FRAMES,
            50,
        )
        search_tracker = camera_line_frame.VirtualLineSearchTracker()
        search_tracker.start("LEFT")
        initial_directions = [
            search_tracker.next_direction()
            for _ in range(camera_line_frame.VIRTUAL_BLIND_SEARCH_INITIAL_FRAMES)
        ]
        self.assertEqual(
            initial_directions,
            ["LEFT"] * camera_line_frame.VIRTUAL_BLIND_SEARCH_INITIAL_FRAMES,
        )
        reverse_directions = [
            search_tracker.next_direction()
            for _ in range(
                camera_line_frame.VIRTUAL_BLIND_SEARCH_REVERSE_FRAMES
            )
        ]
        self.assertEqual(
            reverse_directions,
            ["RIGHT"]
            * camera_line_frame.VIRTUAL_BLIND_SEARCH_REVERSE_FRAMES,
        )
        self.assertEqual(search_tracker.next_direction(), "LEFT")

    def test_any_sensor_interrupts_blind_search(self):
        cases = []
        near_sensors = sensor_values(None, None, None)
        near_sensors["nearCenter"] = 0.20
        near_sensors["nearFinePosition"] = -0.80
        cases.append(("NEAR-C", near_sensors, (0.75, 0.75)))
        cases.append(("MEDIUM", sensor_values(None, 0.80, None), (0.75, 0.0)))
        cases.append(("FAR BAND", sensor_values(None, None, 0.80), (0.75, 0.0)))
        cases.append(("MEDIUM CENTER", sensor_values(None, 0.0, None), (0.75, 0.75)))

        for row_name, sensors, expected_powers in cases:
            with self.subTest(row=row_name):
                search_tracker = camera_line_frame.VirtualLineSearchTracker()
                search_tracker.start("LEFT")
                calculate_command(
                    sensor_values(None, None, None),
                    line_search_tracker=search_tracker,
                )
                result = calculate_command(
                    sensors,
                    line_search_tracker=search_tracker,
                )
                self.assertFalse(search_tracker.active)
                self.assertEqual(
                    result["controlSource"],
                    "virtual-sensor-recovery",
                )
                self.assertEqual(
                    (result["left_power"], result["right_power"]),
                    expected_powers,
                )


if __name__ == "__main__":
    unittest.main()
