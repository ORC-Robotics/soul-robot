"""Testes determinísticos do tracker virtual sem acessar câmera ou motores."""

import importlib.util
import math
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np


MODULE_PATH = Path(__file__).with_name("camera_line_frame.py")
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
):
    """Cria uma leitura completa sem depender da câmera."""

    return {
        "farLeft": 0.10,
        "farCenter": 0.40,
        "farRight": 0.10,
        "farPosition": 0.0,
        "farBandLeft": 0.10,
        "farBandCenter": 0.10,
        "farBandRight": 0.10,
        "farBandPosition": far_band_position,
        "mediumLeft": 0.10,
        "mediumCenter": 0.20,
        "mediumRight": 0.10,
        "mediumPosition": medium_position,
        "nearLeft": 0.10,
        "nearCenter": 0.20,
        "nearRight": 0.10,
        "nearPosition": None if steering_error is None else 0.0,
        "headingAngle": 0.0,
        "steeringError": steering_error,
    }


def calculate_command(
    sensors,
    tracker=None,
    pivot_state_tracker=None,
    green_direction="NENHUMA",
    gap_active=False,
    line_search_tracker=None,
    blind_search_requested=False,
    sensor_recovery_requested=False,
    centerline_guidance=None,
    centerline_decisions=None,
    curve_intent_tracker=None,
):
    """Executa somente o controle virtual com leituras determinísticas."""

    mask = np.zeros((10, 10), dtype=np.uint8)
    with patch.object(
        camera_line_frame,
        "read_virtual_line_sensors",
        return_value=sensors,
    ):
        return camera_line_frame.calculate_line_follower_command(
            mask,
            {},
            centerline_guidance,
            direcao_verde_ativa=green_direction,
            gap_forward_active=gap_active,
            centerline_decisions=centerline_decisions,
            virtual_turn_tracker=tracker,
            curve_intent_tracker=curve_intent_tracker,
            pivot_state_tracker=pivot_state_tracker,
            line_search_tracker=line_search_tracker,
            blind_search_requested=blind_search_requested,
            sensor_recovery_requested=sensor_recovery_requested,
        )


def expected_normal_motor_powers(steering_error):
    """Replica a transição progressiva do mapper normal até o pivot."""

    if steering_error is None:
        return 0.0, 0.0
    if steering_error >= camera_line_frame.PIVOT_ENTER_THRESHOLD:
        return 0.85, 0.0
    if steering_error <= -camera_line_frame.PIVOT_ENTER_THRESHOLD:
        return 0.0, 0.85

    steering_magnitude = abs(steering_error)
    if steering_magnitude >= camera_line_frame.NORMAL_FULL_STEERING_ERROR:
        transition_progress = (
            steering_magnitude
            - camera_line_frame.NORMAL_FULL_STEERING_ERROR
        ) / (
            camera_line_frame.PIVOT_ENTER_THRESHOLD
            - camera_line_frame.NORMAL_FULL_STEERING_ERROR
        )
        transition_progress = max(0.0, min(1.0, transition_progress))
        transition_progress *= transition_progress
        outer_power = 0.75 + transition_progress * (0.85 - 0.75)
        inner_power = 0.66 - transition_progress * (0.66 - 0.61)
    else:
        steering_strength = (
            steering_magnitude
            / camera_line_frame.NORMAL_FULL_STEERING_ERROR
        )
        outer_power = 0.69 + steering_strength * (0.75 - 0.69)
        inner_power = 0.69 - steering_strength * (0.69 - 0.66)
    if steering_error > 0.0:
        return outer_power, inner_power
    return inner_power, outer_power


class VirtualSensorRegressionTests(unittest.TestCase):
    def test_opposite_heading_cannot_invert_near_without_medium(self):
        steering_error = camera_line_frame.protect_virtual_near_direction(
            near_position=0.23,
            medium_position=-0.06,
            heading_angle=-14.7,
            steering_error=-0.26,
        )
        self.assertTrue(math.isclose(steering_error, 0.23))

    def test_strong_medium_allows_heading_to_invert_near(self):
        steering_error = camera_line_frame.protect_virtual_near_direction(
            near_position=0.23,
            medium_position=-0.25,
            heading_angle=-14.7,
            steering_error=-0.26,
        )
        self.assertTrue(math.isclose(steering_error, -0.26))

    def test_legacy_geometry_uses_original_direct_roi(self):
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
            geometry["near"]["left"]["y0"],
            round(101 * camera_line_frame.VIRTUAL_NEAR_Y0),
        )
        self.assertEqual(
            geometry["near"]["left"]["y1"],
            round(101 * camera_line_frame.VIRTUAL_NEAR_Y1),
        )
        self.assertEqual(geometry["farBand"]["left"]["y1"], round(101 * 0.27))
        self.assertEqual(geometry["medium"]["left"]["y0"], round(101 * 0.27))

    def test_legacy_outputs_match_direct_six_sensor_reference(self):
        random_generator = np.random.default_rng(2026)
        mask = (
            random_generator.random((101, 203)) > 0.72
        ).astype(np.uint8) * 255
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(mask.shape)

        for green_direction in ("NENHUMA", "ESQUERDA", "DIREITA"):
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
            near_left = camera_line_frame.read_virtual_sensor(
                mask, geometry["near"]["left"]
            )
            near_center = camera_line_frame.read_virtual_sensor(
                mask, geometry["near"]["center"]
            )
            near_right = camera_line_frame.read_virtual_sensor(
                mask, geometry["near"]["right"]
            )
            raw_far = (far_left, far_center, far_right)
            raw_near = (near_left, near_center, near_right)
            if green_direction == "ESQUERDA":
                far_center = 0.0
                far_right = 0.0
                near_right = 0.0
            elif green_direction == "DIREITA":
                far_left = 0.0
                far_center = 0.0
                near_left = 0.0

            far_position = camera_line_frame.calculate_virtual_row_position(
                far_left, far_center, far_right
            )
            near_position = camera_line_frame.calculate_virtual_row_position(
                near_left, near_center, near_right
            )
            heading_angle = camera_line_frame.calculate_virtual_heading_angle(
                far_position,
                near_position,
                geometry,
            )
            steering_error = camera_line_frame.calculate_virtual_steering_error(
                near_position,
                heading_angle,
            )
            expected = {
                "farLeft": raw_far[0],
                "farCenter": raw_far[1],
                "farRight": raw_far[2],
                "farPosition": far_position,
                "nearLeft": raw_near[0],
                "nearCenter": raw_near[1],
                "nearRight": raw_near[2],
                "nearPosition": near_position,
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


class DynamicCenterlineLookaheadTests(unittest.TestCase):
    def path_with_turn(self, angle_deg):
        """Cria um path longo com trecho local reto e curva de ângulo conhecido."""

        start = (320.0, 400.0)
        corner = (320.0, 280.0)
        turn_length_px = 220.0
        angle_rad = math.radians(angle_deg)
        end = (
            corner[0] + math.sin(angle_rad) * turn_length_px,
            corner[1] - math.cos(angle_rad) * turn_length_px,
        )
        return [start, corner, end]

    def assert_point_on_path(self, point, path, tolerance=1e-6):
        """Confirma que o target pertence a algum segmento da polilinha."""

        point_x = float(point[0])
        point_y = float(point[1])
        for segment_start, segment_end in zip(path, path[1:]):
            start_x = float(segment_start[0])
            start_y = float(segment_start[1])
            segment_x = float(segment_end[0]) - start_x
            segment_y = float(segment_end[1]) - start_y
            segment_squared = segment_x * segment_x + segment_y * segment_y
            if segment_squared <= 0.0:
                continue
            projection = (
                (point_x - start_x) * segment_x
                + (point_y - start_y) * segment_y
            ) / segment_squared
            projection = max(0.0, min(1.0, projection))
            closest_x = start_x + projection * segment_x
            closest_y = start_y + projection * segment_y
            if math.hypot(point_x - closest_x, point_y - closest_y) <= tolerance:
                return
        self.fail(f"Target {point} não pertence ao path {path}")

    def test_straight_path_keeps_target_at_minimum_distance(self):
        guidance = camera_line_frame.calculate_geometric_centerline_guidance(
            self.path_with_turn(0.0),
            640,
        )
        self.assertTrue(math.isclose(guidance["turnAheadDeg"], 0.0))
        self.assertTrue(math.isclose(
            guidance["dynamicLookaheadDistancePx"],
            camera_line_frame.LOOKAHEAD_MIN_PX,
        ))

    def test_moderate_curve_uses_intermediate_distance(self):
        guidance = camera_line_frame.calculate_geometric_centerline_guidance(
            self.path_with_turn(30.0),
            640,
        )
        expected_distance = (
            camera_line_frame.LOOKAHEAD_MIN_PX
            + (
                camera_line_frame.LOOKAHEAD_MAX_PX
                - camera_line_frame.LOOKAHEAD_MIN_PX
            ) * 0.5
        )
        self.assertTrue(math.isclose(
            guidance["turnAheadDeg"],
            30.0,
            abs_tol=1e-6,
        ))
        self.assertTrue(math.isclose(
            guidance["dynamicLookaheadDistancePx"],
            expected_distance,
            abs_tol=1e-6,
        ))

    def test_strong_curve_reaches_maximum_distance(self):
        guidance = camera_line_frame.calculate_geometric_centerline_guidance(
            self.path_with_turn(60.0),
            640,
        )
        self.assertTrue(math.isclose(
            guidance["dynamicLookaheadDistancePx"],
            camera_line_frame.LOOKAHEAD_MAX_PX,
            abs_tol=1e-6,
        ))

    def test_left_and_right_dynamic_targets_are_symmetric(self):
        left = camera_line_frame.calculate_geometric_centerline_guidance(
            self.path_with_turn(-45.0),
            640,
        )
        right = camera_line_frame.calculate_geometric_centerline_guidance(
            self.path_with_turn(45.0),
            640,
        )
        self.assertTrue(math.isclose(
            left["dynamicLookaheadDistancePx"],
            right["dynamicLookaheadDistancePx"],
        ))
        self.assertTrue(math.isclose(
            left["dynamicTargetPoint"][0] - 320.0,
            -(right["dynamicTargetPoint"][0] - 320.0),
        ))
        self.assertTrue(math.isclose(
            left["targetAngleDeg"],
            -right["targetAngleDeg"],
        ))

    def test_dynamic_target_never_leaves_selected_path(self):
        paths = [
            self.path_with_turn(angle_deg)
            for angle_deg in (-90.0, -30.0, 0.0, 30.0, 90.0)
        ]
        paths.append([(320.0, 400.0), (320.0, 350.0)])
        for path in paths:
            with self.subTest(path=path):
                guidance = (
                    camera_line_frame.calculate_geometric_centerline_guidance(
                        path,
                        640,
                    )
                )
                self.assertLessEqual(
                    guidance["dynamicLookaheadDistancePx"],
                    guidance["pathLengthPx"],
                )
                self.assert_point_on_path(
                    guidance["dynamicTargetPoint"],
                    path,
                )

    def test_guidance_does_not_change_branch_selection(self):
        straight_path = [
            (50, 100),
            (50, 90),
            (50, 80),
            (50, 70),
            (50, 60),
        ]
        right_path = [
            (50, 100),
            (50, 90),
            (60, 90),
            (70, 90),
            (80, 90),
        ]
        candidates = [straight_path, right_path]
        selected_before = (
            camera_line_frame.select_geometric_centerline_branch_path(
                candidates,
            )
        )
        camera_line_frame.calculate_geometric_centerline_guidance(
            selected_before,
            100,
        )
        selected_after = (
            camera_line_frame.select_geometric_centerline_branch_path(
                candidates,
            )
        )
        self.assertEqual(selected_before, straight_path)
        self.assertEqual(selected_after, selected_before)
        self.assertEqual(candidates, [straight_path, right_path])

    def test_supported_ninety_degree_curve_keeps_virtual_control(self):
        path = self.path_with_turn(90.0)
        original_path = list(path)
        guidance = camera_line_frame.calculate_geometric_centerline_guidance(
            path,
            640,
        )
        sensors = sensor_values(0.30, None, None)
        mask = np.zeros((10, 10), dtype=np.uint8)
        with patch.object(
            camera_line_frame,
            "read_virtual_line_sensors",
            return_value=sensors,
        ):
            command = camera_line_frame.calculate_line_follower_command(
                mask,
                {},
                guidance,
                virtual_turn_tracker=(
                    camera_line_frame.VirtualTurnStateTracker()
                ),
            )

        self.assertEqual(path, original_path)
        self.assertTrue(math.isclose(
            guidance["dynamicLookaheadDistancePx"],
            camera_line_frame.LOOKAHEAD_MAX_PX,
        ))
        self.assertEqual(command["controlSource"], "virtual")
        self.assertTrue(math.isclose(command["finalSteering"], 0.30))
        self.assertEqual(
            (command["left_power"], command["right_power"]),
            (0.75, 0.66),
        )

    def test_experimental_hard_reorient_is_completely_absent(self):
        source = MODULE_PATH.read_text(encoding="utf-8")
        for forbidden_reference in (
            "HARD_REORIENT",
            "HardCurveReorient",
            "hard_reorient",
            "hardReorient",
        ):
            self.assertNotIn(forbidden_reference, source)


class FarPathConsensusTests(unittest.TestCase):
    def extreme_path(self, direction):
        """Cria uma curva de 90 graus longa e simétrica."""

        sign = -1.0 if direction == "LEFT" else 1.0
        return [
            (320.0, 400.0),
            (320.0, 300.0),
            (320.0 + sign * 300.0, 300.0),
        ]

    def assert_point_on_path(self, point, path, tolerance=1e-6):
        """Confirma que o ponto interpolado permanece na polilinha."""

        point_x = float(point[0])
        point_y = float(point[1])
        for segment_start, segment_end in zip(path, path[1:]):
            start_x = float(segment_start[0])
            start_y = float(segment_start[1])
            segment_x = float(segment_end[0]) - start_x
            segment_y = float(segment_end[1]) - start_y
            segment_squared = segment_x * segment_x + segment_y * segment_y
            if segment_squared <= 0.0:
                continue
            projection = (
                (point_x - start_x) * segment_x
                + (point_y - start_y) * segment_y
            ) / segment_squared
            projection = max(0.0, min(1.0, projection))
            closest_x = start_x + projection * segment_x
            closest_y = start_y + projection * segment_y
            if math.hypot(point_x - closest_x, point_y - closest_y) <= tolerance:
                return
        self.fail(f"Ponto {point} não pertence ao path {path}")

    def test_median_accepts_stable_strong_right_curve(self):
        geometry = camera_line_frame.evaluate_far_path_angles(
            (42.0, 48.0, 53.0),
            False,
        )
        self.assertTrue(geometry["valid"])
        self.assertEqual(geometry["candidateDirection"], "RIGHT")
        self.assertTrue(math.isclose(geometry["medianAngleDeg"], 48.0))
        self.assertTrue(math.isclose(geometry["spreadDeg"], 11.0))

    def test_unstable_or_conflicting_geometry_is_rejected(self):
        cases = (
            (45.0, 8.0, 55.0),
            (40.0, -20.0, 48.0),
        )
        for angles in cases:
            with self.subTest(angles=angles):
                geometry = camera_line_frame.evaluate_far_path_angles(
                    angles,
                    False,
                )
                self.assertFalse(geometry["valid"])
                self.assertEqual(geometry["candidateDirection"], "NONE")

    def test_ambiguous_path_never_produces_consensus(self):
        geometry = camera_line_frame.evaluate_far_path_angles(
            (42.0, 48.0, 53.0),
            True,
        )
        self.assertFalse(geometry["valid"])
        self.assertEqual(geometry["candidateDirection"], "NONE")

    def test_same_side_requires_three_consecutive_frames(self):
        tracker = camera_line_frame.FarPathConsensusTracker()
        observed = [tracker.update("RIGHT") for _ in range(3)]
        self.assertEqual(observed, ["NONE", "NONE", "RIGHT"])
        self.assertEqual(tracker.confirmation_frames, 3)

    def test_dropout_and_opposite_side_do_not_switch_immediately(self):
        tracker = camera_line_frame.FarPathConsensusTracker()
        for _ in range(camera_line_frame.FAR_PATH_CONFIRM_FRAMES):
            tracker.update("RIGHT")

        self.assertEqual(tracker.update("NONE"), "NONE")
        self.assertEqual(tracker.update("LEFT"), "NONE")
        self.assertEqual(tracker.update("LEFT"), "NONE")
        self.assertEqual(tracker.update("LEFT"), "LEFT")

    def test_confirmed_extreme_curve_advances_target_without_endpoint(self):
        tracker = camera_line_frame.FarPathConsensusTracker()
        path = self.extreme_path("RIGHT")
        guidance_frames = []
        for _ in range(camera_line_frame.FAR_PATH_CONFIRM_FRAMES):
            guidance_frames.append(
                camera_line_frame.calculate_geometric_centerline_guidance(
                    path,
                    640,
                    centerline_decisions=[],
                    far_path_consensus_tracker=tracker,
                )
            )

        first, second, confirmed = guidance_frames
        self.assertEqual(first["farConsensusDirection"], "NONE")
        self.assertEqual(second["farConsensusDirection"], "NONE")
        self.assertEqual(confirmed["farConsensusDirection"], "RIGHT")
        self.assertGreater(
            confirmed["dynamicLookaheadDistancePx"],
            second["dynamicLookaheadDistancePx"],
        )
        self.assertLessEqual(
            confirmed["dynamicLookaheadDistancePx"],
            confirmed["pathLengthPx"]
            * camera_line_frame.FAR_PATH_TARGET_MAX_RATIO,
        )
        self.assertNotEqual(confirmed["dynamicTargetPoint"], list(path[-1]))
        self.assert_point_on_path(confirmed["dynamicTargetPoint"], path)
        self.assertEqual(len(confirmed["farPathSamplePoints"]), 3)
        for sample_point in confirmed["farPathSamplePoints"]:
            self.assert_point_on_path(sample_point, path)

    def test_strong_curve_without_consensus_keeps_current_target(self):
        path = self.extreme_path("RIGHT")
        baseline = camera_line_frame.calculate_geometric_centerline_guidance(
            path,
            640,
        )
        tracker = camera_line_frame.FarPathConsensusTracker()
        for _ in range(camera_line_frame.FAR_PATH_CONFIRM_FRAMES + 1):
            ambiguous = (
                camera_line_frame.calculate_geometric_centerline_guidance(
                    path,
                    640,
                    centerline_decisions=[{"ambiguous": True}],
                    far_path_consensus_tracker=tracker,
                )
            )
        self.assertEqual(ambiguous["farConsensusDirection"], "NONE")
        self.assertTrue(math.isclose(
            ambiguous["dynamicLookaheadDistancePx"],
            baseline["dynamicLookaheadDistancePx"],
        ))

    def test_left_and_right_consensus_are_symmetric(self):
        results = {}
        for direction in ("LEFT", "RIGHT"):
            tracker = camera_line_frame.FarPathConsensusTracker()
            path = self.extreme_path(direction)
            for _ in range(camera_line_frame.FAR_PATH_CONFIRM_FRAMES):
                results[direction] = (
                    camera_line_frame.calculate_geometric_centerline_guidance(
                        path,
                        640,
                        centerline_decisions=[],
                        far_path_consensus_tracker=tracker,
                    )
                )

        left = results["LEFT"]
        right = results["RIGHT"]
        self.assertEqual(left["farConsensusDirection"], "LEFT")
        self.assertEqual(right["farConsensusDirection"], "RIGHT")
        self.assertTrue(math.isclose(
            left["farPathAngleDeg"],
            -right["farPathAngleDeg"],
        ))
        self.assertTrue(math.isclose(
            left["dynamicLookaheadDistancePx"],
            right["dynamicLookaheadDistancePx"],
        ))
        self.assertTrue(math.isclose(
            left["dynamicTargetAngleDeg"],
            -right["dynamicTargetAngleDeg"],
        ))

    def test_consensus_does_not_steer_without_curve_intent(self):
        tracker = camera_line_frame.FarPathConsensusTracker()
        path = self.extreme_path("RIGHT")
        guidance = None
        for _ in range(camera_line_frame.FAR_PATH_CONFIRM_FRAMES):
            guidance = (
                camera_line_frame.calculate_geometric_centerline_guidance(
                    path,
                    640,
                    centerline_decisions=[],
                    far_path_consensus_tracker=tracker,
                )
            )

        sensors = sensor_values(0.18, 0.0, 0.0)
        without_intent = calculate_command(
            sensors,
            camera_line_frame.VirtualTurnStateTracker(),
            centerline_guidance=guidance,
            centerline_decisions=[],
        )
        self.assertTrue(math.isclose(without_intent["finalSteering"], 0.18))

        curve_tracker = camera_line_frame.CurveIntentTracker()
        with_intent = None
        for _ in range(camera_line_frame.CURVE_INTENT_CONFIRM_FRAMES):
            with_intent = calculate_command(
                sensors,
                camera_line_frame.VirtualTurnStateTracker(),
                centerline_guidance=guidance,
                centerline_decisions=[],
                curve_intent_tracker=curve_tracker,
            )
        self.assertEqual(with_intent["curveIntentState"], "RIGHT")
        self.assertTrue(math.isclose(
            with_intent["finalSteering"],
            camera_line_frame.CURVE_INTENT_MIN_STEERING,
        ))

    def test_consensus_guidance_does_not_change_branch_selection(self):
        candidates = [
            self.extreme_path("LEFT"),
            self.extreme_path("RIGHT"),
        ]
        original_candidates = [list(path) for path in candidates]
        selected_before = (
            camera_line_frame.select_geometric_centerline_branch_path(
                candidates,
            )
        )
        tracker = camera_line_frame.FarPathConsensusTracker()
        for _ in range(camera_line_frame.FAR_PATH_CONFIRM_FRAMES):
            camera_line_frame.calculate_geometric_centerline_guidance(
                selected_before,
                640,
                centerline_decisions=[],
                far_path_consensus_tracker=tracker,
            )
        selected_after = (
            camera_line_frame.select_geometric_centerline_branch_path(
                candidates,
            )
        )
        self.assertEqual(selected_after, selected_before)
        self.assertEqual(candidates, original_candidates)

    def test_endpoint_angle_is_not_a_control_field(self):
        source = MODULE_PATH.read_text(encoding="utf-8")
        self.assertNotIn("endpointTargetAngleDeg", source)


class CurveIntentTests(unittest.TestCase):
    def guidance(self, direction, target_angle_deg=30.0, turn_ahead_deg=25.0):
        """Cria somente a geometria necessária para validar o Curve Intent."""

        sign = -1.0 if direction == "LEFT" else 1.0
        return {
            "dynamicTargetAngleDeg": sign * target_angle_deg,
            "targetAngleDeg": sign * target_angle_deg,
            "turnAheadDeg": sign * turn_ahead_deg,
        }

    def activate(self, direction, steering_magnitude=0.18):
        """Confirma o intent e devolve os objetos usados pelo teste."""

        sign = -1.0 if direction == "LEFT" else 1.0
        tracker = camera_line_frame.CurveIntentTracker()
        sensors = sensor_values(sign * steering_magnitude, 0.0, 0.0)
        guidance = self.guidance(direction)
        result = None
        for _ in range(camera_line_frame.CURVE_INTENT_CONFIRM_FRAMES):
            result = calculate_command(
                sensors,
                camera_line_frame.VirtualTurnStateTracker(),
                centerline_guidance=guidance,
                centerline_decisions=[],
                curve_intent_tracker=tracker,
            )
        return tracker, sensors, guidance, result

    def test_strong_curve_activates_after_two_frames(self):
        tracker = camera_line_frame.CurveIntentTracker()
        sensors = sensor_values(0.18, 0.0, 0.0)
        guidance = self.guidance("RIGHT")

        first = calculate_command(
            sensors,
            camera_line_frame.VirtualTurnStateTracker(),
            centerline_guidance=guidance,
            centerline_decisions=[],
            curve_intent_tracker=tracker,
        )
        self.assertEqual(first["curveIntentState"], "NONE")
        self.assertTrue(math.isclose(first["finalSteering"], 0.18))

        second = calculate_command(
            sensors,
            camera_line_frame.VirtualTurnStateTracker(),
            centerline_guidance=guidance,
            centerline_decisions=[],
            curve_intent_tracker=tracker,
        )
        self.assertEqual(second["curveIntentState"], "RIGHT")
        self.assertTrue(second["curveIntentApplied"])

    def test_curve_diagnostics_expose_existing_values_and_tracker_counts(self):
        tracker = camera_line_frame.CurveIntentTracker()
        guidance = self.guidance("RIGHT")
        guidance.update({
            "farPathSampleAnglesDeg": [36.0, 48.0, 61.0],
            "farPathAngleSpreadDeg": 25.0,
            "farPathAngleDeg": 48.0,
            "farConsensusDirection": "RIGHT",
            "farConsensusConfirmFrames": 2,
            "dynamicLookaheadDistancePx": 72.0,
            "pathAmbiguous": False,
        })
        sensors = sensor_values(0.18, 0.10, 0.20)

        first = calculate_command(
            sensors,
            camera_line_frame.VirtualTurnStateTracker(),
            centerline_guidance=guidance,
            centerline_decisions=[],
            curve_intent_tracker=tracker,
        )
        second = calculate_command(
            sensors,
            camera_line_frame.VirtualTurnStateTracker(),
            centerline_guidance=guidance,
            centerline_decisions=[],
            curve_intent_tracker=tracker,
        )

        self.assertEqual(
            (first["farAngle60"], first["farAngle75"], first["farAngle90"]),
            (36.0, 48.0, 61.0),
        )
        self.assertEqual(first["curveIntentConfirmFrames"], 1)
        self.assertEqual(second["curveIntentConfirmFrames"], 2)
        self.assertTrue(math.isclose(first["hybridSteering"], 0.18))
        self.assertTrue(math.isclose(first["finalSteering"], 0.18))
        self.assertTrue(math.isclose(second["hybridSteering"], 0.18))
        self.assertTrue(math.isclose(
            second["finalSteering"],
            camera_line_frame.CURVE_INTENT_MIN_STEERING,
        ))
        self.assertEqual(second["dynamicLookaheadPx"], 72.0)
        self.assertFalse(second["pathAmbiguous"])
        self.assertEqual(second["lineState"], "LINE")

    def test_diagnostic_line_state_excludes_green_and_gap(self):
        sensors = sensor_values(0.18, 0.0, 0.0)
        green = calculate_command(sensors, green_direction="DIREITA")
        gap = calculate_command(sensors, gap_active=True)

        self.assertEqual(green["lineState"], "GREEN")
        self.assertEqual(gap["lineState"], "GAP")

    def test_straight_or_ambiguous_path_never_activates(self):
        cases = (
            ("STRAIGHT", self.guidance("RIGHT", 0.0, 0.0), []),
            (
                "AMBIGUOUS",
                self.guidance("RIGHT"),
                [{"ambiguous": True}],
            ),
        )
        for case_name, guidance, decisions in cases:
            with self.subTest(case=case_name):
                tracker = camera_line_frame.CurveIntentTracker()
                for _ in range(camera_line_frame.CURVE_INTENT_CONFIRM_FRAMES + 2):
                    result = calculate_command(
                        sensor_values(0.18, 0.0, 0.0),
                        camera_line_frame.VirtualTurnStateTracker(),
                        centerline_guidance=guidance,
                        centerline_decisions=decisions,
                        curve_intent_tracker=tracker,
                    )
                self.assertEqual(result["curveIntentState"], "NONE")
                self.assertTrue(math.isclose(result["finalSteering"], 0.18))

    def test_intent_keeps_minimum_steering(self):
        _tracker, _sensors, _guidance, result = self.activate("RIGHT")
        self.assertTrue(math.isclose(
            result["finalSteering"],
            camera_line_frame.CURVE_INTENT_MIN_STEERING,
        ))

    def test_stronger_normal_steering_keeps_precedence(self):
        _tracker, _sensors, _guidance, result = self.activate(
            "RIGHT",
            steering_magnitude=0.65,
        )
        self.assertTrue(math.isclose(result["finalSteering"], 0.65))
        self.assertFalse(result["curveIntentApplied"])

    def test_intent_releases_after_two_aligned_frames(self):
        tracker, sensors, _guidance, _result = self.activate("RIGHT")
        aligned_guidance = self.guidance(
            "RIGHT",
            target_angle_deg=5.0,
            turn_ahead_deg=5.0,
        )

        first = calculate_command(
            sensors,
            camera_line_frame.VirtualTurnStateTracker(),
            centerline_guidance=aligned_guidance,
            centerline_decisions=[],
            curve_intent_tracker=tracker,
        )
        self.assertEqual(first["curveIntentState"], "RIGHT")
        self.assertTrue(math.isclose(first["finalSteering"], 0.30))

        second = calculate_command(
            sensors,
            camera_line_frame.VirtualTurnStateTracker(),
            centerline_guidance=aligned_guidance,
            centerline_decisions=[],
            curve_intent_tracker=tracker,
        )
        self.assertEqual(second["curveIntentState"], "NONE")
        self.assertTrue(math.isclose(second["finalSteering"], 0.18))

    def test_strong_opposite_near_blocks_intent_without_medium(self):
        tracker, _sensors, guidance, _result = self.activate("RIGHT")
        conflicting_sensors = sensor_values(-0.25, 0.0, 0.0)
        conflicting_sensors["nearPosition"] = -0.25
        result = calculate_command(
            conflicting_sensors,
            camera_line_frame.VirtualTurnStateTracker(),
            centerline_guidance=guidance,
            centerline_decisions=[],
            curve_intent_tracker=tracker,
        )
        self.assertEqual(result["curveIntentState"], "RIGHT")
        self.assertFalse(result["curveIntentApplied"])
        self.assertTrue(math.isclose(result["finalSteering"], -0.25))

    def test_left_and_right_are_symmetric(self):
        results = {}
        for direction in ("LEFT", "RIGHT"):
            _tracker, _sensors, _guidance, results[direction] = (
                self.activate(direction)
            )
        self.assertTrue(math.isclose(
            results["LEFT"]["finalSteering"],
            -results["RIGHT"]["finalSteering"],
        ))
        self.assertTrue(math.isclose(
            results["LEFT"]["left_power"],
            results["RIGHT"]["right_power"],
        ))
        self.assertTrue(math.isclose(
            results["LEFT"]["right_power"],
            results["RIGHT"]["left_power"],
        ))

    def test_mapper_and_original_reorient_remain_unchanged(self):
        sensors = sensor_values(0.35, 0.0, 0.0)
        baseline = calculate_command(
            sensors,
            camera_line_frame.VirtualTurnStateTracker(),
        )
        with_curve_intent = calculate_command(
            sensors,
            camera_line_frame.VirtualTurnStateTracker(),
            centerline_guidance=self.guidance("RIGHT", 0.0, 0.0),
            centerline_decisions=[],
            curve_intent_tracker=camera_line_frame.CurveIntentTracker(),
        )
        self.assertEqual(
            (with_curve_intent["left_power"], with_curve_intent["right_power"]),
            (baseline["left_power"], baseline["right_power"]),
        )

        recovery_tracker = camera_line_frame.VirtualTurnStateTracker()
        curve_tracker = camera_line_frame.CurveIntentTracker()
        lost_sensors = sensor_values(None, 0.80, 0.80)
        for _ in range(camera_line_frame.VIRTUAL_REORIENT_CONFIRMATION_FRAMES):
            recovery_result = calculate_command(
                lost_sensors,
                recovery_tracker,
                centerline_guidance=self.guidance("RIGHT"),
                centerline_decisions=[],
                curve_intent_tracker=curve_tracker,
            )
        self.assertEqual(
            recovery_tracker.state,
            camera_line_frame.VIRTUAL_STATE_REORIENT_RIGHT,
        )
        self.assertEqual(curve_tracker.state, camera_line_frame.CURVE_INTENT_NONE)
        self.assertEqual(recovery_result["controlSource"], "virtual-reorient")


class VirtualRecoveryTests(unittest.TestCase):
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
        self.assertEqual((result["left_power"], result["right_power"]), (0.0, 0.69))
        self.assertEqual(result["controlSource"], "virtual-medium-scan")

    def test_invalid_steering_uses_right_medium_scan(self):
        tracker = camera_line_frame.VirtualTurnStateTracker()
        result = calculate_command(sensor_values(None, 0.80, None), tracker)
        self.assertIsNone(result["finalSteering"])
        self.assertEqual((result["left_power"], result["right_power"]), (0.69, 0.0))
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
            self.assertEqual((result["left_power"], result["right_power"]), (0.69, 0.0))

        result = calculate_command(sensors, tracker)
        self.assertEqual(result["controlSource"], "virtual-no-line")
        self.assertEqual((result["left_power"], result["right_power"]), (0.0, 0.0))

    def test_matching_left_recovery_enters_reorient(self):
        tracker, result = self.enter_reorient("LEFT")
        self.assertEqual(tracker.state, camera_line_frame.VIRTUAL_STATE_REORIENT_LEFT)
        self.assertEqual(result["finalSteering"], -1.0)
        self.assertEqual((result["left_power"], result["right_power"]), (0.0, 0.85))

    def test_matching_right_recovery_enters_reorient(self):
        tracker, result = self.enter_reorient("RIGHT")
        self.assertEqual(tracker.state, camera_line_frame.VIRTUAL_STATE_REORIENT_RIGHT)
        self.assertEqual(result["finalSteering"], 1.0)
        self.assertEqual((result["left_power"], result["right_power"]), (0.85, 0.0))

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
        self.assertIsNone(result["finalSteering"])
        self.assertEqual(result["controlSource"], "virtual-green")
        self.assertEqual(result["greenDirection"], "DIREITA")

    def test_gap_uses_medium_and_far_band_lateral_recovery(self):
        cases = (
            (sensor_values(None, -0.80, None), "LEFT", (0.0, 0.69)),
            (sensor_values(None, None, 0.80), "RIGHT", (0.69, 0.0)),
        )
        for sensors, _direction, expected_powers in cases:
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
                    "gap-sensor-recovery",
                )
                self.assertEqual(
                    (result["left_power"], result["right_power"]),
                    expected_powers,
                )

    def test_gap_recovery_prioritizes_near_over_medium_and_far(self):
        sensors = sensor_values(None, 0.80, 0.80)
        sensors["rawNearPosition"] = -0.80
        result = calculate_command(sensors, gap_active=True)
        self.assertEqual(result["controlSource"], "gap-sensor-recovery")
        self.assertEqual((result["left_power"], result["right_power"]), (0.0, 0.69))

    def test_gap_only_ends_after_confirmed_near_reacquisition(self):
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

    def test_green_right_preserves_raw_near_left(self):
        mask = np.zeros((100, 100), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(mask.shape)
        near_left = geometry["near"]["left"]
        mask[near_left["y0"]:near_left["y1"], near_left["x0"]:near_left["x1"]] = 255
        sensors = camera_line_frame.read_virtual_line_sensors(mask, "DIREITA")
        self.assertGreater(sensors["nearLeft"], 0.0)

    def test_green_left_preserves_raw_near_right(self):
        mask = np.zeros((100, 100), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(mask.shape)
        near_right = geometry["near"]["right"]
        mask[near_right["y0"]:near_right["y1"], near_right["x0"]:near_right["x1"]] = 255
        sensors = camera_line_frame.read_virtual_line_sensors(mask, "ESQUERDA")
        self.assertGreater(sensors["nearRight"], 0.0)

    def test_normal_mapper_below_point_thirty_is_unchanged(self):
        for steering_error, expected_powers in (
            (0.20, (0.73, 0.67)),
            (-0.20, (0.67, 0.73)),
        ):
            result = calculate_command(
                sensor_values(steering_error, None, None),
                camera_line_frame.VirtualTurnStateTracker(),
            )
            self.assertTrue(math.isclose(
                result["left_power"], expected_powers[0]
            ))
            self.assertTrue(math.isclose(
                result["right_power"], expected_powers[1]
            ))

    def test_normal_mapper_is_continuous_at_point_thirty(self):
        before_transition = calculate_command(
            sensor_values(0.30 - 1e-9, None, None),
            camera_line_frame.VirtualTurnStateTracker(),
        )
        at_transition = calculate_command(
            sensor_values(0.30, None, None),
            camera_line_frame.VirtualTurnStateTracker(),
        )
        self.assertTrue(math.isclose(at_transition["left_power"], 0.75))
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

    def test_normal_mapper_uses_quadratic_progress_at_point_thirty_five(self):
        result = calculate_command(
            sensor_values(0.35, None, None),
            camera_line_frame.VirtualTurnStateTracker(),
        )
        self.assertTrue(math.isclose(result["left_power"], 0.75625))
        self.assertTrue(math.isclose(result["right_power"], 0.656875))

    def test_normal_mapper_approaches_strong_limits_before_point_fifty(self):
        result = calculate_command(
            sensor_values(0.50 - 1e-9, None, None),
            camera_line_frame.VirtualTurnStateTracker(),
        )
        self.assertTrue(math.isclose(
            result["left_power"], 0.85, abs_tol=1e-8
        ))
        self.assertTrue(math.isclose(
            result["right_power"], 0.61, abs_tol=1e-8
        ))

    def test_pivot_inner_power_is_zero_at_point_fifty(self):
        result = calculate_command(
            sensor_values(0.50, None, None),
            camera_line_frame.VirtualTurnStateTracker(),
        )
        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (0.85, 0.0),
        )

    def test_normal_mapper_transition_is_symmetric(self):
        right = calculate_command(
            sensor_values(0.35, None, None),
            camera_line_frame.VirtualTurnStateTracker(),
        )
        left = calculate_command(
            sensor_values(-0.35, None, None),
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

    def test_pivot_hysteresis_holds_until_error_falls_below_exit(self):
        pivot_tracker = camera_line_frame.VirtualPivotStateTracker()
        for steering_error, expected_state, expected_powers in (
            (0.52, camera_line_frame.PIVOT_STATE_RIGHT, (0.85, 0.0)),
            (0.48, camera_line_frame.PIVOT_STATE_RIGHT, (0.85, 0.0)),
            (0.40, camera_line_frame.PIVOT_STATE_RIGHT, (0.85, 0.0)),
            (0.34, camera_line_frame.PIVOT_STATE_NONE, (0.754, 0.658)),
        ):
            result = calculate_command(
                sensor_values(steering_error, None, None),
                camera_line_frame.VirtualTurnStateTracker(),
                pivot_state_tracker=pivot_tracker,
            )
            self.assertEqual(pivot_tracker.state, expected_state)
            self.assertTrue(math.isclose(
                result["left_power"], expected_powers[0]
            ))
            self.assertTrue(math.isclose(
                result["right_power"], expected_powers[1]
            ))

    def test_pivot_sign_inversion_exits_before_opposite_pivot(self):
        pivot_tracker = camera_line_frame.VirtualPivotStateTracker()
        calculate_command(
            sensor_values(-0.52, None, None),
            camera_line_frame.VirtualTurnStateTracker(),
            pivot_state_tracker=pivot_tracker,
        )
        self.assertEqual(
            pivot_tracker.state,
            camera_line_frame.PIVOT_STATE_LEFT,
        )

        inverted = calculate_command(
            sensor_values(0.52, None, None),
            camera_line_frame.VirtualTurnStateTracker(),
            pivot_state_tracker=pivot_tracker,
        )
        self.assertEqual(
            pivot_tracker.state,
            camera_line_frame.PIVOT_STATE_NONE,
        )
        self.assertTrue(math.isclose(inverted["left_power"], 0.85))
        self.assertTrue(math.isclose(inverted["right_power"], 0.61))

        opposite_pivot = calculate_command(
            sensor_values(0.52, None, None),
            camera_line_frame.VirtualTurnStateTracker(),
            pivot_state_tracker=pivot_tracker,
        )
        self.assertEqual(
            pivot_tracker.state,
            camera_line_frame.PIVOT_STATE_RIGHT,
        )
        self.assertEqual(
            (opposite_pivot["left_power"], opposite_pivot["right_power"]),
            (0.85, 0.0),
        )

    def test_green_keeps_previous_normal_curve_scale(self):
        result = calculate_command(
            sensor_values(0.30, None, None),
            green_direction="DIREITA",
        )
        steering_strength = 0.30 / 0.40
        self.assertTrue(math.isclose(
            result["left_power"],
            0.69 + steering_strength * (0.75 - 0.69),
        ))
        self.assertTrue(math.isclose(
            result["right_power"],
            0.69 - steering_strength * (0.69 - 0.66),
        ))


class GreenTimeoutAndBlindSearchTests(unittest.TestCase):
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

    def test_green_timeout_with_raw_line_releases_to_recovery(self):
        timeout = camera_line_frame.update_green_maneuver_state(
            "DIREITA",
            camera_line_frame.GREEN_MANEUVER_TIMEOUT_FRAMES - 1,
            raw_line_visible=True,
        )
        self.assertEqual(timeout["direction"], "NENHUMA")
        self.assertTrue(timeout["timedOut"])
        self.assertIsNone(timeout["searchDirection"])

        result = calculate_command(
            sensor_values(None, -0.80, None),
            line_search_tracker=camera_line_frame.VirtualLineSearchTracker(),
            sensor_recovery_requested=True,
        )
        self.assertEqual(result["controlSource"], "virtual-sensor-recovery")
        self.assertEqual((result["left_power"], result["right_power"]), (0.0, 0.69))

    def test_green_timeout_without_line_starts_in_green_direction(self):
        timeout = camera_line_frame.update_green_maneuver_state(
            "DIREITA",
            camera_line_frame.GREEN_MANEUVER_TIMEOUT_FRAMES - 1,
            raw_line_visible=False,
        )
        search_tracker = camera_line_frame.VirtualLineSearchTracker()
        search_tracker.start(timeout["searchDirection"])
        result = calculate_command(
            sensor_values(None, None, None),
            line_search_tracker=search_tracker,
        )
        self.assertEqual(result["controlSource"], "virtual-blind-search")
        self.assertEqual(result["finalSteering"], 1.0)
        self.assertEqual((result["left_power"], result["right_power"]), (0.85, 0.0))

    def test_blind_search_reverses_after_short_initial_window(self):
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
        self.assertEqual(search_tracker.next_direction(), "RIGHT")

    def test_any_sensor_interrupts_blind_search(self):
        cases = []
        near_sensors = sensor_values(None, None, None)
        near_sensors["rawNearPosition"] = -0.80
        cases.append(("NEAR", near_sensors, (0.0, 0.69)))
        cases.append(("MEDIUM", sensor_values(None, 0.80, None), (0.69, 0.0)))
        cases.append(("FAR BAND", sensor_values(None, None, 0.80), (0.69, 0.0)))
        cases.append(("MEDIUM CENTER", sensor_values(None, 0.0, None), (0.69, 0.69)))

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
