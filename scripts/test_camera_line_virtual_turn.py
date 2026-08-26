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
    near_fine_position=None,
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
        "nearFinePosition": near_fine_position,
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
            direcao_verde_ativa=green_direction,
            gap_forward_active=gap_active,
            virtual_turn_tracker=tracker,
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
        return 0.75, 0.0
    if steering_error <= -camera_line_frame.PIVOT_ENTER_THRESHOLD:
        return 0.0, 0.75

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

    def test_near_fine_preserves_coarse_center_position(self):
        mask = np.zeros((101, 101), dtype=np.uint8)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            mask.shape
        )
        near = geometry["near"]
        mask[near["center"]["y0"]:near["center"]["y1"], 57:60] = 255

        sensors = camera_line_frame.read_virtual_line_sensors(mask)

        self.assertTrue(math.isclose(sensors["nearPosition"], 0.0))
        self.assertGreater(sensors["nearFinePosition"], 0.10)

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

    def test_fine_centering_correction_is_limited(self):
        sensors = sensor_values(
            steering_error=0.25,
            medium_position=None,
            far_band_position=None,
            near_fine_position=1.0,
        )
        sensors["nearPosition"] = 0.05

        result = calculate_command(
            sensors,
            pivot_state_tracker=camera_line_frame.VirtualPivotStateTracker(),
        )

        self.assertTrue(math.isclose(
            result["fineCorrection"],
            camera_line_frame.VIRTUAL_FINE_CENTER_MAX_CORRECTION,
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

    def test_fine_centering_requires_coarse_center_and_weak_steering(self):
        cases = (
            (0.20, 0.11),
            (0.31, 0.00),
        )
        for protected_steering, near_position in cases:
            with self.subTest(
                protected_steering=protected_steering,
                near_position=near_position,
            ):
                sensors = sensor_values(
                    steering_error=protected_steering,
                    near_fine_position=1.0,
                )
                sensors["nearPosition"] = near_position
                result = calculate_command(sensors)
                self.assertEqual(result["fineCorrection"], 0.0)
                self.assertEqual(
                    result["finalSteering"],
                    protected_steering,
                )

    def test_fine_centering_does_not_change_green_or_gap(self):
        sensors = sensor_values(
            steering_error=0.10,
            medium_position=None,
            far_band_position=None,
            near_fine_position=1.0,
        )
        for green_direction, gap_active, expected_source in (
            ("DIREITA", False, "virtual-green"),
            ("NENHUMA", True, "gap-forward"),
        ):
            with self.subTest(expected_source=expected_source):
                result = calculate_command(
                    sensors,
                    green_direction=green_direction,
                    gap_active=gap_active,
                )
                self.assertEqual(result["fineCorrection"], 0.0)
                self.assertEqual(result["controlSource"], expected_source)

    def test_virtual_overlay_shows_near_fine_and_correction(self):
        command = calculate_command(sensor_values(
            steering_error=0.10,
            near_fine_position=0.50,
        ))
        frame = np.zeros((360, 480, 3), dtype=np.uint8)
        with patch.object(camera_line_frame.cv2, "putText") as put_text:
            camera_line_frame.draw_virtual_sensor_geometry(frame, command)

        texts = [call.args[1] for call in put_text.call_args_list]
        self.assertIn("NEAR FINE +0.50", texts)
        self.assertIn("FINE CORR +0.12", texts)

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

    def test_protected_near_steering_reaches_mapper_unchanged(self):
        protected_steering = camera_line_frame.protect_virtual_near_direction(
            near_position=0.23,
            medium_position=-0.06,
            heading_angle=-14.7,
            steering_error=-0.26,
        )
        sensors = sensor_values(protected_steering, -0.06, 0.0)
        sensors["nearPosition"] = 0.23
        sensors["headingAngle"] = -14.7

        result = calculate_command(
            sensors,
            camera_line_frame.VirtualTurnStateTracker(),
        )
        expected_left, expected_right = expected_normal_motor_powers(
            protected_steering
        )

        self.assertTrue(math.isclose(result["finalSteering"], 0.23))
        self.assertTrue(math.isclose(result["left_power"], expected_left))
        self.assertTrue(math.isclose(result["right_power"], expected_right))
        self.assertEqual(result["controlSource"], "virtual")

    def test_control_overlay_reports_only_current_states_and_source(self):
        frame = np.zeros((120, 200, 3), dtype=np.uint8)
        command = {
            "left_power": 0.69,
            "right_power": 0.66,
            "lineProcessingMs": 4.25,
            "lineState": "LINE",
            "virtualState": "NORMAL",
            "controlSource": "virtual",
        }
        with patch.object(camera_line_frame.cv2, "putText") as put_text:
            camera_line_frame.draw_line_control_overlay(frame, command)

        texts = [call.args[1] for call in put_text.call_args_list]
        self.assertEqual(texts[0], "LINE 4.2ms")
        self.assertIn("STATE LINE  VSTATE NORMAL", texts)
        self.assertIn("CONTROL SOURCE: VIRTUAL", texts)

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


class VirtualRecoveryTests(unittest.TestCase):
    def test_gap_geometry_preserves_real_near_path(self):
        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[:85, 47:53] = 255

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

    def test_valid_raw_near_then_lost_near_requests_gap(self):
        recent_near_frames = camera_line_frame.update_gap_recent_near_frames(
            0,
            raw_near_visible=True,
        )
        self.assertEqual(
            recent_near_frames,
            camera_line_frame.GAP_NEAR_HISTORY_FRAMES,
        )

        recent_near_frames = camera_line_frame.update_gap_recent_near_frames(
            recent_near_frames,
            raw_near_visible=False,
        )

        self.assertEqual(
            recent_near_frames,
            camera_line_frame.GAP_NEAR_HISTORY_FRAMES - 1,
        )
        self.assertTrue(camera_line_frame.gap_entry_is_required(
            gap_forward_active=False,
            green_direction="NENHUMA",
            recent_near_frames=recent_near_frames,
            raw_near_visible=False,
            real_near_point=None,
            virtual_near_point=None,
            lateral_exit_target=None,
        ))

    def test_recent_near_loss_uses_far_left_gap_recovery(self):
        recent_near_frames = camera_line_frame.update_gap_recent_near_frames(
            0,
            raw_near_visible=True,
        )
        recent_near_frames = camera_line_frame.update_gap_recent_near_frames(
            recent_near_frames,
            raw_near_visible=False,
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
        sensors["rawNearPosition"] = None

        result = calculate_command(sensors, gap_active=gap_active)

        self.assertEqual(result["lineState"], "GAP")
        self.assertEqual(result["controlSource"], "gap-sensor-recovery")
        self.assertEqual((result["left_power"], result["right_power"]), (0.0, 0.69))

    def test_recent_near_loss_uses_far_right_gap_recovery(self):
        recent_near_frames = camera_line_frame.update_gap_recent_near_frames(
            0,
            raw_near_visible=True,
        )
        recent_near_frames = camera_line_frame.update_gap_recent_near_frames(
            recent_near_frames,
            raw_near_visible=False,
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
        sensors["rawNearPosition"] = None

        result = calculate_command(sensors, gap_active=gap_active)

        self.assertEqual(result["lineState"], "GAP")
        self.assertEqual(result["controlSource"], "gap-sensor-recovery")
        self.assertEqual((result["left_power"], result["right_power"]), (0.69, 0.0))

    def test_gap_near_history_expires_without_being_renewed(self):
        recent_near_frames = camera_line_frame.update_gap_recent_near_frames(
            0,
            raw_near_visible=True,
        )
        observed_history = []
        for _ in range(camera_line_frame.GAP_NEAR_HISTORY_FRAMES):
            recent_near_frames = camera_line_frame.update_gap_recent_near_frames(
                recent_near_frames,
                raw_near_visible=False,
            )
            observed_history.append(recent_near_frames)

        self.assertEqual(observed_history, [2, 1, 0])
        self.assertFalse(camera_line_frame.gap_entry_is_required(
            gap_forward_active=False,
            green_direction="NENHUMA",
            recent_near_frames=recent_near_frames,
            raw_near_visible=False,
            real_near_point=None,
            virtual_near_point=None,
            lateral_exit_target=None,
        ))

        sensors = sensor_values(None, None, -0.80)
        sensors["rawNearPosition"] = None
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
            raw_near_visible=False,
            real_near_point=None,
            virtual_near_point=None,
            lateral_exit_target=None,
        )
        self.assertFalse(gap_active)

        sensors = sensor_values(None, None, -0.80)
        sensors["rawNearPosition"] = None
        result = calculate_command(
            sensors,
            green_direction="DIREITA",
            gap_active=gap_active,
        )
        self.assertEqual(result["lineState"], "GREEN")
        self.assertEqual(result["controlSource"], "virtual-green")

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
        self.assertEqual((result["left_power"], result["right_power"]), (0.0, 0.75))

    def test_matching_right_recovery_enters_reorient(self):
        tracker, result = self.enter_reorient("RIGHT")
        self.assertEqual(tracker.state, camera_line_frame.VIRTUAL_STATE_REORIENT_RIGHT)
        self.assertEqual(result["finalSteering"], 1.0)
        self.assertEqual((result["left_power"], result["right_power"]), (0.75, 0.0))

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
        expected_left, expected_right = expected_normal_motor_powers(0.35)
        self.assertTrue(math.isclose(result["left_power"], expected_left))
        self.assertTrue(math.isclose(result["right_power"], expected_right))

    def test_normal_mapper_approaches_strong_limits_before_pivot(self):
        steering_error = camera_line_frame.PIVOT_ENTER_THRESHOLD - 1e-9
        result = calculate_command(
            sensor_values(steering_error, None, None),
            camera_line_frame.VirtualTurnStateTracker(),
        )
        self.assertTrue(math.isclose(
            result["left_power"], 0.85, abs_tol=1e-8
        ))
        self.assertTrue(math.isclose(
            result["right_power"], 0.61, abs_tol=1e-8
        ))

    def test_pivot_inner_power_is_zero_at_entry_threshold(self):
        result = calculate_command(
            sensor_values(camera_line_frame.PIVOT_ENTER_THRESHOLD, None, None),
            camera_line_frame.VirtualTurnStateTracker(),
        )
        self.assertEqual(
            (result["left_power"], result["right_power"]),
            (0.75, 0.0),
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
        released_powers = expected_normal_motor_powers(0.34)
        for steering_error, expected_state, expected_powers in (
            (0.47, camera_line_frame.PIVOT_STATE_RIGHT, (0.75, 0.0)),
            (0.44, camera_line_frame.PIVOT_STATE_RIGHT, (0.75, 0.0)),
            (0.40, camera_line_frame.PIVOT_STATE_RIGHT, (0.75, 0.0)),
            (0.34, camera_line_frame.PIVOT_STATE_NONE, released_powers),
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
            sensor_values(-0.47, None, None),
            camera_line_frame.VirtualTurnStateTracker(),
            pivot_state_tracker=pivot_tracker,
        )
        self.assertEqual(
            pivot_tracker.state,
            camera_line_frame.PIVOT_STATE_LEFT,
        )

        inverted = calculate_command(
            sensor_values(0.47, None, None),
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
            sensor_values(0.47, None, None),
            camera_line_frame.VirtualTurnStateTracker(),
            pivot_state_tracker=pivot_tracker,
        )
        self.assertEqual(
            pivot_tracker.state,
            camera_line_frame.PIVOT_STATE_RIGHT,
        )
        self.assertEqual(
            (opposite_pivot["left_power"], opposite_pivot["right_power"]),
            (0.75, 0.0),
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
        self.assertEqual((result["left_power"], result["right_power"]), (0.75, 0.0))

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
