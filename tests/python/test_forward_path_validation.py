"""Verifica geometria frontal, histerese e autoridade inferior sem hardware."""

import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from vision.camera_config import FORWARD_PATH_CONFIG
from vision.forward_path import ForwardPathTracker, bottom_reference, predicted_path
from vision.gap_validation import (GapValidator,
                                   bottom_fusion_path_is_connected,
                                   read_json_snapshot)
from vision.line_control import (LineFollowerController, gap_entry_is_required,
                                 update_gap_forward_recovery,
                                 virtual_far_line_is_visible)
from vision.fusion_guidance import extract_fusion_style_line, update_fusion_style_history
from vision.maneuver_state import LineManeuverState
from vision.virtual_sensors import read_virtual_line_sensors


def reference(position=0.0, slope=0.0, curvature=0.0):
    return {"valid": True, "position": position, "slope": slope, "curvature": curvature,
            "timestamp": 100.0, "sequence": 10, "side": "CENTER"}


def tape(mask, curve, thickness=12, start=0.0, end=1.0):
    height, width = mask.shape
    points = [(int(round((curve(d) + 1) * (width - 1) / 2)),
               int(round(height - 1 - d * (height - 1)))) for d in np.linspace(start, end, height)]
    cv2.polylines(mask, [np.array(points, np.int32)], False, 255, thickness)


def reading(sequence, timestamp, state="PRESENT", confidence=1.0):
    return {"forwardPathVersion": 2, "forwardLineVisible": state != "ABSENT",
            "forwardLinePresent": state == "PRESENT",
            "forwardPathState": state, "forwardPathConfidence": confidence,
            "forwardPathReferenceValid": True, "forwardPathReferenceTimestamp": 100.0,
            "forwardPathReferenceSequence": 10,
            "forwardLineSequence": sequence, "forwardLineTimestamp": timestamp}


class ForwardGeometryTest(unittest.TestCase):
    def analyze(self, mask, ref=None, tracker=None, now=100.02):
        height, width = mask.shape
        return (tracker or ForwardPathTracker()).analyze(mask, (0, 0, width, height), ref, now)

    def test_center_side_and_tilted_tape_are_present_without_bottom_reference(self):
        for curve in (lambda d: 0, lambda d: 0.8, lambda d: -0.7 + 1.2 * d):
            mask = np.zeros((360, 640), np.uint8)
            tape(mask, curve, 20)
            self.assertEqual(self.analyze(mask)["forwardPathState"], "PRESENT")

    def test_strong_curve_and_mismatched_heading_do_not_veto_gap(self):
        mask = np.zeros((360, 640), np.uint8)
        tape(mask, lambda d: -0.5 + 1.1 * d ** 2, 20)
        result = self.analyze(mask, reference(0.8, -3, -7))
        self.assertEqual(result["forwardPathState"], "PRESENT")

    def test_horizontal_tape_is_still_real_tape(self):
        mask = np.zeros((360, 640), np.uint8)
        cv2.line(mask, (40, 280), (590, 280), 255, 25)
        self.assertEqual(self.analyze(mask)["forwardPathState"], "PRESENT")

    def test_multiple_tapes_are_not_averaged_or_rejected_as_ambiguous_presence(self):
        mask = np.zeros((360, 640), np.uint8)
        tape(mask, lambda d: -0.45, 20)
        tape(mask, lambda d: 0.45, 20)
        result = self.analyze(mask, reference(-0.45))
        self.assertEqual(result["forwardPathState"], "PRESENT")
        self.assertAlmostEqual(result["forwardLinePosition"], -0.45, delta=0.03)
        self.assertEqual(sum(c["state"] == "PRESENT" for c in result["candidates"]), 2)

    def test_particles_and_short_blobs_are_absent(self):
        for radius in (2, 7, 15):
            mask = np.zeros((360, 640), np.uint8)
            for x in range(100, 550, 70):
                cv2.circle(mask, (x, 280), radius, 255, -1)
            self.assertEqual(self.analyze(mask)["forwardPathState"], "ABSENT")

    def test_only_distant_support_is_uncertain(self):
        mask = np.zeros((360, 640), np.uint8)
        tape(mask, lambda d: 0, 20, start=0.70, end=1)
        self.assertEqual(self.analyze(mask)["forwardPathState"], "UNCERTAIN")

    def test_disconnected_particles_cannot_pool_their_scanlines(self):
        mask = np.zeros((360, 640), np.uint8)
        for y in range(50, 350, 40):
            cv2.rectangle(mask, (300, y), (321, y + 10), 255, -1)
        self.assertEqual(self.analyze(mask)["forwardPathState"], "ABSENT")

    def test_stale_reference_and_temporal_displacement_do_not_veto_present_tape(self):
        tracker = ForwardPathTracker()
        for x, now in ((-0.6, 100), (0.6, 100.04), (0.7, 103)):
            mask = np.zeros((360, 640), np.uint8)
            tape(mask, lambda d: x, 20)
            self.assertEqual(self.analyze(mask, reference(), tracker, now)["forwardPathState"], "PRESENT")

    def test_normalized_position_and_thresholds_scale_with_resolution(self):
        for height, width in ((180, 320), (360, 640)):
            mask = np.zeros((height, width), np.uint8)
            tape(mask, lambda d: 0.3, width // 30)
            result = self.analyze(mask)
            self.assertEqual(result["forwardPathState"], "PRESENT")
            self.assertAlmostEqual(result["forwardLinePosition"], 0.3, delta=0.03)


class GapDecisionTest(unittest.TestCase):
    def setUp(self):
        self.validator = GapValidator()
        self.validator.reference = reference()

    def update(self, seconds, front=None, recovered=False, special=False):
        return self.validator.update(True, recovered, special, front or {}, seconds, 100 + seconds)

    def test_gap_requires_two_new_present_frames(self):
        self.assertEqual(self.update(0, reading(1, 100)), "CHECKING")
        self.assertEqual(self.update(0.02, reading(1, 100)), "CHECKING")
        self.assertEqual(self.update(0.04, reading(2, 100.04)), "GAP")

    def test_distant_fusion_requires_trusted_far_and_medium_together(self):
        fusion = {
            "valid": True,
            "selection": "deepestFallback",
            "angleDeg": 105.0,
            "nearPoint": {"x": 240, "y": 359},
            "farPoint": {"x": 300, "y": 80},
            "targetStableFrames": 2,
        }
        incomplete_sensor_cases = (
            {
                "farTrusted": True,
                "farBandPosition": 0.25,
                "mediumTrusted": False,
                "mediumPosition": None,
            },
            {
                "farTrusted": False,
                "farBandPosition": None,
                "mediumTrusted": True,
                "mediumPosition": -0.25,
            },
        )
        for sensors in incomplete_sensor_cases:
            with self.subTest(sensors=sensors):
                validator = GapValidator()
                self.assertFalse(validator.observe_bottom_fusion(
                    fusion, sensors, True, True, False,
                ))
                self.assertFalse(validator.observe_bottom_fusion(
                    fusion, sensors, True, True, False,
                ))

        complete = {
            "farTrusted": True,
            "farBandPosition": 0.25,
            "mediumTrusted": True,
            "mediumPosition": 0.20,
        }
        connected_mask = np.zeros((360, 480), np.uint8)
        cv2.line(connected_mask, (240, 210), (300, 50), 255, 30)
        validator = GapValidator()
        self.assertFalse(validator.observe_bottom_fusion(
            fusion, complete, True, True, False, connected_mask,
        ))
        self.assertTrue(validator.observe_bottom_fusion(
            fusion, complete, True, True, False, connected_mask,
        ))

    def test_distant_fusion_rejects_disconnected_far_and_medium_tapes(self):
        mask = np.zeros((360, 480), np.uint8)
        cv2.line(mask, (100, 294), (100, 170), 255, 30)
        cv2.line(mask, (360, 145), (360, 50), 255, 30)
        sensors = read_virtual_line_sensors(mask, "NENHUMA", False)
        fusion = extract_fusion_style_line(mask)
        fusion["targetStableFrames"] = 3

        self.assertTrue(sensors["farTrusted"])
        self.assertTrue(sensors["mediumTrusted"])
        self.assertFalse(bottom_fusion_path_is_connected(mask, fusion, 3))
        validator = GapValidator()
        for _ in range(3):
            self.assertFalse(validator.observe_bottom_fusion(
                fusion, sensors, True, True, False, mask,
            ))
        self.assertFalse(validator.bottom_fusion_connected)
        self.assertEqual(validator.bottom_fusion_frames, 0)

    def test_strong_continuous_curve_can_reacquire_fusion(self):
        mask = np.zeros((360, 480), np.uint8)
        points = np.array(
            [(100, 294), (115, 240), (160, 185), (260, 130), (400, 50)],
            np.int32,
        )
        cv2.polylines(mask, [points], False, 255, 30)
        sensors = read_virtual_line_sensors(mask, "NENHUMA", False)
        fusion = extract_fusion_style_line(mask)
        fusion["targetStableFrames"] = 3

        self.assertTrue(sensors["farTrusted"])
        self.assertTrue(sensors["mediumTrusted"])
        self.assertTrue(bottom_fusion_path_is_connected(mask, fusion, 3))
        validator = GapValidator()
        self.assertFalse(validator.observe_bottom_fusion(
            fusion, sensors, True, True, False, mask,
        ))
        self.assertTrue(validator.observe_bottom_fusion(
            fusion, sensors, True, True, False, mask,
        ))

    def test_repeated_sequence_and_out_of_order_frames_do_not_confirm_gap(self):
        self.update(0, reading(2, 100))
        self.assertEqual(self.update(0.02, reading(2, 100.02)), "CHECKING")
        self.assertEqual(self.update(0.04, reading(1, 100.04)), "CHECKING")
        self.assertEqual(self.update(0.06), "CHECKING")
        self.assertEqual(self.update(0.08, reading(2, 100)), "CHECKING")
        self.assertEqual(self.update(0.10, reading(3, 100.10)), "CHECKING")
        self.assertEqual(self.update(0.12, reading(4, 100.12)), "GAP")

    def test_future_front_is_rejected_but_expired_bottom_reference_is_optional(self):
        self.assertFalse(self.validator.present_reading(reading(1, 101), 100)[0])
        self.assertTrue(self.validator.present_reading(reading(2, 102.1), 102.1)[0])

    def test_reference_tracks_only_normal_observations_and_freezes_on_loss(self):
        mask = np.zeros((360, 480), np.uint8)
        tape(mask, lambda d: 0.10 + 0.1 * d, 45)
        fusion = extract_fusion_style_line(mask)
        controller = LineFollowerController()
        command = controller.calculate(mask, {}, fusion_style_line=fusion)
        self.validator.remember(mask, fusion, command, 100.02, 11)
        recent = copy.deepcopy(self.validator.reference)
        self.assertEqual(recent["sequence"], 11)
        self.assertGreater(recent["position"], 0.1)
        for state in ("GAP", "GREEN", "LOST"):
            self.validator.remember(mask, fusion, {**command, "lineState": state}, 100.04, 12)
            self.assertEqual(self.validator.reference, recent)
        self.update(0.04)
        self.validator.remember(mask, fusion, command, 100.06, 13)
        self.assertEqual(self.validator.reference, recent)

    def test_isolated_empty_frame_does_not_declare_lost(self):
        self.assertEqual(self.update(0, reading(1, 100, "ABSENT", 0)), "CHECKING")
        self.assertEqual(self.update(0.04, reading(2, 100.04)), "CHECKING")
        self.assertEqual(self.update(0.08, reading(3, 100.08)), "GAP")
        self.assertEqual(self.update(0.12, reading(4, 100.12, "ABSENT", 0)), "GAP")

    def test_prolonged_none_uncertain_incoherent_and_stale_become_lost(self):
        for state in ("ABSENT", "UNCERTAIN", "PRESENT"):
            self.setUp()
            self.update(0, reading(1, 100, state))
            self.assertEqual(self.update(0.51, reading(1, 100, state)), "LOST")

    def test_confirmed_gap_has_absolute_timeout_even_with_visible_front(self):
        self.update(0, reading(1, 100))
        for i in range(1, 15):
            self.update(i * 0.1, reading(i + 1, 100 + i * 0.1))
        self.assertEqual(self.update(1.51, reading(20, 101.51)), "LOST")

    def test_late_present_frame_cannot_reopen_expired_confirmation_or_grace(self):
        self.update(0, reading(1, 100))
        self.assertEqual(self.update(0.51, reading(2, 100.51)), "LOST")
        self.setUp()
        self.update(0, reading(1, 100))
        self.update(0.04, reading(2, 100.04))
        self.assertEqual(self.update(0.26, reading(3, 100.26)), "LOST")
        self.assertEqual(self.validator.reason, "CONTINUATION_EXPIRED")

    def test_reference_expiration_and_old_schema_cannot_confirm_gap(self):
        for change in ({"forwardPathVersion": 1}, {"forwardLinePresent": False},
                       {"forwardLineVisible": False}, {"forwardPathConfidence": float("nan")}):
            self.setUp()
            for i in range(16):
                result = self.update(i * 0.04, {**reading(i + 1, 100 + i * 0.04), **change})
            self.assertEqual(result, "LOST")

    def test_trusted_bottom_far_can_recover_after_lost(self):
        self.assertEqual(self.update(0), "CHECKING")
        self.assertEqual(self.update(0.51), "LOST")
        self.assertEqual(
            self.validator.update(
                True,
                False,
                False,
                {},
                0.55,
                100.55,
                bottom_far_confirmed=True,
            ),
            "GAP",
        )
        self.assertEqual(
            self.validator.reason,
            "BOTTOM_FAR_RECOVERED_AFTER_LOST",
        )

    def test_malformed_sequence_does_not_break_next_valid_observations(self):
        for invalid in (None, "1", True, [], -1):
            self.setUp()
            self.update(0, {**reading(1, 100), "forwardLineSequence": invalid})
            self.assertEqual(self.update(0.04, reading(2, 100.04)), "CHECKING")
            self.assertEqual(self.update(0.08, reading(3, 100.08)), "GAP")

    def test_recovered_bottom_and_special_maneuver_restore_bottom_immediately(self):
        self.update(0)
        self.update(0.3)
        self.assertEqual(self.update(0.31, reading(4, 100.31), recovered=True), "NORMAL")
        self.update(0.4)
        self.assertEqual(self.update(0.41, special=True), "NORMAL")
        self.assertIsNone(self.validator.reference)

    def test_front_cannot_change_normal_straight_or_curve_commands(self):
        for curve in (lambda d: 0, lambda d: -0.45 * d ** 2, lambda d: 0.65 * d):
            mask = np.zeros((360, 480), np.uint8)
            tape(mask, curve, 45)
            fusion = extract_fusion_style_line(mask)
            for state in ("PRESENT", "ABSENT", "UNCERTAIN"):
                native, checked = LineFollowerController(), LineFollowerController()
                maneuver = LineManeuverState()
                self.validator.apply(maneuver, checked, False, True, False,
                                     reading(1, 100, state), 0, 100)
                original = native.calculate(mask, {}, fusion_style_line=fusion)
                actual = checked.calculate(mask, {}, gap_forward_active=maneuver.gap_forward_active,
                                           fusion_style_line=fusion)
                for key in ("left_power", "right_power", "controlSource", "virtualState", "finalSteering"):
                    self.assertEqual(original[key], actual[key], (state, key))

    def test_gap_uses_native_crossing_and_lost_uses_existing_backup_search(self):
        maneuver, controller = LineManeuverState(), LineFollowerController()
        mask = np.zeros((360, 480), np.uint8)
        self.validator.apply(maneuver, controller, True, False, False, reading(1, 100), 0, 100)
        self.validator.apply(maneuver, controller, True, False, False, reading(2, 100.04), 0.04, 100.04)
        command = controller.calculate(mask, {}, gap_forward_active=maneuver.gap_forward_active)
        self.assertEqual(command["controlSource"], "gap-forward")
        self.assertEqual(command["left_power"], command["right_power"])
        self.validator.apply(maneuver, controller, True, False, False, {}, 0.4, 100.4, "LEFT")
        command = controller.calculate(mask, {}, gap_forward_active=maneuver.gap_forward_active)
        self.assertEqual(command["controlSource"], "virtual-blind-search-backup")
        self.assertLess(command["left_power"], 0)
        self.assertEqual(controller.line_search_tracker.initial_direction, "LEFT")

    def test_gap_reacquisition_still_requires_native_two_frames(self):
        first = update_gap_forward_recovery(True, 3, 0, True, True)
        self.assertTrue(first["active"])
        second = update_gap_forward_recovery(True, first["forwardFrames"], first["reacquireFrames"], True, True)
        self.assertFalse(second["active"])

    def test_json_reader_fails_closed(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "status.json"
            self.assertEqual(read_json_snapshot(path), {})
            for text in ("{", "[]", " " * 32769):
                path.write_text(text)
                self.assertEqual(read_json_snapshot(path), {})


class VirtualNearGateTest(unittest.TestCase):
    """Executa o gate do NEAR-C virtual antes do controlador inferior real."""

    def setUp(self):
        self.validator = GapValidator()
        self.controller = LineFollowerController()
        self.maneuver = LineManeuverState()
        self.sequence = 0
        self.line = np.zeros((360, 480), np.uint8)
        tape(self.line, lambda d: 0, 80)
        self.fragment = np.zeros_like(self.line)
        cv2.circle(self.fragment, (235, 335), 14, 255, -1)

    def frame(self, mask, front_state="PRESENT", sequence=None, seconds=None,
              bottom_far_present=None, fusion_style_line=None):
        self.sequence = self.sequence + 1 if sequence is None else sequence
        now = self.sequence / 30 if seconds is None else seconds
        sensors = read_virtual_line_sensors(mask, "NENHUMA", False)
        if bottom_far_present is None:
            bottom_far_present = virtual_far_line_is_visible(sensors)
        fusion = (
            extract_fusion_style_line(mask)
            if fusion_style_line is None
            else fusion_style_line
        )
        blind_search = self.validator.process_frame(
            mask, self.maneuver, self.controller, self.sequence, 100 + now,
            reading(self.sequence, 100 + now, front_state), now, 100 + now,
            bottom_far_present=bottom_far_present,
            fusion_style_line=fusion,
            virtual_sensors=sensors)
        return self.controller.calculate(
            mask, {}, gap_forward_active=self.maneuver.gap_forward_active,
            virtual_sensors=sensors, fusion_style_line=fusion,
            blind_search_requested=blind_search,
            local_line_lost=self.validator.decision == "LOST",
            gap_fusion_reacquire_active=(
                self.maneuver.gap_fusion_reacquire_active
            ))

    def arm(self):
        self.frame(self.line)
        self.frame(self.line)
        self.assertTrue(self.validator.near_armed)

    def test_normal_straight_and_strong_lateral_curves_keep_exact_commands(self):
        for curve in (lambda d: 0, lambda d: -0.78 + 0.9 * d ** 2,
                      lambda d: 0.7 - 0.5 * d, lambda d: -0.78):
            self.setUp()
            mask = np.zeros_like(self.line)
            tape(mask, curve, 50)
            native = LineFollowerController()
            for state in ("ABSENT", "UNCERTAIN", "PRESENT"):
                original = native.calculate(mask, {}, fusion_style_line=extract_fusion_style_line(mask))
                actual = self.frame(mask, state)
                self.assertEqual(self.validator.decision, "NORMAL")
                for key in ("left_power", "right_power", "controlSource", "virtualState", "finalSteering"):
                    self.assertEqual(original[key], actual[key], key)

    def test_small_particles_do_not_pool_near_center_presence(self):
        for radius in (2, 7, 14):
            mask = np.zeros_like(self.line)
            for x, y in ((70, 290), (210, 320), (380, 280)):
                cv2.circle(mask, (x, y), radius, 255, -1)
            result = self.validator.observe_near(mask, 100, 1, 100)
            self.assertFalse(result["present"], result)

    def test_lateral_shadow_with_valid_fusion_enters_gap_from_near_center_loss(self):
        self.arm()
        lateral_shadow = np.zeros_like(self.line)
        cv2.line(lateral_shadow, (0, 340), (180, 359), 255, 45)
        sensors = read_virtual_line_sensors(lateral_shadow, "NENHUMA", False)
        fusion = extract_fusion_style_line(lateral_shadow)
        self.assertTrue(fusion["valid"])
        self.assertEqual(fusion["selection"], "nearCenter")
        self.assertIsNone(sensors["nearFinePosition"])
        self.assertFalse(sensors["farTrusted"])
        self.assertFalse(sensors["mediumTrusted"])

        self.frame(lateral_shadow, fusion_style_line=fusion)
        checking = self.frame(lateral_shadow, fusion_style_line=fusion)
        self.assertEqual(self.validator.decision, "CHECKING")
        self.assertEqual(checking["controlSource"], "gap-forward")
        self.assertFalse(self.validator.near["present"])

        confirmed = self.frame(lateral_shadow, fusion_style_line=fusion)
        self.assertEqual(self.validator.decision, "GAP")
        self.assertEqual(confirmed["controlSource"], "gap-forward")

    def test_fusion_valid_fragment_cannot_veto_confirmed_near_loss(self):
        self.arm()
        fusion = extract_fusion_style_line(self.fragment)
        self.assertTrue(fusion["valid"])
        self.frame(self.fragment)
        self.assertEqual(self.validator.decision, "NORMAL")
        command = self.frame(self.fragment)
        self.assertEqual(self.validator.decision, "CHECKING")
        self.assertEqual(command["controlSource"], "gap-forward")
        self.assertTrue(gap_entry_is_required(
            False, "NENHUMA", near_center_visible=True, real_near_point={"x": 235, "y": 335},
            fusion_near_connected=True, near_line_present=False, near_loss_confirmed=True))
        self.frame(self.fragment)
        self.assertEqual(self.validator.decision, "GAP")

    def test_short_gap_uses_trusted_bottom_far_before_forward_camera(self):
        for x in (240, 70):
            self.setUp()
            self.arm()
            far_only = np.zeros_like(self.line)
            cv2.line(far_only, (x, 20), (x, 150), 255, 40)
            sensors = read_virtual_line_sensors(far_only, "NENHUMA", False)
            self.assertTrue(sensors["farTrusted"])
            self.assertIsNotNone(sensors["farBandPosition"])

            self.frame(far_only, "ABSENT")
            command = self.frame(far_only, "ABSENT")

            self.assertEqual(self.validator.decision, "GAP")
            self.assertEqual(self.validator.reason, "BOTTOM_FAR_PRESENT")
            self.assertEqual(command["controlSource"], "virtual-gap-far")
            self.assertGreater(command["left_power"], 0.0)
            self.assertGreater(command["right_power"], 0.0)
            if x == 240:
                self.assertAlmostEqual(
                    command["left_power"], command["right_power"]
                )
            else:
                self.assertLess(command["left_power"], command["right_power"])
            self.assertFalse(self.validator.near["present"])

    def test_alternating_far_sides_use_smooth_forward_gap_steering(self):
        self.arm()
        commands = []
        for index, x in enumerate((70, 410, 70, 410, 70, 410)):
            far_only = np.zeros_like(self.line)
            cv2.line(far_only, (x, 20), (x, 150), 255, 40)
            commands.append(self.frame(far_only, "ABSENT"))

        self.assertEqual(self.validator.decision, "GAP")
        for command in commands[1:]:
            self.assertEqual(command["controlSource"], "virtual-gap-far")
            self.assertGreater(command["left_power"], 0.0)
            self.assertGreater(command["right_power"], 0.0)
            self.assertNotEqual(command["left_power"], command["right_power"])

    def test_far_outside_far_band_confirms_guides_and_keeps_gap_alive(self):
        self.arm()
        lower_far = np.zeros_like(self.line)
        cv2.line(lower_far, (80, 100), (80, 150), 255, 40)
        sensors = read_virtual_line_sensors(lower_far, "NENHUMA", False)
        self.assertTrue(sensors["farTrusted"])
        self.assertIsNotNone(sensors["farPosition"])
        self.assertIsNone(sensors["farBandPosition"])
        self.assertTrue(virtual_far_line_is_visible(sensors))

        self.frame(lower_far, "ABSENT")
        command = self.frame(lower_far, "ABSENT")
        self.assertEqual(self.validator.decision, "GAP")
        self.assertEqual(self.validator.reason, "BOTTOM_FAR_PRESENT")
        self.assertEqual(command["controlSource"], "virtual-gap-far")
        self.assertGreater(command["left_power"], 0.0)
        self.assertGreater(command["right_power"], 0.0)
        self.assertLess(command["left_power"], command["right_power"])

        for _ in range(50):
            command = self.frame(lower_far, "ABSENT")
        self.assertEqual(self.validator.decision, "GAP")
        self.assertEqual(command["controlSource"], "virtual-gap-far")

        for _ in range(7):
            command = self.frame(np.zeros_like(self.line), "ABSENT")
        self.assertEqual(self.validator.decision, "LOST")
        self.assertTrue(command["controlSource"].startswith("virtual-blind-search"))

    def test_far_reappearing_after_lost_stops_search_and_resumes_gap(self):
        self.arm()
        empty = np.zeros_like(self.line)
        for _ in range(17):
            command = self.frame(empty, "ABSENT")
        self.assertEqual(self.validator.decision, "LOST")
        self.assertTrue(command["controlSource"].startswith("virtual-blind-search"))

        far_only = np.zeros_like(self.line)
        cv2.line(far_only, (110, 20), (110, 150), 255, 40)
        self.frame(far_only, "ABSENT")
        recovered = self.frame(far_only, "ABSENT")

        self.assertEqual(self.validator.decision, "GAP")
        self.assertEqual(
            self.validator.reason,
            "BOTTOM_FAR_RECOVERED_AFTER_LOST",
        )
        self.assertEqual(recovered["controlSource"], "virtual-gap-far")
        self.assertGreater(recovered["left_power"], 0.0)
        self.assertGreater(recovered["right_power"], 0.0)
        self.assertFalse(self.controller.line_search_tracker.active)

    def test_stable_distant_fusion_reacquires_before_near_then_returns_normal(self):
        self.arm()
        distant = np.zeros_like(self.line)
        cv2.line(distant, (240, 210), (300, 50), 255, 30)
        first_fusion = extract_fusion_style_line(distant)
        history = update_fusion_style_history(None, first_fusion)
        second_fusion = extract_fusion_style_line(distant, history)
        history = update_fusion_style_history(history, second_fusion)
        third_fusion = extract_fusion_style_line(distant, history)

        self.frame(distant, fusion_style_line=first_fusion)
        straight = self.frame(distant, fusion_style_line=second_fusion)
        reacquired = self.frame(distant, fusion_style_line=third_fusion)

        self.assertFalse(self.validator.near["present"])
        self.assertEqual(straight["controlSource"], "virtual-gap-far")
        self.assertGreater(straight["left_power"], 0.0)
        self.assertGreater(straight["right_power"], 0.0)
        self.assertEqual(reacquired["lineState"], "GAP")
        self.assertEqual(
            reacquired["controlSource"],
            "fusion-gap-reacquire",
        )
        self.assertTrue(reacquired["fusionControlActive"])
        self.assertGreater(reacquired["left_power"], 0.0)
        self.assertGreater(reacquired["right_power"], 0.0)
        self.assertTrue(self.maneuver.gap_fusion_reacquire_active)

        self.frame(self.line)
        normal = self.frame(self.line)
        self.assertEqual(self.validator.decision, "NORMAL")
        self.assertEqual(normal["lineState"], "LINE")
        self.assertEqual(normal["controlSource"], "fusion")
        self.assertFalse(self.maneuver.gap_fusion_reacquire_active)

    def test_lost_distant_fusion_returns_to_straight_gap_crossing(self):
        self.arm()
        distant = np.zeros_like(self.line)
        cv2.line(distant, (240, 210), (300, 50), 255, 30)
        first = extract_fusion_style_line(distant)
        second = extract_fusion_style_line(
            distant,
            update_fusion_style_history(None, first),
        )
        third = extract_fusion_style_line(
            distant,
            update_fusion_style_history(second, second),
        )
        self.frame(distant, fusion_style_line=first)
        self.frame(distant, fusion_style_line=second)
        command = self.frame(distant, fusion_style_line=third)
        self.assertEqual(command["controlSource"], "fusion-gap-reacquire")

        empty = self.frame(np.zeros_like(self.line), front_state="PRESENT")
        self.assertEqual(self.validator.decision, "GAP")
        self.assertEqual(empty["controlSource"], "gap-forward")
        self.assertFalse(self.maneuver.gap_fusion_reacquire_active)

    def test_oscillating_distant_fusion_cannot_reacquire_gap(self):
        self.arm()
        masks = []
        for start_x, end_x in ((180, 80), (300, 400)):
            mask = np.zeros_like(self.line)
            cv2.line(mask, (start_x, 210), (end_x, 50), 255, 30)
            masks.append(mask)

        history = None
        commands = []
        for index in range(8):
            mask = masks[index % 2]
            fusion = extract_fusion_style_line(mask, history)
            history = update_fusion_style_history(history, fusion)
            commands.append(self.frame(mask, fusion_style_line=fusion))

        self.assertEqual(self.validator.decision, "GAP")
        self.assertFalse(self.maneuver.gap_fusion_reacquire_active)
        for command in commands[1:]:
            self.assertEqual(command["controlSource"], "virtual-gap-far")
            self.assertGreater(command["left_power"], 0.0)
            self.assertGreater(command["right_power"], 0.0)

    def test_duplicate_stale_and_future_frames_cannot_confirm_local_loss(self):
        self.arm()
        self.frame(self.fragment, sequence=3)
        self.frame(self.fragment, sequence=3)
        self.assertEqual(self.validator.missing_frames, 1)
        for timestamp in (90.0, 101.0):
            result = self.validator.observe_near(self.fragment, timestamp, 4, 100.14)
            self.assertFalse(result["fresh"])
            self.assertFalse(result["lossConfirmed"])
        self.assertEqual(self.validator.decision, "NORMAL")
        self.frame(self.fragment, sequence=4)
        self.assertEqual(self.validator.decision, "CHECKING")

    def test_one_empty_frame_does_not_change_gate_and_present_resets_loss_count(self):
        self.arm()
        self.frame(np.zeros_like(self.line), "ABSENT")
        self.assertEqual(self.validator.decision, "NORMAL")
        self.frame(self.line)
        self.assertEqual(self.validator.decision, "NORMAL")
        self.assertEqual(self.validator.missing_frames, 0)

    def test_confirmed_loss_runs_original_backup_then_search_despite_residual_fusion(self):
        self.arm()
        commands = [self.frame(self.fragment, "ABSENT") for _ in range(23)]
        sources = [c["controlSource"] for c in commands]
        self.assertEqual(self.validator.decision, "LOST")
        self.assertIn("virtual-blind-search-backup", sources)
        self.assertIn("virtual-blind-search", sources)
        self.assertNotIn("fusion", sources[2:])
        self.assertTrue(self.controller.line_search_tracker.active)

    def test_reacquisition_requires_two_real_new_frames_then_restores_fusion(self):
        for initially_present in (True, False):
            self.setUp()
            self.arm()
            for _ in range(17 if not initially_present else 3):
                self.frame(self.fragment, "PRESENT" if initially_present else "ABSENT")
            self.assertEqual(self.validator.decision, "GAP" if initially_present else "LOST")
            self.frame(self.line)
            self.assertNotEqual(self.validator.decision, "NORMAL")
            command = self.frame(self.line)
            self.assertEqual(self.validator.decision, "NORMAL")
            self.assertEqual(command["controlSource"], "fusion")
            self.assertFalse(self.controller.line_search_tracker.active)
            self.assertFalse(self.maneuver.gap_forward_active)

    def test_no_recent_presence_does_not_invent_gap_and_special_controls_keep_priority(self):
        self.frame(self.fragment)
        self.frame(self.fragment)
        self.assertEqual(self.validator.decision, "NORMAL")
        self.arm()
        self.maneuver.green_direction = "DIREITA"
        self.frame(self.fragment)
        self.frame(self.fragment)
        self.assertEqual(self.validator.decision, "NORMAL")
        self.assertFalse(self.validator.near_armed)

    def test_horizontal_local_tape_and_visible_end_at_frame_border_are_present(self):
        horizontal = np.zeros_like(self.line)
        cv2.line(horizontal, (10, 315), (470, 315), 255, 70)
        end = np.zeros_like(self.line)
        cv2.fillConvexPoly(end, np.array([(179, 261), (269, 271), (257, 359), (167, 359)]), 255)
        for mask in (horizontal, end):
            self.assertTrue(self.validator.observe_near(mask, 100, 1, 100)["present"])
        # A ponta de fita pode ser curta na imagem porque continua além da borda.
        for width in (94, 95):
            end = np.zeros_like(self.line)
            end[301:, 205:205 + width] = 255
            self.assertTrue(self.validator.observe_near(end, 100, 1, 100)["present"])

    def test_expired_near_history_does_not_rearm_with_single_particle(self):
        self.arm()
        self.frame(self.fragment, seconds=1)
        self.assertFalse(self.validator.near_armed)
        self.frame(self.fragment, seconds=1.04)
        self.assertEqual(self.validator.decision, "NORMAL")

    def test_active_pivot_and_hard_corner_keep_priority(self):
        for tracker_name in ("virtual_turn_tracker", "pivot_state_tracker", "medium_spin_tracker"):
            self.setUp()
            self.arm()
            tracker = getattr(self.controller, tracker_name)
            tracker.state = "LEFT"
            self.validator.process_frame(self.fragment, self.maneuver, self.controller, 3, 100.1,
                                         reading(3, 100.1), 0.1, 100.1)
            self.assertEqual(self.validator.decision, "NORMAL")
            self.assertFalse(self.maneuver.gap_forward_active)
            self.assertEqual(tracker.state, "LEFT")


if __name__ == "__main__":
    unittest.main()
