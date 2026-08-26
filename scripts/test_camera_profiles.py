import json
import os
import tempfile
import unittest

import cv2
import numpy as np

import camera_line_frame


def rectangle_contour(x1, y1, x2, y2):
    return np.array(
        [[[x1, y1]], [[x2, y1]], [[x2, y2]], [[x1, y2]]],
        dtype=np.int32,
    )


def paint_roi(mask, roi):
    x1, y1, x2, y2 = (int(value) for value in roi)
    clipped_x1 = max(0, min(mask.shape[1], x1))
    clipped_y1 = max(0, min(mask.shape[0], y1))
    clipped_x2 = max(0, min(mask.shape[1], x2))
    clipped_y2 = max(0, min(mask.shape[0], y2))
    mask[clipped_y1:clipped_y2, clipped_x1:clipped_x2] = 255


def marker_mask(shape, marker_settings):
    mask = np.zeros(shape, dtype=np.uint8)
    for contour, enabled_sections in marker_settings:
        geometry = camera_line_frame.green_marker_roi_geometry(
            contour,
            shape[1],
        )
        horizontal_x1, horizontal_y1, horizontal_x2, horizontal_y2 = (
            geometry["horizontal_roi"]
        )
        marker_left_x, marker_right_x = geometry[
            "marker_horizontal_bounds"
        ]
        sections = {
            "upper": geometry["upper_roi"],
            "horizontal_left": (
                horizontal_x1,
                horizontal_y1,
                marker_left_x,
                horizontal_y2,
            ),
            "horizontal_right": (
                marker_right_x,
                horizontal_y1,
                horizontal_x2,
                horizontal_y2,
            ),
        }
        for section_name in enabled_sections:
            paint_roi(mask, sections[section_name])
    return mask


class CameraProfilesTest(unittest.TestCase):
    def test_dual_camera_assignments_keep_configured_roles(self):
        camera_infos = [
            {"Num": 0, "Model": "down"},
            {"Num": 1, "Model": "forward"},
        ]
        assignments = camera_line_frame.resolve_camera_assignments(
            camera_infos,
            {"down": 0, "forward": 1},
        )
        self.assertTrue(assignments["down"]["available"])
        self.assertTrue(assignments["forward"]["available"])
        self.assertEqual(assignments["down"]["info"]["Model"], "down")
        self.assertEqual(assignments["forward"]["info"]["Model"], "forward")

    def test_camera_indices_reject_duplicate_roles(self):
        with self.assertRaises(ValueError):
            camera_line_frame.configured_camera_indices({
                "OBR_DOWNWARD_CAMERA_INDEX": "0",
                "OBR_FORWARD_CAMERA_INDEX": "0",
            })

    def test_only_downward_role_publishes_line_status(self):
        self.assertTrue(
            camera_line_frame.camera_role_publishes_line_status("down")
        )
        self.assertFalse(
            camera_line_frame.camera_role_publishes_line_status("forward")
        )

    def test_down_profile_preserves_validated_capture(self):
        profile = camera_line_frame.CAMERA_PROFILES["down"]
        self.assertEqual(profile["main_size"], (480, 360))
        self.assertEqual(profile["sensor_size"], (1640, 1232))
        self.assertEqual(profile["target_fps"], 30)
        self.assertEqual(camera_line_frame.CAMERA_ROTATION_DEGREES, 180)

    def test_down_geometry_contains_only_structural_limit(self):
        profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]
        geometry = camera_line_frame.resolve_vision_geometry(360, profile)
        self.assertEqual(geometry["structural_end_y"], 319)
        self.assertEqual(geometry["ignored_start_y"], 319)
        self.assertEqual(
            set(geometry),
            {"structural_end_y", "ignored_start_y", "pixel_scale"},
        )

    def test_black_mask_keeps_dark_tape_and_rejects_white_floor(self):
        profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]
        gray = np.full((120, 200), 205, dtype=np.uint8)
        gray[:, 90:110] = 25
        mask = camera_line_frame.create_line_binary_mask(gray, profile)
        self.assertGreater(np.count_nonzero(mask[:, 94:106]), 0)
        self.assertEqual(np.count_nonzero(mask[:, :50]), 0)

    def test_structural_mask_removes_ignored_bottom_region(self):
        source = np.full((100, 80), 255, dtype=np.uint8)
        result = camera_line_frame.create_structural_line_mask(source, 0, 70)
        self.assertTrue(np.all(result[:70] == 255))
        self.assertTrue(np.all(result[70:] == 0))

    def test_line_candidate_mask_rejects_giant_dark_component(self):
        profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]
        mask = np.full((319, 480), 255, dtype=np.uint8)
        result = camera_line_frame.create_line_candidate_mask(mask, profile)
        self.assertEqual(np.count_nonzero(result), 0)

    def test_display_line_mode_does_not_modify_source_masks(self):
        raw = np.zeros((80, 120, 3), dtype=np.uint8)
        structural = np.zeros((80, 120), dtype=np.uint8)
        candidates = np.zeros_like(structural)
        green = np.zeros_like(structural)
        structural[10:40, 20:50] = 255
        candidates[15:35, 25:45] = 255
        green[45:60, 70:90] = 255
        originals = tuple(item.copy() for item in (
            raw,
            structural,
            candidates,
            green,
        ))
        display = camera_line_frame.create_display_frame(
            raw,
            candidates,
            green,
            0,
            camera_line_frame.DISPLAY_MODE_LINE,
            structural,
        )
        self.assertTrue(np.any(display != 0))
        for current, original in zip(
            (raw, structural, candidates, green),
            originals,
        ):
            self.assertTrue(np.array_equal(current, original))

    def test_green_hsv_rejects_non_green_colors(self):
        green_pixel = camera_line_frame.rgb_pixel_to_camera_array(0, 180, 0)
        frame = np.zeros((40, 100, 3), dtype=np.uint8)
        frame[:, 0:20] = green_pixel
        frame[:, 20:40] = camera_line_frame.rgb_pixel_to_camera_array(
            255, 255, 255
        )
        frame[:, 40:60] = camera_line_frame.rgb_pixel_to_camera_array(0, 0, 0)
        frame[:, 60:80] = camera_line_frame.rgb_pixel_to_camera_array(255, 0, 0)
        frame[:, 80:100] = camera_line_frame.rgb_pixel_to_camera_array(
            255, 255, 0
        )
        mask = camera_line_frame.create_green_mask(frame, 40)
        self.assertGreater(np.count_nonzero(mask[:, 0:20]), 0)
        self.assertEqual(np.count_nonzero(mask[:, 20:]), 0)

    def test_green_fragments_merge_into_one_candidate(self):
        contours = [
            rectangle_contour(100, 100, 130, 150),
            rectangle_contour(136, 100, 166, 150),
        ]
        merged = camera_line_frame.merge_green_fragments(contours, 480)
        self.assertEqual(len(merged), 1)

    def test_green_geometry_preserves_strong_partial_candidate(self):
        contour = rectangle_contour(0, 120, 80, 200)
        description = camera_line_frame.describe_green_contour(
            contour,
            480,
            319,
            360,
        )
        self.assertTrue(description["partial"])
        self.assertTrue(description["geometry_valid"])

    def test_green_geometry_exposes_only_two_perpendicular_rois(self):
        contour = rectangle_contour(200, 180, 250, 230)
        geometry = camera_line_frame.green_marker_roi_geometry(contour, 480)
        self.assertIn("horizontal_roi", geometry)
        self.assertIn("upper_roi", geometry)
        self.assertNotIn("left_roi", geometry)
        self.assertNotIn("right_roi", geometry)
        horizontal = geometry["horizontal_roi"]
        upper = geometry["upper_roi"]
        self.assertGreater(
            horizontal[2] - horizontal[0],
            horizontal[3] - horizontal[1],
        )
        self.assertGreater(upper[3] - upper[1], upper[2] - upper[0])

    def test_green_without_local_black_is_false(self):
        contour = rectangle_contour(200, 180, 250, 230)
        result = camera_line_frame.analyze_green_marker_contours(
            [contour],
            np.zeros((319, 480), dtype=np.uint8),
        )
        self.assertEqual(result["interpretation"], "VERDE_FALSO")
        self.assertFalse(result["path_black_valid"])

    def test_horizontal_black_does_not_bypass_white_upper_roi(self):
        contour = rectangle_contour(200, 180, 250, 230)
        mask = marker_mask(
            (319, 480),
            [(contour, ("horizontal_left",))],
        )
        result = camera_line_frame.analyze_green_marker_contours(
            [contour],
            mask,
        )
        self.assertEqual(result["interpretation"], "VERDE_FALSO")
        self.assertTrue(result["markers"][0]["upper"]["measured"])
        self.assertFalse(result["markers"][0]["upper"]["valid"])

    def test_green_without_upper_measurement_is_ambiguous(self):
        contour = rectangle_contour(200, 0, 250, 30)
        result = camera_line_frame.analyze_green_marker_contours(
            [contour],
            np.zeros((319, 480), dtype=np.uint8),
        )
        self.assertEqual(result["interpretation"], "AMBIGUO")
        self.assertFalse(result["markers"][0]["upper"]["measured"])

    def test_missing_upper_measurement_blocks_other_marker_decision(self):
        unmeasured = rectangle_contour(80, 0, 130, 30)
        measured = rectangle_contour(300, 180, 350, 230)
        mask = marker_mask(
            (319, 480),
            [(measured, ("upper", "horizontal_left"))],
        )
        result = camera_line_frame.analyze_green_marker_contours(
            [unmeasured, measured],
            mask,
        )
        self.assertEqual(result["interpretation"], "AMBIGUO")
        self.assertFalse(result["path_black_valid"])

    def test_horizontal_left_black_classifies_right_marker(self):
        contour = rectangle_contour(200, 180, 250, 230)
        mask = marker_mask(
            (319, 480),
            [(contour, ("upper", "horizontal_left"))],
        )
        result = camera_line_frame.analyze_green_marker_contours(
            [contour],
            mask,
        )
        self.assertEqual(result["interpretation"], "DIREITA")
        self.assertTrue(result["path_black_valid"])

    def test_horizontal_right_black_classifies_left_marker(self):
        contour = rectangle_contour(200, 180, 250, 230)
        mask = marker_mask(
            (319, 480),
            [(contour, ("upper", "horizontal_right"))],
        )
        result = camera_line_frame.analyze_green_marker_contours(
            [contour],
            mask,
        )
        self.assertEqual(result["interpretation"], "ESQUERDA")
        self.assertTrue(result["path_black_valid"])

    def test_black_on_both_horizontal_sides_is_ambiguous(self):
        contour = rectangle_contour(200, 180, 250, 230)
        mask = marker_mask(
            (319, 480),
            [(
                contour,
                ("upper", "horizontal_left", "horizontal_right"),
            )],
        )
        result = camera_line_frame.analyze_green_marker_contours(
            [contour],
            mask,
        )
        self.assertEqual(result["interpretation"], "AMBIGUO")
        self.assertFalse(result["path_black_valid"])

    def test_compatible_opposite_markers_classify_return(self):
        left = rectangle_contour(90, 180, 140, 230)
        right = rectangle_contour(320, 182, 370, 232)
        mask = marker_mask(
            (319, 480),
            [
                (left, ("upper", "horizontal_right")),
                (right, ("upper", "horizontal_left")),
            ],
        )
        result = camera_line_frame.analyze_green_marker_contours(
            [left, right],
            mask,
        )
        self.assertEqual(result["interpretation"], "RETORNO_180")
        self.assertTrue(result["pair_compatible"])
        self.assertTrue(result["path_black_valid"])

    def test_vertically_incompatible_pair_is_ambiguous(self):
        left = rectangle_contour(90, 80, 130, 120)
        right = rectangle_contour(320, 210, 360, 250)
        mask = marker_mask(
            (319, 480),
            [
                (left, ("upper", "horizontal_right")),
                (right, ("upper", "horizontal_left")),
            ],
        )
        result = camera_line_frame.analyze_green_marker_contours(
            [left, right],
            mask,
        )
        self.assertEqual(result["interpretation"], "AMBIGUO")
        self.assertFalse(result["pair_compatible"])

    def test_green_tracker_requires_new_consecutive_frames(self):
        tracker = camera_line_frame.GreenObservationTracker()
        first = tracker.update(1, "ESQUERDA", 1.0)
        repeated = tracker.update(1, "ESQUERDA", 1.1)
        second = tracker.update(2, "ESQUERDA", 1.2)
        self.assertEqual(first, ("SEM_DECISAO", False, 1))
        self.assertEqual(repeated, first)
        self.assertEqual(second, ("ESQUERDA", True, 2))

    def test_green_tracker_never_confirms_ambiguous(self):
        tracker = camera_line_frame.GreenObservationTracker()
        tracker.update(1, "AMBIGUO", 1.0)
        result = tracker.update(2, "AMBIGUO", 1.1)
        self.assertEqual(result, ("AMBIGUO", False, 2))

    def test_line_follower_extension_stays_stopped(self):
        result = camera_line_frame.calculate_line_follower_command(
            np.full((40, 60), 255, dtype=np.uint8),
            {"greenInterpretation": "RETORNO_180"},
        )
        self.assertEqual(result, {"left_power": 0.0, "right_power": 0.0})

    def test_fast_status_has_no_legacy_control_fields(self):
        status = camera_line_frame.empty_green_status()
        status.update({
            "greenInterpretation": "ESQUERDA",
            "greenConfirmed": True,
            "greenPathBlackValid": True,
            "greenCandidateCount": 1,
        })
        with tempfile.TemporaryDirectory() as directory:
            original_status_path = camera_line_frame.LINE_STATUS_PATH
            original_temp_path = camera_line_frame.TEMP_LINE_STATUS_PATH
            camera_line_frame.LINE_STATUS_PATH = os.path.join(
                directory,
                "line.json",
            )
            camera_line_frame.TEMP_LINE_STATUS_PATH = os.path.join(
                directory,
                "line.tmp.json",
            )
            try:
                camera_line_frame.save_line_status(
                    {
                        "left_power": 0.0,
                        "right_power": 0.0,
                        "farAngle60": 42.0,
                        "farPathAngleDeg": 48.0,
                        "farConsensusDirection": "RIGHT",
                        "farConsensusConfirmFrames": 2,
                        "curveIntentConfirmFrames": 1,
                        "curveIntentReleaseFrames": 0,
                        "pathAmbiguous": True,
                        "virtualState": "NORMAL",
                        "lineState": "LINE",
                    },
                    123.0,
                    7,
                    status,
                )
                with open(
                    camera_line_frame.LINE_STATUS_PATH,
                    encoding="utf-8",
                ) as status_file:
                    published = json.load(status_file)
            finally:
                camera_line_frame.LINE_STATUS_PATH = original_status_path
                camera_line_frame.TEMP_LINE_STATUS_PATH = original_temp_path

        self.assertEqual(published["lineFollowerLeftPower"], 0.0)
        self.assertEqual(published["lineFollowerRightPower"], 0.0)
        self.assertFalse(published["lineNearDetected"])
        self.assertEqual(published["greenInterpretation"], "ESQUERDA")
        self.assertEqual(published["curveIntentState"], "NONE")
        self.assertFalse(published["curveIntentApplied"])
        self.assertIsNone(published["dynamicTargetAngleDeg"])
        self.assertEqual(published["farAngle60"], 42.0)
        self.assertIsNone(published["farAngle75"])
        self.assertEqual(published["farPathAngleDeg"], 48.0)
        self.assertEqual(published["farConsensusDirection"], "RIGHT")
        self.assertEqual(published["farConsensusConfirmFrames"], 2)
        self.assertEqual(published["curveIntentConfirmFrames"], 1)
        self.assertEqual(published["curveIntentReleaseFrames"], 0)
        self.assertTrue(published["pathAmbiguous"])
        self.assertEqual(published["vstate"], "NORMAL")
        self.assertEqual(published["lineState"], "LINE")
        expected_keys = set(status) | {
            "lineFollowerLeftPower",
            "lineFollowerRightPower",
            "lineNearDetected",
            "lineControlSource",
            "curveIntentState",
            "curveIntentApplied",
            "curveIntentConfirmFrames",
            "curveIntentReleaseFrames",
            "farAngle60",
            "farAngle75",
            "farAngle90",
            "farAngleSpread",
            "dynamicTargetAngleDeg",
            "dynamicLookaheadPx",
            "farPathAngleDeg",
            "farConsensusDirection",
            "farConsensusConfirmFrames",
            "nearPosition",
            "mediumPosition",
            "farBandPosition",
            "headingAngleDeg",
            "baseVirtualSteering",
            "hybridSteering",
            "finalSteering",
            "pathAmbiguous",
            "vstate",
            "lineState",
            "lineTimestamp",
            "lineSequence",
            "specularRepairPixels",
            "specularRepairComponents",
        }
        self.assertEqual(set(published), expected_keys)


if __name__ == "__main__":
    unittest.main()
