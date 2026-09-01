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
        self.assertEqual(profile["rotation_degrees"], 180)

    def test_camera_profiles_keep_independent_orientations(self):
        down_settings = camera_line_frame.camera_transform_settings(
            camera_line_frame.CAMERA_PROFILES["down"]
        )
        forward_settings = camera_line_frame.camera_transform_settings(
            camera_line_frame.CAMERA_PROFILES["forward"]
        )

        self.assertEqual(down_settings["name"], "hvflip")
        self.assertTrue(down_settings["hflip"])
        self.assertTrue(down_settings["vflip"])
        self.assertEqual(forward_settings["name"], "identity")
        self.assertFalse(forward_settings["hflip"])
        self.assertFalse(forward_settings["vflip"])

    def test_down_geometry_contains_only_structural_limit(self):
        profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]
        geometry = camera_line_frame.resolve_vision_geometry(360, profile)
        self.assertEqual(geometry["structural_end_y"], 360)
        self.assertEqual(geometry["green_end_y"], 300)
        self.assertEqual(geometry["ignored_start_y"], 360)
        self.assertEqual(
            set(geometry),
            {
                "structural_end_y",
                "green_end_y",
                "ignored_start_y",
                "pixel_scale",
            },
        )

    def test_green_minimum_area_preserves_calibrated_roi(self):
        self.assertAlmostEqual(
            camera_line_frame.green_minimum_area(480, 300),
            5850.0,
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

    def test_structural_mask_removes_dead_zone_above_far(self):
        source = np.full((100, 80), 255, dtype=np.uint8)
        result = camera_line_frame.create_structural_line_mask(
            source,
            0,
            100,
            structural_start_y=20,
        )
        self.assertTrue(np.all(result[:20] == 0))
        self.assertTrue(np.all(result[20:] == 255))

    def test_line_candidate_mask_preserves_full_frame_component(self):
        profile = camera_line_frame.CAMERA_PROFILES["down"]["vision"]
        mask = np.full((319, 480), 255, dtype=np.uint8)
        result = camera_line_frame.create_line_candidate_mask(mask, profile)
        self.assertEqual(np.count_nonzero(result), mask.size)

    def test_empty_virtual_rows_have_zero_line_confidence(self):
        mask = np.zeros((100, 200), dtype=np.uint8)

        sensors = camera_line_frame.read_virtual_line_sensors(mask)

        self.assertEqual(sensors["farLineConfidence"], 0.0)
        self.assertEqual(sensors["mediumLineConfidence"], 0.0)
        self.assertEqual(sensors["farThicknessConsistency"], 0.0)
        self.assertEqual(sensors["mediumThicknessConsistency"], 0.0)
        self.assertFalse(sensors["farTrusted"])
        self.assertFalse(sensors["mediumTrusted"])

    def test_wide_black_tape_is_trusted_in_far(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        mask[47:155, 220:260] = 255

        sensors = camera_line_frame.read_virtual_line_sensors(mask)

        self.assertTrue(sensors["farTrusted"])

    def test_wide_black_tape_is_trusted_in_medium(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        mask[155:295, 220:260] = 255

        sensors = camera_line_frame.read_virtual_line_sensors(mask)

        self.assertTrue(sensors["mediumTrusted"])

    def test_wide_diagonal_tape_remains_trusted(self):
        cases = (
            ("far", (100, 140), (380, 60)),
            ("medium", (100, 285), (380, 170)),
        )
        for row_name, start_point, end_point in cases:
            with self.subTest(row=row_name):
                mask = np.zeros((360, 480), dtype=np.uint8)
                cv2.line(mask, start_point, end_point, 255, 40)

                sensors = camera_line_frame.read_virtual_line_sensors(mask)

                self.assertTrue(sensors[f"{row_name}Trusted"])

    def test_wide_right_angle_tape_remains_trusted(self):
        cases = (
            ("far", ((240, 145), (240, 95), (380, 95))),
            ("medium", ((240, 285), (240, 220), (380, 220))),
        )
        for row_name, points in cases:
            with self.subTest(row=row_name):
                mask = np.zeros((360, 480), dtype=np.uint8)
                cv2.line(mask, points[0], points[1], 255, 40)
                cv2.line(mask, points[1], points[2], 255, 40)

                sensors = camera_line_frame.read_virtual_line_sensors(mask)

                self.assertTrue(sensors[f"{row_name}Trusted"])

    def test_thin_horizontal_seam_is_not_trusted(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        mask[201:204, 50:430] = 255

        sensors = camera_line_frame.read_virtual_line_sensors(mask)

        self.assertFalse(sensors["mediumTrusted"])

    def test_untrusted_far_candidate_is_removed_from_control_reading(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        mask[80:96, 20:220] = 255

        sensors = camera_line_frame.read_virtual_line_sensors(mask)

        self.assertFalse(sensors["farTrusted"])
        self.assertIsNotNone(sensors["rawFarPosition"])
        self.assertIsNone(sensors["farPosition"])
        self.assertIsNotNone(sensors["rawFarBandPosition"])
        self.assertIsNone(sensors["farBandPosition"])
        self.assertEqual(
            (
                sensors["controlFarLeft"],
                sensors["controlFarCenter"],
                sensors["controlFarRight"],
                sensors["controlFarBandLeft"],
                sensors["controlFarBandCenter"],
                sensors["controlFarBandRight"],
            ),
            (0.0,) * 6,
        )

    def test_untrusted_medium_candidate_is_removed_from_control_reading(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        mask[200:216, 20:460] = 255

        sensors = camera_line_frame.read_virtual_line_sensors(mask)

        self.assertFalse(sensors["mediumTrusted"])
        self.assertIsNotNone(sensors["rawMediumPosition"])
        self.assertIsNone(sensors["mediumPosition"])
        self.assertEqual(
            (
                sensors["controlMediumLeft"],
                sensors["controlMediumCenter"],
                sensors["controlMediumRight"],
            ),
            (0.0,) * 3,
        )

    def test_thin_diagonal_seam_is_not_trusted(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        cv2.line(mask, (150, 285), (330, 165), 255, 3)

        sensors = camera_line_frame.read_virtual_line_sensors(mask)

        self.assertFalse(sensors["mediumTrusted"])

    def test_absolute_thickness_veto_rejects_high_confidence_candidate(self):
        measurement = {
            "lineConfidence": 1.0,
            "robustThicknessPx": 10.0,
            "thicknessConsistency": 1.0,
        }

        trusted = camera_line_frame.line_measurement_is_trusted(
            measurement,
            camera_line_frame.FAR_TRUST_MIN_CONFIDENCE,
            camera_line_frame.FAR_TRUST_MIN_THICKNESS_PX,
        )

        self.assertFalse(trusted)

    def test_high_confidence_consistent_thin_seam_is_not_trusted(self):
        measurement = {
            "lineConfidence": 0.95,
            "robustThicknessPx": 16.0,
            "thicknessConsistency": 0.95,
        }

        trusted = camera_line_frame.line_measurement_is_trusted(
            measurement,
            camera_line_frame.FAR_TRUST_MIN_CONFIDENCE,
            camera_line_frame.FAR_TRUST_MIN_THICKNESS_PX,
        )

        self.assertFalse(trusted)

    def test_zero_consistency_does_not_veto_thick_candidate(self):
        measurement = {
            "lineConfidence": 0.90,
            "robustThicknessPx": 60.0,
            "thicknessConsistency": 0.0,
        }

        trusted = camera_line_frame.line_measurement_is_trusted(
            measurement,
            camera_line_frame.MEDIUM_TRUST_MIN_CONFIDENCE,
            camera_line_frame.MEDIUM_TRUST_MIN_THICKNESS_PX,
        )

        self.assertTrue(trusted)

    def test_wide_continuous_line_has_high_line_confidence(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        mask[47:338, 220:260] = 255
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(mask.shape)

        far_measurement = camera_line_frame.measure_virtual_row_line_confidence(
            mask,
            geometry["far"],
        )
        medium_measurement = (
            camera_line_frame.measure_virtual_row_line_confidence(
                mask, geometry["medium"]
            )
        )

        self.assertGreater(far_measurement["lineConfidence"], 0.80)
        self.assertGreater(medium_measurement["lineConfidence"], 0.80)
        self.assertGreater(far_measurement["robustThicknessPx"], 30.0)
        self.assertGreater(medium_measurement["robustThicknessPx"], 30.0)

    def test_fragmented_component_has_lower_line_confidence(self):
        continuous = np.zeros((360, 480), dtype=np.uint8)
        continuous[47:155, 225:255] = 255
        fragments = np.zeros_like(continuous)
        for y in range(48, 154, 16):
            fragments[y:y + 5, 225:255] = 255
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            continuous.shape
        )

        continuous_confidence = (
            camera_line_frame.calculate_virtual_row_line_confidence(
                continuous,
                geometry["far"],
            )
        )
        fragment_confidence = (
            camera_line_frame.calculate_virtual_row_line_confidence(
                fragments,
                geometry["far"],
            )
        )

        self.assertLess(fragment_confidence, continuous_confidence)

    def test_long_thin_seam_has_low_line_confidence(self):
        thin_seam = np.zeros((360, 480), dtype=np.uint8)
        thin_seam[47:155, 239:242] = 255
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            thin_seam.shape
        )

        seam_measurement = (
            camera_line_frame.measure_virtual_row_line_confidence(
                thin_seam, geometry["far"]
            )
        )

        self.assertLess(seam_measurement["robustThicknessPx"], 5.0)
        self.assertLess(seam_measurement["thicknessScore"], 0.25)
        self.assertLess(seam_measurement["lineConfidence"], 0.55)

    def test_transverse_thickness_is_similar_in_all_orientations(self):
        shape = (360, 480)
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            shape
        )["medium"]
        directions = (
            ((240, 170), (240, 280)),
            ((100, 225), (380, 225)),
            ((140, 275), (340, 175)),
        )
        measured_thicknesses = []
        measured_consistencies = []
        for start_point, end_point in directions:
            mask = np.zeros(shape, dtype=np.uint8)
            cv2.line(mask, start_point, end_point, 255, 20)
            measurement = (
                camera_line_frame.measure_virtual_row_line_confidence(
                    mask, geometry
                )
            )
            measured_thicknesses.append(
                measurement["robustThicknessPx"]
            )
            measured_consistencies.append(
                measurement["thicknessConsistency"]
            )

        median_thickness = float(np.median(measured_thicknesses))
        for measured_thickness in measured_thicknesses:
            self.assertAlmostEqual(
                measured_thickness,
                median_thickness,
                delta=2.0,
            )
        for measured_consistency in measured_consistencies:
            self.assertGreater(measured_consistency, 0.90)
            self.assertAlmostEqual(
                measured_consistency,
                measured_consistencies[0],
                delta=0.10,
            )

    def test_uniform_thickness_has_high_consistency(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        mask[165:285, 230:250] = 255
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            mask.shape
        )

        measurement = camera_line_frame.measure_virtual_row_line_confidence(
            mask,
            geometry["medium"],
        )

        self.assertGreater(measurement["thicknessConsistency"], 0.90)
        expected_confidence = (
            0.60 * measurement["thicknessScore"]
            + 0.20 * measurement["thicknessConsistency"]
            + 0.15 * measurement["continuityScore"]
            + 0.05 * measurement["areaScore"]
        )
        self.assertAlmostEqual(
            measurement["lineConfidence"],
            expected_confidence,
        )

    def test_irregular_thickness_has_low_consistency(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        mask[165:205, 237:243] = 255
        mask[205:245, 222:258] = 255
        mask[245:285, 235:245] = 255
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            mask.shape
        )

        measurement = camera_line_frame.measure_virtual_row_line_confidence(
            mask,
            geometry["medium"],
        )

        self.assertLess(measurement["thicknessConsistency"], 0.40)

    def test_long_thin_horizontal_seam_has_low_thickness(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        mask[201:203, 50:430] = 255
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            mask.shape
        )

        measurement = camera_line_frame.measure_virtual_row_line_confidence(
            mask,
            geometry["medium"],
        )

        self.assertGreater(measurement["robustThicknessPx"], 0.0)
        self.assertLess(measurement["robustThicknessPx"], 5.0)
        self.assertLess(measurement["thicknessScore"], 0.25)
        self.assertLess(measurement["lineConfidence"], 0.30)

    def test_thick_horizontal_band_has_high_thickness(self):
        mask = np.zeros((360, 480), dtype=np.uint8)
        mask[190:230, 50:430] = 255
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            mask.shape
        )

        measurement = camera_line_frame.measure_virtual_row_line_confidence(
            mask,
            geometry["medium"],
        )

        self.assertGreater(measurement["robustThicknessPx"], 30.0)
        self.assertGreater(measurement["thicknessScore"], 0.90)
        self.assertGreater(measurement["lineConfidence"], 0.75)

    def test_expected_line_thickness_increases_toward_frame_bottom(self):
        frame_shape = (360, 480)

        top_thickness = camera_line_frame.expected_virtual_line_thickness_px(
            frame_shape, 0
        )
        bottom_thickness = (
            camera_line_frame.expected_virtual_line_thickness_px(
                frame_shape, frame_shape[0] - 1
            )
        )

        self.assertLess(top_thickness, bottom_thickness)

    def test_line_confidence_is_always_limited_to_unit_interval(self):
        masks = (
            np.zeros((100, 200), dtype=np.uint8),
            np.full((100, 200), 255, dtype=np.uint8),
        )
        geometry = camera_line_frame.resolve_virtual_sensor_geometry(
            masks[0].shape
        )

        for mask in masks:
            for row_name in ("far", "medium"):
                with self.subTest(mask_active=bool(np.any(mask)), row=row_name):
                    measurement = (
                        camera_line_frame.measure_virtual_row_line_confidence(
                            mask, geometry[row_name]
                        )
                    )
                    for score_name in (
                        "lineConfidence",
                        "thicknessScore",
                        "thicknessConsistency",
                        "continuityScore",
                        "areaScore",
                    ):
                        self.assertGreaterEqual(measurement[score_name], 0.0)
                        self.assertLessEqual(measurement[score_name], 1.0)

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
        self.assertTrue(np.array_equal(display[50, 80], (0, 255, 0)))
        for current, original in zip(
            (raw, structural, candidates, green),
            originals,
        ):
            self.assertTrue(np.array_equal(current, original))

    def test_green_overlays_preserve_candidates_decisions_and_rois(self):
        contour = rectangle_contour(45, 35, 75, 65)
        candidate = {
            "contour": contour,
            "centroid": (60.0, 50.0),
        }
        rejected_frame = np.zeros((100, 120, 3), dtype=np.uint8)
        accepted_frame = np.zeros_like(rejected_frame)
        line_mode_frame = np.zeros_like(rejected_frame)

        camera_line_frame.draw_green_candidate_overlays(
            rejected_frame,
            [candidate],
            "DIREITA",
            False,
        )
        camera_line_frame.draw_green_candidate_overlays(
            accepted_frame,
            [candidate],
            "DIREITA",
            True,
        )
        camera_line_frame.draw_line_mode_green_overlays(
            line_mode_frame,
            [candidate],
            "DIREITA",
            True,
        )
        roi_interpretation = camera_line_frame.analyze_green_marker_contours(
            [contour],
            np.zeros((100, 120), dtype=np.uint8),
        )
        camera_line_frame.draw_green_roi_overlays(
            line_mode_frame,
            roi_interpretation,
        )

        self.assertGreater(np.count_nonzero(rejected_frame), 0)
        self.assertGreater(np.count_nonzero(accepted_frame), 0)
        self.assertGreater(np.count_nonzero(line_mode_frame), 0)
        self.assertFalse(np.array_equal(rejected_frame, accepted_frame))

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

    def test_green_mask_ignores_dead_zone_above_far(self):
        green_pixel = camera_line_frame.rgb_pixel_to_camera_array(0, 180, 0)
        frame = np.full((40, 60, 3), green_pixel, dtype=np.uint8)
        mask = camera_line_frame.create_green_mask(
            frame,
            40,
            green_start_y=10,
        )
        self.assertEqual(np.count_nonzero(mask[:10]), 0)
        self.assertGreater(np.count_nonzero(mask[10:]), 0)

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

    def test_green_classification_does_not_override_line_follower_directly(self):
        mask = np.full((40, 60), 255, dtype=np.uint8)
        baseline = camera_line_frame.calculate_line_follower_command(
            mask,
            {},
        )
        classified = camera_line_frame.calculate_line_follower_command(
            mask,
            {"greenInterpretation": "RETORNO_180"},
        )
        self.assertEqual(
            (classified["left_power"], classified["right_power"]),
            (baseline["left_power"], baseline["right_power"]),
        )
        self.assertEqual(classified["controlSource"], "virtual")

    def test_fast_status_preserves_virtual_baseline_and_green_fields(self):
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
                        "controlSource": "virtual",
                        "nearCenter": 0.20,
                        "nearFinePosition": 0.12,
                        "mediumPosition": 0.08,
                        "mediumLineConfidence": 0.64,
                        "mediumThicknessConsistency": 0.58,
                        "mediumTrusted": False,
                        "farLineConfidence": 0.72,
                        "farThicknessConsistency": 0.81,
                        "farTrusted": True,
                        "farBandPosition": -0.04,
                        "headingAngle": 5.5,
                        "finalSteering": 0.18,
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
        self.assertTrue(published["lineNearDetected"])
        self.assertEqual(published["lineControlSource"], "virtual")
        self.assertEqual(published["greenInterpretation"], "ESQUERDA")
        self.assertEqual(published["nearFinePosition"], 0.12)
        self.assertIsNone(published["mediumPosition"])
        self.assertEqual(published["mediumLineConfidence"], 0.64)
        self.assertEqual(published["mediumThicknessConsistency"], 0.58)
        self.assertFalse(published["mediumTrusted"])
        self.assertEqual(published["farLineConfidence"], 0.72)
        self.assertEqual(published["farThicknessConsistency"], 0.81)
        self.assertTrue(published["farTrusted"])
        self.assertEqual(published["farBandPosition"], -0.04)
        self.assertEqual(published["headingAngleDeg"], 5.5)
        self.assertEqual(published["finalSteering"], 0.18)
        self.assertEqual(published["vstate"], "NORMAL")
        self.assertEqual(published["lineState"], "LINE")
        self.assertEqual(published["trustedDirection"], "NONE")
        expected_keys = set(status) | {
            "lineFollowerLeftPower",
            "lineFollowerRightPower",
            "lineNearDetected",
            "lineControlSource",
            "nearFinePosition",
            "mediumPosition",
            "mediumLineConfidence",
            "mediumThicknessConsistency",
            "mediumTrusted",
            "farLineConfidence",
            "farThicknessConsistency",
            "farTrusted",
            "farBandPosition",
            "headingAngleDeg",
            "finalSteering",
            "vstate",
            "lineState",
            "trustedDirection",
            "lineTimestamp",
            "lineSequence",
            "specularRepairPixels",
            "specularRepairComponents",
        }
        self.assertEqual(set(published), expected_keys)

    def test_camera_status_exposes_far_and_medium_line_confidence(self):
        with tempfile.TemporaryDirectory() as directory:
            original_status_path = camera_line_frame.STATUS_PATH
            original_temp_path = camera_line_frame.TEMP_STATUS_PATH
            camera_line_frame.STATUS_PATH = os.path.join(
                directory,
                "camera-status.json",
            )
            camera_line_frame.TEMP_STATUS_PATH = os.path.join(
                directory,
                "camera-status.tmp.json",
            )
            try:
                camera_line_frame.save_status(
                    30.0,
                    camera_line_frame.CAMERA_PROFILES["down"],
                    {},
                    far_line_confidence=0.41,
                    medium_line_confidence=0.62,
                    far_thickness_consistency=0.73,
                    medium_thickness_consistency=0.54,
                )
                with open(
                    camera_line_frame.STATUS_PATH,
                    encoding="utf-8",
                ) as status_file:
                    published = json.load(status_file)
            finally:
                camera_line_frame.STATUS_PATH = original_status_path
                camera_line_frame.TEMP_STATUS_PATH = original_temp_path

        self.assertEqual(published["farLineConfidence"], 0.41)
        self.assertEqual(published["mediumLineConfidence"], 0.62)
        self.assertEqual(published["farThicknessConsistency"], 0.73)
        self.assertEqual(published["mediumThicknessConsistency"], 0.54)


if __name__ == "__main__":
    unittest.main()
