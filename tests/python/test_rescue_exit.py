import json
import sys
import unittest
from pathlib import Path
from unittest import mock

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from vision.rescue_exit import (
    analyze_exit_candidates,
    create_exit_black_mask,
    create_exit_display_frame,
    draw_exit_overlay,
    exit_line_is_unbranched,
    read_exit_control,
    remove_rescue_zones_from_black,
    rescue_zone_blocked_sectors,
    unbranched_black_path,
)


class RescueExitVisionTest(unittest.TestCase):
    def test_distant_deformed_mass_is_accepted_without_near_tape(self):
        mask = np.zeros((240, 320), np.uint8)
        cv2.fillPoly(mask, [np.array([(150, 10), (205, 15), (180, 40), (148, 25)])], 255)
        candidates = analyze_exit_candidates(mask, 120)
        visible = [c for c in candidates.values() if c["visible"]]
        self.assertTrue(visible)
        self.assertTrue(all(c["nearestBand"] == 0 and not c["tapeValid"] for c in visible))

    def test_particles_are_ignored(self):
        mask = np.zeros((240, 320), np.uint8)
        mask[::15, ::15] = 255
        self.assertFalse(any(c["visible"] for c in analyze_exit_candidates(mask, 120).values()))

    def test_multiple_candidates_and_depth_continuity(self):
        mask = np.zeros((240, 320), np.uint8)
        cv2.line(mask, (40, 10), (65, 220), 255, 8)
        cv2.rectangle(mask, (260, 10), (280, 40), 255, -1)
        candidates = analyze_exit_candidates(mask, 120)
        self.assertTrue(candidates["sector0"]["visible"])
        self.assertTrue(candidates["sector4"]["visible"])
        self.assertEqual(candidates["sector0"]["depthBands"], 3)
        self.assertGreater(candidates["sector0"]["score"], candidates["sector4"]["score"])
        self.assertLess(candidates["sector0"]["txDegrees"], 0)
        self.assertGreater(candidates["sector4"]["txDegrees"], 0)

    def test_two_point_guidance_matches_fusion_angle_convention(self):
        for far_x, comparison in [(70, "left"), (160, "straight"), (250, "right")]:
            with self.subTest(far_x=far_x):
                mask = np.zeros((240, 320), np.uint8)
                cv2.line(mask, (160, 239), (far_x, 15), 255, 14)
                candidates = analyze_exit_candidates(mask, 120)
                candidate = max(candidates.values(), key=lambda item: item["score"])
                self.assertTrue(candidate["guidanceValid"])
                self.assertEqual(candidate["nearPoint"], {"x": 160, "y": 239})
                self.assertIsNotNone(candidate["farPoint"])
                angle = candidate["guidanceAngleDegrees"]
                if comparison == "left":
                    self.assertLess(angle, 90.0)
                elif comparison == "right":
                    self.assertGreater(angle, 90.0)
                else:
                    self.assertAlmostEqual(angle, 90.0, delta=1.0)

    def test_transverse_entry_tape_cannot_steal_guidance_from_continuation(self):
        mask = np.zeros((240, 320), np.uint8)
        cv2.rectangle(mask, (0, 185), (215, 197), 255, -1)
        cv2.line(mask, (235, 180), (150, 55), 255, 10)

        candidates = analyze_exit_candidates(mask, 120)
        visible = [candidate for candidate in candidates.values()
                   if candidate["visible"]]

        self.assertEqual(len(visible), 1)
        self.assertLess(visible[0]["farPoint"]["y"], 100)
        self.assertGreater(visible[0]["farPoint"]["x"], 130)

    def test_tiny_distant_fragment_is_rejected(self):
        mask = np.zeros((240, 320), np.uint8)
        cv2.line(mask, (172, 42), (180, 55), 255, 4)

        candidates = analyze_exit_candidates(mask, 120)

        self.assertFalse(any(candidate["guidanceValid"]
                             for candidate in candidates.values()))

    def test_low_contrast_distant_black_generates_guidance(self):
        frame = np.full((240, 320, 3), (205, 240, 203), np.uint8)
        cv2.line(frame, (80, 115), (245, 70), (130, 160, 125), 6)

        mask = create_exit_black_mask(frame)
        candidates = analyze_exit_candidates(mask, 120)

        self.assertTrue(any(candidate["guidanceValid"]
                            for candidate in candidates.values()))

    def test_close_longitudinal_black_below_distant_far_limit_is_kept(self):
        frame = np.full((240, 320, 3), (205, 240, 203), np.uint8)
        cv2.line(frame, (160, 239), (170, 158), (120, 150, 110), 14)

        candidates = analyze_exit_candidates(create_exit_black_mask(frame), 120)

        self.assertTrue(any(candidate["guidanceValid"]
                            for candidate in candidates.values()))

    def test_lateral_entry_points_right_before_left_continuation(self):
        mask = np.zeros((240, 320), np.uint8)
        cv2.line(mask, (285, 220), (135, 65), 255, 12)
        candidates = analyze_exit_candidates(mask, 120)
        candidate = max(candidates.values(), key=lambda item: item["score"])

        self.assertTrue(candidate["guidanceValid"])
        self.assertGreater(candidate["entryOffsetNormalized"], 0.35)
        self.assertGreater(candidate["entryDepthNormalized"], 0.85)
        self.assertGreater(candidate["entryAngleDegrees"], 90.0)
        self.assertLess(candidate["guidanceAngleDegrees"], 90.0)

    def test_shallow_distant_seam_does_not_generate_guidance(self):
        mask = np.zeros((240, 320), np.uint8)
        cv2.line(mask, (20, 100), (300, 120), 255, 4)

        candidates = analyze_exit_candidates(mask, 120)

        self.assertFalse(any(candidate["guidanceValid"]
                             for candidate in candidates.values()))

    def test_thin_wall_fissure_is_rejected_at_camera_resolution(self):
        mask = np.zeros((540, 960), np.uint8)
        cv2.line(mask, (870, 539), (790, 390), 255, 2)

        candidates = analyze_exit_candidates(mask, 120)

        self.assertFalse(any(candidate["guidanceValid"]
                             for candidate in candidates.values()))

    def test_forward_tape_outranks_distant_lateral_corner(self):
        mask = np.zeros((540, 960), np.uint8)
        cv2.line(mask, (475, 300), (455, 180), 255, 10)
        cv2.line(mask, (250, 190), (225, 125), 255, 8)

        candidates = analyze_exit_candidates(mask, 120)
        visible = [candidate for candidate in candidates.values()
                   if candidate["guidanceValid"]]

        self.assertGreaterEqual(len(visible), 2)
        best = max(visible, key=lambda candidate: candidate["score"])
        self.assertLess(abs(best["farPoint"]["x"] - 480), 80)

    def test_top_side_frame_edge_is_not_a_floor_path(self):
        mask = np.zeros((540, 960), np.uint8)
        cv2.rectangle(mask, (0, 0), (22, 330), 255, -1)
        cv2.line(mask, (610, 330), (720, 190), 255, 12)

        candidates = analyze_exit_candidates(mask, 120)
        visible = [candidate for candidate in candidates.values()
                   if candidate["guidanceValid"]]

        self.assertEqual(len(visible), 1)
        self.assertGreater(visible[0]["farPoint"]["x"], 500)

    def test_continuous_tape_outranks_fragmented_noisy_strip(self):
        mask = np.zeros((540, 960), np.uint8)
        cv2.line(mask, (610, 355), (690, 175), 255, 12)
        for index, y in enumerate(range(170, 350, 18)):
            center_x = 220 + (18 if index % 2 else -12)
            cv2.rectangle(mask, (center_x - 9, y),
                          (center_x + 9, y + 9), 255, -1)

        candidates = analyze_exit_candidates(mask, 120)
        visible = [candidate for candidate in candidates.values()
                   if candidate["guidanceValid"]]

        self.assertEqual(len(visible), 1)
        best = max(visible, key=lambda candidate: candidate["score"])
        self.assertTrue(best["solidBlack"])
        self.assertGreater(best["farPoint"]["x"], 500)

    def test_reflective_fragmented_corridor_does_not_publish_guidance(self):
        mask = np.zeros((240, 320), np.uint8)
        for y in range(35, 220):
            center = 150 + (y % 9) - 4
            if y % 5 != 0:
                mask[y, center - 6:center - 1] = 255
            if y % 4 != 0:
                mask[y, center + 2:center + 8] = 255

        candidates = analyze_exit_candidates(mask, 120)
        noisy = [candidate for candidate in candidates.values()
                 if candidate.get("grayNoiseLikely")]

        self.assertTrue(noisy)
        self.assertFalse(any(candidate["guidanceValid"] for candidate in noisy))
        self.assertTrue(all(not candidate["blockedByColor"] for candidate in noisy))

    def test_horizontal_tape_alone_never_publishes_guidance(self):
        mask = np.zeros((240, 320), np.uint8)
        cv2.rectangle(mask, (0, 180), (319, 192), 255, -1)

        candidates = analyze_exit_candidates(mask, 120)

        self.assertFalse(any(candidate["visible"]
                             for candidate in candidates.values()))

    def test_thick_perspective_entry_tape_in_lower_third_is_not_guidance(self):
        mask = np.zeros((240, 320), np.uint8)
        cv2.fillPoly(mask, [np.array([
            (0, 177), (190, 160), (220, 191), (0, 225),
        ])], 255)

        candidates = analyze_exit_candidates(mask, 120)

        self.assertFalse(any(candidate["guidanceValid"]
                             for candidate in candidates.values()))

    def test_exit_line_mode_shows_the_exact_black_mask_full_frame(self):
        frame = np.full((240, 320, 3), 180, np.uint8)
        mask = np.zeros((240, 320), np.uint8)
        cv2.line(mask, (160, 210), (180, 40), 255, 8)
        reading = {"exitCandidates": analyze_exit_candidates(mask, 120)}

        display = create_exit_display_frame(frame, mask, "line")
        draw_exit_overlay(display, reading, {"sector": 2}, mask)

        self.assertTrue(np.any(np.all(display == 255, axis=2)))
        self.assertTrue(np.any(np.all(display == 0, axis=2)))
        self.assertTrue(np.all(create_exit_display_frame(frame, mask, "real") == frame))

    def test_forward_strip_survives_wide_transverse_intersections(self):
        mask = np.zeros((540, 960), np.uint8)
        cv2.rectangle(mask, (405, 195), (500, 420), 255, -1)
        cv2.line(mask, (0, 405), (959, 380), 255, 24)
        cv2.line(mask, (0, 228), (620, 220), 255, 12)

        candidates = analyze_exit_candidates(mask, 120)
        candidate = max(candidates.values(), key=lambda item: item["score"])

        self.assertTrue(candidate["guidanceValid"])
        self.assertLess(candidate["farPoint"]["y"], 270)
        self.assertGreater(len(candidate["pathPoints"]), 10)

    def test_green_and_red_zones_block_their_exit_sectors(self):
        for color, rectangle, expected_sector in [
                ((0, 255, 0), (5, 20, 55, 150), 0),
                ((0, 0, 255), (265, 20, 315, 150), 4),
        ]:
            with self.subTest(color=color):
                frame = np.full((240, 320, 3), 220, np.uint8)
                cv2.rectangle(frame, rectangle[:2], rectangle[2:], color, -1)
                mask = np.zeros((240, 320), np.uint8)
                cv2.rectangle(mask, rectangle[:2], rectangle[2:], 255, -1)
                cv2.rectangle(mask, (145, 20), (175, 150), 255, -1)
                blocked = rescue_zone_blocked_sectors(frame)
                candidates = analyze_exit_candidates(mask, 120, blocked)
                self.assertIn(expected_sector, blocked)
                self.assertFalse(candidates[f"sector{expected_sector}"]["visible"])
                self.assertTrue(candidates["sector2"]["visible"])

    def test_colored_hull_removal_preserves_black_in_same_sector(self):
        frame = np.full((240, 320, 3), 220, np.uint8)
        black = np.zeros((240, 320), np.uint8)
        cv2.rectangle(black, (130, 30), (145, 150), 255, -1)
        cv2.rectangle(black, (155, 30), (165, 150), 255, -1)
        hull = np.asarray([(126, 20), (149, 20), (149, 160), (126, 160)],
                          dtype=np.int32).reshape(-1, 1, 2)

        with mock.patch("vision.rescue_exit.analyze_rescue_zones", return_value={
                "green": {"detected": True, "_hull": hull},
                "red": {"detected": False, "_hull": None},
        }):
            cleaned, removed, blocked = remove_rescue_zones_from_black(frame, black)

        self.assertTrue(removed)
        self.assertIn(2, blocked)
        self.assertEqual(int(cleaned[80, 135]), 0)
        self.assertEqual(int(cleaned[80, 160]), 255)
        self.assertTrue(any(candidate["guidanceValid"]
                            for candidate in analyze_exit_candidates(cleaned, 120).values()))

    def test_straight_and_single_curves(self):
        for points in (((80, 0), (80, 119)), ((80, 119), (80, 60), (25, 20)),
                       ((80, 119), (80, 60), (0, 60))):
            with self.subTest(points=points):
                mask = np.zeros((120, 160), np.uint8)
                cv2.polylines(mask, [np.array(points)], False, 255, 9)
                self.assertTrue(unbranched_black_path(mask))

    def test_t_x_y_and_competing_paths(self):
        examples = [
            [((80, 119), (80, 50)), ((10, 50), (150, 50))],
            [((20, 0), (140, 119)), ((140, 0), (20, 119))],
            [((80, 119), (80, 60)), ((80, 60), (20, 0)), ((80, 60), (140, 0))],
            [((30, 0), (30, 119)), ((120, 0), (120, 119))],
        ]
        for lines in examples:
            with self.subTest(lines=lines):
                mask = np.zeros((120, 160), np.uint8)
                for a, b in lines:
                    cv2.line(mask, a, b, 255, 9)
                self.assertFalse(unbranched_black_path(mask))

    def test_empty_and_fully_covered_image_are_not_a_path(self):
        self.assertFalse(unbranched_black_path(np.zeros((120, 160), np.uint8)))
        self.assertFalse(unbranched_black_path(np.full((120, 160), 255, np.uint8)))

    def test_control_requires_fresh_valid_heartbeat(self):
        with mock.patch("builtins.open", side_effect=FileNotFoundError):
            self.assertFalse(read_exit_control(10)["enabled"])
        for data, expected in [
                ({"enabled": True, "timestamp": 9.9, "runSequence": 7}, True),
                ({"enabled": True, "timestamp": 9, "runSequence": 7}, False),
                ({"enabled": True, "timestamp": 11, "runSequence": 7}, False),
                ({"enabled": False, "timestamp": 9.9, "runSequence": 7}, False),
                ({"enabled": True, "timestamp": 9.9, "runSequence": True}, False),
        ]:
            with mock.patch("builtins.open", mock.mock_open(read_data=json.dumps(data))):
                self.assertEqual(read_exit_control(10)["enabled"], expected)

    def test_acquisition_requires_fusion_target_on_same_continuous_tape(self):
        mask = np.zeros((240, 320), np.uint8)
        cv2.line(mask, (160, 0), (160, 239), 255, 12)
        fusion = {"farPoint": {"x": 160, "y": 60}}
        self.assertTrue(exit_line_is_unbranched(mask, fusion))
        fusion["farPoint"]["x"] = 250
        self.assertFalse(exit_line_is_unbranched(mask, fusion))


if __name__ == "__main__":
    unittest.main()
