import json
import sys
import unittest
from pathlib import Path
from unittest import mock

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from vision.rescue_exit import analyze_exit_candidates, read_exit_control, unbranched_black_path, exit_line_is_unbranched


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
