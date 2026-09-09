import copy
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np


SCRIPTS_DIRECTORY = Path(__file__).resolve().parents[2] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIRECTORY))

from vision.camera_config import CAMERA_PROFILES
from vision.illumination_correction import (
    _cached_line_illumination_data,
    apply_line_illumination_correction,
    build_smoothed_line_illumination_reference,
    line_illumination_data,
)
from vision.line_masks import create_filtered_line_mask
from vision.stream_display import draw_line_illumination_overlay


def file_sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class LineIlluminationReferenceTest(unittest.TestCase):
    def tearDown(self):
        _cached_line_illumination_data.cache_clear()

    def profile_for_reference(self, path, sha256):
        return {
            "line_illumination_correction_enabled": True,
            "line_illumination_reference_path": str(path),
            "line_illumination_reference_sha256": sha256,
            "line_illumination_target_percentile": 85.0,
            "line_illumination_max_gain": 2.0,
            "line_illumination_useful_start_ratio": 0.0,
            "line_illumination_overlay_gain_threshold": 1.10,
            "line_illumination_overlay_margin_px": 0,
        }

    def test_temporal_median_rejects_one_frame_particle(self):
        frames = [np.full((20, 30), 100, np.uint8) for _ in range(5)]
        frames[0][10, 15] = 0
        reference = build_smoothed_line_illumination_reference(frames, 1.0)
        self.assertGreaterEqual(int(reference[10, 15]), 99)

    def test_correction_brightens_shadow_and_preserves_black_inside_it(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "reference.png"
            reference = np.full((6, 8), 100, np.uint8)
            reference[:, :3] = 50
            self.assertTrue(cv2.imwrite(str(path), reference))
            profile = self.profile_for_reference(path, file_sha256(path))
            gray = reference.copy()
            gray[3, 1] = 20
            original_gray = gray.copy()
            corrected, status = apply_line_illumination_correction(
                gray,
                profile,
                (6, 8, 3),
            )
            self.assertTrue(status["illuminationCorrectionActive"])
            self.assertEqual(int(corrected[1, 1]), 100)
            self.assertEqual(int(corrected[3, 1]), 40)
            self.assertEqual(int(corrected[3, 6]), 100)
            np.testing.assert_array_equal(gray, original_gray)

    def test_invalid_hash_falls_back_without_changing_gray(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "reference.png"
            self.assertTrue(cv2.imwrite(str(path), np.full((6, 8), 100, np.uint8)))
            profile = self.profile_for_reference(path, "0" * 64)
            gray = np.full((6, 8), 70, np.uint8)
            corrected, status = apply_line_illumination_correction(
                gray,
                profile,
                (6, 8, 3),
            )
            self.assertFalse(status["illuminationCorrectionActive"])
            self.assertIn("hash", status["illuminationCorrectionError"])
            np.testing.assert_array_equal(corrected, gray)

    def test_absolute_dark_floor_still_preserves_border_tape(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "reference.png"
            reference = np.full((30, 40), 100, np.uint8)
            reference[:, :15] = 50
            self.assertTrue(cv2.imwrite(str(path), reference))
            profile = {
                **self.profile_for_reference(path, file_sha256(path)),
                "line_roi_start_ratio": 0.0,
                "line_background_kernel_size": 9,
                "line_max_background_ratio_percent": 61,
                "line_min_threshold": 40,
                "line_max_brightness": 190,
                "open_kernel_shape": "ellipse",
                "open_kernel_size": 1,
                "close_kernel_size": 1,
            }
            frame = cv2.cvtColor(reference, cv2.COLOR_GRAY2BGR)
            frame[15:, :15] = 32
            filtered, _roi, status = create_filtered_line_mask(
                frame,
                profile,
                return_repair_status=True,
            )
            self.assertTrue(np.all(filtered[20:, :10] == 255))
            self.assertGreater(status["illuminationDarkPixelsPreserved"], 0)

    def test_wrong_resolution_falls_back_without_resizing_map(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "reference.png"
            self.assertTrue(cv2.imwrite(str(path), np.full((6, 8), 100, np.uint8)))
            profile = self.profile_for_reference(path, file_sha256(path))
            gray = np.full((7, 8), 70, np.uint8)
            corrected, status = apply_line_illumination_correction(
                gray,
                profile,
                (7, 8, 3),
            )
            self.assertFalse(status["illuminationCorrectionActive"])
            self.assertIn("forma", status["illuminationCorrectionError"])
            np.testing.assert_array_equal(corrected, gray)

    def test_versioned_map_covers_both_lower_sides(self):
        profile = copy.deepcopy(CAMERA_PROFILES["down"]["vision"])
        gain, zone, status = line_illumination_data(profile, (360, 480, 3))
        self.assertTrue(status["illuminationCorrectionActive"])
        self.assertEqual(gain.shape, (360, 480))
        self.assertEqual(zone.shape, (360, 480))
        self.assertGreater(np.count_nonzero(zone[260:, :120]), 0)
        self.assertGreater(np.count_nonzero(zone[260:, 360:]), 0)
        self.assertLessEqual(float(gain.max()), 2.0)

    def test_overlay_marks_zone_without_mutating_candidate_mask(self):
        display = np.full((60, 100, 3), 100, np.uint8)
        candidates = np.zeros((60, 100), np.uint8)
        candidates[28:32, 12:20] = 255
        original_candidates = candidates.copy()
        zone = np.zeros((60, 100), np.uint8)
        zone[20:40, :] = 255
        status = {
            "illuminationCorrectionConfigured": True,
            "illuminationCorrectionActive": True,
            "illuminationMaximumGain": 2.0,
            "illuminationCorrectionMs": 0.4,
        }
        draw_line_illumination_overlay(
            display,
            candidates,
            zone,
            status,
            "real",
        )
        np.testing.assert_array_equal(candidates, original_candidates)
        self.assertGreater(int(display[25, 50, 0]), int(display[25, 50, 2]))
        self.assertGreater(int(display[29, 15, 1]), int(display[29, 15, 2]))

    def test_line_diagnostic_marks_removed_candidate_in_red(self):
        display = np.zeros((60, 100, 3), np.uint8)
        candidates = np.zeros((60, 100), np.uint8)
        candidates[28:32, 12:20] = 255
        display[candidates > 0] = (255, 255, 255)
        old_candidates = candidates.copy()
        old_candidates[35:39, 70:78] = 255
        zone = np.full((60, 100), 255, np.uint8)
        status = {
            "illuminationCorrectionConfigured": True,
            "illuminationCorrectionActive": True,
            "illuminationMaximumGain": 2.0,
            "illuminationCorrectionMs": 0.4,
        }
        draw_line_illumination_overlay(
            display,
            candidates,
            zone,
            status,
            "line",
            old_candidates,
        )
        self.assertEqual(tuple(display[36, 73]), (0, 0, 255))
        self.assertEqual(tuple(display[28, 12]), (0, 255, 0))
        self.assertEqual(tuple(display[29, 15]), (255, 255, 255))

    def test_overlay_aligns_roi_mask_with_bottom_of_full_frame(self):
        display = np.zeros((60, 100, 3), np.uint8)
        candidates = np.zeros((40, 100), np.uint8)
        candidates[8:12, 12:20] = 255
        zone = np.full((60, 100), 255, np.uint8)
        status = {
            "illuminationCorrectionConfigured": True,
            "illuminationCorrectionActive": True,
        }
        draw_line_illumination_overlay(
            display,
            candidates,
            zone,
            status,
            "real",
        )
        self.assertEqual(tuple(display[5, 50]), (0, 0, 0))
        self.assertGreater(int(display[25, 50, 0]), 0)
        self.assertGreater(int(display[29, 15, 1]), int(display[29, 15, 2]))


if __name__ == "__main__":
    unittest.main()
