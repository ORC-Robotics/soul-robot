"""Verifica contratos de captura e métricas que poderiam favorecer máscaras erradas."""

import json
import copy
import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from line_calibration import analyze, frame_metrics
from compare_line_calibration import contrast_auc, group_configurations
from vision import calibration_capture as capture
from vision.camera_config import CAMERA_PROFILES
from vision.line_masks import create_line_candidate_mask, create_line_binary_mask


class FakeCamera:
    """Simula controles sem abrir sensor, GPIO ou conexão com o robô."""

    camera_controls = {"AeEnable": (False, True, True), "AwbEnable": (False, True, True),
                       "ExposureTime": (100, 100000, 10000), "AnalogueGain": (1, 16, 1),
                       "ColourGains": (0, 32, 1)}

    def __init__(self):
        self.applied = []

    def set_controls(self, controls):
        self.applied.append(dict(controls))

    def camera_configuration(self):
        return {}


class FakeGuard:
    """Representa uma telemetria estacionária validada apenas durante os testes."""

    telemetry = {"emergency": True}

    def require_stopped(self):
        pass

    def close(self):
        pass


class CalibrationTests(unittest.TestCase):
    def test_minimum_threshold_preserves_dark_corner_without_whitening_floor(self):
        gray = np.full((100, 100), 200, np.uint8)
        gray[70:, :50] = 20
        profile = {"line_background_kernel_size": 31,
                   "line_max_background_ratio_percent": 61, "line_max_brightness": 190}
        original = create_line_binary_mask(gray, profile)
        corrected = create_line_binary_mask(gray, {**profile, "line_min_threshold": 40})
        self.assertEqual(original[95, 5], 0)
        self.assertEqual(corrected[95, 5], 255)
        self.assertTrue(np.all(corrected[gray == 200] == 0))

    def test_green_exclusion_preserves_original_association_mask_and_tape(self):
        structural = np.zeros((360, 480), np.uint8)
        structural[100:180, 27:466] = 255
        structural[180:, 160:230] = 255
        tape = structural > 0
        structural[180:260, 230:320] = 255
        original = structural.copy()
        green = np.zeros((300, 480), np.uint8)
        green[180:260, 230:320] = 255
        mask = create_line_candidate_mask(structural, CAMERA_PROFILES["down"]["vision"], green_mask=green)
        self.assertTrue(np.array_equal(structural, original))
        self.assertTrue(np.all(mask[tape] == 255))
        self.assertFalse(np.any(mask[:300][green > 0]))

    def test_analysis_rejects_modified_rgb_and_incompatible_baseline_mask(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dataset = root / "dataset"
            for name in ("raw", "mask", "metadata"):
                (dataset / name).mkdir(parents=True)
            raw = dataset / "raw/0000.png"
            cv2.imwrite(str(raw), np.full((360, 480, 3), 220, np.uint8))
            cv2.imwrite(str(dataset / "mask/0000.png"), np.full((360, 480), 255, np.uint8))
            (dataset / "config.json").write_text(json.dumps({"camera_profile": CAMERA_PROFILES["down"]}))
            annotation = root / "annotation.json"
            annotation.write_text(json.dumps({"shape": [360, 480], "line_polygons": [],
                                  "evaluation_polygons": [[[0, 0], [479, 0], [479, 359], [0, 359]]],
                                  "boundary_uncertainty_px": 0}))
            metadata = dataset / "metadata/0000.json"
            metadata.write_text(json.dumps({"raw_sha256": "modified"}))
            with self.assertRaisesRegex(ValueError, "modificado"):
                analyze(dataset, annotation, root / "bad_rgb", {})
            metadata.write_text(json.dumps({"raw_sha256": hashlib.sha256(raw.read_bytes()).hexdigest()}))
            with self.assertRaisesRegex(ValueError, "não reproduz"):
                analyze(dataset, annotation, root / "bad_mask", {})

    def test_worst_scenario_is_not_hidden_by_more_easy_captures(self):
        row = {"requested_camera_controls": {"AeEnable": True},
               "segmentation": {"ratio": 55}, "experiment": "candidate", "frames": 90}
        rows = [{**row, "dataset": "straight_1", "scenario": "straight", "score": 99},
                {**row, "dataset": "straight_2", "scenario": "straight", "score": 100},
                {**row, "dataset": "gap_1", "scenario": "gap", "score": 20}]
        groups = group_configurations(rows)
        self.assertEqual(len(groups), 1)
        self.assertEqual(groups[0]["scenario_count"], 2)
        self.assertEqual(groups[0]["worst_scenario_score"], 20)

    def test_contrast_does_not_reward_inverted_or_equal_intensities(self):
        positive = np.array([[True, False]])
        negative = ~positive
        self.assertEqual(contrast_auc(np.array([[20, 200]], np.uint8), positive, negative), 1)
        self.assertEqual(contrast_auc(np.array([[200, 20]], np.uint8), positive, negative), 0)
        self.assertEqual(contrast_auc(np.array([[20, 20]], np.uint8), positive, negative), 0.5)

    def test_rejects_unsafe_or_incomplete_requests_before_camera_changes(self):
        camera = FakeCamera()
        cases = [{"experiment": "../escape"}, {"experiment": "test", "frames": True},
                 {"experiment": "test", "frames": 101},
                 {"experiment": "test", "controls": {"AeEnable": False}},
                 {"experiment": "test", "controls": {"AwbEnable": False}},
                 {"experiment": "test", "controls": {"Brightness": 1}},
                 {"experiment": "test", "segmentation": {"line_max_background_ratio_percent": True}},
                 {"experiment": "test", "segmentation": {"line_max_background_ratio_percent": None}},
                 {"experiment": "test", "segmentation": {"line_max_background_ratio_percent": 101}},
                 {"experiment": "test", "segmentation": {"line_exclude_green": 1}},
                 {"experiment": "test", "segmentation": {"line_illumination_correction_enabled": 1}},
                 {"experiment": "test", "segmentation": {"line_min_threshold": True}},
                 {"experiment": "test", "segmentation": {"line_min_threshold": 191}},
                 {"experiment": "test", "segmentation": {"open_kernel_size": 9}},
                 {"experiment": "test", "segmentation": {"line_max_background_ratio_percent": 55}, "controls": {"AeEnable": True}},
                 {"experiment": "test", "controls": {"AeEnable": False, "ExposureTime": 40000, "AnalogueGain": 1}},
                 {"experiment": "test", "controls": {"AwbEnable": False, "ColourGains": [float("nan"), 1]}}]
        for request in cases:
            with self.subTest(request=request), self.assertRaises(ValueError):
                capture.validate_request(request, camera)
        self.assertEqual(camera.applied, [])

    def test_live_pairs_preserve_pixels_metadata_and_restore_controls(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            request = root / "request.json"
            request.write_text(json.dumps({"experiment": "paired", "frames": 30,
                               "controls": {"AeEnable": False, "ExposureTime": 1000, "AnalogueGain": 1}}))
            camera = FakeCamera()
            with patch.object(capture, "REQUEST_PATH", request), patch.object(capture, "RESULT_PATH", root / "result.json"), patch.object(capture, "CAPTURE_ROOT", root / "data"), patch.object(capture, "StationaryGuard", FakeGuard):
                recorder = capture.CalibrationCapture(camera, CAMERA_PROFILES["down"], {})
                self.assertTrue(recorder.before_frame())
                for i in range(60):
                    frame = np.full((4, 6, 3), i, np.uint8)
                    mask = np.full((4, 6), (i % 2) * 255, np.uint8)
                    recorder.after_frame(frame, mask, mask, {"SensorTimestamp": i}, i, {})
                recorder.close()
                result = json.loads((root / "result.json").read_text())
                self.assertEqual(result["state"], "complete")
                persisted_result = json.loads((root / "data/paired/result.json").read_text())
                self.assertEqual(persisted_result["frames"], 30)
                saved = cv2.imread(str(root / "data/paired/raw/0000.png"))
                self.assertTrue(np.all(saved == 30))
                metadata = json.loads((root / "data/paired/metadata/0000.json").read_text())
                self.assertEqual(metadata["camera_metadata"]["SensorTimestamp"], 30)
                self.assertEqual(camera.applied[-1], capture.original_controls())

    def test_temporary_segmentation_uses_shared_profile_and_restores_on_abort(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            request = root / "request.json"
            request.write_text(json.dumps({"experiment": "ratio55", "frames": 30,
                               "segmentation": {"line_max_background_ratio_percent": 55}}))
            profile = copy.deepcopy(CAMERA_PROFILES["down"])
            original = copy.deepcopy(profile)
            vision_reference = profile["vision"]
            camera = FakeCamera()
            with patch.object(capture, "REQUEST_PATH", request), patch.object(capture, "RESULT_PATH", root / "result.json"), patch.object(capture, "CAPTURE_ROOT", root / "data"), patch.object(capture, "StationaryGuard", FakeGuard):
                recorder = capture.CalibrationCapture(camera, profile, {})
                self.assertTrue(recorder.before_frame())
                self.assertEqual(vision_reference["line_max_background_ratio_percent"], 55)
                self.assertEqual(recorder.capture_config["camera_profile"]["vision"]["line_max_background_ratio_percent"], 55)
                recorder.abort(RuntimeError("test"))
                self.assertEqual(profile, original)
                self.assertIs(vision_reference, profile["vision"])
                self.assertEqual(camera.applied, [])

    def test_abort_restores_controls_when_telemetry_stops_being_safe(self):
        camera = FakeCamera()
        recorder = capture.CalibrationCapture(camera, {}, {})
        recorder.changed_controls, recorder.active = True, True
        recorder.guard = FakeGuard()
        with patch.object(recorder.guard, "require_stopped", side_effect=RuntimeError("moving")), patch.object(capture, "publish_result") as result:
            self.assertFalse(recorder.before_frame())
            self.assertEqual(camera.applied[-1], capture.original_controls())
            self.assertEqual(result.call_args.args[0]["state"], "error")

    def test_empty_mask_cannot_score_as_preserved_line(self):
        expected = np.zeros((360, 480), np.uint8)
        expected[60:, 200:280] = 255
        positive, region = expected > 0, np.ones_like(expected, bool)
        negative = ~positive
        frame = np.full((360, 480, 3), 220, np.uint8)
        frame[positive] = 20
        empty, _ = frame_metrics(frame, np.zeros_like(expected), expected, positive, negative, region, None)
        self.assertEqual(empty["line_recall"], 0)
        self.assertEqual(empty["line_continuity"], 0)
        self.assertFalse(empty["trajectory_target_on_line"])

    def test_non_floor_contrast_exclusion_keeps_false_positive_penalty(self):
        expected = np.zeros((360, 480), np.uint8)
        expected[60:, 200:280] = 255
        positive = expected > 0
        region = np.ones_like(positive)
        negative = ~positive
        marker = np.zeros_like(positive)
        marker[100:250, 300:420] = True
        frame = np.full((360, 480, 3), 220, np.uint8)
        frame[positive] = 20
        frame[marker] = 10
        mask = expected.copy()
        mask[marker] = 255
        metrics, _ = frame_metrics(frame, mask, expected, positive, negative,
                                   region, None, negative & ~marker)
        self.assertEqual(metrics["false_positive_area_px"], int(marker.sum()))
        self.assertAlmostEqual(metrics["contrast_margin_8bit_normalized"], 200 / 255)

    def test_connected_false_pixels_are_counted_as_background_errors(self):
        expected = np.zeros((360, 480), np.uint8)
        expected[60:, 200:280] = 255
        positive, region = expected > 0, np.ones_like(expected, bool)
        mask = expected.copy()
        mask[300:, 280:330] = 255
        frame = np.full((360, 480, 3), 220, np.uint8)
        frame[positive] = 20
        metrics, _ = frame_metrics(frame, mask, expected, positive, ~positive, region, None)
        self.assertEqual(metrics["false_component_count"], 0)
        self.assertEqual(metrics["false_positive_area_px"], 3000)


if __name__ == "__main__":
    unittest.main()
