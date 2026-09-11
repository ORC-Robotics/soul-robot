"""Valida a gravação diagnóstica sem câmera nem motores conectados."""

import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import forward_reacquisition_recorder as recorder_module


class ForwardReacquisitionRecorderTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.control_path = self.root / "control.json"
        self.session_root = self.root / "sessions"

    def recorder_without_thread(self):
        with mock.patch("threading.Thread.start"):
            return recorder_module.ForwardReacquisitionRecorder(
                self.control_path,
                self.session_root,
            )

    def start(self, label="obstacle_exit_left"):
        with mock.patch.object(
            recorder_module, "_git_commit", return_value="abc123"
        ):
            return recorder_module.start_session(
                label,
                control_path=self.control_path,
                session_root=self.session_root,
            )

    def test_disabled_recorder_creates_no_data(self):
        recorder = self.recorder_without_thread()
        frame = np.zeros((8, 8, 3), dtype=np.uint8)
        self.assertFalse(
            recorder.submit(
                frame, None, frame, {}, None, (0, 0, 8, 8), 1.0, 1
            )
        )
        self.assertFalse(self.session_root.exists())

    def test_start_creates_valid_session(self):
        session_dir = self.start()
        session = json.loads((session_dir / "session.json").read_text())
        control = json.loads(self.control_path.read_text())
        self.assertEqual(session["label"], "obstacle_exit_left")
        self.assertEqual(session["status"], "recording")
        self.assertEqual(session["gitCommit"], "abc123")
        self.assertIsNone(session["endedAt"])
        self.assertTrue(control["active"])
        for name in ("frames", "masks", "overlays"):
            self.assertTrue((session_dir / name).is_dir())

    def test_stop_closes_session(self):
        session_dir = self.start("rescue_exit")
        stopped_dir = recorder_module.stop_session(
            self.control_path, self.session_root
        )
        session = json.loads((session_dir / "session.json").read_text())
        control = json.loads(self.control_path.read_text())
        self.assertEqual(stopped_dir, session_dir)
        self.assertEqual(session["status"], "stopped")
        self.assertIsNotNone(session["endedAt"])
        self.assertFalse(control["active"])

    def test_strict_json_preserves_candidates_and_selected_bands(self):
        session_dir = self.start()
        recorder = self.recorder_without_thread()
        frame = np.zeros((8, 8, 3), dtype=np.uint8)
        mask = np.zeros((8, 8), dtype=np.uint8)
        selected_bands = [
            {"x": np.float64(3.5), "y": np.int64(7), "width": 2}
        ]
        candidates = [
            {
                "box": [1, 2, 3, 4],
                "state": "PRESENT",
                "score": float("nan"),
                "bands": selected_bands,
                "extent": float("inf"),
            }
        ]
        reading = {
            "forwardPathState": "PRESENT",
            "forwardLinePresent": True,
            "forwardLineVisible": True,
            "forwardLinePosition": 0.25,
            "forwardLineConfidence": 0.9,
            "forwardPathConfidence": 0.9,
            "forwardPathComponents": {"score": float("-inf")},
            "candidates": candidates,
            "selectedBands": selected_bands,
            "prediction": [(3, 7)],
        }
        self.assertTrue(
            recorder.submit(
                frame,
                mask,
                frame,
                reading,
                None,
                (0, 4, 8, 8),
                123.0,
                17,
                now=1.0,
            )
        )
        sample = recorder.queue.get_nowait()
        recorder._write_sample(sample)
        recorder.queue.task_done()
        line = (session_dir / "vision.jsonl").read_text().strip()
        self.assertNotIn("NaN", line)
        self.assertNotIn("Infinity", line)
        saved = json.loads(line)
        self.assertEqual(saved["candidates"][0]["bands"], selected_bands)
        self.assertEqual(saved["selectedBands"], selected_bands)
        self.assertIsNone(saved["candidates"][0]["score"])
        self.assertIsNone(saved["candidates"][0]["extent"])
        self.assertIsNone(saved["forwardPathComponents"]["score"])

    def test_enqueue_failure_never_escapes_to_camera_control(self):
        self.start()
        recorder = self.recorder_without_thread()
        recorder.queue.put_nowait = mock.Mock(side_effect=OSError("disk"))
        frame = np.zeros((8, 8, 3), dtype=np.uint8)
        self.assertFalse(
            recorder.submit(
                frame, None, frame, {}, None, (0, 0, 8, 8), 1.0, 1
            )
        )

    def test_overlay_explains_rejected_candidate_without_changing_reading(self):
        frame = np.zeros((120, 200, 3), dtype=np.uint8)
        candidate = {
            "box": [20, 30, 25, 60],
            "state": "UNCERTAIN",
            "reason": "INSUFFICIENT_SUPPORT",
            "score": 0.8,
            "bands": [{"x": 30, "y": 50, "width": 8}],
            "extent": 10,
            "elongation": 1.0,
            "clipped": False,
        }
        reading = {
            "forwardPathState": "UNCERTAIN",
            "candidates": [candidate],
            "roi": [0, 20, 200, 120],
        }
        original = json.loads(json.dumps(reading))
        mask = np.zeros((120, 200), dtype=np.uint8)
        mask[30:90, 20:45] = 255

        with mock.patch.object(recorder_module.cv2, "putText") as put_text:
            recorder_module.draw_forward_path_diagnostic_overlay(
                frame, reading, (0, 20, 200, 120), mask
            )

        overlay_texts = [call.args[1] for call in put_text.call_args_list]
        rejection_text = next(text for text in overlay_texts if text.startswith("REJ1"))
        self.assertTrue(any(text.startswith("MASK EFETIVA") for text in overlay_texts))
        self.assertIn("BANDS", rejection_text)
        self.assertIn("EXT", rejection_text)
        self.assertIn("ELONG", rejection_text)
        self.assertIn("NEAR", rejection_text)
        self.assertEqual(reading, original)

    def test_recording_header_does_not_duplicate_candidate_diagnostics(self):
        frame = np.zeros((120, 200, 3), dtype=np.uint8)
        reading = {
            "forwardPathState": "ABSENT",
            "candidates": [{
                "box": [20, 30, 25, 60],
                "state": "ABSENT",
                "reason": "SHORT_OR_THIN",
                "bands": [],
            }],
        }

        with mock.patch.object(recorder_module.cv2, "putText") as put_text:
            recorder_module.recording_overlay(
                frame, "rescue_exit", 43, reading, (0, 20, 200, 120)
            )

        overlay_texts = [call.args[1] for call in put_text.call_args_list]
        self.assertFalse(any(text.startswith("REJ") for text in overlay_texts))
        self.assertFalse(any(text.startswith("CANDIDATES") for text in overlay_texts))


if __name__ == "__main__":
    unittest.main()
