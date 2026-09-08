"""Valida coleta e publicação modular sem câmeras ou motores conectados."""

import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import silver_dataset_recorder as dataset
import capture_silver_dataset as capture
import forward_camera_stream as forward
from vision import status_publisher
from vision.camera_config import CAMERA_PROFILES


class SilverDatasetTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        for name, path in {
            "CONTROL_PATH": self.root / "control.json",
            "STATUS_PATH": self.root / "status.json",
            "DATASET_ROOT": self.root / "raw",
        }.items():
            patcher = mock.patch.object(dataset, name, path)
            patcher.start()
            self.addCleanup(patcher.stop)

    def recorder(self, camera="down"):
        # Executa as operações das threads explicitamente para testar sem concorrência externa.
        with mock.patch("threading.Thread.start"):
            recorder = dataset.SilverDatasetRecorder(camera)
        recorder.temp_status_path = self.root / f"{camera}.tmp.json"
        return recorder

    def test_existing_session_waits_without_drops_and_keeps_label_counts(self):
        folder = dataset.DATASET_ROOT / "down" / "session_001" / "silver"
        folder.mkdir(parents=True)
        (folder / "existing.jpg").touch()
        control = {"camera": "down", "session": "session_001", "label": "silver", "fps": 4, "active": True}
        dataset.CONTROL_PATH.write_text(json.dumps(control))
        recorder = self.recorder()
        frame = np.zeros((8, 8, 3), dtype=np.uint8)
        self.assertFalse(recorder.submit(frame))
        self.assertEqual(recorder.dropped_count, 0)
        recorder._write_status(reload_counts=True)
        self.assertTrue(recorder.submit(frame))
        recorder._save_frame(*recorder.frame_queue.get_nowait())
        self.assertEqual(recorder.counts["silver"], 2)
        recorder.frame_queue.task_done()
        control["label"] = "other"
        dataset.CONTROL_PATH.write_text(json.dumps(control))
        recorder.control_mtime_ns = None
        with mock.patch.object(recorder, "_load_counts", side_effect=AssertionError("recontagem inesperada")):
            self.assertTrue(recorder.submit(frame))
            recorder._save_frame(*recorder.frame_queue.get_nowait())
        recorder.frame_queue.task_done()
        self.assertEqual(recorder.counts, {"black": 0, "other": 1, "silver": 2})

    def test_bad_status_and_stale_are_safe(self):
        for payload in ("null", "[]", "{inválido"):
            dataset.STATUS_PATH.write_text(payload)
            self.assertEqual(dataset.read_dataset_status(), dataset.empty_dataset_status())
        data = dataset.empty_dataset_status()
        data.update(datasetCaptureActive=True, datasetCaptureUpdatedAt=10)
        dataset.STATUS_PATH.write_text(json.dumps(data))
        with mock.patch.object(dataset.time, "time", return_value=13):
            self.assertTrue(dataset.read_dataset_status()["datasetCaptureActive"])
        with mock.patch.object(dataset.time, "time", return_value=15):
            self.assertTrue(dataset.read_dataset_status()["datasetCaptureStale"])

    def test_both_camera_publishers_include_dataset(self):
        data = dataset.empty_dataset_status()
        data.update(datasetCaptureCamera="down", datasetCaptureSession="session_001")
        dataset.STATUS_PATH.write_text(json.dumps(data))
        publisher = status_publisher.LineStatusPublisher(
            status_path=str(self.root / "down.json"),
            temp_status_path=str(self.root / "down.tmp.json"),
        )
        publisher.save_status(0.0, CAMERA_PROFILES["down"], {}, active=False)
        with mock.patch.object(forward, "STATUS_PATH", str(self.root / "forward.json")), \
             mock.patch.object(forward, "TEMP_STATUS_PATH", str(self.root / "forward.tmp.json")):
            forward.save_status(False, False, "disabled")
        for camera in ("down", "forward"):
            status = json.loads((self.root / f"{camera}.json").read_text())
            self.assertEqual(status["datasetCaptureSession"], "session_001")
            self.assertFalse(status["datasetCaptureActive"])

    def test_controller_rejects_path_traversal(self):
        with self.assertRaises(ValueError):
            capture.prepare_session("down", "../../outside")


if __name__ == "__main__":
    unittest.main()
