"""Testes do fluxo que prepara sessões anotadas para o treinamento."""

import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import prepare_dataset


class PrepareDatasetTest(unittest.TestCase):
    def create_session(self, root, name, label_content="0 0.5 0.5 0.2 0.2\n"):
        images = root / name / "images"
        labels = root / name / "labels"
        images.mkdir(parents=True)
        labels.mkdir(parents=True)
        (images / "frame.jpg").write_bytes(b"jpeg-for-test")
        (labels / "frame.txt").write_text(label_content, encoding="utf-8")

    def test_prepares_whole_sessions_in_different_splits(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source = root / "annotated"
            self.create_session(source, "session_a")
            self.create_session(source, "session_b", "")

            with (
                patch.object(prepare_dataset, "DATASET_DIR", root / "datasets"),
                patch.object(prepare_dataset, "MANIFEST_PATH", root / "manifest.json"),
            ):
                manifest, copied_files = prepare_dataset.prepare_dataset(
                    source, validation_ratio=0.20, seed=2026
                )

            splits = {
                status["split"] for status in manifest["sessions"].values()
            }
            self.assertEqual(splits, {"train", "valid"})
            self.assertEqual(copied_files, 4)
            self.assertEqual(
                sum(status["negatives"] for status in manifest["sessions"].values()),
                1,
            )

    def test_rejects_image_without_label(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            source = Path(temporary_directory) / "annotated"
            self.create_session(source, "session_a")
            (source / "session_a" / "labels" / "frame.txt").unlink()

            with self.assertRaisesRegex(ValueError, "Label ausente"):
                prepare_dataset.load_sessions(source)

    def test_preserves_existing_assignment(self):
        sessions = {"session_a": {}, "session_b": {}, "session_c": {}}
        assignments = prepare_dataset.assign_splits(
            sessions,
            validation_ratio=0.20,
            seed=2026,
            existing_assignments={"session_a": "valid", "session_b": "train"},
        )
        self.assertEqual(assignments["session_a"], "valid")
        self.assertEqual(assignments["session_b"], "train")


if __name__ == "__main__":
    unittest.main()
