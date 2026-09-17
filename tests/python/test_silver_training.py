"""Valida a divisão do treinamento do classificador de faixa prata."""

from pathlib import Path
import sys
import tempfile
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "training"))
import training_silver_classifier as training


class SilverTrainingSplitTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.camera_dir = Path(self.directory.name) / "down"

    def create_session(self, name, image_count=100):
        session = self.camera_dir / name
        for class_name in training.CLASS_NAMES:
            class_dir = session / class_name
            class_dir.mkdir(parents=True)
            for index in range(image_count):
                (class_dir / f"{index:04d}.jpg").touch()
        return session

    def test_discovers_two_sessions(self):
        expected = [
            self.create_session("light_off"),
            self.create_session("light_on"),
        ]

        self.assertEqual(training.discover_sessions(self.camera_dir), expected)

    def test_rejects_only_one_session(self):
        self.create_session("light_on")

        with self.assertRaisesRegex(RuntimeError, "pelo menos 2 sessões"):
            training.discover_sessions(self.camera_dir)

    def test_two_sessions_keep_each_block_in_only_one_split(self):
        sessions = [
            self.create_session("light_off"),
            self.create_session("light_on"),
        ]
        train, validation, test = training.split_two_session_samples(
            sessions,
            seed=42,
        )
        split_by_path = {}
        for split_name, samples in (
            ("train", train),
            ("validation", validation),
            ("test", test),
        ):
            for path, _ in samples:
                self.assertNotIn(path, split_by_path)
                split_by_path[path] = split_name

        self.assertEqual(len(split_by_path), 600)

        for session in sessions:
            for class_name in training.CLASS_NAMES:
                for block_start in range(0, 100, training.SAMPLE_BLOCK_SIZE):
                    block_paths = [
                        str(session / class_name / f"{index:04d}.jpg")
                        for index in range(
                            block_start,
                            min(block_start + training.SAMPLE_BLOCK_SIZE, 100),
                        )
                    ]
                    assigned_splits = {
                        split_by_path[path]
                        for path in block_paths
                    }
                    self.assertEqual(len(assigned_splits), 1)

        for samples in (train, validation, test):
            self.assertEqual(
                training.count_classes(samples).keys(),
                set(training.CLASS_NAMES),
            )
            for session in sessions:
                self.assertTrue(
                    any(str(session) in path for path, _ in samples)
                )

    def test_requires_three_blocks_per_class_and_session(self):
        sessions = [
            self.create_session("light_off", image_count=40),
            self.create_session("light_on", image_count=40),
        ]

        with self.assertRaisesRegex(RuntimeError, "pelo menos 41 imagens"):
            training.split_two_session_samples(sessions, seed=42)


if __name__ == "__main__":
    unittest.main()
