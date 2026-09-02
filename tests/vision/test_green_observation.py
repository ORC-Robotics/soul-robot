import sys
import unittest
from pathlib import Path


SCRIPTS_DIRECTORY = Path(__file__).resolve().parents[2] / "scripts"
if str(SCRIPTS_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIRECTORY))

from vision.green_observation import (  # noqa: E402
    GreenObservationConfig,
    GreenObservationTracker,
)


class GreenObservationConfigTest(unittest.TestCase):
    def test_rejects_non_positive_frame_counts(self):
        invalid_configs = (
            {"confirmation_frames": 0},
            {"single_observation_frames": 0},
            {"clear_hysteresis_frames": 0},
        )
        for values in invalid_configs:
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    GreenObservationConfig(**values)

    def test_rejects_negative_retention(self):
        with self.assertRaises(ValueError):
            GreenObservationConfig(direction_retention_seconds=-0.1)


class GreenObservationTrackerTest(unittest.TestCase):
    def test_duplicate_sequence_does_not_advance_confirmation(self):
        tracker = GreenObservationTracker()

        first = tracker.update(1, "ESQUERDA", observed_at=1.0)
        duplicate = tracker.update(1, "ESQUERDA", observed_at=1.1)
        confirmed = tracker.update(2, "ESQUERDA", observed_at=1.2)

        self.assertEqual(first, ("SEM_DECISAO", False, 1))
        self.assertEqual(duplicate, first)
        self.assertEqual(confirmed, ("ESQUERDA", True, 2))

    def test_lateral_direction_is_retained_only_inside_time_window(self):
        tracker = GreenObservationTracker()
        tracker.update(1, "DIREITA", observed_at=1.0)
        tracker.update(2, "DIREITA", observed_at=1.1)

        retained = tracker.update(3, "SEM_DECISAO", observed_at=1.39)
        expired = tracker.update(4, "SEM_DECISAO", observed_at=1.41)

        self.assertEqual(retained, ("DIREITA", True, 2))
        self.assertEqual(expired, ("SEM_DECISAO", False, 0))

    def test_non_lateral_decision_uses_clear_hysteresis(self):
        tracker = GreenObservationTracker()
        tracker.update(1, "RETORNO_180", observed_at=1.0)
        tracker.update(2, "RETORNO_180", observed_at=1.1)

        first_missing = tracker.update(3, "SEM_DECISAO", observed_at=1.2)
        second_missing = tracker.update(4, "SEM_DECISAO", observed_at=1.3)

        self.assertEqual(first_missing, ("RETORNO_180", True, 2))
        self.assertEqual(second_missing, ("SEM_DECISAO", False, 0))

    def test_custom_confirmation_count_is_respected(self):
        config = GreenObservationConfig(
            confirmation_frames=3,
            single_observation_frames=3,
        )
        tracker = GreenObservationTracker(config)

        tracker.update(1, "VERDE_FALSO", observed_at=1.0)
        second = tracker.update(2, "VERDE_FALSO", observed_at=1.1)
        third = tracker.update(3, "VERDE_FALSO", observed_at=1.2)

        self.assertEqual(second, ("SEM_DECISAO", False, 2))
        self.assertEqual(third, ("VERDE_FALSO", True, 3))


if __name__ == "__main__":
    unittest.main()
