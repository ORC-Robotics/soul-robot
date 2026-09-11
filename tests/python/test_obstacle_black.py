"""Valida o detector obstacleBlack sem câmera ou motores."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from vision.obstacle_black import classify_obstacle_black


class ObstacleBlackTest(unittest.TestCase):
    def test_real_observations_follow_the_visibility_gate(self):
        cases = (
            (0.3293, 20984, (True, "VALID_BLACK")),
            (1.0000, 63722, (False, "SATURATED_BLACK")),
            (0.0000, 0, (False, "LOW_BLACK")),
            (0.0240, 1530, (False, "LOW_BLACK")),
        )
        for ratio, largest, expected in cases:
            with self.subTest(ratio=ratio, largest=largest):
                self.assertEqual(
                    classify_obstacle_black(ratio, largest), expected
                )


if __name__ == "__main__":
    unittest.main()
