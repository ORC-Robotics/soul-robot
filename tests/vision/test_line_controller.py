"""Testes da fachada que possui os estados temporais do seguidor."""

import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np


SCRIPTS_DIRECTORY = Path(__file__).resolve().parents[2] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIRECTORY))

import vision.line_control as line_control


class LineFollowerControllerTests(unittest.TestCase):
    def test_search_lifecycle_is_owned_by_controller(self):
        controller = line_control.LineFollowerController()

        controller.start_search("LEFT")

        self.assertTrue(controller.line_search_tracker.active)
        self.assertEqual(controller.line_search_tracker.initial_direction, "LEFT")

        controller.stop_search()

        self.assertFalse(controller.line_search_tracker.active)
        self.assertIsNone(controller.line_search_tracker.initial_direction)

    def test_calculate_reuses_controller_trackers(self):
        controller = line_control.LineFollowerController()
        mask = np.zeros((10, 10), dtype=np.uint8)
        expected_command = {"left_power": 0.0, "right_power": 0.0}

        with patch.object(
            line_control,
            "calculate_line_follower_command",
            return_value=expected_command,
        ) as calculate:
            result = controller.calculate(mask, {})

        self.assertIs(result, expected_command)
        arguments = calculate.call_args.kwargs
        self.assertIs(
            arguments["virtual_turn_tracker"],
            controller.virtual_turn_tracker,
        )
        self.assertIs(
            arguments["pivot_state_tracker"],
            controller.pivot_state_tracker,
        )
        self.assertIs(
            arguments["medium_spin_tracker"],
            controller.medium_spin_tracker,
        )
        self.assertIs(
            arguments["line_search_tracker"],
            controller.line_search_tracker,
        )


if __name__ == "__main__":
    unittest.main()
