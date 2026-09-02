"""Testes do estado persistente de GREEN e GAP."""

import sys
import unittest
from pathlib import Path


SCRIPTS_DIRECTORY = Path(__file__).resolve().parents[2] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIRECTORY))

from vision.line_control import (
    GEOMETRIC_GAP_FORWARD_MAX_FRAMES,
    GREEN_MANEUVER_TIMEOUT_FRAMES,
    QUADROS_CENTRALIZADO_PARA_CONCLUIR,
    QUADROS_PARA_REARMAR_VERDE,
)
from vision.maneuvers import LineManeuverState


class FakeLineController:
    def __init__(self):
        self.started_searches = []
        self.stop_count = 0

    def start_search(self, preferred_direction=None):
        self.started_searches.append(preferred_direction)

    def stop_search(self):
        self.stop_count += 1


class LineManeuverStateTests(unittest.TestCase):
    def test_green_timeout_starts_search_in_confirmed_direction(self):
        controller = FakeLineController()
        state = LineManeuverState(
            green_direction="ESQUERDA",
            green_active_frames=GREEN_MANEUVER_TIMEOUT_FRAMES - 1,
        )

        result = state.apply_green_timeout(False, controller)

        self.assertTrue(result["timedOut"])
        self.assertFalse(result["sensorRecoveryRequested"])
        self.assertEqual(state.green_direction, "NENHUMA")
        self.assertEqual(controller.started_searches, ["LEFT"])

    def test_gap_enters_searches_and_finishes_after_near_reacquisition(self):
        controller = FakeLineController()
        state = LineManeuverState()
        state.update_near_history(True)

        entered = state.enter_gap_if_required(
            near_center_visible=False,
            real_near_point=None,
            virtual_near_point=None,
            lateral_exit_target=None,
        )

        self.assertTrue(entered)
        state.gap_forward_frames = GEOMETRIC_GAP_FORWARD_MAX_FRAMES - 1
        self.assertTrue(state.update_gap_recovery(False, controller))
        self.assertTrue(state.update_gap_recovery(True, controller))
        self.assertFalse(state.update_gap_recovery(True, controller))
        self.assertFalse(state.gap_forward_active)
        self.assertEqual(controller.stop_count, 1)

    def test_green_alignment_completes_and_rearms_after_clear_frames(self):
        controller = FakeLineController()
        state = LineManeuverState(
            green_direction="DIREITA",
            green_armed=False,
            green_clear_frames=QUADROS_PARA_REARMAR_VERDE,
        )

        state.update_green_alignment(0.20, True, controller)
        self.assertTrue(state.green_curve_started)

        for _ in range(QUADROS_CENTRALIZADO_PARA_CONCLUIR):
            state.update_green_alignment(0.0, True, controller)

        self.assertEqual(state.green_direction, "NENHUMA")
        self.assertFalse(state.green_curve_started)
        self.assertTrue(state.green_armed)
        self.assertEqual(state.green_clear_frames, 0)
        self.assertEqual(controller.stop_count, 1)


if __name__ == "__main__":
    unittest.main()
