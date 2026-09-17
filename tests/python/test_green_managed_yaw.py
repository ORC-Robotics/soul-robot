"""Verifica a liberação do centro pelo coordenador, sem câmera ou motores."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))

from vision.fusion_guidance import managed_green_curve_started
from vision.virtual_sensors import select_virtual_trust_row_geometry


class ManagedGreenYawTest(unittest.TestCase):
    def test_both_sides_include_center_only_after_minimum_yaw(self):
        geometry = {side: object() for side in ("left", "center", "right")}
        for direction, camera_direction, side in (
            ("LEFT", "ESQUERDA", "left"),
            ("RIGHT", "DIREITA", "right"),
        ):
            with self.subTest(direction=direction):
                control = {
                    "greenManagedByRobot": True,
                    "greenManeuverDirection": direction,
                    "greenMinimumYawReached": False,
                }
                before = select_virtual_trust_row_geometry(
                    geometry, camera_direction, managed_green_curve_started(control)
                )
                self.assertEqual(set(before), {side})
                control["greenMinimumYawReached"] = True
                after = select_virtual_trust_row_geometry(
                    geometry, camera_direction, managed_green_curve_started(control)
                )
                self.assertEqual(set(after), {side, "center"})

    def test_missing_or_inactive_authorization_does_not_release_center(self):
        for control in (
            {},
            {"greenManagedByRobot": True, "greenManeuverDirection": "RIGHT"},
            {"greenManagedByRobot": False, "greenManeuverDirection": "LEFT",
             "greenMinimumYawReached": True},
            {"greenManagedByRobot": True, "greenManeuverDirection": "NONE",
             "greenMinimumYawReached": True},
            {"greenManagedByRobot": True, "greenManeuverDirection": "RIGHT",
             "greenMinimumYawReached": "true"},
        ):
            with self.subTest(control=control):
                self.assertFalse(managed_green_curve_started(control))


if __name__ == "__main__":
    unittest.main()
