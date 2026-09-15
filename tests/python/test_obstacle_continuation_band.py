"""Valida a faixa transversal no IPC inferior sem câmera ou motores."""

import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))

from vision import status_publisher
from vision.camera_config import VIRTUAL_ROW_MIN_ACTIVATION


class ObstacleContinuationBandTests(unittest.TestCase):
    def publish_band(self, sectors):
        """Executa o publicador real e lê o booleano gravado no IPC."""
        command = {
            "left_power": 0.5,
            "right_power": 0.5,
            "controlSource": "fusion",
            "nearCenter": 0.0,
            "nearFinePosition": None,
            **sectors,
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "line.json"
            with patch.object(status_publisher, "LINE_STATUS_PATH", str(path)), \
                    patch.object(status_publisher, "TEMP_LINE_STATUS_PATH", str(path) + ".tmp"):
                status_publisher.save_line_status(command, 100.0, 1, {})
            published = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(published["lineControlSource"], "fusion")
        self.assertEqual(published["lineFollowerLeftPower"], 0.5)
        self.assertEqual(published["lineFollowerRightPower"], 0.5)
        self.assertIs(type(published["obstacleContinuationBand"]), bool)
        return published["obstacleContinuationBand"]

    def test_only_coverage_of_all_columns_confirms(self):
        # Todas as 64 combinações incluem linhas laterais e misturas de fileiras.
        keys = [row + side for row in ("medium", "far")
                for side in ("Left", "Center", "Right")]
        for mask in range(64):
            with self.subTest(mask=mask):
                sectors = {key: VIRTUAL_ROW_MIN_ACTIVATION if mask & (1 << index) else 0.0
                           for index, key in enumerate(keys)}
                expected = ((mask & 7) | (mask >> 3)) == 7
                self.assertEqual(self.publish_band(sectors), expected)

    def test_diagonal_coverage_confirms_with_rows_mixed(self):
        for keys in (("mediumLeft", "farCenter", "mediumRight"),
                     ("farLeft", "mediumCenter", "farRight")):
            with self.subTest(keys=keys):
                self.assertTrue(self.publish_band(
                    {key: VIRTUAL_ROW_MIN_ACTIVATION for key in keys}))

    def test_missing_invalid_or_below_existing_threshold_needs_other_row(self):
        self.assertFalse(self.publish_band({}))
        for row in ("medium", "far"):
            for side in ("Left", "Center", "Right"):
                for value in (None, "invalid", float("nan"), float("inf"),
                              VIRTUAL_ROW_MIN_ACTIVATION - 0.000001):
                    with self.subTest(row=row, side=side, value=value):
                        sectors = {row + sector: VIRTUAL_ROW_MIN_ACTIVATION
                                   for sector in ("Left", "Center", "Right")}
                        sectors[row + side] = value
                        self.assertFalse(self.publish_band(sectors))
                        other_row = "far" if row == "medium" else "medium"
                        sectors[other_row + side] = VIRTUAL_ROW_MIN_ACTIVATION
                        self.assertTrue(self.publish_band(sectors))


if __name__ == "__main__":
    unittest.main()
