"""Verificação curta da área válida e do debounce da chegada."""

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from vision.red_detection import RedFinishDetector, red_valid_mask, draw_red_overlay
from vision.virtual_sensors import resolve_virtual_sensor_geometry, virtual_sensor_regions


class RedDetectionTest(unittest.TestCase):
    def test_union_and_consecutive_frames(self):
        shape = (360, 480)
        valid = red_valid_mask(*shape)
        expected = np.zeros(shape, np.uint8)
        for row in resolve_virtual_sensor_geometry(shape).values():
            for name, sensor in row.items():
                if name != "position":
                    for region in virtual_sensor_regions(sensor):
                        expected[region["y0"]:region["y1"], region["x0"]:region["x1"]] = 255
        np.testing.assert_array_equal(valid, expected)
        detector = RedFinishDetector()
        control = dict(redMinRatio=0.10, redConfirmFrames=4, redMaxFrameGapMs=125)
        frame = np.zeros((*shape, 3), np.uint8)
        frame[valid == 0] = (0, 0, 255)
        timestamp = 1.0

        def process(image):
            nonlocal timestamp
            timestamp += 0.02
            return detector.process(image, "RGB888", timestamp, control)

        status, _ = process(frame)
        self.assertEqual(status["redRatio"], 0.0)
        # Apenas parte da união basta; não é necessário atingir todos os sensores.
        pixels = np.argwhere(valid != 0)
        count = int(np.ceil(len(pixels) * 0.11))
        selected = pixels[:count]
        frame[:] = 0
        frame[selected[:, 0], selected[:, 1]] = (0, 0, 255)
        for _ in range(3):
            self.assertFalse(process(frame)[0]["redConfirmed"])
        status, mask = process(frame)
        self.assertTrue(status["redConfirmed"])

        # Captura inválida e timestamp repetido quebram a sequência pendente.
        self.assertFalse(detector.process(None, "RGB888", timestamp, control)[0]["redValid"])
        for _ in range(3):
            self.assertFalse(process(frame)[0]["redConfirmed"])
        self.assertFalse(detector.process(frame, "RGB888", timestamp, control)[0]["redValid"])
        self.assertFalse(process(frame)[0]["redConfirmed"])
        self.assertAlmostEqual(status["redRatio"], count / len(pixels))
        original = frame.copy()
        draw_red_overlay(frame.copy(), status, mask)
        np.testing.assert_array_equal(frame, original)
        for color in [(0, 0, 0), (0, 255, 0), (255, 255, 255)]:
            frame[:] = color
            self.assertFalse(process(frame)[0]["redConfirmed"])
        self.assertTrue(process(frame)[0]["redClearConfirmed"])
        # O outro extremo do círculo HSV também representa vermelho.
        hsv = np.zeros_like(frame)
        hsv[:] = (175, 255, 255)
        frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        for _ in range(3):
            self.assertFalse(process(frame)[0]["redConfirmed"])
        timestamp += 1.0
        self.assertFalse(process(frame)[0]["redConfirmed"])
        for _ in range(3):
            status, _ = process(frame)
        self.assertTrue(status["redConfirmed"])


if __name__ == "__main__":
    unittest.main()
