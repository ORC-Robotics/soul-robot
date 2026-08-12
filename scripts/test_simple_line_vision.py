"""Testes sintéticos do detector simples, sem câmera ou GPIO."""

import unittest

import numpy as np

try:
    from simple_line_vision import ROI_TOP_RATIO, detect_line, render_line_debug
except ModuleNotFoundError as error:
    if error.name != "cv2":
        raise
    ROI_TOP_RATIO = 0.75
    detect_line = None
    render_line_debug = None


@unittest.skipIf(detect_line is None, "OpenCV não está instalado neste ambiente.")
class SimpleLineVisionTest(unittest.TestCase):
    def make_frame(self):
        return np.full((240, 320, 3), 255, dtype=np.uint8)

    def paint_line(self, frame, center_x, width=24):
        roi_top = int(frame.shape[0] * ROI_TOP_RATIO)
        left = center_x - width // 2
        right = center_x + width // 2
        frame[roi_top:, left:right] = 0

    def test_centered_line_has_near_zero_error(self):
        frame = self.make_frame()
        self.paint_line(frame, 160)

        detection = detect_line(frame)

        self.assertTrue(detection.detected)
        self.assertAlmostEqual(detection.error_normalized, 0.0, delta=0.01)
        self.assertLess(detection.angle_from_vertical_degrees, 5.0)
        self.assertFalse(detection.sharp_turn_candidate)

    def test_line_on_left_has_negative_error(self):
        frame = self.make_frame()
        self.paint_line(frame, 80)

        detection = detect_line(frame)

        self.assertTrue(detection.detected)
        self.assertLess(detection.error_normalized, -0.45)

    def test_line_on_right_has_positive_error(self):
        frame = self.make_frame()
        self.paint_line(frame, 240)

        detection = detect_line(frame)

        self.assertTrue(detection.detected)
        self.assertGreater(detection.error_normalized, 0.45)

    def test_black_above_roi_is_ignored(self):
        frame = self.make_frame()
        roi_top = int(frame.shape[0] * ROI_TOP_RATIO)
        frame[:roi_top, 140:180] = 0

        detection = detect_line(frame)

        self.assertFalse(detection.detected)

    def test_blank_frame_has_no_line(self):
        detection = detect_line(self.make_frame())

        self.assertFalse(detection.detected)
        self.assertEqual(detection.error_normalized, 0.0)

    def test_horizontal_segment_on_right_is_sharp_turn_candidate(self):
        frame = self.make_frame()
        roi_top = int(frame.shape[0] * ROI_TOP_RATIO)
        frame[roi_top + 5:roi_top + 35, 140:320] = 0

        detection = detect_line(frame)

        self.assertTrue(detection.sharp_turn_candidate)
        self.assertEqual(detection.sharp_turn_direction, 1)
        self.assertGreater(detection.angle_from_vertical_degrees, 80.0)

    def test_horizontal_segment_on_left_is_sharp_turn_candidate(self):
        frame = self.make_frame()
        roi_top = int(frame.shape[0] * ROI_TOP_RATIO)
        frame[roi_top + 5:roi_top + 35, 0:180] = 0

        detection = detect_line(frame)

        self.assertTrue(detection.sharp_turn_candidate)
        self.assertEqual(detection.sharp_turn_direction, -1)

    def test_debug_frame_preserves_size_and_adds_visual_information(self):
        frame = self.make_frame()
        self.paint_line(frame, 160)
        detection = detect_line(frame)

        debug_frame = render_line_debug(frame, detection)

        self.assertEqual(debug_frame.shape, frame.shape)
        self.assertFalse(np.array_equal(debug_frame, frame))


if __name__ == "__main__":
    unittest.main()
