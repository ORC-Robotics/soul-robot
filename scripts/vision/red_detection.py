"""Chegada por cor na união dos sensores virtuais da câmera inferior."""

import math
from functools import lru_cache

import cv2
import numpy as np

from .camera_config import (
    RED_HUE_LOW_MAX,
    RED_HUE_HIGH_MIN,
    RED_MIN_SATURATION,
    RED_MIN_VALUE,
)
from .green_detection import frame_to_hsv
from .virtual_sensors import resolve_virtual_sensor_geometry, virtual_sensor_regions


@lru_cache(maxsize=8)
def red_valid_mask(height, width):
    """Une as áreas reais, incluindo extensões, sem duplicar sobreposições."""
    mask = np.zeros((height, width), dtype=np.uint8)
    geometry = resolve_virtual_sensor_geometry(mask.shape)
    for row in geometry.values():
        for name, sensor in row.items():
            # position descreve a busca de coordenada, não um sensor virtual.
            if name == "position":
                continue
            for region in virtual_sensor_regions(sensor):
                mask[max(0, region["y0"]):min(height, region["y1"]),
                     max(0, region["x0"]):min(width, region["x1"])] = 255
    return mask


class RedFinishDetector:
    """Confirma presença e ausência em frames novos; não comanda o robô."""

    def __init__(self):
        self.red_frames = 0
        self.clear_frames = 0
        self.last_timestamp = 0.0
        self.settings = None

    def process(self, frame, camera_format, timestamp, control):
        status = {"redValid": False, "redRatio": 0.0,
                  "redConfirmed": False, "redClearConfirmed": False}
        frame_valid = (isinstance(frame, np.ndarray) and frame.ndim == 3
                       and frame.shape[2] == 3 and frame.size > 0
                       and frame.dtype == np.uint8)
        mask = np.zeros(frame.shape[:2] if frame_valid else (0, 0), dtype=np.uint8)
        if (not frame_valid or not isinstance(control, dict)
                or not isinstance(timestamp, (int, float))
                or not math.isfinite(timestamp) or timestamp <= self.last_timestamp):
            self.red_frames = self.clear_frames = 0
            return status, mask
        threshold = control.get("redMinRatio")
        frames = control.get("redConfirmFrames")
        max_gap = control.get("redMaxFrameGapMs")
        valid_config = (isinstance(threshold, (int, float))
                        and math.isfinite(threshold) and 0 < threshold <= 1
                        and type(frames) is int and frames > 0
                        and isinstance(max_gap, (int, float))
                        and math.isfinite(max_gap) and max_gap > 0)
        settings = (threshold, frames, max_gap)
        if (not valid_config or settings != self.settings
                or not 0 < timestamp - self.last_timestamp <= max_gap / 1000.0):
            self.red_frames = self.clear_frames = 0
        self.settings = settings
        self.last_timestamp = timestamp
        if not valid_config:
            return status, mask
        try:
            valid_mask = red_valid_mask(*frame.shape[:2])
            area = cv2.countNonZero(valid_mask)
            if area == 0:
                raise ValueError("Área dos sensores virtuais vazia.")
            hsv = frame_to_hsv(frame, camera_format)
            mask = cv2.bitwise_or(
                cv2.inRange(hsv, (0, RED_MIN_SATURATION, RED_MIN_VALUE),
                            (RED_HUE_LOW_MAX, 255, 255)),
                cv2.inRange(hsv, (RED_HUE_HIGH_MIN, RED_MIN_SATURATION, RED_MIN_VALUE),
                            (179, 255, 255)))
            mask = cv2.bitwise_and(mask, valid_mask)
            ratio = cv2.countNonZero(mask) / area
            self.red_frames = min(frames, self.red_frames + 1) if ratio >= threshold else 0
            self.clear_frames = min(frames, self.clear_frames + 1) if ratio < threshold else 0
            status.update(redValid=True, redRatio=ratio,
                          redConfirmed=self.red_frames >= frames,
                          redClearConfirmed=self.clear_frames >= frames)
        except (ValueError, cv2.error):
            self.red_frames = self.clear_frames = 0
        return status, mask


def draw_red_overlay(frame, status, mask):
    """Desenha somente na visualização, sem modificar a imagem analisada."""
    active = mask != 0
    frame[active] = (frame[active].astype(np.float32) * 0.6
                     + np.array([0, 0, 255], dtype=np.float32) * 0.4).astype(np.uint8)
    label = f"RED: {status['redRatio'] * 100:.1f}%"
    if status["redConfirmed"]:
        label += "  RED CONFIRMED"
    elif not status["redValid"]:
        label += "  UNAVAILABLE"
    origin = (8, frame.shape[0] - 12)
    cv2.putText(frame, label, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
    cv2.putText(frame, label, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
