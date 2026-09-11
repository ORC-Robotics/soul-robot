"""Mede preto nas ROIs exclusivas do caso 3 do obstáculo."""

import cv2
import numpy as np

from .obstacle_black import (
    OBSTACLE_BLACK_MIN_COMPONENT_AREA_PX,
    OBSTACLE_BLACK_THRESHOLD,
)


PARABOLA_NEAR_Y0 = 0.55
PARABOLA_NEAR_Y1 = 1.00
PARABOLA_LEFT_X0 = 0.05
PARABOLA_LEFT_X1 = 0.50
PARABOLA_RIGHT_X0 = 0.50
PARABOLA_RIGHT_X1 = 0.95
PARABOLA_FORWARD_X0 = 0.25
PARABOLA_FORWARD_X1 = 0.75
PARABOLA_FORWARD_MIN_BLACK_PIXELS = 3000


def _resolve_roi(frame_shape, x0_ratio, x1_ratio):
    """Converte uma ROI normalizada do caso 3 em limites inteiros."""

    height, width = int(frame_shape[0]), int(frame_shape[1])
    if height <= 0 or width <= 0:
        raise ValueError("O frame frontal não pode estar vazio.")
    x0 = max(0, min(width - 1, int(round(width * x0_ratio))))
    x1 = max(x0 + 1, min(width, int(round(width * x1_ratio))))
    y0 = max(0, min(height - 1, int(round(height * PARABOLA_NEAR_Y0))))
    y1 = max(y0 + 1, min(height, int(round(height * PARABOLA_NEAR_Y1))))
    return x0, y0, x1, y1


def _measure_filtered_black(frame, roi):
    """Mede preto após remover somente componentes equivalentes a ruído."""

    x0, y0, x1, y1 = roi
    gray = cv2.cvtColor(frame[y0:y1, x0:x1, :3], cv2.COLOR_BGR2GRAY)
    raw_mask = np.where(gray <= OBSTACLE_BLACK_THRESHOLD, 255, 0).astype(np.uint8)
    count, _, stats, _ = cv2.connectedComponentsWithStats(raw_mask, connectivity=8)
    pixel_count = 0
    largest_component = 0
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < OBSTACLE_BLACK_MIN_COMPONENT_AREA_PX:
            continue
        pixel_count += area
        largest_component = max(largest_component, area)
    return pixel_count, largest_component


def analyze_parabola_black(frame, sequence):
    """Mede as duas laterais e a validação central NEAR do caso 3."""

    if not isinstance(frame, np.ndarray) or frame.ndim != 3 or frame.shape[2] < 3:
        raise ValueError("O detector da parábola exige um frame colorido.")
    sequence = int(sequence)
    if sequence < 0:
        raise ValueError("A sequência da parábola não pode ser negativa.")

    left_black, _ = _measure_filtered_black(
        frame, _resolve_roi(frame.shape, PARABOLA_LEFT_X0, PARABOLA_LEFT_X1)
    )
    right_black, _ = _measure_filtered_black(
        frame, _resolve_roi(frame.shape, PARABOLA_RIGHT_X0, PARABOLA_RIGHT_X1)
    )
    forward_black, forward_largest = _measure_filtered_black(
        frame, _resolve_roi(frame.shape, PARABOLA_FORWARD_X0, PARABOLA_FORWARD_X1)
    )
    return {
        "parabolaLeftBlack": left_black,
        "parabolaRightBlack": right_black,
        "parabolaSequence": sequence,
        "parabolaNearForwardBlack": forward_black,
        "parabolaNearForwardLargest": forward_largest,
        "parabolaNearForwardVisible": (
            forward_black >= PARABOLA_FORWARD_MIN_BLACK_PIXELS
            and forward_largest >= PARABOLA_FORWARD_MIN_BLACK_PIXELS
        ),
    }


def empty_parabola_black(sequence):
    """Publica evidência neutra quando a CAM1 está em outro modo exclusivo."""

    return {
        "parabolaLeftBlack": 0,
        "parabolaRightBlack": 0,
        "parabolaSequence": int(sequence),
        "parabolaNearForwardBlack": 0,
        "parabolaNearForwardLargest": 0,
        "parabolaNearForwardVisible": False,
    }


def draw_parabola_black_overlay(frame, result):
    """Desenha somente as duas medições laterais usadas pelo caso 3."""

    left_roi = _resolve_roi(frame.shape, PARABOLA_LEFT_X0, PARABOLA_LEFT_X1)
    right_roi = _resolve_roi(frame.shape, PARABOLA_RIGHT_X0, PARABOLA_RIGHT_X1)
    left_black = int(result.get("parabolaLeftBlack", 0))
    right_black = int(result.get("parabolaRightBlack", 0))
    current_winner = (
        "LEFT" if left_black > right_black
        else "RIGHT" if right_black > left_black
        else "NONE"
    )

    neutral_color = (220, 180, 0)
    winner_color = (0, 220, 255)
    for side, roi in (("LEFT", left_roi), ("RIGHT", right_roi)):
        x0, y0, x1, y1 = roi
        winning = side == current_winner
        color = winner_color if winning else neutral_color
        thickness = 3 if winning else 1
        cv2.rectangle(frame, (x0, y0), (x1 - 1, y1 - 1), color, thickness)
        label = f"PARABOLA {side} ROI"
        cv2.putText(
            frame, label, (x0 + 6, y0 + 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 3, cv2.LINE_AA,
        )
        cv2.putText(
            frame, label, (x0 + 6, y0 + 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA,
        )

    for index, text in enumerate((
        "PARABOLA:",
        f"L={left_black}  R={right_black}",
    )):
        y = 24 + index * 22
        cv2.putText(
            frame, text, (10, y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 3, cv2.LINE_AA,
        )
        cv2.putText(
            frame, text, (10, y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.52, winner_color, 1, cv2.LINE_AA,
        )
    return frame
