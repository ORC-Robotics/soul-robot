"""Detecta massa preta simples para diagnóstico do scan de obstáculo."""

import cv2
import numpy as np


# ROI normalizada e simétrica usada somente pelo detector obstacleBlack.
OBSTACLE_BLACK_X0 = 0.28
OBSTACLE_BLACK_X1 = 0.72
OBSTACLE_BLACK_Y0 = 0.72
OBSTACLE_BLACK_Y1 = 1.00

# Limiar absoluto exclusivo deste detector. Ele não altera a segmentação da linha.
OBSTACLE_BLACK_THRESHOLD = 160
# Componentes abaixo desta área, em pixels, são tratados como partículas isoladas.
OBSTACLE_BLACK_MIN_COMPONENT_AREA_PX = 24
# Limites observados no scan real. Pouco preto rejeita ruído e emendas; preto
# demais rejeita o obstáculo ocupando quase toda a ROI.
OBSTACLE_BLACK_MIN_RATIO = 0.06
OBSTACLE_BLACK_MAX_RATIO = 0.65
OBSTACLE_BLACK_MIN_LARGEST_COMPONENT = 3000

def resolve_obstacle_black_roi(frame_shape):
    """Converte a ROI normalizada em limites inteiros exclusivos."""

    height, width = int(frame_shape[0]), int(frame_shape[1])
    if height <= 0 or width <= 0:
        raise ValueError("O frame frontal não pode estar vazio.")
    x0 = max(0, min(width - 1, int(round(width * OBSTACLE_BLACK_X0))))
    x1 = max(x0 + 1, min(width, int(round(width * OBSTACLE_BLACK_X1))))
    y0 = max(0, min(height - 1, int(round(height * OBSTACLE_BLACK_Y0))))
    y1 = max(y0 + 1, min(height, int(round(height * OBSTACLE_BLACK_Y1))))
    return x0, y0, x1, y1


def classify_obstacle_black(ratio, largest_component):
    """Aplica somente os três limites obtidos no teste físico."""

    if ratio > OBSTACLE_BLACK_MAX_RATIO:
        return False, "SATURATED_BLACK"
    if (
        ratio < OBSTACLE_BLACK_MIN_RATIO
        or largest_component < OBSTACLE_BLACK_MIN_LARGEST_COMPONENT
    ):
        return False, "LOW_BLACK"
    return True, "VALID_BLACK"


def analyze_obstacle_black(frame, sequence):
    """Mede preto conectado na ROI sem usar o ForwardPathTracker."""

    if not isinstance(frame, np.ndarray) or frame.ndim != 3 or frame.shape[2] < 3:
        raise ValueError("O detector obstacleBlack exige um frame colorido.")
    sequence = int(sequence)
    if sequence < 0:
        raise ValueError("A sequência obstacleBlack não pode ser negativa.")

    roi = resolve_obstacle_black_roi(frame.shape)
    x0, y0, x1, y1 = roi
    gray = cv2.cvtColor(frame[y0:y1, x0:x1, :3], cv2.COLOR_BGR2GRAY)
    raw_mask = np.where(gray <= OBSTACLE_BLACK_THRESHOLD, 255, 0).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        raw_mask, connectivity=8
    )
    mask = np.zeros_like(raw_mask)
    largest_component = 0
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < OBSTACLE_BLACK_MIN_COMPONENT_AREA_PX:
            continue
        mask[labels == label] = 255
        largest_component = max(largest_component, area)

    pixel_count = int(np.count_nonzero(mask))
    ratio = pixel_count / float(mask.size)
    visible, reason = classify_obstacle_black(ratio, largest_component)
    return {
        "obstacleBlackPixelCount": pixel_count,
        "obstacleBlackRatio": ratio,
        "obstacleBlackLargestComponent": largest_component,
        "obstacleBlackSequence": sequence,
        "obstacleBlackVisible": visible,
        "obstacleBlackReason": reason,
        "roi": roi,
        "mask": mask,
    }


def empty_obstacle_black(frame_shape, sequence):
    """Produz diagnóstico neutro quando outro modo exclusivo usa a CAM1."""

    roi = resolve_obstacle_black_roi(frame_shape)
    x0, y0, x1, y1 = roi
    return {
        "obstacleBlackPixelCount": 0,
        "obstacleBlackRatio": 0.0,
        "obstacleBlackLargestComponent": 0,
        "obstacleBlackSequence": int(sequence),
        "obstacleBlackVisible": False,
        "obstacleBlackReason": "LOW_BLACK",
        "roi": roi,
        "mask": np.zeros((y1 - y0, x1 - x0), dtype=np.uint8),
    }


def draw_obstacle_black_overlay(frame, result):
    """Desenha a ROI e a máscara exclusiva sem alterar o resultado publicado."""

    roi = result.get("roi")
    mask = result.get("mask")
    if not isinstance(roi, (list, tuple)) or len(roi) != 4:
        return frame
    x0, y0, x1, y1 = (int(value) for value in roi)
    visible = result.get("obstacleBlackVisible") is True
    reason = str(result.get("obstacleBlackReason", "LOW_BLACK"))
    color = (0, 220, 0) if visible else (0, 0, 255)
    cv2.rectangle(frame, (x0, y0), (x1 - 1, y1 - 1), color, 2)

    thumbnail_width = min(180, max(80, frame.shape[1] // 5))
    thumbnail_height = max(
        1,
        int(round(thumbnail_width * (y1 - y0) / max(1, x1 - x0))),
    )
    if isinstance(mask, np.ndarray) and mask.ndim == 2 and mask.size > 0:
        thumbnail = cv2.resize(
            mask,
            (thumbnail_width, thumbnail_height),
            interpolation=cv2.INTER_NEAREST,
        )
        thumbnail = cv2.cvtColor(thumbnail, cv2.COLOR_GRAY2BGR)
        thumbnail_x = 8
        thumbnail_y = 92
        frame[
            thumbnail_y:thumbnail_y + thumbnail_height,
            thumbnail_x:thumbnail_x + thumbnail_width,
        ] = thumbnail
        cv2.rectangle(
            frame,
            (thumbnail_x, thumbnail_y),
            (thumbnail_x + thumbnail_width - 1, thumbnail_y + thumbnail_height - 1),
            color,
            1,
        )
    else:
        thumbnail_width = 0

    text_x = 16 + thumbnail_width
    lines = (
        f"OBST BLACK {'YES' if visible else 'NO'} {reason}",
        f"RATIO {float(result.get('obstacleBlackRatio', 0.0)):.4f}",
        f"LARGEST {int(result.get('obstacleBlackLargestComponent', 0))}",
    )
    for index, text in enumerate(lines):
        cv2.putText(
            frame, text, (text_x, 106 + index * 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 3, cv2.LINE_AA,
        )
        cv2.putText(
            frame, text, (text_x, 106 + index * 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA,
        )
    return frame
