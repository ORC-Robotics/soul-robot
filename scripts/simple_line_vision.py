"""Detecta uma linha preta usando somente uma ROI horizontal inferior."""

from dataclasses import dataclass
import math

import cv2


# A ROI usa apenas o quarto inferior da imagem, fisicamente mais próximo do robô.
# Aumentar este valor reduz ainda mais a antecipação, mas também diminui a área útil.
ROI_TOP_RATIO = 0.75

# Pixels com intensidade abaixo deste limite entram na máscara da linha preta.
# Ajuste este valor após observar a iluminação real da pista.
BLACK_THRESHOLD = 100

# Área mínima, em pixels, necessária para impedir que pequenos ruídos gerem comando.
MINIMUM_BLACK_PIXELS = 100

# Uma curva fechada precisa combinar deslocamento, orientação horizontal,
# largura e cobertura. Nenhum critério isolado pode armar o giro no próprio eixo.
SHARP_TURN_MINIMUM_ABS_ERROR = 0.20
SHARP_TURN_MINIMUM_ANGLE_FROM_VERTICAL_DEGREES = 45.0
SHARP_TURN_MINIMUM_HORIZONTAL_SPAN_RATIO = 0.50
SHARP_TURN_MINIMUM_MASK_COVERAGE = 0.12


@dataclass(frozen=True)
class LineDetection:
    detected: bool
    error_normalized: float
    center_x: float
    roi_top_y: int
    roi_height: int
    black_pixel_count: int
    angle_from_vertical_degrees: float = 0.0
    horizontal_span_ratio: float = 0.0
    mask_coverage: float = 0.0
    sharp_turn_candidate: bool = False
    sharp_turn_direction: int = 0


def _create_black_mask(frame):
    """Cria a máscara usada tanto pelo controle quanto pelo debug visual."""

    frame_height, frame_width = frame.shape[:2]
    roi_top_y = int(frame_height * ROI_TOP_RATIO)
    roi = frame[roi_top_y:frame_height, 0:frame_width]
    if roi.size == 0:
        return roi_top_y, None

    if roi.ndim == 2:
        gray = roi
    else:
        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)

    _, black_mask = cv2.threshold(
        gray,
        BLACK_THRESHOLD,
        255,
        cv2.THRESH_BINARY_INV,
    )
    return roi_top_y, black_mask


def detect_line(frame) -> LineDetection:
    """Retorna o centroide da máscara preta dentro de uma única ROI inferior."""

    if frame is None or frame.ndim < 2:
        return LineDetection(False, 0.0, -1.0, 0, 0, 0)

    frame_height, frame_width = frame.shape[:2]
    if frame_height <= 0 or frame_width <= 0:
        return LineDetection(False, 0.0, -1.0, 0, 0, 0)

    roi_top_y, black_mask = _create_black_mask(frame)
    if black_mask is None:
        return LineDetection(False, 0.0, -1.0, roi_top_y, 0, 0)
    roi_height = black_mask.shape[0]

    # binaryImage=True faz o momento M00 representar a quantidade de pixels
    # ativos, independentemente do valor 255 usado na máscara.
    moments = cv2.moments(black_mask, binaryImage=True)
    black_pixel_count = int(round(moments["m00"]))
    if black_pixel_count < MINIMUM_BLACK_PIXELS:
        return LineDetection(
            False,
            0.0,
            -1.0,
            roi_top_y,
            roi_height,
            black_pixel_count,
        )

    center_x = moments["m10"] / moments["m00"]
    image_center_x = frame_width * 0.5
    error_normalized = (center_x - image_center_x) / image_center_x
    error_normalized = max(-1.0, min(1.0, float(error_normalized)))

    # A orientação principal vem dos momentos centrais da mesma máscara. Zero
    # graus significa segmento vertical; 90 graus significa segmento horizontal.
    angle_from_horizontal_degrees = 0.5 * math.degrees(
        math.atan2(
            2.0 * moments["mu11"],
            moments["mu20"] - moments["mu02"],
        )
    )
    angle_from_vertical_degrees = max(
        0.0,
        min(90.0, 90.0 - abs(angle_from_horizontal_degrees)),
    )

    _active_y, active_x = black_mask.nonzero()
    horizontal_span_pixels = int(active_x.max() - active_x.min() + 1)
    horizontal_span_ratio = horizontal_span_pixels / frame_width
    mask_coverage = black_pixel_count / float(black_mask.size)

    sharp_turn_candidate = (
        abs(error_normalized) >= SHARP_TURN_MINIMUM_ABS_ERROR
        and angle_from_vertical_degrees
        >= SHARP_TURN_MINIMUM_ANGLE_FROM_VERTICAL_DEGREES
        and horizontal_span_ratio >= SHARP_TURN_MINIMUM_HORIZONTAL_SPAN_RATIO
        and mask_coverage >= SHARP_TURN_MINIMUM_MASK_COVERAGE
    )
    sharp_turn_direction = 0
    if sharp_turn_candidate:
        sharp_turn_direction = 1 if error_normalized > 0.0 else -1

    return LineDetection(
        True,
        error_normalized,
        float(center_x),
        roi_top_y,
        roi_height,
        black_pixel_count,
        angle_from_vertical_degrees,
        horizontal_span_ratio,
        mask_coverage,
        sharp_turn_candidate,
        sharp_turn_direction,
    )


def render_line_debug(frame, detection: LineDetection):
    """Desenha no stream a ROI, a máscara aceita e o centroide calculado."""

    if frame is None or frame.ndim < 2:
        return frame

    debug_frame = frame.copy()
    if debug_frame.ndim == 2:
        debug_frame = cv2.cvtColor(debug_frame, cv2.COLOR_GRAY2RGB)
    frame_height, frame_width = debug_frame.shape[:2]
    roi_top_y, black_mask = _create_black_mask(frame)
    if black_mask is None:
        return debug_frame

    # Escurece o trecho que não participa do controle para deixar explícito que
    # somente a faixa inferior influencia os motores.
    if roi_top_y > 0:
        debug_frame[:roi_top_y] = cv2.convertScaleAbs(
            debug_frame[:roi_top_y], alpha=0.35, beta=0
        )

    roi_debug = debug_frame[roi_top_y:frame_height]
    highlighted_roi = roi_debug.copy()
    highlighted_roi[black_mask > 0] = (0, 255, 0)
    cv2.addWeighted(highlighted_roi, 0.45, roi_debug, 0.55, 0, roi_debug)

    center_x = int(round(frame_width * 0.5))
    cv2.line(debug_frame, (0, roi_top_y), (frame_width - 1, roi_top_y), (0, 255, 255), 2)
    cv2.line(debug_frame, (center_x, roi_top_y), (center_x, frame_height - 1), (255, 255, 0), 2)

    if detection.detected:
        centroid_x = int(round(detection.center_x))
        centroid_y = roi_top_y + max(1, detection.roi_height // 2)
        cv2.circle(debug_frame, (centroid_x, centroid_y), 10, (255, 0, 255), 3)
        if detection.sharp_turn_candidate:
            direction_text = "DIREITA" if detection.sharp_turn_direction > 0 else "ESQUERDA"
            state_text = f"CURVA 90 {direction_text} | erro={detection.error_normalized:+.3f}"
            arrow_end_x = frame_width - 30 if detection.sharp_turn_direction > 0 else 30
            cv2.arrowedLine(
                debug_frame,
                (center_x, roi_top_y + max(20, detection.roi_height // 2)),
                (arrow_end_x, roi_top_y + max(20, detection.roi_height // 2)),
                (0, 140, 255),
                5,
                tipLength=0.15,
            )
        else:
            state_text = f"LINHA OK | erro={detection.error_normalized:+.3f}"
    else:
        state_text = "LINHA AUSENTE"

    detail_text = (
        f"ROI {int((1.0 - ROI_TOP_RATIO) * 100)}% | "
        f"anguloV={detection.angle_from_vertical_degrees:.1f} | "
        f"largura={detection.horizontal_span_ratio * 100:.0f}% | "
        f"cobertura={detection.mask_coverage * 100:.0f}%"
    )
    cv2.putText(
        debug_frame,
        state_text,
        (18, 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 0) if detection.detected else (255, 80, 80),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        debug_frame,
        detail_text,
        (18, 62),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return debug_frame
