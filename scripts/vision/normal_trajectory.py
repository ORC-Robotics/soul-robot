"""Extração da trajetória normal por scanlines."""

import math
import time
from functools import lru_cache
import cv2  # type: ignore
import numpy as np
from .camera_config import (
    GEOMETRIC_PATH_BAND_HALF_HEIGHT,
    NORMAL_TRAJECTORY_BAND_HALF_HEIGHT_PX,
    NORMAL_TRAJECTORY_MAX_MISSING_SCANLINES,
    NORMAL_TRAJECTORY_MAX_SHIFT_WIDTH_RATIO,
    NORMAL_TRAJECTORY_MAX_SLOPE_X_PER_Y,
    NORMAL_TRAJECTORY_MAX_WIDTH_FACTOR,
    NORMAL_TRAJECTORY_MIN_POINTS,
    NORMAL_TRAJECTORY_MIN_VERTICAL_COVERAGE,
    NORMAL_TRAJECTORY_MIN_WIDTH_FACTOR,
    NORMAL_TRAJECTORY_REFERENCE_HEIGHT_PX,
    NORMAL_TRAJECTORY_SCAN_STEP_PX,
)
from .virtual_sensors import (
    cached_virtual_sensor_geometry,
    expected_virtual_line_thickness_px,
)

def find_active_band_segments(processed_line_mask, y):
    """
    Encontra os segmentos horizontais da faixa preta em uma pequena
    banda ao redor de uma determinada altura da imagem.

    Cada segmento retornado contém o centro horizontal ponderado
    pela quantidade real de pixels da máscara.
    """

    height, width = processed_line_mask.shape[:2]

    y0 = max(
        0,
        y - GEOMETRIC_PATH_BAND_HALF_HEIGHT,
    )
    y1 = min(
        height,
        y + GEOMETRIC_PATH_BAND_HALF_HEIGHT + 1,
    )

    band = processed_line_mask[y0:y1, :]

    if band.size == 0:
        return []

    active_columns = np.any(band > 0, axis=0)
    active_x = np.flatnonzero(active_columns)

    if active_x.size == 0:
        return []

    split_indices = np.where(
        np.diff(active_x) > 1
    )[0] + 1

    groups = np.split(
        active_x,
        split_indices,
    )

    segments = []

    for group in groups:
        if group.size == 0:
            continue

        x0 = int(group[0])
        x1 = int(group[-1])

        column_weights = np.count_nonzero(
            band[:, x0:x1 + 1],
            axis=0,
        ).astype(np.float32)

        total_weight = float(column_weights.sum())

        if total_weight <= 0.0:
            continue

        columns = np.arange(
            x0,
            x1 + 1,
            dtype=np.float32,
        )

        center_x = float(
            np.sum(columns * column_weights)
            / total_weight
        )

        segments.append({
            "x0": x0,
            "x1": x1,
            "centerX": center_x,
        })

    return segments


def empty_normal_trajectory(processing_ms=0.0):
    """Cria a telemetria vazia da trajetória NORMAL experimental."""

    return {
        "enabled": False,
        "valid": False,
        "coordinateFrame": "processedLineMaskPixels",
        "pointOrder": "nearToFar",
        # No Soul, o eixo traseiro é a referência principal do movimento. Sua
        # projeção ainda precisa de calibração extrínseca; até lá, os pontos
        # permanecem originais para não confundir centro da imagem com erro
        # lateral ou heading físico do robô.
        "rearAxleProjectionCalibrated": False,
        "points": [],
        "scanlines": [],
        "pointCount": 0,
        "sampledScanlineCount": 0,
        "evaluatedScanlineCount": 0,
        "observedScanlineCount": 0,
        "ambiguousScanlineCount": 0,
        "rejectedSegmentCount": 0,
        "verticalCoverage": 0.0,
        "processingMs": max(0.0, float(processing_ms)),
    }


@lru_cache(maxsize=8)
def cached_normal_trajectory_envelope(height, width):
    """Calcula uma vez o envelope Fusion e seus limites por scanline."""

    if height <= 0 or width <= 0:
        return None

    geometry = cached_virtual_sensor_geometry(height, width)
    far_y = max(0, min(height - 1, geometry["far"]["left"]["y0"]))
    near_start_y = max(
        far_y,
        min(height - 1, geometry["near"]["position"]["y0"]),
    )
    near_y = max(
        near_start_y,
        min(height - 1, geometry["near"]["position"]["y1"] - 1),
    )
    envelope = {
        "geometry": geometry,
        "farY": far_y,
        "nearStartY": near_start_y,
        "nearY": near_y,
        "width": width,
    }
    # Os limites dependem somente da resolução e das ROIs calibradas. Guardá-los
    # evita reconstruir a mesma geometria em cada scanline de cada frame.
    envelope["horizontalBounds"] = tuple(
        normal_trajectory_horizontal_bounds(envelope, y)
        for y in range(height)
    )
    return envelope


def resolve_normal_trajectory_envelope(frame_shape):
    """Reaproveita FAR, MEDIUM e NEAR como envelope físico do traçado."""

    height, width = frame_shape[:2]
    return cached_normal_trajectory_envelope(int(height), int(width))


def normal_trajectory_horizontal_bounds(envelope, y):
    """Limita cada scanline à ROI calibrada que cobre sua altura."""

    horizontal_bounds = envelope.get("horizontalBounds")
    if (
        horizontal_bounds is not None
        and isinstance(y, (int, np.integer))
        and 0 <= int(y) < len(horizontal_bounds)
    ):
        return horizontal_bounds[int(y)]

    geometry = envelope["geometry"]
    width = envelope["width"]
    if y < geometry["medium"]["left"]["y0"]:
        left_x = geometry["far"]["left"]["x0"]
        right_x = geometry["far"]["right"]["x1"]
    elif y < geometry["near"]["position"]["y0"]:
        left_x = geometry["medium"]["left"]["x0"]
        right_x = geometry["medium"]["right"]["x1"]
    else:
        left_x = geometry["near"]["position"]["x0"]
        right_x = geometry["near"]["position"]["x1"]

    left_x = max(0, min(width, int(left_x)))
    right_x = max(left_x, min(width, int(right_x)))
    return left_x, right_x


def normal_trajectory_scanline_ys(envelope, frame_height):
    """Gera alturas do NEAR ao FAR e inclui exatamente os limites das ROIs."""

    scan_step = max(
        1,
        int(round(
            NORMAL_TRAJECTORY_SCAN_STEP_PX
            * float(frame_height)
            / NORMAL_TRAJECTORY_REFERENCE_HEIGHT_PX
        )),
    )
    scanline_ys = list(range(
        envelope["nearY"],
        envelope["farY"] - 1,
        -scan_step,
    ))
    for boundary_y in (envelope["nearStartY"], envelope["farY"]):
        if boundary_y not in scanline_ys:
            scanline_ys.append(boundary_y)
    return sorted(set(scanline_ys), reverse=True)


def find_normal_trajectory_segments(processed_line_mask, envelope, y):
    """Encontra segmentos com largura plausível em uma scanline horizontal."""

    height, _width = processed_line_mask.shape[:2]
    band_half_height = max(
        0,
        int(round(
            NORMAL_TRAJECTORY_BAND_HALF_HEIGHT_PX
            * float(height)
            / NORMAL_TRAJECTORY_REFERENCE_HEIGHT_PX
        )),
    )
    left_x, right_x = normal_trajectory_horizontal_bounds(envelope, y)
    y0 = max(0, int(y) - band_half_height)
    y1 = min(height, int(y) + band_half_height + 1)
    band = processed_line_mask[y0:y1, left_x:right_x]
    if band.size == 0:
        return [], []

    active_columns = np.any(band > 0, axis=0)
    active_x = np.flatnonzero(active_columns)
    if active_x.size == 0:
        return [], []

    groups = np.split(active_x, np.where(np.diff(active_x) > 1)[0] + 1)
    expected_width_px = expected_virtual_line_thickness_px(
        processed_line_mask.shape,
        y,
    )
    minimum_width_px = max(
        2.0,
        expected_width_px * NORMAL_TRAJECTORY_MIN_WIDTH_FACTOR,
    )
    maximum_width_px = max(
        minimum_width_px,
        expected_width_px * NORMAL_TRAJECTORY_MAX_WIDTH_FACTOR,
    )
    segments = []
    rejected_segments = []
    for group in groups:
        if group.size == 0:
            continue
        local_x0 = int(group[0])
        local_x1 = int(group[-1])
        segment_width_px = float(local_x1 - local_x0 + 1)
        if not minimum_width_px <= segment_width_px <= maximum_width_px:
            rejected_segments.append({
                "x0": left_x + local_x0,
                "x1": left_x + local_x1,
                "centerX": float(
                    2 * left_x + local_x0 + local_x1
                ) / 2.0,
                "widthPx": segment_width_px,
                "expectedWidthPx": float(expected_width_px),
                "rejectionReason": "width",
            })
            continue

        column_weights = np.count_nonzero(
            band[:, local_x0:local_x1 + 1],
            axis=0,
        ).astype(np.float32)
        total_weight = float(column_weights.sum())
        if total_weight <= 0.0:
            rejected_segments.append({
                "x0": left_x + local_x0,
                "x1": left_x + local_x1,
                "centerX": float(
                    2 * left_x + local_x0 + local_x1
                ) / 2.0,
                "widthPx": segment_width_px,
                "expectedWidthPx": float(expected_width_px),
                "rejectionReason": "empty",
            })
            continue
        absolute_columns = np.arange(
            left_x + local_x0,
            left_x + local_x1 + 1,
            dtype=np.float32,
        )
        center_x = float(
            np.sum(absolute_columns * column_weights) / total_weight
        )
        segments.append({
            "x0": left_x + local_x0,
            "x1": left_x + local_x1,
            "centerX": center_x,
            "widthPx": segment_width_px,
            "expectedWidthPx": float(expected_width_px),
        })
    return segments, rejected_segments


def select_normal_trajectory_continuation(segments, path_points, y, frame_width):
    """Escolhe o segmento que melhor prolonga posição, largura e direção."""

    if not segments or not path_points:
        return None

    previous = path_points[-1]
    forward_delta_y = max(1.0, float(previous["y"] - y))
    previous_slope = None
    predicted_x = float(previous["x"])
    if len(path_points) >= 2:
        before_previous = path_points[-2]
        previous_delta_y = max(
            1.0,
            float(before_previous["y"] - previous["y"]),
        )
        previous_slope = (
            float(previous["x"]) - float(before_previous["x"])
        ) / previous_delta_y
        predicted_x += previous_slope * forward_delta_y

    maximum_shift_px = max(
        float(previous["expectedWidthPx"]) * 1.5,
        forward_delta_y * NORMAL_TRAJECTORY_MAX_SLOPE_X_PER_Y,
    )
    maximum_shift_px = min(
        maximum_shift_px,
        float(frame_width) * NORMAL_TRAJECTORY_MAX_SHIFT_WIDTH_RATIO,
    )

    compatible = []
    for segment in segments:
        center_distance_px = abs(float(segment["centerX"]) - predicted_x)
        if center_distance_px > maximum_shift_px:
            continue

        width_change = abs(math.log(
            max(1.0, float(segment["widthPx"]))
            / max(1.0, float(previous["widthPx"]))
        ))
        direction_change = 0.0
        if previous_slope is not None:
            candidate_slope = (
                float(segment["centerX"]) - float(previous["x"])
            ) / forward_delta_y
            direction_change = abs(candidate_slope - previous_slope)
        score = (
            center_distance_px / max(1.0, maximum_shift_px)
            + 0.20 * min(2.0, width_change)
            + 0.25 * min(2.0, direction_change)
        )
        compatible.append((score, center_distance_px, segment))

    if not compatible:
        return None
    return min(compatible, key=lambda item: (item[0], item[1]))[2]


def normal_trajectory_segment_telemetry(segment, rejection_reason=None):
    """Converte um segmento interno em dados simples para overlay e status."""

    telemetry = {
        "x0": int(segment["x0"]),
        "x1": int(segment["x1"]),
        "centerX": round(float(segment["centerX"]), 2),
        "widthPx": round(float(segment["widthPx"]), 2),
        "expectedWidthPx": round(float(segment["expectedWidthPx"]), 2),
    }
    if rejection_reason is not None:
        telemetry["rejectionReason"] = str(rejection_reason)
    return telemetry


def extract_normal_line_trajectory(processed_line_mask):
    """Extrai pontos contínuos para diagnóstico do futuro steering NORMAL."""

    extraction_started = time.perf_counter()
    if (
        not isinstance(processed_line_mask, np.ndarray)
        or processed_line_mask.ndim != 2
        or processed_line_mask.size == 0
    ):
        return empty_normal_trajectory(
            (time.perf_counter() - extraction_started) * 1000.0
        )

    envelope = resolve_normal_trajectory_envelope(processed_line_mask.shape)
    if envelope is None:
        return empty_normal_trajectory(
            (time.perf_counter() - extraction_started) * 1000.0
        )

    scanline_ys = normal_trajectory_scanline_ys(
        envelope,
        processed_line_mask.shape[0],
    )
    near_position = envelope["geometry"]["near"]["position"]
    seed_reference_x = (
        float(near_position["x0"] + near_position["x1"] - 1) / 2.0
    )
    path_points = []
    scanline_diagnostics = []
    missing_scanlines = 0
    observed_scanline_count = 0
    ambiguous_scanline_count = 0
    rejected_segment_count = 0

    for y in scanline_ys:
        segments, width_rejected_segments = find_normal_trajectory_segments(
            processed_line_mask,
            envelope,
            y,
        )
        if segments:
            observed_scanline_count += 1
        if len(segments) > 1:
            ambiguous_scanline_count += 1

        selected_segment = None
        near_anchor_missing = False
        if not path_points:
            # O início precisa existir no NEAR. Sem essa âncora, escolher um
            # ramo distante confundiria antecipação com reaquisição de GAP.
            if y < envelope["nearStartY"]:
                near_anchor_missing = True
            elif segments:
                selected_segment = min(
                    segments,
                    key=lambda segment: (
                        abs(float(segment["centerX"]) - seed_reference_x),
                        abs(
                            float(segment["widthPx"])
                            - float(segment["expectedWidthPx"])
                        ),
                    ),
                )
        else:
            selected_segment = select_normal_trajectory_continuation(
                segments,
                path_points,
                y,
                processed_line_mask.shape[1],
            )

        accepted_point = None
        if selected_segment is not None:
            accepted_point = {
                "x": float(selected_segment["centerX"]),
                "y": int(y),
                "widthPx": float(selected_segment["widthPx"]),
                "expectedWidthPx": float(selected_segment["expectedWidthPx"]),
            }

        rejected_segments = [
            normal_trajectory_segment_telemetry(
                segment,
                segment.get("rejectionReason", "width"),
            )
            for segment in width_rejected_segments
        ]
        for segment in segments:
            if segment is selected_segment:
                continue
            if near_anchor_missing:
                rejection_reason = "nearAnchorRequired"
            elif selected_segment is None and path_points:
                rejection_reason = "continuity"
            else:
                rejection_reason = "alternative"
            rejected_segments.append(
                normal_trajectory_segment_telemetry(
                    segment,
                    rejection_reason,
                )
            )

        left_x, right_x = normal_trajectory_horizontal_bounds(envelope, y)
        scanline_diagnostics.append({
            "y": int(y),
            "x0": int(left_x),
            "x1": int(right_x),
            "status": "accepted" if accepted_point is not None else "noPoint",
            "acceptedPoint": (
                {
                    "x": round(float(accepted_point["x"]), 2),
                    "y": int(accepted_point["y"]),
                }
                if accepted_point is not None
                else None
            ),
            "selectedSegment": (
                normal_trajectory_segment_telemetry(selected_segment)
                if selected_segment is not None
                else None
            ),
            "rejectedSegments": rejected_segments,
        })
        rejected_segment_count += len(rejected_segments)

        if near_anchor_missing:
            break

        if selected_segment is None:
            if path_points:
                missing_scanlines += 1
                if missing_scanlines > NORMAL_TRAJECTORY_MAX_MISSING_SCANLINES:
                    break
            continue

        missing_scanlines = 0
        path_points.append(accepted_point)

    available_span_px = max(1.0, float(envelope["nearY"] - envelope["farY"]))
    traced_span_px = (
        float(path_points[0]["y"] - path_points[-1]["y"])
        if len(path_points) >= 2
        else 0.0
    )
    vertical_coverage = max(0.0, min(1.0, traced_span_px / available_span_px))
    valid = bool(
        len(path_points) >= NORMAL_TRAJECTORY_MIN_POINTS
        and vertical_coverage >= NORMAL_TRAJECTORY_MIN_VERTICAL_COVERAGE
    )
    processing_ms = (time.perf_counter() - extraction_started) * 1000.0
    return {
        "enabled": True,
        "valid": valid,
        "coordinateFrame": "processedLineMaskPixels",
        "pointOrder": "nearToFar",
        "rearAxleProjectionCalibrated": False,
        "points": [
            {
                "x": round(float(point["x"]), 2),
                "y": int(point["y"]),
                "widthPx": round(float(point["widthPx"]), 2),
                "expectedWidthPx": round(float(point["expectedWidthPx"]), 2),
            }
            for point in path_points
        ],
        "scanlines": scanline_diagnostics,
        "pointCount": len(path_points),
        "sampledScanlineCount": len(scanline_ys),
        "evaluatedScanlineCount": len(scanline_diagnostics),
        "observedScanlineCount": observed_scanline_count,
        "ambiguousScanlineCount": ambiguous_scanline_count,
        "rejectedSegmentCount": rejected_segment_count,
        "verticalCoverage": round(vertical_coverage, 4),
        "processingMs": max(0.0, float(processing_ms)),
    }


def draw_normal_trajectory_overlay(frame, normal_trajectory):
    """Desenha a trajetória experimental sem alterar nenhuma decisão de controle."""

    if not isinstance(normal_trajectory, dict):
        return

    # As cores permanecem fixas para facilitar a comparação entre capturas:
    # cinza = scanline, vermelho = rejeitado/sem ponto, amarelo = segmento
    # escolhido, verde = ponto aceito e ciano = trajetória final válida.
    scanline_color = (96, 96, 96)
    rejected_color = (0, 0, 255)
    selected_color = (0, 255, 255)
    accepted_color = (0, 255, 0)
    path_color = (
        (255, 255, 0)
        if normal_trajectory.get("valid") is True
        else (0, 180, 255)
    )

    scanlines = normal_trajectory.get("scanlines", [])
    if isinstance(scanlines, list):
        for scanline in scanlines:
            try:
                scanline_y = int(scanline["y"])
                scanline_x0 = int(scanline["x0"])
                scanline_x1 = int(scanline["x1"]) - 1
            except (KeyError, TypeError, ValueError):
                continue
            cv2.line(
                frame,
                (scanline_x0, scanline_y),
                (scanline_x1, scanline_y),
                scanline_color,
                1,
                cv2.LINE_AA,
            )

            rejected_segments = scanline.get("rejectedSegments", [])
            if isinstance(rejected_segments, list):
                for segment in rejected_segments:
                    try:
                        rejected_x0 = int(segment["x0"])
                        rejected_x1 = int(segment["x1"])
                    except (KeyError, TypeError, ValueError):
                        continue
                    cv2.line(
                        frame,
                        (rejected_x0, scanline_y),
                        (rejected_x1, scanline_y),
                        rejected_color,
                        2,
                        cv2.LINE_AA,
                    )

            selected_segment = scanline.get("selectedSegment")
            if isinstance(selected_segment, dict):
                try:
                    selected_x0 = int(selected_segment["x0"])
                    selected_x1 = int(selected_segment["x1"])
                except (KeyError, TypeError, ValueError):
                    selected_segment = None
                if selected_segment is not None:
                    cv2.line(
                        frame,
                        (selected_x0, scanline_y),
                        (selected_x1, scanline_y),
                        selected_color,
                        3,
                        cv2.LINE_AA,
                    )

            accepted_point = scanline.get("acceptedPoint")
            if isinstance(accepted_point, dict):
                try:
                    point = (
                        int(round(float(accepted_point["x"]))),
                        int(round(float(accepted_point["y"]))),
                    )
                except (KeyError, TypeError, ValueError):
                    continue
                cv2.circle(frame, point, 4, (0, 0, 0), -1, cv2.LINE_AA)
                cv2.circle(frame, point, 2, accepted_color, -1, cv2.LINE_AA)
            else:
                # Um X no começo da linha deixa explícito que aquela altura foi
                # avaliada, mas não produziu um ponto para a trajetória.
                marker_x = min(scanline_x1 - 4, scanline_x0 + 7)
                cv2.line(
                    frame,
                    (marker_x - 4, scanline_y - 4),
                    (marker_x + 4, scanline_y + 4),
                    rejected_color,
                    2,
                    cv2.LINE_AA,
                )
                cv2.line(
                    frame,
                    (marker_x - 4, scanline_y + 4),
                    (marker_x + 4, scanline_y - 4),
                    rejected_color,
                    2,
                    cv2.LINE_AA,
                )

    points = normal_trajectory.get("points", [])
    overlay_points = []
    if isinstance(points, list):
        for point in points:
            try:
                point_x = int(round(float(point["x"])))
                point_y = int(round(float(point["y"])))
            except (KeyError, TypeError, ValueError):
                continue
            overlay_points.append((point_x, point_y))
    if len(overlay_points) >= 2:
        cv2.polylines(
            frame,
            [np.asarray(overlay_points, dtype=np.int32)],
            False,
            path_color,
            2,
            cv2.LINE_AA,
        )
    for point in overlay_points:
        cv2.circle(frame, point, 2, accepted_color, -1, cv2.LINE_AA)

    def safe_overlay_number(field_name, default=0.0):
        """Impede que um valor diagnóstico inválido interrompa o stream."""

        try:
            value = float(normal_trajectory.get(field_name, default))
        except (TypeError, ValueError):
            return float(default)
        return value if math.isfinite(value) else float(default)

    metric_texts = (
        f"trajectoryPoints {int(safe_overlay_number('pointCount'))}",
        f"coverage {safe_overlay_number('verticalCoverage'):.3f}",
        f"ambiguities {int(safe_overlay_number('ambiguousScanlineCount'))}",
        f"trajectoryMs {safe_overlay_number('processingMs'):.2f}",
    )
    metrics_x = max(8, frame.shape[1] - 190)
    for line_index, metric_text in enumerate(metric_texts):
        text_origin = (metrics_x, 18 + line_index * 18)
        # O contorno preto mantém os números legíveis sobre piso claro ou fita.
        cv2.putText(
            frame,
            metric_text,
            text_origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            metric_text,
            text_origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            path_color,
            1,
            cv2.LINE_AA,
        )
