"""Detecção, classificação e diagnóstico dos marcadores verdes."""

import json
import math
import os
import time

import cv2  # type: ignore
import numpy as np

from .numeric import (
    DOWNWARD_REFERENCE_FRAME_HEIGHT,
    scaled_odd_kernel_size,
    scaled_reference_pixels,
)


# O array RGB888 do Picamera2 chega na ordem BGR usada pelo OpenCV.
CAMERA_ARRAY_COLOR_ORDER = "BGR"

GREEN_CAPTURE_RAW_PATH = "/dev/shm/obr_green_raw.png"
GREEN_CAPTURE_HSV_MASK_PATH = "/dev/shm/obr_green_hsv_mask.png"
GREEN_CAPTURE_FINAL_MASK_PATH = "/dev/shm/obr_green_final_mask.png"
GREEN_CAPTURE_CANDIDATES_PATH = "/dev/shm/obr_green_candidates.png"
GREEN_CAPTURE_STATS_PATH = "/dev/shm/obr_green_stats.json"
# Mantém compatibilidade para chamadas diagnósticas fora do runtime principal.
DEFAULT_CAPTURE_EXPOSURE_VALUE = 0.4

# O branco da pista sob iluminação esverdeada chegou à saturação 136.
# Exigir 140 preserva o cartão verde e bloqueia esse falso positivo.
GREEN_HUE_MIN = 40
GREEN_HUE_MAX = 90
GREEN_SATURATION_MIN = 140
GREEN_VALUE_MIN = 60
GREEN_OPEN_KERNEL_SIZE = 5
GREEN_CLOSE_KERNEL_SIZE = 5
GREEN_OPEN_ITERATIONS = 2
GREEN_CLOSE_ITERATIONS = 2

# Estes limites aceitam o marcador oficial observado, enquanto HSV, formato e
# associação com a faixa continuam bloqueando ruídos verdes.
GREEN_MIN_AREA_RATIO = 2600.0 / (320.0 * 200.0)
GREEN_MIN_AREA_PX = 80.0
GREEN_MIN_DIMENSION_PX = 6.0
GREEN_ASPECT_RATIO_MIN = 0.35
GREEN_ASPECT_RATIO_MAX = 1.0
GREEN_MIN_EXTENT = 0.35
GREEN_PARTIAL_BORDER_TOLERANCE_PX = 4
GREEN_PARTIAL_AREA_FACTOR = 0.40
GREEN_PARTIAL_DIMENSION_FACTOR = 0.50
GREEN_PARTIAL_ASPECT_RATIO_MIN = 0.20
GREEN_PARTIAL_EXTENT_MIN = 0.20
GREEN_FRAGMENT_MERGE_DISTANCE_PX = 12

# As ROIs relacionam cada marcador verde com a faixa preta ao redor.
GREEN_ROI_HALF_SIZE_DIVISOR = 20
GREEN_ROI_MIN_VISIBLE_RATIO = 0.50
GREEN_ROI_MIN_BLACK_RATIO = 0.25
GREEN_PAIR_MAX_VERTICAL_DISTANCE_HEIGHTS = 1.5

GREEN_OBSERVATION_STATES = {
    "SEM_VERDE",
    "UM_CANDIDATO",
    "DOIS_CANDIDATOS",
    "MULTIPLOS_AMBIGUOS",
}
GREEN_INTERPRETATIONS = {
    "SEM_DECISAO",
    "ESQUERDA",
    "DIREITA",
    "RETORNO_180",
    "VERDE_FALSO",
    "AMBIGUO",
}
VISIBLE_GREEN_INTERPRETATIONS = {
    "ESQUERDA",
    "DIREITA",
    "RETORNO_180",
}


def frame_to_hsv(frame, camera_format="RGB888"):
    """Converte o array da câmera para HSV respeitando a ordem real dos canais."""

    if camera_format != "RGB888" or CAMERA_ARRAY_COLOR_ORDER != "BGR":
        raise ValueError("Formato ou ordem de canais não suportados para HSV.")
    return cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


def rgb_pixel_to_camera_array(red, green, blue):
    """Representa uma cor RGB na ordem BGR entregue por capture_array."""

    if CAMERA_ARRAY_COLOR_ORDER != "BGR":
        raise ValueError("Ordem de canais inesperada.")
    return int(blue), int(green), int(red)


def is_hsv_green(hue, saturation, value):
    """Aplica aos pixels sintéticos os mesmos limites usados por cv2.inRange."""

    return (
        GREEN_HUE_MIN <= int(hue) <= GREEN_HUE_MAX
        and int(saturation) >= GREEN_SATURATION_MIN
        and int(value) >= GREEN_VALUE_MIN
    )


def create_green_mask(
    frame,
    green_end_y,
    camera_format="RGB888",
    green_start_y=0,
):
    """Segmenta verde entre a zona morta superior e o limite verde inferior."""

    useful_end_y = max(0, min(frame.shape[0], int(green_end_y)))
    useful_start_y = max(0, min(useful_end_y, int(green_start_y)))
    green_mask = np.zeros(
        (useful_end_y, frame.shape[1]),
        dtype=np.uint8,
    )
    if useful_start_y >= useful_end_y:
        return green_mask

    useful_frame = frame[useful_start_y:useful_end_y, :]
    hsv_frame = frame_to_hsv(useful_frame, camera_format)
    useful_green_mask = cv2.inRange(
        hsv_frame,
        (GREEN_HUE_MIN, GREEN_SATURATION_MIN, GREEN_VALUE_MIN),
        (GREEN_HUE_MAX, 255, 255),
    )
    if cv2.countNonZero(useful_green_mask) == 0:
        return green_mask
    green_kernel_size = scaled_odd_kernel_size(
        GREEN_OPEN_KERNEL_SIZE, frame.shape[0]
    )
    open_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (green_kernel_size, green_kernel_size),
    )
    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (green_kernel_size, green_kernel_size),
    )
    useful_green_mask = cv2.morphologyEx(
        useful_green_mask,
        cv2.MORPH_OPEN,
        open_kernel,
        iterations=GREEN_OPEN_ITERATIONS,
    )
    useful_green_mask = cv2.morphologyEx(
        useful_green_mask,
        cv2.MORPH_CLOSE,
        close_kernel,
        iterations=GREEN_CLOSE_ITERATIONS,
    )
    green_mask[useful_start_y:useful_end_y, :] = useful_green_mask
    return green_mask


def create_green_mask_stages(
    frame,
    green_end_y,
    camera_format="RGB888",
    green_start_y=0,
):
    """Expõe as etapas da segmentação dentro dos mesmos limites verticais."""

    useful_end_y = max(0, min(frame.shape[0], int(green_end_y)))
    useful_start_y = max(0, min(useful_end_y, int(green_start_y)))
    hsv_frame = np.zeros(
        (useful_end_y, frame.shape[1], 3),
        dtype=np.uint8,
    )
    empty_mask = np.zeros(
        (useful_end_y, frame.shape[1]),
        dtype=np.uint8,
    )
    if useful_start_y >= useful_end_y:
        return (
            hsv_frame,
            empty_mask,
            empty_mask.copy(),
            empty_mask.copy(),
            empty_mask.copy(),
        )

    useful_frame = frame[useful_start_y:useful_end_y, :]
    useful_hsv_frame = frame_to_hsv(useful_frame, camera_format)
    useful_hue_mask = cv2.inRange(
        useful_hsv_frame,
        (GREEN_HUE_MIN, 0, 0),
        (GREEN_HUE_MAX, 255, 255),
    )
    useful_hue_saturation_mask = cv2.inRange(
        useful_hsv_frame,
        (GREEN_HUE_MIN, GREEN_SATURATION_MIN, 0),
        (GREEN_HUE_MAX, 255, 255),
    )
    useful_hsv_mask = cv2.inRange(
        useful_hsv_frame,
        (GREEN_HUE_MIN, GREEN_SATURATION_MIN, GREEN_VALUE_MIN),
        (GREEN_HUE_MAX, 255, 255),
    )
    useful_final_mask = useful_hsv_mask.copy()
    if cv2.countNonZero(useful_hsv_mask) > 0:
        green_kernel_size = scaled_odd_kernel_size(
            GREEN_OPEN_KERNEL_SIZE, frame.shape[0]
        )
        open_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (green_kernel_size, green_kernel_size),
        )
        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (green_kernel_size, green_kernel_size),
        )
        useful_final_mask = cv2.morphologyEx(
            useful_final_mask,
            cv2.MORPH_OPEN,
            open_kernel,
            iterations=GREEN_OPEN_ITERATIONS,
        )
        useful_final_mask = cv2.morphologyEx(
            useful_final_mask,
            cv2.MORPH_CLOSE,
            close_kernel,
            iterations=GREEN_CLOSE_ITERATIONS,
        )
    hue_mask = empty_mask.copy()
    hue_saturation_mask = empty_mask.copy()
    hsv_mask = empty_mask.copy()
    final_mask = empty_mask.copy()
    active_rows = slice(useful_start_y, useful_end_y)
    hsv_frame[active_rows, :] = useful_hsv_frame
    hue_mask[active_rows, :] = useful_hue_mask
    hue_saturation_mask[active_rows, :] = useful_hue_saturation_mask
    hsv_mask[active_rows, :] = useful_hsv_mask
    final_mask[active_rows, :] = useful_final_mask
    return hsv_frame, hue_mask, hue_saturation_mask, hsv_mask, final_mask


def expanded_boxes_overlap(first_box, second_box, distance_px):
    """Indica se dois fragmentos podem pertencer à mesma marcação verde."""

    first_x, first_y, first_width, first_height = first_box
    second_x, second_y, second_width, second_height = second_box
    return not (
        first_x + first_width + distance_px < second_x
        or second_x + second_width + distance_px < first_x
        or first_y + first_height + distance_px < second_y
        or second_y + second_height + distance_px < first_y
    )


def group_fragment_boxes(
    boxes,
    distance_px=GREEN_FRAGMENT_MERGE_DISTANCE_PX,
):
    """Agrupa caixas próximas de forma transitiva antes de unir os contornos."""

    groups = []
    for box_index, box in enumerate(boxes):
        matching_groups = []
        for group_index, group in enumerate(groups):
            if any(
                expanded_boxes_overlap(box, boxes[index], distance_px)
                for index in group
            ):
                matching_groups.append(group_index)
        merged_group = [box_index]
        for group_index in reversed(matching_groups):
            merged_group.extend(groups.pop(group_index))
        groups.append(merged_group)
    return groups


def merge_green_fragments(contours, frame_height=DOWNWARD_REFERENCE_FRAME_HEIGHT):
    """Une fragmentos próximos para que um quadrado não seja contado duas vezes."""

    valid_contours = [
        contour
        for contour in contours
        if contour is not None and len(contour) >= 3
    ]
    boxes = [cv2.boundingRect(contour) for contour in valid_contours]
    groups = group_fragment_boxes(
        boxes,
        distance_px=scaled_reference_pixels(
            GREEN_FRAGMENT_MERGE_DISTANCE_PX, frame_height
        ),
    )
    return [
        cv2.convexHull(np.concatenate(
            [valid_contours[index] for index in group], axis=0
        ))
        for group in groups
    ]


def green_geometry_is_valid(
    area,
    short_side,
    aspect_ratio,
    extent,
    partial,
    frame_height=DOWNWARD_REFERENCE_FRAME_HEIGHT,
):
    """Aplica filtros amplos de tamanho e formato ao marcador oficial."""

    dimension_scale = float(frame_height) / DOWNWARD_REFERENCE_FRAME_HEIGHT
    minimum_area = GREEN_MIN_AREA_PX * dimension_scale * dimension_scale
    minimum_dimension = scaled_reference_pixels(
        GREEN_MIN_DIMENSION_PX, frame_height
    )
    minimum_aspect = GREEN_ASPECT_RATIO_MIN
    minimum_extent = GREEN_MIN_EXTENT
    if partial:
        minimum_area *= GREEN_PARTIAL_AREA_FACTOR
        minimum_dimension *= GREEN_PARTIAL_DIMENSION_FACTOR
        minimum_aspect = GREEN_PARTIAL_ASPECT_RATIO_MIN
        minimum_extent = GREEN_PARTIAL_EXTENT_MIN
    return (
        math.isfinite(float(area))
        and float(area) >= minimum_area
        and float(short_side) >= minimum_dimension
        and minimum_aspect <= float(aspect_ratio) <= GREEN_ASPECT_RATIO_MAX
        and float(extent) >= minimum_extent
    )


def contour_touches_useful_border(
    box,
    frame_width,
    useful_height,
    frame_height=DOWNWARD_REFERENCE_FRAME_HEIGHT,
):
    """Marca candidatos parciais próximos de qualquer limite da área útil."""

    x, y, width, height = box
    tolerance = scaled_reference_pixels(
        GREEN_PARTIAL_BORDER_TOLERANCE_PX, frame_height
    )
    return (
        x <= tolerance
        or y <= tolerance
        or x + width >= frame_width - tolerance
        or y + height >= useful_height - tolerance
    )


def describe_green_contour(
    contour,
    frame_width,
    useful_height,
    frame_height=DOWNWARD_REFERENCE_FRAME_HEIGHT,
):
    """Calcula a geometria local do candidato sem referência de controle."""

    area = float(cv2.contourArea(contour))
    box = cv2.boundingRect(contour)
    rotated_rect = cv2.minAreaRect(contour)
    rect_width, rect_height = rotated_rect[1]
    short_side = min(float(rect_width), float(rect_height))
    long_side = max(float(rect_width), float(rect_height))
    aspect_ratio = short_side / long_side if long_side > 0.0 else 0.0
    box_area = float(box[2] * box[3])
    extent = area / box_area if box_area > 0.0 else 0.0
    moments = cv2.moments(contour)
    if moments["m00"] > 0.0:
        center_x = float(moments["m10"] / moments["m00"])
        center_y = float(moments["m01"] / moments["m00"])
    else:
        center_x = float(box[0] + box[2] / 2.0)
        center_y = float(box[1] + box[3] / 2.0)
    centroid = (center_x, center_y)
    partial = contour_touches_useful_border(
        box, frame_width, useful_height, frame_height
    )

    geometry_valid = green_geometry_is_valid(
        area,
        short_side,
        aspect_ratio,
        extent,
        partial,
        frame_height,
    )

    return {
        "contour": contour,
        "area": area,
        "centroid": centroid,
        "bounding_box": box,
        "rotated_rect": rotated_rect,
        "short_side": short_side,
        "long_side": long_side,
        "aspect_ratio": aspect_ratio,
        "extent": extent,
        "partial": partial,
        "geometry_valid": geometry_valid,
        "frameHeight": int(frame_height),
    }


def find_green_candidates(
    frame,
    green_end_y,
    camera_format="RGB888",
    green_start_y=0,
    timings=None,
):
    """Segmenta e separa candidatos geométricos de ruídos verdes rejeitados."""

    mask_started = time.perf_counter() if timings is not None else 0.0
    green_mask = create_green_mask(
        frame,
        green_end_y,
        camera_format,
        green_start_y,
    )
    if timings is not None:
        timings["green_mask_ms"] = (
            time.perf_counter() - mask_started
        ) * 1000.0
    contours_started = time.perf_counter() if timings is not None else 0.0
    contours, _ = cv2.findContours(
        green_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    merged_contours = merge_green_fragments(contours, frame.shape[0])
    candidates = []
    rejected = []
    scaled_minimum_area = green_minimum_area(
        frame.shape[1], green_mask.shape[0]
    )
    for contour in merged_contours:
        description = describe_green_contour(
            contour,
            frame.shape[1],
            green_mask.shape[0],
            frame.shape[0],
        )
        description["scaled_minimum_area"] = scaled_minimum_area
        description["scaled_area_valid"] = (
            description["area"] > scaled_minimum_area
        )
        # O teste estrito preserva o comportamento original de area > 4000.
        if (
            not description["scaled_area_valid"]
            or not description["geometry_valid"]
        ):
            rejected.append(description)
            continue
        candidates.append(description)
    candidates.sort(key=lambda candidate: candidate["area"], reverse=True)
    if timings is not None:
        timings["green_contours_ms"] = (
            time.perf_counter() - contours_started
        ) * 1000.0
    return green_mask, candidates, rejected


def green_minimum_area(frame_width, useful_height):
    """Escala os 2.600 px de referência pela área útil da resolução atual."""

    return float(frame_width) * float(useful_height) * GREEN_MIN_AREA_RATIO


def camera_array_rgb_channels(frame):
    """Nomeia os canais segundo a ordem que o detector realmente interpreta."""

    if CAMERA_ARRAY_COLOR_ORDER != "BGR":
        raise ValueError("Ordem de canais inesperada para o diagnóstico verde.")
    blue = frame[:, :, 0]
    green = frame[:, :, 1]
    red = frame[:, :, 2]
    return red, green, blue


def channel_percentiles(channel):
    """Resume um canal da região útil com percentis comparáveis entre ensaios."""

    values = np.percentile(channel, (1, 5, 50, 95, 99))
    return {
        "p01": float(values[0]),
        "p05": float(values[1]),
        "p50": float(values[2]),
        "p95": float(values[3]),
        "p99": float(values[4]),
    }


def mask_active_percent(mask):
    """Calcula a fração ativa da máscara em porcentagem da região útil."""

    return (
        100.0 * float(cv2.countNonZero(mask)) / float(mask.size)
        if mask.size > 0
        else 0.0
    )


def green_geometry_rejection_reasons(candidate):
    """Detalha quais filtros geométricos vigentes rejeitaram um componente."""

    partial = bool(candidate["partial"])
    frame_height = candidate.get(
        "frameHeight", DOWNWARD_REFERENCE_FRAME_HEIGHT
    )
    dimension_scale = float(frame_height) / DOWNWARD_REFERENCE_FRAME_HEIGHT
    minimum_area = GREEN_MIN_AREA_PX * dimension_scale * dimension_scale
    minimum_dimension = scaled_reference_pixels(
        GREEN_MIN_DIMENSION_PX, frame_height
    )
    minimum_aspect = GREEN_ASPECT_RATIO_MIN
    minimum_extent = GREEN_MIN_EXTENT
    if partial:
        minimum_area *= GREEN_PARTIAL_AREA_FACTOR
        minimum_dimension *= GREEN_PARTIAL_DIMENSION_FACTOR
        minimum_aspect = GREEN_PARTIAL_ASPECT_RATIO_MIN
        minimum_extent = GREEN_PARTIAL_EXTENT_MIN

    reasons = []
    if not candidate.get("scaled_area_valid", True):
        reasons.append("area_below_scaled_reference")
    if not math.isfinite(candidate["area"]) or candidate["area"] < minimum_area:
        reasons.append("area_below_minimum")
    if candidate["short_side"] < minimum_dimension:
        reasons.append("dimension_below_minimum")
    if not minimum_aspect <= candidate["aspect_ratio"] <= GREEN_ASPECT_RATIO_MAX:
        reasons.append("aspect_ratio_outside_range")
    if candidate["extent"] < minimum_extent:
        reasons.append("extent_below_minimum")
    return reasons


def json_safe_camera_metadata(metadata):
    """Seleciona apenas metadados necessários e converte valores para JSON."""

    def safe_value(value):
        if value is None or isinstance(value, (str, bool, int)):
            return value
        if isinstance(value, float):
            return value if math.isfinite(value) else None
        if isinstance(value, (tuple, list)):
            return [safe_value(item) for item in value]
        try:
            converted = float(value)
            return converted if math.isfinite(converted) else None
        except (TypeError, ValueError):
            return str(value)

    names = (
        "ExposureTime",
        "AnalogueGain",
        "ColourGains",
        "ColourTemperature",
        "AwbEnable",
        "AeEnable",
    )
    return {name: safe_value(metadata.get(name)) for name in names}


def component_pixel_statistics(candidate, useful_frame, hsv_frame, final_mask):
    """Mede cor somente nos pixels finais pertencentes ao componente."""

    component_mask = np.zeros(final_mask.shape, dtype=np.uint8)
    cv2.drawContours(component_mask, [candidate["contour"]], -1, 255, -1)
    active_pixels = (component_mask > 0) & (final_mask > 0)
    red, green, blue = camera_array_rgb_channels(useful_frame)
    hue = hsv_frame[:, :, 0][active_pixels]
    saturation = hsv_frame[:, :, 1][active_pixels]
    value = hsv_frame[:, :, 2][active_pixels]
    red_values = red[active_pixels]
    green_values = green[active_pixels]
    blue_values = blue[active_pixels]
    dominance = green_values.astype(np.int16) - np.maximum(
        red_values, blue_values
    ).astype(np.int16)

    def median(values):
        return float(np.median(values)) if values.size else 0.0

    def range_summary(values):
        if not values.size:
            return {"min": 0.0, "median": 0.0, "max": 0.0}
        return {
            "min": float(np.min(values)),
            "median": median(values),
            "max": float(np.max(values)),
        }

    return {
        "sampled_pixel_count": int(np.count_nonzero(active_pixels)),
        "hue_median": median(hue),
        "saturation": range_summary(saturation),
        "value": range_summary(value),
        "rgb_medians": {
            "red": median(red_values),
            "green": median(green_values),
            "blue": median(blue_values),
        },
        "green_dominance_median": median(dominance),
    }


def build_green_capture_stats(
    frame,
    camera_format,
    mask_stages,
    candidates,
    rejected,
    interpretation,
    camera_metadata,
    camera_exposure_value=DEFAULT_CAPTURE_EXPOSURE_VALUE,
):
    """Monta a evidência one-shot do pipeline antes de desenhar o overlay."""

    hsv_frame, hue_mask, hue_saturation_mask, hsv_mask, final_mask = mask_stages
    useful_frame = frame[:hsv_frame.shape[0], :]
    red, green, blue = camera_array_rgb_channels(useful_frame)
    dominance = green.astype(np.int16) - np.maximum(red, blue).astype(np.int16)
    hsv_contours, _ = cv2.findContours(
        hsv_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    final_contours, _ = cv2.findContours(
        final_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    all_components = [
        (candidate, True) for candidate in candidates
    ] + [
        (candidate, False) for candidate in rejected
    ]
    component_stats = []
    for component_id, (candidate, accepted) in enumerate(all_components, start=1):
        geometry_reasons = green_geometry_rejection_reasons(candidate)
        reason = "geometria válida" if accepted else ", ".join(geometry_reasons)
        hull_area = float(cv2.contourArea(cv2.convexHull(candidate["contour"])))
        component_stats.append({
            "id": component_id,
            "area": float(candidate["area"]),
            "bbox": [int(value) for value in candidate["bounding_box"]],
            "centroid": [float(value) for value in candidate["centroid"]],
            "partial": bool(candidate["partial"]),
            "aspect_ratio": float(candidate["aspect_ratio"]),
            "extent": float(candidate["extent"]),
            "solidity": (
                float(candidate["area"] / hull_area) if hull_area > 0.0 else 0.0
            ),
            "geometry_valid": bool(candidate["geometry_valid"]),
            "accepted": accepted,
            "rejected": not accepted,
            "reason": reason,
            **component_pixel_statistics(
                candidate, useful_frame, hsv_frame, final_mask
            ),
        })

    return {
        "captured_at_unix_seconds": time.time(),
        "detector_region": {
            "x": 0,
            "y": 0,
            "width": int(useful_frame.shape[1]),
            "height": int(useful_frame.shape[0]),
        },
        "camera": {
            "requested_format": camera_format,
            "detector_array_order": CAMERA_ARRAY_COLOR_ORDER,
            "requested_controls": {
                "AwbEnable": True,
                "AeEnable": True,
                "ExposureValue": float(camera_exposure_value),
            },
            "metadata": json_safe_camera_metadata(camera_metadata),
        },
        "thresholds": {
            "hue": [GREEN_HUE_MIN, GREEN_HUE_MAX],
            "saturation_min": GREEN_SATURATION_MIN,
            "value_min": GREEN_VALUE_MIN,
            "open_kernel": [GREEN_OPEN_KERNEL_SIZE, GREEN_OPEN_KERNEL_SIZE],
            "close_kernel": [GREEN_CLOSE_KERNEL_SIZE, GREEN_CLOSE_KERNEL_SIZE],
            "minimum_area_px": GREEN_MIN_AREA_PX,
            "minimum_dimension_px": GREEN_MIN_DIMENSION_PX,
            "aspect_ratio": [GREEN_ASPECT_RATIO_MIN, GREEN_ASPECT_RATIO_MAX],
            "minimum_extent": GREEN_MIN_EXTENT,
            "solidity_filter_enabled": False,
            "fragment_merge_distance_px": GREEN_FRAGMENT_MERGE_DISTANCE_PX,
        },
        "frame_percentiles": {
            "array_channels": {
                "channel_0": channel_percentiles(useful_frame[:, :, 0]),
                "channel_1": channel_percentiles(useful_frame[:, :, 1]),
                "channel_2": channel_percentiles(useful_frame[:, :, 2]),
            },
            "rgb": {
                "red": channel_percentiles(red),
                "green": channel_percentiles(green),
                "blue": channel_percentiles(blue),
            },
            "hsv": {
                "hue": channel_percentiles(hsv_frame[:, :, 0]),
                "saturation": channel_percentiles(hsv_frame[:, :, 1]),
                "value": channel_percentiles(hsv_frame[:, :, 2]),
            },
        },
        "mean_green_dominance": float(np.mean(dominance)),
        "mask_active_percent": {
            "hue_only": mask_active_percent(hue_mask),
            "hue_and_saturation": mask_active_percent(hue_saturation_mask),
            "full_hsv": mask_active_percent(hsv_mask),
            "after_morphology": mask_active_percent(final_mask),
        },
        "mask_active_pixels": {
            "hue_only": int(cv2.countNonZero(hue_mask)),
            "hue_and_saturation": int(cv2.countNonZero(hue_saturation_mask)),
            "full_hsv": int(cv2.countNonZero(hsv_mask)),
            "after_morphology": int(cv2.countNonZero(final_mask)),
        },
        "pipeline_counts": {
            "hsv_mask_external_contours": len(hsv_contours),
            "final_mask_external_contours": len(final_contours),
            "accepted_candidates": len(candidates),
            "rejected_candidates": len(rejected),
            "detector_components_after_merge": len(all_components),
        },
        "interpretation": interpretation.get("interpretation", "SEM_DECISAO"),
        "components": component_stats,
    }


def save_green_capture(
    frame,
    camera_format,
    mask_stages,
    candidates,
    rejected,
    interpretation,
    camera_metadata,
    camera_exposure_value=DEFAULT_CAPTURE_EXPOSURE_VALUE,
):
    """Grava uma captura diagnóstica completa por substituições atômicas."""

    _hsv_frame, _hue_mask, _hue_saturation_mask, hsv_mask, final_mask = (
        mask_stages
    )
    candidates_image = frame.copy()
    for component_id, candidate in enumerate(candidates, start=1):
        cv2.drawContours(
            candidates_image, [candidate["contour"]], -1, (0, 255, 0), 2
        )
        center = tuple(int(round(value)) for value in candidate["centroid"])
        cv2.putText(
            candidates_image, str(component_id), center,
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA,
        )
    rejected_id_start = len(candidates) + 1
    for offset, candidate in enumerate(rejected):
        component_id = rejected_id_start + offset
        cv2.drawContours(
            candidates_image, [candidate["contour"]], -1, (180, 80, 180), 1
        )
        center = tuple(int(round(value)) for value in candidate["centroid"])
        cv2.putText(
            candidates_image, str(component_id), center,
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 80, 180), 1, cv2.LINE_AA,
        )

    stats = build_green_capture_stats(
        frame,
        camera_format,
        mask_stages,
        candidates,
        rejected,
        interpretation,
        camera_metadata,
        camera_exposure_value,
    )
    image_outputs = (
        (GREEN_CAPTURE_RAW_PATH, "/dev/shm/.obr_green_raw.tmp.png", frame),
        (
            GREEN_CAPTURE_HSV_MASK_PATH,
            "/dev/shm/.obr_green_hsv_mask.tmp.png",
            hsv_mask,
        ),
        (
            GREEN_CAPTURE_FINAL_MASK_PATH,
            "/dev/shm/.obr_green_final_mask.tmp.png",
            final_mask,
        ),
        (
            GREEN_CAPTURE_CANDIDATES_PATH,
            "/dev/shm/.obr_green_candidates.tmp.png",
            candidates_image,
        ),
    )
    temporary_paths = [temporary for _target, temporary, _image in image_outputs]
    temporary_paths.append("/dev/shm/.obr_green_stats.tmp.json")
    try:
        for _target, temporary, image in image_outputs:
            if not cv2.imwrite(temporary, image):
                raise OSError(f"Não foi possível gravar {temporary}.")
        with open(temporary_paths[-1], "w", encoding="utf-8") as stats_file:
            json.dump(stats, stats_file, indent=2, allow_nan=False)
        for target, temporary, _image in image_outputs:
            os.replace(temporary, target)
        os.replace(temporary_paths[-1], GREEN_CAPTURE_STATS_PATH)
    finally:
        for temporary in temporary_paths:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def green_observation_state(candidate_count):
    """Converte a quantidade de marcações geométricas no estado de observação."""

    if candidate_count <= 0:
        return "SEM_VERDE"
    if candidate_count == 1:
        return "UM_CANDIDATO"
    if candidate_count == 2:
        return "DOIS_CANDIDATOS"
    return "MULTIPLOS_AMBIGUOS"


def empty_green_status():
    """Cria um estado verde finito e seguro para publicação diagnóstica."""

    return {
        "greenObservationState": "SEM_VERDE",
        "greenInterpretation": "SEM_DECISAO",
        "greenConfirmed": False,
        "greenRawInterpretation": "SEM_DECISAO",
        "greenDecisionState": "idle",
        "greenPathBlackValid": False,
        "greenCandidateCount": 0,
        "greenRejectedCount": 0,
        "greenMarkerCount": 0,
        "greenValidatedMarkerCount": 0,
        "greenFrontRoiMeasured": False,
        "greenFrontBlackRatio": 0.0,
        "greenFrontRoiValid": False,
        "greenLeftRoiMeasured": False,
        "greenLeftBlackRatio": 0.0,
        "greenLeftRoiValid": False,
        "greenRightRoiMeasured": False,
        "greenRightBlackRatio": 0.0,
        "greenRightRoiValid": False,
        "greenLeftSeen": False,
        "greenRightSeen": False,
        "greenPairCompatible": False,
        "greenPrimaryX": 0.0,
        "greenPrimaryY": 0.0,
        "greenPrimaryArea": 0.0,
        "greenSecondaryX": 0.0,
        "greenSecondaryY": 0.0,
        "greenSecondaryArea": 0.0,
        "greenConsecutiveSamples": 0,
        "greenProcessingMs": 0.0,
    }


def green_marker_roi_geometry(contour, frame_width):
    """Calcula as ROIs horizontal e superior ao redor do marcador verde."""

    box = cv2.boxPoints(cv2.minAreaRect(contour))
    minimum_x = float(np.min(box[:, 0]))
    maximum_x = float(np.max(box[:, 0]))
    minimum_y = float(np.min(box[:, 1]))
    maximum_y = float(np.max(box[:, 1]))
    center_x = int(round((minimum_x + maximum_x) / 2.0))
    center_y = int(round((minimum_y + maximum_y) / 2.0))
    half_size = max(1, int(frame_width) // GREEN_ROI_HALF_SIZE_DIVISOR)

    # A ROI horizontal cruza o marcador e mede separadamente somente as partes
    # externas à esquerda e à direita. Assim existe uma única ROI lateral.
    horizontal_roi = (
        int(round(minimum_x)) - 2 * half_size,
        center_y - half_size,
        int(round(maximum_x)) + 2 * half_size,
        center_y + half_size,
    )
    # A ROI superior é perpendicular à horizontal. Ela deve ter amostra
    # visível antes que o marcador possa ser chamado de verdadeiro ou falso.
    upper_roi = (
        center_x - half_size,
        int(round(minimum_y)) - 3 * half_size,
        center_x + half_size,
        int(round(minimum_y)),
    )
    return {
        "box": box,
        "center": (center_x, center_y),
        "marker_horizontal_bounds": (
            int(round(minimum_x)),
            int(round(maximum_x)),
        ),
        "horizontal_roi": horizontal_roi,
        "upper_roi": upper_roi,
    }


def measure_black_roi(black_mask, roi):
    """Mede preto somente quando ao menos metade da ROI permanece na imagem."""

    x1, y1, x2, y2 = (int(value) for value in roi)
    nominal_width = max(0, x2 - x1)
    nominal_height = max(0, y2 - y1)
    nominal_area = nominal_width * nominal_height
    if black_mask is None or black_mask.size == 0 or nominal_area <= 0:
        return {
            "measured": False,
            "valid": False,
            "black_ratio": 0.0,
            "visible_ratio": 0.0,
        }

    clipped_x1 = max(0, min(black_mask.shape[1], x1))
    clipped_y1 = max(0, min(black_mask.shape[0], y1))
    clipped_x2 = max(0, min(black_mask.shape[1], x2))
    clipped_y2 = max(0, min(black_mask.shape[0], y2))
    visible_width = max(0, clipped_x2 - clipped_x1)
    visible_height = max(0, clipped_y2 - clipped_y1)
    visible_area = visible_width * visible_height
    visible_ratio = visible_area / nominal_area
    if visible_area <= 0 or visible_ratio < GREEN_ROI_MIN_VISIBLE_RATIO:
        return {
            "measured": False,
            "valid": False,
            "black_ratio": 0.0,
            "visible_ratio": visible_ratio,
        }

    region = black_mask[clipped_y1:clipped_y2, clipped_x1:clipped_x2]
    black_ratio = float(np.count_nonzero(region > 0)) / visible_area
    return {
        "measured": True,
        "valid": black_ratio >= GREEN_ROI_MIN_BLACK_RATIO,
        "black_ratio": black_ratio,
        "visible_ratio": visible_ratio,
    }


def measure_horizontal_black_roi(black_mask, geometry):
    """Mede os dois lados dentro da única ROI horizontal."""

    x1, y1, x2, y2 = geometry["horizontal_roi"]
    marker_left_x, marker_right_x = geometry["marker_horizontal_bounds"]
    left = measure_black_roi(
        black_mask,
        (x1, y1, marker_left_x, y2),
    )
    right = measure_black_roi(
        black_mask,
        (marker_right_x, y1, x2, y2),
    )
    return {
        "measured": left["measured"] and right["measured"],
        "valid": left["valid"] or right["valid"],
        "left_measured": left["measured"],
        "left_valid": left["valid"],
        "left_black_ratio": left["black_ratio"],
        "right_measured": right["measured"],
        "right_valid": right["valid"],
        "right_black_ratio": right["black_ratio"],
    }


def analyze_green_marker_contours(green_contours, selected_black_mask):
    """Classifica o verde pelas ROIs horizontal e superior."""

    result = {
        "interpretation": "SEM_DECISAO",
        "observation_state": "SEM_VERDE",
        "left_seen": False,
        "right_seen": False,
        "pair_compatible": False,
        "path_black_valid": False,
        "markers": [],
    }
    if selected_black_mask is None or selected_black_mask.size == 0:
        return result

    upper_valid_markers = []
    upper_measurement_missing = False
    for contour in green_contours:
        geometry = green_marker_roi_geometry(
            contour,
            selected_black_mask.shape[1],
        )
        upper_measurement = measure_black_roi(
            selected_black_mask,
            geometry["upper_roi"],
        )
        marker = {
            "contour": contour,
            "geometry": geometry,
            "upper": upper_measurement,
            "horizontal": measure_horizontal_black_roi(
                selected_black_mask,
                geometry,
            ),
        }
        result["markers"].append(marker)
        if upper_measurement["valid"]:
            upper_valid_markers.append(marker)
        elif not upper_measurement["measured"]:
            upper_measurement_missing = True

    result["observation_state"] = green_observation_state(len(green_contours))
    if green_contours and upper_measurement_missing:
        # Nenhuma classificação verdadeira ou falsa é publicada enquanto
        # uma ROI superior ainda não possui área visível suficiente.
        result["interpretation"] = "AMBIGUO"
        return result
    if not upper_valid_markers:
        if green_contours:
            result["interpretation"] = "VERDE_FALSO"
        return result

    for marker in upper_valid_markers:
        horizontal = marker["horizontal"]
        left_valid = horizontal["left_valid"]
        right_valid = horizontal["right_valid"]
        marker["interpretation"] = (
            "AMBIGUO" if not horizontal["measured"] else
            "DIREITA" if left_valid and not right_valid else
            "ESQUERDA" if right_valid and not left_valid else
            "AMBIGUO"
        )

    if len(upper_valid_markers) > 2:
        result["interpretation"] = "AMBIGUO"
        return result

    if len(upper_valid_markers) == 2:
        first, second = upper_valid_markers
        first_center = np.mean(first["geometry"]["box"], axis=0)
        second_center = np.mean(second["geometry"]["box"], axis=0)
        first_height = float(np.ptp(first["geometry"]["box"][:, 1]))
        second_height = float(np.ptp(second["geometry"]["box"][:, 1]))
        vertical_tolerance = (
            GREEN_PAIR_MAX_VERTICAL_DISTANCE_HEIGHTS
            * max(1.0, first_height, second_height)
        )
        directions = {
            first["interpretation"],
            second["interpretation"],
        }
        pair_compatible = (
            directions == {"ESQUERDA", "DIREITA"}
            and abs(float(first_center[1] - second_center[1]))
            <= vertical_tolerance
        )
        result["left_seen"] = "ESQUERDA" in directions
        result["right_seen"] = "DIREITA" in directions
        result["pair_compatible"] = pair_compatible
        if not pair_compatible:
            result["interpretation"] = "AMBIGUO"
            return result
        result.update({
            "interpretation": "RETORNO_180",
            "left_seen": True,
            "right_seen": True,
            "pair_compatible": True,
            "path_black_valid": True,
        })
        return result

    marker = upper_valid_markers[0]
    interpretation = marker["interpretation"]
    result["interpretation"] = interpretation
    result["left_seen"] = interpretation == "ESQUERDA"
    result["right_seen"] = interpretation == "DIREITA"
    result["path_black_valid"] = interpretation in (
        "ESQUERDA",
        "DIREITA",
    )
    return result


def build_green_status(
    candidates,
    rejected_count,
    interpretation_result,
    tracker_result,
    processing_ms,
):
    """Monta os campos diagnósticos sem permitir NaN no JSON rápido."""

    status = empty_green_status()
    published_interpretation, confirmed, consecutive_samples = tracker_result
    status.update({
        "greenObservationState": interpretation_result["observation_state"],
        "greenInterpretation": published_interpretation,
        "greenConfirmed": confirmed,
        "greenCandidateCount": len(candidates),
        "greenRejectedCount": int(rejected_count),
        "greenLeftSeen": interpretation_result["left_seen"],
        "greenRightSeen": interpretation_result["right_seen"],
        "greenPairCompatible": interpretation_result["pair_compatible"],
        "greenConsecutiveSamples": int(consecutive_samples),
        "greenProcessingMs": float(processing_ms),
    })

    # A ROI superior valida a associação com a faixa. Os campos laterais
    # preservam o IPC, mas agora descrevem as metades da única ROI horizontal.
    markers = interpretation_result.get("markers", [])
    upper_valid_markers = [
        marker for marker in markers
        if marker.get("upper", {}).get("valid", False)
    ]
    status["greenMarkerCount"] = len(markers)
    status["greenValidatedMarkerCount"] = len(upper_valid_markers)
    diagnostic_marker = (
        upper_valid_markers[0] if upper_valid_markers else
        (markers[0] if markers else None)
    )
    if diagnostic_marker is not None:
        upper = diagnostic_marker.get("upper", {})
        status["greenFrontRoiMeasured"] = bool(
            upper.get("measured", False)
        )
        status["greenFrontBlackRatio"] = float(
            upper.get("black_ratio", 0.0)
        )
        status["greenFrontRoiValid"] = bool(upper.get("valid", False))
        horizontal = diagnostic_marker.get("horizontal", {})
        for prefix, side in (("greenLeft", "left"), ("greenRight", "right")):
            status[f"{prefix}RoiMeasured"] = bool(
                horizontal.get(f"{side}_measured", False)
            )
            status[f"{prefix}BlackRatio"] = float(
                horizontal.get(f"{side}_black_ratio", 0.0)
            )
            status[f"{prefix}RoiValid"] = bool(
                horizontal.get(f"{side}_valid", False)
            )
    for prefix, candidate in zip(("greenPrimary", "greenSecondary"), candidates):
        status[f"{prefix}X"] = float(candidate["centroid"][0])
        status[f"{prefix}Y"] = float(candidate["centroid"][1])
        status[f"{prefix}Area"] = float(candidate["area"])

    for key, value in tuple(status.items()):
        if isinstance(value, float) and not math.isfinite(value):
            status[key] = 0.0
    if status["greenObservationState"] not in GREEN_OBSERVATION_STATES:
        status["greenObservationState"] = "SEM_VERDE"
    if status["greenInterpretation"] not in GREEN_INTERPRETATIONS:
        status["greenInterpretation"] = "SEM_DECISAO"
        status["greenConfirmed"] = False
    return status
