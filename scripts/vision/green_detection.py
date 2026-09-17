"""Detecção, classificação e estado observado do verde."""

import json
import math
import os
import time
import cv2  # type: ignore
import numpy as np
from .camera_config import (
    CAMERA_ARRAY_COLOR_ORDER,
    CAMERA_EXPOSURE_VALUE,
    DOWNWARD_REFERENCE_FRAME_HEIGHT,
    GREEN_AMBIGUITY_HYSTERESIS_FRAMES,
    GREEN_ASPECT_RATIO_MAX,
    GREEN_ASPECT_RATIO_MIN,
    GREEN_CAPTURE_CANDIDATES_PATH,
    GREEN_CAPTURE_FINAL_MASK_PATH,
    GREEN_CAPTURE_HSV_MASK_PATH,
    GREEN_CAPTURE_RAW_PATH,
    GREEN_CAPTURE_STATS_PATH,
    GREEN_CLEAR_HYSTERESIS_FRAMES,
    GREEN_CLOSE_ITERATIONS,
    GREEN_CLOSE_KERNEL_SIZE,
    GREEN_CONFIRMATION_FRAMES,
    GREEN_DIRECTION_RETENTION_SECONDS,
    GREEN_FRAGMENT_MERGE_DISTANCE_PX,
    GREEN_HUE_MAX,
    GREEN_HUE_MIN,
    GREEN_INTERPRETATIONS,
    GREEN_MIN_AREA_PX,
    GREEN_MIN_AREA_RATIO,
    GREEN_MIN_DIMENSION_PX,
    GREEN_MIN_EXTENT,
    GREEN_OBSERVATION_STATES,
    GREEN_OPEN_ITERATIONS,
    GREEN_OPEN_KERNEL_SIZE,
    GREEN_PAIR_MAX_BLACK_ORIENTATION_DELTA_DEGREES,
    GREEN_PAIR_MAX_VERTICAL_DISTANCE_HEIGHTS,
    GREEN_PARTIAL_AREA_FACTOR,
    GREEN_PARTIAL_ASPECT_RATIO_MIN,
    GREEN_PARTIAL_BORDER_TOLERANCE_PX,
    GREEN_PARTIAL_DIMENSION_FACTOR,
    GREEN_PARTIAL_EXTENT_MIN,
    GREEN_PRIMARY_MARKER_MIN_IOU,
    GREEN_PRIMARY_MARKER_MIN_CONTAINMENT,
    GREEN_PRIMARY_MARKER_MAX_CENTER_SHIFT,
    GREEN_ROI_HALF_SIZE_DIVISOR,
    GREEN_ROI_MIN_BLACK_RATIO,
    GREEN_ROI_MIN_VISIBLE_RATIO,
    GREEN_SIDE_ROI_MIN_BLACK_RATIO,
    GREEN_SIDE_REFERENCE_MIN_OFFSET_WIDTHS,
    GREEN_LOCAL_L_CONFIRMATION_ENABLED,
    GREEN_LOCAL_L_SIDE_DEPTH_SCALE,
    GREEN_LOCAL_TRACK_MIN_COVERAGE,
    GREEN_LOCAL_TRACK_MIN_WIDTH_SCALE,
    GREEN_LOCAL_TRACK_MAX_WIDTH_SCALE,
    GREEN_LOCAL_TRACK_MAX_RESIDUAL_SCALE,
    GREEN_SATURATION_MIN,
    GREEN_SINGLE_OBSERVATION_FRAMES,
    GREEN_TURNAROUND_CONFIRMATION_FRAMES,
    GREEN_UPPER_ROI_HALF_WIDTH_SCALE,
    GREEN_VALUE_MIN,
)
from .numeric import (
    scaled_odd_kernel_size,
    scaled_reference_pixels,
)

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
    """Escala os 4.000 px de referência pela área útil da resolução atual."""

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
                "ExposureValue": CAMERA_EXPOSURE_VALUE,
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
        "greenControlActive": False,
        "greenControlDirection": "NENHUMA",
        "greenCandidateHoldActive": False,
        "greenCandidateHoldFrames": 0,
        "greenArmed": True,
        "greenRearmClearFrames": 0,
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


def green_marker_roi_geometry(contour, frame_width, local_l=False):
    """Calcula as ROIs horizontal e superior ao redor do marcador verde."""

    box = cv2.boxPoints(cv2.minAreaRect(contour))
    minimum_x = float(np.min(box[:, 0]))
    maximum_x = float(np.max(box[:, 0]))
    minimum_y = float(np.min(box[:, 1]))
    maximum_y = float(np.max(box[:, 1]))
    center_x = int(round((minimum_x + maximum_x) / 2.0))
    center_y = int(round((minimum_y + maximum_y) / 2.0))
    half_size = max(1, int(frame_width) // GREEN_ROI_HALF_SIZE_DIVISOR)
    upper_half_width = max(
        half_size,
        int(round(half_size * GREEN_UPPER_ROI_HALF_WIDTH_SCALE)),
    )

    # No experimento, cada hipótese em L usa apenas o preto adjacente ao verde.
    # A geometria extensa original continua sendo usada para pares de 180°.
    side_depth = (max(1, int(round(half_size * GREEN_LOCAL_L_SIDE_DEPTH_SCALE)))
                  if local_l else 2 * half_size)
    horizontal_roi = (
        int(round(minimum_x)) - side_depth,
        center_y - half_size,
        int(round(maximum_x)) + side_depth,
        center_y + half_size,
    )
    # A ROI superior é perpendicular à horizontal. Ela deve ter amostra
    # visível antes que o marcador possa ser chamado de verdadeiro ou falso.
    upper_roi = (
        center_x - upper_half_width,
        int(round(minimum_y)) - 3 * half_size,
        center_x + upper_half_width,
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
        "local_l": local_l,
    }


def measure_black_roi(
    black_mask,
    roi,
    minimum_black_ratio=GREEN_ROI_MIN_BLACK_RATIO,
):
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
        "valid": black_ratio >= minimum_black_ratio,
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
        GREEN_SIDE_ROI_MIN_BLACK_RATIO,
    )
    right = measure_black_roi(
        black_mask,
        (marker_right_x, y1, x2, y2),
        GREEN_SIDE_ROI_MIN_BLACK_RATIO,
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


def create_green_association_mask(structural_mask, green_mask, dead_zone_end_y):
    """Preserva o preto visível ao redor do verde sem alterar a máscara do seguidor."""

    association_mask = structural_mask.copy()
    association_mask[:dead_zone_end_y, :] = 0
    useful_height = min(association_mask.shape[0], green_mask.shape[0])
    useful_width = min(association_mask.shape[1], green_mask.shape[1])
    region = association_mask[:useful_height, :useful_width]
    region[green_mask[:useful_height, :useful_width] > 0] = 0
    # O limite inferior segmenta candidatos verdes, não delimita o preto.
    # Zerar o preto abaixo dele faz uma ROI visível parecer branca.
    return association_mask


def black_roi_orientation_degrees(black_mask, roi, diagnostics=None):
    """Estima a orientação axial do preto dentro de uma ROI."""

    if diagnostics is not None:
        diagnostics.update(confidence=0.0, black_pixels=0, fully_black=False)
    x1, y1, x2, y2 = (int(value) for value in roi)
    x1 = max(0, min(black_mask.shape[1], x1))
    y1 = max(0, min(black_mask.shape[0], y1))
    x2 = max(0, min(black_mask.shape[1], x2))
    y2 = max(0, min(black_mask.shape[0], y2))
    if x2 <= x1 or y2 <= y1:
        return None

    points_y, points_x = np.nonzero(black_mask[y1:y2, x1:x2] > 0)
    if diagnostics is not None:
        diagnostics["black_pixels"] = int(points_x.size)
        # Uma amostra totalmente preta não contém bordas internas que revelem
        # a direção da faixa. O eixo calculado pode ser apenas o formato da ROI.
        diagnostics["fully_black"] = points_x.size == (x2 - x1) * (y2 - y1)
    if points_x.size < 8:
        return None
    points = np.column_stack((points_x, points_y)).astype(np.float64)
    covariance = np.cov(points, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if not np.all(np.isfinite(eigenvalues)) or eigenvalues[-1] <= 0.0:
        return None
    if diagnostics is not None:
        # A confiança mede somente o alongamento da amostra, entre 0 e 1.
        # Zero indica distribuição sem eixo dominante; não é probabilidade de
        # acerto. Este diagnóstico não altera o ângulo nem a decisão do par.
        diagnostics["confidence"] = float(np.clip(
            (eigenvalues[-1] - eigenvalues[0]) / eigenvalues[-1], 0.0, 1.0,
        ))
    direction = eigenvectors[:, -1]
    return float(
        math.degrees(math.atan2(direction[1], direction[0])) % 180.0
    )


def axial_angle_difference_degrees(first, second):
    """Calcula a menor diferença entre orientações sem sentido de percurso."""

    difference = abs(float(first) - float(second)) % 180.0
    return min(difference, 180.0 - difference)


def green_local_track_reference(geometry, black_mask):
    """Ajusta a continuação local abaixo do verde e projeta seu centro até ele."""

    marker_left, marker_right = geometry["marker_horizontal_bounds"]
    marker_width = max(1, marker_right - marker_left)
    marker_height = max(1, int(round(np.ptp(geometry["box"][:, 1]))))
    center_x, center_y = geometry["center"]
    half_size = max(1, black_mask.shape[1] // GREEN_ROI_HALF_SIZE_DIVISOR)
    band_height = max(marker_height, 2 * half_size)
    start_y = int(np.ceil(np.max(geometry["box"][:, 1]))) + 1
    end_y = min(black_mask.shape[0], start_y + band_height)
    # A banda acompanha uma faixa inclinada até sua parte inferior. A busca
    # mais larga evita recortar a faixa real e trocar por um fragmento externo;
    # a projeção abaixo ainda exige que ela esteja adjacente ao verde.
    search_width = max(3 * marker_width, 4 * half_size)
    x1, x2 = max(0, center_x - search_width), min(black_mask.shape[1], center_x + search_width)
    reference = {"valid": False, "samples": 0, "track_x": 0.0, "slope": 0.0,
                 "reason": "insufficient_local_track", "roi": (x1, start_y, x2, end_y)}
    if end_y - start_y < band_height * GREEN_LOCAL_TRACK_MIN_COVERAGE:
        return reference

    minimum_width = max(3, marker_width * GREEN_LOCAL_TRACK_MIN_WIDTH_SCALE)
    maximum_width = marker_width * GREEN_LOCAL_TRACK_MAX_WIDTH_SCALE
    separation = max(1.0, marker_width * GREEN_SIDE_REFERENCE_MIN_OFFSET_WIDTHS)
    rows, centers = [], []
    for y in range(start_y, end_y):
        active = black_mask[y, x1:x2] > 0
        changes = np.diff(np.concatenate(([False], active, [False])).astype(np.int8))
        starts, ends = np.flatnonzero(changes == 1), np.flatnonzero(changes == -1)
        runs = [(abs((left + right - 1) / 2.0 + x1 - center_x),
                 (left + right - 1) / 2.0 + x1)
                for left, right in zip(starts, ends)
                if minimum_width <= right - left <= maximum_width
                and left > 0 and right < active.size]
        runs.sort()
        # Dois segmentos igualmente próximos não oferecem uma referência única.
        if not runs or (len(runs) > 1 and runs[1][0] - runs[0][0] < separation):
            continue
        rows.append(y)
        centers.append(runs[0][1])

    reference["samples"] = len(rows)
    if (len(rows) < band_height * GREEN_LOCAL_TRACK_MIN_COVERAGE
            or rows[-1] - rows[0] < band_height * GREEN_LOCAL_TRACK_MIN_COVERAGE):
        return reference
    rows, centers = np.asarray(rows, dtype=float), np.asarray(centers, dtype=float)
    # Centrar o ajuste em y evita instabilidade numérica. A projeção compensa
    # a inclinação do robô; não usa o centro da câmera nem o alvo do Fusion.
    slope, intercept = np.polyfit(rows - center_y, centers, 1)
    residual = np.abs(centers - (slope * (rows - center_y) + intercept))
    if np.max(residual) > max(2.0, marker_width * GREEN_LOCAL_TRACK_MAX_RESIDUAL_SCALE):
        reference["reason"] = "conflicting_local_track"
        return reference
    offset = center_x - intercept
    # A faixa precisa permanecer adjacente ao marcador ao ser projetada, não
    # ser um preto distante ou atravessar o centro do próprio verde.
    if (not np.isfinite(intercept) or not np.isfinite(slope)
            or abs(offset) < marker_width / 2.0
            or abs(offset) > marker_width + 2 * half_size):
        reference["reason"] = "local_track_not_adjacent"
        return reference
    reference.update(valid=True, track_x=float(intercept), slope=float(slope),
                     reason="local_track_reference")
    return reference


def classify_green_marker_side(marker, black_mask):
    """Escolhe o lado sem exigir piso branco na metade lateral oposta."""

    horizontal = marker["horizontal"]
    left_valid = horizontal["left_valid"]
    right_valid = horizontal["right_valid"]
    if marker["geometry"].get("local_l", False):
        reference = green_local_track_reference(marker["geometry"], black_mask)
        marker["local_track_reference"] = reference
        if reference["valid"]:
            # O preto superior já foi validado pelo chamador. O centro da faixa
            # longitudinal local prevalece sobre ambas as amostras da interseção.
            marker["side_decision_source"] = "local_track_reference"
            return ("DIREITA" if marker["geometry"]["center"][0] > reference["track_x"]
                    else "ESQUERDA")
        if reference["reason"] in ("conflicting_local_track", "local_track_not_adjacent"):
            # Uma referência presente, mas contraditória, não pode ser substituída
            # por uma lateral isolada e fixar o lado errado no coordenador.
            return "AMBIGUO"
        # Cada L exige preto superior e apenas sua própria lateral visível.
        # A lateral oposta fora da imagem não veta uma hipótese válida.
        # Duas hipóteses válidas não escolhem um lado nem representam um 180°.
        if left_valid and right_valid:
            return "AMBIGUO"
        if left_valid:
            marker["side_decision_source"] = "experimental_local_l"
            return "DIREITA"
        if right_valid:
            marker["side_decision_source"] = "experimental_local_l"
            return "ESQUERDA"
        return "AMBIGUO"
    if not horizontal["measured"]:
        return "AMBIGUO"
    if not left_valid and not right_valid:
        return "AMBIGUO"
    if left_valid and not right_valid:
        return "DIREITA"
    if right_valid and not left_valid:
        return "ESQUERDA"

    # Uma interseção pode preencher ambas as metades laterais. Nesse caso,
    # o marcador é verdadeiro; o lado vem da posição relativa ao preto acima,
    # nunca do centro da câmera nem de escolher a maior razão lateral.
    geometry = marker["geometry"]
    x1, y1, x2, y2 = geometry["upper_roi"]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(black_mask.shape[1], x2), min(black_mask.shape[0], y2)
    _black_y, black_x = np.nonzero(black_mask[y1:y2, x1:x2] > 0)
    if black_x.size == 0:
        return "AMBIGUO"
    black_center_x = x1 + float(np.mean(black_x))
    marker["associated_black_x"] = black_center_x
    marker_left, marker_right = geometry["marker_horizontal_bounds"]
    marker_center_x = (marker_left + marker_right) / 2.0
    minimum_offset = max(1.0, (marker_right - marker_left)
                         * GREEN_SIDE_REFERENCE_MIN_OFFSET_WIDTHS)
    offset = marker_center_x - black_center_x
    if offset >= minimum_offset:
        return "DIREITA"
    if offset <= -minimum_offset:
        return "ESQUERDA"
    # Preto simétrico confirma a associação, mas não inventa um lado.
    return "AMBIGUO"


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
            local_l=GREEN_LOCAL_L_CONFIRMATION_ENABLED and len(green_contours) == 1,
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
        marker["interpretation"] = classify_green_marker_side(
            marker, selected_black_mask,
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
        first_orientation_diagnostics, second_orientation_diagnostics = {}, {}
        first_orientation = black_roi_orientation_degrees(
            selected_black_mask,
            first["geometry"]["upper_roi"],
            first_orientation_diagnostics,
        )
        second_orientation = black_roi_orientation_degrees(
            selected_black_mask,
            second["geometry"]["upper_roi"],
            second_orientation_diagnostics,
        )
        first_orientation_defined = not first_orientation_diagnostics.get("fully_black", False)
        second_orientation_defined = not second_orientation_diagnostics.get("fully_black", False)
        orientation_comparison_available = first_orientation_defined and second_orientation_defined
        # Somente o preenchimento completo torna o eixo inconclusivo; confiança
        # baixa, por si só, não muda o critério. Sem um dos eixos, não há conflito
        # angular comprovado. Ambos os pretos superiores e a altura do par ainda
        # devem ser válidos no mesmo frame, com a confirmação temporal original.
        orientation_compatible = (
            first_orientation is not None
            and second_orientation is not None
            and (not orientation_comparison_available or axial_angle_difference_degrees(
                first_orientation,
                second_orientation,
            ) <= GREEN_PAIR_MAX_BLACK_ORIENTATION_DELTA_DEGREES)
        )
        # Uma chegada diagonal gira as duas faixas locais juntas. Orientações
        # divergentes indicam verdes pertencentes a ramos diferentes.
        vertical_compatible = (
            abs(float(first_center[1] - second_center[1]))
            <= vertical_tolerance
        )
        pair_compatible = vertical_compatible and orientation_compatible
        result["pair_compatible"] = pair_compatible
        # Registra os gates e distingue um eixo indefinido de uma comparação
        # angular aprovada. O ângulo bruto continua disponível para diagnóstico.
        result["pair_diagnostics"] = {
            "vertical_delta_px": abs(float(first_center[1] - second_center[1])),
            "vertical_tolerance_px": vertical_tolerance,
            "vertical_compatible": vertical_compatible,
            "first_angle_degrees": first_orientation,
            "second_angle_degrees": second_orientation,
            "angle_delta_degrees": (axial_angle_difference_degrees(first_orientation, second_orientation)
                                    if first_orientation is not None and second_orientation is not None else None),
            "orientation_compatible": orientation_compatible,
            "first_orientation_defined": first_orientation_defined,
            "second_orientation_defined": second_orientation_defined,
            "orientation_comparison_available": orientation_comparison_available,
            "first_confidence": first_orientation_diagnostics.get("confidence", 0.0),
            "second_confidence": second_orientation_diagnostics.get("confidence", 0.0),
            "first_black_pixels": first_orientation_diagnostics.get("black_pixels", 0),
            "second_black_pixels": second_orientation_diagnostics.get("black_pixels", 0),
            "first_upper_ratio": first["upper"]["black_ratio"],
            "second_upper_ratio": second["upper"]["black_ratio"],
        }
        if not pair_compatible:
            # Dois verdes nunca podem virar uma curva lateral por desempate.
            # Se o par ainda não estiver coerente, aguarda outro frame para não
            # executar esquerda ou direita em uma marca real de retorno.
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

    if len(green_contours) != 1:
        # A presença de outro contorno verde pode ser o segundo marcador de um
        # retorno de 180° ainda parcialmente oculto ou sem faixa preta válida.
        # A leitura lateral só é segura quando existe exatamente um contorno.
        result["interpretation"] = "AMBIGUO"
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


def green_geometry_reason(result):
    """Explica a classificação atual pelas mesmas condições geométricas do detector."""

    if "primary_marker_reason" in result:
        return result["primary_marker_reason"]
    markers = result.get("markers", [])
    if not markers:
        return "no_candidate"
    if any(not marker["upper"]["measured"] for marker in markers):
        return "upper_roi_not_visible"
    valid = [marker for marker in markers if marker["upper"]["valid"]]
    if not valid:
        return "upper_black_missing"
    if len(valid) > 2:
        return "multiple_valid_markers"
    if len(valid) == 2:
        return "compatible_pair" if result["pair_compatible"] else "pair_geometry_incompatible"
    if len(markers) != 1:
        return "additional_unvalidated_marker"
    horizontal = valid[0]["horizontal"]
    if result["interpretation"] in ("ESQUERDA", "DIREITA"):
        if valid[0].get("side_decision_source") == "local_track_reference":
            return "local_track_reference"
        if valid[0].get("side_decision_source") == "experimental_local_l":
            return "experimental_local_l"
        return "lateral_geometry_valid"
    local_reason = valid[0].get("local_track_reference", {}).get("reason")
    if local_reason in ("conflicting_local_track", "local_track_not_adjacent"):
        return local_reason
    if not horizontal["measured"]:
        return "side_roi_not_visible"
    if horizontal["left_valid"] and horizontal["right_valid"]:
        if valid[0]["geometry"].get("local_l", False):
            return "both_local_l_hypotheses_valid"
        return "upper_side_reference_inconclusive"
    if not horizontal["left_valid"] and not horizontal["right_valid"]:
        return "side_black_missing"
    return "lateral_geometry_valid"


class GreenObservationTracker:
    """Confirma observações novas e remove decisões após curta histerese."""

    def __init__(self):
        self.last_sequence = None
        self.pending_interpretation = "SEM_DECISAO"
        self.consecutive_samples = 0
        self.missing_samples = 0
        self.ambiguous_samples = 0
        self.confirmed_interpretation = "SEM_DECISAO"
        self.last_direction_seen_at = None
        self.primary_marker_bounds = None
        self.primary_marker_side = "SEM_DECISAO"
        self.primary_marker_clear_frames = 0

    def resolve_primary_marker(self, result, black_mask):
        """Revalida o primeiro verde sem deixar outro contorno substituir sua evidência."""

        markers = result["markers"]
        if not markers:
            self.primary_marker_clear_frames += 1
            if self.primary_marker_clear_frames >= GREEN_CLEAR_HYSTERESIS_FRAMES:
                self.primary_marker_bounds = None
                self.primary_marker_side = "SEM_DECISAO"
            return result
        self.primary_marker_clear_frames = 0

        # O par continua sendo avaliado pela geometria original, antes de
        # qualquer leitura individual. A promoção temporal também não muda.
        if result["pair_compatible"]:
            return result

        if self.primary_marker_bounds is None:
            if len(markers) == 1:
                # Identifica o candidato antes de conhecer seu lado. Isso não
                # valida o verde: preto e confirmação temporal continuam exigidos.
                box = markers[0]["geometry"]["box"]
                self.primary_marker_bounds = (
                    np.min(box, axis=0), np.max(box, axis=0),
                )
                if result["interpretation"] in ("ESQUERDA", "DIREITA"):
                    self.primary_marker_side = result["interpretation"]
            return result

        previous_min, previous_max = self.primary_marker_bounds
        matches = []
        best_iou = 0.0
        best_containment = 0.0
        for index, marker in enumerate(markers):
            box = marker["geometry"]["box"]
            current_min, current_max = np.min(box, axis=0), np.max(box, axis=0)
            overlap = np.maximum(
                0.0, np.minimum(previous_max, current_max) - np.maximum(previous_min, current_min),
            )
            intersection = float(np.prod(overlap))
            previous_size = previous_max - previous_min
            current_size = current_max - current_min
            previous_area, current_area = float(np.prod(previous_size)), float(np.prod(current_size))
            union = previous_area + current_area - intersection
            iou = intersection / union if union > 0.0 else 0.0
            smaller_area = min(previous_area, current_area)
            containment = intersection / smaller_area if smaller_area > 0.0 else 0.0
            center_shift = np.abs((current_min + current_max - previous_min - previous_max) / 2.0)
            centers_close = np.all(center_shift <= GREEN_PRIMARY_MARKER_MAX_CENTER_SHIFT
                                   * np.maximum(previous_size, current_size))
            best_iou = max(best_iou, iou)
            best_containment = max(best_containment, containment)
            # Um recorte que cresce perde IoU mesmo mantendo o mesmo verde.
            # A contenção recupera esse caso sem escolher o melhor de dois pares.
            if iou >= GREEN_PRIMARY_MARKER_MIN_IOU:
                matches.append((index, current_min, current_max, "iou"))
            elif containment >= GREEN_PRIMARY_MARKER_MIN_CONTAINMENT and centers_close:
                matches.append((index, current_min, current_max, "size_change"))

        resolved = dict(result)
        resolved["primary_match_count"] = len(matches)
        resolved["primary_best_iou"] = best_iou
        resolved["primary_best_containment"] = best_containment
        resolved.update(interpretation="AMBIGUO", left_seen=False,
                        right_seen=False, path_black_valid=False)
        if len(matches) != 1:
            # Sem correspondência única, não usa um verde novo para confirmar
            # o anterior. A ausência real continua necessária para o rearme.
            resolved["primary_marker_reason"] = (
                "primary_marker_missing" if not matches else "primary_marker_multiple_matches"
            )
            return resolved

        index, current_min, current_max, match_method = matches[0]
        resolved["primary_match_method"] = match_method
        self.primary_marker_bounds = (current_min, current_max)
        individual = analyze_green_marker_contours([markers[index]["contour"]], black_mask)
        side = individual["interpretation"]
        resolved["primary_marker_index"] = index
        resolved["primary_marker_reason"] = "tracked_primary_" + green_geometry_reason(individual)
        resolved["markers"] = list(markers)
        resolved["markers"][index] = individual["markers"][0]
        if self.primary_marker_side == "SEM_DECISAO" and side in ("ESQUERDA", "DIREITA"):
            # Trava o primeiro lado validado do marcador já acompanhado, mesmo
            # que outro verde tenha entrado no quadro antes dessa evidência.
            self.primary_marker_side = side
        if side == self.primary_marker_side:
            # Este frame contém nova validação superior e lateral do mesmo
            # marcador: pode completar a confirmação, independentemente do outro.
            resolved.update(interpretation=side, left_seen=side == "ESQUERDA",
                            right_seen=side == "DIREITA", path_black_valid=True)
        elif side == "VERDE_FALSO":
            resolved["interpretation"] = side
        return resolved

    def update(self, line_sequence, interpretation, observed_at=None):
        """Confirma quadros novos e retém orientação lateral por 0,5 segundo."""

        observed_at = (
            time.perf_counter() if observed_at is None else float(observed_at)
        )
        if line_sequence == self.last_sequence:
            return (
                self.confirmed_interpretation,
                self.confirmed_interpretation != "SEM_DECISAO",
                self.consecutive_samples,
            )
        self.last_sequence = line_sequence

        has_valid_evidence = (
            self.pending_interpretation in (
                "ESQUERDA",
                "DIREITA",
                "RETORNO_180",
            )
            or self.confirmed_interpretation in (
                "ESQUERDA",
                "DIREITA",
                "RETORNO_180",
            )
        )
        if interpretation == "AMBIGUO" and has_valid_evidence:
            self.ambiguous_samples += 1
            if self.confirmed_interpretation in (
                "ESQUERDA", "DIREITA", "RETORNO_180",
            ):
                # A ambiguidade do segundo verde não desfaz uma decisão já
                # confirmada. Só uma ausência real rearma o próximo evento.
                if (
                    self.ambiguous_samples > GREEN_AMBIGUITY_HYSTERESIS_FRAMES
                    and self.pending_interpretation == "RETORNO_180"
                    and self.confirmed_interpretation != "RETORNO_180"
                ):
                    self.pending_interpretation = self.confirmed_interpretation
                    self.consecutive_samples = 0
                return (
                    self.confirmed_interpretation,
                    True,
                    self.consecutive_samples,
                )
            if (
                self.ambiguous_samples
                <= GREEN_AMBIGUITY_HYSTERESIS_FRAMES
            ):
                # Um frame inconclusivo não contradiz a evidência válida já
                # acumulada. A decisão confirmada continua disponível e uma
                # confirmação em andamento retoma do mesmo ponto no próximo
                # frame conclusivo.
                if self.confirmed_interpretation != "SEM_DECISAO":
                    return (
                        self.confirmed_interpretation,
                        True,
                        self.consecutive_samples,
                    )
                return "AMBIGUO", False, self.consecutive_samples

            # Ambiguidade persistente invalida a memória para não permitir que
            # uma leitura antiga seja completada por outro marcador da pista.
            self.pending_interpretation = "AMBIGUO"
            self.confirmed_interpretation = "SEM_DECISAO"
            self.consecutive_samples = 1
            return "AMBIGUO", False, self.consecutive_samples

        self.ambiguous_samples = 0

        if (
            self.confirmed_interpretation == "RETORNO_180"
            and interpretation != "SEM_DECISAO"
        ):
            # Dois verdes já confirmados não podem virar uma curva lateral
            # enquanto um deles oscila ou sai parcialmente da imagem.
            self.pending_interpretation = "RETORNO_180"
            self.missing_samples = 0
            return "RETORNO_180", True, self.consecutive_samples

        if (
            self.confirmed_interpretation in ("ESQUERDA", "DIREITA")
            and interpretation != "SEM_DECISAO"
        ):
            if interpretation == "RETORNO_180":
                # Um par compatível precisa completar a confirmação temporal;
                # até lá, preserva o lado já confirmado do mesmo evento.
                if self.pending_interpretation == "RETORNO_180":
                    self.consecutive_samples += 1
                else:
                    self.pending_interpretation = "RETORNO_180"
                    self.consecutive_samples = 1
                required_samples = max(
                    GREEN_CONFIRMATION_FRAMES,
                    GREEN_TURNAROUND_CONFIRMATION_FRAMES,
                )
                self.consecutive_samples = min(self.consecutive_samples, required_samples)
                if self.consecutive_samples >= required_samples:
                    self.confirmed_interpretation = "RETORNO_180"
                return self.confirmed_interpretation, True, self.consecutive_samples

            # Leituras falsas ou do lado oposto não substituem a curva já
            # confirmada. Apenas o retorno geometricamente válido pode promovê-la.
            if interpretation == self.confirmed_interpretation:
                self.last_direction_seen_at = observed_at
            self.pending_interpretation = self.confirmed_interpretation
            self.missing_samples = 0
            return (
                self.confirmed_interpretation,
                True,
                self.consecutive_samples,
            )

        if interpretation in ("ESQUERDA", "DIREITA"):
            # O instante é renovado em todo frame detectado, inclusive durante
            # a confirmação, para que a retenção conte da última visão real.
            self.last_direction_seen_at = observed_at

        if interpretation == "SEM_DECISAO":
            direction_is_retained = (
                self.confirmed_interpretation in ("ESQUERDA", "DIREITA")
                and self.last_direction_seen_at is not None
                and observed_at - self.last_direction_seen_at
                < GREEN_DIRECTION_RETENTION_SECONDS
            )
            if direction_is_retained:
                return (
                    self.confirmed_interpretation,
                    True,
                    self.consecutive_samples,
                )
            if self.confirmed_interpretation in ("ESQUERDA", "DIREITA"):
                self.pending_interpretation = "SEM_DECISAO"
                self.confirmed_interpretation = "SEM_DECISAO"
                self.consecutive_samples = 0
                self.missing_samples = 0
                return "SEM_DECISAO", False, 0
            self.missing_samples += 1
            if self.missing_samples >= GREEN_CLEAR_HYSTERESIS_FRAMES:
                self.pending_interpretation = "SEM_DECISAO"
                self.confirmed_interpretation = "SEM_DECISAO"
                self.consecutive_samples = 0
            return (
                self.confirmed_interpretation,
                self.confirmed_interpretation != "SEM_DECISAO",
                self.consecutive_samples,
            )

        self.missing_samples = 0
        if interpretation != self.pending_interpretation:
            self.pending_interpretation = interpretation
            self.consecutive_samples = 1
            self.confirmed_interpretation = "SEM_DECISAO"
        else:
            self.consecutive_samples += 1

        required_samples = GREEN_CONFIRMATION_FRAMES
        if interpretation in ("ESQUERDA", "DIREITA"):
            required_samples = max(
                required_samples, GREEN_SINGLE_OBSERVATION_FRAMES
            )
        elif interpretation == "RETORNO_180":
            required_samples = max(
                required_samples,
                GREEN_TURNAROUND_CONFIRMATION_FRAMES,
            )
        # A telemetria representa o progresso da confirmação, não há motivo
        # para crescer sem limite depois de a decisão já estar aceita.
        self.consecutive_samples = min(
            self.consecutive_samples,
            required_samples,
        )
        confirmable = interpretation not in ("AMBIGUO", "SEM_DECISAO")
        if confirmable and self.consecutive_samples >= required_samples:
            self.confirmed_interpretation = interpretation

        published_interpretation = self.confirmed_interpretation
        if interpretation == "AMBIGUO":
            return "AMBIGUO", False, self.consecutive_samples
        return (
            published_interpretation,
            self.confirmed_interpretation != "SEM_DECISAO",
            self.consecutive_samples,
        )


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
        "greenGeometryReason": green_geometry_reason(interpretation_result),
        "greenInterpretation": published_interpretation,
        "greenConfirmed": confirmed,
        "greenCandidateCount": len(candidates),
        "greenRejectedCount": int(rejected_count),
        "greenLeftSeen": interpretation_result["left_seen"],
        "greenRightSeen": interpretation_result["right_seen"],
        "greenPairCompatible": interpretation_result["pair_compatible"],
        "greenConsecutiveSamples": int(consecutive_samples),
        "greenProcessingMs": float(processing_ms),
        "greenPrimaryMatchCount": interpretation_result.get("primary_match_count", 0),
        "greenPrimaryBestIou": interpretation_result.get("primary_best_iou", 0.0),
        "greenPrimaryBestContainment": interpretation_result.get("primary_best_containment", 0.0),
        "greenPrimaryMatchMethod": interpretation_result.get("primary_match_method", "none"),
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
        markers[interpretation_result["primary_marker_index"]]
        if "primary_marker_index" in interpretation_result else
        upper_valid_markers[0] if upper_valid_markers else
        (markers[0] if markers else None)
    )
    if diagnostic_marker is not None:
        local_reference = diagnostic_marker.get("local_track_reference", {})
        status["greenLocalReferenceValid"] = bool(local_reference.get("valid", False))
        status["greenLocalTrackX"] = float(local_reference.get("track_x", 0.0))
        status["greenLocalTrackSlope"] = float(local_reference.get("slope", 0.0))
        status["greenLocalReferenceSamples"] = int(local_reference.get("samples", 0))
        status["greenLocalReferenceReason"] = local_reference.get("reason", "not_evaluated")
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


def confirmed_green_path_black_valid(
    interpretation_result,
    tracker_result,
):
    """Preserva o gate preto durante oscilações após uma confirmação lateral."""

    published_interpretation, confirmed, _consecutive_samples = tracker_result
    if interpretation_result.get("path_black_valid", False):
        return True

    # A retenção vale durante uma oscilação inconclusiva ou falsa posterior à
    # confirmação. Uma ausência real continua removendo o gate atual. Para o
    # retorno, o C++ ainda exige exatamente dois candidatos no mesmo frame.
    return bool(
        confirmed
        and interpretation_result.get("interpretation") in (
            "AMBIGUO",
            "VERDE_FALSO",
        )
        and published_interpretation in (
            "ESQUERDA",
            "DIREITA",
            "RETORNO_180",
        )
    )
