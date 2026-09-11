"""Máscaras puras usadas para segmentar a faixa preta."""

import math
import time
from functools import lru_cache
import cv2  # type: ignore
import numpy as np
from .camera_config import (
    LINE_MIN_COMPONENT_AREA_PX,
    LINE_MIN_COMPONENT_CORE_RATIO,
    LINE_MIN_COMPONENT_THICKNESS_PX,
    SPECULAR_REPAIR_MAX_DIAMETER_PX,
    SPECULAR_REPAIR_MAX_SATURATION,
    SPECULAR_REPAIR_MIN_VALUE,
    SPECULAR_REPAIR_REFERENCE_FRAME_HEIGHT,
)
from .green_detection import (
    frame_to_hsv,
)
from .illumination_correction import (
    apply_line_illumination_correction,
)
from .numeric import (
    scaled_odd_kernel_size,
)

@lru_cache(maxsize=16)
def cached_structuring_element(shape, width, height):
    """Reutiliza até 16 kernels imutáveis definidos apenas por sua geometria."""

    return cv2.getStructuringElement(shape, (width, height))


@lru_cache(maxsize=16)
def cached_relative_line_threshold_lut(
    ratio_percent,
    maximum_brightness,
    minimum_threshold=0,
):
    """Cria uma vez os limites equivalentes ao threshold relativo da linha."""

    background_values = np.arange(256, dtype=np.uint16)
    relative_limits = (
        background_values * int(ratio_percent)
    ) // 100
    threshold_lut = np.clip(
        relative_limits,
        int(minimum_threshold),
        int(maximum_brightness),
    ).astype(np.uint8)
    return threshold_lut.reshape(1, 256)


def create_line_binary_mask(gray_roi, vision_profile, timings=None):
    """Separa a fita preta usando o método configurado para cada câmera."""

    if timings is not None:
        timings["backgroundKernelMs"] = 0.0
        timings["backgroundCloseMs"] = 0.0
        timings["binaryCompareMs"] = 0.0
    background_kernel_size = vision_profile.get("line_background_kernel_size")
    if background_kernel_size is None:
        _, binary_mask = cv2.threshold(
            gray_roi,
            vision_profile["line_threshold"],
            255,
            cv2.THRESH_BINARY_INV,
        )
        return binary_mask

    # O fechamento grande remove estruturas escuras menores que o kernel e
    # produz uma estimativa da iluminação do piso. A comparação relativa
    # continua funcionando quando o papel branco fica escuro sem o LED.
    background_kernel_started = (
        time.perf_counter() if timings is not None else 0.0
    )
    background_kernel = cached_structuring_element(
        cv2.MORPH_RECT,
        background_kernel_size,
        background_kernel_size,
    )
    if timings is not None:
        timings["backgroundKernelMs"] = (
            time.perf_counter() - background_kernel_started
        ) * 1000.0
    background_close_started = (
        time.perf_counter() if timings is not None else 0.0
    )
    local_background = cv2.morphologyEx(
        gray_roi,
        cv2.MORPH_CLOSE,
        background_kernel,
    )
    if timings is not None:
        timings["backgroundCloseMs"] = (
            time.perf_counter() - background_close_started
        ) * 1000.0
    ratio_percent = vision_profile["line_max_background_ratio_percent"]
    maximum_brightness = vision_profile["line_max_brightness"]
    binary_compare_started = (
        time.perf_counter() if timings is not None else 0.0
    )
    threshold_lut = cached_relative_line_threshold_lut(
        ratio_percent,
        maximum_brightness,
        # Um piso de limiar em cinza de 8 bits impede que uma fita escura que
        # ocupa o canto seja confundida com seu próprio fundo. Zero desativa.
        vision_profile.get("line_min_threshold", 0),
    )
    local_threshold = cv2.LUT(local_background, threshold_lut)
    binary_mask = cv2.compare(
        gray_roi,
        local_threshold,
        cv2.CMP_LE,
    )
    if timings is not None:
        timings["binaryCompareMs"] = (
            time.perf_counter() - binary_compare_started
        ) * 1000.0
    return binary_mask


def repair_small_specular_holes(binary_mask, color_roi, camera_format):
    """Preenche apenas reflexos claros, pequenos e internos à fita preta."""

    repair_status = {
        "specularRepairPixels": 0,
        "specularRepairComponents": 0,
    }
    if binary_mask.size == 0:
        return binary_mask, repair_status

    frame_height = color_roi.shape[0]
    maximum_diameter = max(1, int(round(
        SPECULAR_REPAIR_MAX_DIAMETER_PX * frame_height /
        SPECULAR_REPAIR_REFERENCE_FRAME_HEIGHT
    )))
    maximum_area = maximum_diameter * maximum_diameter
    try:
        hsv_roi = frame_to_hsv(color_roi, camera_format)
    except ValueError:
        # Sem a ordem de cores confirmada, não é seguro classificar um brilho.
        return binary_mask, repair_status

    contours, hierarchy = cv2.findContours(
        binary_mask.copy(),
        cv2.RETR_CCOMP,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    if hierarchy is None:
        return binary_mask, repair_status

    repaired_mask = binary_mask.copy()
    for contour_index, contour in enumerate(contours):
        # No modo CCOMP, apenas os contornos com pai representam buracos
        # fechados. Regiões ligadas ao fundo sempre permanecem de fora.
        if hierarchy[0][contour_index][3] < 0:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        if (width > maximum_diameter or height > maximum_diameter or
                cv2.contourArea(contour) > maximum_area):
            # Uma abertura larga pode ser uma separação real da fita.
            continue

        contour_in_roi = contour.copy()
        contour_in_roi[:, :, 0] -= x
        contour_in_roi[:, :, 1] -= y
        hole_shape = np.zeros((height, width), dtype=np.uint8)
        cv2.drawContours(hole_shape, [contour_in_roi], -1, 255, -1)
        hole_pixels = np.logical_and(
            hole_shape > 0,
            repaired_mask[y:y + height, x:x + width] == 0,
        )
        hole_pixel_count = int(np.count_nonzero(hole_pixels))
        if hole_pixel_count == 0 or hole_pixel_count > maximum_area:
            continue

        hole_hsv = hsv_roi[y:y + height, x:x + width]
        clear_and_unsaturated = np.logical_and(
            hole_hsv[:, :, 2] >= SPECULAR_REPAIR_MIN_VALUE,
            hole_hsv[:, :, 1] <= SPECULAR_REPAIR_MAX_SATURATION,
        )
        if not np.all(clear_and_unsaturated[hole_pixels]):
            # Verde e outras cores saturadas nunca podem virar preto.
            continue

        repaired_mask[y:y + height, x:x + width][hole_pixels] = 255
        repair_status["specularRepairPixels"] += hole_pixel_count
        repair_status["specularRepairComponents"] += 1
    return repaired_mask, repair_status


def create_filtered_line_mask(
    frame,
    vision_profile,
    camera_format="RGB888",
    return_repair_status=False,
    timings=None,
):
    """Segmenta a linha preta na parte inferior sem gerar decisões de controle."""

    frame_height = frame.shape[0]
    roi_start_y = int(round(frame_height * vision_profile["line_roi_start_ratio"]))
    line_roi = frame[roi_start_y:frame_height, :]
    uncorrected_gray_roi = cv2.cvtColor(line_roi, cv2.COLOR_BGR2GRAY)
    gray_roi, illumination_status = apply_line_illumination_correction(
        uncorrected_gray_roi,
        vision_profile,
        frame.shape,
        roi_start_y,
    )
    if timings is not None:
        timings["illuminationCorrectionMs"] = illumination_status[
            "illuminationCorrectionMs"
        ]

    scaled_profile = dict(vision_profile)
    if "geometry_reference" in vision_profile:
        if "line_background_kernel_size" in scaled_profile:
            scaled_profile["line_background_kernel_size"] = scaled_odd_kernel_size(
                scaled_profile["line_background_kernel_size"],
                frame_height,
            )
        scaled_profile["open_kernel_size"] = scaled_odd_kernel_size(
            vision_profile["open_kernel_size"], frame_height
        )
        scaled_profile["close_kernel_size"] = scaled_odd_kernel_size(
            vision_profile["close_kernel_size"], frame_height
        )
    binary_started = time.perf_counter() if timings is not None else 0.0
    binary_mask = create_line_binary_mask(
        gray_roi,
        scaled_profile,
        timings=timings,
    )
    if illumination_status["illuminationCorrectionActive"]:
        # A compensação não pode desfazer o piso absoluto de cinza que
        # recupera fita real quando ela ocupa a borda e contamina o fundo local.
        minimum_threshold = int(scaled_profile.get("line_min_threshold", 0))
        if minimum_threshold > 0:
            dark_pixels = uncorrected_gray_roi <= minimum_threshold
            illumination_status["illuminationDarkPixelsPreserved"] = int(
                np.count_nonzero(dark_pixels & (binary_mask == 0))
            )
            binary_mask[dark_pixels] = 255
    if timings is not None:
        timings["binaryMs"] = (
            time.perf_counter() - binary_started
        ) * 1000.0
    specular_started = time.perf_counter() if timings is not None else 0.0

    repair_status = {
        "specularRepairPixels": 0,
        "specularRepairComponents": 0,
        **illumination_status,
    }
    if timings is not None:
        timings["specularMs"] = (
            time.perf_counter() - specular_started
        ) * 1000.0

    open_kernel_shape = (
        cv2.MORPH_ELLIPSE
        if scaled_profile.get("open_kernel_shape") == "ellipse"
        else cv2.MORPH_RECT
    )
    open_kernel = cached_structuring_element(
        open_kernel_shape,
        scaled_profile["open_kernel_size"],
        scaled_profile["open_kernel_size"],
    )
    close_kernel = cached_structuring_element(
        cv2.MORPH_RECT,
        scaled_profile["close_kernel_size"],
        scaled_profile["close_kernel_size"],
    )
    morph_open_started = time.perf_counter() if timings is not None else 0.0
    filtered_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, open_kernel)
    morph_open_ms = (
        (time.perf_counter() - morph_open_started) * 1000.0
        if timings is not None
        else 0.0
    )
    morph_close_started = time.perf_counter() if timings is not None else 0.0
    filtered_mask = cv2.morphologyEx(filtered_mask, cv2.MORPH_CLOSE, close_kernel)
    morph_close_ms = (
        (time.perf_counter() - morph_close_started) * 1000.0
        if timings is not None
        else 0.0
    )
    if timings is not None:
        timings["morphOpenMs"] = morph_open_ms
        timings["morphCloseMs"] = morph_close_ms
        timings["morphMs"] = morph_open_ms + morph_close_ms
    if return_repair_status:
        return filtered_mask, roi_start_y, repair_status
    return filtered_mask, roi_start_y


def scale_reference_y(reference_y, reference_height, frame_height):
    """Converte uma coordenada vertical de referência para a altura real."""

    return int(round(frame_height * reference_y / reference_height))


def resolve_vision_geometry(frame_height, vision_profile):
    """Converte os limites verticais da máscara preta e da visão verde."""

    geometry_reference = vision_profile.get("geometry_reference")
    if geometry_reference is None:
        return {
            "structural_end_y": frame_height,
            "green_end_y": frame_height,
            "ignored_start_y": None,
            "pixel_scale": 1.0,
        }

    reference_height = geometry_reference["frame_height"]
    structural_end_y = scale_reference_y(
        geometry_reference["structural_end_y"],
        reference_height,
        frame_height,
    )
    green_end_y = scale_reference_y(
        geometry_reference.get(
            "green_end_y",
            geometry_reference["structural_end_y"],
        ),
        reference_height,
        frame_height,
    )
    if not 0 < structural_end_y <= frame_height:
        raise ValueError("O limite estrutural da câmera está fora do frame.")
    if not 0 < green_end_y <= frame_height:
        raise ValueError("O limite da visão verde está fora do frame.")

    return {
        "structural_end_y": structural_end_y,
        "green_end_y": green_end_y,
        "ignored_start_y": structural_end_y,
        "pixel_scale": float(frame_height) / float(reference_height),
    }


def create_structural_line_mask(
    filtered_mask,
    roi_start_y,
    structural_end_y,
    structural_start_y=0,
):
    """Remove da máscara as áreas físicas que não podem gerar candidatos."""

    structural_start_in_roi = max(
        0,
        min(filtered_mask.shape[0], structural_start_y - roi_start_y),
    )

    structural_end_in_roi = max(
        0,
        min(filtered_mask.shape[0], structural_end_y - roi_start_y),
    )

    if (
        structural_start_in_roi <= 0
        and structural_end_in_roi >= filtered_mask.shape[0]
    ):
        return filtered_mask

    structural_mask = filtered_mask.copy()
    structural_mask[:structural_start_in_roi, :] = 0
    structural_mask[structural_end_in_roi:, :] = 0

    return structural_mask


def component_has_min_thickness(
    component_mask,
    minimum_thickness_px=LINE_MIN_COMPONENT_THICKNESS_PX,
    minimum_core_ratio=LINE_MIN_COMPONENT_CORE_RATIO,
):
    """
    Rejeita componentes predominantemente finos,
    como frestas entre placas da pista.

    Não basta existir um único ponto grosso:
    uma fração relevante do componente precisa
    possuir espessura compatível com a fita.
    """

    if component_mask.size == 0:
        return False

    component_pixels = cv2.countNonZero(
        component_mask
    )

    if component_pixels == 0:
        return False

    distance_map = cv2.distanceTransform(
        component_mask,
        cv2.DIST_L2,
        3,
    )

    minimum_radius = (
        minimum_thickness_px
        / 2.0
    )

    thick_core_pixels = int(
        np.count_nonzero(
            distance_map >= minimum_radius
        )
    )

    thick_core_ratio = (
        float(thick_core_pixels)
        / float(component_pixels)
    )

    return (
        thick_core_ratio
        >= minimum_core_ratio
    )


def create_line_candidate_mask(
    structural_mask,
    vision_profile,
    return_accepted_contours=False,
    green_mask=None,
):
    """Mantém somente componentes compatíveis com a fita preta."""

    if green_mask is not None:
        # Exclui cor já identificada antes de preencher os contornos. A cópia
        # preserva o preto estrutural usado pela associação dos marcadores verdes.
        structural_mask = structural_mask.copy()
        useful_height = min(structural_mask.shape[0], green_mask.shape[0])
        useful_width = min(structural_mask.shape[1], green_mask.shape[1])
        region = structural_mask[:useful_height, :useful_width]
        region[green_mask[:useful_height, :useful_width] > 0] = 0

    full_contours, _ = cv2.findContours(
        structural_mask.copy(),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    accepted_contours = []

    minimum_area_px = vision_profile.get(
        "line_min_component_area_px",
        LINE_MIN_COMPONENT_AREA_PX,
    )

    minimum_short_side_px = (
        min(structural_mask.shape[:2])
        * vision_profile[
            "full_line_min_short_side_ratio"
        ]
    )

    maximum_area = (
        structural_mask.size
        * vision_profile.get(
            "full_line_max_area_ratio",
            1.0,
        )
    )

    for contour in full_contours:
        contour_area = cv2.contourArea(
            contour
        )

        # Componentes minúsculos não podem representar
        # uma faixa útil para o robô.
        if (
            contour_area
            < minimum_area_px
        ):
            continue

        if contour_area > maximum_area:
            continue

        rect = cv2.minAreaRect(
            contour
        )

        width, height = rect[1]

        if (
            not math.isfinite(width)
            or not math.isfinite(height)
            or width <= 0.0
            or height <= 0.0
        ):
            continue

        short_side_px = min(
            width,
            height,
        )

        if (
            short_side_px
            < minimum_short_side_px
        ):
            continue

        # Analisa a espessura real do próprio componente.
        # Isso evita aceitar uma fresta longa apenas porque
        # sua caixa rotacionada ficou larga.
        component_mask = np.zeros_like(
            structural_mask
        )

        cv2.drawContours(
            component_mask,
            [contour],
            -1,
            255,
            cv2.FILLED,
        )

        if not component_has_min_thickness(
            component_mask,
            vision_profile.get(
                "line_min_component_thickness_px",
                LINE_MIN_COMPONENT_THICKNESS_PX,
            ),
            vision_profile.get(
                "line_min_component_core_ratio",
                LINE_MIN_COMPONENT_CORE_RATIO,
            ),
        ):
            continue

        accepted_contours.append(
            contour
        )

    line_candidate_mask = (
        structural_mask.copy()
    )

    line_candidate_mask.fill(0)

    if accepted_contours:
        cv2.drawContours(
            line_candidate_mask,
            accepted_contours,
            -1,
            255,
            cv2.FILLED,
        )

    if return_accepted_contours:
        return line_candidate_mask, tuple(accepted_contours)
    return line_candidate_mask
