"""Orientação Fusion e retenções associadas ao verde."""

import math
import time
import cv2  # type: ignore
import numpy as np
from .camera_config import (
    FUSION_EXTREME_PIVOT_GUARD_ENTER_ERROR_DEG,
    FUSION_EXTREME_PIVOT_GUARD_RELEASE_ERROR_DEG,
    FUSION_FULL_NORMAL_STEERING_DEG,
    FUSION_FULL_PIVOT_STEERING_DEG,
    FUSION_MIN_FORWARD_SPEED_SCALE,
    FUSION_POWER_CURVE,
    FUSION_STEERING_DEADBAND_DEG,
    FUSION_STRONG_CORRECTION_ERROR_DEG,
    FUSION_STYLE_NEAR_BAND_HEIGHT_RATIO,
    FUSION_STYLE_TARGET_BAND_HEIGHT_RATIO,
    FUSION_STYLE_TARGET_SEARCH_HEIGHT_RATIO,
    FUSION_STYLE_TRANSVERSE_WIDTH_STABILITY_RATIO,
    FUSION_TARGET_CONSISTENCY_ANGLE_SCALE_DEG,
    FUSION_TARGET_CONSISTENCY_SHIFT_WIDTH_RATIO,
    FUSION_TARGET_ESTABLISHED_LENGTH_RATIO,
    FUSION_TARGET_HISTORY_MAX_MISSED_FRAMES,
    FUSION_TARGET_SHORT_LENGTH_RATIO,
    FUSION_TARGET_STABLE_ANGLE_DELTA_DEG,
    FUSION_TARGET_STABLE_FRAMES_FOR_FULL_SPEED,
    FUSION_TARGET_STABLE_SHIFT_WIDTH_RATIO,
    GREEN_CANDIDATE_HOLD_MAX_FRAMES,
    GREEN_FUSION_TARGET_HOLD_MAX_FRAMES,
    GREEN_TRUSTED_POSITION_CENTER_LIMIT,
    LIMIAR_CENTRALIZACAO_VERDE,
    NORMAL_BASE_POWER,
    NORMAL_FULL_STEERING_ERROR,
    NORMAL_INNER_MIN_POWER,
    NORMAL_MAX_POWER,
    QUADROS_PARA_REARMAR_VERDE,
)
from .normal_trajectory import (
    empty_normal_trajectory,
    extract_normal_line_trajectory,
    normal_trajectory_horizontal_bounds,
    resolve_normal_trajectory_envelope,
)
from .numeric import (
    finite_virtual_position,
)

def empty_fusion_style_line(processing_ms=0.0):
    """Cria a telemetria vazia do diagnóstico inspirado no FusionZero."""

    return {
        "enabled": False,
        "valid": False,
        "coordinateFrame": "processedLineMaskPixels",
        "angleConvention": "90StraightBelow90LeftAbove90Right",
        "angleDeg": None,
        "nearPoint": None,
        "farPoint": None,
        "topBandPoint": None,
        "topBand": None,
        "referenceSource": "none",
        "preferredDirection": "NONE",
        "candidateContourCount": 0,
        "contourAreaPx": 0.0,
        "selection": "none",
        "targetLengthRatio": 0.0,
        "targetConsistency": 0.0,
        "targetStableFrames": 0,
        "targetReacquired": False,
        "speedRecoveryFrames": 0,
        "fusionSpeedScale": FUSION_MIN_FORWARD_SPEED_SCALE,
        "pivotDirectionGuard": "NONE",
        "pivotDirectionGuardActive": False,
        "pivotDirectionGuardRejectedOpposite": False,
        "missedFrames": 0,
        "processingMs": max(0.0, float(processing_ms)),
    }


def create_fusion_style_physical_mask(processed_line_mask, envelope):
    """Recorta a máscara usando o mesmo envelope físico das ROIs do Soul."""

    physical_mask = np.zeros_like(processed_line_mask, dtype=np.uint8)
    for y in range(envelope["farY"], envelope["nearY"] + 1):
        left_x, right_x = normal_trajectory_horizontal_bounds(envelope, y)
        physical_mask[y, left_x:right_x] = processed_line_mask[
            y,
            left_x:right_x,
        ]
    return physical_mask


def select_fusion_style_contour(
    physical_mask,
    envelope,
    accepted_contours=None,
):
    """Seleciona o componente que melhor representa a linha próxima ao robô."""

    if accepted_contours is None:
        contours, _ = cv2.findContours(
            physical_mask.copy(),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
    else:
        contours = list(accepted_contours)
    contours = [
        contour
        for contour in contours
        if contour.size > 0 and cv2.contourArea(contour) > 0.0
    ]
    if not contours:
        return None, 0, "none"

    height, width = physical_mask.shape[:2]
    near_band_height = max(
        1,
        int(round(height * FUSION_STYLE_NEAR_BAND_HEIGHT_RATIO)),
    )
    near_band_start_y = max(
        envelope["farY"],
        envelope["nearY"] - near_band_height + 1,
    )

    # O FusionZero dá preferência aos contornos que chegam à região inferior
    # central. Aqui a mesma regra usa o limite inferior calibrado do Soul.
    center_margin_px = min(
        max(0, int(round(3.0 * height / 20.0))),
        max(0, (width - 1) // 2),
    )
    center_x0 = center_margin_px
    center_x1 = max(center_x0 + 1, width - center_margin_px)
    near_contours = []
    for contour in contours:
        contour_points = contour.reshape(-1, 2)
        touches_near_center = np.any(
            (contour_points[:, 1] >= near_band_start_y)
            & (contour_points[:, 0] >= center_x0)
            & (contour_points[:, 0] < center_x1)
        )
        if touches_near_center:
            near_contours.append(contour)

    if near_contours:
        selected_contour = max(near_contours, key=cv2.contourArea)
        selection = "nearCenter"
    else:
        # Sem âncora inferior, o diagnóstico ainda mostra o fragmento que mais
        # se aproxima do robô. Isso não inicia recovery nem comanda os motores.
        selected_contour = max(
            contours,
            key=lambda contour: (
                int(np.max(contour[:, 0, 1])),
                float(cv2.contourArea(contour)),
            ),
        )
        selection = "deepestFallback"

    return selected_contour, len(contours), selection


def create_fusion_style_selected_contour_mask(physical_mask, contour):
    """Mantém somente o preto real pertencente ao contorno selecionado."""

    contour_mask = np.zeros_like(physical_mask, dtype=np.uint8)
    cv2.drawContours(contour_mask, [contour], -1, 255, cv2.FILLED)
    return cv2.bitwise_and(physical_mask, contour_mask)


def fusion_style_band_components(
    band_mask,
    offset_x=0,
    offset_y=0,
    include_transverse_center=False,
):
    """Lista segmentos contínuos e, quando pedido, sua seção transversal."""

    if band_mask.size == 0 or np.count_nonzero(band_mask) == 0:
        return []
    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
        (band_mask > 0).astype(np.uint8),
        connectivity=8,
    )
    components = []
    for label in range(1, component_count):
        x = int(stats[label, cv2.CC_STAT_LEFT])
        y = int(stats[label, cv2.CC_STAT_TOP])
        width = int(stats[label, cv2.CC_STAT_WIDTH])
        height = int(stats[label, cv2.CC_STAT_HEIGHT])
        area = int(stats[label, cv2.CC_STAT_AREA])
        if width <= 0 or height <= 0 or area <= 0:
            continue
        component = {
            "center": (
                offset_x + x + (width - 1) // 2,
                offset_y + y + (height - 1) // 2,
            ),
            "width": width,
            "height": height,
            "area": area,
        }
        if include_transverse_center:
            transverse_sections = []
            component_center_x = x + (width - 1) / 2.0
            for row_y in range(y, y + height):
                row_xs = np.flatnonzero(labels[row_y] == label)
                if row_xs.size == 0:
                    continue
                run_breaks = np.flatnonzero(np.diff(row_xs) > 1)
                run_starts = np.concatenate(([0], run_breaks + 1))
                run_ends = np.concatenate((run_breaks, [row_xs.size - 1]))
                best_row_section = None
                for run_start, run_end in zip(run_starts, run_ends):
                    left_x = int(row_xs[run_start])
                    right_x = int(row_xs[run_end])
                    center_x = (left_x + right_x) // 2
                    section_rank = (
                        right_x - left_x + 1,
                        -abs(center_x - component_center_x),
                    )
                    if (
                        best_row_section is None
                        or section_rank > best_row_section[0]
                    ):
                        best_row_section = (
                            section_rank,
                            {
                                "center": (
                                    offset_x + center_x,
                                    offset_y + row_y,
                                ),
                                "width": right_x - left_x + 1,
                            },
                        )
                if best_row_section is not None:
                    transverse_sections.append(best_row_section[1])
            component["transverseSections"] = transverse_sections
            widest_section = max(
                transverse_sections,
                key=lambda section: (
                    section["width"],
                    section["center"][1],
                ),
                default=None,
            )
            component["transverseCenter"] = (
                widest_section["center"]
                if widest_section is not None
                else None
            )
        components.append(component)
    return components


def calculate_fusion_style_top_contour(
    physical_mask,
    contour,
    selected_contour_mask=None,
):
    """Centraliza o target no segmento preto válido da banda superior."""

    if selected_contour_mask is None:
        selected_contour_mask = create_fusion_style_selected_contour_mask(
            physical_mask,
            contour,
        )
    contour_top_y = int(np.min(contour[:, 0, 1]))
    band_height = max(
        1,
        int(round(
            physical_mask.shape[0] * FUSION_STYLE_TARGET_BAND_HEIGHT_RATIO
        )),
    )
    search_height = max(
        band_height,
        int(round(
            physical_mask.shape[0]
            * FUSION_STYLE_TARGET_SEARCH_HEIGHT_RATIO
        )),
    )
    search_end_y = min(
        physical_mask.shape[0],
        contour_top_y + search_height,
    )
    contour_x, _, contour_width, _ = cv2.boundingRect(contour)
    search_x0 = max(0, int(contour_x))
    search_x1 = min(
        physical_mask.shape[1],
        int(contour_x + contour_width),
    )
    components = fusion_style_band_components(
        selected_contour_mask[
            contour_top_y:search_end_y,
            search_x0:search_x1,
        ],
        offset_x=search_x0,
        offset_y=contour_top_y,
        include_transverse_center=True,
    )
    if not components:
        return None, contour_top_y, min(
            physical_mask.shape[0],
            contour_top_y + band_height,
        )

    image_center_x = physical_mask.shape[1] / 2.0
    selected_component = max(
        components,
        key=lambda component: (
            component["area"],
            -abs(component["center"][0] - image_center_x),
        ),
    )
    transverse_sections = selected_component.get("transverseSections", [])
    for start_index in range(len(transverse_sections) - band_height + 1):
        stable_sections = transverse_sections[
            start_index:start_index + band_height
        ]
        first_y = stable_sections[0]["center"][1]
        last_y = stable_sections[-1]["center"][1]
        if last_y - first_y != band_height - 1:
            continue
        widths = [section["width"] for section in stable_sections]
        width_tolerance_px = max(
            2.0,
            max(widths) * FUSION_STYLE_TRANSVERSE_WIDTH_STABILITY_RATIO,
        )
        if max(widths) - min(widths) <= width_tolerance_px:
            return stable_sections[0]["center"], first_y, last_y + 1

    fallback_target = selected_component.get("transverseCenter")
    if fallback_target is None:
        return None, contour_top_y, min(
            physical_mask.shape[0],
            contour_top_y + band_height,
        )
    fallback_end_y = fallback_target[1] + 1
    fallback_start_y = max(contour_top_y, fallback_end_y - band_height)
    return fallback_target, fallback_start_y, fallback_end_y


def calculate_fusion_style_edge_segment_center(
    physical_mask,
    contour,
    envelope,
    edge_name,
    selected_contour_mask=None,
):
    """Centraliza uma única faixa preta que cruza a banda lateral."""

    if edge_name not in ("leftEdge", "rightEdge"):
        return None

    if selected_contour_mask is None:
        selected_contour_mask = create_fusion_style_selected_contour_mask(
            physical_mask,
            contour,
        )
    edge_margin_px = max(1, int(envelope["width"]) // 16)
    far_y = int(envelope["farY"])
    near_y = int(envelope["nearY"])
    edge_rows = []
    edge_x0 = physical_mask.shape[1]
    edge_x1 = 0
    for y in range(far_y, near_y + 1):
        left_x, right_x = normal_trajectory_horizontal_bounds(envelope, y)
        if edge_name == "leftEdge":
            band_x0 = left_x
            band_x1 = min(right_x, left_x + edge_margin_px + 1)
        else:
            band_x0 = max(left_x, right_x - edge_margin_px - 1)
            band_x1 = right_x
        edge_rows.append((y, band_x0, band_x1))
        edge_x0 = min(edge_x0, band_x0)
        edge_x1 = max(edge_x1, band_x1)

    if edge_x1 <= edge_x0 or near_y < far_y:
        return None

    # A análise usa somente a faixa lateral efetiva. Os offsets mantêm as
    # coordenadas publicadas no mesmo sistema global da imagem completa.
    edge_band_mask = np.zeros(
        (near_y - far_y + 1, edge_x1 - edge_x0),
        dtype=np.uint8,
    )
    for y, band_x0, band_x1 in edge_rows:
        edge_band_mask[
            y - far_y,
            band_x0 - edge_x0:band_x1 - edge_x0,
        ] = selected_contour_mask[
            y,
            band_x0:band_x1,
        ]

    components = fusion_style_band_components(
        edge_band_mask,
        offset_x=edge_x0,
        offset_y=far_y,
    )
    if len(components) != 1:
        return None

    component = components[0]
    # Uma faixa transversal deve ocupar ao menos metade da banda lateral e não
    # pode se prolongar verticalmente como um trecho que apenas acompanha a ROI.
    minimum_transverse_width_px = max(2, edge_margin_px // 2)
    maximum_longitudinal_height_px = max(3, edge_margin_px * 2)
    if (
        component["width"] < minimum_transverse_width_px
        or component["height"] > maximum_longitudinal_height_px
    ):
        return None
    return component["center"]


def calculate_fusion_style_angle(near_point, far_point):
    """Calcula o ângulo bruto entre a âncora inferior e um target Fusion."""

    delta_x = float(near_point[0] - far_point[0])
    delta_y = float(near_point[1] - far_point[1])
    if delta_x == 0.0 and delta_y == 0.0:
        return None
    return math.degrees(math.atan2(delta_y, delta_x))


def fusion_style_previous_target(previous_fusion_line):
    """Lê a observação anterior usada só na recuperação de velocidade."""

    if not isinstance(previous_fusion_line, dict):
        return None
    if previous_fusion_line.get("valid") is not True:
        return None
    try:
        missed_frames = int(previous_fusion_line.get("missedFrames", 0))
        previous_angle = float(previous_fusion_line.get("angleDeg"))
        previous_far = previous_fusion_line.get("farPoint", {})
        previous_far_x = float(previous_far.get("x"))
        previous_far_y = float(previous_far.get("y"))
    except (AttributeError, TypeError, ValueError):
        return None
    if (
        missed_frames < 0
        or missed_frames > FUSION_TARGET_HISTORY_MAX_MISSED_FRAMES
        or not all(math.isfinite(value) for value in (
            previous_angle,
            previous_far_x,
            previous_far_y,
        ))
    ):
        return None
    return {
        "angleDeg": previous_angle,
        "farPoint": (previous_far_x, previous_far_y),
        "missedFrames": missed_frames,
    }


def select_fusion_style_reference_point(
    physical_mask,
    contour,
    top_point,
    envelope,
    selected_contour_mask=None,
    preferred_direction=None,
):
    """Usa apenas a geometria atual: topo distante ou borda em curva de 90°."""

    if top_point is None:
        return None, "none"

    preferred_direction = (
        preferred_direction
        if preferred_direction in ("LEFT", "RIGHT")
        else None
    )
    geometry = envelope["geometry"]
    far_band_end_y = int(geometry["farBand"]["center"]["y1"])
    top_band_available = top_point[1] < far_band_end_y
    if preferred_direction is None and top_band_available:
        return top_point, "topBand"

    width = int(envelope["width"])
    edge_margin_px = max(1, width // 16)
    contour_points = contour.reshape(-1, 2)
    left_edge_points = []
    right_edge_points = []
    for point_x, point_y in contour_points:
        left_x, right_x = normal_trajectory_horizontal_bounds(
            envelope,
            int(point_y),
        )
        if int(point_x) <= left_x + edge_margin_px:
            left_edge_points.append((int(point_x), int(point_y)))
        if int(point_x) >= right_x - 1 - edge_margin_px:
            right_edge_points.append((int(point_x), int(point_y)))

    left_y = (
        int(np.mean([point[1] for point in left_edge_points]))
        if left_edge_points
        else None
    )
    right_y = (
        int(np.mean([point[1] for point in right_edge_points]))
        if right_edge_points
        else None
    )
    if preferred_direction == "LEFT" and left_y is not None:
        centered_point = calculate_fusion_style_edge_segment_center(
            physical_mask,
            contour,
            envelope,
            "leftEdge",
            selected_contour_mask,
        )
        if centered_point is not None:
            return centered_point, "leftEdge"
        left_x, _ = normal_trajectory_horizontal_bounds(envelope, left_y)
        return (left_x, left_y), "leftEdge"
    if preferred_direction == "RIGHT" and right_y is not None:
        centered_point = calculate_fusion_style_edge_segment_center(
            physical_mask,
            contour,
            envelope,
            "rightEdge",
            selected_contour_mask,
        )
        if centered_point is not None:
            return centered_point, "rightEdge"
        _, right_x = normal_trajectory_horizontal_bounds(envelope, right_y)
        return (right_x - 1, right_y), "rightEdge"
    if top_band_available:
        return top_point, "topBand"

    # A preferência GREEN pode usar a continuação frontal como fallback, mas
    # nunca deve atravessar para o edge oposto ao ramo escolhido pelo marcador.
    if preferred_direction is not None:
        return None, "none"

    if left_y is not None and right_y is None:
        centered_point = calculate_fusion_style_edge_segment_center(
            physical_mask,
            contour,
            envelope,
            "leftEdge",
            selected_contour_mask,
        )
        if centered_point is not None:
            return centered_point, "leftEdge"
        left_x, _ = normal_trajectory_horizontal_bounds(envelope, left_y)
        return (left_x, left_y), "leftEdge"
    if right_y is not None and left_y is None:
        centered_point = calculate_fusion_style_edge_segment_center(
            physical_mask,
            contour,
            envelope,
            "rightEdge",
            selected_contour_mask,
        )
        if centered_point is not None:
            return centered_point, "rightEdge"
        _, right_x = normal_trajectory_horizontal_bounds(envelope, right_y)
        return (right_x - 1, right_y), "rightEdge"
    if left_y is not None and right_y is not None:
        # Sem estado temporal, a diferença vertical resolve a direção local.
        # Em empate, o ponto superior continua sendo a referência geométrica.
        if left_y < right_y:
            centered_point = calculate_fusion_style_edge_segment_center(
                physical_mask,
                contour,
                envelope,
                "leftEdge",
                selected_contour_mask,
            )
            if centered_point is not None:
                return centered_point, "leftEdge"
            left_x, _ = normal_trajectory_horizontal_bounds(envelope, left_y)
            return (left_x, left_y), "leftEdge"
        if right_y < left_y:
            centered_point = calculate_fusion_style_edge_segment_center(
                physical_mask,
                contour,
                envelope,
                "rightEdge",
                selected_contour_mask,
            )
            if centered_point is not None:
                return centered_point, "rightEdge"
            _, right_x = normal_trajectory_horizontal_bounds(envelope, right_y)
            return (right_x - 1, right_y), "rightEdge"

    return top_point, "topBand"


def fusion_style_angle_direction(angle_deg):
    """Converte o lado do ângulo Fusion sem alterar sua magnitude."""

    if angle_deg > 90.0:
        return "RIGHT"
    if angle_deg < 90.0:
        return "LEFT"
    return "NONE"


def fusion_style_blind_search_direction(fusion_history):
    """Obtém o último lado Fusion recente para iniciar a busca cega."""

    previous_target = fusion_style_previous_target(fusion_history)
    if previous_target is None:
        return None

    angle_deg = previous_target["angleDeg"]
    if (
        not 0.0 <= angle_deg <= 180.0
        or abs(angle_deg - 90.0) <= FUSION_STEERING_DEADBAND_DEG
    ):
        return None
    return fusion_style_angle_direction(angle_deg)


def fusion_style_has_forward_target(result, envelope):
    """Confirma que o target atual voltou à região frontal distante."""

    if result.get("referenceSource") != "topBand":
        return False
    far_point = result.get("farPoint")
    if not isinstance(far_point, dict):
        return False
    try:
        far_y = float(far_point.get("y"))
    except (TypeError, ValueError):
        return False
    far_band_end_y = float(
        envelope["geometry"]["farBand"]["center"]["y1"]
    )
    return math.isfinite(far_y) and far_y < far_band_end_y


def apply_fusion_extreme_pivot_direction_guard(
    result,
    previous_fusion_line,
    envelope,
):
    """Impede somente a inversão espúria durante um pivot Fusion extremo."""

    angle_deg = finite_virtual_position(result.get("angleDeg"))
    if angle_deg is None:
        return

    angle_error_deg = angle_deg - 90.0
    current_direction = fusion_style_angle_direction(angle_deg)
    previous_guard_active = (
        isinstance(previous_fusion_line, dict)
        and previous_fusion_line.get("valid") is True
        and previous_fusion_line.get("missedFrames", 0) == 0
        and previous_fusion_line.get("pivotDirectionGuardActive") is True
    )
    previous_direction = (
        str(previous_fusion_line.get("pivotDirectionGuard", "NONE"))
        if previous_guard_active
        else "NONE"
    )
    if previous_direction not in ("LEFT", "RIGHT"):
        previous_guard_active = False

    if previous_guard_active:
        if (
            abs(angle_error_deg)
            <= FUSION_EXTREME_PIVOT_GUARD_RELEASE_ERROR_DEG
        ):
            return

        result["pivotDirectionGuard"] = previous_direction
        result["pivotDirectionGuardActive"] = True
        if current_direction == previous_direction:
            return

        previous_target = fusion_style_previous_target(
            previous_fusion_line
        )
        if previous_target is None:
            result["pivotDirectionGuard"] = "NONE"
            result["pivotDirectionGuardActive"] = False
            return

        # Durante o pivot, uma troca direta de lado usa somente o último target
        # extremo aceito. Ausência de target continua seguindo o fallback atual.
        previous_far_x, previous_far_y = previous_target["farPoint"]
        result["angleDeg"] = previous_target["angleDeg"]
        result["farPoint"] = {
            "x": int(round(previous_far_x)),
            "y": int(round(previous_far_y)),
        }
        result["referenceSource"] = str(
            previous_fusion_line.get("referenceSource", "none")
        )
        result["pivotDirectionGuardRejectedOpposite"] = True
        return

    if (
        current_direction in ("LEFT", "RIGHT")
        and abs(angle_error_deg)
        >= FUSION_EXTREME_PIVOT_GUARD_ENTER_ERROR_DEG
    ):
        result["pivotDirectionGuard"] = current_direction
        result["pivotDirectionGuardActive"] = True


def update_fusion_style_target_metrics(
    result,
    previous_fusion_line,
    near_point,
    far_point,
    envelope,
):
    """Mede estabilidade do target sem alterar o ângulo escolhido no frame."""

    forward_span_px = max(
        1.0,
        float(envelope["nearY"] - envelope["farY"]),
    )
    target_length_ratio = max(
        0.0,
        min(
            1.0,
            math.hypot(
                float(far_point[0] - near_point[0]),
                float(far_point[1] - near_point[1]),
            ) / forward_span_px,
        ),
    )
    previous_target = fusion_style_previous_target(previous_fusion_line)
    continuous_observation = (
        previous_target is not None
        and previous_target["missedFrames"] == 0
    )
    target_consistency = 0.0
    target_stable_frames = 1
    if continuous_observation:
        angle_delta_deg = abs(
            float(result["angleDeg"]) - previous_target["angleDeg"]
        )
        target_shift_ratio = math.hypot(
            float(far_point[0]) - previous_target["farPoint"][0],
            float(far_point[1]) - previous_target["farPoint"][1],
        ) / max(1.0, float(envelope["width"]))
        angle_consistency = 1.0 - min(
            1.0,
            angle_delta_deg / FUSION_TARGET_CONSISTENCY_ANGLE_SCALE_DEG,
        )
        position_consistency = 1.0 - min(
            1.0,
            target_shift_ratio
            / FUSION_TARGET_CONSISTENCY_SHIFT_WIDTH_RATIO,
        )
        target_consistency = (
            0.70 * angle_consistency + 0.30 * position_consistency
        )
        if (
            angle_delta_deg <= FUSION_TARGET_STABLE_ANGLE_DELTA_DEG
            and target_shift_ratio
            <= FUSION_TARGET_STABLE_SHIFT_WIDTH_RATIO
        ):
            try:
                previous_stable_frames = int(
                    previous_fusion_line.get("targetStableFrames", 0)
                )
            except (AttributeError, TypeError, ValueError):
                previous_stable_frames = 0
            target_stable_frames = max(1, previous_stable_frames + 1)

    current_error_deg = abs(float(result["angleDeg"]) - 90.0)
    if current_error_deg >= FUSION_STRONG_CORRECTION_ERROR_DEG:
        speed_recovery_frames = 0
    elif continuous_observation:
        try:
            previous_recovery_frames = int(
                previous_fusion_line.get("speedRecoveryFrames", 0)
            )
        except (AttributeError, TypeError, ValueError):
            previous_recovery_frames = 0
        speed_recovery_frames = min(
            FUSION_TARGET_STABLE_FRAMES_FOR_FULL_SPEED,
            max(0, previous_recovery_frames) + 1,
        )
    else:
        speed_recovery_frames = 1

    length_confidence = max(
        0.0,
        min(
            1.0,
            (
                target_length_ratio - FUSION_TARGET_SHORT_LENGTH_RATIO
            ) / (
                FUSION_TARGET_ESTABLISHED_LENGTH_RATIO
                - FUSION_TARGET_SHORT_LENGTH_RATIO
            ),
        ),
    )
    stability_confidence = min(
        1.0,
        target_stable_frames
        / FUSION_TARGET_STABLE_FRAMES_FOR_FULL_SPEED,
    )
    recovery_confidence = min(
        1.0,
        speed_recovery_frames
        / FUSION_TARGET_STABLE_FRAMES_FOR_FULL_SPEED,
    )
    future_target_confidence = min(
        length_confidence,
        target_consistency,
        stability_confidence,
        recovery_confidence,
    )
    result.update({
        "targetLengthRatio": round(target_length_ratio, 4),
        "targetConsistency": round(target_consistency, 4),
        "targetStableFrames": target_stable_frames,
        "targetReacquired": not continuous_observation,
        "speedRecoveryFrames": speed_recovery_frames,
        "fusionSpeedScale": round(
            FUSION_MIN_FORWARD_SPEED_SCALE
            + (1.0 - FUSION_MIN_FORWARD_SPEED_SCALE)
            * future_target_confidence,
            4,
        ),
    })


def extract_fusion_style_line(
    processed_line_mask,
    previous_fusion_line=None,
    accepted_contours=None,
    preferred_direction=None,
):
    """Extrai o target atual e mede confiança para recuperar velocidade."""

    extraction_started = time.perf_counter()
    if (
        not isinstance(processed_line_mask, np.ndarray)
        or processed_line_mask.ndim != 2
        or processed_line_mask.size == 0
    ):
        return empty_fusion_style_line(
            (time.perf_counter() - extraction_started) * 1000.0
        )

    envelope = resolve_normal_trajectory_envelope(processed_line_mask.shape)
    if envelope is None:
        return empty_fusion_style_line(
            (time.perf_counter() - extraction_started) * 1000.0
        )

    near_position = envelope["geometry"]["near"]["position"]
    near_x = max(
        int(near_position["x0"]),
        min(int(near_position["x1"]) - 1, processed_line_mask.shape[1] // 2),
    )
    near_point = (near_x, int(envelope["nearY"]))
    result = empty_fusion_style_line()
    fusion_preferred_direction = (
        preferred_direction
        if preferred_direction in ("LEFT", "RIGHT")
        else "NONE"
    )
    result.update({
        "enabled": True,
        "nearPoint": {"x": near_point[0], "y": near_point[1]},
        "preferredDirection": fusion_preferred_direction,
    })

    physical_mask = create_fusion_style_physical_mask(
        processed_line_mask,
        envelope,
    )
    reusable_contours = None
    if (
        accepted_contours is not None
        and np.array_equal(physical_mask, processed_line_mask)
    ):
        # Os contornos anteriores são equivalentes somente quando o envelope
        # não recortou nenhum pixel. Se houve recorte, o fallback preserva a
        # separação e a geometria produzidas pelo findContours original.
        reusable_contours = accepted_contours
    contour, contour_count, selection = select_fusion_style_contour(
        physical_mask,
        envelope,
        reusable_contours,
    )
    result["candidateContourCount"] = contour_count
    result["selection"] = selection
    if contour is None:
        result["processingMs"] = max(
            0.0,
            (time.perf_counter() - extraction_started) * 1000.0,
        )
        return result

    selected_contour_mask = create_fusion_style_selected_contour_mask(
        physical_mask,
        contour,
    )
    top_point, top_y, band_end_y = calculate_fusion_style_top_contour(
        physical_mask,
        contour,
        selected_contour_mask,
    )
    result["contourAreaPx"] = round(float(cv2.contourArea(contour)), 2)
    result["topBand"] = {
        "y0": top_y,
        "y1": band_end_y,
    }
    if top_point is not None:
        result["topBandPoint"] = {
            "x": top_point[0],
            "y": top_point[1],
        }
    far_point, reference_source = select_fusion_style_reference_point(
        physical_mask,
        contour,
        top_point,
        envelope,
        selected_contour_mask,
        fusion_preferred_direction,
    )
    result["referenceSource"] = reference_source
    if far_point is not None:
        angle_deg = calculate_fusion_style_angle(near_point, far_point)
        if angle_deg is not None:
            # A convenção publicada deixa 90° como reto, valores menores para
            # a esquerda e valores maiores para a direita.
            result.update({
                "valid": True,
                "angleDeg": round(float(angle_deg), 2),
                "farPoint": {"x": far_point[0], "y": far_point[1]},
            })
            apply_fusion_extreme_pivot_direction_guard(
                result,
                previous_fusion_line,
                envelope,
            )
            guarded_far_point = result["farPoint"]
            update_fusion_style_target_metrics(
                result,
                previous_fusion_line,
                near_point,
                (
                    int(guarded_far_point["x"]),
                    int(guarded_far_point["y"]),
                ),
                envelope,
            )

    result["processingMs"] = max(
        0.0,
        (time.perf_counter() - extraction_started) * 1000.0,
    )
    return result


def extract_line_diagnostics(
    processed_line_mask,
    camera_role,
    legacy_debug_enabled=False,
    previous_fusion_line=None,
    accepted_contours=None,
    preferred_direction=None,
):
    """Executa somente os diagnósticos habilitados para a câmera inferior."""

    normal_trajectory = empty_normal_trajectory()
    fusion_style_line = empty_fusion_style_line()
    if camera_role != "down":
        return normal_trajectory, fusion_style_line

    if legacy_debug_enabled:
        # O extractor por scanlines continua disponível para comparação, mas
        # fica totalmente fora do hot path quando o debug legado está inativo.
        normal_trajectory = extract_normal_line_trajectory(
            processed_line_mask
        )

    # O Fusion-style fornece o steering NORMAL; o legado continua opcional e
    # serve como diagnóstico ou fallback quando o target Fusion fica inválido.
    fusion_style_line = extract_fusion_style_line(
        processed_line_mask,
        previous_fusion_line,
        accepted_contours,
        preferred_direction,
    )
    return normal_trajectory, fusion_style_line


def update_fusion_style_history(previous_fusion_line, fusion_style_line):
    """Mantém somente a última observação válida e a idade da interrupção."""

    if (
        isinstance(fusion_style_line, dict)
        and fusion_style_line.get("valid") is True
    ):
        return dict(fusion_style_line)
    if (
        not isinstance(previous_fusion_line, dict)
        or previous_fusion_line.get("valid") is not True
    ):
        return None
    history = dict(previous_fusion_line)
    try:
        missed_frames = int(history.get("missedFrames", 0))
    except (TypeError, ValueError):
        missed_frames = 0
    history["missedFrames"] = max(0, missed_frames) + 1
    return history


def draw_fusion_style_line_overlay(
    frame,
    fusion_style_line,
    line_follower_command=None,
):
    """Desenha somente as informações necessárias para validar o Fusion-style."""

    if not isinstance(fusion_style_line, dict):
        return

    angle_color = (255, 0, 255)
    reference_color = (230, 230, 230)
    near_point_data = fusion_style_line.get("nearPoint")
    far_point_data = fusion_style_line.get("farPoint")
    near_point = None
    far_point = None
    try:
        if isinstance(near_point_data, dict):
            near_point = (
                int(near_point_data["x"]),
                int(near_point_data["y"]),
            )
        if isinstance(far_point_data, dict):
            far_point = (
                int(far_point_data["x"]),
                int(far_point_data["y"]),
            )
    except (KeyError, TypeError, ValueError):
        near_point = None
        far_point = None

    if near_point is not None:
        reference_end_y = max(0, near_point[1] - 42)
        cv2.line(
            frame,
            near_point,
            (near_point[0], reference_end_y),
            reference_color,
            1,
            cv2.LINE_AA,
        )
        cv2.circle(frame, near_point, 5, (0, 0, 0), -1, cv2.LINE_AA)
        cv2.circle(frame, near_point, 3, reference_color, -1, cv2.LINE_AA)

    if near_point is not None and far_point is not None:
        # O magenta mantém alvo e vetor reconhecíveis sobre a máscara ou o piso.
        cv2.line(frame, near_point, far_point, (0, 0, 0), 5, cv2.LINE_AA)
        cv2.line(frame, near_point, far_point, angle_color, 3, cv2.LINE_AA)
        cv2.circle(frame, far_point, 7, (0, 0, 0), -1, cv2.LINE_AA)
        cv2.circle(frame, far_point, 5, angle_color, -1, cv2.LINE_AA)
        target_text_origin = (
            min(frame.shape[1] - 76, max(4, far_point[0] + 9)),
            max(14, far_point[1] - 9),
        )
        cv2.putText(
            frame,
            "F TARGET",
            target_text_origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            "F TARGET",
            target_text_origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            angle_color,
            1,
            cv2.LINE_AA,
        )

    angle_value = fusion_style_line.get("angleDeg")
    try:
        safe_angle = float(angle_value)
    except (TypeError, ValueError):
        safe_angle = float("nan")
    angle_text = (
        f"FusionAngle {safe_angle:.1f} deg"
        if math.isfinite(safe_angle)
        else "FusionAngle --"
    )
    try:
        processing_ms = float(fusion_style_line.get("processingMs", 0.0))
    except (TypeError, ValueError):
        processing_ms = 0.0
    if not math.isfinite(processing_ms) or processing_ms < 0.0:
        processing_ms = 0.0
    validity_text = (
        "Fusion VALID"
        if fusion_style_line.get("valid") is True
        else "Fusion INVALID"
    )
    validity_text = f"{validity_text} {processing_ms:.2f} ms"

    command = (
        line_follower_command
        if isinstance(line_follower_command, dict)
        else {}
    )
    control_source = str(command.get("controlSource", "--")).strip() or "--"
    source_text = f"Source {control_source}"
    try:
        left_power = float(command.get("left_power"))
        right_power = float(command.get("right_power"))
    except (TypeError, ValueError):
        left_power = float("nan")
        right_power = float("nan")
    powers_text = (
        f"Powers L {left_power:+.2f} R {right_power:+.2f}"
        if math.isfinite(left_power) and math.isfinite(right_power)
        else "Powers L -- R --"
    )

    overlay_texts = (
        angle_text,
        validity_text,
        source_text,
        powers_text,
    )
    for line_index, overlay_text in enumerate(overlay_texts):
        text_origin = (8, 20 + line_index * 18)
        cv2.putText(
            frame,
            overlay_text,
            text_origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            overlay_text,
            text_origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            angle_color,
            1,
            cv2.LINE_AA,
        )


def fusion_target_is_valid(point):
    """Confirma que o target Fusion existe e contém coordenadas finitas."""

    if not isinstance(point, dict):
        return False
    point_x = finite_virtual_position(point.get("x"))
    point_y = finite_virtual_position(point.get("y"))
    return point_x is not None and point_y is not None


def map_fusion_angle_to_steering_error(fusion_angle):
    """Converte o ângulo Fusion em steering contínuo sem memória temporal."""

    fusion_angle = finite_virtual_position(fusion_angle)
    if fusion_angle is None or not 0.0 <= fusion_angle <= 180.0:
        return None

    angle_error_deg = fusion_angle - 90.0
    if abs(angle_error_deg) <= FUSION_STEERING_DEADBAND_DEG:
        return 0.0

    angle_error_magnitude_deg = abs(angle_error_deg)
    if angle_error_magnitude_deg < FUSION_FULL_NORMAL_STEERING_DEG:
        normal_angle_span_deg = max(
            1.0,
            FUSION_FULL_NORMAL_STEERING_DEG
            - FUSION_STEERING_DEADBAND_DEG,
        )
        steering_progress = (
            angle_error_magnitude_deg - FUSION_STEERING_DEADBAND_DEG
        ) / normal_angle_span_deg
        # O smoothstep mantém inclinação zero junto à deadband e ao limite
        # NORMAL, evitando um degrau quando começa a faixa de pivot.
        steering_strength = (
            steering_progress
            * steering_progress
            * (3.0 - 2.0 * steering_progress)
        )
        steering_magnitude = (
            steering_strength * NORMAL_FULL_STEERING_ERROR
        )
    else:
        pivot_angle_span_deg = max(
            1.0,
            FUSION_FULL_PIVOT_STEERING_DEG
            - FUSION_FULL_NORMAL_STEERING_DEG,
        )
        pivot_progress = min(
            1.0,
            (
                angle_error_magnitude_deg
                - FUSION_FULL_NORMAL_STEERING_DEG
            ) / pivot_angle_span_deg,
        )
        pivot_strength = (
            pivot_progress
            * pivot_progress
            * (3.0 - 2.0 * pivot_progress)
        )
        steering_magnitude = (
            NORMAL_FULL_STEERING_ERROR
            + pivot_strength * (1.0 - NORMAL_FULL_STEERING_ERROR)
        )

    steering_error = math.copysign(steering_magnitude, angle_error_deg)
    return max(-1.0, min(1.0, steering_error))


def calculate_fusion_control_status(fusion_style_line):
    """Valida a geometria Fusion e prepara sua telemetria de controle."""

    result = {
        "fusionAngle": None,
        "filteredFusionAngle": None,
        "fusionSteeringError": None,
        "fusionControlActive": False,
        "fusionTargetLengthRatio": 0.0,
        "fusionTargetConsistency": 0.0,
        "fusionTargetStableFrames": 0,
        "fusionTargetReacquired": False,
        "fusionSpeedScale": FUSION_MIN_FORWARD_SPEED_SCALE,
    }
    if not isinstance(fusion_style_line, dict):
        return result

    fusion_angle = finite_virtual_position(fusion_style_line.get("angleDeg"))
    result["fusionAngle"] = fusion_angle
    if fusion_angle is None or not 0.0 <= fusion_angle <= 180.0:
        return result

    # Nesta primeira etapa não há filtro temporal: o valor filtrado é uma cópia
    # direta do ângulo validado para deixar explícita essa decisão na telemetria.
    result["filteredFusionAngle"] = fusion_angle
    fusion_geometry_valid = (
        fusion_style_line.get("valid") is True
        and fusion_style_line.get("selection") == "nearCenter"
        and fusion_target_is_valid(fusion_style_line.get("nearPoint"))
        and fusion_target_is_valid(fusion_style_line.get("farPoint"))
    )
    if not fusion_geometry_valid:
        return result

    result["fusionSteeringError"] = map_fusion_angle_to_steering_error(
        fusion_angle
    )
    result["fusionTargetLengthRatio"] = max(
        0.0,
        min(
            1.0,
            finite_virtual_position(
                fusion_style_line.get("targetLengthRatio")
            ) or 0.0,
        ),
    )
    result["fusionTargetConsistency"] = max(
        0.0,
        min(
            1.0,
            finite_virtual_position(
                fusion_style_line.get("targetConsistency")
            ) or 0.0,
        ),
    )
    try:
        result["fusionTargetStableFrames"] = max(
            0,
            int(fusion_style_line.get("targetStableFrames", 0)),
        )
    except (TypeError, ValueError):
        result["fusionTargetStableFrames"] = 0
    result["fusionTargetReacquired"] = (
        fusion_style_line.get("targetReacquired") is True
    )
    speed_scale = finite_virtual_position(
        fusion_style_line.get("fusionSpeedScale")
    )
    if speed_scale is not None:
        result["fusionSpeedScale"] = max(
            FUSION_MIN_FORWARD_SPEED_SCALE,
            min(1.0, speed_scale),
        )
    return result


def map_normal_steering_error(steering_error):
    """Converte erro lateral somente no intervalo de autoridade NORMAL."""

    steering_error = finite_virtual_position(steering_error)
    if steering_error is None:
        return None

    limited_error = max(
        -NORMAL_FULL_STEERING_ERROR,
        min(NORMAL_FULL_STEERING_ERROR, steering_error),
    )
    steering_strength = min(
        1.0,
        abs(limited_error) / NORMAL_FULL_STEERING_ERROR,
    )
    outer_power = (
        NORMAL_BASE_POWER
        + steering_strength * (NORMAL_MAX_POWER - NORMAL_BASE_POWER)
    )
    inner_power = (
        NORMAL_BASE_POWER
        - steering_strength
        * (NORMAL_BASE_POWER - NORMAL_INNER_MIN_POWER)
    )
    if limited_error > 0.0:
        return {
            "left_power": outer_power,
            "right_power": inner_power,
        }
    if limited_error < 0.0:
        return {
            "left_power": inner_power,
            "right_power": outer_power,
        }
    return {
        "left_power": NORMAL_BASE_POWER,
        "right_power": NORMAL_BASE_POWER,
    }


def map_fusion_angle_to_motor_powers(fusion_angle):
    """Converte o ângulo Fusion na curva contínua de potência do Soul."""

    fusion_angle = finite_virtual_position(fusion_angle)
    if fusion_angle is None or not 0.0 <= fusion_angle <= 180.0:
        return None

    angle_error_deg = fusion_angle - 90.0
    angle_error_magnitude_deg = abs(angle_error_deg)
    lower_error_deg, lower_outer_power, lower_inner_power = (
        FUSION_POWER_CURVE[0]
    )
    outer_power = lower_outer_power
    inner_power = lower_inner_power

    for upper_point in FUSION_POWER_CURVE[1:]:
        upper_error_deg, upper_outer_power, upper_inner_power = upper_point
        if angle_error_magnitude_deg <= upper_error_deg:
            angle_span_deg = max(1.0, upper_error_deg - lower_error_deg)
            transition_progress = (
                angle_error_magnitude_deg - lower_error_deg
            ) / angle_span_deg
            transition_progress = max(
                0.0,
                min(1.0, transition_progress),
            )
            # Inclinação zero nas duas pontas evita degraus entre as faixas.
            transition_strength = (
                transition_progress
                * transition_progress
                * (3.0 - 2.0 * transition_progress)
            )
            outer_power = (
                lower_outer_power
                + transition_strength
                * (upper_outer_power - lower_outer_power)
            )
            inner_power = (
                lower_inner_power
                + transition_strength
                * (upper_inner_power - lower_inner_power)
            )
            break

        lower_error_deg = upper_error_deg
        lower_outer_power = upper_outer_power
        lower_inner_power = upper_inner_power
        outer_power = upper_outer_power
        inner_power = upper_inner_power

    outer_power = max(-1.0, min(1.0, outer_power))
    inner_power = max(-1.0, min(1.0, inner_power))

    if angle_error_deg > 0.0:
        return {
            "left_power": outer_power,
            "right_power": inner_power,
        }
    if angle_error_deg < 0.0:
        return {
            "left_power": inner_power,
            "right_power": outer_power,
        }
    return {
        "left_power": NORMAL_BASE_POWER,
        "right_power": NORMAL_BASE_POWER,
    }


def apply_fusion_forward_speed_limit(command, speed_scale):
    """Limita apenas o avanço médio e preserva o diferencial Fusion."""

    if not isinstance(command, dict):
        return None
    left_power = finite_virtual_position(command.get("left_power"))
    right_power = finite_virtual_position(command.get("right_power"))
    speed_scale = finite_virtual_position(speed_scale)
    if left_power is None or right_power is None or speed_scale is None:
        return None

    safe_scale = max(
        FUSION_MIN_FORWARD_SPEED_SCALE,
        min(1.0, speed_scale),
    )
    forward_mean = (left_power + right_power) / 2.0
    maximum_forward_mean = NORMAL_BASE_POWER * safe_scale
    if forward_mean > maximum_forward_mean:
        # Subtrair o mesmo valor das duas rodas mantém o yaw produzido pelo
        # mapper e reduz somente a componente que empurra o robô para frente.
        forward_reduction = forward_mean - maximum_forward_mean
        left_power -= forward_reduction
        right_power -= forward_reduction

    return {
        "left_power": max(-1.0, min(1.0, left_power)),
        "right_power": max(-1.0, min(1.0, right_power)),
    }


def green_direction_to_search_direction(direction):
    """Converte a direção do GREEN para o vocabulário usado pela busca cega."""

    return {
        "ESQUERDA": "LEFT",
        "DIREITA": "RIGHT",
    }.get(direction)


def select_confirmed_green_direction(
    armed,
    active_direction,
    green_status,
):
    """Seleciona um GREEN confirmado somente quando o controle está armado."""

    if (
        not armed
        or active_direction != "NENHUMA"
        or not green_status.get("greenConfirmed", False)
    ):
        return None

    interpretation = green_status.get("greenInterpretation")
    if interpretation in ("ESQUERDA", "DIREITA"):
        return interpretation
    return None


def capture_valid_fusion_command(command):
    """Copia somente a decisão Fusion válida que pode alimentar o hold curto."""

    if not isinstance(command, dict):
        return None
    if (
        command.get("controlSource") != "fusion"
        or command.get("fusionControlActive") is not True
    ):
        return None

    try:
        left_power = float(command["left_power"])
        right_power = float(command["right_power"])
    except (KeyError, TypeError, ValueError):
        return None
    if not all(
        math.isfinite(power) and -1.0 <= power <= 1.0
        for power in (left_power, right_power)
    ):
        return None

    snapshot = {
        "left_power": left_power,
        "right_power": right_power,
        "controlSource": "fusion",
        "fusionControlActive": True,
    }
    for field_name in (
        "steeringError",
        "finalSteering",
        "fusionAngle",
        "filteredFusionAngle",
        "fusionSteeringError",
        "fusionSpeedScale",
    ):
        if field_name in command:
            snapshot[field_name] = command[field_name]
    return snapshot


def apply_green_candidate_fusion_hold(
    current_command,
    previous_fusion_command,
    armed,
    active_direction,
    green_status,
    hold_frames,
    hold_blocked,
):
    """Preserva brevemente o último Fusion enquanto um GREEN aguarda decisão."""

    status = green_status if isinstance(green_status, dict) else {}
    try:
        candidate_count = int(status.get("greenCandidateCount", 0))
    except (TypeError, ValueError):
        candidate_count = 0
    candidate_pending = (
        bool(armed)
        and active_direction == "NENHUMA"
        and candidate_count > 0
        and status.get("greenConfirmed") is not True
        and status.get("greenRawInterpretation") != "VERDE_FALSO"
    )
    if not candidate_pending:
        return {
            "command": current_command,
            "active": False,
            "frames": 0,
            "blocked": False,
        }

    safe_hold_frames = max(0, int(hold_frames))
    if hold_blocked or previous_fusion_command is None:
        return {
            "command": current_command,
            "active": False,
            "frames": min(
                safe_hold_frames,
                GREEN_CANDIDATE_HOLD_MAX_FRAMES,
            ),
            "blocked": True,
        }
    if safe_hold_frames >= GREEN_CANDIDATE_HOLD_MAX_FRAMES:
        return {
            "command": current_command,
            "active": False,
            "frames": GREEN_CANDIDATE_HOLD_MAX_FRAMES,
            "blocked": True,
        }

    held_command = dict(current_command)
    held_command.update(previous_fusion_command)
    next_hold_frames = safe_hold_frames + 1
    return {
        "command": held_command,
        "active": True,
        "frames": next_hold_frames,
        "blocked": (
            next_hold_frames >= GREEN_CANDIDATE_HOLD_MAX_FRAMES
        ),
    }


def update_green_fusion_target_hold(
    active_direction,
    fusion_style_line,
    previous_valid_fusion_line,
    missing_frames,
):
    """Tolera uma perda curta do target e depois solicita recovery direcional."""

    current_line = (
        fusion_style_line
        if isinstance(fusion_style_line, dict)
        else empty_fusion_style_line()
    )
    if active_direction not in ("ESQUERDA", "DIREITA"):
        return {
            "fusionLine": current_line,
            "previousValidFusionLine": None,
            "missingFrames": 0,
            "holdActive": False,
            "recoveryDirection": None,
        }

    if current_line.get("valid") is True:
        return {
            "fusionLine": current_line,
            "previousValidFusionLine": dict(current_line),
            "missingFrames": 0,
            "holdActive": False,
            "recoveryDirection": None,
        }

    next_missing_frames = max(0, int(missing_frames)) + 1
    if (
        isinstance(previous_valid_fusion_line, dict)
        and previous_valid_fusion_line.get("valid") is True
        and next_missing_frames <= GREEN_FUSION_TARGET_HOLD_MAX_FRAMES
    ):
        held_line = dict(previous_valid_fusion_line)
        held_line["processingMs"] = current_line.get(
            "processingMs",
            held_line.get("processingMs", 0.0),
        )
        return {
            "fusionLine": held_line,
            "previousValidFusionLine": previous_valid_fusion_line,
            "missingFrames": next_missing_frames,
            "holdActive": True,
            "recoveryDirection": None,
        }

    if next_missing_frames <= GREEN_FUSION_TARGET_HOLD_MAX_FRAMES:
        return {
            "fusionLine": current_line,
            "previousValidFusionLine": previous_valid_fusion_line,
            "missingFrames": next_missing_frames,
            "holdActive": False,
            "recoveryDirection": None,
        }

    return {
        "fusionLine": current_line,
        "previousValidFusionLine": None,
        "missingFrames": 0,
        "holdActive": False,
        "recoveryDirection": green_direction_to_search_direction(
            active_direction
        ),
    }


def update_green_rearm_state(
    armed,
    clear_frames,
    active_direction,
    candidate_count,
):
    """Rearma o GREEN após cinco quadros limpos fora da manobra ativa."""

    if armed:
        return True, 0
    if active_direction != "NENHUMA":
        return False, 0
    if candidate_count > 0:
        return False, 0

    clear_frames += 1
    if clear_frames >= QUADROS_PARA_REARMAR_VERDE:
        return True, 0
    return False, clear_frames


def update_green_control_telemetry(
    green_status,
    active_direction,
    armed,
    rearm_clear_frames,
):
    """Distingue a detecção visual da ativação efetiva no controle."""

    control_active = active_direction != "NENHUMA"
    green_status["greenControlActive"] = control_active
    green_status["greenControlDirection"] = active_direction
    green_status["greenArmed"] = bool(armed)
    green_status["greenRearmClearFrames"] = int(rearm_clear_frames)

    if control_active:
        green_status["greenDecisionState"] = "active"
    elif green_status.get("greenConfirmed", False):
        green_status["greenDecisionState"] = "detected"
    elif green_status.get("greenRawInterpretation") != "SEM_DECISAO":
        green_status["greenDecisionState"] = "candidate"
    else:
        green_status["greenDecisionState"] = "idle"


def green_maneuver_is_geometrically_complete(
    curva_verde_iniciada,
    near_fine_position,
    medium_trusted,
    medium_position,
    far_trusted,
    far_position,
):
    """Confirma o alinhamento local e frontal ao final da manobra verde."""

    if not curva_verde_iniciada:
        return False

    near_position = finite_virtual_position(near_fine_position)
    if (
        near_position is None
        or abs(near_position) > LIMIAR_CENTRALIZACAO_VERDE
    ):
        return False

    # MEDIUM é a referência frontal mais próxima. FAR assume somente quando
    # MEDIUM não possui uma leitura trusted válida neste frame.
    trusted_position = (
        finite_virtual_position(medium_position)
        if medium_trusted
        else None
    )
    if trusted_position is None and far_trusted:
        trusted_position = finite_virtual_position(far_position)

    return (
        trusted_position is not None
        and abs(trusted_position) <= GREEN_TRUSTED_POSITION_CENTER_LIMIT
    )
