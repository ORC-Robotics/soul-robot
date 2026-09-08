"""Geometria e leitura dos sensores virtuais."""

import math
from functools import lru_cache
import cv2  # type: ignore
import numpy as np
from .camera_config import (
    FAR_TRUST_MIN_CONFIDENCE,
    FAR_TRUST_MIN_THICKNESS_PX,
    LINE_TRUST_ABSOLUTE_THIN_VETO_PX,
    MEDIUM_TRUST_MIN_CONFIDENCE,
    MEDIUM_TRUST_MIN_THICKNESS_PX,
    VIRTUAL_CENTER_X0,
    VIRTUAL_CENTER_X1,
    VIRTUAL_FAR_BAND_Y0,
    VIRTUAL_FAR_BAND_Y1,
    VIRTUAL_FAR_CENTER_X0,
    VIRTUAL_FAR_CENTER_X1,
    VIRTUAL_FAR_LEFT_X0,
    VIRTUAL_FAR_LEFT_X1,
    VIRTUAL_FAR_RIGHT_X0,
    VIRTUAL_FAR_RIGHT_X1,
    VIRTUAL_FAR_Y0,
    VIRTUAL_FAR_Y1,
    VIRTUAL_FINE_CENTER_DEADBAND,
    VIRTUAL_HEADING_FULL_SCALE_DEG,
    VIRTUAL_HEADING_GAIN,
    VIRTUAL_LEFT_X0,
    VIRTUAL_LEFT_X1,
    VIRTUAL_LINE_CONFIDENCE_AREA_WEIGHT,
    VIRTUAL_LINE_CONFIDENCE_CONSISTENCY_WEIGHT,
    VIRTUAL_LINE_CONFIDENCE_CONTINUITY_WEIGHT,
    VIRTUAL_LINE_CONFIDENCE_REFERENCE_HEIGHT_PX,
    VIRTUAL_LINE_CONFIDENCE_REFERENCE_WIDTH_PX,
    VIRTUAL_LINE_CONFIDENCE_SAMPLE_STEP_PX,
    VIRTUAL_LINE_CONFIDENCE_THICKNESS_WEIGHT,
    VIRTUAL_LINE_CONSISTENCY_MAX_RELATIVE_DISPERSION,
    VIRTUAL_LINE_EXPECTED_THICKNESS_BOTTOM_PX,
    VIRTUAL_LINE_EXPECTED_THICKNESS_TOP_PX,
    VIRTUAL_LINE_THICKNESS_CORE_PERCENTILE,
    VIRTUAL_MEDIUM_CENTER_X0,
    VIRTUAL_MEDIUM_CENTER_X1,
    VIRTUAL_MEDIUM_LEFT_X0,
    VIRTUAL_MEDIUM_LEFT_X1,
    VIRTUAL_MEDIUM_RIGHT_X0,
    VIRTUAL_MEDIUM_RIGHT_X1,
    VIRTUAL_MEDIUM_WING_Y0,
    VIRTUAL_MEDIUM_WING_Y1,
    VIRTUAL_MEDIUM_Y0,
    VIRTUAL_MEDIUM_Y1,
    VIRTUAL_NEAR_Y0,
    VIRTUAL_NEAR_Y1,
    VIRTUAL_RIGHT_X0,
    VIRTUAL_RIGHT_X1,
    VIRTUAL_ROW_MIN_ACTIVATION,
)
from .numeric import (
    finite_virtual_position,
)

@lru_cache(maxsize=8)
def cached_virtual_sensor_geometry(height, width):
    """
    Converte FAR, FAR BAND, MEDIUM e o único sensor NEAR-C para pixels.

    O retângulo FAR original é calculado diretamente para preservar
    exatamente a leitura usada pelo seguidor base.
    """

    far_y0 = int(round(height * VIRTUAL_FAR_Y0))
    far_y1 = int(round(height * VIRTUAL_FAR_Y1))

    far_band_y0 = int(round(height * VIRTUAL_FAR_BAND_Y0))
    far_band_y1 = int(round(height * VIRTUAL_FAR_BAND_Y1))

    medium_y0 = int(round(height * VIRTUAL_MEDIUM_Y0))
    medium_y1 = int(round(height * VIRTUAL_MEDIUM_Y1))
    medium_wing_y0 = int(round(height * VIRTUAL_MEDIUM_WING_Y0))
    medium_wing_y1 = int(round(height * VIRTUAL_MEDIUM_WING_Y1))

    near_y0 = int(round(height * VIRTUAL_NEAR_Y0))
    near_y1 = int(round(height * VIRTUAL_NEAR_Y1))

    left_x0 = int(round(width * VIRTUAL_LEFT_X0))
    left_x1 = int(round(width * VIRTUAL_LEFT_X1))

    center_x0 = int(round(width * VIRTUAL_CENTER_X0))
    center_x1 = int(round(width * VIRTUAL_CENTER_X1))

    right_x0 = int(round(width * VIRTUAL_RIGHT_X0))
    right_x1 = int(round(width * VIRTUAL_RIGHT_X1))

    medium_left_x0 = int(round(width * VIRTUAL_MEDIUM_LEFT_X0))
    medium_left_x1 = int(round(width * VIRTUAL_MEDIUM_LEFT_X1))

    medium_center_x0 = int(round(width * VIRTUAL_MEDIUM_CENTER_X0))
    medium_center_x1 = int(round(width * VIRTUAL_MEDIUM_CENTER_X1))

    medium_right_x0 = int(round(width * VIRTUAL_MEDIUM_RIGHT_X0))
    medium_right_x1 = int(round(width * VIRTUAL_MEDIUM_RIGHT_X1))

    far_left_x0 = int(round(width * VIRTUAL_FAR_LEFT_X0))
    far_left_x1 = int(round(width * VIRTUAL_FAR_LEFT_X1))

    far_center_x0 = int(round(width * VIRTUAL_FAR_CENTER_X0))
    far_center_x1 = int(round(width * VIRTUAL_FAR_CENTER_X1))

    far_right_x0 = int(round(width * VIRTUAL_FAR_RIGHT_X0))
    far_right_x1 = int(round(width * VIRTUAL_FAR_RIGHT_X1))

    return {
        "far": {
            "left": {
                "x0": far_left_x0,
                "y0": far_y0,
                "x1": far_left_x1,
                "y1": far_y1,
            },
            "center": {
                "x0": far_center_x0,
                "y0": far_y0,
                "x1": far_center_x1,
                "y1": far_y1,
            },
            "right": {
                "x0": far_right_x0,
                "y0": far_y0,
                "x1": far_right_x1,
                "y1": far_y1,
            },
        },

        "farBand": {
            "left": {
                "x0": left_x0,
                "y0": far_band_y0,
                "x1": left_x1,
                "y1": far_band_y1,
            },
            "center": {
                "x0": center_x0,
                "y0": far_band_y0,
                "x1": center_x1,
                "y1": far_band_y1,
            },
            "right": {
                "x0": right_x0,
                "y0": far_band_y0,
                "x1": right_x1,
                "y1": far_band_y1,
            },
        },

        "medium": {
            "left": {
                "x0": medium_left_x0,
                "y0": medium_y0,
                "x1": medium_left_x1,
                "y1": medium_y1,
                "regions": (
                    {
                        "x0": medium_left_x0,
                        "y0": medium_y0,
                        "x1": medium_left_x1,
                        "y1": medium_y1,
                    },
                    {
                        "x0": left_x0,
                        "y0": medium_wing_y0,
                        "x1": center_x0,
                        "y1": medium_wing_y1,
                    },
                ),
            },
            "center": {
                "x0": medium_center_x0,
                "y0": medium_y0,
                "x1": medium_center_x1,
                "y1": medium_y1,
            },
            "right": {
                "x0": medium_right_x0,
                "y0": medium_y0,
                "x1": medium_right_x1,
                "y1": medium_y1,
                "regions": (
                    {
                        "x0": medium_right_x0,
                        "y0": medium_y0,
                        "x1": medium_right_x1,
                        "y1": medium_y1,
                    },
                    {
                        "x0": center_x1,
                        "y0": medium_wing_y0,
                        "x1": right_x1,
                        "y1": medium_wing_y1,
                    },
                ),
            },
        },

        "near": {
            "center": {
                "x0": center_x0,
                "y0": near_y0,
                "x1": center_x1,
                "y1": near_y1,
            },
            # Esta faixa não é outro sensor. Ela converte os pixels da linha
            # local em uma coordenada X contínua para centralização e heading.
            "position": {
                "x0": 0,
                "y0": near_y0,
                "x1": width,
                "y1": near_y1,
            },
        },
    }


def resolve_virtual_sensor_geometry(frame_shape):
    """Reutiliza a geometria imutável enquanto a resolução não mudar."""

    height, width = frame_shape[:2]
    return cached_virtual_sensor_geometry(int(height), int(width))


def virtual_sensor_regions(sensor_geometry):
    """Retorna os blocos disjuntos que formam uma única leitura virtual."""

    regions = sensor_geometry.get("regions")
    return tuple(regions) if regions else (sensor_geometry,)


def normalized_line_confidence(value):
    """Converte uma medição de confidence para a faixa pública de 0,0 a 1,0."""

    try:
        value = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(value):
        return 0.0
    return float(max(0.0, min(1.0, value)))


def non_negative_line_measurement(value):
    """Sanitiza uma medição diagnóstica que não pode ser negativa."""

    try:
        value = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(value):
        return 0.0
    return max(0.0, value)


def create_virtual_row_component_mask(processed_line_mask, row_geometry):
    """Recorta os pixels finais cobertos pelos blocos de uma fileira virtual."""

    frame_height, frame_width = processed_line_mask.shape[:2]
    clipped_regions = []
    for sensor_geometry in row_geometry.values():
        for region in virtual_sensor_regions(sensor_geometry):
            x0 = max(0, min(frame_width, int(region["x0"])))
            y0 = max(0, min(frame_height, int(region["y0"])))
            x1 = max(x0, min(frame_width, int(region["x1"])))
            y1 = max(y0, min(frame_height, int(region["y1"])))
            if x1 > x0 and y1 > y0:
                clipped_regions.append((x0, y0, x1, y1))

    if not clipped_regions:
        return np.zeros((0, 0), dtype=np.uint8), 0, 0, 0

    bounds_x0 = min(region[0] for region in clipped_regions)
    bounds_y0 = min(region[1] for region in clipped_regions)
    bounds_x1 = max(region[2] for region in clipped_regions)
    bounds_y1 = max(region[3] for region in clipped_regions)
    component_mask = np.zeros(
        (bounds_y1 - bounds_y0, bounds_x1 - bounds_x0),
        dtype=np.uint8,
    )
    for x0, y0, x1, y1 in clipped_regions:
        target = component_mask[
            y0 - bounds_y0:y1 - bounds_y0,
            x0 - bounds_x0:x1 - bounds_x0,
        ]
        source = processed_line_mask[y0:y1, x0:x1]
        np.maximum(target, source, out=target)

    return component_mask, bounds_x0, bounds_y0, bounds_y1


def empty_virtual_row_line_measurement():
    """Cria uma medição diagnóstica vazia para FAR ou MEDIUM."""

    return {
        "lineConfidence": 0.0,
        "thicknessScore": 0.0,
        "thicknessConsistency": 0.0,
        "continuityScore": 0.0,
        "areaScore": 0.0,
        "robustThicknessPx": 0.0,
        "componentCenterX": None,
    }


def expected_virtual_line_thickness_px(frame_shape, absolute_y):
    """Estima a largura da fita em um Y, compensando a perspectiva fixa."""

    frame_height, frame_width = frame_shape[:2]
    if frame_height <= 0 or frame_width <= 0:
        return 0.0

    normalized_y = float(absolute_y) / float(max(1, frame_height - 1))
    normalized_y = max(0.0, min(1.0, normalized_y))
    reference_thickness_px = (
        VIRTUAL_LINE_EXPECTED_THICKNESS_TOP_PX
        + (
            VIRTUAL_LINE_EXPECTED_THICKNESS_BOTTOM_PX
            - VIRTUAL_LINE_EXPECTED_THICKNESS_TOP_PX
        ) * normalized_y
    )
    return max(
        1.0,
        reference_thickness_px
        * float(frame_width)
        / VIRTUAL_LINE_CONFIDENCE_REFERENCE_WIDTH_PX,
    )


def component_labels_in_row(label_row):
    """Retorna os componentes presentes em uma amostra horizontal."""

    if label_row.size == 0:
        return ()

    return tuple(
        int(label)
        for label in np.unique(label_row)
        if label > 0
    )


def normalized_thickness_consistency(local_thicknesses_px):
    """Converte a dispersão robusta das espessuras locais em score 0..1."""

    if local_thicknesses_px.size == 0:
        return 0.0

    median_thickness_px = float(np.median(local_thicknesses_px))
    if median_thickness_px <= 0.0:
        return 0.0

    thickness_mad_px = float(np.median(
        np.abs(local_thicknesses_px - median_thickness_px)
    ))
    lower_percentile_px, upper_percentile_px = np.percentile(
        local_thicknesses_px,
        (10.0, 90.0),
    )
    relative_mad = thickness_mad_px / median_thickness_px
    relative_percentile_span = (
        float(upper_percentile_px - lower_percentile_px)
        / (2.0 * median_thickness_px)
    )
    robust_relative_dispersion = max(
        relative_mad,
        relative_percentile_span,
    )
    return normalized_line_confidence(
        1.0
        - robust_relative_dispersion
        / VIRTUAL_LINE_CONSISTENCY_MAX_RELATIVE_DISPERSION
    )


def measure_component_thickness(
    labels,
    stats,
    label,
    shared_distance_map=None,
    shared_local_maximum_map=None,
):
    """Mede espessura transversal e sua consistência no mesmo componente."""

    component_x = int(stats[label, cv2.CC_STAT_LEFT])
    component_y = int(stats[label, cv2.CC_STAT_TOP])
    component_width = int(stats[label, cv2.CC_STAT_WIDTH])
    component_height = int(stats[label, cv2.CC_STAT_HEIGHT])
    if component_width <= 0 or component_height <= 0:
        return {
            "robustThicknessPx": 0.0,
            "thicknessConsistency": 0.0,
        }

    component_labels = labels[
        component_y:component_y + component_height,
        component_x:component_x + component_width,
    ]
    component_mask = np.where(component_labels == label, 255, 0).astype(
        np.uint8
    )

    if (
        shared_distance_map is None
        or shared_local_maximum_map is None
    ):
        # A borda preta garante que componentes cortados pelo limite da ROI
        # também tenham distância finita até o fundo em todas as orientações.
        padded_mask = cv2.copyMakeBorder(
            component_mask,
            1,
            1,
            1,
            1,
            cv2.BORDER_CONSTANT,
            value=0,
        )
        padded_distance_map = cv2.distanceTransform(
            padded_mask,
            cv2.DIST_L2,
            3,
        )
        component_distance_map = padded_distance_map[1:-1, 1:-1]
        component_local_maximum_map = cv2.dilate(
            padded_distance_map,
            np.ones((3, 3), dtype=np.uint8),
        )[1:-1, 1:-1]
    else:
        component_distance_map = shared_distance_map[
            component_y:component_y + component_height,
            component_x:component_x + component_width,
        ]
        component_local_maximum_map = shared_local_maximum_map[
            component_y:component_y + component_height,
            component_x:component_x + component_width,
        ]
    component_distances = component_distance_map[component_mask != 0]
    if component_distances.size == 0:
        return {
            "robustThicknessPx": 0.0,
            "thicknessConsistency": 0.0,
        }

    core_minimum_radius = float(np.percentile(
        component_distances,
        VIRTUAL_LINE_THICKNESS_CORE_PERCENTILE,
    ))
    core_distances = component_distances[
        component_distances >= core_minimum_radius
    ]
    if core_distances.size == 0:
        return {
            "robustThicknessPx": 0.0,
            "thicknessConsistency": 0.0,
        }

    robust_radius_px = float(np.median(core_distances))
    robust_thickness_px = max(0.0, 2.0 * robust_radius_px)

    # Os máximos locais formam o núcleo interno sem executar skeleton. Assim,
    # a dispersão não inclui o gradiente inevitável entre borda e centro.
    ridge_mask = (
        (component_mask != 0)
        & (
            component_distance_map
            >= component_local_maximum_map - 1e-6
        )
    )
    local_thicknesses_px = 2.0 * component_distance_map[ridge_mask]
    thickness_consistency = normalized_thickness_consistency(
        local_thicknesses_px
    )
    return {
        "robustThicknessPx": robust_thickness_px,
        "thicknessConsistency": thickness_consistency,
    }


def robust_component_thickness_px(labels, stats, label):
    """Mantém a API escalar da espessura diagnóstica validada."""

    return measure_component_thickness(
        labels,
        stats,
        label,
    )["robustThicknessPx"]


def measure_virtual_row_line_confidence(processed_line_mask, row_geometry):
    """Mede a geometria de FAR/MEDIUM sem participar do controle do robô."""

    if processed_line_mask.ndim != 2 or processed_line_mask.size == 0:
        return empty_virtual_row_line_measurement()

    (
        component_mask,
        bounds_x0,
        bounds_y0,
        bounds_y1,
    ) = create_virtual_row_component_mask(
        processed_line_mask,
        row_geometry,
    )
    if component_mask.size == 0 or bounds_y1 <= bounds_y0:
        return empty_virtual_row_line_measurement()

    # A máscara candidata já chega em 0/255. O OpenCV considera qualquer
    # valor diferente de zero como foreground, portanto a cópia criada por
    # cv2.compare não mudava labels, área, centróides ou scores de trust.
    active_pixel_count = int(cv2.countNonZero(component_mask))
    if active_pixel_count <= 0:
        return empty_virtual_row_line_measurement()

    label_count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        component_mask,
        connectivity=8,
    )
    if label_count <= 1:
        return empty_virtual_row_line_measurement()

    # Componentes desconectados compartilham o mesmo fundo zero. Portanto, o
    # distanceTransform e a dilatação da fileira completa produzem, dentro de
    # cada label, os mesmos valores que os antigos mapas calculados isoladamente.
    padded_component_mask = cv2.copyMakeBorder(
        component_mask,
        1,
        1,
        1,
        1,
        cv2.BORDER_CONSTANT,
        value=0,
    )
    padded_distance_map = cv2.distanceTransform(
        padded_component_mask,
        cv2.DIST_L2,
        3,
    )
    shared_distance_map = padded_distance_map[1:-1, 1:-1]
    shared_local_maximum_map = cv2.dilate(
        padded_distance_map,
        np.ones((3, 3), dtype=np.uint8),
    )[1:-1, 1:-1]

    frame_height = processed_line_mask.shape[0]
    sample_step = max(
        1,
        int(round(
            VIRTUAL_LINE_CONFIDENCE_SAMPLE_STEP_PX
            * float(frame_height)
            / VIRTUAL_LINE_CONFIDENCE_REFERENCE_HEIGHT_PX
        )),
    )
    sampled_local_rows = list(range(0, component_mask.shape[0], sample_step))
    last_local_y = component_mask.shape[0] - 1
    if sampled_local_rows[-1] != last_local_y:
        sampled_local_rows.append(last_local_y)

    sampled_rows_by_label = {
        label: []
        for label in range(1, label_count)
    }
    for local_y in sampled_local_rows:
        absolute_y = bounds_y0 + local_y
        for label in component_labels_in_row(labels[local_y]):
            sampled_rows_by_label[label].append(absolute_y)

    # A área de referência representa uma fita em perspectiva atravessando
    # verticalmente toda a fileira observada.
    reference_area = max(
        1.0,
        sum(
            expected_virtual_line_thickness_px(
                processed_line_mask.shape,
                absolute_y,
            )
            for absolute_y in range(bounds_y0, bounds_y1)
        ),
    )
    best_measurement = empty_virtual_row_line_measurement()
    for label in range(1, label_count):
        component_sampled_rows = sampled_rows_by_label[label]
        component_thickness = measure_component_thickness(
            labels,
            stats,
            label,
            shared_distance_map,
            shared_local_maximum_map,
        )
        robust_thickness_px = component_thickness["robustThicknessPx"]
        thickness_consistency = component_thickness[
            "thicknessConsistency"
        ]
        reference_y = bounds_y0 + float(centroids[label][1])
        expected_thickness_px = expected_virtual_line_thickness_px(
            processed_line_mask.shape,
            reference_y,
        )
        thickness_score = normalized_line_confidence(
            robust_thickness_px / expected_thickness_px
        )

        component_area = int(stats[label, cv2.CC_STAT_AREA])
        vertical_coverage = (
            float(len(component_sampled_rows))
            / float(len(sampled_local_rows))
        )
        component_share = float(component_area) / float(active_pixel_count)
        continuity_score = normalized_line_confidence(
            vertical_coverage * component_share
        )
        area_score = normalized_line_confidence(
            float(component_area) / reference_area
        )
        line_confidence = normalized_line_confidence(
            VIRTUAL_LINE_CONFIDENCE_THICKNESS_WEIGHT * thickness_score
            + (
                VIRTUAL_LINE_CONFIDENCE_CONSISTENCY_WEIGHT
                * thickness_consistency
            )
            + VIRTUAL_LINE_CONFIDENCE_CONTINUITY_WEIGHT * continuity_score
            + VIRTUAL_LINE_CONFIDENCE_AREA_WEIGHT * area_score
        )
        candidate_measurement = {
            "lineConfidence": line_confidence,
            "thicknessScore": thickness_score,
            "thicknessConsistency": thickness_consistency,
            "continuityScore": continuity_score,
            "areaScore": area_score,
            "robustThicknessPx": robust_thickness_px,
            "componentCenterX": (
                float(bounds_x0) + float(centroids[label][0])
            ),
        }
        if line_confidence > best_measurement["lineConfidence"]:
            best_measurement = candidate_measurement

    return best_measurement


def calculate_virtual_row_line_confidence(processed_line_mask, row_geometry):
    """Mantém a API escalar da confidence usada pela telemetria existente."""

    measurement = measure_virtual_row_line_confidence(
        processed_line_mask,
        row_geometry,
    )
    return measurement["lineConfidence"]


def line_measurement_is_trusted(
    measurement,
    minimum_confidence,
    minimum_thickness_px,
):
    """Aplica o gate de trust usando somente confidence e espessura."""

    line_confidence = normalized_line_confidence(
        measurement.get("lineConfidence")
    )
    thickness_px = non_negative_line_measurement(
        measurement.get("robustThicknessPx")
    )
    if thickness_px <= LINE_TRUST_ABSOLUTE_THIN_VETO_PX:
        return False
    return bool(
        line_confidence >= minimum_confidence
        and thickness_px >= minimum_thickness_px
    )


def virtual_sensor_trust_is_active(sensors, trust_name):
    """Aceita somente o booleano verdadeiro produzido pelo gate visual."""

    return sensors.get(trust_name) is True


def draw_virtual_sensor_geometry(
    frame,
    line_follower_command,
    show_debug_details=True,
):
    """Desenha a direção e, quando habilitados, os sensores de depuração."""

    geometry = resolve_virtual_sensor_geometry(
        frame.shape
    )

    if show_debug_details:
        sensors = (
            (
                "FAR-L",
                geometry["far"]["left"],
                line_follower_command["farLeft"],
            ),
            (
                "FAR-C",
                geometry["far"]["center"],
                line_follower_command["farCenter"],
            ),
            (
                "FAR-R",
                geometry["far"]["right"],
                line_follower_command["farRight"],
            ),
            (
                "MEDIUM-L",
                geometry["medium"]["left"],
                line_follower_command["mediumLeft"],
            ),
            (
                "MEDIUM-C",
                geometry["medium"]["center"],
                line_follower_command["mediumCenter"],
            ),
            (
                "MEDIUM-R",
                geometry["medium"]["right"],
                line_follower_command["mediumRight"],
            ),
            (
                "NEAR-C",
                geometry["near"]["center"],
                line_follower_command["nearCenter"],
            ),
        )

        for name, sensor, value in sensors:
            for region in virtual_sensor_regions(sensor):
                cv2.rectangle(
                    frame,
                    (region["x0"], region["y0"]),
                    (region["x1"], region["y1"]),
                    (255, 0, 255),
                    2,
                )
            # O CENTER do MEDIUM é estreito. Sua legenda fica em outra linha
            # para não sobrepor os textos completos de MEDIUM-L e MEDIUM-R.
            sensor_text_y_offset = 48 if name == "MEDIUM-C" else 24
            cv2.putText(
                frame,
                f"{name} {value:.2f}",
                (
                    sensor["x0"] + 8,
                    sensor["y0"] + sensor_text_y_offset,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 0, 255),
                1,
                cv2.LINE_AA,
            )

        # O contorno interno identifica somente a faixa FAR BAND usada pelo
        # controle, sem substituir o retângulo completo do FAR principal.
        far_band_left = geometry["farBand"]["left"]
        far_band_right = geometry["farBand"]["right"]
        cv2.rectangle(
            frame,
            (far_band_left["x0"], far_band_left["y0"]),
            (far_band_right["x1"], far_band_right["y1"]),
            (255, 160, 0),
            1,
        )
        cv2.putText(
            frame,
            "FAR BAND",
            (
                far_band_left["x0"] + 8,
                far_band_left["y1"] - 8,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 160, 0),
            1,
            cv2.LINE_AA,
        )

        position_details = (
            (
                "FAR BAND POS",
                line_follower_command["farBandPosition"],
                (
                    geometry["farBand"]["center"]["x0"] + 8,
                    far_band_left["y1"] - 8,
                ),
            ),
            (
                "MEDIUM POS",
                line_follower_command["mediumPosition"],
                (
                    geometry["medium"]["center"]["x0"] + 8,
                    geometry["medium"]["center"]["y0"] + 72,
                ),
            ),
            (
                "NEAR FINE POS",
                line_follower_command["nearFinePosition"],
                (
                    geometry["near"]["center"]["x0"] + 8,
                    geometry["near"]["center"]["y0"] + 48,
                ),
            ),
        )
        for label, position, text_origin in position_details:
            position_text = (
                f"{label} {position:+.2f}"
                if position is not None
                else f"{label} INVALID"
            )
            cv2.putText(
                frame,
                position_text,
                text_origin,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )

    reference_point = virtual_fine_position_to_point(
        line_follower_command["nearFinePosition"],
        geometry["near"]["position"],
    )
    lookahead_position = line_follower_command["farPosition"]
    lookahead_geometry = geometry["far"]
    if reference_point is not None and lookahead_position is None:
        lookahead_position = line_follower_command["mediumPosition"]
        lookahead_geometry = geometry["medium"]
    elif (
        reference_point is None
        and line_follower_command["headingAngle"] is not None
    ):
        reference_point = virtual_row_position_to_point(
            line_follower_command["mediumPosition"],
            geometry["medium"],
        )
    lookahead_point = virtual_row_position_to_point(
        lookahead_position,
        lookahead_geometry,
    )

    if lookahead_point is not None and reference_point is not None:
        lookahead_point_int = (
            int(round(lookahead_point[0])),
            int(round(lookahead_point[1])),
        )

        reference_point_int = (
            int(round(reference_point[0])),
            int(round(reference_point[1])),
        )

        cv2.line(
            frame,
            reference_point_int,
            lookahead_point_int,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.circle(
            frame,
            reference_point_int,
            5,
            (0, 255, 255),
            -1,
        )

        cv2.circle(
            frame,
            lookahead_point_int,
            5,
            (0, 255, 255),
            -1,
        )

    if show_debug_details:
        heading_angle = line_follower_command["headingAngle"]
        heading_text = (
            f"HEADING {heading_angle:+.1f} deg"
            if heading_angle is not None
            else "HEADING INVALID"
        )
        steering_error = line_follower_command["steeringError"]
        steering_text = (
            f"STEERING {steering_error:+.2f}"
            if steering_error is not None
            else "STEERING INVALID"
        )
        near_center = geometry["near"]["center"]
        for line_index, debug_text in enumerate((heading_text, steering_text)):
            cv2.putText(
                frame,
                debug_text,
                (
                    near_center["x0"] + 8,
                    near_center["y0"] + 72 + line_index * 24,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )


def read_virtual_sensor(processed_line_mask, sensor_geometry):
    """
    Mede quanto da área de um sensor virtual está ocupada
    pela máscara final da linha preta.

    Retorno:
        0.0 = nenhuma linha no sensor
        1.0 = sensor completamente ocupado pela linha
    """

    active_pixels = 0
    sensor_area = 0
    for region in virtual_sensor_regions(sensor_geometry):
        sensor_roi = processed_line_mask[
            region["y0"]:region["y1"],
            region["x0"]:region["x1"],
        ]
        if sensor_roi.size == 0:
            continue
        active_pixels += cv2.countNonZero(sensor_roi)
        sensor_area += sensor_roi.size

    if sensor_area == 0:
        return 0.0

    # Os blocos compostos são disjuntos. A soma preserva uma única ocupação
    # normalizada sem contar novamente pixels na transição entre as partes.
    return float(active_pixels) / float(sensor_area)


def select_virtual_trust_row_geometry(
    row_geometry,
    green_direction,
    curva_verde_iniciada=False,
):
    """Limita o trust ao ramo permitido durante a manobra verde."""

    selected_sides = {
        "ESQUERDA": (
            ("left", "center")
            if curva_verde_iniciada
            else ("left",)
        ),
        "DIREITA": (
            ("center", "right")
            if curva_verde_iniciada
            else ("right",)
        ),
    }.get(green_direction)
    if selected_sides is None:
        return row_geometry

    selected_geometry = {
        side: row_geometry[side]
        for side in selected_sides
        if side in row_geometry
    }
    if not selected_geometry:
        return row_geometry
    return selected_geometry


def calculate_virtual_row_position(
    left,
    center,
    right,
):
    """
    Converte três sensores analógicos L/C/R
    em uma posição lateral contínua.

    -1.0 = esquerda
     0.0 = centro
    +1.0 = direita

    Retorna None quando nenhuma linha é observada.
    """

    total = left + center + right

    if total < VIRTUAL_ROW_MIN_ACTIVATION:
        return None

    position = (
        -left + right
    ) / total

    return float(position)


def calculate_virtual_near_fine_position(
    processed_line_mask,
    near_geometry,
):
    """Mede o centro X real dos pixels ativos na faixa vertical do NEAR."""

    position_geometry = near_geometry["position"]
    near_x0 = position_geometry["x0"]
    near_y0 = position_geometry["y0"]
    near_x1 = position_geometry["x1"]
    near_y1 = position_geometry["y1"]
    near_roi = processed_line_mask[near_y0:near_y1, near_x0:near_x1]
    if near_roi.size == 0 or near_roi.shape[1] <= 1:
        return None

    _active_y, active_x = np.nonzero(near_roi)
    if active_x.size == 0:
        return None

    mean_x = float(np.mean(active_x))
    maximum_x = float(near_roi.shape[1] - 1)
    normalized_position = 2.0 * mean_x / maximum_x - 1.0
    return float(max(-1.0, min(1.0, normalized_position)))


def create_green_row_point(row_measurement, row_geometry, trusted):
    """Cria um ponto físico usando o centro do componente trusted da fileira."""

    if not trusted:
        return None

    center_x = finite_virtual_position(
        row_measurement.get("componentCenterX")
    )
    if center_x is None:
        return None

    center_geometry = row_geometry["center"]
    center_y = (
        float(center_geometry["y0"])
        + float(center_geometry["y1"])
    ) / 2.0
    return float(center_x), float(center_y)


def green_point_to_normalized_position(point, frame_width):
    """Normaliza o campo position sem reconstruir o ponto usado no heading."""

    if point is None or frame_width <= 1:
        return None
    normalized_position = 2.0 * float(point[0]) / float(frame_width - 1) - 1.0
    return float(max(-1.0, min(1.0, normalized_position)))


def calculate_green_heading_angle(
    near_point,
    green_medium_point,
    green_far_point,
):
    """Calcula o heading GREEN diretamente entre pontos físicos do frame."""

    reference_point = near_point
    lookahead_point = green_far_point
    if reference_point is not None:
        if lookahead_point is None:
            lookahead_point = green_medium_point
    else:
        if green_medium_point is None or green_far_point is None:
            return None
        reference_point = green_medium_point

    if reference_point is None or lookahead_point is None:
        return None

    reference_x = finite_virtual_position(reference_point[0])
    reference_y = finite_virtual_position(reference_point[1])
    lookahead_x = finite_virtual_position(lookahead_point[0])
    lookahead_y = finite_virtual_position(lookahead_point[1])
    if None in (reference_x, reference_y, lookahead_x, lookahead_y):
        return None

    delta_y = reference_y - lookahead_y
    if delta_y <= 0.0:
        return None

    return float(math.degrees(math.atan2(
        lookahead_x - reference_x,
        delta_y,
    )))


def read_virtual_line_sensors(
    processed_line_mask,
    direcao_verde_ativa="NENHUMA",
    curva_verde_iniciada=False,
):
    """
    Lê FAR, FAR BAND, MEDIUM e o sensor local NEAR-C separadamente.
    """

    geometry = resolve_virtual_sensor_geometry(
        processed_line_mask.shape
    )

    # Durante o GREEN, o trust precisa avaliar somente o ramo escolhido. Medir
    # a interseção inteira pode rejeitar uma curva válida por causa da espessura
    # ou da continuidade dos outros ramos que não participarão do controle.
    far_trust_geometry = select_virtual_trust_row_geometry(
        geometry["far"],
        direcao_verde_ativa,
        curva_verde_iniciada,
    )
    medium_trust_geometry = select_virtual_trust_row_geometry(
        geometry["medium"],
        direcao_verde_ativa,
        curva_verde_iniciada,
    )
    far_line_measurement = measure_virtual_row_line_confidence(
        processed_line_mask,
        far_trust_geometry,
    )
    medium_line_measurement = measure_virtual_row_line_confidence(
        processed_line_mask,
        medium_trust_geometry,
    )
    far_trusted = line_measurement_is_trusted(
        far_line_measurement,
        FAR_TRUST_MIN_CONFIDENCE,
        FAR_TRUST_MIN_THICKNESS_PX,
    )
    medium_trusted = line_measurement_is_trusted(
        medium_line_measurement,
        MEDIUM_TRUST_MIN_CONFIDENCE,
        MEDIUM_TRUST_MIN_THICKNESS_PX,
    )

    raw_far_left = read_virtual_sensor(
        processed_line_mask,
        geometry["far"]["left"],
    )

    raw_far_center = read_virtual_sensor(
        processed_line_mask,
        geometry["far"]["center"],
    )

    raw_far_right = read_virtual_sensor(
        processed_line_mask,
        geometry["far"]["right"],
    )

    near_center = read_virtual_sensor(
        processed_line_mask,
        geometry["near"]["center"],
    )

    raw_far_band_left = read_virtual_sensor(
        processed_line_mask,
        geometry["farBand"]["left"],
    )

    raw_far_band_center = read_virtual_sensor(
        processed_line_mask,
        geometry["farBand"]["center"],
    )

    raw_far_band_right = read_virtual_sensor(
        processed_line_mask,
        geometry["farBand"]["right"],
    )

    raw_medium_left = read_virtual_sensor(
        processed_line_mask,
        geometry["medium"]["left"],
    )

    raw_medium_center = read_virtual_sensor(
        processed_line_mask,
        geometry["medium"]["center"],
    )

    raw_medium_right = read_virtual_sensor(
        processed_line_mask,
        geometry["medium"]["right"],
    )

    raw_far_position = calculate_virtual_row_position(
        raw_far_left,
        raw_far_center,
        raw_far_right,
    )
    raw_far_band_position = calculate_virtual_row_position(
        raw_far_band_left,
        raw_far_band_center,
        raw_far_band_right,
    )
    raw_medium_position = calculate_virtual_row_position(
        raw_medium_left,
        raw_medium_center,
        raw_medium_right,
    )
    near_center_visible = virtual_sensor_is_active(near_center)
    near_fine_position = None
    if near_center_visible:
        near_fine_position = calculate_virtual_near_fine_position(
            processed_line_mask,
            geometry["near"],
        )
    control_far_left = raw_far_left
    control_far_center = raw_far_center
    control_far_right = raw_far_right
    control_medium_left = raw_medium_left
    control_medium_center = raw_medium_center
    control_medium_right = raw_medium_right

    # Antes de iniciar a curva, FAR e MEDIUM enxergam somente o lado escolhido.
    # Durante o giro, CENTER é liberado para o mesmo ramo migrar ao centro,
    # enquanto o lado oposto permanece bloqueado.
    if direcao_verde_ativa == "ESQUERDA":
        control_far_right = 0.0
        control_medium_right = 0.0
        if not curva_verde_iniciada:
            control_far_center = 0.0
            control_medium_center = 0.0

    elif direcao_verde_ativa == "DIREITA":
        control_far_left = 0.0
        control_medium_left = 0.0
        if not curva_verde_iniciada:
            control_far_center = 0.0
            control_medium_center = 0.0

    # A conversão RAW -> controle ocorre aqui, antes de heading, trackers ou
    # steering. Uma fileira sem trust equivale integralmente a linha ausente.
    if not far_trusted:
        control_far_left = 0.0
        control_far_center = 0.0
        control_far_right = 0.0
        control_far_band_left = 0.0
        control_far_band_center = 0.0
        control_far_band_right = 0.0
    else:
        control_far_band_left = raw_far_band_left
        control_far_band_center = raw_far_band_center
        control_far_band_right = raw_far_band_right

    if not medium_trusted:
        control_medium_left = 0.0
        control_medium_center = 0.0
        control_medium_right = 0.0

    far_band_position = calculate_virtual_row_position(
        control_far_band_left,
        control_far_band_center,
        control_far_band_right,
    )

    green_active = direcao_verde_ativa in ("ESQUERDA", "DIREITA")
    green_far_point = None
    green_medium_point = None
    if green_active:
        # O GREEN usa o centro físico do componente que venceu o trust. As
        # ocupações LEFT/CENTER/RIGHT continuam apenas como telemetria.
        green_far_point = create_green_row_point(
            far_line_measurement,
            geometry["far"],
            far_trusted,
        )
        green_medium_point = create_green_row_point(
            medium_line_measurement,
            geometry["medium"],
            medium_trusted,
        )
        far_position = green_point_to_normalized_position(
            green_far_point,
            processed_line_mask.shape[1],
        )
        medium_position = green_point_to_normalized_position(
            green_medium_point,
            processed_line_mask.shape[1],
        )
        near_point = virtual_fine_position_to_point(
            near_fine_position,
            geometry["near"]["position"],
        )
        heading_angle = calculate_green_heading_angle(
            near_point,
            green_medium_point,
            green_far_point,
        )
        # O heading GREEN permanece apenas como diagnóstico. Durante a manobra,
        # somente o Fusion selecionado pelo marcador possui autoridade de steering.
        steering_error = None
    else:
        far_position = calculate_virtual_row_position(
            control_far_left,
            control_far_center,
            control_far_right,
        )
        medium_position = calculate_virtual_row_position(
            control_medium_left,
            control_medium_center,
            control_medium_right,
        )
        heading_angle = calculate_virtual_heading_angle(
            far_position,
            near_fine_position,
            geometry,
            medium_position=medium_position,
        )
        steering_error = calculate_virtual_steering_error(
            near_fine_position is not None,
            heading_angle,
            fallback_medium_position=(
                medium_position
                if near_fine_position is None
                else None
            ),
            fallback_far_position=(
                far_position
                if near_fine_position is None
                else None
            ),
        )

    return {
        "farLeft": raw_far_left,
        "farCenter": raw_far_center,
        "farRight": raw_far_right,
        "controlFarLeft": control_far_left,
        "controlFarCenter": control_far_center,
        "controlFarRight": control_far_right,
        "farPosition": far_position,
        "rawFarPosition": raw_far_position,
        "farLineConfidence": far_line_measurement["lineConfidence"],
        "farLineThicknessPx": far_line_measurement["robustThicknessPx"],
        "farThicknessConsistency": far_line_measurement[
            "thicknessConsistency"
        ],
        "farTrusted": far_trusted,
        "farThicknessScore": far_line_measurement["thicknessScore"],
        "farContinuityScore": far_line_measurement["continuityScore"],
        "farAreaScore": far_line_measurement["areaScore"],

        "farBandLeft": raw_far_band_left,
        "farBandCenter": raw_far_band_center,
        "farBandRight": raw_far_band_right,
        "controlFarBandLeft": control_far_band_left,
        "controlFarBandCenter": control_far_band_center,
        "controlFarBandRight": control_far_band_right,
        "farBandPosition": far_band_position,
        "rawFarBandPosition": raw_far_band_position,

        "mediumLeft": raw_medium_left,
        "mediumCenter": raw_medium_center,
        "mediumRight": raw_medium_right,
        "controlMediumLeft": control_medium_left,
        "controlMediumCenter": control_medium_center,
        "controlMediumRight": control_medium_right,
        "mediumPosition": medium_position,
        "rawMediumPosition": raw_medium_position,
        "mediumLineConfidence": medium_line_measurement["lineConfidence"],
        "mediumLineThicknessPx": medium_line_measurement["robustThicknessPx"],
        "mediumThicknessConsistency": medium_line_measurement[
            "thicknessConsistency"
        ],
        "mediumTrusted": medium_trusted,
        "mediumThicknessScore": medium_line_measurement["thicknessScore"],
        "mediumContinuityScore": medium_line_measurement["continuityScore"],
        "mediumAreaScore": medium_line_measurement["areaScore"],

        "nearCenter": near_center,
        "nearFinePosition": near_fine_position,
        "greenFarPoint": green_far_point,
        "greenMediumPoint": green_medium_point,
        "headingAngle": heading_angle,
        "steeringError": steering_error,
    }


def virtual_sensor_is_active(value):
    """Valida se uma leitura analógica supera o limiar global dos sensores."""

    value = finite_virtual_position(value)
    return value is not None and value >= VIRTUAL_ROW_MIN_ACTIVATION


def apply_virtual_fine_center_deadband(fine_position):
    """Remove a trepidação central sem reduzir o alcance da posição fina."""

    if abs(fine_position) <= VIRTUAL_FINE_CENTER_DEADBAND:
        return 0.0
    return math.copysign(
        (abs(fine_position) - VIRTUAL_FINE_CENTER_DEADBAND)
        / (1.0 - VIRTUAL_FINE_CENTER_DEADBAND),
        fine_position,
    )


def virtual_row_position_to_point(
    position,
    row_geometry,
):
    """
    Converte uma posição normalizada -1..+1
    em um ponto real (x, y) dentro da fileira.

    -1 = centro do sensor LEFT
     0 = centro do sensor CENTER
    +1 = centro do sensor RIGHT
    """

    if position is None:
        return None

    left_center_x = (
        row_geometry["left"]["x0"]
        + row_geometry["left"]["x1"]
    ) / 2.0

    right_center_x = (
        row_geometry["right"]["x0"]
        + row_geometry["right"]["x1"]
    ) / 2.0

    center_y = (
        row_geometry["center"]["y0"]
        + row_geometry["center"]["y1"]
    ) / 2.0

    normalized = (position + 1.0) / 2.0

    x = (
        left_center_x
        + normalized
        * (right_center_x - left_center_x)
    )

    return (
        float(x),
        float(center_y),
    )


def virtual_fine_position_to_point(position, position_geometry):
    """Converte o X contínuo do NEAR em um ponto físico da faixa local."""

    if position is None:
        return None

    x0 = position_geometry["x0"]
    x1 = position_geometry["x1"]
    center_y = (
        position_geometry["y0"]
        + position_geometry["y1"]
    ) / 2.0
    maximum_offset = max(0.0, float(x1 - x0 - 1))
    normalized = (position + 1.0) / 2.0
    x = float(x0) + normalized * maximum_offset
    return float(x), float(center_y)


def calculate_virtual_heading_angle(
    far_position,
    near_fine_position,
    geometry,
    medium_position=None,
):
    """
    Calcula a direção usando os pontos físicos disponíveis no frame atual.

    0°  = reta
    >0° = aponta para a direita
    <0° = aponta para a esquerda

    Com NEAR válido, preserva FAR↔NEAR e o fallback MEDIUM↔NEAR. Sem NEAR,
    exige FAR e MEDIUM válidos e usa FAR↔MEDIUM. Um único sensor à frente não
    cria orientação nova e permanece disponível para o recovery existente.
    """

    reference_point = virtual_fine_position_to_point(
        near_fine_position,
        geometry["near"]["position"],
    )
    lookahead_position = far_position
    lookahead_geometry = geometry["far"]

    if reference_point is not None:
        if lookahead_position is None:
            lookahead_position = medium_position
            lookahead_geometry = geometry["medium"]
    else:
        if far_position is None or medium_position is None:
            return None
        reference_point = virtual_row_position_to_point(
            medium_position,
            geometry["medium"],
        )

    lookahead_point = virtual_row_position_to_point(
        lookahead_position,
        lookahead_geometry,
    )

    if lookahead_point is None or reference_point is None:
        return None

    delta_x = lookahead_point[0] - reference_point[0]
    delta_y = reference_point[1] - lookahead_point[1]

    if delta_y <= 0.0:
        return None

    angle_radians = math.atan2(
        delta_x,
        delta_y,
    )

    return float(
        math.degrees(angle_radians)
    )


def calculate_virtual_steering_error(
    near_fine_position_valid,
    heading_angle,
    fallback_medium_position=None,
    fallback_far_position=None,
):
    """
    Usa o heading físico como direção da trajetória.

    Com a posição fina válida, preserva o controle local. Sem ela, MEDIUM
    assume a correção lateral; se MEDIUM faltar, FAR mantém a antecipação.

    -1.0 = correção máxima para esquerda
     0.0 = seguir reto
    +1.0 = correção máxima para direita
    """

    fallback_medium_position = finite_virtual_position(
        fallback_medium_position
    )
    fallback_far_position = finite_virtual_position(
        fallback_far_position
    )

    if heading_angle is None:
        if near_fine_position_valid:
            return 0.0
        if fallback_medium_position is not None:
            return float(max(
                -1.0,
                min(1.0, 0.50 * fallback_medium_position),
            ))
        if fallback_far_position is not None:
            return float(max(
                -1.0,
                min(
                    1.0,
                    VIRTUAL_HEADING_GAIN * fallback_far_position,
                ),
            ))
        return None

    heading_normalized = (
        heading_angle
        / VIRTUAL_HEADING_FULL_SCALE_DEG
    )

    heading_normalized = max(
        -1.0,
        min(1.0, heading_normalized),
    )

    steering_error = VIRTUAL_HEADING_GAIN * heading_normalized
    if fallback_medium_position is not None:
        # O peso lateral preservado de 0,50 entra somente neste gate;
        # MEDIUM substitui a referência local ausente sem criar outro ganho.
        steering_error = (
            0.50 * fallback_medium_position
            + steering_error
        )
    elif not near_fine_position_valid:
        return None

    steering_error = max(
        -1.0,
        min(1.0, steering_error),
    )

    return float(steering_error)
