"""Detecta as áreas verde e vermelha da sala de resgate."""

import math
import os
import time

import cv2  # type: ignore
import numpy as np


GEOMETRY_STATES = (
    "NOT_DETECTED",
    "BOUNDS_UNKNOWN",
    "LEFT_BOUND_ONLY",
    "RIGHT_BOUND_ONLY",
    "FULL_BOUNDS",
)

# Limites HSV amplos toleram variação de luz sem misturar as duas cores.
# O vermelho usa dois intervalos porque o Hue do OpenCV reinicia em 180.
GREEN_HSV_RANGES = (((35, 70, 45), (90, 255, 255)),)
RED_HSV_RANGES = (
    ((0, 80, 45), (12, 255, 255)),
    ((168, 80, 45), (179, 255, 255)),
)

# O Hue continua amplo para aceitar verdes variados de competição. Estes
# limites exigem apenas que o verde seja cromático e realmente dominante. A
# separação G-B fica bem abaixo dos verdes reais medidos (0,11 a 0,15) e acima
# do piso branco/ciano observado (0,002 a 0,006).
MIN_GREEN_BLUE_DOMINANCE = 0.025
MIN_GREEN_RED_DOMINANCE = 0.025
MIN_GREEN_NORMALIZED_CHROMA = 0.10

# A evidência forte valida o componente inteiro sem apagar suas bordas mais
# fracas. O piso MDF/oliva medido possui S entre 78 e 108, enquanto o limiar 120
# ainda preserva verdes plausíveis lavados e fica bem abaixo dos verdes reais.
# valor triangulo verde estadual/regional > 90, interno senai > 120
STRONG_GREEN_HUE_MIN = 35
STRONG_GREEN_HUE_MAX = 90
STRONG_GREEN_MIN_SATURATION = 90
STRONG_GREEN_MIN_VALUE = 45
STRONG_GREEN_BLUE_DOMINANCE = 0.04
STRONG_GREEN_RED_DOMINANCE = 0.04
STRONG_GREEN_NORMALIZED_CHROMA = 0.12
MIN_STRONG_GREEN_FRACTION = 0.20

# Três observações eliminam falsos positivos curtos. Duas perdas evitam
# piscar o estado confirmado sem reutilizar geometria de um frame anterior.
ZONE_CONFIRMATION_REQUIRED_FRAMES = 3
ZONE_LOSS_REQUIRED_FRAMES = 2

# Diagnóstico temporário de calibração. O overlay operacional permanece limpo;
# defina OBR_RESCUE_ZONE_COLOR_DIAGNOSTICS=1 para reexibir as medianas RGB/HSV.
SHOW_COLOR_DIAGNOSTICS = (
    os.environ.get("OBR_RESCUE_ZONE_COLOR_DIAGNOSTICS", "0") == "1"
)

# Componentes abaixo desta fração do frame são ruído colorido. O piso
# absoluto preserva o mesmo comportamento em imagens sintéticas pequenas.
MIN_COMPONENT_AREA_RATIO = 0.001
MIN_COMPONENT_AREA_PX = 40
MIN_ZONE_AREA_RATIO = 0.004

# Uma borda lateral só é observável quando existe uma faixa de fundo real.
# Isso impede que uma zona preenchendo quase todo o frame produza um aimX falso.
MIN_BACKGROUND_MARGIN_RATIO = 0.025
SCANLINE_COUNT = 15
MIN_BOUND_SAMPLE_RATIO = 0.50
MIN_BILATERAL_COVERAGE = 0.50
ULTRASONIC_MINIMUM_CM = 2.0
ULTRASONIC_MAXIMUM_CM = 400.0

# O diagnóstico usa apenas uma faixa externa ao hull e não participa da
# segmentação nem da decisão geométrica da zona.
BACKGROUND_SAMPLE_KERNEL_RATIO = 0.08
BACKGROUND_MINIMUM_VISIBLE_VALUE = 35
MINIMUM_DIAGNOSTIC_SAMPLE_PIXELS = 20


def _odd_kernel_size(frame_shape, ratio, minimum):
    """Escala kernels morfológicos sem depender da resolução da CAM1."""

    shortest_side = min(int(frame_shape[0]), int(frame_shape[1]))
    size = max(minimum, int(round(shortest_side * ratio)))
    return size if size % 2 == 1 else size + 1


def _segment_hsv(hsv_frame, ranges):
    """Combina intervalos HSV e remove somente ruído e interrupções pequenas."""

    mask = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
    for lower, upper in ranges:
        mask = cv2.bitwise_or(
            mask,
            cv2.inRange(
                hsv_frame,
                np.asarray(lower, dtype=np.uint8),
                np.asarray(upper, dtype=np.uint8),
            ),
        )

    open_size = _odd_kernel_size(mask.shape, 0.006, 3)
    close_size = _odd_kernel_size(mask.shape, 0.015, 5)
    open_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (open_size, open_size)
    )
    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (close_size, close_size)
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)


def _green_channel_metrics(frame_bgr):
    """Calcula dominâncias normalizadas sem depender do brilho absoluto."""

    blue = frame_bgr[:, :, 0].astype(np.float32)
    green = frame_bgr[:, :, 1].astype(np.float32)
    red = frame_bgr[:, :, 2].astype(np.float32)
    channel_sum = np.maximum(red + green + blue, 1.0)
    green_blue_dominance = (green - blue) / channel_sum
    green_red_dominance = (green - red) / channel_sum
    normalized_chroma = (
        np.maximum(np.maximum(red, green), blue)
        - np.minimum(np.minimum(red, green), blue)
    ) / channel_sum
    return (
        red,
        green,
        blue,
        green_blue_dominance,
        green_red_dominance,
        normalized_chroma,
    )


def _green_chromatic_mask(frame_bgr, channel_metrics=None):
    """Produz a camada permissiva que preserva pixels GREEN mais fracos."""

    channel_metrics = (
        _green_channel_metrics(frame_bgr)
        if channel_metrics is None
        else channel_metrics
    )
    (
        red,
        green,
        blue,
        green_blue_dominance,
        green_red_dominance,
        normalized_chroma,
    ) = channel_metrics
    accepted = (
        (green > blue)
        & (green > red)
        & (green_blue_dominance >= MIN_GREEN_BLUE_DOMINANCE)
        & (green_red_dominance >= MIN_GREEN_RED_DOMINANCE)
        & (normalized_chroma >= MIN_GREEN_NORMALIZED_CHROMA)
    )
    return accepted.astype(np.uint8) * 255


def _strong_green_mask(frame_bgr, hsv_frame, channel_metrics=None):
    """Marca pixels com evidência cromática inequívoca de verde."""

    channel_metrics = (
        _green_channel_metrics(frame_bgr)
        if channel_metrics is None
        else channel_metrics
    )
    (
        red,
        green,
        blue,
        green_blue_dominance,
        green_red_dominance,
        normalized_chroma,
    ) = channel_metrics
    hue = hsv_frame[:, :, 0]
    saturation = hsv_frame[:, :, 1]
    value = hsv_frame[:, :, 2]
    accepted = (
        (hue >= STRONG_GREEN_HUE_MIN)
        & (hue <= STRONG_GREEN_HUE_MAX)
        & (saturation >= STRONG_GREEN_MIN_SATURATION)
        & (value >= STRONG_GREEN_MIN_VALUE)
        & (green > red)
        & (green > blue)
        & (green_blue_dominance >= STRONG_GREEN_BLUE_DOMINANCE)
        & (green_red_dominance >= STRONG_GREEN_RED_DOMINANCE)
        & (normalized_chroma >= STRONG_GREEN_NORMALIZED_CHROMA)
    )
    return accepted.astype(np.uint8) * 255


def _component_records(mask, strong_mask=None, minimum_strong_fraction=0.0):
    """Retorna somente componentes grandes o bastante para pertencer a uma zona."""

    frame_area = mask.shape[0] * mask.shape[1]
    minimum_area = max(
        MIN_COMPONENT_AREA_PX,
        int(round(frame_area * MIN_COMPONENT_AREA_RATIO)),
    )
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    components = []
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < minimum_area:
            continue
        strong_fraction = None
        if strong_mask is not None:
            strong_pixels = int(np.count_nonzero(strong_mask[labels == label]))
            strong_fraction = strong_pixels / float(area)
            if strong_fraction < minimum_strong_fraction:
                continue
        components.append(
            {
                "label": label,
                "area": area,
                "x": int(stats[label, cv2.CC_STAT_LEFT]),
                "y": int(stats[label, cv2.CC_STAT_TOP]),
                "width": int(stats[label, cv2.CC_STAT_WIDTH]),
                "height": int(stats[label, cv2.CC_STAT_HEIGHT]),
                "strongFraction": strong_fraction,
            }
        )
    components.sort(key=lambda component: component["area"], reverse=True)
    return labels, components


def _components_are_coherent(first, second, frame_shape):
    """Aceita fragmentos próximos que podem ter sido separados por uma oclusão."""

    frame_height, frame_width = frame_shape[:2]
    first_right = first["x"] + first["width"]
    second_right = second["x"] + second["width"]
    first_bottom = first["y"] + first["height"]
    second_bottom = second["y"] + second["height"]
    horizontal_gap = max(first["x"], second["x"]) - min(
        first_right, second_right
    )
    vertical_gap = max(first["y"], second["y"]) - min(
        first_bottom, second_bottom
    )
    return (
        horizontal_gap <= frame_width * 0.28
        and vertical_gap <= frame_height * 0.22
    )


def _coherent_component_labels(components, frame_shape):
    """Agrupa ao maior componente somente fragmentos geometricamente coerentes."""

    if not components:
        return []
    selected = [components[0]]
    for component in components[1:]:
        if any(
            _components_are_coherent(component, accepted, frame_shape)
            for accepted in selected
        ):
            selected.append(component)
    return [component["label"] for component in selected]


def _reconstruct_hull(mask, strong_mask=None, minimum_strong_fraction=0.0):
    """Reconstrói uma zona coerente com convex hull para tolerar oclusões."""

    labels, components = _component_records(
        mask,
        strong_mask,
        minimum_strong_fraction,
    )
    selected_labels = _coherent_component_labels(components, mask.shape)
    if not selected_labels:
        return None, 0, None, None

    selected_mask = np.isin(labels, selected_labels).astype(np.uint8) * 255
    contours, _ = cv2.findContours(
        selected_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return None, 0, None, None
    points = np.concatenate(contours, axis=0)
    hull = cv2.convexHull(points)
    area_pixels = int(round(cv2.contourArea(hull)))
    minimum_zone_area = mask.shape[0] * mask.shape[1] * MIN_ZONE_AREA_RATIO
    if area_pixels < minimum_zone_area:
        return None, 0, None, None
    strong_fraction = None
    if strong_mask is not None:
        selected_pixels = cv2.countNonZero(selected_mask)
        strong_pixels = int(np.count_nonzero(strong_mask[selected_mask > 0]))
        strong_fraction = strong_pixels / float(max(1, selected_pixels))
    return hull, area_pixels, selected_mask, strong_fraction


def _reconstruct_green_hull(strong_mask):
    """Forma a região GREEN exclusivamente com componentes strong contínuos."""

    # A máscara candidata continua disponível para diagnóstico, mas não pode
    # propagar o hull por pisos ou paredes com dominante verde fraca.
    return _reconstruct_hull(
        strong_mask,
        strong_mask,
        MIN_STRONG_GREEN_FRACTION,
    )


def _empty_result():
    """Cria o resultado seguro usado quando nenhuma zona foi detectada."""

    return {
        "detected": False,
        "confidence": 0.0,
        "geometryState": "NOT_DETECTED",
        "aimValid": False,
        "aimX": None,
        "aimNormalized": None,
        "leftEdgeX": None,
        "rightEdgeX": None,
        "areaPixels": 0,
        "frameCoverage": 0.0,
        "boundsCoverage": 0.0,
        "_hull": None,
    }


def _robust_edge(samples):
    """Combina transições laterais sem deixar uma linha isolada dominar."""

    if not samples:
        return None
    values = np.asarray(samples, dtype=np.float64)
    median = float(np.median(values))
    deviations = np.abs(values - median)
    median_deviation = float(np.median(deviations))
    if median_deviation > 0.0:
        values = values[deviations <= 3.0 * median_deviation]
    return float(np.median(values)) if values.size else median


def _measure_geometry(hull, area_pixels, frame_shape):
    """Mede as duas bordas em várias linhas e valida o ponto de mira bilateral."""

    frame_height, frame_width = frame_shape[:2]
    hull_mask = np.zeros((frame_height, frame_width), dtype=np.uint8)
    cv2.drawContours(hull_mask, [hull], -1, 255, cv2.FILLED)
    _, y, _, height = cv2.boundingRect(hull)
    scan_top = y + int(round(height * 0.15))
    scan_bottom = y + height - 1 - int(round(height * 0.08))
    if scan_bottom < scan_top:
        scan_top, scan_bottom = y, y + height - 1
    scan_rows = np.unique(
        np.linspace(scan_top, scan_bottom, SCANLINE_COUNT).astype(np.int32)
    )

    minimum_margin = max(3, int(round(frame_width * MIN_BACKGROUND_MARGIN_RATIO)))
    left_samples = []
    right_samples = []
    bilateral_rows = 0
    useful_rows = 0
    for row_y in scan_rows:
        colored_x = np.flatnonzero(hull_mask[row_y])
        if colored_x.size == 0:
            continue
        useful_rows += 1
        left_x = int(colored_x[0])
        right_x = int(colored_x[-1])
        left_visible = left_x >= minimum_margin
        right_visible = frame_width - 1 - right_x >= minimum_margin
        if left_visible:
            left_samples.append(left_x)
        if right_visible:
            right_samples.append(right_x)
        if left_visible and right_visible:
            bilateral_rows += 1

    minimum_samples = max(
        3, int(math.ceil(max(1, useful_rows) * MIN_BOUND_SAMPLE_RATIO))
    )
    left_valid = len(left_samples) >= minimum_samples
    right_valid = len(right_samples) >= minimum_samples
    left_edge = _robust_edge(left_samples) if left_valid else None
    right_edge = _robust_edge(right_samples) if right_valid else None
    coverage = bilateral_rows / useful_rows if useful_rows else 0.0
    aim_valid = (
        left_valid
        and right_valid
        and coverage >= MIN_BILATERAL_COVERAGE
        and right_edge > left_edge
    )

    if aim_valid:
        geometry_state = "FULL_BOUNDS"
    elif left_valid and not right_valid:
        geometry_state = "LEFT_BOUND_ONLY"
    elif right_valid and not left_valid:
        geometry_state = "RIGHT_BOUND_ONLY"
    else:
        geometry_state = "BOUNDS_UNKNOWN"

    aim_x = (left_edge + right_edge) / 2.0 if aim_valid else None
    area_ratio = area_pixels / float(frame_height * frame_width)
    confidence = min(1.0, 0.35 + min(0.35, area_ratio * 1.5) + coverage * 0.30)
    return {
        "detected": True,
        "confidence": round(confidence, 3),
        "geometryState": geometry_state,
        "aimValid": aim_valid,
        "aimX": round(aim_x, 2) if aim_x is not None else None,
        "aimNormalized": (
            round(2.0 * aim_x / (frame_width - 1) - 1.0, 4)
            if aim_x is not None and frame_width > 1
            else None
        ),
        "leftEdgeX": round(left_edge, 2) if left_edge is not None else None,
        "rightEdgeX": round(right_edge, 2) if right_edge is not None else None,
        "areaPixels": area_pixels,
        "frameCoverage": round(area_ratio, 4),
        "boundsCoverage": round(coverage, 3),
        "_hull": hull,
    }


def _median_channels(image, sample_mask):
    """Calcula a mediana de cada canal ou retorna None sem amostra suficiente."""

    if sample_mask is None:
        return None
    pixels = image[sample_mask > 0]
    if pixels.shape[0] < MINIMUM_DIAGNOSTIC_SAMPLE_PIXELS:
        return None
    return [int(round(float(value))) for value in np.median(pixels, axis=0)]


def _nearby_background_mask(hsv_frame, selected_mask, hull):
    """Cria o anel local externo usado para comparar a cor com o fundo."""

    if selected_mask is None or hull is None:
        return None
    hull_mask = np.zeros(selected_mask.shape, dtype=np.uint8)
    cv2.drawContours(hull_mask, [hull], -1, 255, cv2.FILLED)
    kernel_size = _odd_kernel_size(
        selected_mask.shape,
        BACKGROUND_SAMPLE_KERNEL_RATIO,
        9,
    )
    margin_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
    )
    expanded_hull = cv2.dilate(hull_mask, margin_kernel)
    background_mask = cv2.bitwise_and(
        expanded_hull,
        cv2.bitwise_not(hull_mask),
    )

    # Evita que rodas, vítimas ou sombras pretas dominem a referência local.
    visible_background = background_mask.copy()
    visible_background[hsv_frame[:, :, 2] < BACKGROUND_MINIMUM_VISIBLE_VALUE] = 0
    if cv2.countNonZero(visible_background) >= MINIMUM_DIAGNOSTIC_SAMPLE_PIXELS:
        return visible_background
    return background_mask


def _empty_green_color_diagnostics():
    """Mantém o contrato IPC sem calcular estatísticas quando o diagnóstico está desligado."""

    return {
        "greenMedianRgb": None,
        "greenMedianHsv": None,
        "backgroundMedianRgb": None,
        "backgroundMedianHsv": None,
    }


def _green_color_diagnostics(frame_bgr, hsv_frame, selected_mask, hull):
    """Mede a zona GREEN e um anel de fundo sem alterar sua classificação."""

    diagnostics = _empty_green_color_diagnostics()
    if selected_mask is None or hull is None:
        return diagnostics

    green_median_bgr = _median_channels(frame_bgr, selected_mask)
    diagnostics["greenMedianRgb"] = (
        list(reversed(green_median_bgr))
        if green_median_bgr is not None
        else None
    )
    diagnostics["greenMedianHsv"] = _median_channels(
        hsv_frame, selected_mask
    )

    background_mask = _nearby_background_mask(
        hsv_frame, selected_mask, hull
    )

    background_median_bgr = _median_channels(frame_bgr, background_mask)
    diagnostics["backgroundMedianRgb"] = (
        list(reversed(background_median_bgr))
        if background_median_bgr is not None
        else None
    )
    diagnostics["backgroundMedianHsv"] = _median_channels(
        hsv_frame, background_mask
    )
    return diagnostics


def _empty_red_color_diagnostics():
    """Mantém as chaves RED nulas sem construir o anel de fundo diagnóstico."""

    return {
        "redMedianRgb": None,
        "redMedianHsv": None,
        "redBackgroundMedianRgb": None,
        "redBackgroundMedianHsv": None,
    }


def _red_color_diagnostics(frame_bgr, hsv_frame, selected_mask, hull):
    """Mede a zona RED e seu fundo sem alterar a máscara segmentada."""

    diagnostics = _empty_red_color_diagnostics()
    if selected_mask is None or hull is None:
        return diagnostics

    red_median_bgr = _median_channels(frame_bgr, selected_mask)
    diagnostics["redMedianRgb"] = (
        list(reversed(red_median_bgr))
        if red_median_bgr is not None
        else None
    )
    diagnostics["redMedianHsv"] = _median_channels(hsv_frame, selected_mask)

    hull_mask = np.zeros(selected_mask.shape, dtype=np.uint8)
    cv2.drawContours(hull_mask, [hull], -1, 255, cv2.FILLED)
    kernel_size = _odd_kernel_size(
        selected_mask.shape,
        BACKGROUND_SAMPLE_KERNEL_RATIO,
        9,
    )
    margin_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
    )
    expanded_hull = cv2.dilate(hull_mask, margin_kernel)
    background_mask = cv2.bitwise_and(
        expanded_hull,
        cv2.bitwise_not(hull_mask),
    )
    visible_background = background_mask.copy()
    visible_background[hsv_frame[:, :, 2] < BACKGROUND_MINIMUM_VISIBLE_VALUE] = 0
    if cv2.countNonZero(visible_background) >= MINIMUM_DIAGNOSTIC_SAMPLE_PIXELS:
        background_mask = visible_background

    background_median_bgr = _median_channels(frame_bgr, background_mask)
    diagnostics["redBackgroundMedianRgb"] = (
        list(reversed(background_median_bgr))
        if background_median_bgr is not None
        else None
    )
    diagnostics["redBackgroundMedianHsv"] = _median_channels(
        hsv_frame, background_mask
    )
    return diagnostics


def _analyze_color(
    frame_bgr,
    hsv_frame,
    ranges,
    collect_green_diagnostics=False,
    collect_red_diagnostics=False,
    apply_green_chromatic_filter=False,
):
    """Segmenta uma cor e produz sua geometria sem depender da outra zona."""

    mask = _segment_hsv(hsv_frame, ranges)
    strong_mask = None
    if apply_green_chromatic_filter:
        channel_metrics = _green_channel_metrics(frame_bgr)
        mask = cv2.bitwise_and(
            mask,
            _green_chromatic_mask(frame_bgr, channel_metrics),
        )
        strong_mask = _strong_green_mask(
            frame_bgr,
            hsv_frame,
            channel_metrics,
        )
    if apply_green_chromatic_filter:
        hull, area_pixels, selected_mask, strong_fraction = (
            _reconstruct_green_hull(strong_mask)
        )
    else:
        hull, area_pixels, selected_mask, strong_fraction = _reconstruct_hull(
            mask
        )
    if hull is None:
        result = _empty_result()
    else:
        result = _measure_geometry(hull, area_pixels, mask.shape)
    if apply_green_chromatic_filter:
        result["strongGreenFraction"] = (
            round(strong_fraction, 3)
            if strong_fraction is not None
            else None
        )
    if apply_green_chromatic_filter:
        result.update(_empty_green_color_diagnostics())
    else:
        result.update(_empty_red_color_diagnostics())
    if collect_green_diagnostics:
        result.update(
            _green_color_diagnostics(
                frame_bgr,
                hsv_frame,
                selected_mask,
                hull,
            )
        )
    if collect_red_diagnostics:
        result.update(
            _red_color_diagnostics(
                frame_bgr,
                hsv_frame,
                selected_mask,
                hull,
            )
        )
    return result


def analyze_rescue_zones(
    frame_bgr,
    collect_color_diagnostics=None,
    profile_timings=None,
):
    """Retorna resultados independentes para as áreas verde e vermelha."""

    if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
        raise ValueError("O frame frontal deve possuir três canais BGR.")
    diagnostics_enabled = (
        SHOW_COLOR_DIAGNOSTICS
        if collect_color_diagnostics is None
        else bool(collect_color_diagnostics)
    )
    conversion_started = time.perf_counter() if profile_timings is not None else 0.0
    hsv_frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    if profile_timings is not None:
        profile_timings["colorConversionMs"] = (
            time.perf_counter() - conversion_started
        ) * 1000.0
    return {
        "green": _analyze_color(
            frame_bgr,
            hsv_frame,
            GREEN_HSV_RANGES,
            collect_green_diagnostics=diagnostics_enabled,
            apply_green_chromatic_filter=True,
        ),
        "red": _analyze_color(
            frame_bgr,
            hsv_frame,
            RED_HSV_RANGES,
            collect_red_diagnostics=diagnostics_enabled,
        ),
    }


class RescueZoneTemporalFilter:
    """Confirma GREEN e RED independentemente sem guardar geometria antiga."""

    def __init__(self):
        self.reset()

    def reset(self):
        """Remove confirmações ao fechar o gate ou perder a CAM1."""

        self._states = {
            color: {
                "confirmed": False,
                "positiveFrames": 0,
                "negativeFrames": 0,
            }
            for color in ("green", "red")
        }

    def update(self, candidate_results):
        """Aplica confirmação 3/2 mantendo os dados do frame corrente."""

        filtered_results = {}
        for color in ("green", "red"):
            candidate = candidate_results.get(color, _empty_result())
            result = dict(candidate)
            candidate_detected = candidate.get("detected") is True
            state = self._states[color]

            if candidate_detected:
                state["negativeFrames"] = 0
                if state["confirmed"]:
                    state["positiveFrames"] = ZONE_CONFIRMATION_REQUIRED_FRAMES
                else:
                    state["positiveFrames"] = min(
                        ZONE_CONFIRMATION_REQUIRED_FRAMES,
                        state["positiveFrames"] + 1,
                    )
                    if (
                        state["positiveFrames"]
                        >= ZONE_CONFIRMATION_REQUIRED_FRAMES
                    ):
                        state["confirmed"] = True
            else:
                state["positiveFrames"] = 0
                if state["confirmed"]:
                    state["negativeFrames"] += 1
                    if state["negativeFrames"] >= ZONE_LOSS_REQUIRED_FRAMES:
                        state["confirmed"] = False
                        state["negativeFrames"] = 0
                else:
                    state["negativeFrames"] = 0

            result["candidateDetected"] = candidate_detected
            result["candidateAimValid"] = (
                candidate_detected
                and candidate.get("geometryState") == "FULL_BOUNDS"
                and candidate.get("aimValid") is True
            )
            result["detected"] = state["confirmed"]
            result["confirmationFrames"] = state["positiveFrames"]
            result["confirmationRequiredFrames"] = (
                ZONE_CONFIRMATION_REQUIRED_FRAMES
            )
            result["lossFrames"] = state["negativeFrames"]
            result["lossRequiredFrames"] = ZONE_LOSS_REQUIRED_FRAMES
            result["aimValid"] = (
                state["confirmed"]
                and result["candidateAimValid"]
            )
            filtered_results[color] = result
        return filtered_results


def serializable_results(results):
    """Remove apenas a geometria OpenCV antes da publicação no IPC."""

    serializable = {
        color: {key: value for key, value in result.items() if key != "_hull"}
        for color, result in results.items()
    }
    green = serializable.get("green", {})
    for key in (
        "greenMedianRgb",
        "greenMedianHsv",
        "backgroundMedianRgb",
        "backgroundMedianHsv",
    ):
        serializable[key] = green.get(key)
    red = serializable.get("red", {})
    for key in (
        "redMedianRgb",
        "redMedianHsv",
        "redBackgroundMedianRgb",
        "redBackgroundMedianHsv",
    ):
        serializable[key] = red.get(key)
    return serializable


def _format_color_diagnostic(label, rgb, hsv):
    """Formata uma linha curta sem inventar valores quando a amostra falhar."""

    rgb_text = " ".join(str(value) for value in rgb) if rgb else "--"
    hsv_text = " ".join(str(value) for value in hsv) if hsv else "--"
    return f"{label} RGB {rgb_text} | HSV {hsv_text}"


def _draw_diagnostic_panel(display, lines, bottom_margin):
    """Desenha um painel cromático e retorna sua área reservada."""

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.46
    thickness = 1
    line_height = 19
    widths = [
        cv2.getTextSize(line, font, font_scale, thickness)[0][0]
        for line in lines
    ]
    panel_x = 10
    panel_width = min(display.shape[1] - panel_x, max(widths) + 14)
    panel_height = line_height * len(lines) + 9
    panel_y = max(10, display.shape[0] - panel_height - bottom_margin)
    cv2.rectangle(
        display,
        (panel_x, panel_y),
        (panel_x + panel_width, panel_y + panel_height),
        (25, 25, 25),
        -1,
    )
    for index, line in enumerate(lines):
        cv2.putText(
            display,
            line,
            (panel_x + 7, panel_y + 18 + index * line_height),
            font,
            font_scale,
            (210, 210, 210),
            thickness,
            cv2.LINE_AA,
        )
    return (panel_x, panel_y, panel_x + panel_width, panel_y + panel_height)


def _draw_color_diagnostics(display, green_result, bottom_margin=10):
    """Desenha as medianas GREEN quando o diagnóstico está habilitado."""

    lines = (
        _format_color_diagnostic(
            "GREEN",
            green_result.get("greenMedianRgb"),
            green_result.get("greenMedianHsv"),
        ),
        _format_color_diagnostic(
            "BG   ",
            green_result.get("backgroundMedianRgb"),
            green_result.get("backgroundMedianHsv"),
        ),
    )
    return _draw_diagnostic_panel(display, lines, bottom_margin)


def _draw_red_color_diagnostics(display, red_result, bottom_margin):
    """Desenha as medianas RED sem compartilhar a amostra de fundo GREEN."""

    lines = (
        _format_color_diagnostic(
            "RED",
            red_result.get("redMedianRgb"),
            red_result.get("redMedianHsv"),
        ),
        _format_color_diagnostic(
            "BG ",
            red_result.get("redBackgroundMedianRgb"),
            red_result.get("redBackgroundMedianHsv"),
        ),
    )
    return _draw_diagnostic_panel(display, lines, bottom_margin)


def _rectangles_overlap(first, second):
    """Indica se duas caixas do overlay ocupam a mesma região do frame."""

    return not (
        first[2] < second[0]
        or second[2] < first[0]
        or first[3] < second[1]
        or second[3] < first[1]
    )


def _place_zone_label(frame_shape, hull_box, text_size, occupied_rectangles):
    """Posiciona o rótulo sem encobrir os outros painéis do overlay."""

    frame_height, frame_width = frame_shape[:2]
    hull_x, hull_y, hull_width, hull_height = hull_box
    text_width, text_height = text_size
    box_width = min(frame_width, text_width + 8)
    box_height = text_height + 10
    box_x = min(max(0, hull_x), max(0, frame_width - box_width))
    preferred_tops = (
        hull_y - box_height - 4,
        hull_y + hull_height + 4,
    )
    scan_tops = range(4, max(5, frame_height - box_height), box_height + 3)
    for candidate_top in (*preferred_tops, *scan_tops):
        box_top = min(max(0, candidate_top), max(0, frame_height - box_height))
        candidate = (
            box_x,
            box_top,
            box_x + box_width,
            box_top + box_height,
        )
        if not any(
            _rectangles_overlap(candidate, occupied)
            for occupied in occupied_rectangles
        ):
            return candidate

    # Frames muito pequenos podem não comportar todas as caixas. O topo ainda
    # preserva o texto dentro da imagem e evita coordenadas inválidas.
    return (box_x, 0, box_x + box_width, box_height)


def _draw_standalone_ultrasonic(display, ultrasonic_text):
    """Mantém a distância visível quando nenhuma zona gera um rótulo."""

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.58
    thickness = 1
    (text_width, text_height), _ = cv2.getTextSize(
        ultrasonic_text,
        font,
        font_scale,
        thickness,
    )
    margin = 10
    text_x = max(margin + 5, display.shape[1] - text_width - margin - 5)
    text_y = margin + text_height + 5
    cv2.rectangle(
        display,
        (text_x - 5, margin),
        (min(display.shape[1] - 1, text_x + text_width + 5), text_y + 5),
        (25, 25, 25),
        -1,
    )
    cv2.putText(
        display,
        ultrasonic_text,
        (text_x, text_y),
        font,
        font_scale,
        (210, 210, 210),
        thickness,
        cv2.LINE_AA,
    )


def draw_rescue_zone_overlay(
    frame_bgr,
    results,
    ultrasonic=None,
    show_color_diagnostics=None,
):
    """Desenha o overlay normal e as medianas temporárias de calibração."""

    display = frame_bgr.copy()
    ultrasonic_distance = (
        ultrasonic.get("ultrasonicDistanceCm")
        if isinstance(ultrasonic, dict)
        else None
    )
    ultrasonic_available = (
        isinstance(ultrasonic, dict)
        and ultrasonic.get("ultrasonicFresh") is True
        and ultrasonic.get("ultrasonicValid") is True
        and isinstance(ultrasonic_distance, (int, float))
        and not isinstance(ultrasonic_distance, bool)
        and math.isfinite(float(ultrasonic_distance))
        and ULTRASONIC_MINIMUM_CM
        <= float(ultrasonic_distance)
        <= ULTRASONIC_MAXIMUM_CM
    )
    ultrasonic_text = (
        f"ULTRA {float(ultrasonic_distance):.1f} cm"
        if ultrasonic_available
        else "ULTRA --"
    )
    labels = {
        "green": ("AREA VERDE", (0, 220, 0)),
        "red": ("AREA VERMELHA", (0, 0, 230)),
    }
    # Os contornos são desenhados antes dos painéis para nunca atravessarem os
    # textos. Depois, cada caixa reserva seu espaço para impedir sobreposição.
    for color_name in ("green", "red"):
        result = results.get(color_name, {})
        hull = result.get("_hull")
        if result.get("detected") and hull is not None:
            cv2.drawContours(
                display,
                [hull],
                -1,
                labels[color_name][1],
                2,
                cv2.LINE_AA,
            )
    diagnostics_enabled = (
        SHOW_COLOR_DIAGNOSTICS
        if show_color_diagnostics is None
        else bool(show_color_diagnostics)
    )
    occupied_rectangles = []
    if diagnostics_enabled:
        green_diagnostic_rectangle = _draw_color_diagnostics(
            display, results.get("green", {})
        )
        occupied_rectangles.append(green_diagnostic_rectangle)
        green_panel_height = (
            green_diagnostic_rectangle[3] - green_diagnostic_rectangle[1]
        )
        occupied_rectangles.append(
            _draw_red_color_diagnostics(
                display,
                results.get("red", {}),
                bottom_margin=10 + green_panel_height + 6,
            )
        )
    zone_label_drawn = False
    for color_name in ("green", "red"):
        result = results.get(color_name, {})
        hull = result.get("_hull")
        if not result.get("detected") or hull is None:
            continue
        area_label, color = labels[color_name]
        label = f"{area_label}  |  {ultrasonic_text}"
        zone_label_drawn = True
        hull_box = cv2.boundingRect(hull)
        (text_width, text_height), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.58, 1
        )
        label_rectangle = _place_zone_label(
            display.shape,
            hull_box,
            (text_width, text_height),
            occupied_rectangles,
        )
        occupied_rectangles.append(label_rectangle)
        box_left, box_top, box_right, box_bottom = label_rectangle
        text_x = box_left + 4
        text_y = box_bottom - 4
        cv2.rectangle(
            display,
            (box_left, box_top),
            (min(display.shape[1] - 1, box_right), box_bottom),
            (25, 25, 25),
            -1,
        )
        cv2.putText(
            display,
            label,
            (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            color,
            1,
            cv2.LINE_AA,
        )
        # A fonte Hershey não possui caracteres acentuados. Este pequeno traço
        # completa visualmente o "Á" sem adicionar uma dependência de renderização.
        cv2.line(
            display,
            (text_x + 4, text_y - text_height - 1),
            (text_x + 8, text_y - text_height - 5),
            color,
            1,
            cv2.LINE_AA,
        )
    if not zone_label_drawn:
        _draw_standalone_ultrasonic(display, ultrasonic_text)
    return display
