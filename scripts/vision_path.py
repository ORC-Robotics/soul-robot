"""Extração simples da geometria da pista a partir de uma máscara binária.

Este módulo não acessa câmera, GPIO, arquivos nem motores. Ele existe para que a
geometria usada pelo seguidor de linha possa ser validada com máscaras sintéticas.
As coordenadas permanecem em pixels; nenhuma conversão métrica é presumida.
"""

from dataclasses import dataclass, field
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np


# Altura e passo, em pixels da imagem de processamento, das bandas horizontais.
# Bandas maiores toleram falhas pequenas, mas misturam mais geometria em curvas.
SCAN_BAND_HEIGHT = 5
SCAN_STEP_PIXELS = 5

# Fração inferior da máscara onde o primeiro ponto pode ser procurado.
# Aumentar tolera uma linha que não alcance a borda; também aceita objetos mais distantes.
START_SEARCH_DEPTH_RATIO = 0.24

# Limites experimentais da raiz do CurrentPath, em pixels da imagem reduzida.
# A raiz precisa permanecer perto do centro útil da câmera e da previsão anterior.
# Aumentar estes valores tolera desalinhamentos maiores, mas facilita que uma branch
# lateral seja confundida com a linha que realmente chega até o robô.
ROOT_MAX_IMAGE_CENTER_OFFSET_PIXELS = 64.0
ROOT_MAX_PREVIOUS_CENTER_JUMP_PIXELS = 48.0

# Largura máxima absoluta da raiz e tolerância relativa ao quadro anterior.
# Na imagem reduzida de 320 px, a faixa real chegou a ultrapassar 60 px perto da
# câmera. O limite de 112 px tolera essa perspectiva; os gates de centro, histórico
# e extensão vertical continuam impedindo que um blob lateral vire CurrentPath.
ROOT_MAX_ABSOLUTE_WIDTH_PIXELS = 112.0
ROOT_MAX_WIDTH_RATIO = 2.20
ROOT_MAX_WIDTH_GROWTH_PIXELS = 14.0

# Depois de quadros inválidos, a busca pode começar um pouco mais à frente. A
# recuperação continua restrita ao centro para não promover uma branch lateral.
ROOT_RECOVERY_SEARCH_DEPTH_RATIO = 0.42
ROOT_RECOVERY_MAX_IMAGE_CENTER_OFFSET_PIXELS = 36.0

# Quantidade máxima de bandas vazias depois que o caminho começou.
# Valores maiores atravessam oclusões, mas podem conectar regiões que não pertencem à pista.
MAX_MISSING_BANDS = 2

# Largura mínima e suporte vertical mínimo de uma região preta válida.
# Limites menores aceitam linhas finas, porém ficam mais sensíveis a ruído isolado.
MIN_REGION_WIDTH_PIXELS = 3
MIN_COLUMN_FILL_RATIO = 0.30

# Salto máximo do centro entre bandas vizinhas, em pixels de processamento.
# Uma região larga que ainda contém o ponto anterior pode ultrapassar esse valor,
# pois esse alargamento é justamente um dos sinais usados para detectar corners.
MAX_CENTER_JUMP_PIXELS = 52.0
REGION_CONTAINS_MARGIN_PIXELS = 4.0

# Janela local e limites para validar a largura durante o percurso bottom-up.
# Um aumento gradual acompanha perspectiva e curvas suaves; uma expansão abrupta
# encerra o CurrentPath e permanece disponível como evidência visual de corner.
LOCAL_WIDTH_HISTORY_SAMPLES = 5
PATH_MAX_LOCAL_WIDTH_RATIO = 1.70
PATH_MAX_LOCAL_WIDTH_GROWTH_PIXELS = 10.0

# A zona de controle ocupa a parte inferior da ROI. Proximidade 1,0 representa
# a base da imagem e 0,0 representa o topo.
CONTROL_ZONE_MIN_PROXIMITY = 0.58
CONTROL_POINT_PROXIMITY = 0.80
CURRENT_PATH_MIN_PROXIMITY = 0.30
MIN_CONTROL_SAMPLES = 4

# Extensão vertical mínima, em pixels reduzidos, dos samples usados na regressão.
# Reduzir aceita geometrias muito curtas e instáveis; aumentar exige mais linha
# visível perto do robô antes de publicar positionError e headingError.
MIN_CONTROL_SPAN_PIXELS = 15.0

# Uma mudança concentrada precisa superar estes limites experimentais para
# separar o CurrentPath do Preview. Aumentar reduz falsos corners; diminuir faz
# a separação acontecer mais cedo em mudanças suaves.
CURRENT_PATH_HEADING_JUMP_DEGREES = 38.0
CURRENT_PATH_MIN_LATERAL_JUMP_PIXELS = 9.0
CURRENT_PATH_WIDTH_RATIO = 1.65
CURRENT_PATH_MIN_WIDTH_GROWTH_PIXELS = 10.0

# Sinais experimentais combinados para classificar uma extensão como corner.
# Nenhum sinal isolado confirma o evento.
CORNER_MIN_HEADING_CHANGE_DEGREES = 42.0
CORNER_MIN_LATERAL_EXTENSION_PIXELS = 18.0
CORNER_MIN_WIDTH_RATIO = 1.55
CORNER_MIN_CONFIDENCE = 0.48

# Um corner precisa crescer claramente mais para um lado do que para o outro.
# Isso separa uma branch real do alargamento aproximadamente simétrico provocado
# por perspectiva, sombra ou proximidade da câmera.
CORNER_MIN_DIRECTIONAL_EXCESS_PIXELS = 9.0
CORNER_MIN_DIRECTIONAL_RATIO = 1.30

# Janela, em pixels reduzidos, usada para localizar a âncora dentro das bandas
# consecutivas da expansão. Uma janela maior suaviza a borda do blob, mas pode
# misturar outra geometria futura; uma menor aproxima a âncora da primeira borda.
CORNER_ANCHOR_MAX_SUPPORT_PIXELS = 25.0
CORNER_ANCHOR_MIN_WIDTH_RATIO = 1.35


@dataclass
class PathSample:
    """Representa o centro de uma região preta em uma banda horizontal."""

    x: float
    y: float
    width: float
    confidence: float
    rejection_reason: str = ""


@dataclass
class CornerObservation:
    """Descreve um possível corner futuro em coordenadas normalizadas da ROI."""

    detected: bool = False
    direction: str = "NONE"
    proximity: float = 0.0
    confidence: float = 0.0
    anchor_x: float = 0.0
    anchor_y: float = 0.0


@dataclass
class PathObservation:
    """Resultado compacto consumido pelo script da câmera e pelo status JSON."""

    line_detected: bool = False
    current_path_valid: bool = False
    samples: List[PathSample] = field(default_factory=list)
    current_path: List[PathSample] = field(default_factory=list)
    preview_path: List[PathSample] = field(default_factory=list)
    rejected_samples: List[PathSample] = field(default_factory=list)
    control_samples: List[PathSample] = field(default_factory=list)
    control_sample_count: int = 0
    position_error_pixels: float = 0.0
    heading_error_degrees: float = 0.0
    path_confidence: float = 0.0
    near_path_x: float = 0.0
    mid_path_x: float = 0.0
    far_path_x: float = 0.0
    control_x: float = 0.0
    control_y: float = 0.0
    expected_root_x: float = 0.0
    root_x: float = 0.0
    root_width: float = 0.0
    corner: CornerObservation = field(default_factory=CornerObservation)


@dataclass
class _PathExtraction:
    samples: List[PathSample] = field(default_factory=list)
    rejected_samples: List[PathSample] = field(default_factory=list)
    rooted: bool = False


@dataclass
class _Region:
    center_x: float
    width: float
    density: float
    left: int
    right: int


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def _angle_difference_degrees(first: float, second: float) -> float:
    difference = (second - first + 180.0) % 360.0 - 180.0
    return abs(difference)


def _is_directional_width_expansion(
    reference_x: float,
    reference_width: float,
    candidate_x: float,
    candidate_width: float,
) -> bool:
    """Indica se o aumento de largura cresce claramente para apenas um lado."""

    reference_left = reference_x - reference_width * 0.5
    reference_right = reference_x + reference_width * 0.5
    candidate_left = candidate_x - candidate_width * 0.5
    candidate_right = candidate_x + candidate_width * 0.5
    left_growth = max(0.0, reference_left - candidate_left)
    right_growth = max(0.0, candidate_right - reference_right)
    dominant_growth = max(left_growth, right_growth)
    opposite_growth = min(left_growth, right_growth)
    return (
        dominant_growth - opposite_growth >= CORNER_MIN_DIRECTIONAL_EXCESS_PIXELS
        and dominant_growth / max(1.0, opposite_growth)
        >= CORNER_MIN_DIRECTIONAL_RATIO
    )


def _find_regions(band: np.ndarray) -> List[_Region]:
    if band.size == 0:
        return []

    black = band > 0
    support = np.count_nonzero(black, axis=0)
    required_support = max(1, int(math.ceil(band.shape[0] * MIN_COLUMN_FILL_RATIO)))
    active_columns = support >= required_support
    regions: List[_Region] = []
    start: Optional[int] = None

    for column, active in enumerate(active_columns):
        if active and start is None:
            start = column
        at_end = column == len(active_columns) - 1
        if start is not None and ((not active) or at_end):
            end = column if active and at_end else column - 1
            width = end - start + 1
            if width >= MIN_REGION_WIDTH_PIXELS:
                region_support = support[start : end + 1].astype(np.float64)
                columns = np.arange(start, end + 1, dtype=np.float64)
                total_support = float(np.sum(region_support))
                center_x = float(np.sum(columns * region_support) / total_support)
                density = total_support / float(band.shape[0] * width)
                regions.append(_Region(center_x, float(width), density, start, end))
            start = None

    return regions


def _select_region(
    regions: Sequence[_Region], previous_x: float
) -> Optional[Tuple[_Region, float]]:
    best_region: Optional[_Region] = None
    best_confidence = 0.0
    best_score = -1.0

    for region in regions:
        distance = abs(region.center_x - previous_x)
        contains_previous = (
            region.left - REGION_CONTAINS_MARGIN_PIXELS
            <= previous_x
            <= region.right + REGION_CONTAINS_MARGIN_PIXELS
        )
        if distance > MAX_CENTER_JUMP_PIXELS and not contains_previous:
            continue

        continuity = 1.0 if contains_previous else _clamp01(
            1.0 - distance / MAX_CENTER_JUMP_PIXELS
        )
        confidence = _clamp01(0.65 * region.density + 0.35 * continuity)
        score = 0.70 * continuity + 0.30 * region.density
        if score > best_score:
            best_score = score
            best_region = region
            best_confidence = confidence

    if best_region is None:
        return None
    return best_region, best_confidence


def _sample_from_region(
    region: _Region,
    center_y: float,
    reference_x: float,
    rejection_reason: str = "",
) -> PathSample:
    distance = abs(region.center_x - reference_x)
    continuity = _clamp01(1.0 - distance / max(1.0, MAX_CENTER_JUMP_PIXELS))
    confidence = _clamp01(0.65 * region.density + 0.35 * continuity)
    return PathSample(
        region.center_x,
        center_y,
        region.width,
        confidence,
        rejection_reason,
    )


def _root_rejection_reason(
    region: _Region,
    expected_x: float,
    image_center_x: float,
    expected_width: Optional[float],
) -> str:
    if abs(region.center_x - image_center_x) > ROOT_MAX_IMAGE_CENTER_OFFSET_PIXELS:
        return "root_center"

    previous_hint_reliable = (
        abs(expected_x - image_center_x) <= ROOT_MAX_IMAGE_CENTER_OFFSET_PIXELS
    )
    if previous_hint_reliable and (
        abs(region.center_x - expected_x) > ROOT_MAX_PREVIOUS_CENTER_JUMP_PIXELS
    ):
        return "root_jump"

    maximum_width = ROOT_MAX_ABSOLUTE_WIDTH_PIXELS
    if expected_width is not None and expected_width > 0.0:
        maximum_width = min(
            maximum_width,
            max(
                expected_width * ROOT_MAX_WIDTH_RATIO,
                expected_width + ROOT_MAX_WIDTH_GROWTH_PIXELS,
            ),
        )
    if region.width > maximum_width:
        return "root_width"
    return ""


def _select_root_region(
    regions: Sequence[_Region],
    expected_x: float,
    image_center_x: float,
    expected_width: Optional[float],
    allow_root_recovery: bool,
) -> Tuple[Optional[_Region], List[Tuple[_Region, str]]]:
    valid_regions: List[_Region] = []
    rejected: List[Tuple[_Region, str]] = []
    for region in regions:
        reason = _root_rejection_reason(
            region, expected_x, image_center_x, expected_width
        )
        if reason:
            rejected.append((region, reason))
        else:
            valid_regions.append(region)

    if not valid_regions:
        # Uma previsão válida no quadro anterior pode ficar mais de 48 pixels
        # distante depois de uma correção. Se existe uma faixa central clara,
        # recupera a raiz em vez de manter o rastreador preso no histórico antigo.
        recovery_candidates = [
            region
            for region, reason in rejected
            if abs(region.center_x - image_center_x)
            <= ROOT_RECOVERY_MAX_IMAGE_CENTER_OFFSET_PIXELS
            and (
                reason == "root_jump"
                or (
                    allow_root_recovery
                    and reason == "root_width"
                    and region.width <= ROOT_MAX_ABSOLUTE_WIDTH_PIXELS
                )
            )
        ]
        if not recovery_candidates:
            return None, rejected
        selected = min(
            recovery_candidates,
            key=lambda region: abs(region.center_x - image_center_x) - region.density * 8.0,
        )
        rejected = [(region, reason) for region, reason in rejected if region is not selected]
        return selected, rejected

    previous_hint_reliable = (
        abs(expected_x - image_center_x) <= ROOT_MAX_IMAGE_CENTER_OFFSET_PIXELS
    )
    reference_x = expected_x if previous_hint_reliable else image_center_x
    selected = min(
        valid_regions,
        key=lambda region: abs(region.center_x - reference_x) - region.density * 8.0,
    )
    return selected, rejected


def _extract_path_geometry(
    mask: np.ndarray,
    start_x: float,
    expected_root_width: Optional[float] = None,
    start_search_depth_ratio: float = START_SEARCH_DEPTH_RATIO,
    allow_root_recovery: bool = False,
) -> _PathExtraction:
    """Enraíza a centerline perto do robô antes de percorrê-la bottom-up."""

    if mask.ndim != 2 or mask.size == 0:
        return _PathExtraction()

    height, width = mask.shape
    half_band = SCAN_BAND_HEIGHT // 2
    minimum_start_y = int(round((height - 1) * (1.0 - start_search_depth_ratio)))
    previous_x = float(start_x)
    image_center_x = width * 0.5
    started = False
    preview_started = False
    preview_rejection_reason = ""
    missing_bands = 0
    samples: List[PathSample] = []
    rejected_samples: List[PathSample] = []

    for center_y in range(height - 1 - half_band, half_band - 1, -SCAN_STEP_PIXELS):
        top = max(0, center_y - half_band)
        bottom = min(height, center_y + half_band + 1)
        regions = _find_regions(mask[top:bottom, :])

        if not started:
            if center_y < minimum_start_y:
                for region in regions:
                    rejected_samples.append(
                        _sample_from_region(
                            region, float(center_y), image_center_x, "unrooted"
                        )
                    )
                continue

            root_region, rejected_regions = _select_root_region(
                regions,
                float(start_x),
                image_center_x,
                expected_root_width,
                allow_root_recovery,
            )
            for region, reason in rejected_regions:
                rejected_samples.append(
                    _sample_from_region(region, float(center_y), start_x, reason)
                )
            if root_region is None:
                continue

            root_sample = _sample_from_region(
                root_region, float(center_y), float(start_x)
            )
            samples.append(root_sample)
            previous_x = root_region.center_x
            started = True
            missing_bands = 0
            continue

        selected = _select_region(regions, previous_x)

        if selected is None:
            if regions:
                nearest = min(regions, key=lambda region: abs(region.center_x - previous_x))
                reason = preview_rejection_reason or "jump"
                rejected = _sample_from_region(
                    nearest, float(center_y), previous_x, reason
                )
                samples.append(rejected)
                rejected_samples.append(rejected)
                previous_x = nearest.center_x
                preview_started = True
                preview_rejection_reason = reason
                missing_bands = 0
                continue
            missing_bands += 1
            if missing_bands > MAX_MISSING_BANDS:
                break
            continue

        region, confidence = selected
        missing_bands = 0
        if preview_started:
            rejected = PathSample(
                region.center_x,
                float(center_y),
                region.width,
                confidence,
                preview_rejection_reason,
            )
            samples.append(rejected)
            rejected_samples.append(rejected)
            previous_x = region.center_x
            continue

        # As primeiras quatro bandas estabilizam a largura da raiz. A faixa pode
        # entrar parcialmente cortada pela borda inferior da imagem e crescer
        # logo acima; tratar isso como corner deixaria apenas um ou dois samples.
        if len(samples) >= MIN_CONTROL_SAMPLES:
            local_width = float(
                np.median(
                    [sample.width for sample in samples[-LOCAL_WIDTH_HISTORY_SAMPLES:]]
                )
            )
            maximum_local_width = max(
                local_width * PATH_MAX_LOCAL_WIDTH_RATIO,
                local_width + PATH_MAX_LOCAL_WIDTH_GROWTH_PIXELS,
            )
            directional_expansion = _is_directional_width_expansion(
                previous_x,
                local_width,
                region.center_x,
                region.width,
            )
            if region.width > maximum_local_width and directional_expansion:
                rejected = PathSample(
                    region.center_x,
                    float(center_y),
                    region.width,
                    confidence,
                    "width",
                )
                samples.append(rejected)
                rejected_samples.append(rejected)
                previous_x = region.center_x
                preview_started = True
                preview_rejection_reason = "width"
                continue

        samples.append(PathSample(region.center_x, float(center_y), region.width, confidence))
        previous_x = region.center_x

    return _PathExtraction(samples, rejected_samples, started)


def extract_path_samples(
    mask: np.ndarray,
    start_x: float,
    expected_root_width: Optional[float] = None,
) -> List[PathSample]:
    """Retorna os candidatos enraizados sem promover geometria distante."""

    return _extract_path_geometry(mask, start_x, expected_root_width).samples


def _segment_heading(first: PathSample, second: PathSample) -> float:
    forward_pixels = first.y - second.y
    if forward_pixels <= 0.0:
        return 0.0
    return math.degrees(math.atan2(second.x - first.x, forward_pixels))


def _find_abrupt_change(samples: Sequence[PathSample]) -> Optional[int]:
    if len(samples) < 4:
        return None

    stable_widths: List[float] = [samples[0].width]
    stable_headings: List[float] = []
    for index in range(1, len(samples)):
        sample = samples[index]
        previous = samples[index - 1]
        if sample.rejection_reason in ("width", "jump"):
            return index
        heading = _segment_heading(previous, sample)
        reference_heading = float(np.median(stable_headings[-3:])) if stable_headings else 0.0
        reference_width = max(1.0, float(np.median(stable_widths[-5:])))
        lateral_jump = abs(sample.x - previous.x)
        heading_jump = _angle_difference_degrees(reference_heading, heading)
        width_ratio = sample.width / reference_width
        width_growth = sample.width - reference_width

        heading_break = (
            heading_jump >= CURRENT_PATH_HEADING_JUMP_DEGREES
            and lateral_jump >= CURRENT_PATH_MIN_LATERAL_JUMP_PIXELS
        )
        width_break = (
            width_ratio >= CURRENT_PATH_WIDTH_RATIO
            and width_growth >= CURRENT_PATH_MIN_WIDTH_GROWTH_PIXELS
            and lateral_jump >= CURRENT_PATH_MIN_LATERAL_JUMP_PIXELS
            and _is_directional_width_expansion(
                previous.x,
                reference_width,
                sample.x,
                sample.width,
            )
        )
        if index >= MIN_CONTROL_SAMPLES and (heading_break or width_break):
            return index

        stable_headings.append(heading)
        stable_widths.append(sample.width)

    return None


def _nearest_x(samples: Sequence[PathSample], proximity: float, height: int, fallback: float) -> float:
    if not samples:
        return fallback
    target_y = proximity * max(1, height - 1)
    return min(samples, key=lambda sample: abs(sample.y - target_y)).x


def _fit_control_geometry(
    current_path: Sequence[PathSample], image_center_x: float, height: int
) -> Tuple[bool, float, float, float, float, float, List[PathSample]]:
    control_samples = [
        sample
        for sample in current_path
        if sample.y / max(1.0, float(height - 1)) >= CONTROL_ZONE_MIN_PROXIMITY
    ]
    nominal_control_y = CONTROL_POINT_PROXIMITY * max(1, height - 1)
    if len(control_samples) < MIN_CONTROL_SAMPLES:
        return (
            False,
            0.0,
            0.0,
            0.0,
            image_center_x,
            nominal_control_y,
            control_samples,
        )

    y_values = np.array([sample.y for sample in control_samples], dtype=np.float64)
    x_values = np.array([sample.x for sample in control_samples], dtype=np.float64)
    control_span = float(np.max(y_values) - np.min(y_values))
    if control_span < MIN_CONTROL_SPAN_PIXELS:
        return (
            False,
            0.0,
            0.0,
            0.0,
            image_center_x,
            nominal_control_y,
            control_samples,
        )

    slope, intercept = np.polyfit(y_values, x_values, 1)
    predicted = slope * y_values + intercept
    residual = float(np.mean(np.abs(x_values - predicted)))
    # Mantém o ponto de controle dentro do trecho realmente observado. O valor
    # nominal continua sendo usado quando está coberto pelos samples; perto de um
    # corner, o ponto é limitado à última geometria válida em vez de extrapolado.
    control_y = float(np.clip(nominal_control_y, np.min(y_values), np.max(y_values)))
    control_x = float(slope * control_y + intercept)
    position_error = control_x - image_center_x
    heading_error = math.degrees(math.atan(-float(slope)))
    sample_confidence = float(np.mean([sample.confidence for sample in control_samples]))
    coverage = _clamp01(len(control_samples) / 7.0)
    fit_confidence = _clamp01(1.0 - residual / 14.0)
    path_confidence = _clamp01(sample_confidence * coverage * fit_confidence)
    return (
        True,
        position_error,
        heading_error,
        path_confidence,
        control_x,
        control_y,
        control_samples,
    )


def _detect_corner(
    samples: Sequence[PathSample], break_index: Optional[int], height: int
) -> CornerObservation:
    if break_index is None or break_index < 2 or break_index >= len(samples):
        return CornerObservation()

    event_sample = samples[break_index]
    stable_samples = samples[max(0, break_index - 4) : break_index]
    stable_width = max(1.0, float(np.median([sample.width for sample in stable_samples])))
    transition_samples: List[PathSample] = []
    for sample in samples[break_index:]:
        if event_sample.y - sample.y > CORNER_ANCHOR_MAX_SUPPORT_PIXELS:
            break
        if sample.width < stable_width * CORNER_ANCHOR_MIN_WIDTH_RATIO:
            break
        transition_samples.append(sample)
    if not transition_samples:
        transition_samples = [event_sample]

    # A âncora Y usa o centro robusto das bandas consecutivas onde a largura
    # deixa de ser normal. Isso remove a borda inferior do blob sem recorrer à
    # bounding box inteira da branch.
    anchor_y = float(np.median([sample.y for sample in transition_samples]))
    stable_y_values = np.array([sample.y for sample in stable_samples], dtype=np.float64)
    stable_x_values = np.array([sample.x for sample in stable_samples], dtype=np.float64)
    if len(stable_samples) >= 2 and float(np.ptp(stable_y_values)) > 0.0:
        anchor_slope, anchor_intercept = np.polyfit(
            stable_y_values, stable_x_values, 1
        )
        anchor_x = float(anchor_slope * anchor_y + anchor_intercept)
    else:
        anchor_x = stable_samples[-1].x
    stable_headings = [
        _segment_heading(stable_samples[index - 1], stable_samples[index])
        for index in range(1, len(stable_samples))
    ]
    stable_heading = float(np.median(stable_headings)) if stable_headings else 0.0
    representative_event = max(transition_samples, key=lambda sample: sample.width)
    event_heading = _segment_heading(samples[break_index - 1], representative_event)
    heading_change = _angle_difference_degrees(stable_heading, event_heading)

    left_edge = min(sample.x - sample.width * 0.5 for sample in transition_samples)
    right_edge = max(sample.x + sample.width * 0.5 for sample in transition_samples)
    left_extension = anchor_x - left_edge
    right_extension = right_edge - anchor_x
    if right_extension > left_extension:
        direction = "RIGHT"
        lateral_extension = right_extension
        opposite_extension = left_extension
    else:
        direction = "LEFT"
        lateral_extension = left_extension
        opposite_extension = right_extension

    directional_excess = lateral_extension - opposite_extension
    directional_ratio = lateral_extension / max(1.0, opposite_extension)
    direction_is_clear = (
        directional_excess >= CORNER_MIN_DIRECTIONAL_EXCESS_PIXELS
        and directional_ratio >= CORNER_MIN_DIRECTIONAL_RATIO
    )

    width_ratio = representative_event.width / stable_width
    center_discontinuity = max(
        abs(sample.x - anchor_x) for sample in transition_samples
    )
    signals = (
        int(heading_change >= CORNER_MIN_HEADING_CHANGE_DEGREES)
        + int(lateral_extension >= CORNER_MIN_LATERAL_EXTENSION_PIXELS)
        + int(width_ratio >= CORNER_MIN_WIDTH_RATIO)
        + int(center_discontinuity >= CURRENT_PATH_MIN_LATERAL_JUMP_PIXELS)
    )
    confidence = _clamp01(
        0.30 * min(heading_change / 90.0, 1.0)
        + 0.25 * min(lateral_extension / 45.0, 1.0)
        + 0.20 * min(max(0.0, width_ratio - 1.0), 1.0)
        + 0.15 * min(center_discontinuity / 30.0, 1.0)
        + 0.10 * float(np.mean([sample.confidence for sample in transition_samples]))
    )
    detected = (
        direction_is_clear
        and signals >= 2
        and confidence >= CORNER_MIN_CONFIDENCE
    )
    # A proximidade nasce no ponto onde o caminho estável termina. O centro e a
    # bounding box da branch não participam do gatilho de execução.
    proximity = _clamp01(anchor_y / max(1.0, float(height - 1)))
    return CornerObservation(
        detected=detected,
        direction=direction if detected else "NONE",
        proximity=proximity,
        confidence=confidence if detected else 0.0,
        anchor_x=anchor_x,
        anchor_y=anchor_y,
    )


def analyze_primary_path(
    mask: np.ndarray,
    start_x: Optional[float] = None,
    expected_root_width: Optional[float] = None,
    allow_root_recovery: bool = False,
) -> PathObservation:
    """Separa geometria próxima e futura sem deixar o Preview controlar direção."""

    if mask.ndim != 2 or mask.size == 0:
        return PathObservation()

    height, width = mask.shape
    image_center_x = width * 0.5
    expected_root_x = image_center_x if start_x is None else float(start_x)
    extraction = _extract_path_geometry(
        mask,
        expected_root_x,
        expected_root_width,
        ROOT_RECOVERY_SEARCH_DEPTH_RATIO
        if allow_root_recovery
        else START_SEARCH_DEPTH_RATIO,
        allow_root_recovery,
    )
    samples = extraction.samples
    all_candidates = list(samples)
    for rejected in extraction.rejected_samples:
        if rejected not in all_candidates:
            all_candidates.append(rejected)

    if not extraction.rooted or not samples:
        return PathObservation(
            samples=all_candidates,
            rejected_samples=list(extraction.rejected_samples),
            near_path_x=image_center_x,
            mid_path_x=image_center_x,
            far_path_x=image_center_x,
            control_x=image_center_x,
            control_y=CONTROL_POINT_PROXIMITY * max(1, height - 1),
            expected_root_x=expected_root_x,
        )

    break_index = _find_abrupt_change(samples)
    lookahead_index = len(samples)
    for index, sample in enumerate(samples):
        proximity = sample.y / max(1.0, float(height - 1))
        if proximity < CURRENT_PATH_MIN_PROXIMITY:
            lookahead_index = index
            break

    current_end = min(lookahead_index, break_index if break_index is not None else len(samples))
    current_end = max(1, current_end)
    current_path = list(samples[:current_end])
    preview_path = list(samples[current_end:])
    (
        valid,
        position_error,
        heading_error,
        confidence,
        control_x,
        control_y,
        control_samples,
    ) = _fit_control_geometry(current_path, image_center_x, height)
    root_sample = samples[0]

    return PathObservation(
        line_detected=valid,
        current_path_valid=valid,
        samples=all_candidates,
        current_path=current_path,
        preview_path=preview_path,
        rejected_samples=list(extraction.rejected_samples),
        control_samples=control_samples,
        control_sample_count=len(control_samples),
        position_error_pixels=position_error,
        heading_error_degrees=heading_error,
        path_confidence=confidence,
        near_path_x=_nearest_x(samples, 0.82, height, image_center_x),
        mid_path_x=_nearest_x(samples, 0.55, height, image_center_x),
        far_path_x=_nearest_x(samples, 0.25, height, image_center_x),
        control_x=control_x,
        control_y=control_y,
        expected_root_x=expected_root_x,
        root_x=root_sample.x,
        root_width=root_sample.width,
        corner=_detect_corner(samples, break_index, height),
    )


def path_x_at_y(samples: Sequence[PathSample], y: float, fallback_x: float) -> float:
    """Retorna o X local do caminho mais próximo da altura informada."""

    if not samples:
        return fallback_x
    return min(samples, key=lambda sample: abs(sample.y - y)).x
