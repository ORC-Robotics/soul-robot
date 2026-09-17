"""Evidências da saída; reutiliza máscaras existentes e nunca comanda motores."""

import json
import math
import time

import cv2
import numpy as np

from .fusion_guidance import calculate_fusion_style_angle
from .gap_validation import bottom_fusion_path_is_connected
from .camera_config import GAP_VALIDATION_CONFIG
from .line_presence import row_runs
from .rescue_zone import analyze_rescue_zones


CONTROL_PATH = "/dev/shm/obr_rescue_exit_control.json"
# Cinco setores e três profundidades mantêm o diagnóstico legível na CAM1.
SECTOR_COUNT = 5
DEPTH_COUNT = 3
# Reduz somente a análise topológica da CAM0; a segmentação original é preservada.
TOPOLOGY_MAX_WIDTH = 160
TOPOLOGY_MAX_ITERATIONS = 160
# Uma massa que ocupa metade do piso não fornece geometria confiável de fita.
TOPOLOGY_MAX_BLACK_RATIO = 0.50
MIN_BAND_PIXELS = 3
TOPOLOGY_MIN_COMPONENT_RATIO = 0.001
# A saída distante ocupa poucos pixels. A forma e os três frames de
# confirmação filtram o ruído antes de liberar movimento no C++.
DISTANT_COMPONENT_AREA_SCALE = 0.50
# Uma faixa de orientação precisa avançar na imagem. Isso impede que a fita
# transversal da entrada aponte para uma de suas extremidades.
GUIDANCE_MIN_VERTICAL_SPAN_RATIO = 0.07
GUIDANCE_MIN_VERTICAL_SPAN_PIXELS = 8
# O alvo FAR deve estar acima do terço inferior. A fita que marca a soleira
# pode parecer diagonal pela perspectiva, mas continua sendo um alvo lateral.
GUIDANCE_FAR_MAX_Y_RATIO = 0.66
# Uma faixa próxima pode começar abaixo do FAR distante. Ela só é aceita nessa
# região se possuir profundidade suficiente para não ser a soleira transversal.
GUIDANCE_NEAR_FAR_MAX_Y_RATIO = 0.84
GUIDANCE_NEAR_MIN_VERTICAL_SPAN_RATIO = 0.16
GUIDANCE_NEAR_MAX_SLOPE = 2.5
# Uma continuação frontal pode ser bem lateral, mas não quase horizontal como
# rodapés e frestas longas da parede.
GUIDANCE_MAX_NET_SLOPE = 4.5
# Amostras largas representam cruzamentos ou a fita transversal. O rastreador
# pula essas linhas e religa a continuação logo acima, usando apenas preto real.
GUIDANCE_MAX_RUN_WIDTH_RATIO = 0.28
GUIDANCE_SAMPLE_ROWS = 90
GUIDANCE_MAX_GAP_SAMPLES = 6
GUIDANCE_MIN_PATH_POINTS = 4
# A fita real mantém espessura e continuidade ao longo do traçado. Frestas da
# parede até podem atravessar várias linhas, mas possuem pouco suporte por altura.
GUIDANCE_MIN_SUPPORT_WIDTH_RATIO = 0.007
# Uma estrutura que nasce no topo e permanece colada à lateral é moldura/parede,
# não uma faixa no piso. A combinação preserva fitas distantes no topo central.
GUIDANCE_TOP_EDGE_RATIO = 0.05
GUIDANCE_SIDE_EDGE_RATIO = 0.025
GUIDANCE_MAX_SIDE_HUG_FRACTION = 0.55
GUIDANCE_PATH_QUALITY_WEIGHT = 0.12
# O cinza refletivo forma ilhas e falhas dentro da suposta faixa. Qualquer
# sinal ruim impede que a textura seja publicada como saída.
GUIDANCE_MIN_PATH_CONTINUITY = 0.72
GUIDANCE_MIN_PATH_SOLIDITY = 0.65
GUIDANCE_MAX_FRAGMENTED_ROW_RATIO = 0.35
# Segmentação exclusiva da saída. O contraste local recupera fita distante
# sob iluminação desigual sem transformar uma sombra ampla em preto absoluto.
EXIT_BLACK_MAX_GRAY = 195
EXIT_BLACK_MIN_LOCAL_CONTRAST = 10
EXIT_BLACK_ABSOLUTE_GRAY = 75
EXIT_BLACK_BACKGROUND_SIGMA_RATIO = 0.045
EXIT_BLACK_MIN_COMPONENT_PIXELS = 12
EXIT_BLACK_MIN_COMPONENT_AREA_RATIO = 0.00006


def read_camera_control(now=None, path=CONTROL_PATH):
    """Lê parâmetros da CAM0 mesmo quando a busca da saída está desativada."""
    try:
        with open(path, encoding="utf-8") as source:
            data = json.load(source)
        if not isinstance(data, dict):
            return {}
        age = (time.time() if now is None else now) - float(data["timestamp"])
        return data if math.isfinite(age) and 0 <= age <= 0.5 else {}
    except (OSError, ValueError, KeyError, TypeError):
        return {}


def read_exit_control(now=None, path=CONTROL_PATH):
    """Aceita heartbeat recente de controle ou overlay, mantendo enabled separado."""
    try:
        with open(path, encoding="utf-8") as source:
            data = json.load(source)
        timestamp = float(data["timestamp"])
        age = (time.time() if now is None else now) - timestamp
        sequence = data["runSequence"]
        if (not math.isfinite(age) or not 0 <= age <= 0.5
                or type(sequence) is not int or sequence <= 0
                or type(data.get("enabled")) is not bool
                or (data.get("enabled") is not True and
                    data.get("exitOverlayEnabled") is not True)):
            return {"enabled": False}
        return data
    except (OSError, ValueError, KeyError, TypeError):
        return {"enabled": False}


def rescue_zone_blocked_sectors(frame_bgr):
    """Retorna setores ocupados por áreas verdes ou vermelhas da sala."""
    width = frame_bgr.shape[1]
    blocked = set()
    for zone in analyze_rescue_zones(frame_bgr).values():
        hull = zone.get("_hull")
        if not zone.get("detected") or hull is None:
            continue
        x, _, zone_width, _ = cv2.boundingRect(hull)
        for sector in range(SECTOR_COUNT):
            start = sector * width // SECTOR_COUNT
            end = (sector + 1) * width // SECTOR_COUNT
            if x < end and x + zone_width > start:
                blocked.add(sector)
    return blocked


def remove_rescue_zones_from_black(frame_bgr, black_mask):
    """Remove os hulls e veta setores ocupados pelas zonas calibradas."""
    exclusion = np.zeros(black_mask.shape, dtype=np.uint8)
    detected = False
    blocked = set()
    for zone in analyze_rescue_zones(frame_bgr).values():
        hull = zone.get("_hull")
        if not zone.get("detected") or hull is None:
            continue
        cv2.drawContours(exclusion, [hull], -1, 255, -1)
        x, _, zone_width, _ = cv2.boundingRect(hull)
        for sector in range(SECTOR_COUNT):
            start = sector * black_mask.shape[1] // SECTOR_COUNT
            end = (sector + 1) * black_mask.shape[1] // SECTOR_COUNT
            if x < end and x + zone_width > start:
                blocked.add(sector)
        detected = True
    if not detected:
        return black_mask, False, blocked

    # A margem retira a borda escura do material colorido. O setor inteiro
    # também fica bloqueado para impedir uma tentativa em direção ao triângulo.
    margin = max(3, int(round(min(black_mask.shape) * 0.012)))
    if margin % 2 == 0:
        margin += 1
    exclusion = cv2.dilate(
        exclusion,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (margin, margin)),
    )
    return cv2.bitwise_and(black_mask, cv2.bitwise_not(exclusion)), True, blocked


def _empty_exit_candidate(tx_degrees=0.0, blocked_by_color=False):
    """Cria o contrato seguro de um setor sem candidata de saída."""
    return {"visible": False, "txDegrees": float(tx_degrees), "score": 0.0,
            "depthBands": 0, "nearestBand": 0, "tapeValid": False,
            "guidanceValid": False, "guidanceAngleDegrees": 90.0,
            "nearPoint": None, "farPoint": None, "entryPoint": None,
            "entryAngleDegrees": 90.0, "entryOffsetNormalized": 0.0,
            "entryDepthNormalized": 0.0,
            "pathContinuity": 0.0, "pathSolidity": 0.0,
            "fragmentedRowRatio": 0.0, "grayNoiseLikely": False,
            "solidBlack": False, "blockedByColor": bool(blocked_by_color)}


def empty_exit_candidates(frame_width):
    """Publica o contrato seguro sem executar a geometria pesada da saída."""
    del frame_width
    return {
        f"sector{sector}": _empty_exit_candidate()
        for sector in range(SECTOR_COUNT)
    }


def _measure_path_surface(mask, path):
    """Mede se a faixa é preenchida ou formada por reflexos fragmentados."""

    top_y = int(path[-1][1])
    bottom_y = int(path[0][1])
    y_samples = np.asarray([point[1] for point in reversed(path)], dtype=np.float64)
    x_samples = np.asarray([point[0] for point in reversed(path)], dtype=np.float64)
    width_samples = np.asarray([point[2] for point in reversed(path)], dtype=np.float64)
    # A mesma grade de amostragem da geometria evita varrer centenas de linhas
    # extras em 960x540, preservando a taxa de frames do seguidor frontal.
    rows = np.linspace(top_y, bottom_y, min(90, bottom_y - top_y + 1),
                       dtype=np.int32)
    center_samples = np.interp(rows, y_samples, x_samples)
    expected_widths = np.interp(rows, y_samples, width_samples)
    present_rows = 0
    fragmented_rows = 0
    solidities = []
    for y, center_x, width_value in zip(rows, center_samples, expected_widths):
        expected_width = max(3.0, float(width_value))
        # Observa também as laterais da trilha. A fita preta gera um único run;
        # o piso cinza refletivo costuma gerar várias ilhas paralelas próximas.
        half_width = max(
            3,
            int(round(expected_width * 1.5)),
            int(round(mask.shape[1] * 0.015)),
        )
        start = max(0, int(round(center_x)) - half_width)
        end = min(mask.shape[1], int(round(center_x)) + half_width + 1)
        corridor = mask[y, start:end]
        if not corridor.size:
            solidities.append(0.0)
            continue
        runs = row_runs(corridor)
        if runs:
            present_rows += 1
            # A faixa preta ocupa sobretudo um único run transversal. Duas
            # ilhas com o mesmo total de pixels possuem solidez menor.
            largest_run = max(end - start for start, end in runs)
            occupied_span = runs[-1][1] - runs[0][0]
            solidities.append(largest_run / max(1, occupied_span))
        else:
            solidities.append(0.0)
        if len(runs) > 1:
            fragmented_rows += 1

    row_count = max(1, len(rows))
    continuity = present_rows / float(row_count)
    solidity = float(np.median(solidities)) if solidities else 0.0
    fragmentation = fragmented_rows / float(row_count)
    failed_signals = sum((
        continuity < GUIDANCE_MIN_PATH_CONTINUITY,
        solidity < GUIDANCE_MIN_PATH_SOLIDITY,
        fragmentation > GUIDANCE_MAX_FRAGMENTED_ROW_RATIO,
    ))
    return continuity, solidity, fragmentation, failed_signals >= 1


def create_exit_black_mask(frame_bgr):
    """Segmenta preto distante por contraste local, sem sensores inferiores."""
    if not isinstance(frame_bgr, np.ndarray) or frame_bgr.ndim != 3:
        raise ValueError("A segmentação da saída exige um frame BGR.")
    gray = cv2.cvtColor(frame_bgr[:, :, :3], cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 3)
    sigma = max(3.0, min(gray.shape) * EXIT_BLACK_BACKGROUND_SIGMA_RATIO)
    local_background = cv2.GaussianBlur(gray, (0, 0), sigmaX=sigma, sigmaY=sigma)
    contrast = local_background.astype(np.int16) - gray.astype(np.int16)
    accepted = (
        ((gray <= EXIT_BLACK_MAX_GRAY) &
         (contrast >= EXIT_BLACK_MIN_LOCAL_CONTRAST))
        | (gray <= EXIT_BLACK_ABSOLUTE_GRAY)
    )
    mask = accepted.astype(np.uint8) * 255
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
    )

    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    filtered = np.zeros_like(mask)
    minimum_component_pixels = max(
        EXIT_BLACK_MIN_COMPONENT_PIXELS,
        int(round(mask.size * EXIT_BLACK_MIN_COMPONENT_AREA_RATIO)),
    )
    for label in range(1, count):
        if int(stats[label, cv2.CC_STAT_AREA]) >= minimum_component_pixels:
            filtered[labels == label] = 255
    return filtered


def _trace_black_continuations(mask, min_component_area):
    """Rastreia centros pretos por altura, sem sensores ou referência externa."""
    height, width = mask.shape
    sample_step = max(2, height // GUIDANCE_SAMPLE_ROWS)
    maximum_gap = sample_step * GUIDANCE_MAX_GAP_SAMPLES
    maximum_run_width = max(8, int(round(width * GUIDANCE_MAX_RUN_WIDTH_RATIO)))
    paths = []

    # Caminha de baixo para cima. Um cruzamento largo não vira alvo; ele apenas
    # cria um pequeno gap que pode ser atravessado pela mesma continuação.
    for y in range(height - 1, -1, -sample_step):
        observations = []
        for start, end in row_runs(mask[y]):
            run_width = end - start
            if 2 <= run_width <= maximum_run_width:
                observations.append(((start + end - 1) / 2.0, run_width))

        pairs = []
        for path_index, path in enumerate(paths):
            previous_x, previous_y, previous_width = path[-1]
            gap = previous_y - y
            if not 0 < gap <= maximum_gap:
                continue
            for observation_index, (center_x, run_width) in enumerate(observations):
                maximum_shift = max(
                    width * 0.055,
                    gap * 2.4 + (previous_width + run_width) * 0.35,
                )
                shift = abs(center_x - previous_x)
                if shift <= maximum_shift:
                    pairs.append((shift, path_index, observation_index))

        used_paths = set()
        used_observations = set()
        for _, path_index, observation_index in sorted(pairs):
            if path_index in used_paths or observation_index in used_observations:
                continue
            center_x, run_width = observations[observation_index]
            paths[path_index].append((center_x, y, run_width))
            used_paths.add(path_index)
            used_observations.add(observation_index)
        for observation_index, (center_x, run_width) in enumerate(observations):
            if observation_index not in used_observations:
                paths.append([(center_x, y, run_width)])

    minimum_span = max(GUIDANCE_MIN_VERTICAL_SPAN_PIXELS,
                       int(round(height * GUIDANCE_MIN_VERTICAL_SPAN_RATIO)))
    far_limit_y = int(round((height - 1) * GUIDANCE_FAR_MAX_Y_RATIO))
    minimum_area = max(MIN_BAND_PIXELS * 4,
                       int(round(min_component_area * DISTANT_COMPONENT_AREA_SCALE)))
    minimum_support_width = max(
        3.0, width * GUIDANCE_MIN_SUPPORT_WIDTH_RATIO)
    side_margin = max(3.0, width * GUIDANCE_SIDE_EDGE_RATIO)
    top_margin = max(sample_step * 2, int(round(height * GUIDANCE_TOP_EDGE_RATIO)))
    continuations = []
    for path in paths:
        if len(path) < GUIDANCE_MIN_PATH_POINTS:
            continue
        bottom_y = int(path[0][1])
        top_y = int(path[-1][1])
        vertical_span = bottom_y - top_y + 1
        sampled_area = sum(point[2] for point in path) * sample_step
        if vertical_span < minimum_span or sampled_area < minimum_area:
            continue
        # Usa área por altura, não apenas área total. Assim, uma sequência de
        # frestas pequenas não ganha validade só porque foi religada por gaps.
        if sampled_area / max(1, vertical_span) < minimum_support_width:
            continue

        net_slope = abs(path[-1][0] - path[0][0]) / max(1, vertical_span)
        if net_slope > GUIDANCE_MAX_NET_SLOPE:
            continue

        side_hug_fraction = sum(
            1 for point in path
            if point[0] <= side_margin or point[0] >= width - 1 - side_margin
        ) / len(path)
        if top_y <= top_margin and side_hug_fraction >= GUIDANCE_MAX_SIDE_HUG_FRACTION:
            continue

        if top_y > far_limit_y:
            near_far_limit = int(round((height - 1) * GUIDANCE_NEAR_FAR_MAX_Y_RATIO))
            if (top_y > near_far_limit or
                    vertical_span < height * GUIDANCE_NEAR_MIN_VERTICAL_SPAN_RATIO or
                    net_slope > GUIDANCE_NEAR_MAX_SLOPE):
                continue

        far_samples = path[-min(3, len(path)):]
        entry_samples = path[:min(3, len(path))]
        entry_point = (
            int(round(float(np.median([point[0] for point in entry_samples])))),
            int(round(float(np.median([point[1] for point in entry_samples])))),
        )
        far_point = (
            int(round(float(np.median([point[0] for point in far_samples])))),
            int(round(float(np.median([point[1] for point in far_samples])))),
        )
        near_point = (width // 2, height - 1)
        guidance_angle = calculate_fusion_style_angle(near_point, far_point)
        if guidance_angle is None:
            continue
        entry_angle = calculate_fusion_style_angle(near_point, entry_point)
        if entry_angle is None:
            entry_angle = 90.0
        entry_offset = max(-1.0, min(1.0,
            (entry_point[0] - width * 0.5) / max(1.0, width * 0.5)))
        expected_samples = vertical_span / sample_step + 1.0
        coverage = min(1.0, len(path) / max(1.0, expected_samples))
        path_y = np.asarray([point[1] for point in path], dtype=np.float64)
        path_centers = np.asarray([point[0] for point in path], dtype=np.float64)
        linear_fit = np.polyfit(path_y, path_centers, 1)
        residual = path_centers - np.polyval(linear_fit, path_y)
        smoothness = 1.0 - min(
            1.0, float(np.sqrt(np.mean(residual * residual))) /
            max(2.0, width * 0.035))
        widths = np.asarray([point[2] for point in path], dtype=np.float64)
        median_width = float(np.median(widths))
        width_stability = 1.0 - min(
            1.0, float(np.median(np.abs(widths - median_width))) /
            max(1.0, median_width))
        path_quality = (coverage + smoothness + width_stability) / 3.0
        continuity, solidity, fragmentation, gray_noise = _measure_path_surface(
            mask, path
        )
        solid_black = (
            continuity >= GUIDANCE_MIN_PATH_CONTINUITY and
            solidity >= GUIDANCE_MIN_PATH_SOLIDITY and
            fragmentation <= GUIDANCE_MAX_FRAGMENTED_ROW_RATIO
        )
        path_x = float(far_point[0])
        touched_bands = {min(DEPTH_COUNT - 1, int(point[1]) * DEPTH_COUNT // height)
                         for point in path}
        continuations.append({
            "aimX": path_x,
            "depthBands": len(touched_bands),
            "nearestBand": max(touched_bands),
            "sampledArea": sampled_area,
            "verticalSpan": vertical_span,
            "nearPoint": near_point,
            "farPoint": far_point,
            "entryPoint": entry_point,
            "entryAngleDegrees": float(entry_angle),
            "entryOffsetNormalized": float(entry_offset),
            "entryDepthNormalized": float(entry_point[1] / max(1, height - 1)),
            "guidanceAngleDegrees": float(guidance_angle),
            "pathQuality": path_quality,
            "pathContinuity": continuity,
            "pathSolidity": solidity,
            "fragmentedRowRatio": fragmentation,
            "grayNoiseLikely": gray_noise,
            "solidBlack": solid_black,
            "pathPoints": [[int(round(x)), int(y)] for x, y, _ in path],
        })
    return continuations


def analyze_exit_candidates(mask, min_component_area, blocked_sectors=()):
    """Pontua continuações pretas frontais sem depender de sensores inferiores."""
    # Apenas a CAM1 importa o pacote de vítimas; a CAM0 não precisa carregar ONNX.
    from ball_vision.main import calculate_horizontal_angle

    height, width = mask.shape
    continuations = _trace_black_continuations(mask, min_component_area)
    result = {}
    blocked_sectors = set(blocked_sectors)
    for sector in range(SECTOR_COUNT):
        sector_center_x = (sector + 0.5) * width / SECTOR_COUNT
        sector_tx = calculate_horizontal_angle(sector_center_x, width)
        if sector in blocked_sectors:
            result[f"sector{sector}"] = _empty_exit_candidate(
                sector_tx, blocked_by_color=True
            )
            continue
        start, end = sector * width // SECTOR_COUNT, (sector + 1) * width // SECTOR_COUNT
        best = None
        for continuation in continuations:
            aim_x = continuation["aimX"]
            entry_sector = min(SECTOR_COUNT - 1,
                               int(continuation["entryPoint"][0]) * SECTOR_COUNT // width)
            if entry_sector in blocked_sectors:
                continue
            # Cada continuação pertence a um setor pelo seu centro de mira.
            # Não replica uma faixa diagonal em todos os setores que ela cruza.
            if not start <= aim_x < end:
                continue
            area_score = continuation["sampledArea"] / max(1, width * height)
            depth_score = continuation["verticalSpan"] / max(1, height)
            score = min(1.0, area_score * 4.0 + depth_score * 0.7 +
                        continuation["depthBands"] * 0.05)
            # Entre duas rotas plausíveis, prefere a continuação mais frontal.
            # O bônus não elimina uma saída lateral quando ela é a única visível.
            centrality = 1.0 - min(1.0, abs(aim_x - width * 0.5) /
                                   max(1.0, width * 0.5))
            score = min(1.0, score + centrality * 0.08 +
                        continuation["pathQuality"] * GUIDANCE_PATH_QUALITY_WEIGHT)
            # O intervalo superior pertence sempre a uma faixa sólida. Assim,
            # reflexos com área grande não roubam a direção de uma fita fina.
            score = (0.5 + 0.5 * score) if continuation["solidBlack"] else (0.5 * score)
            gray_noise = continuation["grayNoiseLikely"]
            candidate = {"visible": not gray_noise,
                         "txDegrees": calculate_horizontal_angle(aim_x, width),
                         "score": score,
                         "depthBands": continuation["depthBands"],
                         "nearestBand": continuation["nearestBand"],
                         "tapeValid": False,
                         "guidanceValid": not gray_noise,
                         "guidanceAngleDegrees": continuation["guidanceAngleDegrees"],
                         "nearPoint": {"x": continuation["nearPoint"][0],
                                       "y": continuation["nearPoint"][1]},
                         "farPoint": {"x": continuation["farPoint"][0],
                                      "y": continuation["farPoint"][1]},
                         "entryPoint": {"x": continuation["entryPoint"][0],
                                        "y": continuation["entryPoint"][1]},
                         "entryAngleDegrees": continuation["entryAngleDegrees"],
                         "entryOffsetNormalized": continuation["entryOffsetNormalized"],
                         "entryDepthNormalized": continuation["entryDepthNormalized"],
                         "pathContinuity": continuation["pathContinuity"],
                         "pathSolidity": continuation["pathSolidity"],
                         "fragmentedRowRatio": continuation["fragmentedRowRatio"],
                         "grayNoiseLikely": gray_noise,
                         "solidBlack": continuation["solidBlack"],
                         "blockedByColor": False,
                         "pathPoints": continuation["pathPoints"]}
            # Uma rota sólida sempre vence uma textura cinza fragmentada.
            if (best is None or
                    (candidate["guidanceValid"], score) >
                    (best["guidanceValid"], best["score"])):
                best = candidate
        result[f"sector{sector}"] = best or _empty_exit_candidate(sector_tx)
    return result


def _thin(mask):
    """Afina a fita para contar ramificações, preservando sua conectividade."""
    skeleton = np.pad((mask != 0).astype(np.uint8), 1)
    for _ in range(TOPOLOGY_MAX_ITERATIONS):
        changed = False
        for step in (0, 1):
            center = skeleton[1:-1, 1:-1]
            p = [skeleton[:-2, 1:-1], skeleton[:-2, 2:], skeleton[1:-1, 2:],
                 skeleton[2:, 2:], skeleton[2:, 1:-1], skeleton[2:, :-2],
                 skeleton[1:-1, :-2], skeleton[:-2, :-2]]
            neighbors = sum(p)
            transitions = sum(((p[i] == 0) & (p[(i + 1) % 8] == 1)).astype(np.uint8) for i in range(8))
            if step == 0:
                removable = (p[0] * p[2] * p[4] == 0) & (p[2] * p[4] * p[6] == 0)
            else:
                removable = (p[0] * p[2] * p[6] == 0) & (p[0] * p[4] * p[6] == 0)
            remove = (center == 1) & (neighbors >= 2) & (neighbors <= 6) & (transitions == 1) & removable
            changed = changed or bool(np.any(remove))
            center[remove] = 0
        if not changed:
            return skeleton
    # Não aprova uma topologia cuja análise ainda não convergiu.
    return None


def unbranched_black_path(mask):
    """Aceita uma fita única, reta ou curva, com duas extremidades e sem ramos."""
    if mask is None or mask.ndim != 2 or not mask.size:
        return False
    width = min(mask.shape[1], TOPOLOGY_MAX_WIDTH)
    height = max(1, round(mask.shape[0] * width / mask.shape[1]))
    reduced = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)
    if np.count_nonzero(reduced) >= reduced.size * TOPOLOGY_MAX_BLACK_RATIO:
        return False
    count, labels, stats, _ = cv2.connectedComponentsWithStats((reduced != 0).astype(np.uint8), 8)
    useful = [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >=
              max(MIN_BAND_PIXELS, width * height * TOPOLOGY_MIN_COMPONENT_RATIO)]
    if len(useful) != 1:
        return False
    skeleton = _thin(labels == useful[0])
    if skeleton is None:
        return False
    center = skeleton[1:-1, 1:-1]
    p = [skeleton[:-2, 1:-1], skeleton[:-2, 2:], skeleton[1:-1, 2:],
         skeleton[2:, 2:], skeleton[2:, 1:-1], skeleton[2:, :-2],
         skeleton[1:-1, :-2], skeleton[:-2, :-2]]
    transitions = sum(((p[i] == 0) & (p[(i + 1) % 8] == 1)).astype(np.uint8) for i in range(8))
    endpoints = np.count_nonzero((center == 1) & (sum(p) == 1))
    return bool(endpoints == 2 and not np.any((center == 1) & (transitions >= 3)))


def exit_line_is_unbranched(mask, fusion):
    """Exige que a fita única seja também o caminho FAR/MID escolhido pelo Fusion."""
    return bottom_fusion_path_is_connected(
        mask, fusion, GAP_VALIDATION_CONFIG["bottom_fusion_target_radius_px"]
    ) and unbranched_black_path(mask)


def create_exit_display_frame(raw_frame, black_mask, display_mode):
    """Seleciona imagem real ou máscara integral sem alterar a visão de controle."""
    if str(display_mode).strip().lower() != "line":
        return raw_frame.copy()
    display = np.zeros_like(raw_frame)
    if isinstance(black_mask, np.ndarray) and black_mask.ndim == 2:
        useful_height = min(display.shape[0], black_mask.shape[0])
        useful_width = min(display.shape[1], black_mask.shape[1])
        display[:useful_height, :useful_width][
            black_mask[:useful_height, :useful_width] != 0
        ] = (255, 255, 255)
    return display


def select_exit_guidance(candidates):
    """Escolhe o mesmo alvo válido aceito pela missão C++, sem promover preto isolado."""
    valid = []
    for candidate in candidates.values():
        try:
            angle = float(candidate.get("guidanceAngleDegrees", float("nan")))
            depth = float(candidate.get("entryDepthNormalized", float("nan")))
            score = float(candidate.get("score", float("nan")))
        except (TypeError, ValueError, OverflowError):
            continue
        if (candidate.get("visible") is True and candidate.get("guidanceValid") is True and
                not candidate.get("blockedByColor") and not candidate.get("grayNoiseLikely") and
                math.isfinite(score) and math.isfinite(angle) and 0 <= angle <= 180 and
                math.isfinite(depth) and 0 <= depth <= 1):
            valid.append(candidate)
    return max(valid, key=lambda candidate: float(candidate["score"]), default=None)


def draw_exit_overlay(frame, reading, control, black_mask=None):
    """Mostra Fusion, fase e bloqueios da retomada sem alterar o controle."""
    height, width = frame.shape[:2]
    candidates = reading.get("exitCandidates", {})
    guidance = select_exit_guidance(candidates)
    selected = next((sector for sector in range(SECTOR_COUNT)
                     if candidates.get(f"sector{sector}") is guidance), -1)
    for sector in range(SECTOR_COUNT):
        x = sector * width // SECTOR_COUNT
        candidate = reading.get("exitCandidates", {}).get(f"sector{sector}", {})
        color = (0, 255, 0) if sector == selected else (0, 255, 255)
        cv2.line(frame, (x, 0), (x, height - 1), color, 1)
        block = " C" if candidate.get("blockedByColor") else (
            " G" if candidate.get("grayNoiseLikely") else ""
        )
        cv2.putText(frame, f"{sector}: {candidate.get('score', 0):.3f}{block}", (x + 2, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
    for depth in (1, 2):
        cv2.line(frame, (0, height * depth // DEPTH_COUNT), (width - 1, height * depth // DEPTH_COUNT), (0, 255, 255), 1)
    far_limit_y = int(round((height - 1) * GUIDANCE_FAR_MAX_Y_RATIO))
    cv2.line(frame, (0, far_limit_y), (width - 1, far_limit_y), (255, 180, 0), 1)
    guidance = guidance or {}
    near_point = guidance.get("nearPoint")
    far_point = guidance.get("farPoint")
    entry_point = guidance.get("entryPoint")
    if guidance.get("guidanceValid") and near_point and far_point:
        near = (int(near_point["x"]), int(near_point["y"]))
        far = (int(far_point["x"]), int(far_point["y"]))
        path_points = guidance.get("pathPoints", [])
        if len(path_points) >= 2:
            path = np.asarray(path_points, dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(frame, [path], False, (0, 165, 255), 2, cv2.LINE_AA)
        cv2.line(frame, near, far, (255, 80, 255), 3, cv2.LINE_AA)
        cv2.circle(frame, near, 5, (255, 80, 255), -1)
        cv2.circle(frame, far, 5, (255, 80, 255), -1)
        if entry_point:
            entry = (int(entry_point["x"]), int(entry_point["y"]))
            cv2.line(frame, near, entry, (255, 255, 0), 2, cv2.LINE_AA)
            cv2.circle(frame, entry, 5, (255, 255, 0), -1)
    guidance_text = (
        f"guide {guidance.get('guidanceAngleDegrees', 90.0):.1f} deg "
        f"entry depth {guidance.get('entryDepthNormalized', 0.0):.2f}"
        if guidance.get("guidanceValid") else "guide --"
    )
    surface_text = "NO VALID FUSION" if not guidance else (
        "REFLECTIVE GRAY NOISE"
        if guidance.get("grayNoiseLikely") else "SOLID BLACK"
    )
    quality_text = (
        f"CONT {guidance.get('pathContinuity', 0.0):.2f} "
        f"SOL {guidance.get('pathSolidity', 0.0):.2f} "
        f"FRAG {guidance.get('fragmentedRowRatio', 0.0):.2f}"
    )

    # Resume a mesma máscara exibida integralmente pelo modo LINHA.
    mask_ratio = 0.0
    largest_component = 0
    if isinstance(black_mask, np.ndarray) and black_mask.ndim == 2 and black_mask.size:
        binary_mask = np.where(black_mask != 0, 255, 0).astype(np.uint8)
        mask_ratio = float(np.count_nonzero(binary_mask)) / float(binary_mask.size)
        count, _, stats, _ = cv2.connectedComponentsWithStats(binary_mask, 8)
        if count > 1:
            largest_component = int(np.max(stats[1:, cv2.CC_STAT_AREA]))
        guide_count = sum(1 for item in candidates.values()
                          if item.get("guidanceValid"))
        diagnostic_color = (0, 220, 0) if guide_count else (0, 0, 255)
        diagnostic_lines = (
            f"EXIT BLACK {'GUIDE' if guide_count else 'NO GUIDE'}",
            f"RATIO {mask_ratio:.4f} LARGEST {largest_component}",
            f"CANDIDATES {guide_count}",
            surface_text,
            quality_text,
            str(reading.get("exitColorStatus", "COLOR REARMED")),
        )
        text_x = 8
        for index, diagnostic_text in enumerate(diagnostic_lines):
            text_y = 34 + index * 18
            cv2.putText(frame, diagnostic_text, (text_x, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.43, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(frame, diagnostic_text, (text_x, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.43, diagnostic_color, 1, cv2.LINE_AA)
    # O diagnóstico permanece visível no modo isolado após a entrega à CAM0.
    active_control = control.get("enabled") is True
    failed = control.get("phase") == "rescue_exit_failed"
    if active_control or failed:
        guidance_state = "STOPPED" if failed else control.get("guidanceState") or "WAITING"
        bottom_text = (f"CAM0 {control.get('bottomBlocker', 'WAITING')}  "
                       f"frames {control.get('bottomFrames', 0)}/{control.get('bottomFramesRequired', 4)}")
    else:
        guidance_state = "CAM0 / COURSE"
        bottom_text = "CAM0 HANDED_OFF"
    lines = [str(control.get("phase", "rescue_exit")),
             f"CONTROL {guidance_state}  CAM0 {reading.get('source', 'UNAVAILABLE')}",
             bottom_text,
             f"NO FUSION {control.get('fallbackAdvanceCm', 0):.1f}/{control.get('fallbackMaximumAdvanceCm', 60):.0f} cm",
             guidance_text,
             f"yaw {control.get('heading', 0):.1f} round {control.get('round', 1)} score {control.get('confidence', 0):.3f}",
             f"advance {control.get('advanceCm', 0):.1f} reverse {control.get('reverseCm', 0):.1f} cm",
             str(control.get("rejections", "")), str(control.get("lastFailure", ""))]
    lines = [text for text in lines if text]
    spacing = max(13, min(22, height // 25))
    font_scale = max(0.35, min(0.55, width / 1920.0))
    first_line_y = max(12, height - len(lines) * spacing)
    for index, text in enumerate(lines):
        # A fonte simples do OpenCV não possui glifos acentuados.
        text = text.encode("ascii", "replace").decode("ascii")
        position = (4, first_line_y + index * spacing)
        cv2.putText(frame, text, position, cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, text, position, cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale, (0, 255, 255), 1, cv2.LINE_AA)
