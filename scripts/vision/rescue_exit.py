"""Evidências da saída; reutiliza máscaras existentes e nunca comanda motores."""

import json
import math
import time

import cv2
import numpy as np

from .forward_path import ForwardPathTracker
from .gap_validation import bottom_fusion_path_is_connected
from .camera_config import FORWARD_PRESENCE_CONFIG, GAP_VALIDATION_CONFIG


CONTROL_PATH = "/dev/shm/obr_rescue_exit_control.json"
# Cinco setores e três profundidades mantêm o diagnóstico legível na CAM1.
SECTOR_COUNT = 5
DEPTH_COUNT = 3
# Reduz somente a análise topológica da CAM0; a segmentação original é preservada.
TOPOLOGY_MAX_WIDTH = 160
TOPOLOGY_MAX_ITERATIONS = 160
# Uma massa que ocupa metade do piso não fornece geometria confiável de fita.
TOPOLOGY_MAX_BLACK_RATIO = 0.50
# Pesos normalizados: MID pesa mais porque está próximo do ponto de transferência.
FAR_AIM_WEIGHT = 0.4
MID_AIM_WEIGHT = 0.6
MIN_BAND_PIXELS = 3
MIN_BAND_COMPONENT_FRACTION = 0.10
TOPOLOGY_MIN_COMPONENT_RATIO = 0.001


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
    """Aceita apenas o heartbeat recente da execução autônoma atual."""
    try:
        with open(path, encoding="utf-8") as source:
            data = json.load(source)
        timestamp = float(data["timestamp"])
        age = (time.time() if now is None else now) - timestamp
        sequence = data["runSequence"]
        if (not math.isfinite(age) or not 0 <= age <= 0.5
                or type(sequence) is not int or sequence <= 0
                or data.get("enabled") is not True):
            return {"enabled": False}
        return data
    except (OSError, ValueError, KeyError, TypeError):
        return {"enabled": False}


def analyze_exit_candidates(mask, min_component_area):
    """Pontua massas pretas por setor e continuidade, sem exigir fita distante."""
    # Apenas a CAM1 importa o pacote de vítimas; a CAM0 não precisa carregar ONNX.
    from ball_vision.main import calculate_horizontal_angle

    height, width = mask.shape
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    tracker = ForwardPathTracker()
    components = []
    # Reutiliza o limite de componentes do detector frontal para limitar o custo por frame.
    eligible = sorted(range(1, count), key=lambda label: int(stats[label, cv2.CC_STAT_AREA]), reverse=True)
    for label in eligible[:FORWARD_PRESENCE_CONFIG["max_components"]]:
        if stats[label, cv2.CC_STAT_AREA] < min_component_area:
            continue
        component = (labels == label).astype(np.uint8) * 255
        ys, xs = np.nonzero(component)
        bands = np.minimum(DEPTH_COUNT - 1, ys * DEPTH_COUNT // height)
        # Uma partícula na borda da banda não deve simular continuidade.
        band_counts = np.bincount(bands, minlength=DEPTH_COUNT)
        supported = band_counts >= max(MIN_BAND_PIXELS, min_component_area * MIN_BAND_COMPONENT_FRACTION)
        depths = int(np.count_nonzero(supported))
        if not depths:
            continue
        nearest = int(np.flatnonzero(supported)[-1])
        steering_points = xs[bands == 1] if supported[1] else xs[bands == 0] if supported[0] else xs
        aim_x = float(np.mean(steering_points))
        if supported[0] and supported[1]:
            aim_x = FAR_AIM_WEIGHT * float(np.mean(xs[bands == 0])) + MID_AIM_WEIGHT * aim_x
        # A geometria detalhada só é necessária quando a candidata alcança a faixa próxima.
        tape_valid = False
        if nearest == DEPTH_COUNT - 1:
            tape = tracker.analyze(component, (0, 0, width, height), None, time.time(), min_component_area)
            tape_valid = tape["forwardLinePresent"]
        components.append((xs, aim_x, depths, nearest, tape_valid))
    result = {}
    for sector in range(SECTOR_COUNT):
        start, end = sector * width // SECTOR_COUNT, (sector + 1) * width // SECTOR_COUNT
        best = None
        for xs, aim_x, depths, nearest, tape_valid in components:
            pixels = int(np.count_nonzero((xs >= start) & (xs < end)))
            if pixels < max(MIN_BAND_PIXELS, min_component_area * MIN_BAND_COMPONENT_FRACTION):
                continue
            ratio = pixels / max(1, (end - start) * height)
            score = min(1.0, ratio * (1.0 + (depths - 1) / DEPTH_COUNT))
            candidate = {"visible": True, "txDegrees": calculate_horizontal_angle(aim_x, width),
                         "score": score, "depthBands": depths, "nearestBand": nearest,
                         "tapeValid": bool(tape_valid)}
            if best is None or score > best["score"]:
                best = candidate
        result[f"sector{sector}"] = best or {"visible": False, "txDegrees": 0.0, "score": 0.0,
                                            "depthBands": 0, "nearestBand": 0, "tapeValid": False}
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


def draw_exit_overlay(frame, reading, control):
    """Mostra setores e diagnóstico do controlador sobre o stream existente."""
    height, width = frame.shape[:2]
    selected = control.get("sector", -1)
    for sector in range(SECTOR_COUNT):
        x = sector * width // SECTOR_COUNT
        candidate = reading.get("exitCandidates", {}).get(f"sector{sector}", {})
        color = (0, 255, 0) if sector == selected else (0, 255, 255)
        cv2.line(frame, (x, 0), (x, height - 1), color, 1)
        cv2.putText(frame, f"{sector}: {candidate.get('score', 0):.3f}", (x + 2, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
    for depth in (1, 2):
        cv2.line(frame, (0, height * depth // DEPTH_COUNT), (width - 1, height * depth // DEPTH_COUNT), (0, 255, 255), 1)
    lines = [str(control.get("phase", "rescue_exit")),
             f"yaw {control.get('heading', 0):.1f} round {control.get('round', 1)} score {control.get('confidence', 0):.3f}",
             f"advance {control.get('advanceCm', 0):.1f} reverse {control.get('reverseCm', 0):.1f} cm",
             str(control.get("rejections", "")), str(control.get("lastFailure", ""))]
    for index, text in enumerate(lines):
        # A fonte simples do OpenCV não possui glifos acentuados.
        text = text.encode("ascii", "replace").decode("ascii")
        cv2.putText(frame, text, (4, height - 65 + index * 13), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
