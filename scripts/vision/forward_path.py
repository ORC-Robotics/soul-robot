"""Valida presença de fita frontal, com referência inferior opcional para debug."""

import math

import numpy as np

from .camera_config import FORWARD_PATH_CONFIG, GAP_VALIDATION_CONFIG, FORWARD_PRESENCE_CONFIG
from .line_presence import analyze_line_presence, row_runs


def finite_reference(reference, now):
    """Recusa referências incompletas, futuras ou antigas antes da extrapolação."""
    if not isinstance(reference, dict):
        return False
    try:
        return (reference.get("valid") is True
                and 0 <= now - float(reference["timestamp"]) <= GAP_VALIDATION_CONFIG["reference_timeout"]
                and type(reference["sequence"]) is int and reference["sequence"] > 0
                and all(math.isfinite(float(reference[k])) for k in ("position", "slope", "curvature"))
                and abs(reference["position"]) <= 1.5
                and abs(reference["slope"]) <= 4 and abs(reference["curvature"]) <= 8)
    except (KeyError, TypeError, ValueError):
        return False


def bottom_reference(mask, fusion, timestamp, sequence):
    """Amostra somente a linha escolhida pela inferior, sem mudar sua seleção."""
    if not fusion.get("valid") or not fusion.get("nearPoint") or not fusion.get("farPoint"):
        return None
    near, far = fusion["nearPoint"], fusion["farPoint"]
    height, width = mask.shape
    if near["y"] - far["y"] < height * 0.15:
        return None
    samples = []
    # A reta entre os pontos Fusion apenas escolhe um run; cada centro amostrado
    # continua sendo uma observação real, inclusive durante uma curva.
    for y in np.linspace(near["y"], far["y"], 9).astype(int):
        if not 0 <= y < height:
            continue
        expected_x = near["x"] + (far["x"] - near["x"]) * (near["y"] - y) / (near["y"] - far["y"])
        runs = row_runs(mask[y])
        if not runs:
            continue
        start, end = min(runs, key=lambda r: abs((r[0] + r[1] - 1) / 2 - expected_x))
        center = (start + end - 1) / 2
        if abs(center - expected_x) > width * 0.25 or end - start > width * 0.40:
            continue
        samples.append(((near["y"] - y) / height, 2 * center / (width - 1) - 1))
    if len(samples) < 4:
        return None
    values = np.asarray(samples)
    curve = np.polyfit(values[:, 0], values[:, 1], 2)
    if np.max(np.abs(np.polyval(curve, values[:, 0]) - values[:, 1])) > 0.12:
        return None
    end = values[-1, 0]
    position = float(np.polyval(curve, end))
    return {"valid": True, "timestamp": timestamp, "sequence": sequence,
            "position": position, "slope": float(np.clip(2 * curve[0] * end + curve[1], -4, 4)),
            "curvature": float(np.clip(2 * curve[0], -8, 8)),
            "side": "LEFT" if position < -0.20 else "RIGHT" if position > 0.20 else "CENTER"}


def predicted_path(reference, depth, config):
    advance = config["projection_near_offset"] + depth * config["projection_depth_scale"]
    return config["bottom_x_scale"] * (reference["position"] + reference["slope"] * advance
                                       + 0.5 * reference["curvature"] * advance ** 2)


class ForwardPathTracker:
    """Classifica fita frontal; a previsão inferior serve somente ao diagnóstico e desempate."""

    def __init__(self, config=None):
        self.config = dict(FORWARD_PATH_CONFIG if config is None else config)

    def analyze(self, mask, roi, reference, now, min_component_area=120):
        """Aceita qualquer faixa plausível próxima, sem veto por heading ou posição inferior."""
        cfg = dict(FORWARD_PRESENCE_CONFIG)
        cfg["min_area"] = min_component_area / (mask.shape[1] ** 2)
        presence = analyze_line_presence(mask, roi, cfg)
        valid_ref = reference if finite_reference(reference, now) else None
        accepted = [c for c in presence["candidates"] if c["state"] == "PRESENT"]
        uncertain = [c for c in presence["candidates"] if c["state"] == "UNCERTAIN" and c["bands"]]
        width = mask.shape[1]
        def rank(candidate):
            near = max(candidate["bands"], key=lambda p: p["y"])
            lateral = 2 * near["x"] / (width - 1) - 1
            expected = valid_ref["position"] if valid_ref else 0
            return (-near["y"], abs(lateral - expected))
        best = min(accepted or uncertain, key=rank, default=None)
        position = None
        if best:
            near = max(best["bands"], key=lambda p: p["y"])
            position = float(np.clip(2 * near["x"] / (width - 1) - 1, -1, 1))
        x0, y0, x1, y1 = roi
        return {"forwardLineVisible": bool(accepted or uncertain), "forwardLinePosition": position,
                "forwardLineConfidence": presence["score"], "forwardPathConfidence": presence["score"],
                "forwardPathState": presence["state"], "forwardLinePresent": presence["present"],
                "forwardPathReferenceValid": valid_ref is not None,
                "forwardPathReferenceSequence": valid_ref["sequence"] if valid_ref else 0,
                "forwardPathReferenceTimestamp": valid_ref["timestamp"] if valid_ref else 0.0,
                "forwardPathComponents": {k: best[k] for k in ("score", "extent", "thickness", "elongation")} if best else {},
                "candidates": presence["candidates"], "selectedBands": best["bands"] if best else [],
                "prediction": [(int(round((float(predicted_path(valid_ref, d, self.config)) + 1) * (width - 1) / 2)),
                                int(round(y1 - 1 - d * (y1 - y0 - 1)))) for d in np.linspace(0, 1, 15)] if valid_ref else []}
