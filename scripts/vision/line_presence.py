"""Mede fita física em uma ROI sem decidir steering ou alterar a máscara."""

import cv2
import numpy as np


def row_runs(row):
    """Separa intervalos do mesmo componente em uma banda de amostragem."""
    edges = np.diff(np.pad((row > 0).astype(np.int8), (1, 1)))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def analyze_line_presence(mask, roi, config):
    """Exige espessura, extensão e bandas conectadas, incluindo fita lateral."""
    x0, y0, x1, y1 = roi
    scale = mask.shape[1]
    crop = mask[y0:y1, x0:x1]
    count, labels, stats, _ = cv2.connectedComponentsWithStats(crop, connectivity=8)
    candidates = []
    # Área só limita trabalho em partículas óbvias; nunca confirma presença.
    eligible = sorted(range(1, count), key=lambda i: int(stats[i, cv2.CC_STAT_AREA]), reverse=True)
    examined = 0
    overloaded = False
    for label in eligible:
        x, y, width, height, area = map(int, stats[label])
        candidate = {"box": [x + x0, y + y0, width, height], "state": "ABSENT",
                     "score": 0.0, "bands": [], "reason": "SHORT_OR_THIN"}
        candidates.append(candidate)
        if max(width, height) < config["min_extent"] * scale or area < config["min_area"] * scale * scale:
            continue
        examined += 1
        if examined > config["max_components"]:
            overloaded = True
            candidate.update(state="UNCERTAIN", reason="COMPONENT_LIMIT")
            continue
        component = labels[y:y + height, x:x + width] == label
        best = None
        # As duas orientações evitam tratar uma curva quase horizontal como GAP.
        for horizontal in (False, True):
            view = component.T if horizontal else component
            chains = []
            finished = []
            for index in np.unique(np.linspace(0, view.shape[0] - 1, config["bands"]).astype(int)):
                runs = [(a, b) for a, b in row_runs(view[index])
                        if config["min_width"] * scale <= b - a <= config["max_width"] * scale]
                # Mantém apenas a cadeia mais longa por intervalo desta banda.
                next_chains = []
                for start, end in runs:
                    center = (start + end - 1) / 2
                    previous = [c for c in chains if abs(center - c[-1][1]) <=
                                config["max_slope"] * (index - c[-1][0])
                                and index - c[-1][0] <= (config["max_missing_bands"] + 1) *
                                ((view.shape[0] - 1) / (config["bands"] - 1) + 1)]
                    chain = max(previous, key=len, default=[])
                    next_chains.append(chain + [(int(index), float(center), int(end - start))])
                if next_chains:
                    finished.extend(chains)
                    chains = next_chains
            for chain in chains + finished:
                if len(chain) < 2:
                    continue
                extent = chain[-1][0] - chain[0][0]
                thickness = float(np.median([p[2] for p in chain]))
                elongation = extent / max(thickness, 1)
                clipped = (x == 0 or x + width == crop.shape[1]) if horizontal else y + height == crop.shape[0]
                required_elongation = config["clipped_elongation"] if clipped else config["min_elongation"]
                score = min(1.0, len(chain) / config["min_bands"],
                            extent / (config["min_extent"] * scale), elongation / required_elongation)
                bands = [{"x": x + x0 + (p[0] if horizontal else p[1]),
                          "y": y + y0 + (p[1] if horizontal else p[0]), "width": p[2],
                          "horizontal": horizontal} for p in chain]
                evidence = {"score": score, "bands": bands, "extent": extent,
                            "thickness": thickness, "elongation": elongation, "clipped": clipped,
                            "nearBands": sum(p["y"] >= y1 - config["near_fraction"] * (y1 - y0) for p in bands)}
                if best is None or (min(score, evidence["nearBands"] / config["near_bands"]), len(bands)) > (
                        min(best["score"], best["nearBands"] / config["near_bands"]), len(best["bands"])):
                    best = evidence
        if best:
            candidate.update(best)
            near_bands = sum(p["y"] >= y1 - config["near_fraction"] * (y1 - y0) for p in best["bands"])
            shape_present = best["score"] >= 1.0
            candidate["state"] = "PRESENT" if shape_present and near_bands >= config["near_bands"] else "UNCERTAIN"
            if best["score"] < config["uncertain_threshold"]:
                candidate["state"] = "ABSENT"
            candidate["reason"] = "PLAUSIBLE_TAPE" if candidate["state"] == "PRESENT" else "INSUFFICIENT_SUPPORT"
    present = [c for c in candidates if c["state"] == "PRESENT"]
    uncertain = [c for c in candidates if c["state"] == "UNCERTAIN"]
    state = "PRESENT" if present else "UNCERTAIN" if uncertain or overloaded else "ABSENT"
    return {"state": state, "present": bool(present), "candidates": candidates,
            "roi": list(roi), "score": max((c["score"] for c in candidates), default=0.0)}


def draw_near_presence_overlay(frame, evidence, command):
    """Mostra a faixa NEAR usada pelo gate de GAP e sua decisão temporal."""
    roi = evidence.get("roi")
    if isinstance(roi, (list, tuple)) and len(roi) == 4:
        color = (0, 255, 0) if evidence.get("present") else (0, 0, 255)
        cv2.rectangle(
            frame,
            (int(roi[0]), int(roi[1])),
            (int(roi[2]), int(roi[3])),
            color,
            1,
        )
    for candidate in evidence.get("candidates", []):
        color = (0, 255, 0) if candidate["state"] == "PRESENT" else (0, 255, 255) if candidate["state"] == "UNCERTAIN" else (0, 0, 255)
        for band in candidate["bands"]:
            cv2.circle(frame, (round(band["x"]), round(band["y"])), 2, color, -1)
    near_value = evidence.get("sensorValue")
    near_percent = (
        f" {100.0 * float(near_value):.1f}%"
        if isinstance(near_value, (int, float))
        else ""
    )
    gap_decision = command.get("gapValidationDecision", "NORMAL")
    if command.get("fusionLateralCurveContinuation"):
        bottom_fusion_state = "CURVE"
    elif command.get("bottomFusionReacquireReady"):
        bottom_fusion_state = "READY"
    elif command.get("bottomFusionPathConnected"):
        bottom_fusion_state = "LINK"
    elif gap_decision in ("CHECKING", "GAP", "LOST"):
        bottom_fusion_state = "REJECT"
    else:
        bottom_fusion_state = "WAIT"
    lines = (f"NEAR-ALL {command.get('nearLineState', 'UNKNOWN')}{near_percent}",
             f"B-FAR {'PRESENT' if command.get('bottomFarLinePresent') else 'ABSENT'} "
             f"{command.get('bottomFarPresentFrames', 0)}",
              f"B-FUSION {bottom_fusion_state} "
              f"{command.get('bottomFusionReacquireFrames', 0)}",
              f"FWD {command.get('forwardPresenceState', 'UNCERTAIN')}",
              f"DECISION {gap_decision}")
    cv2.rectangle(frame, (4, 78), (214, 166), (25, 25, 25), -1)
    for index, text in enumerate(lines):
        cv2.putText(frame, text, (8, 92 + 17 * index), cv2.FONT_HERSHEY_SIMPLEX,
                    0.43, (0, 255, 255), 1, cv2.LINE_AA)
