"""Consolida os indicadores e o pior caso sem ajustar pesos aos resultados."""

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from line_calibration import write_json
from vision.camera_config import FUSION_TARGET_STABLE_SHIFT_WIDTH_RATIO


def contrast_auc(gray, positive, negative):
    """Probabilidade de um pixel de fita ser mais escuro que um pixel de fundo."""
    if not positive.any() or not negative.any():
        return None
    tape = np.bincount(gray[positive], minlength=256).astype(float)
    floor = np.bincount(gray[negative], minlength=256).astype(float)
    # Empates contam metade; histogramas evitam comparar todos os pares de pixels.
    brighter = floor.sum() - np.cumsum(floor) + 0.5 * floor
    return float(np.dot(tape, brighter) / (tape.sum() * floor.sum()))


def comparison_row(path, region_definitions):
    """Acrescenta contraste e regiões difíceis a todos os ensaios pelo mesmo critério."""
    metrics = json.loads((path / "metrics.json").read_text(encoding="utf-8"))
    config = json.loads((path / "config.json").read_text(encoding="utf-8"))
    records = json.loads((path / "frames.json").read_text(encoding="utf-8"))
    dataset = Path(config["dataset"])
    annotation = config["annotation"]
    shape = tuple(annotation["shape"])
    expected = np.zeros(shape, np.uint8)
    region = np.zeros(shape, np.uint8)
    for polygon in annotation["line_polygons"]:
        cv2.fillPoly(expected, [np.asarray(polygon, np.int32)], 255)
    for polygon in annotation["evaluation_polygons"]:
        cv2.fillPoly(region, [np.asarray(polygon, np.int32)], 255)
    margin = annotation["boundary_uncertainty_px"]
    kernel = np.ones((2 * margin + 1, 2 * margin + 1), np.uint8)
    positive = (cv2.erode(expected, kernel) > 0) & (region > 0)
    expected_with_uncertainty = cv2.dilate(expected, kernel) > 0
    negative = ~expected_with_uncertainty & (region > 0)
    # A marcação verde continua contando como falso positivo na máscara preta;
    # apenas a distribuição chamada piso branco precisa excluí-la.
    contrast_negative = negative.copy()
    non_floor = np.zeros(shape, np.uint8)
    for polygon in annotation.get("non_floor_polygons", []):
        cv2.fillPoly(non_floor, [np.asarray(polygon, np.int32)], 255)
    contrast_negative[non_floor > 0] = False
    regions = {}
    diagnostic_regions = {**annotation.get("diagnostic_regions", {}),
                          **region_definitions.get(annotation["scenario"], {})}
    for name, polygons in diagnostic_regions.items():
        selected = np.zeros(shape, np.uint8)
        cv2.fillPoly(selected, [np.asarray(p, np.int32) for p in polygons], 255)
        regions[name] = {"positive": positive & (selected > 0), "negative": negative & (selected > 0),
                         "recall": [], "specificity": []}
    auc, connectivity, target_within_uncertainty = [], [], []
    expected_count, expected_labels = cv2.connectedComponents(expected)
    for record in records:
        if positive.any():
            target = record["trajectory"].get("farPoint")
            # Usa a mesma margem quadrada já aplicada às bordas dos pixels.
            # O indicador estrito continua no JSON; um alvo na borda incerta
            # não deve zerar sozinho o score de uma saída lateral correta.
            target_within_uncertainty.append(bool(
                record["trajectory"].get("valid") and target
                and 0 <= target["y"] < shape[0] and 0 <= target["x"] < shape[1]
                and expected_with_uncertainty[target["y"], target["x"]]))
        name = record["sample_id"] + ".png"
        frame = cv2.imread(str(dataset / "raw" / name))
        value = contrast_auc(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), positive, contrast_negative)
        if value is not None:
            auc.append(value)
        active = cv2.imread(str(path / "mask" / name), cv2.IMREAD_GRAYSCALE) > 0
        if positive.any():
            _count, labels = cv2.connectedComponents(active.astype(np.uint8))
            covered = 0
            for label in range(1, expected_count):
                overlaps = np.bincount(labels[positive & (expected_labels == label)])
                if overlaps.size:
                    overlaps[0] = 0
                    covered += int(overlaps.max())
            connectivity.append(covered / np.count_nonzero(positive))
        for data in regions.values():
            if data["positive"].any():
                data["recall"].append(float(active[data["positive"]].mean()))
            if data["negative"].any():
                data["specificity"].append(float(1 - active[data["negative"]].mean()))
    summary = metrics["summary"]
    mean = lambda key: summary.get(key, {}).get("mean")
    components = dict(metrics["score_components"])
    if target_within_uncertainty:
        components.pop("trajectory_target_on_line", None)
        components["trajectory_within_annotation_uncertainty"] = float(np.mean(target_within_uncertainty))
    if auc:
        components["contrast_auc"] = float(np.mean(auc))
    if connectivity:
        # Cada trecho verdadeiro pode ser separado por um gap anotado; só a
        # fragmentação dentro do mesmo trecho reduz esta continuidade.
        components["line_connectivity_worst_frame"] = float(min(connectivity))
    if mean("width_relative_error") is not None:
        components["width_fidelity"] = max(0, 1 - mean("width_relative_error"))
    confidences = [r["trajectory"].get("targetConsistency") for r in records[1:]
                   if r["trajectory"].get("valid")]
    confidences = [c for c in confidences if c is not None]
    if confidences:
        # O primeiro frame do replay não tem histórico. A escala de deslocamento
        # é a tolerância já usada pelo Fusion, não um peso ajustado neste ensaio.
        components["trajectory_confidence"] = float(np.mean(confidences))
    jitter95 = summary.get("trajectory_jitter_px", {}).get("p95")
    if jitter95 is not None:
        tolerance = FUSION_TARGET_STABLE_SHIFT_WIDTH_RATIO * shape[1]
        components["trajectory_shift_stability"] = max(0, 1 - jitter95 / tolerance)
    for name, data in regions.items():
        for key in ("recall", "specificity"):
            if data[key]:
                components[f"{name}_{key}_worst_frame"] = float(min(data[key]))
    complete = all(v is not None for v in components.values())
    score = 100 * min(components.values()) if complete else None
    # O menor recall por frame expõe falhas breves que a média esconderia.
    # Os percentis dos outros indicadores permanecem no relatório original.
    worst_frame_recall = min((r["line_recall"] for r in records if r["line_recall"] is not None), default=None)
    camera_config = json.loads((dataset / "config.json").read_text(encoding="utf-8"))
    metadata = [r["camera_metadata"] for r in records]
    controls = {}
    for key in ("ExposureTime", "AnalogueGain", "DigitalGain", "ColourGains"):
        values = [r[key] for r in metadata if r.get(key) is not None]
        if values:
            array = np.asarray(values)
            controls[key] = {"min": array.min(axis=0).tolist(), "max": array.max(axis=0).tolist()}
    timestamps = [r.get("SensorTimestamp") for r in metadata]
    cadence = None
    if all(t is not None for t in timestamps) and len(timestamps) > 1:
        intervals = np.diff(timestamps) / 1e6
        cadence = {"min_ms": float(intervals.min()), "max_ms": float(intervals.max()),
                   "mean_ms": float(intervals.mean()), "fps": float(1000 / intervals.mean()),
                   "duplicate_or_backward_timestamps": int(np.count_nonzero(intervals <= 0))}
    return {"dataset": dataset.name, "experiment": path.name, "metrics_path": str(path),
            "scenario": annotation["scenario"], "segmentation": config["segmentation"],
            "frames": len(records), "score": score, "score_components": components,
            "worst_frame_line_recall": worst_frame_recall, "camera_metadata_ranges": controls,
            "requested_camera_controls": camera_config.get("requested_camera_controls"),
            "cadence": cadence, "summary": summary}


def group_configurations(rows):
    """Compara o pior cenário sem dar mais peso ao cenário com mais capturas."""
    groups = {}
    for row in rows:
        segmentation = dict(row["segmentation"])
        # Ausência dessas opções corresponde a false no código. Normalizar
        # impede separar configurações que produzem exatamente a mesma máscara.
        segmentation.setdefault("line_exclude_green", False)
        segmentation.setdefault("calibration_exclude_green", False)
        segmentation.setdefault("line_min_threshold", 0)
        settings = {"camera_controls": row["requested_camera_controls"],
                    "segmentation": segmentation}
        key = hashlib.sha256(json.dumps(settings, sort_keys=True).encode()).hexdigest()[:12]
        group = groups.setdefault(key, {"id": key, **settings, "scenarios": {}})
        scenario = group["scenarios"].setdefault(row["scenario"], [])
        scenario.append({"dataset": row["dataset"], "experiment": row["experiment"],
                         "score": row["score"], "frames": row["frames"]})
    for group in groups.values():
        complete = all(r["score"] is not None for captures in group["scenarios"].values() for r in captures)
        group["scenario_count"] = len(group["scenarios"])
        group["worst_scenario_score"] = min(r["score"] for captures in group["scenarios"].values() for r in captures) if complete else None
    return list(groups.values())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiments", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--regions", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    regions = json.loads(args.regions.read_text(encoding="utf-8")) if args.regions else {}
    rows = [comparison_row(p.parent, regions) for p in sorted(args.experiments.rglob("metrics.json"))]
    result = {"score_version": 3, "score_rule": "100 * min(quality components), then worst scenario",
              "rationale": "Bottleneck score avoids compensation of missed tape by clean background. No fitted weights. Width fidelity is 1 minus relative width error against manual geometry. Contrast AUC is an empirical probability. Named difficult regions prevent dilution of local failures.",
              "limitations": "Scores are exploratory. No full run recommendation without distinct physical scenarios and holdout captures. Isolated component counts are reported, not penalized independently from their false positive pixels. Genuine gaps require separate line polygons. V3 applies the existing annotation boundary uncertainty to the Fusion target too; strict target-on-tape remains in summary.",
              "diagnostic_regions": regions,
              "trajectory_shift_tolerance_width_ratio": FUSION_TARGET_STABLE_SHIFT_WIDTH_RATIO,
              "configurations": group_configurations(rows), "rows": rows}
    write_json(args.output / "comparison.json", result)
    lines = ["# Comparação experimental", "", "Score v3: mínimo dos indicadores; componentes completos em `comparison.json`.",
             "A incerteza da anotação também vale para o alvo Fusion; o indicador estrito permanece no JSON.",
             "Os scores v1 dos ensaios originais e o snapshot v2 foram preservados para rastreabilidade.", "",
             "| Captura | Segmentação | Frames | Falsos componentes | FP (px) | Fita preservada | Jitter centro (px) | Score v3 |",
             "|---|---|---:|---:|---:|---:|---:|---:|"]
    for row in rows:
        summary = row["summary"]
        mean = lambda k: summary.get(k, {}).get("mean")
        display = lambda value: "n/d" if value is None else f"{value:.4f}"
        lines.append(f"| {row['dataset']} | {row['experiment']} | {row['frames']} | {display(mean('false_component_count'))} | {display(mean('false_positive_area_px'))} | {display(mean('line_recall'))} | {display(mean('center_jitter_px'))} | {display(row['score'])} |")
    lines.extend(["", "Fita preservada é recall nas regiões anotadas, excluindo bordas incertas.",
                  "Jitter do centro e jitter do alvo Fusion são medidas diferentes; consulte os dois.",
                  f"Cenários anotados nesta tabela: {', '.join(sorted({r['scenario'] for r in rows}))}.",
                  "O pior cenário de cada configuração está em comparison.json; isso ainda não equivale a validar uma run."])
    (args.output / "comparison.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({"experiments": len(rows), "output": str(args.output)}))


if __name__ == "__main__":
    main()
