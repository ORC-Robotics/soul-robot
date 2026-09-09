"""Compara segmentações em RGB reais preservados, sem acessar motores ou câmera."""

import argparse
import copy
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from vision.camera_config import CAMERA_PROFILES

from vision import fusion_guidance, line_masks
from vision import virtual_sensors
from vision.green_detection import create_green_mask


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False,
                               allow_nan=False), encoding="utf-8")


def describe(values):
    """Resume apenas medidas válidas; ausência não vira estabilidade perfeita."""
    array = np.asarray([v for v in values if v is not None], dtype=float)
    if not array.size:
        return {"count": 0, "mean": None, "std": None, "p95": None}
    return {"count": int(array.size), "mean": float(array.mean()),
            "std": float(array.std()), "p95": float(np.percentile(array, 95)),
            "min": float(array.min()), "max": float(array.max())}


def load_annotation(path, shape):
    """Usa geometria anotada no RGB e ignora a faixa de incerteza das bordas."""
    annotation = json.loads(path.read_text(encoding="utf-8"))
    if list(shape) != annotation["shape"]:
        raise ValueError("Resolução diferente da anotação.")
    expected = np.zeros(shape, np.uint8)
    region = np.zeros(shape, np.uint8)
    for polygon in annotation["line_polygons"]:
        cv2.fillPoly(expected, [np.asarray(polygon, np.int32)], 255)
    for polygon in annotation["evaluation_polygons"]:
        cv2.fillPoly(region, [np.asarray(polygon, np.int32)], 255)
    margin = int(annotation["boundary_uncertainty_px"])
    kernel = np.ones((2 * margin + 1, 2 * margin + 1), np.uint8)
    positive = (cv2.erode(expected, kernel) > 0) & (region > 0)
    negative = (cv2.dilate(expected, kernel) == 0) & (region > 0)
    if not negative.any():
        raise ValueError("A anotação precisa conter fundo conhecido.")
    return annotation, expected, positive, negative, region > 0


def segment(frame, profile):
    """Reproduz as três etapas originais, com o mesmo recorte do aplicativo."""
    filtered, roi = line_masks.create_filtered_line_mask(frame, profile)
    geometry = line_masks.resolve_vision_geometry(frame.shape[0], profile)
    start = virtual_sensors.resolve_virtual_sensor_geometry(frame.shape)["far"]["left"]["y0"]
    structural = line_masks.create_structural_line_mask(
        filtered, roi, geometry["structural_end_y"], start)
    green = None
    if profile.get("line_exclude_green", False):
        green = create_green_mask(frame, geometry["green_end_y"], green_start_y=start)
    candidate = line_masks.create_line_candidate_mask(structural, profile, green_mask=green)
    if profile.get("calibration_exclude_green", False):
        # Experimento offline: reutiliza o HSV existente sem alterar o preto
        # estrutural utilizado pela associação dos marcadores verdes.
        green = create_green_mask(frame, geometry["green_end_y"], green_start_y=start)
        candidate[:green.shape[0]][green > 0] = 0
    return structural, candidate


def frame_metrics(frame, mask, expected, positive, negative, region, history, contrast_negative=None):
    """Mede erros contra rótulos externos e calcula o alvo do Fusion original."""
    active = mask > 0
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    overlaps = np.bincount(labels[positive], minlength=count)
    overlaps[0] = 0
    # Um gap pode ter mais de um componente verdadeiro. Todos os que tocam
    # fita anotada são preservados na avaliação da continuidade.
    line_ids = np.flatnonzero(overlaps)
    false_ids = [i for i in range(1, count) if overlaps[i] == 0
                 and np.any((labels == i) & negative)]
    line_pixels = np.isin(labels, line_ids) & active
    expected_rows = np.any(positive, axis=1)
    observed_rows = np.any(active & positive, axis=1)
    continuity = float(observed_rows[expected_rows].mean()) if expected_rows.any() else None
    expected_widths = np.count_nonzero((expected > 0) & region, axis=1)
    widths = np.count_nonzero(line_pixels & region, axis=1)
    ys = np.flatnonzero(expected_rows)
    centers = [float(np.flatnonzero(line_pixels[y] & region[y]).mean())
               if np.any(line_pixels[y] & region[y]) else None for y in ys]
    ratios = widths[ys] / np.maximum(1, expected_widths[ys])
    width_error = float(np.mean(np.abs(ratios - 1))) if ys.size else None
    # A confiança do próprio seguidor é registrada separadamente: uma sombra
    # pode receber confiança alta sem ser fita verdadeira.
    fusion = fusion_guidance.extract_fusion_style_line(mask, history)
    point = fusion.get("farPoint")
    target_on_line = None
    if positive.any():
        target_on_line = bool(fusion["valid"] and point and
                              expected[point["y"], point["x"]] > 0)
    false_pixels = int(np.count_nonzero(active & negative))
    recall = float(np.mean(active[positive])) if positive.any() else None
    specificity = float(1 - np.mean(active[negative]))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    contrast = None
    distributions = {}
    floor_region = negative if contrast_negative is None else contrast_negative
    if positive.any() and floor_region.any():
        tape, floor = gray[positive], gray[floor_region]
        tape95, floor05 = float(np.percentile(tape, 95)), float(np.percentile(floor, 5))
        # Margem robusta normalizada pela escala de 8 bits. Negativo significa
        # sobreposição das distribuições, mesmo se a média parecer boa.
        contrast = (floor05 - tape95) / 255.0
        distributions = {"tape_p05_p50_p95": np.percentile(tape, [5, 50, 95]).tolist(),
                         "floor_p05_p50_p95": np.percentile(floor, [5, 50, 95]).tolist()}
    connected_coverage = None
    if positive.any():
        connected_coverage = float(max(overlaps, default=0) / np.count_nonzero(positive))
    row_confidence = virtual_sensors.read_virtual_line_sensors(mask)
    return {"component_count": count - 1, "false_component_count": len(false_ids),
            "false_component_total_area_px": int(sum(stats[i, cv2.CC_STAT_AREA] for i in false_ids)),
            "false_positive_area_px": false_pixels,
            "false_positive_fraction": 1 - specificity,
            "line_recall": recall, "background_specificity": specificity,
            "line_continuity": continuity, "largest_component_line_recall": connected_coverage,
            "width_relative_error": width_error, "widths_px": widths[ys].tolist(),
            "row_centers_px": centers, "row_ys": ys.tolist(),
            "area_px": int(np.count_nonzero(active)),
            "contrast_margin_8bit_normalized": contrast, "intensity_distributions": distributions,
            "trajectory_valid": bool(fusion["valid"]), "trajectory_target_on_line": target_on_line,
            "trajectory": fusion,
            "far_line_confidence": row_confidence.get("farLineConfidence"),
            "medium_line_confidence": row_confidence.get("mediumLineConfidence")}, fusion


def analyze(dataset, annotations, output, overrides):
    """Avalia uma configuração sem sobrescrever RGB, máscaras ou resultados anteriores."""
    config = json.loads((dataset / "config.json").read_text(encoding="utf-8"))
    camera_profile = config.get("requested_camera_profile", config.get("camera_profile"))
    profile = copy.deepcopy(camera_profile["vision"])
    profile.update(overrides)
    if "line_threshold" in overrides:
        profile.pop("line_background_kernel_size", None)
    output.mkdir(parents=True, exist_ok=False)
    (output / "mask").mkdir()
    files = sorted((dataset / "raw").glob("*.png"))
    if not files:
        raise ValueError("Dataset sem RGB PNG.")
    first = cv2.imread(str(files[0]))
    annotation, expected, positive, negative, region = load_annotation(annotations, first.shape[:2])
    # Verde é fundo negativo para a linha preta, mas não representa piso branco
    # na distribuição de contraste. A exclusão não altera os falsos positivos.
    contrast_negative = negative.copy()
    non_floor = np.zeros(first.shape[:2], np.uint8)
    for polygon in annotation.get("non_floor_polygons", []):
        cv2.fillPoly(non_floor, [np.asarray(polygon, np.int32)], 255)
    contrast_negative[non_floor > 0] = False
    source_hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                     for p in Path(__file__).parent.joinpath("vision").glob("*.py")}
    write_json(output / "config.json", {"dataset": str(dataset.resolve()),
               "annotation": annotation, "annotation_sha256": hashlib.sha256(annotations.read_bytes()).hexdigest(),
               "segmentation": profile, "overrides": overrides,
               "opencv": cv2.__version__, "source_sha256": source_hashes,
               "trajectory_history": "replay; begins empty; no motor/state logic executed"})
    records, history, previous_mask, previous = [], None, None, None
    for path in files:
        metadata = json.loads((dataset / "metadata" / (path.stem + ".json")).read_text(encoding="utf-8"))
        expected_hash = metadata.get("raw_sha256")
        if not expected_hash or hashlib.sha256(path.read_bytes()).hexdigest() != expected_hash:
            raise ValueError(f"RGB ausente do manifesto ou modificado: {path}.")
        frame = cv2.imread(str(path))
        structural, mask = segment(frame, profile)
        metrics, fusion = frame_metrics(frame, mask, expected, positive, negative, region, history, contrast_negative)
        history = fusion_guidance.update_fusion_style_history(history, fusion)
        metrics["sample_id"] = path.stem
        metrics["timestamp"] = metadata.get("captured_at_unix_seconds", metadata.get("timestamp_unix_seconds"))
        metrics["camera_metadata"] = metadata.get("camera", {}).get("metadata", metadata.get("camera_metadata", {}))
        if previous_mask is not None:
            union = ((mask > 0) | (previous_mask > 0)) & region
            difference = (mask != previous_mask) & region
            metrics["temporal_iou"] = (float(np.count_nonzero((mask > 0) & (previous_mask > 0) & region)
                                             / np.count_nonzero(union)) if union.any() else None)
            metrics["changed_pixels"] = int(np.count_nonzero(difference))
            metrics["area_delta_px"] = abs(metrics["area_px"] - previous["area_px"])
            metrics["component_count_delta"] = abs(metrics["component_count"] - previous["component_count"])
            shifts = [abs(a - b) for a, b in zip(metrics["row_centers_px"], previous["row_centers_px"])
                      if a is not None and b is not None]
            metrics["center_jitter_px"] = float(np.mean(shifts)) if shifts else None
            metrics["width_jitter_px"] = float(np.mean(np.abs(np.array(metrics["widths_px"]) - previous["widths_px"]))) if metrics["widths_px"] else None
            first_point, last_point = fusion.get("farPoint"), previous["trajectory"].get("farPoint")
            metrics["trajectory_jitter_px"] = float(np.hypot(first_point["x"] - last_point["x"], first_point["y"] - last_point["y"])) if first_point and last_point else None
        if not overrides:
            saved = cv2.imread(str(dataset / "mask" / path.name), cv2.IMREAD_GRAYSCALE)
            if saved is None or saved.shape != mask.shape:
                raise ValueError(f"Máscara original ausente ou incompatível: {path.name}.")
            metrics["saved_mask_difference_px"] = int(np.count_nonzero(mask != saved))
            if metrics["saved_mask_difference_px"]:
                raise ValueError("O replay não reproduz a máscara original; confira a versão do pipeline antes de comparar.")
        if not cv2.imwrite(str(output / "mask" / path.name), mask):
            raise OSError("Falha ao gravar máscara.")
        records.append(metrics)
        previous_mask, previous = mask, metrics
        if len(records) == 1:
            diagnostic = frame.copy()
            diagnostic[(structural > 0) & (mask == 0)] = (255, 0, 255)
            diagnostic[(mask > 0) & negative] = (0, 0, 255)
            cv2.polylines(diagnostic, [np.asarray(p, np.int32) for p in annotation["line_polygons"]], True, (0, 255, 255), 1)
            fusion_guidance.draw_fusion_style_line_overlay(diagnostic, fusion)
            cv2.imwrite(str(output / "diagnostic.png"), diagnostic)
    numeric_keys = [k for k, v in records[-1].items() if isinstance(v, (int, float, bool)) or v is None]
    summary = {k: describe([r.get(k) for r in records]) for k in numeric_keys}
    mean = lambda key: summary.get(key, {}).get("mean")
    components = {"line_recall": mean("line_recall"), "background_specificity": mean("background_specificity"),
                  "line_continuity": mean("line_continuity"), "temporal_iou": mean("temporal_iou"),
                  "trajectory_target_on_line": mean("trajectory_target_on_line")}
    if not positive.any():
        # Piso vazio não exige trajetória. Uma máscara vazia só é correta nesse
        # cenário explicitamente anotado como negativo.
        components = {"background_specificity": mean("background_specificity"),
                      "trajectory_absence": 1 - mean("trajectory_valid")}
    valid = [v for v in components.values() if v is not None]
    score = 100 * min(valid) if valid and len(valid) == len(components) else None
    result = {"frames": len(records), "summary": summary, "score": score,
              "score_components": components,
              "score_rule": "100 * minimum of available required quality indicators; no fitted weights",
              "score_scope": "single annotated pose; exploratory, not run validation",
              "contrast_and_width": "reported separately; absolute acceptance limits require track geometry and multiple scenarios",
              "temporal_scope": "adjacent saved samples, not necessarily adjacent sensor frames"}
    write_json(output / "frames.json", records)
    write_json(output / "metrics.json", result)
    print(json.dumps({"output": str(output), "frames": len(records), "score": score,
                      **{key: mean(key) for key in ["false_component_count", "false_positive_area_px", "line_recall", "line_continuity", "center_jitter_px"]}}))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--annotations", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--threshold", type=int, choices=range(256), metavar="0..255")
    parser.add_argument("--ratio", type=int, choices=range(1, 101), metavar="1..100")
    parser.add_argument("--open-kernel", type=int)
    parser.add_argument("--close-kernel", type=int)
    parser.add_argument("--background-kernel", type=int)
    parser.add_argument("--min-threshold", type=int, choices=range(191), metavar="0..190")
    parser.add_argument("--exclude-green", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument(
        "--illumination-correction",
        action=argparse.BooleanOptionalAction,
        default=None,
    )
    args = parser.parse_args()
    overrides = {key: value for key, value in [("line_threshold", args.threshold),
                 ("line_max_background_ratio_percent", args.ratio),
                 ("open_kernel_size", args.open_kernel), ("close_kernel_size", args.close_kernel),
                 ("line_background_kernel_size", args.background_kernel),
                 ("line_min_threshold", args.min_threshold)] if value is not None}
    if args.exclude_green is not None:
        overrides["line_exclude_green"] = args.exclude_green
    if args.illumination_correction is not None:
        overrides["line_illumination_correction_enabled"] = (
            args.illumination_correction
        )
        if args.illumination_correction:
            current_profile = CAMERA_PROFILES["down"]["vision"]
            for key in (
                "line_illumination_reference_path",
                "line_illumination_reference_sha256",
                "line_illumination_target_percentile",
                "line_illumination_max_gain",
                "line_illumination_useful_start_ratio",
                "line_illumination_overlay_gain_threshold",
                "line_illumination_overlay_margin_px",
            ):
                overrides[key] = current_profile[key]
    if args.threshold is not None and args.ratio is not None:
        parser.error("Teste threshold global ou relativo separadamente.")
    if any(value <= 0 for value in (args.open_kernel, args.close_kernel, args.background_kernel) if value is not None):
        parser.error("Kernels devem ser positivos.")
    analyze(args.dataset, args.annotations, args.output, overrides)


if __name__ == "__main__":
    main()
