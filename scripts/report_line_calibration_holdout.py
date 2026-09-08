"""Compara somente capturas posteriores ao congelamento da candidata visual."""

import argparse
import json
from pathlib import Path

from compare_line_calibration import comparison_row
from line_calibration import write_json


def normalized_profile(profile):
    result = dict(profile)
    result.setdefault("line_min_threshold", 0)
    result.setdefault("line_exclude_green", False)
    result.setdefault("calibration_exclude_green", False)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path, nargs="?", default=Path("calibration"))
    args = parser.parse_args()
    frozen = json.loads((args.root / "candidate_v1.json").read_text(encoding="utf-8"))
    candidate = normalized_profile(frozen["camera_profile"]["vision"])
    original = {**candidate, "line_max_background_ratio_percent": 70,
                "line_min_threshold": 0, "line_exclude_green": False}
    # Esta captura precedeu o congelamento e contém os fontes efetivamente
    # instalados no Raspberry, sem confundir diferenças CRLF/LF do computador.
    reference = json.loads((args.root / "captures/live_border_minimum_40_01/config.json").read_text(encoding="utf-8"))
    relevant_sources = ("application.py", "camera.py", "camera_config.py", "line_masks.py",
                        "green_detection.py", "fusion_guidance.py", "virtual_sensors.py")
    rows = []
    for dataset in sorted((args.root / "captures").glob("holdout_*")):
        if not dataset.is_dir():
            continue
        config = json.loads((dataset / "config.json").read_text(encoding="utf-8"))
        if config["request"]["requested_at_unix_seconds"] <= frozen["frozen_at_unix_seconds"]:
            raise ValueError(f"Captura anterior ao congelamento: {dataset.name}")
        if config["requested_camera_controls"] != frozen["requested_camera_controls"]:
            raise ValueError(f"Controles de câmera diferentes: {dataset.name}")
        for name in relevant_sources:
            if config["source_sha256"].get(name) != reference["source_sha256"].get(name):
                raise ValueError(f"Fonte alterado durante validação: {dataset.name}/{name}")
        for key in ("main_size", "sensor_size", "rotation_degrees", "target_fps", "sensor_bit_depth"):
            if config["camera_profile"][key] != frozen["camera_profile"][key]:
                raise ValueError(f"Geometria da câmera diferente: {dataset.name}/{key}")
        actual = normalized_profile(config["camera_profile"]["vision"])
        if actual not in (candidate, original):
            raise ValueError(f"Configuração não prevista: {dataset.name}")
        metadata = [json.loads(p.read_text(encoding="utf-8"))
                    for p in sorted((dataset / "metadata").glob("*.json"))]
        if not 30 <= len(metadata) <= 100:
            raise ValueError(f"Quantidade de frames inválida: {dataset.name}")
        for record in metadata:
            telemetry = record["robot_telemetry"]
            if (telemetry.get("emergency") is not True or telemetry.get("esp32EmergencyStop") is not True
                    or telemetry.get("esp32SensorFresh") is not True
                    or any(telemetry.get(k) != 0 for k in ("appliedLeft", "appliedRight", "leftEncoderRate", "rightEncoderRate"))):
                raise ValueError(f"Segurança não confirmada em todos os frames: {dataset.name}")
        variants = {}
        captured_mask_checked = False
        for path in sorted((args.root / "experiments" / dataset.name).glob("*/metrics.json")):
            experiment_config = json.loads((path.parent / "config.json").read_text(encoding="utf-8"))
            profile = normalized_profile(experiment_config["segmentation"])
            label = "candidate" if profile == candidate else "original" if profile == original else None
            if label is None:
                raise ValueError(f"Ajuste novo misturado à validação: {path.parent}")
            row = comparison_row(path.parent, {})
            if row["frames"] != len(metadata):
                raise ValueError(f"Análise incompleta: {path.parent}")
            difference = row["summary"].get("saved_mask_difference_px", {})
            captured_mask_checked |= (difference.get("count") == len(metadata) and difference.get("max") == 0)
            variants[label] = row
        if set(variants) != {"original", "candidate"} or not captured_mask_checked:
            raise ValueError(f"Falta comparação ou verificação da máscara real: {dataset.name}")
        old, new = variants["original"], variants["candidate"]
        # Os mesmos RGB são usados em cada par; não misturamos médias de
        # exposições ou momentos diferentes para atribuir a melhora à máscara.
        regressions = []
        for key in ("line_recall", "line_continuity"):
            old_min = old["summary"].get(key, {}).get("min")
            new_min = new["summary"].get(key, {}).get("min")
            if old_min is not None and new_min is not None and new_min < old_min:
                regressions.append(key)
        rows.append({"dataset": dataset.name, "scenario": new["scenario"], "frames": len(metadata),
                     "source_and_camera_settings_verified": True,
                     "safety_verified_all_frames": True, "captured_masks_reproduced": True,
                     "preservation_regressions": regressions, "original": old, "candidate": new})
    result = {"candidate_id": frozen["candidate_id"], "score_version": 3,
              "scope": "New static captures after parameter freeze. Each original/candidate pair uses identical RGB.",
              "frames": sum(r["frames"] for r in rows), "scenarios": sorted({r["scenario"] for r in rows}),
              "rows": rows, "preservation_regression_detected": any(r["preservation_regressions"] for r in rows)}
    write_json(args.root / "report/holdout.json", result)
    lines = ["# Validação da candidata congelada", "",
             "Cada comparação usa os mesmos RGB. Todos os frames são posteriores ao congelamento.", "",
             "| Captura | Frames | FP original → candidata, px/frame | Fita original → candidata | Jitter centro original → candidata, px | Score v3 original → candidata |",
             "|---|---:|---:|---:|---:|---:|"]
    for row in rows:
        def pair(key):
            values = [row[label]["summary"].get(key, {}).get("mean") for label in ("original", "candidate")]
            return " → ".join("n/d" if value is None else f"{value:.4f}" for value in values)
        lines.append(f"| {row['dataset']} | {row['frames']} | {pair('false_positive_area_px')} | {pair('line_recall')} | {pair('center_jitter_px')} | {row['original']['score']:.2f} → {row['candidate']['score']:.2f} |")
    lines.extend(["", "Os indicadores completos, mínimos por frame e configurações estão em `holdout.json`.",
                  "Isso não é teste com motores em movimento nem comprovação de uma run completa."])
    (args.root / "report/holdout.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({"frames": result["frames"], "scenarios": result["scenarios"],
                      "preservation_regression_detected": result["preservation_regression_detected"]}))


if __name__ == "__main__":
    main()
