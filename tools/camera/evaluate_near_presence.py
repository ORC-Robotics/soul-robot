"""Resume presença local em máscaras já salvas, sem câmera ou motores."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from vision.camera_config import NEAR_VIRTUAL_SENSOR_CONFIG
from vision.gap_validation import GapValidator


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("datasets", nargs="+", type=Path, help="Pastas contendo mask/*.png.")
    parser.add_argument("--output", type=Path, default=Path("build/gap-presence-replay.json"))
    args = parser.parse_args()
    report = {"origin": "offline saved masks; no new camera capture", "config": NEAR_VIRTUAL_SENSOR_CONFIG,
              "datasets": []}
    for dataset in args.datasets:
        counts, digest, details = Counter(), hashlib.sha256(), []
        files = sorted((dataset / "mask").glob("*.png"))
        if not files:
            parser.error(f"Nenhuma máscara PNG em {dataset}.")
        for path in files:
            digest.update(path.name.encode())
            digest.update(path.read_bytes())
            mask = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if mask is None:
                parser.error(f"Máscara inválida: {path}.")
            result = GapValidator().observe_near(mask, 100.0, 1, 100.0)
            counts[result["state"]] += 1
            details.append({"frame": path.name, "state": result["state"], "score": result["score"]})
        report["datasets"].append({"dataset": dataset.as_posix(), "frames": len(files),
                                   "counts": dict(counts), "input_sha256": digest.hexdigest(), "per_frame": details})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps([{k: v for k, v in d.items() if k != "per_frame"} for d in report["datasets"]]))


if __name__ == "__main__":
    main()
