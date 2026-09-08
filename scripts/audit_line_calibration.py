"""Verifica integridade e reprodução das capturas reais sem acessar o robô."""

import argparse
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np

from line_calibration import segment, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path, nargs="?", default=Path("calibration"))
    args = parser.parse_args()
    datasets = [args.root / "baseline/current_pose_01"]
    datasets.extend(sorted((args.root / "captures").iterdir()))
    reports = []
    for dataset in datasets:
        if not (dataset / "raw").is_dir():
            continue
        config = json.loads((dataset / "config.json").read_text(encoding="utf-8"))
        profile = config.get("requested_camera_profile", config.get("camera_profile"))["vision"]
        files = sorted((dataset / "raw").glob("*.png"))
        if not files:
            raise ValueError(f"Captura sem frames: {dataset}")
        for path in files:
            metadata = json.loads((dataset / "metadata" / (path.stem + ".json")).read_text(encoding="utf-8"))
            if metadata.get("raw_sha256") != hashlib.sha256(path.read_bytes()).hexdigest():
                raise ValueError(f"RGB modificado: {path}")
            frame = cv2.imread(str(path))
            mask = cv2.imread(str(dataset / "mask" / path.name), cv2.IMREAD_GRAYSCALE)
            if frame is None or mask is None or frame.shape[:2] != mask.shape:
                raise ValueError(f"Par incompleto ou dimensões incompatíveis: {path}")
            if not np.all((mask == 0) | (mask == 255)):
                raise ValueError(f"Máscara não binária: {path}")
            _structural, replay = segment(frame, profile)
            if not np.array_equal(mask, replay):
                raise ValueError(f"O código atual não reproduz a máscara salva: {path}")
        reports.append({"dataset": str(dataset), "frames": len(files),
                        "raw_hashes_verified": True,
                        "paired_shapes_and_binary_masks_verified": True,
                        "current_pipeline_reproduces_saved_masks": True})
    result = {"checked_at_unix_seconds": time.time(), "datasets": reports,
              "total_real_frames": sum(r["frames"] for r in reports)}
    (args.root / "report").mkdir(exist_ok=True)
    write_json(args.root / "report/integrity.json", result)
    print(json.dumps({"verified_datasets": len(reports), "verified_frames": result["total_real_frames"]}))


if __name__ == "__main__":
    main()
