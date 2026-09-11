"""Controla a mesma captura YOLO exposta pela dashboard do robô."""

import argparse
import json
import os
from pathlib import Path
import re
import time


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DATASET_ROOT = PROJECT_ROOT / "yolo_ball" / "datasets" / "images" / "raw"
CONTROL_PATH = Path("/dev/shm/obr_silver_dataset_capture.json")
TEMP_CONTROL_PATH = Path("/dev/shm/obr_silver_dataset_capture.tmp.json")
SAFE_NAME_PATTERN = re.compile(r"^[A-Za-z0-9_-]+$")


def write_capture_command(active, camera, session, label, fps):
    """Publica um comando validado sem abrir novamente a câmera do robô."""

    if camera not in ("forward", "down"):
        raise ValueError("camera deve ser 'forward' ou 'down'.")
    if label not in ("black", "silver", "other"):
        raise ValueError("label deve ser 'black', 'silver' ou 'other'.")
    if not SAFE_NAME_PATTERN.fullmatch(session):
        raise ValueError("session aceita somente letras, números, '_' e '-'.")
    if not 0.5 <= float(fps) <= 10.0:
        raise ValueError("fps deve ficar entre 0,5 e 10.")
    payload = {"active": bool(active), "dataset": "yolo_ball", "camera": camera,
               "session": session, "label": label, "fps": float(fps)}
    TEMP_CONTROL_PATH.write_text(json.dumps(payload), encoding="utf-8")
    os.replace(TEMP_CONTROL_PATH, CONTROL_PATH)


def main(arguments=None):
    parser = argparse.ArgumentParser(description="Captura frames crus para o dataset YOLO.")
    parser.add_argument("--camera", choices=("forward", "down"), default="forward")
    parser.add_argument("--session", required=True)
    parser.add_argument("--label", choices=("black", "silver", "other"), required=True)
    parser.add_argument("--fps", type=float, default=4.0)
    parser.add_argument("--seconds", type=float, default=1.0)
    args = parser.parse_args(arguments)
    if args.seconds <= 0.0:
        raise ValueError("seconds deve ser positivo.")
    write_capture_command(True, args.camera, args.session, args.label, args.fps)
    try:
        time.sleep(args.seconds)
    finally:
        write_capture_command(False, args.camera, args.session, args.label, args.fps)
    output = DATASET_ROOT / args.camera / args.session / args.label
    print(f"Captura encerrada. Imagens: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
