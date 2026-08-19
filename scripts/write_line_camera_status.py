"""Publica o estado do serviço da câmera inferior sem reusar telemetria antiga."""

import argparse
import json
import os
import time


STATUS_PATH = "/tmp/obr_camera_status.json"
TEMP_STATUS_PATH = "/tmp/obr_camera_status.tmp.json"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("state", choices=("INICIANDO", "PARADA", "FALHA"))
    parser.add_argument("--error", default="")
    parser.add_argument("--enabled", choices=("0", "1"), default="0")
    arguments = parser.parse_args()
    status = {
        "active": False,
        "enabled": arguments.enabled == "1",
        "fps": 0.0,
        "state": arguments.state,
        "cameraRole": "down",
        "width": 480,
        "height": 360,
        "targetCameraFps": 30,
        "error": arguments.error,
        "timestamp": time.time(),
    }
    with open(TEMP_STATUS_PATH, "w", encoding="utf-8") as status_file:
        json.dump(status, status_file, allow_nan=False)
    os.replace(TEMP_STATUS_PATH, STATUS_PATH)


if __name__ == "__main__":
    main()
