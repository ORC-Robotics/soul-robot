"""Solicita uma sequência diagnóstica no Raspberry com E-Stop já confirmado."""

import argparse
import json
import os
from pathlib import Path
import tempfile
import time

from vision.calibration_capture import REQUEST_PATH, RESULT_PATH, StationaryGuard


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("experiment")
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--frames", type=int, default=60, choices=range(30, 101), metavar="30..100")
    parser.add_argument("--controls-json", type=Path)
    parser.add_argument("--line-ratio", type=int, choices=range(1, 101), metavar="1..100")
    parser.add_argument("--exclude-green", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument(
        "--illumination-correction",
        action=argparse.BooleanOptionalAction,
        default=None,
    )
    parser.add_argument("--min-threshold", type=int, choices=range(191), metavar="0..190")
    args = parser.parse_args()
    controls = json.loads(args.controls_json.read_text(encoding="utf-8")) if args.controls_json else {}
    if controls and (
        args.line_ratio is not None
        or args.exclude_green is not None
        or args.min_threshold is not None
        or args.illumination_correction is not None
    ):
        parser.error("Altere câmera ou segmentação, uma família por captura.")
    guard = StationaryGuard()
    try:
        segmentation = {}
        if args.line_ratio is not None:
            segmentation["line_max_background_ratio_percent"] = args.line_ratio
        if args.exclude_green is not None:
            segmentation["line_exclude_green"] = args.exclude_green
        if args.min_threshold is not None:
            segmentation["line_min_threshold"] = args.min_threshold
        if args.illumination_correction is not None:
            segmentation["line_illumination_correction_enabled"] = (
                args.illumination_correction
            )
        request = {"experiment": args.experiment, "scenario": args.scenario,
                   "frames": args.frames, "controls": controls,
                   "segmentation": segmentation,
                   "requested_at_unix_seconds": time.time()}
        # O link publica o JSON completo e falha se outro pedido estiver pendente.
        descriptor, name = tempfile.mkstemp(prefix=".obr_calibration_", dir=str(REQUEST_PATH.parent))
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                json.dump(request, stream, allow_nan=False)
            os.link(name, REQUEST_PATH)
        finally:
            Path(name).unlink()
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            guard.require_stopped()
            if RESULT_PATH.exists():
                result = json.loads(RESULT_PATH.read_text(encoding="utf-8"))
                fresh = result.get("timestamp_unix_seconds", 0) >= request["requested_at_unix_seconds"]
                if fresh and result.get("state") == "error" and not REQUEST_PATH.exists():
                    raise RuntimeError(result.get("error"))
                if fresh and result.get("experiment") == args.experiment and result.get("state") == "complete":
                    print(json.dumps(result))
                    return
            time.sleep(0.1)
        raise TimeoutError("Captura não confirmou conclusão em 30 segundos; consulte o resultado e o serviço.")
    finally:
        guard.close()


if __name__ == "__main__":
    main()
