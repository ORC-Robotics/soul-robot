"""Mede visão frontal sintética localmente, sem abrir câmeras ou comandar o robô."""

import argparse
import json
import platform
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from forward_camera_stream import (calculate_forward_line_assist, draw_forward_assist_overlay,
                                   process_forward_frame, resolve_forward_assist_roi)
from vision.camera_config import FORWARD_PATH_CONFIG, FORWARD_PRESENCE_CONFIG, NEAR_LINE_PRESENCE_CONFIG
from vision.gap_validation import GapValidator
from vision.forward_path import ForwardPathTracker, bottom_reference, predicted_path
from vision.fusion_guidance import extract_fusion_style_line


def paint_path(mask, curve, roi, thickness=18):
    """Desenha fita sintética com coordenadas normalizadas no frame inteiro."""
    _, y0, _, y1 = roi
    points = [(int(round((curve(d) + 1) * (mask.shape[1] - 1) / 2)),
               int(round(y1 - 1 - d * (y1 - y0 - 1)))) for d in np.linspace(0, 1, 250)]
    cv2.polylines(mask, [np.asarray(points, np.int32)], False, 255, thickness)


def timed(operation, frames):
    """Exclui aquecimento e informa distribuição, sem prometer FPS no Raspberry."""
    for i in range(10):
        operation(i)
    durations = []
    for i in range(frames):
        started = time.perf_counter()
        operation(i + 10)
        durations.append((time.perf_counter() - started) * 1000)
    return {"median_ms": float(np.median(durations)), "p95_ms": float(np.percentile(durations, 95)),
            "max_ms": max(durations)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=100)
    parser.add_argument("--output", type=Path, default=Path("build/forward-gap-validation"))
    args = parser.parse_args()
    if not 10 <= args.frames <= 1000:
        parser.error("Use entre 10 e 1000 frames por cenário.")
    args.output.mkdir(parents=True, exist_ok=True)
    # Uma thread evita comparar agendadores diferentes entre máquinas.
    cv2.setNumThreads(1)
    report = {"synthetic": True, "platform": platform.platform(), "processor": platform.processor(),
              "python": platform.python_version(), "opencv": cv2.__version__,
              "opencv_threads": cv2.getNumThreads(), "resolution": [960, 540],
              "frames_per_scenario": args.frames, "forward_config": dict(FORWARD_PRESENCE_CONFIG),
              "near_config": dict(NEAR_LINE_PRESENCE_CONFIG), "scenarios": {}}
    for name in ("center", "strong_curve", "multiple", "crossing", "serrated", "overload", "side", "tilted", "particles"):
        mask = np.zeros((540, 960), np.uint8)
        roi = resolve_forward_assist_roi(mask.shape)
        ref = {"valid": True, "position": 0.0, "slope": 0.0, "curvature": 0.0,
               "timestamp": 100.0, "sequence": 1, "side": "CENTER"}
        if name == "strong_curve":
            ref.update(position=-0.3, slope=0.15, curvature=5.0)
        if name == "overload":
            for x in np.linspace(-0.85, 0.85, 10):
                paint_path(mask, lambda d: x, roi)
        elif name == "side":
            paint_path(mask, lambda d: 0.8, roi, 35)
        elif name == "tilted":
            paint_path(mask, lambda d: -0.7 + 1.2 * d, roi, 30)
        elif name == "particles":
            for x in range(100, 900, 80):
                cv2.circle(mask, (x, 480), 7, 255, -1)
        elif name == "serrated":
            paint_path(mask, lambda d: 0.18 * np.sin(7 * np.pi * d), roi)
        else:
            paint_path(mask, lambda d: predicted_path(ref, d, FORWARD_PATH_CONFIG), roi)
            if name == "multiple":
                paint_path(mask, lambda d: 0.7, roi, 28)
            elif name == "crossing":
                cv2.line(mask, (roi[0], 418), (roi[2] - 1, 418), 255, 9)
        frame = cv2.cvtColor(255 - mask, cv2.COLOR_GRAY2BGR)
        tracker = ForwardPathTracker()

        def measure_frame(i, geometry_only=False):
            timestamp = 100.0 + i / 30.0
            ref["timestamp"] = timestamp - 0.02
            if geometry_only:
                return calculate_forward_line_assist(mask, ref, tracker, timestamp)
            return process_forward_frame(frame, "RGB888", ref, tracker, timestamp)

        geometry = timed(lambda i: measure_frame(i, True), args.frames)
        vision = timed(measure_frame, args.frames)
        result = measure_frame(args.frames + 11)
        result["decision"] = "GAP" if result["forwardPathState"] == "PRESENT" else "CHECKING"
        result.update(nearState="LOST", source="gap-forward")
        display = frame.copy()
        draw_forward_assist_overlay(display, result)
        cv2.imwrite(str(args.output / (name + ".png")), display)
        report["scenarios"][name] = {"geometry": geometry, "segmentation_and_geometry": vision,
                                     "state": result["forwardPathState"],
                                     "confidence": result["forwardPathConfidence"],
                                     "components": result["forwardPathComponents"]}
    bottom = np.zeros((360, 480), np.uint8)
    paint_path(bottom, lambda d: 0.20 * d ** 2, (0, 0, 480, 360), 40)
    validator = GapValidator()
    report["near_presence"] = timed(lambda i: validator.observe_near(bottom, 100 + i / 30, i + 1, 100 + i / 30), args.frames)
    fusion = extract_fusion_style_line(bottom)
    report["bottom_reference"] = timed(lambda i: bottom_reference(bottom, fusion, 100 + i / 30, i + 1), args.frames)
    (args.output / "benchmark.json").write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
