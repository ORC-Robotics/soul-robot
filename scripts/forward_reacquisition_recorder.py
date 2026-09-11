"""Grava sessões diagnósticas da CAM1 sem participar das decisões do robô."""

from datetime import datetime, timezone
from pathlib import Path
from queue import Full, Queue
import argparse
import json
import math
import os
import re
import subprocess
import threading
import time

import cv2  # type: ignore
import numpy as np

from vision.camera_config import (
    CAMERA_PROFILES,
    FORWARD_PATH_CONFIG,
    FORWARD_PRESENCE_CONFIG,
    GAP_VALIDATION_CONFIG,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SESSION_ROOT = PROJECT_ROOT / "logs" / "forward_reacquisition"
CONTROL_PATH = Path("/dev/shm/obr_forward_reacquisition_capture.json")
CONTROL_TEMP_PATH = Path("/dev/shm/obr_forward_reacquisition_capture.tmp.json")
DEFAULT_IMAGE_FPS = 5.0
MIN_IMAGE_FPS = 0.5
MAX_IMAGE_FPS = 10.0
PNG_COMPRESSION = 3
SAFE_NAME_PATTERN = re.compile(r"^[A-Za-z0-9_-]+$")
MAX_REJECTED_CANDIDATES_IN_OVERLAY = 5


def _finite_json(value):
    """Substitui números não finitos antes de gravar JSON estrito."""

    if isinstance(value, np.ndarray):
        return _finite_json(value.tolist())
    if isinstance(value, np.generic):
        return _finite_json(value.item())
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): _finite_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite_json(item) for item in value]
    return value


def _write_json_atomic(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(_finite_json(data), ensure_ascii=False, allow_nan=False, indent=2),
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _read_control(path=CONTROL_PATH):
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, ValueError, TypeError, json.JSONDecodeError):
        return None
    if not isinstance(data, dict) or data.get("active") is not True:
        return None
    session = data.get("session")
    label = data.get("label")
    try:
        image_fps = float(data.get("imageFps", DEFAULT_IMAGE_FPS))
    except (TypeError, ValueError):
        return None
    if (
        not isinstance(session, str)
        or not SAFE_NAME_PATTERN.fullmatch(session)
        or not isinstance(label, str)
        or not SAFE_NAME_PATTERN.fullmatch(label)
        or not MIN_IMAGE_FPS <= image_fps <= MAX_IMAGE_FPS
    ):
        return None
    return {
        "active": True,
        "session": session,
        "label": label,
        "imageFps": image_fps,
        "startedAt": data.get("startedAt"),
    }


def _rejection_label(candidate, frame_width, roi):
    """Resume apenas para o overlay quais gates do detector não foram atendidos."""

    if candidate.get("state") == "PRESENT":
        return None
    reason = str(candidate.get("reason", "REJECTED"))
    if reason != "INSUFFICIENT_SUPPORT":
        return reason

    config = FORWARD_PRESENCE_CONFIG
    bands = candidate.get("bands", [])
    failures = []
    if len(bands) < config["min_bands"]:
        failures.append(f"BANDS {len(bands)}/{config['min_bands']}")

    extent = candidate.get("extent")
    minimum_extent = config["min_extent"] * frame_width
    if isinstance(extent, (int, float)) and extent < minimum_extent:
        failures.append(f"EXT {extent:.0f}/{minimum_extent:.0f}")

    elongation = candidate.get("elongation")
    required_elongation = (
        config["clipped_elongation"]
        if candidate.get("clipped") is True
        else config["min_elongation"]
    )
    if isinstance(elongation, (int, float)) and elongation < required_elongation:
        failures.append(f"ELONG {elongation:.1f}/{required_elongation:.1f}")

    if isinstance(roi, (list, tuple)) and len(roi) == 4:
        near_limit = roi[3] - config["near_fraction"] * (roi[3] - roi[1])
        near_bands = sum(
            isinstance(band, dict) and band.get("y", -1) >= near_limit
            for band in bands
        )
        if near_bands < config["near_bands"]:
            failures.append(f"NEAR {near_bands}/{config['near_bands']}")

    score = candidate.get("score")
    if not failures and isinstance(score, (int, float)) and score < 1.0:
        failures.append(f"SCORE {score:.2f}/1.00")
    return " ".join(failures) if failures else reason


def _near_band_count(candidate, roi):
    """Conta as bandas próximas usando a mesma fronteira do detector atual."""

    near_bands = candidate.get("nearBands")
    if isinstance(near_bands, (int, float)):
        return int(near_bands)
    near_limit = (
        roi[3]
        - FORWARD_PRESENCE_CONFIG["near_fraction"] * (roi[3] - roi[1])
    )
    return sum(
        isinstance(band, dict) and band.get("y", -1) >= near_limit
        for band in candidate.get("bands", [])
    )


def _metric_text(value, digits=1):
    """Formata métricas ausentes sem criar valores que o detector não calculou."""

    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        return "--"
    return f"{float(value):.{digits}f}"


def _draw_mask_thumbnail(frame, mask):
    """Mostra a máscara exata entregue ao tracker, sem alterar seus pixels."""

    if not isinstance(mask, np.ndarray) or mask.ndim != 2 or mask.size == 0:
        return 24
    maximum_width = min(260, max(80, frame.shape[1] // 3))
    maximum_height = min(150, max(50, frame.shape[0] // 3))
    scale = min(maximum_width / mask.shape[1], maximum_height / mask.shape[0])
    thumbnail_width = max(1, int(round(mask.shape[1] * scale)))
    thumbnail_height = max(1, int(round(mask.shape[0] * scale)))
    thumbnail = cv2.resize(
        mask, (thumbnail_width, thumbnail_height), interpolation=cv2.INTER_NEAREST
    )
    if thumbnail.dtype != np.uint8:
        thumbnail = np.where(thumbnail > 0, 255, 0).astype(np.uint8)
    thumbnail_bgr = cv2.cvtColor(thumbnail, cv2.COLOR_GRAY2BGR)
    x = max(0, frame.shape[1] - thumbnail_width - 8)
    y = 22
    frame[y:y + thumbnail_height, x:x + thumbnail_width] = thumbnail_bgr
    cv2.rectangle(
        frame, (x, y), (x + thumbnail_width - 1, y + thumbnail_height - 1),
        (255, 255, 255), 1,
    )
    cv2.putText(
        frame, "MASK EFETIVA -> TRACKER", (x, max(14, y - 5)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 255, 255), 1, cv2.LINE_AA,
    )
    return y + thumbnail_height


def draw_forward_path_diagnostic_overlay(frame, reading, roi=None, mask=None):
    """Expõe máscara e geometria no stream sem participar da decisão visual."""

    if not isinstance(roi, (list, tuple)) or len(roi) != 4:
        roi = (0, 0, frame.shape[1], frame.shape[0])
    candidates = [
        (index, candidate)
        for index, candidate in enumerate(reading.get("candidates", []), start=1)
        if isinstance(candidate, dict)
    ]
    selected_bands = reading.get("selectedBands", [])

    def priority(item):
        _, candidate = item
        bands = candidate.get("bands", [])
        selected = bool(selected_bands) and bands == selected_bands
        state_priority = {"PRESENT": 2, "UNCERTAIN": 1}.get(
            candidate.get("state"), 0
        )
        score = candidate.get("score", 0.0)
        score = float(score) if isinstance(score, (int, float)) else 0.0
        return selected, state_priority, bool(bands), score

    relevant = sorted(candidates, key=priority, reverse=True)[
        :MAX_REJECTED_CANDIDATES_IN_OVERLAY
    ]
    panel_top = _draw_mask_thumbnail(frame, mask) + 8
    panel_x = max(4, frame.shape[1] - 520)
    panel_bottom = min(
        frame.shape[0] - 2,
        panel_top + 22 + 34 * len(relevant),
    )
    if relevant and panel_bottom > panel_top:
        cv2.rectangle(
            frame, (panel_x, panel_top), (frame.shape[1] - 4, panel_bottom),
            (20, 20, 20), -1,
        )
    cv2.putText(
        frame, f"CANDIDATES {len(relevant)}/{len(candidates)}",
        (panel_x + 5, panel_top + 15), cv2.FONT_HERSHEY_SIMPLEX,
        0.40, (255, 255, 255), 1, cv2.LINE_AA,
    )

    for display_index, (candidate_index, candidate) in enumerate(relevant):
        box = candidate.get("box")
        state = str(candidate.get("state", "ABSENT"))
        color = {
            "PRESENT": (0, 220, 0),
            "UNCERTAIN": (0, 190, 255),
            "ABSENT": (0, 0, 255),
        }.get(state, (0, 190, 255))
        bands = candidate.get("bands", [])
        selected = bool(selected_bands) and bands == selected_bands
        if isinstance(box, (list, tuple)) and len(box) == 4:
            x, y, width, height = (int(round(value)) for value in box)
            cv2.rectangle(
                frame, (x, y), (x + width, y + height),
                (255, 255, 0) if selected else color, 3 if selected else 1,
            )
        band_points = [
            (int(round(band["x"])), int(round(band["y"])))
            for band in bands
            if isinstance(band, dict) and "x" in band and "y" in band
        ]
        for point in band_points:
            cv2.circle(
                frame, point, 4 if selected else 3,
                (255, 255, 0) if selected else color, -1,
            )
        if len(band_points) > 1:
            cv2.polylines(
                frame, [np.asarray(band_points, dtype=np.int32)], False,
                (255, 255, 0) if selected else color, 2 if selected else 1,
                cv2.LINE_AA,
            )

        near_bands = _near_band_count(candidate, roi)
        marker = "SEL" if selected else f"C{candidate_index}"
        clipped = candidate.get("clipped")
        clipped_text = "Y" if clipped is True else "N" if clipped is False else "--"
        first_line = (
            f"{marker} {state} {candidate.get('reason', '--')} "
            f"S={_metric_text(candidate.get('score'), 2)} "
            f"B={len(bands)} N={near_bands}"
        )
        second_line = (
            f"E={_metric_text(candidate.get('extent'))} "
            f"T={_metric_text(candidate.get('thickness'))} "
            f"L={_metric_text(candidate.get('elongation'))} "
            f"CLIP={clipped_text}"
        )
        line_y = panel_top + 32 + display_index * 34
        cv2.putText(
            frame, first_line, (panel_x + 5, line_y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.36, color, 1, cv2.LINE_AA,
        )
        cv2.putText(
            frame, second_line, (panel_x + 5, line_y + 14),
            cv2.FONT_HERSHEY_SIMPLEX, 0.36, color, 1, cv2.LINE_AA,
        )
        if state != "PRESENT" and isinstance(box, (list, tuple)) and len(box) == 4:
            rejection = _rejection_label(candidate, frame.shape[1], roi)
            cv2.putText(
                frame, f"REJ{candidate_index} {rejection}",
                (max(2, x), max(16, y - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                0.38, color, 1, cv2.LINE_AA,
            )


def recording_overlay(frame, label, sequence, reading, roi=None, mask=None):
    """Acrescenta somente a identificação da gravação ao overlay frontal."""

    del roi, mask

    state = str(reading.get("forwardPathState", "UNCERTAIN"))
    confidence = reading.get(
        "forwardPathConfidence", reading.get("forwardLineConfidence", 0.0)
    )
    position = reading.get("forwardLinePosition")
    confidence_text = (
        f"{float(confidence):.2f}"
        if isinstance(confidence, (int, float)) and math.isfinite(float(confidence))
        else "--"
    )
    position_text = (
        f"{float(position):+.2f}"
        if isinstance(position, (int, float)) and math.isfinite(float(position))
        else "--"
    )
    lines = (
        f"REC {label}",
        f"SEQ {sequence}  {state}",
        f"CONF {confidence_text}  POS {position_text}",
    )
    for index, text in enumerate(lines):
        y = 24 + index * 22
        cv2.putText(
            frame, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
            (0, 0, 0), 3, cv2.LINE_AA,
        )
        cv2.putText(
            frame, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
            (0, 0, 255), 1, cv2.LINE_AA,
        )
    return frame


class ForwardReacquisitionRecorder:
    """Copia amostras para uma fila; falhas de disco nunca chegam ao controle."""

    def __init__(self, control_path=CONTROL_PATH, session_root=SESSION_ROOT):
        self.control_path = Path(control_path)
        self.session_root = Path(session_root)
        self.queue = Queue(maxsize=2)
        self.last_capture_monotonic = 0.0
        self.last_control_mtime_ns = None
        self.control = None
        self.failed_session = None
        self.worker = threading.Thread(target=self._writer_loop, daemon=True)
        self.worker.start()

    def current_control(self):
        """Relê o controle somente quando START ou STOP alterar o arquivo."""

        try:
            mtime_ns = self.control_path.stat().st_mtime_ns
        except (FileNotFoundError, OSError):
            self.control = None
            self.last_control_mtime_ns = None
            return None
        if mtime_ns != self.last_control_mtime_ns:
            self.control = _read_control(self.control_path)
            self.last_control_mtime_ns = mtime_ns
            self.last_capture_monotonic = 0.0
            if (
                self.control is not None
                and self.control["session"] != self.failed_session
            ):
                self.failed_session = None
        return self.control

    def active(self):
        control = self.current_control()
        return (
            control is not None
            and control["session"] != self.failed_session
        )

    def capture_due(self, now=None):
        control = self.current_control()
        if control is None or control["session"] == self.failed_session:
            return False
        now = time.monotonic() if now is None else float(now)
        return now - self.last_capture_monotonic >= 1.0 / control["imageFps"]

    def submit(
        self,
        frame,
        mask,
        overlay,
        reading,
        reference,
        roi,
        timestamp,
        sequence,
        now=None,
        capture_due_confirmed=False,
    ):
        """Enfileira cópias sem bloquear a captura nem alterar os objetos recebidos."""

        if not capture_due_confirmed and not self.capture_due(now):
            return False
        try:
            control = dict(self.control)
            timestamp = float(timestamp)
            sample = {
                "control": control,
                "frame": frame.copy(),
                "mask": None if mask is None else mask.copy(),
                "overlay": overlay.copy(),
                "metadata": _finite_json({
                    "timestamp": timestamp,
                    "timestampUtc": datetime.fromtimestamp(
                        timestamp, timezone.utc
                    ).isoformat(),
                    "forwardLineSequence": int(sequence),
                    "forwardLineTimestamp": timestamp,
                    "forwardPathState": reading.get(
                        "forwardPathState", "UNCERTAIN"
                    ),
                    "forwardLinePresent": bool(
                        reading.get("forwardLinePresent", False)
                    ),
                    "forwardLineVisible": bool(
                        reading.get("forwardLineVisible", False)
                    ),
                    "forwardLinePosition": reading.get(
                        "forwardLinePosition"
                    ),
                    "forwardLineConfidence": reading.get(
                        "forwardLineConfidence", 0.0
                    ),
                    "forwardPathConfidence": reading.get(
                        "forwardPathConfidence",
                        reading.get("forwardLineConfidence", 0.0),
                    ),
                    "forwardPathComponents": reading.get(
                        "forwardPathComponents", {}
                    ),
                    "candidates": reading.get("candidates", []),
                    "selectedBands": reading.get("selectedBands", []),
                    "roi": {
                        "x0": roi[0],
                        "y0": roi[1],
                        "x1": roi[2],
                        "y1": roi[3],
                    },
                    "bottomReference": reference,
                    "prediction": reading.get("prediction", []),
                    "maskAvailable": mask is not None,
                }),
            }
            self.queue.put_nowait(sample)
        except Full:
            return False
        except Exception as error:
            # A captura diagnóstica nunca propaga uma falha para a CAM1.
            if self.control is not None:
                self.failed_session = self.control.get("session")
            print(
                "Gravação diagnóstica da CAM1 desativada após falha ao "
                f"enfileirar: {error}",
                flush=True,
            )
            return False
        self.last_capture_monotonic = time.monotonic() if now is None else float(now)
        return True

    def _writer_loop(self):
        while True:
            sample = self.queue.get()
            try:
                self._write_sample(sample)
            except Exception as error:
                # A instrumentação é descartável; o erro nunca pode atingir o loop da câmera.
                self.failed_session = sample["control"]["session"]
                print(
                    f"Gravação diagnóstica da CAM1 desativada após falha: {error}",
                    flush=True,
                )
            finally:
                self.queue.task_done()

    def _write_sample(self, sample):
        session = sample["control"]["session"]
        session_dir = self.session_root / session
        metadata = sample["metadata"]
        sequence = metadata["forwardLineSequence"]
        timestamp_ns = int(round(metadata["timestamp"] * 1_000_000_000))
        filename = f"{sequence:010d}_{timestamp_ns}.png"

        for directory in ("frames", "masks", "overlays"):
            (session_dir / directory).mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(
            str(session_dir / "frames" / filename),
            sample["frame"],
            [cv2.IMWRITE_PNG_COMPRESSION, PNG_COMPRESSION],
        ):
            raise OSError("não foi possível salvar o frame frontal")
        if sample["mask"] is not None and not cv2.imwrite(
            str(session_dir / "masks" / filename),
            sample["mask"],
            [cv2.IMWRITE_PNG_COMPRESSION, PNG_COMPRESSION],
        ):
            raise OSError("não foi possível salvar a máscara frontal")
        if not cv2.imwrite(
            str(session_dir / "overlays" / filename),
            sample["overlay"],
            [cv2.IMWRITE_PNG_COMPRESSION, PNG_COMPRESSION],
        ):
            raise OSError("não foi possível salvar o overlay frontal")

        metadata["frameFile"] = f"frames/{filename}"
        metadata["maskFile"] = f"masks/{filename}" if sample["mask"] is not None else None
        metadata["overlayFile"] = f"overlays/{filename}"
        with (session_dir / "vision.jsonl").open("a", encoding="utf-8") as output:
            output.write(json.dumps(metadata, ensure_ascii=False, allow_nan=False) + "\n")


def _git_commit():
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=PROJECT_ROOT,
            check=True, capture_output=True, text=True, timeout=2,
        )
        return result.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


def start_session(label, image_fps=DEFAULT_IMAGE_FPS,
                  control_path=CONTROL_PATH, session_root=SESSION_ROOT):
    if not isinstance(label, str) or not SAFE_NAME_PATTERN.fullmatch(label):
        raise ValueError("O label deve conter somente letras, números, '_' ou '-'.")
    image_fps = float(image_fps)
    if not MIN_IMAGE_FPS <= image_fps <= MAX_IMAGE_FPS:
        raise ValueError(f"FPS deve ficar entre {MIN_IMAGE_FPS} e {MAX_IMAGE_FPS}.")
    if _read_control(control_path) is not None:
        raise RuntimeError("Já existe uma sessão diagnóstica ativa; execute stop primeiro.")

    started_at = datetime.now(timezone.utc)
    session = started_at.strftime("%Y%m%dT%H%M%S_%fZ_") + label
    session_dir = Path(session_root) / session
    for directory in ("frames", "masks", "overlays"):
        (session_dir / directory).mkdir(parents=True, exist_ok=False)
    (session_dir / "vision.jsonl").touch()
    (session_dir / "control.jsonl").touch()
    profile = CAMERA_PROFILES["forward"]
    vision = profile["vision"]
    session_data = {
        "label": label,
        "session": session,
        "startedAt": started_at.isoformat(),
        "endedAt": None,
        "status": "recording",
        "gitCommit": _git_commit(),
        "camera": {
            "role": "forward",
            "mainResolution": {
                "width": profile["main_size"][0],
                "height": profile["main_size"][1],
            },
            "sensorResolution": {
                "width": profile["sensor_size"][0],
                "height": profile["sensor_size"][1],
            },
            "sensorBitDepth": profile["sensor_bit_depth"],
            "targetFps": profile["target_fps"],
            "imageRecordingFps": image_fps,
            "configuredRotationDegrees": profile["rotation_degrees"],
            "appliedArrayRotationDegrees": 180,
            "format": "RGB888/BGR array",
        },
        "forwardVision": {
            "roiNormalized": {"x0": 0.05, "y0": 0.55, "x1": 0.95, "y1": 1.0},
            "minimumComponentAreaPx": 120,
            "segmentation": {
                "lineThreshold": vision["line_threshold"],
                "openKernelSize": vision["open_kernel_size"],
                "closeKernelSize": vision["close_kernel_size"],
            },
            "presenceConfig": dict(FORWARD_PRESENCE_CONFIG),
            "pathConfig": dict(FORWARD_PATH_CONFIG),
            "referenceTimeoutSeconds": GAP_VALIDATION_CONFIG[
                "reference_timeout"
            ],
        },
    }
    _write_json_atomic(session_dir / "session.json", session_data)
    control = {
        "active": True,
        "session": session,
        "label": label,
        "imageFps": image_fps,
        "startedAt": started_at.isoformat(),
    }
    _write_json_atomic(Path(control_path), control)
    return session_dir


def stop_session(control_path=CONTROL_PATH, session_root=SESSION_ROOT):
    control_path = Path(control_path)
    control = _read_control(control_path)
    if control is None:
        raise RuntimeError("Não existe sessão diagnóstica ativa.")
    stopped = dict(control)
    stopped["active"] = False
    stopped["stoppedAt"] = datetime.now(timezone.utc).isoformat()
    _write_json_atomic(control_path, stopped)

    session_dir = Path(session_root) / control["session"]
    session_path = session_dir / "session.json"
    session_data = json.loads(session_path.read_text(encoding="utf-8"))
    session_data["endedAt"] = stopped["stoppedAt"]
    session_data["status"] = "stopped"
    _write_json_atomic(session_path, session_data)
    return session_dir


def main(argv=None):
    parser = argparse.ArgumentParser(description="Grava diagnóstico da CAM1.")
    subparsers = parser.add_subparsers(dest="command", required=True)
    start = subparsers.add_parser("start")
    start.add_argument("label")
    start.add_argument("--fps", type=float, default=DEFAULT_IMAGE_FPS)
    subparsers.add_parser("stop")
    subparsers.add_parser("status")
    args = parser.parse_args(argv)

    if args.command == "start":
        print(start_session(args.label, args.fps))
        return 0
    if args.command == "stop":
        print(stop_session())
        return 0
    control = _read_control()
    print(json.dumps(control or {"active": False}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
