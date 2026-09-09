from datetime import datetime, timezone
from pathlib import Path
from queue import Empty, Full, Queue
import json
import math
import os
import re
import threading
import time

import cv2


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATASET_ROOT = PROJECT_ROOT / "dataset" / "raw"

CONTROL_PATH = Path("/dev/shm/obr_silver_dataset_capture.json")
STATUS_PATH = Path("/dev/shm/obr_silver_dataset_status.json")

CAMERA_ROLES = ("forward", "down")
CLASS_NAMES = ("black", "other", "silver")

JPEG_QUALITY = 95
MIN_CAPTURE_FPS = 0.5
MAX_CAPTURE_FPS = 10.0
ACTIVE_STATUS_TIMEOUT_SECONDS = 4.0

SAFE_NAME_PATTERN = re.compile(r"^[A-Za-z0-9_-]+$")


def empty_dataset_status():
    return {
        "datasetCaptureActive": False,
        "datasetCaptureStale": False,
        "datasetCaptureCamera": "",
        "datasetCaptureSession": "",
        "datasetCaptureLabel": "",
        "datasetCaptureFps": 0.0,
        "datasetCaptureCounts": {"black": 0, "other": 0, "silver": 0},
        "datasetCaptureDropped": 0,
        "datasetCaptureLastSavedAt": 0.0,
        "datasetCaptureUpdatedAt": 0.0,
    }


def read_dataset_status():
    try:
        data = json.loads(STATUS_PATH.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, ValueError, TypeError, json.JSONDecodeError):
        return empty_dataset_status()

    if not isinstance(data, dict):
        return empty_dataset_status()

    status = empty_dataset_status()
    status["datasetCaptureActive"] = data.get("datasetCaptureActive") is True
    status["datasetCaptureStale"] = data.get("datasetCaptureStale") is True

    for key in ("datasetCaptureCamera", "datasetCaptureSession", "datasetCaptureLabel"):
        value = data.get(key)
        if isinstance(value, str):
            status[key] = value

    for key in ("datasetCaptureFps", "datasetCaptureLastSavedAt", "datasetCaptureUpdatedAt"):
        try:
            value = float(data.get(key, 0.0))
        except (TypeError, ValueError):
            value = 0.0
        status[key] = value if math.isfinite(value) and value >= 0.0 else 0.0

    raw_counts = data.get("datasetCaptureCounts")
    if isinstance(raw_counts, dict):
        for class_name in CLASS_NAMES:
            try:
                value = int(raw_counts.get(class_name, 0))
            except (TypeError, ValueError, OverflowError):
                value = 0
            status["datasetCaptureCounts"][class_name] = max(0, value)

    try:
        dropped_count = int(data.get("datasetCaptureDropped", 0))
    except (TypeError, ValueError, OverflowError):
        dropped_count = 0
    status["datasetCaptureDropped"] = max(0, dropped_count)

    if status["datasetCaptureActive"] is True:
        try:
            updated_at = status["datasetCaptureUpdatedAt"]
            age = time.time() - updated_at
        except (TypeError, ValueError):
            updated_at = 0.0
            age = ACTIVE_STATUS_TIMEOUT_SECONDS + 1.0

        if updated_at <= 0.0 or age < 0.0 or age > ACTIVE_STATUS_TIMEOUT_SECONDS:
            status["datasetCaptureActive"] = False
            status["datasetCaptureStale"] = True

    return status


class SilverDatasetRecorder:
    def __init__(self, camera_role):
        if camera_role not in CAMERA_ROLES:
            raise ValueError(f"Câmera inválida: {camera_role}")

        self.camera_role = camera_role
        self.frame_queue = Queue(maxsize=2)
        self.temp_status_path = Path(f"/dev/shm/obr_silver_dataset_status_{camera_role}.tmp.json")

        self.control_mtime_ns = None
        self.control = None
        self.session_key = None
        self.session_counts_ready = False
        self.last_capture_at = 0.0

        self.counts = {class_name: 0 for class_name in CLASS_NAMES}
        self.dropped_count = 0
        self.last_saved_at = 0.0

        self.status_lock = threading.Lock()
        self.storage_lock = threading.Lock()
        self.status_queue = Queue(maxsize=1)

        self.writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
        self.writer_thread.start()
        self.status_thread = threading.Thread(target=self._status_loop, daemon=True)
        self.status_thread.start()

    def submit(self, frame):
        control = self._read_control()

        if control is None or control["camera"] != self.camera_role or not control["active"]:
            return False

        with self.status_lock:
            session_counts_ready = (
                self.session_key == (control["camera"], control["session"])
                and self.session_counts_ready
            )

        if not session_counts_ready:
            return False

        now = time.monotonic()
        capture_interval = 1.0 / control["fps"]

        if now - self.last_capture_at < capture_interval:
            return False

        self.last_capture_at = now

        try:
            self.frame_queue.put_nowait((frame.copy(), dict(control), time.time_ns()))
            return True
        except Full:
            with self.status_lock:
                self.dropped_count += 1

            self._publish_status()
            return False

    def _read_control(self):
        try:
            stat = CONTROL_PATH.stat()
        except FileNotFoundError:
            self.control = None
            self.control_mtime_ns = None
            return None
        except OSError:
            return self.control

        if self.control_mtime_ns == stat.st_mtime_ns:
            return self.control

        try:
            data = json.loads(CONTROL_PATH.read_text(encoding="utf-8"))
            control = self._validate_control(data)
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
            print(f"Controle de dataset inválido: {error}", flush=True)
            self.control = None
            self.control_mtime_ns = stat.st_mtime_ns
            return None

        previous_identity = None

        if self.control is not None:
            previous_identity = (
                self.control["camera"],
                self.control["session"],
                self.control["label"],
                self.control["fps"],
                self.control["active"],
            )

        current_identity = (
            control["camera"],
            control["session"],
            control["label"],
            control["fps"],
            control["active"],
        )

        self.control = control
        self.control_mtime_ns = stat.st_mtime_ns

        if control["camera"] == self.camera_role:
            session_key = (control["camera"], control["session"])

            if session_key != self.session_key:
                with self.status_lock:
                    self.counts = {class_name: 0 for class_name in CLASS_NAMES}
                    self.dropped_count = 0
                    self.last_saved_at = 0.0
                    self.session_key = session_key
                    self.session_counts_ready = False

                self._queue_status(reload_counts=True)

            if current_identity != previous_identity:
                self.last_capture_at = 0.0
                self._publish_status()

        return self.control

    def _validate_control(self, data):
        if not isinstance(data, dict):
            raise ValueError("O controle deve ser um objeto JSON.")

        active = data.get("active")
        camera = data.get("camera")
        session = data.get("session")
        label = data.get("label")
        fps = float(data.get("fps", 4.0))

        if not isinstance(active, bool):
            raise ValueError("'active' deve ser booleano.")

        if camera not in CAMERA_ROLES:
            raise ValueError(f"Câmera inválida: {camera}")

        if label not in CLASS_NAMES:
            raise ValueError(f"Classe inválida: {label}")

        if not isinstance(session, str) or not SAFE_NAME_PATTERN.fullmatch(session):
            raise ValueError("Nome de sessão inválido.")

        if not MIN_CAPTURE_FPS <= fps <= MAX_CAPTURE_FPS:
            raise ValueError(f"FPS deve ficar entre {MIN_CAPTURE_FPS} e {MAX_CAPTURE_FPS}.")

        return {"active": active, "camera": camera, "session": session, "label": label, "fps": fps}

    def _load_counts(self, camera, session):
        session_dir = DATASET_ROOT / camera / session

        return {
            class_name: len(list((session_dir / class_name).glob("*.jpg")))
            for class_name in CLASS_NAMES
        }

    def _writer_loop(self):
        while True:
            frame, control, timestamp_ns = self.frame_queue.get()

            try:
                self._save_frame(frame, control, timestamp_ns)
            except Exception as error:
                print(f"Falha ao salvar frame do dataset: {error}", flush=True)
            finally:
                self.frame_queue.task_done()

    def _save_frame(self, frame, control, timestamp_ns):
        session_dir = DATASET_ROOT / control["camera"] / control["session"]
        class_dir = session_dir / control["label"]

        with self.storage_lock:
            class_dir.mkdir(parents=True, exist_ok=True)
            self._ensure_session_metadata(session_dir, control)

            output_path = class_dir / f"{timestamp_ns}.jpg"

            success = cv2.imwrite(str(output_path), frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])

            if not success:
                raise RuntimeError(f"OpenCV não conseguiu salvar {output_path}")

            with self.status_lock:
                if self.session_key == (control["camera"], control["session"]):
                    self.counts[control["label"]] += 1
                    self.last_saved_at = time.time()

        self._publish_status()

    def _publish_status(self):
        control = self.control

        if control is None or control["camera"] != self.camera_role:
            return

        self._queue_status()

    def _queue_status(self, reload_counts=False):
        """Agenda a publicação sem bloquear o loop de captura da câmera."""

        try:
            self.status_queue.put_nowait(bool(reload_counts))
        except Full:
            try:
                pending_reload = self.status_queue.get_nowait()
                self.status_queue.task_done()
            except Empty:
                pending_reload = False

            try:
                self.status_queue.put_nowait(bool(reload_counts or pending_reload))
            except Full:
                pass

    def _status_loop(self):
        """Grava o status em uma thread separada do loop da câmera."""

        while True:
            reload_counts = self.status_queue.get()

            try:
                self._write_status(bool(reload_counts))
            except Exception as error:
                print(f"Falha ao publicar status do dataset: {error}", flush=True)
            finally:
                self.status_queue.task_done()

    def _write_status(self, reload_counts=False):
        """Publica um retrato consistente da sessão selecionada."""

        control = self.control

        if control is None or control["camera"] != self.camera_role:
            return

        session_key = (control["camera"], control["session"])

        if reload_counts:
            with self.storage_lock:
                loaded_counts = self._load_counts(control["camera"], control["session"])

                with self.status_lock:
                    if self.session_key != session_key:
                        return
                    self.counts = loaded_counts
                    self.session_counts_ready = True

        with self.status_lock:
            if self.session_key != session_key:
                return

            status = {
                "datasetCaptureActive": bool(control["active"]),
                "datasetCaptureStale": False,
                "datasetCaptureCamera": control["camera"],
                "datasetCaptureSession": control["session"],
                "datasetCaptureLabel": control["label"],
                "datasetCaptureFps": float(control["fps"]),
                "datasetCaptureCounts": dict(self.counts),
                "datasetCaptureDropped": self.dropped_count,
                "datasetCaptureLastSavedAt": self.last_saved_at,
                "datasetCaptureUpdatedAt": time.time(),
            }

        self.temp_status_path.write_text(json.dumps(status), encoding="utf-8")
        os.replace(self.temp_status_path, STATUS_PATH)

    def _ensure_session_metadata(self, session_dir, control):
        metadata_path = session_dir / "session.json"

        if metadata_path.exists():
            return

        metadata = {
            "session": control["session"],
            "camera": control["camera"],
            "createdAt": datetime.now(timezone.utc).isoformat(),
            "source": "raw-camera-frame",
            "jpegQuality": JPEG_QUALITY,
        }

        temp_path = session_dir / "session.tmp.json"
        temp_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        os.replace(temp_path, metadata_path)
