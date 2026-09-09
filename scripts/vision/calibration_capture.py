"""Captura experimental limitada, com pares exatos e retorno aos controles originais."""

import base64
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import select
import socket
import struct
import threading
import time

import cv2

from . import camera_config as config


REQUEST_PATH = Path("/dev/shm/obr_line_calibration_request.json")
RESULT_PATH = Path("/dev/shm/obr_line_calibration_result.json")
CAPTURE_ROOT = Path(__file__).resolve().parents[2] / "calibration" / "captures"
# Cem frames 480×360 cabem em memória sem gravar PNG no caminho da visão.
MAX_FRAMES = 100
# Um segundo descarta controles em trânsito; os metadados ainda precisam
# confirmar os valores efetivos, pois exposição e ganho sofrem quantização.
SETTLE_FRAMES = 30


def safe_json(value):
    """Converte metadados sem transformar ausência em um número inventado."""
    if value is None or isinstance(value, (str, int, bool)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(k): safe_json(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [safe_json(v) for v in value]
    return str(value)


def publish_result(value):
    temporary = RESULT_PATH.with_suffix(".tmp")
    value = {"timestamp_unix_seconds": time.time(), **value}
    temporary.write_text(json.dumps(safe_json(value), indent=2, allow_nan=False), encoding="utf-8")
    os.replace(temporary, RESULT_PATH)


def original_controls():
    return {"AeEnable": True, "AwbEnable": True,
            "ExposureValue": config.CAMERA_EXPOSURE_VALUE,
            "Sharpness": config.CAMERA_SHARPNESS,
            "Contrast": config.CAMERA_CONTRAST,
            "Saturation": config.CAMERA_SATURATION}


class StationaryGuard:
    """Lê a telemetria existente; nunca envia Start, comandos ou potência."""

    def __init__(self):
        self.socket = socket.create_connection(("127.0.0.1", 8080), timeout=1)
        self.buffer = b""
        self.last_received = 0.0
        self.telemetry = {}
        key = base64.b64encode(os.urandom(16)).decode()
        request = ("GET /ws HTTP/1.1\r\nHost: 127.0.0.1:8080\r\nUpgrade: websocket\r\n"
                   "Connection: Upgrade\r\nSec-WebSocket-Version: 13\r\n"
                   f"Sec-WebSocket-Key: {key}\r\n\r\n")
        try:
            self.socket.sendall(request.encode())
            while b"\r\n\r\n" not in self.buffer:
                data = self.socket.recv(32768)
                if not data:
                    raise OSError("Dashboard desconectado.")
                self.buffer += data
            header, self.buffer = self.buffer.split(b"\r\n\r\n", 1)
            expected = base64.b64encode(hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest())
            if b" 101 " not in header or expected.lower() not in header.lower():
                raise OSError("Handshake de telemetria inválido.")
            deadline = time.monotonic() + 1
            while not self.last_received and time.monotonic() < deadline:
                self.poll(0.05)
            self.require_stopped()
        except Exception:
            self.close()
            raise

    def poll(self, timeout=0):
        """Consome frames completos do servidor local com limite de tamanho."""
        if select.select([self.socket], [], [], timeout)[0]:
            data = self.socket.recv(65536)
            if not data:
                raise OSError("Telemetria desconectada.")
            self.buffer += data
        while len(self.buffer) >= 2:
            opcode, length = self.buffer[0] & 15, self.buffer[1] & 127
            offset = 2
            if self.buffer[1] & 128 or not self.buffer[0] & 128:
                raise ValueError("Frame de telemetria inesperado.")
            if length == 126:
                if len(self.buffer) < 4:
                    return
                length, offset = struct.unpack("!H", self.buffer[2:4])[0], 4
            if length == 127 or length > 32768:
                raise ValueError("Telemetria excedeu o limite de tamanho.")
            if len(self.buffer) < offset + length:
                return
            payload = self.buffer[offset:offset + length]
            self.buffer = self.buffer[offset + length:]
            if opcode == 8:
                raise OSError("Dashboard encerrou a telemetria.")
            if opcode == 1:
                self.telemetry = json.loads(payload)
                self.last_received = time.monotonic()

    def require_stopped(self):
        self.poll()
        t = self.telemetry
        if (time.monotonic() - self.last_received > 1
                or t.get("emergency") is not True
                or t.get("esp32EmergencyStop") is not True
                or t.get("esp32SensorFresh") is not True
                or any(t.get(k) != 0 for k in ("appliedLeft", "appliedRight", "leftEncoderRate", "rightEncoderRate"))):
            raise RuntimeError("Calibração exige E-Stop confirmado e motores/encoders zerados.")

    def close(self):
        self.socket.close()


def validate_request(request, camera):
    """Aceita somente controles conhecidos e um nome que não escape do dataset."""
    name = request["experiment"]
    if not isinstance(name, str) or not name or len(name) > 100 or any(c not in "abcdefghijklmnopqrstuvwxyz0123456789_-" for c in name):
        raise ValueError("Identificador de experimento inválido.")
    count = request.get("frames", 60)
    if type(count) is not int or not 30 <= count <= MAX_FRAMES:
        raise ValueError("Solicite entre 30 e 100 frames.")
    controls = request.get("controls", {})
    segmentation = request.get("segmentation", {})
    allowed_segmentation = {
        "line_max_background_ratio_percent",
        "line_exclude_green",
        "line_min_threshold",
        "line_illumination_correction_enabled",
    }
    if not isinstance(segmentation, dict) or set(segmentation) - allowed_segmentation:
        raise ValueError(
            "Somente razão relativa, limite mínimo, exclusão do verde e "
            "compensação de iluminação podem variar nesta captura."
        )
    if "line_min_threshold" in segmentation:
        minimum = segmentation["line_min_threshold"]
        maximum = config.CAMERA_PROFILES["down"]["vision"]["line_max_brightness"]
        if type(minimum) is not int or not 0 <= minimum <= maximum:
            raise ValueError("O limite mínimo deve ser inteiro entre zero e o teto de cinza do perfil.")
    if "line_exclude_green" in segmentation and type(segmentation["line_exclude_green"]) is not bool:
        raise ValueError("A exclusão do verde deve ser booleana.")
    if (
        "line_illumination_correction_enabled" in segmentation
        and type(segmentation["line_illumination_correction_enabled"]) is not bool
    ):
        raise ValueError("A compensação de iluminação deve ser booleana.")
    ratio = segmentation.get("line_max_background_ratio_percent")
    if "line_max_background_ratio_percent" in segmentation and (type(ratio) is not int or not 1 <= ratio <= 100):
        raise ValueError("A razão deve ser um inteiro entre 1 e 100.")
    if segmentation and controls:
        raise ValueError("Altere câmera ou segmentação, uma família por captura.")
    allowed = {"ExposureTime", "AnalogueGain", "AeEnable", "AwbEnable", "ColourGains"}
    if not isinstance(controls, dict) or set(controls) - allowed:
        raise ValueError("Controle experimental não permitido.")
    if controls.get("AeEnable") is False and not {"ExposureTime", "AnalogueGain"} <= controls.keys():
        raise ValueError("AE fixo exige exposição e ganho explícitos.")
    if controls.get("AwbEnable") is False and "ColourGains" not in controls:
        raise ValueError("AWB fixo exige os dois ganhos de cor.")
    if ({"ExposureTime", "AnalogueGain"} & controls.keys()) and controls.get("AeEnable") is not False:
        raise ValueError("Exposição e ganho manuais exigem AeEnable=false.")
    if "ColourGains" in controls and controls.get("AwbEnable") is not False:
        raise ValueError("Ganhos de cor manuais exigem AwbEnable=false.")
    for key, value in controls.items():
        if key not in camera.camera_controls:
            raise ValueError(f"Câmera não suporta {key}.")
        if key in ("AeEnable", "AwbEnable"):
            if type(value) is not bool:
                raise ValueError("Controles automáticos devem ser booleanos.")
            continue
        if key == "ColourGains" and (not isinstance(value, (tuple, list)) or len(value) != 2):
            raise ValueError("ColourGains exige exatamente dois valores.")
        if key != "ColourGains" and isinstance(value, (tuple, list)):
            raise ValueError(f"{key} exige um único valor.")
        low, high, _default = camera.camera_controls[key]
        values = value if isinstance(value, (tuple, list)) else [value]
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not low <= v <= high for v in values):
            raise ValueError(f"{key} fora dos limites do sensor.")
        if key == "ExposureTime" and (type(value) is not int or value > 33333):
            raise ValueError("Exposição deve ser inteira em µs e caber em 30 FPS.")
    return name, count, controls


class CalibrationCapture:
    """Mantém no máximo uma sequência em RAM e grava PNG fora do loop visual."""

    def __init__(self, camera, profile, details):
        self.camera, self.profile, self.details = camera, profile, details
        self.active = False
        self.frames = []
        self.guard = None
        self.writer = None
        self.changed_controls = False
        self.original_vision = None

    def before_frame(self):
        try:
            if self.active:
                self.guard.require_stopped()
            elif REQUEST_PATH.exists() and not (self.writer and self.writer.is_alive()):
                request_text = REQUEST_PATH.read_text(encoding="utf-8")
                REQUEST_PATH.unlink()
                request = json.loads(request_text)
                name, count, controls = validate_request(request, self.camera)
                output = CAPTURE_ROOT / name
                if output.exists():
                    raise FileExistsError("Experimento já existe; use outro identificador.")
                self.guard = StationaryGuard()
                output.mkdir(parents=True)
                self.request, self.output, self.count = request, output, count
                self.frames, self.settle = [], SETTLE_FRAMES
                segmentation = request.get("segmentation", {})
                if segmentation:
                    # O aplicativo conserva a referência deste dicionário. O
                    # ajuste temporário só ocorre depois de validar o E-Stop.
                    self.original_vision = copy.deepcopy(self.profile["vision"])
                    self.profile["vision"].update(segmentation)
                source_root = Path(__file__).parent
                self.capture_config = {
                    "camera_profile": copy.deepcopy(self.profile), "camera_details": self.details,
                    "segmentation_before_experiment": self.original_vision,
                    "requested_camera_controls": {**original_controls(), **controls},
                    "supported_camera_controls": safe_json(self.camera.camera_controls),
                    "camera_configuration": safe_json(self.camera.camera_configuration()),
                    "request": request, "mask_origin": "live_same_request_before_overlay",
                    "raw_format": "lossless PNG from BGR main; not Bayer", "opencv": cv2.__version__,
                    "component_filters": {"minimum_area_px": config.LINE_MIN_COMPONENT_AREA_PX,
                                          "minimum_thickness_px": config.LINE_MIN_COMPONENT_THICKNESS_PX,
                                          "minimum_core_ratio": config.LINE_MIN_COMPONENT_CORE_RATIO},
                    "source_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in source_root.glob("*.py")},
                    "safety_at_start": self.guard.telemetry,
                }
                if controls:
                    self.changed_controls = True
                    self.camera.set_controls(controls)
                self.active = True
                publish_result({"state": "capturing", "experiment": name})
        except Exception as error:
            self.abort(error)
        return self.active

    def after_frame(self, frame, mask, structural, metadata, sequence, fusion, green_status=None):
        if not self.active:
            return
        try:
            self.guard.require_stopped()
            if self.settle:
                self.settle -= 1
                return
            self.frames.append((frame.copy(), mask.copy(), structural.copy(), {
                "timestamp_unix_seconds": time.time(), "line_sequence": sequence,
                "camera_metadata": safe_json(metadata), "live_trajectory": copy.deepcopy(fusion),
                "green_status": safe_json(green_status),
                "robot_telemetry": copy.deepcopy(self.guard.telemetry),
            }))
            if len(self.frames) == self.count:
                self.restore()
                self.active = False
                self.writer = threading.Thread(target=self.save, daemon=False)
                self.writer.start()
        except Exception as error:
            self.abort(error)

    def restore(self):
        """Restaura câmera e segmentação também em aborto e encerramento."""
        if self.original_vision is not None:
            self.profile["vision"].clear()
            self.profile["vision"].update(self.original_vision)
            self.original_vision = None
        if self.changed_controls:
            self.camera.set_controls(original_controls())
            self.changed_controls = False
        if self.guard is not None:
            self.guard.close()
            self.guard = None

    def abort(self, error):
        self.active = False
        try:
            self.restore()
        finally:
            self.frames = []
            publish_result({"state": "error", "error": str(error)})
            print(f"Calibração abortada: {error}", flush=True)

    def save(self):
        try:
            for name in ("raw", "mask", "structural", "metadata"):
                (self.output / name).mkdir()
            (self.output / "config.json").write_text(json.dumps(safe_json(self.capture_config), indent=2, allow_nan=False), encoding="utf-8")
            for index, (frame, mask, structural, metadata) in enumerate(self.frames):
                stem = f"{index:04d}"
                for stage, pixels in (("raw", frame), ("mask", mask), ("structural", structural)):
                    if not cv2.imwrite(str(self.output / stage / (stem + ".png")), pixels):
                        raise OSError("Falha ao gravar PNG de calibração.")
                metadata["raw_sha256"] = hashlib.sha256((self.output / "raw" / (stem + ".png")).read_bytes()).hexdigest()
                (self.output / "metadata" / (stem + ".json")).write_text(json.dumps(metadata, indent=2, allow_nan=False), encoding="utf-8")
            result = {"timestamp_unix_seconds": time.time(), "state": "complete",
                      "experiment": self.request["experiment"], "frames": len(self.frames),
                      "output": str(self.output), "original_controls_restored": True,
                      "original_segmentation_restored": True}
            # /dev/shm é volátil. A confirmação também fica junto dos dados para
            # sobreviver à limpeza da memória compartilhada ou ao próximo boot.
            (self.output / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
            publish_result(result)
        except Exception as error:
            publish_result({"state": "error", "error": str(error)})
        finally:
            self.frames = []

    def close(self):
        if self.active:
            self.abort(RuntimeError("Serviço encerrado antes de completar a sequência."))
        self.restore()
        if self.writer:
            self.writer.join(timeout=10)
