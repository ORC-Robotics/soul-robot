"""Processa continuamente a câmera frontal e oferece seu stream de diagnóstico."""

import json
import math
import os
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

import camera_line_frame
import cv2  # type: ignore
import numpy as np

try:
    from silver_dataset_recorder import SilverDatasetRecorder, read_dataset_status
except ModuleNotFoundError as error:
    if error.name != "silver_dataset_recorder":
        raise

    # A coleta é opcional e não pode impedir a inicialização da câmera frontal.
    SilverDatasetRecorder = None

    def read_dataset_status():
        return {}
from vision.forward_path import ForwardPathTracker
from vision.gap_validation import read_json_snapshot
from vision.camera_config import GAP_VALIDATION_CONFIG, LINE_STATUS_PATH
from vision.rescue_zone import (
    RescueZoneTemporalFilter,
    ULTRASONIC_MAXIMUM_CM,
    ULTRASONIC_MINIMUM_CM,
    analyze_rescue_zones,
    draw_rescue_zone_overlay,
    serializable_results,
)


CONTROL_PATH = "/dev/shm/obr_forward_camera_enabled"
TEMP_CONTROL_PATH = "/dev/shm/obr_forward_camera_enabled.tmp"
STATUS_PATH = "/tmp/obr_forward_camera_status.json"
TEMP_STATUS_PATH = "/tmp/obr_forward_camera_status.tmp.json"
FORWARD_LINE_STATUS_PATH = "/dev/shm/obr_forward_line_status.json"
TEMP_FORWARD_LINE_STATUS_PATH = "/dev/shm/obr_forward_line_status.tmp.json"
RESCUE_ZONE_CONTROL_PATH = "/dev/shm/obr_rescue_zone_detection_enabled"
RESCUE_ZONE_STATUS_PATH = "/dev/shm/obr_rescue_zone_status.json"
TEMP_RESCUE_ZONE_STATUS_PATH = "/dev/shm/obr_rescue_zone_status.tmp.json"
# O heartbeat do C++ chega a cada 50 ms. Após 300 ms, o gate e o ultrassônico
# ficam indisponíveis mesmo se o serviço encerrar sem escrever o valor zero.
RESCUE_ZONE_INPUT_TIMEOUT_SECONDS = 0.30
STREAM_PORT = 8091
STREAM_PATH = "/stream.mjpg"
STATUS_FPS = 5
IDLE_POLL_SECONDS = 0.10
ERROR_RETRY_SECONDS = 1.0

# O profiling é temporário e permanece desligado para não afetar a CAM1 normal.
# Cada condição acumula uma janela inteira antes de publicar um único resumo.
RESCUE_ZONE_PROFILE_ENABLED = os.environ.get(
    "OBR_RESCUE_ZONE_PROFILE", "0"
).strip().lower() in ("1", "true", "yes", "on")
RESCUE_ZONE_PROFILE_WINDOW_FRAMES = 300
RESCUE_ZONE_PROFILE_METADATA_INTERVAL_FRAMES = 30


def _profile_summary(values):
    """Calcula média e P95 sem depender de bibliotecas estatísticas externas."""

    ordered = sorted(float(value) for value in values)
    percentile_index = max(0, math.ceil(len(ordered) * 0.95) - 1)
    return {
        "average": round(sum(ordered) / len(ordered), 3),
        "p95": round(ordered[percentile_index], 3),
    }


class RescueZoneCycleProfiler:
    """Agrupa custos da CAM1 por presença de cliente sem imprimir a cada frame."""

    STAGES = (
        "captureMs",
        "preparationMs",
        "forwardAssistMs",
        "rescueZoneDetectionMs",
        "overlayMs",
        "jpegEncodeMs",
        "ipcPublishMs",
        "otherMs",
        "totalCycleMs",
    )

    def __init__(self, enabled=False, window_frames=RESCUE_ZONE_PROFILE_WINDOW_FRAMES):
        self.enabled = bool(enabled)
        self.window_frames = max(1, int(window_frames))
        self.samples = {
            "withoutStreamClient": [],
            "withStreamClient": [],
        }

    def metadata_is_due(self, bucket):
        """Amostra metadados espaçadamente para não distorcer o ciclo medido."""

        if not self.enabled:
            return False
        count = len(self.samples[bucket])
        return count % RESCUE_ZONE_PROFILE_METADATA_INTERVAL_FRAMES == 0

    def record(self, has_stream_client, timings, metadata=None):
        """Registra um ciclo e emite somente resumos de janelas completas."""

        if not self.enabled:
            return
        bucket = "withStreamClient" if has_stream_client else "withoutStreamClient"
        sample = {
            stage: max(0.0, float(timings.get(stage, 0.0)))
            for stage in self.STAGES
        }
        if metadata:
            for name in ("ExposureTime", "FrameDuration"):
                value = metadata.get(name)
                if isinstance(value, (int, float)) and not isinstance(value, bool):
                    sample[name] = float(value)
        self.samples[bucket].append(sample)
        if len(self.samples[bucket]) < self.window_frames:
            return

        window = self.samples[bucket]
        stages = {
            stage: _profile_summary([sample[stage] for sample in window])
            for stage in self.STAGES
        }
        total_average_ms = stages["totalCycleMs"]["average"]
        report = {
            "profile": "CAM1_RESCUE_ZONE",
            "condition": bucket,
            "frames": len(window),
            "effectiveFps": round(1000.0 / total_average_ms, 2)
            if total_average_ms > 0.0
            else 0.0,
            "stagesMs": stages,
            "camera": {
                "targetFps": camera_line_frame.CAMERA_PROFILES["forward"]["target_fps"],
                "configuredFrameDurationLimitsUs": [
                    int(
                        1_000_000
                        / camera_line_frame.CAMERA_PROFILES["forward"]["target_fps"]
                    )
                ] * 2,
            },
        }
        for name in ("ExposureTime", "FrameDuration"):
            values = [sample[name] for sample in window if name in sample]
            if values:
                report["camera"][f"{name}Us"] = _profile_summary(values)
        print(f"CAM1_PROFILE {json.dumps(report, allow_nan=False)}", flush=True)
        self.samples[bucket] = []

# A OV5647 frontal está fisicamente invertida, mas o hvflip solicitado ao
# Picamera2 não alterou o JPEG de forma consistente. Rotacionar o array uma vez,
# antes da visão e do stream, mantém percepção e diagnóstico na mesma orientação.
FORWARD_FRAME_ROTATION_DEGREES = 180

# A ROI usa frações do frame frontal para continuar válida em testes com outras
# resoluções. Estes quatro valores são o ponto central da calibração física.
FORWARD_ASSIST_X0 = 0.05
FORWARD_ASSIST_X1 = 0.95
FORWARD_ASSIST_Y0 = 0.55
FORWARD_ASSIST_Y1 = 1.00

# Componentes menores que esta área, em pixels, são tratados como ruído. O
# limite é baixo para preservar a linha distante e não adiciona geometria da CAM0.
FORWARD_ASSIST_MIN_COMPONENT_AREA_PX = 120

running = True
stream_active = False
latest_jpeg = None
latest_jpeg_sequence = 0
active_stream_clients = 0
frame_condition = threading.Condition()
def environment_enabled():
    """Inicia o stream ligado, sem alterar a captura frontal contínua."""

    value = os.environ.get("OBR_FORWARD_CAMERA_ENABLED", "1").strip().lower()
    if value in ("1", "true", "yes", "on"):
        return True
    if value in ("0", "false", "no", "off", ""):
        return False
    raise ValueError(
        "OBR_FORWARD_CAMERA_ENABLED deve ser 0/1, false/true, no/yes ou off/on."
    )


def write_requested_enabled(enabled):
    """Publica atomicamente se o stream frontal deve ficar disponível."""

    with open(TEMP_CONTROL_PATH, "w", encoding="utf-8") as control_file:
        control_file.write("1\n" if enabled else "0\n")
    os.replace(TEMP_CONTROL_PATH, CONTROL_PATH)


def requested_enabled():
    """Mantém o stream ligado sem arquivo; zero explícito continua desligando."""

    try:
        with open(CONTROL_PATH, "r", encoding="utf-8") as control_file:
            return control_file.read().strip() == "1"
    except FileNotFoundError:
        # /dev/shm pode ser limpo durante a operação. A ausência restaura o
        # padrão ligado, enquanto um arquivo existente com "0" preserva o
        # desligamento solicitado pelo dashboard.
        return True
    except OSError:
        return False


def read_rescue_zone_detection_input(now=None):
    """Lê o gate e o ultrassônico, recusando heartbeat antigo ou malformado."""

    try:
        with open(RESCUE_ZONE_CONTROL_PATH, "r", encoding="utf-8") as control_file:
            input_status = json.load(control_file)
        timestamp = float(input_status["timestamp"])
        current_time = time.time() if now is None else float(now)
        heartbeat_fresh = (
            math.isfinite(timestamp)
            and 0.0 <= current_time - timestamp <= RESCUE_ZONE_INPUT_TIMEOUT_SECONDS
        )
        distance = input_status.get("ultrasonicDistanceCm")
        distance_valid = (
            input_status.get("ultrasonicValid") is True
            and isinstance(distance, (int, float))
            and not isinstance(distance, bool)
            and math.isfinite(float(distance))
            and ULTRASONIC_MINIMUM_CM
            <= float(distance)
            <= ULTRASONIC_MAXIMUM_CM
        )
        return {
            "enabled": heartbeat_fresh and input_status.get("enabled") is True,
            "ultrasonicFresh": (
                heartbeat_fresh and input_status.get("ultrasonicFresh") is True
            ),
            "ultrasonicValid": distance_valid,
            "ultrasonicDistanceCm": round(float(distance), 2)
            if distance_valid
            else None,
        }
    except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError):
        return {
            "enabled": False,
            "ultrasonicFresh": False,
            "ultrasonicValid": False,
            "ultrasonicDistanceCm": None,
        }


def requested_rescue_zone_detection_enabled():
    """Informa somente se o gate atual da percepção está aberto."""

    return read_rescue_zone_detection_input()["enabled"]


def save_rescue_zone_status(results, timestamp, sequence, detection_input):
    """Publica atomicamente as duas zonas observadas no mesmo frame frontal."""

    status = {
        "active": True,
        "timestamp": float(timestamp),
        "sequence": int(sequence),
        "ultrasonic": {
            "fresh": bool(detection_input["ultrasonicFresh"]),
            "valid": bool(detection_input["ultrasonicValid"]),
            "distanceCm": detection_input["ultrasonicDistanceCm"],
        },
    }
    status.update(serializable_results(results))
    with open(TEMP_RESCUE_ZONE_STATUS_PATH, "w", encoding="utf-8") as status_file:
        json.dump(status, status_file, allow_nan=False)
    os.replace(TEMP_RESCUE_ZONE_STATUS_PATH, RESCUE_ZONE_STATUS_PATH)


def clear_rescue_zone_status():
    """Invalida imediatamente o IPC ao fechar o gate ou perder a câmera."""

    for path in (RESCUE_ZONE_STATUS_PATH, TEMP_RESCUE_ZONE_STATUS_PATH):
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass
        except OSError as error:
            print(f"Falha ao remover IPC das áreas de resgate {path}: {error}", flush=True)


def save_status(enabled, active, state, fps=0.0, camera_format="", details=None,
                error_message="", processing_active=None):
    """Expõe saúde da captura sem misturar o novo IPC com o segue-faixa."""

    profile = camera_line_frame.CAMERA_PROFILES["forward"]
    width, height = profile["main_size"]
    sensor_width, sensor_height = profile["sensor_size"]
    configured_index = camera_line_frame.configured_camera_indices()["forward"]
    details = details or {}
    sensor_mode = details.get("sensorMode") or {
        "width": sensor_width,
        "height": sensor_height,
        "bitDepth": profile["sensor_bit_depth"],
        "format": "",
    }
    status = {
        "managerActive": True,
        "enabled": bool(enabled),
        "active": bool(active),
        "processingActive": (
            bool(active) if processing_active is None else bool(processing_active)
        ),
        "state": state,
        "fps": round(float(fps), 2),
        "cameraRole": "forward",
        "cameraIndex": details.get("cameraIndex", configured_index),
        "cameraId": details.get("cameraId", ""),
        "cameraModel": details.get("cameraModel", ""),
        "cameraFormat": camera_format,
        "width": width,
        "height": height,
        "mainResolution": {"width": width, "height": height},
        "sensorMode": sensor_mode,
        "scalerCrop": details.get("scalerCrop"),
        "rotationDegrees": FORWARD_FRAME_ROTATION_DEGREES,
        "captureTransform": details.get(
            "transform",
            camera_line_frame.camera_transform_settings(profile)["name"],
        ),
        "transform": "opencv-rotate-180",
        "targetCameraFps": profile["target_fps"],
        "streamPort": STREAM_PORT,
        "streamPath": STREAM_PATH,
        "error": error_message,
        "timestamp": time.time(),
    }
    status.update(read_dataset_status())
    with open(TEMP_STATUS_PATH, "w", encoding="utf-8") as status_file:
        json.dump(status, status_file, allow_nan=False)
    os.replace(TEMP_STATUS_PATH, STATUS_PATH)


def resolve_forward_assist_roi(mask_shape):
    """Converte a ROI frontal normalizada em limites inteiros exclusivos."""

    if len(mask_shape) < 2:
        raise ValueError("A máscara frontal deve possuir altura e largura.")
    height = int(mask_shape[0])
    width = int(mask_shape[1])
    if height <= 0 or width <= 0:
        raise ValueError("A máscara frontal não pode estar vazia.")
    if not (
        0.0 <= FORWARD_ASSIST_X0 < FORWARD_ASSIST_X1 <= 1.0
        and 0.0 <= FORWARD_ASSIST_Y0 < FORWARD_ASSIST_Y1 <= 1.0
    ):
        raise ValueError("Os limites normalizados da ROI frontal são inválidos.")

    x0 = max(0, min(width - 1, int(round(width * FORWARD_ASSIST_X0))))
    x1 = max(x0 + 1, min(width, int(round(width * FORWARD_ASSIST_X1))))
    y0 = max(0, min(height - 1, int(round(height * FORWARD_ASSIST_Y0))))
    y1 = max(y0 + 1, min(height, int(round(height * FORWARD_ASSIST_Y1))))
    return x0, y0, x1, y1


def calculate_forward_line_assist(filtered_line_mask, reference=None, tracker=None, now=None):
    """Avalia caminhos separados; a posição publicada pertence ao candidato escolhido."""

    if filtered_line_mask.ndim != 2:
        raise ValueError("A máscara preta frontal deve possuir um único canal.")

    tracker = tracker if tracker is not None else ForwardPathTracker()
    return tracker.analyze(filtered_line_mask, resolve_forward_assist_roi(filtered_line_mask.shape),
                           reference, time.time() if now is None else now,
                           FORWARD_ASSIST_MIN_COMPONENT_AREA_PX)


def process_forward_frame(frame, camera_format, reference=None, tracker=None, now=None):
    """Aplica somente a segmentação preta configurada para o perfil frontal."""

    profile = camera_line_frame.CAMERA_PROFILES["forward"]
    filtered_mask, roi_start_y = camera_line_frame.create_filtered_line_mask(
        frame,
        profile["vision"],
        camera_format,
    )
    if roi_start_y == 0 and filtered_mask.shape == frame.shape[:2]:
        full_filtered_mask = filtered_mask
    else:
        # A ROI do assistente é relativa ao frame completo, mesmo se a segmentação
        # frontal ganhar no futuro um recorte vertical próprio.
        full_filtered_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        available_height = min(
            filtered_mask.shape[0],
            full_filtered_mask.shape[0] - roi_start_y,
        )
        available_width = min(filtered_mask.shape[1], full_filtered_mask.shape[1])
        if available_height > 0 and available_width > 0:
            full_filtered_mask[
                roi_start_y:roi_start_y + available_height,
                :available_width,
            ] = filtered_mask[:available_height, :available_width]
    return calculate_forward_line_assist(full_filtered_mask, reference, tracker, now)


def orient_forward_frame(frame):
    """Corrige a montagem frontal antes de qualquer processamento ou stream."""

    if FORWARD_FRAME_ROTATION_DEGREES != 180:
        raise ValueError("A rotação frontal implementada deve permanecer em 180 graus.")
    return cv2.rotate(frame, cv2.ROTATE_180)


def save_forward_line_status(reading, timestamp, sequence):
    """Publica atomicamente apenas a leitura leve da câmera frontal."""

    try:
        timestamp = float(timestamp)
        if not math.isfinite(timestamp) or timestamp < 0.0:
            raise ValueError("forwardLineTimestamp inválido")
        if (
            not isinstance(sequence, int)
            or isinstance(sequence, bool)
            or sequence < 0
        ):
            raise ValueError("forwardLineSequence inválido")

        visible = bool(reading["forwardLineVisible"])
        position = reading["forwardLinePosition"]
        if visible:
            position = float(position)
            if not math.isfinite(position) or not -1.0 <= position <= 1.0:
                raise ValueError("forwardLinePosition inválido")
        elif position is not None:
            raise ValueError("forwardLinePosition deve ser None sem linha visível")

        confidence = float(reading["forwardLineConfidence"])
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise ValueError("forwardLineConfidence inválido")

        status = {
            "forwardLineVisible": visible,
            "forwardLinePosition": position,
            "forwardLineConfidence": confidence,
            "forwardLineSequence": sequence,
            "forwardLineTimestamp": timestamp,
            # Campos antigos permanecem nulos para não autorizar um consumidor
            # antigo a usar a frontal como controlador de motores.
            "forwardLineNormalLeftPower": None,
            "forwardLineNormalRightPower": None,
            "forwardPathVersion": 2,
            "forwardLinePresent": reading.get("forwardLinePresent", False),
            "forwardPathState": reading.get("forwardPathState", "UNCERTAIN"),
            "forwardPathConfidence": confidence,
            "forwardPathReferenceValid": reading.get("forwardPathReferenceValid", False),
            "forwardPathReferenceSequence": reading.get("forwardPathReferenceSequence", 0),
            "forwardPathReferenceTimestamp": reading.get("forwardPathReferenceTimestamp", 0.0),
            "forwardPathComponents": reading.get("forwardPathComponents", {}),
        }
        if status["forwardPathState"] not in ("PRESENT", "UNCERTAIN", "ABSENT"):
            raise ValueError("Estado frontal desconhecido")
        if status["forwardLinePresent"] != (status["forwardPathState"] == "PRESENT"):
            raise ValueError("Presença frontal inconsistente com seu estado")
        with open(
            TEMP_FORWARD_LINE_STATUS_PATH,
            "w",
            encoding="utf-8",
        ) as status_file:
            json.dump(status, status_file, allow_nan=False)
        os.replace(TEMP_FORWARD_LINE_STATUS_PATH, FORWARD_LINE_STATUS_PATH)
        return True
    except (KeyError, OSError, TypeError, ValueError) as error:
        print(
            f"Falha ao publicar leitura leve da câmera frontal: {error}",
            flush=True,
        )
        return False


def clear_forward_line_status():
    """Remove somente o IPC frontal quando não existe captura válida."""

    for path in (FORWARD_LINE_STATUS_PATH, TEMP_FORWARD_LINE_STATUS_PATH):
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass
        except OSError as error:
            print(f"Falha ao remover IPC frontal {path}: {error}", flush=True)


def draw_forward_assist_overlay(display_frame, reading):
    """Exibe somente a área analisada, o caminho escolhido e o estado atual."""

    x0, y0, x1, y1 = resolve_forward_assist_roi(display_frame.shape)
    right = x1 - 1
    bottom = y1 - 1
    state = reading.get("forwardPathState", "UNCERTAIN")
    state_color = {
        "PRESENT": (0, 210, 0),
        "UNCERTAIN": (0, 210, 255),
        "ABSENT": (0, 0, 230),
    }.get(state, (0, 210, 255))

    # A borda discreta informa exatamente qual parte do frame participa da
    # leitura frontal sem cobrir a pista com linhas auxiliares.
    cv2.rectangle(display_frame, (x0, y0), (right, bottom), (150, 150, 150), 1)

    selected_points = [
        (int(round(band["x"])), int(round(band["y"])))
        for band in reading.get("selectedBands", [])
    ]
    if len(selected_points) > 1:
        # Apenas o caminho realmente escolhido aparece no stream. Candidatos
        # rejeitados continuam disponíveis no processamento e na telemetria.
        cv2.polylines(
            display_frame,
            [np.asarray(selected_points, dtype=np.int32)],
            False,
            state_color,
            2,
            cv2.LINE_AA,
        )

    confidence = float(reading.get("forwardLineConfidence", 0.0))
    decision = reading.get("decision", "UNAVAILABLE")
    label = f"FRENTE {state}  {confidence:.2f}  |  {decision}"
    label_top = max(0, y0 - 28)
    label_bottom = max(22, y0 - 4)
    label_right = min(right, x0 + 370)
    # A faixa compacta mantém o estado legível sobre pisos claros sem ocupar
    # a região usada para inspecionar a fita.
    cv2.rectangle(
        display_frame,
        (x0, label_top),
        (label_right, label_bottom),
        (25, 25, 25),
        -1,
    )
    cv2.putText(
        display_frame,
        label,
        (x0 + 6, label_bottom - 6),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        state_color,
        1,
        cv2.LINE_AA,
    )


def publish_frame(jpeg):
    """Acorda somente clientes frontais com o JPEG mais recente."""

    global latest_jpeg, latest_jpeg_sequence
    with frame_condition:
        latest_jpeg = jpeg
        latest_jpeg_sequence += 1
        frame_condition.notify_all()


def clear_frame():
    """Encerra streams pendentes sem interromper o processamento da câmera."""

    global latest_jpeg
    with frame_condition:
        latest_jpeg = None
        frame_condition.notify_all()


def register_stream_client():
    """Registra um cliente para codificar JPEG somente quando necessário."""

    global active_stream_clients
    with frame_condition:
        active_stream_clients += 1


def unregister_stream_client():
    """Remove um cliente do stream sem permitir uma contagem negativa."""

    global active_stream_clients
    with frame_condition:
        active_stream_clients = max(0, active_stream_clients - 1)


def stream_has_clients():
    """Evita o custo do JPEG quando ninguém está observando a câmera frontal."""

    with frame_condition:
        return active_stream_clients > 0


def handle_signal(signum, frame):
    """Solicita encerramento limpo do servidor e da câmera."""

    del signum, frame
    global running
    running = False
    with frame_condition:
        frame_condition.notify_all()


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


class ForwardStreamHandler(BaseHTTPRequestHandler):
    def log_message(self, format_text, *args):
        del format_text, args

    def do_GET(self):
        if urlsplit(self.path).path != STREAM_PATH:
            self.send_response(404)
            self.end_headers()
            return
        if not stream_active:
            self.send_response(503)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write(b"Forward camera disabled")
            return

        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        last_sequence = -1
        register_stream_client()
        try:
            while running and stream_active:
                with frame_condition:
                    frame_condition.wait_for(
                        lambda: (
                            latest_jpeg is not None
                            and latest_jpeg_sequence != last_sequence
                        )
                        or not stream_active or not running,
                        timeout=1.0,
                    )
                    if latest_jpeg is None:
                        continue
                    jpeg = latest_jpeg
                    last_sequence = latest_jpeg_sequence
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(
                    f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii")
                )
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            return
        finally:
            unregister_stream_client()

    def do_HEAD(self):
        if self.path.split("?", 1)[0] != STREAM_PATH:
            self.send_response(404)
        elif stream_active:
            self.send_response(200)
            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame",
            )
        else:
            self.send_response(503)
        self.end_headers()


def start_stream_server():
    """Mantém a porta do stream disponível sem controlar a captura física."""

    server = ReusableThreadingHTTPServer(
        ("127.0.0.1", STREAM_PORT),
        ForwardStreamHandler,
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def open_forward_camera():
    """Abre exclusivamente a CAM1 usando o perfil frontal centralizado."""

    profile = camera_line_frame.CAMERA_PROFILES["forward"]
    camera_indices = camera_line_frame.configured_camera_indices()
    camera_infos = camera_line_frame.Picamera2.global_camera_info()
    assignments = camera_line_frame.resolve_camera_assignments(
        camera_infos,
        camera_indices,
    )
    camera_line_frame.log_camera_inventory(camera_infos, assignments)
    selected = camera_line_frame.require_camera_assignment(assignments, "forward")
    camera, camera_format, details = camera_line_frame.create_camera(
        profile,
        selected["index"],
    )
    camera.start()
    camera_line_frame.tune_camera_image(camera)
    metadata = camera.capture_metadata()
    details["scalerCrop"] = camera_line_frame.rectangle_values(
        metadata["ScalerCrop"]
    )
    return camera, camera_format, details


def close_forward_camera(camera):
    """Para e libera a CAM1 somente no encerramento ou após uma falha."""

    if camera is None:
        return
    try:
        camera.stop()
    except Exception as error:
        print(f"Câmera frontal não confirmou a parada: {error}", flush=True)
    camera.close()


def main():
    global stream_active

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    clear_forward_line_status()
    clear_rescue_zone_status()
    if camera_line_frame.Picamera2 is None:
        save_status(
            False,
            False,
            "error",
            error_message="Picamera2 indisponível.",
            processing_active=False,
        )
        return 1

    initial_enabled = environment_enabled()
    write_requested_enabled(initial_enabled)
    save_status(
        initial_enabled,
        False,
        "starting",
        processing_active=False,
    )
    stream_server = start_stream_server()
    camera = None
    camera_format = ""
    details = {}
    previous_time = 0.0
    smoothed_fps = 0.0
    last_status_time = 0.0
    next_retry_time = 0.0
    line_sequence = 0
    rescue_zone_sequence = 0
    rescue_zone_active = False
    rescue_zone_results = None
    rescue_zone_input = None
    rescue_zone_temporal_filter = RescueZoneTemporalFilter()
    path_tracker = ForwardPathTracker()
    rescue_zone_profiler = RescueZoneCycleProfiler(
        RESCUE_ZONE_PROFILE_ENABLED
    )

    silver_dataset_recorder = None
    if SilverDatasetRecorder is not None:
        try:
            silver_dataset_recorder = SilverDatasetRecorder("forward")
        except Exception as error:
            print(f"Coleta do dataset frontal indisponível: {error}", flush=True)

    try:
        while running:
            enabled = requested_enabled()
            if camera is None:
                stream_active = False
                clear_frame()
                current_time = time.monotonic()
                if current_time < next_retry_time:
                    time.sleep(IDLE_POLL_SECONDS)
                    continue
                save_status(
                    enabled,
                    False,
                    "starting",
                    processing_active=False,
                )
                try:
                    camera, camera_format, details = open_forward_camera()
                    previous_time = time.monotonic()
                    smoothed_fps = 0.0
                    last_status_time = 0.0
                    stream_active = bool(enabled)
                    print(
                        "Câmera frontal iniciada para processamento contínuo.",
                        flush=True,
                    )
                except Exception as error:
                    close_forward_camera(camera)
                    camera = None
                    stream_active = False
                    clear_frame()
                    next_retry_time = time.monotonic() + ERROR_RETRY_SECONDS
                    save_status(
                        enabled,
                        False,
                        "error",
                        error_message=str(error),
                        processing_active=False,
                    )
                    print(f"Câmera frontal não pôde ser iniciada: {error}", flush=True)
                    continue

            if not enabled and stream_active:
                stream_active = False
                clear_frame()
            elif enabled:
                stream_active = True

            profile_this_cycle = (
                rescue_zone_profiler.enabled and rescue_zone_active
            )
            profile_timings = {
                stage: 0.0 for stage in RescueZoneCycleProfiler.STAGES
            } if profile_this_cycle else None
            cycle_started = time.perf_counter() if profile_this_cycle else 0.0

            try:
                stage_started = time.perf_counter() if profile_this_cycle else 0.0
                frame = camera.capture_array("main")
                if profile_this_cycle:
                    profile_timings["captureMs"] = (
                        time.perf_counter() - stage_started
                    ) * 1000.0
                    stage_started = time.perf_counter()
                frame = orient_forward_frame(frame)
                if profile_this_cycle:
                    profile_timings["preparationMs"] = (
                        time.perf_counter() - stage_started
                    ) * 1000.0
                if silver_dataset_recorder is not None:
                    try:
                        silver_dataset_recorder.submit(frame)
                    except Exception as error:
                        print(f"Coleta do dataset frontal desativada após erro inesperado: {error}", flush=True)
                        silver_dataset_recorder = None
                line_timestamp = time.time()
                stage_started = time.perf_counter() if profile_this_cycle else 0.0
                bottom_status = read_json_snapshot(LINE_STATUS_PATH)
                try:
                    bottom_age = line_timestamp - float(bottom_status["lineTimestamp"])
                    bottom_fresh = 0 <= bottom_age <= GAP_VALIDATION_CONFIG["source_timeout"]
                except (KeyError, TypeError, ValueError):
                    bottom_fresh = False
                reading = process_forward_frame(
                    frame, camera_format,
                    bottom_status.get("bottomPathReference") if bottom_fresh else None,
                    path_tracker, line_timestamp,
                )
                reading["decision"] = bottom_status.get("gapValidationDecision", "UNAVAILABLE") if bottom_fresh else "UNAVAILABLE"
                reading["nearState"] = bottom_status.get("nearLineState", "UNKNOWN") if bottom_fresh else "UNKNOWN"
                reading["bottomFarState"] = (
                    "PRESENT" if bottom_status.get("bottomFarLinePresent") is True
                    else "ABSENT" if bottom_fresh else "UNKNOWN"
                )
                reading["source"] = bottom_status.get("lineControlSource", "UNAVAILABLE") if bottom_fresh else "UNAVAILABLE"
                if profile_this_cycle:
                    profile_timings["forwardAssistMs"] = (
                        time.perf_counter() - stage_started
                    ) * 1000.0
                line_sequence += 1
                stage_started = time.perf_counter() if profile_this_cycle else 0.0
                save_forward_line_status(reading, line_timestamp, line_sequence)
                if profile_this_cycle:
                    profile_timings["ipcPublishMs"] += (
                        time.perf_counter() - stage_started
                    ) * 1000.0

                rescue_zone_input = read_rescue_zone_detection_input()
                detection_requested = rescue_zone_input["enabled"]
                if detection_requested != rescue_zone_active:
                    rescue_zone_active = detection_requested
                    rescue_zone_sequence = 0
                    rescue_zone_results = None
                    rescue_zone_temporal_filter.reset()
                    if not rescue_zone_active:
                        rescue_zone_input = None
                        clear_rescue_zone_status()
                if rescue_zone_active:
                    stage_started = time.perf_counter() if profile_this_cycle else 0.0
                    detection_timings = {} if profile_this_cycle else None
                    rescue_zone_candidates = analyze_rescue_zones(
                        frame,
                        profile_timings=detection_timings,
                    )
                    rescue_zone_results = rescue_zone_temporal_filter.update(
                        rescue_zone_candidates
                    )
                    if profile_this_cycle:
                        detection_total_ms = (
                            time.perf_counter() - stage_started
                        ) * 1000.0
                        color_conversion_ms = detection_timings.get(
                            "colorConversionMs", 0.0
                        )
                        profile_timings["preparationMs"] += color_conversion_ms
                        profile_timings["rescueZoneDetectionMs"] = max(
                            0.0,
                            detection_total_ms - color_conversion_ms,
                        )
                    # O gate é relido depois do processamento para reduzir a
                    # janela entre STOP/E-Stop e a remoção do resultado anterior.
                    latest_rescue_zone_input = read_rescue_zone_detection_input()
                    if latest_rescue_zone_input["enabled"]:
                        rescue_zone_input = latest_rescue_zone_input
                        rescue_zone_sequence += 1
                        stage_started = (
                            time.perf_counter() if profile_this_cycle else 0.0
                        )
                        save_rescue_zone_status(
                            rescue_zone_results,
                            line_timestamp,
                            rescue_zone_sequence,
                            rescue_zone_input,
                        )
                        if profile_this_cycle:
                            profile_timings["ipcPublishMs"] += (
                                time.perf_counter() - stage_started
                            ) * 1000.0
                    else:
                        rescue_zone_active = False
                        rescue_zone_results = None
                        rescue_zone_input = None
                        rescue_zone_temporal_filter.reset()
                        clear_rescue_zone_status()
            except Exception as error:
                stream_active = False
                clear_frame()
                close_forward_camera(camera)
                camera = None
                path_tracker = ForwardPathTracker()
                rescue_zone_results = None
                rescue_zone_input = None
                rescue_zone_temporal_filter.reset()
                clear_rescue_zone_status()
                next_retry_time = time.monotonic() + ERROR_RETRY_SECONDS
                save_status(
                    enabled,
                    False,
                    "error",
                    error_message=str(error),
                    processing_active=False,
                )
                print(
                    "Captura ou processamento frontal falhou; "
                    f"nova tentativa será feita: {error}",
                    flush=True,
                )
                continue

            has_stream_client = enabled and stream_has_clients()
            if has_stream_client:
                stage_started = time.perf_counter() if profile_this_cycle else 0.0
                display_frame = frame.copy()
                if rescue_zone_active and rescue_zone_results is not None:
                    display_frame = draw_rescue_zone_overlay(
                        display_frame,
                        rescue_zone_results,
                        rescue_zone_input,
                    )
                else:
                    draw_forward_assist_overlay(display_frame, reading)
                if profile_this_cycle:
                    profile_timings["overlayMs"] = (
                        time.perf_counter() - stage_started
                    ) * 1000.0
                    stage_started = time.perf_counter()
                jpeg = camera_line_frame.encode_frame(display_frame)
                if profile_this_cycle:
                    profile_timings["jpegEncodeMs"] = (
                        time.perf_counter() - stage_started
                    ) * 1000.0
                if jpeg is not None:
                    publish_frame(jpeg)

            current_time = time.monotonic()
            elapsed = current_time - previous_time
            previous_time = current_time
            if elapsed > 0.0:
                current_fps = 1.0 / elapsed
                smoothed_fps = (
                    smoothed_fps * 0.90 + current_fps * 0.10
                    if smoothed_fps > 0.0 else current_fps
                )
            if current_time - last_status_time >= 1.0 / STATUS_FPS:
                stage_started = time.perf_counter() if profile_this_cycle else 0.0
                save_status(
                    enabled,
                    enabled,
                    "online" if enabled else "disabled",
                    smoothed_fps,
                    camera_format,
                    details,
                    processing_active=True,
                )
                if profile_this_cycle:
                    profile_timings["ipcPublishMs"] += (
                        time.perf_counter() - stage_started
                    ) * 1000.0
                last_status_time = current_time
            if profile_this_cycle and rescue_zone_active:
                profile_timings["totalCycleMs"] = (
                    time.perf_counter() - cycle_started
                ) * 1000.0
                accounted_stages = (
                    "captureMs",
                    "preparationMs",
                    "forwardAssistMs",
                    "rescueZoneDetectionMs",
                    "overlayMs",
                    "jpegEncodeMs",
                    "ipcPublishMs",
                )
                profile_timings["otherMs"] = max(
                    0.0,
                    profile_timings["totalCycleMs"]
                    - sum(profile_timings[stage] for stage in accounted_stages),
                )
                bucket = (
                    "withStreamClient"
                    if has_stream_client
                    else "withoutStreamClient"
                )
                metadata = None
                if rescue_zone_profiler.metadata_is_due(bucket):
                    try:
                        metadata = camera.capture_metadata()
                    except Exception:
                        metadata = None
                rescue_zone_profiler.record(
                    has_stream_client,
                    profile_timings,
                    metadata,
                )
    finally:
        stream_active = False
        close_forward_camera(camera)
        clear_frame()
        clear_forward_line_status()
        clear_rescue_zone_status()
        save_status(
            False,
            False,
            "stopped",
            processing_active=False,
        )
        stream_server.shutdown()
        stream_server.server_close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
