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


CONTROL_PATH = "/dev/shm/obr_forward_camera_enabled"
TEMP_CONTROL_PATH = "/dev/shm/obr_forward_camera_enabled.tmp"
STATUS_PATH = "/tmp/obr_forward_camera_status.json"
TEMP_STATUS_PATH = "/tmp/obr_forward_camera_status.tmp.json"
FORWARD_LINE_STATUS_PATH = "/dev/shm/obr_forward_line_status.json"
TEMP_FORWARD_LINE_STATUS_PATH = "/dev/shm/obr_forward_line_status.tmp.json"
STREAM_PORT = 8091
STREAM_PATH = "/stream.mjpg"
STATUS_FPS = 5
IDLE_POLL_SECONDS = 0.10
ERROR_RETRY_SECONDS = 1.0

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


def calculate_forward_line_assist(filtered_line_mask):
    """Calcula uma posição horizontal leve sem usar sensores virtuais da CAM0."""

    if filtered_line_mask.ndim != 2:
        raise ValueError("A máscara preta frontal deve possuir um único canal.")

    x0, y0, x1, y1 = resolve_forward_assist_roi(filtered_line_mask.shape)
    roi_mask = filtered_line_mask[y0:y1, x0:x1]
    label_count, _labels, stats, centroids = cv2.connectedComponentsWithStats(
        roi_mask,
        connectivity=8,
    )

    valid_labels = []
    valid_pixel_count = 0
    weighted_centroid_x = 0.0
    for label in range(1, label_count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < FORWARD_ASSIST_MIN_COMPONENT_AREA_PX:
            continue
        valid_labels.append(label)
        valid_pixel_count += area
        weighted_centroid_x += float(centroids[label][0]) * float(area)

    if not valid_labels or valid_pixel_count <= 0:
        return {
            "forwardLineVisible": False,
            "forwardLinePosition": None,
            "forwardLineConfidence": 0.0,
        }

    centroid_x = weighted_centroid_x / float(valid_pixel_count)
    roi_width = x1 - x0
    if roi_width <= 1:
        normalized_position = 0.0
    else:
        normalized_position = 2.0 * centroid_x / float(roi_width - 1) - 1.0
    normalized_position = max(-1.0, min(1.0, normalized_position))

    # A confiança é a fração da ROI ocupada somente pelos componentes válidos.
    # Ela não cria decisão de movimento e pode ser recalibrada depois com dados reais.
    confidence = float(valid_pixel_count) / float(roi_mask.size)
    confidence = max(0.0, min(1.0, confidence))
    return {
        "forwardLineVisible": True,
        "forwardLinePosition": normalized_position,
        "forwardLineConfidence": confidence,
    }


def process_forward_frame(frame, camera_format):
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
    return calculate_forward_line_assist(full_filtered_mask)


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

        forward_error = (
            max(-1.0, min(1.0, position))
            if visible
            else None
        )
        normal_command = (
            camera_line_frame.map_normal_steering_error(forward_error)
            if visible
            else None
        )
        if visible and normal_command is None:
            raise ValueError("mapper NORMAL não aceitou a posição frontal")

        confidence = float(reading["forwardLineConfidence"])
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise ValueError("forwardLineConfidence inválido")

        status = {
            "forwardLineVisible": visible,
            "forwardLinePosition": position,
            "forwardLineConfidence": confidence,
            "forwardLineSequence": sequence,
            "forwardLineTimestamp": timestamp,
            "forwardLineNormalLeftPower": (
                normal_command["left_power"]
                if normal_command is not None
                else None
            ),
            "forwardLineNormalRightPower": (
                normal_command["right_power"]
                if normal_command is not None
                else None
            ),
        }
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
    """Desenha somente a ROI, o centro e a posição produzida pelo assistente."""

    x0, y0, x1, y1 = resolve_forward_assist_roi(display_frame.shape)
    right = x1 - 1
    bottom = y1 - 1
    center_x = int(round((x0 + right) / 2.0))
    cv2.rectangle(display_frame, (x0, y0), (right, bottom), (0, 255, 255), 1)
    cv2.line(
        display_frame,
        (center_x, y0),
        (center_x, bottom),
        (255, 255, 0),
        1,
        cv2.LINE_AA,
    )

    position = reading["forwardLinePosition"]
    if reading["forwardLineVisible"] and position is not None:
        position_x = int(round(
            x0 + (float(position) + 1.0) * 0.5 * float(right - x0)
        ))
        cv2.line(
            display_frame,
            (position_x, y0),
            (position_x, bottom),
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        position_text = f"{float(position):+.3f}"
    else:
        position_text = "--"

    confidence = float(reading["forwardLineConfidence"])
    text_y = y0 - 7 if y0 >= 20 else min(bottom, y0 + 16)
    cv2.putText(
        display_frame,
        f"FORWARD POS {position_text} CONF {confidence:.3f}",
        (x0 + 4, text_y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (0, 255, 255),
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

            try:
                frame = camera.capture_array("main")
                frame = orient_forward_frame(frame)
                reading = process_forward_frame(frame, camera_format)
                line_timestamp = time.time()
                line_sequence += 1
                save_forward_line_status(reading, line_timestamp, line_sequence)
            except Exception as error:
                stream_active = False
                clear_frame()
                close_forward_camera(camera)
                camera = None
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

            if enabled and stream_has_clients():
                display_frame = frame.copy()
                draw_forward_assist_overlay(display_frame, reading)
                jpeg = camera_line_frame.encode_frame(display_frame)
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
                save_status(
                    enabled,
                    enabled,
                    "online" if enabled else "disabled",
                    smoothed_fps,
                    camera_format,
                    details,
                    processing_active=True,
                )
                last_status_time = current_time
    finally:
        stream_active = False
        close_forward_camera(camera)
        clear_frame()
        clear_forward_line_status()
        save_status(False, False, "stopped", processing_active=False)
        stream_server.shutdown()
        stream_server.server_close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
