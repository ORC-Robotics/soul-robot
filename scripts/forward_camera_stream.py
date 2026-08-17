"""Mantém a câmera frontal desligada até receber uma ativação explícita."""

import json
import os
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

import camera_line_frame


CONTROL_PATH = "/dev/shm/obr_forward_camera_enabled"
TEMP_CONTROL_PATH = "/dev/shm/obr_forward_camera_enabled.tmp"
STATUS_PATH = "/tmp/obr_forward_camera_status.json"
TEMP_STATUS_PATH = "/tmp/obr_forward_camera_status.tmp.json"
STREAM_PORT = 8091
STREAM_PATH = "/stream.mjpg"
STATUS_FPS = 5
IDLE_POLL_SECONDS = 0.10
ERROR_RETRY_SECONDS = 1.0

running = True
camera_active = False
latest_jpeg = None
latest_jpeg_sequence = 0
frame_condition = threading.Condition()


def environment_enabled():
    """Lê o estado inicial; o padrão seguro mantém a câmera fechada."""

    value = os.environ.get("OBR_FORWARD_CAMERA_ENABLED", "0").strip().lower()
    if value in ("1", "true", "yes", "on"):
        return True
    if value in ("0", "false", "no", "off", ""):
        return False
    raise ValueError(
        "OBR_FORWARD_CAMERA_ENABLED deve ser 0/1, false/true, no/yes ou off/on."
    )


def write_requested_enabled(enabled):
    """Publica atomicamente o estado solicitado para o gerenciador frontal."""

    with open(TEMP_CONTROL_PATH, "w", encoding="utf-8") as control_file:
        control_file.write("1\n" if enabled else "0\n")
    os.replace(TEMP_CONTROL_PATH, CONTROL_PATH)


def requested_enabled():
    """Retorna falso quando o arquivo de controle não existe ou é inválido."""

    try:
        with open(CONTROL_PATH, "r", encoding="utf-8") as control_file:
            return control_file.read().strip() == "1"
    except OSError:
        return False


def save_status(enabled, active, state, fps=0.0, camera_format="", details=None,
                error_message=""):
    """Expõe saúde e configuração sem publicar qualquer dado do segue-faixa."""

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
        "transform": details.get("transform", "hvflip"),
        "targetCameraFps": profile["target_fps"],
        "streamPort": STREAM_PORT,
        "streamPath": STREAM_PATH,
        "error": error_message,
        "timestamp": time.time(),
    }
    with open(TEMP_STATUS_PATH, "w", encoding="utf-8") as status_file:
        json.dump(status, status_file, allow_nan=False)
    os.replace(TEMP_STATUS_PATH, STATUS_PATH)


def publish_frame(jpeg):
    """Acorda somente clientes frontais com o JPEG mais recente."""

    global latest_jpeg, latest_jpeg_sequence
    with frame_condition:
        latest_jpeg = jpeg
        latest_jpeg_sequence += 1
        frame_condition.notify_all()


def clear_frame():
    """Encerra streams pendentes quando a câmera física é fechada."""

    global latest_jpeg
    with frame_condition:
        latest_jpeg = None
        frame_condition.notify_all()


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
        if not camera_active:
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
        try:
            while running and camera_active:
                with frame_condition:
                    frame_condition.wait_for(
                        lambda: latest_jpeg_sequence != last_sequence
                        or not camera_active or not running,
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

    def do_HEAD(self):
        if self.path.split("?", 1)[0] != STREAM_PATH:
            self.send_response(404)
        elif camera_active:
            self.send_response(200)
            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame",
            )
        else:
            self.send_response(503)
        self.end_headers()


def start_stream_server():
    """Mantém a porta de controle disponível mesmo com a câmera fechada."""

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
    """Para a captura e libera a CAM1 para reduzir o consumo quando desativada."""

    if camera is None:
        return
    try:
        camera.stop()
    except Exception as error:
        print(f"Câmera frontal não confirmou a parada: {error}", flush=True)
    camera.close()


def main():
    global camera_active

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    if camera_line_frame.Picamera2 is None:
        save_status(False, False, "error", error_message="Picamera2 indisponível.")
        return 1

    initial_enabled = environment_enabled()
    write_requested_enabled(initial_enabled)
    save_status(initial_enabled, False, "starting" if initial_enabled else "disabled")
    stream_server = start_stream_server()
    camera = None
    camera_format = ""
    details = {}
    previous_time = 0.0
    smoothed_fps = 0.0
    last_status_time = 0.0
    next_retry_time = 0.0

    try:
        while running:
            enabled = requested_enabled()
            if not enabled:
                state_changed = False
                if camera is not None:
                    camera_active = False
                    clear_frame()
                    close_forward_camera(camera)
                    camera = None
                    camera_format = ""
                    details = {}
                    state_changed = True
                    print("Câmera frontal desativada e liberada.", flush=True)
                current_time = time.monotonic()
                if (state_changed or
                        current_time - last_status_time >= 1.0 / STATUS_FPS):
                    save_status(False, False, "disabled")
                    last_status_time = current_time
                time.sleep(IDLE_POLL_SECONDS)
                continue

            if camera is None:
                current_time = time.monotonic()
                if current_time < next_retry_time:
                    time.sleep(IDLE_POLL_SECONDS)
                    continue
                save_status(True, False, "starting")
                try:
                    camera, camera_format, details = open_forward_camera()
                    previous_time = time.monotonic()
                    smoothed_fps = 0.0
                    last_status_time = 0.0
                    camera_active = True
                    print("Câmera frontal ativada.", flush=True)
                except Exception as error:
                    close_forward_camera(camera)
                    camera = None
                    camera_active = False
                    clear_frame()
                    next_retry_time = time.monotonic() + ERROR_RETRY_SECONDS
                    save_status(True, False, "error", error_message=str(error))
                    print(f"Câmera frontal não pôde ser ativada: {error}", flush=True)
                    continue

            try:
                frame = camera.capture_array("main")
                jpeg = camera_line_frame.encode_frame(frame)
            except Exception as error:
                camera_active = False
                clear_frame()
                close_forward_camera(camera)
                camera = None
                next_retry_time = time.monotonic() + ERROR_RETRY_SECONDS
                save_status(True, False, "error", error_message=str(error))
                print(f"Captura frontal falhou; nova tentativa será feita: {error}",
                      flush=True)
                continue
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
                save_status(True, True, "online", smoothed_fps,
                            camera_format, details)
                last_status_time = current_time
    finally:
        camera_active = False
        close_forward_camera(camera)
        clear_frame()
        save_status(False, False, "stopped")
        stream_server.shutdown()
        stream_server.server_close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
