"""Mantém a câmera frontal desligada até receber uma ativação explícita."""

import json
import os
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

import cv2  # type: ignore

import camera_line_frame
from ball_vision.ball_detector import BallDetector
from ball_vision.distance_calibration import DistanceCalibration
from ball_vision.ball_tracker import BallTracker
from ball_vision.main import (
    analyze_frame,
    build_esp32_payload,
    draw_overlay,
)


CONTROL_PATH = "/dev/shm/obr_forward_camera_enabled"
TEMP_CONTROL_PATH = "/dev/shm/obr_forward_camera_enabled.tmp"
BALL_DETECTION_CONTROL_PATH = "/dev/shm/obr_forward_ball_detection_enabled"
TEMP_BALL_DETECTION_CONTROL_PATH = (
    "/dev/shm/obr_forward_ball_detection_enabled.tmp"
)
STATUS_PATH = "/tmp/obr_forward_camera_status.json"
TEMP_STATUS_PATH = "/tmp/obr_forward_camera_status.tmp.json"
BALL_STATUS_PATH = "/dev/shm/obr_forward_ball_status.json"
TEMP_BALL_STATUS_PATH = "/dev/shm/obr_forward_ball_status.tmp.json"
STREAM_PORT = 8091
STREAM_PATH = "/stream.mjpg"
STREAM_FPS = 12
STREAM_JPEG_QUALITY = 70
OPENCV_THREAD_COUNT = 2
STATUS_FPS = 5
IDLE_POLL_SECONDS = 0.10
ERROR_RETRY_SECONDS = 1.0

running = True
camera_active = False
latest_jpeg = None
latest_jpeg_sequence = 0
active_stream_clients = 0
frame_condition = threading.Condition()
ball_detector = BallDetector()
distance_calibration = DistanceCalibration()
ball_tracker = BallTracker()


def empty_ball_status(processing_ms=0.0, detection_enabled=True):
    """Remove qualquer detecção antiga quando o frame atual não tem bola."""

    return {
        "ballDetectionEnabled": bool(detection_enabled),
        "ballDetected": False,
        "ballType": "",
        "ballCenterX": None,
        "ballCenterY": None,
        "ballRadiusPixels": None,
        "ballDiameterPixels": None,
        "ballDistanceCm": None,
        "ballDistanceExtrapolated": False,
        "ballAngleDegrees": None,
        "ballTxDegrees": None,
        "ballPosition": "nenhuma",
        "ballCircularity": None,
        "ballTopClipped": False,
        "ballDetectionMethod": "",
        "ballProcessingMs": round(float(processing_ms), 2),
        "ballPayload": None,
    }


def build_ball_status(observation, processing_ms):
    """Converte a observação em valores simples consumidos pelo dashboard."""

    if observation is None:
        return empty_ball_status(processing_ms)
    candidate = observation.candidate
    return {
        "ballDetectionEnabled": True,
        "ballDetected": True,
        "ballType": candidate.ball_type,
        "ballCenterX": round(float(candidate.center_x), 2),
        "ballCenterY": round(float(candidate.center_y), 2),
        "ballRadiusPixels": round(float(candidate.radius_pixels), 2),
        "ballDiameterPixels": round(float(candidate.diameter_pixels), 2),
        "ballDistanceCm": round(float(observation.distance.distance_cm), 2),
        "ballDistanceExtrapolated": bool(observation.distance.extrapolated),
        "ballAngleDegrees": round(float(observation.angle_degrees), 2),
        "ballTxDegrees": round(float(observation.angle_degrees), 2),
        "ballPosition": observation.position,
        "ballCircularity": round(float(candidate.circularity), 3),
        "ballTopClipped": bool(candidate.top_clipped),
        "ballDetectionMethod": candidate.detection_method,
        "ballProcessingMs": round(float(processing_ms), 2),
        "ballPayload": build_esp32_payload(
            candidate.ball_type,
            observation.distance.distance_cm,
            observation.angle_degrees,
        ),
    }


def save_ball_control_status(active, ball_status=None):
    """Publica em RAM somente os campos usados pelo controle de alinhamento."""

    current_ball = ball_status or empty_ball_status()
    status = {
        "active": bool(active),
        "timestamp": time.time(),
        "ballDetected": bool(current_ball["ballDetected"]),
        "ballType": current_ball["ballType"],
        "ballTxDegrees": current_ball["ballTxDegrees"],
        "ballDistanceCm": current_ball["ballDistanceCm"],
        "ballRadiusPixels": current_ball["ballRadiusPixels"],
    }
    with open(TEMP_BALL_STATUS_PATH, "w", encoding="utf-8") as status_file:
        json.dump(status, status_file, allow_nan=False)
    os.replace(TEMP_BALL_STATUS_PATH, BALL_STATUS_PATH)


def analyze_ball_frame(frame):
    """Analisa um frame sem gastar CPU com desenho ou JPEG."""

    processing_started = time.perf_counter()
    observation, candidates = analyze_frame(
        frame,
        ball_detector,
        distance_calibration,
        tracker=ball_tracker,
    )
    processing_ms = (time.perf_counter() - processing_started) * 1000.0
    return (
        observation,
        candidates,
        build_ball_status(observation, processing_ms),
    )


def analyze_requested_ball_frame(frame, detection_enabled):
    """Executa a visão somente quando a missão de alinhamento a autoriza."""

    if not detection_enabled:
        return None, [], empty_ball_status(detection_enabled=False)
    return analyze_ball_frame(frame)


def process_ball_frame(frame):
    """Analisa e anota um frame para testes ou visualização local."""

    observation, candidates, ball_status = analyze_ball_frame(frame)
    display_frame = draw_overlay(frame, observation, candidates)
    return display_frame, ball_status


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


def write_requested_ball_detection_enabled(enabled):
    """Define se a missão atual autoriza executar os detectores de bolas."""

    with open(
        TEMP_BALL_DETECTION_CONTROL_PATH,
        "w",
        encoding="utf-8",
    ) as control_file:
        control_file.write("1\n" if enabled else "0\n")
    os.replace(
        TEMP_BALL_DETECTION_CONTROL_PATH,
        BALL_DETECTION_CONTROL_PATH,
    )


def requested_ball_detection_enabled():
    """Retorna falso fora da missão de alinhamento ou com IPC inválido."""

    try:
        with open(
            BALL_DETECTION_CONTROL_PATH,
            "r",
            encoding="utf-8",
        ) as control_file:
            return control_file.read().strip() == "1"
    except OSError:
        return False


def save_status(enabled, active, state, fps=0.0, camera_format="", details=None,
                error_message="", ball_status=None):
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
        "streamFps": STREAM_FPS,
        "jpegQuality": STREAM_JPEG_QUALITY,
        "opencvThreads": OPENCV_THREAD_COUNT,
        "silverProcessingScale": (
            ball_detector.silver_detector.config.processing_scale
        ),
        "error": error_message,
        "timestamp": time.time(),
    }
    status.update(
        ball_status or empty_ball_status(
            detection_enabled=requested_ball_detection_enabled()
        )
    )
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


def register_stream_client():
    """Habilita desenho e JPEG somente quando alguém acompanha o dashboard."""

    global active_stream_clients
    with frame_condition:
        active_stream_clients += 1


def unregister_stream_client():
    """Remove um cliente sem permitir que o contador fique negativo."""

    global active_stream_clients
    with frame_condition:
        active_stream_clients = max(0, active_stream_clients - 1)


def stream_frame_is_due(now, last_stream_time):
    """Limita o vídeo de debug sem reduzir a frequência do tx de controle."""

    with frame_condition:
        has_clients = active_stream_clients > 0
    return has_clients and now - last_stream_time >= 1.0 / STREAM_FPS


def encode_stream_frame(frame):
    """Codifica o stream frontal com qualidade suficiente para depuração."""

    parameters = [int(cv2.IMWRITE_JPEG_QUALITY), STREAM_JPEG_QUALITY]
    ok, encoded = cv2.imencode(".jpg", frame, parameters)
    return encoded.tobytes() if ok else None


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
        register_stream_client()
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
        finally:
            unregister_stream_client()

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

    # Limitar o paralelismo evita que uma única câmera ocupe todos os núcleos
    # da Raspberry Pi e eleve desnecessariamente a temperatura do processador.
    cv2.setNumThreads(OPENCV_THREAD_COUNT)
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    if camera_line_frame.Picamera2 is None:
        save_status(False, False, "error", error_message="Picamera2 indisponível.")
        save_ball_control_status(False)
        return 1

    initial_enabled = environment_enabled()
    write_requested_enabled(initial_enabled)
    save_status(initial_enabled, False, "starting" if initial_enabled else "disabled")
    save_ball_control_status(False)
    stream_server = start_stream_server()
    camera = None
    camera_format = ""
    details = {}
    ball_detection_active = False
    ball_status = empty_ball_status(detection_enabled=False)
    previous_time = 0.0
    smoothed_fps = 0.0
    last_status_time = 0.0
    last_stream_time = 0.0
    next_retry_time = 0.0

    try:
        while running:
            enabled = requested_enabled()
            detection_requested = requested_ball_detection_enabled()
            if not enabled:
                state_changed = False
                if camera is not None:
                    camera_active = False
                    clear_frame()
                    close_forward_camera(camera)
                    camera = None
                    camera_format = ""
                    details = {}
                    ball_detection_active = False
                    ball_status = empty_ball_status(
                        detection_enabled=detection_requested
                    )
                    ball_tracker.reset()
                    state_changed = True
                    print("Câmera frontal desativada e liberada.", flush=True)
                current_time = time.monotonic()
                if (state_changed or
                        current_time - last_status_time >= 1.0 / STATUS_FPS):
                    save_status(
                        False,
                        False,
                        "disabled",
                        ball_status=ball_status,
                    )
                    save_ball_control_status(False, ball_status)
                    last_status_time = current_time
                time.sleep(IDLE_POLL_SECONDS)
                continue

            if camera is None:
                current_time = time.monotonic()
                if current_time < next_retry_time:
                    time.sleep(IDLE_POLL_SECONDS)
                    continue
                save_status(True, False, "starting")
                save_ball_control_status(False)
                try:
                    camera, camera_format, details = open_forward_camera()
                    previous_time = time.monotonic()
                    smoothed_fps = 0.0
                    last_status_time = 0.0
                    ball_detection_active = False
                    ball_status = empty_ball_status(
                        detection_enabled=detection_requested
                    )
                    ball_tracker.reset()
                    camera_active = True
                    print("Câmera frontal ativada.", flush=True)
                except Exception as error:
                    close_forward_camera(camera)
                    camera = None
                    camera_active = False
                    clear_frame()
                    next_retry_time = time.monotonic() + ERROR_RETRY_SECONDS
                    save_status(True, False, "error", error_message=str(error))
                    save_ball_control_status(False)
                    print(f"Câmera frontal não pôde ser ativada: {error}", flush=True)
                    continue

            try:
                frame = camera.capture_array("main")
                detection_requested = requested_ball_detection_enabled()
                if detection_requested != ball_detection_active:
                    ball_tracker.reset()
                    ball_detection_active = detection_requested

                # Fora do alinhamento, mantém somente captura e stream cru.
                # Nenhum HSV, Hough, payload ou tx antigo pode ser publicado.
                observation, candidates, ball_status = (
                    analyze_requested_ball_frame(frame, ball_detection_active)
                )

                # O IPC rápido só fica ativo quando a missão autoriza detecção.
                save_ball_control_status(ball_detection_active, ball_status)
                current_time = time.monotonic()
                jpeg = None
                if stream_frame_is_due(current_time, last_stream_time):
                    display_frame = (
                        draw_overlay(frame, observation, candidates)
                        if ball_detection_active else frame
                    )
                    jpeg = encode_stream_frame(display_frame)
                    last_stream_time = current_time
            except Exception as error:
                camera_active = False
                clear_frame()
                close_forward_camera(camera)
                camera = None
                ball_status = empty_ball_status(
                    detection_enabled=detection_requested
                )
                ball_tracker.reset()
                next_retry_time = time.monotonic() + ERROR_RETRY_SECONDS
                save_status(True, False, "error", error_message=str(error))
                save_ball_control_status(False)
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
                save_status(
                    True,
                    True,
                    "online",
                    smoothed_fps,
                    camera_format,
                    details,
                    ball_status=ball_status,
                )
                last_status_time = current_time
    finally:
        camera_active = False
        close_forward_camera(camera)
        clear_frame()
        save_status(
            False,
            False,
            "stopped",
            ball_status=empty_ball_status(detection_enabled=False),
        )
        save_ball_control_status(False)
        write_requested_ball_detection_enabled(False)
        stream_server.shutdown()
        stream_server.server_close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
