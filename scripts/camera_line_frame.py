"""Publica a imagem da câmera frontal no dashboard.

A segmentação experimental destaca a linha preta apenas na imagem de debug.
Ela não calcula comandos nem interfere no controle do robô.
"""

import json
import os
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

try:
    from libcamera import Transform  # type: ignore[import]
    from picamera2 import Picamera2  # type: ignore[import]
except ImportError:
    Transform = None
    Picamera2 = None


FRAME_PATH = "/tmp/obr_camera_frame.jpg"
TEMP_FRAME_PATH = "/tmp/obr_camera_frame.tmp.jpg"
STATUS_PATH = "/tmp/obr_camera_status.json"
TEMP_STATUS_PATH = "/tmp/obr_camera_status.tmp.json"

LIGHT_PIN_BOARD = 40
FRAME_WIDTH = 960
FRAME_HEIGHT = 540
TARGET_CAMERA_FPS = 30
MJPEG_STREAM_PORT = 8090
MJPEG_STREAM_PATH = "/stream.mjpg"
MJPEG_STREAM_FPS = 30
SNAPSHOT_FRAME_FPS = 2
STATUS_FPS = 5
JPEG_QUALITY = 82
CAMERA_PIXEL_FORMATS = ("RGB888",)

# A câmera está instalada fisicamente de cabeça para baixo. A rotação no
# pipeline evita processar novamente cada frame antes de enviá-lo ao dashboard.
CAMERA_ROTATION_DEGREES = 180

# Ajustes básicos de imagem. Eles afetam somente a visualização e não geram
# qualquer decisão de movimento.
CAMERA_SHARPNESS = 1.2
CAMERA_CONTRAST = 1.05
CAMERA_SATURATION = 1.0
CAMERA_EXPOSURE_VALUE = 0.4

# A segmentação experimental usa somente os 25% inferiores do frame.
# Alterar este valor muda apenas a região destacada no vídeo de debug.
LINE_ROI_START_RATIO = 0.75
# Pixels abaixo deste valor são considerados parte da linha preta.
# Este threshold é fixo e não altera a exposição configurada da câmera.
LINE_THRESHOLD = 100
# A abertura 3x3 remove pequenos ruídos isolados do piso.
OPEN_KERNEL_SIZE = 3
# O fechamento 5x5 preenche pequenos buracos dentro da faixa preta.
CLOSE_KERNEL_SIZE = 5

running = True
latest_jpeg = None
latest_jpeg_sequence = 0
frame_condition = threading.Condition()


def handle_signal(signum, frame):
    """Encerra o stream de forma limpa quando o serviço recebe um sinal."""

    del signum, frame
    global running
    running = False
    with frame_condition:
        frame_condition.notify_all()


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


class CameraStreamHandler(BaseHTTPRequestHandler):
    def log_message(self, format_text, *args):
        del format_text, args

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path != MJPEG_STREAM_PATH:
            self.send_response(404)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write(b"Not found")
            return

        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        last_sequence = -1
        try:
            while running:
                with frame_condition:
                    frame_condition.wait_for(
                        lambda: latest_jpeg_sequence != last_sequence or not running,
                        timeout=1.0,
                    )
                    if latest_jpeg is None:
                        continue
                    jpeg = latest_jpeg
                    last_sequence = latest_jpeg_sequence

                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            return

    def do_HEAD(self):
        path = self.path.split("?", 1)[0]
        if path != MJPEG_STREAM_PATH:
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()


def start_stream_server():
    """Inicia o servidor local usado pelo proxy do dashboard."""

    server = ReusableThreadingHTTPServer(("127.0.0.1", MJPEG_STREAM_PORT), CameraStreamHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def create_camera():
    """Configura a Camera V2 com fallback simples de formato e FPS."""

    picam2 = Picamera2()
    frame_duration_us = int(1_000_000 / TARGET_CAMERA_FPS)
    camera_transform = Transform(hflip=True, vflip=True)

    for pixel_format in CAMERA_PIXEL_FORMATS:
        try:
            camera_config = picam2.create_video_configuration(
                main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": pixel_format},
                controls={"FrameDurationLimits": (frame_duration_us, frame_duration_us)},
                transform=camera_transform,
                buffer_count=4,
            )
            picam2.configure(camera_config)
            print(
                f"Câmera configurada em {pixel_format} com alvo de "
                f"{TARGET_CAMERA_FPS} FPS.",
                flush=True,
            )
            return picam2, pixel_format
        except Exception as error:
            print(f"Configuração {pixel_format} falhou: {error}", flush=True)

    camera_config = picam2.create_still_configuration(
        {"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"},
        transform=camera_transform,
    )
    picam2.configure(camera_config)
    print("Câmera configurada em modo still como fallback.", flush=True)
    return picam2, "RGB888"


def tune_camera_image(picam2):
    """Aplica somente controles visuais suportados pela câmera instalada."""

    controls = {
        "AeEnable": True,
        "AwbEnable": True,
        "ExposureValue": CAMERA_EXPOSURE_VALUE,
        "Sharpness": CAMERA_SHARPNESS,
        "Contrast": CAMERA_CONTRAST,
        "Saturation": CAMERA_SATURATION,
    }
    for name, value in controls.items():
        try:
            picam2.set_controls({name: value})
        except Exception as error:
            print(f"Controle de câmera {name} não foi aplicado: {error}", flush=True)


def create_filtered_line_mask(frame):
    """Segmenta a linha preta na parte inferior sem gerar decisões de controle."""

    frame_height = frame.shape[0]
    roi_start_y = int(frame_height * LINE_ROI_START_RATIO)
    line_roi = frame[roi_start_y:frame_height, :]
    gray_roi = cv2.cvtColor(line_roi, cv2.COLOR_BGR2GRAY)

    _, binary_mask = cv2.threshold(
        gray_roi,
        LINE_THRESHOLD,
        255,
        cv2.THRESH_BINARY_INV,
    )

    open_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (OPEN_KERNEL_SIZE, OPEN_KERNEL_SIZE),
    )
    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (CLOSE_KERNEL_SIZE, CLOSE_KERNEL_SIZE),
    )
    filtered_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, open_kernel)
    filtered_mask = cv2.morphologyEx(filtered_mask, cv2.MORPH_CLOSE, close_kernel)
    return filtered_mask, roi_start_y


def encode_frame(frame):
    """Converte para JPEG o frame com a visualização experimental."""

    parameters = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
    ok, encoded = cv2.imencode(".jpg", frame, parameters)
    return encoded.tobytes() if ok else None


def publish_stream_frame(jpeg):
    global latest_jpeg, latest_jpeg_sequence
    with frame_condition:
        latest_jpeg = jpeg
        latest_jpeg_sequence += 1
        frame_condition.notify_all()


def save_frame(jpeg):
    with open(TEMP_FRAME_PATH, "wb") as frame_file:
        frame_file.write(jpeg)
    os.replace(TEMP_FRAME_PATH, FRAME_PATH)


def save_status(
    fps,
    camera_format="",
    active=True,
    error_message="",
):
    """Publica somente saúde e características do stream da câmera."""

    status = {
        "fps": round(fps, 2),
        "active": active,
        "width": FRAME_WIDTH,
        "height": FRAME_HEIGHT,
        "jpegQuality": JPEG_QUALITY,
        "targetCameraFps": TARGET_CAMERA_FPS,
        "cameraFormat": camera_format,
        "rotationDegrees": CAMERA_ROTATION_DEGREES,
        "streamPort": MJPEG_STREAM_PORT,
        "streamPath": MJPEG_STREAM_PATH,
        "streamFps": MJPEG_STREAM_FPS,
        "error": error_message,
        "timestamp": time.time(),
    }
    with open(TEMP_STATUS_PATH, "w", encoding="utf-8") as status_file:
        json.dump(status, status_file)
    os.replace(TEMP_STATUS_PATH, STATUS_PATH)


def main():
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    if GPIO is None or Picamera2 is None or Transform is None:
        error_message = "Dependências GPIO, libcamera ou Picamera2 não encontradas."
        print(error_message, flush=True)
        save_status(0.0, active=False, error_message=error_message)
        return 1

    save_status(0.0)
    stream_server = None
    picam2 = None
    camera_started = False
    light_ready = False

    try:
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(LIGHT_PIN_BOARD, GPIO.OUT)
        GPIO.output(LIGHT_PIN_BOARD, GPIO.HIGH)
        light_ready = True

        stream_server = start_stream_server()
        picam2, camera_format = create_camera()
        picam2.start()
        tune_camera_image(picam2)
        camera_started = True

        previous_time = time.monotonic()
        last_stream_time = 0.0
        last_snapshot_time = 0.0
        last_status_time = 0.0
        smoothed_fps = 0.0

        while running:
            frame = picam2.capture_array()
            filtered_mask, roi_start_y = create_filtered_line_mask(frame)

            # O verde mostra somente os pixels aceitos pela máscara filtrada.
            # Esse destaque é visual e não é usado para controlar os motores.
            line_roi_debug = frame[roi_start_y:frame.shape[0], :]
            line_roi_debug[filtered_mask > 0] = (0, 255, 0)

            # A linha amarela marca onde começam os 25% processados da imagem.
            cv2.line(
                frame,
                (0, roi_start_y),
                (frame.shape[1] - 1, roi_start_y),
                (0, 255, 255),
                2,
            )
            active_pixel_count = cv2.countNonZero(filtered_mask)
            debug_lines = (
                "MÁSCARA EXPERIMENTAL",
                f"THRESHOLD: {LINE_THRESHOLD}",
                f"PIXELS ATIVOS: {active_pixel_count}",
            )
            for index, debug_text in enumerate(debug_lines):
                cv2.putText(
                    frame,
                    debug_text,
                    (12, roi_start_y + 28 + index * 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            now = time.monotonic()
            elapsed = now - previous_time
            previous_time = now
            if elapsed > 0.0:
                actual_fps = 1.0 / elapsed
                smoothed_fps = (
                    smoothed_fps * 0.90 + actual_fps * 0.10
                    if smoothed_fps > 0.0
                    else actual_fps
                )

            stream_due = now - last_stream_time >= 1.0 / MJPEG_STREAM_FPS
            snapshot_due = now - last_snapshot_time >= 1.0 / SNAPSHOT_FRAME_FPS
            if stream_due or snapshot_due:
                jpeg = encode_frame(frame)
                if jpeg is not None:
                    if stream_due:
                        publish_stream_frame(jpeg)
                        last_stream_time = now
                    if snapshot_due:
                        save_frame(jpeg)
                        last_snapshot_time = now

            if now - last_status_time >= 1.0 / STATUS_FPS:
                save_status(smoothed_fps, camera_format)
                last_status_time = now
    except Exception as error:
        error_message = f"Camera script failed: {error}"
        print(error_message, flush=True)
        save_status(0.0, active=False, error_message=error_message)
        return 1
    finally:
        if stream_server is not None:
            stream_server.shutdown()
            stream_server.server_close()
        if GPIO is not None and light_ready:
            GPIO.output(LIGHT_PIN_BOARD, GPIO.LOW)
            GPIO.cleanup()
        if picam2 is not None and camera_started:
            picam2.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
