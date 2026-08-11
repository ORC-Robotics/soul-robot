"""Publica a imagem da câmera frontal no dashboard.

A segmentação experimental destaca a linha preta apenas na imagem de debug.
Ela não calcula comandos nem interfere no controle do robô.
"""

import json
import math
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
LINE_STATUS_PATH = "/dev/shm/obr_line_status.json"
TEMP_LINE_STATUS_PATH = "/dev/shm/obr_line_status.tmp.json"

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

# A segmentação experimental usa somente os 32,5% inferiores do frame.
# Alterar este valor muda apenas a região destacada no vídeo de debug.
LINE_ROI_START_RATIO = 0
# Pixels abaixo deste valor são considerados parte da linha preta.
# Este threshold é fixo e não altera a exposição configurada da câmera.
LINE_THRESHOLD = 100
# A abertura 3x3 remove pequenos ruídos isolados do piso.
OPEN_KERNEL_SIZE = 3
# O fechamento 5x5 preenche pequenos buracos dentro da faixa preta.
CLOSE_KERNEL_SIZE = 5
# Espessura mínima medida no contorno completo, antes dos recortes das bandas.
FULL_LINE_MIN_SHORT_SIDE_PX = 50.0
# A far_band diagnóstica vai de 67,5% até 82,5% da altura total do frame.
FAR_BAND_START_RATIO = 0.1
FAR_BAND_END_RATIO = 0.825
# A near_band preserva exatamente os 7,5% inferiores do frame.
NEAR_BAND_START_RATIO = 0.925
# A prévia usa a mesma escala normalizada de potência dos motores.
# A base coincide com o piso operacional necessário para iniciar o movimento.
BASE_SPEED_PREVIEW = 0.65
KP_PREVIEW = 0.30
MAX_CORRECTION_PREVIEW = 0.15
# Limite superior da prévia na escala normalizada do protocolo de motores.
MAX_OPERATIONAL_PREVIEW = 1.00
# A zona morta usa o erro normalizado, não a correção de potência.
NEAR_DEADZONE = 0.10
# A conversão aproximada considera a fita física de 2 cm apenas para debug.
REFERENCE_LINE_WIDTH_CM = 2.0
# Espaçamento, em pixels, entre as marcas pequenas da régua da near_band.
PIXEL_RULER_STEP = 20
# Habilita ou desabilita somente os textos de diagnóstico desenhados no frame.
DEBUG_TEXT_OVERLAY = False

running = True
latest_jpeg = None
latest_jpeg_sequence = 0
frame_condition = threading.Condition()


def put_debug_text(*args, **kwargs):
    """Desenha textos de diagnóstico somente quando o overlay está habilitado."""
    if DEBUG_TEXT_OVERLAY:
        cv2.putText(*args, **kwargs)


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
    roi_start_y = int(round(frame_height * LINE_ROI_START_RATIO))
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


def create_line_candidate_mask(filtered_mask):
    """Mantém somente contornos completos com espessura compatível com a fita."""

    full_contours, _ = cv2.findContours(
        filtered_mask.copy(),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    accepted_contours = []
    for contour in full_contours:
        rect = cv2.minAreaRect(contour)
        width, height = rect[1]
        if (
            not math.isfinite(width)
            or not math.isfinite(height)
            or width <= 0.0
            or height <= 0.0
        ):
            continue

        short_side_px = min(width, height)
        if short_side_px >= FULL_LINE_MIN_SHORT_SIDE_PX:
            accepted_contours.append(contour)

    line_candidate_mask = filtered_mask.copy()
    line_candidate_mask.fill(0)
    if accepted_contours:
        cv2.drawContours(
            line_candidate_mask,
            accepted_contours,
            -1,
            255,
            cv2.FILLED,
        )
    return line_candidate_mask


def select_largest_line_contour(contours):
    """Seleciona o maior contorno da banda com momento válido."""

    selected_contour = None
    selected_area = 0.0
    selected_moments = None

    for contour in contours:
        contour_area = cv2.contourArea(contour)
        contour_moments = cv2.moments(contour)
        if contour_area > selected_area and contour_moments["m00"] > 0.0:
            selected_contour = contour
            selected_area = contour_area
            selected_moments = contour_moments

    return selected_contour, selected_area, selected_moments


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


def save_line_status(
    near_valid,
    near_error,
    control_error,
    correction,
    left_preview,
    right_preview,
    far_valid,
    far_error,
    far_area,
    center_delta_valid,
    center_delta_px,
    line_timestamp,
    line_sequence,
):
    """Publica em memória compartilhada o resultado visual já calculado."""

    requested_near_valid = bool(near_valid)
    line_values_finite = False
    if requested_near_valid:
        try:
            near_error = float(near_error)
            control_error = float(control_error)
            correction = float(correction)
            left_preview = float(left_preview)
            right_preview = float(right_preview)
            line_values_finite = all(
                math.isfinite(value)
                for value in (
                    near_error,
                    control_error,
                    correction,
                    left_preview,
                    right_preview,
                )
            )
        except (TypeError, ValueError):
            line_values_finite = False

    near_valid = requested_near_valid and line_values_finite
    if not near_valid:
        near_error = 0.0
        control_error = 0.0
        correction = 0.0
        left_preview = 0.0
        right_preview = 0.0

    requested_far_valid = bool(far_valid)
    far_values_valid = False
    if requested_far_valid:
        try:
            far_error = float(far_error)
            far_area = float(far_area)
            far_values_valid = (
                math.isfinite(far_error)
                and -1.0 <= far_error <= 1.0
                and math.isfinite(far_area)
                and far_area >= 0.0
            )
        except (TypeError, ValueError):
            far_values_valid = False

    far_valid = requested_far_valid and far_values_valid
    if not far_valid:
        far_error = 0.0
        far_area = 0.0

    requested_center_delta_valid = bool(center_delta_valid)
    center_delta_finite = False
    if requested_center_delta_valid:
        try:
            center_delta_px = float(center_delta_px)
            center_delta_finite = math.isfinite(center_delta_px)
        except (TypeError, ValueError):
            center_delta_finite = False

    center_delta_valid = (
        requested_center_delta_valid
        and near_valid
        and far_valid
        and center_delta_finite
    )
    if not center_delta_valid:
        center_delta_px = 0.0

    try:
        line_timestamp = float(line_timestamp)
        if not math.isfinite(line_timestamp):
            raise ValueError("lineTimestamp inválido")
        if (
            not isinstance(line_sequence, int)
            or isinstance(line_sequence, bool)
            or line_sequence < 0
        ):
            raise ValueError("lineSequence inválido")

        line_status = {
            "nearValid": near_valid,
            "nearError": near_error,
            "controlError": control_error,
            "correction": correction,
            "leftPreview": left_preview,
            "rightPreview": right_preview,
            "farValid": far_valid,
            "farError": far_error,
            "farArea": far_area,
            "centerDeltaValid": center_delta_valid,
            "centerDeltaPx": center_delta_px,
            "lineTimestamp": line_timestamp,
            "lineSequence": line_sequence,
        }
        with open(TEMP_LINE_STATUS_PATH, "w", encoding="utf-8") as status_file:
            json.dump(line_status, status_file, allow_nan=False)
        os.replace(TEMP_LINE_STATUS_PATH, LINE_STATUS_PATH)
    except (OSError, TypeError, ValueError) as error:
        print(f"Falha ao publicar telemetria rápida da linha: {error}", flush=True)


def save_status(
    fps,
    camera_format="",
    active=True,
    error_message="",
    near_valid=False,
    near_error=0.0,
    near_area=0.0,
    near_height_px=0,
    control_error=0.0,
    correction=0.0,
    left_preview=0.0,
    right_preview=0.0,
    far_valid=False,
    far_error=0.0,
    far_area=0.0,
    far_height_px=0,
    center_delta_valid=False,
    center_delta_px=0.0,
    line_timestamp=0.0,
    line_sequence=0,
):
    """Publica saúde da câmera e a telemetria visual já calculada."""

    requested_near_valid = bool(near_valid)
    line_values_finite = not requested_near_valid
    if requested_near_valid:
        try:
            near_error = float(near_error)
            near_area = float(near_area)
            control_error = float(control_error)
            correction = float(correction)
            left_preview = float(left_preview)
            right_preview = float(right_preview)
            line_values_finite = all(
                math.isfinite(value)
                for value in (
                    near_error,
                    near_area,
                    control_error,
                    correction,
                    left_preview,
                    right_preview,
                )
            ) and near_area >= 0.0
        except (TypeError, ValueError):
            line_values_finite = False

    near_height_valid = (
        isinstance(near_height_px, int)
        and not isinstance(near_height_px, bool)
        and near_height_px >= 0
    )
    if not near_height_valid:
        near_height_px = 0

    requested_far_valid = bool(far_valid)
    far_values_valid = not requested_far_valid
    if requested_far_valid:
        try:
            far_error = float(far_error)
            far_area = float(far_area)
            far_values_valid = (
                math.isfinite(far_error)
                and -1.0 <= far_error <= 1.0
                and math.isfinite(far_area)
                and far_area >= 0.0
            )
        except (TypeError, ValueError):
            far_values_valid = False

    far_height_valid = (
        isinstance(far_height_px, int)
        and not isinstance(far_height_px, bool)
        and far_height_px >= 0
    )
    if not far_height_valid:
        far_height_px = 0

    requested_center_delta_valid = bool(center_delta_valid)
    center_delta_finite = False
    if requested_center_delta_valid:
        try:
            center_delta_px = float(center_delta_px)
            center_delta_finite = math.isfinite(center_delta_px)
        except (TypeError, ValueError):
            center_delta_finite = False

    try:
        line_timestamp = float(line_timestamp)
        line_timestamp_valid = (
            math.isfinite(line_timestamp) and line_timestamp >= 0.0
        )
    except (TypeError, ValueError):
        line_timestamp_valid = False

    line_sequence_valid = (
        isinstance(line_sequence, int)
        and not isinstance(line_sequence, bool)
        and line_sequence >= 0
    )
    near_valid = (
        requested_near_valid
        and line_values_finite
        and line_timestamp_valid
        and line_sequence_valid
    )
    far_valid = (
        requested_far_valid
        and far_values_valid
        and line_timestamp_valid
        and line_sequence_valid
    )
    center_delta_valid = (
        requested_center_delta_valid
        and near_valid
        and far_valid
        and center_delta_finite
    )
    if not near_valid:
        near_error = 0.0
        near_area = 0.0
        control_error = 0.0
        correction = 0.0
        left_preview = 0.0
        right_preview = 0.0
    if not far_valid:
        far_error = 0.0
        far_area = 0.0
    if not center_delta_valid:
        center_delta_px = 0.0
    if not line_timestamp_valid:
        line_timestamp = 0.0
    if not line_sequence_valid:
        line_sequence = 0

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
        "nearValid": near_valid,
        "nearError": near_error,
        "nearArea": near_area,
        "nearHeightPx": near_height_px,
        "controlError": control_error,
        "correction": correction,
        "leftPreview": left_preview,
        "rightPreview": right_preview,
        "farValid": far_valid,
        "farError": far_error,
        "farArea": far_area,
        "farHeightPx": far_height_px,
        "centerDeltaValid": center_delta_valid,
        "centerDeltaPx": center_delta_px,
        "lineTimestamp": line_timestamp,
        "lineSequence": line_sequence,
    }
    with open(TEMP_STATUS_PATH, "w", encoding="utf-8") as status_file:
        json.dump(status, status_file, allow_nan=False)
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
        line_sequence = 0

        while running:
            frame = picam2.capture_array()
            filtered_mask, roi_start_y = create_filtered_line_mask(frame)
            line_candidate_mask = create_line_candidate_mask(filtered_mask)

            frame_height = frame.shape[0]
            far_band_start_y = int(round(
                frame_height * FAR_BAND_START_RATIO
            ))
            far_band_end_y = int(round(
                frame_height * FAR_BAND_END_RATIO
            ))
            far_band_start_in_roi = far_band_start_y - roi_start_y
            far_band_end_in_roi = far_band_end_y - roi_start_y
            far_band = line_candidate_mask[
                far_band_start_in_roi:far_band_end_in_roi,
                :,
            ]
            far_band_height_px = far_band.shape[0]
            far_contours, _ = cv2.findContours(
                far_band.copy(),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )

            selected_far_contour, far_area, far_moments = (
                select_largest_line_contour(far_contours)
            )

            far_valid = selected_far_contour is not None and far_moments is not None
            far_center_x = 0.0
            far_error = 0.0
            far_center = None
            if far_valid:
                far_center_x = far_moments["m10"] / far_moments["m00"]
                far_center_y = far_moments["m01"] / far_moments["m00"]
                frame_half_width = frame.shape[1] / 2.0
                far_error = (
                    far_center_x - frame_half_width
                ) / frame_half_width
                far_center = (
                    int(round(far_center_x)),
                    far_band_start_y + int(round(far_center_y)),
                )
            else:
                far_area = 0.0

            near_band_start_y = int(round(
                frame_height * NEAR_BAND_START_RATIO
            ))
            near_band_start_in_roi = near_band_start_y - roi_start_y
            near_band = line_candidate_mask[near_band_start_in_roi:, :]
            near_band_height_px = near_band.shape[0]
            near_contours, _ = cv2.findContours(
                near_band.copy(),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )

            selected_near_contour, near_contour_area, near_moments = (
                select_largest_line_contour(near_contours)
            )

            near_error = None
            near_center = None
            line_center_x = None
            if selected_near_contour is not None and near_moments is not None:
                center_x = near_moments["m10"] / near_moments["m00"]
                center_y = near_moments["m01"] / near_moments["m00"]
                frame_half_width = frame.shape[1] / 2.0
                near_error = (center_x - frame_half_width) / frame_half_width
                line_center_x = center_x
                near_center = (
                    int(round(center_x)),
                    near_band_start_y + int(round(center_y)),
                )

            near_valid = near_error is not None
            center_delta_valid = near_valid and far_valid
            center_delta_px = 0.0
            if center_delta_valid:
                center_delta_px = line_center_x - far_center_x

            frame_width = frame.shape[1]
            frame_center_x = frame_width / 2.0
            safe_half_width_px = round(
                NEAR_DEADZONE * frame_width / 2.0
            )
            safe_left_x = int(round(frame_center_x - safe_half_width_px))
            safe_right_x = int(round(frame_center_x + safe_half_width_px))

            if not near_valid:
                control_error = 0.0
                correction = 0.0
                left_preview = 0.0
                right_preview = 0.0
                offset_px = None
                preview_state = "LINHA INVALIDA"
                preview_direction = "SEM COMANDO"
            elif abs(near_error) <= NEAR_DEADZONE:
                control_error = 0.0
                correction = 0.0
                left_preview = BASE_SPEED_PREVIEW
                right_preview = BASE_SPEED_PREVIEW
                offset_px = near_error * (frame_width / 2.0)
                preview_state = "RETO SEGURO"
                preview_direction = "RETO"
            else:
                error_sign = 1.0 if near_error > 0.0 else -1.0
                control_error = error_sign * (
                    (abs(near_error) - NEAR_DEADZONE)
                    / (1.0 - NEAR_DEADZONE)
                )
                correction = max(
                    -MAX_CORRECTION_PREVIEW,
                    min(KP_PREVIEW * control_error, MAX_CORRECTION_PREVIEW),
                )
                offset_px = near_error * (frame_width / 2.0)
                if correction > 0.0:
                    left_preview = BASE_SPEED_PREVIEW + correction
                    right_preview = BASE_SPEED_PREVIEW
                    preview_state = "CORRIGINDO DIREITA"
                    preview_direction = "DIREITA"
                elif correction < 0.0:
                    left_preview = BASE_SPEED_PREVIEW
                    right_preview = BASE_SPEED_PREVIEW + abs(correction)
                    preview_state = "CORRIGINDO ESQUERDA"
                    preview_direction = "ESQUERDA"
                else:
                    left_preview = BASE_SPEED_PREVIEW
                    right_preview = BASE_SPEED_PREVIEW
                    preview_state = "RETO"
                    preview_direction = "RETO"

            if near_valid:
                left_preview = min(left_preview, MAX_OPERATIONAL_PREVIEW)
                right_preview = min(right_preview, MAX_OPERATIONAL_PREVIEW)

            line_timestamp = time.time()
            line_sequence += 1
            save_line_status(
                near_valid,
                near_error,
                control_error,
                correction,
                left_preview,
                right_preview,
                far_valid,
                far_error,
                far_area,
                center_delta_valid,
                center_delta_px,
                line_timestamp,
                line_sequence,
            )

            # A largura é medida em uma única altura fixa da near_band.
            # A estimativa em centímetros só é útil com a fita aproximadamente
            # longitudinal e nunca participa do cálculo de controle.
            measurement_y_in_band = near_band.shape[0] // 2
            measurement_y_frame = near_band_start_y + measurement_y_in_band
            measurement_row = near_band[measurement_y_in_band, :]
            line_runs = []
            run_start_x = None
            for pixel_x in range(frame_width):
                if measurement_row[pixel_x] != 0 and run_start_x is None:
                    run_start_x = pixel_x
                elif measurement_row[pixel_x] == 0 and run_start_x is not None:
                    line_runs.append((run_start_x, pixel_x - 1))
                    run_start_x = None
            if run_start_x is not None:
                line_runs.append((run_start_x, frame_width - 1))

            line_left_x = None
            line_right_x = None
            line_width_px = None
            if line_center_x is not None and line_runs:
                nearest_run_distance = float("inf")
                for run_left_x, run_right_x in line_runs:
                    if run_left_x <= line_center_x <= run_right_x:
                        run_distance = 0.0
                    else:
                        run_distance = min(
                            abs(line_center_x - run_left_x),
                            abs(line_center_x - run_right_x),
                        )
                    if run_distance < nearest_run_distance:
                        nearest_run_distance = run_distance
                        line_left_x = run_left_x
                        line_right_x = run_right_x

            px_per_cm_approx = None
            offset_cm_approx = None
            if line_left_x is not None and line_right_x is not None:
                line_width_px = line_right_x - line_left_x + 1
                px_per_cm_approx = line_width_px / REFERENCE_LINE_WIDTH_CM
                if offset_px is not None and px_per_cm_approx > 0.0:
                    offset_cm_approx = offset_px / px_per_cm_approx

            # O fundo preto elimina da visualização tudo que não pertence à linha.
            # Somente os candidatos globais aprovados aparecem em branco.
            frame[:] = (0, 0, 0)
            line_roi_debug = frame[roi_start_y:frame.shape[0], :]
            line_roi_debug[line_candidate_mask > 0] = (255, 255, 255)

            # O preenchimento usa somente o recorte estreito da zona segura
            # para reduzir cópias de imagem e preservar o FPS do stream.
            safe_zone_debug = frame[
                near_band_start_y:frame.shape[0],
                safe_left_x:safe_right_x,
            ]
            safe_zone_green = safe_zone_debug.copy()
            safe_zone_green[:] = (0, 255, 0)
            cv2.addWeighted(
                safe_zone_green,
                0.20,
                safe_zone_debug,
                0.80,
                0.0,
                safe_zone_debug,
            )

            # A linha amarela marca onde começam os 32,5% processados da imagem.
            cv2.line(
                frame,
                (0, roi_start_y),
                (frame.shape[1] - 1, roi_start_y),
                (0, 255, 255),
                2,
            )
            # A far_band e seu centro são referências diagnósticas em laranja.
            cv2.rectangle(
                frame,
                (0, far_band_start_y),
                (frame.shape[1] - 1, far_band_end_y - 1),
                (0, 165, 255),
                2,
            )
            # A near_band e seu centro são apenas referências visuais em azul.
            cv2.rectangle(
                frame,
                (0, near_band_start_y),
                (frame.shape[1] - 1, frame.shape[0] - 1),
                (255, 0, 0),
                2,
            )

            if not near_valid:
                safe_limit_color = (0, 0, 255)
            elif abs(near_error) > NEAR_DEADZONE:
                safe_limit_color = (0, 255, 255)
            else:
                safe_limit_color = (0, 255, 0)
            cv2.line(
                frame,
                (int(round(frame_center_x)), near_band_start_y),
                (int(round(frame_center_x)), frame.shape[0] - 1),
                (255, 255, 0),
                2,
            )
            for safe_limit_x in (safe_left_x, safe_right_x):
                cv2.line(
                    frame,
                    (safe_limit_x, near_band_start_y),
                    (safe_limit_x, frame.shape[0] - 1),
                    safe_limit_color,
                    2,
                )

            cv2.line(
                frame,
                (0, measurement_y_frame),
                (frame_width - 1, measurement_y_frame),
                (180, 180, 180),
                1,
            )
            if line_left_x is not None and line_right_x is not None:
                cv2.line(
                    frame,
                    (line_left_x, measurement_y_frame),
                    (line_right_x, measurement_y_frame),
                    (255, 0, 255),
                    2,
                )
                cv2.circle(
                    frame,
                    (line_left_x, measurement_y_frame),
                    4,
                    (255, 0, 255),
                    -1,
                )
                cv2.circle(
                    frame,
                    (line_right_x, measurement_y_frame),
                    4,
                    (255, 0, 255),
                    -1,
                )

            ruler_bottom_y = frame.shape[0] - 3
            first_ruler_offset = -(
                int(frame_center_x) // PIXEL_RULER_STEP
            ) * PIXEL_RULER_STEP
            for ruler_offset in range(
                first_ruler_offset,
                frame_width,
                PIXEL_RULER_STEP,
            ):
                ruler_x = int(round(frame_center_x + ruler_offset))
                if ruler_x < 0 or ruler_x >= frame_width:
                    continue
                labeled_tick = ruler_offset % (PIXEL_RULER_STEP * 2) == 0
                tick_height = 11 if labeled_tick else 6
                cv2.line(
                    frame,
                    (ruler_x, ruler_bottom_y),
                    (ruler_x, ruler_bottom_y - tick_height),
                    (255, 255, 255),
                    1,
                )
                if labeled_tick:
                    ruler_label = (
                        f"+{ruler_offset}" if ruler_offset > 0 else str(ruler_offset)
                    )
                    put_debug_text(
                        frame,
                        ruler_label,
                        (max(0, ruler_x - 11), ruler_bottom_y - 14),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.30,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA,
                    )

            if near_center is not None:
                cv2.circle(frame, near_center, 6, (255, 0, 0), -1)
            if far_center is not None:
                cv2.circle(frame, far_center, 6, (0, 165, 255), -1)

            active_pixel_count = cv2.countNonZero(filtered_mask)
            debug_lines = (
                "MASCARA EXPERIMENTAL",
                f"THRESHOLD: {LINE_THRESHOLD}",
                f"PIXELS ATIVOS: {active_pixel_count}",
            )
            for index, debug_text in enumerate(debug_lines):
                put_debug_text(
                    frame,
                    debug_text,
                    (12, roi_start_y + 28 + index * 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            offset_px_text = (
                f"{offset_px:+.1f}" if offset_px is not None else "INVALIDO"
            )
            line_center_text = (
                f"{line_center_x:.1f}" if line_center_x is not None else "INVALIDO"
            )
            control_debug_lines = (
                f"CONTROL ERROR: {control_error:+.3f}",
                f"OFFSET PX: {offset_px_text}",
                f"SAFE ZONE: +/-{safe_half_width_px} PX",
                f"CENTER X: {frame_center_x:.1f}",
                f"LINE CENTER X: {line_center_text}",
                f"ESTADO: {preview_state}",
            )
            control_debug_x = max(12, int(frame_width * 0.36))
            for index, debug_text in enumerate(control_debug_lines):
                put_debug_text(
                    frame,
                    debug_text,
                    (control_debug_x, 30 + index * 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (230, 230, 230),
                    1,
                    cv2.LINE_AA,
                )

            line_width_text = (
                str(line_width_px) if line_width_px is not None else "INVALIDO"
            )
            px_per_cm_text = (
                f"{px_per_cm_approx:.2f}"
                if px_per_cm_approx is not None
                else "INVALIDO"
            )
            offset_cm_text = (
                f"{offset_cm_approx:+.2f}"
                if offset_cm_approx is not None
                else "INVALIDO"
            )
            measurement_debug_lines = (
                f"LINE WIDTH PX: {line_width_text}",
                f"PX/CM APROX: {px_per_cm_text}",
                f"OFFSET CM APROX: {offset_cm_text}",
                "APROX: VALIDA SO NESTA ALTURA",
                "FITA APROX. LONGITUDINAL",
            )
            for index, debug_text in enumerate(measurement_debug_lines):
                put_debug_text(
                    frame,
                    debug_text,
                    (12, 30 + index * 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.48,
                    (255, 0, 255),
                    1,
                    cv2.LINE_AA,
                )

            near_debug_lines = (
                (
                    f"NEAR ERROR: {near_error:+.3f}"
                    if near_error is not None
                    else "NEAR ERROR: INVALIDO"
                ),
                f"NEAR VALID: {'SIM' if near_error is not None else 'NAO'}",
                f"NEAR AREA: {near_contour_area:.1f}",
                f"NEAR HEIGHT PX: {near_band_height_px}",
            )
            near_debug_x = max(12, frame.shape[1] - 330)
            for index, debug_text in enumerate(near_debug_lines):
                put_debug_text(
                    frame,
                    debug_text,
                    (near_debug_x, 30 + index * 26),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (255, 0, 0),
                    2,
                    cv2.LINE_AA,
                )

            center_delta_text = (
                f"{center_delta_px:+.1f}"
                if center_delta_valid
                else "INVALIDO"
            )
            far_debug_lines = (
                f"FAR VALID: {'SIM' if far_valid else 'NAO'}",
                f"FAR ERROR: {far_error:+.3f}",
                f"FAR AREA: {far_area:.1f}",
                f"CENTER DELTA PX: {center_delta_text}",
                f"FAR HEIGHT PX: {far_band_height_px}",
            )
            for index, debug_text in enumerate(far_debug_lines):
                put_debug_text(
                    frame,
                    debug_text,
                    (near_debug_x, 270 + index * 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.60,
                    (0, 165, 255),
                    2,
                    cv2.LINE_AA,
                )

            preview_debug_lines = (
                f"CORRECTION: {correction:+.3f}",
                f"LEFT PREVIEW: {left_preview:.3f}",
                f"RIGHT PREVIEW: {right_preview:.3f}",
            )
            for index, debug_text in enumerate(preview_debug_lines):
                put_debug_text(
                    frame,
                    debug_text,
                    (near_debug_x, 136 + index * 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            put_debug_text(
                frame,
                preview_direction,
                (near_debug_x, 225),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            if near_valid:
                arrow_start = (frame.shape[1] - 80, 245)
                if preview_direction == "ESQUERDA":
                    arrow_end = (arrow_start[0] - 60, arrow_start[1])
                elif preview_direction == "DIREITA":
                    arrow_end = (arrow_start[0] + 60, arrow_start[1])
                else:
                    arrow_end = (arrow_start[0], arrow_start[1] - 50)
                cv2.arrowedLine(
                    frame,
                    arrow_start,
                    arrow_end,
                    (0, 255, 255),
                    3,
                    cv2.LINE_AA,
                    tipLength=0.30,
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
                save_status(
                    smoothed_fps,
                    camera_format,
                    near_valid=near_valid,
                    near_error=near_error,
                    near_area=near_contour_area,
                    near_height_px=near_band_height_px,
                    control_error=control_error,
                    correction=correction,
                    left_preview=left_preview,
                    right_preview=right_preview,
                    far_valid=far_valid,
                    far_error=far_error,
                    far_area=far_area,
                    far_height_px=far_band_height_px,
                    center_delta_valid=center_delta_valid,
                    center_delta_px=center_delta_px,
                    line_timestamp=line_timestamp,
                    line_sequence=line_sequence,
                )
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
