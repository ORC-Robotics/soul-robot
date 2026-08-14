"""Publica no dashboard a imagem da câmera selecionada.

A segmentação experimental destaca a linha preta apenas na imagem de debug.
Ela não calcula comandos nem interfere no controle do robô.
"""

import argparse
import json
import math
import os
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

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
MJPEG_STREAM_PORT = 8090
MJPEG_STREAM_PATH = "/stream.mjpg"
MJPEG_STREAM_FPS = 30
SNAPSHOT_FRAME_FPS = 2
STATUS_FPS = 5
JPEG_QUALITY = 82
CAMERA_PIXEL_FORMATS = ("RGB888",)

# O nome RGB888 segue a convenção do libcamera. No array retornado pelo
# Picamera2, cada pixel fica em ordem B, G, R, que é a ordem nativa do OpenCV.
CAMERA_ARRAY_COLOR_ORDER = "BGR"

# Limites iniciais deliberadamente amplos para o verde. Estes valores ainda
# precisam de calibração física sob a iluminação da pista da competição.
GREEN_HUE_MIN = 35
GREEN_HUE_MAX = 90
GREEN_SATURATION_MIN = 70
GREEN_VALUE_MIN = 50
GREEN_OPEN_KERNEL_SIZE = 3
GREEN_CLOSE_KERNEL_SIZE = 5
GREEN_MIN_AREA_PX = 80.0
GREEN_MIN_DIMENSION_PX = 6.0
GREEN_ASPECT_RATIO_MIN = 0.35
GREEN_ASPECT_RATIO_MAX = 1.0
GREEN_MIN_EXTENT = 0.35
GREEN_PARTIAL_BORDER_TOLERANCE_PX = 4
GREEN_PARTIAL_AREA_FACTOR = 0.40
GREEN_PARTIAL_DIMENSION_FACTOR = 0.50
GREEN_PARTIAL_ASPECT_RATIO_MIN = 0.20
GREEN_PARTIAL_EXTENT_MIN = 0.20
GREEN_FRAGMENT_MERGE_GAP_PX = 12
GREEN_LINE_AXIS_MIN_LENGTH_PX = 20.0
GREEN_SIDE_MIN_DISTANCE_PX = 5.0
GREEN_LINE_DISTANCE_MAX_RATIO = 4.0
GREEN_MARKER_TO_LINE_MIN_RATIO = 0.40
GREEN_MARKER_TO_LINE_MAX_RATIO = 3.0
GREEN_PAIR_LONGITUDINAL_TOLERANCE_LINE_WIDTHS = 2.5
GREEN_ENCOUNTER_DISTANCE_LINE_WIDTHS = 8.0
GREEN_CONFIRMATION_FRAMES = 3
GREEN_SINGLE_OBSERVATION_FRAMES = 3
GREEN_CLEAR_HYSTERESIS_FRAMES = 2
GREEN_TOPOLOGY_SAMPLE_STEP = 2

GREEN_OBSERVATION_STATES = {
    "SEM_VERDE",
    "UM_CANDIDATO",
    "DOIS_CANDIDATOS",
    "MULTIPLOS_AMBIGUOS",
}
GREEN_INTERPRETATIONS = {
    "SEM_DECISAO",
    "ESQUERDA",
    "DIREITA",
    "RETORNO_180",
    "VERDE_FALSO_NO_SENTIDO_ATUAL",
    "AMBIGUO",
}

# As duas posições previstas usam a câmera montada de cabeça para baixo.
# Manter a transformação no Picamera2 evita rotacionar cada frame no OpenCV.
CAMERA_ROTATION_DEGREES = 180

# Ajustes básicos de imagem. Eles afetam somente a visualização e não geram
# qualquer decisão de movimento.
CAMERA_SHARPNESS = 1.2
CAMERA_CONTRAST = 1.05
CAMERA_SATURATION = 1.0
CAMERA_EXPOSURE_VALUE = 0.4

# A prévia usa a mesma escala normalizada de potência dos motores.
# A base coincide com o piso operacional necessário para iniciar o movimento.
BASE_SPEED_PREVIEW = 0.65
KP_PREVIEW = 0.30
MAX_CORRECTION_PREVIEW = 0.15
# Limite superior da prévia na escala normalizada do protocolo de motores.
MAX_OPERATIONAL_PREVIEW = 1.00
# A conversão aproximada considera a fita física de 2 cm apenas para debug.
REFERENCE_LINE_WIDTH_CM = 2.0

# Cada papel define de forma independente a captura e os parâmetros visuais.
# O perfil inferior não herda ROIs nem limites em pixels da câmera frontal.
CAMERA_PROFILES = {
    "forward": {
        "role": "forward",
        "main_size": (960, 540),
        "sensor_size": (1920, 1080),
        "sensor_bit_depth": 10,
        "target_fps": 30,
        "vision": {
            "line_roi_start_ratio": 0.0,
            "line_threshold": 100,
            "open_kernel_size": 3,
            "close_kernel_size": 5,
            "full_line_min_short_side_ratio": 50.0 / 540.0,
            "far_band_start_ratio": 0.1,
            "far_band_end_ratio": 0.825,
            "near_band_start_ratio": 0.925,
            "near_deadzone_ratio": 0.10,
            "pixel_ruler_step_ratio": 20.0 / 960.0,
            "overlay_line_thickness": 2,
            "overlay_thin_line_thickness": 1,
            "overlay_measurement_radius": 4,
            "overlay_center_radius": 6,
            "overlay_arrow_start_ratio": (0.9167, 0.4537),
            "overlay_arrow_horizontal_ratio": 0.0625,
            "overlay_arrow_vertical_ratio": 0.0926,
            "overlay_arrow_thickness": 3,
            "debug_text_overlay": False,
        },
    },
    "down": {
        "role": "down",
        "main_size": (640, 480),
        "sensor_size": (1640, 1232),
        "sensor_bit_depth": 10,
        "target_fps": 30,
        "vision": {
            "line_roi_start_ratio": 0.0,
            "line_threshold": 100,
            "open_kernel_size": 3,
            "close_kernel_size": 5,
            # Vinte pixels mantêm aproximadamente a mesma espessura angular
            # mínima do perfil frontal após o aumento de campo de visão.
            "full_line_min_short_side_ratio": 20.0 / 480.0,
            # As coordenadas usam o frame de referência 640x480 validado na Pi 4.
            # A conversão centralizada mantém a mesma geometria proporcional se
            # a altura real do frame for diferente durante um diagnóstico.
            "geometry_reference": {
                "frame_height": 480,
                "structural_end_y": 425,
                "far_band_y": (80, 180),
                "near_band_y": (210, 270),
            },
            "ahead_heading_gain": 0.90,
            "near_deadzone_ratio": 0.10,
            "base_speed_preview": 0.70,
            "balanced_differential_mixing": True,
            "minimum_tracking_power": 0.65,
            "green_detection_enabled": True,
            "pixel_ruler_step_ratio": 16.0 / 640.0,
            "overlay_line_thickness": 2,
            "overlay_thin_line_thickness": 1,
            "overlay_measurement_radius": 3,
            "overlay_center_radius": 4,
            "overlay_arrow_start_ratio": (0.9167, 0.4537),
            "overlay_arrow_horizontal_ratio": 0.0625,
            "overlay_arrow_vertical_ratio": 0.0926,
            "overlay_arrow_thickness": 3,
            "debug_text_overlay": False,
        },
    },
}

running = True
latest_jpeg = None
latest_jpeg_sequence = 0
frame_condition = threading.Condition()


def put_debug_text(enabled, *args, **kwargs):
    """Desenha textos de diagnóstico somente quando o overlay está habilitado."""
    if enabled:
        cv2.putText(*args, **kwargs)


def calculate_control_preview(
    vision_profile,
    near_valid,
    near_error,
    far_valid,
    far_error,
):
    """Calcula a prévia de controle sem substituir os erros das duas bandas."""

    if not near_valid:
        return 0.0, 0.0, 0.0, 0.0, 0.0

    guidance_error = near_error
    ahead_heading_gain = vision_profile.get("ahead_heading_gain")
    if far_valid and ahead_heading_gain is not None:
        heading_error = far_error - near_error
        guidance_error += ahead_heading_gain * heading_error
        guidance_error = max(-1.0, min(guidance_error, 1.0))

    deadzone_ratio = vision_profile["near_deadzone_ratio"]
    base_speed_preview = vision_profile.get(
        "base_speed_preview", BASE_SPEED_PREVIEW
    )
    if abs(guidance_error) <= deadzone_ratio:
        control_error = 0.0
        correction = 0.0
        left_preview = base_speed_preview
        right_preview = base_speed_preview
    else:
        error_sign = 1.0 if guidance_error > 0.0 else -1.0
        control_error = error_sign * (
            (abs(guidance_error) - deadzone_ratio)
            / (1.0 - deadzone_ratio)
        )
        correction = max(
            -MAX_CORRECTION_PREVIEW,
            min(KP_PREVIEW * control_error, MAX_CORRECTION_PREVIEW),
        )
        if vision_profile.get("balanced_differential_mixing", False):
            minimum_tracking_power = vision_profile["minimum_tracking_power"]
            maximum_balanced_delta = (
                base_speed_preview - minimum_tracking_power
            )
            requested_delta = correction / 2.0
            balanced_delta = max(
                -maximum_balanced_delta,
                min(requested_delta, maximum_balanced_delta),
            )
            left_preview = base_speed_preview + balanced_delta
            right_preview = base_speed_preview - balanced_delta
        elif correction > 0.0:
            left_preview = base_speed_preview + correction
            right_preview = base_speed_preview
        elif correction < 0.0:
            left_preview = base_speed_preview
            right_preview = base_speed_preview + abs(correction)
        else:
            left_preview = base_speed_preview
            right_preview = base_speed_preview

    left_preview = min(left_preview, MAX_OPERATIONAL_PREVIEW)
    right_preview = min(right_preview, MAX_OPERATIONAL_PREVIEW)
    return (
        guidance_error,
        control_error,
        correction,
        left_preview,
        right_preview,
    )


def resolve_horizontal_deadzone(frame_width, vision_profile):
    """Converte a zona morta normalizada nos mesmos limites usados no vídeo."""

    frame_center_x = frame_width / 2.0
    half_width_px = round(
        vision_profile["near_deadzone_ratio"] * frame_width / 2.0
    )
    return (
        half_width_px,
        int(round(frame_center_x - half_width_px)),
        int(round(frame_center_x + half_width_px)),
    )


def parse_camera_profile(arguments=None):
    """Seleciona o papel da câmera; o argumento tem prioridade sobre o ambiente."""

    environment_role = os.environ.get("OBR_CAMERA_ROLE", "forward").strip().lower()
    parser = argparse.ArgumentParser(description="Captura e processa a câmera do robô.")
    parser.add_argument(
        "--camera-role",
        choices=tuple(CAMERA_PROFILES),
        default=environment_role,
        help="Papel físico da câmera conectada: forward ou down.",
    )
    parsed = parser.parse_args(arguments)
    if parsed.camera_role not in CAMERA_PROFILES:
        parser.error(
            "OBR_CAMERA_ROLE deve ser 'forward' ou 'down', "
            f"mas recebeu {parsed.camera_role!r}."
        )
    return CAMERA_PROFILES[parsed.camera_role]


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


def rectangle_values(rectangle):
    """Converte um Rectangle do libcamera em valores simples para o status."""

    if not hasattr(rectangle, "x"):
        x, y, width, height = rectangle
        return {
            "x": int(x),
            "y": int(y),
            "width": int(width),
            "height": int(height),
        }
    return {
        "x": int(rectangle.x),
        "y": int(rectangle.y),
        "width": int(rectangle.width),
        "height": int(rectangle.height),
    }


def camera_runtime_details(picam2, camera_profile, camera_config):
    """Registra a câmera e o modo físico realmente aceitos pelo Picamera2."""

    sensor_config = camera_config["sensor"]
    sensor_size = tuple(int(value) for value in sensor_config["output_size"])
    requested_sensor_size = tuple(camera_profile["sensor_size"])
    requested_bit_depth = int(camera_profile["sensor_bit_depth"])
    sensor_bit_depth = int(sensor_config["bit_depth"])
    if sensor_size != requested_sensor_size or sensor_bit_depth != requested_bit_depth:
        raise RuntimeError(
            "O Picamera2 não aplicou o modo físico solicitado: "
            f"esperado={requested_sensor_size}/{requested_bit_depth}-bit, "
            f"aplicado={sensor_size}/{sensor_bit_depth}-bit."
        )

    camera = getattr(picam2, "camera", None)
    camera_id = str(getattr(camera, "id", ""))
    properties = getattr(picam2, "camera_properties", {})
    return {
        "cameraId": camera_id,
        "cameraModel": str(properties.get("Model", "")),
        "sensorMode": {
            "width": sensor_size[0],
            "height": sensor_size[1],
            "bitDepth": sensor_bit_depth,
            "format": str(camera_config["raw"]["format"]),
        },
        "scalerCrop": None,
        "transform": "hvflip",
    }


def create_camera(camera_profile):
    """Configura a Camera V2 e exige o modo físico definido para seu papel."""

    picam2 = Picamera2()
    frame_width, frame_height = camera_profile["main_size"]
    target_fps = camera_profile["target_fps"]
    frame_duration_us = int(1_000_000 / target_fps)
    camera_transform = Transform(hflip=True, vflip=True)
    sensor = {
        "output_size": camera_profile["sensor_size"],
        "bit_depth": camera_profile["sensor_bit_depth"],
    }

    for pixel_format in CAMERA_PIXEL_FORMATS:
        try:
            camera_config = picam2.create_video_configuration(
                main={"size": (frame_width, frame_height), "format": pixel_format},
                sensor=sensor,
                controls={"FrameDurationLimits": (frame_duration_us, frame_duration_us)},
                transform=camera_transform,
                buffer_count=4,
            )
            picam2.configure(camera_config)
            applied_config = picam2.camera_configuration()
            runtime_details = camera_runtime_details(
                picam2,
                camera_profile,
                applied_config,
            )
            print(
                f"Câmera {camera_profile['role']} configurada em {pixel_format}, "
                f"main {frame_width}x{frame_height}, sensor "
                f"{sensor['output_size'][0]}x{sensor['output_size'][1]} "
                f"{sensor['bit_depth']}-bit e alvo de {target_fps} FPS.",
                flush=True,
            )
            return picam2, pixel_format, runtime_details
        except Exception as error:
            print(f"Configuração {pixel_format} falhou: {error}", flush=True)

    camera_config = picam2.create_still_configuration(
        {"size": (frame_width, frame_height), "format": "RGB888"},
        sensor=sensor,
        transform=camera_transform,
    )
    picam2.configure(camera_config)
    applied_config = picam2.camera_configuration()
    runtime_details = camera_runtime_details(picam2, camera_profile, applied_config)
    print(
        f"Câmera {camera_profile['role']} configurada em modo still como fallback.",
        flush=True,
    )
    return picam2, "RGB888", runtime_details


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


def create_filtered_line_mask(frame, vision_profile):
    """Segmenta a linha preta na parte inferior sem gerar decisões de controle."""

    frame_height = frame.shape[0]
    roi_start_y = int(round(frame_height * vision_profile["line_roi_start_ratio"]))
    line_roi = frame[roi_start_y:frame_height, :]
    gray_roi = cv2.cvtColor(line_roi, cv2.COLOR_BGR2GRAY)

    _, binary_mask = cv2.threshold(
        gray_roi,
        vision_profile["line_threshold"],
        255,
        cv2.THRESH_BINARY_INV,
    )

    open_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (vision_profile["open_kernel_size"], vision_profile["open_kernel_size"]),
    )
    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (vision_profile["close_kernel_size"], vision_profile["close_kernel_size"]),
    )
    filtered_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, open_kernel)
    filtered_mask = cv2.morphologyEx(filtered_mask, cv2.MORPH_CLOSE, close_kernel)
    return filtered_mask, roi_start_y


def scale_reference_y(reference_y, reference_height, frame_height):
    """Converte uma coordenada vertical de referência para a altura real."""

    return int(round(frame_height * reference_y / reference_height))


def resolve_vision_geometry(frame_height, vision_profile):
    """Calcula os limites estruturais e das bandas para o frame atual."""

    geometry_reference = vision_profile.get("geometry_reference")
    if geometry_reference is None:
        return {
            "structural_end_y": frame_height,
            "ignored_start_y": None,
            "far_band_start_y": int(round(
                frame_height * vision_profile["far_band_start_ratio"]
            )),
            "far_band_end_y": int(round(
                frame_height * vision_profile["far_band_end_ratio"]
            )),
            "near_band_start_y": int(round(
                frame_height * vision_profile["near_band_start_ratio"]
            )),
            "near_band_end_y": frame_height,
        }

    reference_height = geometry_reference["frame_height"]
    structural_end_y = scale_reference_y(
        geometry_reference["structural_end_y"],
        reference_height,
        frame_height,
    )
    far_band_start_y, far_band_end_y = (
        scale_reference_y(reference_y, reference_height, frame_height)
        for reference_y in geometry_reference["far_band_y"]
    )
    near_band_start_y, near_band_end_y = (
        scale_reference_y(reference_y, reference_height, frame_height)
        for reference_y in geometry_reference["near_band_y"]
    )

    if not (
        0 <= far_band_start_y < far_band_end_y <= structural_end_y
        and 0 <= near_band_start_y < near_band_end_y <= structural_end_y
        and structural_end_y <= frame_height
    ):
        raise ValueError("A geometria vertical da câmera está fora do frame.")

    return {
        "structural_end_y": structural_end_y,
        "ignored_start_y": structural_end_y,
        "far_band_start_y": far_band_start_y,
        "far_band_end_y": far_band_end_y,
        "near_band_start_y": near_band_start_y,
        "near_band_end_y": near_band_end_y,
    }


def create_structural_line_mask(
    filtered_mask,
    roi_start_y,
    structural_end_y,
):
    """Remove da máscara a área física que não pode gerar candidatos."""

    structural_end_in_roi = max(
        0,
        min(filtered_mask.shape[0], structural_end_y - roi_start_y),
    )
    if structural_end_in_roi >= filtered_mask.shape[0]:
        return filtered_mask

    structural_mask = filtered_mask.copy()
    structural_mask[structural_end_in_roi:, :] = 0
    return structural_mask


def create_line_candidate_mask(structural_mask, vision_profile):
    """Mantém somente contornos completos com espessura compatível com a fita."""

    full_contours, _ = cv2.findContours(
        structural_mask.copy(),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    accepted_contours = []
    minimum_short_side_px = (
        min(structural_mask.shape[:2])
        * vision_profile["full_line_min_short_side_ratio"]
    )
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
        if short_side_px >= minimum_short_side_px:
            accepted_contours.append(contour)

    line_candidate_mask = structural_mask.copy()
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


def frame_to_hsv(frame, camera_format="RGB888"):
    """Converte o array da câmera para HSV respeitando a ordem real dos canais."""

    if camera_format != "RGB888" or CAMERA_ARRAY_COLOR_ORDER != "BGR":
        raise ValueError("Formato ou ordem de canais não suportados para HSV.")
    return cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


def rgb_pixel_to_camera_array(red, green, blue):
    """Representa uma cor RGB na ordem BGR entregue por capture_array."""

    if CAMERA_ARRAY_COLOR_ORDER != "BGR":
        raise ValueError("Ordem de canais inesperada.")
    return int(blue), int(green), int(red)


def is_hsv_green(hue, saturation, value):
    """Aplica aos pixels sintéticos os mesmos limites usados por cv2.inRange."""

    return (
        GREEN_HUE_MIN <= int(hue) <= GREEN_HUE_MAX
        and int(saturation) >= GREEN_SATURATION_MIN
        and int(value) >= GREEN_VALUE_MIN
    )


def create_green_mask(frame, structural_end_y, camera_format="RGB888"):
    """Segmenta verde somente na área útil, sem tocar na máscara da linha."""

    useful_end_y = max(0, min(frame.shape[0], int(structural_end_y)))
    useful_frame = frame[:useful_end_y, :]
    hsv_frame = frame_to_hsv(useful_frame, camera_format)
    green_mask = cv2.inRange(
        hsv_frame,
        (GREEN_HUE_MIN, GREEN_SATURATION_MIN, GREEN_VALUE_MIN),
        (GREEN_HUE_MAX, 255, 255),
    )
    if cv2.countNonZero(green_mask) == 0:
        return green_mask
    open_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (GREEN_OPEN_KERNEL_SIZE, GREEN_OPEN_KERNEL_SIZE),
    )
    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (GREEN_CLOSE_KERNEL_SIZE, GREEN_CLOSE_KERNEL_SIZE),
    )
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, open_kernel)
    return cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, close_kernel)


def expanded_boxes_overlap(first_box, second_box, gap_px):
    """Indica se dois fragmentos podem pertencer à mesma marcação verde."""

    first_x, first_y, first_width, first_height = first_box
    second_x, second_y, second_width, second_height = second_box
    return not (
        first_x + first_width + gap_px < second_x
        or second_x + second_width + gap_px < first_x
        or first_y + first_height + gap_px < second_y
        or second_y + second_height + gap_px < first_y
    )


def group_fragment_boxes(boxes, gap_px=GREEN_FRAGMENT_MERGE_GAP_PX):
    """Agrupa caixas próximas de forma transitiva antes de unir os contornos."""

    groups = []
    for box_index, box in enumerate(boxes):
        matching_groups = []
        for group_index, group in enumerate(groups):
            if any(
                expanded_boxes_overlap(box, boxes[index], gap_px)
                for index in group
            ):
                matching_groups.append(group_index)
        merged_group = [box_index]
        for group_index in reversed(matching_groups):
            merged_group.extend(groups.pop(group_index))
        groups.append(merged_group)
    return groups


def merge_green_fragments(contours):
    """Une fragmentos próximos para que um quadrado não seja contado duas vezes."""

    valid_contours = [
        contour
        for contour in contours
        if contour is not None and len(contour) >= 3
    ]
    boxes = [cv2.boundingRect(contour) for contour in valid_contours]
    groups = group_fragment_boxes(boxes)
    return [
        cv2.convexHull(np.concatenate(
            [valid_contours[index] for index in group], axis=0
        ))
        for group in groups
    ]


def green_geometry_is_valid(
    area,
    short_side,
    aspect_ratio,
    extent,
    partial,
):
    """Aplica filtros amplos de tamanho e formato ao marcador oficial."""

    minimum_area = GREEN_MIN_AREA_PX
    minimum_dimension = GREEN_MIN_DIMENSION_PX
    minimum_aspect = GREEN_ASPECT_RATIO_MIN
    minimum_extent = GREEN_MIN_EXTENT
    if partial:
        minimum_area *= GREEN_PARTIAL_AREA_FACTOR
        minimum_dimension *= GREEN_PARTIAL_DIMENSION_FACTOR
        minimum_aspect = GREEN_PARTIAL_ASPECT_RATIO_MIN
        minimum_extent = GREEN_PARTIAL_EXTENT_MIN
    return (
        math.isfinite(float(area))
        and float(area) >= minimum_area
        and float(short_side) >= minimum_dimension
        and minimum_aspect <= float(aspect_ratio) <= GREEN_ASPECT_RATIO_MAX
        and float(extent) >= minimum_extent
    )


def estimate_local_line_width(line_mask, point):
    """Estima a largura da faixa no segmento horizontal mais próximo do ponto."""

    if line_mask is None or line_mask.size == 0:
        return 0.0
    point_x = int(round(point[0]))
    point_y = int(round(point[1]))
    point_y = max(0, min(line_mask.shape[0] - 1, point_y))
    active_x = np.flatnonzero(line_mask[point_y] > 0)
    if active_x.size == 0:
        return 0.0

    runs = []
    run_start = int(active_x[0])
    previous_x = run_start
    for active_pixel_x in active_x[1:]:
        active_pixel_x = int(active_pixel_x)
        if active_pixel_x != previous_x + 1:
            runs.append((run_start, previous_x))
            run_start = active_pixel_x
        previous_x = active_pixel_x
    runs.append((run_start, previous_x))

    selected_run = min(
        runs,
        key=lambda run: 0.0
        if run[0] <= point_x <= run[1]
        else min(abs(point_x - run[0]), abs(point_x - run[1])),
    )
    return float(selected_run[1] - selected_run[0] + 1)


def build_line_axis(near_center, far_center):
    """Cria o eixo local da linha no sentido físico de avanço após o hvflip."""

    if near_center is None or far_center is None:
        return {"valid": False}
    origin_x = float(near_center[0])
    origin_y = float(near_center[1])
    forward_x = float(far_center[0]) - origin_x
    forward_y = float(far_center[1]) - origin_y
    length = math.hypot(forward_x, forward_y)
    if not math.isfinite(length) or length < GREEN_LINE_AXIS_MIN_LENGTH_PX:
        return {"valid": False}

    forward_x /= length
    forward_y /= length
    # Com o topo da imagem apontando para a frente, este vetor normal positivo
    # aponta para a direita física do robô na imagem já transformada por hvflip.
    right_x = -forward_y
    right_y = forward_x
    return {
        "valid": True,
        "origin": (origin_x, origin_y),
        "forward": (forward_x, forward_y),
        "right": (right_x, right_y),
        "length": length,
    }


def project_point_on_line_axis(point, line_axis):
    """Retorna as coordenadas longitudinal e lateral no referencial da linha."""

    if not line_axis.get("valid", False):
        return 0.0, 0.0
    delta_x = float(point[0]) - line_axis["origin"][0]
    delta_y = float(point[1]) - line_axis["origin"][1]
    longitudinal = (
        delta_x * line_axis["forward"][0]
        + delta_y * line_axis["forward"][1]
    )
    lateral = (
        delta_x * line_axis["right"][0]
        + delta_y * line_axis["right"][1]
    )
    return longitudinal, lateral


def point_from_line_axis(line_axis, longitudinal, lateral=0.0):
    """Converte uma posição local da linha novamente para coordenadas da imagem."""

    return (
        line_axis["origin"][0]
        + longitudinal * line_axis["forward"][0]
        + lateral * line_axis["right"][0],
        line_axis["origin"][1]
        + longitudinal * line_axis["forward"][1]
        + lateral * line_axis["right"][1],
    )


def contour_touches_useful_border(box, frame_width, useful_height):
    """Marca candidatos parciais próximos de qualquer limite da área útil."""

    x, y, width, height = box
    tolerance = GREEN_PARTIAL_BORDER_TOLERANCE_PX
    return (
        x <= tolerance
        or y <= tolerance
        or x + width >= frame_width - tolerance
        or y + height >= useful_height - tolerance
    )


def describe_green_contour(
    contour,
    frame_width,
    useful_height,
    line_mask,
    line_axis,
    sampled_line_points=None,
):
    """Calcula geometria, relação com a faixa e posição local do candidato."""

    area = float(cv2.contourArea(contour))
    box = cv2.boundingRect(contour)
    rotated_rect = cv2.minAreaRect(contour)
    rect_width, rect_height = rotated_rect[1]
    short_side = min(float(rect_width), float(rect_height))
    long_side = max(float(rect_width), float(rect_height))
    aspect_ratio = short_side / long_side if long_side > 0.0 else 0.0
    box_area = float(box[2] * box[3])
    extent = area / box_area if box_area > 0.0 else 0.0
    moments = cv2.moments(contour)
    if moments["m00"] > 0.0:
        center_x = float(moments["m10"] / moments["m00"])
        center_y = float(moments["m01"] / moments["m00"])
    else:
        center_x = float(box[0] + box[2] / 2.0)
        center_y = float(box[1] + box[3] / 2.0)
    centroid = (center_x, center_y)
    partial = contour_touches_useful_border(
        box, frame_width, useful_height
    )

    geometry_valid = green_geometry_is_valid(
        area,
        short_side,
        aspect_ratio,
        extent,
        partial,
    )

    line_distance_px = 0.0
    if sampled_line_points is not None and sampled_line_points.size > 0:
        delta = sampled_line_points - np.array(
            (center_x, center_y), dtype=np.float32
        )
        squared_distance = np.sum(delta * delta, axis=1)
        line_distance_px = float(math.sqrt(float(np.min(squared_distance))))

    longitudinal, lateral = project_point_on_line_axis(centroid, line_axis)
    projected_point = (
        point_from_line_axis(line_axis, longitudinal)
        if line_axis.get("valid", False)
        else centroid
    )
    local_line_width_px = (
        estimate_local_line_width(line_mask, projected_point)
        if line_axis.get("valid", False)
        else 0.0
    )
    marker_to_line_ratio = (
        long_side / local_line_width_px if local_line_width_px > 0.0 else 0.0
    )
    scale_compatible = (
        local_line_width_px <= 0.0
        or GREEN_MARKER_TO_LINE_MIN_RATIO
        <= marker_to_line_ratio
        <= GREEN_MARKER_TO_LINE_MAX_RATIO
    )
    associated_with_line = (
        line_axis.get("valid", False)
        and local_line_width_px > 0.0
        and abs(lateral) >= GREEN_SIDE_MIN_DISTANCE_PX
        and line_distance_px
        <= GREEN_LINE_DISTANCE_MAX_RATIO * local_line_width_px
        and scale_compatible
    )

    return {
        "contour": contour,
        "area": area,
        "centroid": centroid,
        "bounding_box": box,
        "rotated_rect": rotated_rect,
        "short_side": short_side,
        "long_side": long_side,
        "aspect_ratio": aspect_ratio,
        "extent": extent,
        "partial": partial,
        "geometry_valid": geometry_valid,
        "line_distance_px": line_distance_px,
        "local_line_width_px": local_line_width_px,
        "marker_to_line_ratio": marker_to_line_ratio,
        "scale_compatible": scale_compatible,
        "associated_with_line": associated_with_line,
        "longitudinal": longitudinal,
        "lateral": lateral,
        "side": "DIREITA" if lateral > 0.0 else "ESQUERDA",
    }


def find_green_candidates(
    frame,
    structural_end_y,
    line_mask,
    line_axis,
    camera_format="RGB888",
):
    """Segmenta e separa candidatos geométricos de ruídos verdes rejeitados."""

    green_mask = create_green_mask(frame, structural_end_y, camera_format)
    contours, _ = cv2.findContours(
        green_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    merged_contours = merge_green_fragments(contours)
    sampled_line_points = np.empty((0, 2), dtype=np.float32)
    if merged_contours:
        sampled_y, sampled_x = np.nonzero(
            line_mask[
                ::GREEN_TOPOLOGY_SAMPLE_STEP,
                ::GREEN_TOPOLOGY_SAMPLE_STEP,
            ]
        )
        sampled_line_points = np.column_stack((
            sampled_x * GREEN_TOPOLOGY_SAMPLE_STEP,
            sampled_y * GREEN_TOPOLOGY_SAMPLE_STEP,
        )).astype(np.float32)

    candidates = []
    rejected = []
    for contour in merged_contours:
        description = describe_green_contour(
            contour,
            frame.shape[1],
            green_mask.shape[0],
            line_mask,
            line_axis,
            sampled_line_points,
        )
        if description["geometry_valid"]:
            candidates.append(description)
        else:
            rejected.append(description)
    candidates.sort(key=lambda candidate: candidate["area"], reverse=True)
    return green_mask, candidates, rejected


def analyze_line_topology(line_mask, line_axis, reference_line_width_px):
    """Observa encontro, continuação e ramificações com evidências graduais."""

    empty_result = {
        "entry_valid": False,
        "junction_valid": False,
        "junction": (0.0, 0.0),
        "junction_longitudinal": 0.0,
        "forward_branch": False,
        "left_branch": False,
        "right_branch": False,
        "line_termination": False,
        "confidence": 0.0,
        "left_evidence": 0.0,
        "right_evidence": 0.0,
        "forward_evidence": 0.0,
    }
    if (
        line_mask is None
        or line_mask.size == 0
        or not line_axis.get("valid", False)
    ):
        return empty_result

    topology_mask = line_mask[
        ::GREEN_TOPOLOGY_SAMPLE_STEP,
        ::GREEN_TOPOLOGY_SAMPLE_STEP,
    ]
    active_y, active_x = np.nonzero(topology_mask)
    active_y *= GREEN_TOPOLOGY_SAMPLE_STEP
    active_x *= GREEN_TOPOLOGY_SAMPLE_STEP
    if active_x.size < 20:
        return empty_result
    delta_x = active_x.astype(np.float32) - line_axis["origin"][0]
    delta_y = active_y.astype(np.float32) - line_axis["origin"][1]
    longitudinal = (
        delta_x * line_axis["forward"][0]
        + delta_y * line_axis["forward"][1]
    )
    lateral = (
        delta_x * line_axis["right"][0]
        + delta_y * line_axis["right"][1]
    )

    line_width = max(8.0, float(reference_line_width_px))
    center_limit = line_width * 0.75
    branch_limit = line_width * 1.35
    sample_area_factor = GREEN_TOPOLOGY_SAMPLE_STEP ** 2
    branch_min_pixels = max(
        5,
        int(line_width * line_width * 0.35 / sample_area_factor),
    )
    center_pixels = np.abs(lateral) <= center_limit
    entry_pixels = center_pixels & (
        (longitudinal >= -line_width * 2.0)
        & (longitudinal <= line_width * 1.5)
    )
    entry_count = int(np.count_nonzero(entry_pixels))
    entry_valid = entry_count >= branch_min_pixels

    # Uma ramificação deve se afastar lateralmente em uma faixa longitudinal
    # concentrada. Isso evita confundir uma curva longa com uma interseção.
    left_pixels = (lateral <= -branch_limit) & (longitudinal >= line_width)
    right_pixels = (lateral >= branch_limit) & (longitudinal >= line_width)
    left_count = int(np.count_nonzero(left_pixels))
    right_count = int(np.count_nonzero(right_pixels))
    left_evidence = min(1.0, left_count / max(1.0, branch_min_pixels * 2.0))
    right_evidence = min(1.0, right_count / max(1.0, branch_min_pixels * 2.0))
    def concentrated_branch(side_pixels):
        if int(np.count_nonzero(side_pixels)) < branch_min_pixels:
            return False
        branch_longitudinal = longitudinal[side_pixels]
        branch_lateral = np.abs(lateral[side_pixels])
        longitudinal_spread = float(
            np.percentile(branch_longitudinal, 90)
            - np.percentile(branch_longitudinal, 10)
        )
        lateral_reach = float(np.max(branch_lateral) - branch_limit)
        return (
            longitudinal_spread <= line_width * 3.0
            and lateral_reach >= line_width * 1.5
        )

    left_branch = concentrated_branch(left_pixels)
    right_branch = concentrated_branch(right_pixels)
    side_structure_ambiguous = (
        (left_count >= branch_min_pixels and not left_branch)
        or (right_count >= branch_min_pixels and not right_branch)
    )

    junction_longitudinal = 0.0
    line_termination = False
    junction_valid = False
    if left_branch or right_branch:
        side_longitudinal = longitudinal[left_pixels | right_pixels]
        junction_longitudinal = float(np.median(side_longitudinal))
        junction_valid = entry_valid
    elif np.any(center_pixels) and not side_structure_ambiguous:
        junction_longitudinal = float(np.max(longitudinal[center_pixels]))
        junction_point = point_from_line_axis(line_axis, junction_longitudinal)
        border_margin = line_width + GREEN_PARTIAL_BORDER_TOLERANCE_PX
        line_termination = (
            border_margin < junction_point[0] < line_mask.shape[1] - border_margin
            and border_margin
            < junction_point[1]
            < line_mask.shape[0] - border_margin
        )
        junction_valid = entry_valid and line_termination

    forward_count = 0
    if junction_valid:
        forward_pixels = center_pixels & (
            longitudinal >= junction_longitudinal + line_width * 1.5
        )
        forward_count = int(np.count_nonzero(forward_pixels))
    forward_evidence = min(
        1.0, forward_count / max(1.0, branch_min_pixels * 2.0)
    )
    forward_branch = forward_count >= branch_min_pixels
    junction = (
        point_from_line_axis(line_axis, junction_longitudinal)
        if junction_valid
        else (0.0, 0.0)
    )
    confidence = 0.0
    if junction_valid:
        strongest_branch = max(left_evidence, right_evidence, forward_evidence)
        confidence = min(1.0, 0.60 + strongest_branch * 0.40)

    return {
        "entry_valid": entry_valid,
        "junction_valid": junction_valid,
        "junction": junction,
        "junction_longitudinal": junction_longitudinal,
        "forward_branch": forward_branch,
        "left_branch": left_branch,
        "right_branch": right_branch,
        "line_termination": line_termination,
        "confidence": confidence,
        "left_evidence": left_evidence,
        "right_evidence": right_evidence,
        "forward_evidence": forward_evidence,
    }


def green_observation_state(candidate_count):
    """Converte a quantidade de marcações geométricas no estado de observação."""

    if candidate_count <= 0:
        return "SEM_VERDE"
    if candidate_count == 1:
        return "UM_CANDIDATO"
    if candidate_count == 2:
        return "DOIS_CANDIDATOS"
    return "MULTIPLOS_AMBIGUOS"


def interpret_green_candidates(candidates, line_axis, topology):
    """Interpreta candidatos priorizando retorno, direção, falso e ambiguidade."""

    candidate_count = len(candidates)
    observation_state = green_observation_state(candidate_count)
    result = {
        "observation_state": observation_state,
        "interpretation": "SEM_DECISAO",
        "left_seen": False,
        "right_seen": False,
        "pair_compatible": False,
    }
    if candidate_count == 0:
        return result
    if candidate_count > 2 or not line_axis.get("valid", False):
        result["interpretation"] = "AMBIGUO"
        return result

    for candidate in candidates:
        if candidate["side"] == "ESQUERDA":
            result["left_seen"] = True
        else:
            result["right_seen"] = True

    line_widths = [
        candidate["local_line_width_px"]
        for candidate in candidates
        if candidate["local_line_width_px"] > 0.0
    ]
    reference_line_width = max(8.0, sum(line_widths) / len(line_widths)) \
        if line_widths else 8.0

    # O retorno de 180° tem prioridade e não depende da existência de saídas.
    if candidate_count == 2:
        first, second = candidates
        opposite_sides = first["side"] != second["side"]
        longitudinal_compatible = (
            abs(first["longitudinal"] - second["longitudinal"])
            <= GREEN_PAIR_LONGITUDINAL_TOLERANCE_LINE_WIDTHS
            * reference_line_width
        )
        before_same_encounter = False
        if topology.get("junction_valid", False):
            junction_longitudinal = topology["junction_longitudinal"]
            before_same_encounter = all(
                candidate["longitudinal"] < junction_longitudinal
                and junction_longitudinal - candidate["longitudinal"]
                <= GREEN_ENCOUNTER_DISTANCE_LINE_WIDTHS * reference_line_width
                for candidate in candidates
            )
        pair_compatible = (
            opposite_sides
            and longitudinal_compatible
            and before_same_encounter
            and all(
                candidate["associated_with_line"] and not candidate["partial"]
                for candidate in candidates
            )
        )
        result["pair_compatible"] = pair_compatible
        result["interpretation"] = (
            "RETORNO_180" if pair_compatible else "AMBIGUO"
        )
        return result

    candidate = candidates[0]
    if candidate["partial"] or not candidate["associated_with_line"]:
        result["interpretation"] = "AMBIGUO"
        return result
    if (
        not topology.get("junction_valid", False)
        or topology.get("confidence", 0.0) < 0.50
    ):
        result["interpretation"] = "AMBIGUO"
        return result

    before_junction = (
        candidate["longitudinal"] < topology["junction_longitudinal"]
    )
    if not before_junction:
        result["interpretation"] = "VERDE_FALSO_NO_SENTIDO_ATUAL"
        return result

    matching_branch = (
        topology["left_branch"]
        if candidate["side"] == "ESQUERDA"
        else topology["right_branch"]
    )
    result["interpretation"] = (
        candidate["side"]
        if matching_branch
        else "VERDE_FALSO_NO_SENTIDO_ATUAL"
    )
    return result


def empty_green_status():
    """Cria um estado verde finito e seguro para publicação diagnóstica."""

    return {
        "greenObservationState": "SEM_VERDE",
        "greenInterpretation": "SEM_DECISAO",
        "greenConfirmed": False,
        "greenCandidateCount": 0,
        "greenRejectedCount": 0,
        "greenLeftSeen": False,
        "greenRightSeen": False,
        "greenPairCompatible": False,
        "greenJunctionValid": False,
        "greenJunctionX": 0.0,
        "greenJunctionY": 0.0,
        "greenForwardBranch": False,
        "greenLeftBranch": False,
        "greenRightBranch": False,
        "greenPrimaryX": 0.0,
        "greenPrimaryY": 0.0,
        "greenPrimaryArea": 0.0,
        "greenSecondaryX": 0.0,
        "greenSecondaryY": 0.0,
        "greenSecondaryArea": 0.0,
        "greenConsecutiveSamples": 0,
        "greenProcessingMs": 0.0,
    }


class GreenObservationTracker:
    """Confirma observações novas e remove decisões após curta histerese."""

    def __init__(self):
        self.last_sequence = None
        self.pending_interpretation = "SEM_DECISAO"
        self.consecutive_samples = 0
        self.missing_samples = 0
        self.confirmed_interpretation = "SEM_DECISAO"

    def update(self, line_sequence, interpretation):
        if line_sequence == self.last_sequence:
            return (
                self.confirmed_interpretation,
                self.confirmed_interpretation != "SEM_DECISAO",
                self.consecutive_samples,
            )
        self.last_sequence = line_sequence

        if interpretation == "SEM_DECISAO":
            self.missing_samples += 1
            if self.missing_samples >= GREEN_CLEAR_HYSTERESIS_FRAMES:
                self.pending_interpretation = "SEM_DECISAO"
                self.confirmed_interpretation = "SEM_DECISAO"
                self.consecutive_samples = 0
            return (
                self.confirmed_interpretation,
                self.confirmed_interpretation != "SEM_DECISAO",
                self.consecutive_samples,
            )

        self.missing_samples = 0
        if interpretation != self.pending_interpretation:
            self.pending_interpretation = interpretation
            self.consecutive_samples = 1
            self.confirmed_interpretation = "SEM_DECISAO"
        else:
            self.consecutive_samples += 1

        required_samples = GREEN_CONFIRMATION_FRAMES
        if interpretation in ("ESQUERDA", "DIREITA"):
            required_samples = max(
                required_samples, GREEN_SINGLE_OBSERVATION_FRAMES
            )
        confirmable = interpretation not in ("AMBIGUO", "SEM_DECISAO")
        if confirmable and self.consecutive_samples >= required_samples:
            self.confirmed_interpretation = interpretation

        published_interpretation = self.confirmed_interpretation
        if interpretation == "AMBIGUO":
            published_interpretation = "AMBIGUO"
        return (
            published_interpretation,
            self.confirmed_interpretation != "SEM_DECISAO",
            self.consecutive_samples,
        )


def build_green_status(
    candidates,
    rejected_count,
    interpretation_result,
    topology,
    tracker_result,
    processing_ms,
):
    """Monta os campos diagnósticos sem permitir NaN no JSON rápido."""

    status = empty_green_status()
    published_interpretation, confirmed, consecutive_samples = tracker_result
    status.update({
        "greenObservationState": interpretation_result["observation_state"],
        "greenInterpretation": published_interpretation,
        "greenConfirmed": confirmed,
        "greenCandidateCount": len(candidates),
        "greenRejectedCount": int(rejected_count),
        "greenLeftSeen": interpretation_result["left_seen"],
        "greenRightSeen": interpretation_result["right_seen"],
        "greenPairCompatible": interpretation_result["pair_compatible"],
        "greenJunctionValid": bool(topology.get("junction_valid", False)),
        "greenForwardBranch": bool(topology.get("forward_branch", False)),
        "greenLeftBranch": bool(topology.get("left_branch", False)),
        "greenRightBranch": bool(topology.get("right_branch", False)),
        "greenConsecutiveSamples": int(consecutive_samples),
        "greenProcessingMs": float(processing_ms),
    })
    if status["greenJunctionValid"]:
        status["greenJunctionX"] = float(topology["junction"][0])
        status["greenJunctionY"] = float(topology["junction"][1])
    for prefix, candidate in zip(("greenPrimary", "greenSecondary"), candidates):
        status[f"{prefix}X"] = float(candidate["centroid"][0])
        status[f"{prefix}Y"] = float(candidate["centroid"][1])
        status[f"{prefix}Area"] = float(candidate["area"])

    for key, value in tuple(status.items()):
        if isinstance(value, float) and not math.isfinite(value):
            status[key] = 0.0
    if status["greenObservationState"] not in GREEN_OBSERVATION_STATES:
        status["greenObservationState"] = "SEM_VERDE"
    if status["greenInterpretation"] not in GREEN_INTERPRETATIONS:
        status["greenInterpretation"] = "SEM_DECISAO"
        status["greenConfirmed"] = False
    return status


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
    green_status=None,
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
        line_status.update(green_status or empty_green_status())
        with open(TEMP_LINE_STATUS_PATH, "w", encoding="utf-8") as status_file:
            json.dump(line_status, status_file, allow_nan=False)
        os.replace(TEMP_LINE_STATUS_PATH, LINE_STATUS_PATH)
    except (OSError, TypeError, ValueError) as error:
        print(f"Falha ao publicar telemetria rápida da linha: {error}", flush=True)


def save_status(
    fps,
    camera_profile,
    camera_details,
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

    frame_width, frame_height = camera_profile["main_size"]
    status = {
        "fps": round(fps, 2),
        "active": active,
        "cameraRole": camera_profile["role"],
        "width": frame_width,
        "height": frame_height,
        "mainResolution": {
            "width": frame_width,
            "height": frame_height,
        },
        "jpegQuality": JPEG_QUALITY,
        "targetCameraFps": camera_profile["target_fps"],
        "cameraFormat": camera_format,
        "rotationDegrees": CAMERA_ROTATION_DEGREES,
        "cameraId": camera_details.get("cameraId", ""),
        "cameraModel": camera_details.get("cameraModel", ""),
        "sensorMode": camera_details.get("sensorMode"),
        "scalerCrop": camera_details.get("scalerCrop"),
        "transform": camera_details.get("transform", "hvflip"),
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
    camera_profile = parse_camera_profile()
    vision_profile = camera_profile["vision"]
    camera_details = {}

    if GPIO is None or Picamera2 is None or Transform is None:
        error_message = "Dependências GPIO, libcamera ou Picamera2 não encontradas."
        print(error_message, flush=True)
        save_status(
            0.0,
            camera_profile,
            camera_details,
            active=False,
            error_message=error_message,
        )
        return 1

    save_status(0.0, camera_profile, camera_details)
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
        picam2, camera_format, camera_details = create_camera(camera_profile)
        picam2.start()
        tune_camera_image(picam2)
        capture_metadata = picam2.capture_metadata()
        camera_details["scalerCrop"] = rectangle_values(
            capture_metadata["ScalerCrop"]
        )
        print(
            f"Câmera id={camera_details['cameraId']} "
            f"modelo={camera_details['cameraModel']} "
            f"transformação={camera_details['transform']} "
            f"ScalerCrop={camera_details['scalerCrop']}.",
            flush=True,
        )
        camera_started = True

        previous_time = time.monotonic()
        last_stream_time = 0.0
        last_snapshot_time = 0.0
        last_status_time = 0.0
        smoothed_fps = 0.0
        line_sequence = 0
        green_tracker = GreenObservationTracker()

        while running:
            frame = picam2.capture_array()
            frame_height = frame.shape[0]
            vision_geometry = resolve_vision_geometry(
                frame_height,
                vision_profile,
            )
            filtered_mask, roi_start_y = create_filtered_line_mask(
                frame,
                vision_profile,
            )
            structural_mask = create_structural_line_mask(
                filtered_mask,
                roi_start_y,
                vision_geometry["structural_end_y"],
            )
            line_candidate_mask = create_line_candidate_mask(
                structural_mask,
                vision_profile,
            )

            far_band_start_y = vision_geometry["far_band_start_y"]
            far_band_end_y = vision_geometry["far_band_end_y"]
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

            near_band_start_y = vision_geometry["near_band_start_y"]
            near_band_end_y = vision_geometry["near_band_end_y"]
            near_band_start_in_roi = near_band_start_y - roi_start_y
            near_band_end_in_roi = near_band_end_y - roi_start_y
            near_band = line_candidate_mask[
                near_band_start_in_roi:near_band_end_in_roi,
                :,
            ]
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

            line_axis = build_line_axis(near_center, far_center)
            green_candidates = []
            green_rejected = []
            green_topology = analyze_line_topology(None, line_axis, 0.0)
            green_interpretation = interpret_green_candidates(
                green_candidates,
                line_axis,
                green_topology,
            )
            green_processing_started = time.perf_counter()
            if vision_profile.get("green_detection_enabled", False):
                _, green_candidates, green_rejected = find_green_candidates(
                    frame,
                    vision_geometry["structural_end_y"],
                    line_candidate_mask,
                    line_axis,
                    camera_format,
                )
                # A topologia é a parte mais cara e só roda quando a cor e a
                # geometria já produziram pelo menos um candidato plausível.
                if green_candidates:
                    reference_center = near_center or far_center
                    reference_line_width_px = (
                        estimate_local_line_width(
                            line_candidate_mask,
                            reference_center,
                        )
                        if reference_center is not None
                        else 0.0
                    )
                    green_topology = analyze_line_topology(
                        line_candidate_mask,
                        line_axis,
                        reference_line_width_px,
                    )
                green_interpretation = interpret_green_candidates(
                    green_candidates,
                    line_axis,
                    green_topology,
                )
            green_processing_ms = (
                time.perf_counter() - green_processing_started
            ) * 1000.0

            frame_width = frame.shape[1]
            frame_center_x = frame_width / 2.0
            (
                safe_half_width_px,
                safe_left_x,
                safe_right_x,
            ) = resolve_horizontal_deadzone(
                frame_width,
                vision_profile,
            )

            (
                _guidance_error,
                control_error,
                correction,
                left_preview,
                right_preview,
            ) = calculate_control_preview(
                vision_profile,
                near_valid,
                near_error,
                far_valid,
                far_error,
            )

            if not near_valid:
                offset_px = None
                preview_state = "LINHA INVALIDA"
                preview_direction = "SEM COMANDO"
            elif control_error == 0.0:
                offset_px = near_error * (frame_width / 2.0)
                preview_state = "RETO SEGURO"
                preview_direction = "RETO"
            else:
                offset_px = near_error * (frame_width / 2.0)
                if correction > 0.0:
                    preview_state = "CORRIGINDO DIREITA"
                    preview_direction = "DIREITA"
                elif correction < 0.0:
                    preview_state = "CORRIGINDO ESQUERDA"
                    preview_direction = "ESQUERDA"
                else:
                    preview_state = "RETO"
                    preview_direction = "RETO"

            line_timestamp = time.time()
            line_sequence += 1
            green_tracker_result = green_tracker.update(
                line_sequence,
                green_interpretation["interpretation"],
            )
            green_status = build_green_status(
                green_candidates,
                len(green_rejected),
                green_interpretation,
                green_topology,
                green_tracker_result,
                green_processing_ms,
            )
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
                green_status,
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
                near_band_start_y:near_band_end_y,
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
                vision_profile["overlay_line_thickness"],
            )
            # A banda AHEAD/FAR e seu centro são referências em laranja.
            cv2.rectangle(
                frame,
                (0, far_band_start_y),
                (frame.shape[1] - 1, far_band_end_y - 1),
                (0, 165, 255),
                vision_profile["overlay_line_thickness"],
            )
            # A banda PIVOT/NEAR e seu centro são referências em azul.
            cv2.rectangle(
                frame,
                (0, near_band_start_y),
                (frame.shape[1] - 1, near_band_end_y - 1),
                (255, 0, 0),
                vision_profile["overlay_line_thickness"],
            )

            ignored_start_y = vision_geometry["ignored_start_y"]
            if ignored_start_y is not None:
                cv2.line(
                    frame,
                    (0, ignored_start_y),
                    (frame.shape[1] - 1, ignored_start_y),
                    (0, 0, 255),
                    vision_profile["overlay_thin_line_thickness"],
                )

            if not near_valid:
                safe_limit_color = (0, 0, 255)
            elif abs(near_error) > vision_profile["near_deadzone_ratio"]:
                safe_limit_color = (0, 255, 255)
            else:
                safe_limit_color = (0, 255, 0)
            cv2.line(
                frame,
                (int(round(frame_center_x)), near_band_start_y),
                (int(round(frame_center_x)), near_band_end_y - 1),
                (255, 255, 0),
                vision_profile["overlay_line_thickness"],
            )
            for safe_limit_x in (safe_left_x, safe_right_x):
                cv2.line(
                    frame,
                    (safe_limit_x, near_band_start_y),
                    (safe_limit_x, near_band_end_y - 1),
                    safe_limit_color,
                    vision_profile["overlay_line_thickness"],
                )

            cv2.line(
                frame,
                (0, measurement_y_frame),
                (frame_width - 1, measurement_y_frame),
                (180, 180, 180),
                vision_profile["overlay_thin_line_thickness"],
            )
            if line_left_x is not None and line_right_x is not None:
                cv2.line(
                    frame,
                    (line_left_x, measurement_y_frame),
                    (line_right_x, measurement_y_frame),
                    (255, 0, 255),
                    vision_profile["overlay_line_thickness"],
                )
                cv2.circle(
                    frame,
                    (line_left_x, measurement_y_frame),
                    vision_profile["overlay_measurement_radius"],
                    (255, 0, 255),
                    -1,
                )
                cv2.circle(
                    frame,
                    (line_right_x, measurement_y_frame),
                    vision_profile["overlay_measurement_radius"],
                    (255, 0, 255),
                    -1,
                )

            ruler_step_px = max(
                1,
                int(round(frame_width * vision_profile["pixel_ruler_step_ratio"])),
            )
            ruler_bottom_y = near_band_end_y - 3
            first_ruler_offset = -(
                int(frame_center_x) // ruler_step_px
            ) * ruler_step_px
            for ruler_offset in range(
                first_ruler_offset,
                frame_width,
                ruler_step_px,
            ):
                ruler_x = int(round(frame_center_x + ruler_offset))
                if ruler_x < 0 or ruler_x >= frame_width:
                    continue
                labeled_tick = ruler_offset % (ruler_step_px * 2) == 0
                tick_height = 11 if labeled_tick else 6
                cv2.line(
                    frame,
                    (ruler_x, ruler_bottom_y),
                    (ruler_x, ruler_bottom_y - tick_height),
                    (255, 255, 255),
                    vision_profile["overlay_thin_line_thickness"],
                )
                if labeled_tick:
                    ruler_label = (
                        f"+{ruler_offset}" if ruler_offset > 0 else str(ruler_offset)
                    )
                    put_debug_text(
                        vision_profile["debug_text_overlay"],
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
                cv2.circle(
                    frame,
                    near_center,
                    vision_profile["overlay_center_radius"],
                    (255, 0, 0),
                    -1,
                )
            if far_center is not None:
                cv2.circle(
                    frame,
                    far_center,
                    vision_profile["overlay_center_radius"],
                    (0, 165, 255),
                    -1,
                )

            if vision_profile.get("green_detection_enabled", False):
                for rejected_candidate in green_rejected:
                    cv2.drawContours(
                        frame,
                        [rejected_candidate["contour"]],
                        -1,
                        (180, 80, 180),
                        1,
                    )

                interpretation = green_status["greenInterpretation"]
                for candidate in green_candidates:
                    cv2.drawContours(
                        frame,
                        [candidate["contour"]],
                        -1,
                        (0, 255, 0),
                        2,
                    )
                    center = (
                        int(round(candidate["centroid"][0])),
                        int(round(candidate["centroid"][1])),
                    )
                    cv2.circle(frame, center, 3, (0, 255, 0), -1)
                    if interpretation == "RETORNO_180":
                        candidate_letter = "R"
                    elif interpretation == "VERDE_FALSO_NO_SENTIDO_ATUAL":
                        candidate_letter = "F"
                    elif interpretation == "AMBIGUO":
                        candidate_letter = "?"
                    else:
                        candidate_letter = (
                            "E" if candidate["side"] == "ESQUERDA" else "D"
                        )
                    cv2.putText(
                        frame,
                        candidate_letter,
                        (center[0] + 5, max(12, center[1] - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        (0, 255, 0),
                        1,
                        cv2.LINE_AA,
                    )

                if line_axis.get("valid", False):
                    axis_start = point_from_line_axis(line_axis, -30.0)
                    axis_end = point_from_line_axis(
                        line_axis,
                        float(vision_geometry["structural_end_y"]),
                    )
                    cv2.line(
                        frame,
                        tuple(int(round(value)) for value in axis_start),
                        tuple(int(round(value)) for value in axis_end),
                        (255, 255, 0),
                        1,
                        cv2.LINE_AA,
                    )

                if green_topology.get("junction_valid", False):
                    junction = tuple(
                        int(round(value))
                        for value in green_topology["junction"]
                    )
                    cv2.circle(frame, junction, 4, (0, 255, 255), -1)
                    branch_length = 28.0
                    branch_directions = []
                    if green_topology["forward_branch"]:
                        branch_directions.append(line_axis["forward"])
                    if green_topology["left_branch"]:
                        branch_directions.append((
                            -line_axis["right"][0],
                            -line_axis["right"][1],
                        ))
                    if green_topology["right_branch"]:
                        branch_directions.append(line_axis["right"])
                    for direction_x, direction_y in branch_directions:
                        branch_end = (
                            int(round(junction[0] + direction_x * branch_length)),
                            int(round(junction[1] + direction_y * branch_length)),
                        )
                        cv2.line(
                            frame,
                            junction,
                            branch_end,
                            (0, 255, 255),
                            2,
                            cv2.LINE_AA,
                        )

                green_text = {
                    "SEM_DECISAO": "SEM DECISAO",
                    "ESQUERDA": "ESQUERDA",
                    "DIREITA": "DIREITA",
                    "RETORNO_180": "RETORNO 180 GRAUS",
                    "VERDE_FALSO_NO_SENTIDO_ATUAL": "FALSO NO SENTIDO ATUAL",
                    "AMBIGUO": "AMBIGUO",
                }[interpretation]
                cv2.putText(
                    frame,
                    f"VERDE: {green_text}",
                    (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.48,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )

            active_pixel_count = cv2.countNonZero(filtered_mask)
            debug_lines = (
                "MASCARA EXPERIMENTAL",
                f"THRESHOLD: {vision_profile['line_threshold']}",
                f"PIXELS ATIVOS: {active_pixel_count}",
            )
            for index, debug_text in enumerate(debug_lines):
                put_debug_text(
                    vision_profile["debug_text_overlay"],
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
                    vision_profile["debug_text_overlay"],
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
                    vision_profile["debug_text_overlay"],
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
                    vision_profile["debug_text_overlay"],
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
                    vision_profile["debug_text_overlay"],
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
                    vision_profile["debug_text_overlay"],
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
                vision_profile["debug_text_overlay"],
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
                arrow_start = (
                    int(round(
                        frame_width
                        * vision_profile["overlay_arrow_start_ratio"][0]
                    )),
                    int(round(
                        frame_height
                        * vision_profile["overlay_arrow_start_ratio"][1]
                    )),
                )
                arrow_horizontal_length = int(round(
                    frame_width * vision_profile["overlay_arrow_horizontal_ratio"]
                ))
                arrow_vertical_length = int(round(
                    frame_height * vision_profile["overlay_arrow_vertical_ratio"]
                ))
                if preview_direction == "ESQUERDA":
                    arrow_end = (
                        arrow_start[0] - arrow_horizontal_length,
                        arrow_start[1],
                    )
                elif preview_direction == "DIREITA":
                    arrow_end = (
                        arrow_start[0] + arrow_horizontal_length,
                        arrow_start[1],
                    )
                else:
                    arrow_end = (
                        arrow_start[0],
                        arrow_start[1] - arrow_vertical_length,
                    )
                cv2.arrowedLine(
                    frame,
                    arrow_start,
                    arrow_end,
                    (0, 255, 255),
                    vision_profile["overlay_arrow_thickness"],
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
                    camera_profile,
                    camera_details,
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
        save_status(
            0.0,
            camera_profile,
            camera_details,
            active=False,
            error_message=error_message,
        )
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
