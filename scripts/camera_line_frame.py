"""Publica no dashboard a imagem da câmera selecionada.

A segmentação experimental destaca a linha preta apenas na imagem de debug.
Ela não calcula comandos nem interfere no controle do robô.
"""

import argparse
import csv
import json
import math
import os
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

import cv2  # type: ignore
import numpy as np

try:
    import RPi.GPIO as GPIO  # type: ignore
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
GREEN_CAPTURE_REQUEST_PATH = "/dev/shm/obr_green_capture_request"
GREEN_CAPTURE_RAW_PATH = "/dev/shm/obr_green_raw.png"
GREEN_CAPTURE_HSV_MASK_PATH = "/dev/shm/obr_green_hsv_mask.png"
GREEN_CAPTURE_FINAL_MASK_PATH = "/dev/shm/obr_green_final_mask.png"
GREEN_CAPTURE_CANDIDATES_PATH = "/dev/shm/obr_green_candidates.png"
GREEN_CAPTURE_STATS_PATH = "/dev/shm/obr_green_stats.json"
LINE_TRACE_REQUEST_PATH = "/dev/shm/obr_line_trace_request"
VISION_TRACE_SAMPLE_PATH = "/dev/shm/obr_line_trace_vision_sample.csv"
TEMP_VISION_TRACE_SAMPLE_PATH = "/dev/shm/obr_line_trace_vision_sample.tmp.csv"

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
DEFAULT_CAMERA_INDICES = {
    "down": 0,
    "forward": 1,
}
CAMERA_INDEX_ENVIRONMENT = {
    "down": "OBR_DOWNWARD_CAMERA_INDEX",
    "forward": "OBR_FORWARD_CAMERA_INDEX",
}
DISPLAY_MODE_REAL = "real"
DISPLAY_MODE_LINE = "line"
DISPLAY_MODE_GREEN = "green"
DISPLAY_MODES = (
    DISPLAY_MODE_REAL,
    DISPLAY_MODE_LINE,
)


def environment_flag(name, default):
    """Lê uma flag booleana de ambiente sem aceitar valores ambíguos."""

    value = os.environ.get(name)
    if value is None:
        return bool(default)
    normalized = value.strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise ValueError(f"{name} deve ser 0/1, true/false, yes/no ou on/off.")


def configured_camera_indices(environment=None):
    """Lê os dois índices físicos sem permitir fallback ou papéis duplicados."""

    environment = os.environ if environment is None else environment
    indices = {}
    for role, variable_name in CAMERA_INDEX_ENVIRONMENT.items():
        raw_value = environment.get(
            variable_name,
            str(DEFAULT_CAMERA_INDICES[role]),
        )
        try:
            camera_index = int(raw_value)
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"{variable_name} deve ser um índice inteiro não negativo."
            ) from error
        if camera_index < 0:
            raise ValueError(
                f"{variable_name} deve ser um índice inteiro não negativo."
            )
        indices[role] = camera_index

    if indices["down"] == indices["forward"]:
        raise ValueError(
            "As câmeras inferior e frontal não podem usar o mesmo índice."
        )
    return indices


def camera_number(camera_info, fallback_index):
    """Obtém o número enumerado sem deduzir o papel pelo modelo do sensor."""

    value = camera_info.get("Num", fallback_index)
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(fallback_index)


def resolve_camera_assignments(camera_infos, camera_indices):
    """Associa índices configurados aos papéis sem trocar câmeras ausentes."""

    enumerated = {
        camera_number(camera_info, fallback_index): camera_info
        for fallback_index, camera_info in enumerate(camera_infos)
    }
    return {
        role: {
            "role": role,
            "index": camera_index,
            "available": camera_index in enumerated,
            "info": enumerated.get(camera_index),
        }
        for role, camera_index in camera_indices.items()
    }


def require_camera_assignment(assignments, role):
    """Falha claramente se a câmera exigida para o papel não foi enumerada."""

    assignment = assignments[role]
    if not assignment["available"]:
        raise RuntimeError(
            f"Câmera {role} configurada no índice {assignment['index']} "
            "não foi encontrada; não haverá troca automática de papel."
        )
    return assignment


def camera_role_publishes_line_status(role):
    """Restringe o IPC do segue-faixa ao papel físico da câmera inferior."""

    return role == "down"


def log_camera_inventory(camera_infos, assignments):
    """Registra inventário, identificador e papel configurado de cada câmera."""

    if not camera_infos:
        print("Nenhuma câmera foi enumerada pelo Picamera2.", flush=True)
    for fallback_index, camera_info in enumerate(camera_infos):
        camera_index = camera_number(camera_info, fallback_index)
        print(
            "Câmera enumerada: "
            f"índice={camera_index}, modelo={camera_info.get('Model', '')}, "
            f"identificador={camera_info.get('Id', '')}, "
            f"localização={camera_info.get('Location', '')}, "
            f"rotação={camera_info.get('Rotation', '')}.",
            flush=True,
        )
    for role in ("down", "forward"):
        assignment = assignments[role]
        availability = "disponível" if assignment["available"] else "indisponível"
        print(
            f"Papel configurado: {role}=índice {assignment['index']} "
            f"({availability}).",
            flush=True,
        )


# A missão usa os marcadores verdes para decidir curvas e retorno.
# Estas chaves permanecem separadas para permitir diagnósticos sem movimento.
GREEN_PROCESSING_ENABLED = environment_flag("GREEN_PROCESSING_ENABLED", True)
GREEN_DECISIONS_ENABLED = environment_flag("GREEN_DECISIONS_ENABLED", True)


def resolve_green_experiment_mode(
    profile_enabled,
    processing_enabled,
    decisions_enabled,
):
    """Resolve os modos A/B/C sem permitir decisão quando não há processamento."""

    effective_processing = bool(profile_enabled and processing_enabled)
    effective_decisions = bool(effective_processing and decisions_enabled)
    if not effective_processing:
        return "C", False, False
    if not effective_decisions:
        return "B", True, False
    return "A", True, True

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
# Distância lateral máxima entre os centros NEAR e FAR para considerar que o
# preto superior ainda pertence ao mesmo encontro, em pixels da imagem 640x480.
GREEN_PATH_MAX_CENTER_DELTA_PX = 160.0
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
                # As duas bandas foram deslocadas 80 px para cima. Suas
                # alturas e a distância de 30 px entre elas permanecem iguais.
                "far_band_y": (0, 100),
                "near_band_y": (130, 190),
            },
            "ahead_heading_gain": 0.90,
            "near_deadzone_ratio": 0.10,
            "gap_detection_enabled": True,
            # A extremidade pode aparecer desde a FAR até dentro da NEAR, como
            # ocorre no enquadramento físico atual. As margens evitam usar as
            # bordas das duas regiões, onde a classificação fica instável.
            "gap_endpoint_start_ratio": 0.10,
            "gap_endpoint_end_ratio": 0.95,
            # O alinhamento compara o quarto superior e o quarto inferior do
            # mesmo segmento. Quarenta pixels garantem distância longitudinal
            # suficiente para estimar a direção sem depender de farValid.
            "gap_alignment_sample_ratio": 0.25,
            "gap_min_alignment_span_ratio": 40.0 / 480.0,
            # Uma expansão lateral grande perto da extremidade normalmente é
            # um cruzamento ou uma curva, não uma interrupção simples da fita.
            "gap_max_row_width_ratio": 1.80,
            # Uma continuação desconectada só pertence ao mesmo caminho quando
            # permanece próxima da projeção da fita que chega ao robô.
            "gap_return_corridor_ratio": 0.20,
            "gap_return_min_separation_px": 5,
            "base_speed_preview": 0.66,
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
selected_display_mode = DISPLAY_MODE_REAL
display_mode_lock = threading.Lock()


def normalize_display_mode(value):
    """Mantém o modo visual dentro das três opções aceitas pelo dashboard."""

    normalized = str(value or "").strip().lower()
    return normalized if normalized in DISPLAY_MODES else DISPLAY_MODE_REAL


def set_display_mode(value):
    """Seleciona apenas a imagem codificada, sem alterar o processamento visual."""

    global selected_display_mode
    normalized = normalize_display_mode(value)
    with display_mode_lock:
        selected_display_mode = normalized
    return normalized


def get_display_mode():
    """Lê o modo visual solicitado pela conexão MJPEG ativa."""

    with display_mode_lock:
        return selected_display_mode


def create_display_frame(
    raw_frame,
    line_candidate_mask,
    green_mask,
    roi_start_y,
    display_mode,
):
    """Cria a base exibida sem modificar o frame ou as máscaras da visão."""

    normalized_mode = normalize_display_mode(display_mode)
    if normalized_mode == DISPLAY_MODE_REAL:
        return raw_frame.copy()

    display_frame = np.zeros_like(raw_frame)
    if normalized_mode == DISPLAY_MODE_LINE:
        line_roi = display_frame[roi_start_y:raw_frame.shape[0], :]
        line_roi[line_candidate_mask > 0] = (255, 255, 255)
    else:
        useful_green_region = display_frame[:green_mask.shape[0], :]
        useful_green_region[green_mask > 0] = (0, 255, 0)
    return display_frame


def green_candidate_direction(candidate, interpretation):
    """Resume a direção do candidato usando somente resultados já calculados."""

    if interpretation == "RETORNO_180":
        return "R"
    if interpretation in ("AMBIGUO", "VERDE_FALSO_NO_SENTIDO_ATUAL"):
        return "?"
    return "E" if candidate.get("side") == "ESQUERDA" else "D"


def draw_green_candidate_overlays(
    display_frame,
    candidates,
    rejected_candidates,
    line_axis,
    interpretation,
):
    """Identifica candidatos aceitos e rejeitados sem preencher a imagem real."""

    accepted_color = (64, 255, 96)
    rejected_color = (255, 0, 255)
    for candidate_id, candidate in enumerate(candidates, start=1):
        vote_state, _reason = classify_green_candidate_vote(candidate, line_axis)
        state_text = "ACEITO" if vote_state == "VALIDO" else "IRRESOLUVEL"
        direction = green_candidate_direction(candidate, interpretation)
        center = tuple(int(round(value)) for value in candidate["centroid"])
        cv2.drawContours(
            display_frame,
            [candidate["contour"]],
            -1,
            accepted_color,
            1,
        )
        cv2.circle(display_frame, center, 3, accepted_color, -1)
        cv2.putText(
            display_frame,
            f"C{candidate_id} {state_text} {direction}",
            (center[0] + 5, max(12, center[1] - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            accepted_color,
            1,
            cv2.LINE_AA,
        )

    rejected_id_start = len(candidates) + 1
    for offset, candidate in enumerate(rejected_candidates):
        candidate_id = rejected_id_start + offset
        center = tuple(int(round(value)) for value in candidate["centroid"])
        cv2.drawContours(
            display_frame,
            [candidate["contour"]],
            -1,
            rejected_color,
            1,
        )
        cv2.putText(
            display_frame,
            f"C{candidate_id} REJEITADO ?",
            (center[0] + 5, max(12, center[1] - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            rejected_color,
            1,
            cv2.LINE_AA,
        )


def draw_line_mode_green_overlays(
    display_frame,
    candidates,
    rejected_candidates,
    interpretation,
):
    """Preserva os contornos verdes da visualização binária anterior."""

    for candidate in rejected_candidates:
        cv2.drawContours(
            display_frame,
            [candidate["contour"]],
            -1,
            (180, 80, 180),
            1,
        )
    for candidate in candidates:
        cv2.drawContours(
            display_frame,
            [candidate["contour"]],
            -1,
            (0, 255, 0),
            2,
        )
        center = tuple(int(round(value)) for value in candidate["centroid"])
        cv2.circle(display_frame, center, 3, (0, 255, 0), -1)
        if interpretation == "RETORNO_180":
            candidate_letter = "R"
        elif interpretation == "VERDE_FALSO_NO_SENTIDO_ATUAL":
            candidate_letter = "F"
        elif interpretation == "AMBIGUO":
            candidate_letter = "?"
        else:
            candidate_letter = "E" if candidate["side"] == "ESQUERDA" else "D"
        cv2.putText(
            display_frame,
            candidate_letter,
            (center[0] + 5, max(12, center[1] - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )


def green_status_overlay(green_processing_enabled, green_status):
    """Define texto e cor sem reinterpretar ou substituir a decisão publicada."""

    if not green_processing_enabled:
        return "FALHA DE PROCESSAMENTO", (0, 0, 255)

    interpretation = green_status["greenInterpretation"]
    text = {
        "SEM_DECISAO": "SEM DECISAO",
        "ESQUERDA": "ESQUERDA",
        "DIREITA": "DIREITA",
        "RETORNO_180": "RETORNO 180 GRAUS",
        "VERDE_FALSO_NO_SENTIDO_ATUAL": "FALSO NO SENTIDO ATUAL",
        "AMBIGUO": "AMBIGUO",
    }[interpretation]
    if interpretation == "AMBIGUO":
        return text, (0, 255, 255)
    if green_status["greenConfirmed"] and interpretation in (
        "ESQUERDA",
        "DIREITA",
        "RETORNO_180",
    ):
        return text, (64, 255, 96)
    return text, (220, 220, 220)


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
        request_url = urlsplit(self.path)
        path = request_url.path
        if path != MJPEG_STREAM_PATH:
            self.send_response(404)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write(b"Not found")
            return

        query = parse_qs(request_url.query)
        set_display_mode(query.get("mode", [DISPLAY_MODE_REAL])[0])

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


def camera_runtime_details(picam2, camera_profile, camera_config, camera_index):
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
        "cameraIndex": int(camera_index),
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


def create_camera(camera_profile, camera_index):
    """Configura a Camera V2 e exige o modo físico definido para seu papel."""

    picam2 = Picamera2(camera_index)
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
                camera_index,
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
    runtime_details = camera_runtime_details(
        picam2,
        camera_profile,
        applied_config,
        camera_index,
    )
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


def empty_gap_observation():
    """Cria um resultado de gap seguro para frames sem geometria suficiente."""

    return {
        "candidate": False,
        "alignment_valid": False,
        "alignment_error": 0.0,
        "return_valid": False,
        "return_error": 0.0,
        "endpoint": None,
    }


def analyze_gap_geometry(
    line_candidate_mask,
    vision_geometry,
    vision_profile,
    near_center,
):
    """Detecta uma extremidade desconectada da fita que ainda alcança a NEAR."""

    observation = empty_gap_observation()
    if (
        not vision_profile.get("gap_detection_enabled", False)
        or near_center is None
        or line_candidate_mask.ndim != 2
    ):
        return observation

    binary_mask = (line_candidate_mask > 0).astype(np.uint8)
    component_count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        binary_mask,
        connectivity=8,
    )
    if component_count <= 1:
        return observation

    near_start_y = vision_geometry["near_band_start_y"]
    near_end_y = vision_geometry["near_band_end_y"]
    far_start_y = vision_geometry["far_band_start_y"]
    structural_end_y = vision_geometry["structural_end_y"]
    frame_width = line_candidate_mask.shape[1]

    # Associa a observação ao componente que realmente cruza a NEAR. Isso
    # impede que outro objeto preto maior, porém distante, vire o gap principal.
    primary_label = 0
    primary_distance = float("inf")
    for label in range(1, component_count):
        component_near = labels[near_start_y:near_end_y, :] == label
        near_y, near_x = np.nonzero(component_near)
        if near_x.size == 0:
            continue
        distance = abs(float(np.mean(near_x)) - float(near_center[0]))
        if distance < primary_distance:
            primary_label = label
            primary_distance = distance

    if primary_label == 0:
        return observation

    primary_mask = labels == primary_label
    primary_y, primary_x = np.nonzero(primary_mask)
    if primary_y.size == 0:
        return observation

    endpoint_y = int(np.min(primary_y))
    endpoint_window_height = near_end_y - far_start_y
    endpoint_start_y = far_start_y + int(round(
        endpoint_window_height
        * vision_profile["gap_endpoint_start_ratio"]
    ))
    endpoint_end_y = far_start_y + int(round(
        endpoint_window_height
        * vision_profile["gap_endpoint_end_ratio"]
    ))
    if endpoint_y < endpoint_start_y or endpoint_y > endpoint_end_y:
        return observation

    visible_bottom_y = min(int(np.max(primary_y)) + 1, structural_end_y)
    visible_span_px = visible_bottom_y - endpoint_y
    minimum_alignment_span_px = int(round(
        line_candidate_mask.shape[0]
        * vision_profile["gap_min_alignment_span_ratio"]
    ))
    if visible_span_px < minimum_alignment_span_px:
        return observation

    sample_height_px = max(1, int(round(
        visible_span_px * vision_profile["gap_alignment_sample_ratio"]
    )))
    upper_end_y = min(endpoint_y + sample_height_px, visible_bottom_y)
    lower_start_y = max(endpoint_y, visible_bottom_y - sample_height_px)
    primary_upper = primary_mask[endpoint_y:upper_end_y, :]
    primary_lower = primary_mask[lower_start_y:visible_bottom_y, :]
    upper_y_local, upper_x = np.nonzero(primary_upper)
    lower_y_local, lower_x = np.nonzero(primary_lower)
    if upper_x.size == 0 or lower_x.size == 0:
        return observation

    lower_row_widths = [
        int(row_x.max() - row_x.min() + 1)
        for row in primary_lower
        if (row_x := np.flatnonzero(row)).size > 0
    ]
    endpoint_rows = primary_upper
    endpoint_row_widths = [
        int(row_x.max() - row_x.min() + 1)
        for row in endpoint_rows
        if (row_x := np.flatnonzero(row)).size > 0
    ]
    if not lower_row_widths or not endpoint_row_widths:
        return observation

    reference_width = float(np.median(lower_row_widths))
    maximum_endpoint_width = float(max(endpoint_row_widths))
    if maximum_endpoint_width > (
        reference_width * vision_profile["gap_max_row_width_ratio"]
    ):
        return observation

    near_x_center = float(np.mean(lower_x))
    near_y_center = lower_start_y + float(np.mean(lower_y_local))
    far_x_center = float(np.mean(upper_x))
    far_y_center = endpoint_y + float(np.mean(upper_y_local))
    frame_half_width = frame_width / 2.0
    near_error = (near_x_center - frame_half_width) / frame_half_width
    far_error = (far_x_center - frame_half_width) / frame_half_width
    heading_error = far_error - near_error

    # O sinal vem do maior desvio. Assim, uma compensação acidental entre
    # posição e ângulo não pode declarar o robô alinhado quando um deles é ruim.
    dominant_error = (
        near_error
        if abs(near_error) >= abs(heading_error)
        else heading_error
    )
    alignment_error = math.copysign(
        max(abs(near_error), abs(heading_error)),
        dominant_error,
    ) if dominant_error != 0.0 else 0.0

    endpoint_x_values = primary_x[primary_y == endpoint_y]
    endpoint_x = int(round(float(np.mean(endpoint_x_values))))
    observation.update({
        "candidate": True,
        "alignment_valid": True,
        "alignment_error": max(-1.0, min(alignment_error, 1.0)),
        "endpoint": (endpoint_x, endpoint_y),
    })

    if abs(near_y_center - far_y_center) < 1.0:
        return observation

    corridor_px = frame_width * vision_profile["gap_return_corridor_ratio"]
    minimum_separation = vision_profile["gap_return_min_separation_px"]
    best_return = None
    best_distance = float("inf")
    for label in range(1, component_count):
        if label == primary_label:
            continue
        component_top = stats[label, cv2.CC_STAT_TOP]
        component_height = stats[label, cv2.CC_STAT_HEIGHT]
        component_bottom = component_top + component_height - 1
        if component_bottom >= endpoint_y - minimum_separation:
            continue

        return_x = float(centroids[label][0])
        return_y = float(centroids[label][1])
        projected_x = near_x_center + (
            (far_x_center - near_x_center)
            * (return_y - near_y_center)
            / (far_y_center - near_y_center)
        )
        axis_distance = abs(return_x - projected_x)
        if axis_distance <= corridor_px and axis_distance < best_distance:
            best_return = return_x
            best_distance = axis_distance

    if best_return is not None:
        observation["return_valid"] = True
        observation["return_error"] = max(
            -1.0,
            min((best_return - frame_half_width) / frame_half_width, 1.0),
        )
    return observation


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


def create_green_mask_stages(frame, structural_end_y, camera_format="RGB888"):
    """Expõe as etapas da segmentação somente para a captura one-shot."""

    useful_end_y = max(0, min(frame.shape[0], int(structural_end_y)))
    useful_frame = frame[:useful_end_y, :]
    hsv_frame = frame_to_hsv(useful_frame, camera_format)
    hue_mask = cv2.inRange(
        hsv_frame,
        (GREEN_HUE_MIN, 0, 0),
        (GREEN_HUE_MAX, 255, 255),
    )
    hue_saturation_mask = cv2.inRange(
        hsv_frame,
        (GREEN_HUE_MIN, GREEN_SATURATION_MIN, 0),
        (GREEN_HUE_MAX, 255, 255),
    )
    hsv_mask = cv2.inRange(
        hsv_frame,
        (GREEN_HUE_MIN, GREEN_SATURATION_MIN, GREEN_VALUE_MIN),
        (GREEN_HUE_MAX, 255, 255),
    )
    final_mask = hsv_mask.copy()
    if cv2.countNonZero(hsv_mask) > 0:
        open_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (GREEN_OPEN_KERNEL_SIZE, GREEN_OPEN_KERNEL_SIZE),
        )
        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (GREEN_CLOSE_KERNEL_SIZE, GREEN_CLOSE_KERNEL_SIZE),
        )
        final_mask = cv2.morphologyEx(
            final_mask, cv2.MORPH_OPEN, open_kernel
        )
        final_mask = cv2.morphologyEx(
            final_mask, cv2.MORPH_CLOSE, close_kernel
        )
    return hsv_frame, hue_mask, hue_saturation_mask, hsv_mask, final_mask


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
    timings=None,
):
    """Segmenta e separa candidatos geométricos de ruídos verdes rejeitados."""

    mask_started = time.perf_counter() if timings is not None else 0.0
    green_mask = create_green_mask(frame, structural_end_y, camera_format)
    if timings is not None:
        timings["green_mask_ms"] = (
            time.perf_counter() - mask_started
        ) * 1000.0
    contours_started = time.perf_counter() if timings is not None else 0.0
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
        if not description["geometry_valid"]:
            rejected.append(description)
            continue
        vote_state, _reason = classify_green_candidate_vote(
            description, line_axis
        )
        if vote_state == "REJEITADO":
            rejected.append(description)
        else:
            candidates.append(description)
    candidates.sort(key=lambda candidate: candidate["area"], reverse=True)
    if timings is not None:
        timings["green_contours_ms"] = (
            time.perf_counter() - contours_started
        ) * 1000.0
    return green_mask, candidates, rejected


def camera_array_rgb_channels(frame):
    """Nomeia os canais segundo a ordem que o detector realmente interpreta."""

    if CAMERA_ARRAY_COLOR_ORDER != "BGR":
        raise ValueError("Ordem de canais inesperada para o diagnóstico verde.")
    blue = frame[:, :, 0]
    green = frame[:, :, 1]
    red = frame[:, :, 2]
    return red, green, blue


def channel_percentiles(channel):
    """Resume um canal da região útil com percentis comparáveis entre ensaios."""

    values = np.percentile(channel, (1, 5, 50, 95, 99))
    return {
        "p01": float(values[0]),
        "p05": float(values[1]),
        "p50": float(values[2]),
        "p95": float(values[3]),
        "p99": float(values[4]),
    }


def mask_active_percent(mask):
    """Calcula a fração ativa da máscara em porcentagem da região útil."""

    return (
        100.0 * float(cv2.countNonZero(mask)) / float(mask.size)
        if mask.size > 0
        else 0.0
    )


def green_geometry_rejection_reasons(candidate):
    """Detalha quais filtros geométricos vigentes rejeitaram um componente."""

    partial = bool(candidate["partial"])
    minimum_area = GREEN_MIN_AREA_PX
    minimum_dimension = GREEN_MIN_DIMENSION_PX
    minimum_aspect = GREEN_ASPECT_RATIO_MIN
    minimum_extent = GREEN_MIN_EXTENT
    if partial:
        minimum_area *= GREEN_PARTIAL_AREA_FACTOR
        minimum_dimension *= GREEN_PARTIAL_DIMENSION_FACTOR
        minimum_aspect = GREEN_PARTIAL_ASPECT_RATIO_MIN
        minimum_extent = GREEN_PARTIAL_EXTENT_MIN

    reasons = []
    if not math.isfinite(candidate["area"]) or candidate["area"] < minimum_area:
        reasons.append("area_below_minimum")
    if candidate["short_side"] < minimum_dimension:
        reasons.append("dimension_below_minimum")
    if not minimum_aspect <= candidate["aspect_ratio"] <= GREEN_ASPECT_RATIO_MAX:
        reasons.append("aspect_ratio_outside_range")
    if candidate["extent"] < minimum_extent:
        reasons.append("extent_below_minimum")
    return reasons


def green_ambiguity_reasons(candidates, line_axis, topology, interpretation):
    """Explica a ambiguidade atual sem modificar a classificação publicada."""

    if interpretation.get("interpretation") != "AMBIGUO":
        return []
    votes = [
        classify_green_candidate_vote(candidate, line_axis)
        for candidate in candidates
    ]
    unresolved_reasons = sorted({
        reason for state, reason in votes if state == "IRRESOLUVEL"
    })
    if unresolved_reasons:
        return unresolved_reasons
    if (
        interpretation.get("left_seen", False)
        and interpretation.get("right_seen", False)
        and not interpretation.get("pair_compatible", False)
    ):
        return ["candidatos opostos sem par compatível para retorno"]
    if not topology.get("junction_valid", False):
        return ["topologia do encontro inválida"]
    if topology.get("confidence", 0.0) < 0.50:
        return ["confiança da topologia abaixo de 0,50"]
    return ["candidatos válidos sem decisão direcional única"]


def json_safe_camera_metadata(metadata):
    """Seleciona apenas metadados necessários e converte valores para JSON."""

    def safe_value(value):
        if value is None or isinstance(value, (str, bool, int)):
            return value
        if isinstance(value, float):
            return value if math.isfinite(value) else None
        if isinstance(value, (tuple, list)):
            return [safe_value(item) for item in value]
        try:
            converted = float(value)
            return converted if math.isfinite(converted) else None
        except (TypeError, ValueError):
            return str(value)

    names = (
        "ExposureTime",
        "AnalogueGain",
        "ColourGains",
        "ColourTemperature",
        "AwbEnable",
        "AeEnable",
    )
    return {name: safe_value(metadata.get(name)) for name in names}


def component_pixel_statistics(candidate, useful_frame, hsv_frame, final_mask):
    """Mede cor somente nos pixels finais pertencentes ao componente."""

    component_mask = np.zeros(final_mask.shape, dtype=np.uint8)
    cv2.drawContours(component_mask, [candidate["contour"]], -1, 255, -1)
    active_pixels = (component_mask > 0) & (final_mask > 0)
    red, green, blue = camera_array_rgb_channels(useful_frame)
    hue = hsv_frame[:, :, 0][active_pixels]
    saturation = hsv_frame[:, :, 1][active_pixels]
    value = hsv_frame[:, :, 2][active_pixels]
    red_values = red[active_pixels]
    green_values = green[active_pixels]
    blue_values = blue[active_pixels]
    dominance = green_values.astype(np.int16) - np.maximum(
        red_values, blue_values
    ).astype(np.int16)

    def median(values):
        return float(np.median(values)) if values.size else 0.0

    def range_summary(values):
        if not values.size:
            return {"min": 0.0, "median": 0.0, "max": 0.0}
        return {
            "min": float(np.min(values)),
            "median": median(values),
            "max": float(np.max(values)),
        }

    return {
        "sampled_pixel_count": int(np.count_nonzero(active_pixels)),
        "hue_median": median(hue),
        "saturation": range_summary(saturation),
        "value": range_summary(value),
        "rgb_medians": {
            "red": median(red_values),
            "green": median(green_values),
            "blue": median(blue_values),
        },
        "green_dominance_median": median(dominance),
    }


def build_green_capture_stats(
    frame,
    camera_format,
    mask_stages,
    candidates,
    rejected,
    line_axis,
    topology,
    interpretation,
    camera_metadata,
):
    """Monta a evidência one-shot do pipeline antes de desenhar o overlay."""

    hsv_frame, hue_mask, hue_saturation_mask, hsv_mask, final_mask = mask_stages
    useful_frame = frame[:hsv_frame.shape[0], :]
    red, green, blue = camera_array_rgb_channels(useful_frame)
    dominance = green.astype(np.int16) - np.maximum(red, blue).astype(np.int16)
    hsv_contours, _ = cv2.findContours(
        hsv_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    final_contours, _ = cv2.findContours(
        final_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    all_components = [
        (candidate, True) for candidate in candidates
    ] + [
        (candidate, False) for candidate in rejected
    ]
    component_stats = []
    for component_id, (candidate, accepted) in enumerate(all_components, start=1):
        vote_state, vote_reason = classify_green_candidate_vote(
            candidate, line_axis
        )
        geometry_reasons = green_geometry_rejection_reasons(candidate)
        reason = vote_reason if candidate["geometry_valid"] else ", ".join(
            geometry_reasons
        )
        hull_area = float(cv2.contourArea(cv2.convexHull(candidate["contour"])))
        component_stats.append({
            "id": component_id,
            "area": float(candidate["area"]),
            "bbox": [int(value) for value in candidate["bounding_box"]],
            "centroid": [float(value) for value in candidate["centroid"]],
            "partial": bool(candidate["partial"]),
            "aspect_ratio": float(candidate["aspect_ratio"]),
            "extent": float(candidate["extent"]),
            "solidity": (
                float(candidate["area"] / hull_area) if hull_area > 0.0 else 0.0
            ),
            "geometry_valid": bool(candidate["geometry_valid"]),
            "associated_with_line": bool(candidate["associated_with_line"]),
            "accepted": accepted,
            "rejected": not accepted,
            "vote_state": vote_state,
            "reason": reason,
            **component_pixel_statistics(
                candidate, useful_frame, hsv_frame, final_mask
            ),
        })

    return {
        "captured_at_unix_seconds": time.time(),
        "detector_region": {
            "x": 0,
            "y": 0,
            "width": int(useful_frame.shape[1]),
            "height": int(useful_frame.shape[0]),
        },
        "camera": {
            "requested_format": camera_format,
            "detector_array_order": CAMERA_ARRAY_COLOR_ORDER,
            "requested_controls": {
                "AwbEnable": True,
                "AeEnable": True,
                "ExposureValue": CAMERA_EXPOSURE_VALUE,
            },
            "metadata": json_safe_camera_metadata(camera_metadata),
        },
        "thresholds": {
            "hue": [GREEN_HUE_MIN, GREEN_HUE_MAX],
            "saturation_min": GREEN_SATURATION_MIN,
            "value_min": GREEN_VALUE_MIN,
            "open_kernel": [GREEN_OPEN_KERNEL_SIZE, GREEN_OPEN_KERNEL_SIZE],
            "close_kernel": [GREEN_CLOSE_KERNEL_SIZE, GREEN_CLOSE_KERNEL_SIZE],
            "minimum_area_px": GREEN_MIN_AREA_PX,
            "minimum_dimension_px": GREEN_MIN_DIMENSION_PX,
            "aspect_ratio": [GREEN_ASPECT_RATIO_MIN, GREEN_ASPECT_RATIO_MAX],
            "minimum_extent": GREEN_MIN_EXTENT,
            "solidity_filter_enabled": False,
            "fragment_merge_gap_px": GREEN_FRAGMENT_MERGE_GAP_PX,
        },
        "frame_percentiles": {
            "array_channels": {
                "channel_0": channel_percentiles(useful_frame[:, :, 0]),
                "channel_1": channel_percentiles(useful_frame[:, :, 1]),
                "channel_2": channel_percentiles(useful_frame[:, :, 2]),
            },
            "rgb": {
                "red": channel_percentiles(red),
                "green": channel_percentiles(green),
                "blue": channel_percentiles(blue),
            },
            "hsv": {
                "hue": channel_percentiles(hsv_frame[:, :, 0]),
                "saturation": channel_percentiles(hsv_frame[:, :, 1]),
                "value": channel_percentiles(hsv_frame[:, :, 2]),
            },
        },
        "mean_green_dominance": float(np.mean(dominance)),
        "mask_active_percent": {
            "hue_only": mask_active_percent(hue_mask),
            "hue_and_saturation": mask_active_percent(hue_saturation_mask),
            "full_hsv": mask_active_percent(hsv_mask),
            "after_morphology": mask_active_percent(final_mask),
        },
        "mask_active_pixels": {
            "hue_only": int(cv2.countNonZero(hue_mask)),
            "hue_and_saturation": int(cv2.countNonZero(hue_saturation_mask)),
            "full_hsv": int(cv2.countNonZero(hsv_mask)),
            "after_morphology": int(cv2.countNonZero(final_mask)),
        },
        "pipeline_counts": {
            "hsv_mask_external_contours": len(hsv_contours),
            "final_mask_external_contours": len(final_contours),
            "accepted_candidates": len(candidates),
            "rejected_candidates": len(rejected),
            "detector_components_after_merge": len(all_components),
        },
        "line_axis_valid": bool(line_axis.get("valid", False)),
        "interpretation": interpretation.get("interpretation", "SEM_DECISAO"),
        "ambiguity_reasons": green_ambiguity_reasons(
            candidates, line_axis, topology, interpretation
        ),
        "components": component_stats,
    }


def save_green_capture(
    frame,
    camera_format,
    mask_stages,
    candidates,
    rejected,
    line_axis,
    topology,
    interpretation,
    camera_metadata,
):
    """Grava uma captura diagnóstica completa por substituições atômicas."""

    _hsv_frame, _hue_mask, _hue_saturation_mask, hsv_mask, final_mask = (
        mask_stages
    )
    candidates_image = frame.copy()
    for component_id, candidate in enumerate(candidates, start=1):
        cv2.drawContours(
            candidates_image, [candidate["contour"]], -1, (0, 255, 0), 2
        )
        center = tuple(int(round(value)) for value in candidate["centroid"])
        cv2.putText(
            candidates_image, str(component_id), center,
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA,
        )
    rejected_id_start = len(candidates) + 1
    for offset, candidate in enumerate(rejected):
        component_id = rejected_id_start + offset
        cv2.drawContours(
            candidates_image, [candidate["contour"]], -1, (180, 80, 180), 1
        )
        center = tuple(int(round(value)) for value in candidate["centroid"])
        cv2.putText(
            candidates_image, str(component_id), center,
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 80, 180), 1, cv2.LINE_AA,
        )

    stats = build_green_capture_stats(
        frame,
        camera_format,
        mask_stages,
        candidates,
        rejected,
        line_axis,
        topology,
        interpretation,
        camera_metadata,
    )
    image_outputs = (
        (GREEN_CAPTURE_RAW_PATH, "/dev/shm/.obr_green_raw.tmp.png", frame),
        (
            GREEN_CAPTURE_HSV_MASK_PATH,
            "/dev/shm/.obr_green_hsv_mask.tmp.png",
            hsv_mask,
        ),
        (
            GREEN_CAPTURE_FINAL_MASK_PATH,
            "/dev/shm/.obr_green_final_mask.tmp.png",
            final_mask,
        ),
        (
            GREEN_CAPTURE_CANDIDATES_PATH,
            "/dev/shm/.obr_green_candidates.tmp.png",
            candidates_image,
        ),
    )
    temporary_paths = [temporary for _target, temporary, _image in image_outputs]
    temporary_paths.append("/dev/shm/.obr_green_stats.tmp.json")
    try:
        for _target, temporary, image in image_outputs:
            if not cv2.imwrite(temporary, image):
                raise OSError(f"Não foi possível gravar {temporary}.")
        with open(temporary_paths[-1], "w", encoding="utf-8") as stats_file:
            json.dump(stats, stats_file, indent=2, allow_nan=False)
        for target, temporary, _image in image_outputs:
            os.replace(temporary, target)
        os.replace(temporary_paths[-1], GREEN_CAPTURE_STATS_PATH)
    finally:
        for temporary in temporary_paths:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


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


def classify_green_candidate_vote(candidate, line_axis):
    """Define se o candidato pode votar em um lado e explica a decisão."""

    if not line_axis.get("valid", False):
        return "IRRESOLUVEL", "eixo local da linha inválido"
    if candidate.get("partial", False):
        # Um marcador cortado pela borda ainda pode comandar a curva quando a
        # parte visível satisfaz os limites normais, sem tolerâncias reduzidas.
        partial_geometry_is_strong = green_geometry_is_valid(
            candidate.get("area", 0.0),
            candidate.get("short_side", 0.0),
            candidate.get("aspect_ratio", 0.0),
            candidate.get("extent", 0.0),
            False,
        )
        if not partial_geometry_is_strong:
            return "IRRESOLUVEL", "recorte parcial insuficiente"

    lateral = float(candidate.get("lateral", float("nan")))
    side = candidate.get("side", "UNKNOWN")
    if not math.isfinite(lateral) or abs(lateral) < GREEN_SIDE_MIN_DISTANCE_PX:
        return "IRRESOLUVEL", "distância lateral insuficiente para definir o lado"
    expected_side = "DIREITA" if lateral > 0.0 else "ESQUERDA"
    if side not in ("ESQUERDA", "DIREITA") or side != expected_side:
        return "IRRESOLUVEL", "lado incompatível com a geometria local da linha"
    if not candidate.get("associated_with_line", False):
        return "REJEITADO", "sem associação espacial válida com a linha"
    return "VALIDO", "evidência espacial válida"


def green_pair_is_compatible(first, second, topology, reference_line_width):
    """Aplica ao par esquerda/direita os critérios existentes do retorno de 180°."""

    if first["side"] == second["side"]:
        return False
    longitudinal_compatible = (
        abs(first["longitudinal"] - second["longitudinal"])
        <= GREEN_PAIR_LONGITUDINAL_TOLERANCE_LINE_WIDTHS
        * reference_line_width
    )
    if not topology.get("junction_valid", False):
        return False
    junction_longitudinal = topology["junction_longitudinal"]
    before_same_encounter = all(
        candidate["longitudinal"] < junction_longitudinal
        and junction_longitudinal - candidate["longitudinal"]
        <= GREEN_ENCOUNTER_DISTANCE_LINE_WIDTHS * reference_line_width
        for candidate in (first, second)
    )
    return longitudinal_compatible and before_same_encounter


def interpret_green_candidates(candidates, line_axis, topology):
    """Interpreta candidatos priorizando retorno, direção, falso e ambiguidade."""

    candidate_votes = [
        (candidate, *classify_green_candidate_vote(candidate, line_axis))
        for candidate in candidates
    ]
    valid_candidates = [
        candidate
        for candidate, vote_state, _reason in candidate_votes
        if vote_state == "VALIDO"
    ]
    unresolved_candidates = [
        candidate
        for candidate, vote_state, _reason in candidate_votes
        if vote_state == "IRRESOLUVEL"
    ]
    observation_state = green_observation_state(
        len(valid_candidates) + len(unresolved_candidates)
    )
    result = {
        "observation_state": observation_state,
        "interpretation": "SEM_DECISAO",
        "left_seen": False,
        "right_seen": False,
        "pair_compatible": False,
    }
    for candidate in valid_candidates:
        if candidate["side"] == "ESQUERDA":
            result["left_seen"] = True
        else:
            result["right_seen"] = True

    # Um candidato significativo sem lado confiável impede uma decisão segura.
    # Componentes apenas rejeitados espacialmente não participam da decisão.
    if unresolved_candidates:
        result["interpretation"] = "AMBIGUO"
        return result
    if not valid_candidates:
        return result

    line_widths = [
        candidate["local_line_width_px"]
        for candidate in valid_candidates
        if candidate["local_line_width_px"] > 0.0
    ]
    reference_line_width = max(8.0, sum(line_widths) / len(line_widths)) \
        if line_widths else 8.0

    left_candidates = [
        candidate for candidate in valid_candidates
        if candidate["side"] == "ESQUERDA"
    ]
    right_candidates = [
        candidate for candidate in valid_candidates
        if candidate["side"] == "DIREITA"
    ]

    # O retorno de 180° continua tendo prioridade quando há evidência válida
    # nos dois lados. Fragmentos extras não eliminam um par compatível.
    if left_candidates and right_candidates:
        pair_compatible = any(
            green_pair_is_compatible(
                left_candidate,
                right_candidate,
                topology,
                reference_line_width,
            )
            for left_candidate in left_candidates
            for right_candidate in right_candidates
        )
        result["pair_compatible"] = pair_compatible
        result["interpretation"] = (
            "RETORNO_180" if pair_compatible else "AMBIGUO"
        )
        return result

    if (
        not topology.get("junction_valid", False)
        or topology.get("confidence", 0.0) < 0.50
    ):
        result["interpretation"] = "AMBIGUO"
        return result

    candidates_before_junction = [
        candidate for candidate in valid_candidates
        if candidate["longitudinal"] < topology["junction_longitudinal"]
    ]
    if not candidates_before_junction:
        result["interpretation"] = "VERDE_FALSO_NO_SENTIDO_ATUAL"
        return result

    resolved_side = valid_candidates[0]["side"]
    matching_branch = (
        topology["left_branch"]
        if resolved_side == "ESQUERDA"
        else topology["right_branch"]
    )
    result["interpretation"] = (
        resolved_side
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
        "greenNearSeen": False,
        "greenRawInterpretation": "SEM_DECISAO",
        "greenPathBlackValid": False,
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


def green_seen_in_vertical_band(
    candidates,
    band_start_y,
    band_end_y,
    line_axis,
):
    """Aceita na banda somente verde completo e associado ao eixo da linha."""

    return any(
        band_start_y <= float(candidate["centroid"][1]) < band_end_y
        and classify_green_candidate_vote(candidate, line_axis)[0] == "VALIDO"
        for candidate in candidates
    )


def actionable_green_candidates(candidates, band_start_y, band_end_y, line_axis):
    """Seleciona quadrados completos que podem interferir no movimento."""

    return [
        candidate
        for candidate in candidates
        if band_start_y <= float(candidate["centroid"][1]) < band_end_y
        and classify_green_candidate_vote(candidate, line_axis)[0] == "VALIDO"
    ]


def interpret_actionable_green_candidates(candidates):
    """Decide a curva usando somente marcadores aceitos dentro da ROI azul."""

    left_candidates = [
        candidate for candidate in candidates
        if candidate["side"] == "ESQUERDA"
    ]
    right_candidates = [
        candidate for candidate in candidates
        if candidate["side"] == "DIREITA"
    ]
    result = {
        "observation_state": green_observation_state(len(candidates)),
        "interpretation": "SEM_DECISAO",
        "left_seen": bool(left_candidates),
        "right_seen": bool(right_candidates),
        "pair_compatible": False,
    }
    if left_candidates and right_candidates:
        # Um marcador aceito de cada lado representa o retorno de 180 graus.
        # Candidatos rejeitados ou irresolúveis não chegam a esta função.
        result["pair_compatible"] = True
        result["interpretation"] = "RETORNO_180"
    elif left_candidates:
        result["interpretation"] = "ESQUERDA"
    elif right_candidates:
        result["interpretation"] = "DIREITA"
    return result


def green_path_black_is_valid(
    near_valid,
    far_valid,
    center_delta_valid,
    center_delta_px,
    line_axis,
    interpretation,
):
    """Confirma que a faixa preta superior pertence ao mesmo trajeto."""

    return bool(
        near_valid
        and far_valid
        and center_delta_valid
        and math.isfinite(float(center_delta_px))
        and abs(float(center_delta_px)) <= GREEN_PATH_MAX_CENTER_DELTA_PX
        and line_axis.get("valid", False)
        and interpretation in ("ESQUERDA", "DIREITA", "RETORNO_180")
    )


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
    gap_candidate,
    gap_alignment_valid,
    gap_alignment_error,
    gap_return_valid,
    gap_return_error,
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

    gap_candidate = bool(gap_candidate and near_valid)
    try:
        gap_alignment_error = float(gap_alignment_error)
        gap_alignment_finite = math.isfinite(gap_alignment_error)
    except (TypeError, ValueError):
        gap_alignment_finite = False
    gap_alignment_valid = bool(
        gap_candidate and gap_alignment_valid and gap_alignment_finite
    )
    if not gap_alignment_valid:
        gap_alignment_error = 0.0

    try:
        gap_return_error = float(gap_return_error)
        gap_return_finite = math.isfinite(gap_return_error)
    except (TypeError, ValueError):
        gap_return_finite = False
    gap_return_valid = bool(
        gap_candidate and gap_return_valid and gap_return_finite
    )
    if not gap_return_valid:
        gap_return_error = 0.0

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
            "gapCandidate": gap_candidate,
            "gapAlignmentValid": gap_alignment_valid,
            "gapAlignmentError": gap_alignment_error,
            "gapReturnValid": gap_return_valid,
            "gapReturnError": gap_return_error,
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
    gap_candidate=False,
    gap_alignment_valid=False,
    gap_alignment_error=0.0,
    gap_return_valid=False,
    gap_return_error=0.0,
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

    gap_candidate = bool(gap_candidate and near_valid)
    try:
        gap_alignment_error = float(gap_alignment_error)
        gap_alignment_finite = math.isfinite(gap_alignment_error)
    except (TypeError, ValueError):
        gap_alignment_finite = False
    gap_alignment_valid = bool(
        gap_candidate and gap_alignment_valid and gap_alignment_finite
    )
    if not gap_alignment_valid:
        gap_alignment_error = 0.0

    try:
        gap_return_error = float(gap_return_error)
        gap_return_finite = math.isfinite(gap_return_error)
    except (TypeError, ValueError):
        gap_return_finite = False
    gap_return_valid = bool(
        gap_candidate and gap_return_valid and gap_return_finite
    )
    if not gap_return_valid:
        gap_return_error = 0.0
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
        "cameraIndex": camera_details.get("cameraIndex"),
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
        "gapCandidate": gap_candidate,
        "gapAlignmentValid": gap_alignment_valid,
        "gapAlignmentError": gap_alignment_error,
        "gapReturnValid": gap_return_valid,
        "gapReturnError": gap_return_error,
        "lineTimestamp": line_timestamp,
        "lineSequence": line_sequence,
    }
    with open(TEMP_STATUS_PATH, "w", encoding="utf-8") as status_file:
        json.dump(status, status_file, allow_nan=False)
    os.replace(TEMP_STATUS_PATH, STATUS_PATH)


class VisionRegressionProfiler:
    """Publica uma amostra temporária somente durante o trace solicitado."""

    # O processo C++ remove o gatilho quando grava 300 amostras correlacionadas.
    # Este tempo é apenas uma proteção caso o consumidor deixe de responder.
    MAX_DURATION_SECONDS = 60.0

    def __init__(self):
        self.active = False
        self.started_at = 0.0
        self.sample_count = 0
        self.last_capture_time = None
        self.last_processing_time = None
        self.last_ipc_time = None
        self.last_mjpeg_time = None
        self.capture_hz = 0.0
        self.processing_hz = 0.0
        self.ipc_hz = 0.0
        self.mjpeg_hz = 0.0

    @staticmethod
    def rate(previous_time, current_time):
        """Calcula uma frequência instantânea somente entre eventos novos."""

        if previous_time is None or current_time <= previous_time:
            return 0.0
        return 1.0 / (current_time - previous_time)

    def begin_frame(self, current_time):
        """Ativa o perfil somente enquanto o trigger existir."""

        if not os.path.isfile(LINE_TRACE_REQUEST_PATH):
            self.active = False
            return False
        if not self.active:
            self.active = True
            self.started_at = current_time
            self.sample_count = 0
            self.last_capture_time = None
            self.last_processing_time = None
            self.last_ipc_time = None
            self.last_mjpeg_time = None
            self.capture_hz = 0.0
            self.processing_hz = 0.0
            self.ipc_hz = 0.0
            self.mjpeg_hz = 0.0
        return True

    def note_capture(self, current_time):
        self.capture_hz = self.rate(self.last_capture_time, current_time)
        self.last_capture_time = current_time

    def note_ipc(self, current_time):
        self.ipc_hz = self.rate(self.last_ipc_time, current_time)
        self.last_ipc_time = current_time

    def note_mjpeg(self, current_time):
        self.mjpeg_hz = self.rate(self.last_mjpeg_time, current_time)
        self.last_mjpeg_time = current_time

    def publish_sample(
        self,
        line_sequence,
        frame_timestamp,
        timings,
        green_raw,
        green_confirmed,
        completed_at,
    ):
        """Entrega ao processo C++ uma linha temporária do mesmo frame."""

        self.processing_hz = self.rate(
            self.last_processing_time, completed_at
        )
        self.last_processing_time = completed_at
        row = (
            int(line_sequence),
            float(frame_timestamp),
            self.capture_hz,
            self.processing_hz,
            self.ipc_hz,
            self.mjpeg_hz,
            timings.get("capture_ms", 0.0),
            timings.get("line_detection_ms", 0.0),
            timings.get("green_mask_ms", 0.0),
            timings.get("green_contours_ms", 0.0),
            timings.get("topology_ms", 0.0),
            timings.get("green_processing_ms", 0.0),
            timings.get("overlay_ms", 0.0),
            timings.get("mjpeg_ms", 0.0),
            timings.get("ipc_ms", 0.0),
            timings.get("total_vision_ms", 0.0),
            str(green_raw),
            bool(green_confirmed),
        )
        with open(
            TEMP_VISION_TRACE_SAMPLE_PATH,
            "w",
            encoding="utf-8",
            newline="",
        ) as sample_file:
            csv.writer(sample_file).writerow(row)
        os.replace(TEMP_VISION_TRACE_SAMPLE_PATH, VISION_TRACE_SAMPLE_PATH)

        self.sample_count += 1
        duration = completed_at - self.started_at
        if duration >= self.MAX_DURATION_SECONDS:
            try:
                os.unlink(LINE_TRACE_REQUEST_PATH)
            except FileNotFoundError:
                pass


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
        camera_indices = configured_camera_indices()
        camera_infos = Picamera2.global_camera_info()
        camera_assignments = resolve_camera_assignments(
            camera_infos,
            camera_indices,
        )
        log_camera_inventory(camera_infos, camera_assignments)
        selected_camera = require_camera_assignment(
            camera_assignments,
            camera_profile["role"],
        )
        camera_details["cameraIndex"] = selected_camera["index"]
        line_ipc_enabled = camera_role_publishes_line_status(
            camera_profile["role"]
        )

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(LIGHT_PIN_BOARD, GPIO.OUT)
        GPIO.output(LIGHT_PIN_BOARD, GPIO.HIGH)
        light_ready = True

        stream_server = start_stream_server()
        picam2, camera_format, camera_details = create_camera(
            camera_profile,
            selected_camera["index"],
        )
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
        vision_profiler = VisionRegressionProfiler()
        (
            green_experiment_mode,
            green_processing_enabled,
            green_decisions_enabled,
        ) = resolve_green_experiment_mode(
            vision_profile.get("green_detection_enabled", False),
            GREEN_PROCESSING_ENABLED,
            GREEN_DECISIONS_ENABLED,
        )
        print(
            f"Modo {green_experiment_mode} do verde: "
            f"processamento={'ligado' if green_processing_enabled else 'desligado'}, "
            f"decisões={'ligadas' if green_decisions_enabled else 'ignoradas'}.",
            flush=True,
        )

        while running:
            loop_started = time.perf_counter()
            trace_active = vision_profiler.begin_frame(loop_started)
            timings = {} if trace_active else None
            capture_started = time.perf_counter() if trace_active else 0.0
            green_capture_requested = os.path.isfile(GREEN_CAPTURE_REQUEST_PATH)
            green_capture_metadata = {}
            if green_capture_requested:
                # A requisição preserva os metadados do mesmo frame usado no
                # diagnóstico. Fora do gatilho, o caminho normal não muda.
                camera_request = picam2.capture_request()
                try:
                    raw_frame = camera_request.make_array("main")
                    green_capture_metadata = camera_request.get_metadata()
                finally:
                    camera_request.release()
            else:
                raw_frame = picam2.capture_array()
            capture_completed = time.perf_counter()
            if trace_active:
                timings["capture_ms"] = (
                    capture_completed - capture_started
                ) * 1000.0
                vision_profiler.note_capture(capture_completed)
                line_detection_started = capture_completed
            # O frame capturado permanece intacto. Máscaras e decisões são
            # calculadas antes de criar a cópia exclusiva do dashboard.
            frame_height = raw_frame.shape[0]
            vision_geometry = resolve_vision_geometry(
                frame_height,
                vision_profile,
            )
            filtered_mask, roi_start_y = create_filtered_line_mask(
                raw_frame,
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
                frame_half_width = raw_frame.shape[1] / 2.0
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
                frame_half_width = raw_frame.shape[1] / 2.0
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

            gap_observation = analyze_gap_geometry(
                line_candidate_mask,
                vision_geometry,
                vision_profile,
                near_center,
            )
            line_axis = build_line_axis(near_center, far_center)
            # A parada inicial pelo quadrado não depende de já existir preto
            # na FAR. A curva continua exigindo o eixo real entre as duas ROIs.
            green_line_axis = line_axis
            if not green_line_axis.get("valid", False) and near_center is not None:
                green_line_axis = build_line_axis(
                    near_center,
                    (near_center[0], far_band_start_y + far_band_height_px // 2),
                )
            if trace_active:
                timings["line_detection_ms"] = (
                    time.perf_counter() - line_detection_started
                ) * 1000.0
                timings["green_mask_ms"] = 0.0
                timings["green_contours_ms"] = 0.0
                timings["topology_ms"] = 0.0
            green_candidates = []
            green_rejected = []
            green_mask = np.zeros(
                (vision_geometry["structural_end_y"], raw_frame.shape[1]),
                dtype=np.uint8,
            )
            green_topology = analyze_line_topology(None, green_line_axis, 0.0)
            green_interpretation = interpret_green_candidates(
                green_candidates,
                green_line_axis,
                green_topology,
            )
            green_processing_started = time.perf_counter()
            if green_processing_enabled:
                green_mask, green_candidates, green_rejected = find_green_candidates(
                    raw_frame,
                    vision_geometry["structural_end_y"],
                    line_candidate_mask,
                    green_line_axis,
                    camera_format,
                    timings,
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
                    topology_started = (
                        time.perf_counter() if trace_active else 0.0
                    )
                    green_topology = analyze_line_topology(
                        line_candidate_mask,
                        green_line_axis,
                        reference_line_width_px,
                    )
                    if trace_active:
                        timings["topology_ms"] = (
                            time.perf_counter() - topology_started
                        ) * 1000.0
                actionable_candidates = actionable_green_candidates(
                    green_candidates,
                    near_band_start_y,
                    near_band_end_y,
                    green_line_axis,
                )
                green_interpretation = interpret_actionable_green_candidates(
                    actionable_candidates,
                )
            green_processing_ms = (
                time.perf_counter() - green_processing_started
            ) * 1000.0
            if trace_active:
                timings["green_processing_ms"] = green_processing_ms

            frame_width = raw_frame.shape[1]
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
            green_raw_interpretation = green_interpretation["interpretation"]
            green_path_valid = green_path_black_is_valid(
                near_valid,
                far_valid,
                center_delta_valid,
                center_delta_px,
                green_line_axis,
                green_raw_interpretation,
            )
            tracker_interpretation = (
                green_raw_interpretation
                if green_decisions_enabled and green_path_valid
                else "SEM_DECISAO"
            )
            green_tracker_result = green_tracker.update(
                line_sequence,
                tracker_interpretation,
            )
            green_status = build_green_status(
                green_candidates,
                len(green_rejected),
                green_interpretation,
                green_topology,
                green_tracker_result,
                green_processing_ms,
            )
            green_status["greenNearSeen"] = green_seen_in_vertical_band(
                green_candidates,
                near_band_start_y,
                near_band_end_y,
                green_line_axis,
            )
            green_status["greenRawInterpretation"] = green_raw_interpretation
            green_status["greenPathBlackValid"] = green_path_valid
            ipc_started = time.perf_counter() if trace_active else 0.0
            if line_ipc_enabled:
                # Somente a CAM0/downward publica dados usados pelo segue-faixa.
                # A câmera frontal nunca pode substituir silenciosamente essa fonte.
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
                    gap_observation["candidate"],
                    gap_observation["alignment_valid"],
                    gap_observation["alignment_error"],
                    gap_observation["return_valid"],
                    gap_observation["return_error"],
                    line_timestamp,
                    line_sequence,
                    green_status,
                )
            if trace_active:
                ipc_completed = time.perf_counter()
                timings["ipc_ms"] = (ipc_completed - ipc_started) * 1000.0
                if line_ipc_enabled:
                    vision_profiler.note_ipc(ipc_completed)

            if green_capture_requested:
                # A captura ocorre antes de qualquer desenho no frame e é
                # removida do fluxo após uma única tentativa, mesmo se falhar.
                try:
                    green_mask_stages = create_green_mask_stages(
                        raw_frame,
                        vision_geometry["structural_end_y"],
                        camera_format,
                    )
                    save_green_capture(
                        raw_frame,
                        camera_format,
                        green_mask_stages,
                        green_candidates,
                        green_rejected,
                        green_line_axis,
                        green_topology,
                        green_interpretation,
                        green_capture_metadata,
                    )
                    print(
                        "Captura diagnóstica verde gravada em /dev/shm.",
                        flush=True,
                    )
                except Exception as error:
                    print(
                        f"Falha na captura diagnóstica verde: {error}",
                        flush=True,
                    )
                finally:
                    try:
                        os.unlink(GREEN_CAPTURE_REQUEST_PATH)
                    except FileNotFoundError:
                        pass

            overlay_started = time.perf_counter() if trace_active else 0.0

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

            display_mode = get_display_mode()
            display_frame = create_display_frame(
                raw_frame,
                line_candidate_mask,
                green_mask,
                roi_start_y,
                display_mode,
            )
            # No modo VERDE, os desenhos de linha abaixo ficam em uma imagem
            # descartável. Assim, a saída final contém apenas a máscara verde,
            # seus candidatos e o estado semântico solicitado.
            frame = (
                np.zeros_like(display_frame)
                if display_mode == DISPLAY_MODE_GREEN
                else display_frame
            )
            debug_text_overlay_enabled = (
                vision_profile["debug_text_overlay"]
                and display_mode != DISPLAY_MODE_GREEN
            )

            if (
                display_mode != DISPLAY_MODE_GREEN
                and gap_observation["candidate"]
                and gap_observation["endpoint"] is not None
            ):
                gap_color = (0, 165, 255)
                cv2.circle(
                    frame,
                    gap_observation["endpoint"],
                    7,
                    gap_color,
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame,
                    "GAP CANDIDATO",
                    (12, 300),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    gap_color,
                    2,
                    cv2.LINE_AA,
                )

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
                        debug_text_overlay_enabled,
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

            if display_mode == DISPLAY_MODE_GREEN:
                frame = display_frame

            interpretation = green_status["greenInterpretation"]
            if green_processing_enabled:
                if display_mode == DISPLAY_MODE_LINE:
                    draw_line_mode_green_overlays(
                        frame,
                        green_candidates,
                        green_rejected,
                        interpretation,
                    )
                else:
                    draw_green_candidate_overlays(
                        frame,
                        green_candidates,
                        green_rejected,
                        green_line_axis,
                        green_raw_interpretation,
                    )

                if (
                    display_mode != DISPLAY_MODE_GREEN
                    and line_axis.get("valid", False)
                ):
                    axis_start = point_from_line_axis(line_axis, -30.0)
                    axis_end = point_from_line_axis(
                        green_line_axis,
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

                if (
                    display_mode != DISPLAY_MODE_GREEN
                    and green_topology.get("junction_valid", False)
                ):
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

            active_pixel_count = cv2.countNonZero(filtered_mask)
            debug_lines = (
                "MASCARA EXPERIMENTAL",
                f"THRESHOLD: {vision_profile['line_threshold']}",
                f"PIXELS ATIVOS: {active_pixel_count}",
            )
            for index, debug_text in enumerate(debug_lines):
                put_debug_text(
                    debug_text_overlay_enabled,
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
                    debug_text_overlay_enabled,
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
                    debug_text_overlay_enabled,
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
                    debug_text_overlay_enabled,
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
                    debug_text_overlay_enabled,
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
                    debug_text_overlay_enabled,
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
                debug_text_overlay_enabled,
                frame,
                preview_direction,
                (near_debug_x, 225),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            if display_mode != DISPLAY_MODE_GREEN and near_valid:
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

            overlay_completed = time.perf_counter()
            if trace_active:
                timings["overlay_ms"] = (
                    overlay_completed - overlay_started
                ) * 1000.0

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
            mjpeg_started = time.perf_counter() if trace_active else 0.0
            if stream_due or snapshot_due:
                jpeg = encode_frame(frame)
                if jpeg is not None:
                    if stream_due:
                        publish_stream_frame(jpeg)
                        last_stream_time = now
                        if trace_active:
                            vision_profiler.note_mjpeg(time.perf_counter())
                    if snapshot_due:
                        save_frame(jpeg)
                        last_snapshot_time = now
            if trace_active:
                timings["mjpeg_ms"] = (
                    time.perf_counter() - mjpeg_started
                ) * 1000.0

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
                    gap_candidate=gap_observation["candidate"],
                    gap_alignment_valid=gap_observation["alignment_valid"],
                    gap_alignment_error=gap_observation["alignment_error"],
                    gap_return_valid=gap_observation["return_valid"],
                    gap_return_error=gap_observation["return_error"],
                    line_timestamp=line_timestamp,
                    line_sequence=line_sequence,
                )
                last_status_time = now
            if trace_active:
                profiling_completed = time.perf_counter()
                timings["total_vision_ms"] = (
                    profiling_completed - loop_started
                ) * 1000.0
                try:
                    vision_profiler.publish_sample(
                        line_sequence,
                        line_timestamp,
                        timings,
                        green_raw_interpretation,
                        green_tracker_result[1],
                        profiling_completed,
                    )
                except (OSError, TypeError, ValueError) as error:
                    print(
                        f"Falha ao publicar amostra do trace visual: {error}",
                        flush=True,
                    )
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
