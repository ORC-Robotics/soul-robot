"""Processa a câmera selecionada e publica máscaras e telemetria visual.

Somente o papel ``down`` publica o ponto de extensão ainda zerado do seguidor.
"""

import argparse
import json
import math
import os
import signal
import threading
import time
from functools import lru_cache
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import traceback
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

LIGHT_PIN_BOARD = 40
MJPEG_STREAM_PORT = 8090
MJPEG_STREAM_PATH = "/stream.mjpg"
MJPEG_STREAM_FPS = 30
SNAPSHOT_FRAME_FPS = 2
STATUS_FPS = 5
JPEG_QUALITY = 82
CAMERA_PIXEL_FORMATS = ("RGB888",)
VIRTUAL_ROW_MIN_ACTIVATION = 0.10

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


# Esta chave permite desativar apenas o diagnóstico verde sem alterar a câmera.
GREEN_PROCESSING_ENABLED = environment_flag("GREEN_PROCESSING_ENABLED", True)

# A câmera da pista mostrou que o branco sob iluminação esverdeada chega a
# saturação 136. Exigir 140 preserva o cartão verde saturado e bloqueia esse
# falso positivo antes de qualquer geometria ou decisão autônoma.
GREEN_HUE_MIN = 40
GREEN_HUE_MAX = 90
GREEN_SATURATION_MIN = 140
GREEN_VALUE_MIN = 60
GREEN_OPEN_KERNEL_SIZE = 5
GREEN_CLOSE_KERNEL_SIZE = 5
GREEN_OPEN_ITERATIONS = 2
GREEN_CLOSE_ITERATIONS = 2
# O detector de referência exige que o verde ocupe mais de 6,25% da região
# estrutural. Como a área cresce com a resolução, este limite não fica preso
# aos 4.000 pixels usados originalmente em 320 x 200.
LINE_MIN_COMPONENT_AREA_PX = 120
LINE_MIN_COMPONENT_THICKNESS_PX = 11.0
LINE_MIN_COMPONENT_CORE_RATIO = 0.15
GREEN_MIN_AREA_RATIO = 4000.0 / (320.0 * 200.0)
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
GREEN_FRAGMENT_MERGE_DISTANCE_PX = 12
GREEN_CONFIRMATION_FRAMES = 2
GREEN_SINGLE_OBSERVATION_FRAMES = 2
GREEN_CLEAR_HYSTERESIS_FRAMES = 2
# Cada ROI verde possui 10% da largura da imagem em cada lado. A metade abaixo
# reproduz width // 20 do detector de referência sem fixar a resolução.
GREEN_ROI_HALF_SIZE_DIVISOR = 20
# Pelo menos metade da ROI nominal deve existir dentro da imagem. Uma amostra
# menor poderia aceitar ruído de borda como se fosse a faixa preta.
GREEN_ROI_MIN_VISIBLE_RATIO = 0.50
# Fração mínima de pixels ativos do componente preto em cada ROI.
GREEN_ROI_MIN_BLACK_RATIO = 0.25
# Mantém a orientação durante meio segundo depois da última leitura válida.
GREEN_DIRECTION_RETENTION_SECONDS = 0.5
# Dois marcadores só representam retorno quando estão na mesma altura local.
# A tolerância usa a maior altura observada para acompanhar a perspectiva.
GREEN_PAIR_MAX_VERTICAL_DISTANCE_HEIGHTS = 1.5

# O LED físico pode criar pequenos reflexos brancos dentro da fita preta. Este
# reparo atua somente em ilhas claras completamente cercadas pela máscara preta;
# jamais fecha uma abertura ligada ao fundo, pois ela pode ser uma interrupção real.
SPECULAR_REPAIR_REFERENCE_FRAME_HEIGHT = 360.0
SPECULAR_REPAIR_MAX_DIAMETER_PX = 12.0
SPECULAR_REPAIR_MIN_VALUE = 180
SPECULAR_REPAIR_MAX_SATURATION = 60

VIRTUAL_HEADING_FULL_SCALE_DEG = 30.0
VIRTUAL_HEADING_GAIN = 0.80

# O heading não pode inverter uma leitura lateral clara do NEAR sem que a
# banda MEDIUM também confirme o novo lado observado mais à frente.
VIRTUAL_NEAR_DIRECTION_PROTECTION_THRESHOLD = 0.20

# A correção normal alcança toda a diferença de potência em 0,30.
# Entre 0,30 e a entrada do pivot em 0,50, a faixa forte aumenta o diferencial.
NORMAL_FULL_STEERING_ERROR = 0.30

# A histerese impede alternância rápida entre a faixa forte e o pivot.
# A entrada exige erro alto; a saída ocorre somente após cair até 0,35.
PIVOT_ENTER_THRESHOLD = 0.45
PIVOT_EXIT_THRESHOLD = 0.35

PIVOT_STATE_NONE = "NONE"
PIVOT_STATE_LEFT = "LEFT"
PIVOT_STATE_RIGHT = "RIGHT"

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
    "VERDE_FALSO",
    "AMBIGUO",
}
VISIBLE_GREEN_INTERPRETATIONS = {
    "ESQUERDA",
    "DIREITA",
    "RETORNO_180",
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

# Referência da geometria inferior validada em 640×480. Os kernels e limites
# em pixels são redimensionados pela altura real do frame para não ficarem
# presos a uma resolução específica.
DOWNWARD_REFERENCE_FRAME_HEIGHT = 480

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
            "overlay_line_thickness": 2,
            "overlay_thin_line_thickness": 1,
            "debug_text_overlay": False,
        },
    },
    "down": {
        "role": "down",
        "main_size": (480, 360),
        "sensor_size": (1640, 1232),
        "sensor_bit_depth": 10,
        "target_fps": 30,
        "vision": {
            "line_roi_start_ratio": 0.0,
            # A câmera inferior perdeu a iluminação dedicada. O fechamento
            # grande estima a claridade do piso ao redor da fita e evita que
            # papel branco sombreado seja classificado como linha preta.
            # Valor de referência em 640×480. A criação da máscara escala para
            # a altura recebida: em 480×360, 201 vira 151.
            "line_background_kernel_size": 201,
            # Um pixel precisa estar pelo menos 32% abaixo do fundo local.
            # Aumentar este valor aceita linhas com menos contraste, mas também
            # aumenta o risco de aceitar sombras como parte da linha.
            "line_max_background_ratio_percent": 70,
            # Mesmo com contraste local, tons acima deste limite não são pretos.
            # A unidade é o nível de cinza de 8 bits, entre 0 e 255. Aumentar o
            # limite aceita sombras; reduzir demais pode perder uma fita clara.
            "line_max_brightness": 190,
            # Valores de referência em 640×480; são sempre escalados para
            # kernels ímpares antes da morfologia (17 vira 13 e 7 vira 5).
            "open_kernel_shape": "ellipse",
            "open_kernel_size": 17,
            # O fechamento 7×7 preenche pequenas falhas sem unir objetos
            # separados à linha de 2 cm.
            "close_kernel_size": 11,
            # Vinte pixels mantêm aproximadamente a mesma espessura angular
            # mínima do perfil frontal após o aumento de campo de visão.
            "full_line_min_short_side_ratio": 20.0 / 480.0,
            # Uma linha ou cruzamento normal não deve ocupar mais de 30% da
            # máscara. Componentes maiores indicam sombra ou obstrução e
            # são rejeitados para o robô não seguir um falso contorno.
            "full_line_max_area_ratio": 1.0,
            # As coordenadas usam o frame de referência 640×480 validado.
            # A conversão centralizada mantém a mesma geometria proporcional se
            # a altura real do frame for diferente durante um diagnóstico.
            "geometry_reference": {
                "frame_height": 480,
                "structural_end_y": 400,
            },
            "green_detection_enabled": True,
            "overlay_line_thickness": 2,
            "overlay_thin_line_thickness": 1,
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
    """Mantém o modo visual dentro das duas opções aceitas pelo dashboard."""

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
    structural_mask=None,
):
    """Cria a base exibida sem modificar o frame ou as máscaras da visão."""

    normalized_mode = normalize_display_mode(display_mode)
    if normalized_mode == DISPLAY_MODE_REAL:
        return raw_frame.copy()

    display_frame = np.zeros_like(raw_frame)
    if normalized_mode == DISPLAY_MODE_LINE:
        line_roi = display_frame[roi_start_y:raw_frame.shape[0], :]
        if structural_mask is not None:
            # Cinza mostra preto segmentado, mas rejeitado pelos filtros
            # estruturais. Branco fica reservado à máscara pronta para o
            # futuro seguidor por sensores virtuais.
            line_roi[structural_mask > 0] = (96, 96, 96)
        line_roi[line_candidate_mask > 0] = (255, 255, 255)
        # A máscara HSV verde aparece junto da faixa preta para depuração sem
        # exigir uma segunda aba nem reativar textos sobre o vídeo.
        useful_green_region = display_frame[:green_mask.shape[0], :]
        useful_green_region[green_mask > 0] = (0, 255, 0)
    return display_frame


def draw_green_decision_symbol(display_frame, center, interpretation, accepted):
    """Desenha símbolos geométricos leves, sem renderizar texto no vídeo."""

    color = (64, 255, 96) if accepted else (0, 220, 255)
    center_x, center_y = center
    if interpretation == "RETORNO_180":
        cv2.ellipse(
            display_frame,
            center,
            (10, 10),
            0,
            25,
            330,
            color,
            1,
            cv2.LINE_AA,
        )
        cv2.arrowedLine(
            display_frame,
            (center_x + 7, center_y - 7),
            (center_x + 11, center_y - 2),
            color,
            1,
            cv2.LINE_AA,
            tipLength=0.45,
        )
    elif interpretation in ("ESQUERDA", "DIREITA"):
        direction = -1 if interpretation == "ESQUERDA" else 1
        cv2.arrowedLine(
            display_frame,
            (center_x, center_y + 10),
            (center_x + direction * 14, center_y + 10),
            color,
            1,
            cv2.LINE_AA,
            tipLength=0.40,
        )

    if accepted:
        cv2.line(
            display_frame,
            (center_x - 7, center_y - 7),
            (center_x - 3, center_y - 3),
            color,
            2,
            cv2.LINE_AA,
        )
        cv2.line(
            display_frame,
            (center_x - 3, center_y - 3),
            (center_x + 6, center_y - 12),
            color,
            2,
            cv2.LINE_AA,
        )
    else:
        diamond = np.asarray([
            (center_x, center_y - 6),
            (center_x + 6, center_y),
            (center_x, center_y + 6),
            (center_x - 6, center_y),
        ], dtype=np.int32)
        cv2.polylines(display_frame, [diamond], True, color, 1, cv2.LINE_AA)


def draw_green_candidate_overlays(
    display_frame,
    candidates,
    interpretation,
    accepted,
):
    """Desenha o candidato forte ou a decisão verde já aceita."""

    color = (64, 255, 96) if accepted else (0, 220, 255)
    for candidate in candidates:
        center = tuple(int(round(value)) for value in candidate["centroid"])
        cv2.drawContours(
            display_frame,
            [candidate["contour"]],
            -1,
            color,
            1,
        )
        cv2.circle(display_frame, center, 3, color, -1)
        draw_green_decision_symbol(
            display_frame,
            center,
            interpretation,
            accepted,
        )


def draw_line_mode_green_overlays(
    display_frame,
    candidates,
    interpretation,
    accepted,
):
    """Desenha na máscara de linha o candidato forte ou o verde aceito."""

    for candidate in candidates:
        color = (0, 255, 0) if accepted else (0, 220, 255)
        cv2.drawContours(
            display_frame,
            [candidate["contour"]],
            -1,
            color,
            2,
        )
        center = tuple(int(round(value)) for value in candidate["centroid"])
        cv2.circle(display_frame, center, 3, color, -1)
        draw_green_decision_symbol(
            display_frame,
            center,
            interpretation,
            accepted,
        )


def draw_green_roi_overlays(display_frame, roi_interpretation):
    """Desenha as duas ROIs perpendiculares usadas na classificação."""

    for marker in roi_interpretation.get("markers", []):
        geometry = marker.get("geometry", {})
        upper = marker.get("upper", {})
        horizontal = marker.get("horizontal", {})
        # A ROI superior confirma a associação com a faixa. A horizontal
        # atravessa os dois lados do marcador e localiza a faixa preta.
        roi_entries = (
            (geometry.get("upper_roi"), upper),
            (geometry.get("horizontal_roi"), horizontal),
        )

        for roi, measurement in roi_entries:
            if roi is None:
                continue
            x1, y1, x2, y2 = (int(value) for value in roi)
            if not measurement.get("measured", False):
                color = (160, 160, 160)
            elif measurement.get("valid", False):
                color = (64, 255, 96)
            else:
                color = (0, 220, 255)
            cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 1)


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


@lru_cache(maxsize=16)
def cached_structuring_element(shape, width, height):
    """Reutiliza até 16 kernels imutáveis definidos apenas por sua geometria."""

    return cv2.getStructuringElement(shape, (width, height))


def create_line_binary_mask(gray_roi, vision_profile, timings=None):
    """Separa a fita preta usando o método configurado para cada câmera."""

    if timings is not None:
        timings["backgroundKernelMs"] = 0.0
        timings["backgroundCloseMs"] = 0.0
        timings["binaryCompareMs"] = 0.0
    background_kernel_size = vision_profile.get("line_background_kernel_size")
    if background_kernel_size is None:
        _, binary_mask = cv2.threshold(
            gray_roi,
            vision_profile["line_threshold"],
            255,
            cv2.THRESH_BINARY_INV,
        )
        return binary_mask

    # O fechamento grande remove estruturas escuras menores que o kernel e
    # produz uma estimativa da iluminação do piso. A comparação relativa
    # continua funcionando quando o papel branco fica escuro sem o LED.
    background_kernel_started = (
        time.perf_counter() if timings is not None else 0.0
    )
    background_kernel = cached_structuring_element(
        cv2.MORPH_RECT,
        background_kernel_size,
        background_kernel_size,
    )
    if timings is not None:
        timings["backgroundKernelMs"] = (
            time.perf_counter() - background_kernel_started
        ) * 1000.0
    background_close_started = (
        time.perf_counter() if timings is not None else 0.0
    )
    local_background = cv2.morphologyEx(
        gray_roi,
        cv2.MORPH_CLOSE,
        background_kernel,
    )
    if timings is not None:
        timings["backgroundCloseMs"] = (
            time.perf_counter() - background_close_started
        ) * 1000.0
    ratio_percent = vision_profile["line_max_background_ratio_percent"]
    maximum_brightness = vision_profile["line_max_brightness"]
    binary_compare_started = (
        time.perf_counter() if timings is not None else 0.0
    )
    gray_16 = gray_roi.astype(np.uint16)
    background_16 = local_background.astype(np.uint16)
    np.multiply(gray_16, 100, out=gray_16)
    np.multiply(background_16, ratio_percent, out=background_16)
    relative_mask = cv2.compare(gray_16, background_16, cv2.CMP_LE)
    absolute_mask = cv2.inRange(gray_roi, 0, maximum_brightness)
    cv2.bitwise_and(relative_mask, absolute_mask, dst=relative_mask)
    binary_mask = relative_mask
    if timings is not None:
        timings["binaryCompareMs"] = (
            time.perf_counter() - binary_compare_started
        ) * 1000.0
    return binary_mask


def scaled_odd_kernel_size(reference_size, frame_height):
    """Escala um kernel de referência e mantém o tamanho ímpar mínimo de 3."""

    scaled = int(round(
        float(reference_size) * float(frame_height) /
        DOWNWARD_REFERENCE_FRAME_HEIGHT
    ))
    scaled = max(3, scaled)
    return scaled if scaled % 2 == 1 else scaled + 1


def scaled_reference_pixels(reference_value, frame_height, minimum=1.0):
    """Escala uma distância de referência vertical para o frame processado."""

    scaled = float(reference_value) * float(frame_height) / (
        DOWNWARD_REFERENCE_FRAME_HEIGHT
    )
    return max(float(minimum), scaled)


def repair_small_specular_holes(binary_mask, color_roi, camera_format):
    """Preenche apenas reflexos claros, pequenos e internos à fita preta."""

    repair_status = {
        "specularRepairPixels": 0,
        "specularRepairComponents": 0,
    }
    if binary_mask.size == 0:
        return binary_mask, repair_status

    frame_height = color_roi.shape[0]
    maximum_diameter = max(1, int(round(
        SPECULAR_REPAIR_MAX_DIAMETER_PX * frame_height /
        SPECULAR_REPAIR_REFERENCE_FRAME_HEIGHT
    )))
    maximum_area = maximum_diameter * maximum_diameter
    try:
        hsv_roi = frame_to_hsv(color_roi, camera_format)
    except ValueError:
        # Sem a ordem de cores confirmada, não é seguro classificar um brilho.
        return binary_mask, repair_status

    contours, hierarchy = cv2.findContours(
        binary_mask.copy(),
        cv2.RETR_CCOMP,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    if hierarchy is None:
        return binary_mask, repair_status

    repaired_mask = binary_mask.copy()
    for contour_index, contour in enumerate(contours):
        # No modo CCOMP, apenas os contornos com pai representam buracos
        # fechados. Regiões ligadas ao fundo sempre permanecem de fora.
        if hierarchy[0][contour_index][3] < 0:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        if (width > maximum_diameter or height > maximum_diameter or
                cv2.contourArea(contour) > maximum_area):
            # Uma abertura larga pode ser uma separação real da fita.
            continue

        contour_in_roi = contour.copy()
        contour_in_roi[:, :, 0] -= x
        contour_in_roi[:, :, 1] -= y
        hole_shape = np.zeros((height, width), dtype=np.uint8)
        cv2.drawContours(hole_shape, [contour_in_roi], -1, 255, -1)
        hole_pixels = np.logical_and(
            hole_shape > 0,
            repaired_mask[y:y + height, x:x + width] == 0,
        )
        hole_pixel_count = int(np.count_nonzero(hole_pixels))
        if hole_pixel_count == 0 or hole_pixel_count > maximum_area:
            continue

        hole_hsv = hsv_roi[y:y + height, x:x + width]
        clear_and_unsaturated = np.logical_and(
            hole_hsv[:, :, 2] >= SPECULAR_REPAIR_MIN_VALUE,
            hole_hsv[:, :, 1] <= SPECULAR_REPAIR_MAX_SATURATION,
        )
        if not np.all(clear_and_unsaturated[hole_pixels]):
            # Verde e outras cores saturadas nunca podem virar preto.
            continue

        repaired_mask[y:y + height, x:x + width][hole_pixels] = 255
        repair_status["specularRepairPixels"] += hole_pixel_count
        repair_status["specularRepairComponents"] += 1
    return repaired_mask, repair_status


def create_filtered_line_mask(
    frame,
    vision_profile,
    camera_format="RGB888",
    return_repair_status=False,
    timings=None,
):
    """Segmenta a linha preta na parte inferior sem gerar decisões de controle."""

    frame_height = frame.shape[0]
    roi_start_y = int(round(frame_height * vision_profile["line_roi_start_ratio"]))
    line_roi = frame[roi_start_y:frame_height, :]
    gray_roi = cv2.cvtColor(line_roi, cv2.COLOR_BGR2GRAY)

    scaled_profile = dict(vision_profile)
    if "geometry_reference" in vision_profile:
        if "line_background_kernel_size" in scaled_profile:
            scaled_profile["line_background_kernel_size"] = scaled_odd_kernel_size(
                scaled_profile["line_background_kernel_size"],
                frame_height,
            )
        scaled_profile["open_kernel_size"] = scaled_odd_kernel_size(
            vision_profile["open_kernel_size"], frame_height
        )
        scaled_profile["close_kernel_size"] = scaled_odd_kernel_size(
            vision_profile["close_kernel_size"], frame_height
        )
    binary_started = time.perf_counter() if timings is not None else 0.0
    binary_mask = create_line_binary_mask(
        gray_roi,
        scaled_profile,
        timings=timings,
    )
    if timings is not None:
        timings["binaryMs"] = (
            time.perf_counter() - binary_started
        ) * 1000.0
    specular_started = time.perf_counter() if timings is not None else 0.0

    repair_status = {
        "specularRepairPixels": 0,
        "specularRepairComponents": 0,
    }
    if timings is not None:
        timings["specularMs"] = (
            time.perf_counter() - specular_started
        ) * 1000.0

    open_kernel_shape = (
        cv2.MORPH_ELLIPSE
        if scaled_profile.get("open_kernel_shape") == "ellipse"
        else cv2.MORPH_RECT
    )
    open_kernel = cached_structuring_element(
        open_kernel_shape,
        scaled_profile["open_kernel_size"],
        scaled_profile["open_kernel_size"],
    )
    close_kernel = cached_structuring_element(
        cv2.MORPH_RECT,
        scaled_profile["close_kernel_size"],
        scaled_profile["close_kernel_size"],
    )
    morph_open_started = time.perf_counter() if timings is not None else 0.0
    filtered_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, open_kernel)
    morph_open_ms = (
        (time.perf_counter() - morph_open_started) * 1000.0
        if timings is not None
        else 0.0
    )
    morph_close_started = time.perf_counter() if timings is not None else 0.0
    filtered_mask = cv2.morphologyEx(filtered_mask, cv2.MORPH_CLOSE, close_kernel)
    morph_close_ms = (
        (time.perf_counter() - morph_close_started) * 1000.0
        if timings is not None
        else 0.0
    )
    if timings is not None:
        timings["morphOpenMs"] = morph_open_ms
        timings["morphCloseMs"] = morph_close_ms
        timings["morphMs"] = morph_open_ms + morph_close_ms
    if return_repair_status:
        return filtered_mask, roi_start_y, repair_status
    return filtered_mask, roi_start_y


def scale_reference_y(reference_y, reference_height, frame_height):
    """Converte uma coordenada vertical de referência para a altura real."""

    return int(round(frame_height * reference_y / reference_height))


def resolve_vision_geometry(frame_height, vision_profile):
    """Calcula somente o limite estrutural validado da máscara preta."""

    geometry_reference = vision_profile.get("geometry_reference")
    if geometry_reference is None:
        return {
            "structural_end_y": frame_height,
            "ignored_start_y": None,
            "pixel_scale": 1.0,
        }

    reference_height = geometry_reference["frame_height"]
    structural_end_y = scale_reference_y(
        geometry_reference["structural_end_y"],
        reference_height,
        frame_height,
    )
    if not 0 < structural_end_y <= frame_height:
        raise ValueError("O limite estrutural da câmera está fora do frame.")

    return {
        "structural_end_y": structural_end_y,
        "ignored_start_y": structural_end_y,
        "pixel_scale": float(frame_height) / float(reference_height),
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


def component_has_min_thickness(component_mask):
    """
    Rejeita componentes predominantemente finos,
    como frestas entre placas da pista.

    Não basta existir um único ponto grosso:
    uma fração relevante do componente precisa
    possuir espessura compatível com a fita.
    """

    if component_mask.size == 0:
        return False

    component_pixels = cv2.countNonZero(
        component_mask
    )

    if component_pixels == 0:
        return False

    distance_map = cv2.distanceTransform(
        component_mask,
        cv2.DIST_L2,
        3,
    )

    minimum_radius = (
        LINE_MIN_COMPONENT_THICKNESS_PX
        / 2.0
    )

    thick_core_pixels = int(
        np.count_nonzero(
            distance_map >= minimum_radius
        )
    )

    thick_core_ratio = (
        float(thick_core_pixels)
        / float(component_pixels)
    )

    return (
        thick_core_ratio
        >= LINE_MIN_COMPONENT_CORE_RATIO
    )


def create_line_candidate_mask(
    structural_mask,
    vision_profile,
):
    """Mantém somente componentes compatíveis com a fita preta."""

    full_contours, _ = cv2.findContours(
        structural_mask.copy(),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    accepted_contours = []

    minimum_short_side_px = (
        min(structural_mask.shape[:2])
        * vision_profile[
            "full_line_min_short_side_ratio"
        ]
    )

    maximum_area = (
        structural_mask.size
        * vision_profile.get(
            "full_line_max_area_ratio",
            1.0,
        )
    )

    for contour in full_contours:
        contour_area = cv2.contourArea(
            contour
        )

        # Componentes minúsculos não podem representar
        # uma faixa útil para o robô.
        if (
            contour_area
            < LINE_MIN_COMPONENT_AREA_PX
        ):
            continue

        if contour_area > maximum_area:
            continue

        rect = cv2.minAreaRect(
            contour
        )

        width, height = rect[1]

        if (
            not math.isfinite(width)
            or not math.isfinite(height)
            or width <= 0.0
            or height <= 0.0
        ):
            continue

        short_side_px = min(
            width,
            height,
        )

        if (
            short_side_px
            < minimum_short_side_px
        ):
            continue

        # Analisa a espessura real do próprio componente.
        # Isso evita aceitar uma fresta longa apenas porque
        # sua caixa rotacionada ficou larga.
        component_mask = np.zeros_like(
            structural_mask
        )

        cv2.drawContours(
            component_mask,
            [contour],
            -1,
            255,
            cv2.FILLED,
        )

        if not component_has_min_thickness(
            component_mask
        ):
            continue

        accepted_contours.append(
            contour
        )

    line_candidate_mask = (
        structural_mask.copy()
    )

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
    green_kernel_size = scaled_odd_kernel_size(
        GREEN_OPEN_KERNEL_SIZE, frame.shape[0]
    )
    open_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (green_kernel_size, green_kernel_size),
    )
    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (green_kernel_size, green_kernel_size),
    )
    green_mask = cv2.morphologyEx(
        green_mask,
        cv2.MORPH_OPEN,
        open_kernel,
        iterations=GREEN_OPEN_ITERATIONS,
    )
    return cv2.morphologyEx(
        green_mask,
        cv2.MORPH_CLOSE,
        close_kernel,
        iterations=GREEN_CLOSE_ITERATIONS,
    )


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
        green_kernel_size = scaled_odd_kernel_size(
            GREEN_OPEN_KERNEL_SIZE, frame.shape[0]
        )
        open_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (green_kernel_size, green_kernel_size),
        )
        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (green_kernel_size, green_kernel_size),
        )
        final_mask = cv2.morphologyEx(
            final_mask,
            cv2.MORPH_OPEN,
            open_kernel,
            iterations=GREEN_OPEN_ITERATIONS,
        )
        final_mask = cv2.morphologyEx(
            final_mask,
            cv2.MORPH_CLOSE,
            close_kernel,
            iterations=GREEN_CLOSE_ITERATIONS,
        )
    return hsv_frame, hue_mask, hue_saturation_mask, hsv_mask, final_mask


def expanded_boxes_overlap(first_box, second_box, distance_px):
    """Indica se dois fragmentos podem pertencer à mesma marcação verde."""

    first_x, first_y, first_width, first_height = first_box
    second_x, second_y, second_width, second_height = second_box
    return not (
        first_x + first_width + distance_px < second_x
        or second_x + second_width + distance_px < first_x
        or first_y + first_height + distance_px < second_y
        or second_y + second_height + distance_px < first_y
    )


def group_fragment_boxes(
    boxes,
    distance_px=GREEN_FRAGMENT_MERGE_DISTANCE_PX,
):
    """Agrupa caixas próximas de forma transitiva antes de unir os contornos."""

    groups = []
    for box_index, box in enumerate(boxes):
        matching_groups = []
        for group_index, group in enumerate(groups):
            if any(
                expanded_boxes_overlap(box, boxes[index], distance_px)
                for index in group
            ):
                matching_groups.append(group_index)
        merged_group = [box_index]
        for group_index in reversed(matching_groups):
            merged_group.extend(groups.pop(group_index))
        groups.append(merged_group)
    return groups


def merge_green_fragments(contours, frame_height=DOWNWARD_REFERENCE_FRAME_HEIGHT):
    """Une fragmentos próximos para que um quadrado não seja contado duas vezes."""

    valid_contours = [
        contour
        for contour in contours
        if contour is not None and len(contour) >= 3
    ]
    boxes = [cv2.boundingRect(contour) for contour in valid_contours]
    groups = group_fragment_boxes(
        boxes,
        distance_px=scaled_reference_pixels(
            GREEN_FRAGMENT_MERGE_DISTANCE_PX, frame_height
        ),
    )
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
    frame_height=DOWNWARD_REFERENCE_FRAME_HEIGHT,
):
    """Aplica filtros amplos de tamanho e formato ao marcador oficial."""

    dimension_scale = float(frame_height) / DOWNWARD_REFERENCE_FRAME_HEIGHT
    minimum_area = GREEN_MIN_AREA_PX * dimension_scale * dimension_scale
    minimum_dimension = scaled_reference_pixels(
        GREEN_MIN_DIMENSION_PX, frame_height
    )
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


def contour_touches_useful_border(
    box,
    frame_width,
    useful_height,
    frame_height=DOWNWARD_REFERENCE_FRAME_HEIGHT,
):
    """Marca candidatos parciais próximos de qualquer limite da área útil."""

    x, y, width, height = box
    tolerance = scaled_reference_pixels(
        GREEN_PARTIAL_BORDER_TOLERANCE_PX, frame_height
    )
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
    frame_height=DOWNWARD_REFERENCE_FRAME_HEIGHT,
):
    """Calcula a geometria local do candidato sem referência de controle."""

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
        box, frame_width, useful_height, frame_height
    )

    geometry_valid = green_geometry_is_valid(
        area,
        short_side,
        aspect_ratio,
        extent,
        partial,
        frame_height,
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
        "frameHeight": int(frame_height),
    }


def find_green_candidates(
    frame,
    structural_end_y,
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
    merged_contours = merge_green_fragments(contours, frame.shape[0])
    candidates = []
    rejected = []
    scaled_minimum_area = green_minimum_area(
        frame.shape[1], green_mask.shape[0]
    )
    for contour in merged_contours:
        description = describe_green_contour(
            contour,
            frame.shape[1],
            green_mask.shape[0],
            frame.shape[0],
        )
        description["scaled_minimum_area"] = scaled_minimum_area
        description["scaled_area_valid"] = (
            description["area"] > scaled_minimum_area
        )
        # O teste estrito preserva o comportamento original de area > 4000.
        if (
            not description["scaled_area_valid"]
            or not description["geometry_valid"]
        ):
            rejected.append(description)
            continue
        candidates.append(description)
    candidates.sort(key=lambda candidate: candidate["area"], reverse=True)
    if timings is not None:
        timings["green_contours_ms"] = (
            time.perf_counter() - contours_started
        ) * 1000.0
    return green_mask, candidates, rejected


def green_minimum_area(frame_width, useful_height):
    """Escala os 4.000 px de referência pela área útil da resolução atual."""

    return float(frame_width) * float(useful_height) * GREEN_MIN_AREA_RATIO


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
    frame_height = candidate.get(
        "frameHeight", DOWNWARD_REFERENCE_FRAME_HEIGHT
    )
    dimension_scale = float(frame_height) / DOWNWARD_REFERENCE_FRAME_HEIGHT
    minimum_area = GREEN_MIN_AREA_PX * dimension_scale * dimension_scale
    minimum_dimension = scaled_reference_pixels(
        GREEN_MIN_DIMENSION_PX, frame_height
    )
    minimum_aspect = GREEN_ASPECT_RATIO_MIN
    minimum_extent = GREEN_MIN_EXTENT
    if partial:
        minimum_area *= GREEN_PARTIAL_AREA_FACTOR
        minimum_dimension *= GREEN_PARTIAL_DIMENSION_FACTOR
        minimum_aspect = GREEN_PARTIAL_ASPECT_RATIO_MIN
        minimum_extent = GREEN_PARTIAL_EXTENT_MIN

    reasons = []
    if not candidate.get("scaled_area_valid", True):
        reasons.append("area_below_scaled_reference")
    if not math.isfinite(candidate["area"]) or candidate["area"] < minimum_area:
        reasons.append("area_below_minimum")
    if candidate["short_side"] < minimum_dimension:
        reasons.append("dimension_below_minimum")
    if not minimum_aspect <= candidate["aspect_ratio"] <= GREEN_ASPECT_RATIO_MAX:
        reasons.append("aspect_ratio_outside_range")
    if candidate["extent"] < minimum_extent:
        reasons.append("extent_below_minimum")
    return reasons


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
        geometry_reasons = green_geometry_rejection_reasons(candidate)
        reason = "geometria válida" if accepted else ", ".join(geometry_reasons)
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
            "accepted": accepted,
            "rejected": not accepted,
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
            "fragment_merge_distance_px": GREEN_FRAGMENT_MERGE_DISTANCE_PX,
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
        "interpretation": interpretation.get("interpretation", "SEM_DECISAO"),
        "components": component_stats,
    }


def save_green_capture(
    frame,
    camera_format,
    mask_stages,
    candidates,
    rejected,
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


def green_observation_state(candidate_count):
    """Converte a quantidade de marcações geométricas no estado de observação."""

    if candidate_count <= 0:
        return "SEM_VERDE"
    if candidate_count == 1:
        return "UM_CANDIDATO"
    if candidate_count == 2:
        return "DOIS_CANDIDATOS"
    return "MULTIPLOS_AMBIGUOS"


def empty_green_status():
    """Cria um estado verde finito e seguro para publicação diagnóstica."""

    return {
        "greenObservationState": "SEM_VERDE",
        "greenInterpretation": "SEM_DECISAO",
        "greenConfirmed": False,
        "greenRawInterpretation": "SEM_DECISAO",
        "greenDecisionState": "idle",
        "greenPathBlackValid": False,
        "greenCandidateCount": 0,
        "greenRejectedCount": 0,
        "greenMarkerCount": 0,
        "greenValidatedMarkerCount": 0,
        "greenFrontRoiMeasured": False,
        "greenFrontBlackRatio": 0.0,
        "greenFrontRoiValid": False,
        "greenLeftRoiMeasured": False,
        "greenLeftBlackRatio": 0.0,
        "greenLeftRoiValid": False,
        "greenRightRoiMeasured": False,
        "greenRightBlackRatio": 0.0,
        "greenRightRoiValid": False,
        "greenLeftSeen": False,
        "greenRightSeen": False,
        "greenPairCompatible": False,
        "greenPrimaryX": 0.0,
        "greenPrimaryY": 0.0,
        "greenPrimaryArea": 0.0,
        "greenSecondaryX": 0.0,
        "greenSecondaryY": 0.0,
        "greenSecondaryArea": 0.0,
        "greenConsecutiveSamples": 0,
        "greenProcessingMs": 0.0,
    }


def green_marker_roi_geometry(contour, frame_width):
    """Calcula as ROIs horizontal e superior ao redor do marcador verde."""

    box = cv2.boxPoints(cv2.minAreaRect(contour))
    minimum_x = float(np.min(box[:, 0]))
    maximum_x = float(np.max(box[:, 0]))
    minimum_y = float(np.min(box[:, 1]))
    maximum_y = float(np.max(box[:, 1]))
    center_x = int(round((minimum_x + maximum_x) / 2.0))
    center_y = int(round((minimum_y + maximum_y) / 2.0))
    half_size = max(1, int(frame_width) // GREEN_ROI_HALF_SIZE_DIVISOR)

    # A ROI horizontal cruza o marcador e mede separadamente somente as partes
    # externas à esquerda e à direita. Assim existe uma única ROI lateral.
    horizontal_roi = (
        int(round(minimum_x)) - 2 * half_size,
        center_y - half_size,
        int(round(maximum_x)) + 2 * half_size,
        center_y + half_size,
    )
    # A ROI superior é perpendicular à horizontal. Ela deve ter amostra
    # visível antes que o marcador possa ser chamado de verdadeiro ou falso.
    upper_roi = (
        center_x - half_size,
        int(round(minimum_y)) - 3 * half_size,
        center_x + half_size,
        int(round(minimum_y)),
    )
    return {
        "box": box,
        "center": (center_x, center_y),
        "marker_horizontal_bounds": (
            int(round(minimum_x)),
            int(round(maximum_x)),
        ),
        "horizontal_roi": horizontal_roi,
        "upper_roi": upper_roi,
    }


def measure_black_roi(black_mask, roi):
    """Mede preto somente quando ao menos metade da ROI permanece na imagem."""

    x1, y1, x2, y2 = (int(value) for value in roi)
    nominal_width = max(0, x2 - x1)
    nominal_height = max(0, y2 - y1)
    nominal_area = nominal_width * nominal_height
    if black_mask is None or black_mask.size == 0 or nominal_area <= 0:
        return {
            "measured": False,
            "valid": False,
            "black_ratio": 0.0,
            "visible_ratio": 0.0,
        }

    clipped_x1 = max(0, min(black_mask.shape[1], x1))
    clipped_y1 = max(0, min(black_mask.shape[0], y1))
    clipped_x2 = max(0, min(black_mask.shape[1], x2))
    clipped_y2 = max(0, min(black_mask.shape[0], y2))
    visible_width = max(0, clipped_x2 - clipped_x1)
    visible_height = max(0, clipped_y2 - clipped_y1)
    visible_area = visible_width * visible_height
    visible_ratio = visible_area / nominal_area
    if visible_area <= 0 or visible_ratio < GREEN_ROI_MIN_VISIBLE_RATIO:
        return {
            "measured": False,
            "valid": False,
            "black_ratio": 0.0,
            "visible_ratio": visible_ratio,
        }

    region = black_mask[clipped_y1:clipped_y2, clipped_x1:clipped_x2]
    black_ratio = float(np.count_nonzero(region > 0)) / visible_area
    return {
        "measured": True,
        "valid": black_ratio >= GREEN_ROI_MIN_BLACK_RATIO,
        "black_ratio": black_ratio,
        "visible_ratio": visible_ratio,
    }


def measure_horizontal_black_roi(black_mask, geometry):
    """Mede os dois lados dentro da única ROI horizontal."""

    x1, y1, x2, y2 = geometry["horizontal_roi"]
    marker_left_x, marker_right_x = geometry["marker_horizontal_bounds"]
    left = measure_black_roi(
        black_mask,
        (x1, y1, marker_left_x, y2),
    )
    right = measure_black_roi(
        black_mask,
        (marker_right_x, y1, x2, y2),
    )
    return {
        "measured": left["measured"] and right["measured"],
        "valid": left["valid"] or right["valid"],
        "left_measured": left["measured"],
        "left_valid": left["valid"],
        "left_black_ratio": left["black_ratio"],
        "right_measured": right["measured"],
        "right_valid": right["valid"],
        "right_black_ratio": right["black_ratio"],
    }


def analyze_green_marker_contours(green_contours, selected_black_mask):
    """Classifica o verde pelas ROIs horizontal e superior."""

    result = {
        "interpretation": "SEM_DECISAO",
        "observation_state": "SEM_VERDE",
        "left_seen": False,
        "right_seen": False,
        "pair_compatible": False,
        "path_black_valid": False,
        "markers": [],
    }
    if selected_black_mask is None or selected_black_mask.size == 0:
        return result

    upper_valid_markers = []
    upper_measurement_missing = False
    for contour in green_contours:
        geometry = green_marker_roi_geometry(
            contour,
            selected_black_mask.shape[1],
        )
        upper_measurement = measure_black_roi(
            selected_black_mask,
            geometry["upper_roi"],
        )
        marker = {
            "contour": contour,
            "geometry": geometry,
            "upper": upper_measurement,
            "horizontal": measure_horizontal_black_roi(
                selected_black_mask,
                geometry,
            ),
        }
        result["markers"].append(marker)
        if upper_measurement["valid"]:
            upper_valid_markers.append(marker)
        elif not upper_measurement["measured"]:
            upper_measurement_missing = True

    result["observation_state"] = green_observation_state(len(green_contours))
    if green_contours and upper_measurement_missing:
        # Nenhuma classificação verdadeira ou falsa é publicada enquanto
        # uma ROI superior ainda não possui área visível suficiente.
        result["interpretation"] = "AMBIGUO"
        return result
    if not upper_valid_markers:
        if green_contours:
            result["interpretation"] = "VERDE_FALSO"
        return result

    for marker in upper_valid_markers:
        horizontal = marker["horizontal"]
        left_valid = horizontal["left_valid"]
        right_valid = horizontal["right_valid"]
        marker["interpretation"] = (
            "AMBIGUO" if not horizontal["measured"] else
            "DIREITA" if left_valid and not right_valid else
            "ESQUERDA" if right_valid and not left_valid else
            "AMBIGUO"
        )

    if len(upper_valid_markers) > 2:
        result["interpretation"] = "AMBIGUO"
        return result

    if len(upper_valid_markers) == 2:
        first, second = upper_valid_markers
        first_center = np.mean(first["geometry"]["box"], axis=0)
        second_center = np.mean(second["geometry"]["box"], axis=0)
        first_height = float(np.ptp(first["geometry"]["box"][:, 1]))
        second_height = float(np.ptp(second["geometry"]["box"][:, 1]))
        vertical_tolerance = (
            GREEN_PAIR_MAX_VERTICAL_DISTANCE_HEIGHTS
            * max(1.0, first_height, second_height)
        )
        directions = {
            first["interpretation"],
            second["interpretation"],
        }
        pair_compatible = (
            directions == {"ESQUERDA", "DIREITA"}
            and abs(float(first_center[1] - second_center[1]))
            <= vertical_tolerance
        )
        result["left_seen"] = "ESQUERDA" in directions
        result["right_seen"] = "DIREITA" in directions
        result["pair_compatible"] = pair_compatible
        if not pair_compatible:
            result["interpretation"] = "AMBIGUO"
            return result
        result.update({
            "interpretation": "RETORNO_180",
            "left_seen": True,
            "right_seen": True,
            "pair_compatible": True,
            "path_black_valid": True,
        })
        return result

    marker = upper_valid_markers[0]
    interpretation = marker["interpretation"]
    result["interpretation"] = interpretation
    result["left_seen"] = interpretation == "ESQUERDA"
    result["right_seen"] = interpretation == "DIREITA"
    result["path_black_valid"] = interpretation in (
        "ESQUERDA",
        "DIREITA",
    )
    return result

VIRTUAL_FAR_Y0 = 0.00
VIRTUAL_FAR_Y1 = 0.54

"""rois virtuais para o seguidor de linha, em coordenadas normalizadas"""
VIRTUAL_NEAR_Y0 = 0.54
VIRTUAL_NEAR_Y1 = 0.83

# As duas novas bandas dividem somente a inteligência de curva. O FAR legado
# acima continua cobrindo 0,00–0,54 diretamente e não é reconstruído por elas.
VIRTUAL_FAR_BAND_Y0 = 0.00
VIRTUAL_FAR_BAND_Y1 = 0.27
VIRTUAL_MEDIUM_Y0 = 0.27
VIRTUAL_MEDIUM_Y1 = 0.54
# Divisão horizontal dos três sensores.
#
# Existe uma pequena sobreposição entre L/C e C/R.
#
# 0.00                                      1.00
# ├──────── L ────────┤
#              ├──────── C ────────┤
#                           ├──────── R ────────┤ - isx art

VIRTUAL_LEFT_X0 = 0.04
VIRTUAL_LEFT_X1 = 0.385

VIRTUAL_CENTER_X0 = 0.385
VIRTUAL_CENTER_X1 = 0.615

VIRTUAL_RIGHT_X0 = 0.615
VIRTUAL_RIGHT_X1 = 0.96

# O scan usa somente uma direção clara do MEDIUM e termina após três frames.
VIRTUAL_MEDIUM_SCAN_POSITION_THRESHOLD = 0.20
VIRTUAL_MEDIUM_SCAN_MAX_FRAMES = 3

# MEDIUM e FAR BAND precisam concordar claramente antes do pivot de recovery.
VIRTUAL_REORIENT_DIRECTION_THRESHOLD = 0.20
VIRTUAL_REORIENT_CONFIRMATION_FRAMES = 4

# O steering normal precisa reaparecer em dois frames antes de encerrar o
# estado persistente, mas recebe autoridade já no primeiro frame válido.
VIRTUAL_REORIENT_RECOVERY_FRAMES = 2

VIRTUAL_STATE_NORMAL = "NORMAL"
VIRTUAL_STATE_REORIENT_LEFT = "REORIENT_LEFT"
VIRTUAL_STATE_REORIENT_RIGHT = "REORIENT_RIGHT"

# Controle da prioridade de direção após um verde confirmado.
#
# A direção permanece memorizada durante toda a curva.
# ESQUERDA e DIREITA usam os sensores virtuais para selecionar
# somente o ramo permitido da interseção.
#
# Após terminar a curva, novos verdes continuam bloqueados até
# que nenhum candidato verde seja visto por esta quantidade
# de quadros consecutivos.
QUADROS_PARA_REARMAR_VERDE = 60

# A curva é considerada iniciada quando o NEAR se desloca
# suficientemente para o lado escolhido.
LIMIAR_CURVA_VERDE_INICIADA = 0.20

# Após a curva ter começado, o retorno do NEAR para esta região
# central indica que o robô entrou e se alinhou com a nova faixa.
LIMIAR_CENTRALIZACAO_VERDE = 0.18
# Evita encerrar a prioridade por uma leitura central isolada.
QUADROS_CENTRALIZADO_PARA_CONCLUIR = 4

# A manobra verde não pode manter a máscara de controle indefinidamente.
# Em 30 FPS, noventa frames correspondem a aproximadamente três segundos.
GREEN_MANEUVER_TIMEOUT_FRAMES = 60

# A busca cega começa no último lado confiável por uma janela curta e depois
# varre o lado oposto por mais tempo. O ciclo se repete até a linha reaparecer.
VIRTUAL_BLIND_SEARCH_INITIAL_FRAMES = 12
VIRTUAL_BLIND_SEARCH_REVERSE_FRAMES = 30

# Reaquisição geométrica após gaps.
#
# Quando não existe faixa no NEAR, procura uma continuação válida
# mais à frente antes de declarar a trajetória completamente perdida.
GEOMETRIC_GAP_FORWARD_MAX_FRAMES = 45
GEOMETRIC_GAP_REACQUIRE_FRAMES = 2
GEOMETRIC_GAP_SEARCH_STEP_PX = 6
GEOMETRIC_GAP_MAX_SEARCH_PX = 120

# Mantém por poucos quadros a evidência de que o NEAR RAW estava visível.
# A memória permite reconhecer o início de um GAP sem depender de uma única
# amostra geométrica na altura exata usada pelo rastreador preservado.
GAP_NEAR_HISTORY_FRAMES = 3

# Uma faixa encontrada após o gap precisa continuar também nesta
# distância para não aceitarmos um pequeno blob isolado como caminho.
GEOMETRIC_GAP_CONFIRM_OFFSET_PX = 12
GEOMETRIC_GAP_PROJECTION_POINTS = 5
# Limita quanto o centro da faixa pode mudar entre a primeira
# detecção após o gap e sua amostra de confirmação.
GEOMETRIC_GAP_MAX_CENTER_SHIFT_PX = 50
# Geometria preservada exclusivamente para detectar e reaquistar GAP.
# Estes parâmetros não antecipam curvas nem alteram o controle LINE normal.
GEOMETRIC_PATH_BAND_HALF_HEIGHT = 2
GEOMETRIC_PATH_LOCAL_HEADING_POINTS = 5

# Distância aproximada entre pontos consecutivos da trajetória.
GEOMETRIC_TRACE_STEP_PX = 10.0

# Permite ao rastreador procurar o centro da faixa um pouco
# antes ou depois da distância nominal de avanço.
GEOMETRIC_TRACE_SEARCH_RANGE_PX = 5.0

GEOMETRIC_TRACE_MAX_BACKTRACK_Y_PX = 10.0

# Permite inclusive uma mudança de 90 graus entre dois passos.
# Nunca permite continuar para trás.
GEOMETRIC_TRACE_MAX_TURN_DEG = 90.0

# Limite de segurança contra caminhos que entrem em ciclos.
GEOMETRIC_TRACE_MAX_POINTS = 80

# Peso dado ao progresso em direção a uma saída lateral
# inequívoca do componente da faixa.
GEOMETRIC_TRACE_TARGET_PROGRESS_GAIN = 2.0

# Pequena margem usada para considerar que a faixa alcançou
# uma borda da imagem.
GEOMETRIC_TRACE_EXIT_MARGIN_PX = 3

# FAR e NEAR continuam presos às alturas extremas escolhidas.
GEOMETRIC_PATH_FAR_Y_RATIO = 0.00
GEOMETRIC_PATH_NEAR_Y_RATIO = VIRTUAL_NEAR_Y1

def resolve_virtual_sensor_geometry(frame_shape):
    """
    Converte o FAR/NEAR legado e as novas bandas para pixels.

    O retângulo FAR original é calculado diretamente para preservar
    exatamente a leitura usada pelo seguidor base.
    """

    height, width = frame_shape[:2]

    far_y0 = int(round(height * VIRTUAL_FAR_Y0))
    far_y1 = int(round(height * VIRTUAL_FAR_Y1))

    far_band_y0 = int(round(height * VIRTUAL_FAR_BAND_Y0))
    far_band_y1 = int(round(height * VIRTUAL_FAR_BAND_Y1))

    medium_y0 = int(round(height * VIRTUAL_MEDIUM_Y0))
    medium_y1 = int(round(height * VIRTUAL_MEDIUM_Y1))

    near_y0 = int(round(height * VIRTUAL_NEAR_Y0))
    near_y1 = int(round(height * VIRTUAL_NEAR_Y1))

    left_x0 = int(round(width * VIRTUAL_LEFT_X0))
    left_x1 = int(round(width * VIRTUAL_LEFT_X1))

    center_x0 = int(round(width * VIRTUAL_CENTER_X0))
    center_x1 = int(round(width * VIRTUAL_CENTER_X1))

    right_x0 = int(round(width * VIRTUAL_RIGHT_X0))
    right_x1 = int(round(width * VIRTUAL_RIGHT_X1))

    return {
        "far": {
            "left": {
                "x0": left_x0,
                "y0": far_y0,
                "x1": left_x1,
                "y1": far_y1,
            },
            "center": {
                "x0": center_x0,
                "y0": far_y0,
                "x1": center_x1,
                "y1": far_y1,
            },
            "right": {
                "x0": right_x0,
                "y0": far_y0,
                "x1": right_x1,
                "y1": far_y1,
            },
        },

        "farBand": {
            "left": {
                "x0": left_x0,
                "y0": far_band_y0,
                "x1": left_x1,
                "y1": far_band_y1,
            },
            "center": {
                "x0": center_x0,
                "y0": far_band_y0,
                "x1": center_x1,
                "y1": far_band_y1,
            },
            "right": {
                "x0": right_x0,
                "y0": far_band_y0,
                "x1": right_x1,
                "y1": far_band_y1,
            },
        },

        "medium": {
            "left": {
                "x0": left_x0,
                "y0": medium_y0,
                "x1": left_x1,
                "y1": medium_y1,
            },
            "center": {
                "x0": center_x0,
                "y0": medium_y0,
                "x1": center_x1,
                "y1": medium_y1,
            },
            "right": {
                "x0": right_x0,
                "y0": medium_y0,
                "x1": right_x1,
                "y1": medium_y1,
            },
        },

        "near": {
            "left": {
                "x0": left_x0,
                "y0": near_y0,
                "x1": left_x1,
                "y1": near_y1,
            },
            "center": {
                "x0": center_x0,
                "y0": near_y0,
                "x1": center_x1,
                "y1": near_y1,
            },
            "right": {
                "x0": right_x0,
                "y0": near_y0,
                "x1": right_x1,
                "y1": near_y1,
            },
        },
    }

def draw_virtual_sensor_geometry(
    frame,
    line_follower_command,
):
    """
    Desenha as bandas FAR, MEDIUM e NEAR com o estado virtual atual.

    O heading continua usando o FAR legado, embora seus retângulos não
    sejam repetidos sobre as nove regiões exibidas.
    """

    geometry = resolve_virtual_sensor_geometry(
        frame.shape
    )

    sensors = (
        (
            "FB-L",
            geometry["farBand"]["left"],
            line_follower_command["farBandLeft"],
        ),
        (
            "FB-C",
            geometry["farBand"]["center"],
            line_follower_command["farBandCenter"],
        ),
        (
            "FB-R",
            geometry["farBand"]["right"],
            line_follower_command["farBandRight"],
        ),
        (
            "MED-L",
            geometry["medium"]["left"],
            line_follower_command["mediumLeft"],
        ),
        (
            "MED-C",
            geometry["medium"]["center"],
            line_follower_command["mediumCenter"],
        ),
        (
            "MED-R",
            geometry["medium"]["right"],
            line_follower_command["mediumRight"],
        ),
        (
            "NEAR-L",
            geometry["near"]["left"],
            line_follower_command["nearLeft"],
        ),
        (
            "NEAR-C",
            geometry["near"]["center"],
            line_follower_command["nearCenter"],
        ),
        (
            "NEAR-R",
            geometry["near"]["right"],
            line_follower_command["nearRight"],
        ),
    )

    for name, sensor, value in sensors:
        cv2.rectangle(
            frame,
            (sensor["x0"], sensor["y0"]),
            (sensor["x1"], sensor["y1"]),
            (255, 0, 255),
            2,
        )

        cv2.putText(
            frame,
            f"{name} {value:.2f}",
            (
                sensor["x0"] + 8,
                sensor["y0"] + 24,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 0, 255),
            1,
            cv2.LINE_AA,
        )

    far_position = line_follower_command["farPosition"]
    far_band_position = line_follower_command["farBandPosition"]
    medium_position = line_follower_command["mediumPosition"]
    near_position = line_follower_command["nearPosition"]

    far_band_text = (
        f"FAR BAND POS {far_band_position:+.2f}"
        if far_band_position is not None
        else "FAR BAND POS INVALID"
    )

    medium_text = (
        f"MEDIUM POS {medium_position:+.2f}"
        if medium_position is not None
        else "MEDIUM POS INVALID"
    )

    near_text = (
        f"NEAR POS {near_position:+.2f}"
        if near_position is not None
        else "NEAR POS INVALID"
    )

    position_texts = (
        (far_band_text, geometry["farBand"]["center"]),
        (medium_text, geometry["medium"]["center"]),
        (near_text, geometry["near"]["center"]),
    )
    for position_text, position_geometry in position_texts:
        cv2.putText(
            frame,
            position_text,
            (
                position_geometry["x0"] + 8,
                position_geometry["y0"] + 48,
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
    heading_angle = line_follower_command["headingAngle"]
    far_point = virtual_row_position_to_point(
        far_position,
        geometry["far"],
    )

    near_point = virtual_row_position_to_point(
        near_position,
        geometry["near"],
    )

    if far_point is not None and near_point is not None:
        far_point_int = (
            int(round(far_point[0])),
            int(round(far_point[1])),
        )

        near_point_int = (
            int(round(near_point[0])),
            int(round(near_point[1])),
        )

        cv2.line(
            frame,
            near_point_int,
            far_point_int,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.circle(
            frame,
            near_point_int,
            5,
            (0, 255, 255),
            -1,
        )

        cv2.circle(
            frame,
            far_point_int,
            5,
            (0, 255, 255),
            -1,
        )

    heading_text = (
        f"HEADING {heading_angle:+.1f} deg"
        if heading_angle is not None
        else "HEADING INVALID"
    )

    cv2.putText(
        frame,
        heading_text,
        (
            geometry["near"]["center"]["x0"] + 8,
            geometry["near"]["center"]["y0"] + 72,
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 255, 255),
        1,
        cv2.LINE_AA,
    )
    steering_error = line_follower_command["steeringError"]
    steering_text = (
        f"STEERING {steering_error:+.2f}"
        if steering_error is not None
        else "STEERING INVALID"
    )

    cv2.putText(
        frame,
        steering_text,
        (
            geometry["near"]["center"]["x0"] + 8,
            geometry["near"]["center"]["y0"] + 96,
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 255, 255),
        1,
        cv2.LINE_AA,
    )
    line_processing_ms = line_follower_command.get(
        "lineProcessingMs", 0.0,
    )
    cv2.putText(
        frame,
        f"{line_processing_ms:.2f} ms",
        (
            geometry["near"]["center"]["x0"] + 8,
            geometry["near"]["center"]["y0"] + 120,
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 255, 255),
        1,
        cv2.LINE_AA,
    )

    virtual_state_labels = {
        VIRTUAL_STATE_NORMAL: "NORMAL",
        VIRTUAL_STATE_REORIENT_LEFT: "REORIENT LEFT",
        VIRTUAL_STATE_REORIENT_RIGHT: "REORIENT RIGHT",
    }
    virtual_state = line_follower_command.get(
        "virtualState",
        VIRTUAL_STATE_NORMAL,
    )
    cv2.putText(
        frame,
        f"VSTATE {virtual_state_labels.get(virtual_state, 'NORMAL')}",
        (8, 88),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        (0, 220, 255),
        1,
        cv2.LINE_AA,
    )

    green_direction = line_follower_command.get(
        "greenDirection",
        "NENHUMA",
    )
    cv2.putText(
        frame,
        f"GREEN DIR {green_direction}",
        (8, 110),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        (0, 220, 255),
        1,
        cv2.LINE_AA,
    )

def find_active_band_segments(processed_line_mask, y):
    """
    Encontra os segmentos horizontais da faixa preta em uma pequena
    banda ao redor de uma determinada altura da imagem.

    Cada segmento retornado contém o centro horizontal ponderado
    pela quantidade real de pixels da máscara.
    """

    height, width = processed_line_mask.shape[:2]

    y0 = max(
        0,
        y - GEOMETRIC_PATH_BAND_HALF_HEIGHT,
    )
    y1 = min(
        height,
        y + GEOMETRIC_PATH_BAND_HALF_HEIGHT + 1,
    )

    band = processed_line_mask[y0:y1, :]

    if band.size == 0:
        return []

    active_columns = np.any(band > 0, axis=0)
    active_x = np.flatnonzero(active_columns)

    if active_x.size == 0:
        return []

    split_indices = np.where(
        np.diff(active_x) > 1
    )[0] + 1

    groups = np.split(
        active_x,
        split_indices,
    )

    segments = []

    for group in groups:
        if group.size == 0:
            continue

        x0 = int(group[0])
        x1 = int(group[-1])

        column_weights = np.count_nonzero(
            band[:, x0:x1 + 1],
            axis=0,
        ).astype(np.float32)

        total_weight = float(column_weights.sum())

        if total_weight <= 0.0:
            continue

        columns = np.arange(
            x0,
            x1 + 1,
            dtype=np.float32,
        )

        center_x = float(
            np.sum(columns * column_weights)
            / total_weight
        )

        segments.append({
            "x0": x0,
            "x1": x1,
            "centerX": center_x,
        })

    return segments

def find_geometric_lateral_exit(
    processed_line_mask,
    start_point,
):
    """
    Detecta uma saída lateral inequívoca do mesmo componente
    conectado que contém o início da trajetória.

    Se o componente alcançar o topo, mantém o comportamento
    geométrico normal.

    Se tocar somente a esquerda ou somente a direita, retorna
    um alvo naquela borda.

    Se houver ambiguidade, não interfere no rastreamento.
    """

    height, width = processed_line_mask.shape[:2]

    binary_mask = (
        processed_line_mask > 0
    ).astype(np.uint8)

    _, labels = cv2.connectedComponents(
        binary_mask,
        connectivity=8,
    )

    start_x = max(
        0,
        min(
            width - 1,
            int(round(start_point[0])),
        ),
    )

    start_y = max(
        0,
        min(
            height - 1,
            int(round(start_point[1])),
        ),
    )

    component_label = int(
        labels[start_y, start_x]
    )

    if component_label == 0:
        return None

    margin = GEOMETRIC_TRACE_EXIT_MARGIN_PX

    top_region = labels[
        0:min(height, margin + 1),
        :
    ]

    # Se a própria faixa chega ao topo, o tracer normal já
    # possui uma continuação natural para frente.
    if np.any(
        top_region == component_label
    ):
        return None

    left_region = labels[
        :,
        0:min(width, margin + 1),
    ]

    right_region = labels[
        :,
        max(0, width - margin - 1):width,
    ]

    left_positions = np.argwhere(
        left_region == component_label
    )

    right_positions = np.argwhere(
        right_region == component_label
    )

    touches_left = (
        left_positions.size > 0
    )

    touches_right = (
        right_positions.size > 0
    )

    # Nenhuma saída lateral ou duas saídas:
    # geometria ambígua, então não escolhemos por conta própria.
    if touches_left == touches_right:
        return None

    if touches_left:
        target_y = float(
            np.mean(left_positions[:, 0])
        )

        return (
            0.0,
            target_y,
        )

    target_y = float(
        np.mean(right_positions[:, 0])
    )

    return (
        float(width - 1),
        target_y,
    )

def find_geometric_gap_start(
    processed_line_mask,
    near_y,
):
    """
    Procura uma continuação válida da faixa à frente quando o
    NEAR está vazio.

    A primeira banda encontrada só é aceita se existir uma segunda
    amostra coerente um pouco mais à frente. Isso ajuda a rejeitar
    manchas pretas isoladas.

    Não cria pixels nem preenche o gap. Apenas encontra onde a
    faixa real reaparece.
    """

    height, width = processed_line_mask.shape[:2]

    image_center_x = (
        float(width - 1) / 2.0
    )

    minimum_y = max(
        0,
        near_y - GEOMETRIC_GAP_MAX_SEARCH_PX,
    )

    search_y = (
        near_y - GEOMETRIC_GAP_SEARCH_STEP_PX
    )

    while search_y >= minimum_y:
        segments = find_active_band_segments(
            processed_line_mask,
            search_y,
        )

        if not segments:
            search_y -= (
                GEOMETRIC_GAP_SEARCH_STEP_PX
            )
            continue

        confirmation_y = max(
            0,
            search_y
            - GEOMETRIC_GAP_CONFIRM_OFFSET_PX,
        )

        confirmation_segments = (
            find_active_band_segments(
                processed_line_mask,
                confirmation_y,
            )
        )

        if not confirmation_segments:
            search_y -= (
                GEOMETRIC_GAP_SEARCH_STEP_PX
            )
            continue

        valid_candidates = []

        for segment in segments:
            center_x = float(
                segment["centerX"]
            )

            confirmation_segment = min(
                confirmation_segments,
                key=lambda candidate: abs(
                    float(candidate["centerX"])
                    - center_x
                ),
            )

            confirmation_x = float(
                confirmation_segment["centerX"]
            )

            center_shift = abs(
                confirmation_x - center_x
            )

            if (
                center_shift
                <= GEOMETRIC_GAP_MAX_CENTER_SHIFT_PX
            ):
                valid_candidates.append(
                    segment
                )

        if valid_candidates:
            selected_segment = min(
                valid_candidates,
                key=lambda segment: abs(
                    float(segment["centerX"])
                    - image_center_x
                ),
            )

            start_point = (
                float(
                    selected_segment["centerX"]
                ),
                float(search_y),
            )

            return start_point

        search_y -= (
            GEOMETRIC_GAP_SEARCH_STEP_PX
        )

    return None

def find_next_geometric_path_point(
    distance_map,
    current_point,
    direction,
    target_point=None,
):
    """
    Procura o próximo ponto central da faixa ao redor do ponto atual.

    A busca acontece em uma coroa circular, permitindo que a
    trajetória avance em qualquer direção até 90 graus em relação
    à direção atual.

    O distance transform favorece pontos mais distantes das bordas,
    ou seja, próximos do eixo central da faixa.
    """

    height, width = distance_map.shape[:2]

    current_x = float(current_point[0])
    current_y = float(current_point[1])

    step = GEOMETRIC_TRACE_STEP_PX
    search_range = GEOMETRIC_TRACE_SEARCH_RANGE_PX

    minimum_step = max(
        2.0,
        step - search_range,
    )

    maximum_step = (
        step + search_range
    )

    search_radius = int(
        math.ceil(maximum_step)
    )

    center_x = int(round(current_x))
    center_y = int(round(current_y))

    x0 = max(
        0,
        center_x - search_radius,
    )
    x1 = min(
        width,
        center_x + search_radius + 1,
    )

    y0 = max(
        0,
        center_y - search_radius,
    )
    y1 = min(
        height,
        center_y + search_radius + 1,
    )

    local_distance = distance_map[
        y0:y1,
        x0:x1,
    ]

    if local_distance.size == 0:
        return None

    local_y, local_x = np.nonzero(
        local_distance > 0.0
    )

    if local_x.size == 0:
        return None

    candidate_x = (
        local_x.astype(np.float32)
        + float(x0)
    )

    candidate_y = (
        local_y.astype(np.float32)
        + float(y0)
    )

    delta_x = (
        candidate_x - current_x
    )

    delta_y = (
        candidate_y - current_y
    )

    radial_distance = np.hypot(
        delta_x,
        delta_y,
    )

    valid_distance = (
        (radial_distance >= minimum_step)
        & (radial_distance <= maximum_step)
    )

    safe_distance = np.maximum(
        radial_distance,
        1e-6,
    )

    alignment = (
        delta_x * float(direction[0])
        + delta_y * float(direction[1])
    ) / safe_distance

    minimum_alignment = math.cos(
        math.radians(
            GEOMETRIC_TRACE_MAX_TURN_DEG
        )
    )

    # Tolerância numérica para que uma mudança exatamente
    # perpendicular seja realmente aceita quando o limite é 90°.
    alignment_tolerance = 1e-6

    valid_direction = (
        alignment >= minimum_alignment - alignment_tolerance
    )

    valid = (
        valid_distance
        & valid_direction
    )

    if not np.any(valid):
        return None

    center_strength = local_distance[
        local_y,
        local_x,
    ].astype(np.float32)

    # A distância ao contorno é medida em pixels e pode crescer
    # muito dentro de regiões pretas largas. Normalizamos somente
    # entre os candidatos geometricamente válidos para que uma
    # grande massa preta não domine a continuidade da trajetória.
    maximum_center_strength = float(
        np.max(
            center_strength[valid]
        )
    )

    if maximum_center_strength > 0.0:
        center_strength = (
            center_strength
            / maximum_center_strength
        )

    # Prioridades:
    #
    # 1. manter continuidade com a direção atual da faixa;
    # 2. preferir o centro físico entre caminhos coerentes;
    # 3. manter aproximadamente o passo nominal.
    score = (
        center_strength
        + alignment * 2.0
        - np.abs(
            radial_distance - step
        ) * 0.10
    )

    if target_point is not None:
        target_x = float(
            target_point[0]
        )

        target_y = float(
            target_point[1]
        )

        current_target_distance = (
            math.hypot(
                target_x - current_x,
                target_y - current_y,
            )
        )

        candidate_target_distance = (
            np.hypot(
                target_x - candidate_x,
                target_y - candidate_y,
            )
        )

        # Positivo = candidato aproxima da saída correta.
        # Negativo = candidato se afasta dela.
        target_progress = (
            current_target_distance
            - candidate_target_distance
        )

        score += (
            target_progress
            * GEOMETRIC_TRACE_TARGET_PROGRESS_GAIN
        )

    score[~valid] = -np.inf

    best_index = int(
        np.argmax(score)
    )

    next_x = float(
        candidate_x[best_index]
    )

    next_y = float(
        candidate_y[best_index]
    )

    movement_x = (
        next_x - current_x
    )

    movement_y = (
        next_y - current_y
    )

    movement_length = math.hypot(
        movement_x,
        movement_y,
    )

    if movement_length <= 0.0:
        return None

    next_direction = (
        movement_x / movement_length,
        movement_y / movement_length,
    )

    return (
        (next_x, next_y),
        next_direction,
    )

def estimate_geometric_initial_direction(
    processed_line_mask,
    near_point,
):
    """
    Estima a direção inicial real da faixa a partir do NEAR.

    Usa duas pequenas amostras à frente do NEAR e acompanha
    o segmento mais próximo entre elas. Depois ajusta uma reta
    aos pontos encontrados.

    Caso não exista informação suficiente, mantém o fallback
    seguro apontando para a frente da câmera.
    """

    height, _ = processed_line_mask.shape[:2]

    near_x = float(near_point[0])
    near_y = float(near_point[1])

    direction_points = [
        (
            near_x,
            near_y,
        )
    ]

    previous_x = near_x

    for multiplier in (
        1.0,
        2.0,
    ):
        sample_y = int(round(
            near_y
            - GEOMETRIC_TRACE_STEP_PX
            * multiplier
        ))

        if sample_y < 0:
            break

        if sample_y >= height:
            continue

        segments = find_active_band_segments(
            processed_line_mask,
            sample_y,
        )

        if not segments:
            break

        selected_segment = min(
            segments,
            key=lambda segment: abs(
                segment["centerX"]
                - previous_x
            ),
        )

        sample_x = float(
            selected_segment["centerX"]
        )

        direction_points.append(
            (
                sample_x,
                float(sample_y),
            )
        )

        previous_x = sample_x

    if len(direction_points) < 2:
        return (
            0.0,
            -1.0,
        )

    fit_points = np.asarray(
        direction_points,
        dtype=np.float32,
    ).reshape(-1, 1, 2)

    vx, vy, _, _ = cv2.fitLine(
        fit_points,
        cv2.DIST_L2,
        0,
        0.01,
        0.01,
    ).flatten()

    vx = float(vx)
    vy = float(vy)

    reference_x = (
        direction_points[-1][0]
        - near_x
    )

    reference_y = (
        direction_points[-1][1]
        - near_y
    )

    # O cv2.fitLine não possui sentido definido.
    # Orienta o vetor do NEAR em direção à faixa à frente.
    if (
        vx * reference_x
        + vy * reference_y
    ) < 0.0:
        vx = -vx
        vy = -vy

    length = math.hypot(
        vx,
        vy,
    )

    if length <= 0.0:
        return (
            0.0,
            -1.0,
        )

    return (
        vx / length,
        vy / length,
    )

def project_geometric_gap_to_near(
    path_points,
    near_y,
    frame_width,
):
    """
    Projeta a direção local da faixa reaparecida através do gap
    até a altura fixa do NEAR.

    O ponto retornado é apenas uma estimativa geométrica.
    Ele não representa pixels realmente observados na máscara.
    """

    point_count = min(
        len(path_points),
        GEOMETRIC_GAP_PROJECTION_POINTS,
    )

    if point_count < 2:
        return None

    local_points = np.asarray(
        path_points[:point_count],
        dtype=np.float32,
    ).reshape(-1, 1, 2)

    vx, vy, _, _ = cv2.fitLine(
        local_points,
        cv2.DIST_L2,
        0,
        0.01,
        0.01,
    ).flatten()

    vx = float(vx)
    vy = float(vy)

    first_point = path_points[0]
    last_point = path_points[point_count - 1]

    reference_x = (
        float(last_point[0])
        - float(first_point[0])
    )

    reference_y = (
        float(last_point[1])
        - float(first_point[1])
    )

    # O cv2.fitLine não define o sentido do vetor.
    # Orienta a direção do começo da faixa para o FAR.
    if (
        vx * reference_x
        + vy * reference_y
    ) < 0.0:
        vx = -vx
        vy = -vy

    # Uma direção praticamente horizontal não possui uma
    # interseção estável com a altura fixa do NEAR.
    if abs(vy) <= 1e-6:
        return None

    start_x = float(first_point[0])
    start_y = float(first_point[1])

    scale = (
        float(near_y) - start_y
    ) / vy

    projected_x = (
        start_x + vx * scale
    )

    if not math.isfinite(projected_x):
        return None

    # Se a continuação atingiria o NEAR fora da imagem,
    # não fingimos possuir uma referência utilizável.
    if (
        projected_x < 0.0
        or projected_x > float(frame_width - 1)
    ):
        return None

    return (
        projected_x,
        float(near_y),
    )

def calculate_geometric_far_heading(path_points):
    """
    Calcula a direção local no final da trajetória disponível.

    Não exige que a trajetória alcance o FAR superior:
    também funciona quando ela sai lateralmente da imagem.

    0 graus representa seguir para a frente.
    Valor positivo aponta para a direita.
    Valor negativo aponta para a esquerda.
    """

    point_count = min(
        len(path_points),
        GEOMETRIC_PATH_LOCAL_HEADING_POINTS,
    )

    if point_count < 2:
        return None

    local_points = path_points[
        -point_count:
    ]

    fit_points = np.asarray(
        local_points,
        dtype=np.float32,
    ).reshape(-1, 1, 2)

    vx, vy, _, _ = cv2.fitLine(
        fit_points,
        cv2.DIST_L2,
        0,
        0.01,
        0.01,
    ).flatten()

    vx = float(vx)
    vy = float(vy)

    first_point = local_points[0]
    last_point = local_points[-1]

    reference_x = (
        float(last_point[0])
        - float(first_point[0])
    )

    reference_y = (
        float(last_point[1])
        - float(first_point[1])
    )

    # O fitLine não possui sentido definido.
    # Orienta o vetor no mesmo sentido em que a trajetória
    # foi percorrida, do NEAR em direção ao futuro.
    if (
        vx * reference_x
        + vy * reference_y
    ) < 0.0:
        vx = -vx
        vy = -vy

    return float(
        math.degrees(
            math.atan2(
                vx,
                -vy,
            )
        )
    )

def extract_geometric_line_path(processed_line_mask):
    """
    Rastreia a faixa somente para travessia e reaquisição de GAP.

    Diferentemente da versão baseada em bandas horizontais,
    os pontos intermediários não possuem Y fixo. O caminho pode
    avançar verticalmente, diagonalmente ou lateralmente.

    NEAR e FAR continuam presos às alturas de referência.
    """

    height, width = processed_line_mask.shape[:2]

    far_y = int(round(
        height * GEOMETRIC_PATH_FAR_Y_RATIO
    ))

    near_y = int(round(
        height * GEOMETRIC_PATH_NEAR_Y_RATIO
    ))

    far_y = max(
        0,
        min(height - 1, far_y),
    )

    near_y = max(
        0,
        min(height - 1, near_y),
    )

    # No início, escolhe a faixa mais próxima do centro físico
    # da câmera. Depois disso, a própria continuidade geométrica
    # decide o caminho.
    image_center_x = (
        float(width - 1) / 2.0
    )

    near_segments = find_active_band_segments(
        processed_line_mask,
        near_y,
    )

    gap_reacquired = False
    if near_segments:
        near_segment = min(
            near_segments,
            key=lambda segment: abs(
                float(segment["centerX"])
                - image_center_x
            ),
        )

        start_point = (
            float(near_segment["centerX"]),
            float(near_y),
        )

        # Há observação real exatamente no NEAR.
        near_point = start_point

    else:
        gap_start = find_geometric_gap_start(
            processed_line_mask,
            near_y,
        )

        if gap_start is None:
            return {
                "nearPoint": None,
                "farHeadingDeg": None,
                "virtualNearPoint": None,
                "lateralExitTarget": None,
            }

        start_point = gap_start

        gap_reacquired = True

        # Não fingimos que existe uma medição no NEAR.
        # A trajetória começa onde a faixa reaparece.
        near_point = None

    lateral_exit_target = (
        find_geometric_lateral_exit(
            processed_line_mask,
            start_point,
        )
    )

    # Quanto maior o valor, mais longe este pixel está das bordas
    # da faixa. Os máximos locais formam aproximadamente seu eixo.
    distance_map = cv2.distanceTransform(
        processed_line_mask,
        cv2.DIST_L2,
        3,
    )

    path_points = [
        start_point
    ]

    current_point = start_point

    # A direção inicial é medida na própria geometria da faixa.
    # Depois do primeiro passo, o rastreador continua atualizando
    # a direção normalmente pelos pontos encontrados.
    direction = estimate_geometric_initial_direction(
        processed_line_mask,
        start_point,
    )

    maximum_step = (
        GEOMETRIC_TRACE_STEP_PX
        + GEOMETRIC_TRACE_SEARCH_RANGE_PX
    )

    virtual_near_point = None

    for _ in range(
        GEOMETRIC_TRACE_MAX_POINTS - 1
    ):
        current_x = float(
            current_point[0]
        )

        current_y = float(
            current_point[1]
        )

        if lateral_exit_target is not None:
            distance_to_lateral_exit = math.hypot(
                float(lateral_exit_target[0])
                - current_x,
                float(lateral_exit_target[1])
                - current_y,
            )

            if distance_to_lateral_exit <= maximum_step:
                if distance_to_lateral_exit > 1.0:
                    path_points.append(
                        lateral_exit_target
                    )

                break

        # Quando a trajetória chega suficientemente perto do FAR,
        # tenta conectar ao centro real da faixa exatamente no Y
        # superior de referência.
        if current_y <= (
            far_y + maximum_step
        ):
            far_segments = (
                find_active_band_segments(
                    processed_line_mask,
                    far_y,
                )
            )

            if far_segments:
                selected_far_segment = min(
                    far_segments,
                    key=lambda segment: abs(
                        segment["centerX"]
                        - current_x
                    ),
                )

                candidate_far_point = (
                    float(
                        selected_far_segment[
                            "centerX"
                        ]
                    ),
                    float(far_y),
                )

                distance_to_far = math.hypot(
                    (
                        candidate_far_point[0]
                        - current_x
                    ),
                    (
                        candidate_far_point[1]
                        - current_y
                    ),
                )

                if distance_to_far <= (
                    maximum_step * 2.0
                ):
                    if distance_to_far > 1.0:
                        path_points.append(
                            candidate_far_point
                        )

                    break

        next_result = (
            find_next_geometric_path_point(
                distance_map,
                current_point,
                direction,
                lateral_exit_target,
            )
        )

        if next_result is None:
            break

        next_point, next_direction = (
            next_result
        )

        path_points.append(
            next_point
        )

        current_point = (
            next_point
        )

        direction = (
            next_direction
        )

    if gap_reacquired:
        virtual_near_point = (
            project_geometric_gap_to_near(
                path_points,
                near_y,
                width,
            )
        )

    far_heading = calculate_geometric_far_heading(
        path_points,
    )

    return {
        "nearPoint": near_point,
        "farHeadingDeg": far_heading,
        "virtualNearPoint": virtual_near_point,
        "lateralExitTarget": lateral_exit_target,
    }

def draw_line_control_overlay(frame, line_follower_command):
    """Desenha somente o comando e os estados usados pelo controle atual."""

    processing_ms = finite_virtual_position(
        line_follower_command.get("lineProcessingMs")
    )
    processing_text = (
        f"LINE {processing_ms:.1f}ms"
        if processing_ms is not None
        else "LINE --ms"
    )
    control_source = str(
        line_follower_command.get("controlSource", "unknown")
    ).strip().upper() or "UNKNOWN"
    line_state = str(
        line_follower_command.get("lineState", "INVALID")
    ).strip().upper() or "INVALID"
    virtual_state = str(
        line_follower_command.get("virtualState", "INVALID")
    ).strip().upper() or "INVALID"

    overlay_texts = (
        processing_text,
        (
            f"L {float(line_follower_command['left_power']):.2f}  "
            f"R {float(line_follower_command['right_power']):.2f}"
        ),
        f"STATE {line_state}  VSTATE {virtual_state}",
        f"CONTROL SOURCE: {control_source}",
    )
    overlay_text_start_y = max(
        22,
        frame.shape[0] - (len(overlay_texts) - 1) * 22 - 6,
    )
    for line_index, overlay_text in enumerate(overlay_texts):
        cv2.putText(
            frame,
            overlay_text,
            (8, overlay_text_start_y + line_index * 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )


def read_virtual_sensor(processed_line_mask, sensor_geometry):
    """
    Mede quanto da área de um sensor virtual está ocupada
    pela máscara final da linha preta.

    Retorno:
        0.0 = nenhuma linha no sensor
        1.0 = sensor completamente ocupado pela linha
    """

    x0 = sensor_geometry["x0"]
    y0 = sensor_geometry["y0"]
    x1 = sensor_geometry["x1"]
    y1 = sensor_geometry["y1"]

    sensor_roi = processed_line_mask[
        y0:y1,
        x0:x1,
    ]

    if sensor_roi.size == 0:
        return 0.0

    active_pixels = cv2.countNonZero(sensor_roi)

    return float(active_pixels) / float(sensor_roi.size)

def calculate_virtual_row_position(
    left,
    center,
    right,
):
    """
    Converte três sensores analógicos L/C/R
    em uma posição lateral contínua.

    -1.0 = esquerda
     0.0 = centro
    +1.0 = direita

    Retorna None quando nenhuma linha é observada.
    """

    total = left + center + right

    if total < VIRTUAL_ROW_MIN_ACTIVATION:
        return None

    position = (
        -left + right
    ) / total

    return float(position)

def read_virtual_line_sensors(
    processed_line_mask,
    direcao_verde_ativa="NENHUMA",
):
    """
    Preserva o FAR/NEAR legado e lê FAR BAND/MEDIUM separadamente.
    """

    geometry = resolve_virtual_sensor_geometry(
        processed_line_mask.shape
    )

    far_left = read_virtual_sensor(
        processed_line_mask,
        geometry["far"]["left"],
    )

    far_center = read_virtual_sensor(
        processed_line_mask,
        geometry["far"]["center"],
    )

    far_right = read_virtual_sensor(
        processed_line_mask,
        geometry["far"]["right"],
    )

    near_left = read_virtual_sensor(
        processed_line_mask,
        geometry["near"]["left"],
    )

    near_center = read_virtual_sensor(
        processed_line_mask,
        geometry["near"]["center"],
    )

    near_right = read_virtual_sensor(
        processed_line_mask,
        geometry["near"]["right"],
    )

    far_band_left = read_virtual_sensor(
        processed_line_mask,
        geometry["farBand"]["left"],
    )

    far_band_center = read_virtual_sensor(
        processed_line_mask,
        geometry["farBand"]["center"],
    )

    far_band_right = read_virtual_sensor(
        processed_line_mask,
        geometry["farBand"]["right"],
    )

    medium_left = read_virtual_sensor(
        processed_line_mask,
        geometry["medium"]["left"],
    )

    medium_center = read_virtual_sensor(
        processed_line_mask,
        geometry["medium"]["center"],
    )

    medium_right = read_virtual_sensor(
        processed_line_mask,
        geometry["medium"]["right"],
    )

    raw_far_position = calculate_virtual_row_position(
        far_left,
        far_center,
        far_right,
    )
    raw_near_position = calculate_virtual_row_position(
        near_left,
        near_center,
        near_right,
    )

    control_far_left = far_left
    control_far_center = far_center
    control_far_right = far_right
    control_near_left = near_left
    control_near_center = near_center
    control_near_right = near_right

    # Durante uma interseção sinalizada por verde, somente o ramo
    # permitido deve influenciar a antecipação do FAR.
    #
    # As cópias de controle preservam no overlay as leituras RAW da câmera.
    if direcao_verde_ativa == "ESQUERDA":
        control_far_center = 0.0
        control_far_right = 0.0
        control_near_right = 0.0

    elif direcao_verde_ativa == "DIREITA":
        control_far_left = 0.0
        control_far_center = 0.0
        control_near_left = 0.0

    far_position = calculate_virtual_row_position(
        control_far_left,
        control_far_center,
        control_far_right,
    )

    near_position = calculate_virtual_row_position(
        control_near_left,
        control_near_center,
        control_near_right,
    )

    far_band_position = calculate_virtual_row_position(
        far_band_left,
        far_band_center,
        far_band_right,
    )

    medium_position = calculate_virtual_row_position(
        medium_left,
        medium_center,
        medium_right,
    )

    heading_angle = calculate_virtual_heading_angle(
    far_position,
    near_position,
    geometry,
)
    steering_error = calculate_virtual_steering_error(
    near_position,
    heading_angle,
)
    if direcao_verde_ativa == "NENHUMA":
        steering_error = protect_virtual_near_direction(
            near_position,
            medium_position,
            heading_angle,
            steering_error,
        )

    return {
        "farLeft": far_left,
        "farCenter": far_center,
        "farRight": far_right,
        "farPosition": far_position,
        "rawFarPosition": raw_far_position,

        "farBandLeft": far_band_left,
        "farBandCenter": far_band_center,
        "farBandRight": far_band_right,
        "farBandPosition": far_band_position,

        "mediumLeft": medium_left,
        "mediumCenter": medium_center,
        "mediumRight": medium_right,
        "mediumPosition": medium_position,

        "nearLeft": near_left,
        "nearCenter": near_center,
        "nearRight": near_right,
        "nearPosition": near_position,
        "rawNearPosition": raw_near_position,
        "headingAngle": heading_angle,
        "steeringError": steering_error,
    }

def virtual_reorient_direction(sensors):
    """Detecta concordância lateral entre MEDIUM e FAR BAND."""

    medium_position = sensors.get("mediumPosition")
    far_band_position = sensors.get("farBandPosition")
    if medium_position is None or far_band_position is None:
        return None

    threshold = VIRTUAL_REORIENT_DIRECTION_THRESHOLD
    if medium_position >= threshold and far_band_position >= threshold:
        return "RIGHT"
    if medium_position <= -threshold and far_band_position <= -threshold:
        return "LEFT"
    return None


def virtual_medium_scan_direction(sensors):
    """Retorna o lado confiável observado somente pelo MEDIUM."""

    medium_position = sensors.get("mediumPosition")
    if medium_position is None:
        return None
    try:
        medium_position = float(medium_position)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(medium_position):
        return None
    if medium_position <= -VIRTUAL_MEDIUM_SCAN_POSITION_THRESHOLD:
        return "LEFT"
    if medium_position >= VIRTUAL_MEDIUM_SCAN_POSITION_THRESHOLD:
        return "RIGHT"
    return None


def finite_virtual_position(value):
    """Valida uma posição virtual antes de compará-la com thresholds."""

    if value is None:
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def virtual_recovery_sensor_direction(sensors):
    """Escolhe uma direção lateral RAW na ordem NEAR, MEDIUM e FAR BAND."""

    positions = (
        sensors.get("rawNearPosition", sensors.get("nearPosition")),
        sensors.get("mediumPosition"),
        sensors.get("farBandPosition"),
    )
    for position in positions:
        position = finite_virtual_position(position)
        if position is None:
            continue
        if position <= -VIRTUAL_MEDIUM_SCAN_POSITION_THRESHOLD:
            return "LEFT"
        if position >= VIRTUAL_MEDIUM_SCAN_POSITION_THRESHOLD:
            return "RIGHT"
    return None


def virtual_raw_line_is_visible(sensors):
    """Indica se alguma das três fileiras RAW possui posição utilizável."""

    positions = (
        sensors.get("rawNearPosition", sensors.get("nearPosition")),
        sensors.get("mediumPosition"),
        sensors.get("farBandPosition"),
    )
    return any(finite_virtual_position(position) is not None for position in positions)


def update_gap_recent_near_frames(recent_near_frames, raw_near_visible):
    """Atualiza a memória curta de uma observação real no NEAR RAW."""

    if raw_near_visible:
        return GAP_NEAR_HISTORY_FRAMES
    return max(0, int(recent_near_frames) - 1)


def gap_entry_is_required(
    gap_forward_active,
    green_direction,
    recent_near_frames,
    raw_near_visible,
    real_near_point,
    virtual_near_point,
    lateral_exit_target,
):
    """Reconhece a perda recente do NEAR sem disputar prioridade com verde."""

    return (
        not gap_forward_active
        and green_direction == "NENHUMA"
        and recent_near_frames > 0
        and not raw_near_visible
        and real_near_point is None
        and virtual_near_point is None
        and lateral_exit_target is None
    )


class VirtualLineSearchTracker:
    """Alterna uma busca cega curta e outra maior sem memorizar steering."""

    def __init__(self):
        self.last_direction = None
        self.active = False
        self.initial_direction = None
        self.search_frames = 0

    def remember(self, direction):
        """Guarda somente uma direção lateral realmente observada."""

        if direction in ("LEFT", "RIGHT"):
            self.last_direction = direction

    def stop(self):
        """Interrompe a busca sem apagar a última direção confiável."""

        self.active = False
        self.initial_direction = None
        self.search_frames = 0

    def start(self, preferred_direction=None):
        """Inicia a busca uma única vez com a melhor direção disponível."""

        if self.active:
            return
        initial_direction = (
            preferred_direction
            if preferred_direction in ("LEFT", "RIGHT")
            else self.last_direction
        )
        self.initial_direction = initial_direction or "RIGHT"
        self.active = True
        self.search_frames = 0

    def next_direction(self):
        """Retorna o lado da janela atual e avança um frame."""

        if not self.active:
            return None
        cycle_frames = (
            VIRTUAL_BLIND_SEARCH_INITIAL_FRAMES
            + VIRTUAL_BLIND_SEARCH_REVERSE_FRAMES
        )
        cycle_index = self.search_frames % cycle_frames
        if cycle_index < VIRTUAL_BLIND_SEARCH_INITIAL_FRAMES:
            direction = self.initial_direction
        else:
            direction = (
                "RIGHT" if self.initial_direction == "LEFT" else "LEFT"
            )
        self.search_frames += 1
        return direction


def update_gap_forward_recovery(
    active,
    forward_frames,
    reacquire_frames,
    line_lost_seen,
    near_reacquired,
):
    """Atualiza o GAP sem permitir que MEDIUM ou FAR encerrem a travessia."""

    if not active:
        return {
            "active": False,
            "forwardFrames": 0,
            "reacquireFrames": 0,
            "lineLostSeen": False,
            "blindSearchRequested": False,
        }

    forward_frames += 1
    if not near_reacquired:
        line_lost_seen = True
    if line_lost_seen and near_reacquired:
        reacquire_frames += 1
    else:
        reacquire_frames = 0

    if reacquire_frames >= GEOMETRIC_GAP_REACQUIRE_FRAMES:
        return {
            "active": False,
            "forwardFrames": 0,
            "reacquireFrames": 0,
            "lineLostSeen": False,
            "blindSearchRequested": False,
        }

    return {
        "active": True,
        "forwardFrames": forward_frames,
        "reacquireFrames": reacquire_frames,
        "lineLostSeen": line_lost_seen,
        "blindSearchRequested": (
            forward_frames >= GEOMETRIC_GAP_FORWARD_MAX_FRAMES
        ),
    }


def update_green_maneuver_state(
    direction,
    active_frames,
    raw_line_visible,
    completed=False,
):
    """Conclui o verde normalmente ou libera sua máscara após o timeout."""

    if direction == "NENHUMA":
        return {
            "direction": "NENHUMA",
            "activeFrames": 0,
            "timedOut": False,
            "searchDirection": None,
        }
    if completed:
        return {
            "direction": "NENHUMA",
            "activeFrames": 0,
            "timedOut": False,
            "searchDirection": None,
        }

    active_frames += 1
    if active_frames < GREEN_MANEUVER_TIMEOUT_FRAMES:
        return {
            "direction": direction,
            "activeFrames": active_frames,
            "timedOut": False,
            "searchDirection": None,
        }
    return {
        "direction": "NENHUMA",
        "activeFrames": 0,
        "timedOut": True,
        "searchDirection": None if raw_line_visible else direction,
    }


class VirtualTurnStateTracker:
    """Mantém somente o pivot temporário usado para recuperar a linha."""

    def __init__(self):
        self.reset()

    def reset(self):
        """Retorna ao seguidor normal e limpa todas as confirmações."""

        self.state = VIRTUAL_STATE_NORMAL
        self.reorient_candidate = None
        self.reorient_frames = 0
        self.normal_recovery_frames = 0
        self.medium_scan_frames = 0
        return self.state

    def allow_medium_scan(self, direction):
        """Consome no máximo três frames de scan enquanto permanece NORMAL."""

        if direction is None or self.state != VIRTUAL_STATE_NORMAL:
            return False
        if self.medium_scan_frames >= VIRTUAL_MEDIUM_SCAN_MAX_FRAMES:
            return False
        self.medium_scan_frames += 1
        return True

    def update(self, sensors, steering_valid):
        """Confirma o recovery e devolve autoridade ao steering normal."""

        if steering_valid:
            self.reorient_candidate = None
            self.reorient_frames = 0
            self.medium_scan_frames = 0
            if self.state in (
                VIRTUAL_STATE_REORIENT_LEFT,
                VIRTUAL_STATE_REORIENT_RIGHT,
            ):
                self.normal_recovery_frames += 1
                if (
                    self.normal_recovery_frames
                    >= VIRTUAL_REORIENT_RECOVERY_FRAMES
                ):
                    return self.reset()
                return self.state
            return self.reset()

        self.normal_recovery_frames = 0
        if self.state in (
            VIRTUAL_STATE_REORIENT_LEFT,
            VIRTUAL_STATE_REORIENT_RIGHT,
        ):
            return self.state

        direction = virtual_reorient_direction(sensors)
        if direction is None:
            self.reorient_candidate = None
            self.reorient_frames = 0
            return self.state

        if direction == self.reorient_candidate:
            self.reorient_frames += 1
        else:
            self.reorient_candidate = direction
            self.reorient_frames = 1

        if self.reorient_frames >= VIRTUAL_REORIENT_CONFIRMATION_FRAMES:
            self.state = (
                VIRTUAL_STATE_REORIENT_LEFT
                if direction == "LEFT"
                else VIRTUAL_STATE_REORIENT_RIGHT
            )
            self.medium_scan_frames = 0
        return self.state


class VirtualPivotStateTracker:
    """Mantém o lado do pivot normal e aplica histerese sem troca direta."""

    def __init__(self):
        self.state = PIVOT_STATE_NONE

    def reset(self):
        """Encerra o pivot persistente antes de outro modo assumir."""

        self.state = PIVOT_STATE_NONE
        return self.state

    def update(self, steering_error):
        """Atualiza a entrada ou saída do pivot usando o erro do frame atual."""

        steering_error = finite_virtual_position(steering_error)
        if steering_error is None:
            return self.reset()

        steering_magnitude = abs(steering_error)
        if self.state == PIVOT_STATE_RIGHT:
            if (
                steering_error <= 0.0
                or steering_magnitude <= PIVOT_EXIT_THRESHOLD
            ):
                return self.reset()
            return self.state

        if self.state == PIVOT_STATE_LEFT:
            if (
                steering_error >= 0.0
                or steering_magnitude <= PIVOT_EXIT_THRESHOLD
            ):
                return self.reset()
            return self.state

        if steering_error >= PIVOT_ENTER_THRESHOLD:
            self.state = PIVOT_STATE_RIGHT
        elif steering_error <= -PIVOT_ENTER_THRESHOLD:
            self.state = PIVOT_STATE_LEFT
        return self.state


def calculate_line_follower_command(
    processed_line_mask,
    green_detection_result,
    direcao_verde_ativa="NENHUMA",
    gap_forward_active=False,
    virtual_turn_tracker=None,
    pivot_state_tracker=None,
    virtual_sensors=None,
    line_search_tracker=None,
    blind_search_requested=False,
    sensor_recovery_requested=False,
):
    """
    Aplica o seguidor virtual validado pela câmera inferior.

    Verde e gap mantêm prioridade. No modo normal, somente o steering dos nove
    sensores virtuais, já protegido por NEAR/MEDIUM, chega ao mapper.
    """

    _ = green_detection_result

    sensors = (
        virtual_sensors
        if isinstance(virtual_sensors, dict)
        else read_virtual_line_sensors(
            processed_line_mask,
            direcao_verde_ativa,
        )
    )
    observed_recovery_direction = virtual_recovery_sensor_direction(sensors)
    raw_line_visible = virtual_raw_line_is_visible(sensors)
    if line_search_tracker is not None:
        line_search_tracker.remember(observed_recovery_direction)

    protected_virtual_steering = sensors["steeringError"]
    virtual_state = VIRTUAL_STATE_NORMAL
    medium_scan_direction = None
    direct_recovery_direction = None
    normal_steering_mapper = False
    line_state = "LINE"

    if direcao_verde_ativa != "NENHUMA":
        line_state = "GREEN"
        if virtual_turn_tracker is not None:
            virtual_turn_tracker.reset()
        if line_search_tracker is not None:
            line_search_tracker.stop()
        steering_error = sensors[
            "steeringError"
        ]
        control_source = "virtual-green"

    elif gap_forward_active:
        line_state = "GAP"
        if virtual_turn_tracker is not None:
            virtual_turn_tracker.reset()
        if observed_recovery_direction is not None:
            if line_search_tracker is not None:
                line_search_tracker.stop()
            steering_error = None
            direct_recovery_direction = observed_recovery_direction
            control_source = "gap-sensor-recovery"
        elif raw_line_visible:
            # Qualquer linha RAW encerra a busca cega, mas somente o NEAR
            # confirmado pode encerrar o estado GAP fora deste mapper.
            if line_search_tracker is not None:
                line_search_tracker.stop()
            steering_error = 0.0
            control_source = "gap-forward"
        else:
            if line_search_tracker is not None and blind_search_requested:
                line_search_tracker.start()
            blind_direction = (
                line_search_tracker.next_direction()
                if line_search_tracker is not None
                else None
            )
            if blind_direction == "LEFT":
                steering_error = -1.0
                control_source = "gap-blind-search"
            elif blind_direction == "RIGHT":
                steering_error = 1.0
                control_source = "gap-blind-search"
            else:
                steering_error = 0.0
                control_source = "gap-forward"

    else:
        normal_steering_valid = protected_virtual_steering is not None
        if virtual_turn_tracker is not None:
            virtual_state = virtual_turn_tracker.update(
                sensors,
                normal_steering_valid,
            )

        if normal_steering_valid:
            # O seguidor normal recupera autoridade no primeiro frame válido,
            # mesmo enquanto o tracker confirma a saída do REORIENT.
            steering_error = protected_virtual_steering
            normal_steering_mapper = True
            if line_search_tracker is not None:
                line_search_tracker.stop()
            control_source = "virtual"
        else:
            if virtual_state == VIRTUAL_STATE_REORIENT_LEFT:
                steering_error = -1.0
                control_source = "virtual-reorient"
            elif virtual_state == VIRTUAL_STATE_REORIENT_RIGHT:
                steering_error = 1.0
                control_source = "virtual-reorient"
            elif (
                raw_line_visible
                and line_search_tracker is not None
                and (
                    line_search_tracker.active
                    or sensor_recovery_requested
                )
            ):
                line_search_tracker.stop()
                if observed_recovery_direction is None:
                    steering_error = 0.0
                else:
                    steering_error = None
                    direct_recovery_direction = observed_recovery_direction
                control_source = "virtual-sensor-recovery"
            elif (
                line_search_tracker is not None
                and line_search_tracker.active
            ):
                blind_direction = line_search_tracker.next_direction()
                steering_error = -1.0 if blind_direction == "LEFT" else 1.0
                control_source = "virtual-blind-search"
            else:
                steering_error = None
                observed_medium_direction = virtual_medium_scan_direction(
                    sensors
                )
                if (
                    virtual_turn_tracker is not None
                    and virtual_turn_tracker.allow_medium_scan(
                        observed_medium_direction
                    )
                ):
                    medium_scan_direction = observed_medium_direction
                    control_source = "virtual-medium-scan"
                else:
                    if (
                        observed_recovery_direction is None
                        and not raw_line_visible
                        and line_search_tracker is not None
                    ):
                        line_search_tracker.start()
                        blind_direction = line_search_tracker.next_direction()
                        steering_error = (
                            -1.0 if blind_direction == "LEFT" else 1.0
                        )
                        control_source = "virtual-blind-search"
                    else:
                        control_source = "virtual-no-line"

    # --------------------------------------------------------
    # CONTROLE DE MOTORES
    # --------------------------------------------------------

    BASE_POWER = 0.69
    MAX_POWER = 0.75
    NORMAL_INNER_MIN_POWER = 0.66

    # A faixa forte amplia a curva sem reduzir nenhuma roda abaixo de 0,61.
    STRONG_TURN_OUTER_POWER = 0.85
    STRONG_TURN_INNER_MIN_POWER = 0.61

    # Verde, GAP e recoveries preservam o limite usado antes da histerese.
    NON_NORMAL_PIVOT_THRESHOLD = 0.40

    # Potência durante pivot.
    PIVOT_OUTER_POWER = 0.75
    PIVOT_INNER_POWER = 0.0

    pivot_state = PIVOT_STATE_NONE
    if normal_steering_mapper:
        if pivot_state_tracker is not None:
            pivot_state = pivot_state_tracker.update(steering_error)
        elif steering_error is not None:
            if steering_error >= PIVOT_ENTER_THRESHOLD:
                pivot_state = PIVOT_STATE_RIGHT
            elif steering_error <= -PIVOT_ENTER_THRESHOLD:
                pivot_state = PIVOT_STATE_LEFT
    else:
        if pivot_state_tracker is not None:
            pivot_state_tracker.reset()
        if steering_error is not None:
            if steering_error >= NON_NORMAL_PIVOT_THRESHOLD:
                pivot_state = PIVOT_STATE_RIGHT
            elif steering_error <= -NON_NORMAL_PIVOT_THRESHOLD:
                pivot_state = PIVOT_STATE_LEFT

    if steering_error is None:
        left_power = 0.0
        right_power = 0.0

    elif pivot_state == PIVOT_STATE_RIGHT:
        # Curva forte para DIREITA.
        #
        # Esquerda para frente e direita parada.
        left_power = PIVOT_OUTER_POWER
        right_power = PIVOT_INNER_POWER

    elif pivot_state == PIVOT_STATE_LEFT:
        # Curva forte para ESQUERDA.
        #
        # Direita para frente e esquerda parada.
        left_power = PIVOT_INNER_POWER
        right_power = PIVOT_OUTER_POWER

    else:
        # Correção normal.
        steering_magnitude = abs(steering_error)
        if (
            normal_steering_mapper
            and steering_magnitude >= NORMAL_FULL_STEERING_ERROR
        ):
            # A progressão quadrática suaviza o início da faixa forte sem
            # impedir que o diferencial se aproxime do máximo antes do pivot.
            transition_progress = (
                steering_magnitude - NORMAL_FULL_STEERING_ERROR
            ) / (
                PIVOT_ENTER_THRESHOLD - NORMAL_FULL_STEERING_ERROR
            )
            transition_progress = max(
                0.0,
                min(1.0, transition_progress),
            )
            transition_progress *= transition_progress
            outer_power = (
                MAX_POWER
                + transition_progress
                * (STRONG_TURN_OUTER_POWER - MAX_POWER)
            )
            inner_power = (
                NORMAL_INNER_MIN_POWER
                - transition_progress
                * (
                    NORMAL_INNER_MIN_POWER
                    - STRONG_TURN_INNER_MIN_POWER
                )
            )
        else:
            full_steering_error = (
                NORMAL_FULL_STEERING_ERROR
                if normal_steering_mapper
                else NON_NORMAL_PIVOT_THRESHOLD
            )
            steering_strength = min(
                1.0,
                steering_magnitude / full_steering_error,
            )

            outer_power = (
                BASE_POWER
                + steering_strength
                * (MAX_POWER - BASE_POWER)
            )
            inner_power = (
                BASE_POWER
                - steering_strength
                * (BASE_POWER - NORMAL_INNER_MIN_POWER)
            )

        if steering_error > 0.0:
            # Curva para DIREITA.
            left_power = outer_power
            right_power = inner_power

        else:
            # Curva para ESQUERDA.
            left_power = inner_power
            right_power = outer_power

    # O scan não cria steering nem passa pelo mapper. Ele gira lentamente
    # usando somente a potência base e é reavaliado no próximo frame.
    if medium_scan_direction == "LEFT":
        left_power = 0.0
        right_power = BASE_POWER
    elif medium_scan_direction == "RIGHT":
        left_power = BASE_POWER
        right_power = 0.0
    elif direct_recovery_direction == "LEFT":
        left_power = 0.0
        right_power = BASE_POWER
    elif direct_recovery_direction == "RIGHT":
        left_power = BASE_POWER
        right_power = 0.0

    return {
        "left_power": left_power,
        "right_power": right_power,

        "farLeft": sensors["farLeft"],
        "farCenter": sensors["farCenter"],
        "farRight": sensors["farRight"],
        "farPosition": sensors["farPosition"],

        "farBandLeft": sensors["farBandLeft"],
        "farBandCenter": sensors["farBandCenter"],
        "farBandRight": sensors["farBandRight"],
        "farBandPosition": sensors["farBandPosition"],

        "mediumLeft": sensors["mediumLeft"],
        "mediumCenter": sensors["mediumCenter"],
        "mediumRight": sensors["mediumRight"],
        "mediumPosition": sensors["mediumPosition"],

        "nearLeft": sensors["nearLeft"],
        "nearCenter": sensors["nearCenter"],
        "nearRight": sensors["nearRight"],
        "nearPosition": sensors["nearPosition"],
        "rawNearPosition": sensors.get(
            "rawNearPosition",
            sensors["nearPosition"],
        ),

        "headingAngle": sensors["headingAngle"],
        "steeringError": steering_error,
        "finalSteering": steering_error,
        "virtualState": virtual_state,
        "lineState": line_state,
        "greenDirection": direcao_verde_ativa,
        "controlSource": control_source,
    }

def virtual_row_position_to_point(
    position,
    row_geometry,
):
    """
    Converte uma posição normalizada -1..+1
    em um ponto real (x, y) dentro da fileira.

    -1 = centro do sensor LEFT
     0 = centro do sensor CENTER
    +1 = centro do sensor RIGHT
    """

    if position is None:
        return None

    left_center_x = (
        row_geometry["left"]["x0"]
        + row_geometry["left"]["x1"]
    ) / 2.0

    right_center_x = (
        row_geometry["right"]["x0"]
        + row_geometry["right"]["x1"]
    ) / 2.0

    center_y = (
        row_geometry["center"]["y0"]
        + row_geometry["center"]["y1"]
    ) / 2.0

    normalized = (position + 1.0) / 2.0

    x = (
        left_center_x
        + normalized
        * (right_center_x - left_center_x)
    )

    return (
        float(x),
        float(center_y),
    )


def calculate_virtual_heading_angle(
    far_position,
    near_position,
    geometry,
):
    """
    Calcula a direção da faixa entre NEAR e FAR.

    0°  = reta
    >0° = aponta para a direita
    <0° = aponta para a esquerda

    Retorna None se FAR ou NEAR forem inválidos.
    """

    far_point = virtual_row_position_to_point(
        far_position,
        geometry["far"],
    )

    near_point = virtual_row_position_to_point(
        near_position,
        geometry["near"],
    )

    if far_point is None or near_point is None:
        return None

    delta_x = far_point[0] - near_point[0]
    delta_y = near_point[1] - far_point[1]

    if delta_y <= 0.0:
        return None

    angle_radians = math.atan2(
        delta_x,
        delta_y,
    )

    return float(
        math.degrees(angle_radians)
    )

def calculate_virtual_steering_error(
    near_position,
    heading_angle,
):
    """
    Combina posição lateral atual e antecipação da trajetória.

    Se FAR desaparecer, continua seguindo somente por NEAR.

    -1.0 = correção máxima para esquerda
     0.0 = seguir reto
    +1.0 = correção máxima para direita
    """

    if near_position is None:
        return None

    # FAR é antecipação, não requisito para continuar seguindo.
    if heading_angle is None:
        return float(
            max(-1.0, min(1.0, near_position))
        )

    heading_normalized = (
        heading_angle
        / VIRTUAL_HEADING_FULL_SCALE_DEG
    )

    heading_normalized = max(
        -1.0,
        min(1.0, heading_normalized),
    )

    steering_error = (
        near_position
        + VIRTUAL_HEADING_GAIN * heading_normalized
    )

    steering_error = max(
        -1.0,
        min(1.0, steering_error),
    )

    return float(steering_error)


def protect_virtual_near_direction(
    near_position,
    medium_position,
    heading_angle,
    steering_error,
):
    """Impede que o heading inverta sozinho um NEAR lateral confiável."""

    values = (
        finite_virtual_position(near_position),
        finite_virtual_position(heading_angle),
        finite_virtual_position(steering_error),
    )
    near_position, heading_angle, steering_error = values
    if (
        near_position is None
        or heading_angle is None
        or steering_error is None
        or abs(near_position)
        < VIRTUAL_NEAR_DIRECTION_PROTECTION_THRESHOLD
        or near_position * steering_error >= 0.0
    ):
        return steering_error

    medium_position = finite_virtual_position(medium_position)
    medium_confirms_heading = (
        medium_position is not None
        and abs(medium_position)
        >= VIRTUAL_NEAR_DIRECTION_PROTECTION_THRESHOLD
        and medium_position * heading_angle > 0.0
    )
    if medium_confirms_heading:
        return steering_error

    # Sem confirmação à frente, usa somente a posição realmente vista no NEAR.
    return max(-1.0, min(1.0, near_position))

class GreenObservationTracker:
    """Confirma observações novas e remove decisões após curta histerese."""

    def __init__(self):
        self.last_sequence = None
        self.pending_interpretation = "SEM_DECISAO"
        self.consecutive_samples = 0
        self.missing_samples = 0
        self.confirmed_interpretation = "SEM_DECISAO"
        self.last_direction_seen_at = None

    def update(self, line_sequence, interpretation, observed_at=None):
        """Confirma quadros novos e retém orientação lateral por 0,5 segundo."""

        observed_at = (
            time.perf_counter() if observed_at is None else float(observed_at)
        )
        if line_sequence == self.last_sequence:
            return (
                self.confirmed_interpretation,
                self.confirmed_interpretation != "SEM_DECISAO",
                self.consecutive_samples,
            )
        self.last_sequence = line_sequence

        if interpretation in ("ESQUERDA", "DIREITA"):
            # O instante é renovado em todo frame detectado, inclusive durante
            # a confirmação, para que a retenção conte da última visão real.
            self.last_direction_seen_at = observed_at

        if interpretation == "SEM_DECISAO":
            direction_is_retained = (
                self.confirmed_interpretation in ("ESQUERDA", "DIREITA")
                and self.last_direction_seen_at is not None
                and observed_at - self.last_direction_seen_at
                < GREEN_DIRECTION_RETENTION_SECONDS
            )
            if direction_is_retained:
                return (
                    self.confirmed_interpretation,
                    True,
                    self.consecutive_samples,
                )
            if self.confirmed_interpretation in ("ESQUERDA", "DIREITA"):
                self.pending_interpretation = "SEM_DECISAO"
                self.confirmed_interpretation = "SEM_DECISAO"
                self.consecutive_samples = 0
                self.missing_samples = 0
                return "SEM_DECISAO", False, 0
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
        # A telemetria representa o progresso da confirmação, não há motivo
        # para crescer sem limite depois de a decisão já estar aceita.
        self.consecutive_samples = min(
            self.consecutive_samples,
            required_samples,
        )
        confirmable = interpretation not in ("AMBIGUO", "SEM_DECISAO")
        if confirmable and self.consecutive_samples >= required_samples:
            self.confirmed_interpretation = interpretation

        published_interpretation = self.confirmed_interpretation
        if interpretation == "AMBIGUO":
            return "AMBIGUO", False, self.consecutive_samples
        return (
            published_interpretation,
            self.confirmed_interpretation != "SEM_DECISAO",
            self.consecutive_samples,
        )


def build_green_status(
    candidates,
    rejected_count,
    interpretation_result,
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
        "greenConsecutiveSamples": int(consecutive_samples),
        "greenProcessingMs": float(processing_ms),
    })

    # A ROI superior valida a associação com a faixa. Os campos laterais
    # preservam o IPC, mas agora descrevem as metades da única ROI horizontal.
    markers = interpretation_result.get("markers", [])
    upper_valid_markers = [
        marker for marker in markers
        if marker.get("upper", {}).get("valid", False)
    ]
    status["greenMarkerCount"] = len(markers)
    status["greenValidatedMarkerCount"] = len(upper_valid_markers)
    diagnostic_marker = (
        upper_valid_markers[0] if upper_valid_markers else
        (markers[0] if markers else None)
    )
    if diagnostic_marker is not None:
        upper = diagnostic_marker.get("upper", {})
        status["greenFrontRoiMeasured"] = bool(
            upper.get("measured", False)
        )
        status["greenFrontBlackRatio"] = float(
            upper.get("black_ratio", 0.0)
        )
        status["greenFrontRoiValid"] = bool(upper.get("valid", False))
        horizontal = diagnostic_marker.get("horizontal", {})
        for prefix, side in (("greenLeft", "left"), ("greenRight", "right")):
            status[f"{prefix}RoiMeasured"] = bool(
                horizontal.get(f"{side}_measured", False)
            )
            status[f"{prefix}BlackRatio"] = float(
                horizontal.get(f"{side}_black_ratio", 0.0)
            )
            status[f"{prefix}RoiValid"] = bool(
                horizontal.get(f"{side}_valid", False)
            )
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
    line_follower_command,
    line_timestamp,
    line_sequence,
    green_status,
    specular_repair_status=None,
):
    """Publica controle visual e telemetria leve no IPC rápido da linha."""

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

        normal_left = float(line_follower_command["left_power"])
        normal_right = float(line_follower_command["right_power"])
        if not all(
            math.isfinite(value) and -1.0 <= value <= 1.0
            for value in (normal_left, normal_right)
        ):
            raise ValueError("Comando visual fora da faixa normalizada")

        repair_status = specular_repair_status or {}
        line_status = {
            "lineFollowerLeftPower": normal_left,
            "lineFollowerRightPower": normal_right,
            "lineNearDetected": bool(
                finite_virtual_position(
                    line_follower_command.get(
                        "rawNearPosition",
                        line_follower_command.get("nearPosition"),
                    )
                )
                is not None
            ),
            "lineControlSource": str(
                line_follower_command.get("controlSource", "unknown")
            ),
            "nearPosition": finite_virtual_position(
                line_follower_command.get("nearPosition")
            ),
            "mediumPosition": finite_virtual_position(
                line_follower_command.get("mediumPosition")
            ),
            "farBandPosition": finite_virtual_position(
                line_follower_command.get("farBandPosition")
            ),
            "headingAngleDeg": finite_virtual_position(
                line_follower_command.get("headingAngle")
            ),
            "finalSteering": finite_virtual_position(
                line_follower_command.get("finalSteering")
            ),
            "vstate": str(
                line_follower_command.get("virtualState", "INVALID")
            ),
            "lineState": str(
                line_follower_command.get("lineState", "INVALID")
            ),
            "lineTimestamp": line_timestamp,
            "lineSequence": line_sequence,
            "specularRepairPixels": max(
                0, int(repair_status.get("specularRepairPixels", 0))
            ),
            "specularRepairComponents": max(
                0, int(repair_status.get("specularRepairComponents", 0))
            ),
        }
        line_status.update(green_status)
        with open(TEMP_LINE_STATUS_PATH, "w", encoding="utf-8") as status_file:
            json.dump(line_status, status_file, allow_nan=False)
        os.replace(TEMP_LINE_STATUS_PATH, LINE_STATUS_PATH)
    except (KeyError, OSError, TypeError, ValueError) as error:
        print(
            f"Falha ao publicar telemetria rápida da linha: {error}",
            flush=True,
        )


def save_status(
    fps,
    camera_profile,
    camera_details,
    camera_format="",
    active=True,
    error_message="",
    line_timestamp=0.0,
    line_sequence=0,
    specular_repair_status=None,
    green_status=None,
    line_timings=None,
):
    """Publica somente a saúde da câmera e os resultados visuais preservados."""

    try:
        line_timestamp = float(line_timestamp)
        line_timestamp_valid = (
            math.isfinite(line_timestamp) and line_timestamp >= 0.0
        )
    except (TypeError, ValueError):
        line_timestamp_valid = False
        line_timestamp = 0.0

    line_sequence_valid = (
        isinstance(line_sequence, int)
        and not isinstance(line_sequence, bool)
        and line_sequence >= 0
    )
    if not line_sequence_valid:
        line_sequence = 0

    frame_width, frame_height = camera_profile["main_size"]
    line_ipc_fresh = (
        camera_profile["role"] != "down"
        or (
            line_timestamp_valid
            and line_timestamp > 0.0
            and time.time() - line_timestamp <= 0.5
        )
    )
    repair_status = specular_repair_status or {}
    repaired_pixels = max(
        0, int(repair_status.get("specularRepairPixels", 0))
    )
    repaired_components = max(
        0, int(repair_status.get("specularRepairComponents", 0))
    )
    timing_status = line_timings if isinstance(line_timings, dict) else {}
    safe_line_timings = {}
    for timing_name in (
        "lineProcessingMs",
        "binaryMs",
        "backgroundKernelMs",
        "backgroundCloseMs",
        "binaryCompareMs",
        "specularMs",
        "morphMs",
        "morphOpenMs",
        "morphCloseMs",
        "contoursMs",
    ):
        try:
            timing_value = float(timing_status.get(timing_name, 0.0))
        except (TypeError, ValueError):
            timing_value = 0.0
        if not math.isfinite(timing_value) or timing_value < 0.0:
            timing_value = 0.0
        safe_line_timings[timing_name] = timing_value
    status = {
        "fps": round(fps, 2),
        "active": active,
        # O processo inferior só existe enquanto seu gerenciador está ativo.
        "enabled": camera_profile["role"] == "down",
        "state": "ONLINE" if active and line_ipc_fresh else (
            "FALHA" if error_message else "INICIANDO"
        ),
        "lineIpcFresh": line_ipc_fresh,
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
        "lineFollowerImplemented": False,
        "lineTimestamp": line_timestamp,
        "lineSequence": line_sequence,
        "specularRepairPixels": repaired_pixels,
        "specularRepairComponents": repaired_components,
        "lineProcessingMs": safe_line_timings["lineProcessingMs"],
        "binaryMs": safe_line_timings["binaryMs"],
        "backgroundKernelMs": safe_line_timings["backgroundKernelMs"],
        "backgroundCloseMs": safe_line_timings["backgroundCloseMs"],
        "binaryCompareMs": safe_line_timings["binaryCompareMs"],
        "specularMs": safe_line_timings["specularMs"],
        "morphMs": safe_line_timings["morphMs"],
        "morphOpenMs": safe_line_timings["morphOpenMs"],
        "morphCloseMs": safe_line_timings["morphCloseMs"],
        "contoursMs": safe_line_timings["contoursMs"],
    }
    status.update(green_status or empty_green_status())
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
        virtual_turn_tracker = VirtualTurnStateTracker()
        pivot_state_tracker = VirtualPivotStateTracker()
        line_search_tracker = VirtualLineSearchTracker()

        # Estado persistente das manobras sinalizadas por verde.
        direcao_verde_ativa = "NENHUMA"
        curva_verde_iniciada = False
        quadros_centralizado_verde = 0
        quadros_verde_ativo = 0

         # Estado persistente da travessia de gap.
        gap_forward_active = False
        gap_forward_frames = 0
        gap_reacquire_frames = 0
        gap_line_lost_seen = False
        gap_recent_near_frames = 0

        # Impede que o mesmo marcador verde seja aceito novamente.
        verde_armado = True
        quadros_sem_verde = 0

        green_processing_enabled = bool(
            vision_profile.get("green_detection_enabled", False)
            and GREEN_PROCESSING_ENABLED
        )
        print(
            "Visão verde para overlay e telemetria: "
            f"{'ligada' if green_processing_enabled else 'desligada'}.",
            flush=True,
        )

        while running:
            green_capture_requested = os.path.isfile(GREEN_CAPTURE_REQUEST_PATH)
            green_capture_metadata = {}
            if green_capture_requested:
                # A captura preserva metadados do mesmo frame usado no diagnóstico.
                camera_request = picam2.capture_request()
                try:
                    raw_frame = camera_request.make_array("main")
                    green_capture_metadata = camera_request.get_metadata()
                finally:
                    camera_request.release()
            else:
                raw_frame = picam2.capture_array()

            frame_height = raw_frame.shape[0]
            vision_geometry = resolve_vision_geometry(
                frame_height,
                vision_profile,
            )
            line_vision_started = time.perf_counter()
            line_timings = {}

            filtered_mask, roi_start_y, specular_repair_status = (
                create_filtered_line_mask(
                    raw_frame,
                    vision_profile,
                    camera_format,
                    return_repair_status=True,
                    timings=line_timings,
                )
            )
            structural_mask = create_structural_line_mask(
                filtered_mask,
                roi_start_y,
                vision_geometry["structural_end_y"],
            )
            contours_started = time.perf_counter()
            line_candidate_mask = create_line_candidate_mask(
                structural_mask,
                vision_profile,
            )
            line_timings["contoursMs"] = (
                time.perf_counter() - contours_started
            ) * 1000.0
            line_vision_ms = (
                time.perf_counter() - line_vision_started
            ) * 1000.0

            green_candidates = []
            green_rejected = []
            green_mask = np.zeros(
                (vision_geometry["structural_end_y"], raw_frame.shape[1]),
                dtype=np.uint8,
            )
            green_interpretation = analyze_green_marker_contours(
                [],
                structural_mask,
            )
            green_processing_started = time.perf_counter()
            if green_processing_enabled:
                green_mask, green_candidates, green_rejected = (
                    find_green_candidates(
                        raw_frame,
                        vision_geometry["structural_end_y"],
                        camera_format,
                    )
                )
                # A classificação usa o preto estrutural local, não uma
                # referência de direção ou posição destinada ao controle.
                green_association_mask = structural_mask.copy()
                useful_height = min(
                    green_association_mask.shape[0],
                    green_mask.shape[0],
                )
                useful_width = min(
                    green_association_mask.shape[1],
                    green_mask.shape[1],
                )
                association_region = green_association_mask[
                    :useful_height,
                    :useful_width,
                ]
                association_region[
                    green_mask[:useful_height, :useful_width] > 0
                ] = 0
                green_interpretation = analyze_green_marker_contours(
                    [candidate["contour"] for candidate in green_candidates],
                    green_association_mask,
                )
            green_processing_ms = (
                time.perf_counter() - green_processing_started
            ) * 1000.0

            line_timestamp = time.time()
            line_sequence += 1
            green_raw_interpretation = green_interpretation["interpretation"]
            green_tracker_result = green_tracker.update(
                line_sequence,
                green_raw_interpretation,
                time.perf_counter(),
            )
            green_status = build_green_status(
                green_candidates,
                len(green_rejected),
                green_interpretation,
                green_tracker_result,
                green_processing_ms,
            )
            green_status["greenRawInterpretation"] = green_raw_interpretation
            green_status["greenPathBlackValid"] = bool(
                green_interpretation["path_black_valid"]
            )
            green_status["greenDecisionState"] = (
                "accepted"
                if green_status["greenConfirmed"]
                else "candidate"
                if green_raw_interpretation != "SEM_DECISAO"
                else "idle"
            )

                        # Um verde confirmado é aceito apenas quando o sistema está armado
            # e nenhuma outra direção verde está sendo executada.
            if (
                verde_armado
                and direcao_verde_ativa == "NENHUMA"
                and green_status["greenConfirmed"]
            ):
                interpretacao_verde = green_status["greenInterpretation"]

                if interpretacao_verde == "ESQUERDA":
                    direcao_verde_ativa = "ESQUERDA"
                    curva_verde_iniciada = False
                    quadros_centralizado_verde = 0
                    quadros_verde_ativo = 0
                    line_search_tracker.stop()
                    verde_armado = False
                    quadros_sem_verde = 0

                elif interpretacao_verde == "DIREITA":
                    direcao_verde_ativa = "DIREITA"
                    curva_verde_iniciada = False
                    quadros_centralizado_verde = 0
                    quadros_verde_ativo = 0
                    line_search_tracker.stop()
                    verde_armado = False
                    quadros_sem_verde = 0

            # Depois que um verde foi aceito, o sistema só poderá ser armado
            # novamente após vários quadros consecutivos sem nenhum candidato verde. % isaque hulk verde
            if not verde_armado:
                if green_status["greenCandidateCount"] == 0:
                    quadros_sem_verde += 1
                else:
                    quadros_sem_verde = 0

            geometric_started = time.perf_counter()

            geometric_guidance = (
                extract_geometric_line_path(
                    line_candidate_mask
                )
            )

            geometric_guidance[
                "processingMs"
            ] = (
                time.perf_counter()
                - geometric_started
            ) * 1000.0

            geometric_heading = geometric_guidance.get(
                "farHeadingDeg"
            )

            lateral_exit_target = geometric_guidance.get(
                "lateralExitTarget"
            )

            real_near_point = geometric_guidance.get(
                "nearPoint"
            )

            virtual_near_point = geometric_guidance.get(
                "virtualNearPoint"
            )

            trace_folded_back = (
                geometric_heading is not None
                and abs(float(geometric_heading)) > 90.0
            )

            virtual_sensors = read_virtual_line_sensors(
                line_candidate_mask,
                direcao_verde_ativa,
            )
            raw_near_visible = (
                finite_virtual_position(
                    virtual_sensors.get("rawNearPosition")
                )
                is not None
            )
            gap_recent_near_frames = update_gap_recent_near_frames(
                gap_recent_near_frames,
                raw_near_visible,
            )
            raw_line_visible = virtual_raw_line_is_visible(
                virtual_sensors
            )
            green_timeout_state = update_green_maneuver_state(
                direcao_verde_ativa,
                quadros_verde_ativo,
                raw_line_visible,
            )
            sensor_recovery_requested = False
            if green_timeout_state["timedOut"]:
                direcao_verde_ativa = green_timeout_state["direction"]
                quadros_verde_ativo = green_timeout_state["activeFrames"]
                curva_verde_iniciada = False
                quadros_centralizado_verde = 0
                search_direction = green_timeout_state["searchDirection"]
                if search_direction is not None:
                    line_search_tracker.start(search_direction)
                else:
                    sensor_recovery_requested = True
                # Remove a máscara verde já no mesmo frame do timeout.
                virtual_sensors = read_virtual_line_sensors(
                    line_candidate_mask,
                    direcao_verde_ativa,
                )
            else:
                direcao_verde_ativa = green_timeout_state["direction"]
                quadros_verde_ativo = green_timeout_state["activeFrames"]

            if gap_entry_is_required(
                gap_forward_active,
                direcao_verde_ativa,
                gap_recent_near_frames,
                raw_near_visible,
                real_near_point,
                virtual_near_point,
                lateral_exit_target,
            ):
                gap_forward_active = True
                gap_forward_frames = 0
                gap_reacquire_frames = 0
                gap_line_lost_seen = True
                

            gap_blind_search_requested = False
            if gap_forward_active:
                raw_near_reacquired = (
                    finite_virtual_position(
                        virtual_sensors.get("rawNearPosition")
                    )
                    is not None
                )
                near_reacquired = (
                    raw_near_reacquired
                    or (
                        real_near_point is not None
                        and not trace_folded_back
                    )
                )
                gap_state = update_gap_forward_recovery(
                    gap_forward_active,
                    gap_forward_frames,
                    gap_reacquire_frames,
                    gap_line_lost_seen,
                    near_reacquired,
                )
                gap_forward_active = gap_state["active"]
                gap_forward_frames = gap_state["forwardFrames"]
                gap_reacquire_frames = gap_state["reacquireFrames"]
                gap_line_lost_seen = gap_state["lineLostSeen"]
                gap_blind_search_requested = gap_state[
                    "blindSearchRequested"
                ]
                if not gap_forward_active:
                    line_search_tracker.stop()

            line_control_started = time.perf_counter()

            line_follower_command = (
                calculate_line_follower_command(
                    line_candidate_mask,
                    green_status,
                    direcao_verde_ativa,
                    gap_forward_active,
                    virtual_turn_tracker=virtual_turn_tracker,
                    pivot_state_tracker=pivot_state_tracker,
                    virtual_sensors=virtual_sensors,
                    line_search_tracker=line_search_tracker,
                    blind_search_requested=gap_blind_search_requested,
                    sensor_recovery_requested=sensor_recovery_requested,
                )
            )

            near_position = line_follower_command["nearPosition"]

            # Confirma que o robô realmente começou a entrar no ramo
            # indicado pelo marcador verde.
            if not curva_verde_iniciada:
                if (
                    direcao_verde_ativa == "ESQUERDA"
                    and near_position is not None
                    and near_position <= -LIMIAR_CURVA_VERDE_INICIADA
                ):
                    curva_verde_iniciada = True
                    quadros_centralizado_verde = 0

                elif (
                    direcao_verde_ativa == "DIREITA"
                    and near_position is not None
                    and near_position >= LIMIAR_CURVA_VERDE_INICIADA
                ):
                    curva_verde_iniciada = True
                    quadros_centralizado_verde = 0

            # Depois que a curva começou, espera o NEAR voltar ao centro
            # por vários quadros consecutivos. Isso indica que o robô
            # já entrou e se alinhou com a nova faixa.
            if (
                direcao_verde_ativa != "NENHUMA"
                and curva_verde_iniciada
            ):
                if (
                    near_position is not None
                    and abs(near_position) <= LIMIAR_CENTRALIZACAO_VERDE
                ):
                    quadros_centralizado_verde += 1
                else:
                    quadros_centralizado_verde = 0

                if (
                    quadros_centralizado_verde
                    >= QUADROS_CENTRALIZADO_PARA_CONCLUIR
                ):
                    completed_green_state = update_green_maneuver_state(
                        direcao_verde_ativa,
                        quadros_verde_ativo,
                        raw_line_visible,
                        completed=True,
                    )
                    direcao_verde_ativa = completed_green_state["direction"]
                    quadros_verde_ativo = completed_green_state[
                        "activeFrames"
                    ]
                    curva_verde_iniciada = False
                    quadros_centralizado_verde = 0
                    line_search_tracker.stop()

            # A curva já pode ter terminado, mas um novo verde só será
            # aceito depois de X quadros consecutivos sem candidato verde.
            if (
                not verde_armado
                and direcao_verde_ativa == "NENHUMA"
                and quadros_sem_verde >= QUADROS_PARA_REARMAR_VERDE
            ):
                verde_armado = True
                quadros_sem_verde = 0

            line_control_ms = (
                time.perf_counter() - line_control_started
            ) * 1000.0

            line_follower_command["lineProcessingMs"] = (
                line_vision_ms
                + geometric_guidance["processingMs"]
                + line_control_ms
            )

            line_timings["lineProcessingMs"] = (
                line_follower_command["lineProcessingMs"]
            )

            if line_ipc_enabled:
                # Somente a CAM0/inferior publica o ponto de extensão 0/0.
                save_line_status(
                    line_follower_command,
                    line_timestamp,
                    line_sequence,
                    green_status,
                    specular_repair_status=specular_repair_status,
                )

            if green_capture_requested:
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

            display_mode = get_display_mode()
            frame = create_display_frame(
                raw_frame,
                line_candidate_mask,
                green_mask,
                roi_start_y,
                display_mode,
                structural_mask,
            )

            if camera_profile["role"] == "down":
                draw_line_control_overlay(frame, line_follower_command)
                draw_virtual_sensor_geometry(
                    frame,
                    line_follower_command,
                )

            # A câmera inferior mantém o controle e os nove sensores virtuais.
            # As linhas estruturais antigas permanecem nas outras câmeras.
            if camera_profile["role"] != "down":
                cv2.line(
                    frame,
                    (0, roi_start_y),
                    (frame.shape[1] - 1, roi_start_y),
                    (0, 255, 255),
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

            # Os contornos, símbolos e ROIs verdes continuam visíveis também
            # na câmera inferior sem alterar a classificação ou o controle.
            if green_processing_enabled:
                interpretation = green_status["greenInterpretation"]
                green_overlay_accepted = bool(
                    green_status["greenConfirmed"]
                    and interpretation in VISIBLE_GREEN_INTERPRETATIONS
                    and green_raw_interpretation == interpretation
                    and green_status["greenPathBlackValid"]
                )
                overlay_interpretation = (
                    interpretation
                    if green_overlay_accepted
                    else green_raw_interpretation
                )
                if display_mode == DISPLAY_MODE_LINE:
                    draw_line_mode_green_overlays(
                        frame,
                        green_candidates,
                        overlay_interpretation,
                        green_overlay_accepted,
                    )
                else:
                    draw_green_candidate_overlays(
                        frame,
                        green_candidates,
                        overlay_interpretation,
                        green_overlay_accepted,
                    )
                draw_green_roi_overlays(
                    frame,
                    green_interpretation,
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
                    line_timestamp=line_timestamp,
                    line_sequence=line_sequence,
                    specular_repair_status=specular_repair_status,
                    green_status=green_status,
                    line_timings=line_timings,
                )
                last_status_time = now
    except Exception as error:
        import traceback
        traceback.print_exc()
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
