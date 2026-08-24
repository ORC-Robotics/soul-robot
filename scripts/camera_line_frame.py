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
            "line_max_background_ratio_percent": 68,
            # Mesmo com contraste local, tons acima deste limite não são pretos.
            # A unidade é o nível de cinza de 8 bits, entre 0 e 255. Aumentar o
            # limite aceita sombras; reduzir demais pode perder uma fita clara.
            "line_max_brightness": 110,
            # Valores de referência em 640×480; são sempre escalados para
            # kernels ímpares antes da morfologia (17 vira 13 e 7 vira 5).
            "open_kernel_shape": "ellipse",
            "open_kernel_size": 17,
            # O fechamento 7×7 preenche pequenas falhas sem unir objetos
            # separados à linha de 2 cm.
            "close_kernel_size": 7,
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
    maximum_area = (
        structural_mask.size
        * vision_profile.get("full_line_max_area_ratio", 1.0)
    )
    for contour in full_contours:
        contour_area = cv2.contourArea(contour)
        if contour_area > maximum_area:
            continue

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

# Orientação geométrica experimental da faixa.
#
# Este método ainda não controla os motores. Ele apenas extrai
# uma trajetória central da máscara para comparação visual com
# o seguidor atual por sensores virtuais.
GEOMETRIC_PATH_SAMPLE_COUNT = 21
GEOMETRIC_PATH_BAND_HALF_HEIGHT = 2
GEOMETRIC_PATH_LOCAL_HEADING_POINTS = 5

# FAR e NEAR permanecem em alturas fixas da imagem.
GEOMETRIC_PATH_FAR_Y_RATIO = 0.00
GEOMETRIC_PATH_NEAR_Y_RATIO = VIRTUAL_NEAR_Y1

def resolve_virtual_sensor_geometry(frame_shape):
    """
    Converte a geometria normalizada dos seis sensores
    para coordenadas reais em pixels.
    """

    height, width = frame_shape[:2]

    far_y0 = int(round(height * VIRTUAL_FAR_Y0))
    far_y1 = int(round(height * VIRTUAL_FAR_Y1))

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
    Desenha os seis sensores, suas leituras analógicas
    e as posições FAR/NEAR.
    """

    geometry = resolve_virtual_sensor_geometry(
        frame.shape
    )

    sensors = (
        (
            "FAR-L",
            geometry["far"]["left"],
            line_follower_command["farLeft"],
        ),
        (
            "FAR-C",
            geometry["far"]["center"],
            line_follower_command["farCenter"],
        ),
        (
            "FAR-R",
            geometry["far"]["right"],
            line_follower_command["farRight"],
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
    near_position = line_follower_command["nearPosition"]

    far_text = (
        f"FAR POS {far_position:+.2f}"
        if far_position is not None
        else "FAR POS INVALID"
    )

    near_text = (
        f"NEAR POS {near_position:+.2f}"
        if near_position is not None
        else "NEAR POS INVALID"
    )

    cv2.putText(
        frame,
        far_text,
        (
            geometry["far"]["center"]["x0"] + 8,
            geometry["far"]["center"]["y0"] + 48,
        ),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 255, 255),
        1,
        cv2.LINE_AA,
    )

    cv2.putText(
        frame,
        near_text,
        (
            geometry["near"]["center"]["x0"] + 8,
            geometry["near"]["center"]["y0"] + 48,
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


def calculate_geometric_far_heading(path_points):
    """
    Calcula a direção local da faixa perto do FAR.

    0 graus representa uma faixa vertical na imagem.
    Valor positivo aponta para a direita.
    Valor negativo aponta para a esquerda.
    """

    point_count = min(
        len(path_points),
        GEOMETRIC_PATH_LOCAL_HEADING_POINTS,
    )

    if point_count < 2:
        return None

    far_points = np.asarray(
        path_points[-point_count:],
        dtype=np.float32,
    ).reshape(-1, 1, 2)

    vx, vy, _, _ = cv2.fitLine(
        far_points,
        cv2.DIST_L2,
        0,
        0.01,
        0.01,
    ).flatten()

    vx = float(vx)
    vy = float(vy)

    # O vetor deve apontar do NEAR em direção ao FAR.
    if vy > 0.0:
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
    Percorre a faixa do NEAR para o FAR usando centros reais
    da máscara em várias alturas fixas.

    Quando aparecem múltiplos segmentos em uma mesma banda,
    escolhe o mais próximo da trajetória encontrada anteriormente.
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

    sample_y_values = np.linspace(
        near_y,
        far_y,
        GEOMETRIC_PATH_SAMPLE_COUNT,
    )

    path_points = []
    previous_x = float(width) / 2.0

    for sample_y in sample_y_values:
        y = int(round(sample_y))

        segments = find_active_band_segments(
            processed_line_mask,
            y,
        )

        if not segments:
            break

        selected_segment = min(
            segments,
            key=lambda segment: abs(
                segment["centerX"] - previous_x
            ),
        )

        center_x = selected_segment["centerX"]

        path_points.append(
            (
                float(center_x),
                float(y),
            )
        )

        previous_x = center_x

    near_point = (
        path_points[0]
        if path_points
        else None
    )

    far_point = (
        path_points[-1]
        if (
            path_points
            and int(round(path_points[-1][1])) == far_y
        )
        else None
    )

    far_heading = calculate_geometric_far_heading(
        path_points
    )

    return {
        "points": path_points,
        "nearPoint": near_point,
        "farPoint": far_point,
        "farHeadingDeg": far_heading,
    }

def draw_geometric_line_overlay(
    frame,
    processed_line_mask,
    geometric_guidance,
):
    """
    Desenha a orientação geométrica experimental sem alterar
    nenhuma decisão de controle do robô.
    """

    contours, _ = cv2.findContours(
        processed_line_mask.copy(),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    # Contorno azul da faixa, semelhante ao diagnóstico visual
    # usado como referência.
    if contours:
        cv2.drawContours(
            frame,
            contours,
            -1,
            (255, 0, 0),
            1,
            cv2.LINE_AA,
        )

    path_points = geometric_guidance["points"]

    if len(path_points) >= 2:
        polyline = np.asarray(
            [
                (
                    int(round(x)),
                    int(round(y)),
                )
                for x, y in path_points
            ],
            dtype=np.int32,
        ).reshape(-1, 1, 2)

        cv2.polylines(
            frame,
            [polyline],
            False,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

    near_point = geometric_guidance["nearPoint"]

    if near_point is not None:
        cv2.circle(
            frame,
            (
                int(round(near_point[0])),
                int(round(near_point[1])),
            ),
            5,
            (255, 0, 0),
            -1,
            cv2.LINE_AA,
        )

    far_point = geometric_guidance["farPoint"]

    if far_point is not None:
        cv2.circle(
            frame,
            (
                int(round(far_point[0])),
                int(round(far_point[1])),
            ),
            5,
            (0, 0, 255),
            -1,
            cv2.LINE_AA,
        )

    far_heading = geometric_guidance["farHeadingDeg"]

    heading_text = (
        f"{far_heading:+.0f}deg"
        if far_heading is not None
        else "--deg"
    )

    processing_ms = geometric_guidance.get(
        "processingMs",
        0.0,
    )

    cv2.putText(
        frame,
        "LINE",
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        (0, 0, 255),
        1,
        cv2.LINE_AA,
    )

    cv2.putText(
        frame,
        heading_text,
        (8, 44),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        (0, 0, 255),
        1,
        cv2.LINE_AA,
    )

    cv2.putText(
        frame,
        f"{processing_ms:.1f}ms",
        (8, 66),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        (0, 0, 255),
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
    Lê os seis sensores virtuais e calcula
    a posição lateral de FAR e NEAR.
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

    # Durante uma interseção sinalizada por verde, somente o ramo
    # permitido deve influenciar a antecipação do FAR.
    #
    # O NEAR mantém o centro ativo para representar a posição física
    # atual do robô enquanto ele abandona a linha antiga e entra na nova.
    if direcao_verde_ativa == "ESQUERDA":
        far_center = 0.0
        far_right = 0.0
        near_right = 0.0

    elif direcao_verde_ativa == "DIREITA":
        far_left = 0.0
        far_center = 0.0
        near_left = 0.0

    far_position = calculate_virtual_row_position(
        far_left,
        far_center,
        far_right,
    )

    near_position = calculate_virtual_row_position(
        near_left,
        near_center,
        near_right,
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

    return {
        "farLeft": far_left,
        "farCenter": far_center,
        "farRight": far_right,
        "farPosition": far_position,

        "nearLeft": near_left,
        "nearCenter": near_center,
        "nearRight": near_right,
        "nearPosition": near_position,
        "headingAngle": heading_angle,
        "steeringError": steering_error,
    }


def calculate_line_follower_command(
    processed_line_mask,
    green_detection_result,
    direcao_verde_ativa="NENHUMA",
):
    """
    Primeiro teste físico do novo seguidor.

    Usa somente steeringError para gerar
    uma correção diferencial simples.

    Ainda não há PID.
    """

    _ = green_detection_result

    sensors = read_virtual_line_sensors(
        processed_line_mask,
        direcao_verde_ativa,
    )

    steering_error = sensors["steeringError"]

        # --------------------------------------------------------
    # CONTROLE DE MOTORES
    # --------------------------------------------------------

    BASE_POWER = 0.68
    MAX_POWER = 0.78

    # A partir daqui a curva é forte o suficiente
    # para exigir pivot.
    PIVOT_THRESHOLD = 0.36

    # Potência durante pivot.
    PIVOT_OUTER_POWER = 0.75 #roda de giro
    PIVOT_INNER_POWER = 0.0 #roda de dentro desligada..

    if steering_error is None:
        left_power = 0.0
        right_power = 0.0

    elif steering_error >= PIVOT_THRESHOLD:
        # Curva forte para DIREITA.
        #
        # Esquerda para frente
        # Direita para trás
        left_power = PIVOT_OUTER_POWER
        right_power = -PIVOT_INNER_POWER

    elif steering_error <= -PIVOT_THRESHOLD:
        # Curva forte para ESQUERDA.
        #
        # Direita para frente
        # Esquerda para trás
        left_power = -PIVOT_INNER_POWER
        right_power = PIVOT_OUTER_POWER

    else:
        # Correção normal.
        #
        # Escala steering até o limite antes do pivot.
        normalized_steering = (
            steering_error / PIVOT_THRESHOLD
        )

        correction = (
            normalized_steering
            * (MAX_POWER - BASE_POWER)
        )

        if correction > 0.0:
            # Direita
            left_power = BASE_POWER + correction
            right_power = BASE_POWER

        else:
            # Esquerda
            left_power = BASE_POWER
            right_power = BASE_POWER - correction

        left_power = min(MAX_POWER, left_power)
        right_power = min(MAX_POWER, right_power)

    return {
        "left_power": left_power,
        "right_power": right_power,

        "farLeft": sensors["farLeft"],
        "farCenter": sensors["farCenter"],
        "farRight": sensors["farRight"],
        "farPosition": sensors["farPosition"],

        "nearLeft": sensors["nearLeft"],
        "nearCenter": sensors["nearCenter"],
        "nearRight": sensors["nearRight"],
        "nearPosition": sensors["nearPosition"],

        "headingAngle": sensors["headingAngle"],
        "steeringError": sensors["steeringError"],
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
    """Publica somente a interface normal e os dados exigidos pelo verde."""

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

        # Estado persistente das manobras sinalizadas por verde.
        direcao_verde_ativa = "NENHUMA"
        curva_verde_iniciada = False
        quadros_centralizado_verde = 0

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
                    verde_armado = False
                    quadros_sem_verde = 0

                elif interpretacao_verde == "DIREITA":
                    direcao_verde_ativa = "DIREITA"
                    curva_verde_iniciada = False
                    quadros_centralizado_verde = 0
                    verde_armado = False
                    quadros_sem_verde = 0

            # Depois que um verde foi aceito, o sistema só poderá ser armado
            # novamente após vários quadros consecutivos sem nenhum candidato verde. % isaque hulk verde
            if not verde_armado:
                if green_status["greenCandidateCount"] == 0:
                    quadros_sem_verde += 1
                else:
                    quadros_sem_verde = 0


            line_control_started = time.perf_counter()

            line_follower_command = calculate_line_follower_command(
                line_candidate_mask,
                green_status,
                direcao_verde_ativa,
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
                    direcao_verde_ativa = "NENHUMA"
                    curva_verde_iniciada = False
                    quadros_centralizado_verde = 0

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
                line_vision_ms + line_control_ms
            )
            line_timings["lineProcessingMs"] = line_follower_command[
                "lineProcessingMs"
            ]
            
            
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
            geometric_started = time.perf_counter()

            geometric_guidance = extract_geometric_line_path(
                line_candidate_mask
            )

            geometric_guidance["processingMs"] = (
                time.perf_counter() - geometric_started
            ) * 1000.0

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
                  draw_geometric_line_overlay(frame, line_follower_command, geometric_guidance)

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
