import json
import os
import signal
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

from vision_path import (
    CONTROL_ZONE_MIN_PROXIMITY,
    analyze_primary_path,
    path_x_at_y,
)

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
# Resolução enviada para o dashboard.
# Subir este valor melhora a nitidez, mas aumenta o custo de processamento e rede.
FRAME_WIDTH = 960
FRAME_HEIGHT = 540
SETPOINT_X = FRAME_WIDTH // 2
# FPS alvo da câmera.
# Em 30 FPS a Camera V2 tem mais tempo de exposição e gera menos ruído.
TARGET_CAMERA_FPS = 30
MJPEG_STREAM_PORT = 8090
MJPEG_STREAM_PATH = "/stream.mjpg"
MJPEG_STREAM_FPS = 30
SNAPSHOT_FRAME_FPS = 2
# O controle recebe 20 status por segundo. Assim, três confirmações distintas
# de um corner levam cerca de 150 ms sem aumentar a resolução processada.
STATUS_FPS = 20
# Qualidade do JPEG do stream MJPEG.
# Valores maiores deixam a imagem mais limpa, mas usam mais banda na rede.
JPEG_QUALITY = 82
# Limites do threshold automático aplicado ao canal V do HSV. O Otsu acompanha
# mudanças de iluminação; estes limites impedem aceitar sombra demais ou voltar
# ao valor fixo baixo que escondia uma faixa preta claramente visível.
BLACK_THRESHOLD_MIN = 70
BLACK_THRESHOLD_MAX = 155
# O processamento da linha usa uma cópia menor para preservar FPS.
# A imagem do dashboard continua saindo na resolução principal.
PROCESS_WIDTH = 320
PROCESS_HEIGHT = 180
# Fração superior ignorada no processamento da pista.
# Valor menor faz a câmera olhar mais à frente, útil para curvas e marcações verdes.
ROI_TOP_RATIO = 0.20
ROI_TOP_Y = int(FRAME_HEIGHT * ROI_TOP_RATIO)
PROCESS_ROI_TOP_Y = int(PROCESS_HEIGHT * ROI_TOP_RATIO)
# Área mínima da linha em pixels na imagem reduzida de processamento.
# Isso evita aceitar ruído pequeno como se fosse a faixa preta.
MIN_LINE_AREA = 80
# Faixa HSV usada para detectar marcações verdes na pista.
# Esses valores foram calibrados para o piso de teste e podem variar com a iluminação.
GREEN_LOWER = np.array([35, 50, 40])
GREEN_UPPER = np.array([90, 255, 255])
# Área mínima, em pixels, para aceitar uma marcação verde.
# Limites baixos demais podem transformar ruído colorido em manobra do robô.
MIN_GREEN_AREA = 50

# Depois de dois quadros sem CurrentPath, a previsão antiga é descartada e a
# visão procura novamente perto do centro, inclusive um pouco mais à frente.
# Isso evita que um erro transitório deixe o rastreador preso na raiz anterior.
ROOT_RECOVERY_AFTER_INVALID_FRAMES = 2

# Proximidades normalizadas desenhadas no debug para aproximação e ação.
# Elas devem acompanhar os limites C++ de mesmo significado. São parâmetros
# experimentais em imagem, não representam milímetros nem distância física.
# Aumentar ACTION move a linha para perto da base; diminuir antecipa o gatilho.
EVENT_APPROACH_PROXIMITY = 0.52
EVENT_ACTION_PROXIMITY = 0.72

# Confiança mínima publicada para um marcador verde relacionado à linha preta.
# Aumentar reduz falsos positivos, mas exige uma marca mais limpa e bem iluminada.
MIN_GREEN_CONFIDENCE = 0.45

# Ajustes visuais para a Raspberry Pi Camera V2.
# Eles favorecem nitidez no dashboard sem mudar a lógica de segurança do robô.
CAMERA_SHARPNESS = 1.2
CAMERA_CONTRAST = 1.05
CAMERA_SATURATION = 1.0
CAMERA_EXPOSURE_VALUE = 0.4
CAMERA_PIXEL_FORMATS = ("RGB888",)
# A câmera está instalada fisicamente de cabeça para baixo. A transformação
# ocorre no pipeline da câmera para que o dashboard e a visão processem o mesmo
# quadro já corrigido, sem gastar CPU girando cada imagem com OpenCV.
CAMERA_ROTATION_DEGREES = 180

running = True
latest_jpeg = None
latest_jpeg_sequence = 0
frame_condition = threading.Condition()


@dataclass
class GreenObservation:
    action: str = "NENHUM"
    proximity: float = 0.0
    confidence: float = 0.0


@dataclass
class VisionStatus:
    line_detected: bool = False
    current_path_valid: bool = False
    line_error: float = 0.0
    position_error_pixels: float = 0.0
    heading_error_degrees: float = 0.0
    path_confidence: float = 0.0
    near_path_x: float = 0.0
    mid_path_x: float = 0.0
    far_path_x: float = 0.0
    control_sample_count: int = 0
    rejected_sample_count: int = 0
    preview_event_detected: bool = False
    preview_event_type: str = "NONE"
    preview_event_direction: str = "NONE"
    preview_event_proximity: float = 0.0
    preview_event_confidence: float = 0.0
    corner_anchor_x: float = 0.0
    corner_anchor_y: float = 0.0
    green_action: str = "NENHUM"
    green_proximity: float = 0.0
    green_confidence: float = 0.0


@dataclass
class PathTrackingState:
    # A previsão só é atualizada depois de um CurrentPath válido. Após duas falhas,
    # ela volta ao centro para não manter o rastreador preso em um histórico antigo.
    expected_root_x_frame: float = float(SETPOINT_X)
    expected_root_y_frame: float = float(FRAME_HEIGHT - 1)
    expected_root_width_process: float = 0.0
    invalid_frame_count: int = 0

def handle_signal(signum, frame):
    global running
    running = False
    with frame_condition:
        frame_condition.notify_all()

class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True

class CameraStreamHandler(BaseHTTPRequestHandler):
    def log_message(self, format_text, *args):
        return

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
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.end_headers()
            return

        self.send_response(200)
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

def start_stream_server():
    try:
        server = ReusableThreadingHTTPServer(("127.0.0.1", MJPEG_STREAM_PORT), CameraStreamHandler)
    except OSError as error:
        print(f"Não foi possível iniciar o stream MJPEG na porta {MJPEG_STREAM_PORT}: {error}", flush=True)
        return None

    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"Stream MJPEG disponível em http://127.0.0.1:{MJPEG_STREAM_PORT}{MJPEG_STREAM_PATH}", flush=True)
    return server

def scale_roi_contour_to_frame(contour):
    scaled_contour = contour.astype(np.float32)
    scaled_contour[:, :, 0] *= FRAME_WIDTH / PROCESS_WIDTH
    scaled_contour[:, :, 1] = (scaled_contour[:, :, 1] + PROCESS_ROI_TOP_Y) * FRAME_HEIGHT / PROCESS_HEIGHT
    return scaled_contour.astype(np.int32)

def build_line_candidate_mask(mask_black):
    """Mantém todos os blobs grandes; o enraizamento decide qual chega ao robô."""

    contours, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidate_mask = np.zeros_like(mask_black)
    valid_contours = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_LINE_AREA:
            continue
        valid_contours.append(contour)

    if valid_contours:
        cv2.drawContours(candidate_mask, valid_contours, -1, 255, thickness=cv2.FILLED)
    return candidate_mask, valid_contours

def detect_green_observation(frame, mask_green, mask_black, path_observation):
    contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    middle_x = PROCESS_WIDTH // 2
    left_marker_valid = False
    right_marker_valid = False
    valid_proximities = []
    valid_confidences = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_GREEN_AREA:
            continue

        moments = cv2.moments(contour)
        if moments["m00"] <= 0:
            continue

        green_x = int(moments["m10"] / moments["m00"])
        green_y = int(moments["m01"] / moments["m00"])
        check_top = max(0, green_y - 35)
        check_left = max(0, green_x - 10)
        check_right = min(PROCESS_WIDTH, green_x + 10)
        black_before_green = mask_black[check_top:green_y, check_left:check_right]
        marker_valid = np.any(black_before_green == 255)
        local_path_x = path_x_at_y(path_observation.current_path, green_y, middle_x)
        black_support = float(np.count_nonzero(black_before_green)) / max(1, black_before_green.size)
        marker_confidence = min(
            1.0,
            0.60 * min(area / float(MIN_GREEN_AREA * 3), 1.0)
            + 0.40 * min(black_support / 0.25, 1.0),
        )
        marker_valid = (
            marker_valid
            and path_observation.current_path_valid
            and marker_confidence >= MIN_GREEN_CONFIDENCE
        )

        if marker_valid:
            # O lado do verde é relativo ao caminho naquela altura, não ao
            # centro absoluto da imagem. Isso continua correto em pista inclinada.
            if green_x < local_path_x:
                left_marker_valid = True
            else:
                right_marker_valid = True
            valid_proximities.append(green_y / max(1.0, mask_green.shape[0] - 1.0))
            valid_confidences.append(marker_confidence)

        # Verde validado fica destacado; verde sem linha antes dele aparece cinza.
        color = (0, 255, 0) if marker_valid else (100, 100, 100)
        cv2.drawContours(frame, [scale_roi_contour_to_frame(contour)], -1, color, 3)
        local_path_x_frame = int(local_path_x * FRAME_WIDTH / PROCESS_WIDTH)
        green_y_frame = int((green_y + PROCESS_ROI_TOP_Y) * FRAME_HEIGHT / PROCESS_HEIGHT)
        cv2.circle(frame, (local_path_x_frame, green_y_frame), 4, (255, 255, 0), -1)

    if left_marker_valid and right_marker_valid:
        action = "MEIA VOLTA"
    elif left_marker_valid:
        action = "ESQUERDA"
    elif right_marker_valid:
        action = "DIREITA"
    else:
        action = "NENHUM"

    return GreenObservation(
        action=action,
        proximity=max(valid_proximities, default=0.0),
        confidence=max(valid_confidences, default=0.0),
    )


def path_sample_to_frame(sample):
    x = int(sample.x * FRAME_WIDTH / PROCESS_WIDTH)
    y = int((sample.y + PROCESS_ROI_TOP_Y) * FRAME_HEIGHT / PROCESS_HEIGHT)
    return x, y


def proximity_to_frame_y(proximity, roi_height):
    process_y = proximity * max(1, roi_height - 1) + PROCESS_ROI_TOP_Y
    return int(process_y * FRAME_HEIGHT / PROCESS_HEIGHT)


def draw_path_debug(
    frame,
    path_observation,
    green_observation,
    roi_height,
    black_threshold,
    black_coverage_percent,
    candidate_contour_count,
):
    # CurrentPath em azul controla direção; Preview em amarelo apenas descreve
    # o futuro. As cores diferentes tornam uma curva antecipada visível no stream.
    for sample in path_observation.samples:
        cv2.circle(frame, path_sample_to_frame(sample), 2, (160, 160, 160), -1)

    for sample in path_observation.rejected_samples:
        color = (255, 0, 255) if "width" in sample.rejection_reason else (0, 0, 255)
        point = path_sample_to_frame(sample)
        cv2.drawMarker(frame, point, color, cv2.MARKER_TILTED_CROSS, 12, 2)
        cv2.putText(
            frame,
            sample.rejection_reason.upper(),
            (point[0] + 7, point[1] - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.32,
            color,
            1,
        )

    for samples, color in (
        (path_observation.current_path, (255, 0, 0)),
        (path_observation.preview_path, (0, 255, 255)),
    ):
        for sample in samples:
            cv2.circle(frame, path_sample_to_frame(sample), 4, color, -1)
        for first, second in zip(samples, samples[1:]):
            cv2.line(frame, path_sample_to_frame(first), path_sample_to_frame(second), color, 3)

    for sample in path_observation.control_samples:
        cv2.circle(frame, path_sample_to_frame(sample), 6, (255, 255, 0), 2)

    if path_observation.current_path_valid:
        control_point = (
            int(path_observation.control_x * FRAME_WIDTH / PROCESS_WIDTH),
            int((path_observation.control_y + PROCESS_ROI_TOP_Y) * FRAME_HEIGHT / PROCESS_HEIGHT),
        )
        cv2.circle(frame, control_point, 7, (255, 255, 0), -1)
        heading_radians = np.deg2rad(path_observation.heading_error_degrees)
        heading_length = 58
        heading_end = (
            int(control_point[0] + np.sin(heading_radians) * heading_length),
            int(control_point[1] - np.cos(heading_radians) * heading_length),
        )
        cv2.arrowedLine(frame, control_point, heading_end, (255, 255, 0), 2, tipLength=0.25)

    if path_observation.current_path:
        root_point = path_sample_to_frame(path_observation.current_path[0])
        cv2.drawMarker(frame, root_point, (0, 165, 255), cv2.MARKER_DIAMOND, 14, 2)
        cv2.putText(frame, "ROOT", (root_point[0] + 8, root_point[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 165, 255), 1)

    control_zone_y = proximity_to_frame_y(CONTROL_ZONE_MIN_PROXIMITY, roi_height)
    approach_zone_y = proximity_to_frame_y(EVENT_APPROACH_PROXIMITY, roi_height)
    action_zone_y = proximity_to_frame_y(EVENT_ACTION_PROXIMITY, roi_height)
    cv2.line(frame, (0, control_zone_y), (FRAME_WIDTH - 1, control_zone_y), (255, 110, 0), 1)
    cv2.line(frame, (0, approach_zone_y), (FRAME_WIDTH - 1, approach_zone_y), (0, 165, 255), 1)
    cv2.line(frame, (0, action_zone_y), (FRAME_WIDTH - 1, action_zone_y), (0, 0, 255), 2)
    cv2.putText(frame, "CONTROL", (FRAME_WIDTH - 105, control_zone_y + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 110, 0), 1)
    cv2.putText(frame, "PREVIEW", (FRAME_WIDTH - 100, ROI_TOP_Y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    cv2.putText(frame, "APPROACH", (FRAME_WIDTH - 115, approach_zone_y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 165, 255), 1)
    cv2.putText(frame, "ACTION", (FRAME_WIDTH - 90, action_zone_y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

    corner = path_observation.corner
    if corner.detected:
        corner_point = (
            int(corner.anchor_x * FRAME_WIDTH / PROCESS_WIDTH),
            int((corner.anchor_y + PROCESS_ROI_TOP_Y) * FRAME_HEIGHT / PROCESS_HEIGHT),
        )
        cv2.drawMarker(frame, corner_point, (0, 0, 255), cv2.MARKER_DIAMOND, 22, 3)
        cv2.putText(frame, f"ANCHOR {corner.direction}", (corner_point[0] + 14, corner_point[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 255), 2)

    if not path_observation.current_path_valid:
        vision_state = "LINE_LOST / ROOT_REJECTED"
    elif corner.detected and corner.proximity >= EVENT_ACTION_PROXIMITY:
        vision_state = "EVENT_AT_ACTION"
    elif corner.detected and corner.proximity >= EVENT_APPROACH_PROXIMITY:
        vision_state = "APPROACHING_EVENT"
    elif corner.detected:
        vision_state = "PREVIEW_ONLY"
    else:
        vision_state = "FOLLOWING"

    status_text = "VALID" if path_observation.current_path_valid else "INVALID"
    event_text = (
        f"corner: {corner.direction} anchorY={corner.anchor_y:.1f}px p={corner.proximity:.2f} c={corner.confidence:.2f}"
        if corner.detected
        else "corner: NONE anchorY=--"
    )
    rejection_reasons = ",".join(
        sorted({sample.rejection_reason for sample in path_observation.rejected_samples})
    ) or "none"
    overlay_lines = [
        f"FPS: {getattr(process_frame, 'current_fps', 0):.1f}  CurrentPath={status_text}",
        f"vision cue: {vision_state}",
        f"position: {path_observation.position_error_pixels * FRAME_WIDTH / PROCESS_WIDTH:+.1f}px",
        f"heading: {path_observation.heading_error_degrees:+.1f}deg",
        f"path confidence: {path_observation.path_confidence:.2f}  control samples: {path_observation.control_sample_count}",
        f"root expected/accepted/width: {path_observation.expected_root_x:.1f} / {path_observation.root_x:.1f} / {path_observation.root_width:.1f}px",
        f"black mask: V<={black_threshold} coverage={black_coverage_percent:.1f}% blobs={candidate_contour_count}",
        f"rejected: {len(path_observation.rejected_samples)} ({rejection_reasons})",
        event_text,
        f"green: {green_observation.action} p={green_observation.proximity:.2f} c={green_observation.confidence:.2f}",
    ]
    cv2.rectangle(frame, (4, 4), (650, 241), (5, 12, 16), -1)
    for index, text in enumerate(overlay_lines):
        color = (0, 255, 0) if index in (0, 9) else (235, 235, 235)
        cv2.putText(frame, text, (10, 25 + index * 23), cv2.FONT_HERSHEY_SIMPLEX, 0.53, color, 1, cv2.LINE_AA)


def process_frame(frame, tracking_state):
    # O segue-linha olha a região inferior da imagem em baixa resolução para
    # preservar FPS sem perder a imagem detalhada enviada ao dashboard.
    process_frame_small = cv2.resize(frame, (PROCESS_WIDTH, PROCESS_HEIGHT), interpolation=cv2.INTER_AREA)
    roi = process_frame_small[PROCESS_ROI_TOP_Y:PROCESS_HEIGHT, :]
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    kernel = np.ones((3, 3), np.uint8)
    blurred_value = cv2.GaussianBlur(hsv_roi[:, :, 2], (5, 5), 0)
    otsu_threshold, _ = cv2.threshold(
        blurred_value,
        0,
        255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU,
    )
    effective_black_threshold = int(
        np.clip(otsu_threshold, BLACK_THRESHOLD_MIN, BLACK_THRESHOLD_MAX)
    )
    _, mask_black = cv2.threshold(
        blurred_value,
        effective_black_threshold,
        255,
        cv2.THRESH_BINARY_INV,
    )
    mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, kernel, iterations=1)
    black_coverage_percent = (
        100.0 * float(np.count_nonzero(mask_black)) / max(1, mask_black.size)
    )
    mask_green = cv2.inRange(hsv_roi, GREEN_LOWER, GREEN_UPPER)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel, iterations=1)

    # Todos os blobs com área útil permanecem disponíveis. A escolha da linha
    # acontece pelas bandas próximas do robô, não pelo centro da bounding box
    # do contorno inteiro, que muda bastante em curvas.
    primary_mask, line_contours = build_line_candidate_mask(mask_black)
    for contour in line_contours:
        cv2.drawContours(frame, [scale_roi_contour_to_frame(contour)], -1, (75, 75, 75), 1)
    start_x_small = (
        tracking_state.expected_root_x_frame * PROCESS_WIDTH / FRAME_WIDTH
    )
    expected_root_width = (
        tracking_state.expected_root_width_process
        if tracking_state.expected_root_width_process > 0.0
        else None
    )
    allow_root_recovery = (
        tracking_state.invalid_frame_count >= ROOT_RECOVERY_AFTER_INVALID_FRAMES
    )
    path_observation = analyze_primary_path(
        primary_mask,
        start_x_small,
        expected_root_width,
        allow_root_recovery=allow_root_recovery,
    )

    if path_observation.current_path_valid:
        tracking_state.invalid_frame_count = 0
        tracking_state.expected_root_x_frame = (
            path_observation.root_x * FRAME_WIDTH / PROCESS_WIDTH
        )
        tracking_state.expected_root_y_frame = (
            (path_observation.current_path[0].y + PROCESS_ROI_TOP_Y)
            * FRAME_HEIGHT
            / PROCESS_HEIGHT
        )
        tracking_state.expected_root_width_process = path_observation.root_width
    else:
        tracking_state.invalid_frame_count += 1
        if tracking_state.invalid_frame_count >= ROOT_RECOVERY_AFTER_INVALID_FRAMES:
            # A próxima análise parte de uma referência neutra. Manter X ou
            # largura antigos aqui poderia rejeitar novamente a linha frontal.
            tracking_state.expected_root_x_frame = float(SETPOINT_X)
            tracking_state.expected_root_y_frame = float(FRAME_HEIGHT - 1)
            tracking_state.expected_root_width_process = 0.0

    green_observation = detect_green_observation(frame, mask_green, mask_black, path_observation)
    draw_path_debug(
        frame,
        path_observation,
        green_observation,
        mask_black.shape[0],
        effective_black_threshold,
        black_coverage_percent,
        len(line_contours),
    )

    cv2.line(frame, (SETPOINT_X, ROI_TOP_Y), (SETPOINT_X, FRAME_HEIGHT - 1), (255, 255, 0), 1)
    cv2.rectangle(frame, (0, ROI_TOP_Y), (FRAME_WIDTH - 1, FRAME_HEIGHT - 1), (80, 80, 80), 1)

    position_error_full = path_observation.position_error_pixels * FRAME_WIDTH / PROCESS_WIDTH
    corner_anchor_x_full = (
        path_observation.corner.anchor_x * FRAME_WIDTH / PROCESS_WIDTH
        if path_observation.corner.detected
        else 0.0
    )
    corner_anchor_y_full = (
        (path_observation.corner.anchor_y + PROCESS_ROI_TOP_Y)
        * FRAME_HEIGHT
        / PROCESS_HEIGHT
        if path_observation.corner.detected
        else 0.0
    )
    status = VisionStatus(
        line_detected=path_observation.line_detected,
        current_path_valid=path_observation.current_path_valid,
        line_error=position_error_full,
        position_error_pixels=position_error_full,
        heading_error_degrees=path_observation.heading_error_degrees,
        path_confidence=path_observation.path_confidence,
        near_path_x=path_observation.near_path_x * FRAME_WIDTH / PROCESS_WIDTH,
        mid_path_x=path_observation.mid_path_x * FRAME_WIDTH / PROCESS_WIDTH,
        far_path_x=path_observation.far_path_x * FRAME_WIDTH / PROCESS_WIDTH,
        control_sample_count=path_observation.control_sample_count,
        rejected_sample_count=len(path_observation.rejected_samples),
        preview_event_detected=path_observation.corner.detected,
        preview_event_type="CORNER" if path_observation.corner.detected else "NONE",
        preview_event_direction=path_observation.corner.direction,
        preview_event_proximity=path_observation.corner.proximity,
        preview_event_confidence=path_observation.corner.confidence,
        corner_anchor_x=corner_anchor_x_full,
        corner_anchor_y=corner_anchor_y_full,
        green_action=green_observation.action,
        green_proximity=green_observation.proximity,
        green_confidence=green_observation.confidence,
    )
    return frame, tracking_state, status

def encode_frame(frame):
    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
    ok, encoded = cv2.imencode(".jpg", frame, encode_params)
    if not ok:
        return None

    return encoded.tobytes()

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

def save_status(fps, vision_status=None, camera_format="", active=True, error_message=""):
    vision_status = vision_status or VisionStatus()
    status = {
        "fps": round(fps, 2),
        "active": active,
        "lineDetected": vision_status.line_detected,
        "currentPathValid": vision_status.current_path_valid,
        # Mantém compatibilidade: lineError continua sendo o erro de posição.
        "lineError": round(vision_status.line_error, 3),
        "positionErrorPixels": round(vision_status.position_error_pixels, 3),
        "headingErrorDegrees": round(vision_status.heading_error_degrees, 3),
        "pathConfidence": round(vision_status.path_confidence, 3),
        "nearPathX": round(vision_status.near_path_x, 2),
        "midPathX": round(vision_status.mid_path_x, 2),
        "farPathX": round(vision_status.far_path_x, 2),
        "controlSampleCount": vision_status.control_sample_count,
        "rejectedSampleCount": vision_status.rejected_sample_count,
        "previewEventDetected": vision_status.preview_event_detected,
        "previewEventType": vision_status.preview_event_type,
        "previewEventDirection": vision_status.preview_event_direction,
        "previewEventProximity": round(vision_status.preview_event_proximity, 3),
        "previewEventConfidence": round(vision_status.preview_event_confidence, 3),
        "cornerAnchorX": round(vision_status.corner_anchor_x, 2),
        "cornerAnchorY": round(vision_status.corner_anchor_y, 2),
        "greenAction": vision_status.green_action,
        "greenProximity": round(vision_status.green_proximity, 3),
        "greenConfidence": round(vision_status.green_confidence, 3),
        "width": FRAME_WIDTH,
        "height": FRAME_HEIGHT,
        "processWidth": PROCESS_WIDTH,
        "processHeight": PROCESS_HEIGHT,
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

def create_camera():
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
            print(f"Câmera configurada em {pixel_format} com alvo de {TARGET_CAMERA_FPS} FPS.", flush=True)
            return picam2, pixel_format
        except Exception as error:
            print(f"Configuração {pixel_format} com FPS fixo falhou: {error}", flush=True)

    for pixel_format in CAMERA_PIXEL_FORMATS:
        try:
            camera_config = picam2.create_video_configuration(
                main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": pixel_format},
                transform=camera_transform,
                buffer_count=4,
            )
            picam2.configure(camera_config)
            print(f"Câmera configurada em {pixel_format} sem FPS fixo.", flush=True)
            return picam2, pixel_format
        except Exception as error:
            print(f"Configuração {pixel_format} simples falhou: {error}", flush=True)

    camera_config = picam2.create_still_configuration(
        {"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"},
        transform=camera_transform,
    )
    picam2.configure(camera_config)
    print("Câmera configurada em modo still como fallback.", flush=True)
    return picam2, "RGB888"

def normalize_frame_colors(frame, camera_format):
    # O frame já está no formato correto para o pipeline OpenCV usado aqui.
    # Não aplicamos conversão de RGB para BGR porque isso inverteu vermelho e azul.
    return frame


def camera_format_label(camera_format):
    return camera_format

def tune_camera_image(picam2):
    # Estes ajustes são visuais e não fazem parte da lógica de segurança.
    # Se a câmera não suportar algum controle, o script continua rodando.
    camera_controls = {
        "AeEnable": True,
        "AwbEnable": True,
        "ExposureValue": CAMERA_EXPOSURE_VALUE,
        "Sharpness": CAMERA_SHARPNESS,
        "Contrast": CAMERA_CONTRAST,
        "Saturation": CAMERA_SATURATION,
    }

    applied_controls = []
    for name, value in camera_controls.items():
        try:
            picam2.set_controls({name: value})
            applied_controls.append(name)
        except Exception as error:
            print(f"Controle de câmera {name} não foi aplicado: {error}", flush=True)

    if applied_controls:
        print(f"Ajustes de imagem aplicados: {', '.join(applied_controls)}.", flush=True)

def main():
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    if GPIO is None or Picamera2 is None:
        error_message = "Dependências (GPIO ou Picamera2) não encontradas."
        print(error_message, flush=True)
        save_status(0.0, active=False, error_message=error_message)
        return 1

    # Cria o arquivo de status inicial para evitar que o dashboard fique em
    # 'aguardando' indefinidamente enquanto a câmera está sendo preparada.
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

        tracking_state = PathTrackingState()

        prev_time = time.monotonic()
        last_stream_frame_time = 0.0
        last_snapshot_save_time = 0.0
        last_status_save_time = 0.0
        stream_frame_interval = 1.0 / MJPEG_STREAM_FPS
        snapshot_save_interval = 1.0 / SNAPSHOT_FRAME_FPS
        status_save_interval = 1.0 / STATUS_FPS
        smoothed_fps = 0.0
        vision_status = VisionStatus()

        while running:
            frame = picam2.capture_array()
            frame = normalize_frame_colors(frame, camera_format)

            curr_time = time.monotonic()
            elapsed = curr_time - prev_time
            prev_time = curr_time

            if elapsed > 0:
                actual_fps = 1.0 / elapsed
                smoothed_fps = (smoothed_fps * 0.90) + (actual_fps * 0.10) if smoothed_fps > 0 else actual_fps

            process_frame.current_fps = smoothed_fps
            frame, tracking_state, vision_status = process_frame(frame, tracking_state)

            should_stream = curr_time - last_stream_frame_time >= stream_frame_interval
            should_save_snapshot = curr_time - last_snapshot_save_time >= snapshot_save_interval

            # O stream MJPEG fica em memória; o arquivo em /tmp é só compatibilidade.
            # Assim o dashboard pode ver vídeo rápido sem forçar escrita em disco.
            if should_stream or should_save_snapshot:
                jpeg = encode_frame(frame)
                if jpeg is not None:
                    if should_stream:
                        publish_stream_frame(jpeg)
                        last_stream_frame_time = curr_time

                    if should_save_snapshot:
                        save_frame(jpeg)
                        last_snapshot_save_time = curr_time

            if curr_time - last_status_save_time >= status_save_interval:
                save_status(smoothed_fps, vision_status, camera_format_label(camera_format))
                last_status_save_time = curr_time
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
