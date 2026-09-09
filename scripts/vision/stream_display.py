"""Stream MJPEG, snapshots e composição visual."""

import argparse
import os
import threading
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
from urllib.parse import parse_qs
from urllib.parse import urlsplit
import cv2  # type: ignore
import numpy as np
from .camera_config import (
    CAMERA_PROFILES,
    DISPLAY_MODES,
    DISPLAY_MODE_LINE,
    DISPLAY_MODE_REAL,
    FRAME_PATH,
    JPEG_QUALITY,
    MJPEG_STREAM_FPS,
    MJPEG_STREAM_PATH,
    MJPEG_STREAM_PORT,
    TEMP_FRAME_PATH,
)
from .green_detection import (
    green_geometry_rejection_reasons,
)

running = True
latest_jpeg = None
latest_jpeg_sequence = 0
active_stream_clients = 0
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


def draw_line_illumination_overlay(
    display_frame,
    line_candidate_mask,
    zone_mask,
    illumination_status,
    display_mode,
    uncorrected_line_candidate_mask=None,
):
    """Mostra a área compensada sem modificar a máscara usada no controle."""

    status = illumination_status if isinstance(illumination_status, dict) else {}
    configured = status.get("illuminationCorrectionConfigured") is True
    active = status.get("illuminationCorrectionActive") is True
    if not configured:
        return

    if active and zone_mask is not None:
        candidate_height = (
            line_candidate_mask.shape[0]
            if line_candidate_mask is not None
            else display_frame.shape[0]
        )
        candidate_start_y = max(0, display_frame.shape[0] - candidate_height)
        useful_height = min(
            display_frame.shape[0] - candidate_start_y,
            zone_mask.shape[0] - candidate_start_y,
            candidate_height,
        )
        useful_width = min(display_frame.shape[1], zone_mask.shape[1])
        zone = zone_mask[
            candidate_start_y:candidate_start_y + useful_height,
            :useful_width,
        ]
        candidates = (
            line_candidate_mask[:useful_height, :useful_width]
            if line_candidate_mask is not None
            else np.zeros(zone.shape, dtype=np.uint8)
        )
        background_zone = cv2.bitwise_and(
            zone,
            cv2.bitwise_not(candidates),
        )
        region = display_frame[
            candidate_start_y:candidate_start_y + useful_height,
            :useful_width,
        ]
        line_diagnostic = normalize_display_mode(display_mode) == DISPLAY_MODE_LINE
        if line_diagnostic:
            # Na máscara, ciano escuro delimita a compensação e o branco
            # continua reservado aos pixels realmente entregues ao seguidor.
            cv2.add(
                region,
                (56, 56, 0, 0),
                dst=region,
                mask=background_zone,
            )
            if uncorrected_line_candidate_mask is not None:
                old_candidates = uncorrected_line_candidate_mask[
                    :useful_height,
                    :useful_width,
                ]
                removed = cv2.bitwise_and(
                    old_candidates,
                    cv2.bitwise_not(candidates),
                )
                # Vermelho mostra exatamente o que a pipeline sem compensação
                # entregaria ao Fusion e a correção fotométrica rejeitou.
                region[removed > 0] = (0, 0, 255)

            preserved = cv2.bitwise_and(zone, candidates)
            preserved_contours, _ = cv2.findContours(
                preserved.copy(),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            # O contorno verde identifica fita preservada sem substituir o
            # interior branco, que continua representando a máscara final.
            cv2.drawContours(region, preserved_contours, -1, (0, 255, 0), 1)
        else:
            cv2.add(
                region,
                (42, 42, 0, 0),
                dst=region,
                mask=background_zone,
            )
            # Verde significa candidato preto preservado dentro da área
            # compensada. A confirmação de fita real continua geométrica.
            preserved = cv2.bitwise_and(zone, candidates)
            cv2.add(
                region,
                (0, 110, 0, 0),
                dst=region,
                mask=preserved,
            )
        contours, _ = cv2.findContours(
            zone.copy(),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        cv2.drawContours(region, contours, -1, (255, 255, 0), 1)

    try:
        maximum_gain = float(status.get("illuminationMaximumGain", 1.0))
        correction_ms = float(status.get("illuminationCorrectionMs", 0.0))
    except (TypeError, ValueError):
        maximum_gain = 1.0
        correction_ms = 0.0
    reference_sha256 = str(status.get("illuminationReferenceSha256", ""))
    short_hash = reference_sha256[:8] if reference_sha256 else "NOHASH"
    label = (
        f"ILLUM ON {short_hash} G{maximum_gain:.2f} {correction_ms:.2f} ms"
        if active
        else "ILLUM OFF  MAP INVALID"
    )
    color = (255, 255, 0) if active else (0, 0, 255)
    text_size = cv2.getTextSize(
        label,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.36,
        1,
    )[0]
    left = max(4, display_frame.shape[1] - text_size[0] - 10)
    bottom = max(18, display_frame.shape[0] - 5)
    cv2.rectangle(
        display_frame,
        (left - 4, bottom - 15),
        (display_frame.shape[1] - 3, bottom + 3),
        (0, 0, 0),
        -1,
    )
    cv2.putText(
        display_frame,
        label,
        (left, bottom),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.36,
        color,
        1,
        cv2.LINE_AA,
    )


def draw_silver_shadow_overlay(display_frame, shadow_status):
    """Mostra a decisão shadow somente na cópia enviada ao dashboard."""

    status = shadow_status if isinstance(shadow_status, dict) else {}
    available = status.get("silverShadowAvailable") is True
    detected = status.get("silverShadowDetected") is True

    if available:
        label = str(status.get("silverShadowLabel") or "sem leitura").upper()
        try:
            silver_percent = float(status.get("silverShadowProbability", 0.0)) * 100.0
            margin_percent = float(status.get("silverShadowMargin", 0.0)) * 100.0
            inference_ms = float(status.get("silverShadowInferenceMs", 0.0))
        except (TypeError, ValueError):
            silver_percent = 0.0
            margin_percent = 0.0
            inference_ms = 0.0
        title = f"PRATA {label}"
        details = (
            f"S {silver_percent:.1f}%  M {margin_percent:.1f}%  "
            f"{inference_ms:.1f} ms"
        )
        confirmation_frames = int(status.get("silverConfirmationFrames", 0))
        required_frames = int(
            status.get("silverConfirmationRequiredFrames", 4)
        )
        title = (
            "ENTRADA RESGATE CONFIRMADA"
            if status.get("courseMarkerConfirmed") is True
            else f"{title}  {confirmation_frames}/{required_frames}"
        )
    else:
        title = "PRATA INDISPONIVEL"
        details = "CLASSIFICADOR DE PRATA"

    color = (0, 255, 255) if detected else (190, 190, 190)
    font = cv2.FONT_HERSHEY_SIMPLEX
    title_size = cv2.getTextSize(title, font, 0.46, 1)[0]
    details_size = cv2.getTextSize(details, font, 0.36, 1)[0]
    box_width = max(title_size[0], details_size[0]) + 14
    box_height = 43
    left = max(0, display_frame.shape[1] - box_width - 5)
    top = 5
    right = display_frame.shape[1] - 5
    bottom = min(display_frame.shape[0] - 1, top + box_height)

    cv2.rectangle(display_frame, (left, top), (right, bottom), (0, 0, 0), -1)
    cv2.rectangle(display_frame, (left, top), (right, bottom), color, 1)
    cv2.putText(
        display_frame,
        title,
        (left + 7, top + 17),
        font,
        0.46,
        color,
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        display_frame,
        details,
        (left + 7, top + 35),
        font,
        0.36,
        color,
        1,
        cv2.LINE_AA,
    )


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


def draw_green_rejection_details(display_frame, rejected_candidates):
    """Mostra por que um componente verde não chegou à classificação."""

    for candidate in rejected_candidates:
        reasons = green_geometry_rejection_reasons(candidate)
        if "area_below_scaled_reference" in reasons:
            detail = (
                f"GREEN AREA {candidate['area']:.0f}/"
                f"{candidate['scaled_minimum_area']:.0f}"
            )
        else:
            detail = "GREEN REJECT " + (
                ",".join(reasons) if reasons else "UNKNOWN"
            )
        x, y, _width, _height = candidate["bounding_box"]
        cv2.putText(
            display_frame,
            detail,
            (max(4, int(x)), max(14, int(y) - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (0, 220, 255),
            1,
            cv2.LINE_AA,
        )


def parse_camera_profile(arguments=None):
    """Aceita somente a CAM0; a frontal possui um processo dedicado e leve."""

    environment_role = os.environ.get("OBR_CAMERA_ROLE", "down").strip().lower()
    parser = argparse.ArgumentParser(
        description="Captura e processa a câmera inferior do robô."
    )
    parser.add_argument(
        "--camera-role",
        choices=("down",),
        default=environment_role,
        help="Papel físico desta captura: somente down.",
    )
    parsed = parser.parse_args(arguments)
    if parsed.camera_role != "down":
        parser.error(
            "OBR_CAMERA_ROLE deve ser 'down'; use forward_camera_stream.py "
            "para a câmera frontal."
        )
    return CAMERA_PROFILES["down"]


def handle_signal(signum, frame):
    """Encerra o stream de forma limpa quando o serviço recebe um sinal."""

    del signum, frame
    global running
    running = False
    with frame_condition:
        frame_condition.notify_all()


def register_stream_client():
    """Registra uma conexão MJPEG para habilitar a codificação rápida."""

    global active_stream_clients
    with frame_condition:
        active_stream_clients += 1


def unregister_stream_client():
    """Remove uma conexão MJPEG encerrada sem permitir contagem negativa."""

    global active_stream_clients
    with frame_condition:
        active_stream_clients = max(0, active_stream_clients - 1)


def stream_frame_is_due(now, last_stream_time):
    """Solicita JPEG de stream somente enquanto existe cliente conectado."""

    with frame_condition:
        has_clients = active_stream_clients > 0
    return (
        has_clients
        and now - last_stream_time >= 1.0 / MJPEG_STREAM_FPS
    )


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
        register_stream_client()
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
        finally:
            unregister_stream_client()

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
