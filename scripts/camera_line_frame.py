import json
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
    from picamera2 import Picamera2  # type: ignore[import]
except ImportError:
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
STATUS_FPS = 10
# Qualidade do JPEG do stream MJPEG.
# Valores maiores deixam a imagem mais limpa, mas usam mais banda na rede.
JPEG_QUALITY = 82
# Limite de brilho usado para separar a linha preta do piso.
# Aumentar este valor aceita tons mais claros como linha, mas pode pegar sombras.
BLACK_THRESHOLD = 80
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

# Ajustes visuais para a Raspberry Pi Camera V2.
# Eles favorecem nitidez no dashboard sem mudar a lógica de segurança do robô.
CAMERA_SHARPNESS = 1.2
CAMERA_CONTRAST = 1.05
CAMERA_SATURATION = 1.0
CAMERA_EXPOSURE_VALUE = 0.4
CAMERA_PIXEL_FORMATS = ("RGB888",)

running = True
latest_jpeg = None
latest_jpeg_sequence = 0
frame_condition = threading.Condition()

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

def find_line_contour(mask_black, last_position):
    contours, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    last_center_x_small = int(last_position[0] * PROCESS_WIDTH / FRAME_WIDTH)
    best_contour = None
    best_distance = float("inf")

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_LINE_AREA:
            continue

        x, _, width, _ = cv2.boundingRect(contour)
        center_x_small = x + (width // 2)
        distance = abs(center_x_small - last_center_x_small)
        if distance < best_distance:
            best_distance = distance
            best_contour = contour

    return best_contour

def detect_green_action(frame, mask_green, mask_black):
    contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    middle_x = PROCESS_WIDTH // 2
    left_marker_valid = False
    right_marker_valid = False

    for contour in contours:
        if cv2.contourArea(contour) < MIN_GREEN_AREA:
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

        if marker_valid:
            if green_x < middle_x:
                left_marker_valid = True
            else:
                right_marker_valid = True

        # Verde validado fica destacado; verde sem linha antes dele aparece cinza.
        color = (0, 255, 0) if marker_valid else (100, 100, 100)
        cv2.drawContours(frame, [scale_roi_contour_to_frame(contour)], -1, color, 3)

    if left_marker_valid and right_marker_valid:
        return "MEIA VOLTA"
    if left_marker_valid:
        return "ESQUERDA"
    if right_marker_valid:
        return "DIREITA"
    return "NENHUM"

def process_frame(frame, last_position):
    # O segue-linha olha a região inferior da imagem em baixa resolução para
    # preservar FPS sem perder a imagem detalhada enviada ao dashboard.
    process_frame_small = cv2.resize(frame, (PROCESS_WIDTH, PROCESS_HEIGHT), interpolation=cv2.INTER_AREA)
    roi = process_frame_small[PROCESS_ROI_TOP_Y:PROCESS_HEIGHT, :]
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    kernel = np.ones((3, 3), np.uint8)
    blurred_value = cv2.GaussianBlur(hsv_roi[:, :, 2], (5, 5), 0)
    _, mask_black = cv2.threshold(blurred_value, BLACK_THRESHOLD, 255, cv2.THRESH_BINARY_INV)
    mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, kernel, iterations=1)
    mask_green = cv2.inRange(hsv_roi, GREEN_LOWER, GREEN_UPPER)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel, iterations=1)

    line_detected = False
    error = 0
    green_action = detect_green_action(frame, mask_green, mask_black)
    contour = find_line_contour(mask_black, last_position)

    if contour is not None:
        moments = cv2.moments(contour)
        if moments["m00"] > 0:
            center_x_small = int(moments["m10"] / moments["m00"])
            center_y_small = int(moments["m01"] / moments["m00"]) + PROCESS_ROI_TOP_Y
            center_x = int(center_x_small * FRAME_WIDTH / PROCESS_WIDTH)
            center_y = int(center_y_small * FRAME_HEIGHT / PROCESS_HEIGHT)
            last_position = (center_x, center_y)
            error = int(center_x - SETPOINT_X)
            line_detected = True

            cv2.drawContours(frame, [scale_roi_contour_to_frame(contour)], -1, (255, 0, 0), 2)
            cv2.line(frame, (center_x, ROI_TOP_Y), (center_x, FRAME_HEIGHT - 1), (255, 0, 0), 2)

    cv2.line(frame, (SETPOINT_X, ROI_TOP_Y), (SETPOINT_X, FRAME_HEIGHT - 1), (255, 255, 0), 1)
    cv2.rectangle(frame, (0, ROI_TOP_Y), (FRAME_WIDTH - 1, FRAME_HEIGHT - 1), (80, 80, 80), 1)

    status_text = "linha ok" if line_detected else "linha perdida"
    cv2.putText(frame, f"FPS: {getattr(process_frame, 'current_fps', 0):.1f}", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(frame, f"erro: {error}", (8, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    cv2.putText(frame, status_text, (8, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(frame, f"verde: {green_action}", (8, 96), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return frame, last_position, line_detected, error, green_action

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

def save_status(fps, line_detected, line_error, green_action="NENHUM", camera_format="", active=True, error_message=""):
    status = {
        "fps": round(fps, 2),
        "active": active,
        "lineDetected": line_detected,
        "lineError": line_error,
        "greenAction": green_action,
        "width": FRAME_WIDTH,
        "height": FRAME_HEIGHT,
        "processWidth": PROCESS_WIDTH,
        "processHeight": PROCESS_HEIGHT,
        "jpegQuality": JPEG_QUALITY,
        "targetCameraFps": TARGET_CAMERA_FPS,
        "cameraFormat": camera_format,
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

    for pixel_format in CAMERA_PIXEL_FORMATS:
        try:
            camera_config = picam2.create_video_configuration(
                main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": pixel_format},
                controls={"FrameDurationLimits": (frame_duration_us, frame_duration_us)},
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
                buffer_count=4,
            )
            picam2.configure(camera_config)
            print(f"Câmera configurada em {pixel_format} sem FPS fixo.", flush=True)
            return picam2, pixel_format
        except Exception as error:
            print(f"Configuração {pixel_format} simples falhou: {error}", flush=True)

    camera_config = picam2.create_still_configuration({"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"})
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
        save_status(0.0, False, 0, active=False, error_message=error_message)
        return 1

    # Cria o arquivo de status inicial para evitar que o dashboard fique em
    # 'aguardando' indefinidamente enquanto a câmera está sendo preparada.
    save_status(0.0, False, 0)

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

        last_position = (FRAME_WIDTH // 2, FRAME_HEIGHT // 2)

        prev_time = time.monotonic()
        last_stream_frame_time = 0.0
        last_snapshot_save_time = 0.0
        last_status_save_time = 0.0
        stream_frame_interval = 1.0 / MJPEG_STREAM_FPS
        snapshot_save_interval = 1.0 / SNAPSHOT_FRAME_FPS
        status_save_interval = 1.0 / STATUS_FPS
        smoothed_fps = 0.0
        line_detected = False
        line_error = 0
        green_action = "NENHUM"

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
            frame, last_position, line_detected, line_error, green_action = process_frame(frame, last_position)

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
                save_status(smoothed_fps, line_detected, line_error, green_action, camera_format_label(camera_format))
                last_status_save_time = curr_time
    except Exception as error:
        error_message = f"Camera script failed: {error}"
        print(error_message, flush=True)
        save_status(0.0, False, 0, active=False, error_message=error_message)
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
