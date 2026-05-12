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

# --- CONFIGURAÇÕES DO SISTEMA ---
FRAME_PATH = "/tmp/obr_camera_frame.jpg"
TEMP_FRAME_PATH = "/tmp/obr_camera_frame.tmp.jpg"
STATUS_PATH = "/tmp/obr_camera_status.json"
TEMP_STATUS_PATH = "/tmp/obr_camera_status.tmp.json"
LIGHT_PIN_BOARD = 40

FRAME_WIDTH = 960
FRAME_HEIGHT = 540
SETPOINT_X = FRAME_WIDTH // 2
TARGET_CAMERA_FPS = 30
MJPEG_STREAM_PORT = 8090
MJPEG_STREAM_PATH = "/stream.mjpg"
MJPEG_STREAM_FPS = 30
SNAPSHOT_FRAME_FPS = 2
STATUS_FPS = 10
JPEG_QUALITY = 82
BLACK_THRESHOLD = 75

PROCESS_WIDTH = 320
PROCESS_HEIGHT = 180
ROI_TOP_RATIO = 0.35
ROI_TOP_Y = int(FRAME_HEIGHT * ROI_TOP_RATIO)
PROCESS_ROI_TOP_Y = int(PROCESS_HEIGHT * ROI_TOP_RATIO)
MIN_LINE_AREA = 80

# --- AJUSTES DE IMAGEM ---
CAMERA_SHARPNESS = 1.2
CAMERA_CONTRAST = 1.05
CAMERA_SATURATION = 1.0
CAMERA_EXPOSURE_VALUE = 0.4
CAMERA_PIXEL_FORMATS = ("RGB888",)

# --- DETECÇÃO DE VERDE ---
GREEN_LOWER = np.array([35, 50, 50])
GREEN_UPPER = np.array([85, 255, 255])
MIN_GREEN_AREA = 100

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

def start_stream_server():
    try:
        server = ReusableThreadingHTTPServer(("127.0.0.1", MJPEG_STREAM_PORT), CameraStreamHandler)
        server.daemon_threads = True
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        print(f"Stream MJPEG em http://127.0.0.1:{MJPEG_STREAM_PORT}{MJPEG_STREAM_PATH}")
        return server
    except OSError as error:
        print(f"Erro ao iniciar servidor: {error}")
        return None

def process_frame(frame, last_position):
    # Resize para performance
    small = cv2.resize(frame, (PROCESS_WIDTH, PROCESS_HEIGHT), interpolation=cv2.INTER_AREA)
    roi = small[PROCESS_ROI_TOP_Y:PROCESS_HEIGHT, :]
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # Máscaras
    _, mask_black = cv2.threshold(hsv_roi[:,:,2], BLACK_THRESHOLD, 255, cv2.THRESH_BINARY_INV)
    mask_green = cv2.inRange(hsv_roi, GREEN_LOWER, GREEN_UPPER)

    kernel = np.ones((3, 3), np.uint8)
    mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_OPEN, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)

    line_detected = False
    error = 0
    green_action = "NENHUM"
    
    # --- LINHA CENTRAL DE REFERÊNCIA (AMARELA) ---
    # Desenha uma linha vertical no centro exato da tela
    cv2.line(frame, (SETPOINT_X, ROI_TOP_Y), (SETPOINT_X, FRAME_HEIGHT), (0, 255, 255), 1)

    # 1. LINHA PRETA
    contours_black, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours_black:
        c_black = max(contours_black, key=cv2.contourArea)
        if cv2.contourArea(c_black) >= MIN_LINE_AREA:
            M = cv2.moments(c_black)
            if M["m00"] > 0:
                cx_small = int(M["m10"] / M["m00"])
                center_x = int(cx_small * FRAME_WIDTH / PROCESS_WIDTH)
                error = center_x - SETPOINT_X
                line_detected = True
                
                # Linha AZUL indicando a posição atual da linha preta
                cv2.line(frame, (center_x, ROI_TOP_Y), (center_x, FRAME_HEIGHT), (255, 0, 0), 2)

    # 2. VERDE (Lógica de Lados)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_left = False
    green_right = False
    mid_x = PROCESS_WIDTH // 2

    for c_green in contours_green:
        if cv2.contourArea(c_green) >= MIN_GREEN_AREA:
            M = cv2.moments(c_green)
            if M["m00"] > 0:
                gx = int(M["m10"] / M["m00"])
                if gx < mid_x:
                    green_left = True
                else:
                    green_right = True
                
                # Desenho técnico do contorno verde
                poly_green = c_green.astype(np.float32)
                poly_green[:, :, 0] *= FRAME_WIDTH / PROCESS_WIDTH
                poly_green[:, :, 1] = (poly_green[:, :, 1] + PROCESS_ROI_TOP_Y) * (FRAME_HEIGHT / PROCESS_HEIGHT)
                cv2.drawContours(frame, [poly_green.astype(np.int32)], -1, (0, 255, 0), 3)

    # Decisão da Ação
    if green_left and green_right:
        green_action = "MEIA VOLTA"
    elif green_left:
        green_action = "ESQUERDA"
    elif green_right:
        green_action = "DIREITA"

    # Overlay Dashboard
    cv2.putText(frame, f"ACAO: {green_action}", (8, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(frame, f"Erro: {error}", (8, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    return frame, last_position, line_detected, error, green_action

def encode_frame(frame):
    ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
    return encoded.tobytes() if ok else None

def save_status(fps, line_detected, line_error, green_action, camera_format="", active=True, error_message=""):
    status = {
        "fps": round(fps, 2),
        "active": active,
        "lineDetected": line_detected,
        "lineError": line_error,
        "greenAction": green_action,
        "timestamp": time.time(),
    }
    with open(TEMP_STATUS_PATH, "w", encoding="utf-8") as f:
        json.dump(status, f)
    os.replace(TEMP_STATUS_PATH, STATUS_PATH)

def create_camera():
    picam2 = Picamera2()
    f_dur = int(1_000_000 / TARGET_CAMERA_FPS)
    config = picam2.create_video_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"},
        controls={"FrameDurationLimits": (f_dur, f_dur)},
        buffer_count=4,
    )
    picam2.configure(config)
    return picam2, "RGB888"

def main():
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    if GPIO is None or Picamera2 is None:
        return 1

    save_status(0.0, False, 0, "NENHUM")
    
    try:
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(LIGHT_PIN_BOARD, GPIO.OUT)
        GPIO.output(LIGHT_PIN_BOARD, GPIO.HIGH)
        
        server = start_stream_server()
        picam2, camera_format = create_camera()
        picam2.start()
        
        # Ajustes finos
        picam2.set_controls({"Sharpness": CAMERA_SHARPNESS, "Contrast": CAMERA_CONTRAST})

        last_pos = (FRAME_WIDTH // 2, FRAME_HEIGHT // 2)
        prev_time = time.monotonic()
        last_stream_t = 0
        last_status_t = 0

        while running:
            frame = picam2.capture_array()
            curr_time = time.monotonic()
            
            fps = 1.0 / (curr_time - prev_time) if (curr_time - prev_time) > 0 else 0
            prev_time = curr_time

            frame, last_pos, detected, error, action = process_frame(frame, last_pos)

            if curr_time - last_stream_t >= (1.0 / MJPEG_STREAM_FPS):
                jpeg = encode_frame(frame)
                if jpeg:
                    global latest_jpeg, latest_jpeg_sequence
                    with frame_condition:
                        latest_jpeg = jpeg
                        latest_jpeg_sequence += 1
                        frame_condition.notify_all()
                last_stream_t = curr_time

            if curr_time - last_status_t >= (1.0 / STATUS_FPS):
                save_status(fps, detected, error, action, camera_format)
                last_status_t = curr_time

    finally:
        if 'server' in locals() and server: server.shutdown()
        if GPIO: GPIO.cleanup()
        if 'picam2' in locals(): picam2.stop()

if __name__ == "__main__":
    main()