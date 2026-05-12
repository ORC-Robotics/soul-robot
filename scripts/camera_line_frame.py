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
    from picamera2 import Picamera2 # type: ignore[import]
except ImportError:
    Picamera2 = None

# --- CONFIGURAÇÕES ---
FRAME_WIDTH, FRAME_HEIGHT = 960, 540
SETPOINT_X = FRAME_WIDTH // 2
PROCESS_WIDTH, PROCESS_HEIGHT = 320, 180
ROI_TOP_RATIO = 0.20 
ROI_TOP_Y = int(FRAME_HEIGHT * ROI_TOP_RATIO)
PROCESS_ROI_TOP_Y = int(PROCESS_HEIGHT * ROI_TOP_RATIO)

BLACK_THRESHOLD = 80 
GREEN_LOWER = np.array([35, 50, 40])
GREEN_UPPER = np.array([90, 255, 255])
MIN_GREEN_AREA = 50 

STATUS_PATH = "/tmp/obr_camera_status.json"
TEMP_STATUS_PATH = "/tmp/obr_camera_status.tmp.json"
MJPEG_STREAM_PORT = 8090

last_cx_small = PROCESS_WIDTH // 2 

running = True
latest_jpeg = None
latest_jpeg_sequence = 0
frame_condition = threading.Condition()

def handle_signal(signum, frame):
    global running
    running = False
    with frame_condition: frame_condition.notify_all()

class CameraStreamHandler(BaseHTTPRequestHandler):
    def log_message(self, format_text, *args): return
    def do_GET(self):
        if self.path != "/stream.mjpg":
            self.send_response(404); self.end_headers(); return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        last_sequence = -1
        try:
            while running:
                with frame_condition:
                    frame_condition.wait_for(lambda: latest_jpeg_sequence != last_sequence or not running, timeout=1.0)
                    jpeg = latest_jpeg
                    last_sequence = latest_jpeg_sequence
                if jpeg:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(jpeg + b"\r\n")
        except: return

def process_frame(frame):
    global last_cx_small
    
    # 1. Visualização de Referência
    cv2.line(frame, (0, ROI_TOP_Y), (FRAME_WIDTH, ROI_TOP_Y), (255, 0, 255), 2)
    cv2.line(frame, (SETPOINT_X, ROI_TOP_Y), (SETPOINT_X, FRAME_HEIGHT), (0, 255, 255), 1)

    small = cv2.resize(frame, (PROCESS_WIDTH, PROCESS_HEIGHT), interpolation=cv2.INTER_AREA)
    roi_bgr = small[PROCESS_ROI_TOP_Y:, :]
    hsv_roi = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    
    # 2. Máscaras
    blurred_v = cv2.GaussianBlur(hsv_roi[:,:,2], (5, 5), 0)
    _, mask_black = cv2.threshold(blurred_v, BLACK_THRESHOLD, 255, cv2.THRESH_BINARY_INV)
    mask_green = cv2.inRange(hsv_roi, GREEN_LOWER, GREEN_UPPER)

    kernel = np.ones((3, 3), np.uint8)
    mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)

    green_action = "NENHUM"
    error = 0
    line_detected = False
    
    # 3. DETECÇÃO E CONTORNO DA LINHA PRETA
    contours_black, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_contour = None
    min_dist = float('inf')

    for cnt in contours_black:
        if cv2.contourArea(cnt) > 60:
            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + (w // 2)
            dist = abs(cx - last_cx_small)
            if dist < min_dist:
                min_dist = dist
                best_contour = cnt

    if best_contour is not None:
        # --- NOVO: Desenhar o contorno de toda a linha reconhecida ---
        # Ajustamos as coordenadas do contorno pequeno para o frame original (960x540)
        poly_line = best_contour.astype(np.float32)
        poly_line[:, :, 0] *= (FRAME_WIDTH / PROCESS_WIDTH)
        poly_line[:, :, 1] = (poly_line[:, :, 1] + PROCESS_ROI_TOP_Y) * (FRAME_HEIGHT / PROCESS_HEIGHT)
        
        # Desenha o contorno azul em volta de toda a linha
        cv2.drawContours(frame, [poly_line.astype(np.int32)], -1, (255, 0, 0), 2)

        # Calcula Centro e Erro
        x, y, w, h = cv2.boundingRect(best_contour)
        last_cx_small = x + (w // 2)
        center_x = int(last_cx_small * FRAME_WIDTH / PROCESS_WIDTH)
        error = center_x - SETPOINT_X
        line_detected = True
        
        # Linha vertical de centro (azul mais grossa)
        cv2.line(frame, (center_x, ROI_TOP_Y), (center_x, FRAME_HEIGHT), (255, 0, 0), 3)

    # 4. DETECÇÃO DOS VERDES
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    mid_x = PROCESS_WIDTH // 2
    raw_l, raw_r, val_l, val_r = False, False, False, False

    for c in contours_green:
        if cv2.contourArea(c) >= MIN_GREEN_AREA:
            M = cv2.moments(c)
            if M["m00"] > 0:
                gx, gy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                if gx < mid_x: raw_l = True
                else: raw_r = True

                y_start = max(0, gy - 35)
                x_start, x_end = max(0, gx - 10), min(PROCESS_WIDTH, gx + 10)
                check_zone = mask_black[y_start:gy, x_start:x_end]
                
                if np.any(check_zone == 255):
                    if gx < mid_x: val_l = True
                    else: val_r = True
                    color = (0, 255, 0)
                else: color = (100, 100, 100)

                poly = (c.astype(np.float32))
                poly[:,:,0] *= (FRAME_WIDTH / PROCESS_WIDTH)
                poly[:,:,1] = (poly[:,:,1] + PROCESS_ROI_TOP_Y) * (FRAME_HEIGHT / PROCESS_HEIGHT)
                cv2.drawContours(frame, [poly.astype(np.int32)], -1, color, 3)

    if raw_l and raw_r: green_action = "MEIA VOLTA"
    elif val_l: green_action = "ESQUERDA"
    elif val_r: green_action = "DIREITA"

    cv2.putText(frame, f"ACAO: {green_action}", (10, 140), cv2.FONT_HERSHEY_DUPLEX, 0.9, (255, 255, 255), 2)
    cv2.putText(frame, f"Erro: {error}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return frame, line_detected, error, green_action

def main():
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    server = ThreadingHTTPServer(("127.0.0.1", MJPEG_STREAM_PORT), CameraStreamHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    
    if Picamera2 is None:
        return 1

    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}))
    picam2.start()
    previous_time = time.monotonic()
    
    try:
        while running:
            frame = picam2.capture_array()
            frame, det, err, act = process_frame(frame)
            current_time = time.monotonic()
            elapsed = current_time - previous_time
            previous_time = current_time
            fps = 1.0 / elapsed if elapsed > 0 else 0.0

            ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                global latest_jpeg, latest_jpeg_sequence
                with frame_condition:
                    latest_jpeg = jpeg.tobytes()
                    latest_jpeg_sequence += 1
                    frame_condition.notify_all()
            status = {
                "fps": round(fps, 2),
                "active": True,
                "lineDetected": det,
                "lineError": err,
                "greenAction": act,
                "width": FRAME_WIDTH,
                "height": FRAME_HEIGHT,
                "cameraFormat": "RGB888",
                "timestamp": time.time(),
            }
            with open(TEMP_STATUS_PATH, "w") as f: json.dump(status, f)
            os.replace(TEMP_STATUS_PATH, STATUS_PATH)
    finally:
        status = {"fps": 0.0, "active": False, "lineDetected": False, "lineError": 0, "greenAction": "NENHUM", "timestamp": time.time()}
        with open(TEMP_STATUS_PATH, "w") as f: json.dump(status, f)
        os.replace(TEMP_STATUS_PATH, STATUS_PATH)
        picam2.stop()

if __name__ == "__main__":
    main()
