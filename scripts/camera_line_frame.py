import json
import os
import signal
import time
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

# Constantes permanecem as mesmas
FRAME_PATH = "/tmp/obr_camera_frame.jpg"
TEMP_FRAME_PATH = "/tmp/obr_camera_frame.tmp.jpg"
STATUS_PATH = "/tmp/obr_camera_status.json"
TEMP_STATUS_PATH = "/tmp/obr_camera_status.tmp.json"
LIGHT_PIN_BOARD = 40
FRAME_WIDTH = 320  # Resolução reduzida para aumentar o FPS
FRAME_HEIGHT = 240  # Resolução reduzida para aumentar o FPS
SETPOINT_X = FRAME_WIDTH // 2

running = True

def handle_signal(signum, frame):
    global running
    running = False

def process_frame(frame, last_position):
    # Detecta a linha preta
    blackline = cv2.inRange(frame, (0, 0, 0), (75, 75, 75))
    kernel = np.ones((3, 3), np.uint8)  # Tamanho menor do kernel
    blackline = cv2.erode(blackline, kernel, iterations=1)  # Menos iterações de erosão
    blackline = cv2.dilate(blackline, kernel, iterations=1)  # Menos iterações de dilatação

    # Usando a Transformada de Hough para detectar as linhas
    edges = cv2.Canny(blackline, 100, 200)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=50, maxLineGap=10)
    
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # A posição da linha detectada pode ser calculada
        x_min = (x1 + x2) // 2
        y_min = (y1 + y2) // 2
        last_position = (x_min, y_min)

        # Calcular erro
        error = int(x_min - SETPOINT_X)

        # Exibe os valores no frame
        cv2.putText(frame, f"FPS: {getattr(process_frame, 'current_fps', 0):.1f}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, str(error), (10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.line(frame, (int(x_min), 200), (int(x_min), 250), (255, 0, 0), 3)

    return frame, last_position

def save_frame(frame):
    if cv2.imwrite(TEMP_FRAME_PATH, frame):
        os.replace(TEMP_FRAME_PATH, FRAME_PATH)

def save_status(fps):
    status = {
        "fps": round(fps, 2),
        "active": True,
        "timestamp": time.time(),
    }
    with open(TEMP_STATUS_PATH, "w", encoding="utf-8") as status_file:
        json.dump(status, status_file)
    os.replace(TEMP_STATUS_PATH, STATUS_PATH)

def main():
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    if GPIO is None or Picamera2 is None:
        print("Dependências (GPIO ou Picamera2) não encontradas.")
        return 1

    # Cria o arquivo de status inicial para evitar que o dashboard fique em
    # 'aguardando' indefinidamente enquanto a câmera está sendo preparada.
    save_status(0.0)

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(LIGHT_PIN_BOARD, GPIO.OUT)
    GPIO.output(LIGHT_PIN_BOARD, GPIO.HIGH)

    picam2 = Picamera2()
    picam2.configure(picam2.create_still_configuration({"size": (FRAME_WIDTH, FRAME_HEIGHT)}))
    picam2.start()

    last_position = (FRAME_WIDTH // 2, FRAME_HEIGHT // 2)
    
    # Inicialização do tempo
    prev_time = time.monotonic()
    smoothed_fps = 0.0

    try:
        while running:
            # Captura e Processamento
            frame = picam2.capture_array()
            
            # Cálculo de FPS (Calculado antes do sleep para precisão do processamento)
            curr_time = time.monotonic()
            elapsed = curr_time - prev_time
            prev_time = curr_time
            
            if elapsed > 0:
                actual_fps = 1.0 / elapsed
                # Filtro passa-baixa para suavizar a leitura (agora com suavização leve)
                smoothed_fps = (smoothed_fps * 0.95) + (actual_fps * 0.05) if smoothed_fps > 0 else actual_fps
            
            # Atribui o fps ao objeto da função para mostrar no frame
            process_frame.current_fps = smoothed_fps
            
            frame, last_position = process_frame(frame, last_position)
            save_frame(frame)
            save_status(smoothed_fps)

            # Sem sleep para maximizar o FPS
            # time.sleep(0.001) 
            
    finally:
        GPIO.output(LIGHT_PIN_BOARD, GPIO.LOW)
        GPIO.cleanup()
        picam2.stop()

    return 0

if __name__ == "__main__":
    raise SystemExit(main())