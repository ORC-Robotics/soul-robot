"""Executa o detector YOLO de vítimas com visualização local."""

import argparse
from dataclasses import dataclass
import json
import math
import signal
import time

import cv2  # type: ignore

try:
    from .ball_detector import BallCandidate
    from .camera import CameraConfig, FrontCamera
    from .ball_tracker import BallTracker
    from .distance_calibration import DistanceCalibration, DistanceEstimate
    from .yolo_detector import YoloBallDetector
except ImportError:
    from ball_detector import BallCandidate
    from camera import CameraConfig, FrontCamera
    from ball_tracker import BallTracker
    from distance_calibration import DistanceCalibration, DistanceEstimate
    from yolo_detector import YoloBallDetector


WINDOW_TITLE = "OBR - Detecção de bolas"
DEFAULT_HORIZONTAL_FOV_DEGREES = 62.0
DEFAULT_CENTER_ANGLE_DEGREES = 5.0
JSON_OUTPUT_FPS = 5.0


@dataclass(frozen=True)
class BallObservation:
    """Combina visão, calibração e direção do alvo principal."""

    candidate: BallCandidate
    distance: DistanceEstimate
    angle_degrees: float
    position: str


def calculate_horizontal_angle(
    center_x,
    frame_width,
    horizontal_fov_degrees=DEFAULT_HORIZONTAL_FOV_DEGREES,
):
    """Converte a posição horizontal em ângulo; direita tem sinal positivo."""

    values = (float(center_x), float(frame_width), float(horizontal_fov_degrees))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Centro, largura e FOV devem ser finitos.")
    if frame_width <= 0.0 or horizontal_fov_degrees <= 0.0:
        raise ValueError("A largura e o FOV horizontal devem ser positivos.")
    normalized_offset = (float(center_x) - frame_width * 0.5) / (
        frame_width * 0.5
    )
    return normalized_offset * horizontal_fov_degrees * 0.5


def classify_position(angle_degrees, center_angle_degrees=DEFAULT_CENTER_ANGLE_DEGREES):
    """Classifica a bola com uma zona central simétrica e configurável."""

    angle_degrees = float(angle_degrees)
    center_angle_degrees = float(center_angle_degrees)
    if not math.isfinite(angle_degrees) or not math.isfinite(center_angle_degrees):
        raise ValueError("Os ângulos devem ser finitos.")
    if center_angle_degrees < 0.0:
        raise ValueError("A zona central não pode ser negativa.")
    if angle_degrees < -center_angle_degrees:
        return "esquerda"
    if angle_degrees > center_angle_degrees:
        return "direita"
    return "centro"


def build_esp32_payload(ball_type, distance_cm, angle_degrees):
    """Monta somente tipos simples para uma futura serialização na UART."""

    if not ball_type:
        raise ValueError("O tipo da bola não pode ser vazio.")
    if not math.isfinite(distance_cm) or not math.isfinite(angle_degrees):
        raise ValueError("Distância e ângulo devem ser finitos.")
    return {
        "type": str(ball_type),
        "distance_cm": int(round(distance_cm)),
        "angle": int(round(angle_degrees)),
    }


def analyze_frame(
    frame,
    detector,
    calibration,
    horizontal_fov_degrees=DEFAULT_HORIZONTAL_FOV_DEGREES,
    center_angle_degrees=DEFAULT_CENTER_ANGLE_DEGREES,
    tracker=None,
    target_type=None,
):
    """Mede o alvo selecionado sem reutilizar observações de frames antigos."""

    detected_candidates = detector.detect(frame)
    if target_type in ("silver_ball", "black_ball"):
        # A missão escolhe qual classe pode entrar no tracker. Filtrar antes da
        # confirmação impede uma vítima preta de bloquear a busca prioritária.
        detected_candidates = [
            candidate
            for candidate in detected_candidates
            if candidate.ball_type == target_type
        ]
    candidates = detected_candidates
    if tracker is not None:
        candidates = tracker.update(detected_candidates, frame.shape[1])
    if not candidates:
        # Durante a aquisição ou perda do alvo, mantém os candidatos somente
        # para desenhá-los em laranja; nenhum deles é publicado como alvo.
        return None, detected_candidates
    candidate = candidates[0]
    distance = calibration.estimate(candidate.radius_pixels)
    angle_degrees = calculate_horizontal_angle(
        candidate.center_x,
        frame.shape[1],
        horizontal_fov_degrees,
    )
    position = classify_position(angle_degrees, center_angle_degrees)
    return BallObservation(candidate, distance, angle_degrees, position), candidates


def draw_overlay(frame, observation, candidates):
    """Desenha candidatos, alvo principal e valores usados na decisão."""

    display = frame.copy()

    def draw_candidate(candidate, color, thickness, show_label):
        """Desenha a caixa YOLO quando disponível, sem perder o overlay legado."""

        if candidate.bounding_box is not None:
            left, top, width, height = candidate.bounding_box
            start = (int(round(left)), int(round(top)))
            end = (
                int(round(left + width)),
                int(round(top + height)),
            )
            cv2.rectangle(display, start, end, color, thickness, cv2.LINE_AA)
            if show_label:
                label = (
                    ("PRATA" if candidate.ball_type == "silver_ball" else "PRETA") +
                    f" {candidate.circle_fill_ratio:.0%}"
                )
                cv2.putText(
                    display,
                    label,
                    (start[0], max(22, start[1] - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    color,
                    2,
                    cv2.LINE_AA,
                )
            return

        # Mantém o círculo para o detector legado e para diagnósticos antigos.
        center = (int(round(candidate.center_x)), int(round(candidate.center_y)))
        cv2.circle(
            display,
            center,
            int(round(candidate.radius_pixels)),
            color,
            thickness,
            cv2.LINE_AA,
        )

    secondary_candidates = candidates[1:] if observation is not None else candidates
    for candidate in secondary_candidates:
        draw_candidate(candidate, (0, 180, 255), 1, True)

    if observation is None:
        has_unconfirmed_candidate = bool(candidates)
        cv2.putText(
            display,
            (
                "CONFIRMANDO ALVO"
                if has_unconfirmed_candidate
                else "NENHUMA BOLA ENCONTRADA"
            ),
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 180, 255) if has_unconfirmed_candidate else (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        return display

    candidate = observation.candidate
    is_silver = candidate.ball_type == "silver_ball"
    ball_label = "BOLA PRATA" if is_silver else "BOLA PRETA"
    # Verde sempre identifica o alvo travado; todas as outras candidatas ficam
    # em laranja, independentemente de serem pretas ou pratas.
    target_color = (0, 255, 0)
    center = (int(round(candidate.center_x)), int(round(candidate.center_y)))
    draw_candidate(candidate, target_color, 2, True)
    cv2.circle(display, center, 4, (255, 0, 255), -1, cv2.LINE_AA)
    cv2.line(
        display,
        (display.shape[1] // 2, 0),
        (display.shape[1] // 2, display.shape[0]),
        (255, 120, 0),
        1,
        cv2.LINE_AA,
    )

    extrapolated_text = " (extrapolada)" if observation.distance.extrapolated else ""
    lines = (
        ball_label,
        f"Centro: ({candidate.center_x:.0f}, {candidate.center_y:.0f}) px",
        f"Raio: {candidate.radius_pixels:.1f} px | Diametro: {candidate.diameter_pixels:.1f} px",
        f"Area visivel: {candidate.visible_area_pixels:.0f} px",
        f"Distancia: {observation.distance.distance_cm:.1f} cm{extrapolated_text}",
        f"Angulo: {observation.angle_degrees:+.1f} graus",
        f"Posicao: {observation.position}",
    )
    for line_index, text in enumerate(lines):
        cv2.putText(
            display,
            text,
            (20, 35 + line_index * 27),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            target_color,
            2,
            cv2.LINE_AA,
        )
    return display


def parse_arguments(arguments=None):
    parser = argparse.ArgumentParser(
        description=(
            "Detecta bolas pretas e pratas usando exclusivamente a câmera frontal."
        )
    )
    parser.add_argument("--camera-index", type=int)
    parser.add_argument(
        "--horizontal-fov",
        type=float,
        default=DEFAULT_HORIZONTAL_FOV_DEGREES,
    )
    parser.add_argument(
        "--center-angle",
        type=float,
        default=DEFAULT_CENTER_ANGLE_DEGREES,
    )
    parser.add_argument("--maximum-value", type=int, default=100)
    parser.add_argument("--minimum-circularity", type=float, default=0.72)
    parser.add_argument("--print-json", action="store_true")
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args(arguments)


def run(arguments=None):
    args = parse_arguments(arguments)
    if args.camera_index is not None and args.camera_index < 0:
        raise ValueError("--camera-index deve ser não negativo.")

    camera_config = CameraConfig.from_environment()
    if args.camera_index is not None:
        camera_config = CameraConfig(
            camera_index=args.camera_index,
            downward_camera_index=camera_config.downward_camera_index,
        )
    # O utilitário local precisa mostrar exatamente o mesmo modelo do robô.
    # Os argumentos geométricos permanecem aceitos apenas por compatibilidade.
    detector = YoloBallDetector()
    calibration = DistanceCalibration()
    tracker = BallTracker(distance_estimator=calibration.estimate)
    camera = FrontCamera(camera_config)
    running = True

    def stop_running(signum, frame):
        nonlocal running
        del signum, frame
        running = False

    signal.signal(signal.SIGINT, stop_running)
    signal.signal(signal.SIGTERM, stop_running)
    last_json_time = 0.0

    try:
        camera.open()
        while running:
            frame = camera.capture_frame()
            observation, candidates = analyze_frame(
                frame,
                detector,
                calibration,
                args.horizontal_fov,
                args.center_angle,
                tracker,
            )
            if args.print_json and observation is not None:
                current_time = time.monotonic()
                if current_time - last_json_time >= 1.0 / JSON_OUTPUT_FPS:
                    payload = build_esp32_payload(
                        observation.candidate.ball_type,
                        observation.distance.distance_cm,
                        observation.angle_degrees,
                    )
                    print(
                        json.dumps(payload, separators=(",", ":")),
                        flush=True,
                    )
                    last_json_time = current_time

            if not args.headless:
                cv2.imshow(WINDOW_TITLE, frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    break
    finally:
        camera.close()
        if not args.headless:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
