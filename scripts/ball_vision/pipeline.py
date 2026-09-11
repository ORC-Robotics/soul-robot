"""Interface pública e reutilizável da detecção de bolas."""

from dataclasses import dataclass

from .ball_tracker import BallTracker
from .distance_calibration import DistanceCalibration
from .main import BallObservation, analyze_frame, draw_overlay
from .yolo_detector import YoloBallDetector


@dataclass(frozen=True)
class BallVisionResult:
    """Agrupa a bola principal e todos os candidatos do frame atual."""

    observation: BallObservation | None
    candidates: tuple


class BallVisionPipeline:
    """Detecta, seleciona e mede bolas sem depender do dashboard."""

    def __init__(self, detector=None, calibration=None, tracker=None):
        # A missão de resgate usa exclusivamente o modelo treinado. Se o arquivo
        # estiver ausente ou inválido, a câmera falha de forma segura e não
        # substitui o YOLO por um detector geométrico diferente.
        self.detector = detector or YoloBallDetector()
        self.calibration = calibration or DistanceCalibration()
        self.tracker = tracker or BallTracker(
            distance_estimator=self.calibration.estimate
        )

    @property
    def silver_processing_scale(self):
        """Expõe a escala da prata para diagnóstico de desempenho."""

        return self.detector.processing_scale

    @property
    def target_locked(self):
        """Informa se o tracker já confirmou o alvo desta execução."""

        return self.tracker.locked

    def reset(self):
        """Descarta o alvo temporal ao iniciar ou encerrar uma execução."""

        self.tracker.reset()

    def analyze(self, frame):
        """Retorna somente o alvo travado e os demais candidatos detectados."""

        tracked_observation, candidates = analyze_frame(
            frame,
            self.detector,
            self.calibration,
            tracker=self.tracker,
        )
        if not candidates:
            return BallVisionResult(None, ())

        return BallVisionResult(tracked_observation, tuple(candidates))

    @staticmethod
    def draw(frame, result):
        """Desenha o resultado sem modificar o frame original."""

        return draw_overlay(frame, result.observation, result.candidates)
