"""Decisão de presença da faixa prata a partir do classificador TFLite."""

from dataclasses import dataclass
from pathlib import Path
import time

import numpy as np

from silver_classifier import SilverClassification, SilverClassifier


PROJECT_ROOT = Path(__file__).resolve().parents[2]
MODELS_DIR = PROJECT_ROOT / "assets" / "models"

CAMERA_ROLES = ("down", "forward")
FULL_FRAME_ROI = (0.0, 0.0, 1.0, 1.0)

# O limite de 80% permite iniciar mais cedo a verificação da faixa cinza.
# A margem e a confirmação temporal ainda rejeitam indícios fracos ou isolados.
DEFAULT_SILVER_CONFIDENCE = 0.80
DEFAULT_SILVER_MARGIN = 0.15

# A busca e a confirmação usam até 30 inferências por segundo. Essa cadência
# reduz a distância percorrida entre o primeiro indício e a confirmação da
# faixa; o laço ainda fica limitado pelo FPS real da câmera e pelo TFLite.
# Quatro positivos consecutivos evitam que um reflexo isolado inicie o resgate.
SILVER_SEARCH_FPS = 30.0
SILVER_CONFIRMATION_FPS = 30.0
SILVER_CONFIRMATION_FRAMES = 4
# Durante a saída, dois frames reduzem a distância percorrida sobre a entrada
# cinza. Fora desse modo, os quatro frames originais continuam preservados.
EXIT_SILVER_CONFIRMATION_FRAMES = 2


@dataclass(frozen=True)
class SilverLineDetection:
    """Descreve a decisão e as probabilidades obtidas no frame atual."""

    detected: bool
    label: str
    confidence: float
    silver_probability: float
    competing_probability: float
    silver_margin: float
    classification: SilverClassification


def empty_silver_shadow_status(error_message=""):
    """Cria a telemetria segura usada quando o shadow ainda não possui leitura."""

    return {
        "silverShadowAvailable": False,
        "silverShadowDetected": False,
        "silverShadowLabel": "",
        "silverShadowConfidence": 0.0,
        "silverShadowBlackProbability": 0.0,
        "silverShadowOtherProbability": 0.0,
        "silverShadowProbability": 0.0,
        "silverShadowMargin": 0.0,
        "silverShadowInferenceMs": 0.0,
        "silverShadowSequence": 0,
        "silverShadowTimestamp": 0.0,
        "silverShadowError": str(error_message),
        "silverConfirmationFrames": 0,
        "silverConfirmationRequiredFrames": SILVER_CONFIRMATION_FRAMES,
        "silverInferenceTargetFps": 0.0,
        "courseMarkerConfirmed": False,
        "courseMarker": "NONE",
    }


class SilverLineDetector:
    """Decide se o frame contém faixa prata sem controlar o robô."""

    def __init__(
        self,
        classifier,
        roi=FULL_FRAME_ROI,
        minimum_silver_confidence=DEFAULT_SILVER_CONFIDENCE,
        minimum_silver_margin=DEFAULT_SILVER_MARGIN,
    ):
        self.classifier = classifier
        self.roi = self._validate_roi(roi)
        self.minimum_silver_confidence = self._validate_probability(
            minimum_silver_confidence,
            "minimum_silver_confidence",
        )
        self.minimum_silver_margin = self._validate_probability(
            minimum_silver_margin,
            "minimum_silver_margin",
        )

    @classmethod
    def from_camera_model(
        cls,
        camera_role,
        roi=FULL_FRAME_ROI,
        minimum_silver_confidence=DEFAULT_SILVER_CONFIDENCE,
        minimum_silver_margin=DEFAULT_SILVER_MARGIN,
        num_threads=2,
        models_dir=MODELS_DIR,
    ):
        """Carrega o modelo produzido pelo treinamento para uma câmera."""

        if camera_role not in CAMERA_ROLES:
            raise ValueError(f"Câmera inválida: {camera_role}")

        model_path = Path(models_dir) / f"silver_{camera_role}.tflite"
        classifier = SilverClassifier(model_path, num_threads=num_threads)
        return cls(
            classifier,
            roi=roi,
            minimum_silver_confidence=minimum_silver_confidence,
            minimum_silver_margin=minimum_silver_margin,
        )

    def detect(self, frame_bgr):
        """Classifica a ROI e retorna uma decisão conservadora para o frame."""

        cropped_frame = self._crop_frame(frame_bgr)
        classification = self.classifier.classify(cropped_frame)
        competing_probability = max(classification.black, classification.other)
        silver_margin = classification.silver - competing_probability
        detected = (
            classification.label == "silver"
            and classification.silver >= self.minimum_silver_confidence
            and silver_margin >= self.minimum_silver_margin
        )

        return SilverLineDetection(
            detected=detected,
            label=classification.label,
            confidence=classification.confidence,
            silver_probability=classification.silver,
            competing_probability=competing_probability,
            silver_margin=silver_margin,
            classification=classification,
        )

    def _crop_frame(self, frame_bgr):
        if not isinstance(frame_bgr, np.ndarray):
            raise TypeError("frame_bgr deve ser um numpy.ndarray.")
        if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            raise ValueError(f"Frame inválido: shape={frame_bgr.shape}. Esperado HxWx3.")

        height, width = frame_bgr.shape[:2]
        left, top, right, bottom = self.roi
        x0 = int(np.floor(left * width + 0.5))
        x1 = int(np.floor(right * width + 0.5))
        y0 = int(np.floor(top * height + 0.5))
        y1 = int(np.floor(bottom * height + 0.5))
        cropped_frame = frame_bgr[y0:y1, x0:x1]

        if cropped_frame.size == 0:
            raise ValueError("A ROI da faixa prata resultou em uma imagem vazia.")

        return np.ascontiguousarray(cropped_frame)

    @staticmethod
    def _validate_roi(roi):
        try:
            left, top, right, bottom = (float(value) for value in roi)
        except (TypeError, ValueError):
            raise ValueError("ROI inválida; informe LEFT TOP RIGHT BOTTOM.") from None

        values = (left, top, right, bottom)
        if not all(np.isfinite(value) for value in values):
            raise ValueError("A ROI deve conter somente números finitos.")
        if not (0.0 <= left < right <= 1.0 and 0.0 <= top < bottom <= 1.0):
            raise ValueError("A ROI deve usar coordenadas normalizadas entre 0 e 1.")
        return values

    @staticmethod
    def _validate_probability(value, name):
        try:
            value = float(value)
        except (TypeError, ValueError):
            raise ValueError(f"{name} deve ser um número entre 0 e 1.") from None
        if not np.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError(f"{name} deve ser um número entre 0 e 1.")
        return value


class SilverShadowMonitor:
    """Confirma a faixa prata antes de sinalizar a entrada no resgate."""

    def __init__(self, detector=None, error_message=""):
        self.detector = detector
        self.status = empty_silver_shadow_status(error_message)
        self.last_detected = False
        self.last_marker_confirmed = False
        self.confirmation_frames = 0
        self.required_confirmation_frames = SILVER_CONFIRMATION_FRAMES
        self.target_fps = SILVER_SEARCH_FPS
        self.last_inference_monotonic = None
        if detector is not None:
            self.status["silverInferenceTargetFps"] = self.target_fps

    @classmethod
    def from_camera_model(cls, camera_role="down"):
        """Tenta carregar o modelo sem impedir a inicialização da câmera."""

        try:
            detector = SilverLineDetector.from_camera_model(camera_role)
        except Exception as error:
            print(
                f"Detector da faixa prata indisponível: {error}",
                flush=True,
            )
            return cls(error_message=error)

        print(
            "Detector da faixa prata ativo: busca e confirmação até 30 FPS.",
            flush=True,
        )
        return cls(detector=detector)

    def process(
        self,
        frame_bgr,
        sequence,
        timestamp,
        monotonic_time=None,
        required_confirmation_frames=SILVER_CONFIRMATION_FRAMES,
    ):
        """Executa a inferência e confirma a quantidade solicitada de positivos."""

        if self.detector is None:
            return self.status

        required_frames = max(1, int(required_confirmation_frames))
        if required_frames != self.required_confirmation_frames:
            # A troca de missão não pode reaproveitar positivos de outro modo.
            self.required_confirmation_frames = required_frames
            self.confirmation_frames = 0
            self.last_marker_confirmed = False

        current_monotonic = (
            time.monotonic()
            if monotonic_time is None
            else float(monotonic_time)
        )
        inference_interval = 1.0 / self.target_fps
        if (
            self.last_inference_monotonic is not None
            and current_monotonic - self.last_inference_monotonic
            < inference_interval
        ):
            return self.status
        self.last_inference_monotonic = current_monotonic

        try:
            result = self.detector.detect(frame_bgr)
        except Exception as error:
            print(
                f"Detector da faixa prata desativado após erro inesperado: {error}",
                flush=True,
            )
            self.detector = None
            self.status = empty_silver_shadow_status(error)
            return self.status

        if result.detected:
            self.confirmation_frames = min(
                required_frames,
                self.confirmation_frames + 1,
            )
        else:
            self.confirmation_frames = 0

        marker_confirmed = (
            self.confirmation_frames >= required_frames
        )
        self.target_fps = (
            SILVER_CONFIRMATION_FPS
            if result.detected and not marker_confirmed
            else SILVER_SEARCH_FPS
        )

        self.status = {
            "silverShadowAvailable": True,
            "silverShadowDetected": result.detected,
            "silverShadowLabel": result.label,
            "silverShadowConfidence": result.confidence,
            "silverShadowBlackProbability": result.classification.black,
            "silverShadowOtherProbability": result.classification.other,
            "silverShadowProbability": result.silver_probability,
            "silverShadowMargin": result.silver_margin,
            "silverShadowInferenceMs": result.classification.total_ms,
            "silverShadowSequence": int(sequence),
            "silverShadowTimestamp": float(timestamp),
            "silverShadowError": "",
            "silverConfirmationFrames": self.confirmation_frames,
            "silverConfirmationRequiredFrames": required_frames,
            "silverInferenceTargetFps": self.target_fps,
            "courseMarkerConfirmed": marker_confirmed,
            "courseMarker": "GRAY" if marker_confirmed else "NONE",
        }

        if marker_confirmed and not self.last_marker_confirmed:
            print(
                "Entrada na área de resgate confirmada por "
                f"{required_frames} frames de prata.",
                flush=True,
            )

        if result.detected != self.last_detected:
            state = "DETECTADA" if result.detected else "NÃO DETECTADA"
            print(
                f"Faixa prata: {state} "
                f"(silver={result.silver_probability:.3f}, "
                f"margem={result.silver_margin:.3f}).",
                flush=True,
            )
        self.last_detected = result.detected
        self.last_marker_confirmed = marker_confirmed
        return self.status
