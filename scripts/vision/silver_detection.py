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

# Estes limites são iniciais e devem ser calibrados com o conjunto de teste real.
# Exigir confiança e vantagem sobre as outras classes reduz falsos positivos.
DEFAULT_SILVER_CONFIDENCE = 0.70
DEFAULT_SILVER_MARGIN = 0.15

# A busca economiza CPU; o primeiro positivo abre uma confirmação mais rápida.
# Quatro positivos consecutivos evitam que um reflexo isolado inicie o resgate.
SILVER_SEARCH_FPS = 6.0
SILVER_CONFIRMATION_FPS = 15.0
SILVER_CONFIRMATION_FRAMES = 4


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
        "silverShadowOnly": True,
        "silverShadowConfirmed": False,
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

    def __init__(self, detector=None, error_message="", publish_course_marker=True):
        self.detector = detector
        self.publish_course_marker = bool(publish_course_marker)
        self.status = empty_silver_shadow_status(error_message)
        self.last_detected = False
        self.last_marker_confirmed = False
        self.confirmation_frames = 0
        self.target_fps = SILVER_SEARCH_FPS
        self.last_inference_monotonic = None
        if detector is not None:
            self.status["silverInferenceTargetFps"] = self.target_fps

    @classmethod
    def from_camera_model(cls, camera_role="down", publish_course_marker=True):
        """Tenta carregar o modelo sem impedir a inicialização da câmera."""

        try:
            detector = SilverLineDetector.from_camera_model(camera_role)
        except Exception as error:
            print(
                f"Detector da faixa prata indisponível: {error}",
                flush=True,
            )
            return cls(
                error_message=error,
                publish_course_marker=publish_course_marker,
            )

        print(
            "Detector da faixa prata ativo: busca a 6 FPS e confirmação a 15 FPS.",
            flush=True,
        )
        mode = "controle ativo" if publish_course_marker else "somente shadow"
        print(f"Modo da faixa prata: {mode}.", flush=True)
        return cls(
            detector=detector,
            publish_course_marker=publish_course_marker,
        )

    def process(self, frame_bgr, sequence, timestamp, monotonic_time=None):
        """Executa a inferência na cadência atual e confirma quatro positivos."""

        if self.detector is None:
            return self.status

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
                SILVER_CONFIRMATION_FRAMES,
                self.confirmation_frames + 1,
            )
        else:
            self.confirmation_frames = 0

        shadow_confirmed = (
            self.confirmation_frames >= SILVER_CONFIRMATION_FRAMES
        )
        marker_confirmed = shadow_confirmed and self.publish_course_marker
        self.target_fps = (
            SILVER_CONFIRMATION_FPS
            if result.detected and not shadow_confirmed
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
            "silverConfirmationRequiredFrames": SILVER_CONFIRMATION_FRAMES,
            "silverInferenceTargetFps": self.target_fps,
            "silverShadowOnly": not self.publish_course_marker,
            "silverShadowConfirmed": shadow_confirmed,
            "courseMarkerConfirmed": marker_confirmed,
            "courseMarker": "GRAY" if marker_confirmed else "NONE",
        }

        if shadow_confirmed and not self.last_marker_confirmed:
            print(
                (
                    "Shadow confirmou quatro frames de prata; missão preservada."
                    if not self.publish_course_marker
                    else "Entrada na área de resgate confirmada por quatro frames de prata."
                ),
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
        self.last_marker_confirmed = shadow_confirmed
        return self.status
