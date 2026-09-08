"""Decisão de presença da faixa prata a partir do classificador TFLite."""

from dataclasses import dataclass
from pathlib import Path

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
