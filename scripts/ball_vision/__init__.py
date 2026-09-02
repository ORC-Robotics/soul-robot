"""Detecção leve de bolas com interface reutilizável."""

from .main import BallObservation, build_esp32_payload
from .pipeline import BallVisionPipeline, BallVisionResult

__all__ = (
    "BallObservation",
    "BallVisionPipeline",
    "BallVisionResult",
    "build_esp32_payload",
)
