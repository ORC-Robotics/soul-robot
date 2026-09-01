"""Calibra a distância da bola a partir do raio observado em 960×540."""

from bisect import bisect_right
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CalibrationPoint:
    """Associa um raio medido à distância real correspondente."""

    radius_pixels: float
    distance_cm: float


@dataclass(frozen=True)
class DistanceEstimate:
    """Informa a distância e se o raio ficou fora da faixa medida."""

    distance_cm: float
    extrapolated: bool


DEFAULT_CALIBRATION_POINTS = (
    CalibrationPoint(45.5, 60.0),
    CalibrationPoint(71.7, 40.0),
    CalibrationPoint(149.0, 20.0),
    CalibrationPoint(207.6, 15.0),
    CalibrationPoint(268.8, 5.0),
)


class DistanceCalibration:
    """Interpola medições reais e extrapola com uma relação inversa."""

    def __init__(self, points=DEFAULT_CALIBRATION_POINTS):
        self.points = tuple(sorted(points, key=lambda point: point.radius_pixels))
        self._validate_points()
        self._radii = tuple(point.radius_pixels for point in self.points)

    def _validate_points(self):
        if len(self.points) < 2:
            raise ValueError("A calibração exige pelo menos dois pontos.")
        previous_radius = 0.0
        previous_distance = math.inf
        for point in self.points:
            values = (point.radius_pixels, point.distance_cm)
            if not all(math.isfinite(value) and value > 0.0 for value in values):
                raise ValueError("Raio e distância de calibração devem ser positivos.")
            if point.radius_pixels <= previous_radius:
                raise ValueError("Os raios da calibração devem ser distintos.")
            if point.distance_cm >= previous_distance:
                raise ValueError(
                    "A distância deve diminuir quando o raio observado aumenta."
                )
            previous_radius = point.radius_pixels
            previous_distance = point.distance_cm

    def estimate(self, radius_pixels):
        """Estima centímetros sem permitir raio inválido ou distância negativa."""

        radius_pixels = float(radius_pixels)
        if not math.isfinite(radius_pixels) or radius_pixels <= 0.0:
            raise ValueError("O raio da bola deve ser finito e positivo.")

        first = self.points[0]
        last = self.points[-1]
        if radius_pixels < first.radius_pixels:
            distance_cm = (
                first.distance_cm * first.radius_pixels / radius_pixels
            )
            return DistanceEstimate(distance_cm, True)
        if radius_pixels > last.radius_pixels:
            distance_cm = last.distance_cm * last.radius_pixels / radius_pixels
            return DistanceEstimate(distance_cm, True)

        upper_index = bisect_right(self._radii, radius_pixels)
        if upper_index == 0:
            return DistanceEstimate(first.distance_cm, False)
        if upper_index >= len(self.points):
            return DistanceEstimate(last.distance_cm, False)

        lower = self.points[upper_index - 1]
        upper = self.points[upper_index]
        fraction = (
            (radius_pixels - lower.radius_pixels) /
            (upper.radius_pixels - lower.radius_pixels)
        )
        distance_cm = lower.distance_cm + fraction * (
            upper.distance_cm - lower.distance_cm
        )
        return DistanceEstimate(float(distance_cm), False)

