"""Estabiliza a geometria da bola entre frames consecutivos."""

from collections import deque
from dataclasses import dataclass, replace
import math
import statistics


@dataclass(frozen=True)
class BallTrackerConfig:
    """Centraliza os limites temporais do rastreamento da bola."""

    acquisition_frames: int = 3
    history_size: int = 5
    smoothing_alpha: float = 0.30
    maximum_center_jump_radii: float = 1.75
    minimum_center_jump_pixels: float = 35.0

    def validate(self):
        if self.acquisition_frames <= 0:
            raise ValueError("A aquisição deve exigir ao menos um frame.")
        if self.history_size <= 0:
            raise ValueError("O histórico do rastreador deve ser positivo.")
        if not 0.0 < self.smoothing_alpha <= 1.0:
            raise ValueError("smoothing_alpha deve estar entre 0 e 1.")
        if self.maximum_center_jump_radii <= 0.0:
            raise ValueError("O salto máximo do centro deve ser positivo.")
        if self.minimum_center_jump_pixels <= 0.0:
            raise ValueError("O salto mínimo em pixels deve ser positivo.")


class BallTracker:
    """Associa o mesmo alvo e reduz oscilações de centro e raio."""

    def __init__(self, config=None):
        self.config = config or BallTrackerConfig()
        self.config.validate()
        self.reset()

    def reset(self):
        """Descarta o alvo somente quando uma nova execução é iniciada."""

        self._measurements = deque(maxlen=self.config.history_size)
        self._ball_type = None
        self._filtered_geometry = None
        self._missing_frames = 0
        self._acquisition_frames = 0
        self._acquisition_candidate = None
        self._last_raw_center = None
        self._velocity = (0.0, 0.0)
        self._locked = False

    @property
    def locked(self):
        """Informa se a aquisição já travou um alvo para a execução atual."""

        return self._locked

    def _association_cost(self, candidate):
        previous_x, previous_y, previous_radius = self._filtered_geometry
        if self._last_raw_center is None:
            expected_x, expected_y = previous_x, previous_y
        else:
            expected_x = self._last_raw_center[0] + self._velocity[0]
            expected_y = self._last_raw_center[1] + self._velocity[1]
        center_distance = math.hypot(
            candidate.center_x - expected_x,
            candidate.center_y - expected_y,
        )
        center_limit = max(
            self.config.minimum_center_jump_pixels,
            previous_radius * self.config.maximum_center_jump_radii,
        )
        if center_distance > center_limit:
            return math.inf
        # Durante o giro, a perspectiva altera o raio aparente e pode inverter
        # a ordem de tamanho das bolas. A identidade depende apenas da posição
        # entre frames; o raio continua sendo filtrado somente para a distância.
        return center_distance / center_limit

    def _select_candidate(self, candidates):
        if self._filtered_geometry is None:
            return candidates[0]
        compatible = [
            candidate
            for candidate in candidates
            if candidate.ball_type == self._ball_type
        ]
        if not compatible:
            return None
        selected = min(compatible, key=self._association_cost)
        if not math.isfinite(self._association_cost(selected)):
            return None
        return selected

    def _robust_geometry(self):
        """Calcula uma geometria robusta, inclusive quando o topo está cortado."""

        measurements = list(self._measurements)
        top_clipped_silver = self._ball_type == "silver_ball" and any(
            measurement[3] for measurement in measurements
        )
        if not top_clipped_silver:
            return tuple(
                statistics.median(values)
                for values in zip(*(measurement[:3] for measurement in measurements))
            )

        # Quando o topo não aparece, o Hough pode encaixar um arco interno e
        # subestimar o raio. O quartil superior conserva as medições do contorno
        # externo; a borda inferior continua visível e ancora o centro vertical.
        sorted_radii = sorted(measurement[2] for measurement in measurements)
        upper_index = int(math.ceil(0.75 * (len(sorted_radii) - 1)))
        radius_threshold = sorted_radii[upper_index]
        outer_measurements = [
            measurement
            for measurement in measurements
            if measurement[2] >= radius_threshold
        ]
        center_x = statistics.median(
            measurement[0] for measurement in outer_measurements
        )
        radius = statistics.median(
            measurement[2] for measurement in outer_measurements
        )
        bottom_y = statistics.median(
            measurement[1] + measurement[2]
            for measurement in outer_measurements
        )
        return center_x, bottom_y - radius, radius

    def update(self, candidates):
        """Retorna o alvo atual suavizado sem publicar frames antigos ausentes."""

        if not candidates:
            self._acquisition_frames = 0
            self._missing_frames += 1
            return []

        if not self._locked:
            # Antes de liberar qualquer movimento, aguarda alguns frames para
            # confirmar que a mesma vencedora por área continua visível.
            selected = max(
                candidates,
                key=lambda candidate: candidate.visible_area_pixels,
            )
            previous = self._acquisition_candidate
            same_candidate = previous is not None and (
                previous.ball_type == selected.ball_type and
                math.hypot(
                    previous.center_x - selected.center_x,
                    previous.center_y - selected.center_y,
                ) <= max(
                    self.config.minimum_center_jump_pixels,
                    previous.radius_pixels *
                    self.config.maximum_center_jump_radii,
                )
            )
            self._acquisition_frames = (
                self._acquisition_frames + 1 if same_candidate else 1
            )
            self._acquisition_candidate = selected
            if self._acquisition_frames < self.config.acquisition_frames:
                return []
            self._locked = True
        else:
            selected = self._select_candidate(candidates)
        if selected is None:
            # Um alvo incompatível nunca assume a execução atual. A ausência é
            # publicada para o C++ parar os motores e controlar o timeout.
            self._missing_frames += 1
            return []

        self._missing_frames = 0
        self._acquisition_frames = 0
        self._ball_type = selected.ball_type
        if self._last_raw_center is not None:
            measured_velocity = (
                selected.center_x - self._last_raw_center[0],
                selected.center_y - self._last_raw_center[1],
            )
            self._velocity = tuple(
                previous * 0.5 + measured * 0.5
                for previous, measured in zip(
                    self._velocity,
                    measured_velocity,
                )
            )
        self._last_raw_center = (selected.center_x, selected.center_y)
        self._measurements.append((
            selected.center_x,
            selected.center_y,
            selected.radius_pixels,
            selected.top_clipped,
        ))
        median_geometry = self._robust_geometry()
        if self._filtered_geometry is None:
            self._filtered_geometry = median_geometry
        else:
            alpha = self.config.smoothing_alpha
            self._filtered_geometry = tuple(
                previous + alpha * (median - previous)
                for previous, median in zip(
                    self._filtered_geometry,
                    median_geometry,
                )
            )

        center_x, center_y, radius = self._filtered_geometry
        smoothed = replace(
            selected,
            center_x=float(center_x),
            center_y=float(center_y),
            radius_pixels=float(radius),
            diameter_pixels=float(radius * 2.0),
        )
        remaining = [candidate for candidate in candidates if candidate is not selected]
        return [smoothed] + remaining
