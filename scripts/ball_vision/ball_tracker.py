"""Estabiliza a geometria da bola entre frames consecutivos."""

from collections import deque
from dataclasses import dataclass, replace
from enum import Enum
import logging
import math
import statistics
import time


LOGGER = logging.getLogger(__name__)


class TargetState(Enum):
    """Representa o estado atual do gerenciamento temporal do alvo."""

    SEARCHING = "SEARCHING"
    TRACKING = "TRACKING"
    REACQUIRE = "REACQUIRE"


@dataclass(frozen=True)
class LockedTarget:
    """Registra todos os dados usados para manter o alvo atual travado."""

    center_x: float
    center_y: float
    radius_pixels: float
    distance_cm: float | None
    color: str
    timestamp: float
    confidence: float
    score: float


@dataclass(frozen=True)
class BallTrackerConfig:
    """Centraliza os limites temporais do rastreamento da bola."""

    acquisition_frames: int = 2
    history_size: int = 5
    smoothing_alpha: float = 0.30
    maximum_center_jump_radii: float = 1.75
    minimum_center_jump_pixels: float = 35.0
    switch_confirmation_frames: int = 4
    reacquire_after_missing_frames: int = 1
    reacquire_confirmation_frames: int = 2
    high_confidence_reacquire_threshold: float = 0.85
    confirmation_missing_frame_tolerance: int = 1
    search_after_missing_frames: int = 15
    minimum_confidence_improvement: float = 0.05
    minimum_distance_improvement_ratio: float = 0.20
    minimum_position_improvement_pixels: float = 50.0
    maximum_switch_distance_change_ratio: float = 0.50
    maximum_switch_jump_pixels: float = 320.0
    maximum_switch_jump_radii: float = 4.0

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
        frame_limits = (
            self.switch_confirmation_frames,
            self.reacquire_after_missing_frames,
            self.reacquire_confirmation_frames,
            self.search_after_missing_frames,
        )
        if any(value <= 0 for value in frame_limits):
            raise ValueError("Os limites temporais do alvo devem ser positivos.")
        if self.confirmation_missing_frame_tolerance < 0:
            raise ValueError(
                "A tolerância de frames ausentes não pode ser negativa."
            )
        if not 0.0 < self.high_confidence_reacquire_threshold <= 1.0:
            raise ValueError(
                "O limite de confiança da reaquisição deve estar entre 0 e 1."
            )
        if self.search_after_missing_frames <= self.reacquire_after_missing_frames:
            raise ValueError("A busca deve começar depois da tentativa de reaquisição.")
        if self.minimum_confidence_improvement < 0.0:
            raise ValueError("A melhoria mínima de confiança não pode ser negativa.")
        if self.minimum_distance_improvement_ratio < 0.0:
            raise ValueError("A melhoria mínima de distância não pode ser negativa.")
        if self.minimum_position_improvement_pixels < 0.0:
            raise ValueError("A melhoria mínima de posição não pode ser negativa.")
        if self.maximum_switch_distance_change_ratio < 0.0:
            raise ValueError("A variação máxima de distância não pode ser negativa.")
        if self.maximum_switch_jump_pixels <= 0.0:
            raise ValueError("O salto máximo de troca deve ser positivo.")
        if self.maximum_switch_jump_radii <= 0.0:
            raise ValueError("O salto máximo em raios deve ser positivo.")


class BallTracker:
    """Associa o mesmo alvo e reduz oscilações de centro e raio."""

    def __init__(self, config=None, distance_estimator=None):
        self.config = config or BallTrackerConfig()
        self.config.validate()
        self._distance_estimator = distance_estimator
        self.reset()

    def reset(self):
        """Descarta o alvo somente quando uma nova execução é iniciada."""

        self._measurements = deque(maxlen=self.config.history_size)
        self._ball_type = None
        self._filtered_geometry = None
        self._missing_frames = 0
        self._acquisition_frames = 0
        self._acquisition_candidate = None
        self.search_candidate = None
        self._last_raw_center = None
        self._velocity = (0.0, 0.0)
        self._locked = False
        self._state = TargetState.SEARCHING
        self._current_target = None
        self._locked_target = None
        self._new_candidate = None
        self._new_candidate_frames = 0
        self._reacquire_candidate = None
        self._reacquire_candidate_frames = 0
        self._confirmation_missing_frames = 0

    @property
    def locked(self):
        """Informa se a aquisição já travou um alvo para a execução atual."""

        return self._locked

    @property
    def state(self):
        """Expõe o estado sem permitir alteração externa da máquina."""

        return self._state

    @property
    def current_target(self):
        """Retorna o alvo atual somente para diagnóstico."""

        return self._locked_target

    @staticmethod
    def _confidence(candidate):
        if candidate.detection_method == "hough":
            return candidate.circularity
        return candidate.circle_fill_ratio

    def _distance_text(self, candidate):
        if self._distance_estimator is None:
            return "indisponível"
        try:
            return f"{self._distance_estimator(candidate.radius_pixels).distance_cm:.1f} cm"
        except (TypeError, ValueError):
            return "indisponível"

    def _distance_cm(self, candidate):
        if self._distance_estimator is None:
            return None
        try:
            distance_cm = float(
                self._distance_estimator(candidate.radius_pixels).distance_cm
            )
        except (AttributeError, TypeError, ValueError):
            return None
        return distance_cm if math.isfinite(distance_cm) and distance_cm > 0.0 else None

    def _score(self, candidate):
        """Mantém o score simples e comparável entre frames do mesmo detector."""

        return self._confidence(candidate)

    def _update_locked_target(self, candidate):
        confidence = self._confidence(candidate)
        self._locked_target = LockedTarget(
            center_x=float(candidate.center_x),
            center_y=float(candidate.center_y),
            radius_pixels=float(candidate.radius_pixels),
            distance_cm=self._distance_cm(candidate),
            color=candidate.ball_type,
            timestamp=time.monotonic(),
            confidence=confidence,
            score=self._score(candidate),
        )

    def _log_decision(self, event, current, candidate=None):
        if candidate is None:
            LOGGER.info(
                "%s\nposition: (%.1f,%.1f)\ndistance: %s\nconfidence: %.2f\nscore: %.2f",
                event,
                current.center_x,
                current.center_y,
                self._distance_text(current),
                self._confidence(current),
                self._score(current),
            )
            return
        LOGGER.info(
            "%s\nCURRENT TARGET: position=(%.1f,%.1f), distance=%s, confidence=%.2f, score=%.2f\n"
            "NEW CANDIDATE: position=(%.1f,%.1f), distance=%s, confidence=%.2f, score=%.2f",
            event,
            current.center_x,
            current.center_y,
            self._distance_text(current),
            self._confidence(current),
            self._score(current),
            candidate.center_x,
            candidate.center_y,
            self._distance_text(candidate),
            self._confidence(candidate),
            self._score(candidate),
        )

    def _same_candidate(self, first, second):
        if first is None or first.ball_type != second.ball_type:
            return False
        limit = max(
            self.config.minimum_center_jump_pixels,
            first.radius_pixels * self.config.maximum_center_jump_radii,
        )
        return math.hypot(
            first.center_x - second.center_x,
            first.center_y - second.center_y,
        ) <= limit

    def _candidate_is_better(self, candidate, frame_width):
        current = self._current_target
        if current is None:
            return True
        # Uma vítima prata é prioritária sobre uma preta, mas ainda precisa
        # respeitar a confirmação temporal e os limites físicos abaixo.
        silver_priority = (
            current.ball_type == "black_ball" and
            candidate.ball_type == "silver_ball"
        )
        score_better = silver_priority or self._score(candidate) >= (
            self._score(current) + self.config.minimum_confidence_improvement
        )
        distance_better = candidate.radius_pixels >= current.radius_pixels * (
            1.0 + self.config.minimum_distance_improvement_ratio
        )
        position_better = False
        if frame_width is not None and frame_width > 0:
            image_center_x = frame_width * 0.5
            position_better = abs(current.center_x - image_center_x) - abs(
                candidate.center_x - image_center_x
            ) >= self.config.minimum_position_improvement_pixels
        current_distance = self._distance_cm(current)
        candidate_distance = self._distance_cm(candidate)
        distance_valid = True
        if self._distance_estimator is not None:
            distance_valid = (
                current_distance is not None
                and candidate_distance is not None
                and abs(candidate_distance - current_distance) / current_distance
                <= self.config.maximum_switch_distance_change_ratio
            )
        center_jump = math.hypot(
            candidate.center_x - current.center_x,
            candidate.center_y - current.center_y,
        )
        maximum_jump = max(
            self.config.maximum_switch_jump_pixels,
            current.radius_pixels * self.config.maximum_switch_jump_radii,
        )
        return (
            score_better
            and distance_valid
            and center_jump <= maximum_jump
            and (silver_priority or distance_better or position_better)
        )

    def _confirmed_replacement(self, candidates, frame_width):
        alternatives = [
            candidate for candidate in candidates
            if self._current_target is None or not self._same_candidate(
                self._current_target,
                candidate,
            )
        ]
        if not alternatives:
            self._new_candidate = None
            self._new_candidate_frames = 0
            return None
        candidate = max(
            alternatives,
            key=lambda item: (
                item.ball_type == "silver_ball",
                self._confidence(item),
            ),
        )
        if not self._candidate_is_better(candidate, frame_width):
            self._new_candidate = None
            self._new_candidate_frames = 0
            self._log_decision("SWITCH REJECTED", self._current_target, candidate)
            return None
        if self._same_candidate(self._new_candidate, candidate):
            self._new_candidate_frames += 1
        else:
            self._new_candidate = candidate
            self._new_candidate_frames = 1
            self._log_decision("SWITCH REQUEST", self._current_target, candidate)
        if self._new_candidate_frames < self.config.switch_confirmation_frames:
            self._log_decision("KEEP TARGET", self._current_target, candidate)
            return None
        self._log_decision("SWITCH ACCEPTED", self._current_target, candidate)
        return candidate

    def _start_target(self, selected, event="TARGET LOCK"):
        """Inicia o histórico de um alvo confirmado sem herdar sua geometria."""

        self._measurements.clear()
        self._filtered_geometry = None
        self._last_raw_center = None
        self._velocity = (0.0, 0.0)
        self._new_candidate = None
        self._new_candidate_frames = 0
        self._reacquire_candidate = None
        self._reacquire_candidate_frames = 0
        self._current_target = selected
        self._update_locked_target(selected)
        self._ball_type = selected.ball_type
        self._locked = True
        self._state = TargetState.TRACKING
        self._log_decision(event, selected)

    def _confirmed_reacquisition(self, candidates):
        """Confirma novamente o mesmo tipo sem usar a posição antiga."""

        compatible = [
            candidate
            for candidate in candidates
            if candidate.ball_type == self._ball_type
        ]
        if not compatible:
            self._reacquire_candidate = None
            self._reacquire_candidate_frames = 0
            return None

        candidate = max(compatible, key=self._confidence)
        if (
            len(compatible) == 1
            and self._confidence(candidate)
            >= self.config.high_confidence_reacquire_threshold
        ):
            # O alvo já foi confirmado nesta execução e continua sendo do mesmo
            # tipo. Uma única candidata forte pode recuperar sua geometria sem
            # confundir a confiança do YOLO com uma nova aquisição completa.
            # Havendo duas vítimas compatíveis, a confirmação temporal continua
            # obrigatória para impedir uma troca silenciosa de alvo.
            return candidate
        if self._same_candidate(self._reacquire_candidate, candidate):
            self._reacquire_candidate_frames += 1
        else:
            self._reacquire_candidate = candidate
            self._reacquire_candidate_frames = 1

        if (
            self._reacquire_candidate_frames
            < self.config.reacquire_confirmation_frames
        ):
            return None
        return candidate

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

    def update(self, candidates, frame_width=None):
        """Retorna o alvo atual suavizado sem publicar frames antigos ausentes."""

        # Expõe somente a candidata escolhida neste frame, sem antecipar o lock.
        self.search_candidate = None
        if not candidates:
            self._confirmation_missing_frames += 1
            if (
                self._confirmation_missing_frames >
                self.config.confirmation_missing_frame_tolerance
            ):
                # Uma inferência vazia pode ocorrer por desfoque durante o
                # movimento. Duas ausências seguidas invalidam a confirmação
                # parcial sem reutilizar qualquer geometria antiga.
                self._acquisition_frames = 0
                self._acquisition_candidate = None
                self._reacquire_candidate = None
                self._reacquire_candidate_frames = 0
            self._missing_frames += 1
            if self._locked and self._missing_frames >= self.config.reacquire_after_missing_frames:
                self._state = TargetState.REACQUIRE
            if self._missing_frames >= self.config.search_after_missing_frames:
                self.reset()
            return []

        self._confirmation_missing_frames = 0

        if not self._locked:
            # Antes de liberar qualquer movimento, aguarda alguns frames para
            # confirmar que a mesma vencedora por área continua visível.
            selected = max(
                candidates,
                key=lambda candidate: (
                    candidate.ball_type == "silver_ball",
                    candidate.circle_fill_ratio,
                    candidate.visible_area_pixels,
                ),
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
            self.search_candidate = selected
            if self._acquisition_frames < self.config.acquisition_frames:
                return []
            self._start_target(selected)
        else:
            selected = self._select_candidate(candidates)
            if selected is None:
                # O YOLO ainda pode estar vendo a mesma vítima depois de um
                # pivô ou avanço, mas a caixa pode saltar além do gate normal.
                # Candidatas visíveis também precisam colocar o tracker em
                # REACQUIRE; antes, somente um frame totalmente vazio fazia isso.
                self._missing_frames += 1
                if (
                    self._locked
                    and self._missing_frames
                    >= self.config.reacquire_after_missing_frames
                ):
                    self._state = TargetState.REACQUIRE
            if selected is None and self._state == TargetState.REACQUIRE:
                selected = self._confirmed_reacquisition(candidates)
                if selected is not None:
                    self._start_target(selected, "TARGET REACQUIRED")
            replacement = self._confirmed_replacement(candidates, frame_width)
            if replacement is not None:
                selected = replacement
                self._start_target(selected)
        if selected is None:
            # Um alvo incompatível nunca assume a execução atual. A ausência é
            # publicada para o C++ parar os motores e controlar o timeout.
            return []

        self.search_candidate = selected
        self._missing_frames = 0
        self._acquisition_frames = 0
        self._ball_type = selected.ball_type
        self._current_target = selected
        self._update_locked_target(selected)
        self._state = TargetState.TRACKING
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
        if selected.bounding_box is not None:
            # A caixa do YOLO já fornece a geometria do frame atual. Aplicar a
            # mediana histórica fazia o ponto de mira ficar atrás da caixa.
            center_x = selected.center_x
            center_y = selected.center_y
            radius = selected.radius_pixels
        smoothed = replace(
            selected,
            center_x=float(center_x),
            center_y=float(center_y),
            radius_pixels=float(radius),
            diameter_pixels=float(radius * 2.0),
        )
        remaining = [candidate for candidate in candidates if candidate is not selected]
        return [smoothed] + remaining
