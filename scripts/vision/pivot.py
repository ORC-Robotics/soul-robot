"""Histerese do pivot comandado pelo seguidor de linha."""

from dataclasses import dataclass

from .numeric import finite_virtual_position


PIVOT_STATE_NONE = "NONE"
PIVOT_STATE_LEFT = "LEFT"
PIVOT_STATE_RIGHT = "RIGHT"


@dataclass(frozen=True)
class PivotConfig:
    """Limiares de entrada e saída do pivot normal."""

    enter_threshold: float = 0.45
    exit_threshold: float = 0.35

    def __post_init__(self):
        if not 0.0 < self.exit_threshold < self.enter_threshold <= 1.0:
            raise ValueError(
                "pivot exige 0 < exit_threshold < enter_threshold <= 1"
            )


DEFAULT_PIVOT_CONFIG = PivotConfig()
PIVOT_ENTER_THRESHOLD = DEFAULT_PIVOT_CONFIG.enter_threshold
PIVOT_EXIT_THRESHOLD = DEFAULT_PIVOT_CONFIG.exit_threshold


class VirtualPivotStateTracker:
    """Mantém o lado do pivot normal e aplica histerese sem troca direta."""

    def __init__(self, config=DEFAULT_PIVOT_CONFIG):
        self.config = config
        self.state = PIVOT_STATE_NONE

    def reset(self):
        """Encerra o pivot persistente antes de outro modo assumir."""

        self.state = PIVOT_STATE_NONE
        return self.state

    def update(self, steering_error):
        """Atualiza a entrada ou saída usando o erro do frame atual."""

        steering_error = finite_virtual_position(steering_error)
        if steering_error is None:
            return self.reset()

        steering_magnitude = abs(steering_error)
        if self.state == PIVOT_STATE_RIGHT:
            if (
                steering_error <= 0.0
                or steering_magnitude <= self.config.exit_threshold
            ):
                return self.reset()
            return self.state

        if self.state == PIVOT_STATE_LEFT:
            if (
                steering_error >= 0.0
                or steering_magnitude <= self.config.exit_threshold
            ):
                return self.reset()
            return self.state

        if steering_error >= self.config.enter_threshold:
            self.state = PIVOT_STATE_RIGHT
        elif steering_error <= -self.config.enter_threshold:
            self.state = PIVOT_STATE_LEFT
        return self.state
