"""Máquina de estado da busca cega usada para recuperar a linha."""

from dataclasses import dataclass


@dataclass(frozen=True)
class LineSearchConfig:
    """Janelas temporais de uma varredura, expressas em frames."""

    initial_frames: int = 12
    reverse_frames: int = 30

    def __post_init__(self):
        if self.initial_frames <= 0:
            raise ValueError("initial_frames deve ser positivo")
        if self.reverse_frames <= 0:
            raise ValueError("reverse_frames deve ser positivo")


DEFAULT_LINE_SEARCH_CONFIG = LineSearchConfig()

# Aliases temporários preservam a API usada pela telemetria e pelos testes.
VIRTUAL_BLIND_SEARCH_INITIAL_FRAMES = (
    DEFAULT_LINE_SEARCH_CONFIG.initial_frames
)
VIRTUAL_BLIND_SEARCH_REVERSE_FRAMES = (
    DEFAULT_LINE_SEARCH_CONFIG.reverse_frames
)


class VirtualLineSearchTracker:
    """Alterna uma busca curta e outra maior sem memorizar steering."""

    def __init__(self, config=DEFAULT_LINE_SEARCH_CONFIG):
        self.config = config
        self.last_direction = None
        self.active = False
        self.initial_direction = None
        self.search_frames = 0

    def remember(self, direction):
        """Guarda somente uma direção lateral realmente observada."""

        if direction in ("LEFT", "RIGHT"):
            self.last_direction = direction

    def stop(self):
        """Interrompe a busca sem apagar a última direção confiável."""

        self.active = False
        self.initial_direction = None
        self.search_frames = 0

    def start(self, preferred_direction=None):
        """Inicia a busca uma única vez com a melhor direção disponível."""

        if self.active:
            return
        initial_direction = (
            preferred_direction
            if preferred_direction in ("LEFT", "RIGHT")
            else self.last_direction
        )
        self.initial_direction = initial_direction or "RIGHT"
        self.active = True
        self.search_frames = 0

    def next_direction(self):
        """Retorna o lado da janela atual e avança um frame."""

        if not self.active:
            return None
        cycle_frames = (
            self.config.initial_frames
            + self.config.reverse_frames
        )
        cycle_index = self.search_frames % cycle_frames
        if cycle_index < self.config.initial_frames:
            direction = self.initial_direction
        else:
            direction = (
                "RIGHT" if self.initial_direction == "LEFT" else "LEFT"
            )
        self.search_frames += 1
        return direction
