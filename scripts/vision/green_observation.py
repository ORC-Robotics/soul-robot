"""Confirma e retém temporalmente as interpretações dos marcadores verdes."""

import time
from dataclasses import dataclass


@dataclass(frozen=True)
class GreenObservationConfig:
    """Agrupa as confirmações e a retenção temporal do detector verde."""

    # Quantidade mínima de frames iguais para publicar uma interpretação.
    confirmation_frames: int = 2
    # Marcadores laterais também precisam desta quantidade de observações.
    single_observation_frames: int = 2
    # Ausências consecutivas exigidas para apagar uma decisão não lateral.
    clear_hysteresis_frames: int = 2
    # Tempo, em segundos, que preserva o último lado realmente observado.
    direction_retention_seconds: float = 0.3

    def __post_init__(self):
        for name, value in (
            ("confirmation_frames", self.confirmation_frames),
            ("single_observation_frames", self.single_observation_frames),
            ("clear_hysteresis_frames", self.clear_hysteresis_frames),
        ):
            if value <= 0:
                raise ValueError(f"{name} deve ser positivo")
        if self.direction_retention_seconds < 0.0:
            raise ValueError(
                "direction_retention_seconds não pode ser negativo"
            )


DEFAULT_GREEN_OBSERVATION_CONFIG = GreenObservationConfig()

# Aliases preservam a API consumida pelo entrypoint e pelos testes existentes.
GREEN_CONFIRMATION_FRAMES = (
    DEFAULT_GREEN_OBSERVATION_CONFIG.confirmation_frames
)
GREEN_SINGLE_OBSERVATION_FRAMES = (
    DEFAULT_GREEN_OBSERVATION_CONFIG.single_observation_frames
)
GREEN_CLEAR_HYSTERESIS_FRAMES = (
    DEFAULT_GREEN_OBSERVATION_CONFIG.clear_hysteresis_frames
)
GREEN_DIRECTION_RETENTION_SECONDS = (
    DEFAULT_GREEN_OBSERVATION_CONFIG.direction_retention_seconds
)


class GreenObservationTracker:
    """Confirma observações novas e remove decisões após curta histerese."""

    def __init__(self, config=DEFAULT_GREEN_OBSERVATION_CONFIG):
        self.config = config
        self.last_sequence = None
        self.pending_interpretation = "SEM_DECISAO"
        self.consecutive_samples = 0
        self.missing_samples = 0
        self.confirmed_interpretation = "SEM_DECISAO"
        self.last_direction_seen_at = None

    def update(self, line_sequence, interpretation, observed_at=None):
        """Confirma frames novos e retém brevemente a orientação lateral."""

        observed_at = (
            time.perf_counter() if observed_at is None else float(observed_at)
        )
        if line_sequence == self.last_sequence:
            return (
                self.confirmed_interpretation,
                self.confirmed_interpretation != "SEM_DECISAO",
                self.consecutive_samples,
            )
        self.last_sequence = line_sequence

        if interpretation in ("ESQUERDA", "DIREITA"):
            # O instante é renovado em todo frame detectado, inclusive durante
            # a confirmação, para que a retenção conte da última visão real.
            self.last_direction_seen_at = observed_at

        if interpretation == "SEM_DECISAO":
            direction_is_retained = (
                self.confirmed_interpretation in ("ESQUERDA", "DIREITA")
                and self.last_direction_seen_at is not None
                and observed_at - self.last_direction_seen_at
                < self.config.direction_retention_seconds
            )
            if direction_is_retained:
                return (
                    self.confirmed_interpretation,
                    True,
                    self.consecutive_samples,
                )
            if self.confirmed_interpretation in ("ESQUERDA", "DIREITA"):
                self.pending_interpretation = "SEM_DECISAO"
                self.confirmed_interpretation = "SEM_DECISAO"
                self.consecutive_samples = 0
                self.missing_samples = 0
                return "SEM_DECISAO", False, 0
            self.missing_samples += 1
            if (
                self.missing_samples
                >= self.config.clear_hysteresis_frames
            ):
                self.pending_interpretation = "SEM_DECISAO"
                self.confirmed_interpretation = "SEM_DECISAO"
                self.consecutive_samples = 0
            return (
                self.confirmed_interpretation,
                self.confirmed_interpretation != "SEM_DECISAO",
                self.consecutive_samples,
            )

        self.missing_samples = 0
        if interpretation != self.pending_interpretation:
            self.pending_interpretation = interpretation
            self.consecutive_samples = 1
            self.confirmed_interpretation = "SEM_DECISAO"
        else:
            self.consecutive_samples += 1

        required_samples = self.config.confirmation_frames
        if interpretation in ("ESQUERDA", "DIREITA"):
            required_samples = max(
                required_samples,
                self.config.single_observation_frames,
            )
        # A telemetria representa o progresso da confirmação, não há motivo
        # para crescer sem limite depois de a decisão já estar aceita.
        self.consecutive_samples = min(
            self.consecutive_samples,
            required_samples,
        )
        confirmable = interpretation not in ("AMBIGUO", "SEM_DECISAO")
        if confirmable and self.consecutive_samples >= required_samples:
            self.confirmed_interpretation = interpretation

        published_interpretation = self.confirmed_interpretation
        if interpretation == "AMBIGUO":
            return "AMBIGUO", False, self.consecutive_samples
        return (
            published_interpretation,
            self.confirmed_interpretation != "SEM_DECISAO",
            self.consecutive_samples,
        )
