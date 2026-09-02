"""Estado de reorientação baseado na concordância entre MEDIUM e FAR."""

from dataclasses import dataclass


VIRTUAL_STATE_NORMAL = "NORMAL"
VIRTUAL_STATE_REORIENT_LEFT = "REORIENT_LEFT"
VIRTUAL_STATE_REORIENT_RIGHT = "REORIENT_RIGHT"


@dataclass(frozen=True)
class ReorientConfig:
    """Confirmações temporais e limiar lateral do recovery."""

    direction_threshold: float = 0.20
    confirmation_frames: int = 4
    recovery_frames: int = 2
    medium_scan_max_frames: int = 3

    def __post_init__(self):
        if not 0.0 < self.direction_threshold <= 1.0:
            raise ValueError("direction_threshold deve estar em (0, 1]")
        if self.confirmation_frames <= 0:
            raise ValueError("confirmation_frames deve ser positivo")
        if self.recovery_frames <= 0:
            raise ValueError("recovery_frames deve ser positivo")
        if self.medium_scan_max_frames <= 0:
            raise ValueError("medium_scan_max_frames deve ser positivo")


DEFAULT_REORIENT_CONFIG = ReorientConfig()
VIRTUAL_MEDIUM_SCAN_MAX_FRAMES = (
    DEFAULT_REORIENT_CONFIG.medium_scan_max_frames
)
VIRTUAL_REORIENT_DIRECTION_THRESHOLD = (
    DEFAULT_REORIENT_CONFIG.direction_threshold
)
VIRTUAL_REORIENT_CONFIRMATION_FRAMES = (
    DEFAULT_REORIENT_CONFIG.confirmation_frames
)
VIRTUAL_REORIENT_RECOVERY_FRAMES = (
    DEFAULT_REORIENT_CONFIG.recovery_frames
)


def virtual_sensor_trust_is_active(sensors, trust_name):
    """Aceita somente o booleano verdadeiro produzido pelo gate visual."""

    return sensors.get(trust_name) is True


def virtual_reorient_direction(sensors, config=DEFAULT_REORIENT_CONFIG):
    """Detecta concordância lateral entre MEDIUM e FAR BAND."""

    if not (
        virtual_sensor_trust_is_active(sensors, "mediumTrusted")
        and virtual_sensor_trust_is_active(sensors, "farTrusted")
    ):
        return None
    medium_position = sensors.get("mediumPosition")
    far_band_position = sensors.get("farBandPosition")
    if medium_position is None or far_band_position is None:
        return None

    threshold = config.direction_threshold
    if medium_position >= threshold and far_band_position >= threshold:
        return "RIGHT"
    if medium_position <= -threshold and far_band_position <= -threshold:
        return "LEFT"
    return None


class VirtualTurnStateTracker:
    """Mantém somente o pivot temporário usado para recuperar a linha."""

    def __init__(self, config=DEFAULT_REORIENT_CONFIG):
        self.config = config
        self.reset()

    def reset(self):
        """Retorna ao seguidor normal e limpa todas as confirmações."""

        self.state = VIRTUAL_STATE_NORMAL
        self.reorient_candidate = None
        self.reorient_frames = 0
        self.normal_recovery_frames = 0
        self.medium_scan_frames = 0
        return self.state

    def allow_medium_scan(self, direction):
        """Consome uma janela curta de scan enquanto permanece NORMAL."""

        if direction is None or self.state != VIRTUAL_STATE_NORMAL:
            return False
        if self.medium_scan_frames >= self.config.medium_scan_max_frames:
            return False
        self.medium_scan_frames += 1
        return True

    def update(self, sensors, steering_valid):
        """Confirma o recovery e devolve autoridade ao steering normal."""

        if steering_valid:
            self.reorient_candidate = None
            self.reorient_frames = 0
            self.medium_scan_frames = 0
            if self.state in (
                VIRTUAL_STATE_REORIENT_LEFT,
                VIRTUAL_STATE_REORIENT_RIGHT,
            ):
                self.normal_recovery_frames += 1
                if (
                    self.normal_recovery_frames
                    >= self.config.recovery_frames
                ):
                    return self.reset()
                return self.state
            return self.reset()

        self.normal_recovery_frames = 0
        if self.state in (
            VIRTUAL_STATE_REORIENT_LEFT,
            VIRTUAL_STATE_REORIENT_RIGHT,
        ):
            return self.state

        direction = virtual_reorient_direction(sensors, self.config)
        if direction is None:
            self.reorient_candidate = None
            self.reorient_frames = 0
            return self.state

        if direction == self.reorient_candidate:
            self.reorient_frames += 1
        else:
            self.reorient_candidate = direction
            self.reorient_frames = 1

        if self.reorient_frames >= self.config.confirmation_frames:
            self.state = (
                VIRTUAL_STATE_REORIENT_LEFT
                if direction == "LEFT"
                else VIRTUAL_STATE_REORIENT_RIGHT
            )
            self.medium_scan_frames = 0
        return self.state
