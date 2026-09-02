"""Estados críticos acionados pela leitura lateral do sensor MEDIUM."""

from dataclasses import dataclass

from .numeric import finite_virtual_position
from .pivot import (
    PIVOT_STATE_LEFT,
    PIVOT_STATE_NONE,
    PIVOT_STATE_RIGHT,
)


@dataclass(frozen=True)
class MediumSpinConfig:
    """Agrupa limiares e tempos do SPIN e do hard corner."""

    # A histerese exige erro maior para entrar do que para permanecer no SPIN.
    spin_enter_threshold: float = 0.45
    spin_exit_threshold: float = 0.30
    # Potência diferencial aplicada durante o giro no próprio eixo.
    spin_power: float = 0.72
    # Abaixo deste erro, o MEDIUM deixa de fornecer uma direção confiável.
    direction_lost_threshold: float = 0.15
    # Quantidade máxima de frames sem direção antes de cancelar o estado.
    critical_invalid_max_frames: int = 10
    # Limites que consideram a faixa novamente alinhada após um hard corner.
    hard_corner_near_recovery_threshold: float = 0.18
    hard_corner_medium_recovery_threshold: float = 0.20
    # Confirmações consecutivas exigidas para devolver o controle ao normal.
    hard_corner_recovery_frames: int = 2
    hard_corner_far_recovery_frames: int = 2
    # Em 30 FPS, trinta frames limitam o hard corner a cerca de um segundo.
    hard_corner_max_frames: int = 30

    def __post_init__(self):
        if not (
            0.0
            < self.spin_exit_threshold
            < self.spin_enter_threshold
            <= 1.0
        ):
            raise ValueError(
                "medium spin exige 0 < saída < entrada <= 1"
            )
        if not 0.0 < self.spin_power <= 1.0:
            raise ValueError("spin_power deve estar em (0, 1]")
        if not 0.0 <= self.direction_lost_threshold < self.spin_exit_threshold:
            raise ValueError(
                "direction_lost_threshold deve ficar abaixo da saída do spin"
            )
        if self.critical_invalid_max_frames < 0:
            raise ValueError(
                "critical_invalid_max_frames não pode ser negativo"
            )
        for name, value in (
            (
                "hard_corner_near_recovery_threshold",
                self.hard_corner_near_recovery_threshold,
            ),
            (
                "hard_corner_medium_recovery_threshold",
                self.hard_corner_medium_recovery_threshold,
            ),
        ):
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} deve estar em [0, 1]")
        for name, value in (
            ("hard_corner_recovery_frames", self.hard_corner_recovery_frames),
            (
                "hard_corner_far_recovery_frames",
                self.hard_corner_far_recovery_frames,
            ),
            ("hard_corner_max_frames", self.hard_corner_max_frames),
        ):
            if value <= 0:
                raise ValueError(f"{name} deve ser positivo")


DEFAULT_MEDIUM_SPIN_CONFIG = MediumSpinConfig()

# Aliases preservam a API de configuração consumida pelo entrypoint e testes.
VIRTUAL_MEDIUM_SPIN_ENTER_THRESHOLD = (
    DEFAULT_MEDIUM_SPIN_CONFIG.spin_enter_threshold
)
VIRTUAL_MEDIUM_SPIN_EXIT_THRESHOLD = (
    DEFAULT_MEDIUM_SPIN_CONFIG.spin_exit_threshold
)
VIRTUAL_MEDIUM_SPIN_POWER = DEFAULT_MEDIUM_SPIN_CONFIG.spin_power
VIRTUAL_MEDIUM_DIRECTION_LOST_THRESHOLD = (
    DEFAULT_MEDIUM_SPIN_CONFIG.direction_lost_threshold
)
VIRTUAL_MEDIUM_CRITICAL_INVALID_MAX_FRAMES = (
    DEFAULT_MEDIUM_SPIN_CONFIG.critical_invalid_max_frames
)
VIRTUAL_HARD_CORNER_NEAR_RECOVERY_THRESHOLD = (
    DEFAULT_MEDIUM_SPIN_CONFIG.hard_corner_near_recovery_threshold
)
VIRTUAL_HARD_CORNER_MEDIUM_RECOVERY_THRESHOLD = (
    DEFAULT_MEDIUM_SPIN_CONFIG.hard_corner_medium_recovery_threshold
)
VIRTUAL_HARD_CORNER_RECOVERY_FRAMES = (
    DEFAULT_MEDIUM_SPIN_CONFIG.hard_corner_recovery_frames
)
VIRTUAL_HARD_CORNER_FAR_RECOVERY_FRAMES = (
    DEFAULT_MEDIUM_SPIN_CONFIG.hard_corner_far_recovery_frames
)
VIRTUAL_HARD_CORNER_MAX_FRAMES = (
    DEFAULT_MEDIUM_SPIN_CONFIG.hard_corner_max_frames
)


class VirtualMediumSpinTracker:
    """Mantém os SPINs do MEDIUM e a direção persistente do hard corner."""

    def __init__(self, config=DEFAULT_MEDIUM_SPIN_CONFIG):
        self.config = config
        self.reset()

    def reset(self):
        """Cancela a ação crítica sem preservar a direção antiga."""

        self.state = PIVOT_STATE_NONE
        self.critical_state = PIVOT_STATE_NONE
        self.invalid_frames = 0
        self.hard_corner_state = PIVOT_STATE_NONE
        self.hard_corner_frames = 0
        self.hard_corner_recovery_frames = 0
        self.hard_corner_far_recovery_frames = 0
        self.hard_corner_entry_blocked = False
        return self.state

    def clear_hard_corner(self, block_entry=False):
        """Limpa a manobra persistente e seus contadores internos."""

        self.hard_corner_state = PIVOT_STATE_NONE
        self.hard_corner_frames = 0
        self.hard_corner_recovery_frames = 0
        self.hard_corner_far_recovery_frames = 0
        self.hard_corner_entry_blocked = bool(block_entry)
        self.state = PIVOT_STATE_NONE
        self.critical_state = PIVOT_STATE_NONE
        self.invalid_frames = 0

    def update(
        self,
        medium_position,
        near_position_valid,
        critical_entry_allowed=True,
        hard_corner_entry_requested=False,
        near_fine_position=None,
        far_position=None,
    ):
        """Atualiza a ação crítica e limita a perda direcional do MEDIUM."""

        medium_position = finite_virtual_position(medium_position)
        near_fine_position = finite_virtual_position(near_fine_position)
        far_position = finite_virtual_position(far_position)
        if not hard_corner_entry_requested:
            # Um timeout não pode reabrir a mesma manobra enquanto o gatilho
            # contínuo permanecer ativo. A próxima observação distinta rearma.
            self.hard_corner_entry_blocked = False

        if (
            self.hard_corner_state == PIVOT_STATE_NONE
            and hard_corner_entry_requested
            and not self.hard_corner_entry_blocked
            and medium_position is not None
        ):
            # A direção observada na entrada permanece fixa durante todo o
            # giro, mesmo que os sensores mudem de lado durante a rotação.
            self.hard_corner_state = (
                PIVOT_STATE_RIGHT
                if medium_position > 0.0
                else PIVOT_STATE_LEFT
            )
            self.hard_corner_frames = 0
            self.hard_corner_recovery_frames = 0
            self.hard_corner_far_recovery_frames = 0
            self.critical_state = PIVOT_STATE_NONE
            self.invalid_frames = 0

        if self.hard_corner_state != PIVOT_STATE_NONE:
            self.hard_corner_frames += 1
            line_aligned = (
                near_position_valid
                and near_fine_position is not None
                and abs(near_fine_position)
                <= self.config.hard_corner_near_recovery_threshold
                and medium_position is not None
                and abs(medium_position)
                <= self.config.hard_corner_medium_recovery_threshold
            )
            if line_aligned:
                self.hard_corner_recovery_frames += 1
            else:
                self.hard_corner_recovery_frames = 0

            if far_position is not None:
                self.hard_corner_far_recovery_frames += 1
            else:
                self.hard_corner_far_recovery_frames = 0

            hard_corner_recovered = (
                self.hard_corner_recovery_frames
                >= self.config.hard_corner_recovery_frames
            )
            hard_corner_far_recovered = (
                self.hard_corner_far_recovery_frames
                >= self.config.hard_corner_far_recovery_frames
            )
            hard_corner_timed_out = (
                self.hard_corner_frames >= self.config.hard_corner_max_frames
            )
            if hard_corner_recovered or hard_corner_far_recovered:
                # O mesmo frame já volta ao fluxo normal ou ao recovery.
                self.clear_hard_corner()
                return self.state
            if hard_corner_timed_out:
                # O timeout não inicia uma busca adicional e bloqueia a
                # reentrada até o gatilho atual desaparecer.
                self.clear_hard_corner(block_entry=True)
                return self.state
            self.state = self.hard_corner_state
            return self.state

        medium_direction_lost = (
            medium_position is None
            or abs(medium_position) < self.config.direction_lost_threshold
        )
        if medium_direction_lost:
            self.state = PIVOT_STATE_NONE
            if near_position_valid or self.critical_state == PIVOT_STATE_NONE:
                return self.reset()
            if (
                self.invalid_frames
                >= self.config.critical_invalid_max_frames
            ):
                return self.reset()
            self.invalid_frames += 1
            self.state = self.critical_state
            return self.state

        medium_magnitude = abs(medium_position)
        medium_state = (
            PIVOT_STATE_RIGHT
            if medium_position > 0.0
            else PIVOT_STATE_LEFT
        )

        # Qualquer leitura válida encerra imediatamente a tolerância. Se ela
        # ainda for crítica, o lado atual substitui a observação anterior.
        self.invalid_frames = 0

        if not critical_entry_allowed:
            return self.reset()

        if near_position_valid:
            self.state = PIVOT_STATE_NONE
            if medium_magnitude >= self.config.spin_enter_threshold:
                self.critical_state = medium_state
            else:
                self.critical_state = PIVOT_STATE_NONE
            return self.state

        if self.state != PIVOT_STATE_NONE:
            if (
                medium_magnitude <= self.config.spin_exit_threshold
                or medium_state != self.state
            ):
                return self.reset()
            return self.state

        if medium_magnitude >= self.config.spin_enter_threshold:
            self.state = medium_state
            self.critical_state = medium_state
        else:
            self.critical_state = PIVOT_STATE_NONE
        return self.state
