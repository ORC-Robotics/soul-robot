"""Estado persistente das manobras verdes e da travessia de GAP."""

from dataclasses import dataclass

from .line_control import (
    LIMIAR_CENTRALIZACAO_VERDE,
    LIMIAR_CURVA_VERDE_INICIADA,
    QUADROS_CENTRALIZADO_PARA_CONCLUIR,
    QUADROS_PARA_REARMAR_VERDE,
    gap_entry_is_required,
    update_gap_forward_recovery,
    update_gap_recent_near_frames,
    update_green_maneuver_state,
)


NO_GREEN_DIRECTION = "NENHUMA"
GREEN_DIRECTIONS = ("ESQUERDA", "DIREITA")


@dataclass
class LineManeuverState:
    """Mantém GREEN e GAP coerentes entre frames consecutivos."""

    green_direction: str = NO_GREEN_DIRECTION
    green_curve_started: bool = False
    green_centered_frames: int = 0
    green_active_frames: int = 0
    green_armed: bool = True
    green_clear_frames: int = 0
    gap_forward_active: bool = False
    gap_forward_frames: int = 0
    gap_reacquire_frames: int = 0
    gap_line_lost_seen: bool = False
    gap_recent_near_frames: int = 0

    def accept_green(self, green_status, line_controller):
        """Aceita uma direção confirmada somente quando o verde está armado."""

        if not (
            self.green_armed
            and self.green_direction == NO_GREEN_DIRECTION
            and green_status["greenConfirmed"]
        ):
            return False

        interpretation = green_status["greenInterpretation"]
        if interpretation not in GREEN_DIRECTIONS:
            return False

        self.green_direction = interpretation
        self.green_curve_started = False
        self.green_centered_frames = 0
        self.green_active_frames = 0
        self.green_armed = False
        self.green_clear_frames = 0
        line_controller.stop_search()
        return True

    def observe_green_candidates(self, candidate_count):
        """Conta frames limpos antes de permitir outro marcador verde."""

        if self.green_armed:
            return
        if candidate_count == 0:
            self.green_clear_frames += 1
        else:
            self.green_clear_frames = 0

    def update_near_history(self, near_center_visible):
        """Atualiza a memória curta que permite reconhecer o início do GAP."""

        self.gap_recent_near_frames = update_gap_recent_near_frames(
            self.gap_recent_near_frames,
            near_center_visible,
        )

    def apply_green_timeout(self, raw_line_visible, line_controller):
        """Atualiza o timeout e solicita recovery quando o verde perde a linha."""

        timeout_state = update_green_maneuver_state(
            self.green_direction,
            self.green_active_frames,
            raw_line_visible,
        )
        self.green_direction = timeout_state["direction"]
        self.green_active_frames = timeout_state["activeFrames"]
        sensor_recovery_requested = False

        if timeout_state["timedOut"]:
            self.green_curve_started = False
            self.green_centered_frames = 0
            search_direction = timeout_state["searchDirection"]
            if search_direction is not None:
                line_controller.start_search(search_direction)
            else:
                sensor_recovery_requested = True

        return {
            "timedOut": timeout_state["timedOut"],
            "sensorRecoveryRequested": sensor_recovery_requested,
        }

    def enter_gap_if_required(
        self,
        near_center_visible,
        real_near_point,
        virtual_near_point,
        lateral_exit_target,
    ):
        """Entra no GAP usando a mesma condição geométrica preservada."""

        should_enter = gap_entry_is_required(
            self.gap_forward_active,
            self.green_direction,
            self.gap_recent_near_frames,
            near_center_visible,
            real_near_point,
            virtual_near_point,
            lateral_exit_target,
        )
        if should_enter:
            self.gap_forward_active = True
            self.gap_forward_frames = 0
            self.gap_reacquire_frames = 0
            self.gap_line_lost_seen = True
        return should_enter

    def update_gap_recovery(self, near_reacquired, line_controller):
        """Avança a travessia de GAP e devolve a solicitação de busca cega."""

        if not self.gap_forward_active:
            return False

        gap_state = update_gap_forward_recovery(
            self.gap_forward_active,
            self.gap_forward_frames,
            self.gap_reacquire_frames,
            self.gap_line_lost_seen,
            near_reacquired,
        )
        self.gap_forward_active = gap_state["active"]
        self.gap_forward_frames = gap_state["forwardFrames"]
        self.gap_reacquire_frames = gap_state["reacquireFrames"]
        self.gap_line_lost_seen = gap_state["lineLostSeen"]

        if not self.gap_forward_active:
            line_controller.stop_search()
        return gap_state["blindSearchRequested"]

    def update_green_alignment(
        self,
        near_fine_position,
        raw_line_visible,
        line_controller,
    ):
        """Conclui a curva verde após deslocamento e centralização estável."""

        if not self.green_curve_started:
            if (
                self.green_direction == "ESQUERDA"
                and near_fine_position is not None
                and near_fine_position <= -LIMIAR_CURVA_VERDE_INICIADA
            ):
                self.green_curve_started = True
                self.green_centered_frames = 0
            elif (
                self.green_direction == "DIREITA"
                and near_fine_position is not None
                and near_fine_position >= LIMIAR_CURVA_VERDE_INICIADA
            ):
                self.green_curve_started = True
                self.green_centered_frames = 0

        if (
            self.green_direction != NO_GREEN_DIRECTION
            and self.green_curve_started
        ):
            if (
                near_fine_position is not None
                and abs(near_fine_position) <= LIMIAR_CENTRALIZACAO_VERDE
            ):
                self.green_centered_frames += 1
            else:
                self.green_centered_frames = 0

            if (
                self.green_centered_frames
                >= QUADROS_CENTRALIZADO_PARA_CONCLUIR
            ):
                completed_state = update_green_maneuver_state(
                    self.green_direction,
                    self.green_active_frames,
                    raw_line_visible,
                    completed=True,
                )
                self.green_direction = completed_state["direction"]
                self.green_active_frames = completed_state["activeFrames"]
                self.green_curve_started = False
                self.green_centered_frames = 0
                line_controller.stop_search()

        if (
            not self.green_armed
            and self.green_direction == NO_GREEN_DIRECTION
            and self.green_clear_frames >= QUADROS_PARA_REARMAR_VERDE
        ):
            self.green_armed = True
            self.green_clear_frames = 0
