"""Controle do segue-faixa e trackers com estado."""

import math
import cv2  # type: ignore
from .camera_config import (
    GAP_NEAR_HISTORY_FRAMES,
    GEOMETRIC_GAP_FORWARD_MAX_FRAMES,
    GEOMETRIC_GAP_REACQUIRE_FRAMES,
    GREEN_ENTRY_FUSION_MIN_STEERING,
    GREEN_ENTRY_PIVOT_MAX_FRAMES,
    GREEN_MANEUVER_TIMEOUT_FRAMES,
    NORMAL_BASE_POWER,
    NORMAL_FULL_STEERING_ERROR,
    NORMAL_INNER_MIN_POWER,
    NORMAL_MAX_POWER,
    PIVOT_ENTER_THRESHOLD,
    PIVOT_EXIT_THRESHOLD,
    PIVOT_INNER_POWER,
    PIVOT_OUTER_POWER,
    PIVOT_STATE_LEFT,
    PIVOT_STATE_NONE,
    PIVOT_STATE_RIGHT,
    VIRTUAL_BLIND_SEARCH_BACKUP_FRAMES,
    VIRTUAL_BLIND_SEARCH_BACKUP_POWER,
    VIRTUAL_BLIND_SEARCH_CONFIRMATION_FRAMES,
    VIRTUAL_BLIND_SEARCH_INITIAL_FRAMES,
    VIRTUAL_BLIND_SEARCH_REVERSE_FRAMES,
    VIRTUAL_FINE_CENTER_GAIN,
    VIRTUAL_FINE_CENTER_MAX_CORRECTION,
    VIRTUAL_HARD_CORNER_FAR_RECOVERY_FRAMES,
    VIRTUAL_HARD_CORNER_MAX_FRAMES,
    VIRTUAL_HARD_CORNER_MEDIUM_RECOVERY_THRESHOLD,
    VIRTUAL_HARD_CORNER_NEAR_RECOVERY_THRESHOLD,
    VIRTUAL_HARD_CORNER_RECOVERY_FRAMES,
    VIRTUAL_MEDIUM_CRITICAL_INVALID_MAX_FRAMES,
    VIRTUAL_MEDIUM_DIRECTION_LOST_THRESHOLD,
    VIRTUAL_MEDIUM_SCAN_MAX_FRAMES,
    VIRTUAL_MEDIUM_SCAN_POSITION_THRESHOLD,
    VIRTUAL_MEDIUM_SPIN_ENTER_THRESHOLD,
    VIRTUAL_MEDIUM_SPIN_EXIT_THRESHOLD,
    VIRTUAL_MEDIUM_SPIN_POWER,
    VIRTUAL_MEDIUM_STRONG_THRESHOLD,
    VIRTUAL_REORIENT_CONFIRMATION_FRAMES,
    VIRTUAL_REORIENT_DIRECTION_THRESHOLD,
    VIRTUAL_REORIENT_RECOVERY_FRAMES,
    VIRTUAL_STATE_NORMAL,
    VIRTUAL_STATE_REORIENT_LEFT,
    VIRTUAL_STATE_REORIENT_RIGHT,
)
from .fusion_guidance import (
    apply_fusion_forward_speed_limit,
    calculate_fusion_control_status,
    green_direction_to_search_direction,
    map_fusion_angle_to_motor_powers,
    map_normal_steering_error,
)
from .numeric import (
    finite_virtual_position,
)
from .virtual_sensors import (
    apply_virtual_fine_center_deadband,
    calculate_virtual_heading_angle,
    calculate_virtual_steering_error,
    non_negative_line_measurement,
    normalized_line_confidence,
    read_virtual_line_sensors,
    resolve_virtual_sensor_geometry,
    virtual_sensor_is_active,
    virtual_sensor_trust_is_active,
)

def draw_line_control_overlay(frame, line_follower_command):
    """Desenha o estado principal e as potências no topo da câmera inferior."""

    processing_ms = finite_virtual_position(
        line_follower_command.get("lineProcessingMs")
    )
    line_state = str(
        line_follower_command.get("lineState", "INVALID")
    ).strip().upper() or "INVALID"
    virtual_state = str(
        line_follower_command.get("virtualState", "INVALID")
    ).strip().upper() or "INVALID"
    display_state = line_state
    if (
        line_state == "LINE"
        and virtual_state in (
            VIRTUAL_STATE_REORIENT_LEFT,
            VIRTUAL_STATE_REORIENT_RIGHT,
        )
    ):
        display_state = "REORIENT"

    processing_text = (
        f"{display_state} {processing_ms:.1f}ms"
        if processing_ms is not None
        else f"{display_state} --ms"
    )

    overlay_texts = (
        processing_text,
        (
            f"L {float(line_follower_command['left_power']):.2f}  "
            f"R {float(line_follower_command['right_power']):.2f}"
        ),
    )
    if "nearLineState" in line_follower_command:
        overlay_texts += (
            f"NEAR {line_follower_command['nearLineState']}  FWD {line_follower_command['forwardPresenceState']}",
            f"DECISION {line_follower_command['gapValidationDecision']}",
            f"SOURCE {line_follower_command['controlSource']}",
        )
    for line_index, overlay_text in enumerate(overlay_texts):
        cv2.putText(
            frame,
            overlay_text,
            (8, 22 + line_index * 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )


def virtual_reorient_direction(sensors):
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

    threshold = VIRTUAL_REORIENT_DIRECTION_THRESHOLD
    if medium_position >= threshold and far_band_position >= threshold:
        return "RIGHT"
    if medium_position <= -threshold and far_band_position <= -threshold:
        return "LEFT"
    return None


def virtual_medium_scan_direction(sensors):
    """Retorna o lado confiável observado somente pelo MEDIUM."""

    if not virtual_sensor_trust_is_active(sensors, "mediumTrusted"):
        return None
    medium_position = sensors.get("mediumPosition")
    if medium_position is None:
        return None
    try:
        medium_position = float(medium_position)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(medium_position):
        return None
    if medium_position <= -VIRTUAL_MEDIUM_SCAN_POSITION_THRESHOLD:
        return "LEFT"
    if medium_position >= VIRTUAL_MEDIUM_SCAN_POSITION_THRESHOLD:
        return "RIGHT"
    return None


def virtual_recovery_sensor_direction(sensors):
    """Escolhe uma direção trusted na ordem MEDIUM e FAR BAND."""

    positions = (
        (
            sensors.get("mediumPosition"),
            virtual_sensor_trust_is_active(sensors, "mediumTrusted"),
        ),
        (
            sensors.get("farBandPosition"),
            virtual_sensor_trust_is_active(sensors, "farTrusted"),
        ),
    )
    for position, trusted in positions:
        if not trusted:
            continue
        position = finite_virtual_position(position)
        if position is None:
            continue
        if position <= -VIRTUAL_MEDIUM_SCAN_POSITION_THRESHOLD:
            return "LEFT"
        if position >= VIRTUAL_MEDIUM_SCAN_POSITION_THRESHOLD:
            return "RIGHT"
    return None


def virtual_raw_line_is_visible(sensors):
    """Indica presença no NEAR-C ou fileira frontal com trust ativo."""

    forward_positions = (
        (
            sensors.get("mediumPosition"),
            virtual_sensor_trust_is_active(sensors, "mediumTrusted"),
        ),
        (
            sensors.get("farBandPosition"),
            virtual_sensor_trust_is_active(sensors, "farTrusted"),
        ),
    )
    return (
        virtual_sensor_is_active(sensors.get("nearCenter"))
        or any(
            trusted and finite_virtual_position(position) is not None
            for position, trusted in forward_positions
        )
    )


def virtual_far_line_is_visible(sensors):
    """Confirma qualquer posição FAR finita protegida pelo gate de trust."""

    if not virtual_sensor_trust_is_active(sensors, "farTrusted"):
        return False
    return any(
        finite_virtual_position(sensors.get(position_name)) is not None
        for position_name in ("farPosition", "farBandPosition")
    )


def update_gap_recent_near_frames(recent_near_frames, near_center_visible):
    """Atualiza a memória curta de uma observação real no NEAR-C."""

    if near_center_visible:
        return GAP_NEAR_HISTORY_FRAMES
    return max(0, int(recent_near_frames) - 1)


def gap_entry_is_required(
    gap_forward_active,
    green_direction,
    recent_near_frames=0,
    near_center_visible=False,
    real_near_point=None,
    virtual_near_point=None,
    lateral_exit_target=None,
    fusion_near_connected=False,
    *,
    near_line_present=None,
    near_loss_confirmed=False,
    special_control=False,
):
    """Usa perda confirmada do NEAR inteiro; conserva a chamada antiga."""

    if near_line_present is not None:
        # A aplicação fornece a presença temporal de toda a faixa NEAR. Enquanto
        # a máscara final ainda toca essa faixa, o Fusion conserva autoridade.
        return (not gap_forward_active and green_direction == "NENHUMA"
                and not special_control and not near_line_present and near_loss_confirmed)

    return (
        not gap_forward_active
        and green_direction == "NENHUMA"
        and not fusion_near_connected
        and recent_near_frames > 0
        and not near_center_visible
        and real_near_point is None
        and virtual_near_point is None
        and lateral_exit_target is None
    )


class VirtualLineSearchTracker:
    """Alterna uma busca cega curta e outra maior sem memorizar steering."""

    def __init__(self):
        self.last_direction = None
        self.active = False
        self.initial_direction = None
        self.search_frames = 0
        self.backup_frames_remaining = 0
        self.entry_confirmation_frames = 0

    def remember(self, direction):
        """Guarda somente uma direção lateral realmente observada."""

        if direction in ("LEFT", "RIGHT"):
            self.last_direction = direction
            self.entry_confirmation_frames = 0

    def stop(self):
        """Interrompe a busca sem apagar a última direção confiável."""

        self.active = False
        self.initial_direction = None
        self.search_frames = 0
        self.backup_frames_remaining = 0
        self.entry_confirmation_frames = 0

    def start(self, preferred_direction=None, backup_frames=0):
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
        self.backup_frames_remaining = max(0, int(backup_frames))
        self.entry_confirmation_frames = 0

    def request_automatic_start(self, preferred_direction=None):
        """Confirma a perda antes de iniciar uma busca automática de LINE."""

        if self.active:
            return True
        self.entry_confirmation_frames += 1
        if (
            self.entry_confirmation_frames
            < VIRTUAL_BLIND_SEARCH_CONFIRMATION_FRAMES
        ):
            return False
        self.start(
            preferred_direction,
            backup_frames=VIRTUAL_BLIND_SEARCH_BACKUP_FRAMES,
        )
        return True

    def next_direction(self):
        """Retorna a ação da janela atual e avança um frame."""

        if not self.active:
            return None
        if self.backup_frames_remaining > 0:
            self.backup_frames_remaining -= 1
            return "BACKWARD"
        cycle_frames = (
            VIRTUAL_BLIND_SEARCH_INITIAL_FRAMES
            + VIRTUAL_BLIND_SEARCH_REVERSE_FRAMES
        )
        cycle_index = self.search_frames % cycle_frames
        if cycle_index < VIRTUAL_BLIND_SEARCH_INITIAL_FRAMES:
            direction = self.initial_direction
        else:
            direction = (
                "RIGHT" if self.initial_direction == "LEFT" else "LEFT"
            )
        self.search_frames += 1
        return direction


def update_gap_forward_recovery(
    active,
    forward_frames,
    reacquire_frames,
    line_lost_seen,
    near_reacquired,
):
    """Atualiza o GAP sem permitir que MEDIUM ou FAR encerrem a travessia."""

    if not active:
        return {
            "active": False,
            "forwardFrames": 0,
            "reacquireFrames": 0,
            "lineLostSeen": False,
            "blindSearchRequested": False,
        }

    forward_frames += 1
    if not near_reacquired:
        line_lost_seen = True
    if line_lost_seen and near_reacquired:
        reacquire_frames += 1
    else:
        reacquire_frames = 0

    if reacquire_frames >= GEOMETRIC_GAP_REACQUIRE_FRAMES:
        return {
            "active": False,
            "forwardFrames": 0,
            "reacquireFrames": 0,
            "lineLostSeen": False,
            "blindSearchRequested": False,
        }

    return {
        "active": True,
        "forwardFrames": forward_frames,
        "reacquireFrames": reacquire_frames,
        "lineLostSeen": line_lost_seen,
        "blindSearchRequested": (
            forward_frames >= GEOMETRIC_GAP_FORWARD_MAX_FRAMES
        ),
    }


def update_green_maneuver_state(
    direction,
    active_frames,
    raw_line_visible,
    completed=False,
):
    """Libera o verde por conclusão, limite angular ou timeout de proteção."""

    if direction == "NENHUMA":
        return {
            "direction": "NENHUMA",
            "activeFrames": 0,
            "timedOut": False,
            "searchDirection": None,
        }
    if completed:
        return {
            "direction": "NENHUMA",
            "activeFrames": 0,
            "timedOut": False,
            "searchDirection": None,
        }

    active_frames += 1
    if active_frames < GREEN_MANEUVER_TIMEOUT_FRAMES:
        return {
            "direction": direction,
            "activeFrames": active_frames,
            "timedOut": False,
            "searchDirection": None,
        }
    return {
        "direction": "NENHUMA",
        "activeFrames": 0,
        "timedOut": True,
        "searchDirection": None,
    }


class VirtualTurnStateTracker:
    """Mantém somente o pivot temporário usado para recuperar a linha."""

    def __init__(self):
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
        """Consome no máximo três frames de scan enquanto permanece NORMAL."""

        if direction is None or self.state != VIRTUAL_STATE_NORMAL:
            return False
        if self.medium_scan_frames >= VIRTUAL_MEDIUM_SCAN_MAX_FRAMES:
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
                    >= VIRTUAL_REORIENT_RECOVERY_FRAMES
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

        direction = virtual_reorient_direction(sensors)
        if direction is None:
            self.reorient_candidate = None
            self.reorient_frames = 0
            return self.state

        if direction == self.reorient_candidate:
            self.reorient_frames += 1
        else:
            self.reorient_candidate = direction
            self.reorient_frames = 1

        if self.reorient_frames >= VIRTUAL_REORIENT_CONFIRMATION_FRAMES:
            self.state = (
                VIRTUAL_STATE_REORIENT_LEFT
                if direction == "LEFT"
                else VIRTUAL_STATE_REORIENT_RIGHT
            )
            self.medium_scan_frames = 0
        return self.state


class VirtualPivotStateTracker:
    """Mantém o lado do pivot normal e aplica histerese sem troca direta."""

    def __init__(self):
        self.state = PIVOT_STATE_NONE

    def reset(self):
        """Encerra o pivot persistente antes de outro modo assumir."""

        self.state = PIVOT_STATE_NONE
        return self.state

    def update(self, steering_error):
        """Atualiza a entrada ou saída do pivot usando o erro do frame atual."""

        steering_error = finite_virtual_position(steering_error)
        if steering_error is None:
            return self.reset()

        steering_magnitude = abs(steering_error)
        if self.state == PIVOT_STATE_RIGHT:
            if (
                steering_error <= 0.0
                or steering_magnitude <= PIVOT_EXIT_THRESHOLD
            ):
                return self.reset()
            return self.state

        if self.state == PIVOT_STATE_LEFT:
            if (
                steering_error >= 0.0
                or steering_magnitude <= PIVOT_EXIT_THRESHOLD
            ):
                return self.reset()
            return self.state

        if steering_error >= PIVOT_ENTER_THRESHOLD:
            self.state = PIVOT_STATE_RIGHT
        elif steering_error <= -PIVOT_ENTER_THRESHOLD:
            self.state = PIVOT_STATE_LEFT
        return self.state


class VirtualMediumSpinTracker:
    """Mantém os SPINs do MEDIUM e a direção persistente do hard corner."""

    def __init__(self):
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
                <= VIRTUAL_HARD_CORNER_NEAR_RECOVERY_THRESHOLD
                and medium_position is not None
                and abs(medium_position)
                <= VIRTUAL_HARD_CORNER_MEDIUM_RECOVERY_THRESHOLD
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
                >= VIRTUAL_HARD_CORNER_RECOVERY_FRAMES
            )
            hard_corner_far_recovered = (
                self.hard_corner_far_recovery_frames
                >= VIRTUAL_HARD_CORNER_FAR_RECOVERY_FRAMES
            )
            hard_corner_timed_out = (
                self.hard_corner_frames >= VIRTUAL_HARD_CORNER_MAX_FRAMES
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
            or abs(medium_position)
            < VIRTUAL_MEDIUM_DIRECTION_LOST_THRESHOLD
        )
        if medium_direction_lost:
            self.state = PIVOT_STATE_NONE
            if near_position_valid or self.critical_state == PIVOT_STATE_NONE:
                return self.reset()
            if (
                self.invalid_frames
                >= VIRTUAL_MEDIUM_CRITICAL_INVALID_MAX_FRAMES
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
            if medium_magnitude >= VIRTUAL_MEDIUM_SPIN_ENTER_THRESHOLD:
                self.critical_state = medium_state
            else:
                self.critical_state = PIVOT_STATE_NONE
            return self.state

        if self.state != PIVOT_STATE_NONE:
            if (
                medium_magnitude <= VIRTUAL_MEDIUM_SPIN_EXIT_THRESHOLD
                or medium_state != self.state
            ):
                return self.reset()
            return self.state

        if medium_magnitude >= VIRTUAL_MEDIUM_SPIN_ENTER_THRESHOLD:
            self.state = medium_state
            self.critical_state = medium_state
        else:
            self.critical_state = PIVOT_STATE_NONE
        return self.state


def calculate_line_follower_command(
    processed_line_mask,
    green_detection_result,
    direcao_verde_ativa="NENHUMA",
    gap_forward_active=False,
    virtual_turn_tracker=None,
    pivot_state_tracker=None,
    medium_spin_tracker=None,
    virtual_sensors=None,
    line_search_tracker=None,
    blind_search_requested=False,
    sensor_recovery_requested=False,
    fusion_style_line=None,
    curva_verde_iniciada=False,
    blind_search_preferred_direction=None,
    local_line_lost=False,
    gap_fusion_reacquire_active=False,
    green_active_frames=0,
):
    """
    Aplica o seguidor virtual validado pela câmera inferior.

    Verde e GAP mantêm prioridade. No modo normal, o ângulo Fusion chega ao
    mapper primeiro e o steering virtual permanece como fallback imediato.
    """

    _ = green_detection_result

    fusion_control_status = calculate_fusion_control_status(
        fusion_style_line,
        allow_distant_reacquisition=gap_fusion_reacquire_active,
        allow_lateral_continuation=(
            direcao_verde_ativa == "NENHUMA"
            and not gap_forward_active
            and not gap_fusion_reacquire_active
        ),
    )
    fusion_steering_error = fusion_control_status["fusionSteeringError"]

    sensors = (
        virtual_sensors
        if isinstance(virtual_sensors, dict)
        else read_virtual_line_sensors(
            processed_line_mask,
            direcao_verde_ativa,
            curva_verde_iniciada,
        )
    )
    far_trusted = virtual_sensor_trust_is_active(sensors, "farTrusted")
    medium_trusted = virtual_sensor_trust_is_active(
        sensors,
        "mediumTrusted",
    )
    trusted_far_position = (
        finite_virtual_position(sensors.get("farPosition"))
        if far_trusted
        else None
    )
    trusted_medium_position = (
        finite_virtual_position(sensors.get("mediumPosition"))
        if medium_trusted
        else None
    )
    trusted_far_band_position = (
        finite_virtual_position(sensors.get("farBandPosition"))
        if far_trusted
        else None
    )
    observed_recovery_direction = virtual_recovery_sensor_direction(sensors)
    raw_line_visible = virtual_raw_line_is_visible(sensors)
    if line_search_tracker is not None:
        line_search_tracker.remember(observed_recovery_direction)

    protected_virtual_steering = sensors["steeringError"]
    protected_heading_angle = sensors.get("headingAngle")
    directional_green_active = direcao_verde_ativa in (
        "ESQUERDA",
        "DIREITA",
    )
    if not directional_green_active and not (far_trusted and medium_trusted):
        # Recalcula o caminho de controle quando uma fileira perde trust. Isso
        # impede que um heading previamente calculado carregue o candidato vetado.
        trusted_near_fine_position = (
            finite_virtual_position(sensors.get("nearFinePosition"))
            if virtual_sensor_is_active(sensors.get("nearCenter"))
            else None
        )
        geometry = resolve_virtual_sensor_geometry(
            processed_line_mask.shape
        )
        protected_heading_angle = calculate_virtual_heading_angle(
            trusted_far_position,
            trusted_near_fine_position,
            geometry,
            medium_position=trusted_medium_position,
        )
        protected_virtual_steering = calculate_virtual_steering_error(
            trusted_near_fine_position is not None,
            protected_heading_angle,
            fallback_medium_position=(
                trusted_medium_position
                if trusted_near_fine_position is None
                else None
            ),
            fallback_far_position=(
                trusted_far_position
                if (
                    trusted_near_fine_position is None
                    and direcao_verde_ativa == "NENHUMA"
                )
                else None
            ),
        )
    virtual_state = VIRTUAL_STATE_NORMAL
    medium_scan_direction = None
    direct_recovery_direction = None
    normal_steering_mapper = False
    medium_spin_state = PIVOT_STATE_NONE
    medium_strong_requested = False
    medium_pivot_requested = False
    medium_hard_corner_spin_requested = False
    blind_search_backup_requested = False
    fine_correction = 0.0
    line_state = "LINE"

    if directional_green_active:
        line_state = "GREEN"
        if virtual_turn_tracker is not None:
            virtual_turn_tracker.reset()
        if line_search_tracker is not None:
            line_search_tracker.stop()
        fusion_branch_ready = (
            fusion_steering_error is not None
            and (
                (
                    direcao_verde_ativa == "DIREITA"
                    and fusion_steering_error
                    >= GREEN_ENTRY_FUSION_MIN_STEERING
                )
                or (
                    direcao_verde_ativa == "ESQUERDA"
                    and fusion_steering_error
                    <= -GREEN_ENTRY_FUSION_MIN_STEERING
                )
            )
        )
        if (
            not curva_verde_iniciada
            and int(green_active_frames) <= GREEN_ENTRY_PIVOT_MAX_FRAMES
            and not fusion_branch_ready
        ):
            # O marcador confirmado deve realmente iniciar a entrada no ramo.
            # O pivô termina quando o FAR encontra a continuação escolhida ou
            # quando a janela curta expira, evitando uma rotação excessiva.
            steering_error = (
                -1.0 if direcao_verde_ativa == "ESQUERDA" else 1.0
            )
            fusion_control_status["fusionControlActive"] = False
            control_source = "green-entry-pivot"
        elif fusion_steering_error is not None:
            steering_error = fusion_steering_error
            fusion_control_status["fusionControlActive"] = True
            control_source = "fusion-green"
        else:
            # A entrada visual continua somente enquanto o limite angular
            # permite; a aplicação libera o Fusion antes de ultrapassar o teto.
            steering_error = (
                -1.0 if direcao_verde_ativa == "ESQUERDA" else 1.0
            )
            control_source = "green-direction-hold"

    elif gap_fusion_reacquire_active:
        line_state = "GAP"
        if virtual_turn_tracker is not None:
            virtual_turn_tracker.reset()
        if line_search_tracker is not None:
            line_search_tracker.stop()
        if fusion_steering_error is not None:
            steering_error = fusion_steering_error
            fusion_control_status["fusionControlActive"] = True
            # O Fusion conserva a direção, mas usa a faixa de potência NORMAL
            # enquanto a fita ainda não alcançou o NEAR.
            normal_steering_mapper = True
            control_source = "fusion-gap-reacquire"
        else:
            # O gate remove este estado quando a geometria deixa de ser válida.
            # Este fallback mantém avanço reto caso isso ocorra no mesmo ciclo.
            steering_error = 0.0
            control_source = "gap-forward"

    elif gap_forward_active:
        line_state = "GAP"
        if virtual_turn_tracker is not None:
            virtual_turn_tracker.reset()
        if line_search_tracker is not None:
            line_search_tracker.stop()
        gap_far_position = (
            trusted_far_band_position
            if trusted_far_band_position is not None
            else trusted_far_position
        )
        if virtual_far_line_is_visible(sensors) and gap_far_position is not None:
            # Durante o vazio local, FAR mantém uma correção contínua e limitada
            # ao mapper NORMAL. As duas rodas seguem para frente; PIVOT, SPIN e
            # a parada de uma roda continuam proibidos dentro do GAP.
            steering_error = calculate_virtual_steering_error(
                False,
                None,
                fallback_far_position=gap_far_position,
            )
            normal_steering_mapper = steering_error is not None
            control_source = (
                "virtual-gap-far"
                if normal_steering_mapper
                else "gap-forward"
            )
        else:
            steering_error = 0.0
            control_source = "gap-forward"

    else:
        # Só LOST confirmado bloqueia fragmentos que cancelariam a busca.
        # Em LINE normal este argumento é falso e o mapper permanece idêntico.
        virtual_normal_steering_valid = protected_virtual_steering is not None and not local_line_lost
        if virtual_turn_tracker is not None and not local_line_lost:
            virtual_state = virtual_turn_tracker.update(
                sensors,
                virtual_normal_steering_valid,
            )

        fusion_can_hold_normal = (
            fusion_steering_error is not None
            and not local_line_lost
            and virtual_state == VIRTUAL_STATE_NORMAL
            and not sensor_recovery_requested
            and (
                line_search_tracker is None
                or not line_search_tracker.active
            )
        )
        if virtual_normal_steering_valid or fusion_can_hold_normal:
            if fusion_steering_error is not None:
                # Um target atual pode sustentar NORMAL durante uma perda curta
                # dos sensores virtuais, mas nunca cancela recovery ou search.
                steering_error = fusion_steering_error
                fusion_control_status["fusionControlActive"] = True
                control_source = "fusion"
            else:
                # O seguidor normal recupera autoridade no primeiro frame válido,
                # mesmo enquanto o tracker confirma a saída do REORIENT.
                steering_error = protected_virtual_steering
                near_fine_position = finite_virtual_position(
                    sensors["nearFinePosition"]
                )
                protected_steering_for_fine = finite_virtual_position(
                    protected_virtual_steering
                )
                if (
                    virtual_sensor_is_active(sensors["nearCenter"])
                    and near_fine_position is not None
                    and protected_steering_for_fine is not None
                    and abs(protected_steering_for_fine)
                    <= NORMAL_FULL_STEERING_ERROR
                ):
                    fine = apply_virtual_fine_center_deadband(
                        near_fine_position
                    )
                    fine_correction = max(
                        -VIRTUAL_FINE_CENTER_MAX_CORRECTION,
                        min(
                            VIRTUAL_FINE_CENTER_MAX_CORRECTION,
                            fine * VIRTUAL_FINE_CENTER_GAIN,
                        ),
                    )
                    steering_error = max(
                        -NORMAL_FULL_STEERING_ERROR,
                        min(
                            NORMAL_FULL_STEERING_ERROR,
                            protected_steering_for_fine + fine_correction,
                        ),
                    )
                control_source = "virtual"
            normal_steering_mapper = True
            if line_search_tracker is not None:
                line_search_tracker.stop()
        else:
            if virtual_state == VIRTUAL_STATE_REORIENT_LEFT:
                steering_error = -1.0
                control_source = "virtual-reorient"
            elif virtual_state == VIRTUAL_STATE_REORIENT_RIGHT:
                steering_error = 1.0
                control_source = "virtual-reorient"
            elif (
                raw_line_visible
                and not local_line_lost
                and line_search_tracker is not None
                and (
                    line_search_tracker.active
                    or sensor_recovery_requested
                )
            ):
                line_search_tracker.stop()
                if observed_recovery_direction is None:
                    steering_error = 0.0
                else:
                    steering_error = None
                    direct_recovery_direction = observed_recovery_direction
                control_source = "virtual-sensor-recovery"
            elif (
                line_search_tracker is not None
                and line_search_tracker.active
            ):
                blind_direction = line_search_tracker.next_direction()
                if blind_direction == "BACKWARD":
                    steering_error = None
                    blind_search_backup_requested = True
                    control_source = "virtual-blind-search-backup"
                else:
                    steering_error = (
                        -1.0 if blind_direction == "LEFT" else 1.0
                    )
                    control_source = "virtual-blind-search"
            else:
                steering_error = None
                observed_medium_direction = virtual_medium_scan_direction(
                    sensors
                )
                if (
                    virtual_turn_tracker is not None
                    and virtual_turn_tracker.allow_medium_scan(
                        observed_medium_direction
                    )
                ):
                    medium_scan_direction = observed_medium_direction
                    control_source = "virtual-medium-scan"
                else:
                    if (
                        observed_recovery_direction is None
                        and not raw_line_visible
                        and line_search_tracker is not None
                    ):
                        if line_search_tracker.request_automatic_start(
                            blind_search_preferred_direction
                        ):
                            blind_direction = (
                                line_search_tracker.next_direction()
                            )
                            if blind_direction == "BACKWARD":
                                steering_error = None
                                blind_search_backup_requested = True
                                control_source = (
                                    "virtual-blind-search-backup"
                                )
                            else:
                                steering_error = (
                                    -1.0
                                    if blind_direction == "LEFT"
                                    else 1.0
                                )
                                control_source = "virtual-blind-search"
                        else:
                            control_source = "virtual-search-wait"
                    else:
                        control_source = "virtual-no-line"

    medium_position = trusted_medium_position
    near_fine_position = finite_virtual_position(
        sensors["nearFinePosition"]
    )
    near_position_valid = (
        virtual_sensor_is_active(sensors["nearCenter"])
        and near_fine_position is not None
    )
    critical_entry_allowed = (
        normal_steering_mapper and control_source == "virtual"
    )
    medium_hard_corner_spin_requested = (
        line_state == "LINE"
        and critical_entry_allowed
        and medium_position is not None
        and abs(medium_position) >= VIRTUAL_MEDIUM_SPIN_ENTER_THRESHOLD
        and near_position_valid
        and abs(near_fine_position) >= VIRTUAL_MEDIUM_STRONG_THRESHOLD
        and medium_position * near_fine_position > 0.0
        and trusted_far_position is None
    )
    if line_state == "LINE" and medium_spin_tracker is not None:
        medium_spin_state = medium_spin_tracker.update(
            medium_position,
            near_position_valid,
            critical_entry_allowed=critical_entry_allowed,
            hard_corner_entry_requested=(
                medium_hard_corner_spin_requested
            ),
            near_fine_position=near_fine_position,
            far_position=trusted_far_position,
        )
    elif line_state != "LINE" and medium_spin_tracker is not None:
        medium_spin_tracker.reset()
    elif medium_hard_corner_spin_requested:
        # Sem tracker não há memória entre frames, mas a chamada isolada ainda
        # preserva o comando seguro correspondente ao gatilho atual.
        medium_spin_state = (
            PIVOT_STATE_RIGHT
            if medium_position > 0.0
            else PIVOT_STATE_LEFT
        )

    if (
        fusion_control_status["fusionControlActive"]
        and medium_spin_state != PIVOT_STATE_NONE
    ):
        # Uma manobra crítica já iniciada pelo controle legado mantém prioridade
        # até seu critério original de saída. O Fusion continua só na telemetria.
        fusion_control_status["fusionControlActive"] = False
        control_source = "virtual"

    if normal_steering_mapper and control_source == "virtual":
        medium_strong_requested = (
            medium_position is not None
            and abs(medium_position) >= VIRTUAL_MEDIUM_STRONG_THRESHOLD
        )
        if not medium_strong_requested:
            # Sem autoridade local do MEDIUM, FAR e heading permanecem no
            # intervalo NORMAL e não alcançam a faixa STRONG do mapper.
            current_steering = finite_virtual_position(steering_error)
            if current_steering is not None:
                steering_error = max(
                    -NORMAL_FULL_STEERING_ERROR,
                    min(NORMAL_FULL_STEERING_ERROR, current_steering),
                )

        if medium_spin_tracker is None and (
            medium_position is not None
            and not near_position_valid
            and abs(medium_position)
            >= VIRTUAL_MEDIUM_SPIN_ENTER_THRESHOLD
        ):
            medium_spin_state = (
                PIVOT_STATE_RIGHT
                if medium_position > 0.0
                else PIVOT_STATE_LEFT
            )

        if (
            medium_spin_state == PIVOT_STATE_NONE
            and medium_strong_requested
        ):
            medium_pivot_requested = (
                near_position_valid
                and abs(medium_position)
                >= VIRTUAL_MEDIUM_SPIN_ENTER_THRESHOLD
            )
            # A promoção adota o lado local do MEDIUM. Somente a leitura
            # crítica com NEAR válido recebe magnitude suficiente para PIVOT.
            current_steering = finite_virtual_position(steering_error)
            current_magnitude = (
                abs(current_steering)
                if current_steering is not None
                else 0.0
            )
            minimum_magnitude = (
                PIVOT_ENTER_THRESHOLD
                if medium_pivot_requested
                else NORMAL_FULL_STEERING_ERROR
            )
            promoted_magnitude = max(
                minimum_magnitude,
                current_magnitude,
            )
            steering_error = (
                promoted_magnitude
                if medium_position > 0.0
                else -promoted_magnitude
            )
    # --------------------------------------------------------
    # CONTROLE DE MOTORES
    # --------------------------------------------------------

    BASE_POWER = NORMAL_BASE_POWER
    MAX_POWER = NORMAL_MAX_POWER

    # A faixa forte amplia a curva sem reduzir nenhuma roda abaixo de 0,61.
    STRONG_TURN_OUTER_POWER = 0.85
    STRONG_TURN_INNER_MIN_POWER = 0.61

    # Verde, GAP e recoveries preservam o limite usado antes da histerese.
    NON_NORMAL_PIVOT_THRESHOLD = 0.40

    pivot_state = PIVOT_STATE_NONE
    if medium_spin_state != PIVOT_STATE_NONE:
        if pivot_state_tracker is not None:
            pivot_state_tracker.reset()
    elif normal_steering_mapper:
        if pivot_state_tracker is not None:
            # Somente MEDIUM libera STRONG. A leitura crítica com NEAR válido
            # é a única que também autoriza PIVOT neste frame.
            pivot_state_tracker.reset()
        if medium_pivot_requested:
            if pivot_state_tracker is not None:
                pivot_state = pivot_state_tracker.update(steering_error)
            else:
                pivot_state = (
                    PIVOT_STATE_RIGHT
                    if medium_position > 0.0
                    else PIVOT_STATE_LEFT
                )
    else:
        if pivot_state_tracker is not None:
            pivot_state_tracker.reset()
        if steering_error is not None:
            if steering_error >= NON_NORMAL_PIVOT_THRESHOLD:
                pivot_state = PIVOT_STATE_RIGHT
            elif steering_error <= -NON_NORMAL_PIVOT_THRESHOLD:
                pivot_state = PIVOT_STATE_LEFT

    if steering_error is None:
        left_power = 0.0
        right_power = 0.0

    elif pivot_state == PIVOT_STATE_RIGHT:
        # PIVOT para DIREITA. A roda esquerda avança um pouco mais forte que
        # a direita recua, preservando uma pequena componente de avanço.
        left_power = PIVOT_OUTER_POWER
        right_power = PIVOT_INNER_POWER

    elif pivot_state == PIVOT_STATE_LEFT:
        # PIVOT para ESQUERDA. A roda direita avança um pouco mais forte que
        # a esquerda recua, preservando uma pequena componente de avanço.
        left_power = PIVOT_INNER_POWER
        right_power = PIVOT_OUTER_POWER

    elif (
        fusion_control_status["fusionControlActive"]
        and line_state == "GAP"
    ):
        # Antes do NEAR reaparecer, o Fusion escolhe a direção, mas não pode
        # transformar a travessia em pivot. Isso preserva avanço suficiente
        # mesmo quando FAR e MEDIUM ocupam lados opostos da imagem.
        normal_command = map_normal_steering_error(steering_error)
        if normal_command is None:
            left_power = 0.0
            right_power = 0.0
        else:
            left_power = normal_command["left_power"]
            right_power = normal_command["right_power"]

    elif fusion_control_status["fusionControlActive"]:
        # O Fusion permanece no seguimento LINE e varia continuamente a potência
        # até o pivot; não cria estado, latch ou novo gatilho de hard corner.
        fusion_command = map_fusion_angle_to_motor_powers(
            fusion_control_status["fusionAngle"]
        )
        fusion_command = apply_fusion_forward_speed_limit(
            fusion_command,
            fusion_control_status["fusionSpeedScale"],
        )
        if fusion_command is None:
            left_power = 0.0
            right_power = 0.0
        else:
            left_power = fusion_command["left_power"]
            right_power = fusion_command["right_power"]

    elif normal_steering_mapper and not medium_strong_requested:
        # O caminho normal e a câmera frontal chamam exatamente o mesmo mapper.
        normal_command = map_normal_steering_error(steering_error)
        if normal_command is None:
            left_power = 0.0
            right_power = 0.0
        else:
            left_power = normal_command["left_power"]
            right_power = normal_command["right_power"]

    else:
        # Correção normal.
        steering_magnitude = abs(steering_error)
        if (
            normal_steering_mapper
            and medium_strong_requested
            and steering_magnitude >= NORMAL_FULL_STEERING_ERROR
        ):
            # A progressão quadrática suaviza o início da faixa forte sem
            # impedir que o diferencial se aproxime do máximo antes do pivot.
            transition_progress = (
                steering_magnitude - NORMAL_FULL_STEERING_ERROR
            ) / (
                PIVOT_ENTER_THRESHOLD - NORMAL_FULL_STEERING_ERROR
            )
            transition_progress = max(
                0.0,
                min(1.0, transition_progress),
            )
            transition_progress *= transition_progress
            outer_power = (
                MAX_POWER
                + transition_progress
                * (STRONG_TURN_OUTER_POWER - MAX_POWER)
            )
            inner_power = (
                NORMAL_INNER_MIN_POWER
                - transition_progress
                * (
                    NORMAL_INNER_MIN_POWER
                    - STRONG_TURN_INNER_MIN_POWER
                )
            )
        else:
            full_steering_error = (
                NORMAL_FULL_STEERING_ERROR
                if normal_steering_mapper
                else NON_NORMAL_PIVOT_THRESHOLD
            )
            steering_strength = min(
                1.0,
                steering_magnitude / full_steering_error,
            )

            outer_power = (
                BASE_POWER
                + steering_strength
                * (MAX_POWER - BASE_POWER)
            )
            inner_power = (
                BASE_POWER
                - steering_strength
                * (BASE_POWER - NORMAL_INNER_MIN_POWER)
            )

        if steering_error > 0.0:
            # Curva para DIREITA.
            left_power = outer_power
            right_power = inner_power

        else:
            # Curva para ESQUERDA.
            left_power = inner_power
            right_power = outer_power

    # O scan não cria steering nem passa pelo mapper. Ele gira lentamente
    # usando somente a potência base e é reavaliado no próximo frame.
    if medium_scan_direction == "LEFT":
        left_power = 0.0
        right_power = BASE_POWER
    elif medium_scan_direction == "RIGHT":
        left_power = BASE_POWER
        right_power = 0.0
    elif direct_recovery_direction == "LEFT":
        left_power = 0.0
        right_power = BASE_POWER
    elif direct_recovery_direction == "RIGHT":
        left_power = BASE_POWER
        right_power = 0.0

    # O SPIN é uma promoção exclusiva do LINE normal. A roda em ré recebe
    # diretamente a potência funcional solicitada, sem passar pelo mapper.
    if medium_spin_state == PIVOT_STATE_LEFT:
        left_power = -VIRTUAL_MEDIUM_SPIN_POWER
        right_power = VIRTUAL_MEDIUM_SPIN_POWER
    elif medium_spin_state == PIVOT_STATE_RIGHT:
        left_power = VIRTUAL_MEDIUM_SPIN_POWER
        right_power = -VIRTUAL_MEDIUM_SPIN_POWER

    if blind_search_backup_requested:
        # A ré antecede apenas a busca automática e nunca altera o sentido da
        # varredura que começará no frame seguinte ao término desta janela.
        left_power = VIRTUAL_BLIND_SEARCH_BACKUP_POWER
        right_power = VIRTUAL_BLIND_SEARCH_BACKUP_POWER

    return {
        "left_power": left_power,
        "right_power": right_power,

        "farLeft": sensors["farLeft"],
        "farCenter": sensors["farCenter"],
        "farRight": sensors["farRight"],
        "controlFarLeft": (
            sensors.get("controlFarLeft", sensors["farLeft"])
            if far_trusted
            else 0.0
        ),
        "controlFarCenter": (
            sensors.get("controlFarCenter", sensors["farCenter"])
            if far_trusted
            else 0.0
        ),
        "controlFarRight": (
            sensors.get("controlFarRight", sensors["farRight"])
            if far_trusted
            else 0.0
        ),
        "farPosition": trusted_far_position,
        "rawFarPosition": sensors.get(
            "rawFarPosition",
            sensors["farPosition"],
        ),
        "farLineConfidence": normalized_line_confidence(
            sensors.get("farLineConfidence")
        ),
        "farLineThicknessPx": non_negative_line_measurement(
            sensors.get("farLineThicknessPx")
        ),
        "farThicknessConsistency": normalized_line_confidence(
            sensors.get("farThicknessConsistency")
        ),
        "farTrusted": far_trusted,

        "farBandLeft": sensors["farBandLeft"],
        "farBandCenter": sensors["farBandCenter"],
        "farBandRight": sensors["farBandRight"],
        "controlFarBandLeft": (
            sensors.get("controlFarBandLeft", sensors["farBandLeft"])
            if far_trusted
            else 0.0
        ),
        "controlFarBandCenter": (
            sensors.get("controlFarBandCenter", sensors["farBandCenter"])
            if far_trusted
            else 0.0
        ),
        "controlFarBandRight": (
            sensors.get("controlFarBandRight", sensors["farBandRight"])
            if far_trusted
            else 0.0
        ),
        "farBandPosition": trusted_far_band_position,
        "rawFarBandPosition": sensors.get(
            "rawFarBandPosition",
            sensors["farBandPosition"],
        ),

        "mediumLeft": sensors["mediumLeft"],
        "mediumCenter": sensors["mediumCenter"],
        "mediumRight": sensors["mediumRight"],
        "controlMediumLeft": (
            sensors.get("controlMediumLeft", sensors["mediumLeft"])
            if medium_trusted
            else 0.0
        ),
        "controlMediumCenter": (
            sensors.get("controlMediumCenter", sensors["mediumCenter"])
            if medium_trusted
            else 0.0
        ),
        "controlMediumRight": (
            sensors.get("controlMediumRight", sensors["mediumRight"])
            if medium_trusted
            else 0.0
        ),
        "mediumPosition": trusted_medium_position,
        "rawMediumPosition": sensors.get(
            "rawMediumPosition",
            sensors["mediumPosition"],
        ),
        "mediumLineConfidence": normalized_line_confidence(
            sensors.get("mediumLineConfidence")
        ),
        "mediumLineThicknessPx": non_negative_line_measurement(
            sensors.get("mediumLineThicknessPx")
        ),
        "mediumThicknessConsistency": normalized_line_confidence(
            sensors.get("mediumThicknessConsistency")
        ),
        "mediumTrusted": medium_trusted,

        "nearCenter": sensors["nearCenter"],
        "nearFinePosition": sensors["nearFinePosition"],

        "headingAngle": protected_heading_angle,
        "fineCorrection": fine_correction,
        "steeringError": steering_error,
        "finalSteering": steering_error,
        "virtualState": virtual_state,
        "lineState": line_state,
        "greenDirection": direcao_verde_ativa,
        "controlSource": control_source,
        "fusionAngle": fusion_control_status["fusionAngle"],
        "filteredFusionAngle": fusion_control_status[
            "filteredFusionAngle"
        ],
        "fusionSteeringError": fusion_control_status[
            "fusionSteeringError"
        ],
        "fusionControlActive": fusion_control_status[
            "fusionControlActive"
        ],
        "fusionPreferredDirection": (
            str(fusion_style_line.get("preferredDirection", "NONE"))
            if isinstance(fusion_style_line, dict)
            else "NONE"
        ),
        "fusionTargetLengthRatio": fusion_control_status[
            "fusionTargetLengthRatio"
        ],
        "fusionTargetConsistency": fusion_control_status[
            "fusionTargetConsistency"
        ],
        "fusionTargetStableFrames": fusion_control_status[
            "fusionTargetStableFrames"
        ],
        "fusionTargetReacquired": fusion_control_status[
            "fusionTargetReacquired"
        ],
        "fusionLateralContinuation": fusion_control_status[
            "fusionLateralContinuation"
        ],
        "fusionSpeedScale": fusion_control_status["fusionSpeedScale"],
        # Reaproveita a direção já escolhida pelo recovery com MEDIUM/FAR
        # trusted; a câmera frontal nunca calcula LEFT ou RIGHT.
        "trustedDirection": observed_recovery_direction or "NONE",
    }


class LineFollowerController:
    """Mantém juntos os trackers temporais usados pelo controle da linha."""

    def __init__(self):
        self.virtual_turn_tracker = VirtualTurnStateTracker()
        self.pivot_state_tracker = VirtualPivotStateTracker()
        self.medium_spin_tracker = VirtualMediumSpinTracker()
        self.line_search_tracker = VirtualLineSearchTracker()

    def calculate(
        self,
        processed_line_mask,
        green_detection_result,
        direcao_verde_ativa="NENHUMA",
        gap_forward_active=False,
        virtual_sensors=None,
        blind_search_requested=False,
        sensor_recovery_requested=False,
        fusion_style_line=None,
        curva_verde_iniciada=False,
        blind_search_preferred_direction=None,
        local_line_lost=False,
        gap_fusion_reacquire_active=False,
        green_active_frames=0,
    ):
        """Calcula o comando reutilizando o mesmo estado entre frames."""

        return calculate_line_follower_command(
            processed_line_mask,
            green_detection_result,
            direcao_verde_ativa,
            gap_forward_active,
            virtual_turn_tracker=self.virtual_turn_tracker,
            pivot_state_tracker=self.pivot_state_tracker,
            medium_spin_tracker=self.medium_spin_tracker,
            virtual_sensors=virtual_sensors,
            line_search_tracker=self.line_search_tracker,
            blind_search_requested=blind_search_requested,
            sensor_recovery_requested=sensor_recovery_requested,
            fusion_style_line=fusion_style_line,
            curva_verde_iniciada=curva_verde_iniciada,
            blind_search_preferred_direction=blind_search_preferred_direction,
            local_line_lost=local_line_lost,
            gap_fusion_reacquire_active=gap_fusion_reacquire_active,
            green_active_frames=green_active_frames,
        )
