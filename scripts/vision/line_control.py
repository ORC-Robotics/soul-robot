"""Converte sensores virtuais e estados de recuperação em potência."""

import math

from .geometry import (
    GAP_NEAR_HISTORY_FRAMES,
    GEOMETRIC_GAP_FORWARD_MAX_FRAMES,
    GEOMETRIC_GAP_REACQUIRE_FRAMES,
)
from .medium_spin import (
    VIRTUAL_MEDIUM_SPIN_ENTER_THRESHOLD,
    VIRTUAL_MEDIUM_SPIN_POWER,
)
from .numeric import finite_virtual_position
from .pivot import (
    PIVOT_ENTER_THRESHOLD,
    PIVOT_STATE_LEFT,
    PIVOT_STATE_NONE,
    PIVOT_STATE_RIGHT,
)
from .reorient import (
    VIRTUAL_STATE_NORMAL,
    VIRTUAL_STATE_REORIENT_LEFT,
    VIRTUAL_STATE_REORIENT_RIGHT,
    virtual_sensor_trust_is_active,
)
from .virtual_sensors import (
    VIRTUAL_FINE_CENTER_GAIN,
    VIRTUAL_FINE_CENTER_MAX_CORRECTION,
    VIRTUAL_MEDIUM_SCAN_POSITION_THRESHOLD,
    apply_virtual_fine_center_deadband,
    calculate_virtual_heading_angle,
    calculate_virtual_steering_error,
    non_negative_line_measurement,
    normalized_line_confidence,
    read_virtual_line_sensors,
    resolve_virtual_sensor_geometry,
    virtual_sensor_is_active,
)


# A correção normal alcança toda a diferença de potência em 0,36.
# Somente MEDIUM a partir de 0,25 libera a faixa forte entre 0,36 e 0,45.
NORMAL_FULL_STEERING_ERROR = 0.36

# Potências do mapper NORMAL compartilhado pelo seguidor inferior e pela
# publicação frontal. Manter uma única função evita que a câmera auxiliar crie
# outra curva de potência ou alcance PIVOT, SPIN e ré.
NORMAL_BASE_POWER = 0.75
NORMAL_MAX_POWER = 0.82
NORMAL_INNER_MIN_POWER = 0.66

# O MEDIUM promove a urgência da curva somente no seguimento LINE normal.
VIRTUAL_MEDIUM_STRONG_THRESHOLD = 0.25

# Controle da prioridade de direção após um verde confirmado.
QUADROS_PARA_REARMAR_VERDE = 60
LIMIAR_CURVA_VERDE_INICIADA = 0.20
LIMIAR_CENTRALIZACAO_VERDE = 0.18
QUADROS_CENTRALIZADO_PARA_CONCLUIR = 6

# A manobra verde não pode manter a máscara de controle indefinidamente.
GREEN_MANEUVER_TIMEOUT_FRAMES = 12


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


def map_normal_steering_error(steering_error):
    """Converte erro lateral somente no intervalo de autoridade NORMAL."""

    steering_error = finite_virtual_position(steering_error)
    if steering_error is None:
        return None

    limited_error = max(
        -NORMAL_FULL_STEERING_ERROR,
        min(NORMAL_FULL_STEERING_ERROR, steering_error),
    )
    steering_strength = min(
        1.0,
        abs(limited_error) / NORMAL_FULL_STEERING_ERROR,
    )
    outer_power = (
        NORMAL_BASE_POWER
        + steering_strength * (NORMAL_MAX_POWER - NORMAL_BASE_POWER)
    )
    inner_power = (
        NORMAL_BASE_POWER
        - steering_strength
        * (NORMAL_BASE_POWER - NORMAL_INNER_MIN_POWER)
    )
    if limited_error > 0.0:
        return {
            "left_power": outer_power,
            "right_power": inner_power,
        }
    if limited_error < 0.0:
        return {
            "left_power": inner_power,
            "right_power": outer_power,
        }
    return {
        "left_power": NORMAL_BASE_POWER,
        "right_power": NORMAL_BASE_POWER,
    }




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


def update_gap_recent_near_frames(recent_near_frames, near_center_visible):
    """Atualiza a memória curta de uma observação real no NEAR-C."""

    if near_center_visible:
        return GAP_NEAR_HISTORY_FRAMES
    return max(0, int(recent_near_frames) - 1)


def gap_entry_is_required(
    gap_forward_active,
    green_direction,
    recent_near_frames,
    near_center_visible,
    real_near_point,
    virtual_near_point,
    lateral_exit_target,
):
    """Reconhece a perda recente do NEAR sem disputar prioridade com verde."""

    return (
        not gap_forward_active
        and green_direction == "NENHUMA"
        and recent_near_frames > 0
        and not near_center_visible
        and real_near_point is None
        and virtual_near_point is None
        and lateral_exit_target is None
    )


def green_direction_to_search_direction(direction):
    """Converte a direção do GREEN para o vocabulário usado pela busca cega."""

    return {
        "ESQUERDA": "LEFT",
        "DIREITA": "RIGHT",
    }.get(direction)


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
    """Conclui o verde normalmente ou libera sua máscara após o timeout."""

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
        "searchDirection": (
            None
            if raw_line_visible
            else green_direction_to_search_direction(direction)
        ),
    }


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
):
    """
    Aplica o seguidor virtual validado pela câmera inferior.

    Verde e GAP mantêm prioridade. No modo normal, o heading dos sensores à
    frente e a centralização local do NEAR-C chegam ao mapper.
    """

    _ = green_detection_result

    sensors = (
        virtual_sensors
        if isinstance(virtual_sensors, dict)
        else read_virtual_line_sensors(
            processed_line_mask,
            direcao_verde_ativa,
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
    if not (far_trusted and medium_trusted):
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
    fine_correction = 0.0
    line_state = "LINE"

    if direcao_verde_ativa != "NENHUMA":
        line_state = "GREEN"
        if virtual_turn_tracker is not None:
            virtual_turn_tracker.reset()
        if line_search_tracker is not None:
            line_search_tracker.stop()
        green_branch_visible = (
            trusted_far_position is not None
            or trusted_medium_position is not None
        )
        if green_branch_visible and protected_virtual_steering is not None:
            steering_error = protected_virtual_steering
        else:
            # O verde pode ser reconhecido antes de o novo ramo alcançar FAR e
            # MEDIUM, ou apenas uma fileira pode vê-lo sem formar heading. Nesse
            # intervalo, parar abandona a interseção; mantém o pivot confirmado.
            green_search_direction = green_direction_to_search_direction(
                direcao_verde_ativa
            )
            if green_search_direction == "LEFT":
                steering_error = -1.0
            elif green_search_direction == "RIGHT":
                steering_error = 1.0
            else:
                steering_error = None
        control_source = "virtual-green"

    elif gap_forward_active:
        line_state = "GAP"
        if virtual_turn_tracker is not None:
            virtual_turn_tracker.reset()
        if observed_recovery_direction is not None:
            if line_search_tracker is not None:
                line_search_tracker.stop()
            steering_error = None
            direct_recovery_direction = observed_recovery_direction
            control_source = "gap-sensor-recovery"
        elif raw_line_visible:
            # Uma linha de controle encerra a busca cega, mas somente o NEAR-C
            # confirmado pode encerrar o estado GAP fora deste mapper.
            if line_search_tracker is not None:
                line_search_tracker.stop()
            steering_error = 0.0
            control_source = "gap-forward"
        else:
            if line_search_tracker is not None and blind_search_requested:
                line_search_tracker.start()
            blind_direction = (
                line_search_tracker.next_direction()
                if line_search_tracker is not None
                else None
            )
            if blind_direction == "LEFT":
                steering_error = -1.0
                control_source = "gap-blind-search"
            elif blind_direction == "RIGHT":
                steering_error = 1.0
                control_source = "gap-blind-search"
            else:
                steering_error = 0.0
                control_source = "gap-forward"

    else:
        normal_steering_valid = protected_virtual_steering is not None
        if virtual_turn_tracker is not None:
            virtual_state = virtual_turn_tracker.update(
                sensors,
                normal_steering_valid,
            )

        if normal_steering_valid:
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
            normal_steering_mapper = True
            if line_search_tracker is not None:
                line_search_tracker.stop()
            control_source = "virtual"
        else:
            if virtual_state == VIRTUAL_STATE_REORIENT_LEFT:
                steering_error = -1.0
                control_source = "virtual-reorient"
            elif virtual_state == VIRTUAL_STATE_REORIENT_RIGHT:
                steering_error = 1.0
                control_source = "virtual-reorient"
            elif (
                raw_line_visible
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
                steering_error = -1.0 if blind_direction == "LEFT" else 1.0
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
                        line_search_tracker.start()
                        blind_direction = line_search_tracker.next_direction()
                        steering_error = (
                            -1.0 if blind_direction == "LEFT" else 1.0
                        )
                        control_source = "virtual-blind-search"
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

    # Potência durante pivot.
    PIVOT_OUTER_POWER = 0.78
    PIVOT_INNER_POWER = -0.72

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
        # Reaproveita a direção já escolhida pelo recovery com MEDIUM/FAR
        # trusted; a câmera frontal nunca calcula LEFT ou RIGHT.
        "trustedDirection": observed_recovery_direction or "NONE",
    }

