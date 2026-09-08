#include "obr/forward_line_assist.h"

#include "obr/config.h"
#include "obr/imu_turn_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

void ForwardLineAssist::reset()
{
    state_ = State::Bottom;
    direction_ = Direction::None;
    yawOriginDegrees_ = 0.0;
    yawDeltaDegrees_ = 0.0;
    bottomStableFrames_ = 0;
    bottomLostFrames_ = 0;
    hasPreviousBottomFrame_ = false;
    previousBottomSequence_ = 0;
    previousBottomTrusted_ = false;
    previousBottomLineNormal_ = false;
    latchedBottomDirection_ = Direction::None;
    mediumFlipCandidateDirection_ = Direction::None;
    mediumFlipConfirmationFrames_ = 0;
    farTrusted_ = false;
    mediumTrusted_ = false;
    gapCandidate_ = false;
    entryAllowed_ = false;
    entryBlocker_ = "WAITING_TRUST";
}

void ForwardLineAssist::updateLatchedBottomDirection(
    Direction farDirection,
    Direction mediumDirection,
    bool consecutiveBottomFrame)
{
    const bool directionsConflict =
        farDirection != Direction::None &&
        mediumDirection != Direction::None &&
        farDirection != mediumDirection;
    if (directionsConflict &&
        latchedBottomDirection_ != Direction::None)
    {
        // Um conflito instantâneo não pode inverter uma curva já memorizada.
        mediumFlipCandidateDirection_ = Direction::None;
        mediumFlipConfirmationFrames_ = 0;
        return;
    }

    if (farDirection != Direction::None)
    {
        // FAR representa melhor a continuação futura e atualiza o lado sem atraso.
        latchedBottomDirection_ = farDirection;
        mediumFlipCandidateDirection_ = Direction::None;
        mediumFlipConfirmationFrames_ = 0;
        return;
    }

    if (mediumDirection == Direction::None)
    {
        // NONE interrompe apenas uma confirmação de inversão; o latch permanece.
        mediumFlipCandidateDirection_ = Direction::None;
        mediumFlipConfirmationFrames_ = 0;
        return;
    }

    if (latchedBottomDirection_ == Direction::None ||
        latchedBottomDirection_ == mediumDirection)
    {
        latchedBottomDirection_ = mediumDirection;
        mediumFlipCandidateDirection_ = Direction::None;
        mediumFlipConfirmationFrames_ = 0;
        return;
    }

    if (mediumFlipCandidateDirection_ != mediumDirection ||
        !consecutiveBottomFrame)
    {
        mediumFlipCandidateDirection_ = mediumDirection;
        mediumFlipConfirmationFrames_ = 1;
        return;
    }

    ++mediumFlipConfirmationFrames_;
    if (mediumFlipConfirmationFrames_ >=
        config::kForwardAssistMediumFlipConfirmationFrames)
    {
        latchedBottomDirection_ = mediumDirection;
        mediumFlipCandidateDirection_ = Direction::None;
        mediumFlipConfirmationFrames_ = 0;
    }
}

AutonomousStatus ForwardLineAssist::status(
    const std::string& phase,
    const std::string& action,
    const ForwardLineSnapshot& forwardLineSnapshot) const
{
    AutonomousStatus status;
    status.phase = phase;
    status.action = action;
    switch (state_)
    {
    case State::SearchSpin:
        status.forwardAssistState = "SEARCH_SPIN";
        break;
    case State::ForwardFollow:
        status.forwardAssistState = "FORWARD_FOLLOW";
        break;
    case State::Bottom:
    default:
        status.forwardAssistState = "BOTTOM";
        break;
    }
    switch (direction_)
    {
    case Direction::Left:
        status.forwardAssistDirection = "LEFT";
        break;
    case Direction::Right:
        status.forwardAssistDirection = "RIGHT";
        break;
    case Direction::None:
    default:
        status.forwardAssistDirection = "NONE";
        break;
    }
    switch (latchedBottomDirection_)
    {
    case Direction::Left:
        status.forwardAssistLatchedDirection = "LEFT";
        break;
    case Direction::Right:
        status.forwardAssistLatchedDirection = "RIGHT";
        break;
    case Direction::None:
    default:
        status.forwardAssistLatchedDirection = "NONE";
        break;
    }
    status.forwardAssistFarTrusted = farTrusted_;
    status.forwardAssistMediumTrusted = mediumTrusted_;
    status.forwardAssistGapCandidate = gapCandidate_;
    status.forwardAssistEntryAllowed = entryAllowed_;
    status.forwardAssistEntryBlocker = entryBlocker_;
    status.forwardAssistYawDeltaDeg = yawDeltaDegrees_;
    status.forwardLineVisible =
        forwardLineSnapshot.lineObservationValid();
    status.forwardLinePosition = status.forwardLineVisible
                                     ? std::clamp(
                                           forwardLineSnapshot.position,
                                           -1.0,
                                           1.0)
                                     : 0.0;
    status.bottomStableFrames = bottomStableFrames_;
    return status;
}

bool ForwardLineAssist::update(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    const CameraLineSnapshot& cameraLineSnapshot,
    const ForwardLineSnapshot& forwardLineSnapshot)
{
    farTrusted_ = cameraLineSnapshot.farTrusted;
    mediumTrusted_ = cameraLineSnapshot.mediumTrusted;
    gapCandidate_ =
        cameraLineSnapshot.curveDiagnostics.lineState == "GAP";
    const bool bottomLineNormal =
        cameraLineSnapshot.curveDiagnostics.lineState == "LINE" &&
        cameraLineSnapshot.curveDiagnostics.virtualState == "NORMAL";
    const bool searchOwnsRecovery =
        state_ == State::SearchSpin;
    const bool greenHasPriority =
        cameraLineSnapshot.curveDiagnostics.lineState == "GREEN";
    if (!bottomLineNormal && (!searchOwnsRecovery || greenHasPriority))
    {
        // Antes de uma busca começar, GREEN, GAP e estados inferiores especiais
        // preservam sua autoridade existente. Depois da entrada legítima no
        // SEARCH, somente GREEN pode retirar essa posse neste gate.
        const bool farTrusted = farTrusted_;
        const bool mediumTrusted = mediumTrusted_;
        const bool gapCandidate = gapCandidate_;
        const std::string entryBlocker =
            cameraLineSnapshot.curveDiagnostics.lineState != "LINE"
                ? "LINE_NOT_NORMAL"
                : "VIRTUAL_NOT_NORMAL";
        reset();
        farTrusted_ = farTrusted;
        mediumTrusted_ = mediumTrusted;
        gapCandidate_ = gapCandidate;
        entryBlocker_ = entryBlocker;
        return false;
    }

    const auto directionFromTrustedPosition = [](
        bool trusted,
        double position)
    {
        if (!trusted || !std::isfinite(position))
        {
            return Direction::None;
        }
        if (position <= -config::kForwardAssistDirectionPositionThreshold)
        {
            return Direction::Left;
        }
        if (position >= config::kForwardAssistDirectionPositionThreshold)
        {
            return Direction::Right;
        }
        return Direction::None;
    };
    const Direction farDirection =
        directionFromTrustedPosition(
            cameraLineSnapshot.farTrusted,
            cameraLineSnapshot.curveDiagnostics.farBandPosition);
    const Direction mediumDirection =
        directionFromTrustedPosition(
            cameraLineSnapshot.mediumTrusted,
            cameraLineSnapshot.curveDiagnostics.mediumPosition);

    const bool currentBottomTrusted =
        cameraLineSnapshot.farTrusted || cameraLineSnapshot.mediumTrusted;
    const bool newBottomFrame =
        cameraLineSnapshot.sourceFresh &&
        (!hasPreviousBottomFrame_ ||
         cameraLineSnapshot.lineSequence != previousBottomSequence_);
    const bool consecutiveBottomFrame =
        newBottomFrame && hasPreviousBottomFrame_ &&
        previousBottomSequence_ !=
            std::numeric_limits<std::uint64_t>::max() &&
        cameraLineSnapshot.lineSequence == previousBottomSequence_ + 1;

    const bool previousBottomTrusted = previousBottomTrusted_;
    const bool previousBottomLineNormal = previousBottomLineNormal_;
    const Direction latchedBottomDirection =
        latchedBottomDirection_;

    if (newBottomFrame)
    {
        hasPreviousBottomFrame_ = true;
        previousBottomSequence_ = cameraLineSnapshot.lineSequence;
        previousBottomTrusted_ = currentBottomTrusted;
        previousBottomLineNormal_ = bottomLineNormal;
        if (state_ == State::Bottom)
        {
            updateLatchedBottomDirection(
                farDirection,
                mediumDirection,
                consecutiveBottomFrame);
        }
    }

    const bool bottomRequestsVirtualBlindSearch =
        cameraLineSnapshot.lineControlSource == "virtual-blind-search" ||
        cameraLineSnapshot.lineControlSource ==
            "virtual-blind-search-backup";
    const bool forwardLineCommandValid =
        forwardLineSnapshot.normalCommandValid();
    const double directionThreshold =
        config::kForwardAssistDirectionPositionThreshold;
    const bool forwardLineCoherent =
        forwardLineCommandValid &&
        ((latchedBottomDirection == Direction::None &&
          std::abs(forwardLineSnapshot.position) <= directionThreshold) ||
         (latchedBottomDirection == Direction::Left &&
          forwardLineSnapshot.position <= directionThreshold) ||
         (latchedBottomDirection == Direction::Right &&
          forwardLineSnapshot.position >= -directionThreshold));

    if (bottomRequestsVirtualBlindSearch)
    {
        // A região frontal já corresponde à parte mais próxima da CAM1. Uma
        // continuação coerente assume o avanço; sem ela, a busca inferior pode
        // executar sua ré e varredura. Leitura stale nunca autoriza movimento.
        yawOriginDegrees_ = 0.0;
        yawDeltaDegrees_ = 0.0;
        bottomStableFrames_ = 0;
        bottomLostFrames_ = 0;

        if (!forwardLineSnapshot.sourceFresh)
        {
            state_ = State::Bottom;
            direction_ = Direction::None;
            entryAllowed_ = false;
            entryBlocker_ = "FORWARD_STALE";
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(status(
                "forward_assist_wait",
                "FWD: aguardando leitura frontal fresh",
                forwardLineSnapshot));
            return true;
        }

        if (forwardLineCoherent)
        {
            state_ = State::ForwardFollow;
            direction_ = latchedBottomDirection;
            entryAllowed_ = true;
            entryBlocker_ = "NONE";
            robotState.driveAutonomous(
                forwardLineSnapshot.normalLeftPower,
                forwardLineSnapshot.normalRightPower);
            robotState.updateAutonomousStatus(status(
                "forward_assist_follow",
                "FWD: continuação coerente antes do blind search",
                forwardLineSnapshot));
            return true;
        }

        state_ = State::Bottom;
        direction_ = Direction::None;
        entryAllowed_ = false;
        if (!forwardLineSnapshot.lineObservationValid())
        {
            entryBlocker_ = "FORWARD_LINE_NOT_VISIBLE";
        }
        else if (!forwardLineCommandValid)
        {
            entryBlocker_ = "FORWARD_COMMAND_INVALID";
        }
        else
        {
            entryBlocker_ = "FORWARD_LINE_INCOHERENT";
        }
        return false;
    }

    const bool bottomLostTrustedRows =
        !cameraLineSnapshot.farTrusted &&
        !cameraLineSnapshot.mediumTrusted;
    const bool bottomRequestsExistingCriticalTurn =
        (cameraLineSnapshot.lineFollowerLeftPower < 0.0 ||
         cameraLineSnapshot.lineFollowerRightPower < 0.0) &&
        cameraLineSnapshot.lineControlSource != "virtual-blind-search";
    if (state_ == State::Bottom && newBottomFrame)
    {
        if (!bottomLostTrustedRows || cameraLineSnapshot.normalSteeringValid)
        {
            bottomLostFrames_ = 0;
        }
        else if (bottomLostFrames_ > 0)
        {
            ++bottomLostFrames_;
        }
        else if (previousBottomTrusted && previousBottomLineNormal)
        {
            bottomLostFrames_ = 1;
        }

        entryAllowed_ = false;
        if (!bottomLostTrustedRows)
        {
            entryBlocker_ = cameraLineSnapshot.farTrusted
                                             ? "FAR_TRUSTED"
                                             : "MEDIUM_TRUSTED";
        }
        else if (cameraLineSnapshot.normalSteeringValid)
        {
            // O Fusion ainda possui target e comando LINE válidos, ou o virtual
            // permanece NORMAL; perder FAR/MEDIUM não autoriza SEARCH_SPIN.
            entryBlocker_ = "NORMAL_STEERING_VALID";
        }
        else if (bottomLostFrames_ == 0)
        {
            entryBlocker_ = "NO_PREVIOUS_TRUST";
        }
        else if (latchedBottomDirection == Direction::None)
        {
            entryBlocker_ = "NO_LATCHED_DIRECTION";
        }
        else if (bottomRequestsExistingCriticalTurn)
        {
            bottomLostFrames_ = 0;
            entryBlocker_ = "BOTTOM_CRITICAL_TURN";
        }
        else if (!ImuTurnController::imuReady(esp32Telemetry))
        {
            bottomLostFrames_ = 0;
            entryBlocker_ = "IMU_NOT_READY";
        }
        else if (bottomLostFrames_ < config::kForwardAssistBottomLossFrames)
        {
            entryBlocker_ = "WAITING_LOSS_CONFIRMATION";
        }
        else
        {
            // A direção trusted permanece latched, mas a busca só começa após
            // dois frames de perda. Saltos no sequence não apagam esse lado.
            entryAllowed_ = true;
            entryBlocker_ = "NONE";
            state_ = State::SearchSpin;
            direction_ = latchedBottomDirection;
            yawOriginDegrees_ = esp32Telemetry.yawZDeg;
            yawDeltaDegrees_ = 0.0;
            bottomStableFrames_ = 0;
            bottomLostFrames_ = 0;
        }
    }

    if (state_ == State::Bottom)
    {
        return false;
    }

    const bool imuReady = ImuTurnController::imuReady(esp32Telemetry);
    if (!imuReady ||
        (state_ != State::SearchSpin &&
         bottomRequestsExistingCriticalTurn))
    {
        // A perda da IMU sempre encerra a assistência. HARD CORNER, PIVOT e
        // SPIN inferiores bloqueiam a entrada, mas não expulsam um SEARCH que
        // já possui direção e orçamento angular válidos.
        state_ = State::Bottom;
        direction_ = Direction::None;
        yawDeltaDegrees_ = 0.0;
        bottomStableFrames_ = 0;
        entryAllowed_ = false;
        entryBlocker_ = !imuReady
                                         ? "IMU_NOT_READY"
                                         : "BOTTOM_CRITICAL_TURN";
        return false;
    }

    yawDeltaDegrees_ = ImuTurnController::angularDistanceDegrees(
        yawOriginDegrees_,
        esp32Telemetry.yawZDeg);

    const auto bottomRecoveryConfirmed = [&]()
    {
        if (!newBottomFrame)
        {
            return false;
        }

        const bool bottomRecovered = currentBottomTrusted &&
                                     cameraLineSnapshot.normalSteeringValid;
        if (!bottomRecovered)
        {
            bottomStableFrames_ = 0;
            return false;
        }

        bottomStableFrames_ =
            bottomStableFrames_ > 0 && consecutiveBottomFrame
                ? bottomStableFrames_ + 1
                : 1;
        return bottomStableFrames_ >=
               config::kForwardAssistBottomStableFrames;
    };

    if (state_ == State::ForwardFollow &&
        bottomRecoveryConfirmed())
    {
        state_ = State::Bottom;
        direction_ = Direction::None;
        yawDeltaDegrees_ = 0.0;
        bottomStableFrames_ = 0;
        entryAllowed_ = false;
        entryBlocker_ = cameraLineSnapshot.farTrusted
                                         ? "FAR_TRUSTED"
                                         : "MEDIUM_TRUSTED";
        return false;
    }

    if (state_ == State::ForwardFollow &&
        !forwardLineSnapshot.lineObservationValid())
    {
        // A tentativa continua com a mesma direção, origem e orçamento angular.
        state_ = State::SearchSpin;
    }

    if (state_ == State::SearchSpin)
    {
        if (yawDeltaDegrees_ >=
                config::kForwardAssistMaximumSearchDegrees)
        {
            // O teto angular entrega imediatamente a decisão ao recovery
            // inferior já existente.
            state_ = State::Bottom;
            direction_ = Direction::None;
            yawDeltaDegrees_ = 0.0;
            bottomStableFrames_ = 0;
            entryAllowed_ = false;
            entryBlocker_ = "SEARCH_ANGLE_LIMIT";
            return false;
        }

        if (forwardLineSnapshot.lineObservationValid())
        {
            // Esta transição acontece antes de qualquer comando de SPIN. Assim,
            // o primeiro frame frontal válido já substitui o giro pelo avanço.
            state_ = State::ForwardFollow;
        }
        else if (bottomRecoveryConfirmed())
        {
            // Um flicker trusted conta apenas como 1/2. A bottom recupera a
            // autoridade somente após dois frames novos, normais e consecutivos.
            state_ = State::Bottom;
            direction_ = Direction::None;
            yawDeltaDegrees_ = 0.0;
            bottomStableFrames_ = 0;
            entryAllowed_ = false;
            entryBlocker_ = cameraLineSnapshot.farTrusted
                                             ? "FAR_TRUSTED"
                                             : "MEDIUM_TRUSTED";
            return false;
        }
        else
        {
            const double spinPower =
                config::kForwardAssistSearchSpinPower;
            const bool turnsLeft =
                direction_ == Direction::Left;
            robotState.driveAutonomous(
                turnsLeft ? -spinPower : spinPower,
                turnsLeft ? spinPower : -spinPower);
            robotState.updateAutonomousStatus(status(
                "forward_assist_search",
                turnsLeft
                    ? "FWD: SEARCH LEFT"
                    : "FWD: SEARCH RIGHT",
                forwardLineSnapshot));
            return true;
        }
    }

    if (!forwardLineSnapshot.normalCommandValid())
    {
        // Linha encontrada com comando ausente ou fora do NORMAL não autoriza
        // continuar girando nem avançar com um valor não validado.
        state_ = State::Bottom;
        direction_ = Direction::None;
        yawDeltaDegrees_ = 0.0;
        bottomStableFrames_ = 0;
        entryAllowed_ = false;
        entryBlocker_ = "FORWARD_COMMAND_INVALID";
        return false;
    }

    robotState.driveAutonomous(
        forwardLineSnapshot.normalLeftPower,
        forwardLineSnapshot.normalRightPower);
    robotState.updateAutonomousStatus(status(
        "forward_assist_follow",
        "FWD: FOLLOW " + std::to_string(forwardLineSnapshot.position),
        forwardLineSnapshot));
    return true;
}
