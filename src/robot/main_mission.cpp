#include "obr/main_mission.h"

#include "obr/config.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace
{
AutonomousStatus makeMainMissionStatus(
    const std::string& phase,
    const std::string& action,
    double progressPercent = 0.0)
{
    AutonomousStatus status;
    status.phase = phase;
    status.action = action;
    status.progressPercent = progressPercent;
    return status;
}

AutonomousStatus makeTurnAroundForwardStatus(
    const std::string& phase,
    const std::string& action,
    double leftDistanceCm,
    double rightDistanceCm,
    double progressPercent)
{
    AutonomousStatus status = makeMainMissionStatus(
        phase, action, progressPercent);
    status.targetDistanceCm = config::kGreenTurnAroundForwardDistanceCm;
    status.leftDistanceCm = leftDistanceCm;
    status.rightDistanceCm = rightDistanceCm;
    status.averageDistanceCm = (leftDistanceCm + rightDistanceCm) * 0.5;
    return status;
}

bool turnAroundDetected(const CameraLineSnapshot& snapshot)
{
    return snapshot.greenConfirmed && snapshot.greenPathBlackValid &&
           snapshot.greenCandidateCount == 2 &&
           snapshot.greenInterpretation == GreenInterpretation::TurnAround180;
}

bool turnAroundEncodersReady(const Esp32TelemetrySnapshot& telemetry)
{
    return telemetry.sensorFresh && telemetry.lastSensorAgeMs >= 0 &&
           telemetry.lastSensorAgeMs <=
               config::kGreenTurnAroundEncoderDataTimeoutMs &&
           std::isfinite(telemetry.leftEncoderRate) &&
           std::isfinite(telemetry.rightEncoderRate);
}

ImuTurnDirection configuredTurnDirection()
{
    return config::kGreenTurnAroundTurnsRight
               ? ImuTurnDirection::Right
               : ImuTurnDirection::Left;
}

double configuredTurnSign()
{
    return config::kGreenTurnAroundTurnsRight ? 1.0 : -1.0;
}

double shortestAngularDistanceDegrees(double first, double second)
{
    double difference = std::fmod(std::abs(second - first), 360.0);
    if (difference > 180.0)
    {
        difference = 360.0 - difference;
    }
    return difference;
}

}

void MainMission::reset()
{
    turnAroundPhase_ = TurnAroundPhase::Idle;
    turnAroundController_.reset();
    turnAroundArmed_ = true;
    forwardStartLeftCount_ = 0;
    forwardStartRightCount_ = 0;
    lineReacquireFrames_ = 0;
    lineSearchStartYawDegrees_ = 0.0;
    resetForwardAssist();
}

void MainMission::resetForwardAssist()
{
    forwardAssistState_ = ForwardAssistState::Bottom;
    forwardAssistDirection_ = ForwardAssistDirection::None;
    forwardAssistYawOriginDegrees_ = 0.0;
    forwardAssistYawDeltaDegrees_ = 0.0;
    bottomStableFrames_ = 0;
    bottomLostFrames_ = 0;
    hasPreviousBottomFrame_ = false;
    previousBottomSequence_ = 0;
    previousBottomTrusted_ = false;
    previousBottomLineNormal_ = false;
    latchedBottomDirection_ = ForwardAssistDirection::None;
    mediumFlipCandidateDirection_ = ForwardAssistDirection::None;
    mediumFlipConfirmationFrames_ = 0;
    forwardAssistFarTrusted_ = false;
    forwardAssistMediumTrusted_ = false;
    forwardAssistGapCandidate_ = false;
    forwardAssistEntryAllowed_ = false;
    forwardAssistEntryBlocker_ = "WAITING_TRUST";
}

void MainMission::updateLatchedBottomDirection(
    ForwardAssistDirection farDirection,
    ForwardAssistDirection mediumDirection,
    bool consecutiveBottomFrame)
{
    const bool directionsConflict =
        farDirection != ForwardAssistDirection::None &&
        mediumDirection != ForwardAssistDirection::None &&
        farDirection != mediumDirection;
    if (directionsConflict &&
        latchedBottomDirection_ != ForwardAssistDirection::None)
    {
        // Um conflito instantâneo não pode inverter uma curva já memorizada.
        mediumFlipCandidateDirection_ = ForwardAssistDirection::None;
        mediumFlipConfirmationFrames_ = 0;
        return;
    }

    if (farDirection != ForwardAssistDirection::None)
    {
        // FAR representa melhor a continuação futura e atualiza o lado sem atraso.
        latchedBottomDirection_ = farDirection;
        mediumFlipCandidateDirection_ = ForwardAssistDirection::None;
        mediumFlipConfirmationFrames_ = 0;
        return;
    }

    if (mediumDirection == ForwardAssistDirection::None)
    {
        // NONE interrompe apenas uma confirmação de inversão; o latch permanece.
        mediumFlipCandidateDirection_ = ForwardAssistDirection::None;
        mediumFlipConfirmationFrames_ = 0;
        return;
    }

    if (latchedBottomDirection_ == ForwardAssistDirection::None ||
        latchedBottomDirection_ == mediumDirection)
    {
        latchedBottomDirection_ = mediumDirection;
        mediumFlipCandidateDirection_ = ForwardAssistDirection::None;
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
        mediumFlipCandidateDirection_ = ForwardAssistDirection::None;
        mediumFlipConfirmationFrames_ = 0;
    }
}

AutonomousStatus MainMission::forwardAssistStatus(
    const std::string& phase,
    const std::string& action,
    const ForwardLineSnapshot& forwardLineSnapshot) const
{
    AutonomousStatus status = makeMainMissionStatus(phase, action);
    switch (forwardAssistState_)
    {
    case ForwardAssistState::SearchSpin:
        status.forwardAssistState = "SEARCH_SPIN";
        break;
    case ForwardAssistState::ForwardFollow:
        status.forwardAssistState = "FORWARD_FOLLOW";
        break;
    case ForwardAssistState::Bottom:
    default:
        status.forwardAssistState = "BOTTOM";
        break;
    }
    switch (forwardAssistDirection_)
    {
    case ForwardAssistDirection::Left:
        status.forwardAssistDirection = "LEFT";
        break;
    case ForwardAssistDirection::Right:
        status.forwardAssistDirection = "RIGHT";
        break;
    case ForwardAssistDirection::None:
    default:
        status.forwardAssistDirection = "NONE";
        break;
    }
    switch (latchedBottomDirection_)
    {
    case ForwardAssistDirection::Left:
        status.forwardAssistLatchedDirection = "LEFT";
        break;
    case ForwardAssistDirection::Right:
        status.forwardAssistLatchedDirection = "RIGHT";
        break;
    case ForwardAssistDirection::None:
    default:
        status.forwardAssistLatchedDirection = "NONE";
        break;
    }
    status.forwardAssistFarTrusted = forwardAssistFarTrusted_;
    status.forwardAssistMediumTrusted = forwardAssistMediumTrusted_;
    status.forwardAssistGapCandidate = forwardAssistGapCandidate_;
    status.forwardAssistEntryAllowed = forwardAssistEntryAllowed_;
    status.forwardAssistEntryBlocker = forwardAssistEntryBlocker_;
    status.forwardAssistYawDeltaDeg = forwardAssistYawDeltaDegrees_;
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

bool MainMission::updateForwardAssist(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    const CameraLineSnapshot& cameraLineSnapshot,
    const ForwardLineSnapshot& forwardLineSnapshot)
{
    forwardAssistFarTrusted_ = cameraLineSnapshot.farTrusted;
    forwardAssistMediumTrusted_ = cameraLineSnapshot.mediumTrusted;
    forwardAssistGapCandidate_ =
        cameraLineSnapshot.curveDiagnostics.lineState == "GAP";
    const bool bottomLineNormal =
        cameraLineSnapshot.curveDiagnostics.lineState == "LINE" &&
        cameraLineSnapshot.curveDiagnostics.virtualState == "NORMAL";
    const bool searchOwnsRecovery =
        forwardAssistState_ == ForwardAssistState::SearchSpin;
    const bool greenHasPriority =
        cameraLineSnapshot.curveDiagnostics.lineState == "GREEN";
    if (!bottomLineNormal && (!searchOwnsRecovery || greenHasPriority))
    {
        // Antes de uma busca começar, GREEN, GAP e estados inferiores especiais
        // preservam sua autoridade existente. Depois da entrada legítima no
        // SEARCH, somente GREEN pode retirar essa posse neste gate.
        const bool farTrusted = forwardAssistFarTrusted_;
        const bool mediumTrusted = forwardAssistMediumTrusted_;
        const bool gapCandidate = forwardAssistGapCandidate_;
        const std::string entryBlocker =
            cameraLineSnapshot.curveDiagnostics.lineState != "LINE"
                ? "LINE_NOT_NORMAL"
                : "VIRTUAL_NOT_NORMAL";
        resetForwardAssist();
        forwardAssistFarTrusted_ = farTrusted;
        forwardAssistMediumTrusted_ = mediumTrusted;
        forwardAssistGapCandidate_ = gapCandidate;
        forwardAssistEntryBlocker_ = entryBlocker;
        return false;
    }

    const auto directionFromTrustedPosition = [](
        bool trusted,
        double position)
    {
        if (!trusted || !std::isfinite(position))
        {
            return ForwardAssistDirection::None;
        }
        if (position <= -config::kForwardAssistDirectionPositionThreshold)
        {
            return ForwardAssistDirection::Left;
        }
        if (position >= config::kForwardAssistDirectionPositionThreshold)
        {
            return ForwardAssistDirection::Right;
        }
        return ForwardAssistDirection::None;
    };
    const ForwardAssistDirection farDirection =
        directionFromTrustedPosition(
            cameraLineSnapshot.farTrusted,
            cameraLineSnapshot.curveDiagnostics.farBandPosition);
    const ForwardAssistDirection mediumDirection =
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
    const ForwardAssistDirection latchedBottomDirection =
        latchedBottomDirection_;

    if (newBottomFrame)
    {
        hasPreviousBottomFrame_ = true;
        previousBottomSequence_ = cameraLineSnapshot.lineSequence;
        previousBottomTrusted_ = currentBottomTrusted;
        previousBottomLineNormal_ = bottomLineNormal;
        if (forwardAssistState_ == ForwardAssistState::Bottom)
        {
            updateLatchedBottomDirection(
                farDirection,
                mediumDirection,
                consecutiveBottomFrame);
        }
    }

    const bool bottomLostTrustedRows =
        !cameraLineSnapshot.farTrusted &&
        !cameraLineSnapshot.mediumTrusted;
    const bool bottomRequestsExistingCriticalTurn =
        (cameraLineSnapshot.lineFollowerLeftPower < 0.0 ||
         cameraLineSnapshot.lineFollowerRightPower < 0.0) &&
        cameraLineSnapshot.lineControlSource != "virtual-blind-search";
    if (forwardAssistState_ == ForwardAssistState::Bottom && newBottomFrame)
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

        forwardAssistEntryAllowed_ = false;
        if (!bottomLostTrustedRows)
        {
            forwardAssistEntryBlocker_ = cameraLineSnapshot.farTrusted
                                             ? "FAR_TRUSTED"
                                             : "MEDIUM_TRUSTED";
        }
        else if (cameraLineSnapshot.normalSteeringValid)
        {
            // O Fusion ainda possui target e comando LINE válidos, ou o virtual
            // permanece NORMAL; perder FAR/MEDIUM não autoriza SEARCH_SPIN.
            forwardAssistEntryBlocker_ = "NORMAL_STEERING_VALID";
        }
        else if (bottomLostFrames_ == 0)
        {
            forwardAssistEntryBlocker_ = "NO_PREVIOUS_TRUST";
        }
        else if (latchedBottomDirection == ForwardAssistDirection::None)
        {
            forwardAssistEntryBlocker_ = "NO_LATCHED_DIRECTION";
        }
        else if (bottomRequestsExistingCriticalTurn)
        {
            bottomLostFrames_ = 0;
            forwardAssistEntryBlocker_ = "BOTTOM_CRITICAL_TURN";
        }
        else if (!ImuTurnController::imuReady(esp32Telemetry))
        {
            bottomLostFrames_ = 0;
            forwardAssistEntryBlocker_ = "IMU_NOT_READY";
        }
        else if (bottomLostFrames_ < config::kForwardAssistBottomLossFrames)
        {
            forwardAssistEntryBlocker_ = "WAITING_LOSS_CONFIRMATION";
        }
        else
        {
            // A direção trusted permanece latched, mas a busca só começa após
            // dois frames de perda. Saltos no sequence não apagam esse lado.
            forwardAssistEntryAllowed_ = true;
            forwardAssistEntryBlocker_ = "NONE";
            forwardAssistState_ = ForwardAssistState::SearchSpin;
            forwardAssistDirection_ = latchedBottomDirection;
            forwardAssistYawOriginDegrees_ = esp32Telemetry.yawZDeg;
            forwardAssistYawDeltaDegrees_ = 0.0;
            bottomStableFrames_ = 0;
            bottomLostFrames_ = 0;
        }
    }

    if (forwardAssistState_ == ForwardAssistState::Bottom)
    {
        return false;
    }

    const bool imuReady = ImuTurnController::imuReady(esp32Telemetry);
    if (!imuReady ||
        (forwardAssistState_ != ForwardAssistState::SearchSpin &&
         bottomRequestsExistingCriticalTurn))
    {
        // A perda da IMU sempre encerra a assistência. HARD CORNER, PIVOT e
        // SPIN inferiores bloqueiam a entrada, mas não expulsam um SEARCH que
        // já possui direção e orçamento angular válidos.
        forwardAssistState_ = ForwardAssistState::Bottom;
        forwardAssistDirection_ = ForwardAssistDirection::None;
        forwardAssistYawDeltaDegrees_ = 0.0;
        bottomStableFrames_ = 0;
        forwardAssistEntryAllowed_ = false;
        forwardAssistEntryBlocker_ = !imuReady
                                         ? "IMU_NOT_READY"
                                         : "BOTTOM_CRITICAL_TURN";
        return false;
    }

    forwardAssistYawDeltaDegrees_ = shortestAngularDistanceDegrees(
        forwardAssistYawOriginDegrees_,
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

    if (forwardAssistState_ == ForwardAssistState::ForwardFollow &&
        bottomRecoveryConfirmed())
    {
        forwardAssistState_ = ForwardAssistState::Bottom;
        forwardAssistDirection_ = ForwardAssistDirection::None;
        forwardAssistYawDeltaDegrees_ = 0.0;
        bottomStableFrames_ = 0;
        forwardAssistEntryAllowed_ = false;
        forwardAssistEntryBlocker_ = cameraLineSnapshot.farTrusted
                                         ? "FAR_TRUSTED"
                                         : "MEDIUM_TRUSTED";
        return false;
    }

    if (forwardAssistState_ == ForwardAssistState::ForwardFollow &&
        !forwardLineSnapshot.lineObservationValid())
    {
        // A tentativa continua com a mesma direção, origem e orçamento angular.
        forwardAssistState_ = ForwardAssistState::SearchSpin;
    }

    if (forwardAssistState_ == ForwardAssistState::SearchSpin)
    {
        if (forwardAssistYawDeltaDegrees_ >=
                config::kForwardAssistMaximumSearchDegrees)
        {
            // O teto angular entrega imediatamente a decisão ao recovery
            // inferior já existente.
            forwardAssistState_ = ForwardAssistState::Bottom;
            forwardAssistDirection_ = ForwardAssistDirection::None;
            forwardAssistYawDeltaDegrees_ = 0.0;
            bottomStableFrames_ = 0;
            forwardAssistEntryAllowed_ = false;
            forwardAssistEntryBlocker_ = "SEARCH_ANGLE_LIMIT";
            return false;
        }

        if (forwardLineSnapshot.lineObservationValid())
        {
            // Esta transição acontece antes de qualquer comando de SPIN. Assim,
            // o primeiro frame frontal válido já substitui o giro pelo avanço.
            forwardAssistState_ = ForwardAssistState::ForwardFollow;
        }
        else if (bottomRecoveryConfirmed())
        {
            // Um flicker trusted conta apenas como 1/2. A bottom recupera a
            // autoridade somente após dois frames novos, normais e consecutivos.
            forwardAssistState_ = ForwardAssistState::Bottom;
            forwardAssistDirection_ = ForwardAssistDirection::None;
            forwardAssistYawDeltaDegrees_ = 0.0;
            bottomStableFrames_ = 0;
            forwardAssistEntryAllowed_ = false;
            forwardAssistEntryBlocker_ = cameraLineSnapshot.farTrusted
                                             ? "FAR_TRUSTED"
                                             : "MEDIUM_TRUSTED";
            return false;
        }
        else
        {
            const double spinPower =
                config::kForwardAssistSearchSpinPower;
            const bool turnsLeft =
                forwardAssistDirection_ == ForwardAssistDirection::Left;
            robotState.driveAutonomous(
                turnsLeft ? -spinPower : spinPower,
                turnsLeft ? spinPower : -spinPower);
            robotState.updateAutonomousStatus(forwardAssistStatus(
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
        forwardAssistState_ = ForwardAssistState::Bottom;
        forwardAssistDirection_ = ForwardAssistDirection::None;
        forwardAssistYawDeltaDegrees_ = 0.0;
        bottomStableFrames_ = 0;
        forwardAssistEntryAllowed_ = false;
        forwardAssistEntryBlocker_ = "FORWARD_COMMAND_INVALID";
        return false;
    }

    robotState.driveAutonomous(
        forwardLineSnapshot.normalLeftPower,
        forwardLineSnapshot.normalRightPower);
    robotState.updateAutonomousStatus(forwardAssistStatus(
        "forward_assist_follow",
        "FWD: FOLLOW " + std::to_string(forwardLineSnapshot.position),
        forwardLineSnapshot));
    return true;
}

void MainMission::update(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    bool cameraReady,
    const CameraLineSnapshot& cameraLineSnapshot,
    const ForwardLineSnapshot& forwardLineSnapshot)
{
    const RobotSnapshot robotSnapshot = robotState.snapshot();
    if (robotSnapshot.mode != "autonomous" ||
        robotSnapshot.autonomousMission != AutonomousMission::MainMission)
    {
        return;
    }

    if (!esp32Telemetry.readyForOperation() || !cameraReady ||
        !cameraLineSnapshot.sourceFresh)
    {
        std::string phase;
        std::string action;
        if (!esp32Telemetry.readyForOperation())
        {
            phase = "esp32_not_ready";
            action = "Missão interrompida: ESP32 sem telemetria pronta";
        }
        else if (!cameraReady)
        {
            phase = "camera_not_ready";
            action = "Missão interrompida: câmera inferior indisponível";
        }
        else
        {
            phase = "line_ipc_stale";
            action = "Missão interrompida: IPC visual ausente ou antigo";
        }
        reset();
        robotState.stop();
        robotState.updateAutonomousStatus(
            makeMainMissionStatus(phase, action));
        std::cout << "MainMission stopped: " << phase << std::endl;
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto startTurnAroundForward = [&]()
    {
        if (!turnAroundEncodersReady(esp32Telemetry) ||
            !ImuTurnController::imuReady(esp32Telemetry))
        {
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_waiting_sensors",
                "Retorno pausado: aguardando encoders e MPU6050"));
            return false;
        }

        turnAroundPhase_ = TurnAroundPhase::DrivingForward;
        forwardStartLeftCount_ = esp32Telemetry.leftEncoderCount;
        forwardStartRightCount_ = esp32Telemetry.rightEncoderCount;
        phaseStartedAt_ = now;
        return true;
    };
    const bool detected180 = turnAroundDetected(cameraLineSnapshot);
    if (detected180 || turnAroundPhase_ != TurnAroundPhase::Idle)
    {
        // O retorno verde é uma prioridade superior e nunca compartilha
        // autoridade com a assistência frontal.
        resetForwardAssist();
    }
    if (turnAroundPhase_ == TurnAroundPhase::Idle && !detected180)
    {
        // Um retorno concluído só pode disparar novamente depois que os dois
        // verdes realmente saírem da percepção confirmada.
        turnAroundArmed_ = true;
    }

    if (turnAroundPhase_ == TurnAroundPhase::Idle && detected180 &&
        turnAroundArmed_)
    {
        if (!turnAroundEncodersReady(esp32Telemetry) ||
            !ImuTurnController::imuReady(esp32Telemetry))
        {
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_waiting_sensors",
                "Retorno detectado: aguardando encoders e MPU6050"));
            return;
        }

        turnAroundArmed_ = false;
        lineReacquireFrames_ = 0;
        lineSearchStartYawDegrees_ = 0.0;
        // O retorno assume os motores no mesmo ciclo da confirmação verde.
        // A primeira pausa elimina qualquer comando residual do segue-linha.
        turnAroundPhase_ = TurnAroundPhase::RecognitionDelay;
        phaseStartedAt_ = now;
    }

    if (turnAroundPhase_ == TurnAroundPhase::Idle)
    {
        if (updateForwardAssist(
                robotState,
                esp32Telemetry,
                cameraLineSnapshot,
                forwardLineSnapshot))
        {
            return;
        }
        robotState.driveAutonomous(
            cameraLineSnapshot.lineFollowerLeftPower,
            cameraLineSnapshot.lineFollowerRightPower);
        robotState.updateAutonomousStatus(forwardAssistStatus(
            "line_following",
            "Seguindo a linha pela câmera inferior",
            forwardLineSnapshot));
        return;
    }

    if (turnAroundPhase_ == TurnAroundPhase::RecognitionDelay)
    {
        robotState.driveAutonomous(0.0, 0.0);
        if (now - phaseStartedAt_ < std::chrono::milliseconds(
                config::kGreenTurnAroundRecognitionDelayMs))
        {
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_recognition_delay",
                "Retorno 180° reconhecido: aguardando antes do alinhamento"));
            return;
        }

        turnAroundPhase_ = TurnAroundPhase::Centering;
        phaseStartedAt_ = now;
    }

    if (turnAroundPhase_ == TurnAroundPhase::Centering)
    {
        const bool nearPositionValid =
            cameraLineSnapshot.lineNearDetected &&
            std::isfinite(cameraLineSnapshot.lineNearFinePosition) &&
            std::abs(cameraLineSnapshot.lineNearFinePosition) <= 1.0;
        const double mediumPosition =
            cameraLineSnapshot.curveDiagnostics.mediumPosition;
        const bool mediumPositionValid =
            cameraLineSnapshot.mediumTrusted &&
            std::isfinite(mediumPosition) &&
            std::abs(mediumPosition) <= 1.0;
        const bool bothCentered =
            nearPositionValid && mediumPositionValid &&
            std::abs(cameraLineSnapshot.lineNearFinePosition) <=
                config::kGreenTurnAroundCenteringTolerance &&
            std::abs(mediumPosition) <=
                config::kGreenTurnAroundCenteringTolerance;
        const bool timedOut =
            now - phaseStartedAt_ >= std::chrono::milliseconds(
                config::kGreenTurnAroundCenteringTimeoutMs);

        if (bothCentered || timedOut)
        {
            // O segundo intervalo sempre começa com os motores zerados. Mesmo
            // no timeout, isso interrompe o giro antes do avanço por encoder.
            turnAroundPhase_ = TurnAroundPhase::PostCenteringDelay;
            phaseStartedAt_ = now;
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                timedOut
                    ? "turnaround_centering_timeout_delay"
                    : "turnaround_centered_delay",
                timedOut
                    ? "Alinhamento expirou: aguardando antes do avanço"
                    : "NEAR e MEDIUM alinhados: aguardando antes do avanço"));
            return;
        }

        double alignmentPosition = 0.0;
        bool alignmentDirectionValid = false;
        if (mediumPositionValid &&
            std::abs(mediumPosition) >
                config::kGreenTurnAroundCenteringTolerance)
        {
            // O MEDIUM corrige primeiro a orientação futura da faixa. Quando
            // ele já está central, o NEAR remove o deslocamento restante.
            alignmentPosition = mediumPosition;
            alignmentDirectionValid = true;
        }
        else if (nearPositionValid &&
                 std::abs(cameraLineSnapshot.lineNearFinePosition) >
                     config::kGreenTurnAroundCenteringTolerance)
        {
            alignmentPosition = cameraLineSnapshot.lineNearFinePosition;
            alignmentDirectionValid = true;
        }

        if (!alignmentDirectionValid)
        {
            // Sem posição lateral confiável, permanecer parado é mais seguro
            // do que escolher um lado e iniciar um giro cego.
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_centering_waiting_line",
                "Aguardando NEAR e MEDIUM válidos para concluir o alinhamento"));
            return;
        }

        const double turnSign = alignmentPosition > 0.0 ? 1.0 : -1.0;
        const double leftPower =
            turnSign * config::kGreenTurnAroundCenteringPower;
        robotState.driveAutonomous(leftPower, -leftPower);
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            "turnaround_centering",
            "Retorno 180°: alinhando NEAR e MEDIUM antes do avanço"));
        return;
    }

    if (turnAroundPhase_ == TurnAroundPhase::PostCenteringDelay)
    {
        robotState.driveAutonomous(0.0, 0.0);
        if (now - phaseStartedAt_ < std::chrono::milliseconds(
                config::kGreenTurnAroundPostCenteringDelayMs))
        {
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_post_centering_delay",
                "Alinhamento encerrado: aguardando antes do avanço"));
            return;
        }
        if (!startTurnAroundForward())
        {
            return;
        }
    }

    if (turnAroundPhase_ == TurnAroundPhase::DrivingForward)
    {
        const double leftCounts = std::abs(static_cast<double>(
            esp32Telemetry.leftEncoderCount - forwardStartLeftCount_));
        const double rightCounts = std::abs(static_cast<double>(
            esp32Telemetry.rightEncoderCount - forwardStartRightCount_));
        const double leftDistanceCm =
            leftCounts / config::kEncoderCountsPerCentimeter;
        const double rightDistanceCm =
            rightCounts / config::kEncoderCountsPerCentimeter;
        const double minimumDistanceCm =
            std::min(leftDistanceCm, rightDistanceCm);
        const double progressPercent = std::clamp(
            minimumDistanceCm /
                config::kGreenTurnAroundForwardDistanceCm * 100.0,
            0.0,
            100.0);

        if (!turnAroundEncodersReady(esp32Telemetry))
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeTurnAroundForwardStatus(
                "turnaround_encoder_lost",
                "Retorno interrompido: encoders sem dados recentes",
                leftDistanceCm,
                rightDistanceCm,
                progressPercent));
            return;
        }
        if (now - phaseStartedAt_ > std::chrono::milliseconds(
                config::kGreenTurnAroundForwardSafetyTimeoutMs))
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeTurnAroundForwardStatus(
                "turnaround_forward_timeout",
                "Retorno interrompido pelo limite absoluto de segurança",
                leftDistanceCm,
                rightDistanceCm,
                progressPercent));
            return;
        }

        const double predictionSeconds =
            config::kDriveDistanceBrakePredictionSeconds +
            esp32Telemetry.lastSensorAgeMs / 1000.0;
        const double projectedLeftCounts =
            leftCounts + std::abs(esp32Telemetry.leftEncoderRate) *
                             predictionSeconds;
        const double projectedRightCounts =
            rightCounts + std::abs(esp32Telemetry.rightEncoderRate) *
                              predictionSeconds;
        const double targetCounts =
            config::kGreenTurnAroundForwardDistanceCm *
            config::kEncoderCountsPerCentimeter;
        if (std::min(projectedLeftCounts, projectedRightCounts) >= targetCounts)
        {
            turnAroundPhase_ = TurnAroundPhase::ForwardSettling;
            phaseStartedAt_ = now;
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeTurnAroundForwardStatus(
                "turnaround_forward_settling",
                "Avanço concluído: aguardando o robô estabilizar",
                leftDistanceCm,
                rightDistanceCm,
                progressPercent));
            return;
        }

        robotState.driveAutonomous(
            config::kGreenTurnAroundForwardPower,
            config::kGreenTurnAroundForwardPower);
        robotState.updateAutonomousStatus(makeTurnAroundForwardStatus(
            "turnaround_forward",
            "Retorno 180°: avançando a distância configurada pelos encoders",
            leftDistanceCm,
            rightDistanceCm,
            progressPercent));
        return;
    }

    if (turnAroundPhase_ == TurnAroundPhase::ForwardSettling)
    {
        robotState.driveAutonomous(0.0, 0.0);
        if (now - phaseStartedAt_ < std::chrono::milliseconds(
                config::kGreenTurnAroundForwardSettleMs))
        {
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_forward_settling",
                "Aguardando a parada antes do giro por IMU"));
            return;
        }
        if (!turnAroundController_.start(
                config::kGreenTurnAroundImuDegrees,
                configuredTurnDirection(),
                esp32Telemetry,
                config::kGreenTurnAroundImuToleranceDegrees))
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turn_imu_lost",
                "Retorno interrompido: MPU6050 sem referência válida"));
            return;
        }
        turnAroundPhase_ = TurnAroundPhase::TurningByImu;
    }

    if (turnAroundPhase_ == TurnAroundPhase::TurningByImu)
    {
        const ImuTurnOutput output =
            turnAroundController_.update(esp32Telemetry);
        if (output.result == ImuTurnResult::Failed)
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                output.phase, output.action, output.progressPercent));
            return;
        }
        if (output.result == ImuTurnResult::Completed)
        {
            // A busca visual começa somente após a IMU concluir e estabilizar
            // o giro inicial. Isso evita trocar cedo para um pivot sem alvo angular.
            turnAroundPhase_ = TurnAroundPhase::SearchingLine;
            phaseStartedAt_ = now;
            lineReacquireFrames_ = 0;
            lineSearchStartYawDegrees_ = esp32Telemetry.yawZDeg;
        }
        else
        {
            robotState.driveAutonomous(output.leftPower, output.rightPower);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_imu", output.action, output.progressPercent));
            return;
        }
    }

    if (turnAroundPhase_ == TurnAroundPhase::SearchingLine)
    {
        if (!ImuTurnController::imuReady(esp32Telemetry))
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turn_imu_lost",
                "Retorno interrompido: IMU perdida durante a busca da linha"));
            return;
        }

        const bool fusionLineRecovered =
            cameraLineSnapshot.lineControlSource == "fusion" &&
            cameraLineSnapshot.normalSteeringValid;
        const bool lineRecovered =
            cameraLineSnapshot.lineNearDetected || fusionLineRecovered;
        if (lineRecovered)
        {
            ++lineReacquireFrames_;
        }
        else
        {
            lineReacquireFrames_ = 0;
        }

        if (lineReacquireFrames_ >=
            config::kGreenTurnAroundLineReacquireFrames)
        {
            turnAroundPhase_ = TurnAroundPhase::Idle;
            turnAroundController_.reset();
            robotState.driveAutonomous(
                cameraLineSnapshot.lineFollowerLeftPower,
                cameraLineSnapshot.lineFollowerRightPower);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "line_following",
                fusionLineRecovered &&
                        !cameraLineSnapshot.lineNearDetected
                    ? "Retorno 180° concluído: linha recuperada pelo Fusion"
                    : "Retorno 180° concluído: linha próxima recuperada",
                100.0));
            return;
        }

        const double lineSearchDegrees = shortestAngularDistanceDegrees(
            lineSearchStartYawDegrees_, esp32Telemetry.yawZDeg);
        if (lineSearchDegrees >=
            config::kGreenTurnAroundLineSearchMaximumDegrees)
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_line_search_angle_limit",
                "Retorno interrompido: limite angular da busca visual atingido"));
            return;
        }
        if (now - phaseStartedAt_ > std::chrono::milliseconds(
                config::kGreenTurnAroundLineSearchTimeoutMs))
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_line_search_timeout",
                "Retorno interrompido: linha não encontrada no tempo seguro"));
            return;
        }

        const double leftPower =
            configuredTurnSign() * config::kGreenTurnAroundLineSearchPower;
        robotState.driveAutonomous(leftPower, -leftPower);
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            "turnaround_searching_line",
            "Continuando o giro até a linha próxima reaparecer"));
    }
}
