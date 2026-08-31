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
    hasPreviousBottomFrame_ = false;
    previousBottomSequence_ = 0;
    previousBottomTrusted_ = false;
    previousBottomLineNormal_ = false;
    previousBottomDirection_ = ForwardAssistDirection::None;
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
    const bool bottomLineNormal =
        cameraLineSnapshot.curveDiagnostics.lineState == "LINE" &&
        cameraLineSnapshot.curveDiagnostics.virtualState == "NORMAL";
    if (!bottomLineNormal)
    {
        // GREEN, GAP e estados de curva superiores sempre retiram a autoridade
        // frontal. A memória anterior também é descartada para impedir que uma
        // direção observada antes desses estados dispare uma busca posterior.
        resetForwardAssist();
        return false;
    }

    ForwardAssistDirection currentTrustedDirection =
        ForwardAssistDirection::None;
    if (cameraLineSnapshot.trustedDirection == "LEFT")
    {
        currentTrustedDirection = ForwardAssistDirection::Left;
    }
    else if (cameraLineSnapshot.trustedDirection == "RIGHT")
    {
        currentTrustedDirection = ForwardAssistDirection::Right;
    }

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
    const ForwardAssistDirection previousBottomDirection =
        previousBottomDirection_;

    if (newBottomFrame)
    {
        hasPreviousBottomFrame_ = true;
        previousBottomSequence_ = cameraLineSnapshot.lineSequence;
        previousBottomTrusted_ = currentBottomTrusted;
        previousBottomLineNormal_ = bottomLineNormal;
        previousBottomDirection_ = currentBottomTrusted
                                       ? currentTrustedDirection
                                       : ForwardAssistDirection::None;
    }

    const bool bottomLostTrustedRows =
        !cameraLineSnapshot.farTrusted &&
        !cameraLineSnapshot.mediumTrusted;
    const bool bottomRequestsExistingCriticalTurn =
        (cameraLineSnapshot.lineFollowerLeftPower < 0.0 ||
         cameraLineSnapshot.lineFollowerRightPower < 0.0) &&
        cameraLineSnapshot.lineControlSource != "virtual-blind-search";
    if (forwardAssistState_ == ForwardAssistState::Bottom &&
        consecutiveBottomFrame && previousBottomTrusted &&
        previousBottomLineNormal &&
        previousBottomDirection != ForwardAssistDirection::None &&
        bottomLostTrustedRows && !bottomRequestsExistingCriticalTurn &&
        ImuTurnController::imuReady(esp32Telemetry))
    {
        // A direção vem do último frame inferior consecutivo e o yaw nasce uma
        // única vez. Voltar do FOLLOW para SEARCH não passa novamente aqui.
        forwardAssistState_ = ForwardAssistState::SearchSpin;
        forwardAssistDirection_ = previousBottomDirection;
        forwardAssistYawOriginDegrees_ = esp32Telemetry.yawZDeg;
        forwardAssistYawDeltaDegrees_ = 0.0;
        bottomStableFrames_ = 0;
    }

    if (forwardAssistState_ == ForwardAssistState::Bottom)
    {
        return false;
    }

    if (bottomRequestsExistingCriticalTurn ||
        !ImuTurnController::imuReady(esp32Telemetry))
    {
        // HARD CORNER, PIVOT, SPIN e perda da IMU devolvem o comando ao fluxo
        // inferior no mesmo ciclo; a frontal não disputa essas prioridades.
        forwardAssistState_ = ForwardAssistState::Bottom;
        forwardAssistDirection_ = ForwardAssistDirection::None;
        forwardAssistYawDeltaDegrees_ = 0.0;
        bottomStableFrames_ = 0;
        return false;
    }

    forwardAssistYawDeltaDegrees_ = shortestAngularDistanceDegrees(
        forwardAssistYawOriginDegrees_,
        esp32Telemetry.yawZDeg);

    if (forwardAssistState_ == ForwardAssistState::ForwardFollow &&
        newBottomFrame)
    {
        const bool bottomRecovered = currentBottomTrusted &&
                                     cameraLineSnapshot.normalSteeringValid;
        if (bottomRecovered)
        {
            bottomStableFrames_ =
                bottomStableFrames_ > 0 && consecutiveBottomFrame
                    ? bottomStableFrames_ + 1
                    : 1;
        }
        else
        {
            bottomStableFrames_ = 0;
        }
        if (bottomStableFrames_ >=
            config::kForwardAssistBottomStableFrames)
        {
            forwardAssistState_ = ForwardAssistState::Bottom;
            forwardAssistDirection_ = ForwardAssistDirection::None;
            forwardAssistYawDeltaDegrees_ = 0.0;
            bottomStableFrames_ = 0;
            return false;
        }
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
            return false;
        }

        if (currentBottomTrusted)
        {
            // Durante SEARCH, qualquer FAR/MEDIUM trusted encerra o SPIN no
            // mesmo ciclo e devolve a decisão ao fluxo inferior existente.
            forwardAssistState_ = ForwardAssistState::Bottom;
            forwardAssistDirection_ = ForwardAssistDirection::None;
            forwardAssistYawDeltaDegrees_ = 0.0;
            bottomStableFrames_ = 0;
            return false;
        }
        bottomStableFrames_ = 0;

        if (forwardLineSnapshot.lineObservationValid())
        {
            // Esta transição acontece antes de qualquer comando de SPIN. Assim,
            // o primeiro frame frontal válido já substitui o giro pelo avanço.
            forwardAssistState_ = ForwardAssistState::ForwardFollow;
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
        // O frame que confirma os dois verdes pode conter um comando forte de
        // curva. O retorno assume os motores imediatamente para impedir que
        // esse comando antigo provoque um SPIN antes do avanço por encoder.
        if (!startTurnAroundForward())
        {
            return;
        }
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

        if (cameraLineSnapshot.lineNearDetected)
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
                "Retorno 180° concluído: linha próxima recuperada",
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
