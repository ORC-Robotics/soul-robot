#include "obr/mission_controller.h"

#include "obr/config.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

namespace
{
AutonomousStatus makeAutonomousStatus(
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

AutonomousStatus makeDistanceStatus(
    const std::string& phase,
    const std::string& action,
    double targetDistanceCm,
    double leftDistanceCm,
    double rightDistanceCm,
    double progressPercent)
{
    AutonomousStatus status = makeAutonomousStatus(
        phase, action, progressPercent);
    status.targetDistanceCm = targetDistanceCm;
    status.leftDistanceCm = leftDistanceCm;
    status.rightDistanceCm = rightDistanceCm;
    status.averageDistanceCm = (leftDistanceCm + rightDistanceCm) * 0.5;
    return status;
}

struct DriveDistanceCommand
{
    double left = config::kDriveDistanceBaseCommandPower;
    double right = config::kDriveDistanceBaseCommandPower;
};

DriveDistanceCommand calculateDriveDistanceCommand(
    double leftDistanceCm,
    double rightDistanceCm)
{
    const double distanceDifferenceCm = leftDistanceCm - rightDistanceCm;
    const double differenceMagnitude = std::abs(distanceDifferenceCm);
    if (differenceMagnitude <= config::kDriveDistanceBalanceDeadbandCm)
    {
        return {};
    }

    const double correction = std::clamp(
        (differenceMagnitude - config::kDriveDistanceBalanceDeadbandCm) *
            config::kDriveDistanceBalanceGainPerCm,
        0.0,
        config::kDriveDistanceMaximumBalanceCorrection);
    DriveDistanceCommand command;
    if (distanceDifferenceCm > 0.0)
    {
        // O lado esquerdo avançou mais: desacelera-o e reforça o direito.
        command.left = std::clamp(
            config::kDriveDistanceBaseCommandPower - correction,
            config::kDriveDistanceMinimumCommandPower,
            config::kDriveDistanceMaximumCommandPower);
        command.right = std::clamp(
            config::kDriveDistanceBaseCommandPower + correction,
            config::kDriveDistanceMinimumCommandPower,
            config::kDriveDistanceMaximumCommandPower);
    }
    else
    {
        // O lado direito avançou mais: aplica a mesma correção no sentido oposto.
        command.left = std::clamp(
            config::kDriveDistanceBaseCommandPower + correction,
            config::kDriveDistanceMinimumCommandPower,
            config::kDriveDistanceMaximumCommandPower);
        command.right = std::clamp(
            config::kDriveDistanceBaseCommandPower - correction,
            config::kDriveDistanceMinimumCommandPower,
            config::kDriveDistanceMaximumCommandPower);
    }
    return command;
}
}

bool MissionController::requiresForwardBallDetection(
    const RobotSnapshot& snapshot) const
{
    if (snapshot.mode != "autonomous" || snapshot.emergencyStop)
    {
        return false;
    }
    if (snapshot.autonomousMission == AutonomousMission::RescueArea)
    {
        return true;
    }
    return snapshot.autonomousMission == AutonomousMission::MainMission &&
           mainMission_.requiresRescueVision();
}

void MissionController::update(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    bool cameraReady,
    const CameraLineSnapshot& cameraLineSnapshot,
    const ForwardLineSnapshot& forwardLineSnapshot,
    const ForwardBallSnapshot& forwardBallSnapshot)
{
    const RobotSnapshot snapshot = robotState.snapshot();
    if (snapshot.mode != "autonomous")
    {
        resetMissionState();
        activeAutonomousRunSequence_ = 0;
        return;
    }

    if (activeAutonomousRunSequence_ != snapshot.autonomousRunSequence)
    {
        // Cada partida recebe um identificador para nunca reutilizar uma fase
        // interna de uma execução anterior depois de Stop seguido de Auto.
        resetMissionState();
        activeAutonomousRunSequence_ = snapshot.autonomousRunSequence;
    }

    switch (snapshot.autonomousMission)
    {
    case AutonomousMission::TurnRight90:
        distancePhase_ = DistancePhase::Idle;
        updateTurnRight90(robotState, esp32Telemetry);
        return;
    case AutonomousMission::DriveDistance:
        testTurnController_.reset();
        rescueAreaMission_.reset();
        obstacleAvoidanceTest_.reset();
        updateDriveDistance(
            robotState, esp32Telemetry, snapshot.driveDistanceTargetCm);
        return;
    case AutonomousMission::RescueArea:
        testTurnController_.reset();
        obstacleAvoidanceTest_.reset();
        distancePhase_ = DistancePhase::Idle;
        mainMission_.reset();
        updateRescueArea(
            robotState, esp32Telemetry, forwardBallSnapshot);
        return;
    case AutonomousMission::ObstacleAvoidance:
        testTurnController_.reset();
        rescueAreaMission_.reset();
        distancePhase_ = DistancePhase::Idle;
        mainMission_.reset();
        updateObstacleAvoidance(robotState, esp32Telemetry);
        return;
    case AutonomousMission::MainMission:
    default:
        testTurnController_.reset();
        rescueAreaMission_.reset();
        obstacleAvoidanceTest_.reset();
        distancePhase_ = DistancePhase::Idle;
        mainMission_.update(
            robotState,
            esp32Telemetry,
            cameraReady,
            cameraLineSnapshot,
            forwardLineSnapshot,
            forwardBallSnapshot,
            activeAutonomousRunSequence_);
        return;
    }
}

void MissionController::updateRescueArea(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    const ForwardBallSnapshot& forwardBallSnapshot)
{
    const RescueAreaOutput output = rescueAreaMission_.update(
        forwardBallSnapshot,
        esp32Telemetry,
        activeAutonomousRunSequence_,
        false,
        true);
    // A ausência ou expiração da visão produz zero neste mesmo ciclo.
    // O RobotState ainda aplica clamp, E-Stop e timeout antes dos motores.
    robotState.driveAutonomous(output.leftPower, output.rightPower);
    if (output.completed || output.failed)
    {
        // Conclusão e falha são estados terminais: parar encerra a missão e
        // impede que a visão pesada continue consumindo CPU sem necessidade.
        robotState.stop();
    }
    robotState.updateAutonomousStatus(output.status);
}

void MissionController::updateObstacleAvoidance(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry)
{
    const ObstacleAvoidanceOutput output = obstacleAvoidanceTest_.update(
        esp32Telemetry,
        true);
    robotState.driveAutonomous(output.leftPower, output.rightPower);

    AutonomousStatus status = makeAutonomousStatus(
        output.phase,
        output.action,
        output.progressPercent);
    status.targetDistanceCm = output.targetDistanceCm;
    status.leftDistanceCm = output.leftDistanceCm;
    status.rightDistanceCm = output.rightDistanceCm;
    status.averageDistanceCm =
        (output.leftDistanceCm + output.rightDistanceCm) * 0.5;
    robotState.updateAutonomousStatus(status);

    if (output.completed || output.failed)
    {
        // O modo isolado termina parado como as demais ferramentas de teste.
        robotState.stop();
    }
}

void MissionController::updateTurnRight90(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry)
{
    if (!testTurnController_.active())
    {
        if (!ImuTurnController::imuReady(esp32Telemetry))
        {
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeAutonomousStatus(
                "waiting_imu", "Aguardando referência angular do MPU6050"));
            return;
        }
        testTurnController_.start(
            config::kTurn90TargetDegrees,
            ImuTurnDirection::Right,
            esp32Telemetry);
    }

    const ImuTurnOutput output = testTurnController_.update(esp32Telemetry);
    robotState.driveAutonomous(output.leftPower, output.rightPower);
    if (output.result == ImuTurnResult::Completed)
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "completed", "Giro de 90° concluído", 100.0));
        return;
    }

    AutonomousStatus status = makeAutonomousStatus(
        output.phase, output.action, output.progressPercent);
    if (output.result == ImuTurnResult::Failed)
    {
        robotState.stop();
        robotState.updateAutonomousStatus(status);
        return;
    }
    robotState.updateAutonomousStatus(status);
}

void MissionController::updateDriveDistance(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    double targetDistanceCm)
{
    const auto now = std::chrono::steady_clock::now();
    const bool encoderReady = esp32Telemetry.sensorFresh &&
                              esp32Telemetry.lastSensorAgeMs >= 0 &&
                              esp32Telemetry.lastSensorAgeMs <=
                                  config::kDriveDistanceEncoderFreshnessMs &&
                              std::isfinite(esp32Telemetry.leftEncoderRate) &&
                              std::isfinite(esp32Telemetry.rightEncoderRate);

    if (distancePhase_ == DistancePhase::Idle)
    {
        if (!std::isfinite(targetDistanceCm) ||
            targetDistanceCm < config::kDriveDistanceMinimumTargetCm ||
            targetDistanceCm > config::kDriveDistanceMaximumTargetCm)
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeDistanceStatus(
                "distance_invalid_target",
                "Distância solicitada fora da faixa segura",
                targetDistanceCm, 0.0, 0.0, 0.0));
            return;
        }

        if (!encoderReady)
        {
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeDistanceStatus(
                "waiting_encoders",
                "Aguardando telemetria recente dos encoders",
                targetDistanceCm, 0.0, 0.0, 0.0));
            return;
        }

        distancePhase_ = DistancePhase::Driving;
        distanceStartLeftCount_ = esp32Telemetry.leftEncoderCount;
        distanceStartRightCount_ = esp32Telemetry.rightEncoderCount;
        activeDistanceTargetCm_ = targetDistanceCm;
        distanceStartedAt_ = now;
        distancePhaseStartedAt_ = now;
        distanceLastProgressAt_ = now;
        lastDistanceProgressCounts_ = 0.0;
        distanceCorrectionPulseCount_ = 0;
        distanceDifferenceSamples_ = 0;
        distanceLastDifferenceUptimeMs_ = 0;
    }

    const double leftCounts = std::abs(static_cast<double>(
        esp32Telemetry.leftEncoderCount - distanceStartLeftCount_));
    const double rightCounts = std::abs(static_cast<double>(
        esp32Telemetry.rightEncoderCount - distanceStartRightCount_));
    const double leftDistanceCm =
        leftCounts / config::kEncoderCountsPerCentimeter;
    const double rightDistanceCm =
        rightCounts / config::kEncoderCountsPerCentimeter;
    const double minimumDistanceCm = std::min(leftDistanceCm, rightDistanceCm);
    const double minimumCounts = std::min(leftCounts, rightCounts);
    const double targetCounts =
        activeDistanceTargetCm_ * config::kEncoderCountsPerCentimeter;
    const double progressPercent = std::clamp(
        minimumDistanceCm / activeDistanceTargetCm_ * 100.0, 0.0, 100.0);
    const DriveDistanceCommand driveCommand = calculateDriveDistanceCommand(
        leftDistanceCm, rightDistanceCm);

    if (now - distanceStartedAt_ >
        std::chrono::milliseconds(config::kDriveDistanceTimeoutMs))
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeDistanceStatus(
            "distance_timeout", "Percurso interrompido pelo tempo limite",
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm,
            progressPercent));
        distancePhase_ = DistancePhase::Idle;
        return;
    }

    if (!encoderReady)
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeDistanceStatus(
            "distance_encoder_lost",
            "Percurso interrompido: encoders sem dados recentes",
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm,
            progressPercent));
        distancePhase_ = DistancePhase::Idle;
        return;
    }

    const bool movementPhase = distancePhase_ == DistancePhase::Driving ||
                               distancePhase_ == DistancePhase::CorrectionPulse;
    const bool newEncoderSample =
        esp32Telemetry.esp32UptimeMs != distanceLastDifferenceUptimeMs_;
    if (movementPhase && newEncoderSample)
    {
        distanceLastDifferenceUptimeMs_ = esp32Telemetry.esp32UptimeMs;
        const double sideDifferenceCm =
            std::abs(leftDistanceCm - rightDistanceCm);
        if (sideDifferenceCm > config::kDriveDistanceMaximumSideDifferenceCm)
        {
            ++distanceDifferenceSamples_;
        }
        else
        {
            distanceDifferenceSamples_ = 0;
        }
    }
    if (movementPhase && distanceDifferenceSamples_ >=
                             config::kDriveDistanceDifferenceConfirmationSamples)
    {
        // Não tenta corrigir uma diferença grande como se fosse um ajuste fino.
        // Parar cedo evita que um motor fraco, roda travada ou encoder incoerente
        // transforme o teste reto em giro no próprio eixo.
        robotState.stop();
        robotState.updateAutonomousStatus(makeDistanceStatus(
            "distance_encoder_mismatch",
            "Percurso interrompido: diferença excessiva entre os lados",
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm,
            progressPercent));
        distancePhase_ = DistancePhase::Idle;
        return;
    }
    if (movementPhase &&
        minimumCounts >= lastDistanceProgressCounts_ +
                             config::kDriveDistanceMinimumProgressCounts)
    {
        lastDistanceProgressCounts_ = minimumCounts;
        distanceLastProgressAt_ = now;
    }
    if (movementPhase &&
        now - distanceLastProgressAt_ >
            std::chrono::milliseconds(config::kDriveDistanceStallTimeoutMs))
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeDistanceStatus(
            "distance_encoder_stall",
            "Percurso interrompido: um lado não avançou",
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm,
            progressPercent));
        distancePhase_ = DistancePhase::Idle;
        return;
    }

    if (distancePhase_ == DistancePhase::Driving)
    {
        const double predictionSeconds =
            config::kDriveDistanceBrakePredictionSeconds +
            esp32Telemetry.lastSensorAgeMs / 1000.0;
        const double projectedLeftCounts =
            leftCounts + std::abs(esp32Telemetry.leftEncoderRate) * predictionSeconds;
        const double projectedRightCounts =
            rightCounts + std::abs(esp32Telemetry.rightEncoderRate) * predictionSeconds;

        if (std::min(projectedLeftCounts, projectedRightCounts) >= targetCounts)
        {
            robotState.driveAutonomous(0.0, 0.0);
            distancePhase_ = DistancePhase::Settling;
            distancePhaseStartedAt_ = now;
            robotState.updateAutonomousStatus(makeDistanceStatus(
                "distance_settling",
                "PWM zerado: aguardando o percurso estabilizar",
                activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm,
                progressPercent));
            return;
        }

        robotState.driveAutonomous(
            driveCommand.left,
            driveCommand.right);
        robotState.updateAutonomousStatus(makeDistanceStatus(
            "driving_distance", "Avançando até a distância selecionada",
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm,
            progressPercent));
        return;
    }

    if (distancePhase_ == DistancePhase::CorrectionPulse)
    {
        if (minimumDistanceCm >=
            activeDistanceTargetCm_ - config::kDriveDistanceToleranceCm)
        {
            robotState.driveAutonomous(0.0, 0.0);
            distancePhase_ = DistancePhase::Settling;
            distancePhaseStartedAt_ = now;
            return;
        }

        if (now - distancePhaseStartedAt_ <
            std::chrono::milliseconds(config::kDriveDistanceCorrectionPulseMs))
        {
            robotState.driveAutonomous(
                driveCommand.left,
                driveCommand.right);
            robotState.updateAutonomousStatus(makeDistanceStatus(
                "distance_correction",
                "Aplicando correção curta de distância",
                activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm,
                progressPercent));
            return;
        }

        robotState.driveAutonomous(0.0, 0.0);
        distancePhase_ = DistancePhase::Settling;
        distancePhaseStartedAt_ = now;
    }

    robotState.driveAutonomous(0.0, 0.0);
    if (now - distancePhaseStartedAt_ <
        std::chrono::milliseconds(config::kDriveDistanceSettleMs))
    {
        robotState.updateAutonomousStatus(makeDistanceStatus(
            "distance_settling", "Aguardando os encoders estabilizarem",
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm,
            progressPercent));
        return;
    }

    if (minimumDistanceCm >=
        activeDistanceTargetCm_ - config::kDriveDistanceToleranceCm)
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeDistanceStatus(
            "distance_completed", "Distância concluída e registrada",
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm, 100.0));
        distancePhase_ = DistancePhase::Idle;
        return;
    }

    if (distanceCorrectionPulseCount_ >=
        config::kDriveDistanceMaximumCorrectionPulses)
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeDistanceStatus(
            "distance_correction_failed",
            "Percurso parado: correções insuficientes",
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm,
            progressPercent));
        distancePhase_ = DistancePhase::Idle;
        return;
    }

    ++distanceCorrectionPulseCount_;
    distancePhase_ = DistancePhase::CorrectionPulse;
    distancePhaseStartedAt_ = now;
    distanceLastProgressAt_ = now;
    lastDistanceProgressCounts_ = minimumCounts;
    robotState.driveAutonomous(
        driveCommand.left,
        driveCommand.right);
    robotState.updateAutonomousStatus(makeDistanceStatus(
        "distance_correction", "Completando os centímetros restantes",
        activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm,
        progressPercent));
}

void MissionController::resetMissionState()
{
    mainMission_.reset();
    rescueAreaMission_.reset();
    obstacleAvoidanceTest_.reset();
    testTurnController_.reset();
    distancePhase_ = DistancePhase::Idle;
    activeDistanceTargetCm_ = 0.0;
    lastDistanceProgressCounts_ = 0.0;
    distanceCorrectionPulseCount_ = 0;
    distanceDifferenceSamples_ = 0;
    distanceLastDifferenceUptimeMs_ = 0;
}
