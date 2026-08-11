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
}

void MissionController::update(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    bool cameraReady,
    const CameraLineSnapshot& cameraLineSnapshot)
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
        turn90Phase_ = Turn90Phase::Idle;
        updateDriveDistance(
            robotState, esp32Telemetry, snapshot.driveDistanceTargetCm);
        return;
    case AutonomousMission::MainMission:
    default:
        turn90Phase_ = Turn90Phase::Idle;
        distancePhase_ = DistancePhase::Idle;
        mainMission_.update(
            robotState,
            esp32Telemetry,
            cameraReady,
            cameraLineSnapshot);
        return;
    }
}

void MissionController::updateTurnRight90(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry)
{
    const auto now = std::chrono::steady_clock::now();
    const bool imuReady = esp32Telemetry.sensorFresh && esp32Telemetry.mpuOk &&
                          esp32Telemetry.lastSensorAgeMs >= 0 &&
                          esp32Telemetry.lastSensorAgeMs <=
                              config::kTurn90ImuFreshnessMs &&
                          std::isfinite(esp32Telemetry.yawZDeg) &&
                          std::isfinite(esp32Telemetry.gyroZDegPerSec);

    if (turn90Phase_ == Turn90Phase::Idle)
    {
        if (!imuReady)
        {
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeAutonomousStatus(
                "waiting_imu", "Aguardando referência angular do MPU6050"));
            return;
        }

        turn90Phase_ = Turn90Phase::Turning;
        turn90StartYawDegrees_ = esp32Telemetry.yawZDeg;
        turn90StartedAt_ = now;
        turn90PhaseStartedAt_ = now;
        turn90CorrectionPulseCount_ = 0;
        turn90CorrectionDirection_ = 1.0;
        std::cout << "Turn-right-90 mission started at yaw="
                  << turn90StartYawDegrees_ << " deg\n";
    }

    if (now - turn90StartedAt_ >
        std::chrono::milliseconds(config::kTurn90TimeoutMs))
    {
        // O modo de teste depende do MPU6050. O limite evita movimento
        // indefinido se o ângulo deixar de responder.
        robotState.stop();
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "turn_timeout", "Giro interrompido pelo tempo limite"));
        turn90Phase_ = Turn90Phase::Idle;
        return;
    }

    if (!imuReady)
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "turn_imu_lost",
            "Giro interrompido: amostra recente do MPU6050 indisponível"));
        turn90Phase_ = Turn90Phase::Idle;
        return;
    }

    const double turnedDegrees = angularDistanceDegrees(
        turn90StartYawDegrees_, esp32Telemetry.yawZDeg);
    const double remainingDegrees = config::kTurn90TargetDegrees - turnedDegrees;
    const double progressPercent = std::clamp(
        turnedDegrees / config::kTurn90TargetDegrees * 100.0, 0.0, 100.0);

    if (turn90Phase_ == Turn90Phase::Turning)
    {
        const double predictionSeconds =
            config::kTurn90BrakePredictionSeconds +
            esp32Telemetry.lastSensorAgeMs / 1000.0;
        const double brakeLeadDegrees = std::clamp(
            std::abs(esp32Telemetry.gyroZDegPerSec) * predictionSeconds,
            config::kTurn90StopToleranceDegrees,
            config::kTurn90MaximumBrakeLeadDegrees);

        if (remainingDegrees <= brakeLeadDegrees)
        {
            robotState.driveAutonomous(0.0, 0.0);
            turn90Phase_ = Turn90Phase::Settling;
            turn90PhaseStartedAt_ = now;
            robotState.updateAutonomousStatus(makeAutonomousStatus(
                "turn_settling",
                "PWM zerado: aguardando o giro estabilizar",
                progressPercent));
            return;
        }

        robotState.driveAutonomous(
            config::kTurn90CommandPower,
            -config::kTurn90CommandPower);
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "turning_right_90",
            "Girando 90° à direita com referência do MPU6050",
            progressPercent));
        return;
    }

    if (turn90Phase_ == Turn90Phase::CorrectionPulse)
    {
        const bool targetReached =
            std::abs(remainingDegrees) <= config::kTurn90StopToleranceDegrees;
        const bool targetCrossedDuringPulse =
            remainingDegrees * turn90CorrectionDirection_ <= 0.0;
        if (targetReached || targetCrossedDuringPulse)
        {
            robotState.driveAutonomous(0.0, 0.0);
            turn90Phase_ = Turn90Phase::Settling;
            turn90PhaseStartedAt_ = now;
            robotState.updateAutonomousStatus(makeAutonomousStatus(
                "turn_settling",
                "Alvo alcançado: aguardando o giro estabilizar",
                progressPercent));
            return;
        }

        if (now - turn90PhaseStartedAt_ <
            std::chrono::milliseconds(config::kTurn90CorrectionPulseMs))
        {
            robotState.driveAutonomous(
                turn90CorrectionDirection_ * config::kTurn90CommandPower,
                -turn90CorrectionDirection_ * config::kTurn90CommandPower);
            robotState.updateAutonomousStatus(makeAutonomousStatus(
                "turn_correction",
                std::string(turn90CorrectionDirection_ > 0.0
                                ? "Completando ângulo com correção curta nº "
                                : "Reduzindo excesso com correção reversa nº ") +
                    std::to_string(turn90CorrectionPulseCount_),
                progressPercent));
            return;
        }

        robotState.driveAutonomous(0.0, 0.0);
        turn90Phase_ = Turn90Phase::Settling;
        turn90PhaseStartedAt_ = now;
    }

    robotState.driveAutonomous(0.0, 0.0);
    const bool angularMotionStopped =
        std::abs(esp32Telemetry.gyroZDegPerSec) <=
        config::kTurn90StationaryRateDegPerSec;
    if (now - turn90PhaseStartedAt_ <
            std::chrono::milliseconds(config::kTurn90SettleMs) ||
        !angularMotionStopped)
    {
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "turn_settling",
            "Aguardando a leitura angular estabilizar",
            progressPercent));
        return;
    }

    if (std::abs(remainingDegrees) <= config::kTurn90StopToleranceDegrees)
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "completed", "Giro de 90° concluído", 100.0));
        turn90Phase_ = Turn90Phase::Idle;
        return;
    }

    if (turn90CorrectionPulseCount_ >=
        config::kTurn90MaximumCorrectionPulses)
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "turn_correction_failed",
            "Giro parado: correções não alcançaram 90°",
            progressPercent));
        turn90Phase_ = Turn90Phase::Idle;
        return;
    }

    ++turn90CorrectionPulseCount_;
    turn90CorrectionDirection_ = remainingDegrees > 0.0 ? 1.0 : -1.0;
    turn90Phase_ = Turn90Phase::CorrectionPulse;
    turn90PhaseStartedAt_ = now;
    robotState.driveAutonomous(
        turn90CorrectionDirection_ * config::kTurn90CommandPower,
        -turn90CorrectionDirection_ * config::kTurn90CommandPower);
    robotState.updateAutonomousStatus(makeAutonomousStatus(
        "turn_correction",
        turn90CorrectionDirection_ > 0.0
            ? "Aplicando correção para completar o ângulo"
            : "Aplicando correção reversa para reduzir o excesso",
        progressPercent));
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
            config::kDriveDistanceCommandPower,
            config::kDriveDistanceCommandPower);
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
                config::kDriveDistanceCommandPower,
                config::kDriveDistanceCommandPower);
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
        config::kDriveDistanceCommandPower,
        config::kDriveDistanceCommandPower);
    robotState.updateAutonomousStatus(makeDistanceStatus(
        "distance_correction", "Completando os centímetros restantes",
        activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm,
        progressPercent));
}

void MissionController::resetMissionState()
{
    mainMission_.reset();
    turn90Phase_ = Turn90Phase::Idle;
    turn90CorrectionPulseCount_ = 0;
    turn90CorrectionDirection_ = 1.0;
    distancePhase_ = DistancePhase::Idle;
    activeDistanceTargetCm_ = 0.0;
    lastDistanceProgressCounts_ = 0.0;
    distanceCorrectionPulseCount_ = 0;
}

double MissionController::angularDistanceDegrees(double first, double second)
{
    double difference = std::fmod(second - first, 360.0);
    if (difference < -180.0)
    {
        difference += 360.0;
    }
    else if (difference > 180.0)
    {
        difference -= 360.0;
    }
    return std::abs(difference);
}
