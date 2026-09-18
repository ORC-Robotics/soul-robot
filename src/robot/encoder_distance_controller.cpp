#include "obr/encoder_distance_controller.h"
#include "obr/config.h"
#include <algorithm>
#include <cmath>
#include <string>

namespace
{
AutonomousStatus makeStatus(
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

}

bool EncoderDistanceController::encodersReady(const Esp32TelemetrySnapshot& telemetry)
{
    return telemetry.sensorFresh && telemetry.lastSensorAgeMs >= 0 &&
           telemetry.lastSensorAgeMs <=
               config::kDriveDistanceEncoderFreshnessMs &&
           std::isfinite(telemetry.leftEncoderRate) &&
           std::isfinite(telemetry.rightEncoderRate);
}

bool EncoderDistanceController::encodersStopped(const Esp32TelemetrySnapshot& telemetry)
{
    return encodersReady(telemetry) &&
           std::abs(telemetry.leftEncoderRate) <=
               config::kBallAlignmentStationaryRateCountsPerSecond &&
           std::abs(telemetry.rightEncoderRate) <=
               config::kBallAlignmentStationaryRateCountsPerSecond;
}

void EncoderDistanceController::start(
    double targetCm,
    double power,
    int directionSign,
    std::chrono::steady_clock::time_point now,
    bool stopOnSideMismatch)
{
    distanceMove_ = {};
    distanceMove_.phase = DistancePhase::Preparing;
    distanceMove_.targetCm = targetCm;
    distanceMove_.power = power;
    distanceMove_.directionSign = directionSign < 0 ? -1 : 1;
    distanceMove_.stopOnSideMismatch = stopOnSideMismatch;
    distanceMove_.phaseStartedAt = now;
    distanceMove_.lastProgressAt = now;
    // O controlador agora é compartilhado: chamadas inválidas não podem iniciar motores.
    if (!std::isfinite(targetCm) || targetCm <= 0.0 || !std::isfinite(power) ||
        power < config::kMotorStartMinimumPower || power > config::kMaxMotorOutput)
    {
        distanceMove_.phase = DistancePhase::Failed;
        distanceMove_.failureStatus = makeStatus("rescue_distance_invalid", "Distância ou potência inválida");
    }
}

EncoderDistanceOutput EncoderDistanceController::update(
    const Esp32TelemetrySnapshot& telemetry,
    std::chrono::steady_clock::time_point now,
    const char* phase,
    const char* action)
{
    EncoderDistanceOutput output;
    if (distanceMove_.phase == DistancePhase::Idle)
        return output;
    if (distanceMove_.phase == DistancePhase::Failed)
    {
        output.failed = true;
        output.status = distanceMove_.failureStatus;
        return output;
    }
    if (distanceMove_.phase == DistancePhase::Completed)
    {
        output.completed = true;
        output.status = makeStatus(phase, action, 100.0);
        return output;
    }

    if (!telemetry.readyForOperation())
    {
        distanceMove_.phase = DistancePhase::Failed;
        distanceMove_.failureStatus = makeStatus(
            "rescue_distance_not_ready", "Deslocamento interrompido: ESP32 ou emergência");
        output.failed = true;
        output.status = distanceMove_.failureStatus;
        return output;
    }

    if (distanceMove_.phase == DistancePhase::Preparing)
    {
        if (!encodersStopped(telemetry))
        {
            if (now - distanceMove_.phaseStartedAt >=
                std::chrono::milliseconds(
                    config::kRescueDistancePreparationTimeoutMs))
            {
                distanceMove_.phase = DistancePhase::Failed;
                distanceMove_.failureStatus = makeStatus(
                    "rescue_distance_preparation_failed",
                    "Deslocamento cancelado: encoders indisponíveis ou rodas em movimento");
                output.failed = true;
                output.status = distanceMove_.failureStatus;
                return output;
            }
            output.status = makeStatus(
                "rescue_distance_preparing",
                "Parado: aguardando os encoders e o fim da inércia");
            return output;
        }

        distanceMove_.startLeftCount = telemetry.leftEncoderCount;
        distanceMove_.startRightCount = telemetry.rightEncoderCount;
        distanceMove_.lastUptimeMs = telemetry.esp32UptimeMs;
        distanceMove_.lastProgressCounts = 0.0;
        distanceMove_.differenceSamples = 0;
        distanceMove_.phaseStartedAt = now;
        distanceMove_.lastProgressAt = now;
        distanceMove_.phase = DistancePhase::Driving;
    }

    const double leftCounts = std::abs(static_cast<double>(
        telemetry.leftEncoderCount - distanceMove_.startLeftCount));
    const double rightCounts = std::abs(static_cast<double>(
        telemetry.rightEncoderCount - distanceMove_.startRightCount));
    const double leftCm = leftCounts / config::kEncoderCountsPerCentimeter;
    const double rightCm = rightCounts / config::kEncoderCountsPerCentimeter;
    const double minimumCounts = std::min(leftCounts, rightCounts);
    const double minimumCm = std::min(leftCm, rightCm);
    const double progress = std::clamp(
        minimumCm / distanceMove_.targetCm * 100.0,
        0.0,
        100.0);

    const auto distanceStatus = [&](const std::string& statusPhase,
                                    const std::string& statusAction) {
        AutonomousStatus status = makeStatus(statusPhase, statusAction, progress);
        status.targetDistanceCm = distanceMove_.targetCm;
        status.leftDistanceCm = leftCm;
        status.rightDistanceCm = rightCm;
        status.averageDistanceCm = (leftCm + rightCm) * 0.5;
        return status;
    };
    const auto failDistance = [&](const char* statusPhase,
                                  const char* statusAction) {
        distanceMove_.phase = DistancePhase::Failed;
        distanceMove_.failureStatus = distanceStatus(statusPhase, statusAction);
        output.failed = true;
        output.status = distanceMove_.failureStatus;
    };

    if (distanceMove_.phase == DistancePhase::Settling)
    {
        output.status = distanceStatus(
            "rescue_distance_settling",
            "PWM zerado: aguardando o deslocamento estabilizar");
        if (now - distanceMove_.phaseStartedAt >=
            std::chrono::milliseconds(config::kRescueDistanceSettleMs))
        {
            distanceMove_.phase = DistancePhase::Completed;
            output.completed = true;
            output.status = distanceStatus(phase, action);
        }
        return output;
    }

    if (!encodersReady(telemetry))
    {
        failDistance(
            "rescue_distance_encoder_lost",
            "Deslocamento interrompido: encoders sem dados recentes");
        return output;
    }
    if (now - distanceMove_.phaseStartedAt >=
        std::chrono::milliseconds(config::kRescueDistanceTimeoutMs))
    {
        failDistance(
            "rescue_distance_timeout",
            "Deslocamento interrompido pelo tempo limite");
        return output;
    }

    if (telemetry.esp32UptimeMs != distanceMove_.lastUptimeMs)
    {
        if (telemetry.esp32UptimeMs < distanceMove_.lastUptimeMs)
        {
            failDistance("rescue_distance_encoder_reset", "ESP32 reiniciou durante o deslocamento");
            return output;
        }
        distanceMove_.lastUptimeMs = telemetry.esp32UptimeMs;
        if (distanceMove_.stopOnSideMismatch &&
            std::abs(leftCm - rightCm) >
                config::kDriveDistanceMaximumSideDifferenceCm)
        {
            ++distanceMove_.differenceSamples;
        }
        else
        {
            distanceMove_.differenceSamples = 0;
        }
    }
    if (distanceMove_.stopOnSideMismatch &&
        distanceMove_.differenceSamples >=
        config::kDriveDistanceDifferenceConfirmationSamples)
    {
        failDistance(
            "rescue_distance_side_mismatch",
            "Deslocamento interrompido: diferença excessiva entre os encoders");
        return output;
    }

    if (minimumCounts >= distanceMove_.lastProgressCounts +
                             config::kDriveDistanceMinimumProgressCounts)
    {
        distanceMove_.lastProgressCounts = minimumCounts;
        distanceMove_.lastProgressAt = now;
    }
    if (now - distanceMove_.lastProgressAt >=
        std::chrono::milliseconds(config::kRescueDistanceStallTimeoutMs))
    {
        failDistance(
            "rescue_distance_stall",
            "Deslocamento interrompido: rodas sem progresso suficiente");
        return output;
    }

    if (minimumCm >= distanceMove_.targetCm)
    {
        distanceMove_.phase = DistancePhase::Settling;
        distanceMove_.phaseStartedAt = now;
        output.status = distanceStatus(
            "rescue_distance_settling",
            "Distância alcançada: PWM zerado para estabilização");
        return output;
    }

    const double differenceCm = leftCm - rightCm;
    const double maximumCorrection = std::max(
        0.0,
        std::min(
            config::kDriveDistanceMaximumBalanceCorrection,
            distanceMove_.power - config::kMotorRunMinimumPower));
    const double correction =
        std::abs(differenceCm) <= config::kDriveDistanceBalanceDeadbandCm
            ? 0.0
            : std::clamp(
                  (std::abs(differenceCm) -
                   config::kDriveDistanceBalanceDeadbandCm) *
                      config::kDriveDistanceBalanceGainPerCm,
                  0.0,
                  maximumCorrection);
    double leftMagnitude = distanceMove_.power;
    double rightMagnitude = distanceMove_.power;
    if (differenceCm > 0.0)
    {
        leftMagnitude -= correction;
        rightMagnitude += correction;
    }
    else if (differenceCm < 0.0)
    {
        leftMagnitude += correction;
        rightMagnitude -= correction;
    }
    output.leftPower = std::clamp(distanceMove_.directionSign * leftMagnitude,
                                 config::kMinMotorOutput, config::kMaxMotorOutput);
    output.rightPower = std::clamp(distanceMove_.directionSign * rightMagnitude,
                                  config::kMinMotorOutput, config::kMaxMotorOutput);
    output.status = distanceStatus(phase, action);
    return output;
}
