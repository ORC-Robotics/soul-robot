#include "obr/imu_turn_controller.h"

#include "obr/config.h"

#include <algorithm>
#include <cmath>

namespace
{
ImuTurnOutput stoppedOutput(
    ImuTurnResult result,
    const std::string& phase,
    const std::string& action,
    double progressPercent)
{
    ImuTurnOutput output;
    output.result = result;
    output.phase = phase;
    output.action = action;
    output.progressPercent = progressPercent;
    return output;
}
}

bool ImuTurnController::imuReady(const Esp32TelemetrySnapshot& telemetry)
{
    return telemetry.sensorFresh && telemetry.mpuOk &&
           telemetry.lastSensorAgeMs >= 0 &&
           telemetry.lastSensorAgeMs <= config::kTurn90ImuFreshnessMs &&
           std::isfinite(telemetry.yawZDeg) &&
           std::isfinite(telemetry.gyroZDegPerSec);
}

bool ImuTurnController::start(
    double targetDegrees,
    ImuTurnDirection direction,
    const Esp32TelemetrySnapshot& telemetry)
{
    if (!imuReady(telemetry) || !std::isfinite(targetDegrees) ||
        targetDegrees <= 0.0 || targetDegrees > 180.0)
    {
        reset();
        return false;
    }

    const auto now = std::chrono::steady_clock::now();
    targetDegrees_ = targetDegrees;
    startYawDegrees_ = telemetry.yawZDeg;
    directionSign_ = direction == ImuTurnDirection::Right ? 1.0 : -1.0;
    correctionDirection_ = 1.0;
    correctionPulseCount_ = 0;
    startedAt_ = now;
    phaseStartedAt_ = now;
    phase_ = Phase::Turning;
    return true;
}

ImuTurnOutput ImuTurnController::update(
    const Esp32TelemetrySnapshot& telemetry)
{
    if (phase_ == Phase::Idle)
    {
        return {};
    }

    const auto now = std::chrono::steady_clock::now();
    const int timeoutMs = targetDegrees_ > config::kTurn90TargetDegrees
                              ? config::kTurn180TimeoutMs
                              : config::kTurn90TimeoutMs;
    if (now - startedAt_ > std::chrono::milliseconds(timeoutMs))
    {
        reset();
        return stoppedOutput(
            ImuTurnResult::Failed,
            "turn_timeout",
            "Giro interrompido pelo tempo limite",
            0.0);
    }
    if (!imuReady(telemetry))
    {
        reset();
        return stoppedOutput(
            ImuTurnResult::Failed,
            "turn_imu_lost",
            "Giro interrompido: MPU6050 sem amostra recente",
            0.0);
    }

    const double turnedDegrees = angularDistanceDegrees(
        startYawDegrees_, telemetry.yawZDeg);
    const double remainingDegrees = targetDegrees_ - turnedDegrees;
    const double progressPercent = std::clamp(
        turnedDegrees / targetDegrees_ * 100.0, 0.0, 100.0);

    if (phase_ == Phase::Turning)
    {
        const double predictionSeconds =
            config::kTurn90BrakePredictionSeconds +
            telemetry.lastSensorAgeMs / 1000.0;
        const double brakeLeadDegrees = std::clamp(
            std::abs(telemetry.gyroZDegPerSec) * predictionSeconds,
            config::kTurn90StopToleranceDegrees,
            config::kTurn90MaximumBrakeLeadDegrees);
        if (remainingDegrees <= brakeLeadDegrees)
        {
            phase_ = Phase::Settling;
            phaseStartedAt_ = now;
            return stoppedOutput(
                ImuTurnResult::Running,
                "turn_settling",
                "PWM zerado: aguardando o giro estabilizar",
                progressPercent);
        }

        ImuTurnOutput output;
        output.result = ImuTurnResult::Running;
        output.leftPower = directionSign_ * config::kTurn90CommandPower;
        output.rightPower = -output.leftPower;
        output.progressPercent = progressPercent;
        output.phase = "turning";
        if (targetDegrees_ > 90.0)
        {
            output.action = "Executando retorno de 180° pelo MPU6050";
        }
        else
        {
            const int targetDegrees = static_cast<int>(std::round(targetDegrees_));
            output.action = "Executando giro de " +
                            std::to_string(targetDegrees) + "° " +
                            (directionSign_ > 0.0 ? "à direita" : "à esquerda");
        }
        return output;
    }

    if (phase_ == Phase::CorrectionPulse)
    {
        if (std::abs(remainingDegrees) <= config::kTurn90StopToleranceDegrees ||
            remainingDegrees * correctionDirection_ <= 0.0)
        {
            phase_ = Phase::Settling;
            phaseStartedAt_ = now;
            return stoppedOutput(
                ImuTurnResult::Running,
                "turn_settling",
                "Alvo alcançado: aguardando estabilização",
                progressPercent);
        }
        if (now - phaseStartedAt_ <
            std::chrono::milliseconds(config::kTurn90CorrectionPulseMs))
        {
            ImuTurnOutput output;
            output.result = ImuTurnResult::Running;
            output.leftPower = directionSign_ * correctionDirection_ *
                               config::kTurn90CommandPower;
            output.rightPower = -output.leftPower;
            output.progressPercent = progressPercent;
            output.phase = "turn_correction";
            output.action = "Aplicando correção angular curta";
            return output;
        }
        phase_ = Phase::Settling;
        phaseStartedAt_ = now;
    }

    if (now - phaseStartedAt_ <
            std::chrono::milliseconds(config::kTurn90SettleMs) ||
        std::abs(telemetry.gyroZDegPerSec) >
            config::kTurn90StationaryRateDegPerSec)
    {
        return stoppedOutput(
            ImuTurnResult::Running,
            "turn_settling",
            "Aguardando a leitura angular estabilizar",
            progressPercent);
    }
    if (std::abs(remainingDegrees) <= config::kTurn90StopToleranceDegrees)
    {
        reset();
        return stoppedOutput(
            ImuTurnResult::Completed,
            "turn_completed",
            "Giro pelo MPU6050 concluído",
            100.0);
    }
    if (correctionPulseCount_ >= config::kTurn90MaximumCorrectionPulses)
    {
        reset();
        return stoppedOutput(
            ImuTurnResult::Failed,
            "turn_correction_failed",
            "Giro parado: correções não alcançaram o alvo",
            progressPercent);
    }

    ++correctionPulseCount_;
    correctionDirection_ = remainingDegrees > 0.0 ? 1.0 : -1.0;
    phase_ = Phase::CorrectionPulse;
    phaseStartedAt_ = now;
    return update(telemetry);
}

void ImuTurnController::reset()
{
    phase_ = Phase::Idle;
    targetDegrees_ = 0.0;
    correctionPulseCount_ = 0;
}

bool ImuTurnController::active() const
{
    return phase_ != Phase::Idle;
}

double ImuTurnController::angularDistanceDegrees(double first, double second)
{
    double difference = std::fmod(std::abs(second - first), 360.0);
    if (difference > 180.0)
    {
        difference = 360.0 - difference;
    }
    return difference;
}
