#include "obr/obstacle_avoidance.h"

#include "obr/config.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace
{
bool ultrasonicReadingIsValid(const Esp32TelemetrySnapshot& telemetry)
{
    return telemetry.sensorFresh && telemetry.lastSensorAgeMs >= 0 &&
           telemetry.lastSensorAgeMs <= config::kObstacleEncoderFreshnessMs &&
           std::isfinite(telemetry.ultrasonicDistanceCm) &&
           telemetry.ultrasonicDistanceCm >= 2.0 &&
           telemetry.ultrasonicDistanceCm <= 400.0;
}

bool encodersAreReady(const Esp32TelemetrySnapshot& telemetry)
{
    return telemetry.sensorFresh && telemetry.lastSensorAgeMs >= 0 &&
           telemetry.lastSensorAgeMs <= config::kObstacleEncoderFreshnessMs &&
           std::isfinite(telemetry.leftEncoderRate) &&
           std::isfinite(telemetry.rightEncoderRate);
}

ObstacleAvoidanceOutput stoppedOutput(
    const std::string& phase,
    const std::string& action,
    double progressPercent = 0.0)
{
    ObstacleAvoidanceOutput output;
    output.hasControl = true;
    output.phase = phase;
    output.action = action;
    output.progressPercent = progressPercent;
    return output;
}
}

ObstacleAvoidanceOutput ObstacleAvoidance::update(
    const Esp32TelemetrySnapshot& telemetry,
    bool allowStart)
{
    if (phase_ == Phase::Idle)
    {
        return updateIdle(telemetry, allowStart);
    }

    if (phase_ == Phase::TurningRight ||
        phase_ == Phase::TurningLeft45 ||
        phase_ == Phase::TurningLeft90 ||
        phase_ == Phase::TurningRight90)
    {
        return updateTurn(telemetry);
    }

    if (phase_ == Phase::InitialSettling ||
        phase_ == Phase::FirstForwardSettling ||
        phase_ == Phase::SecondForwardSettling ||
        phase_ == Phase::ThirdForwardSettling ||
        phase_ == Phase::ReverseSettling)
    {
        return updateSettling(telemetry);
    }

    if (phase_ == Phase::FirstForward)
    {
        return updateDistance(
            telemetry,
            config::kObstacleFirstForwardDistanceCm,
            config::kObstacleForwardPower,
            "obstacle_first_forward",
            "Desvio: avançando ao lado do obstáculo");
    }
    if (phase_ == Phase::SecondForward)
    {
        return updateDistance(
            telemetry,
            config::kObstacleSecondForwardDistanceCm,
            config::kObstacleForwardPower,
            "obstacle_second_forward",
            "Desvio: ultrapassando o comprimento do obstáculo");
    }
    if (phase_ == Phase::ThirdForward)
    {
        return updateDistance(
            telemetry,
            config::kObstacleThirdForwardDistanceCm,
            config::kObstacleForwardPower,
            "obstacle_third_forward",
            "Desvio: aproximando-se novamente da linha");
    }

    return updateDistance(
        telemetry,
        config::kObstacleReverseDistanceCm,
        -config::kObstacleReversePower,
        "obstacle_reversing",
        "Desvio: recuando para posicionar a câmera sobre a linha");
}

void ObstacleAvoidance::reset()
{
    phase_ = Phase::Idle;
    turnController_.reset();
    armed_ = true;
    obstacleConfirmationSamples_ = 0;
    rearmConfirmationSamples_ = 0;
    distanceStartLeftCount_ = 0;
    distanceStartRightCount_ = 0;
}

bool ObstacleAvoidance::active() const
{
    return phase_ != Phase::Idle;
}

ObstacleAvoidanceOutput ObstacleAvoidance::updateIdle(
    const Esp32TelemetrySnapshot& telemetry,
    bool allowStart)
{
    if (!ultrasonicReadingIsValid(telemetry))
    {
        obstacleConfirmationSamples_ = 0;
        rearmConfirmationSamples_ = 0;
        return {};
    }

    if (!armed_)
    {
        if (telemetry.ultrasonicDistanceCm >= config::kObstacleRearmDistanceCm)
        {
            ++rearmConfirmationSamples_;
            if (rearmConfirmationSamples_ >=
                config::kObstacleRearmConfirmationSamples)
            {
                armed_ = true;
                rearmConfirmationSamples_ = 0;
            }
        }
        else
        {
            rearmConfirmationSamples_ = 0;
        }
        return {};
    }

    if (!allowStart ||
        telemetry.ultrasonicDistanceCm > config::kObstacleDetectionDistanceCm)
    {
        obstacleConfirmationSamples_ = 0;
        return {};
    }

    ++obstacleConfirmationSamples_;
    if (obstacleConfirmationSamples_ <
        config::kObstacleDetectionConfirmationSamples)
    {
        return {};
    }

    if (!encodersAreReady(telemetry) ||
        !ImuTurnController::imuReady(telemetry))
    {
        return stoppedOutput(
            "obstacle_waiting_sensors",
            "Obstáculo detectado: aguardando encoders e MPU6050");
    }

    armed_ = false;
    obstacleConfirmationSamples_ = 0;
    startSettling(Phase::InitialSettling);
    return stoppedOutput(
        "obstacle_detected",
        "Obstáculo confirmado: parando antes do giro à direita");
}

ObstacleAvoidanceOutput ObstacleAvoidance::updateTurn(
    const Esp32TelemetrySnapshot& telemetry)
{
    const ImuTurnOutput turnOutput = turnController_.update(telemetry);
    if (turnOutput.result == ImuTurnResult::Failed)
    {
        return fail(
            "obstacle_" + turnOutput.phase,
            "Desvio: " + turnOutput.action);
    }

    if (turnOutput.result != ImuTurnResult::Completed)
    {
        ObstacleAvoidanceOutput output;
        output.hasControl = true;
        output.leftPower = turnOutput.leftPower;
        output.rightPower = turnOutput.rightPower;
        output.progressPercent = turnOutput.progressPercent;
        output.phase = "obstacle_" + turnOutput.phase;
        output.action = "Desvio: " + turnOutput.action;
        return output;
    }

    if (phase_ == Phase::TurningRight)
    {
        startDistance(Phase::FirstForward, telemetry);
        return stoppedOutput(
            "obstacle_first_forward_start",
            "Giro à direita concluído: iniciando a primeira reta");
    }
    if (phase_ == Phase::TurningLeft45)
    {
        startDistance(Phase::SecondForward, telemetry);
        return stoppedOutput(
            "obstacle_second_forward_start",
            "Primeiro giro à esquerda concluído: iniciando a segunda reta");
    }

    if (phase_ == Phase::TurningLeft90)
    {
        startDistance(Phase::ThirdForward, telemetry);
        return stoppedOutput(
            "obstacle_third_forward_start",
            "Segundo giro à esquerda concluído: aproximando-se da linha");
    }

    startDistance(Phase::Reversing, telemetry);
    return stoppedOutput(
        "obstacle_reverse_start",
        "Giro final de 90° concluído: iniciando a ré de 5 cm");
}

ObstacleAvoidanceOutput ObstacleAvoidance::updateSettling(
    const Esp32TelemetrySnapshot& telemetry)
{
    if (std::chrono::steady_clock::now() - phaseStartedAt_ <
        std::chrono::milliseconds(config::kObstacleStageSettleMs))
    {
        return stoppedOutput(
            "obstacle_settling",
            "Desvio: aguardando o robô estabilizar antes da próxima etapa");
    }

    if (phase_ == Phase::InitialSettling)
    {
        if (!startTurn(
                Phase::TurningRight,
                config::kObstacleFirstRightTurnDegrees,
                ImuTurnDirection::Right,
                telemetry))
        {
            return fail(
                "obstacle_turn_start_failed",
                "Desvio interrompido: não foi possível iniciar o giro à direita");
        }
        return updateTurn(telemetry);
    }
    if (phase_ == Phase::FirstForwardSettling)
    {
        if (!startTurn(
                Phase::TurningLeft45,
                config::kObstacleFirstLeftTurnDegrees,
                ImuTurnDirection::Left,
                telemetry))
        {
            return fail(
                "obstacle_turn_start_failed",
                "Desvio interrompido: não foi possível iniciar o primeiro giro à esquerda");
        }
        return updateTurn(telemetry);
    }
    if (phase_ == Phase::SecondForwardSettling)
    {
        if (!startTurn(
                Phase::TurningLeft90,
                config::kObstacleSecondLeftTurnDegrees,
                ImuTurnDirection::Left,
                telemetry))
        {
            return fail(
                "obstacle_turn_start_failed",
                "Desvio interrompido: não foi possível iniciar o segundo giro à esquerda");
        }
        return updateTurn(telemetry);
    }

    if (phase_ == Phase::ThirdForwardSettling)
    {
        if (!startTurn(
                Phase::TurningRight90,
                config::kObstacleFinalRightTurnDegrees,
                ImuTurnDirection::Right,
                telemetry))
        {
            return fail(
                "obstacle_turn_start_failed",
                "Desvio interrompido: não foi possível iniciar o giro final à direita");
        }
        return updateTurn(telemetry);
    }

    phase_ = Phase::Idle;
    ObstacleAvoidanceOutput output;
    output.completed = true;
    output.progressPercent = 100.0;
    output.phase = "obstacle_completed";
    output.action = "Desvio concluído: ré de 5 cm finalizada";
    return output;
}

ObstacleAvoidanceOutput ObstacleAvoidance::updateDistance(
    const Esp32TelemetrySnapshot& telemetry,
    double targetDistanceCm,
    double motorPower,
    const std::string& phase,
    const std::string& action)
{
    const double leftCounts = std::abs(static_cast<double>(
        telemetry.leftEncoderCount - distanceStartLeftCount_));
    const double rightCounts = std::abs(static_cast<double>(
        telemetry.rightEncoderCount - distanceStartRightCount_));
    const double leftDistanceCm =
        leftCounts / config::kEncoderCountsPerCentimeter;
    const double rightDistanceCm =
        rightCounts / config::kEncoderCountsPerCentimeter;
    const double minimumDistanceCm = std::min(leftDistanceCm, rightDistanceCm);
    const double progressPercent = std::clamp(
        minimumDistanceCm / targetDistanceCm * 100.0, 0.0, 100.0);

    if (!encodersAreReady(telemetry))
    {
        return fail(
            "obstacle_encoder_lost",
            "Desvio interrompido: encoders sem dados recentes");
    }
    if (std::chrono::steady_clock::now() - phaseStartedAt_ >
        std::chrono::milliseconds(config::kObstacleDistanceSafetyTimeoutMs))
    {
        return fail(
            "obstacle_distance_timeout",
            "Desvio interrompido: deslocamento excedeu o limite de segurança");
    }

    const double predictionSeconds =
        config::kObstacleBrakePredictionSeconds +
        telemetry.lastSensorAgeMs / 1000.0;
    const double projectedLeftCounts =
        leftCounts + std::abs(telemetry.leftEncoderRate) * predictionSeconds;
    const double projectedRightCounts =
        rightCounts + std::abs(telemetry.rightEncoderRate) * predictionSeconds;
    const double targetCounts =
        targetDistanceCm * config::kEncoderCountsPerCentimeter;

    if (std::min(projectedLeftCounts, projectedRightCounts) >= targetCounts)
    {
        if (phase_ == Phase::FirstForward)
        {
            startSettling(Phase::FirstForwardSettling);
        }
        else if (phase_ == Phase::SecondForward)
        {
            startSettling(Phase::SecondForwardSettling);
        }
        else if (phase_ == Phase::ThirdForward)
        {
            startSettling(Phase::ThirdForwardSettling);
        }
        else
        {
            startSettling(Phase::ReverseSettling);
        }

        ObstacleAvoidanceOutput output = stoppedOutput(
            "obstacle_stage_completed",
            "Reta concluída: motores parados antes da próxima etapa",
            100.0);
        output.targetDistanceCm = targetDistanceCm;
        output.leftDistanceCm = leftDistanceCm;
        output.rightDistanceCm = rightDistanceCm;
        return output;
    }

    ObstacleAvoidanceOutput output;
    output.hasControl = true;
    output.leftPower = motorPower;
    output.rightPower = motorPower;
    output.progressPercent = progressPercent;
    output.targetDistanceCm = targetDistanceCm;
    output.leftDistanceCm = leftDistanceCm;
    output.rightDistanceCm = rightDistanceCm;
    output.phase = phase;
    output.action = action;
    return output;
}

bool ObstacleAvoidance::startTurn(
    Phase phase,
    double degrees,
    ImuTurnDirection direction,
    const Esp32TelemetrySnapshot& telemetry)
{
    if (!turnController_.start(
            degrees,
            direction,
            telemetry,
            config::kObstacleTurnToleranceDegrees))
    {
        return false;
    }
    phase_ = phase;
    phaseStartedAt_ = std::chrono::steady_clock::now();
    return true;
}

void ObstacleAvoidance::startDistance(
    Phase phase,
    const Esp32TelemetrySnapshot& telemetry)
{
    phase_ = phase;
    distanceStartLeftCount_ = telemetry.leftEncoderCount;
    distanceStartRightCount_ = telemetry.rightEncoderCount;
    phaseStartedAt_ = std::chrono::steady_clock::now();
}

void ObstacleAvoidance::startSettling(Phase phase)
{
    phase_ = phase;
    phaseStartedAt_ = std::chrono::steady_clock::now();
}

ObstacleAvoidanceOutput ObstacleAvoidance::fail(
    const std::string& phase,
    const std::string& action)
{
    turnController_.reset();
    phase_ = Phase::Idle;
    obstacleConfirmationSamples_ = 0;
    ObstacleAvoidanceOutput output = stoppedOutput(phase, action);
    output.failed = true;
    return output;
}
