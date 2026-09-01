#include "obr/ball_alignment_mission.h"

#include "obr/config.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
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

std::string txText(double txDegrees)
{
    std::ostringstream text;
    text << std::showpos << std::fixed << std::setprecision(1)
         << txDegrees << " graus";
    return text.str();
}

double alignmentProgress(double txDegrees)
{
    // O FOV horizontal configurado é 62°; cada lado possui cerca de 31°.
    return std::clamp(
        (1.0 - std::abs(txDegrees) / 31.0) * 100.0,
        0.0,
        100.0);
}

double proportionalTurnPower(double absoluteTxDegrees)
{
    const double errorRangeDegrees =
        config::kBallAlignmentFullPowerErrorDegrees -
        config::kBallAlignmentDeadbandDegrees;
    const double normalizedError = std::clamp(
        (absoluteTxDegrees - config::kBallAlignmentDeadbandDegrees) /
            errorRangeDegrees,
        0.0,
        1.0);
    return config::kBallAlignmentMinimumRunPower +
           normalizedError *
               (config::kBallAlignmentMaximumRunPower -
                config::kBallAlignmentMinimumRunPower);
}

std::string powerText(double power)
{
    std::ostringstream text;
    text << std::fixed << std::setprecision(2) << power;
    return text.str();
}

bool encodersConfirmMotion(
    const Esp32TelemetrySnapshot& telemetry,
    double turnDirection)
{
    // A potência aplicada confirma que a taxa pertence ao pivot atual, e não
    // à inércia restante de um comando anterior ou do sentido oposto.
    return telemetry.sensorFresh &&
           std::isfinite(telemetry.leftEncoderRate) &&
           std::isfinite(telemetry.rightEncoderRate) &&
           std::isfinite(telemetry.appliedLeftPower) &&
           std::isfinite(telemetry.appliedRightPower) &&
           telemetry.appliedLeftPower * turnDirection > 0.0 &&
           telemetry.appliedRightPower * -turnDirection > 0.0 &&
           std::abs(telemetry.leftEncoderRate) >=
               config::kMotorRunConfirmationMinimumRateCountsPerSecond &&
           std::abs(telemetry.rightEncoderRate) >=
               config::kMotorRunConfirmationMinimumRateCountsPerSecond;
}
}

BallAlignmentOutput BallAlignmentMission::update(
    const ForwardBallSnapshot& ball,
    const Esp32TelemetrySnapshot& telemetry,
    std::chrono::steady_clock::time_point now)
{
    BallAlignmentOutput output;
    if (!ball.sourceFresh)
    {
        reset();
        output.status = makeStatus(
            "ball_alignment_camera_stale",
            "Parado: câmera frontal sem medição recente");
        return output;
    }
    if (!ball.detected || !std::isfinite(ball.txDegrees))
    {
        reset();
        output.status = makeStatus(
            "ball_alignment_waiting_ball",
            "Parado: aguardando a bola mais próxima");
        return output;
    }

    const double absoluteTx = std::abs(ball.txDegrees);
    const double progress = alignmentProgress(ball.txDegrees);
    if (absoluteTx <= config::kBallAlignmentDeadbandDegrees)
    {
        phase_ = Phase::Tracking;
        lastTurnDirection_ = 0.0;
        startCommandIssued_ = false;
        motionConfirmed_ = false;
        output.status = makeStatus(
            "ball_aligned",
            "Bola alinhada; tx=" + txText(ball.txDegrees),
            100.0);
        return output;
    }

    // tx positivo indica bola à direita: lado esquerdo avança e o direito
    // recua. tx negativo aplica exatamente o pivot oposto.
    const double turnDirection = ball.txDegrees > 0.0 ? 1.0 : -1.0;

    if (phase_ == Phase::BrakingAfterCrossing)
    {
        const bool brakeFinished =
            now - brakingStartedAt_ >=
            std::chrono::milliseconds(config::kBallAlignmentCrossingBrakeMs);
        const bool hasNewMeasurement = ball.timestamp > crossingTimestamp_;
        if (!brakeFinished || !hasNewMeasurement)
        {
            output.status = makeStatus(
                "ball_alignment_braking",
                "Centro cruzado: freando antes de corrigir; tx=" +
                    txText(ball.txDegrees),
                progress);
            return output;
        }
        phase_ = Phase::Tracking;
        lastTurnDirection_ = 0.0;
        startCommandIssued_ = false;
        motionConfirmed_ = false;
    }

    if (lastTurnDirection_ != 0.0 &&
        turnDirection != lastTurnDirection_)
    {
        // Uma troca de sinal entre frames significa que o centro foi cruzado.
        // Zerar o PWM evita ampliar a ultrapassagem com uma reversão imediata.
        phase_ = Phase::BrakingAfterCrossing;
        crossingTimestamp_ = ball.timestamp;
        brakingStartedAt_ = now;
        lastTurnDirection_ = 0.0;
        startCommandIssued_ = false;
        motionConfirmed_ = false;
        output.status = makeStatus(
            "ball_alignment_braking",
            "Centro cruzado: PWM zerado; tx=" + txText(ball.txDegrees),
            progress);
        return output;
    }

    if (lastTurnDirection_ != turnDirection)
    {
        startCommandIssued_ = false;
        motionConfirmed_ = false;
    }
    lastTurnDirection_ = turnDirection;
    if (startCommandIssued_ && !motionConfirmed_ &&
        encodersConfirmMotion(telemetry, turnDirection))
    {
        motionConfirmed_ = true;
    }

    const double turnPower = motionConfirmed_
        ? proportionalTurnPower(absoluteTx)
        : config::kBallAlignmentStartPower;
    output.leftPower = turnDirection * turnPower;
    output.rightPower = -output.leftPower;
    startCommandIssued_ = true;
    output.status = makeStatus(
        "ball_alignment_turning",
        std::string(motionConfirmed_ ? "Controle proporcional para a "
                                     : "Vencendo inércia para a ") +
            (turnDirection > 0.0 ? "direita" : "esquerda") +
            "; tx=" + txText(ball.txDegrees) +
            "; potência=" + powerText(turnPower),
        progress);
    return output;
}

void BallAlignmentMission::reset()
{
    phase_ = Phase::Tracking;
    lastTurnDirection_ = 0.0;
    crossingTimestamp_ = 0.0;
    startCommandIssued_ = false;
    motionConfirmed_ = false;
    brakingStartedAt_ = {};
}
