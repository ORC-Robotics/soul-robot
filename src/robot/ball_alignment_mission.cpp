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
}

BallAlignmentOutput BallAlignmentMission::update(
    const ForwardBallSnapshot& ball,
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
    if (phase_ == Phase::Aligned)
    {
        if (absoluteTx <= config::kBallAlignmentExitDeadbandDegrees)
        {
            output.status = makeStatus(
                "ball_aligned",
                "Bola alinhada; tx=" + txText(ball.txDegrees),
                100.0);
            return output;
        }
        phase_ = Phase::Idle;
    }

    if (phase_ == Phase::TurningPulse)
    {
        if (now - phaseStartedAt_ <
            std::chrono::milliseconds(config::kBallAlignmentPulseMs))
        {
            output.leftPower =
                pulseDirection_ * config::kBallAlignmentTurnPower;
            output.rightPower = -output.leftPower;
            output.status = makeStatus(
                "ball_alignment_turning",
                std::string("Pulso para a ") +
                    (pulseDirection_ > 0.0 ? "direita" : "esquerda") +
                    "; tx=" + txText(ball.txDegrees),
                progress);
            return output;
        }
        phase_ = Phase::Settling;
        phaseStartedAt_ = now;
    }

    if (phase_ == Phase::Settling)
    {
        if (now - phaseStartedAt_ <
            std::chrono::milliseconds(config::kBallAlignmentSettleMs))
        {
            output.status = makeStatus(
                "ball_alignment_settling",
                "PWM zerado: aguardando nova leitura de tx",
                progress);
            return output;
        }
        phase_ = Phase::Idle;
    }

    if (absoluteTx <= config::kBallAlignmentDeadbandDegrees)
    {
        phase_ = Phase::Aligned;
        output.status = makeStatus(
            "ball_aligned",
            "Bola alinhada; tx=" + txText(ball.txDegrees),
            100.0);
        return output;
    }

    // tx positivo indica bola à direita: lado esquerdo avança e o direito
    // recua. tx negativo aplica exatamente o pivot oposto.
    pulseDirection_ = ball.txDegrees > 0.0 ? 1.0 : -1.0;
    phase_ = Phase::TurningPulse;
    phaseStartedAt_ = now;
    output.leftPower = pulseDirection_ * config::kBallAlignmentTurnPower;
    output.rightPower = -output.leftPower;
    output.status = makeStatus(
        "ball_alignment_turning",
        std::string("Iniciando pulso para a ") +
            (pulseDirection_ > 0.0 ? "direita" : "esquerda") +
            "; tx=" + txText(ball.txDegrees),
        progress);
    return output;
}

void BallAlignmentMission::reset()
{
    phase_ = Phase::Idle;
    pulseDirection_ = 0.0;
    phaseStartedAt_ = {};
}
