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
    // O campo horizontal configurado possui cerca de 31° para cada lado.
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

double approachSteeringCorrection(double txDegrees)
{
    const double absoluteTx = std::abs(txDegrees);
    if (absoluteTx <= config::kBallAlignmentDeadbandDegrees)
    {
        return 0.0;
    }

    const double normalizedError = std::clamp(
        (absoluteTx - config::kBallAlignmentDeadbandDegrees) /
            (config::kBallApproachFullSteeringErrorDegrees -
             config::kBallAlignmentDeadbandDegrees),
        0.0,
        1.0);
    return std::copysign(
        normalizedError * config::kBallApproachMaximumSteeringCorrection,
        txDegrees);
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
    // Os encoders confirmam somente que o pivot começou. Eles não definem
    // o ângulo final, que continua sendo controlado pelo tx do alvo travado.
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

bool encodersConfirmStop(const Esp32TelemetrySnapshot& telemetry)
{
    // A posição visual só é aceita depois que os dois lados confirmam que a
    // inércia terminou. As contagens não são convertidas em objetivo angular.
    return telemetry.sensorFresh &&
           std::isfinite(telemetry.leftEncoderRate) &&
           std::isfinite(telemetry.rightEncoderRate) &&
           std::abs(telemetry.leftEncoderRate) <=
               config::kBallAlignmentStationaryRateCountsPerSecond &&
           std::abs(telemetry.rightEncoderRate) <=
               config::kBallAlignmentStationaryRateCountsPerSecond;
}
}

BallAlignmentOutput BallAlignmentMission::update(
    const ForwardBallSnapshot& ball,
    const Esp32TelemetrySnapshot& telemetry,
    std::uint64_t expectedTargetSequence,
    std::chrono::steady_clock::time_point now)
{
    BallAlignmentOutput output;
    if (expectedTargetSequence_ != expectedTargetSequence)
    {
        reset();
        expectedTargetSequence_ = expectedTargetSequence;
    }

    if (phase_ == Phase::Completed)
    {
        output.finished = true;
        output.status = makeStatus(
            "ball_reached",
            "Bola alcançada; alvo permanece travado até uma nova execução",
            100.0);
        return output;
    }
    if (phase_ == Phase::Failed)
    {
        output.finished = true;
        output.status = makeStatus(failurePhase_, failureAction_);
        return output;
    }

    const bool currentTarget =
        ball.targetSequence == expectedTargetSequence_ && ball.targetLocked;
    const bool validMeasurement =
        currentTarget && ball.sourceFresh && ball.detected &&
        std::isfinite(ball.txDegrees);
    if (!validMeasurement)
    {
        // Qualquer perda ou dado antigo zera a saída neste mesmo ciclo. Antes
        // da primeira aquisição, o tempo de falha ainda não deve começar.
        if (!targetAcquired_)
        {
            output.status = makeStatus(
                ball.sourceFresh
                    ? "ball_alignment_waiting_target"
                    : "ball_alignment_camera_stale",
                currentTarget
                    ? "Parado: aguardando a confirmação do alvo"
                    : "Parado: aguardando o alvo da execução atual");
            return output;
        }

        if (!targetLossActive_)
        {
            targetLossActive_ = true;
            targetLostAt_ = now;
        }
        if (now - targetLostAt_ >= std::chrono::milliseconds(
                                      config::kBallAlignmentTargetLossTimeoutMs))
        {
            phase_ = Phase::Failed;
            output.finished = true;
            failurePhase_ = "ball_alignment_target_lost_timeout";
            failureAction_ =
                "Parado: o alvo travado não reapareceu em até 1 segundo";
            output.status = makeStatus(failurePhase_, failureAction_);
            return output;
        }

        output.status = makeStatus(
            "ball_alignment_target_lost",
            "Parado: procurando somente o mesmo alvo travado");
        return output;
    }

    targetAcquired_ = true;
    targetLossActive_ = false;
    const double absoluteTx = std::abs(ball.txDegrees);
    const double progress = alignmentProgress(ball.txDegrees);

    if (phase_ == Phase::Approaching)
    {
        if (!std::isfinite(ball.distanceCm) || ball.distanceCm <= 0.0)
        {
            output.status = makeStatus(
                "ball_approach_waiting_distance",
                "Parado: aguardando distância válida da bola",
                progress);
            return output;
        }
        if (ball.distanceCm <= config::kBallApproachStopDistanceCm)
        {
            phase_ = Phase::Completed;
            output.finished = true;
            output.status = makeStatus(
                "ball_reached",
                "Bola alcançada a " + powerText(ball.distanceCm) + " cm",
                100.0);
            return output;
        }

        // tx positivo indica a bola à direita. Aumentar o lado esquerdo e
        // reduzir o direito corrige o rumo sem interromper o avanço.
        const double steeringCorrection =
            approachSteeringCorrection(ball.txDegrees);
        output.leftPower = std::clamp(
            config::kBallApproachBasePower + steeringCorrection,
            config::kMinMotorOutput,
            config::kMaxMotorOutput);
        output.rightPower = std::clamp(
            config::kBallApproachBasePower - steeringCorrection,
            config::kMinMotorOutput,
            config::kMaxMotorOutput);
        output.status = makeStatus(
            "ball_approaching",
            "Avançando com correção angular; tx=" +
                txText(ball.txDegrees) + "; distância=" +
                powerText(ball.distanceCm) + " cm",
            progress);
        return output;
    }

    if (phase_ == Phase::FineCorrectionPulse)
    {
        if (now - phaseStartedAt_ < std::chrono::milliseconds(
                                        config::kBallAlignmentFineCorrectionPulseMs))
        {
            output.leftPower =
                fineCorrectionDirection_ * config::kBallAlignmentStartPower;
            output.rightPower = -output.leftPower;
            output.status = makeStatus(
                "ball_alignment_fine_correction",
                "Pulso curto para " +
                    std::string(fineCorrectionDirection_ > 0.0
                                    ? "direita"
                                    : "esquerda") +
                    "; tx=" + txText(ball.txDegrees),
                progress);
            return output;
        }

        // Depois de cada pulso, o PWM permanece zerado antes de usar outro tx.
        phase_ = Phase::SettlingForVerification;
        phaseStartedAt_ = now;
        phaseStartBallTimestamp_ = ball.timestamp;
        stableFrameCount_ = 0;
        lastStableBallTimestamp_ = 0.0;
        lastTurnDirection_ = 0.0;
        startCommandIssued_ = false;
        motionConfirmed_ = false;
        output.status = makeStatus(
            "ball_alignment_braking",
            "Pulso concluído: aguardando o robô parar",
            progress);
        return output;
    }

    if (phase_ == Phase::SettlingForVerification)
    {
        const bool brakeFinished =
            now - phaseStartedAt_ >=
            std::chrono::milliseconds(config::kBallAlignmentCrossingBrakeMs);
        const bool hasNewMeasurement =
            ball.timestamp > phaseStartBallTimestamp_;
        if (!brakeFinished || !hasNewMeasurement ||
            !encodersConfirmStop(telemetry))
        {
            output.status = makeStatus(
                "ball_alignment_braking",
                "PWM zerado: aguardando a parada antes de verificar; tx=" +
                    txText(ball.txDegrees),
                progress);
            return output;
        }

        if (absoluteTx <= config::kBallAlignmentDeadbandDegrees)
        {
            if (ball.timestamp > lastStableBallTimestamp_)
            {
                lastStableBallTimestamp_ = ball.timestamp;
                ++stableFrameCount_;
            }
            if (stableFrameCount_ >= config::kBallAlignmentStableFrames)
            {
                phase_ = Phase::Approaching;
                if (!std::isfinite(ball.distanceCm) ||
                    ball.distanceCm <= 0.0)
                {
                    output.status = makeStatus(
                        "ball_approach_waiting_distance",
                        "Alinhada; aguardando distância válida da bola",
                        100.0);
                    return output;
                }
                if (ball.distanceCm <= config::kBallApproachStopDistanceCm)
                {
                    phase_ = Phase::Completed;
                    output.finished = true;
                    output.status = makeStatus(
                        "ball_reached",
                        "Bola já está a " + powerText(ball.distanceCm) +
                            " cm",
                        100.0);
                    return output;
                }
                const double steeringCorrection =
                    approachSteeringCorrection(ball.txDegrees);
                output.leftPower =
                    config::kBallApproachBasePower + steeringCorrection;
                output.rightPower =
                    config::kBallApproachBasePower - steeringCorrection;
                output.status = makeStatus(
                    "ball_approaching",
                    "Alinhada; avançando em direção à bola; distância=" +
                        powerText(ball.distanceCm) + " cm",
                    100.0);
                return output;
            }
            output.status = makeStatus(
                "ball_alignment_verifying",
                "Confirmando posição parada dentro de ±1°; tx=" +
                    txText(ball.txDegrees),
                progress);
            return output;
        }

        stableFrameCount_ = 0;
        lastStableBallTimestamp_ = 0.0;
        const double correctionDirection = ball.txDegrees > 0.0 ? 1.0 : -1.0;
        if (absoluteTx <=
            config::kBallAlignmentFineCorrectionThresholdDegrees)
        {
            // Próximo do centro, um comando contínuo de 0,61 ainda produz
            // inércia excessiva. O pulso curto permite medir entre correções.
            phase_ = Phase::FineCorrectionPulse;
            phaseStartedAt_ = now;
            fineCorrectionDirection_ = correctionDirection;
            output.leftPower =
                fineCorrectionDirection_ * config::kBallAlignmentStartPower;
            output.rightPower = -output.leftPower;
            output.status = makeStatus(
                "ball_alignment_fine_correction",
                "Correção fina por pulso; tx=" + txText(ball.txDegrees),
                progress);
            return output;
        }

        phase_ = Phase::Tracking;
        lastTurnDirection_ = 0.0;
        startCommandIssued_ = false;
        motionConfirmed_ = false;
    }

    if (absoluteTx <=
        config::kBallAlignmentFineCorrectionThresholdDegrees)
    {
        // A frenagem começa antes de ±1° para que a leitura de conclusão seja
        // feita depois da inércia, e não enquanto o robô ainda cruza o centro.
        phase_ = Phase::SettlingForVerification;
        phaseStartedAt_ = now;
        phaseStartBallTimestamp_ = ball.timestamp;
        stableFrameCount_ = 0;
        lastStableBallTimestamp_ = 0.0;
        lastTurnDirection_ = 0.0;
        startCommandIssued_ = false;
        motionConfirmed_ = false;
        output.status = makeStatus(
            "ball_alignment_braking",
            "Próximo do centro: PWM zerado para verificar parado; tx=" +
                txText(ball.txDegrees),
            progress);
        return output;
    }

    // tx positivo indica bola à direita: lado esquerdo avança e o direito
    // recua. tx negativo aplica exatamente o pivot oposto.
    const double turnDirection = ball.txDegrees > 0.0 ? 1.0 : -1.0;
    if (lastTurnDirection_ != 0.0 && turnDirection != lastTurnDirection_)
    {
        // Uma troca de sinal fora da faixa fina ainda exige parar e medir antes
        // de permitir outro sentido de giro.
        phase_ = Phase::SettlingForVerification;
        phaseStartBallTimestamp_ = ball.timestamp;
        phaseStartedAt_ = now;
        stableFrameCount_ = 0;
        lastStableBallTimestamp_ = 0.0;
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
    expectedTargetSequence_ = 0;
    lastTurnDirection_ = 0.0;
    phaseStartBallTimestamp_ = 0.0;
    lastStableBallTimestamp_ = 0.0;
    fineCorrectionDirection_ = 0.0;
    stableFrameCount_ = 0;
    failurePhase_.clear();
    failureAction_.clear();
    targetAcquired_ = false;
    targetLossActive_ = false;
    startCommandIssued_ = false;
    motionConfirmed_ = false;
    phaseStartedAt_ = {};
    targetLostAt_ = {};
}
