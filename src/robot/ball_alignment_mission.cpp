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
        (1.0 - std::abs(txDegrees) /
                   config::kBallAlignmentMaximumVisualErrorDegrees) * 100.0,
        0.0,
        100.0);
}

bool approachHeadingAvailable(const Esp32TelemetrySnapshot& telemetry)
{
    return telemetry.sensorFresh && telemetry.mpuOk &&
           telemetry.lastSensorAgeMs >= 0 &&
           telemetry.lastSensorAgeMs <= config::kTurn90ImuFreshnessMs &&
           std::isfinite(telemetry.yawZDeg);
}

double signedHeadingErrorDegrees(double targetDegrees, double currentDegrees)
{
    return std::remainder(targetDegrees - currentDegrees, 360.0);
}

double correctedApproachHeadingDegrees(
    double currentYawDegrees,
    double txDegrees)
{
    const double adjustmentDegrees = std::clamp(
        txDegrees * config::kBallApproachVisualHeadingGain,
        -config::kBallApproachMaximumHeadingAdjustmentDegrees,
        config::kBallApproachMaximumHeadingAdjustmentDegrees);
    return std::remainder(currentYawDegrees + adjustmentDegrees, 360.0);
}

double approachHeadingCorrection(double headingErrorDegrees)
{
    const double absoluteError = std::abs(headingErrorDegrees);
    if (absoluteError <= config::kBallApproachHeadingDeadbandDegrees)
    {
        return 0.0;
    }

    const double normalizedError = std::clamp(
        (absoluteError - config::kBallApproachHeadingDeadbandDegrees) /
            (config::kBallApproachFullSteeringErrorDegrees -
             config::kBallApproachHeadingDeadbandDegrees),
        0.0,
        1.0);
    return std::copysign(
        normalizedError * config::kBallApproachMaximumSteeringCorrection,
        headingErrorDegrees);
}

std::string powerText(double power)
{
    std::ostringstream text;
    text << std::fixed << std::setprecision(2) << power;
    return text.str();
}

int fineCorrectionPulseMs(double absoluteTxDegrees)
{
    const double normalizedError = std::clamp(
        (absoluteTxDegrees - config::kBallAlignmentDeadbandDegrees) /
            (config::kBallAlignmentFineCorrectionThresholdDegrees -
             config::kBallAlignmentDeadbandDegrees),
        0.0,
        1.0);
    const double durationRange =
        config::kBallAlignmentFineMaximumPulseMs -
        config::kBallAlignmentFineMinimumPulseMs;
    return static_cast<int>(std::lround(
        config::kBallAlignmentFineMinimumPulseMs +
        normalizedError * durationRange));
}

int coarseCorrectionPulseMs(double absoluteTxDegrees)
{
    const double normalizedError = std::clamp(
        (absoluteTxDegrees - config::kBallAlignmentFineCorrectionThresholdDegrees) /
            (config::kBallAlignmentMaximumVisualErrorDegrees -
             config::kBallAlignmentFineCorrectionThresholdDegrees),
        0.0,
        1.0);
    const double durationRange =
        config::kBallAlignmentCoarseMaximumPulseMs -
        config::kBallAlignmentCoarseMinimumPulseMs;
    return static_cast<int>(std::lround(
        config::kBallAlignmentCoarseMinimumPulseMs +
        normalizedError * durationRange));
}

double correctionYawLimitDegrees(double absoluteTxDegrees, bool fine)
{
    if (fine)
    {
        return std::clamp(
            absoluteTxDegrees * config::kBallAlignmentFineYawFraction,
            config::kBallAlignmentFineMinimumYawDegrees,
            config::kBallAlignmentFineMaximumYawDegrees);
    }
    return std::clamp(
        absoluteTxDegrees * config::kBallAlignmentCoarseYawFraction,
        config::kBallAlignmentCoarseMinimumYawDegrees,
        config::kBallAlignmentCoarseMaximumYawDegrees);
}

double angularDistanceDegrees(double first, double second)
{
    return std::abs(std::remainder(second - first, 360.0));
}

bool esp32ConfirmsCorrection(
    const Esp32TelemetrySnapshot& telemetry,
    double direction)
{
    // O tempo útil do pulso começa somente depois que a telemetria confirma
    // que os dois lados receberam o sentido e a potência solicitados.
    return telemetry.sensorFresh &&
           std::isfinite(telemetry.appliedLeftPower) &&
           std::isfinite(telemetry.appliedRightPower) &&
           telemetry.appliedLeftPower * direction >=
               config::kBallAlignmentAppliedPowerMinimum &&
           telemetry.appliedRightPower * -direction >=
               config::kBallAlignmentAppliedPowerMinimum;
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
            if (phase_ == Phase::CorrectionPulse)
            {
                // Uma perda interrompe o pulso atual. Se a vítima reaparecer,
                // o robô deve medir novamente em repouso, sem herdar timeout,
                // sentido ou duração calculados com uma imagem antiga.
                phase_ = Phase::SettlingForVerification;
                phaseStartedAt_ = now;
                phaseStartBallTimestamp_ = ball.timestamp;
                correctionApplied_ = false;
                fineCorrectionActive_ = false;
                correctionYawAvailable_ = false;
            }
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

    const bool victimAlreadyAtCollectionDistance =
        std::isfinite(ball.distanceCm) && ball.distanceCm > 0.0 &&
        ball.distanceCm <= config::kBallApproachStopDistanceCm &&
        absoluteTx <=
            config::kBallCollectionNearAlignmentToleranceDegrees;
    if (victimAlreadyAtCollectionDistance)
    {
        // Perto da vítima, um novo pivô pode deslocá-la para fora do coletor.
        // A etapa seguinte usa os encoders para completar o contato em linha reta.
        phase_ = Phase::Completed;
        output.finished = true;
        output.status = makeStatus(
            "ball_reached",
            "Vítima próxima e dentro da margem de coleta; distância=" +
                powerText(ball.distanceCm) + " cm; tx=" +
                txText(ball.txDegrees),
            100.0);
        return output;
    }

    if (phase_ == Phase::Approaching)
    {
        if (!headingLocked_ || !approachHeadingAvailable(telemetry))
        {
            output.status = makeStatus(
                "ball_approach_waiting_imu",
                "Parado: aguardando yaw válido para manter o heading",
                progress);
            return output;
        }
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

        const bool newVisualMeasurement =
            ball.timestamp > lastApproachBallTimestamp_;
        if (newVisualMeasurement)
        {
            lastApproachBallTimestamp_ = ball.timestamp;
            if (absoluteTx <=
                config::kBallApproachVisualAlignedToleranceDegrees)
            {
                // Ao confirmar o alinhamento visual, o yaw atual vira a nova
                // referência e encerra a correção anterior sem ultrapassar.
                lockedHeadingDegrees_ = telemetry.yawZDeg;
            }
            else
            {
                // O YOLO atualiza apenas o objetivo lento. A IMU mantém a
                // correção responsiva entre dois frames de inferência.
                lockedHeadingDegrees_ = correctedApproachHeadingDegrees(
                    telemetry.yawZDeg,
                    ball.txDegrees);
            }
        }

        const double headingErrorDegrees = signedHeadingErrorDegrees(
            lockedHeadingDegrees_, telemetry.yawZDeg);
        const double steeringCorrection =
            approachHeadingCorrection(headingErrorDegrees);
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
            std::string(absoluteTx <=
                                config::kBallApproachVisualAlignedToleranceDegrees
                            ? "Avançando alinhado; tx="
                            : "Avançando e corrigindo; tx=") +
                txText(ball.txDegrees) + "; erro de heading=" +
                txText(headingErrorDegrees) + "; distância=" +
                powerText(ball.distanceCm) + " cm",
            progress);
        return output;
    }

    if (phase_ == Phase::CorrectionPulse)
    {
        const bool yawLimitReached =
            correctionYawAvailable_ && approachHeadingAvailable(telemetry) &&
            angularDistanceDegrees(
                correctionStartYawDegrees_, telemetry.yawZDeg) >=
                correctionMaximumYawDegrees_;
        if (!correctionApplied_ &&
            esp32ConfirmsCorrection(telemetry, correctionDirection_))
        {
            correctionApplied_ = true;
            correctionAppliedAt_ = now;
        }

        if (!correctionApplied_ &&
            now - phaseStartedAt_ >= std::chrono::milliseconds(
                                         config::kBallAlignmentPulseStartTimeoutMs))
        {
            phase_ = Phase::Failed;
            output.finished = true;
            failurePhase_ = "ball_alignment_motion_timeout";
            failureAction_ =
                "Parado: a ESP32 não confirmou o micro-pivô de alinhamento";
            output.status = makeStatus(failurePhase_, failureAction_, progress);
            return output;
        }

        if (!yawLimitReached &&
            (!correctionApplied_ ||
             now - correctionAppliedAt_ < std::chrono::milliseconds(
                                              correctionPulseDurationMs_)))
        {
            const double pulsePower =
                fineCorrectionActive_
                    ? config::kBallAlignmentFinePulsePower
                    : config::kBallAlignmentCoarsePulsePower;
            output.leftPower = correctionDirection_ * pulsePower;
            output.rightPower = -output.leftPower;
            output.status = makeStatus(
                fineCorrectionActive_
                    ? "ball_alignment_fine_correction"
                    : "ball_alignment_correction_pulse",
                std::string(fineCorrectionActive_
                                ? "Correção fina"
                                : "Micro-pivô") +
                    (correctionApplied_ ? " em movimento para "
                                        : " aguardando partida para ") +
                    std::string(correctionDirection_ > 0.0
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

        if (absoluteTx <= config::kBallApproachStartToleranceDegrees)
        {
            if (ball.timestamp > lastStableBallTimestamp_)
            {
                lastStableBallTimestamp_ = ball.timestamp;
                ++stableFrameCount_;
            }
            if (stableFrameCount_ >= config::kBallAlignmentStableFrames)
            {
                if (!approachHeadingAvailable(telemetry))
                {
                    output.status = makeStatus(
                        "ball_approach_waiting_imu",
                        "Alinhada; aguardando yaw válido para travar o heading",
                        100.0);
                    return output;
                }
                lockedHeadingDegrees_ = telemetry.yawZDeg;
                headingLocked_ = true;
                lastApproachBallTimestamp_ = 0.0;
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
                output.leftPower = config::kBallApproachBasePower;
                output.rightPower = config::kBallApproachBasePower;
                output.status = makeStatus(
                    "ball_approaching",
                    "Alinhada; avançando em direção à bola; distância=" +
                        powerText(ball.distanceCm) + " cm",
                    100.0);
                return output;
            }
            output.status = makeStatus(
                "ball_alignment_verifying",
                "Confirmando posição para aproximar dentro de ±" +
                    powerText(config::kBallApproachStartToleranceDegrees) +
                    "°; tx=" + txText(ball.txDegrees),
                progress);
            return output;
        }

        stableFrameCount_ = 0;
        lastStableBallTimestamp_ = 0.0;
        const double correctionDirection = ball.txDegrees > 0.0 ? 1.0 : -1.0;
        if (absoluteTx <=
            config::kBallAlignmentFineCorrectionThresholdDegrees)
        {
            // Próximo do centro, um comando contínuo ainda produz inércia
            // excessiva. O pulso curto permite medir entre correções.
            phase_ = Phase::CorrectionPulse;
            phaseStartedAt_ = now;
            correctionDirection_ = correctionDirection;
            correctionPulseDurationMs_ =
                fineCorrectionPulseMs(absoluteTx);
            correctionApplied_ = false;
            fineCorrectionActive_ = true;
            correctionYawAvailable_ = approachHeadingAvailable(telemetry);
            correctionStartYawDegrees_ = telemetry.yawZDeg;
            correctionMaximumYawDegrees_ =
                correctionYawLimitDegrees(absoluteTx, true);
            output.leftPower =
                correctionDirection_ * config::kBallAlignmentFinePulsePower;
            output.rightPower = -output.leftPower;
            output.status = makeStatus(
                "ball_alignment_fine_correction",
                "Correção fina por pulso; tx=" + txText(ball.txDegrees),
                progress);
            return output;
        }

        phase_ = Phase::Tracking;
    }

    if (absoluteTx <=
        config::kBallAlignmentFineCorrectionThresholdDegrees)
    {
        // A faixa fina começa com o PWM zerado para que a próxima decisão use
        // uma leitura capturada depois que a inércia do giro terminar.
        phase_ = Phase::SettlingForVerification;
        phaseStartedAt_ = now;
        phaseStartBallTimestamp_ = ball.timestamp;
        stableFrameCount_ = 0;
        lastStableBallTimestamp_ = 0.0;
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
    // Cada medição libera somente um micro-pivô. Isso impede que o comando
    // antigo continue ativo durante os mais de 200 ms da próxima inferência.
    phase_ = Phase::CorrectionPulse;
    phaseStartedAt_ = now;
    correctionDirection_ = turnDirection;
    correctionPulseDurationMs_ = coarseCorrectionPulseMs(absoluteTx);
    correctionApplied_ = false;
    fineCorrectionActive_ = false;
    correctionYawAvailable_ = approachHeadingAvailable(telemetry);
    correctionStartYawDegrees_ = telemetry.yawZDeg;
    correctionMaximumYawDegrees_ =
        correctionYawLimitDegrees(absoluteTx, false);
    output.leftPower =
        turnDirection * config::kBallAlignmentCoarsePulsePower;
    output.rightPower = -output.leftPower;
    output.status = makeStatus(
        "ball_alignment_correction_pulse",
        std::string("Micro-pivô para a ") +
            (turnDirection > 0.0 ? "direita" : "esquerda") +
            "; tx=" + txText(ball.txDegrees) +
            "; potência=" +
                powerText(config::kBallAlignmentCoarsePulsePower),
        progress);
    return output;
}

void BallAlignmentMission::reset()
{
    phase_ = Phase::Tracking;
    expectedTargetSequence_ = 0;
    phaseStartBallTimestamp_ = 0.0;
    lastStableBallTimestamp_ = 0.0;
    lastApproachBallTimestamp_ = 0.0;
    correctionDirection_ = 0.0;
    correctionStartYawDegrees_ = 0.0;
    correctionMaximumYawDegrees_ = 0.0;
    lockedHeadingDegrees_ = 0.0;
    correctionPulseDurationMs_ = 0;
    stableFrameCount_ = 0;
    failurePhase_.clear();
    failureAction_.clear();
    targetAcquired_ = false;
    targetLossActive_ = false;
    headingLocked_ = false;
    correctionApplied_ = false;
    fineCorrectionActive_ = false;
    correctionYawAvailable_ = false;
    phaseStartedAt_ = {};
    correctionAppliedAt_ = {};
    targetLostAt_ = {};
}
