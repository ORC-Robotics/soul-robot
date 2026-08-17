#include "obr/main_mission.h"

#include "obr/config.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

namespace
{
// Valores experimentais usados somente durante a recuperação pela FAR.
constexpr double kFarBaseSpeed = 0.65;
constexpr double kFarDeadzone = 0.10;
constexpr double kFarProportionalGain = 0.30;
constexpr double kFarMaximumCorrection = 0.15;
constexpr double kFarExtremeError = 0.75;

// Um erro deste tamanho registra uma direção útil sem apagar a memória no centro.
constexpr double kSignificantDirectionError = 0.20;
constexpr double kNearReacquireMaxAbsError = 0.40;

// A histerese evita alternar entre tracking e pivô perto do mesmo limiar.
// Somente sequências novas da câmera avançam as confirmações de entrada e saída.
constexpr double kAheadStrongTurnEnterError = 0.25;
constexpr double kStrongSteeringEnterError = 0.40;
constexpr double kStrongSteeringExitError = 0.12;
constexpr double kStrongSteeringDirectionMinimum = 0.05;
constexpr int kAheadStrongTurnEnterSamples = 2;
constexpr int kAheadStrongTurnExitSamples = 3;
// Potência exclusiva da contrarrotação antecipada. As recuperações
// permanecem limitadas pela base de 0,65 definida acima.
constexpr double kAheadStrongTurnPower = 0.65;

// Os limites impedem que o robô procure indefinidamente por uma linha perdida.
constexpr auto kNearRecoveryTimeout = std::chrono::milliseconds(3000);
constexpr auto kTotalLossTimeout = std::chrono::milliseconds(1300);
constexpr int kNearSamplesToConfirmRecovery = 3;

struct MotorCommand
{
    double left = 0.0;
    double right = 0.0;
};

AutonomousStatus makeLineStatus(
    const std::string& phase,
    const std::string& action)
{
    AutonomousStatus status;
    status.phase = phase;
    status.action = action;
    return status;
}

AutonomousStatus makeGapStatus(
    const std::string& phase,
    const std::string& action,
    double leftDistanceCm,
    double rightDistanceCm)
{
    AutonomousStatus status;
    status.phase = phase;
    status.action = action;
    status.targetDistanceCm = config::kGapMaximumDistanceCm;
    status.leftDistanceCm = leftDistanceCm;
    status.rightDistanceCm = rightDistanceCm;
    status.averageDistanceCm = (leftDistanceCm + rightDistanceCm) / 2.0;
    status.progressPercent = std::clamp(
        std::max(leftDistanceCm, rightDistanceCm) /
            config::kGapMaximumDistanceCm * 100.0,
        0.0,
        100.0);
    return status;
}

MotorCommand calculateOneWheelPivotCommand(double error)
{
    return error < 0.0
               ? MotorCommand{0.0, kFarBaseSpeed}
               : MotorCommand{kFarBaseSpeed, 0.0};
}

MotorCommand calculateCounterRotationCommand(bool turnLeft, double power)
{
    return turnLeft
               ? MotorCommand{-power, power}
               : MotorCommand{power, -power};
}

MotorCommand calculateNearReacquisitionCommand(double correction)
{
    // A reaquisição preserva a base validada de 0,65 mesmo quando o perfil
    // inferior publica uma prévia mais rápida para o tracking normal.
    const double safeCorrection = std::clamp(
        correction, -kFarMaximumCorrection, kFarMaximumCorrection);
    if (safeCorrection > 0.0)
    {
        return {kFarBaseSpeed + safeCorrection, kFarBaseSpeed};
    }
    if (safeCorrection < 0.0)
    {
        return {kFarBaseSpeed, kFarBaseSpeed + std::abs(safeCorrection)};
    }
    return {kFarBaseSpeed, kFarBaseSpeed};
}

MotorCommand calculateFarRecoveryCommand(double farError)
{
    const double safeFarError = std::clamp(farError, -1.0, 1.0);
    const double errorMagnitude = std::abs(safeFarError);

    if (errorMagnitude >= kFarExtremeError)
    {
        return calculateOneWheelPivotCommand(safeFarError);
    }

    if (errorMagnitude <= kFarDeadzone)
    {
        return {kFarBaseSpeed, kFarBaseSpeed};
    }

    const double normalizedMagnitude =
        (errorMagnitude - kFarDeadzone) / (1.0 - kFarDeadzone);
    const double correction = std::min(
        kFarMaximumCorrection,
        kFarProportionalGain * normalizedMagnitude);

    if (safeFarError < 0.0)
    {
        return {kFarBaseSpeed, kFarBaseSpeed + correction};
    }
    return {kFarBaseSpeed + correction, kFarBaseSpeed};
}
}

void MainMission::reset()
{
    state_ = LineFollowState::TrackingNear;
    lastSignificantDirection_ = LineDirection::Unknown;
    searchDirection_ = LineDirection::Unknown;
    lastValidError_ = 0.0;
    lastProcessedLineSequence_ = 0;
    hasProcessedLineSequence_ = false;
    consecutiveNearValidSamples_ = 0;
    aheadStrongTurnActive_ = false;
    aheadStrongTurnDirection_ = LineDirection::Unknown;
    aheadStrongTurnEnterSamples_ = 0;
    aheadStrongTurnExitSamples_ = 0;
    aheadStrongTurnLastLineSequence_ = 0;
    aheadStrongTurnHasLineSequence_ = false;
    aheadStrongTurnLastHeadingError_ = 0.0;
    nearRecoveryActive_ = false;
    totalLossActive_ = false;
    nearLostAt_ = {};
    totalLossStartedAt_ = {};
    resetGapTracking();
}

void MainMission::update(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    bool cameraReady,
    const CameraLineSnapshot& cameraLineSnapshot)
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
        // Uma fonte obrigatória indisponível encerra a execução e zera os motores.
        robotState.stop();
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const bool newLineSample =
        !hasProcessedLineSequence_ ||
        cameraLineSnapshot.lineSequence != lastProcessedLineSequence_;
    if (newLineSample)
    {
        hasProcessedLineSequence_ = true;
        lastProcessedLineSequence_ = cameraLineSnapshot.lineSequence;

        if (cameraLineSnapshot.nearValid)
        {
            updateDirectionMemory(cameraLineSnapshot.nearError);
        }
        else if (cameraLineSnapshot.farValid)
        {
            updateDirectionMemory(cameraLineSnapshot.farError);
        }
    }

    const bool gapStateActive =
        state_ == LineFollowState::AligningForGap ||
        state_ == LineFollowState::CrossingGap ||
        state_ == LineFollowState::WaitingAfterGap;
    if (!gapStateActive && state_ == LineFollowState::TrackingNear &&
        newLineSample)
    {
        if (cameraLineSnapshot.gapCandidate &&
            cameraLineSnapshot.gapAlignmentValid)
        {
            ++consecutiveGapCandidateSamples_;
        }
        else
        {
            consecutiveGapCandidateSamples_ = 0;
        }
    }

    if (!gapStateActive && state_ == LineFollowState::TrackingNear &&
        consecutiveGapCandidateSamples_ >= config::kGapConfirmationSamples)
    {
        // O alinhamento do gap tem prioridade sobre curvas e recuperações. Ele
        // só começa enquanto a fita que chega ao robô ainda está visível.
        aheadStrongTurnActive_ = false;
        aheadStrongTurnDirection_ = LineDirection::Unknown;
        aheadStrongTurnEnterSamples_ = 0;
        aheadStrongTurnExitSamples_ = 0;
        nearRecoveryActive_ = false;
        totalLossActive_ = false;
        consecutiveGapAlignedSamples_ = 0;
        transitionTo(LineFollowState::AligningForGap);
    }

    if (state_ == LineFollowState::AligningForGap)
    {
        if (!cameraLineSnapshot.gapCandidate ||
            !cameraLineSnapshot.gapAlignmentValid)
        {
            // Sem a geometria atual não existe referência segura para terminar
            // o alinhamento. A fita normal volta a controlar o robô neste frame.
            consecutiveGapCandidateSamples_ = 0;
            consecutiveGapAlignedSamples_ = 0;
            transitionTo(LineFollowState::TrackingNear);
        }
        else
        {
            const bool aligned =
                std::abs(cameraLineSnapshot.gapAlignmentError) <=
                config::kGapAlignmentTolerance;
            if (newLineSample)
            {
                consecutiveGapAlignedSamples_ = aligned
                                                    ? consecutiveGapAlignedSamples_ + 1
                                                    : 0;
            }

            if (consecutiveGapAlignedSamples_ >=
                config::kGapConfirmationSamples)
            {
                const bool encoderReady =
                    esp32Telemetry.sensorFresh &&
                    esp32Telemetry.lastSensorAgeMs >= 0 &&
                    esp32Telemetry.lastSensorAgeMs <=
                        config::kGapEncoderFreshnessMs &&
                    std::isfinite(esp32Telemetry.leftEncoderRate) &&
                    std::isfinite(esp32Telemetry.rightEncoderRate);
                if (!encoderReady)
                {
                    robotState.stop();
                    robotState.updateAutonomousStatus(makeGapStatus(
                        "gap_encoder_lost",
                        "Gap interrompido: encoders sem dados recentes",
                        0.0,
                        0.0));
                    return;
                }

                gapStartLeftCount_ = esp32Telemetry.leftEncoderCount;
                gapStartRightCount_ = esp32Telemetry.rightEncoderCount;
                gapLastProgressCounts_ = 0.0;
                gapStartedAt_ = now;
                gapLastProgressAt_ = now;
                gapNearLossObserved_ = false;
                consecutiveGapNearReturnSamples_ = 0;
                consecutiveGapFarReturnSamples_ = 0;
                transitionTo(LineFollowState::CrossingGap);
                robotState.driveAutonomous(
                    config::kGapDriveCommandPower,
                    config::kGapDriveCommandPower);
                robotState.updateAutonomousStatus(makeGapStatus(
                    "crossing_gap",
                    "Atravessando gap até 100 mm",
                    0.0,
                    0.0));
                return;
            }

            if (aligned)
            {
                robotState.driveAutonomous(0.0, 0.0);
                robotState.updateAutonomousStatus(makeLineStatus(
                    "aligning_for_gap",
                    "Confirmando alinhamento para o gap"));
                return;
            }

            const MotorCommand command = calculateCounterRotationCommand(
                cameraLineSnapshot.gapAlignmentError < 0.0,
                kFarBaseSpeed);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeLineStatus(
                "aligning_for_gap",
                "Alinhando sobre a fita antes do gap"));
            return;
        }
    }

    if (state_ == LineFollowState::CrossingGap)
    {
        const bool encoderReady =
            esp32Telemetry.sensorFresh &&
            esp32Telemetry.lastSensorAgeMs >= 0 &&
            esp32Telemetry.lastSensorAgeMs <=
                config::kGapEncoderFreshnessMs &&
            std::isfinite(esp32Telemetry.leftEncoderRate) &&
            std::isfinite(esp32Telemetry.rightEncoderRate);
        const double leftCounts = std::abs(static_cast<double>(
            esp32Telemetry.leftEncoderCount - gapStartLeftCount_));
        const double rightCounts = std::abs(static_cast<double>(
            esp32Telemetry.rightEncoderCount - gapStartRightCount_));
        const double leftDistanceCm =
            leftCounts / config::kEncoderCountsPerCentimeter;
        const double rightDistanceCm =
            rightCounts / config::kEncoderCountsPerCentimeter;

        if (!encoderReady)
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeGapStatus(
                "gap_encoder_lost",
                "Gap interrompido: encoders sem dados recentes",
                leftDistanceCm,
                rightDistanceCm));
            return;
        }
        if (now - gapStartedAt_ >
            std::chrono::milliseconds(config::kGapTraversalTimeoutMs))
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeGapStatus(
                "gap_timeout",
                "Gap interrompido pelo tempo limite",
                leftDistanceCm,
                rightDistanceCm));
            return;
        }

        const double minimumCounts = std::min(leftCounts, rightCounts);
        if (minimumCounts >=
            gapLastProgressCounts_ + config::kGapMinimumProgressCounts)
        {
            gapLastProgressCounts_ = minimumCounts;
            gapLastProgressAt_ = now;
        }
        if (now - gapLastProgressAt_ >
            std::chrono::milliseconds(config::kGapStallTimeoutMs))
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeGapStatus(
                "gap_encoder_stall",
                "Gap interrompido: uma roda não avançou",
                leftDistanceCm,
                rightDistanceCm));
            return;
        }

        if (newLineSample)
        {
            if (!cameraLineSnapshot.nearValid)
            {
                gapNearLossObserved_ = true;
            }
            consecutiveGapNearReturnSamples_ =
                gapNearLossObserved_ && cameraLineSnapshot.nearValid
                    ? consecutiveGapNearReturnSamples_ + 1
                    : 0;
            consecutiveGapFarReturnSamples_ =
                cameraLineSnapshot.gapReturnValid ||
                        (!cameraLineSnapshot.nearValid &&
                         cameraLineSnapshot.farValid)
                    ? consecutiveGapFarReturnSamples_ + 1
                    : 0;
        }

        if (consecutiveGapNearReturnSamples_ >=
            config::kGapConfirmationSamples)
        {
            resetGapTracking();
            nearRecoveryActive_ = true;
            transitionTo(LineFollowState::ReacquiringNear);
            const MotorCommand command =
                std::abs(cameraLineSnapshot.nearError) >
                        kNearReacquireMaxAbsError
                    ? calculateOneWheelPivotCommand(
                          cameraLineSnapshot.nearError)
                    : calculateNearReacquisitionCommand(
                          cameraLineSnapshot.correction);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeLineStatus(
                "reacquiring_near",
                "Linha reencontrada após o gap"));
            return;
        }
        if (consecutiveGapFarReturnSamples_ >=
            config::kGapConfirmationSamples)
        {
            if (cameraLineSnapshot.nearValid &&
                cameraLineSnapshot.gapReturnValid)
            {
                const double returnError =
                    cameraLineSnapshot.gapReturnError;
                resetGapTracking();
                gapReturnRecoveryActive_ = true;
                nearRecoveryActive_ = true;
                nearLostAt_ = now;
                transitionTo(LineFollowState::RecoveringFar);
                const MotorCommand command =
                    calculateFarRecoveryCommand(returnError);
                robotState.driveAutonomous(command.left, command.right);
                robotState.updateAutonomousStatus(makeLineStatus(
                    "recovering_far",
                    "Continuação do gap confirmada pela visão"));
                return;
            }

            resetGapTracking();
            nearRecoveryActive_ = true;
            nearLostAt_ = now;
            transitionTo(LineFollowState::RecoveringFar);
            const MotorCommand command =
                calculateFarRecoveryCommand(cameraLineSnapshot.farError);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeLineStatus(
                "recovering_far",
                "Linha reencontrada pela FAR após o gap"));
            return;
        }

        const double predictionSeconds =
            config::kGapBrakePredictionSeconds +
            esp32Telemetry.lastSensorAgeMs / 1000.0;
        const double projectedLeftCounts =
            leftCounts +
            std::abs(esp32Telemetry.leftEncoderRate) * predictionSeconds;
        const double projectedRightCounts =
            rightCounts +
            std::abs(esp32Telemetry.rightEncoderRate) * predictionSeconds;
        const double targetCounts =
            config::kGapMaximumDistanceCm *
            config::kEncoderCountsPerCentimeter;
        if (std::max(projectedLeftCounts, projectedRightCounts) >= targetCounts)
        {
            transitionTo(LineFollowState::WaitingAfterGap);
            consecutiveGapNearReturnSamples_ = 0;
            consecutiveGapFarReturnSamples_ = 0;
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeGapStatus(
                "gap_waiting",
                "100 mm concluídos: aguardando a linha",
                leftDistanceCm,
                rightDistanceCm));
            return;
        }

        robotState.driveAutonomous(
            config::kGapDriveCommandPower,
            config::kGapDriveCommandPower);
        robotState.updateAutonomousStatus(makeGapStatus(
            "crossing_gap",
            "Atravessando gap até 100 mm",
            leftDistanceCm,
            rightDistanceCm));
        return;
    }

    if (state_ == LineFollowState::WaitingAfterGap)
    {
        robotState.driveAutonomous(0.0, 0.0);
        if (newLineSample)
        {
            consecutiveGapNearReturnSamples_ = cameraLineSnapshot.nearValid
                                                   ? consecutiveGapNearReturnSamples_ + 1
                                                   : 0;
            consecutiveGapFarReturnSamples_ =
                !cameraLineSnapshot.nearValid && cameraLineSnapshot.farValid
                    ? consecutiveGapFarReturnSamples_ + 1
                    : 0;
        }

        if (consecutiveGapNearReturnSamples_ >=
            config::kGapConfirmationSamples)
        {
            resetGapTracking();
            nearRecoveryActive_ = true;
            transitionTo(LineFollowState::ReacquiringNear);
            const MotorCommand command =
                std::abs(cameraLineSnapshot.nearError) >
                        kNearReacquireMaxAbsError
                    ? calculateOneWheelPivotCommand(
                          cameraLineSnapshot.nearError)
                    : calculateNearReacquisitionCommand(
                          cameraLineSnapshot.correction);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeLineStatus(
                "reacquiring_near",
                "Linha confirmada após a espera do gap"));
            return;
        }
        if (consecutiveGapFarReturnSamples_ >=
            config::kGapConfirmationSamples)
        {
            resetGapTracking();
            nearRecoveryActive_ = true;
            nearLostAt_ = now;
            transitionTo(LineFollowState::RecoveringFar);
            const MotorCommand command =
                calculateFarRecoveryCommand(cameraLineSnapshot.farError);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeLineStatus(
                "recovering_far",
                "Linha confirmada pela FAR após a espera do gap"));
            return;
        }

        const double leftDistanceCm = std::abs(static_cast<double>(
            esp32Telemetry.leftEncoderCount - gapStartLeftCount_)) /
            config::kEncoderCountsPerCentimeter;
        const double rightDistanceCm = std::abs(static_cast<double>(
            esp32Telemetry.rightEncoderCount - gapStartRightCount_)) /
            config::kEncoderCountsPerCentimeter;
        robotState.updateAutonomousStatus(makeGapStatus(
            "gap_waiting",
            "Aguardando a linha com motores parados",
            leftDistanceCm,
            rightDistanceCm));
        return;
    }

    if (gapReturnRecoveryActive_)
    {
        if (!cameraLineSnapshot.nearValid)
        {
            // A fita antiga saiu da NEAR. A recuperação normal pela FAR pode
            // assumir o controle sem confundir a origem e a continuação do gap.
            gapReturnRecoveryActive_ = false;
        }
        else if (now - nearLostAt_ >= kNearRecoveryTimeout)
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeLineStatus(
                "gap_return_timeout",
                "Continuação do gap perdida durante a recuperação"));
            std::cout << "MainMission stopped: gap return recovery timeout ("
                      << kNearRecoveryTimeout.count() << " ms)" << std::endl;
            return;
        }
        else if (cameraLineSnapshot.gapReturnValid)
        {
            const MotorCommand command = calculateFarRecoveryCommand(
                cameraLineSnapshot.gapReturnError);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeLineStatus(
                "recovering_far",
                "Guiando pela continuação desconectada do gap"));
            return;
        }
        else
        {
            // Se a continuação sumir antes de a fita antiga sair da NEAR, o
            // robô aguarda imóvel em vez de voltar a seguir o segmento antigo.
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeLineStatus(
                "recovering_far",
                "Aguardando novamente a continuação do gap"));
            return;
        }
    }

    // Este detector possui sequência própria para nunca contar o mesmo frame
    // novamente quando o loop C++ roda mais rápido que o processo de visão.
    const bool newAheadStrongTurnSample =
        !aheadStrongTurnHasLineSequence_ ||
        cameraLineSnapshot.lineSequence != aheadStrongTurnLastLineSequence_;
    if (newAheadStrongTurnSample)
    {
        aheadStrongTurnHasLineSequence_ = true;
        aheadStrongTurnLastLineSequence_ = cameraLineSnapshot.lineSequence;
    }

    const auto logAheadStrongTurnEvent =
        [&](const char* event,
            const char* exitReason,
            double headingError,
            const MotorCommand& command)
    {
        std::cout << "MainMission ahead strong turn " << event << ":"
                  << " lineSequence=" << cameraLineSnapshot.lineSequence
                  << " nearError=" << cameraLineSnapshot.nearError
                  << " farError=" << cameraLineSnapshot.farError
                  << " headingError=" << headingError
                  << " controlError=" << cameraLineSnapshot.controlError
                  << " direction="
                  << (aheadStrongTurnDirection_ == LineDirection::Left
                          ? "left"
                          : "right")
                  << " leftCommand=" << command.left
                  << " rightCommand=" << command.right
                  << " exitReason=" << exitReason
                  << std::endl;
    };

    const auto exitAheadStrongTurn =
        [&](const char* exitReason,
            double headingError,
            bool returnToTracking)
    {
        const MotorCommand command =
            calculateCounterRotationCommand(
                aheadStrongTurnDirection_ == LineDirection::Left,
                kAheadStrongTurnPower);
        logAheadStrongTurnEvent(
            "exited", exitReason, headingError, command);
        aheadStrongTurnActive_ = false;
        aheadStrongTurnDirection_ = LineDirection::Unknown;
        aheadStrongTurnEnterSamples_ = 0;
        aheadStrongTurnExitSamples_ = 0;
        if (returnToTracking)
        {
            transitionTo(LineFollowState::TrackingNear);
        }
    };

    const bool aheadStrongTurnBandsValid =
        cameraLineSnapshot.nearValid && cameraLineSnapshot.farValid;
    if (aheadStrongTurnBandsValid)
    {
        const double headingError =
            cameraLineSnapshot.farError - cameraLineSnapshot.nearError;
        const double steeringError = cameraLineSnapshot.controlError;
        aheadStrongTurnLastHeadingError_ = headingError;

        if (aheadStrongTurnActive_)
        {
            if (newAheadStrongTurnSample)
            {
                const bool steeringSignCrossed =
                    std::abs(steeringError) > kStrongSteeringExitError &&
                    ((aheadStrongTurnDirection_ == LineDirection::Left &&
                      steeringError > 0.0) ||
                     (aheadStrongTurnDirection_ == LineDirection::Right &&
                      steeringError < 0.0));
                if (steeringSignCrossed)
                {
                    exitAheadStrongTurn(
                        "steering_sign_crossed", headingError, true);
                }
                else if (std::abs(steeringError) <= kStrongSteeringExitError)
                {
                    ++aheadStrongTurnExitSamples_;
                }
                else
                {
                    aheadStrongTurnExitSamples_ = 0;
                }
            }

            if (aheadStrongTurnActive_ &&
                aheadStrongTurnExitSamples_ >= kAheadStrongTurnExitSamples)
            {
                exitAheadStrongTurn("control_aligned", headingError, true);
            }

            if (aheadStrongTurnActive_)
            {
                const MotorCommand command =
                    calculateCounterRotationCommand(
                        aheadStrongTurnDirection_ == LineDirection::Left,
                        kAheadStrongTurnPower);
                transitionTo(LineFollowState::TurningAhead);
                robotState.driveAutonomous(command.left, command.right);
                robotState.updateAutonomousStatus(makeLineStatus(
                    "turning_ahead", "Curva forte antecipada pela AHEAD"));
                return;
            }
        }
        else if (newAheadStrongTurnSample)
        {
            const bool strongHeadingDemand =
                std::abs(headingError) >= kAheadStrongTurnEnterError;
            const bool strongSteeringDemand =
                std::abs(steeringError) >= kStrongSteeringEnterError;
            const bool directionIsUsable =
                std::abs(steeringError) >= kStrongSteeringDirectionMinimum;
            const bool shouldEnterStrongTurn =
                directionIsUsable &&
                (strongHeadingDemand || strongSteeringDemand);
            if (shouldEnterStrongTurn)
            {
                const LineDirection sampleDirection =
                    steeringError < 0.0
                        ? LineDirection::Left
                        : LineDirection::Right;
                if (sampleDirection == aheadStrongTurnDirection_)
                {
                    ++aheadStrongTurnEnterSamples_;
                }
                else
                {
                    aheadStrongTurnDirection_ = sampleDirection;
                    aheadStrongTurnEnterSamples_ = 1;
                }
            }
            else
            {
                aheadStrongTurnEnterSamples_ = 0;
                aheadStrongTurnDirection_ = LineDirection::Unknown;
            }

            if (aheadStrongTurnEnterSamples_ >= kAheadStrongTurnEnterSamples)
            {
                aheadStrongTurnActive_ = true;
                aheadStrongTurnExitSamples_ = 0;
                const MotorCommand command =
                    calculateCounterRotationCommand(
                        aheadStrongTurnDirection_ == LineDirection::Left,
                        kAheadStrongTurnPower);
                logAheadStrongTurnEvent(
                    "entered", "none", headingError, command);

                transitionTo(LineFollowState::TurningAhead);
                robotState.driveAutonomous(command.left, command.right);
                robotState.updateAutonomousStatus(makeLineStatus(
                    "turning_ahead", "Curva forte antecipada pela AHEAD"));
                return;
            }
        }
    }
    else
    {
        if (aheadStrongTurnActive_)
        {
            const char* exitReason = cameraLineSnapshot.nearValid
                                         ? "far_lost"
                                         : "near_lost";
            exitAheadStrongTurn(
                exitReason,
                aheadStrongTurnLastHeadingError_,
                cameraLineSnapshot.nearValid);
        }
        else
        {
            aheadStrongTurnEnterSamples_ = 0;
            aheadStrongTurnExitSamples_ = 0;
            aheadStrongTurnDirection_ = LineDirection::Unknown;
        }
    }

    if (cameraLineSnapshot.nearValid)
    {
        const bool requiresNearReacquisition =
            nearRecoveryActive_ || state_ != LineFollowState::TrackingNear;
        nearRecoveryActive_ = false;
        nearLostAt_ = {};
        totalLossActive_ = false;
        totalLossStartedAt_ = {};

        if (!requiresNearReacquisition)
        {
            consecutiveNearValidSamples_ = 0;
            transitionTo(LineFollowState::TrackingNear);
            robotState.driveAutonomous(
                cameraLineSnapshot.leftPreview,
                cameraLineSnapshot.rightPreview);
            robotState.updateAutonomousStatus(makeLineStatus(
                "tracking_near", "Seguindo pela NEAR"));
            return;
        }

        transitionTo(LineFollowState::ReacquiringNear);
        if (std::abs(cameraLineSnapshot.nearError) >
            kNearReacquireMaxAbsError)
        {
            consecutiveNearValidSamples_ = 0;

            // Usa o mesmo pivô controlado de um lado empregado pela FAR extrema.
            // Erro negativo aponta para a esquerda; erro positivo, para a direita.
            const MotorCommand command =
                calculateOneWheelPivotCommand(cameraLineSnapshot.nearError);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeLineStatus(
                "reacquiring_near", "Readquirindo NEAR: alinhando ao centro"));
            return;
        }

        if (newLineSample &&
            consecutiveNearValidSamples_ < kNearSamplesToConfirmRecovery)
        {
            ++consecutiveNearValidSamples_;
        }

        const MotorCommand command =
            calculateNearReacquisitionCommand(cameraLineSnapshot.correction);
        robotState.driveAutonomous(command.left, command.right);
        robotState.updateAutonomousStatus(makeLineStatus(
            "reacquiring_near",
            "Readquirindo NEAR " +
                std::to_string(consecutiveNearValidSamples_) + "/3"));

        if (consecutiveNearValidSamples_ >= kNearSamplesToConfirmRecovery)
        {
            totalLossActive_ = false;
            consecutiveNearValidSamples_ = 0;
            searchDirection_ = LineDirection::Unknown;
            totalLossStartedAt_ = {};
            transitionTo(LineFollowState::TrackingNear);
        }
        return;
    }

    consecutiveNearValidSamples_ = 0;
    if (!nearRecoveryActive_)
    {
        // Este tempo mede somente a ausência contínua da NEAR. Qualquer nova
        // amostra NEAR válida encerra esta contagem antes da reaquisição.
        nearRecoveryActive_ = true;
        nearLostAt_ = now;
        searchDirection_ = LineDirection::Unknown;
    }
    if (now - nearLostAt_ >= kNearRecoveryTimeout)
    {
        // A parada é terminal: somente uma nova partida poderá mover o robô.
        robotState.stop();
        std::cout << "MainMission stopped: NEAR recovery timeout ("
                  << kNearRecoveryTimeout.count() << " ms)"
                  << " nearValid=" << std::boolalpha
                  << cameraLineSnapshot.nearValid
                  << " farValid=" << cameraLineSnapshot.farValid
                  << std::noboolalpha
                  << " nearError=" << cameraLineSnapshot.nearError
                  << " state=" << stateName(state_) << std::endl;
        return;
    }

    if (cameraLineSnapshot.farValid)
    {
        totalLossActive_ = false;
        totalLossStartedAt_ = {};
        transitionTo(LineFollowState::RecoveringFar);

        const MotorCommand command =
            calculateFarRecoveryCommand(cameraLineSnapshot.farError);
        robotState.driveAutonomous(command.left, command.right);
        robotState.updateAutonomousStatus(makeLineStatus(
            "recovering_far", "Recuperando pela FAR"));
        return;
    }

    if (!totalLossActive_)
    {
        totalLossActive_ = true;
        totalLossStartedAt_ = now;
    }
    if (now - totalLossStartedAt_ >= kTotalLossTimeout)
    {
        // Sem qualquer linha visível, o giro também possui limite independente.
        robotState.stop();
        std::cout << "MainMission stopped: total line loss timeout ("
                  << kTotalLossTimeout.count() << " ms)" << std::endl;
        return;
    }

    const LineDirection direction = chooseSearchDirection();
    if (direction == LineDirection::Right)
    {
        transitionTo(LineFollowState::SearchingRight);
        const MotorCommand command =
            calculateCounterRotationCommand(false, kFarBaseSpeed);
        robotState.driveAutonomous(command.left, command.right);
        robotState.updateAutonomousStatus(makeLineStatus(
            "searching_right", "Procurando linha à direita"));
        return;
    }

    transitionTo(LineFollowState::SearchingLeft);
    const MotorCommand command =
        calculateCounterRotationCommand(true, kFarBaseSpeed);
    robotState.driveAutonomous(command.left, command.right);
    robotState.updateAutonomousStatus(makeLineStatus(
        "searching_left", "Procurando linha à esquerda"));
}

void MainMission::transitionTo(LineFollowState nextState)
{
    if (state_ == nextState)
    {
        return;
    }

    state_ = nextState;
    std::cout << "MainMission state: " << stateName(state_);
    if (state_ == LineFollowState::ReacquiringNear ||
        state_ == LineFollowState::TrackingNear)
    {
        std::cout << " nearError=" << lastValidError_;
    }
    std::cout << std::endl;
}

void MainMission::resetGapTracking()
{
    consecutiveGapCandidateSamples_ = 0;
    consecutiveGapAlignedSamples_ = 0;
    consecutiveGapNearReturnSamples_ = 0;
    consecutiveGapFarReturnSamples_ = 0;
    gapNearLossObserved_ = false;
    gapReturnRecoveryActive_ = false;
    gapStartLeftCount_ = 0;
    gapStartRightCount_ = 0;
    gapLastProgressCounts_ = 0.0;
    gapStartedAt_ = {};
    gapLastProgressAt_ = {};
}

void MainMission::updateDirectionMemory(double error)
{
    lastValidError_ = error;
    if (std::abs(error) < kSignificantDirectionError)
    {
        return;
    }

    lastSignificantDirection_ =
        error < 0.0 ? LineDirection::Left : LineDirection::Right;
}

MainMission::LineDirection MainMission::chooseSearchDirection()
{
    if (searchDirection_ != LineDirection::Unknown)
    {
        return searchDirection_;
    }

    if (lastSignificantDirection_ != LineDirection::Unknown)
    {
        searchDirection_ = lastSignificantDirection_;
    }
    else if (lastValidError_ > 0.0)
    {
        searchDirection_ = LineDirection::Right;
    }
    else
    {
        // Erro negativo procura à esquerda; zero também escolhe esquerda
        // deterministicamente para impedir alternância durante a tentativa.
        searchDirection_ = LineDirection::Left;
    }
    return searchDirection_;
}

const char* MainMission::stateName(LineFollowState state)
{
    switch (state)
    {
    case LineFollowState::TrackingNear:
        return "TrackingNear";
    case LineFollowState::TurningAhead:
        return "TurningAhead";
    case LineFollowState::AligningForGap:
        return "AligningForGap";
    case LineFollowState::CrossingGap:
        return "CrossingGap";
    case LineFollowState::WaitingAfterGap:
        return "WaitingAfterGap";
    case LineFollowState::ReacquiringNear:
        return "ReacquiringNear";
    case LineFollowState::RecoveringFar:
        return "RecoveringFar";
    case LineFollowState::SearchingLeft:
        return "SearchingLeft";
    case LineFollowState::SearchingRight:
        return "SearchingRight";
    }
    return "Unknown";
}
