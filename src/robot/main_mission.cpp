#include "obr/main_mission.h"

#include "obr/config.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

namespace
{
constexpr double kGreenReacquisitionBaseSpeed =
    config::kMotorStartMinimumPower;
constexpr double kGreenReacquisitionMaximumCorrection = 0.15;
constexpr double kGreenReacquisitionDeadzone = 0.10;
constexpr double kGreenReacquisitionProportionalGain = 0.30;
constexpr double kGreenReacquisitionExtremeError = 0.75;
constexpr double kGreenReacquisitionPivotThreshold = 0.40;
constexpr int kGreenReacquisitionConfirmationSamples = 3;

struct MotorCommand
{
    double left = 0.0;
    double right = 0.0;
};

AutonomousStatus makeMainMissionStatus(
    const std::string& phase,
    const std::string& action)
{
    AutonomousStatus status;
    status.phase = phase;
    status.action = action;
    return status;
}

MotorCommand calculateGreenOneWheelPivotCommand(double error)
{
    return error < 0.0
               ? MotorCommand{0.0, kGreenReacquisitionBaseSpeed}
               : MotorCommand{kGreenReacquisitionBaseSpeed, 0.0};
}

MotorCommand calculateGreenReacquisitionCommand(double correction)
{
    const double safeCorrection = std::clamp(
        correction,
        -kGreenReacquisitionMaximumCorrection,
        kGreenReacquisitionMaximumCorrection);
    if (safeCorrection > 0.0)
    {
        return {
            kGreenReacquisitionBaseSpeed + safeCorrection,
            kGreenReacquisitionBaseSpeed};
    }
    if (safeCorrection < 0.0)
    {
        return {
            kGreenReacquisitionBaseSpeed,
            kGreenReacquisitionBaseSpeed + std::abs(safeCorrection)};
    }
    return {
        kGreenReacquisitionBaseSpeed,
        kGreenReacquisitionBaseSpeed};
}

MotorCommand calculateGreenFallbackCommand(double lineError)
{
    const double safeLineError = std::clamp(lineError, -1.0, 1.0);
    const double errorMagnitude = std::abs(safeLineError);
    if (errorMagnitude >= kGreenReacquisitionExtremeError)
    {
        return calculateGreenOneWheelPivotCommand(safeLineError);
    }
    if (errorMagnitude <= kGreenReacquisitionDeadzone)
    {
        return {
            kGreenReacquisitionBaseSpeed,
            kGreenReacquisitionBaseSpeed};
    }

    const double normalizedMagnitude =
        (errorMagnitude - kGreenReacquisitionDeadzone) /
        (1.0 - kGreenReacquisitionDeadzone);
    const double correction = std::min(
        kGreenReacquisitionMaximumCorrection,
        kGreenReacquisitionProportionalGain * normalizedMagnitude);
    return safeLineError < 0.0
               ? MotorCommand{
                     kGreenReacquisitionBaseSpeed,
                     kGreenReacquisitionBaseSpeed + correction}
               : MotorCommand{
                     kGreenReacquisitionBaseSpeed + correction,
                     kGreenReacquisitionBaseSpeed};
}

bool isDirectionalGreenTurn(GreenTurnDecision decision)
{
    return decision == GreenTurnDecision::GuideLeft ||
           decision == GreenTurnDecision::GuideRight;
}

bool isAcceptedGreenInstruction(const CameraLineSnapshot& snapshot)
{
    return snapshot.greenConfirmed && snapshot.greenPathBlackValid &&
           (isDirectionalGreenTurn(snapshot.greenTurnDecision) ||
            snapshot.greenTurnDecision ==
                GreenTurnDecision::TurnAround180);
}

bool greenTurnProbeEncodersAvailable(
    const Esp32TelemetrySnapshot& telemetry)
{
    return telemetry.sensorFresh && telemetry.lastSensorAgeMs >= 0 &&
           telemetry.lastSensorAgeMs <=
               config::kGreenTurnForwardProbeEncoderFreshnessMs &&
           std::isfinite(telemetry.leftEncoderRate) &&
           std::isfinite(telemetry.rightEncoderRate);
}

double greenTurnProbeDistanceMillimeters(
    const Esp32TelemetrySnapshot& telemetry,
    long long startLeftEncoderCount,
    long long startRightEncoderCount)
{
    const double leftCounts = std::abs(static_cast<double>(
        telemetry.leftEncoderCount - startLeftEncoderCount));
    const double rightCounts = std::abs(static_cast<double>(
        telemetry.rightEncoderCount - startRightEncoderCount));
    // A maior roda limita a sonda para que nenhum lado ultrapasse 20 mm.
    return std::max(leftCounts, rightCounts) /
           config::kEncoderCountsPerCentimeter * 10.0;
}

bool hasGreenTurnVisualHandoffLine(const CameraLineSnapshot& snapshot)
{
    return snapshot.greenManeuverNearValid &&
           snapshot.greenManeuverTrajectoryValid &&
           std::isfinite(snapshot.greenManeuverLeftPower) &&
           std::isfinite(snapshot.greenManeuverRightPower) &&
           snapshot.greenManeuverLeftPower > 0.0 &&
           snapshot.greenManeuverRightPower > 0.0 &&
           !snapshot.greenManeuverGapCandidate;
}

bool hasGreenTurnNearFarRecoveryLine(const CameraLineSnapshot& snapshot)
{
    return snapshot.greenManeuverNearValid &&
           snapshot.greenManeuverFarValid &&
           std::isfinite(snapshot.greenManeuverNearError) &&
           std::isfinite(snapshot.greenManeuverFarError) &&
           std::isfinite(snapshot.greenManeuverCorrection) &&
           !snapshot.greenManeuverGapCandidate;
}
}

void MainMission::reset()
{
    state_ = State::NormalLineFollowing;
    greenTurnController_.reset();
    resetGreenDirectionalTurnTracking();
    greenDecisionLatched_ = false;
    greenTurnAuthorized_ = false;
    greenTurnDirection_ = GreenTurnDecision::None;
    greenTurnConfirmSamples_ = 0;
    lastProcessedLineSequence_ = 0;
    hasProcessedLineSequence_ = false;
    consecutiveGreenReacquisitionSamples_ = 0;
    greenTurnIgnoreUntil_ = {};
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
        std::string phase;
        std::string action;
        if (!esp32Telemetry.readyForOperation())
        {
            phase = "esp32_not_ready";
            action = "Missão interrompida: ESP32 sem telemetria pronta";
        }
        else if (!cameraReady)
        {
            phase = "camera_not_ready";
            action = "Missão interrompida: câmera inferior indisponível";
        }
        else
        {
            phase = "line_ipc_stale";
            action = "Missão interrompida: IPC visual ausente ou antigo";
        }
        robotState.stop();
        robotState.updateAutonomousStatus(
            makeMainMissionStatus(phase, action));
        std::cout << "MainMission stopped: " << phase << std::endl;
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
    }

    // O retorno de 180 graus tem prioridade e retorna antes do comando normal.
    if (updateGreenTurnAround(
            robotState,
            esp32Telemetry,
            cameraLineSnapshot,
            newLineSample))
    {
        return;
    }

    const bool greenApproachActive =
        cameraLineSnapshot.greenPathBlackValid &&
        cameraLineSnapshot.greenTurnDecision == GreenTurnDecision::Approach;

    if (state_ == State::GreenTurnWaitingImu)
    {
        if (newLineSample && !greenDecisionLatched_)
        {
            const bool sameCandidate =
                isAcceptedGreenInstruction(cameraLineSnapshot) &&
                cameraLineSnapshot.greenTurnDecision == greenTurnDirection_;
            if (!sameCandidate)
            {
                greenTurnDirection_ = GreenTurnDecision::None;
                greenTurnConfirmSamples_ = 0;
                greenTurnAuthorized_ = false;
                transitionTo(State::NormalLineFollowing);
                robotState.driveAutonomous(
                    cameraLineSnapshot.lineFollowerLeftPower,
                    cameraLineSnapshot.lineFollowerRightPower);
                robotState.updateAutonomousStatus(makeMainMissionStatus(
                    "green_cancelled",
                    "Marcador verde cancelado: decisão não se manteve"));
                return;
            }
            ++greenTurnConfirmSamples_;
        }

        if (greenTurnConfirmSamples_ >=
            config::kGreenTurnConfirmationFrames)
        {
            greenDecisionLatched_ = true;
            greenTurnAuthorized_ = true;
            if (greenTurnDirection_ == GreenTurnDecision::TurnAround180)
            {
                if (updateGreenTurnAround(
                        robotState,
                        esp32Telemetry,
                        cameraLineSnapshot,
                        newLineSample))
                {
                    return;
                }
            }
            else
            {
                const bool turnLeft =
                    greenTurnDirection_ == GreenTurnDecision::GuideLeft;
                greenTurnAuthorized_ = false;
                resetGreenDirectionalTurnTracking();
                if (!greenDirectionalTurnController_.start(
                        config::kGreenDirectionalTurnTargetDegrees,
                        turnLeft ? ImuTurnDirection::Left
                                 : ImuTurnDirection::Right,
                        esp32Telemetry,
                        config::kGreenDirectionalTurnCompletionToleranceDegrees))
                {
                    robotState.driveAutonomous(0.0, 0.0);
                    robotState.updateAutonomousStatus(makeMainMissionStatus(
                        "green_turn_waiting_imu",
                        "Aguardando IMU para giro verde de 45°"));
                    return;
                }
                transitionTo(
                    turnLeft ? State::GreenTurnLeft
                             : State::GreenTurnRight);
                robotState.updateAutonomousStatus(makeMainMissionStatus(
                    turnLeft ? "green_turn_45_left"
                             : "green_turn_45_right",
                    "Marcador verde confirmado: iniciando giro IMU de 45°"));
                return;
            }
        }

        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            "green_turn_waiting_imu",
            "Marcador verde: amostras " +
                std::to_string(greenTurnConfirmSamples_) + "/" +
                std::to_string(config::kGreenTurnConfirmationFrames)));
        return;
    }

    const bool greenTurnCooldownActive = now < greenTurnIgnoreUntil_;
    if (newLineSample &&
        state_ == State::NormalLineFollowing &&
        !greenDecisionLatched_ &&
        !greenTurnCooldownActive &&
        isAcceptedGreenInstruction(cameraLineSnapshot))
    {
        greenTurnDirection_ = cameraLineSnapshot.greenTurnDecision;
        greenTurnConfirmSamples_ = config::kGreenTurnConfirmationFrames;
        greenDecisionLatched_ = true;
        greenTurnAuthorized_ = false;
        transitionTo(State::GreenTurnWaitingImu);
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            greenTurnDirection_ == GreenTurnDecision::GuideLeft
                ? "green_turn_45_left"
                : greenTurnDirection_ == GreenTurnDecision::GuideRight
                      ? "green_turn_45_right"
                      : "green_turning",
            "Marcador verde: amostras 1/" +
                std::to_string(config::kGreenTurnConfirmationFrames)));
        return;
    }

    if (state_ == State::GreenTurnLeft ||
        state_ == State::GreenTurnRight)
    {
        const bool turnLeft = state_ == State::GreenTurnLeft;
        const ImuTurnOutput output =
            greenDirectionalTurnController_.update(esp32Telemetry);
        if (output.result == ImuTurnResult::Failed)
        {
            resetGreenDirectionalTurnTracking();
            robotState.stop();
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "green_turn_imu_failed",
                "Giro verde de 45° interrompido: " + output.action));
            return;
        }
        if (output.result == ImuTurnResult::Completed)
        {
            greenTurnVisualHandoffStartedAt_ = now;
            greenTurnIgnoreUntil_ = now + std::chrono::milliseconds(
                config::kGreenTurnVisualHandoffCooldownMs);
            transitionTo(State::GreenTurnVisualHandoff);
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "green_turn_visual_handoff",
                "Giro verde de 45° concluído: priorizando trajetória visual"));
            return;
        }

        robotState.driveAutonomous(output.leftPower, output.rightPower);
        AutonomousStatus status;
        status.phase =
            turnLeft ? "green_turn_45_left" : "green_turn_45_right";
        status.action = output.action;
        status.progressPercent = output.progressPercent;
        robotState.updateAutonomousStatus(status);
        return;
    }

    if (state_ == State::GreenTurnVisualHandoff)
    {
        if (newLineSample &&
            hasGreenTurnVisualHandoffLine(cameraLineSnapshot))
        {
            resetGreenDirectionalTurnTracking();
            transitionTo(State::NormalLineFollowing);
            robotState.driveAutonomous(
                cameraLineSnapshot.greenManeuverLeftPower,
                cameraLineSnapshot.greenManeuverRightPower);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "green_turn_handoff_completed",
                "Linha pós-verde forte: manobra verde concluída"));
            return;
        }

        if (newLineSample &&
            hasGreenTurnNearFarRecoveryLine(cameraLineSnapshot))
        {
            consecutiveGreenReacquisitionSamples_ = 0;
            resetGreenDirectionalTurnTracking();
            transitionTo(State::GreenTurnReacquiringLine);
            const MotorCommand command =
                std::abs(cameraLineSnapshot.greenManeuverNearError) >
                        kGreenReacquisitionPivotThreshold
                    ? calculateGreenOneWheelPivotCommand(
                          cameraLineSnapshot.greenManeuverNearError)
                    : calculateGreenReacquisitionCommand(
                          cameraLineSnapshot.greenManeuverCorrection);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "green_turn_reacquiring_line",
                "Linha pós-verde reencontrada: readquirindo"));
            return;
        }

        if (cameraLineSnapshot.greenManeuverGapCandidate)
        {
            resetGreenDirectionalTurnTracking();
            robotState.stop();
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "green_turn_line_not_found",
                "Linha pós-verde ambígua: gap detectado"));
            return;
        }

        if (now - greenTurnVisualHandoffStartedAt_ >=
            std::chrono::milliseconds(config::kGreenTurnAcquireTimeoutMs))
        {
            if (!greenTurnProbeEncodersAvailable(esp32Telemetry))
            {
                resetGreenDirectionalTurnTracking();
                robotState.stop();
                robotState.updateAutonomousStatus(makeMainMissionStatus(
                    "green_turn_encoder_unavailable",
                    "Sonda pós-verde bloqueada: encoders indisponíveis"));
                return;
            }

            greenTurnProbeStartLeftEncoderCount_ =
                esp32Telemetry.leftEncoderCount;
            greenTurnProbeStartRightEncoderCount_ =
                esp32Telemetry.rightEncoderCount;
            greenTurnProbeLastProgressCounts_ = 0.0;
            greenTurnProbeLastProgressAt_ = now;
            transitionTo(State::GreenTurnForwardProbe);
            robotState.driveAutonomous(
                config::kGreenTurnForwardProbePower,
                config::kGreenTurnForwardProbePower);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "green_turn_forward_probe",
                "Sonda pós-verde: 0/" +
                    std::to_string(static_cast<int>(
                        config::kGreenTurnForwardProbeDistanceMm)) +
                    " mm sem rota visual"));
            return;
        }

        const auto elapsedMilliseconds = std::chrono::duration_cast<
            std::chrono::milliseconds>(
                now - greenTurnVisualHandoffStartedAt_).count();
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            "green_turn_visual_handoff",
            "Pós-verde: priorizando trajetória visual — cooldown " +
                std::to_string(std::min<long long>(
                    elapsedMilliseconds,
                    config::kGreenTurnVisualHandoffCooldownMs)) + "/" +
                std::to_string(config::kGreenTurnVisualHandoffCooldownMs) +
                " ms"));
        return;
    }

    if (state_ == State::GreenTurnForwardProbe)
    {
        if (cameraLineSnapshot.greenManeuverGapCandidate)
        {
            resetGreenDirectionalTurnTracking();
            robotState.stop();
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "green_turn_line_not_found",
                "Sonda pós-verde interrompida: gap detectado"));
            return;
        }

        if (newLineSample &&
            hasGreenTurnVisualHandoffLine(cameraLineSnapshot))
        {
            resetGreenDirectionalTurnTracking();
            transitionTo(State::NormalLineFollowing);
            robotState.driveAutonomous(
                cameraLineSnapshot.greenManeuverLeftPower,
                cameraLineSnapshot.greenManeuverRightPower);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "green_turn_handoff_completed",
                "Sonda pós-verde encontrou trajetória visual forte"));
            return;
        }

        if (newLineSample &&
            hasGreenTurnNearFarRecoveryLine(cameraLineSnapshot))
        {
            consecutiveGreenReacquisitionSamples_ = 0;
            resetGreenDirectionalTurnTracking();
            transitionTo(State::GreenTurnReacquiringLine);
            const MotorCommand command =
                std::abs(cameraLineSnapshot.greenManeuverNearError) >
                        kGreenReacquisitionPivotThreshold
                    ? calculateGreenOneWheelPivotCommand(
                          cameraLineSnapshot.greenManeuverNearError)
                    : calculateGreenReacquisitionCommand(
                          cameraLineSnapshot.greenManeuverCorrection);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "green_turn_reacquiring_line",
                "Sonda pós-verde cancelada: rota visual reencontrada"));
            return;
        }

        if (!greenTurnProbeEncodersAvailable(esp32Telemetry))
        {
            resetGreenDirectionalTurnTracking();
            robotState.stop();
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "green_turn_encoder_unavailable",
                "Sonda pós-verde interrompida: encoders indisponíveis"));
            return;
        }

        const double distanceMm = greenTurnProbeDistanceMillimeters(
            esp32Telemetry,
            greenTurnProbeStartLeftEncoderCount_,
            greenTurnProbeStartRightEncoderCount_);
        const double progressCounts =
            distanceMm / 10.0 * config::kEncoderCountsPerCentimeter;
        if (progressCounts >= greenTurnProbeLastProgressCounts_ +
                                  config::kGreenTurnForwardProbeMinimumProgressCounts)
        {
            greenTurnProbeLastProgressCounts_ = progressCounts;
            greenTurnProbeLastProgressAt_ = now;
        }
        if (distanceMm >= config::kGreenTurnForwardProbeDistanceMm)
        {
            resetGreenDirectionalTurnTracking();
            robotState.stop();
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "green_turn_line_not_found",
                "Linha pós-verde não encontrada após sonda de " +
                    std::to_string(static_cast<int>(
                        config::kGreenTurnForwardProbeDistanceMm)) +
                    " mm"));
            return;
        }
        if (now - greenTurnProbeLastProgressAt_ >=
            std::chrono::milliseconds(
                config::kGreenTurnForwardProbeEncoderStallTimeoutMs))
        {
            resetGreenDirectionalTurnTracking();
            robotState.stop();
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "green_turn_encoder_stall",
                "Sonda pós-verde interrompida: encoders sem avanço"));
            return;
        }

        robotState.driveAutonomous(
            config::kGreenTurnForwardProbePower,
            config::kGreenTurnForwardProbePower);
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            "green_turn_forward_probe",
            "Sonda pós-verde: " +
                std::to_string(static_cast<int>(std::round(distanceMm))) +
                "/" +
                std::to_string(static_cast<int>(
                    config::kGreenTurnForwardProbeDistanceMm)) +
                " mm sem rota visual"));
        return;
    }

    if (state_ == State::GreenTurnReacquiringLine)
    {
        if (cameraLineSnapshot.greenManeuverNearValid)
        {
            if (newLineSample &&
                consecutiveGreenReacquisitionSamples_ <
                    kGreenReacquisitionConfirmationSamples)
            {
                ++consecutiveGreenReacquisitionSamples_;
            }
            const MotorCommand command =
                std::abs(cameraLineSnapshot.greenManeuverNearError) >
                        kGreenReacquisitionPivotThreshold
                    ? calculateGreenOneWheelPivotCommand(
                          cameraLineSnapshot.greenManeuverNearError)
                    : calculateGreenReacquisitionCommand(
                          cameraLineSnapshot.greenManeuverCorrection);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "green_turn_reacquiring_line",
                "Readquirindo linha pós-verde " +
                    std::to_string(
                        consecutiveGreenReacquisitionSamples_) + "/3"));
            if (consecutiveGreenReacquisitionSamples_ >=
                kGreenReacquisitionConfirmationSamples)
            {
                consecutiveGreenReacquisitionSamples_ = 0;
                transitionTo(State::NormalLineFollowing);
            }
            return;
        }

        consecutiveGreenReacquisitionSamples_ = 0;
        if (cameraLineSnapshot.greenManeuverFarValid)
        {
            const MotorCommand command = calculateGreenFallbackCommand(
                cameraLineSnapshot.greenManeuverFarError);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "green_turn_reacquiring_line",
                "Pós-verde: alinhando pela referência visual distante"));
            return;
        }

        robotState.stop();
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            "green_turn_line_not_found",
            "Missão interrompida: linha pós-verde não encontrada"));
        return;
    }

    if (cameraLineSnapshot.greenManeuverNearValid &&
        greenApproachActive)
    {
        robotState.driveAutonomous(
            cameraLineSnapshot.greenManeuverLeftPower,
            cameraLineSnapshot.greenManeuverRightPower);
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            "green_approach",
            "Aproximando do marcador verde com velocidade reduzida"));
        return;
    }

    // O comando normal vem do ponto de extensão da visão. Enquanto o TODO
    // permanecer vazio, os dois valores são zero. Esta chamada fica por último
    // para nunca sobrescrever um comando emitido por um estado verde acima.
    robotState.driveAutonomous(
        cameraLineSnapshot.lineFollowerLeftPower,
        cameraLineSnapshot.lineFollowerRightPower);
    robotState.updateAutonomousStatus(makeMainMissionStatus(
        "line_follower_pending",
        "Seguidor de linha ainda não implementado: motores parados"));
}

void MainMission::transitionTo(State nextState)
{
    if (state_ == nextState)
    {
        return;
    }
    state_ = nextState;
    std::cout << "MainMission state: " << stateName(state_) << std::endl;
}

bool MainMission::updateGreenTurnAround(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    const CameraLineSnapshot& cameraLineSnapshot,
    bool newLineSample)
{
    if (state_ == State::TurningAtGreenMarker)
    {
        const ImuTurnOutput output =
            greenTurnController_.update(esp32Telemetry);
        robotState.driveAutonomous(output.leftPower, output.rightPower);

        AutonomousStatus status;
        status.phase =
            output.phase == "turning" ? "green_turning" : output.phase;
        status.action = output.action;
        status.progressPercent = output.progressPercent;
        if (output.result == ImuTurnResult::Completed)
        {
            consecutiveGreenReacquisitionSamples_ = 0;
            transitionTo(State::GreenTurnReacquiringLine);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "green_turn_completed",
                "Retorno verde concluído: procurando a linha"));
            return true;
        }
        if (output.result == ImuTurnResult::Failed)
        {
            robotState.stop();
            robotState.updateAutonomousStatus(status);
            return true;
        }
        robotState.updateAutonomousStatus(status);
        return true;
    }

    if (newLineSample &&
        state_ != State::GreenTurnWaitingImu &&
        !cameraLineSnapshot.greenNearSeen &&
        cameraLineSnapshot.greenCandidateDecision ==
            GreenTurnDecision::None)
    {
        greenDecisionLatched_ = false;
        greenTurnAuthorized_ = false;
        greenTurnDirection_ = GreenTurnDecision::None;
        greenTurnConfirmSamples_ = 0;
    }

    const bool confirmedTurnAround =
        greenTurnAuthorized_ &&
        greenTurnDirection_ == GreenTurnDecision::TurnAround180;
    if (greenDecisionLatched_ && !confirmedTurnAround)
    {
        return false;
    }
    if (!confirmedTurnAround)
    {
        return false;
    }

    if (!ImuTurnController::imuReady(esp32Telemetry))
    {
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            "green_turn_waiting_imu",
            "Aguardando IMU para retorno de 180° à direita"));
        return true;
    }

    greenDecisionLatched_ = true;
    consecutiveGreenReacquisitionSamples_ = 0;
    if (!greenTurnController_.start(
            config::kGreenTurnAroundTargetDegrees,
            ImuTurnDirection::Right,
            esp32Telemetry))
    {
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            "green_turn_waiting_imu",
            "Aguardando IMU para retorno de 180° à direita"));
        return true;
    }

    transitionTo(State::TurningAtGreenMarker);
    greenTurnAuthorized_ = false;
    greenTurnConfirmSamples_ = 0;

    const ImuTurnOutput output =
        greenTurnController_.update(esp32Telemetry);
    robotState.driveAutonomous(output.leftPower, output.rightPower);
    AutonomousStatus status;
    status.phase = "green_turning";
    status.action = output.action;
    status.progressPercent = output.progressPercent;
    robotState.updateAutonomousStatus(status);
    return true;
}

void MainMission::resetGreenDirectionalTurnTracking()
{
    greenDirectionalTurnController_.reset();
    greenTurnVisualHandoffStartedAt_ = {};
    greenTurnProbeStartLeftEncoderCount_ = 0;
    greenTurnProbeStartRightEncoderCount_ = 0;
    greenTurnProbeLastProgressCounts_ = 0.0;
    greenTurnProbeLastProgressAt_ = {};
}

const char* MainMission::stateName(State state)
{
    switch (state)
    {
    case State::NormalLineFollowing:
        return "NormalLineFollowing";
    case State::GreenTurnWaitingImu:
        return "GreenTurnWaitingImu";
    case State::GreenTurnLeft:
        return "GreenTurnLeft";
    case State::GreenTurnRight:
        return "GreenTurnRight";
    case State::GreenTurnVisualHandoff:
        return "GreenTurnVisualHandoff";
    case State::GreenTurnForwardProbe:
        return "GreenTurnForwardProbe";
    case State::GreenTurnReacquiringLine:
        return "GreenTurnReacquiringLine";
    case State::TurningAtGreenMarker:
        return "TurningAtGreenMarker";
    }
    return "Unknown";
}
