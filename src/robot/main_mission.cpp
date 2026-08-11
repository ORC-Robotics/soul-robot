#include "obr/main_mission.h"

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

MotorCommand calculateFarRecoveryCommand(double farError)
{
    const double safeFarError = std::clamp(farError, -1.0, 1.0);
    const double errorMagnitude = std::abs(safeFarError);

    if (errorMagnitude >= kFarExtremeError)
    {
        return safeFarError < 0.0
                   ? MotorCommand{0.0, kFarBaseSpeed}
                   : MotorCommand{kFarBaseSpeed, 0.0};
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
    nearRecoveryActive_ = false;
    totalLossActive_ = false;
    nearLostAt_ = {};
    totalLossStartedAt_ = {};
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
            const MotorCommand command = cameraLineSnapshot.nearError < 0.0
                                             ? MotorCommand{0.0, kFarBaseSpeed}
                                             : MotorCommand{kFarBaseSpeed, 0.0};
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

        robotState.driveAutonomous(
            cameraLineSnapshot.leftPreview,
            cameraLineSnapshot.rightPreview);
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
        robotState.driveAutonomous(kFarBaseSpeed, -kFarBaseSpeed);
        robotState.updateAutonomousStatus(makeLineStatus(
            "searching_right", "Procurando linha à direita"));
        return;
    }

    transitionTo(LineFollowState::SearchingLeft);
    robotState.driveAutonomous(-kFarBaseSpeed, kFarBaseSpeed);
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
