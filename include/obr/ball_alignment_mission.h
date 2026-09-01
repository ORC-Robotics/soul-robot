#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/robot_state.h"

#include <chrono>

// Comando seguro calculado pela missão isolada de alinhamento visual.
struct BallAlignmentOutput
{
    double leftPower = 0.0;
    double rightPower = 0.0;
    AutonomousStatus status;
};

// Gira continuamente até o tx da bola mais próxima entrar na zona central.
// Esta classe não participa da Missão Principal e não acessa motores diretamente.
class BallAlignmentMission
{
public:
    BallAlignmentOutput update(
        const ForwardBallSnapshot& ball,
        const Esp32TelemetrySnapshot& telemetry,
        std::chrono::steady_clock::time_point now =
            std::chrono::steady_clock::now());
    void reset();

private:
    enum class Phase
    {
        Tracking,
        BrakingAfterCrossing
    };

    Phase phase_ = Phase::Tracking;
    double lastTurnDirection_ = 0.0;
    double crossingTimestamp_ = 0.0;
    bool startCommandIssued_ = false;
    bool motionConfirmed_ = false;
    std::chrono::steady_clock::time_point brakingStartedAt_{};
};
