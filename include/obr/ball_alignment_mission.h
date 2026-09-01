#pragma once

#include "obr/camera_monitor.h"
#include "obr/robot_state.h"

#include <chrono>

// Comando seguro calculado pela missão isolada de alinhamento visual.
struct BallAlignmentOutput
{
    double leftPower = 0.0;
    double rightPower = 0.0;
    AutonomousStatus status;
};

// Gira em pulsos curtos até o tx da bola mais próxima entrar na zona central.
// Esta classe não participa da Missão Principal e não acessa motores diretamente.
class BallAlignmentMission
{
public:
    BallAlignmentOutput update(
        const ForwardBallSnapshot& ball,
        std::chrono::steady_clock::time_point now =
            std::chrono::steady_clock::now());
    void reset();

private:
    enum class Phase
    {
        Idle,
        TurningPulse,
        Settling,
        Aligned
    };

    Phase phase_ = Phase::Idle;
    double pulseDirection_ = 0.0;
    std::chrono::steady_clock::time_point phaseStartedAt_{};
};
