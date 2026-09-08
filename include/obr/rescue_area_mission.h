#pragma once

#include "obr/ball_alignment_mission.h"
#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/robot_state.h"

#include <cstdint>

struct RescueAreaOutput
{
    double leftPower = 0.0;
    double rightPower = 0.0;
    bool completed = false;
    bool failed = false;
    AutonomousStatus status;
};

// Possui o ciclo de vida da área de resgate e reutiliza o alinhamento de vítima.
// Até a busca da saída ser implementada, a missão completa aguarda parada.
class RescueAreaMission
{
public:
    RescueAreaOutput update(
        const ForwardBallSnapshot& forwardBallSnapshot,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        std::uint64_t autonomousRunSequence,
        bool rescueExitConfirmed,
        bool finishAfterVictim);
    void reset();

private:
    BallAlignmentMission ballAlignmentMission_;
    bool victimReached_ = false;
    bool failed_ = false;
    AutonomousStatus failureStatus_;
};
