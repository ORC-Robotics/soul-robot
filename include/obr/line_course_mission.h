#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/forward_line_assist.h"
#include "obr/green_maneuver.h"
#include "obr/robot_state.h"
#include "obr/obstacle_avoidance.h"

#include <cstdint>
#include <string>

// Coordena o percurso de linha e as manobras que podem assumir seu controle.
// A classe não acessa hardware diretamente e mantém cada etapa fail-safe.
class LineCourseMission
{
public:
    void reset();
    bool maneuverActive() const
    {
        return obstacleAvoidance_.active() || greenManeuver_.active() ||
               forwardLineAssist_.active() || obstacleRecoveryWaiting_;
    }
    void update(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        bool cameraReady,
        const CameraLineSnapshot& cameraLineSnapshot,
        const ForwardLineSnapshot& forwardLineSnapshot,
        bool allowForwardLostRecovery = false);

private:
    ObstacleAvoidance obstacleAvoidance_;
    GreenManeuver greenManeuver_;
    ForwardLineAssist forwardLineAssist_;
    bool obstacleRecoveryWaiting_ = false;
    int obstacleRecoveryFusionFrames_ = 0;
    std::uint64_t obstacleRecoveryLastLineSequence_ = 0;
    std::string obstacleRecoveryCause_;
};
