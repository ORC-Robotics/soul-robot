#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/robot_state.h"

#include <chrono>
#include <cstdint>
#include <string>

// Recupera a linha pelo último lado visto na CAM1 somente quando o chamador
// libera essa autoridade; fora desse caso, transporta apenas diagnóstico.
class ForwardLineAssist
{
public:
    void reset();
    bool active() const { return recovering_; }
    bool update(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        const CameraLineSnapshot& cameraLineSnapshot,
        const ForwardLineSnapshot& forwardLineSnapshot,
        bool allowRecovery = false);
    AutonomousStatus status(
        const std::string& phase,
        const std::string& action,
        const ForwardLineSnapshot& forwardLineSnapshot) const;

private:
    CameraLineSnapshot bottom_;
    int recoveryDirectionSign_ = 0;
    int bottomLossFrames_ = 0;
    int bottomStableFrames_ = 0;
    std::uint64_t lastBottomSequence_ = 0;
    bool recovering_ = false;
    double recoveryStartYawDegrees_ = 0.0;
    std::chrono::steady_clock::time_point recoveryStartedAt_{};
};
