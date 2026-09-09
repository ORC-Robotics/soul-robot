#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/robot_state.h"

#include <string>

// Transporta o diagnóstico frontal, sem autoridade de motor. GAP e LOST são
// executados pelo controle inferior e seu recovery existente.
class ForwardLineAssist
{
public:
    void reset();
    bool update(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        const CameraLineSnapshot& cameraLineSnapshot,
        const ForwardLineSnapshot& forwardLineSnapshot);
    AutonomousStatus status(
        const std::string& phase,
        const std::string& action,
        const ForwardLineSnapshot& forwardLineSnapshot) const;

private:
    CameraLineSnapshot bottom_;
};
