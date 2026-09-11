#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/robot_state.h"

#include <chrono>

struct SilverEntryOutput
{
    bool hasControl = false;
    bool completed = false;
    double leftPower = 0.0;
    double rightPower = 0.0;
    AutonomousStatus status;
};

// Controla o pequeno avanço reto que confirma fisicamente a entrada cinza.
// A troca de fase continua dependendo da confirmação temporal da câmera.
class SilverEntryManeuver
{
public:
    void reset();
    SilverEntryOutput update(
        const CameraLineSnapshot& vision,
        const Esp32TelemetrySnapshot& telemetry);

private:
    bool active_ = false;
    bool visionConfirmed_ = false;
    long long startLeftCount_ = 0;
    long long startRightCount_ = 0;
    std::chrono::steady_clock::time_point startTime_{};
};
