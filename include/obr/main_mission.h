#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/robot_state.h"

// Mantém a Missão Principal parada até o seguidor normal ser implementado.
// A percepção verde é recebida apenas para telemetria e nunca altera motores.
class MainMission
{
public:
    void reset();
    void update(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        bool cameraReady,
        const CameraLineSnapshot& cameraLineSnapshot);

};
