#pragma once

#include "obr/esp32_bridge.h"
#include "obr/robot_state.h"

// Orquestra os comportamentos da prova que formarão a Missão Principal.
// Enquanto não há comportamentos instalados, mantém o robô parado com segurança.
class MainMission
{
public:
    void reset();
    void update(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry);
};
