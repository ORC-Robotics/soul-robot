#pragma once

#include "obr/camera_monitor.h"
#include "obr/robot_state.h"

struct RescueZoneSearchOutput
{
    double leftPower = 0.0;
    double rightPower = 0.0;
    bool completed = false;
    AutonomousStatus status;
};

// Procura somente a cor selecionada com um pivot contínuo e interrompe o
// movimento imediatamente quando a confirmação temporal da visão detecta a zona.
class RescueZoneSearchMission
{
public:
    RescueZoneSearchOutput update(
        const RescueZoneSnapshot& zones,
        RescueZoneTargetColor targetColor);
    void reset();

private:
    bool completed_ = false;
};
