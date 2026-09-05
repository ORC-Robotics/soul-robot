#pragma once

#include "obr/config.h"

enum class ServoId
{
    Arm,
    Wrist,
    Gripper
};

struct ServoPose
{
    double armDegrees = config::kServoInitialAngleDegrees;
    double wristDegrees = config::kServoInitialAngleDegrees;
    double gripperDegrees = config::kServoInitialAngleDegrees;
};
