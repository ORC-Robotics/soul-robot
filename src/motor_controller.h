#pragma once

#include "gpio.h"
#include "robot_state.h"

class MotorController
{
public:
    MotorController();

    bool begin();
    void apply(const RobotSnapshot& state);
    void stop();

private:
    GpioPin leftEnable_;
    GpioPin leftInput1_;
    GpioPin leftInput2_;
    GpioPin rightEnable_;
    GpioPin rightInput1_;
    GpioPin rightInput2_;

    static int directionFromCommand(double command);
    void setMotor(GpioPin& enable, GpioPin& input1, GpioPin& input2, double command);
};
