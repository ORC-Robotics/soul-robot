#pragma once

#include "obr/gpio.h"
#include "obr/robot_state.h"

// Converte comandos seguros do RobotState em sinais digitais para a ponte H L298N.
// Em parada de emergência ou modo parado, todos os pinos dos motores são zerados.
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
