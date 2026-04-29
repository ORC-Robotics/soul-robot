#pragma once

#include "obr/gpio.h"
#include "obr/robot_state.h"

#include <atomic>
#include <thread>

// Converte comandos seguros do RobotState em sinais digitais para a ponte H L298N.
// Em parada de emergência ou modo parado, todos os pinos dos motores são zerados.
// ENA e ENB recebem PWM por software para controlar a potência dos motores.
class MotorController
{
public:
    MotorController();
    ~MotorController();

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
    std::atomic<bool> pwmRunning_{false};
    std::atomic<double> leftDutyCycle_{0.0};
    std::atomic<double> rightDutyCycle_{0.0};
    std::thread leftPwmThread_;
    std::thread rightPwmThread_;

    static double safeMotorPower(double command);
    void startPwm();
    void stopPwm();
    void pwmLoop(GpioPin& enable, std::atomic<double>& dutyCycle);
    void setMotor(GpioPin& enable, GpioPin& input1, GpioPin& input2, double command);
};
