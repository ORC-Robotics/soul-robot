#include "obr/motor_controller.h"

#include "obr/config.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <thread>

MotorController::MotorController()
    : leftEnable_(config::kLeftEnablePin),
      leftInput1_(config::kLeftInput1Pin),
      leftInput2_(config::kLeftInput2Pin),
      rightEnable_(config::kRightEnablePin),
      rightInput1_(config::kRightInput1Pin),
      rightInput2_(config::kRightInput2Pin)
{
}

MotorController::~MotorController()
{
    stopPwm();
}

bool MotorController::begin()
{
    bool ok = true;
    ok = leftEnable_.beginOutput() && ok;
    ok = leftInput1_.beginOutput() && ok;
    ok = leftInput2_.beginOutput() && ok;
    ok = rightEnable_.beginOutput() && ok;
    ok = rightInput1_.beginOutput() && ok;
    ok = rightInput2_.beginOutput() && ok;

    stop();

    if (ok)
    {
        startPwm();
    }

    if (!ok)
    {
        std::cerr << "Motor GPIO setup failed. Check pins, permissions, and wiring.\n";
    }

    return ok;
}

void MotorController::apply(const RobotSnapshot& state)
{
    if (state.emergencyStop || state.mode == "stopped" || state.mode == "emergency")
    {
        stop();
        return;
    }

    setMotor(leftEnable_, leftInput1_, leftInput2_, state.left);
    setMotor(rightEnable_, rightInput1_, rightInput2_, state.right);
}

void MotorController::stop()
{
    leftDutyCycle_ = 0.0;
    rightDutyCycle_ = 0.0;
    leftEnable_.write(false);
    leftInput1_.write(false);
    leftInput2_.write(false);
    rightEnable_.write(false);
    rightInput1_.write(false);
    rightInput2_.write(false);
}

double MotorController::safeMotorPower(double command)
{
    if (!std::isfinite(command))
    {
        return 0.0;
    }

    return std::clamp(command, config::kMinMotorOutput, config::kMaxMotorOutput);
}

void MotorController::startPwm()
{
    if (pwmRunning_)
    {
        return;
    }

    pwmRunning_ = true;
    leftPwmThread_ = std::thread(&MotorController::pwmLoop, this, std::ref(leftEnable_), std::ref(leftDutyCycle_));
    rightPwmThread_ = std::thread(&MotorController::pwmLoop, this, std::ref(rightEnable_), std::ref(rightDutyCycle_));
}

void MotorController::stopPwm()
{
    if (!pwmRunning_)
    {
        stop();
        return;
    }

    leftDutyCycle_ = 0.0;
    rightDutyCycle_ = 0.0;
    pwmRunning_ = false;

    if (leftPwmThread_.joinable())
    {
        leftPwmThread_.join();
    }

    if (rightPwmThread_.joinable())
    {
        rightPwmThread_.join();
    }

    leftEnable_.write(false);
    rightEnable_.write(false);
}

void MotorController::pwmLoop(GpioPin& enable, std::atomic<double>& dutyCycle)
{
    const auto period = std::chrono::milliseconds(config::kMotorPwmPeriodMs);

    while (pwmRunning_)
    {
        // O PWM por software controla apenas ENA/ENB. Os pinos IN1..IN4
        // continuam definindo a direção e são zerados nas paradas de segurança.
        const double duty = std::clamp(dutyCycle.load(), 0.0, 1.0);

        if (duty < config::kMotorDeadband)
        {
            enable.write(false);
            std::this_thread::sleep_for(period);
            continue;
        }

        if (duty >= config::kMaxMotorOutput)
        {
            enable.write(true);
            std::this_thread::sleep_for(period);
            continue;
        }

        const auto onTime = std::chrono::microseconds(
            static_cast<int>(config::kMotorPwmPeriodMs * 1000.0 * duty));
        const auto offTime = std::chrono::microseconds(config::kMotorPwmPeriodMs * 1000) - onTime;

        enable.write(true);
        std::this_thread::sleep_for(onTime);
        enable.write(false);
        std::this_thread::sleep_for(offTime);
    }

    enable.write(false);
}

void MotorController::setMotor(GpioPin& enable, GpioPin& input1, GpioPin& input2, double command)
{
    std::atomic<double>& dutyCycle = (&enable == &leftEnable_) ? leftDutyCycle_ : rightDutyCycle_;
    const double safeCommand = safeMotorPower(command);

    if (std::abs(safeCommand) < config::kMotorDeadband)
    {
        dutyCycle = 0.0;
        input1.write(false);
        input2.write(false);
        enable.write(false);
        return;
    }

    if (safeCommand > 0.0)
    {
        input1.write(true);
        input2.write(false);
    }
    else
    {
        input1.write(false);
        input2.write(true);
    }

    dutyCycle = std::abs(safeCommand);
}
