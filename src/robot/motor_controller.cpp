#include "obr/motor_controller.h"

#include "obr/config.h"

#include <algorithm>
#include <cmath>
#include <iostream>

MotorController::MotorController()
    : leftEnable_(config::kLeftEnablePin),
      leftInput1_(config::kLeftInput1Pin),
      leftInput2_(config::kLeftInput2Pin),
      rightEnable_(config::kRightEnablePin),
      rightInput1_(config::kRightInput1Pin),
      rightInput2_(config::kRightInput2Pin)
{
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
    leftEnable_.write(false);
    leftInput1_.write(false);
    leftInput2_.write(false);
    rightEnable_.write(false);
    rightInput1_.write(false);
    rightInput2_.write(false);
}

int MotorController::directionFromCommand(double command)
{
    if (!std::isfinite(command))
    {
        return 0;
    }

    const double safeCommand = std::clamp(command, config::kMinMotorOutput, config::kMaxMotorOutput);

    if (std::abs(safeCommand) < config::kMotorDeadband)
    {
        return 0;
    }

    return safeCommand > 0.0 ? 1 : -1;
}

void MotorController::setMotor(GpioPin& enable, GpioPin& input1, GpioPin& input2, double command)
{
    const int direction = directionFromCommand(command);

    if (direction == 0)
    {
        enable.write(false);
        input1.write(false);
        input2.write(false);
    }
    else if (direction > 0)
    {
        input1.write(true);
        input2.write(false);
        enable.write(true);
    }
    else
    {
        input1.write(false);
        input2.write(true);
        enable.write(true);
    }
}
