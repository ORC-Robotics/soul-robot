#include "obr/robot_state.h"

#include "obr/config.h"

#include <algorithm>
#include <cmath>

namespace
{
double clampMotorCommand(double command)
{
    if (!std::isfinite(command))
    {
        return 0.0;
    }

    return std::clamp(command, config::kMinMotorOutput, config::kMaxMotorOutput);
}
}

const char* autonomousMissionName(AutonomousMission mission)
{
    switch (mission)
    {
    case AutonomousMission::TurnRight90:
        return "turn_right_90";
    case AutonomousMission::MainMission:
    default:
        return "main_mission";
    }
}

RobotSnapshot RobotState::snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

void RobotState::start()
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_.emergencyStop = false;
    state_.mode = "manual";
    lastCommand_ = std::chrono::steady_clock::now();
}

void RobotState::startAutonomous()
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_.emergencyStop = false;
    state_.mode = "autonomous";
    state_.left = 0.0;
    state_.right = 0.0;
    lastCommand_ = std::chrono::steady_clock::now();
}

void RobotState::setAutonomousMission(AutonomousMission mission)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // Trocar a missão sempre para o robô. Isso impede que uma nova estratégia
    // assuma os motores no meio de um movimento iniciado pela missão anterior.
    state_.mode = state_.emergencyStop ? "emergency" : "stopped";
    state_.left = 0.0;
    state_.right = 0.0;
    state_.autonomousMission = mission;
    lastCommand_ = std::chrono::steady_clock::now();
}

void RobotState::stop()
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_.mode = "stopped";
    state_.left = 0.0;
    state_.right = 0.0;
    lastCommand_ = std::chrono::steady_clock::now();
}

void RobotState::emergencyStop()
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_.mode = "emergency";
    state_.emergencyStop = true;
    state_.left = 0.0;
    state_.right = 0.0;
    lastCommand_ = std::chrono::steady_clock::now();
}

void RobotState::drive(double left, double right)
{
    std::lock_guard<std::mutex> lock(mutex_);
    lastCommand_ = std::chrono::steady_clock::now();

    if (state_.emergencyStop || state_.mode != "manual")
    {
        // Comandos de movimento só são aceitos depois do Start.
        // Isso impede que o dashboard tire o robô do modo parado por acidente.
        state_.left = 0.0;
        state_.right = 0.0;
        return;
    }

    state_.left = clampMotorCommand(left);
    state_.right = clampMotorCommand(right);
}

void RobotState::driveAutonomous(double left, double right)
{
    std::lock_guard<std::mutex> lock(mutex_);
    lastCommand_ = std::chrono::steady_clock::now();

    if (state_.emergencyStop || state_.mode != "autonomous")
    {
        // O controlador autônomo só pode mover o robô quando o modo autônomo
        // foi ativado explicitamente pelo dashboard.
        return;
    }

    state_.left = clampMotorCommand(left);
    state_.right = clampMotorCommand(right);
}

void RobotState::enforceCommandTimeout(std::chrono::milliseconds timeout)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto age = std::chrono::steady_clock::now() - lastCommand_;

    if ((state_.mode == "manual" || state_.mode == "autonomous") && age > timeout)
    {
        state_.left = 0.0;
        state_.right = 0.0;
    }
}
