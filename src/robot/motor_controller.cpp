#include "obr/motor_controller.h"

#include "obr/config.h"

#include <algorithm>
#include <cmath>
#include <iostream>

MotorController::MotorController(Esp32Bridge& esp32)
    : esp32_(esp32)
{
}

bool MotorController::begin()
{
    const bool ok = esp32_.begin();
    stop();

    if (!ok)
    {
        std::cerr << "ESP32 motor bridge unavailable. Motors will stay stopped until UART works.\n";
    }

    return ok;
}

void MotorController::apply(const RobotSnapshot& state)
{
    if (state.emergencyStop || state.mode == "emergency")
    {
        // A parada de emergência também é enviada para a ESP32.
        // Mesmo que o dashboard continue mandando comandos, a ESP32 recebe zero nos motores.
        esp32_.sendEmergencyStop();
        return;
    }

    if (state.mode == "stopped")
    {
        stop();
        return;
    }

    esp32_.sendMotorCommand(safeMotorPower(state.left), safeMotorPower(state.right), false);
}

void MotorController::stop()
{
    // O STOP mantém a ESP32 sem movimento e não libera a parada de emergência do RobotState.
    esp32_.sendStop();
}

double MotorController::safeMotorPower(double command)
{
    if (!std::isfinite(command))
    {
        return 0.0;
    }

    return std::clamp(command, config::kMinMotorOutput, config::kMaxMotorOutput);
}
