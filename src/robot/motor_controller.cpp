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

    double leftPower = safeMotorPower(state.left);
    double rightPower = safeMotorPower(state.right);

    if (!state.rawMotorCommand)
    {
        // O perfil operacional garante partida confiável a partir de 0,65.
        // O lado direito recebe o ganho medido pelos encoders para acompanhar
        // o lado esquerdo sem obrigar o seguidor de linha a corrigir uma reta.
        leftPower = operationalMotorPower(leftPower, 1.0);
        rightPower = operationalMotorPower(rightPower, config::kRightMotorCalibrationGain);
    }

    esp32_.sendMotorCommand(leftPower, rightPower, false);
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

double MotorController::operationalMotorPower(double command, double calibrationGain)
{
    const double safeCommand = safeMotorPower(command);
    if (std::abs(safeCommand) < 0.000001)
    {
        // Zero permanece zero para que parada, timeout e E-Stop nunca acionem
        // o piso operacional de potência.
        return 0.0;
    }

    const double referenceMagnitude = std::clamp(
        std::abs(safeCommand),
        config::kOperationalMinimumMotorPower,
        config::kOperationalMaximumReferencePower);
    const double calibratedMagnitude = std::clamp(
        referenceMagnitude * calibrationGain,
        0.0,
        config::kMaxMotorOutput);
    return std::copysign(calibratedMagnitude, safeCommand);
}
