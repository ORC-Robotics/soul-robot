#pragma once

#include "obr/esp32_bridge.h"
#include "obr/robot_state.h"

// Converte comandos seguros do RobotState em mensagens UART para a ESP32.
// A ESP32 controla os drivers DRV8833 e também aplica timeout próprio de segurança.
class MotorController
{
public:
    explicit MotorController(Esp32Bridge& esp32);

    bool begin();
    void apply(const RobotSnapshot& state);
    void stop();

private:
    Esp32Bridge& esp32_;

    static double safeMotorPower(double command);
    static double operationalMotorPower(double command, double calibrationGain);
};
