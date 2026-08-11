#include "obr/main_mission.h"

void MainMission::reset()
{
    // Os futuros comportamentos devem limpar aqui seus estados internos.
}

void MainMission::update(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry)
{
    (void)esp32Telemetry;

    // Nenhum comportamento autônomo está instalado. O comando zero impede que
    // a Missão Principal reutilize movimentos de uma implementação removida.
    robotState.driveAutonomous(0.0, 0.0);
    AutonomousStatus status;
    status.phase = "main_waiting_behaviors";
    status.action = "Missão Principal pronta; nenhum comportamento instalado";
    robotState.updateAutonomousStatus(status);
}
