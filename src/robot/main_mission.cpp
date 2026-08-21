#include "obr/main_mission.h"

#include <iostream>
#include <string>

namespace
{
AutonomousStatus makeMainMissionStatus(
    const std::string& phase,
    const std::string& action)
{
    AutonomousStatus status;
    status.phase = phase;
    status.action = action;
    return status;
}
}

void MainMission::reset()
{
    // Não há estado interno enquanto o seguidor normal estiver pendente.
}

void MainMission::update(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    bool cameraReady,
    const CameraLineSnapshot& cameraLineSnapshot)
{
    const RobotSnapshot robotSnapshot = robotState.snapshot();
    if (robotSnapshot.mode != "autonomous" ||
        robotSnapshot.autonomousMission != AutonomousMission::MainMission)
    {
        return;
    }

    if (!esp32Telemetry.readyForOperation() || !cameraReady ||
        !cameraLineSnapshot.sourceFresh)
    {
        std::string phase;
        std::string action;
        if (!esp32Telemetry.readyForOperation())
        {
            phase = "esp32_not_ready";
            action = "Missão interrompida: ESP32 sem telemetria pronta";
        }
        else if (!cameraReady)
        {
            phase = "camera_not_ready";
            action = "Missão interrompida: câmera inferior indisponível";
        }
        else
        {
            phase = "line_ipc_stale";
            action = "Missão interrompida: IPC visual ausente ou antigo";
        }
        robotState.stop();
        robotState.updateAutonomousStatus(
            makeMainMissionStatus(phase, action));
        std::cout << "MainMission stopped: " << phase << std::endl;
        return;
    }

    // A classificação verde termina na telemetria. Somente o ponto de extensão
    // do seguidor normal pode fornecer potência e, nesta etapa, ele publica 0/0.
    robotState.driveAutonomous(
        cameraLineSnapshot.lineFollowerLeftPower,
        cameraLineSnapshot.lineFollowerRightPower);
    robotState.updateAutonomousStatus(makeMainMissionStatus(
        "line_follower_pending",
        "Seguidor de linha ainda não implementado: motores parados"));
}
