#include "obr/main_mission.h"

void MainMission::reset()
{
    lineCourseMission_.reset();
}

void MainMission::update(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    bool cameraReady,
    const CameraLineSnapshot& cameraLineSnapshot,
    const ForwardLineSnapshot& forwardLineSnapshot)
{
    lineCourseMission_.update(
        robotState,
        esp32Telemetry,
        cameraReady,
        cameraLineSnapshot,
        forwardLineSnapshot);
}
