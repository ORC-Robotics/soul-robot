#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/line_course_mission.h"
#include "obr/robot_state.h"

// Coordena as fases completas da missão oficial da competição.
// Nesta etapa, delega o comportamento existente ao percurso de linha.
class MainMission
{
public:
    void reset();
    void update(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        bool cameraReady,
        const CameraLineSnapshot& cameraLineSnapshot,
        const ForwardLineSnapshot& forwardLineSnapshot);

private:
    LineCourseMission lineCourseMission_;
};
