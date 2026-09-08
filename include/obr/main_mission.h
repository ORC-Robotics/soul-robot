#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/line_course_mission.h"
#include "obr/rescue_area_mission.h"
#include "obr/robot_state.h"

#include <cstdint>

// Coordena as fases da missão oficial e delega cada comportamento ao módulo
// responsável, sem carregar algoritmos de movimento neste orquestrador.
class MainMission
{
public:
    void reset();
    bool requiresRescueVision() const;
    void update(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        bool cameraReady,
        const CameraLineSnapshot& cameraLineSnapshot,
        const ForwardLineSnapshot& forwardLineSnapshot);
    void update(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        bool cameraReady,
        const CameraLineSnapshot& cameraLineSnapshot,
        const ForwardLineSnapshot& forwardLineSnapshot,
        const ForwardBallSnapshot& forwardBallSnapshot,
        std::uint64_t autonomousRunSequence);

private:
    enum class Phase
    {
        InitialLineCourse,
        RescueArea,
        FinalLineCourse,
        Completed,
        Failed
    };

    Phase phase_ = Phase::InitialLineCourse;
    LineCourseMission lineCourseMission_;
    RescueAreaMission rescueAreaMission_;
};
