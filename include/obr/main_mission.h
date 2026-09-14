#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/line_course_mission.h"
#include "obr/rescue_room_mission.h"
#include "obr/rescue_exit_mission.h"
#include "obr/robot_state.h"
#include "obr/silver_entry_maneuver.h"

#include <cstdint>

// Coordena as fases da missão oficial e delega cada comportamento ao módulo
// responsável, sem carregar algoritmos de movimento neste orquestrador.
class MainMission
{
public:
    void reset(bool startAtExit = false);
    bool requiresExitVision() const;
    bool requiresRescueVision() const;
    bool requiresRescueZoneDetection() const;
    std::uint64_t rescueBallTargetSequence(
        std::uint64_t autonomousRunSequence) const;
    const char* rescueBallTargetType() const;
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
    void update(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        bool cameraReady,
        const CameraLineSnapshot& cameraLineSnapshot,
        const ForwardLineSnapshot& forwardLineSnapshot,
        const ForwardBallSnapshot& forwardBallSnapshot,
        const RescueZoneSnapshot& rescueZoneSnapshot,
        std::uint64_t autonomousRunSequence);

private:
    enum class Phase
    {
        InitialLineCourse,
        RescueArea,
        ExitSearch,
        FinalLineCourse,
        Failed
    };

    Phase phase_ = Phase::InitialLineCourse;
    LineCourseMission lineCourseMission_;
    SilverEntryManeuver silverEntryManeuver_;
    RescueRoomMission rescueRoomMission_;
    RescueExitMission rescueExitMission_;
    bool savedEntryHeadingValid_ = false;
    double savedEntryHeadingDegrees_ = 0.0;
};
