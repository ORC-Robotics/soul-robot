#pragma once

#include "obr/esp32_bridge.h"
#include "obr/imu_turn_controller.h"
#include "obr/main_mission.h"
#include "obr/obstacle_avoidance.h"
#include "obr/rescue_area_mission.h"
#include "obr/rescue_zone_align_mission.h"
#include "obr/rescue_zone_approach_mission.h"
#include "obr/rescue_zone_search_mission.h"
#include "obr/rescue_zone_triangle_mission.h"
#include "obr/robot_state.h"
#include "obr/servo_routine.h"

#include <chrono>

// Seleciona a missão autônoma ativa e mantém seus estados isolados.
// A Missão Principal delegará para comportamentos pequenos; os outros modos
// continuam sendo ferramentas independentes de teste do hardware.
class MissionController
{
public:
    bool requiresExitVision(const RobotSnapshot& snapshot) const;
    bool requiresForwardBallDetection(const RobotSnapshot& snapshot) const;
    bool requiresRescueZoneDetection(const RobotSnapshot& snapshot) const;
    std::uint64_t forwardBallTargetSequence(
        const RobotSnapshot& snapshot) const;
    const char* forwardBallTargetType(const RobotSnapshot& snapshot) const;
    void update(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        bool cameraReady,
        const CameraLineSnapshot& cameraLineSnapshot,
        const ForwardLineSnapshot& forwardLineSnapshot,
        const ForwardBallSnapshot& forwardBallSnapshot,
        const RescueZoneSnapshot& rescueZoneSnapshot = {});

private:
    enum class DistancePhase
    {
        Idle,
        Driving,
        Settling,
        CorrectionPulse
    };

    MainMission mainMission_;
    RescueAreaMission rescueAreaMission_;
    RescueZoneAlignMission rescueZoneAlignMission_;
    RescueZoneApproachMission rescueZoneApproachMission_;
    RescueZoneSearchMission rescueZoneSearchMission_;
    RescueZoneTriangleMission rescueZoneTriangleMission_;
    ObstacleAvoidance obstacleAvoidanceTest_;
    ImuTurnController testTurnController_;
    ServoRoutine servoRoutine_;

    DistancePhase distancePhase_ = DistancePhase::Idle;
    long long distanceStartLeftCount_ = 0;
    long long distanceStartRightCount_ = 0;
    double activeDistanceTargetCm_ = 0.0;
    std::chrono::steady_clock::time_point distanceStartedAt_{};
    std::chrono::steady_clock::time_point distancePhaseStartedAt_{};
    std::chrono::steady_clock::time_point distanceLastProgressAt_{};
    double lastDistanceProgressCounts_ = 0.0;
    int distanceCorrectionPulseCount_ = 0;
    int distanceDifferenceSamples_ = 0;
    unsigned long long distanceLastDifferenceUptimeMs_ = 0;
    unsigned long long activeAutonomousRunSequence_ = 0;

    void updateTurnRight90(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry);
    void updateDriveDistance(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        double targetDistanceCm);
    void updateRescueArea(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        const ForwardBallSnapshot& forwardBallSnapshot);
    void updateRescueZoneDetection(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry);
    void updateRescueZoneAlign(
        RobotState& robotState,
        const RobotSnapshot& snapshot,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        const RescueZoneSnapshot& rescueZoneSnapshot);
    void updateRescueZoneSearch(
        RobotState& robotState,
        const RobotSnapshot& snapshot,
        const RescueZoneSnapshot& rescueZoneSnapshot);
    void updateRescueZoneApproach(
        RobotState& robotState,
        const RobotSnapshot& snapshot,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        const RescueZoneSnapshot& rescueZoneSnapshot);
    void updateRescueZoneTriangle(
        RobotState& robotState,
        const RobotSnapshot& snapshot,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        const RescueZoneSnapshot& rescueZoneSnapshot);
    void updateObstacleAvoidance(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        const CameraLineSnapshot& cameraLineSnapshot,
        const ForwardLineSnapshot& forwardLineSnapshot);
    void updateServoRoutine(
        RobotState& robotState,
        ServoRoutineKind kind,
        const RobotSnapshot& snapshot,
        const Esp32TelemetrySnapshot& esp32Telemetry);
    void resetMissionState();

};
