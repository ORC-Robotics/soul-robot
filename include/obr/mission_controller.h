#pragma once

#include "obr/ball_alignment_mission.h"
#include "obr/esp32_bridge.h"
#include "obr/imu_turn_controller.h"
#include "obr/main_mission.h"
#include "obr/robot_state.h"

#include <chrono>

// Seleciona a missão autônoma ativa e mantém seus estados isolados.
// A Missão Principal delegará para comportamentos pequenos; os outros modos
// continuam sendo ferramentas independentes de teste do hardware.
class MissionController
{
public:
    void update(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        bool cameraReady,
        const CameraLineSnapshot& cameraLineSnapshot,
        const ForwardBallSnapshot& forwardBallSnapshot);

private:
    enum class DistancePhase
    {
        Idle,
        Driving,
        Settling,
        CorrectionPulse
    };

    MainMission mainMission_;
    BallAlignmentMission ballAlignmentMission_;
    ImuTurnController testTurnController_;

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
    void updateAlignClosestBall(
        RobotState& robotState,
        const ForwardBallSnapshot& forwardBallSnapshot);
    void resetMissionState();

};
