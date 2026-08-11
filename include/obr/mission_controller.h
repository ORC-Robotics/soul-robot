#pragma once

#include "obr/esp32_bridge.h"
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
        const Esp32TelemetrySnapshot& esp32Telemetry);

private:
    enum class Turn90Phase
    {
        Idle,
        Turning,
        Settling,
        CorrectionPulse
    };

    enum class DistancePhase
    {
        Idle,
        Driving,
        Settling,
        CorrectionPulse
    };

    MainMission mainMission_;
    Turn90Phase turn90Phase_ = Turn90Phase::Idle;
    double turn90StartYawDegrees_ = 0.0;
    std::chrono::steady_clock::time_point turn90StartedAt_{};
    std::chrono::steady_clock::time_point turn90PhaseStartedAt_{};
    int turn90CorrectionPulseCount_ = 0;
    double turn90CorrectionDirection_ = 1.0;

    DistancePhase distancePhase_ = DistancePhase::Idle;
    long long distanceStartLeftCount_ = 0;
    long long distanceStartRightCount_ = 0;
    double activeDistanceTargetCm_ = 0.0;
    std::chrono::steady_clock::time_point distanceStartedAt_{};
    std::chrono::steady_clock::time_point distancePhaseStartedAt_{};
    std::chrono::steady_clock::time_point distanceLastProgressAt_{};
    double lastDistanceProgressCounts_ = 0.0;
    int distanceCorrectionPulseCount_ = 0;
    unsigned long long activeAutonomousRunSequence_ = 0;

    void updateTurnRight90(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry);
    void updateDriveDistance(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        double targetDistanceCm);
    void resetMissionState();

    static double angularDistanceDegrees(double first, double second);
};
