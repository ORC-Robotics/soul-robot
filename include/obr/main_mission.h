#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/imu_turn_controller.h"
#include "obstacle_avoidance/obstacle_avoidance.h"
#include "obr/robot_state.h"

#include <chrono>

// Executa o segue-linha, o desvio de obstáculo e as manobras disparadas pelo verde.
// A classe não acessa hardware diretamente e mantém cada etapa fail-safe.
class MainMission
{
public:
    void reset();
    void update(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        bool cameraReady,
        const CameraLineSnapshot& cameraLineSnapshot);

private:
    enum class TurnAroundPhase
    {
        Idle,
        RecognizedStopping,
        DrivingForward,
        ForwardSettling,
        TurningByImu,
        SearchingLine
    };

    TurnAroundPhase turnAroundPhase_ = TurnAroundPhase::Idle;
    ObstacleAvoidance obstacleAvoidance_;
    ImuTurnController turnAroundController_;
    bool turnAroundArmed_ = true;
    long long forwardStartLeftCount_ = 0;
    long long forwardStartRightCount_ = 0;
    int lineReacquireFrames_ = 0;
    bool lineSearchSawNearLine_ = false;
    double lineSearchStartYawDegrees_ = 0.0;
    std::chrono::steady_clock::time_point phaseStartedAt_{};
};
