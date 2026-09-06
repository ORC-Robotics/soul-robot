#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/forward_line_assist.h"
#include "obr/imu_turn_controller.h"
#include "obr/robot_state.h"
#include "obr/obstacle_avoidance.h"

#include <chrono>

// Coordena o percurso de linha e as manobras que podem assumir seu controle.
// A classe não acessa hardware diretamente e mantém cada etapa fail-safe.
class LineCourseMission
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
    enum class TurnAroundPhase
    {
        Idle,
        RecognitionDelay,
        Centering,
        PostCenteringDelay,
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
    double lineSearchStartYawDegrees_ = 0.0;
    std::chrono::steady_clock::time_point phaseStartedAt_{};
    ForwardLineAssist forwardLineAssist_;
};
