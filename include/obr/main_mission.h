#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/imu_turn_controller.h"
#include "obr/robot_state.h"
#include "obstacle_avoidance/obstacle_avoidance.h"

#include <chrono>

// Executa o segue-linha e comportamentos curtos disparados pela visão verde.
// A classe não acessa hardware diretamente e mantém cada etapa fail-safe.
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
    enum class ForwardAssistState
    {
        Bottom,
        SearchSpin,
        ForwardFollow
    };

    enum class ForwardAssistDirection
    {
        None,
        Left,
        Right
    };

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

    ForwardAssistState forwardAssistState_ = ForwardAssistState::Bottom;
    ForwardAssistDirection forwardAssistDirection_ =
        ForwardAssistDirection::None;
    double forwardAssistYawOriginDegrees_ = 0.0;
    double forwardAssistYawDeltaDegrees_ = 0.0;
    int bottomStableFrames_ = 0;
    int bottomLostFrames_ = 0;
    bool hasPreviousBottomFrame_ = false;
    std::uint64_t previousBottomSequence_ = 0;
    bool previousBottomTrusted_ = false;
    bool previousBottomLineNormal_ = false;
    ForwardAssistDirection latchedBottomDirection_ =
        ForwardAssistDirection::None;
    ForwardAssistDirection mediumFlipCandidateDirection_ =
        ForwardAssistDirection::None;
    int mediumFlipConfirmationFrames_ = 0;
    bool forwardAssistFarTrusted_ = false;
    bool forwardAssistMediumTrusted_ = false;
    bool forwardAssistGapCandidate_ = false;
    bool forwardAssistEntryAllowed_ = false;
    std::string forwardAssistEntryBlocker_ = "WAITING_TRUST";

    void resetForwardAssist();
    void updateLatchedBottomDirection(
        ForwardAssistDirection farDirection,
        ForwardAssistDirection mediumDirection,
        bool consecutiveBottomFrame);
    bool updateForwardAssist(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        const CameraLineSnapshot& cameraLineSnapshot,
        const ForwardLineSnapshot& forwardLineSnapshot);
    AutonomousStatus forwardAssistStatus(
        const std::string& phase,
        const std::string& action,
        const ForwardLineSnapshot& forwardLineSnapshot) const;
};
