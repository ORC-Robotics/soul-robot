#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/imu_turn_controller.h"
#include "obr/robot_state.h"

#include <chrono>
#include <cstdint>

// Orquestra o segue-faixa, a recuperação, os gaps e as curvas por marcadores
// verdes da Missão Principal, sempre aplicando as proteções antes dos motores.
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
    enum class LineFollowState
    {
        TrackingNear,
        TurningAtGreenMarker,
        GreenDecisionConfirming,
        GreenTurnLeft,
        GreenTurnRight,
        Corner90Confirming,
        Corner90Left,
        Corner90Right,
        CrossingGap,
        ReacquiringNear,
        SearchingLeft,
        SearchingRight
    };

    enum class LineDirection
    {
        Unknown,
        Left,
        Right
    };

    LineFollowState state_ = LineFollowState::TrackingNear;
    ImuTurnController greenTurnController_;
    bool greenDecisionLatched_ = false;
    bool greenTurnAuthorized_ = false;
    LineDirection lastSignificantDirection_ = LineDirection::Unknown;
    LineDirection searchDirection_ = LineDirection::Unknown;
    double lastValidError_ = 0.0;
    std::uint64_t lastProcessedLineSequence_ = 0;
    bool hasProcessedLineSequence_ = false;
    int consecutiveNearValidSamples_ = 0;
    Corner90Direction corner90Direction_ = Corner90Direction::None;
    int corner90EnterSamples_ = 0;
    int corner90ExitSamples_ = 0;
    bool corner90WatchdogActive_ = false;
    std::chrono::steady_clock::time_point corner90StartedAt_{};
    std::chrono::steady_clock::time_point corner90LastLineSeenAt_{};
    double corner90StartYawDegrees_ = 0.0;
    GreenTurnDecision greenTurnDirection_ = GreenTurnDecision::None;
    int greenTurnConfirmSamples_ = 0;
    int greenTurnExitSamples_ = 0;
    bool nearRecoveryActive_ = false;
    bool totalLossActive_ = false;
    std::chrono::steady_clock::time_point nearLostAt_{};
    std::chrono::steady_clock::time_point totalLossStartedAt_{};
    bool gapNearLossObserved_ = false;
    std::chrono::steady_clock::time_point gapStartedAt_{};

    void transitionTo(LineFollowState nextState);
    bool updateGreenTurn(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        const CameraLineSnapshot& cameraLineSnapshot,
        bool newLineSample);
    void resetGapTracking();
    void startCorner90Watchdog(
        const Esp32TelemetrySnapshot& esp32Telemetry,
        std::chrono::steady_clock::time_point now);
    void resetCorner90Watchdog();
    const char* corner90AbortReason(
        const Esp32TelemetrySnapshot& esp32Telemetry,
        const CameraLineSnapshot& cameraLineSnapshot,
        bool newLineSample,
        std::chrono::steady_clock::time_point now);
    void updateDirectionMemory(double error);
    LineDirection chooseSearchDirection();
    static const char* stateName(LineFollowState state);
};
