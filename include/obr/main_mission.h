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
        GreenTurnWaitingImu,
        GreenTurnLeft,
        GreenTurnRight,
        GreenTurnVisualHandoff,
        GreenTurnForwardProbe,
        Corner90Confirming,
        Corner90Left,
        Corner90Right,
        CrossingGap,
        ReacquiringNear,
        LineRecoveryMemory
    };

    enum class LineDirection
    {
        Unknown,
        Left,
        Right
    };

    LineFollowState state_ = LineFollowState::TrackingNear;
    ImuTurnController greenTurnController_;
    ImuTurnController greenDirectionalTurnController_;
    bool greenDecisionLatched_ = false;
    bool greenTurnAuthorized_ = false;
    LineDirection lineRecoveryDirection_ = LineDirection::Unknown;
    double lastValidError_ = 0.0;
    std::uint64_t lastProcessedLineSequence_ = 0;
    bool hasProcessedLineSequence_ = false;
    int consecutiveNearValidSamples_ = 0;
    BlackLineGeometryDirection corner90Direction_ =
        BlackLineGeometryDirection::None;
    int corner90EnterSamples_ = 0;
    int corner90ConfirmationWindowSamples_ = 0;
    int corner90ExitSamples_ = 0;
    bool corner90WatchdogActive_ = false;
    std::chrono::steady_clock::time_point corner90StartedAt_{};
    std::chrono::steady_clock::time_point corner90LastLineSeenAt_{};
    GreenTurnDecision greenTurnDirection_ = GreenTurnDecision::None;
    int greenTurnConfirmSamples_ = 0;
    std::chrono::steady_clock::time_point greenTurnVisualHandoffStartedAt_{};
    std::chrono::steady_clock::time_point greenTurnIgnoreUntil_{};
    long long greenTurnProbeStartLeftEncoderCount_ = 0;
    long long greenTurnProbeStartRightEncoderCount_ = 0;
    double greenTurnProbeLastProgressCounts_ = 0.0;
    std::chrono::steady_clock::time_point greenTurnProbeLastProgressAt_{};
    bool nearRecoveryActive_ = false;
    bool hasLineRecoveryMemory_ = false;
    std::uint64_t lineRecoveryMemorySequence_ = 0;
    double lineRecoveryLeftPower_ = 0.0;
    double lineRecoveryRightPower_ = 0.0;
    long long lineRecoveryStartLeftEncoderCount_ = 0;
    long long lineRecoveryStartRightEncoderCount_ = 0;
    double lineRecoveryLastProgressCounts_ = 0.0;
    std::chrono::steady_clock::time_point lineRecoveryLastProgressAt_{};
    bool gapNearLossObserved_ = false;
    std::chrono::steady_clock::time_point gapStartedAt_{};

    void transitionTo(LineFollowState nextState);
    bool updateGreenTurn(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        const CameraLineSnapshot& cameraLineSnapshot,
        bool newLineSample);
    void resetGapTracking();
    void resetGreenDirectionalTurnTracking();
    void startCorner90Watchdog(std::chrono::steady_clock::time_point now);
    void resetCorner90Watchdog();
    const char* corner90AbortReason(
        const Esp32TelemetrySnapshot& esp32Telemetry,
        const CameraLineSnapshot& cameraLineSnapshot,
        bool newLineSample,
        std::chrono::steady_clock::time_point now);
    void updateDirectionMemory(double error);
    void resetLineRecoveryMemory();
    static const char* stateName(LineFollowState state);
};
