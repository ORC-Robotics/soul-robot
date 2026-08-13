#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/robot_state.h"

#include <chrono>
#include <cstdint>

// Orquestra os comportamentos da prova que formarão a Missão Principal.
// Nesta etapa, segue a linha pela NEAR e tenta recuperá-la pela FAR ou por busca.
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
        TurningAhead,
        ReacquiringNear,
        RecoveringFar,
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
    LineDirection lastSignificantDirection_ = LineDirection::Unknown;
    LineDirection searchDirection_ = LineDirection::Unknown;
    double lastValidError_ = 0.0;
    std::uint64_t lastProcessedLineSequence_ = 0;
    bool hasProcessedLineSequence_ = false;
    int consecutiveNearValidSamples_ = 0;
    bool aheadStrongTurnActive_ = false;
    LineDirection aheadStrongTurnDirection_ = LineDirection::Unknown;
    int aheadStrongTurnEnterSamples_ = 0;
    int aheadStrongTurnExitSamples_ = 0;
    std::uint64_t aheadStrongTurnLastLineSequence_ = 0;
    bool aheadStrongTurnHasLineSequence_ = false;
    double aheadStrongTurnLastHeadingError_ = 0.0;
    bool nearRecoveryActive_ = false;
    bool totalLossActive_ = false;
    std::chrono::steady_clock::time_point nearLostAt_{};
    std::chrono::steady_clock::time_point totalLossStartedAt_{};

    void transitionTo(LineFollowState nextState);
    void updateDirectionMemory(double error);
    LineDirection chooseSearchDirection();
    static const char* stateName(LineFollowState state);
};
