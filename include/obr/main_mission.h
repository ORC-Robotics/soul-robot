#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/imu_turn_controller.h"
#include "obr/robot_state.h"

#include <chrono>
#include <cstdint>

// Orquestra a Missão Principal e mantém as manobras dos marcadores verdes
// isoladas do ponto ainda não implementado do seguidor normal de linha.
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
    enum class State
    {
        NormalLineFollowing,
        GreenTurnWaitingImu,
        GreenTurnLeft,
        GreenTurnRight,
        GreenTurnVisualHandoff,
        GreenTurnForwardProbe,
        GreenTurnReacquiringLine,
        TurningAtGreenMarker
    };

    State state_ = State::NormalLineFollowing;
    ImuTurnController greenTurnController_;
    ImuTurnController greenDirectionalTurnController_;
    bool greenDecisionLatched_ = false;
    bool greenTurnAuthorized_ = false;
    GreenTurnDecision greenTurnDirection_ = GreenTurnDecision::None;
    int greenTurnConfirmSamples_ = 0;
    std::uint64_t lastProcessedLineSequence_ = 0;
    bool hasProcessedLineSequence_ = false;
    int consecutiveGreenReacquisitionSamples_ = 0;
    std::chrono::steady_clock::time_point greenTurnVisualHandoffStartedAt_{};
    std::chrono::steady_clock::time_point greenTurnIgnoreUntil_{};
    long long greenTurnProbeStartLeftEncoderCount_ = 0;
    long long greenTurnProbeStartRightEncoderCount_ = 0;
    double greenTurnProbeLastProgressCounts_ = 0.0;
    std::chrono::steady_clock::time_point greenTurnProbeLastProgressAt_{};

    void transitionTo(State nextState);
    bool updateGreenTurnAround(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        const CameraLineSnapshot& cameraLineSnapshot,
        bool newLineSample);
    void resetGreenDirectionalTurnTracking();
    static const char* stateName(State state);
};
