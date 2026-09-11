#pragma once

#include "obr/ball_alignment_mission.h"
#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/robot_state.h"

#include <chrono>
#include <cstdint>

struct RescueAreaOutput
{
    double leftPower = 0.0;
    double rightPower = 0.0;
    bool completed = false;
    bool failed = false;
    AutonomousStatus status;
};

// Possui o ciclo de vida da área de resgate e reutiliza o alinhamento de vítima.
// Até a busca da saída ser implementada, a missão completa aguarda parada.
class RescueAreaMission
{
public:
    RescueAreaOutput update(
        const ForwardBallSnapshot& forwardBallSnapshot,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        std::uint64_t autonomousRunSequence,
        bool rescueExitConfirmed,
        bool finishAfterVictim,
        std::chrono::steady_clock::time_point now =
            std::chrono::steady_clock::now());
    void reset();

private:
    enum class Phase
    {
        Searching,
        Aligning,
        PreparingCollectionAdvance,
        AdvancingForCollection,
        SettlingAfterCollectionAdvance,
        VictimReached,
        Failed
    };

    enum class SearchPhase
    {
        Ready,
        Pivoting,
        Settling,
        WaitingForFrame
    };

    BallAlignmentMission ballAlignmentMission_;
    Phase phase_ = Phase::Searching;
    SearchPhase searchPhase_ = SearchPhase::Ready;
    std::chrono::steady_clock::time_point searchPhaseStartedAt_{};
    std::chrono::steady_clock::time_point collectionPhaseStartedAt_{};
    std::chrono::steady_clock::time_point collectionLastProgressAt_{};
    double searchFrameBeforeMotionTimestamp_ = 0.0;
    double collectionLastProgressCounts_ = 0.0;
    long long collectionStartLeftCount_ = 0;
    long long collectionStartRightCount_ = 0;
    long long collectionLastUptimeMs_ = 0;
    int collectionDifferenceSamples_ = 0;
    AutonomousStatus failureStatus_;
};
