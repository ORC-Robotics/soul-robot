#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/robot_state.h"

#include <chrono>
#include <cstdint>
#include <string>

// Comando seguro calculado pela missão isolada de alinhamento visual.
struct BallAlignmentOutput
{
    double leftPower = 0.0;
    double rightPower = 0.0;
    bool finished = false;
    AutonomousStatus status;
};

// Alinha pelo tx e avança até a bola com pequenas correções angulares.
// A missão não acessa os motores diretamente e para se a visão ficar inválida.
class BallAlignmentMission
{
public:
    BallAlignmentOutput update(
        const ForwardBallSnapshot& ball,
        const Esp32TelemetrySnapshot& telemetry,
        std::uint64_t expectedTargetSequence,
        std::chrono::steady_clock::time_point now =
            std::chrono::steady_clock::now());
    void reset();

private:
    enum class Phase
    {
        Tracking,
        SettlingForVerification,
        FineCorrectionPulse,
        Approaching,
        Completed,
        Failed
    };

    Phase phase_ = Phase::Tracking;
    std::uint64_t expectedTargetSequence_ = 0;
    double lastTurnDirection_ = 0.0;
    double phaseStartBallTimestamp_ = 0.0;
    double lastStableBallTimestamp_ = 0.0;
    double fineCorrectionDirection_ = 0.0;
    int stableFrameCount_ = 0;
    std::string failurePhase_;
    std::string failureAction_;
    bool targetAcquired_ = false;
    bool targetLossActive_ = false;
    bool startCommandIssued_ = false;
    bool motionConfirmed_ = false;
    std::chrono::steady_clock::time_point phaseStartedAt_{};
    std::chrono::steady_clock::time_point targetLostAt_{};
};
