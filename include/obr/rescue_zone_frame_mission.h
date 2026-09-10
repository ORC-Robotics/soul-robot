#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/imu_turn_controller.h"
#include "obr/robot_state.h"

#include <chrono>
#include <cstdint>
#include <string>

struct RescueZoneFrameOutput
{
    double leftPower = 0.0;
    double rightPower = 0.0;
    bool completed = false;
    bool failed = false;
    AutonomousStatus status;
};

// Tenta melhorar rapidamente o enquadramento lateral da zona alvo.
// FULL_BOUNDS é a condição ideal, mas a missão aceita o melhor enquadramento
// disponível sem recuar nem criar um centro visual alternativo.
class RescueZoneFrameMission
{
public:
    RescueZoneFrameOutput update(
        const RescueZoneSnapshot& zones,
        const Esp32TelemetrySnapshot& telemetry,
        RescueZoneTargetColor targetColor,
        std::chrono::steady_clock::time_point now =
            std::chrono::steady_clock::now());
    void reset();

private:
    enum class Phase
    {
        WaitingObservation,
        Evaluating,
        Turning,
        LateralSettling,
        Completed,
        Failed
    };

    Phase phase_ = Phase::WaitingObservation;
    RescueZoneTargetColor targetColor_ = RescueZoneTargetColor::Green;
    bool targetColorInitialized_ = false;
    RescueZoneGeometryState movementGeometryState_ =
        RescueZoneGeometryState::NotDetected;
    double pivotLastYawDegrees_ = 0.0;
    double pivotAccumulatedDegrees_ = 0.0;
    bool pivotSafetyReferenceInitialized_ = false;
    int microPivotAttempts_ = 0;
    std::uint64_t lateralMovementEndSequence_ = 0;
    double lateralMovementEndTimestamp_ = 0.0;
    std::string completionReason_;
    std::string completionAction_;
    std::chrono::steady_clock::time_point phaseStartedAt_{};
    std::chrono::steady_clock::time_point pivotOperationStartedAt_{};

    RescueZoneFrameOutput evaluate(
        const RescueZoneSnapshot& zones,
        const Esp32TelemetrySnapshot& telemetry,
        std::chrono::steady_clock::time_point now);
    RescueZoneFrameOutput updateTurn(
        const RescueZoneSnapshot& zones,
        const Esp32TelemetrySnapshot& telemetry,
        std::chrono::steady_clock::time_point now);
    RescueZoneFrameOutput updateLateralSettling(
        const RescueZoneSnapshot& zones,
        const Esp32TelemetrySnapshot& telemetry,
        std::chrono::steady_clock::time_point now);
    RescueZoneFrameOutput complete(
        const std::string& reason,
        const std::string& action);
    RescueZoneFrameOutput fail(const std::string& phase, const std::string& action);
    void beginLateralSettling(
        const RescueZoneSnapshot& zones,
        std::chrono::steady_clock::time_point now);
};
