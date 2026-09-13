#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/robot_state.h"

#include <chrono>
#include <cstdint>
#include <limits>
#include <string>

struct RescueZoneAlignOutput
{
    double leftPower = 0.0;
    double rightPower = 0.0;
    bool completed = false;
    bool failed = false;
    AutonomousStatus status;
};

// Melhora bounds parciais e centraliza a zona quando aimNormalized está válido.
// A classe usa somente micro-pivôs temporizados e nunca depende de encoders.
class RescueZoneAlignMission
{
public:
    RescueZoneAlignOutput update(
        const RescueZoneSnapshot& zones,
        const Esp32TelemetrySnapshot& telemetry,
        RescueZoneTargetColor targetColor,
        std::chrono::steady_clock::time_point now =
            std::chrono::steady_clock::now());
    void reset();

private:
    enum class Phase
    {
        Evaluating,
        Pivoting,
        Settling,
        Completed
    };

    enum class Direction
    {
        Unavailable,
        Left,
        Right,
        Center
    };

    Phase phase_ = Phase::Evaluating;
    RescueZoneTargetColor targetColor_ = RescueZoneTargetColor::Green;
    bool targetColorInitialized_ = false;
    Direction currentDirection_ = Direction::Unavailable;
    Direction activePivotDirection_ = Direction::Unavailable;
    bool activePivotUsesPartialBound_ = false;
    RescueZoneGeometryState activePartialGeometry_ =
        RescueZoneGeometryState::NotDetected;
    double currentAimNormalized_ =
        std::numeric_limits<double>::quiet_NaN();
    double lockedHeading_ = std::numeric_limits<double>::quiet_NaN();
    int centeredFrames_ = 0;
    std::uint64_t lastEvaluatedSequence_ = 0;
    std::uint64_t movementEndSequence_ = 0;
    double movementEndTimestamp_ = 0.0;
    std::string terminalPhase_;
    std::string terminalAction_;
    std::string completionReason_;
    std::string lastLoggedPhase_;
    std::chrono::steady_clock::time_point phaseStartedAt_{};

    // Separa o diagnóstico por transição da decisão de movimento, sem log no laço rápido.
    RescueZoneAlignOutput updatePhase(
        const RescueZoneSnapshot& zones,
        const Esp32TelemetrySnapshot& telemetry,
        RescueZoneTargetColor targetColor,
        std::chrono::steady_clock::time_point now);

    RescueZoneAlignOutput evaluate(
        const RescueZoneSnapshot& zones,
        const Esp32TelemetrySnapshot& telemetry,
        std::chrono::steady_clock::time_point now);
    RescueZoneAlignOutput updatePivot(
        const RescueZoneSnapshot& zones,
        const Esp32TelemetrySnapshot& telemetry,
        std::chrono::steady_clock::time_point now);
    RescueZoneAlignOutput updateSettling(
        const RescueZoneSnapshot& zones,
        const Esp32TelemetrySnapshot& telemetry,
        std::chrono::steady_clock::time_point now);
    RescueZoneAlignOutput startPivot(
        Direction direction,
        bool usesPartialBound,
        RescueZoneGeometryState partialGeometry,
        const RescueZoneSnapshot& zones,
        const Esp32TelemetrySnapshot& telemetry,
        std::chrono::steady_clock::time_point now);
    RescueZoneAlignOutput complete(
        const std::string& reason,
        const std::string& action,
        double headingDegrees);
    RescueZoneAlignOutput pause(
        const std::string& phase,
        const std::string& action);
    RescueZoneAlignOutput stoppedOutput(
        const std::string& phase,
        const std::string& action,
        double progressPercent = 0.0) const;
    void beginSettling(
        const RescueZoneSnapshot& zones,
        std::chrono::steady_clock::time_point now);
    void updateAimTelemetry(const RescueZoneObservation& zone);
    static bool observationConfirmed(const RescueZoneObservation& zone);
    static bool aimUsable(const RescueZoneObservation& zone);
    static Direction classifyAim(double aimNormalized);
    static const char* directionName(Direction direction);
};
