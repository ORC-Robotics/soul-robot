#pragma once

#include "obr/esp32_bridge.h"
#include "obr/imu_turn_controller.h"

#include <chrono>
#include <string>

struct ObstacleAvoidanceOutput
{
    bool hasControl = false;
    bool completed = false;
    bool failed = false;
    double leftPower = 0.0;
    double rightPower = 0.0;
    double progressPercent = 0.0;
    double targetDistanceCm = 0.0;
    double leftDistanceCm = 0.0;
    double rightDistanceCm = 0.0;
    std::string phase;
    std::string action;
};

// Módulo exportável que executa o contorno sem acessar diretamente o hardware.
// Cada etapa mantém autoridade até terminar ou falhar com segurança.
class ObstacleAvoidance
{
public:
    ObstacleAvoidanceOutput update(
        const Esp32TelemetrySnapshot& telemetry,
        bool allowStart);
    void reset();
    bool active() const;

private:
    enum class Phase
    {
        Idle,
        InitialSettling,
        TurningRight,
        FirstForward,
        FirstForwardSettling,
        TurningLeft45,
        SecondForward,
        SecondForwardSettling,
        TurningLeft90,
        ThirdForward,
        ThirdForwardSettling,
        TurningRight90,
        Reversing,
        ReverseSettling
    };

    Phase phase_ = Phase::Idle;
    ImuTurnController turnController_;
    bool armed_ = true;
    int obstacleConfirmationSamples_ = 0;
    int rearmConfirmationSamples_ = 0;
    long long distanceStartLeftCount_ = 0;
    long long distanceStartRightCount_ = 0;
    std::chrono::steady_clock::time_point phaseStartedAt_{};

    ObstacleAvoidanceOutput updateIdle(
        const Esp32TelemetrySnapshot& telemetry,
        bool allowStart);
    ObstacleAvoidanceOutput updateTurn(
        const Esp32TelemetrySnapshot& telemetry);
    ObstacleAvoidanceOutput updateSettling(
        const Esp32TelemetrySnapshot& telemetry);
    ObstacleAvoidanceOutput updateDistance(
        const Esp32TelemetrySnapshot& telemetry,
        double targetDistanceCm,
        double motorPower,
        const std::string& phase,
        const std::string& action);
    bool startTurn(
        Phase phase,
        double degrees,
        ImuTurnDirection direction,
        const Esp32TelemetrySnapshot& telemetry);
    void startDistance(
        Phase phase,
        const Esp32TelemetrySnapshot& telemetry);
    void startSettling(Phase phase);
    ObstacleAvoidanceOutput fail(
        const std::string& phase,
        const std::string& action);
};
