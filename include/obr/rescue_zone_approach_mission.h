#pragma once

#include "obr/esp32_bridge.h"
#include "obr/camera_monitor.h"
#include "obr/robot_state.h"

#include <chrono>
#include <limits>
#include <string>

struct RescueZoneApproachOutput
{
    double leftPower = 0.0;
    double rightPower = 0.0;
    bool completed = false;
    bool failed = false;
    AutonomousStatus status;
};

// Aproxima o robô mantendo o heading salvo pelo ALIGN_ZONE.
// ULTRA controla velocidade; a CAM1 também pode concluir ao perder informação útil.
class RescueZoneApproachMission
{
public:
    RescueZoneApproachOutput update(
        double lockedHeadingDegrees,
        const Esp32TelemetrySnapshot& telemetry,
        std::chrono::steady_clock::time_point now =
            std::chrono::steady_clock::now(),
        const RescueZoneSnapshot& zones = {},
        RescueZoneTargetColor targetColor = RescueZoneTargetColor::Green);
    void reset();
    // Suspende a contagem do avanço final quando o chamador zera os motores.
    // O timeout global continua correndo para limitar a aproximação inteira.
    void pause(std::chrono::steady_clock::time_point now);

private:
    bool started_ = false;
    bool nearLatched_ = false;
    bool finalAdvanceActive_ = false;
    bool paused_ = false;
    std::chrono::steady_clock::time_point pausedAt_{};
    bool completed_ = false;
    bool failed_ = false;
    double lockedHeadingDegrees_ =
        std::numeric_limits<double>::quiet_NaN();
    std::string terminalPhase_;
    std::string terminalAction_;
    std::string completionReason_;
    std::chrono::steady_clock::time_point startedAt_{};
    std::chrono::steady_clock::time_point finalAdvanceStartedAt_{};

    RescueZoneApproachOutput stopWithFailure(
        const std::string& phase,
        const std::string& action,
        const std::string& reason,
        const Esp32TelemetrySnapshot& telemetry);
    RescueZoneApproachOutput stoppedOutput(
        const std::string& phase,
        const std::string& action,
        const Esp32TelemetrySnapshot& telemetry,
        double headingErrorDegrees,
        const std::string& speedState) const;
    static bool ultrasonicFresh(const Esp32TelemetrySnapshot& telemetry);
    static bool ultrasonicValid(const Esp32TelemetrySnapshot& telemetry);
    static double signedHeadingErrorDegrees(
        double targetDegrees,
        double currentDegrees);
};
