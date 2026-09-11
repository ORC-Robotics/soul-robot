#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/imu_turn_controller.h"
#include "obr/line_centering_controller.h"

#include <chrono>
#include <limits>
#include <string>
#include <vector>

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
    double yawBase = std::numeric_limits<double>::quiet_NaN();
    double leftClearance = std::numeric_limits<double>::quiet_NaN();
    double rightClearance = std::numeric_limits<double>::quiet_NaN();
    std::string selectedSide = "NONE";
    std::string phase;
    std::string action;
};

// Centraliza, escolhe o lado livre e avança a primeira reta antes do futuro
// contorno. Esta versão termina parada após 10 cm no yaw selecionado.
class ObstacleAvoidance
{
public:
    ObstacleAvoidanceOutput update(
        const Esp32TelemetrySnapshot& telemetry,
        const CameraLineSnapshot& line,
        bool allowStart);
    void reset();
    bool active() const;

private:
    enum class Phase
    {
        Idle,
        Centering,
        TurningLeftForMeasurement,
        ReturningToBaseBeforeRight,
        TurningRightForMeasurement,
        PositioningSelectedSide,
        DrivingSelectedHeading,
        CurvingAroundObstacle,
        FinalInwardPivot
    };

    Phase phase_ = Phase::Idle;
    ImuTurnController turnController_;
    LineCenteringController lineCenteringController_;
    bool armed_ = true;
    int obstacleConfirmationSamples_ = 0;
    int rearmConfirmationSamples_ = 0;
    double yawBase_ = std::numeric_limits<double>::quiet_NaN();
    double leftClearance_ = std::numeric_limits<double>::quiet_NaN();
    double rightClearance_ = std::numeric_limits<double>::quiet_NaN();
    std::string selectedSide_ = "NONE";
    std::vector<double> clearanceSamples_;
    long long lastSampleUptimeMs_ = -1;
    long long forwardStartLeftCount_ = 0;
    long long forwardStartRightCount_ = 0;
    double selectedHeadingYaw_ = std::numeric_limits<double>::quiet_NaN();
    std::chrono::steady_clock::time_point forwardStartedAt_{};
    long long curveStartLeftCount_ = 0;
    long long curveStartRightCount_ = 0;
    double curveStartYaw_ = std::numeric_limits<double>::quiet_NaN();
    double curveEndYaw_ = std::numeric_limits<double>::quiet_NaN();
    std::chrono::steady_clock::time_point curveStartedAt_{};

    ObstacleAvoidanceOutput updateIdle(
        const Esp32TelemetrySnapshot& telemetry,
        const CameraLineSnapshot& line,
        bool allowStart);
    ObstacleAvoidanceOutput updateCentering(
        const Esp32TelemetrySnapshot& telemetry,
        const CameraLineSnapshot& line);
    ObstacleAvoidanceOutput updateTurn(
        const Esp32TelemetrySnapshot& telemetry);
    ObstacleAvoidanceOutput updateSelectedForward(
        const Esp32TelemetrySnapshot& telemetry);
    ObstacleAvoidanceOutput updateCurve(
        const Esp32TelemetrySnapshot& telemetry);
    void collectClearanceDuringTurn(
        const Esp32TelemetrySnapshot& telemetry,
        bool measuringLeft);
    bool finishClearanceMeasurement(bool measuringLeft);
    bool startTurnToYaw(
        Phase phase,
        double targetYawDegrees,
        const Esp32TelemetrySnapshot& telemetry);
    ObstacleAvoidanceOutput startSelectedForward(
        const Esp32TelemetrySnapshot& telemetry);
    ObstacleAvoidanceOutput startCurve(
        const Esp32TelemetrySnapshot& telemetry);
    ObstacleAvoidanceOutput output(
        const std::string& phase,
        const std::string& action,
        double leftPower = 0.0,
        double rightPower = 0.0) const;
    ObstacleAvoidanceOutput fail(
        const std::string& phase,
        const std::string& action);
    static double minimumClearance(const std::vector<double>& samples);
    static double normalizedYaw(double yawDegrees);
    static double signedYawError(
        double targetDegrees,
        double currentDegrees);
};
