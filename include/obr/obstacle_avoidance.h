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
    bool cameraBlackLeft = false;
    bool cameraBlackRight = false;
    int cameraBlackLeftFrames = 0;
    int cameraBlackRightFrames = 0;
    std::string selectedSideSource = "UNDECIDED";
    std::string rawBestParabolaSide = "NONE";
    std::string bestParabolaSide = "NONE";
    std::uint64_t bestParabolaScore = 0;
    std::uint64_t bestParabolaLeftBlack = 0;
    std::uint64_t bestParabolaRightBlack = 0;
    std::uint64_t bestParabolaSequence = 0;
    bool bestParabolaSideValid = false;
    bool nearForwardLineVisible = false;
    int nearForwardLineVotes = 0;
    int nearForwardLineSamples = 0;
    bool case3Armed = false;
    double case3FusionAcquireTime = 0.0;
    long long case3TimeRemainingMs = 0;
    std::string phase;
    std::string action;
};

// Centraliza, escolhe o lado livre e executa o contorno do obstáculo. Durante a
// curva, o Fusion pode antecipar uma busca curta e limitada pela linha.
class ObstacleAvoidance
{
public:
    ObstacleAvoidanceOutput update(
        const Esp32TelemetrySnapshot& telemetry,
        const CameraLineSnapshot& line,
        bool allowStart,
        const ForwardLineSnapshot& forwardLine = {});
    void reset();
    bool active() const;

private:
    enum class Phase
    {
        Idle,
        ReversingBeforeCentering,
        Centering,
        TurningLeftForMeasurement,
        SamplingLeftClearance,
        ReturningToBaseBeforeRight,
        TurningRightForMeasurement,
        SamplingRightClearance,
        PositioningSelectedSide,
        DrivingSelectedHeading,
        CurvingAroundObstacle,
        ReacquireForward,
        ReacquireSearch,
        ParabolaGapLostValidate,
        ParabolaReacquireForward,
        ParabolaReacquireSearch,
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
    bool cameraBlackLeft_ = false;
    bool cameraBlackRight_ = false;
    int cameraBlackLeftFrames_ = 0;
    int cameraBlackRightFrames_ = 0;
    std::uint64_t lastCameraBlackSequence_ = 0;
    std::string selectedSideSource_ = "UNDECIDED";
    std::vector<double> clearanceSamples_;
    double maximumClearanceAngleDegrees_ =
        std::numeric_limits<double>::quiet_NaN();
    long long lastSampleUptimeMs_ = -1;
    long long forwardStartLeftCount_ = 0;
    long long forwardStartRightCount_ = 0;
    long long reverseStartLeftCount_ = 0;
    long long reverseStartRightCount_ = 0;
    std::chrono::steady_clock::time_point reverseStartedAt_{};
    double selectedHeadingYaw_ = std::numeric_limits<double>::quiet_NaN();
    std::chrono::steady_clock::time_point forwardStartedAt_{};
    std::chrono::steady_clock::time_point clearanceSamplingStartedAt_{};
    long long curveStartLeftCount_ = 0;
    long long curveStartRightCount_ = 0;
    double curveStartYaw_ = std::numeric_limits<double>::quiet_NaN();
    double curveEndYaw_ = std::numeric_limits<double>::quiet_NaN();
    std::chrono::steady_clock::time_point curveStartedAt_{};
    int fusionReacquireFrames_ = 0;
    std::uint64_t lastFusionLineSequence_ = 0;
    long long reacquireForwardStartLeftCount_ = 0;
    long long reacquireForwardStartRightCount_ = 0;
    std::chrono::steady_clock::time_point reacquireForwardStartedAt_{};
    double reacquireSearchStartYaw_ = std::numeric_limits<double>::quiet_NaN();
    std::chrono::steady_clock::time_point reacquireSearchStartedAt_{};
    std::string rawBestParabolaSide_ = "NONE";
    std::string bestParabolaSide_ = "NONE";
    std::uint64_t bestParabolaScore_ = 0;
    std::uint64_t bestParabolaLeftBlack_ = 0;
    std::uint64_t bestParabolaRightBlack_ = 0;
    std::uint64_t bestParabolaSequence_ = 0;
    bool bestParabolaSideValid_ = false;
    std::uint64_t lastParabolaSequence_ = 0;
    bool case3AwaitingFusionAcquire_ = false;
    bool case3Armed_ = false;
    std::chrono::steady_clock::time_point case3FusionAcquireTime_{};
    int case3FusionAcquireFrames_ = 0;
    int case3GapLostFrames_ = 0;
    std::uint64_t lastCase3LineSequence_ = 0;
    bool nearForwardLineVisible_ = false;
    int nearForwardLineVotes_ = 0;
    int nearForwardLineSamples_ = 0;
    std::uint64_t lastNearValidationSequence_ = 0;
    long long parabolaReacquireForwardStartLeftCount_ = 0;
    long long parabolaReacquireForwardStartRightCount_ = 0;
    std::chrono::steady_clock::time_point parabolaReacquireForwardStartedAt_{};
    double parabolaSearchStartYaw_ = std::numeric_limits<double>::quiet_NaN();
    std::chrono::steady_clock::time_point parabolaSearchStartedAt_{};
    bool parabolaSearchOppositeSide_ = false;
    double case3PostObstacleYaw_ = std::numeric_limits<double>::quiet_NaN();
    bool parabolaRearFusionRecovery_ = false;

    ObstacleAvoidanceOutput updateIdle(
        const Esp32TelemetrySnapshot& telemetry,
        const CameraLineSnapshot& line,
        bool allowStart,
        const ForwardLineSnapshot& forwardLine);
    ObstacleAvoidanceOutput updateCentering(
        const Esp32TelemetrySnapshot& telemetry,
        const CameraLineSnapshot& line);
    ObstacleAvoidanceOutput updateInitialReverse(
        const Esp32TelemetrySnapshot& telemetry);
    ObstacleAvoidanceOutput startCentering(
        const std::string& action);
    ObstacleAvoidanceOutput updateTurn(
        const Esp32TelemetrySnapshot& telemetry,
        const CameraLineSnapshot& line,
        const ForwardLineSnapshot& forwardLine);
    ObstacleAvoidanceOutput updateClearanceSampling(
        const Esp32TelemetrySnapshot& telemetry,
        bool measuringLeft);
    ObstacleAvoidanceOutput startClearanceSampling(bool measuringLeft);
    ObstacleAvoidanceOutput continueAfterRightMeasurement(
        const Esp32TelemetrySnapshot& telemetry);
    ObstacleAvoidanceOutput updateSelectedForward(
        const Esp32TelemetrySnapshot& telemetry);
    ObstacleAvoidanceOutput updateCurve(
        const Esp32TelemetrySnapshot& telemetry,
        const CameraLineSnapshot& line,
        const ForwardLineSnapshot& forwardLine);
    ObstacleAvoidanceOutput updateReacquireForward(
        const Esp32TelemetrySnapshot& telemetry,
        const CameraLineSnapshot& line);
    ObstacleAvoidanceOutput updateReacquireSearch(
        const Esp32TelemetrySnapshot& telemetry,
        const CameraLineSnapshot& line);
    ObstacleAvoidanceOutput updateParabolaGapLostValidation(
        const Esp32TelemetrySnapshot& telemetry,
        const CameraLineSnapshot& line,
        const ForwardLineSnapshot& forwardLine);
    ObstacleAvoidanceOutput updateParabolaReacquireForward(
        const Esp32TelemetrySnapshot& telemetry,
        const CameraLineSnapshot& line);
    ObstacleAvoidanceOutput updateParabolaReacquireSearch(
        const Esp32TelemetrySnapshot& telemetry,
        const CameraLineSnapshot& line);
    void collectStableClearance(
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
    std::string observeCameraBlack(
        const Esp32TelemetrySnapshot& telemetry,
        const ForwardLineSnapshot& forwardLine,
        bool measuringLeft);
    bool observeFreshFusion(const CameraLineSnapshot& line);
    void observeParabolaFrame(const ForwardLineSnapshot& forwardLine);
    bool observeCase3FusionAcquire(const CameraLineSnapshot& line);
    ObstacleAvoidanceOutput updateCase3Idle(
        const Esp32TelemetrySnapshot& telemetry,
        const CameraLineSnapshot& line,
        const ForwardLineSnapshot& forwardLine);
    void armCase3FusionWindow();
    ObstacleAvoidanceOutput startParabolaRearFusionRecovery(
        const Esp32TelemetrySnapshot& telemetry,
        const CameraLineSnapshot& line);
    void clearCase3Evidence();
    void resetCameraBlackEvidence();
    static double maximumClearance(const std::vector<double>& samples);
    static double normalizedYaw(double yawDegrees);
    static double signedYawError(
        double targetDegrees,
        double currentDegrees);
};
