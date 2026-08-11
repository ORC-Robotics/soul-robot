#pragma once

#include "obr/esp32_bridge.h"
#include "obr/robot_state.h"

#include <chrono>
#include <string>

// Controlador autônomo simples que lê a visão da câmera e gera comandos seguros.
// Ele não acessa GPIO diretamente; os motores continuam passando pelo RobotState.
class LineFollower
{
public:
    void update(RobotState& robotState, const Esp32TelemetrySnapshot& esp32Telemetry);
    bool cameraReady() const;

private:
    enum class NavigationState
    {
        Following,
        ApproachingEvent,
        AdvancingToCorner,
        ExecutingTurn,
        ReversingAfterCorner,
        Reacquiring,
        LineLost
    };

    enum class ManeuverKind
    {
        None,
        CornerLeft,
        CornerRight,
        GreenLeft,
        GreenRight,
        GreenUTurn
    };

    enum class GreenPhase
    {
        None,
        Approaching,
        Turning
    };

    enum class Turn90Phase
    {
        Idle,
        Turning,
        Settling,
        CorrectionPulse
    };

    enum class DistancePhase
    {
        Idle,
        Driving,
        Settling,
        CorrectionPulse
    };

    struct CameraStatus
    {
        bool valid = false;
        bool active = false;
        bool lineDetected = false;
        bool currentPathValid = false;
        double fps = 0.0;
        double lineError = 0.0;
        double positionErrorPixels = 0.0;
        double headingErrorDegrees = 0.0;
        double pathConfidence = 0.0;
        double nearPathX = 0.0;
        double midPathX = 0.0;
        double farPathX = 0.0;
        bool previewEventDetected = false;
        std::string previewEventType = "NONE";
        std::string previewEventDirection = "NONE";
        double previewEventProximity = 0.0;
        double previewEventConfidence = 0.0;
        double timestampSeconds = 0.0;
        std::string greenAction = "NENHUM";
        double greenProximity = 0.0;
        double greenConfidence = 0.0;
    };

    struct TrackedEvent
    {
        std::string type = "NONE";
        std::string direction = "NONE";
        int hits = 0;
        int misses = 0;
        bool confirmed = false;
        bool latched = false;
        double proximity = 0.0;
        double confidence = 0.0;
        long long latchedLeftEncoderCount = 0;
        long long latchedRightEncoderCount = 0;
        std::chrono::steady_clock::time_point latchedAt{};
    };

    NavigationState navigationState_ = NavigationState::Following;
    ManeuverKind activeManeuver_ = ManeuverKind::None;
    GreenPhase greenPhase_ = GreenPhase::None;
    TrackedEvent cornerEvent_;
    TrackedEvent greenEvent_;
    std::string activeGreenAction_ = "NENHUM";
    std::chrono::steady_clock::time_point phaseUntil_{};
    std::chrono::steady_clock::time_point greenCooldownUntil_{};
    std::chrono::steady_clock::time_point cornerCooldownUntil_{};
    std::chrono::steady_clock::time_point maneuverStartedAt_{};
    std::chrono::steady_clock::time_point reacquireStartedAt_{};
    std::chrono::steady_clock::time_point lineLostStartedAt_{};
    double filteredLineError_ = 0.0;
    double filteredHeadingError_ = 0.0;
    double lastSteeringCorrection_ = 0.0;
    double lastCameraTimestampSeconds_ = 0.0;
    double maneuverStartCameraTimestampSeconds_ = 0.0;
    bool lineErrorFilterInitialized_ = false;
    bool turnDepartedOldPath_ = false;
    bool reacquireStationary_ = false;
    bool initialLineAcquired_ = false;
    int initialLineValidFrames_ = 0;
    int reacquireValidFrames_ = 0;
    int lineLostFrames_ = 0;
    int lastSearchDirection_ = 0;
    std::string pendingCornerDirection_ = "NONE";
    bool cornerTranslationEncoderInitialized_ = false;
    bool cornerTranslationSettling_ = false;
    long long cornerTranslationStartLeftCount_ = 0;
    long long cornerTranslationStartRightCount_ = 0;
    double lastCornerTranslationProgressCounts_ = 0.0;
    std::chrono::steady_clock::time_point cornerTranslationStartedAt_{};
    std::chrono::steady_clock::time_point cornerTranslationLastProgressAt_{};
    Turn90Phase turn90Phase_ = Turn90Phase::Idle;
    double turn90StartYawDegrees_ = 0.0;
    std::chrono::steady_clock::time_point turn90StartedAt_{};
    std::chrono::steady_clock::time_point turn90PhaseStartedAt_{};
    int turn90CorrectionPulseCount_ = 0;
    double turn90CorrectionDirection_ = 1.0;
    DistancePhase distancePhase_ = DistancePhase::Idle;
    long long distanceStartLeftCount_ = 0;
    long long distanceStartRightCount_ = 0;
    double activeDistanceTargetCm_ = 0.0;
    std::chrono::steady_clock::time_point distanceStartedAt_{};
    std::chrono::steady_clock::time_point distancePhaseStartedAt_{};
    std::chrono::steady_clock::time_point distanceLastProgressAt_{};
    double lastDistanceProgressCounts_ = 0.0;
    int distanceCorrectionPulseCount_ = 0;
    unsigned long long activeAutonomousRunSequence_ = 0;

    CameraStatus readCameraStatus() const;
    void updateMainMission(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        const CameraStatus& status,
        std::chrono::steady_clock::time_point now);
    void updateVisionTrackers(
        const CameraStatus& status,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        std::chrono::steady_clock::time_point now);
    void followCurrentPath(
        RobotState& robotState,
        const CameraStatus& status,
        double basePower,
        const std::string& forcedPhase = "",
        const std::string& forcedAction = "",
        bool newCameraSample = false);
    void updateLineLost(
        RobotState& robotState,
        const CameraStatus& status,
        std::chrono::steady_clock::time_point now,
        bool newCameraSample);
    void startCornerAdvance(
        const std::string& direction,
        std::chrono::steady_clock::time_point now);
    void startCornerReverse(std::chrono::steady_clock::time_point now);
    void updateCornerTranslation(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        const CameraStatus& status,
        std::chrono::steady_clock::time_point now,
        bool newCameraSample);
    void startCornerManeuver(
        const std::string& direction,
        std::chrono::steady_clock::time_point now);
    void startGreenManeuver(
        const std::string& action,
        std::chrono::steady_clock::time_point now);
    void updateExecutingTurn(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        const CameraStatus& status,
        std::chrono::steady_clock::time_point now,
        bool newCameraSample);
    void beginReacquiring(
        std::chrono::steady_clock::time_point now,
        bool waitStationary = false);
    void updateReacquiring(
        RobotState& robotState,
        const CameraStatus& status,
        std::chrono::steady_clock::time_point now,
        bool newCameraSample);
    AutonomousStatus makeNavigationStatus(
        const std::string& phase,
        const std::string& action,
        const CameraStatus& status,
        double steeringCorrection = 0.0) const;
    const TrackedEvent* activeNextEvent() const;
    void clearVisionEvents();
    void updateTurnRight90(RobotState& robotState, const Esp32TelemetrySnapshot& esp32Telemetry);
    void updateDriveDistance(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        double targetDistanceCm);
    void resetMissionState();
    void resetLineControl();

    static bool isFresh(const CameraStatus& status);
    static bool isGreenAction(const std::string& action);
    static bool isCornerDirection(const std::string& direction);
    static bool cameraNumbersValid(const CameraStatus& status);
    static const char* navigationStateName(NavigationState state);
    static double angularDistanceDegrees(double first, double second);
    static double getJsonNumber(const std::string& json, const std::string& key, double fallback);
    static bool getJsonBool(const std::string& json, const std::string& key, bool fallback);
    static std::string getJsonString(const std::string& json, const std::string& key, const std::string& fallback);
};
