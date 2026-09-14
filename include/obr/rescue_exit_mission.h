#pragma once

#include "obr/camera_monitor.h"
#include "obr/encoder_distance_controller.h"
#include "obr/imu_turn_controller.h"
#include <array>
#include <vector>

struct RescueExitOutput
{
    double leftPower = 0.0;
    double rightPower = 0.0;
    bool completed = false;
    bool failed = false;
    AutonomousStatus status;
};

// Procura a saída pela CAM1 e entrega autoridade ao seguidor somente após a CAM0.
// Retorna potências limitadas; não acessa GPIO e não contorna RobotState.
class RescueExitMission
{
public:
    void reset();
    // Registra uma referência aproximada da entrada para ordenar os corners.
    void setKnownEntryHeading(double headingDegrees);
    // Mantém o detector colorido somente enquanto a geometria precisa classificar corners.
    bool requiresRescueZoneDetection() const;
    RescueExitOutput update(const CameraLineSnapshot& bottom,
        const ForwardLineSnapshot& forward, const RescueZoneSnapshot& zones,
        const Esp32TelemetrySnapshot& telemetry,
        std::uint64_t runSequence,
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());

private:
    using Time = std::chrono::steady_clock::time_point;
    enum class Phase {
        GeometrySettling, GeometryTurning, GeometryBacking,
        GeometryReturnTurning, Searching, Turning, ExplorationTurning, Exploring,
        ExplorationBacking, ExplorationSettling, Approaching, Backing,
        BottomValidating,
        CornerRecoveryBacking, CornerRecoveryTurningAway,
        CornerRecoveryAdvancing, CornerRecoveryTurningBack,
        Completed, Failed
    };
    struct Rejection { double heading; double toleranceDegrees; bool permanent; };
    struct ExplorationBin
    {
        bool colorBlocked = false;
        bool grayBlocked = false;
        int sightings = 0;
        double bestScore = 0.0;
        double bestHeading = 0.0;
    };
    Phase phase_ = Phase::Searching;
    std::vector<Rejection> rejected_;
    ImuTurnController turn_;
    EncoderDistanceController reverse_;
    EncoderDistanceController cornerRecoveryDistance_;
    bool started_ = false;
    bool sensorsMissing_ = false;
    bool movingForward_ = false;
    bool attempting_ = false;
    bool waitingFrame_ = true;
    bool nearLatched_ = false;
    bool midLatched_ = false;
    bool bottomMissing_ = false;
    bool retryPending_ = false;
    bool retryUsed_ = false;
    bool guidanceLatched_ = false;
    bool steeringNearLatched_ = false;
    bool explorationStarted_ = false;
    bool finalScan_ = false;
    bool resumeExplorationAfterReview_ = false;
    bool resumeExplorationTurn_ = false;
    bool explorationPositiveFirst_ = true;
    bool movingExploration_ = false;
    bool geometryActive_ = true;
    bool geometryReferenceSaved_ = false;
    bool geometryCandidateActive_ = false;
    bool geometryReturnTurnStarted_ = false;
    bool geometryResumeProbesAfterReturn_ = false;
    bool cornerRecoveryUsed_ = false;
    bool silverBlockLatched_ = false;
    bool knownEntryHeadingValid_ = false;
    Time startedAt_{}, missingSince_{}, bottomMissingSince_{}, observedAt_{}, attemptAt_{},
        lastSeenAt_{}, lastGuidanceAt_{}, progressAt_{}, explorationSettleAt_{},
        geometryPhaseAt_{};
    std::array<ExplorationBin, 12> explorationBins_{};
    double lastForwardTimestamp_ = 0.0;
    double lastBottomTimestamp_ = 0.0;
    std::uint64_t lastForwardSequence_ = 0, lastBottomSequence_ = 0;
    std::uint64_t lastZoneSequence_ = 0;
    long long lastLeft_ = 0, lastRight_ = 0;
    long long bottomValidationStartLeft_ = 0, bottomValidationStartRight_ = 0;
    unsigned long long lastUptime_ = 0;
    double advanceLeftCm_ = 0.0, advanceRightCm_ = 0.0, lastProgressCm_ = 0.0;
    double reverseTargetCm_ = 0.0, reversedCm_ = 0.0;
    double targetHeading_ = 0.0, trackingHeading_ = 0.0;
    double retryHeading_ = 0.0;
    double confirmationHeading_ = 0.0, score_ = 0.0;
    double lastGuidanceAngleDegrees_ = 90.0;
    double explorationBaseHeading_ = 0.0, explorationHeading_ = 0.0;
    double explorationAttemptLeftCm_ = 0.0, explorationAttemptRightCm_ = 0.0;
    double explorationTotalCm_ = 0.0, explorationLastProgressCm_ = 0.0;
    double explorationAttemptTargetCm_ = 0.0;
    double explorationRecoveryTargetCm_ = 0.0;
    double geometryReferenceHeading_ = 0.0;
    double knownEntryHeadingDegrees_ = 0.0;
    double cornerRecoveryReturnHeading_ = 0.0;
    int cornerRecoveryTurnSign_ = 1;
    std::array<double, 2> geometryCandidateHeadings_{};
    std::string explorationBlockReason_;
    int sector_ = -1, candidateFrames_ = 0, observedFrames_ = 0, acquisitionFrames_ = 0;
    int scanSteps_ = 0, round_ = 1;
    int explorationAttempt_ = 0, explorationOffsetIndex_ = 0;
    int geometryProbeIndex_ = 0, geometryCandidateIndex_ = 0;
    int geometryEmptyFrames_ = 0;
    std::string failure_, lastPhase_;

    // Cada etapa possui sua decisão visual; update mantém as prioridades de segurança.
    RescueExitOutput updateSearch(const ForwardLineSnapshot& forward,
        const Esp32TelemetrySnapshot& telemetry, Time now, bool newForward);
    RescueExitOutput updateApproach(const ForwardLineSnapshot& forward,
        const Esp32TelemetrySnapshot& telemetry, Time now, bool newForward);
    RescueExitOutput updateExploration(const CameraLineSnapshot& bottom,
        const ForwardLineSnapshot& forward, const Esp32TelemetrySnapshot& telemetry,
        Time now, bool newForward, bool newBottom);
    RescueExitOutput updateGeometry(const RescueZoneSnapshot& zones,
        const Esp32TelemetrySnapshot& telemetry, Time now);
    bool buildGeometryCandidates(double greenHeading);
    bool startNextGeometryProbe(const Esp32TelemetrySnapshot& telemetry, Time now);
    bool startGeometryCandidate(const Esp32TelemetrySnapshot& telemetry, Time now);
    void startGeometryReturn(const char* reason, bool permanent,
        const Esp32TelemetrySnapshot& telemetry, Time now);
    bool startCornerCollisionRecovery(
        const Esp32TelemetrySnapshot& telemetry, Time now);
    RescueExitOutput restartGeometry(
        const Esp32TelemetrySnapshot& telemetry, Time now, const char* reason);
    void recordExplorationEvidence(const ForwardLineSnapshot& forward,
        const Esp32TelemetrySnapshot& telemetry);
    bool startExploration(const Esp32TelemetrySnapshot& telemetry, Time now);
    bool startNextExplorationTurn(const Esp32TelemetrySnapshot& telemetry, Time now);
    void startExplorationAdvance(Time now);
    void startExplorationRecovery(const char* reason, Time now,
        bool permanentlyRejectHeading = false);
    int explorationBin(double heading) const;
    bool rejected(double heading) const;
    bool startTurn(double degrees, const Esp32TelemetrySnapshot& telemetry, Time now);
    void reject(bool permanent, const char* reason,
        const Esp32TelemetrySnapshot& telemetry, Time now,
        bool retrySame = false);
    RescueExitOutput fail(const char* reason);
    RescueExitOutput output(const char* phase, const char* action,
                           double left = 0.0, double right = 0.0);
};
