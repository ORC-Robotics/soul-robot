#pragma once

#include "obr/ball_alignment_mission.h"
#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/imu_turn_controller.h"
#include "obr/rescue_area_mission.h"
#include "obr/rescue_zone_triangle_mission.h"
#include "obr/robot_state.h"
#include "obr/servo_routine.h"

#include <chrono>
#include <cstdint>
#include <string>

struct RescueRoomOutput
{
    double leftPower = 0.0;
    double rightPower = 0.0;
    bool servoPoseRequested = false;
    bool releaseGripper = false;
    bool internalObjectStored = false;
    ServoPose servoPose;
    bool completed = false;
    bool failed = false;
    AutonomousStatus status;
};

// Coordena a sequência completa da sala: busca, coleta, recuo, armazenamento
// e entrega. Os módulos validados continuam responsáveis por visão, servos e
// aproximação dos triângulos; esta classe apenas define a ordem das etapas.
class RescueRoomMission
{
public:
    RescueRoomOutput update(
        const ForwardBallSnapshot& ball,
        const RescueZoneSnapshot& zones,
        const Esp32TelemetrySnapshot& telemetry,
        std::uint64_t autonomousRunSequence,
        unsigned long long servoConfirmationSequence,
        const ServoPose& currentServoPose,
        std::chrono::steady_clock::time_point now =
            std::chrono::steady_clock::now());
    void reset();

    bool requiresBallDetection() const;
    bool requiresRescueZoneDetection() const;
    std::uint64_t ballTargetSequence(
        std::uint64_t autonomousRunSequence) const;
    const char* ballTargetType() const;

private:
    enum class VictimType
    {
        Alive,
        Dead
    };

    enum class Phase
    {
        EntryAdvance,
        SearchVictim,
        AlignVictim,
        PrepareCapture,
        ApproachVictim,
        SecureCapture,
        ReverseAfterCollection,
        StoreFirstAlive,
        FindDepositZone,
        DepositVictims,
        ReverseAfterDeposit,
        Completed,
        Failed
    };

    enum class SweepStep
    {
        Right45,
        Left45,
        Right75,
        Left75,
        Finished
    };

    enum class DistancePhase
    {
        Idle,
        Preparing,
        Driving,
        Settling,
        Completed,
        Failed
    };

    struct DistanceMove
    {
        DistancePhase phase = DistancePhase::Idle;
        double targetCm = 0.0;
        double power = 0.0;
        int directionSign = 1;
        long long startLeftCount = 0;
        long long startRightCount = 0;
        long long lastUptimeMs = 0;
        double lastProgressCounts = 0.0;
        int differenceSamples = 0;
        std::chrono::steady_clock::time_point phaseStartedAt{};
        std::chrono::steady_clock::time_point lastProgressAt{};
        AutonomousStatus failureStatus;
    };

    Phase phase_ = Phase::EntryAdvance;
    VictimType desiredVictimType_ = VictimType::Alive;
    VictimType carriedVictimType_ = VictimType::Alive;
    SweepStep sweepStep_ = SweepStep::Right45;
    bool sweepTurnStarted_ = false;
    bool waitingForSweepFrame_ = false;
    bool finalVerification_ = false;
    bool storedAliveVictim_ = false;
    bool collectionRetentionActive_ = false;
    bool servoMotionStarted_ = false;
    bool servoOutputsConfirmed_ = false;
    int collectedAliveVictims_ = 0;
    int deliveredAliveVictims_ = 0;
    int deliveredDeadVictims_ = 0;
    std::uint64_t ballTargetGeneration_ = 0;
    double sweepFrameTimestamp_ = 0.0;
    double postDepositReverseDistanceCm_ = 0.0;
    ServoPose collectionRetentionPose_{};
    ServoPose pendingServoPose_{};
    std::chrono::steady_clock::time_point servoEnableDeadline_{};
    ServoRoutineKind depositRoutineKind_ = ServoRoutineKind::Deposit;
    ImuTurnController sweepTurnController_;
    BallAlignmentMission initialVictimAlignmentMission_;
    RescueAreaMission victimApproachMission_;
    RescueZoneTriangleMission triangleMission_;
    ServoRoutine servoRoutine_;
    DistanceMove distanceMove_;
    AutonomousStatus failureStatus_;

    RescueRoomOutput updateSearch(
        const ForwardBallSnapshot& ball,
        const Esp32TelemetrySnapshot& telemetry,
        std::uint64_t expectedTargetSequence);
    RescueRoomOutput updateServo(
        ServoRoutineKind kind,
        const Esp32TelemetrySnapshot& telemetry,
        std::uint64_t autonomousRunSequence,
        unsigned long long confirmationSequence,
        const ServoPose& currentPose,
        std::chrono::steady_clock::time_point now);
    RescueRoomOutput updateDistance(
        const Esp32TelemetrySnapshot& telemetry,
        std::chrono::steady_clock::time_point now,
        const char* phase,
        const char* action);
    void startDistance(
        double targetCm,
        double power,
        int directionSign,
        std::chrono::steady_clock::time_point now);
    void startVictimSearch(VictimType type, bool finalVerification);
    void applyCollectionRetention(RescueRoomOutput& output) const;
    void fail(const AutonomousStatus& status);
    static bool matchesVictim(
        const ForwardBallSnapshot& ball,
        VictimType type,
        std::uint64_t expectedTargetSequence);
};
