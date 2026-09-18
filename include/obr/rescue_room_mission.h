#pragma once

#include "obr/ball_alignment_mission.h"
#include "obr/config.h"
#include "obr/encoder_distance_controller.h"
#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/imu_turn_controller.h"
#include "obr/rescue_area_mission.h"
#include "obr/rescue_zone_triangle_mission.h"
#include "obr/robot_state.h"
#include "obr/servo_routine.h"

#include <chrono>
#include <cstdint>
#include <limits>
#include <string>

struct RescueRoomOutput
{
    double leftPower = 0.0;
    double rightPower = 0.0;
    bool servoPoseRequested = false;
    bool releaseGripper = false;
    bool internalObjectStored = false;
    int deliveredAliveVictims = 0;
    int deliveredDeadVictims = 0;
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
    void resumeWithDeliveries(int alive, int dead);

    bool requiresBallDetection() const;
    bool requiresRescueZoneDetection() const;
    // Referência congelada na entrada, antes dos giros de busca e depósito.
    bool entryHeadingValid() const;
    double entryHeadingDegrees() const;
    double lastTriangleHeadingDegrees() const;
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
        LiftAfterCollection,
        StoreFirstAlive,
        FindDepositZone,
        DepositVictims,
        ReverseAfterDeposit,
        Completed,
        Failed
    };

    enum class SweepStep
    {
        First45,
        Opposite45,
        First75,
        Opposite75,
        Finished
    };

    Phase phase_ = Phase::EntryAdvance;
    // O yaw de entrada, em graus, só pode mudar ao iniciar uma nova missão.
    bool entryHeadingValid_ = false;
    double entryHeadingDegrees_ = 0.0;
    VictimType desiredVictimType_ = VictimType::Alive;
    VictimType carriedVictimType_ = VictimType::Alive;
    SweepStep sweepStep_ = SweepStep::First45;
    // O lado observado pertence à vítima atual. O heading fixa os limites da
    // varredura, mesmo quando um giro é interrompido antes de chegar ao destino.
    int candidateSide_ = config::kRescueEntrySearchDirection;
    int sweepFirstSide_ = config::kRescueEntrySearchDirection;
    bool sweepReferenceSet_ = false;
    double sweepReferenceYaw_ = 0.0;
    bool sweepAttemptStarted_ = false;
    int sweepTimeoutCount_ = 0;
    std::chrono::steady_clock::time_point sweepAttemptStartedAt_{};
    bool sweepTurnStarted_ = false;
    bool waitingForSweepFrame_ = false;
    // A alternância angular só é permitida na primeira busca, quando nenhuma
    // vítima foi encontrada durante a entrada na sala.
    bool initialAlternatingSweepAllowed_ = true;
    // Mantém o giro em um único sentido depois da primeira vítima encontrada.
    bool continuousSearchActive_ = false;
    // Detecta quando a busca contínua comanda o giro, mas a IMU não confirma
    // avanço angular suficiente. Nesse caso, o sentido é invertido com segurança.
    bool continuousSearchProgressWatchActive_ = false;
    double continuousSearchProgressYaw_ = 0.0;
    std::chrono::steady_clock::time_point continuousSearchProgressStartedAt_{};
    // Impede que frames intercalados façam a busca ultrapassar uma candidata em confirmação.
    bool candidateConfirmationActive_ = false;
    // Contadores próprios da visão; nunca alteram o yaw de entrada usado na saída.
    bool searchRotationTracked_ = false;
    double searchLastYaw_ = 0.0;
    double searchRotationDegrees_ = 0.0;
    std::chrono::steady_clock::time_point candidateConfirmationStartedAt_{};
    bool searchRepositionActive_ = false;
    bool searchRepositionReversing_ = false;
    ImuTurnController searchRepositionTurnController_;
    std::chrono::steady_clock::time_point candidateLastSeenAt_{};
    bool candidateHeadingValid_ = false;
    double candidateHeadingDegrees_ = 0.0;
    bool finalVerification_ = false;
    // Mede a volta final pela IMU e limita o tempo gasto procurando extras.
    // Ao atingir qualquer limite, a missão para e libera a busca da saída.
    bool finalSearchStarted_ = false;
    double finalSearchLastYaw_ = 0.0;
    double finalSearchAccumulatedDegrees_ = 0.0;
    std::chrono::steady_clock::time_point finalSearchStartedAt_{};
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
    // Registra um único impulso por movimento delicado. Manter este estado na
    // sala impede que o reforço alcance o segue-linha ou a busca da saída.
    bool delicateMotionActive_ = false;
    int delicateMotionLeftDirection_ = 0;
    int delicateMotionRightDirection_ = 0;
    std::chrono::steady_clock::time_point delicateMotionKickDeadline_{};
    ServoRoutineKind depositRoutineKind_ = ServoRoutineKind::Deposit;
    ServoRoutineKind liftRoutineKind_ = ServoRoutineKind::LiftAfterReverse;
    ImuTurnController sweepTurnController_;
    BallAlignmentMission initialVictimAlignmentMission_;
    RescueAreaMission victimApproachMission_;
    RescueZoneTriangleMission triangleMission_;
    double lastTriangleHeadingDegrees_ =
        std::numeric_limits<double>::quiet_NaN();
    ServoRoutine servoRoutine_;
    EncoderDistanceController distanceController_;
    AutonomousStatus failureStatus_;

    RescueRoomOutput updateSearch(
        const ForwardBallSnapshot& ball,
        const Esp32TelemetrySnapshot& telemetry,
        std::uint64_t expectedTargetSequence,
        std::chrono::steady_clock::time_point now);
    RescueRoomOutput updateStep(
        const ForwardBallSnapshot& ball,
        const RescueZoneSnapshot& zones,
        const Esp32TelemetrySnapshot& telemetry,
        std::uint64_t autonomousRunSequence,
        unsigned long long servoConfirmationSequence,
        const ServoPose& currentServoPose,
        std::chrono::steady_clock::time_point now);
    // Reforça apenas o começo de curvas, pivôs e rodas isoladas, preservando o
    // sentido solicitado e retornando automaticamente às potências da missão.
    void applyDelicateMotionKick(
        RescueRoomOutput& output,
        std::chrono::steady_clock::time_point now);
    // Avança a tentativa sem reutilizar o relógio ou o giro interrompido.
    void advanceSweepStep();
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
