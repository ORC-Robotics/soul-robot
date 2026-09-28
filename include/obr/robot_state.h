#pragma once

#include "obr/config.h"
#include "obr/servo_types.h"

#include <chrono>
#include <limits>
#include <mutex>
#include <string>

struct CameraLineSnapshot;

enum class AutonomousMission
{
    MainMission,
    TurnRight90,
    DriveDistance,
    RescueZoneDetection,
    RescueZoneSearch,
    RescueZoneAlign,
    RescueZoneApproach,
    RescueZoneTriangle,
    RescueArea,
    RescueExit,
    RescueExitWithReverse,
    RescueCornerYawTest,
    ObstacleAvoidance,
    ServoInitialize,
    ServoWave,
    ServoCapture,
    ServoInternalStorage,
    ServoDeposit,
    ServoFullSequence,
    ServoFullSequenceTwo
};

enum class RescueZoneTargetColor
{
    Green,
    Red
};

// Retorna o identificador estável usado na telemetria e nos comandos do dashboard.
const char* autonomousMissionName(AutonomousMission mission);
const char* rescueZoneTargetColorName(RescueZoneTargetColor color);

// Descreve a etapa atual da missão para telemetria e diagnóstico no dashboard.
// Estes dados não comandam os motores; apenas refletem a decisão já tomada pelo controle autônomo.
struct AutonomousStatus
{
    std::string phase = "stopped";
    std::string action = "Missão parada";
    double progressPercent = 0.0;
    double targetDistanceCm = 0.0;
    double leftDistanceCm = 0.0;
    double rightDistanceCm = 0.0;
    double averageDistanceCm = 0.0;
    double obstacleYawBase = std::numeric_limits<double>::quiet_NaN();
    double obstacleLeftClearance = std::numeric_limits<double>::quiet_NaN();
    double obstacleRightClearance = std::numeric_limits<double>::quiet_NaN();
    std::string obstacleSelectedSide = "NONE";
    int obstacleWaitSecondsRemaining = -1;
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
    std::string forwardAssistState = "BOTTOM";
    std::string forwardAssistDirection = "NONE";
    std::string forwardAssistLatchedDirection = "NONE";
    std::string forwardAssistEntryBlocker = "WAITING_TRUST";
    double forwardAssistYawDeltaDeg = 0.0;
    bool forwardAssistFarTrusted = false;
    bool forwardAssistMediumTrusted = false;
    bool forwardAssistGapCandidate = false;
    bool forwardAssistEntryAllowed = false;
    bool forwardLineVisible = false;
    double forwardLinePosition = 0.0;
    double forwardPathConfidence = 0.0;
    std::string forwardPathState = "UNCERTAIN";
    std::string gapValidationDecision = "NORMAL";
    std::string nearLineState = "UNKNOWN";
    std::string bottomLineControlSource = "UNAVAILABLE";
    int bottomStableFrames = 0;
    bool servoRoutineWaitingForConfirmation = false;
    bool rescueZoneUltrasonicFresh = false;
    bool rescueZoneUltrasonicValid = false;
    double rescueZoneUltrasonicDistanceCm = 0.0;
    double rescueZoneAlignAimNormalized =
        std::numeric_limits<double>::quiet_NaN();
    std::string rescueZoneAlignState = "UNAVAILABLE";
    double rescueZoneAlignLockedHeading =
        std::numeric_limits<double>::quiet_NaN();
    std::string rescueZoneAlignCompletionReason;
    double rescueZoneApproachLockedHeading =
        std::numeric_limits<double>::quiet_NaN();
    double rescueZoneApproachHeadingError =
        std::numeric_limits<double>::quiet_NaN();
    bool rescueZoneApproachNearLatched = false;
    std::string rescueZoneApproachSpeedState = "STOP";
    std::string rescueZoneApproachCompletionReason;
    std::string rescueZoneSearchTargetColor = "green";
    bool rescueZoneSearchTargetDetected = false;
    std::string rescueZoneSearchState = "SEARCHING";
    std::string rescueZoneSearchCompletionReason;
    std::string rescueZoneTrianglePhase;
    // Diagnóstico da busca e exploração: headings em graus e distâncias em centímetros.
    int exitSector = -1;
    double exitConfidence = 0.0;
    double exitHeadingDegrees = 0.0;
    int exitRound = 1;
    std::string exitRejections;
    std::string exitLastFailure;
    double exitAdvanceCm = 0.0;
    std::string exitGuidanceState;
    std::string exitBottomBlocker;
    int exitBottomFrames = 0;
    double exitFallbackAdvanceCm = 0.0;
    double exitReverseCm = 0.0;
    double exitExplorationHeadingDegrees = 0.0;
    int exitExplorationAttempt = 0;
    double exitExplorationAdvanceCm = 0.0;
    std::string exitExplorationBlockReason;
};

// Cópia imutável do estado atual usada por outros módulos sem segurar o mutex.
struct RobotSnapshot
{
    std::string mode = "stopped";
    AutonomousMission autonomousMission = AutonomousMission::MainMission;
    bool emergencyStop = false;
    // Conclusão da execução atual; somente uma ação explícita libera o estado.
    bool missionFinished = false;
    double redRatio = 0.0;
    bool redValid = false;
    double left = 0.0;
    double right = 0.0;
    bool rawMotorCommand = false;
    // Mantém o sincronismo disponível por padrão. O segue-faixa NORMAL
    // desativa apenas essa etapa, sem contornar os pisos START/RUN.
    bool encoderSynchronizationAllowed = true;
    long long commandAgeMs = 0;
    bool commandTimedOut = false;
    unsigned long long autonomousRunSequence = 0;
    double driveDistanceTargetCm = config::kDriveDistanceDefaultTargetCm;
    RescueZoneTargetColor rescueZoneTargetColor = RescueZoneTargetColor::Green;
    double rescueZoneLockedHeading =
        std::numeric_limits<double>::quiet_NaN();
    ServoPose servoPose;
    bool armServoRequested = false;
    bool wristServoRequested = false;
    bool gripperServoRequested = false;
    unsigned long long servoCommandSequence = 0;
    unsigned long long servoRoutineConfirmationSequence = 0;
    bool servoRoutineInternalObjectStored = false;
    // Solicitação pontual do bônus; não muda a missão selecionada nem libera a tração.
    bool waveBonusRequested = false;
    bool servoCalibrationActive = false;
    // Entregas confirmadas permanecem na RAM após STOP, mas não após reinício.
    int rescueDeliveredAliveVictims = 0;
    int rescueDeliveredDeadVictims = 0;
    bool rescueTestMemoryActive = false;
    AutonomousStatus autonomousStatus;
};

// Guarda modo, parada de emergência e comandos de motor recebidos do dashboard.
// Esta classe também aplica limites e timeout antes que os motores sejam acionados.
class RobotState
{
public:
    RobotSnapshot snapshot() const;

    // Consome evidência nova da CAM0 e retorna true somente ao encerrar a execução.
    bool observeRedFinish(const CameraLineSnapshot& camera);
    void start();
    void startAutonomous();
    bool tryStartAutonomous();
    void setAutonomousMission(AutonomousMission mission);
    bool setRescueTestDeliveries(int alive, int dead);
    void recordRescueDeliveries(int alive, int dead);
    bool setDriveDistanceTargetCm(double targetCm);
    void setRescueZoneTargetColor(RescueZoneTargetColor color);
    bool setRescueZoneLockedHeading(double headingDegrees);
    void stop();
    void emergencyStop();
    void drive(double left, double right);
    void driveRawDiagnostic(double left, double right);
    void driveAutonomous(
        double left,
        double right,
        bool encoderSynchronizationAllowed = true);
    bool setManualServoAngle(ServoId servo, double angleDegrees);
    // A opção wristOnly preserva os alvos e as solicitações de braço e garra.
    bool setAutonomousServoPose(const ServoPose& pose, bool wristOnly = false);
    // Insere o tchauzinho antes de continuar a missão, ou após uma chegada já travada.
    bool requestWaveBonus();
    // A exceção da chegada permite somente a pose do bônus com motores bloqueados.
    bool setWaveBonusServoPose(const ServoPose& pose);
    void completeWaveBonus();
    bool setAutonomousServoOutputEnabled(ServoId servo, bool enabled);
    bool confirmServoRoutineAction();
    void setServoRoutineInternalObjectStored(bool stored);
    void disableServos();
    bool beginServoCalibration();
    void endServoCalibration();
    void updateAutonomousStatus(const AutonomousStatus& status);
    void enforceCommandTimeout(std::chrono::milliseconds timeout);
    void enforceManualServoTimeout(std::chrono::milliseconds timeout);

private:
    mutable std::mutex mutex_;
    RobotSnapshot state_;
    // O rearme visual sobrevive ao START, para permitir sair da mesma faixa.
    bool redFinishArmed_ = true;
    double lastRedTimestamp_ = 0.0;
    std::uint64_t lastRedSequence_ = 0;
    std::chrono::steady_clock::time_point lastCommand_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point lastManualServoCommand_ =
        std::chrono::steady_clock::now();

    void requestInitialServoPoseLocked();
    void disableServosLocked();
    bool applyServoPoseLocked(const ServoPose& pose, bool wristOnly = false);
};
