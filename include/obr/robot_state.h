#pragma once

#include "obr/config.h"

#include <chrono>
#include <mutex>
#include <string>

enum class AutonomousMission
{
    MainMission,
    TurnRight90,
    DriveDistance
};

// Retorna o identificador estável usado na telemetria e nos comandos do dashboard.
const char* autonomousMissionName(AutonomousMission mission);

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
    int bottomStableFrames = 0;
};

// Cópia imutável do estado atual usada por outros módulos sem segurar o mutex.
struct RobotSnapshot
{
    std::string mode = "stopped";
    AutonomousMission autonomousMission = AutonomousMission::MainMission;
    bool emergencyStop = false;
    double left = 0.0;
    double right = 0.0;
    bool rawMotorCommand = false;
    long long commandAgeMs = 0;
    bool commandTimedOut = false;
    unsigned long long autonomousRunSequence = 0;
    double driveDistanceTargetCm = config::kDriveDistanceDefaultTargetCm;
    AutonomousStatus autonomousStatus;
};

// Guarda modo, parada de emergência e comandos de motor recebidos do dashboard.
// Esta classe também aplica limites e timeout antes que os motores sejam acionados.
class RobotState
{
public:
    RobotSnapshot snapshot() const;

    void start();
    void startAutonomous();
    bool tryStartAutonomous();
    void setAutonomousMission(AutonomousMission mission);
    bool setDriveDistanceTargetCm(double targetCm);
    void stop();
    void emergencyStop();
    void drive(double left, double right);
    void driveRawDiagnostic(double left, double right);
    void driveAutonomous(double left, double right);
    void updateAutonomousStatus(const AutonomousStatus& status);
    void enforceCommandTimeout(std::chrono::milliseconds timeout);

private:
    mutable std::mutex mutex_;
    RobotSnapshot state_;
    std::chrono::steady_clock::time_point lastCommand_ = std::chrono::steady_clock::now();
};
