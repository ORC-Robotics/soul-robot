#pragma once

#include <chrono>
#include <mutex>
#include <string>

enum class AutonomousMission
{
    MainMission,
    TurnRight90
};

// Retorna o identificador estável usado na telemetria e nos comandos do dashboard.
const char* autonomousMissionName(AutonomousMission mission);

// Cópia imutável do estado atual usada por outros módulos sem segurar o mutex.
struct RobotSnapshot
{
    std::string mode = "stopped";
    AutonomousMission autonomousMission = AutonomousMission::MainMission;
    bool emergencyStop = false;
    double left = 0.0;
    double right = 0.0;
};

// Guarda modo, parada de emergência e comandos de motor recebidos do dashboard.
// Esta classe também aplica limites e timeout antes que os motores sejam acionados.
class RobotState
{
public:
    RobotSnapshot snapshot() const;

    void start();
    void startAutonomous();
    void setAutonomousMission(AutonomousMission mission);
    void stop();
    void emergencyStop();
    void drive(double left, double right);
    void driveAutonomous(double left, double right);
    void enforceCommandTimeout(std::chrono::milliseconds timeout);

private:
    mutable std::mutex mutex_;
    RobotSnapshot state_;
    std::chrono::steady_clock::time_point lastCommand_ = std::chrono::steady_clock::now();
};
