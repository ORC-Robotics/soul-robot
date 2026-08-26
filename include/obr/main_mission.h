#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/imu_turn_controller.h"
#include "obr/robot_state.h"

#include <chrono>

// Permite que o retorno troque a IMU pela busca visual somente depois do
// progresso angular mínimo configurado. Valores inválidos nunca são aceitos.
bool greenTurnAroundImuProgressAllowsVisualSearch(
    double maximumProgressPercent);

// Executa o segue-linha e comportamentos curtos disparados pela visão verde.
// A classe não acessa hardware diretamente e mantém cada etapa fail-safe.
class MainMission
{
public:
    void reset();
    void update(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        bool cameraReady,
        const CameraLineSnapshot& cameraLineSnapshot);

private:
    enum class TurnAroundPhase
    {
        Idle,
        DrivingForward,
        ForwardSettling,
        TurningByImu,
        SearchingLine
    };

    TurnAroundPhase turnAroundPhase_ = TurnAroundPhase::Idle;
    ImuTurnController turnAroundController_;
    bool turnAroundArmed_ = true;
    long long forwardStartLeftCount_ = 0;
    long long forwardStartRightCount_ = 0;
    int lineReacquireFrames_ = 0;
    double turnAroundMaximumImuProgressPercent_ = 0.0;
    std::chrono::steady_clock::time_point phaseStartedAt_{};
};
