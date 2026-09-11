#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/motor_controller.h"
#include "obr/robot_state.h"

#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

// Registra, fora do loop de controle, o comando que já foi enviado aos motores.
// A classe só observa cópias imutáveis e descarta amostras se a fila estiver cheia.
class ForwardReacquisitionControlRecorder
{
public:
    ForwardReacquisitionControlRecorder(
        std::string controlPath,
        std::string sessionRoot);
    ~ForwardReacquisitionControlRecorder();

    ForwardReacquisitionControlRecorder(
        const ForwardReacquisitionControlRecorder&) = delete;
    ForwardReacquisitionControlRecorder& operator=(
        const ForwardReacquisitionControlRecorder&) = delete;

    void record(
        const RobotSnapshot& robotSnapshot,
        const MotorSynchronizationSnapshot& finalMotorCommand,
        const Esp32TelemetrySnapshot& telemetry,
        const ForwardLineSnapshot& forwardLineSnapshot,
        const std::string& commandSource) noexcept;

private:
    struct Sample
    {
        double timestamp = 0.0;
        RobotSnapshot robot;
        MotorSynchronizationSnapshot motor;
        Esp32TelemetrySnapshot telemetry;
        ForwardLineSnapshot forwardLine;
        std::string commandSource;
        std::string session;
    };

    std::string controlPath_;
    std::string sessionRoot_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<Sample> pendingSamples_;
    std::thread worker_;
    bool workerAvailable_ = false;
    bool stopRequested_ = false;
    bool recordingActive_ = false;
    bool failureReported_ = false;
    std::string activeSession_;
    std::string failedSession_;

    void refreshControl() noexcept;
    void workerLoop() noexcept;
    bool writeSamples(const std::deque<Sample>& samples) noexcept;
    static std::string buildJsonLine(const Sample& sample);
};
