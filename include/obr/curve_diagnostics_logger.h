#pragma once

#include "obr/camera_monitor.h"
#include "obr/motor_controller.h"
#include "obr/robot_state.h"

#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Registra percepção e comando final de motor sem escrever no loop de controle.
// A thread auxiliar agrupa os frames e descarrega o CSV periodicamente.
class CurveDiagnosticsLogger
{
public:
    explicit CurveDiagnosticsLogger(const std::string& outputPath);
    ~CurveDiagnosticsLogger();

    CurveDiagnosticsLogger(const CurveDiagnosticsLogger&) = delete;
    CurveDiagnosticsLogger& operator=(const CurveDiagnosticsLogger&) = delete;

    void record(
        const CameraLineSnapshot& cameraLineSnapshot,
        const RobotSnapshot& robotSnapshot,
        const MotorSynchronizationSnapshot& finalMotorCommand);

private:
    static bool isRelevantLineFrame(
        const CameraLineSnapshot& cameraLineSnapshot,
        const RobotSnapshot& robotSnapshot);
    static std::string buildCsvRow(
        const CameraLineSnapshot& cameraLineSnapshot,
        const MotorSynchronizationSnapshot& finalMotorCommand);
    void workerLoop();

    std::ofstream output_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::vector<std::string> pendingRows_;
    std::thread worker_;
    bool enabled_ = false;
    bool stopRequested_ = false;
    bool writeFailureReported_ = false;
    bool hasLastFrame_ = false;
    std::uint64_t lastFrameId_ = 0;
};
