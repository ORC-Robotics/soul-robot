#pragma once

#include "obr/camera_monitor.h"
#include "obr/motor_controller.h"
#include "obr/robot_state.h"

#include <chrono>
#include <cstdint>
#include <fstream>
#include <string>

// Correlaciona visão, missão e motores somente durante a auditoria acionada.
// A classe nunca altera comandos, estados da missão ou o IPC normal da câmera.
class LineRegressionTrace
{
public:
    void update(
        const CameraLineSnapshot& cameraLineSnapshot,
        const RobotSnapshot& robotSnapshot,
        const MotorSynchronizationSnapshot& motorSynchronization);

private:
    struct VisionSample
    {
        std::uint64_t lineSequence = 0;
        double frameTimestamp = 0.0;
        double captureHz = 0.0;
        double processingHz = 0.0;
        double ipcHz = 0.0;
        double mjpegHz = 0.0;
        double captureMs = 0.0;
        double lineDetectionMs = 0.0;
        double greenMaskMs = 0.0;
        double greenContoursMs = 0.0;
        double topologyMs = 0.0;
        double greenProcessingMs = 0.0;
        double overlayMs = 0.0;
        double mjpegMs = 0.0;
        double ipcMs = 0.0;
        double totalVisionMs = 0.0;
        std::string greenRaw;
        bool greenConfirmed = false;
    };

    bool active_ = false;
    std::ofstream output_;
    std::chrono::steady_clock::time_point startedAt_{};
    std::chrono::steady_clock::time_point lastControlLoopAt_{};
    std::uint64_t lastRecordedLineSequence_ = 0;
    bool hasRecordedLineSequence_ = false;
    int sampleCount_ = 0;
    std::string previousState_;
    std::string turningCandidateDirection_ = "unknown";
    int turningEntryCount_ = 0;

    void start(std::chrono::steady_clock::time_point now);
    void finish();
    static bool requestExists();
    static bool readVisionSample(VisionSample& sample);
    static std::string missionStateName(const std::string& phase);
    static std::string csvText(const std::string& value);
};
