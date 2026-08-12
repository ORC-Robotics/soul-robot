#pragma once

#include "obr/esp32_bridge.h"
#include "obr/robot_state.h"

#include <chrono>
#include <string>

// Lê o resultado da visão, incluindo os indicadores das regiões superiores,
// e converte somente o erro da ROI inferior em comandos seguros pelo RobotState.
class LineFollower
{
public:
    void reset();
    void update(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry);

private:
    enum class Phase
    {
        Following,
        AdvancingToTurn,
        SettlingBeforeTurn,
        TurningToLine
    };

    enum class TurnDirection
    {
        None,
        Left,
        Right
    };

    struct CameraStatus
    {
        bool valid = false;
        bool active = false;
        bool lineDetected = false;
        // Indica uma curva de 90° ou um cruzamento na ROI superior da câmera.
        bool turn90Ahead = false;
        // Usa LEFT, RIGHT, BOTH ou NONE para descrever a expansão horizontal.
        std::string turn90Direction = "NONE";
        // Indica somente a presença de linha na ROI distante entre 25% e 50%.
        bool farLineDetected = false;
        double framesPerSecond = 0.0;
        double lineError = 0.0;
        double timestampSeconds = 0.0;
    };

    Phase phase_ = Phase::Following;
    TurnDirection turnDirection_ = TurnDirection::None;
    long long approachStartLeftCount_ = 0;
    long long approachStartRightCount_ = 0;
    double lastApproachProgressCounts_ = 0.0;
    bool originalLineLost_ = false;
    bool turnTriggerArmed_ = true;
    std::chrono::steady_clock::time_point phaseStartedAt_{};
    std::chrono::steady_clock::time_point lastApproachProgressAt_{};
    std::chrono::steady_clock::time_point turnCooldownUntil_{};

    CameraStatus readCameraStatus() const;
    void startTurnApproach(
        TurnDirection direction,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        std::chrono::steady_clock::time_point now);
    void updateTurnApproach(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        const CameraStatus& cameraStatus,
        std::chrono::steady_clock::time_point now);
    void updateTurningToLine(
        RobotState& robotState,
        const CameraStatus& cameraStatus,
        std::chrono::steady_clock::time_point now);

    static bool isFresh(const CameraStatus& status);
    static bool encodersReady(const Esp32TelemetrySnapshot& esp32Telemetry);
};
