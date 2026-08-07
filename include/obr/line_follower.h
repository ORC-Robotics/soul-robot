#pragma once

#include "obr/esp32_bridge.h"
#include "obr/robot_state.h"

#include <chrono>
#include <string>

// Controlador autônomo simples que lê a visão da câmera e gera comandos seguros.
// Ele não acessa GPIO diretamente; os motores continuam passando pelo RobotState.
class LineFollower
{
public:
    void update(RobotState& robotState, const Esp32TelemetrySnapshot& esp32Telemetry);
    bool cameraReady() const;

private:
    enum class Phase
    {
        Following,
        ApproachingGreen,
        TurningGreen
    };

    struct CameraStatus
    {
        bool valid = false;
        bool active = false;
        bool lineDetected = false;
        double fps = 0.0;
        double lineError = 0.0;
        double timestampSeconds = 0.0;
        std::string greenAction = "NENHUM";
    };

    Phase phase_ = Phase::Following;
    std::string activeGreenAction_ = "NENHUM";
    std::chrono::steady_clock::time_point phaseUntil_{};
    std::chrono::steady_clock::time_point greenCooldownUntil_{};
    double lastLineError_ = 0.0;
    bool turn90Active_ = false;
    double turn90StartYawDegrees_ = 0.0;
    std::chrono::steady_clock::time_point turn90StartedAt_{};

    CameraStatus readCameraStatus() const;
    void followLine(RobotState& robotState, const CameraStatus& status);
    void startGreenManeuver(const std::string& action, std::chrono::steady_clock::time_point now);
    void updateGreenManeuver(RobotState& robotState, std::chrono::steady_clock::time_point now);
    void updateTurnRight90(RobotState& robotState, const Esp32TelemetrySnapshot& esp32Telemetry);
    void resetMissionState();

    static bool isFresh(const CameraStatus& status);
    static bool isGreenAction(const std::string& action);
    static double angularDistanceDegrees(double first, double second);
    static double getJsonNumber(const std::string& json, const std::string& key, double fallback);
    static bool getJsonBool(const std::string& json, const std::string& key, bool fallback);
    static std::string getJsonString(const std::string& json, const std::string& key, const std::string& fallback);
};
