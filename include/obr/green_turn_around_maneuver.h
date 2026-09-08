#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/imu_turn_controller.h"
#include "obr/robot_state.h"

#include <chrono>

// Mantém o lifecycle completo do retorno de 180 graus disparado pelo verde.
// A manobra conserva autoridade até recuperar a linha ou parar com segurança.
class GreenTurnAroundManeuver
{
public:
    void reset();
    bool active() const;
    bool shouldBlockForwardAssist(
        const CameraLineSnapshot& cameraLineSnapshot) const;
    bool update(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        const CameraLineSnapshot& cameraLineSnapshot);

private:
    enum class Phase
    {
        Idle,
        RecognitionDelay,
        Centering,
        PostCenteringDelay,
        DrivingForward,
        ForwardSettling,
        TurningByImu,
        SearchingLine
    };

    Phase phase_ = Phase::Idle;
    ImuTurnController turnController_;
    bool armed_ = true;
    long long forwardStartLeftCount_ = 0;
    long long forwardStartRightCount_ = 0;
    int lineReacquireFrames_ = 0;
    double lineSearchStartYawDegrees_ = 0.0;
    std::chrono::steady_clock::time_point phaseStartedAt_{};
};
