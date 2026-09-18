#pragma once

#include "obr/camera_monitor.h"
#include "obr/encoder_distance_controller.h"
#include "obr/imu_turn_controller.h"
#include "obr/robot_state.h"

struct RescueExitOutput
{
    double leftPower = 0.0;
    double rightPower = 0.0;
    bool completed = false;
    bool failed = false;
    AutonomousStatus status;
};

// Executa a saída fixa, com entrada opcional quando o resgate já terminou,
// e entrega a linha à CAM0 após a confirmação visual.
// Não acessa GPIO; as potências continuam sujeitas às proteções de RobotState.
class RescueExitMission
{
public:
    void reset();
    // Usa a entrada curta antes do giro para a saída quando o resgate já terminou.
    void startCompletedRescueRoute();
    // Define o yaw de entrada, em graus; valores inválidos impedem a saída normal.
    void setReferenceHeading(double headingDegrees);
    bool requiresRescueZoneDetection() const;
    RescueExitOutput update(const CameraLineSnapshot& bottom,
        const ForwardLineSnapshot& forward, const RescueZoneSnapshot& zones,
        const Esp32TelemetrySnapshot& telemetry, std::uint64_t runSequence,
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());

private:
    using Time = std::chrono::steady_clock::time_point;
    enum class Phase { EntryAdvance, Preparing, Turning, Straight, FrontGuidance, Completed, Failed };
    Phase phase_ = Phase::Preparing;
    EncoderDistanceController entryDistance_;
    ImuTurnController turn_;
    bool completedRescueRoute_ = false;
    bool started_ = false;
    bool referenceValid_ = false;
    bool movingForward_ = false;
    bool sensorsMissing_ = false;
    bool cameraMissing_ = false;
    double referenceHeadingDegrees_ = 0.0;
    double targetHeadingDegrees_ = 0.0;
    double leftDistanceCm_ = 0.0, rightDistanceCm_ = 0.0;
    double lastProgressCm_ = 0.0;
    double fallbackStartCm_ = 0.0;
    double fallbackAdvanceCm_ = 0.0;
    std::string guidanceState_, bottomBlocker_;
    long long lastLeftCount_ = 0, lastRightCount_ = 0;
    unsigned long long lastUptimeMs_ = 0;
    std::uint64_t lastBottomSequence_ = 0;
    double lastBottomTimestamp_ = 0.0;
    int acquisitionFrames_ = 0;
    int selectedSector_ = -1;
    double selectedConfidence_ = 0.0;
    Time startedAt_{}, progressAt_{}, sensorsMissingSince_{}, cameraMissingSince_{};
    std::string failure_, lastPhase_;

    // Inicia a medida linear somente depois que o giro terminou.
    void startStraight(const Esp32TelemetrySnapshot& telemetry, Time now);
    RescueExitOutput fail(const char* reason);
    RescueExitOutput output(const char* phase, const std::string& action,
                           double left = 0.0, double right = 0.0);
};
