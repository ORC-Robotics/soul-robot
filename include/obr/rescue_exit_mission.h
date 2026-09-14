#pragma once

#include "obr/camera_monitor.h"
#include "obr/encoder_distance_controller.h"
#include "obr/imu_turn_controller.h"
#include <vector>

struct RescueExitOutput
{
    double leftPower = 0.0;
    double rightPower = 0.0;
    bool completed = false;
    bool failed = false;
    AutonomousStatus status;
};

// Procura a saída pela CAM1 e entrega autoridade ao seguidor somente após a CAM0.
// Retorna potências limitadas; não acessa GPIO e não contorna RobotState.
class RescueExitMission
{
public:
    void reset();
    RescueExitOutput update(const CameraLineSnapshot& bottom,
        const ForwardLineSnapshot& forward, const Esp32TelemetrySnapshot& telemetry,
        std::uint64_t runSequence,
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());

private:
    using Time = std::chrono::steady_clock::time_point;
    enum class Phase { Searching, Turning, Aligning, Approaching, Backing, Completed, Failed };
    struct Rejection { double heading; bool permanent; };
    Phase phase_ = Phase::Searching;
    std::vector<Rejection> rejected_;
    ImuTurnController turn_;
    EncoderDistanceController reverse_;
    bool started_ = false;
    bool sensorsMissing_ = false;
    bool movingForward_ = false;
    bool attempting_ = false;
    bool waitingFrame_ = true;
    bool nearLatched_ = false;
    bool midLatched_ = false;
    Time startedAt_{}, missingSince_{}, observedAt_{}, attemptAt_{}, lastSeenAt_{}, progressAt_{};
    double lastForwardTimestamp_ = 0.0;
    double lastBottomTimestamp_ = 0.0;
    std::uint64_t lastSilverSequence_ = 0;
    std::uint64_t lastForwardSequence_ = 0, lastBottomSequence_ = 0;
    long long lastLeft_ = 0, lastRight_ = 0;
    unsigned long long lastUptime_ = 0;
    double advanceLeftCm_ = 0.0, advanceRightCm_ = 0.0, lastProgressCm_ = 0.0;
    double reverseTargetCm_ = 0.0, reversedCm_ = 0.0;
    double targetHeading_ = 0.0, trackingHeading_ = 0.0;
    double confirmationHeading_ = 0.0, score_ = 0.0;
    int sector_ = -1, candidateFrames_ = 0, observedFrames_ = 0, acquisitionFrames_ = 0;
    int scanSteps_ = 0, round_ = 1;
    std::string failure_, lastPhase_;

    // Cada etapa possui sua decisão visual; update mantém as prioridades de segurança.
    RescueExitOutput updateSearch(const ForwardLineSnapshot& forward,
        const Esp32TelemetrySnapshot& telemetry, Time now, bool newForward);
    RescueExitOutput updateApproach(const ForwardLineSnapshot& forward,
        const Esp32TelemetrySnapshot& telemetry, Time now, bool newForward);
    bool rejected(double heading) const;
    bool startTurn(double degrees, const Esp32TelemetrySnapshot& telemetry, Time now);
    void reject(bool permanent, const char* reason, Time now);
    RescueExitOutput fail(const char* reason);
    RescueExitOutput output(const char* phase, const char* action,
                           double left = 0.0, double right = 0.0);
};
