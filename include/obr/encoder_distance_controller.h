#pragma once

#include "obr/esp32_bridge.h"
#include "obr/robot_state.h"
#include <chrono>

struct EncoderDistanceOutput
{
    double leftPower = 0.0;
    double rightPower = 0.0;
    bool completed = false;
    bool failed = false;
    AutonomousStatus status;
};

// Executa um deslocamento por encoders com preparação, frenagem e limites de segurança.
// Não aplica motores: o chamador continua responsável pelo modo e pela emergência.
class EncoderDistanceController
{
public:
    // Desativar a parada por diferença lateral mantém os limites de distância,
    // tempo, progresso e validade dos encoders durante o deslocamento.
    void start(double targetCm, double power, int directionSign,
               std::chrono::steady_clock::time_point now,
               bool stopOnSideMismatch = true);
    EncoderDistanceOutput update(const Esp32TelemetrySnapshot& telemetry,
        std::chrono::steady_clock::time_point now, const char* phase, const char* action);
    void reset() { distanceMove_ = {}; }
    bool idle() const { return distanceMove_.phase == DistancePhase::Idle; }
    static bool encodersReady(const Esp32TelemetrySnapshot& telemetry);
    static bool encodersStopped(const Esp32TelemetrySnapshot& telemetry);

private:
    enum class DistancePhase
    {
        Idle,
        Preparing,
        Driving,
        Settling,
        Completed,
        Failed
    };

    struct DistanceMove
    {
        DistancePhase phase = DistancePhase::Idle;
        double targetCm = 0.0;
        double power = 0.0;
        int directionSign = 1;
        long long startLeftCount = 0;
        long long startRightCount = 0;
        long long lastUptimeMs = 0;
        double lastProgressCounts = 0.0;
        int differenceSamples = 0;
        bool stopOnSideMismatch = true;
        std::chrono::steady_clock::time_point phaseStartedAt{};
        std::chrono::steady_clock::time_point lastProgressAt{};
        AutonomousStatus failureStatus;
    };

    DistanceMove distanceMove_;
};
