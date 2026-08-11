#pragma once

#include "obr/esp32_bridge.h"
#include "obr/robot_state.h"

#include <mutex>

// Expõe o estado da correção automática sem permitir que o dashboard altere
// comandos de motor. As escalas sempre ficam entre o limite seguro e 1,0.
struct MotorSynchronizationSnapshot
{
    bool eligible = false;
    bool active = false;
    bool encoderDataValid = false;
    bool correctionApplied = false;
    int direction = 0;
    int validSamples = 0;
    double leftScale = 1.0;
    double rightScale = 1.0;
    double filteredLeftEfficiency = 0.0;
    double filteredRightEfficiency = 0.0;
    double correctedLeftPower = 0.0;
    double correctedRightPower = 0.0;
};

// Converte comandos seguros do RobotState em mensagens UART para a ESP32.
// A ESP32 controla os drivers DRV8833 e também aplica timeout próprio de segurança.
class MotorController
{
public:
    explicit MotorController(Esp32Bridge& esp32);

    bool begin();
    void apply(const RobotSnapshot& state);
    void stop();
    MotorSynchronizationSnapshot synchronizationSnapshot() const;

private:
    Esp32Bridge& esp32_;
    mutable std::mutex synchronizationMutex_;
    MotorSynchronizationSnapshot synchronization_;
    int measurementDirection_ = 0;
    int validEncoderSamples_ = 0;
    long long lastEncoderSampleUptimeMs_ = -1;
    bool efficiencyFilterInitialized_ = false;
    double filteredLeftEfficiency_ = 0.0;
    double filteredRightEfficiency_ = 0.0;
    double forwardLeftScale_ = 1.0;
    double forwardRightScale_ = 1.0;
    double reverseLeftScale_ = 1.0;
    double reverseRightScale_ = 1.0;

    static double safeMotorPower(double command);
    static double operationalMotorPower(double command);
    static double moveToward(double current, double target, double maximumStep);
    void applyEncoderSynchronization(
        double& leftPower,
        double& rightPower,
        const Esp32TelemetrySnapshot& telemetry);
    void suspendEncoderSynchronization(double leftPower, double rightPower);
    void resetEncoderMeasurement(int direction);
    void publishSynchronization(
        bool eligible,
        bool active,
        bool encoderDataValid,
        int direction,
        double leftScale,
        double rightScale,
        double correctedLeftPower,
        double correctedRightPower);
};
