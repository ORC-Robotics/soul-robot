#pragma once

#include "obr/esp32_bridge.h"

#include <chrono>
#include <string>

enum class ImuTurnDirection
{
    Left = -1,
    Right = 1
};

enum class ImuTurnResult
{
    Idle,
    Running,
    Completed,
    Failed
};

struct ImuTurnOutput
{
    ImuTurnResult result = ImuTurnResult::Idle;
    double leftPower = 0.0;
    double rightPower = 0.0;
    double progressPercent = 0.0;
    std::string phase;
    std::string action;
};

// Controla um giro no próprio eixo usando yaw e velocidade angular do MPU6050.
// A classe não acessa motores diretamente; o chamador preserva E-Stop e modo.
class ImuTurnController
{
public:
    static bool imuReady(const Esp32TelemetrySnapshot& telemetry);
    static double angularDistanceDegrees(double first, double second);

    bool start(
        double targetDegrees,
        ImuTurnDirection direction,
        const Esp32TelemetrySnapshot& telemetry,
        double completionToleranceDegrees = 0.0);
    ImuTurnOutput update(const Esp32TelemetrySnapshot& telemetry);
    void reset();
    bool active() const;

private:
    enum class Phase
    {
        Idle,
        Turning,
        Settling,
        CorrectionPulse
    };

    Phase phase_ = Phase::Idle;
    double targetDegrees_ = 0.0;
    // A margem de conclusão pode ser maior em manobras de orientação visual,
    // mas o controlador padrão continua usando a tolerância precisa de 90°.
    double completionToleranceDegrees_ = 0.0;
    double startYawDegrees_ = 0.0;
    double directionSign_ = 1.0;
    double correctionDirection_ = 1.0;
    int correctionPulseCount_ = 0;
    std::chrono::steady_clock::time_point startedAt_{};
    std::chrono::steady_clock::time_point phaseStartedAt_{};

};
