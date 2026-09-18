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
        double completionToleranceDegrees = 0.0,
        int correctionPulseMs = 0,
        // Um valor negativo remove apenas o limite de pulsos de correção.
        int maximumCorrectionPulses = 0,
        double commandPower = 0.0,
        // Em milissegundos: zero usa o prazo padrão; negativo desativa o prazo.
        // A validação da IMU e a prioridade do E-Stop continuam obrigatórias.
        int timeoutMs = 0,
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now(),
        bool allowEncoderStopConfirmation = false,
        double stationaryRateDegPerSec = 0.0,
        // Prazo da correção fina: positivo conclui com erro residual ao expirar.
        // Zero preserva a política padrão de falha por limite de pulsos.
        int correctionCompletionTimeoutMs = 0);
    ImuTurnOutput update(const Esp32TelemetrySnapshot& telemetry,
                        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());
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
    int correctionPulseMs_ = 0;
    // Um valor negativo remove o limite de correções; o prazo é independente.
    int maximumCorrectionPulses_ = 0;
    double commandPower_ = 0.0;
    int timeoutMs_ = 0;
    int correctionCompletionTimeoutMs_ = 0;
    std::chrono::steady_clock::time_point correctionStartedAt_{};
    // Esta confirmação auxiliar é habilitada apenas por manobras que aceitam
    // os encoders como prova física de que a inércia terminou.
    bool allowEncoderStopConfirmation_ = false;
    double stationaryRateDegPerSec_ = 0.0;
    std::chrono::steady_clock::time_point startedAt_{};
    std::chrono::steady_clock::time_point phaseStartedAt_{};

};
