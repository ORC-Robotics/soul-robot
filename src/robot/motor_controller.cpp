#include "obr/motor_controller.h"

#include "obr/config.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace
{
// Tolera apenas a diferença numérica residual de um comando realmente reto.
// Diferenças maiores representam direção intencional e não recebem sincronismo.
constexpr double kStraightCommandTolerance = 0.001;
}

MotorController::MotorController(Esp32Bridge& esp32)
    : esp32_(esp32)
{
}

bool MotorController::begin()
{
    const bool ok = esp32_.begin();
    stop();

    if (!ok)
    {
        std::cerr << "ESP32 motor bridge unavailable. Motors will stay stopped until UART works.\n";
    }

    return ok;
}

void MotorController::apply(const RobotSnapshot& state)
{
    if (state.emergencyStop || state.mode == "emergency")
    {
        // A parada de emergência também é enviada para a ESP32.
        // Mesmo que o dashboard continue mandando comandos, a ESP32 recebe zero nos motores.
        suspendEncoderSynchronization(0.0, 0.0);
        esp32_.sendEmergencyStop();
        return;
    }

    if (state.mode == "stopped")
    {
        stop();
        return;
    }

    double leftPower = safeMotorPower(state.left);
    double rightPower = safeMotorPower(state.right);

    if (state.rawMotorCommand)
    {
        // O ajuste individual não recebe sincronismo, mas ainda respeita o piso
        // mecânico de 0,69. Abaixo dele um dos motores não consegue girar.
        leftPower = operationalMotorPower(leftPower);
        rightPower = operationalMotorPower(rightPower);
        suspendEncoderSynchronization(leftPower, rightPower);
        esp32_.sendMotorCommand(leftPower, rightPower, false);
        return;
    }

    // O piso operacional é aplicado antes do sincronismo. A malha reduz somente
    // o lado mais eficiente e recebe outro piso depois da correção, garantindo
    // pelo menos 0,69 em qualquer saída de movimento não nula.
    leftPower = operationalMotorPower(leftPower);
    rightPower = operationalMotorPower(rightPower);

    const bool straightForwardCommand =
        leftPower > 0.0 && rightPower > 0.0 &&
        std::abs(leftPower - rightPower) <= kStraightCommandTolerance;
    if (straightForwardCommand)
    {
        applyEncoderSynchronization(
            leftPower, rightPower, esp32_.telemetrySnapshot());
    }
    else
    {
        // Comandos diferenciais, ré, giros, lados isolados e zero preservam
        // a diferença intencional depois do piso operacional.
        suspendEncoderSynchronization(leftPower, rightPower);
    }

    esp32_.sendMotorCommand(leftPower, rightPower, false);
}

void MotorController::stop()
{
    // O STOP mantém a ESP32 sem movimento e não libera a parada de emergência do RobotState.
    suspendEncoderSynchronization(0.0, 0.0);
    esp32_.sendStop();
}

MotorSynchronizationSnapshot MotorController::synchronizationSnapshot() const
{
    std::lock_guard<std::mutex> lock(synchronizationMutex_);
    return synchronization_;
}

double MotorController::safeMotorPower(double command)
{
    if (!std::isfinite(command))
    {
        return 0.0;
    }

    return std::clamp(command, config::kMinMotorOutput, config::kMaxMotorOutput);
}

double MotorController::operationalMotorPower(double command)
{
    const double safeCommand = safeMotorPower(command);
    if (std::abs(safeCommand) < 0.000001)
    {
        // Zero permanece zero para que parada, timeout e E-Stop nunca acionem
        // o piso operacional de potência.
        return 0.0;
    }

    const double referenceMagnitude = std::clamp(
        std::abs(safeCommand),
        config::kOperationalMinimumMotorPower,
        config::kOperationalMaximumReferencePower);
    return std::copysign(referenceMagnitude, safeCommand);
}

double MotorController::moveToward(
    double current, double target, double maximumStep)
{
    const double difference = target - current;
    return current + std::clamp(difference, -maximumStep, maximumStep);
}

void MotorController::applyEncoderSynchronization(
    double& leftPower,
    double& rightPower,
    const Esp32TelemetrySnapshot& telemetry)
{
    const int direction = leftPower > 0.0 ? 1 : -1;
    if (measurementDirection_ != direction)
    {
        resetEncoderMeasurement(direction);
    }

    double& learnedLeftScale = direction > 0
                                   ? forwardLeftScale_
                                   : reverseLeftScale_;
    double& learnedRightScale = direction > 0
                                    ? forwardRightScale_
                                    : reverseRightScale_;

    const double requestedLeftMagnitude = std::abs(leftPower);
    const double requestedRightMagnitude = std::abs(rightPower);
    const double minimumLeftScale = std::clamp(
        config::kOperationalMinimumMotorPower / requestedLeftMagnitude,
        config::kEncoderSyncMinimumScale,
        1.0);
    const double minimumRightScale = std::clamp(
        config::kOperationalMinimumMotorPower / requestedRightMagnitude,
        config::kEncoderSyncMinimumScale,
        1.0);

    // Uma escala aprendida em velocidade maior não pode derrubar uma nova
    // referência baixa para menos de 0,69 ao começar outro deslocamento.
    learnedLeftScale = std::max(learnedLeftScale, minimumLeftScale);
    learnedRightScale = std::max(learnedRightScale, minimumRightScale);

    const double appliedLeftMagnitude = std::abs(telemetry.appliedLeftPower);
    const double appliedRightMagnitude = std::abs(telemetry.appliedRightPower);
    const double leftRateMagnitude = std::abs(telemetry.leftEncoderRate);
    const double rightRateMagnitude = std::abs(telemetry.rightEncoderRate);
    const bool telemetryNumbersValid =
        std::isfinite(telemetry.appliedLeftPower) &&
        std::isfinite(telemetry.appliedRightPower) &&
        std::isfinite(telemetry.leftEncoderRate) &&
        std::isfinite(telemetry.rightEncoderRate);
    const bool appliedDirectionMatches =
        telemetry.appliedLeftPower * direction > 0.0 &&
        telemetry.appliedRightPower * direction > 0.0;
    const bool encoderDataValid =
        telemetry.sensorFresh && telemetry.lastSensorAgeMs >= 0 &&
        telemetry.lastSensorAgeMs <= config::kEncoderSyncTelemetryMaxAgeMs &&
        telemetryNumbersValid && appliedDirectionMatches &&
        appliedLeftMagnitude >= config::kEncoderSyncMinimumAppliedPower &&
        appliedRightMagnitude >= config::kEncoderSyncMinimumAppliedPower &&
        leftRateMagnitude >= config::kEncoderSyncMinimumRateCountsPerSecond &&
        rightRateMagnitude >= config::kEncoderSyncMinimumRateCountsPerSecond;

    const bool newEncoderSample =
        telemetry.esp32UptimeMs != lastEncoderSampleUptimeMs_;
    if (newEncoderSample)
    {
        lastEncoderSampleUptimeMs_ = telemetry.esp32UptimeMs;
    }

    if (encoderDataValid && newEncoderSample)
    {
        const double leftEfficiency =
            leftRateMagnitude / appliedLeftMagnitude;
        const double rightEfficiency =
            rightRateMagnitude / appliedRightMagnitude;
        if (!efficiencyFilterInitialized_)
        {
            filteredLeftEfficiency_ = leftEfficiency;
            filteredRightEfficiency_ = rightEfficiency;
            efficiencyFilterInitialized_ = true;
        }
        else
        {
            filteredLeftEfficiency_ += config::kEncoderSyncEfficiencyFilterAlpha *
                                       (leftEfficiency - filteredLeftEfficiency_);
            filteredRightEfficiency_ += config::kEncoderSyncEfficiencyFilterAlpha *
                                        (rightEfficiency - filteredRightEfficiency_);
        }

        ++validEncoderSamples_;
        if (validEncoderSamples_ >= config::kEncoderSyncWarmupSamples)
        {
            double targetLeftScale = 1.0;
            double targetRightScale = 1.0;
            const double fasterEfficiency = std::max(
                filteredLeftEfficiency_, filteredRightEfficiency_);
            const double efficiencyDifferenceRatio = fasterEfficiency > 0.0
                                                         ? std::abs(
                                                               filteredLeftEfficiency_ -
                                                               filteredRightEfficiency_) /
                                                               fasterEfficiency
                                                         : 0.0;
            if (efficiencyDifferenceRatio >
                config::kEncoderSyncEfficiencyDeadbandRatio)
            {
                if (filteredLeftEfficiency_ > filteredRightEfficiency_)
                {
                    targetLeftScale = std::clamp(
                        filteredRightEfficiency_ / filteredLeftEfficiency_,
                        minimumLeftScale,
                        1.0);
                }
                else
                {
                    targetRightScale = std::clamp(
                        filteredLeftEfficiency_ / filteredRightEfficiency_,
                        minimumRightScale,
                        1.0);
                }
            }

            // A escala muda somente quando chega uma amostra nova da ESP32.
            // Isso evita aplicar a mesma medição cinco vezes no loop de 20 ms.
            learnedLeftScale = moveToward(
                learnedLeftScale,
                targetLeftScale,
                config::kEncoderSyncMaximumScaleStepPerSample);
            learnedRightScale = moveToward(
                learnedRightScale,
                targetRightScale,
                config::kEncoderSyncMaximumScaleStepPerSample);
        }
    }

    // Esta segunda aplicação do perfil é uma defesa final contra arredondamento
    // e estados aprendidos antigos: saída não nula nunca fica abaixo de 0,69.
    leftPower = operationalMotorPower(leftPower * learnedLeftScale);
    rightPower = operationalMotorPower(rightPower * learnedRightScale);
    publishSynchronization(
        true,
        encoderDataValid &&
            validEncoderSamples_ >= config::kEncoderSyncWarmupSamples,
        encoderDataValid,
        direction,
        learnedLeftScale,
        learnedRightScale,
        leftPower,
        rightPower);
}

void MotorController::suspendEncoderSynchronization(
    double leftPower, double rightPower)
{
    resetEncoderMeasurement(0);
    publishSynchronization(
        false, false, false, 0, 1.0, 1.0, leftPower, rightPower);
}

void MotorController::resetEncoderMeasurement(int direction)
{
    measurementDirection_ = direction;
    validEncoderSamples_ = 0;
    lastEncoderSampleUptimeMs_ = -1;
    efficiencyFilterInitialized_ = false;
    filteredLeftEfficiency_ = 0.0;
    filteredRightEfficiency_ = 0.0;
}

void MotorController::publishSynchronization(
    bool eligible,
    bool active,
    bool encoderDataValid,
    int direction,
    double leftScale,
    double rightScale,
    double correctedLeftPower,
    double correctedRightPower)
{
    std::lock_guard<std::mutex> lock(synchronizationMutex_);
    synchronization_.eligible = eligible;
    synchronization_.active = active;
    synchronization_.encoderDataValid = encoderDataValid;
    synchronization_.correctionApplied =
        leftScale < 0.999 || rightScale < 0.999;
    synchronization_.direction = direction;
    synchronization_.validSamples = validEncoderSamples_;
    synchronization_.leftScale = leftScale;
    synchronization_.rightScale = rightScale;
    synchronization_.filteredLeftEfficiency = filteredLeftEfficiency_;
    synchronization_.filteredRightEfficiency = filteredRightEfficiency_;
    synchronization_.correctedLeftPower = correctedLeftPower;
    synchronization_.correctedRightPower = correctedRightPower;
}
