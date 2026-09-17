#pragma once

#include "obr/camera_monitor.h"
#include "obr/encoder_distance_controller.h"
#include "obr/esp32_bridge.h"
#include "obr/green_turn_around_maneuver.h"
#include "obr/robot_state.h"

#include <chrono>

// Coordena a confirmação dos verdes e as curvas laterais medidas.
// O retorno de 180° continua delegado à rotina específica já existente.
class GreenManeuver
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
        Confirming,
        TurnAround,
        DrivingForward,
        SearchingLine,
        Centering,
        Reversing,
        Failed
    };

    Phase phase_ = Phase::Idle;
    GreenTurnAroundManeuver turnAround_;
    EncoderDistanceController distanceController_;
    GreenInterpretation lateralDecision_ = GreenInterpretation::None;
    bool lateralDecisionConfirmed_ = false;
    bool armed_ = true;
    int clearFrames_ = 0;
    long long confirmationStartLeftCount_ = 0;
    long long confirmationStartRightCount_ = 0;
    bool confirmationEncodersStarted_ = false;
    bool confirmationMovementStopped_ = false;
    bool confirmationExtraWaitGranted_ = false;
    // Preserva o preto válido somente no evento atual, sem confirmar um lado.
    bool confirmationValidBlackSeen_ = false;
    std::uint64_t confirmationEventSequence_ = 0;
    std::uint64_t lastConfirmationTraceSequence_ = 0;
    bool confirmationTraceStarted_ = false;
    // O prazo visual começa após a frenagem; a imagem inicial não descarta o evento.
    std::chrono::steady_clock::time_point confirmationMovementStartedAt_{};
    std::uint64_t confirmationMovementStartLineSequence_ = 0;
    std::chrono::steady_clock::time_point confirmationStoppedAt_{};
    double turnStartYawDegrees_ = 0.0;
    // Referência congelada antes dos avanços; nunca define o rumo final.
    bool entryHeadingReferenceValid_ = false;
    double entryHeadingDegrees_ = 0.0;
    double entryYawDegrees_ = 0.0;
    int earlyBranchStableFrames_ = 0;
    bool earlyBranchRejectionLogged_ = false;
    std::uint64_t lastSearchLineSequence_ = 0;
    double straightStartYawDegrees_ = 0.0;
    bool straightYawReferenceValid_ = false;
    int centeredFrames_ = 0;
    std::chrono::steady_clock::time_point phaseStartedAt_{};

    // Inicia a ré apenas depois da faixa visual encontrada ou centralizada.
    void startReverse(std::chrono::steady_clock::time_point now);
    // Registra evidências e saídas por evento, sem repetir o snapshot no loop rápido.
    void logConfirmation(const CameraLineSnapshot& line,
        const Esp32TelemetrySnapshot& telemetry, const char* reason, const char* nextState) const;
};
