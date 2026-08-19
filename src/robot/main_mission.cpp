#include "obr/main_mission.h"

#include "obr/config.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

namespace
{
// Valores usados no alinhamento seguro ao reencontrar a linha.
constexpr double kLineRecoveryBaseSpeed =
    config::kMotorStartMinimumPower;
constexpr double kLineRecoveryDeadzone = 0.10;
constexpr double kLineRecoveryProportionalGain = 0.30;
constexpr double kLineRecoveryMaximumCorrection = 0.15;
constexpr double kLineRecoveryExtremeError = 0.75;

// Um erro deste tamanho registra uma direção útil sem apagar a memória no centro.
constexpr double kSignificantDirectionError = 0.20;
constexpr double kNearReacquireMaxAbsError = 0.40;

// Os limites impedem que o robô procure indefinidamente por uma linha perdida.
constexpr auto kTotalLossTimeout = std::chrono::milliseconds(1300);
// O robô nunca pode continuar avançando indefinidamente sem reencontrar a linha.
constexpr auto kGapTimeout = std::chrono::milliseconds(2000);
constexpr int kNearSamplesToConfirmRecovery = 3;

double angularDistanceDegrees(double firstDegrees, double secondDegrees)
{
    double difference = std::fmod(
        std::abs(secondDegrees - firstDegrees), 360.0);
    if (difference > 180.0)
    {
        difference = 360.0 - difference;
    }
    return difference;
}

struct MotorCommand
{
    double left = 0.0;
    double right = 0.0;
};

AutonomousStatus makeLineStatus(
    const std::string& phase,
    const std::string& action)
{
    AutonomousStatus status;
    status.phase = phase;
    status.action = action;
    return status;
}

MotorCommand calculateOneWheelPivotCommand(double error)
{
    return error < 0.0
               ? MotorCommand{0.0, kLineRecoveryBaseSpeed}
               : MotorCommand{kLineRecoveryBaseSpeed, 0.0};
}

MotorCommand calculateCounterRotationCommand(bool turnLeft, double power)
{
    return turnLeft
               ? MotorCommand{-power, power}
               : MotorCommand{power, -power};
}

bool isDirectionalGreenTurn(GreenTurnDecision decision)
{
    return decision == GreenTurnDecision::GuideLeft ||
           decision == GreenTurnDecision::GuideRight;
}

bool isStrongGreenCandidate(const CameraLineSnapshot& snapshot)
{
    return snapshot.greenPathBlackValid &&
           (isDirectionalGreenTurn(snapshot.greenCandidateDecision) ||
            snapshot.greenCandidateDecision ==
                GreenTurnDecision::TurnAround180);
}

bool isStrongCorner90Candidate(const CameraLineSnapshot& snapshot)
{
    return snapshot.corner90Candidate &&
           snapshot.corner90Direction != Corner90Direction::None &&
           std::abs(snapshot.corner90Angle) >=
               config::kCorner90MinimumStrongAngleDegrees &&
           std::abs(snapshot.nearError) <= config::kCorner90MaximumCenterError;
}

MotorCommand calculateNearReacquisitionCommand(double correction)
{
    // A reaquisição preserva a base de partida validada de 0,67 mesmo quando o perfil
    // inferior publica uma prévia mais rápida para o tracking normal.
    const double safeCorrection = std::clamp(
        correction,
        -kLineRecoveryMaximumCorrection,
        kLineRecoveryMaximumCorrection);
    if (safeCorrection > 0.0)
    {
        return {kLineRecoveryBaseSpeed + safeCorrection,
                kLineRecoveryBaseSpeed};
    }
    if (safeCorrection < 0.0)
    {
        return {kLineRecoveryBaseSpeed,
                kLineRecoveryBaseSpeed + std::abs(safeCorrection)};
    }
    return {kLineRecoveryBaseSpeed, kLineRecoveryBaseSpeed};
}

MotorCommand calculateLineRecoveryCommand(double lineError)
{
    const double safeLineError = std::clamp(lineError, -1.0, 1.0);
    const double errorMagnitude = std::abs(safeLineError);

    if (errorMagnitude >= kLineRecoveryExtremeError)
    {
        return calculateOneWheelPivotCommand(safeLineError);
    }

    if (errorMagnitude <= kLineRecoveryDeadzone)
    {
        return {kLineRecoveryBaseSpeed, kLineRecoveryBaseSpeed};
    }

    const double normalizedMagnitude =
        (errorMagnitude - kLineRecoveryDeadzone) /
        (1.0 - kLineRecoveryDeadzone);
    const double correction = std::min(
        kLineRecoveryMaximumCorrection,
        kLineRecoveryProportionalGain * normalizedMagnitude);

    if (safeLineError < 0.0)
    {
        return {kLineRecoveryBaseSpeed,
                kLineRecoveryBaseSpeed + correction};
    }
    return {kLineRecoveryBaseSpeed + correction,
            kLineRecoveryBaseSpeed};
}
}

void MainMission::reset()
{
    state_ = LineFollowState::TrackingNear;
    greenTurnController_.reset();
    greenDecisionLatched_ = false;
    greenTurnAuthorized_ = false;
    lastSignificantDirection_ = LineDirection::Unknown;
    searchDirection_ = LineDirection::Unknown;
    lastValidError_ = 0.0;
    lastProcessedLineSequence_ = 0;
    hasProcessedLineSequence_ = false;
    consecutiveNearValidSamples_ = 0;
    corner90Direction_ = Corner90Direction::None;
    corner90EnterSamples_ = 0;
    corner90ExitSamples_ = 0;
    resetCorner90Watchdog();
    greenTurnDirection_ = GreenTurnDecision::None;
    greenTurnConfirmSamples_ = 0;
    greenTurnExitSamples_ = 0;
    nearRecoveryActive_ = false;
    totalLossActive_ = false;
    nearLostAt_ = {};
    totalLossStartedAt_ = {};
    resetGapTracking();
}

void MainMission::update(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    bool cameraReady,
    const CameraLineSnapshot& cameraLineSnapshot)
{
    const RobotSnapshot robotSnapshot = robotState.snapshot();
    if (robotSnapshot.mode != "autonomous" ||
        robotSnapshot.autonomousMission != AutonomousMission::MainMission)
    {
        return;
    }

    if (!esp32Telemetry.readyForOperation() || !cameraReady ||
        !cameraLineSnapshot.sourceFresh)
    {
        // Uma fonte obrigatória indisponível encerra a execução e zera os motores.
        // O motivo terminal permanece no dashboard para diferenciar falha da
        // ESP32, da captura ou do IPC visual sem reutilizar dados antigos.
        std::string phase;
        std::string action;
        if (!esp32Telemetry.readyForOperation())
        {
            phase = "esp32_not_ready";
            action = "Missão interrompida: ESP32 sem telemetria pronta";
        }
        else if (!cameraReady)
        {
            phase = "camera_not_ready";
            action = "Missão interrompida: câmera inferior indisponível";
        }
        else
        {
            phase = "line_ipc_stale";
            action = "Missão interrompida: IPC visual ausente ou antigo";
        }
        robotState.stop();
        robotState.updateAutonomousStatus(makeLineStatus(phase, action));
        std::cout << "MainMission stopped: " << phase << std::endl;
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const bool newLineSample =
        !hasProcessedLineSequence_ ||
        cameraLineSnapshot.lineSequence != lastProcessedLineSequence_;
    if (newLineSample)
    {
        hasProcessedLineSequence_ = true;
        lastProcessedLineSequence_ = cameraLineSnapshot.lineSequence;

        if (cameraLineSnapshot.nearValid)
        {
            updateDirectionMemory(cameraLineSnapshot.nearError);
        }
        else if (cameraLineSnapshot.farValid)
        {
            updateDirectionMemory(cameraLineSnapshot.farError);
        }
    }

    // O verde duplo tem prioridade sobre seguimento, curva antecipada e gap.
    // A manobra usa o IMU e nunca comanda GPIO diretamente.
    if (updateGreenTurn(
            robotState,
            esp32Telemetry,
            cameraLineSnapshot,
            newLineSample))
    {
        return;
    }

    const bool greenApproachActive =
        cameraLineSnapshot.greenPathBlackValid &&
        cameraLineSnapshot.greenTurnDecision == GreenTurnDecision::Approach;

    const bool greenDecisionConfirming =
        state_ == LineFollowState::GreenDecisionConfirming;
    if (greenDecisionConfirming)
    {
        if (newLineSample)
        {
            const bool sameCandidate =
                !cameraLineSnapshot.gapCandidate &&
                isStrongGreenCandidate(cameraLineSnapshot) &&
                cameraLineSnapshot.greenCandidateDecision ==
                    greenTurnDirection_;
            if (!sameCandidate)
            {
                // Uma observação ausente, ambígua ou de lado diferente cancela
                // o verde antes de qualquer pivot ou retorno pelo IMU.
                greenTurnDirection_ = GreenTurnDecision::None;
                greenTurnConfirmSamples_ = 0;
                greenTurnAuthorized_ = false;
                transitionTo(LineFollowState::TrackingNear);
                robotState.driveAutonomous(
                    cameraLineSnapshot.leftPreview,
                    cameraLineSnapshot.rightPreview);
                robotState.updateAutonomousStatus(makeLineStatus(
                    "green_cancelled",
                    "Marcador verde cancelado: decisão não se manteve"));
                return;
            }
            ++greenTurnConfirmSamples_;
        }

        if (greenTurnConfirmSamples_ >= config::kGreenTurnConfirmationFrames)
        {
            greenDecisionLatched_ = true;
            greenTurnAuthorized_ = true;
            corner90Direction_ = Corner90Direction::None;
            corner90EnterSamples_ = 0;
            corner90ExitSamples_ = 0;
            nearRecoveryActive_ = false;
            totalLossActive_ = false;
            resetGapTracking();

            if (greenTurnDirection_ == GreenTurnDecision::TurnAround180)
            {
                // O retorno mantém a checagem de IMU e a máquina existente.
                if (updateGreenTurn(
                        robotState,
                        esp32Telemetry,
                        cameraLineSnapshot,
                        newLineSample))
                {
                    return;
                }
            }
            else
            {
                const bool turnLeft =
                    greenTurnDirection_ == GreenTurnDecision::GuideLeft;
                greenTurnAuthorized_ = false;
                greenTurnExitSamples_ = 0;
                transitionTo(turnLeft ? LineFollowState::GreenTurnLeft
                                      : LineFollowState::GreenTurnRight);
                const MotorCommand command = calculateCounterRotationCommand(
                    turnLeft,
                    config::kCorner90PivotStartPower);
                robotState.driveAutonomous(command.left, command.right);
                robotState.updateAutonomousStatus(makeLineStatus(
                    turnLeft ? "green_turn_left" : "green_turn_right",
                    "Marcador verde confirmado: iniciando pivot direcionado"));
                return;
            }
        }

        // A confirmação usa somente quadros novos e mantém os motores zerados.
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeLineStatus(
            "green_confirming",
            "Marcador verde: amostras " +
                std::to_string(greenTurnConfirmSamples_) + "/" +
                std::to_string(config::kGreenTurnConfirmationFrames)));
        return;
    }

    const bool greenConfirmationEntryAllowed =
        (state_ == LineFollowState::TrackingNear ||
         state_ == LineFollowState::Corner90Confirming) &&
        !cameraLineSnapshot.gapCandidate && !greenDecisionLatched_;
    if (newLineSample && greenConfirmationEntryAllowed &&
        isStrongGreenCandidate(cameraLineSnapshot))
    {
        // Um candidato verde forte tem prioridade sobre o cotovelo de linha.
        // O robô para antes de decidir para não atravessar duas curvas próximas.
        greenTurnDirection_ = cameraLineSnapshot.greenCandidateDecision;
        greenTurnConfirmSamples_ = 1;
        greenTurnAuthorized_ = false;
        corner90Direction_ = Corner90Direction::None;
        corner90EnterSamples_ = 0;
        corner90ExitSamples_ = 0;
        transitionTo(LineFollowState::GreenDecisionConfirming);
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeLineStatus(
            "green_confirming",
            "Marcador verde: amostras 1/" +
                std::to_string(config::kGreenTurnConfirmationFrames)));
        return;
    }

    const bool directionalGreenTurnActive =
        state_ == LineFollowState::GreenTurnLeft ||
        state_ == LineFollowState::GreenTurnRight;
    if (directionalGreenTurnActive)
    {
        // O marcador precisa sair completamente antes de a linha nova poder
        // encerrar o pivô. Isso impede que a faixa de chegada ao verde seja
        // interpretada como a direção já adquirida.
        const bool markerCleared = !cameraLineSnapshot.greenNearSeen &&
                                   !cameraLineSnapshot.greenConfirmed;
        if (newLineSample)
        {
            if (markerCleared && cameraLineSnapshot.corner90ExitAlignment)
            {
                ++greenTurnExitSamples_;
            }
            else
            {
                greenTurnExitSamples_ = 0;
            }
        }

        if (greenTurnExitSamples_ >= config::kGreenTurnExitAlignmentFrames)
        {
            greenTurnDirection_ = GreenTurnDecision::None;
            greenTurnExitSamples_ = 0;
            transitionTo(LineFollowState::TrackingNear);
            robotState.driveAutonomous(
                cameraLineSnapshot.leftPreview,
                cameraLineSnapshot.rightPreview);
            robotState.updateAutonomousStatus(makeLineStatus(
                "tracking_near",
                "Curva verde alinhada: retornando ao visual pursuit"));
            return;
        }

        const bool turnLeft = state_ == LineFollowState::GreenTurnLeft;
        const double pivotPower = markerCleared &&
                                          cameraLineSnapshot.corner90ExitAlignment
                                      ? config::kCorner90PivotRunPower
                                      : config::kCorner90PivotStartPower;
        const MotorCommand command =
            calculateCounterRotationCommand(turnLeft, pivotPower);
        robotState.driveAutonomous(command.left, command.right);
        robotState.updateAutonomousStatus(makeLineStatus(
            turnLeft ? "green_turn_left" : "green_turn_right",
            "Pivot pelo marcador verde: alinhamento " +
                std::to_string(greenTurnExitSamples_) + "/" +
                std::to_string(config::kGreenTurnExitAlignmentFrames)));
        return;
    }

    if (cameraLineSnapshot.nearValid && greenApproachActive)
    {
        // A aproximação ainda usa a prévia reduzida publicada pela visão. Um
        // verde direcional confirmado segue pelo pivô dedicado acima.
        corner90Direction_ = Corner90Direction::None;
        corner90EnterSamples_ = 0;
        corner90ExitSamples_ = 0;
        nearRecoveryActive_ = false;
        totalLossActive_ = false;
        transitionTo(LineFollowState::TrackingNear);
        robotState.driveAutonomous(
            cameraLineSnapshot.leftPreview,
            cameraLineSnapshot.rightPreview);
        robotState.updateAutonomousStatus(makeLineStatus(
            "green_approach",
            "Aproximando do marcador verde com velocidade reduzida"));
        return;
    }

    const bool corner90Confirming =
        state_ == LineFollowState::Corner90Confirming;
    if (corner90Confirming)
    {
        if (newLineSample)
        {
            const bool sameStrongCorner =
                isStrongCorner90Candidate(cameraLineSnapshot) &&
                cameraLineSnapshot.corner90Direction == corner90Direction_;
            if (!sameStrongCorner)
            {
                // Uma amostra incompatível cancela a hipótese antes de qualquer
                // giro. O tracking normal retoma com a prévia visual atual.
                corner90Direction_ = Corner90Direction::None;
                corner90EnterSamples_ = 0;
                transitionTo(LineFollowState::TrackingNear);
                robotState.driveAutonomous(
                    cameraLineSnapshot.leftPreview,
                    cameraLineSnapshot.rightPreview);
                robotState.updateAutonomousStatus(makeLineStatus(
                    "corner90_cancelled",
                    "Cotovelo cancelado: geometria não se manteve"));
                return;
            }

            ++corner90EnterSamples_;
        }

        if (corner90EnterSamples_ >= config::kCorner90ConfirmationFrames)
        {
            corner90ExitSamples_ = 0;
            const bool turnLeft =
                corner90Direction_ == Corner90Direction::Left;
            transitionTo(turnLeft ? LineFollowState::Corner90Left
                                  : LineFollowState::Corner90Right);
            startCorner90Watchdog(esp32Telemetry, now);
            const MotorCommand command = calculateCounterRotationCommand(
                turnLeft,
                config::kCorner90PivotStartPower);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeLineStatus(
                turnLeft ? "corner90_left" : "corner90_right",
                "Cotovelo forte confirmado: iniciando pivot"));
            return;
        }

        // Parada curta de decisão: os frames novos confirmam ou rejeitam a
        // geometria sem deixar o robô avançar até ultrapassar o cotovelo.
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeLineStatus(
            "corner90_confirming",
            "Cotovelo forte: amostras " +
                std::to_string(corner90EnterSamples_) + "/" +
                std::to_string(config::kCorner90ConfirmationFrames)));
        return;
    }

    const bool corner90Active =
        state_ == LineFollowState::Corner90Left ||
        state_ == LineFollowState::Corner90Right;
    if (corner90Active)
    {
        if (const char* abortReason = corner90AbortReason(
                esp32Telemetry, cameraLineSnapshot, newLineSample, now))
        {
            const std::string action =
                std::string(abortReason) == "corner90_stall"
                    ? "Pivot interrompido: motores ou encoders sem giro"
                    : std::string(abortReason) == "corner90_line_lost"
                          ? "Pivot interrompido: linha não foi reencontrada"
                          : "Pivot interrompido: limite angular excedido";
            corner90Direction_ = Corner90Direction::None;
            corner90EnterSamples_ = 0;
            corner90ExitSamples_ = 0;
            resetCorner90Watchdog();
            // Uma falha de atuação ou de visão durante contrarrotação não pode
            // delegar a parada ao timeout externo: os motores são zerados aqui.
            robotState.stop();
            robotState.updateAutonomousStatus(makeLineStatus(abortReason, action));
            return;
        }

        if (newLineSample)
        {
            if (cameraLineSnapshot.corner90ExitAlignment)
            {
                ++corner90ExitSamples_;
            }
            else
            {
                corner90ExitSamples_ = 0;
            }
        }

        if (corner90ExitSamples_ >= config::kCorner90ExitAlignmentFrames)
        {
            // A nova faixa foi vista em duas imagens novas dentro da margem de
            // saída. O Pure Pursuit corrige o restante sem prolongar o pivô.
            corner90Direction_ = Corner90Direction::None;
            corner90EnterSamples_ = 0;
            corner90ExitSamples_ = 0;
            resetCorner90Watchdog();
            transitionTo(LineFollowState::TrackingNear);
            robotState.driveAutonomous(
                cameraLineSnapshot.leftPreview,
                cameraLineSnapshot.rightPreview);
            robotState.updateAutonomousStatus(makeLineStatus(
                "tracking_near",
                "Cotovelo de 90° alinhado: retornando ao visual pursuit"));
            return;
        }

        const bool turnLeft = state_ == LineFollowState::Corner90Left;
        const double pivotPower = cameraLineSnapshot.corner90ExitAlignment
                                      ? config::kCorner90PivotRunPower
                                      : config::kCorner90PivotStartPower;
        const MotorCommand command =
            calculateCounterRotationCommand(turnLeft, pivotPower);
        robotState.driveAutonomous(command.left, command.right);
        robotState.updateAutonomousStatus(makeLineStatus(
            turnLeft ? "corner90_left" : "corner90_right",
            "Pivot no cotovelo de 90°: alinhamento " +
                std::to_string(corner90ExitSamples_) + "/" +
                std::to_string(config::kCorner90ExitAlignmentFrames)));
        return;
    }

    const bool corner90EntryAllowed =
        state_ == LineFollowState::TrackingNear &&
        cameraLineSnapshot.nearValid &&
        !cameraLineSnapshot.gapCandidate &&
        !cameraLineSnapshot.greenNearSeen &&
        !cameraLineSnapshot.greenConfirmed &&
        !greenApproachActive;
    if (newLineSample && corner90EntryAllowed &&
        isStrongCorner90Candidate(cameraLineSnapshot))
    {
        corner90Direction_ = cameraLineSnapshot.corner90Direction;
        corner90EnterSamples_ = 1;
        corner90ExitSamples_ = 0;
        transitionTo(LineFollowState::Corner90Confirming);
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeLineStatus(
            "corner90_confirming",
            "Cotovelo forte: amostras 1/" +
                std::to_string(config::kCorner90ConfirmationFrames)));
        return;
    }
    else if (newLineSample)
    {
        corner90Direction_ = Corner90Direction::None;
        corner90EnterSamples_ = 0;
    }

    const bool gapStateActive = state_ == LineFollowState::CrossingGap;
    if (!gapStateActive &&
        state_ == LineFollowState::TrackingNear &&
        newLineSample && cameraLineSnapshot.gapCandidate &&
        !nearRecoveryActive_ && !totalLossActive_)
    {
        // A detecção pode chegar com a NEAR vazia, pois usa a faixa inferior.
        corner90Direction_ = Corner90Direction::None;
        corner90EnterSamples_ = 0;
        corner90ExitSamples_ = 0;
        nearRecoveryActive_ = false;
        totalLossActive_ = false;
        gapNearLossObserved_ = !cameraLineSnapshot.nearValid;
        gapStartedAt_ = now;
        transitionTo(LineFollowState::CrossingGap);
    }

    if (state_ == LineFollowState::CrossingGap)
    {
        if (gapStartedAt_ != std::chrono::steady_clock::time_point{} &&
            now - gapStartedAt_ >= kGapTimeout)
        {
            // Sem linha ou continuação confirmada, avançar além deste limite
            // seria inseguro. A parada é terminal para esta execução.
            robotState.stop();
            robotState.updateAutonomousStatus(makeLineStatus(
                "gap_timeout",
                "Missão interrompida: gap sem linha por " +
                    std::to_string(kGapTimeout.count()) + " ms"));
            std::cout << "MainMission stopped: gap timeout ("
                      << kGapTimeout.count() << " ms)" << std::endl;
            resetGapTracking();
            return;
        }

        if (newLineSample && !gapNearLossObserved_ &&
            !cameraLineSnapshot.nearValid)
        {
            // A perda da NEAR separa a fita anterior de uma linha reencontrada.
            // O GAP não usa distância, tempo nem telemetria dos encoders.
            gapNearLossObserved_ = true;
        }

        if (newLineSample && gapNearLossObserved_ &&
            cameraLineSnapshot.nearValid)
        {
            resetGapTracking();
            nearRecoveryActive_ = true;
            transitionTo(LineFollowState::ReacquiringNear);
            const MotorCommand command =
                std::abs(cameraLineSnapshot.nearError) >
                        kNearReacquireMaxAbsError
                    ? calculateOneWheelPivotCommand(
                          cameraLineSnapshot.nearError)
                    : calculateNearReacquisitionCommand(
                          cameraLineSnapshot.targetCorrection);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeLineStatus(
                "reacquiring_near",
                "Linha reencontrada após o gap"));
            return;
        }
        if (newLineSample && cameraLineSnapshot.gapReturnValid)
        {
            const double returnError = cameraLineSnapshot.gapReturnError;
            resetGapTracking();
            nearRecoveryActive_ = true;
            nearLostAt_ = now;
            transitionTo(LineFollowState::ReacquiringNear);
            const MotorCommand command =
                calculateLineRecoveryCommand(returnError);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeLineStatus(
                "reacquiring_near",
                "Continuação do gap encontrada pela visão"));
            return;
        }

        robotState.driveAutonomous(
            config::kGapDriveCommandPower,
            config::kGapDriveCommandPower);
        robotState.updateAutonomousStatus(makeLineStatus(
            "crossing_gap",
            "Atravessando gap até reencontrar a linha"));
        return;
    }

    if (cameraLineSnapshot.nearValid)
    {
        const bool requiresNearReacquisition =
            nearRecoveryActive_ || state_ != LineFollowState::TrackingNear;
        nearRecoveryActive_ = false;
        nearLostAt_ = {};
        totalLossActive_ = false;
        totalLossStartedAt_ = {};

        if (!requiresNearReacquisition)
        {
            consecutiveNearValidSamples_ = 0;
            transitionTo(LineFollowState::TrackingNear);
            robotState.driveAutonomous(
                cameraLineSnapshot.leftPreview,
                cameraLineSnapshot.rightPreview);
            robotState.updateAutonomousStatus(makeLineStatus(
                "tracking_near",
                cameraLineSnapshot.trajectoryValid
                    ? "Seguindo a trajetória por visual pursuit"
                    : "Seguindo pelo fallback FAR/NEAR"));
            return;
        }

        transitionTo(LineFollowState::ReacquiringNear);
        if (std::abs(cameraLineSnapshot.nearError) >
            kNearReacquireMaxAbsError)
        {
            consecutiveNearValidSamples_ = 0;

            // O pivô de uma roda reduz o risco de ultrapassar a linha durante
            // um erro extremo de reaquisição.
            const MotorCommand command =
                calculateOneWheelPivotCommand(cameraLineSnapshot.nearError);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeLineStatus(
                "reacquiring_near", "Readquirindo NEAR: alinhando ao centro"));
            return;
        }

        if (newLineSample &&
            consecutiveNearValidSamples_ < kNearSamplesToConfirmRecovery)
        {
            ++consecutiveNearValidSamples_;
        }

        const MotorCommand command =
            calculateNearReacquisitionCommand(
                cameraLineSnapshot.targetCorrection);
        robotState.driveAutonomous(command.left, command.right);
        robotState.updateAutonomousStatus(makeLineStatus(
            "reacquiring_near",
            "Readquirindo NEAR " +
                std::to_string(consecutiveNearValidSamples_) + "/3"));

        if (consecutiveNearValidSamples_ >= kNearSamplesToConfirmRecovery)
        {
            totalLossActive_ = false;
            consecutiveNearValidSamples_ = 0;
            searchDirection_ = LineDirection::Unknown;
            totalLossStartedAt_ = {};
            transitionTo(LineFollowState::TrackingNear);
        }
        return;
    }

    consecutiveNearValidSamples_ = 0;
    if (!nearRecoveryActive_)
    {
        nearRecoveryActive_ = true;
        nearLostAt_ = now;
        searchDirection_ = LineDirection::Unknown;
    }
    if (cameraLineSnapshot.farValid)
    {
        // A FAR isolada não volta ao tracking normal. Ela apenas mantém uma
        // correção limitada até a NEAR reaparecer e confirmar a linha próxima.
        totalLossActive_ = false;
        totalLossStartedAt_ = {};
        transitionTo(LineFollowState::ReacquiringNear);
        const MotorCommand command =
            calculateLineRecoveryCommand(cameraLineSnapshot.farError);
        robotState.driveAutonomous(command.left, command.right);
        robotState.updateAutonomousStatus(makeLineStatus(
            "fallback_far", "NEAR ausente: alinhando pela FAR"));
        return;
    }
    if (!totalLossActive_)
    {
        // Sem qualquer banda válida, este limite curto impede contrarrotação
        // indefinida sem linha visível.
        totalLossActive_ = true;
        totalLossStartedAt_ = now;
    }
    if (now - totalLossStartedAt_ >= kTotalLossTimeout)
    {
        // Sem qualquer linha visível, o giro também possui limite independente.
        robotState.stop();
        robotState.updateAutonomousStatus(makeLineStatus(
            "line_lost_timeout",
            "Missão interrompida: linha não reencontrada por " +
                std::to_string(kTotalLossTimeout.count()) + " ms"));
        std::cout << "MainMission stopped: total line loss timeout ("
                  << kTotalLossTimeout.count() << " ms)" << std::endl;
        return;
    }

    const LineDirection direction = chooseSearchDirection();
    if (direction == LineDirection::Right)
    {
        transitionTo(LineFollowState::SearchingRight);
        const MotorCommand command =
            calculateCounterRotationCommand(false, kLineRecoveryBaseSpeed);
        robotState.driveAutonomous(command.left, command.right);
        robotState.updateAutonomousStatus(makeLineStatus(
            "searching_right", "Procurando linha à direita"));
        return;
    }

    transitionTo(LineFollowState::SearchingLeft);
    const MotorCommand command =
        calculateCounterRotationCommand(true, kLineRecoveryBaseSpeed);
    robotState.driveAutonomous(command.left, command.right);
    robotState.updateAutonomousStatus(makeLineStatus(
        "searching_left", "Procurando linha à esquerda"));
}

void MainMission::transitionTo(LineFollowState nextState)
{
    if (state_ == nextState)
    {
        return;
    }

    const bool leavingCorner90 =
        (state_ == LineFollowState::Corner90Left ||
         state_ == LineFollowState::Corner90Right) &&
        nextState != LineFollowState::Corner90Left &&
        nextState != LineFollowState::Corner90Right;
    if (leavingCorner90)
    {
        resetCorner90Watchdog();
    }

    state_ = nextState;
    std::cout << "MainMission state: " << stateName(state_);
    if (state_ == LineFollowState::ReacquiringNear ||
        state_ == LineFollowState::TrackingNear)
    {
        std::cout << " nearError=" << lastValidError_;
    }
    std::cout << std::endl;
}

bool MainMission::updateGreenTurn(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    const CameraLineSnapshot& cameraLineSnapshot,
    bool newLineSample)
{
    if (state_ == LineFollowState::TurningAtGreenMarker)
    {
        const ImuTurnOutput output = greenTurnController_.update(esp32Telemetry);
        robotState.driveAutonomous(output.leftPower, output.rightPower);

        AutonomousStatus status;
        status.phase = output.phase == "turning"
                           ? "green_turning"
                           : output.phase;
        status.action = output.action;
        status.progressPercent = output.progressPercent;
        if (output.result == ImuTurnResult::Completed)
        {
            // Depois do retorno, a linha pode estar fora da NEAR. A recuperação
            // existente reassume o controle sem usar tempos fixos de giro.
            nearRecoveryActive_ = true;
            nearLostAt_ = std::chrono::steady_clock::now();
            totalLossActive_ = false;
            totalLossStartedAt_ = {};
            searchDirection_ = LineDirection::Unknown;
            transitionTo(LineFollowState::ReacquiringNear);
            robotState.updateAutonomousStatus(makeLineStatus(
                "green_turn_completed",
                "Retorno verde concluído: procurando a linha"));
            return true;
        }
        if (output.result == ImuTurnResult::Failed)
        {
            // Falha de IMU ou tempo limite encerra a missão com motores zerados.
            robotState.stop();
            robotState.updateAutonomousStatus(status);
            return true;
        }
        robotState.updateAutonomousStatus(status);
        return true;
    }

    if (newLineSample && state_ != LineFollowState::GreenDecisionConfirming &&
        !cameraLineSnapshot.greenNearSeen && !cameraLineSnapshot.greenConfirmed)
    {
        // Só libera outro retorno depois que o marcador anterior sair por
        // completo da cena, evitando repetir o giro sobre o mesmo verde duplo.
        greenDecisionLatched_ = false;
        greenTurnAuthorized_ = false;
        greenTurnDirection_ = GreenTurnDecision::None;
        greenTurnConfirmSamples_ = 0;
    }
    const bool confirmedTurnAround =
        greenTurnAuthorized_ &&
        greenTurnDirection_ == GreenTurnDecision::TurnAround180;
    if (greenDecisionLatched_ && !confirmedTurnAround)
    {
        return false;
    }

    if (!confirmedTurnAround)
    {
        // APPROACH continua nas prévias da visão. LEFT e RIGHT são tratados
        // por estados de pivô dedicados no início de update().
        return false;
    }

    if (!ImuTurnController::imuReady(esp32Telemetry))
    {
        // Sem referência angular recente, o robô espera parado sobre o marcador.
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeLineStatus(
            "green_turn_waiting_imu",
            "Aguardando IMU para retorno de 180° à direita"));
        return true;
    }

    greenDecisionLatched_ = true;
    corner90Direction_ = Corner90Direction::None;
    corner90EnterSamples_ = 0;
    corner90ExitSamples_ = 0;
    nearRecoveryActive_ = false;
    totalLossActive_ = false;
    resetGapTracking();

    if (!greenTurnController_.start(
            config::kGreenTurnAroundTargetDegrees,
            ImuTurnDirection::Right,
            esp32Telemetry))
    {
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeLineStatus(
            "green_turn_waiting_imu",
            "Aguardando IMU para retorno de 180° à direita"));
        return true;
    }

    transitionTo(LineFollowState::TurningAtGreenMarker);
    greenTurnAuthorized_ = false;
    greenTurnConfirmSamples_ = 0;

    const ImuTurnOutput output = greenTurnController_.update(esp32Telemetry);
    robotState.driveAutonomous(output.leftPower, output.rightPower);
    AutonomousStatus status;
    status.phase = "green_turning";
    status.action = output.action;
    status.progressPercent = output.progressPercent;
    robotState.updateAutonomousStatus(status);
    return true;
}

void MainMission::resetGapTracking()
{
    gapNearLossObserved_ = false;
    gapStartedAt_ = {};
}

void MainMission::startCorner90Watchdog(
    const Esp32TelemetrySnapshot& esp32Telemetry,
    std::chrono::steady_clock::time_point now)
{
    // O relógio começa junto com o primeiro comando de pivot. A linha já era
    // válida na entrada do Corner90, portanto este instante também é a última
    // referência visual conhecida até chegarem novos frames da câmera.
    corner90WatchdogActive_ = true;
    corner90StartedAt_ = now;
    corner90LastLineSeenAt_ = now;
    corner90StartYawDegrees_ = esp32Telemetry.yawZDeg;
}

void MainMission::resetCorner90Watchdog()
{
    corner90WatchdogActive_ = false;
    corner90StartedAt_ = {};
    corner90LastLineSeenAt_ = {};
    corner90StartYawDegrees_ = 0.0;
}

const char* MainMission::corner90AbortReason(
    const Esp32TelemetrySnapshot& esp32Telemetry,
    const CameraLineSnapshot& cameraLineSnapshot,
    bool newLineSample,
    std::chrono::steady_clock::time_point now)
{
    if (!corner90WatchdogActive_)
    {
        return nullptr;
    }

    if (newLineSample &&
        (cameraLineSnapshot.nearValid || cameraLineSnapshot.farValid))
    {
        corner90LastLineSeenAt_ = now;
    }

    const bool imuReady = ImuTurnController::imuReady(esp32Telemetry);
    const double yawChangeDegrees = imuReady
                                        ? angularDistanceDegrees(
                                              corner90StartYawDegrees_,
                                              esp32Telemetry.yawZDeg)
                                        : 0.0;
    if (imuReady && yawChangeDegrees >= config::kCorner90MaximumYawDegrees)
    {
        return "corner90_overturn";
    }

    const bool bothEncodersMoving =
        std::abs(esp32Telemetry.leftEncoderRate) >=
            config::kCorner90MinimumEncoderRateCountsPerSecond &&
        std::abs(esp32Telemetry.rightEncoderRate) >=
            config::kCorner90MinimumEncoderRateCountsPerSecond;
    const bool imuShowsMotion = imuReady &&
                                (std::abs(esp32Telemetry.gyroZDegPerSec) >=
                                     config::kCorner90MinimumYawRateDegPerSec ||
                                 yawChangeDegrees >=
                                     config::kCorner90MinimumYawChangeDegrees);
    if (now - corner90StartedAt_ >= std::chrono::milliseconds(
                                        config::kCorner90MotionConfirmationTimeoutMs) &&
        !bothEncodersMoving && !imuShowsMotion)
    {
        return "corner90_stall";
    }

    if (now - corner90LastLineSeenAt_ >= std::chrono::milliseconds(
                                               config::kCorner90LineLossTimeoutMs))
    {
        return "corner90_line_lost";
    }
    return nullptr;
}

void MainMission::updateDirectionMemory(double error)
{
    lastValidError_ = error;
    if (std::abs(error) < kSignificantDirectionError)
    {
        return;
    }

    lastSignificantDirection_ =
        error < 0.0 ? LineDirection::Left : LineDirection::Right;
}

MainMission::LineDirection MainMission::chooseSearchDirection()
{
    if (searchDirection_ != LineDirection::Unknown)
    {
        return searchDirection_;
    }

    if (lastSignificantDirection_ != LineDirection::Unknown)
    {
        searchDirection_ = lastSignificantDirection_;
    }
    else if (lastValidError_ > 0.0)
    {
        searchDirection_ = LineDirection::Right;
    }
    else
    {
        // Erro negativo procura à esquerda; zero também escolhe esquerda
        // deterministicamente para impedir alternância durante a tentativa.
        searchDirection_ = LineDirection::Left;
    }
    return searchDirection_;
}

const char* MainMission::stateName(LineFollowState state)
{
    switch (state)
    {
    case LineFollowState::TrackingNear:
        return "TrackingNear";
    case LineFollowState::TurningAtGreenMarker:
        return "TurningAtGreenMarker";
    case LineFollowState::GreenDecisionConfirming:
        return "GreenDecisionConfirming";
    case LineFollowState::GreenTurnLeft:
        return "GreenTurnLeft";
    case LineFollowState::GreenTurnRight:
        return "GreenTurnRight";
    case LineFollowState::Corner90Confirming:
        return "Corner90Confirming";
    case LineFollowState::Corner90Left:
        return "Corner90Left";
    case LineFollowState::Corner90Right:
        return "Corner90Right";
    case LineFollowState::CrossingGap:
        return "CrossingGap";
    case LineFollowState::ReacquiringNear:
        return "ReacquiringNear";
    case LineFollowState::SearchingLeft:
        return "SearchingLeft";
    case LineFollowState::SearchingRight:
        return "SearchingRight";
    }
    return "Unknown";
}
