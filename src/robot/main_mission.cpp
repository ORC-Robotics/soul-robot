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

// O robô nunca pode continuar avançando indefinidamente sem reencontrar a linha.
constexpr auto kGapTimeout = std::chrono::milliseconds(2000);
constexpr int kNearSamplesToConfirmRecovery = 3;

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

bool isAcceptedGreenInstruction(const CameraLineSnapshot& snapshot)
{
    return snapshot.greenConfirmed && snapshot.greenPathBlackValid &&
           (isDirectionalGreenTurn(snapshot.greenTurnDecision) ||
            snapshot.greenTurnDecision ==
                GreenTurnDecision::TurnAround180);
}

bool isCorner90CandidateWithinCenterError(
    bool candidate,
    BlackLineGeometryDirection direction,
    double angle,
    double nearError,
    double maximumCenterError)
{
    return candidate && direction != BlackLineGeometryDirection::None &&
           std::isfinite(angle) &&
           std::abs(angle) >= config::kCorner90MinimumStrongAngleDegrees &&
           std::abs(angle) <= config::kCorner90MaximumStrongAngleDegrees &&
           std::isfinite(nearError) &&
           std::abs(nearError) <= maximumCenterError;
}

bool isStrongCorner90Candidate(const CameraLineSnapshot& snapshot)
{
    return isCorner90CandidateWithinCenterError(
        snapshot.blackLineGeometryCandidate,
        snapshot.blackLineGeometryDirection,
        snapshot.blackLineGeometryAngleDegrees,
        snapshot.nearError,
        config::kCorner90MaximumCenterError) &&
           snapshot.blackLineGeometryConfidence >=
               config::kBlackLineGeometryMinimumConfidence;
}

bool isUsableBlackLineGeometryExit(const CameraLineSnapshot& snapshot)
{
    if (!snapshot.blackLineGeometryExitAlignment || !snapshot.nearValid ||
        !snapshot.trajectoryValid ||
        snapshot.fitSampleCount <
            static_cast<std::uint64_t>(
                config::kBlackLineGeometryExitMinimumFitSamples) ||
        !std::isfinite(snapshot.fitA) || !std::isfinite(snapshot.fitB) ||
        !std::isfinite(snapshot.lookaheadY) ||
        snapshot.lookaheadY < config::kBlackLineGeometryExitMinimumLookahead)
    {
        return false;
    }

    // A derivada do fit é o heading da faixa. Conferir a NEAR e o lookahead
    // garante que a faixa inteira já aponta para frente antes de liberar o
    // Pure Pursuit; FAR isolada não pode encerrar o pivot.
    constexpr double kDegreesToRadians = 0.017453292519943295;
    const double maximumSlope = std::tan(
        config::kBlackLineGeometryExitMaximumHeadingDegrees *
        kDegreesToRadians);
    const double nearSlope = snapshot.fitB;
    const double lookaheadSlope =
        2.0 * snapshot.fitA * snapshot.lookaheadY + snapshot.fitB;
    return std::abs(nearSlope) <= maximumSlope &&
           std::abs(lookaheadSlope) <= maximumSlope;
}

bool isStrongVisualFrame(const CameraLineSnapshot& snapshot)
{
    return snapshot.nearValid &&
           (snapshot.trajectoryValid || snapshot.farValid) &&
           std::isfinite(snapshot.leftPreview) &&
           std::isfinite(snapshot.rightPreview) &&
           snapshot.leftPreview > 0.0 && snapshot.rightPreview > 0.0;
}

bool lineRecoveryEncodersAvailable(const Esp32TelemetrySnapshot& telemetry)
{
    return telemetry.sensorFresh && telemetry.lastSensorAgeMs >= 0 &&
           telemetry.lastSensorAgeMs <=
               config::kLineRecoveryMemoryEncoderFreshnessMs &&
           std::isfinite(telemetry.leftEncoderRate) &&
           std::isfinite(telemetry.rightEncoderRate);
}

bool greenTurnProbeEncodersAvailable(const Esp32TelemetrySnapshot& telemetry)
{
    return telemetry.sensorFresh && telemetry.lastSensorAgeMs >= 0 &&
           telemetry.lastSensorAgeMs <=
               config::kGreenTurnForwardProbeEncoderFreshnessMs &&
           std::isfinite(telemetry.leftEncoderRate) &&
           std::isfinite(telemetry.rightEncoderRate);
}

bool hasGreenTurnVisualHandoffLine(const CameraLineSnapshot& snapshot)
{
    // Um fit atual permite devolver diretamente o comando ao Pure Pursuit.
    // A rota NEAR/FAR sem fit é tratada separadamente pela reaquisição segura.
    return snapshot.nearValid && snapshot.trajectoryValid &&
           std::isfinite(snapshot.leftPreview) &&
           std::isfinite(snapshot.rightPreview) &&
           snapshot.leftPreview > 0.0 && snapshot.rightPreview > 0.0 &&
           !snapshot.gapCandidate;
}

bool hasGreenTurnNearFarRecoveryLine(const CameraLineSnapshot& snapshot)
{
    // NEAR e FAR atuais formam uma rota física suficiente para cancelar a
    // sonda reta. O fit quadrático pode falhar em uma fita muito inclinada,
    // mas isso não autoriza ignorar a faixa já vista pela câmera.
    return snapshot.nearValid && snapshot.farValid &&
           std::isfinite(snapshot.nearError) &&
           std::isfinite(snapshot.farError) &&
           std::isfinite(snapshot.targetCorrection) &&
           !snapshot.gapCandidate;
}

MotorCommand normalizeLineRecoveryMemoryCommand(double leftPower, double rightPower)
{
    const double safeLeft = std::clamp(
        leftPower, 0.0, config::kOperationalMaximumReferencePower);
    const double safeRight = std::clamp(
        rightPower, 0.0, config::kOperationalMaximumReferencePower);
    const double maximumPower = std::max(safeLeft, safeRight);
    if (!std::isfinite(maximumPower) || maximumPower <= 0.0)
    {
        return {};
    }

    const double scale = config::kLineRecoveryMemoryMaximumPower /
                         maximumPower;
    // O piso RUN conserva rodas já em movimento sem introduzir ré ou uma
    // curva nova. O MotorController ainda aplica START quando uma roda parar.
    return {
        std::clamp(
            safeLeft * scale,
            config::kMotorRunMinimumPower,
            config::kLineRecoveryMemoryMaximumPower),
        std::clamp(
            safeRight * scale,
            config::kMotorRunMinimumPower,
            config::kLineRecoveryMemoryMaximumPower),
    };
}

double lineRecoveryDistanceMillimeters(
    const Esp32TelemetrySnapshot& telemetry,
    long long startLeftEncoderCount,
    long long startRightEncoderCount)
{
    const double leftCounts = std::abs(static_cast<double>(
        telemetry.leftEncoderCount - startLeftEncoderCount));
    const double rightCounts = std::abs(static_cast<double>(
        telemetry.rightEncoderCount - startRightEncoderCount));
    // A roda que mais avançou limita o percurso. Assim, uma assimetria não
    // encerra a manobra, mas também não deixa uma roda ultrapassar 100 mm.
    return std::max(leftCounts, rightCounts) /
           config::kEncoderCountsPerCentimeter * 10.0;
}

double greenTurnProbeDistanceMillimeters(
    const Esp32TelemetrySnapshot& telemetry,
    long long startLeftEncoderCount,
    long long startRightEncoderCount)
{
    const double leftCounts = std::abs(static_cast<double>(
        telemetry.leftEncoderCount - startLeftEncoderCount));
    const double rightCounts = std::abs(static_cast<double>(
        telemetry.rightEncoderCount - startRightEncoderCount));
    // A maior roda define o limite físico. Diferenças normais de encoder não
    // encerram a sonda, mas nenhuma roda pode passar de 20 mm.
    return std::max(leftCounts, rightCounts) /
           config::kEncoderCountsPerCentimeter * 10.0;
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
    resetGreenDirectionalTurnTracking();
    greenDecisionLatched_ = false;
    greenTurnIgnoreUntil_ = {};
    lineRecoveryDirection_ = LineDirection::Unknown;
    lastValidError_ = 0.0;
    lastProcessedLineSequence_ = 0;
    hasProcessedLineSequence_ = false;
    consecutiveNearValidSamples_ = 0;
    corner90Direction_ = BlackLineGeometryDirection::None;
    corner90EnterSamples_ = 0;
    corner90ConfirmationWindowSamples_ = 0;
    corner90ExitSamples_ = 0;
    resetCorner90Watchdog();
    greenTurnDirection_ = GreenTurnDecision::None;
    nearRecoveryActive_ = false;
    resetLineRecoveryMemory();
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

        if (isStrongVisualFrame(cameraLineSnapshot))
        {
            // Só uma leitura próxima acompanhada de FAR ou fit confiável pode
            // deixar memória. Ela representa uma direção visual realmente
            // comprovada, nunca um JSON antigo ou um ruído isolado.
            hasLineRecoveryMemory_ = true;
            lineRecoveryMemorySequence_ = cameraLineSnapshot.lineSequence;
            lineRecoveryLeftPower_ = cameraLineSnapshot.leftPreview;
            lineRecoveryRightPower_ = cameraLineSnapshot.rightPreview;
            if (cameraLineSnapshot.nearError < -kSignificantDirectionError)
            {
                lineRecoveryDirection_ = LineDirection::Left;
            }
            else if (cameraLineSnapshot.nearError > kSignificantDirectionError)
            {
                lineRecoveryDirection_ = LineDirection::Right;
            }
            else
            {
                lineRecoveryDirection_ = LineDirection::Unknown;
            }
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

    const bool greenTurnWaitingImu =
        state_ == LineFollowState::GreenTurnWaitingImu;
    if (greenTurnWaitingImu)
    {
        if (newLineSample && !greenDecisionLatched_)
        {
            const bool sameCandidate = greenDecisionLatched_ ||
                (isAcceptedGreenInstruction(cameraLineSnapshot) &&
                 cameraLineSnapshot.greenTurnDecision == greenTurnDirection_);
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
            corner90Direction_ = BlackLineGeometryDirection::None;
            corner90EnterSamples_ = 0;
            corner90ExitSamples_ = 0;
            nearRecoveryActive_ = false;
            resetLineRecoveryMemory();
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
                resetGreenDirectionalTurnTracking();
                if (!greenDirectionalTurnController_.start(
                        config::kGreenDirectionalTurnTargetDegrees,
                        turnLeft ? ImuTurnDirection::Left
                                 : ImuTurnDirection::Right,
                        esp32Telemetry,
                        config::kGreenDirectionalTurnCompletionToleranceDegrees))
                {
                    // Sem uma referência angular recente, não existe giro
                    // seguro por verde. A missão espera parada no marcador.
                    robotState.driveAutonomous(0.0, 0.0);
                    robotState.updateAutonomousStatus(makeLineStatus(
                        "green_turn_waiting_imu",
                        "Aguardando IMU para giro verde de 45°"));
                    return;
                }
                transitionTo(turnLeft ? LineFollowState::GreenTurnLeft
                                      : LineFollowState::GreenTurnRight);
                robotState.updateAutonomousStatus(makeLineStatus(
                    turnLeft ? "green_turn_45_left" : "green_turn_45_right",
                    "Marcador verde confirmado: iniciando giro IMU de 45°"));
                return;
            }
        }

        // A confirmação usa somente quadros novos e mantém os motores zerados.
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeLineStatus(
            "green_turn_waiting_imu",
            "Marcador verde: amostras " +
                std::to_string(greenTurnConfirmSamples_) + "/" +
                std::to_string(config::kGreenTurnConfirmationFrames)));
        return;
    }

    const bool greenTurnCooldownActive = now < greenTurnIgnoreUntil_;
    const bool greenConfirmationEntryAllowed =
        (state_ == LineFollowState::TrackingNear ||
         state_ == LineFollowState::LineRecoveryMemory ||
         state_ == LineFollowState::Corner90Confirming ||
         state_ == LineFollowState::Corner90Left ||
         state_ == LineFollowState::Corner90Right) &&
        !greenDecisionLatched_ && !greenTurnCooldownActive;
    if (newLineSample && greenConfirmationEntryAllowed &&
        isAcceptedGreenInstruction(cameraLineSnapshot))
    {
        // Um candidato verde forte tem prioridade até sobre um pivot de cotovelo
        // recém-iniciado. O robô para antes de decidir para não atravessar duas
        // curvas próximas nem persistir em uma hipótese geométrica errada.
        resetCorner90Watchdog();
        greenTurnDirection_ = cameraLineSnapshot.greenTurnDecision;
        greenTurnConfirmSamples_ = config::kGreenTurnConfirmationFrames;
        greenDecisionLatched_ = true;
        greenTurnAuthorized_ = false;
        corner90Direction_ = BlackLineGeometryDirection::None;
        corner90EnterSamples_ = 0;
        corner90ExitSamples_ = 0;
        resetLineRecoveryMemory();
        transitionTo(LineFollowState::GreenTurnWaitingImu);
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeLineStatus(
            greenTurnDirection_ == GreenTurnDecision::GuideLeft
                ? "green_turn_45_left"
                : greenTurnDirection_ == GreenTurnDecision::GuideRight
                      ? "green_turn_45_right"
                      : "green_turning",
            "Marcador verde: amostras 1/" +
                std::to_string(config::kGreenTurnConfirmationFrames)));
        return;
    }

    if (state_ == LineFollowState::GreenTurnLeft ||
        state_ == LineFollowState::GreenTurnRight)
    {
        const bool turnLeft = state_ == LineFollowState::GreenTurnLeft;
        const ImuTurnOutput output =
            greenDirectionalTurnController_.update(esp32Telemetry);
        if (output.result == ImuTurnResult::Failed)
        {
            // A falha angular não pode deixar o comando anterior ativo.
            resetGreenDirectionalTurnTracking();
            robotState.stop();
            robotState.updateAutonomousStatus(makeLineStatus(
                "green_turn_imu_failed",
                "Giro verde de 45° interrompido: " + output.action));
            return;
        }
        if (output.result == ImuTurnResult::Completed)
        {
            // O giro terminou: a primeira trajetória forte já pode assumir o
            // Pure Pursuit. O cooldown evita repetir o mesmo marcador visível.
            greenTurnVisualHandoffStartedAt_ = now;
            greenTurnIgnoreUntil_ = now + std::chrono::milliseconds(
                config::kGreenTurnVisualHandoffCooldownMs);
            transitionTo(LineFollowState::GreenTurnVisualHandoff);
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeLineStatus(
                "green_turn_visual_handoff",
                "Giro verde de 45° concluído: priorizando trajetória visual"));
            return;
        }

        robotState.driveAutonomous(output.leftPower, output.rightPower);
        AutonomousStatus status;
        status.phase = turnLeft ? "green_turn_45_left" : "green_turn_45_right";
        status.action = output.action;
        status.progressPercent = output.progressPercent;
        robotState.updateAutonomousStatus(status);
        return;
    }

    if (state_ == LineFollowState::GreenTurnVisualHandoff)
    {
        if (newLineSample && hasGreenTurnVisualHandoffLine(cameraLineSnapshot))
        {
            // A rota atual substitui imediatamente a manobra verde. As prévias
            // vêm deste frame novo; não há pivô de reaquisição nem memória velha.
            nearRecoveryActive_ = false;
            consecutiveNearValidSamples_ = 0;
            resetLineRecoveryMemory();
            resetGreenDirectionalTurnTracking();
            transitionTo(LineFollowState::TrackingNear);
            robotState.driveAutonomous(
                cameraLineSnapshot.leftPreview,
                cameraLineSnapshot.rightPreview);
            robotState.updateAutonomousStatus(makeLineStatus(
                "tracking_near",
                "Linha pós-verde forte: retornando ao visual pursuit"));
            return;
        }

        if (newLineSample &&
            hasGreenTurnNearFarRecoveryLine(cameraLineSnapshot))
        {
            // A fita reapareceu, mas o fit ainda não tem amostras suficientes.
            // Em vez de iniciar uma sonda cega, a reaquisição usa a correção
            // atual de NEAR/FAR até o Pure Pursuit voltar a ter trajetória.
            nearRecoveryActive_ = true;
            consecutiveNearValidSamples_ = 0;
            resetLineRecoveryMemory();
            resetGreenDirectionalTurnTracking();
            transitionTo(LineFollowState::ReacquiringNear);
            const MotorCommand command =
                std::abs(cameraLineSnapshot.nearError) >
                        kNearReacquireMaxAbsError
                    ? calculateOneWheelPivotCommand(cameraLineSnapshot.nearError)
                    : calculateNearReacquisitionCommand(
                          cameraLineSnapshot.targetCorrection);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeLineStatus(
                "reacquiring_near",
                "Linha pós-verde reencontrada por NEAR/FAR: readquirindo"));
            return;
        }

        if (cameraLineSnapshot.gapCandidate)
        {
            // Gap continua sendo uma condição especial de segurança durante o
            // handoff; não é seguro seguir uma rota que pode estar desconectada.
            resetGreenDirectionalTurnTracking();
            robotState.stop();
            robotState.updateAutonomousStatus(makeLineStatus(
                "green_turn_line_not_found",
                "Linha pós-verde ambígua: gap detectado"));
            return;
        }

        if (now - greenTurnVisualHandoffStartedAt_ >=
            std::chrono::milliseconds(config::kGreenTurnAcquireTimeoutMs))
        {
            if (!greenTurnProbeEncodersAvailable(esp32Telemetry))
            {
                resetGreenDirectionalTurnTracking();
                robotState.stop();
                robotState.updateAutonomousStatus(makeLineStatus(
                    "green_turn_encoder_unavailable",
                    "Sonda pós-verde bloqueada: encoders indisponíveis"));
                return;
            }

            greenTurnProbeStartLeftEncoderCount_ = esp32Telemetry.leftEncoderCount;
            greenTurnProbeStartRightEncoderCount_ = esp32Telemetry.rightEncoderCount;
            greenTurnProbeLastProgressCounts_ = 0.0;
            greenTurnProbeLastProgressAt_ = now;
            transitionTo(LineFollowState::GreenTurnForwardProbe);
            robotState.driveAutonomous(
                config::kGreenTurnForwardProbePower,
                config::kGreenTurnForwardProbePower);
            robotState.updateAutonomousStatus(makeLineStatus(
                "green_turn_forward_probe",
                "Sonda pós-verde: 0/" +
                    std::to_string(static_cast<int>(
                        config::kGreenTurnForwardProbeDistanceMm)) +
                    " mm sem rota visual"));
            return;
        }

        const auto elapsedMilliseconds = std::chrono::duration_cast<
            std::chrono::milliseconds>(now - greenTurnVisualHandoffStartedAt_).count();
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeLineStatus(
            "green_turn_visual_handoff",
            "Pós-verde: priorizando trajetória visual — cooldown " +
                std::to_string(std::min<long long>(
                    elapsedMilliseconds,
                    config::kGreenTurnVisualHandoffCooldownMs)) + "/" +
                std::to_string(config::kGreenTurnVisualHandoffCooldownMs) +
                " ms"));
        return;
    }

    if (state_ == LineFollowState::GreenTurnForwardProbe)
    {
        if (cameraLineSnapshot.gapCandidate)
        {
            // Mesmo durante a sonda, um gap impede avanço cego adicional.
            resetGreenDirectionalTurnTracking();
            robotState.stop();
            robotState.updateAutonomousStatus(makeLineStatus(
                "green_turn_line_not_found",
                "Sonda pós-verde interrompida: gap detectado"));
            return;
        }

        if (newLineSample && hasGreenTurnVisualHandoffLine(cameraLineSnapshot))
        {
            // A rota forte encontrada pela sonda já é atual; o Pure Pursuit
            // deve retomá-la sem nova espera temporal sobre o mesmo marcador.
            nearRecoveryActive_ = false;
            consecutiveNearValidSamples_ = 0;
            resetLineRecoveryMemory();
            resetGreenDirectionalTurnTracking();
            transitionTo(LineFollowState::TrackingNear);
            robotState.driveAutonomous(
                cameraLineSnapshot.leftPreview,
                cameraLineSnapshot.rightPreview);
            robotState.updateAutonomousStatus(makeLineStatus(
                "tracking_near",
                "Sonda pós-verde encontrou trajetória visual forte"));
            return;
        }

        if (newLineSample &&
            hasGreenTurnNearFarRecoveryLine(cameraLineSnapshot))
        {
            // A sonda nunca pode concluir enquanto NEAR e FAR já descrevem
            // uma rota nova. Cancela-se o avanço reto e a visão reassume de
            // forma limitada, sem depender de um fit quadrático momentâneo.
            nearRecoveryActive_ = true;
            consecutiveNearValidSamples_ = 0;
            resetLineRecoveryMemory();
            resetGreenDirectionalTurnTracking();
            transitionTo(LineFollowState::ReacquiringNear);
            const MotorCommand command =
                std::abs(cameraLineSnapshot.nearError) >
                        kNearReacquireMaxAbsError
                    ? calculateOneWheelPivotCommand(cameraLineSnapshot.nearError)
                    : calculateNearReacquisitionCommand(
                          cameraLineSnapshot.targetCorrection);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeLineStatus(
                "reacquiring_near",
                "Sonda pós-verde cancelada: rota NEAR/FAR reencontrada"));
            return;
        }

        if (!greenTurnProbeEncodersAvailable(esp32Telemetry))
        {
            resetGreenDirectionalTurnTracking();
            robotState.stop();
            robotState.updateAutonomousStatus(makeLineStatus(
                "green_turn_encoder_unavailable",
                "Sonda pós-verde interrompida: encoders indisponíveis"));
            return;
        }

        const double distanceMm = greenTurnProbeDistanceMillimeters(
            esp32Telemetry,
            greenTurnProbeStartLeftEncoderCount_,
            greenTurnProbeStartRightEncoderCount_);
        const double progressCounts = distanceMm / 10.0 *
                                      config::kEncoderCountsPerCentimeter;
        if (progressCounts >= greenTurnProbeLastProgressCounts_ +
                                  config::kGreenTurnForwardProbeMinimumProgressCounts)
        {
            greenTurnProbeLastProgressCounts_ = progressCounts;
            greenTurnProbeLastProgressAt_ = now;
        }

        if (distanceMm >= config::kGreenTurnForwardProbeDistanceMm)
        {
            resetGreenDirectionalTurnTracking();
            robotState.stop();
            robotState.updateAutonomousStatus(makeLineStatus(
                "green_turn_line_not_found",
                "Linha pós-verde não encontrada após sonda de " +
                    std::to_string(static_cast<int>(
                        config::kGreenTurnForwardProbeDistanceMm)) +
                    " mm"));
            return;
        }

        if (now - greenTurnProbeLastProgressAt_ >=
            std::chrono::milliseconds(
                config::kGreenTurnForwardProbeEncoderStallTimeoutMs))
        {
            resetGreenDirectionalTurnTracking();
            robotState.stop();
            robotState.updateAutonomousStatus(makeLineStatus(
                "green_turn_encoder_stall",
                "Sonda pós-verde interrompida: encoders sem avanço"));
            return;
        }

        robotState.driveAutonomous(
            config::kGreenTurnForwardProbePower,
            config::kGreenTurnForwardProbePower);
        robotState.updateAutonomousStatus(makeLineStatus(
            "green_turn_forward_probe",
            "Sonda pós-verde: " +
                std::to_string(static_cast<int>(std::round(distanceMm))) + "/" +
                std::to_string(static_cast<int>(
                    config::kGreenTurnForwardProbeDistanceMm)) +
                " mm sem rota visual"));
        return;
    }

    if (cameraLineSnapshot.nearValid && greenApproachActive)
    {
        // A aproximação ainda usa a prévia reduzida publicada pela visão. Um
        // verde direcional confirmado segue pelo pivô dedicado acima.
        corner90Direction_ = BlackLineGeometryDirection::None;
        corner90EnterSamples_ = 0;
        corner90ExitSamples_ = 0;
        nearRecoveryActive_ = false;
        resetLineRecoveryMemory();
        transitionTo(LineFollowState::TrackingNear);
        robotState.driveAutonomous(
            cameraLineSnapshot.leftPreview,
            cameraLineSnapshot.rightPreview);
        robotState.updateAutonomousStatus(makeLineStatus(
            "green_approach",
            "Aproximando do marcador verde com velocidade reduzida"));
        return;
    }

    const bool corner90Active =
        state_ == LineFollowState::Corner90Left ||
        state_ == LineFollowState::Corner90Right;
    if (corner90Active)
    {
        if (const char* abortReason = corner90AbortReason(
                esp32Telemetry,
                cameraLineSnapshot,
                newLineSample,
                now))
        {
            const std::string action =
                std::string(abortReason) == "corner90_stall"
                    ? "Pivot interrompido: motores ou encoders sem giro"
                    : std::string(abortReason) == "corner90_line_lost"
                          ? "Pivot interrompido: linha não foi reencontrada"
                    : "Pivot interrompido por proteção de segurança";
            corner90Direction_ = BlackLineGeometryDirection::None;
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
            if (isUsableBlackLineGeometryExit(cameraLineSnapshot) &&
                std::isfinite(cameraLineSnapshot.leftPreview) &&
                std::isfinite(cameraLineSnapshot.rightPreview) &&
                cameraLineSnapshot.leftPreview > 0.0 &&
                cameraLineSnapshot.rightPreview > 0.0)
            {
                ++corner90ExitSamples_;
            }
            else
            {
                // A nova faixa precisa substituir visualmente o ramo que
                // causou o pivot; dados incompletos mantêm a proteção ativa.
                corner90ExitSamples_ = 0;
            }
        }

        if (corner90ExitSamples_ >= config::kCorner90ExitAlignmentFrames)
        {
            // A nova faixa atual devolve imediatamente o comando à camada
            // proporcional, sem rearme temporal, gap especial ou alvo IMU.
            corner90Direction_ = BlackLineGeometryDirection::None;
            corner90EnterSamples_ = 0;
            corner90ExitSamples_ = 0;
            resetCorner90Watchdog();
            transitionTo(LineFollowState::TrackingNear);
            robotState.driveAutonomous(
                cameraLineSnapshot.leftPreview,
                cameraLineSnapshot.rightPreview);
            robotState.updateAutonomousStatus(makeLineStatus(
                "tracking_near",
                "Giro visual concluído: Pure Pursuit retomado"));
            return;
        }

        const bool turnLeft = state_ == LineFollowState::Corner90Left;
        const MotorCommand command =
            calculateCounterRotationCommand(
                turnLeft, config::kCorner90PivotStartPower);
        robotState.driveAutonomous(command.left, command.right);
        robotState.updateAutonomousStatus(makeLineStatus(
            turnLeft ? "black_line_geometry_left"
                     : "black_line_geometry_right",
            "Pivot geométrico: alinhamento " +
                std::to_string(corner90ExitSamples_) + "/" +
                std::to_string(config::kCorner90ExitAlignmentFrames)));
        return;
    }

    const bool corner90EntryAllowed =
        (state_ == LineFollowState::TrackingNear ||
         state_ == LineFollowState::LineRecoveryMemory) &&
        cameraLineSnapshot.nearValid &&
        !greenTurnCooldownActive &&
        !cameraLineSnapshot.gapCandidate &&
        !cameraLineSnapshot.greenNearSeen &&
        !cameraLineSnapshot.greenConfirmed &&
        !greenApproachActive;
    if (newLineSample && corner90EntryAllowed &&
        isStrongCorner90Candidate(cameraLineSnapshot))
    {
        corner90Direction_ = cameraLineSnapshot.blackLineGeometryDirection;
        corner90EnterSamples_ = 1;
        corner90ConfirmationWindowSamples_ = 1;
        corner90ExitSamples_ = 0;
        resetLineRecoveryMemory();

        // Uma decisão só chega aqui depois de a câmera validar ângulo e
        // confiança no mesmo frame. O pivot toma prioridade total sobre o
        // Pure Pursuit até o handoff visual da nova faixa.
        const bool turnLeft =
            corner90Direction_ == BlackLineGeometryDirection::Left;
        corner90ConfirmationWindowSamples_ = 0;
        transitionTo(turnLeft ? LineFollowState::Corner90Left
                              : LineFollowState::Corner90Right);
        startCorner90Watchdog(now);
        const MotorCommand command = calculateCounterRotationCommand(
            turnLeft,
            config::kCorner90PivotStartPower);
        robotState.driveAutonomous(command.left, command.right);
        robotState.updateAutonomousStatus(makeLineStatus(
            turnLeft ? "black_line_geometry_left"
                     : "black_line_geometry_right",
            "Geometria forte da linha preta: iniciando pivot imediato"));
        return;
    }
    else if (newLineSample)
    {
        corner90Direction_ = BlackLineGeometryDirection::None;
        corner90EnterSamples_ = 0;
        corner90ConfirmationWindowSamples_ = 0;
    }

    const bool gapStateActive = state_ == LineFollowState::CrossingGap;

    if (!gapStateActive &&
        (state_ == LineFollowState::TrackingNear ||
         state_ == LineFollowState::LineRecoveryMemory) &&
        newLineSample && cameraLineSnapshot.gapCandidate &&
        (!nearRecoveryActive_ ||
         state_ == LineFollowState::LineRecoveryMemory))
    {
        // A detecção pode chegar com a NEAR vazia, pois usa a faixa inferior.
        corner90Direction_ = BlackLineGeometryDirection::None;
        corner90EnterSamples_ = 0;
        corner90ExitSamples_ = 0;
        nearRecoveryActive_ = false;
        resetLineRecoveryMemory();
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
            consecutiveNearValidSamples_ = 0;
            transitionTo(LineFollowState::TrackingNear);
        }
        return;
    }

    consecutiveNearValidSamples_ = 0;
    if (!nearRecoveryActive_)
    {
        nearRecoveryActive_ = true;
    }
    if (cameraLineSnapshot.farValid)
    {
        // A FAR isolada não volta ao tracking normal. Ela apenas mantém uma
        // correção limitada até a NEAR reaparecer e confirmar a linha próxima.
        const bool recoveredFromMemory =
            state_ == LineFollowState::LineRecoveryMemory;
        transitionTo(LineFollowState::ReacquiringNear);
        const MotorCommand command =
            calculateLineRecoveryCommand(cameraLineSnapshot.farError);
        robotState.driveAutonomous(command.left, command.right);
        robotState.updateAutonomousStatus(makeLineStatus(
            "fallback_far",
            recoveredFromMemory
                ? "FAR reencontrada durante a memória visual"
                : "NEAR ausente: alinhando pela FAR"));
        return;
    }

    if (state_ != LineFollowState::LineRecoveryMemory)
    {
        const bool memoryIsRecent = hasLineRecoveryMemory_ &&
            cameraLineSnapshot.lineSequence > lineRecoveryMemorySequence_ &&
            cameraLineSnapshot.lineSequence - lineRecoveryMemorySequence_ <=
                static_cast<std::uint64_t>(
                    config::kLineRecoveryMemoryMaximumSourceSamples);
        if (!newLineSample || !memoryIsRecent)
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeLineStatus(
                "line_recovery_unavailable",
                "Missão interrompida: perda sem memória visual forte"));
            return;
        }
        if (!lineRecoveryEncodersAvailable(esp32Telemetry))
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeLineStatus(
                "line_recovery_encoder_unavailable",
                "Missão interrompida: encoders indisponíveis para recuperação"));
            return;
        }

        const MotorCommand command = normalizeLineRecoveryMemoryCommand(
            lineRecoveryLeftPower_, lineRecoveryRightPower_);
        if (command.left <= 0.0 || command.right <= 0.0)
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeLineStatus(
                "line_recovery_unavailable",
                "Missão interrompida: comando visual guardado inválido"));
            return;
        }

        lineRecoveryLeftPower_ = command.left;
        lineRecoveryRightPower_ = command.right;
        lineRecoveryStartLeftEncoderCount_ = esp32Telemetry.leftEncoderCount;
        lineRecoveryStartRightEncoderCount_ = esp32Telemetry.rightEncoderCount;
        lineRecoveryLastProgressCounts_ = 0.0;
        lineRecoveryLastProgressAt_ = now;
        nearRecoveryActive_ = false;
        transitionTo(LineFollowState::LineRecoveryMemory);
    }

    if (!lineRecoveryEncodersAvailable(esp32Telemetry))
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeLineStatus(
            "line_recovery_encoder_unavailable",
            "Missão interrompida: encoders indisponíveis para recuperação"));
        return;
    }

    const double distanceMillimeters = lineRecoveryDistanceMillimeters(
        esp32Telemetry,
        lineRecoveryStartLeftEncoderCount_,
        lineRecoveryStartRightEncoderCount_);
    const double progressedCounts = distanceMillimeters / 10.0 *
                                   config::kEncoderCountsPerCentimeter;
    if (progressedCounts >= lineRecoveryLastProgressCounts_ +
                                config::kLineRecoveryMemoryMinimumProgressCounts)
    {
        lineRecoveryLastProgressCounts_ = progressedCounts;
        lineRecoveryLastProgressAt_ = now;
    }
    if (distanceMillimeters >= config::kLineRecoveryMemoryMaximumDistanceMm)
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeLineStatus(
            "line_recovery_distance_limit",
            "Missão interrompida: linha não reencontrada em 100 mm"));
        return;
    }
    if (now - lineRecoveryLastProgressAt_ >= std::chrono::milliseconds(
                config::kLineRecoveryMemoryEncoderStallTimeoutMs))
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeLineStatus(
            "line_recovery_encoder_stall",
            "Missão interrompida: encoders sem progresso na recuperação"));
        return;
    }

    const char* direction = lineRecoveryDirection_ == LineDirection::Left
                                ? "esquerda"
                                : lineRecoveryDirection_ == LineDirection::Right
                                      ? "direita"
                                      : "reta";
    robotState.driveAutonomous(lineRecoveryLeftPower_, lineRecoveryRightPower_);
    robotState.updateAutonomousStatus(makeLineStatus(
        "line_recovery_memory",
        "Linha ausente: mantendo direção visual " + std::string(direction) +
            " — " + std::to_string(static_cast<int>(std::round(
                distanceMillimeters))) + "/100 mm"));
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
            resetLineRecoveryMemory();
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

    if (newLineSample && state_ != LineFollowState::GreenTurnWaitingImu &&
        !cameraLineSnapshot.greenNearSeen &&
        cameraLineSnapshot.greenCandidateDecision == GreenTurnDecision::None)
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
    corner90Direction_ = BlackLineGeometryDirection::None;
    corner90EnterSamples_ = 0;
    corner90ExitSamples_ = 0;
    nearRecoveryActive_ = false;
    resetLineRecoveryMemory();
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

void MainMission::resetGreenDirectionalTurnTracking()
{
    greenDirectionalTurnController_.reset();
    greenTurnVisualHandoffStartedAt_ = {};
    greenTurnProbeStartLeftEncoderCount_ = 0;
    greenTurnProbeStartRightEncoderCount_ = 0;
    greenTurnProbeLastProgressCounts_ = 0.0;
    greenTurnProbeLastProgressAt_ = {};
}

void MainMission::startCorner90Watchdog(
    std::chrono::steady_clock::time_point now)
{
    // O relógio começa junto com o primeiro comando de pivot. A linha já era
    // válida na entrada do Corner90, portanto este instante também é a última
    // referência visual conhecida até chegarem novos frames da câmera.
    corner90WatchdogActive_ = true;
    corner90StartedAt_ = now;
    corner90LastLineSeenAt_ = now;
}

void MainMission::resetCorner90Watchdog()
{
    corner90WatchdogActive_ = false;
    corner90StartedAt_ = {};
    corner90LastLineSeenAt_ = {};
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

    const bool bothEncodersMoving =
        std::abs(esp32Telemetry.leftEncoderRate) >=
            config::kCorner90MinimumEncoderRateCountsPerSecond &&
        std::abs(esp32Telemetry.rightEncoderRate) >=
            config::kCorner90MinimumEncoderRateCountsPerSecond;
    if (now - corner90StartedAt_ >= std::chrono::milliseconds(
                                        config::kCorner90MotionConfirmationTimeoutMs) &&
        !bothEncodersMoving)
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
}

void MainMission::resetLineRecoveryMemory()
{
    hasLineRecoveryMemory_ = false;
    lineRecoveryMemorySequence_ = 0;
    lineRecoveryDirection_ = LineDirection::Unknown;
    lineRecoveryLeftPower_ = 0.0;
    lineRecoveryRightPower_ = 0.0;
    lineRecoveryStartLeftEncoderCount_ = 0;
    lineRecoveryStartRightEncoderCount_ = 0;
    lineRecoveryLastProgressCounts_ = 0.0;
    lineRecoveryLastProgressAt_ = {};
}

const char* MainMission::stateName(LineFollowState state)
{
    switch (state)
    {
    case LineFollowState::TrackingNear:
        return "TrackingNear";
    case LineFollowState::TurningAtGreenMarker:
        return "TurningAtGreenMarker";
    case LineFollowState::GreenTurnWaitingImu:
        return "GreenTurnWaitingImu";
    case LineFollowState::GreenTurnLeft:
        return "GreenTurnLeft";
    case LineFollowState::GreenTurnRight:
        return "GreenTurnRight";
    case LineFollowState::GreenTurnVisualHandoff:
        return "GreenTurnVisualHandoff";
    case LineFollowState::GreenTurnForwardProbe:
        return "GreenTurnForwardProbe";
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
    case LineFollowState::LineRecoveryMemory:
        return "LineRecoveryMemory";
    }
    return "Unknown";
}
