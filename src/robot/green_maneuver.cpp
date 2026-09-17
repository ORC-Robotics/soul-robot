#include "obr/green_maneuver.h"

#include "obr/config.h"
#include "obr/imu_turn_controller.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>

namespace
{
AutonomousStatus makeStatus(
    const std::string& phase,
    const std::string& action,
    double progressPercent = 0.0)
{
    AutonomousStatus status;
    status.phase = phase;
    status.action = action;
    status.progressPercent = progressPercent;
    return status;
}

bool frontObstacleConfirmed(const Esp32TelemetrySnapshot& telemetry)
{
    return telemetry.sensorFresh && telemetry.lastSensorAgeMs >= 0 &&
           telemetry.lastSensorAgeMs <= config::kObstacleUltrasonicFreshnessMs &&
           std::isfinite(telemetry.ultrasonicDistanceCm) &&
           telemetry.ultrasonicDistanceCm >= 2.0 &&
           telemetry.ultrasonicDistanceCm <= config::kObstacleDetectionDistanceCm;
}

bool isTurnAround(const CameraLineSnapshot& line)
{
    return config::kGreenTurnAroundEnabled && line.greenConfirmed &&
           line.greenPathBlackValid && line.greenPairCompatible &&
           line.greenCandidateCount == 2 &&
           line.greenInterpretation == GreenInterpretation::TurnAround180;
}

bool isLateral(const CameraLineSnapshot& line)
{
    return line.greenConfirmed && line.greenPathBlackValid &&
           (line.greenInterpretation == GreenInterpretation::Left ||
            line.greenInterpretation == GreenInterpretation::Right);
}

bool isLateralInterpretation(GreenInterpretation interpretation)
{
    return interpretation == GreenInterpretation::Left ||
           interpretation == GreenInterpretation::Right;
}

const char* sideName(GreenInterpretation direction)
{
    return direction == GreenInterpretation::Left ? "left" : "right";
}

const char* interpretationName(GreenInterpretation interpretation)
{
    switch (interpretation)
    {
    case GreenInterpretation::Left: return "LEFT";
    case GreenInterpretation::Right: return "RIGHT";
    case GreenInterpretation::TurnAround180: return "180";
    case GreenInterpretation::FalseMarker: return "FALSE";
    case GreenInterpretation::Ambiguous: return "AMBIGUOUS";
    default: return "NONE";
    }
}

double turnSign(GreenInterpretation direction)
{
    // Na montagem atual, (+esquerdo, -direito) produz giro físico para a
    // direita. Este sinal deve coincidir com o lado já confirmado pela câmera.
    return direction == GreenInterpretation::Left ? -1.0 : 1.0;
}

double yawDistance(double startDegrees, double currentDegrees)
{
    return std::abs(std::remainder(currentDegrees - startDegrees, 360.0));
}

bool centeringGeometryAvailable(const CameraLineSnapshot& line)
{
    // A centralização local só é segura quando as três regiões descrevem
    // juntas a faixa escolhida. Uma leitura parcial pula esta fase opcional.
    if (line.curveDiagnostics.lineState != "GREEN")
    {
        return false;
    }
    const bool nearValid = line.lineNearDetected &&
                           std::isfinite(line.lineNearFinePosition) &&
                           std::abs(line.lineNearFinePosition) <= 1.0;
    const bool mediumValid = line.mediumTrusted &&
                             std::isfinite(line.curveDiagnostics.mediumPosition) &&
                             std::abs(line.curveDiagnostics.mediumPosition) <= 1.0;
    const bool farValid = line.farTrusted &&
                          std::isfinite(line.curveDiagnostics.farBandPosition) &&
                          std::abs(line.curveDiagnostics.farBandPosition) <= 1.0;
    return nearValid && mediumValid && farValid;
}

bool greenPathAvailable(const CameraLineSnapshot& line)
{
    // NEAR sozinho ainda pode ser a faixa antiga sob o robô. MID ou FAR
    // trusted em um frame GREEN novo identificam a continuação após o giro.
    if (line.curveDiagnostics.lineState != "GREEN")
    {
        return false;
    }
    const bool mediumValid = line.mediumTrusted &&
                             std::isfinite(line.curveDiagnostics.mediumPosition) &&
                             std::abs(line.curveDiagnostics.mediumPosition) <= 1.0;
    const bool farValid = line.farTrusted &&
                          std::isfinite(line.curveDiagnostics.farBandPosition) &&
                          std::abs(line.curveDiagnostics.farBandPosition) <= 1.0;
    return mediumValid || farValid;
}

bool correctStraightPowers(
    EncoderDistanceOutput& output,
    const Esp32TelemetrySnapshot& telemetry,
    bool& referenceValid,
    double& referenceYawDegrees,
    double minimumMagnitude = config::kMotorRunMinimumPower)
{
    if (output.leftPower == 0.0 && output.rightPower == 0.0)
    {
        return true;
    }
    if (!ImuTurnController::imuReady(telemetry) ||
        !std::isfinite(output.leftPower) || !std::isfinite(output.rightPower) ||
        output.leftPower * output.rightPower <= 0.0)
    {
        return false;
    }
    if (!referenceValid)
    {
        // A referência nasce no primeiro comando que realmente move as rodas,
        // não durante a espera dos encoders no controlador de distância.
        referenceYawDegrees = telemetry.yawZDeg;
        referenceValid = true;
    }

    // Yaw positivo representa giro físico à direita. Subtrair a correção da
    // roda esquerda e somá-la à direita devolve o rumo tanto na frente quanto
    // na ré; os encoders continuam corrigindo a distância de cada roda.
    const double yawError = std::remainder(
        telemetry.yawZDeg - referenceYawDegrees, 360.0);
    // Na confirmação, limita também pela base para não inverter uma roda
    // nem aumentar a velocidade média quando a base do seguidor é baixa.
    const double maximumCorrection = minimumMagnitude == 0.0
        ? std::min(config::kGreenStraightYawMaximumCorrection,
                   std::min(std::abs(output.leftPower), std::abs(output.rightPower)))
        : config::kGreenStraightYawMaximumCorrection;
    const double correction = std::clamp(
        yawError * config::kGreenStraightYawGainPerDegree,
        -maximumCorrection, maximumCorrection);
    const bool forward = output.leftPower > 0.0;
    const double minimumPower = forward
                                    ? minimumMagnitude
                                    : -config::kMaxMotorOutput;
    const double maximumPower = forward
                                    ? config::kMaxMotorOutput
                                    : -minimumMagnitude;
    output.leftPower = std::clamp(
        output.leftPower - correction, minimumPower, maximumPower);
    output.rightPower = std::clamp(
        output.rightPower + correction, minimumPower, maximumPower);
    return true;
}

std::pair<double, double> confirmationPowers(const CameraLineSnapshot& line)
{
    // Conserva somente a base do seguidor, nunca seu steering. A janela
    // visual avança reta e não começa a curva antes da decisão definitiva.
    const double base = std::clamp(
        (line.lineFollowerLeftPower + line.lineFollowerRightPower) * 0.5,
        0.0, config::kGreenConfirmationMaximumBasePower);
    return {base, base};
}
}

void GreenManeuver::logConfirmation(
    const CameraLineSnapshot& line, const Esp32TelemetrySnapshot& telemetry,
    const char* reason, const char* nextState) const
{
    const double distanceCm = confirmationEncodersStarted_
        ? (std::abs(static_cast<double>(telemetry.leftEncoderCount - confirmationStartLeftCount_)) +
           std::abs(static_cast<double>(telemetry.rightEncoderCount - confirmationStartRightCount_))) /
              (2.0 * config::kEncoderCountsPerCentimeter)
        : 0.0;
    std::cout << "GREEN_EVENT eventSequence=" << confirmationEventSequence_
              << " snapshot=" << line.lineSequence
              << " raw=" << interpretationName(line.greenRawInterpretation)
              << " interpretation=" << interpretationName(line.greenInterpretation)
              << " cameraConfirmed=" << line.greenConfirmed
              << " candidates=" << line.greenCandidateCount
              << " blackValid=" << line.greenPathBlackValid
              << " frontRoiValid=" << line.greenFrontRoiValid
              << " pairCompatible=" << line.greenPairCompatible
              << " latched=" << interpretationName(lateralDecision_)
              << " lateralConfirmed=" << lateralDecisionConfirmed_
              << " extraWaitGranted=" << confirmationExtraWaitGranted_
              << " validBlackSeen=" << confirmationValidBlackSeen_
              << " distanceCm=" << distanceCm
              << " reason=" << reason << " next=" << nextState << '\n';
}

void GreenManeuver::reset()
{
    phase_ = Phase::Idle;
    turnAround_.reset();
    distanceController_.reset();
    lateralDecision_ = GreenInterpretation::None;
    lateralDecisionConfirmed_ = false;
    armed_ = true;
    clearFrames_ = 0;
    confirmationStartLeftCount_ = 0;
    confirmationStartRightCount_ = 0;
    confirmationEncodersStarted_ = false;
    confirmationMovementStopped_ = false;
    confirmationExtraWaitGranted_ = false;
    confirmationValidBlackSeen_ = false;
    confirmationEventSequence_ = 0;
    lastConfirmationTraceSequence_ = 0;
    confirmationTraceStarted_ = false;
    confirmationMovementStartedAt_ = {};
    confirmationMovementStartLineSequence_ = 0;
    confirmationStoppedAt_ = {};
    turnStartYawDegrees_ = 0.0;
    entryHeadingReferenceValid_ = false;
    entryHeadingDegrees_ = 0.0;
    entryYawDegrees_ = 0.0;
    earlyBranchStableFrames_ = 0;
    earlyBranchRejectionLogged_ = false;
    lastSearchLineSequence_ = 0;
    straightStartYawDegrees_ = 0.0;
    straightYawReferenceValid_ = false;
    centeredFrames_ = 0;
    stallRecoveryFusionFrames_ = 0;
    stallRecoveryLastLineSequence_ = 0;
}

void GreenManeuver::startReverse(std::chrono::steady_clock::time_point now)
{
    distanceController_.start(
        config::kGreenLateralReverseDistanceCm,
        config::kGreenLateralReversePower,
        -1,
        now);
    phase_ = Phase::Reversing;
    straightYawReferenceValid_ = false;
}

bool GreenManeuver::active() const
{
    return phase_ != Phase::Idle;
}

bool GreenManeuver::shouldBlockForwardAssist(
    const CameraLineSnapshot& line) const
{
    return active() || (armed_ && line.greenCandidateCount > 0);
}

bool GreenManeuver::update(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line)
{
    const auto now = std::chrono::steady_clock::now();

    if (phase_ == Phase::TurnAround)
    {
        const bool handled = turnAround_.update(robotState, telemetry, line);
        if (!turnAround_.active())
        {
            phase_ = Phase::Idle;
        }
        return handled;
    }

    if (phase_ == Phase::WaitingLineAfterStall)
    {
        // Mantém o PWM zerado até a câmera confirmar uma faixa nova e estável.
        // O stall não mata a missão, mas também não autoriza avançar às cegas.
        const bool lineConfirmed = line.sourceFresh &&
            line.lineControlSource == "fusion" && line.normalSteeringValid &&
            line.curveDiagnostics.lineState == "LINE" &&
            line.curveDiagnostics.virtualState == "NORMAL";
        const bool frontBlocked = frontObstacleConfirmed(telemetry);
        if (lineConfirmed && !frontBlocked &&
            line.lineSequence > stallRecoveryLastLineSequence_)
        {
            ++stallRecoveryFusionFrames_;
            stallRecoveryLastLineSequence_ = line.lineSequence;
        }
        else if (!lineConfirmed || frontBlocked)
        {
            stallRecoveryFusionFrames_ = 0;
        }
        robotState.driveAutonomous(0.0, 0.0);
        if (stallRecoveryFusionFrames_ >=
            config::kGreenStallRecoveryRequiredFusionFrames)
        {
            phase_ = Phase::Idle;
            lateralDecision_ = GreenInterpretation::None;
            lateralDecisionConfirmed_ = false;
            distanceController_.reset();
            robotState.updateAutonomousStatus(makeStatus(
                "green_stall_recovery_ready",
                "Faixa confirmada após travamento: retomando seguidor"));
        }
        else
        {
            robotState.updateAutonomousStatus(makeStatus(
                "green_stall_waiting_line",
                "Manobra verde pausada: aguardando faixa Fusion confiável"));
        }
        return true;
    }

    if (phase_ == Phase::Idle)
    {
        if (line.greenCandidateCount == 0)
        {
            ++clearFrames_;
            if (clearFrames_ >= config::kGreenRearmClearFrames)
            {
                armed_ = true;
                clearFrames_ = 0;
            }
        }
        else
        {
            clearFrames_ = 0;
        }

        if (!armed_ || line.greenCandidateCount == 0)
        {
            return false;
        }

        armed_ = false;
        // Congela a orientação medida na entrada, antes da confirmação e dos
        // 100 mm. Sem referência confiável, preserva o caminho dos 30°.
        entryHeadingReferenceValid_ = line.lineControlSource == "fusion" &&
            (line.mediumTrusted || line.farTrusted) &&
            std::isfinite(line.curveDiagnostics.headingAngleDeg) &&
            std::abs(line.curveDiagnostics.headingAngleDeg) <= 90.0 &&
            ImuTurnController::imuReady(telemetry);
        entryHeadingDegrees_ = line.curveDiagnostics.headingAngleDeg;
        entryYawDegrees_ = telemetry.yawZDeg;
        earlyBranchStableFrames_ = 0;
        earlyBranchRejectionLogged_ = false;
        lateralDecision_ = GreenInterpretation::None;
        lateralDecisionConfirmed_ = false;
        confirmationEncodersStarted_ = false;
        confirmationMovementStopped_ = false;
        confirmationExtraWaitGranted_ = false;
        confirmationValidBlackSeen_ = false;
        confirmationEventSequence_ = line.lineSequence;
        confirmationTraceStarted_ = false;
        confirmationMovementStartedAt_ = {};
        confirmationMovementStartLineSequence_ = 0;
        confirmationStoppedAt_ = {};
        straightYawReferenceValid_ = false;
        phase_ = Phase::Confirming;
        phaseStartedAt_ = now;
    }

    if (phase_ == Phase::Confirming)
    {
        const bool candidateWithValidBlack =
            line.greenCandidateCount > 0 &&
            (line.greenFrontRoiValid || line.greenPathBlackValid);
        // A saída da ROI não apaga a evidência anterior deste evento.
        // Este registro concede apenas espera limitada, nunca LEFT/RIGHT ou 180°.
        confirmationValidBlackSeen_ =
            confirmationValidBlackSeen_ || candidateWithValidBlack;
        if (isTurnAround(line))
        {
            logConfirmation(line, telemetry, "compatible_pair", "TurnAround");
            // A decisão de 180° sempre substitui qualquer leitura lateral
            // acumulada, mas a sequência de movimento continua inalterada.
            phase_ = Phase::TurnAround;
            return turnAround_.update(robotState, telemetry, line);
        }

        const GreenInterpretation currentLateralEvidence =
            isLateralInterpretation(line.greenRawInterpretation)
                ? line.greenRawInterpretation
                : (isLateral(line)
                       ? line.greenInterpretation
                       : GreenInterpretation::None);
        if (isLateralInterpretation(currentLateralEvidence) &&
            lateralDecision_ == GreenInterpretation::None)
        {
            // A primeira evidência lateral válida pertence ao evento que
            // abriu a janela. Um verde oposto visto adiante não cria outro
            // evento nem apaga a decisão já acumulada. Somente um par atual e
            // geometricamente compatível pode promover o evento para 180°.
            lateralDecision_ = currentLateralEvidence;
        }
        const bool hasConfirmedLateral =
            line.greenConfirmed &&
            isLateralInterpretation(line.greenInterpretation);
        if (hasConfirmedLateral &&
            lateralDecision_ == GreenInterpretation::None)
        {
            // A retenção da câmera pode entregar a confirmação depois que o
            // marcador saiu da ROI. Como o evento já está ativo, esta decisão
            // ainda pertence à janela atual e deve ser consumida antes da
            // ausência de candidatos.
            lateralDecision_ = line.greenInterpretation;
        }
        if (hasConfirmedLateral &&
            line.greenInterpretation == lateralDecision_)
        {
            lateralDecisionConfirmed_ = true;
        }
        if (!confirmationTraceStarted_ || line.lineSequence != lastConfirmationTraceSequence_)
        {
            logConfirmation(line, telemetry, "snapshot", "Confirming");
            lastConfirmationTraceSequence_ = line.lineSequence;
            confirmationTraceStarted_ = true;
        }

        if (!confirmationEncodersStarted_)
        {
            const bool stopTimedOut =
                now - phaseStartedAt_ >= std::chrono::milliseconds(
                    config::kRescueDistancePreparationTimeoutMs);
            if (stopTimedOut)
            {
                logConfirmation(line, telemetry, "initial_stop_timeout", "Failed");
                phase_ = Phase::Failed;
                robotState.driveAutonomous(0.0, 0.0);
                robotState.updateAutonomousStatus(makeStatus(
                    "green_confirmation_stop_timeout",
                    "Confirmação verde interrompida: robô não parou a tempo"));
                return true;
            }

            if (!EncoderDistanceController::encodersReady(telemetry))
            {
                robotState.driveAutonomous(0.0, 0.0);
                robotState.updateAutonomousStatus(makeStatus(
                    "green_confirmation_waiting_encoders",
                    "Verde detectado: aguardando encoders para parar"));
                return true;
            }

            if (!EncoderDistanceController::encodersStopped(telemetry))
            {
                robotState.driveAutonomous(0.0, 0.0);
                robotState.updateAutonomousStatus(makeStatus(
                    "green_confirmation_stopping",
                    "Verde detectado: parando antes de medir os 30 mm"));
                return true;
            }

            // A referência nasce somente com as rodas praticamente paradas.
            // O deslocamento da frenagem anterior não consome os 30 mm.
            confirmationStartLeftCount_ = telemetry.leftEncoderCount;
            confirmationStartRightCount_ = telemetry.rightEncoderCount;
            confirmationEncodersStarted_ = true;
            confirmationMovementStartedAt_ = now;
            confirmationMovementStartLineSequence_ = line.lineSequence;
        }
        else if (!EncoderDistanceController::encodersReady(telemetry))
        {
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeStatus(
                "green_confirmation_waiting_encoders",
                "Verde detectado: aguardando encoders para limitar a confirmação"));
            return true;
        }

        const double leftCm = std::abs(static_cast<double>(
            telemetry.leftEncoderCount - confirmationStartLeftCount_)) /
            config::kEncoderCountsPerCentimeter;
        const double rightCm = std::abs(static_cast<double>(
            telemetry.rightEncoderCount - confirmationStartRightCount_)) /
            config::kEncoderCountsPerCentimeter;
        const double distanceCm = (leftCm + rightCm) * 0.5;
        const bool movementLimitReached =
            distanceCm >= config::kGreenConfirmationMaximumDistanceCm;
        if (movementLimitReached && !confirmationMovementStopped_)
        {
            // Os 30 mm limitam somente o movimento. A decisão continua ativa
            // até a câmera confirmar ou descartar o evento atual.
            confirmationMovementStopped_ = true;
            confirmationStoppedAt_ = now;
        }

        // A frenagem não consome o prazo visual. Depois dos 30 mm, preserva
        // também toda a espera parada, sem um FALSE antecipar esse prazo.
        const auto visualDecisionStartedAt = confirmationMovementStopped_
            ? confirmationStoppedAt_ : confirmationMovementStartedAt_;
        const bool hasPostMovementSnapshot =
            line.lineSequence > confirmationMovementStartLineSequence_;
        const bool falseDecisionWindowElapsed = hasPostMovementSnapshot &&
            now - visualDecisionStartedAt >= std::chrono::milliseconds(
                config::kGreenConfirmationDecisionWaitMs +
                (confirmationValidBlackSeen_ ? config::kGreenConfirmationExtraWaitMs : 0));
        if (!lateralDecisionConfirmed_ &&
            lateralDecision_ == GreenInterpretation::None &&
            falseDecisionWindowElapsed &&
            !candidateWithValidBlack &&
            line.greenInterpretation == GreenInterpretation::FalseMarker)
        {
            logConfirmation(line, telemetry, "vision_false", "line_following");
            // FALSE encerra somente uma candidatura provisória em uma imagem
            // posterior ao início do avanço. Uma confirmação nunca é apagada.
            phase_ = Phase::Idle;
            lateralDecision_ = GreenInterpretation::None;
            lateralDecisionConfirmed_ = false;
            return false;
        }

        if (confirmationMovementStopped_ && lateralDecisionConfirmed_)
        {
            logConfirmation(line, telemetry, "lateral_confirmed", "DrivingForward");
            distanceController_.start(
                config::kGreenLateralForwardDistanceCm,
                config::kGreenLateralForwardPower,
                1,
                now);
            phase_ = Phase::DrivingForward;
            phaseStartedAt_ = now;
            straightYawReferenceValid_ = false;
        }
        else if (confirmationMovementStopped_)
        {
            const auto waitingMs = std::chrono::duration_cast<
                std::chrono::milliseconds>(now - confirmationStoppedAt_).count();
            if (!confirmationExtraWaitGranted_ && confirmationValidBlackSeen_ &&
                waitingMs >= config::kGreenConfirmationDecisionWaitMs)
            {
                // Uma concessão por evento, com teto contado da mesma parada.
                // Novas detecções nunca reiniciam este prazo nem os encoders.
                confirmationExtraWaitGranted_ = true;
                logConfirmation(line, telemetry, "extra_wait_valid_black", "Confirming");
            }
            const int decisionWaitMs = config::kGreenConfirmationDecisionWaitMs +
                (confirmationExtraWaitGranted_ ? config::kGreenConfirmationExtraWaitMs : 0);
            if (hasPostMovementSnapshot &&
                waitingMs >= decisionWaitMs)
            {
                logConfirmation(line, telemetry, "decision_timeout", "line_following");
                // Uma visão inconclusiva não é marcada como FALSE. O timeout
                // apenas libera o seguidor para evitar que o robô fique preso.
                phase_ = Phase::Idle;
                lateralDecision_ = GreenInterpretation::None;
                lateralDecisionConfirmed_ = false;
                return false;
            }

            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeStatus(
                "green_confirming",
                "Avanço interrompido nos 30 mm: aguardando decisão da câmera",
                std::clamp(
                    distanceCm / config::kGreenConfirmationMaximumDistanceCm * 100.0,
                    0.0,
                    100.0)));
            return true;
        }
        else
        {
            const auto powers = confirmationPowers(line);
            EncoderDistanceOutput output;
            output.leftPower = powers.first;
            output.rightPower = powers.second;
            // Não aplica o piso de velocidade dos 10 cm à confirmação:
            // uma base já baixa não deve ser aumentada por esta correção.
            if (!correctStraightPowers(
                    output, telemetry, straightYawReferenceValid_,
                    straightStartYawDegrees_, 0.0))
            {
                logConfirmation(line, telemetry, "confirmation_imu_lost", "Failed");
                phase_ = Phase::Failed;
                robotState.driveAutonomous(0.0, 0.0);
                robotState.updateAutonomousStatus(makeStatus(
                    "green_confirmation_imu_lost",
                    "Confirmação verde interrompida: IMU sem rumo confiável"));
                return true;
            }
            robotState.driveAutonomous(output.leftPower, output.rightPower, false);
            robotState.updateAutonomousStatus(makeStatus(
                "green_confirming",
                "Confirmando o verde com avanço reto corrigido pelo yaw",
                std::clamp(
                    distanceCm / config::kGreenConfirmationMaximumDistanceCm * 100.0,
                    0.0,
                    100.0)));
            return true;
        }
    }

    if (phase_ == Phase::DrivingForward)
    {
        EncoderDistanceOutput output = distanceController_.update(
            telemetry,
            now,
            lateralDecision_ == GreenInterpretation::Left
                ? "green_forward_left"
                : "green_forward_right",
            "Verde confirmado: avançando 100 mm antes do giro");
        if (output.failed)
        {
            if (output.status.phase == "rescue_distance_stall")
            {
                phase_ = Phase::WaitingLineAfterStall;
                stallRecoveryFusionFrames_ = 0;
                stallRecoveryLastLineSequence_ = line.lineSequence;
                robotState.driveAutonomous(0.0, 0.0);
                robotState.updateAutonomousStatus(makeStatus(
                    "green_stall_waiting_line",
                    "Avanço verde travado: aguardando faixa Fusion confiável"));
                return true;
            }
            phase_ = Phase::Failed;
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(output.status);
            return true;
        }
        if (!output.completed)
        {
            if (!correctStraightPowers(
                    output, telemetry, straightYawReferenceValid_,
                    straightStartYawDegrees_))
            {
                phase_ = Phase::Failed;
                robotState.driveAutonomous(0.0, 0.0);
                robotState.updateAutonomousStatus(makeStatus(
                    "green_forward_imu_lost",
                    "Avanço verde interrompido: IMU sem rumo confiável"));
                return true;
            }
            robotState.driveAutonomous(
                output.leftPower, output.rightPower, false);
            robotState.updateAutonomousStatus(output.status);
            return true;
        }
        if (!ImuTurnController::imuReady(telemetry))
        {
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeStatus(
                "green_waiting_imu",
                "Avanço concluído: aguardando yaw válido para iniciar o giro"));
            return true;
        }

        // O yaw é apenas a trava mínima. A câmera continua responsável
        // por decidir onde está a trajetória real depois desse deslocamento.
        turnStartYawDegrees_ = telemetry.yawZDeg;
        lastSearchLineSequence_ = line.lineSequence;
        phaseStartedAt_ = now;
        phase_ = Phase::SearchingLine;
    }

    if (phase_ == Phase::SearchingLine)
    {
        if (!ImuTurnController::imuReady(telemetry))
        {
            phase_ = Phase::Failed;
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeStatus(
                "green_search_imu_lost",
                "Busca verde interrompida: IMU sem yaw confiável"));
            return true;
        }

        const double relativeYaw = yawDistance(
            turnStartYawDegrees_, telemetry.yawZDeg);
        const bool minimumYawReached =
            relativeYaw >= config::kGreenLateralMinimumYawDegrees;
        const bool newerVisualSnapshot =
            line.lineSequence > lastSearchLineSequence_;
        if (newerVisualSnapshot)
        {
            // A mesma imagem não pode ser reavaliada como uma faixa recém-
            // encontrada apenas porque o yaw cruzou o mínimo entre ciclos.
            lastSearchLineSequence_ = line.lineSequence;
        }
        // Antes do yaw mínimo, exige faixa atual trusted no lado confirmado.
        // Não usa NEAR, CENTER nem o histórico de verdes como autorização.
        const double confirmedSideSign = turnSign(lateralDecision_);
        const auto isOnConfirmedSide = [&](bool trusted, double position) {
            return trusted && std::isfinite(position) && std::abs(position) <= 1.0 &&
                position * confirmedSideSign >= config::kGreenEarlyLineSideMinimumPosition;
        };
        const bool earlyDirectionalLine = line.lineControlSource == "fusion-green" &&
            std::isfinite(line.curveDiagnostics.finalSteering) &&
            line.curveDiagnostics.finalSteering * confirmedSideSign > 0.0 &&
            (isOnConfirmedSide(line.mediumTrusted, line.curveDiagnostics.mediumPosition) ||
             isOnConfirmedSide(line.farTrusted, line.curveDiagnostics.farBandPosition));
        if (now - phaseStartedAt_ >= std::chrono::milliseconds(
                config::kGreenLateralSearchTimeoutMs))
        {
            phase_ = Phase::Failed;
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeStatus(
                "green_search_timeout",
                "Busca verde interrompida: nova trajetória não encontrada"));
            return true;
        }
        // Posição lateral e steering não distinguem uma reta vista de lado.
        // Soma o yaw relativo à orientação visual para comparar com a entrada.
        // O ângulo é axial: cruzar ±180° não cria um ramo novo artificialmente.
        const double entryRelativeYaw = std::remainder(
            telemetry.yawZDeg - entryYawDegrees_, 360.0);
        const double branchHeadingChange = std::remainder(
            line.curveDiagnostics.headingAngleDeg + entryRelativeYaw -
                entryHeadingDegrees_, 180.0);
        const bool distinctBranch = entryHeadingReferenceValid_ &&
            std::isfinite(line.curveDiagnostics.headingAngleDeg) &&
            std::abs(line.curveDiagnostics.headingAngleDeg) <= 90.0 &&
            branchHeadingChange * confirmedSideSign >=
                config::kGreenEarlyBranchMinimumHeadingChangeDegrees;
        if (newerVisualSnapshot && !minimumYawReached)
        {
            const bool branchEvidence = earlyDirectionalLine &&
                greenPathAvailable(line) && distinctBranch;
            earlyBranchStableFrames_ = branchEvidence
                ? std::min(earlyBranchStableFrames_ + 1,
                    config::kGreenEarlyBranchStableFrames) : 0;
            if (config::kGreenEarlyBranchGuardEnabled && earlyDirectionalLine &&
                !distinctBranch && !earlyBranchRejectionLogged_)
            {
                earlyBranchRejectionLogged_ = true;
                std::cout << "GREEN_SEARCH eventSequence=" << confirmationEventSequence_
                          << " snapshot=" << line.lineSequence
                          << " decision=" << interpretationName(lateralDecision_)
                          << " yawDegrees=" << relativeYaw
                          << " reason=incoming_path_not_distinguished"
                          << " entryReferenceValid=" << entryHeadingReferenceValid_
                          << " entryHeadingDegrees=" << entryHeadingDegrees_
                          << " observedHeadingDegrees=" << line.curveDiagnostics.headingAngleDeg
                          << " entryRelativeYawDegrees=" << entryRelativeYaw
                          << " branchHeadingChangeDegrees=" << branchHeadingChange << '\n';
            }
        }
        const bool earlyBranchAccepted = earlyDirectionalLine &&
            (!config::kGreenEarlyBranchGuardEnabled ||
             earlyBranchStableFrames_ >= config::kGreenEarlyBranchStableFrames);
        if ((minimumYawReached || earlyBranchAccepted) && newerVisualSnapshot &&
            greenPathAvailable(line))
        {
            if (!minimumYawReached)
            {
                std::cout << "GREEN_SEARCH eventSequence=" << confirmationEventSequence_
                          << " snapshot=" << line.lineSequence
                          << " decision=" << interpretationName(lateralDecision_)
                          << " yawDegrees=" << relativeYaw
                          << " reason=confirmed_side_line_before_minimum"
                          << " branchGuardEnabled=" << config::kGreenEarlyBranchGuardEnabled
                          << " branchHeadingChangeDegrees=" << branchHeadingChange
                          << " branchStableFrames=" << earlyBranchStableFrames_ << '\n';
            }
            if (centeringGeometryAvailable(line))
            {
                phase_ = Phase::Centering;
                phaseStartedAt_ = now;
                centeredFrames_ = 0;
            }
            else
            {
                // MID ou FAR válidos bastam para encerrar o pivot. A
                // centralização permanece opcional quando falta uma região.
                startReverse(now);
            }
        }
        else
        {
            const double pivot =
                turnSign(lateralDecision_) * config::kGreenLateralSearchPower;
            robotState.driveAutonomous(pivot, -pivot);
            robotState.updateAutonomousStatus(makeStatus(
                std::string("green_searching_") + sideName(lateralDecision_),
                minimumYawReached
                    ? "Yaw mínimo concluído: procurando MID ou FAR da nova faixa"
                    : "Girando: aguardando 30° ou faixa válida no lado confirmado",
                std::clamp(
                    relativeYaw / config::kGreenLateralMinimumYawDegrees * 100.0,
                    0.0,
                    100.0)));
            return true;
        }
    }

    if (phase_ == Phase::Centering)
    {
        const double nearPosition = line.lineNearFinePosition;
        const double mediumPosition = line.curveDiagnostics.mediumPosition;
        const bool positionsValid = centeringGeometryAvailable(line);

        if (!positionsValid)
        {
            // A faixa já foi encontrada; perder uma região apenas encerra a
            // centralização opcional antes da ré medida.
            startReverse(now);
        }
        else
        {
            const double centerError = (nearPosition + mediumPosition) * 0.5;
            const double headingError = mediumPosition - nearPosition;
            if (std::abs(centerError) <= config::kGreenCenteringPositionTolerance &&
                std::abs(headingError) <= config::kGreenCenteringHeadingTolerance)
            {
                ++centeredFrames_;
            }
            else
            {
                centeredFrames_ = 0;
            }

            const bool timedOut = now - phaseStartedAt_ >=
                                  std::chrono::milliseconds(
                                      config::kGreenCenteringTimeoutMs);
            if (centeredFrames_ >= config::kGreenCenteringRequiredFrames || timedOut)
            {
                startReverse(now);
            }
            else
            {
                const double correction = std::clamp(
                    centerError * config::kGreenCenteringPositionGain +
                        headingError * config::kGreenCenteringHeadingGain,
                    -config::kGreenCenteringMaximumCorrection,
                    config::kGreenCenteringMaximumCorrection);
                const double leftPower = std::clamp(
                    config::kGreenCenteringBasePower + correction,
                    config::kMinMotorOutput,
                    config::kGreenConfirmationMaximumBasePower);
                const double rightPower = std::clamp(
                    config::kGreenCenteringBasePower - correction,
                    config::kMinMotorOutput,
                    config::kGreenConfirmationMaximumBasePower);
                robotState.driveAutonomous(leftPower, rightPower);
                robotState.updateAutonomousStatus(makeStatus(
                    std::string("green_centering_") + sideName(lateralDecision_),
                    "Centralizando pela média e pelo alinhamento de NEAR/MID"));
                return true;
            }
        }
    }

    if (phase_ == Phase::Reversing)
    {
        EncoderDistanceOutput output = distanceController_.update(
            telemetry,
            now,
            lateralDecision_ == GreenInterpretation::Left
                ? "green_reverse_left"
                : "green_reverse_right",
            "Faixa encontrada: recuando 50 mm em linha reta");
        if (output.failed)
        {
            // Um recuo quase concluído não deve bloquear o seguidor para sempre.
            // Encerra a manobra com PWM zerado; se parou antes, aguarda Fusion.
            if (output.status.phase == "rescue_distance_stall" &&
                std::min(output.status.leftDistanceCm,
                         output.status.rightDistanceCm) >=
                    config::kGreenLateralReverseDistanceCm -
                    config::kGreenLateralReverseStallToleranceCm &&
                !frontObstacleConfirmed(telemetry))
            {
                robotState.driveAutonomous(0.0, 0.0);
                AutonomousStatus status = output.status;
                status.phase = "green_reverse_short";
                status.action =
                    "Ré verde encerrada perto da meta: encoders sem progresso";
                robotState.updateAutonomousStatus(status);
                phase_ = Phase::Idle;
                lateralDecision_ = GreenInterpretation::None;
                lateralDecisionConfirmed_ = false;
                distanceController_.reset();
                return true;
            }
            if (output.status.phase == "rescue_distance_stall")
            {
                phase_ = Phase::WaitingLineAfterStall;
                stallRecoveryFusionFrames_ = 0;
                stallRecoveryLastLineSequence_ = line.lineSequence;
                robotState.driveAutonomous(0.0, 0.0);
                robotState.updateAutonomousStatus(makeStatus(
                    "green_stall_waiting_line",
                    "Ré verde travada: aguardando faixa Fusion confiável"));
                return true;
            }
            phase_ = Phase::Failed;
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(output.status);
            return true;
        }
        if (output.completed)
        {
            // Não aplica o seguidor no mesmo ciclo da parada da ré.
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeStatus(
                "green_reverse_complete",
                "Ré verde concluída: devolvendo o controle ao seguidor",
                100.0));
            phase_ = Phase::Idle;
            lateralDecision_ = GreenInterpretation::None;
            lateralDecisionConfirmed_ = false;
            distanceController_.reset();
            return true;
        }
        if (!correctStraightPowers(
                output, telemetry, straightYawReferenceValid_,
                straightStartYawDegrees_))
        {
            phase_ = Phase::Failed;
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeStatus(
                "green_reverse_imu_lost",
                "Ré verde interrompida: IMU sem rumo confiável"));
            return true;
        }
        robotState.driveAutonomous(output.leftPower, output.rightPower, false);
        robotState.updateAutonomousStatus(output.status);
        return true;
    }

    if (phase_ == Phase::Failed)
    {
        robotState.driveAutonomous(0.0, 0.0);
        return true;
    }

    return false;
}
