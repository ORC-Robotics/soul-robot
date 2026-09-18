#include "obr/rescue_exit_mission.h"
#include "obr/config.h"
#include "obr/encoder_distance_controller.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace
{
struct GuidancePowers { double left; double right; };

// Após a reta inicial configurada, o Fusion permite correção leve, inclusive com fita distante.
// A distância inicial já impede que essa correção comece cedo dentro do resgate.
GuidancePowers mapGuidancePowers(double angleDegrees)
{
    const double error = angleDegrees - 90.0;
    double strength = 0.0;
    if (std::abs(error) > config::kRescueExitSteeringDeadbandDegrees)
    {
        const double span = config::kRescueExitSteeringFullDegrees -
                            config::kRescueExitSteeringDeadbandDegrees;
        const double progress = std::clamp(
            (std::abs(error) - config::kRescueExitSteeringDeadbandDegrees) / span, 0.0, 1.0);
        strength = progress * progress * (3.0 - 2.0 * progress);
    }
    const double outer = config::kRescueExitApproachPower + strength *
        (config::kRescueExitSteeringOuterPower - config::kRescueExitApproachPower);
    const double inner = config::kRescueExitApproachPower - strength *
        (config::kRescueExitApproachPower - config::kRescueExitSteeringInnerPower);
    if (error > 0.0) return {outer, inner};
    if (error < 0.0) return {inner, outer};
    return {config::kRescueExitApproachPower, config::kRescueExitApproachPower};
}

// Rejeita evidências incompletas antes de escolher a melhor orientação frontal.
bool usableCandidate(const ExitCandidate& candidate)
{
    return candidate.visible && candidate.guidanceValid &&
        !candidate.blockedByColor && !candidate.grayNoiseLikely &&
        std::isfinite(candidate.score) && std::isfinite(candidate.guidanceAngleDegrees) &&
        candidate.guidanceAngleDegrees >= 0.0 && candidate.guidanceAngleDegrees <= 180.0 &&
        std::isfinite(candidate.entryDepthNormalized) &&
        candidate.entryDepthNormalized >= 0.0 && candidate.entryDepthNormalized <= 1.0;
}
}

void RescueExitMission::reset() { *this = RescueExitMission{}; }

void RescueExitMission::startCompletedRescueRoute()
{
    reset();
    completedRescueRoute_ = true;
    phase_ = Phase::EntryAdvance;
}

void RescueExitMission::setReferenceHeading(double headingDegrees)
{
    referenceValid_ = std::isfinite(headingDegrees);
    if (referenceValid_) referenceHeadingDegrees_ = std::remainder(headingDegrees, 360.0);
}

bool RescueExitMission::requiresRescueZoneDetection() const { return false; }

void RescueExitMission::startStraight(const Esp32TelemetrySnapshot& telemetry, Time now)
{
    phase_ = Phase::Straight;
    lastLeftCount_ = telemetry.leftEncoderCount;
    lastRightCount_ = telemetry.rightEncoderCount;
    leftDistanceCm_ = rightDistanceCm_ = lastProgressCm_ = 0.0;
    progressAt_ = now;
    movingForward_ = false;
}

RescueExitOutput RescueExitMission::fail(const char* reason)
{
    phase_ = Phase::Failed;
    failure_ = reason;
    guidanceState_ = "STOPPED";
    movingForward_ = false;
    return output("rescue_exit_failed", failure_);
}

RescueExitOutput RescueExitMission::output(const char* phase, const std::string& action,
                                         double left, double right)
{
    RescueExitOutput result;
    result.completed = phase_ == Phase::Completed;
    result.failed = phase_ == Phase::Failed;
    // Nenhuma potência não finita chega ao clamp ou ao comando dos motores.
    result.leftPower = std::isfinite(left) ?
        std::clamp(left, config::kMinMotorOutput, config::kMaxMotorOutput) : 0.0;
    result.rightPower = std::isfinite(right) ?
        std::clamp(right, config::kMinMotorOutput, config::kMaxMotorOutput) : 0.0;
    result.status.phase = phase;
    result.status.action = action;
    result.status.exitHeadingDegrees = targetHeadingDegrees_;
    result.status.exitSector = selectedSector_;
    result.status.exitConfidence = selectedConfidence_;
    result.status.exitExplorationHeadingDegrees = targetHeadingDegrees_;
    result.status.exitAdvanceCm = std::min(leftDistanceCm_, rightDistanceCm_);
    result.status.exitGuidanceState = guidanceState_;
    result.status.exitBottomBlocker = bottomBlocker_;
    result.status.exitBottomFrames = acquisitionFrames_;
    result.status.exitFallbackAdvanceCm = fallbackAdvanceCm_;
    result.status.exitExplorationAdvanceCm = result.status.exitAdvanceCm;
    result.status.exitLastFailure = failure_;
    if (lastPhase_ != phase)
    {
        std::cout << "Rescue exit: " << phase << " heading=" << targetHeadingDegrees_
                  << " advanceCm=" << result.status.exitAdvanceCm << " " << action << '\n';
        lastPhase_ = phase;
    }
    return result;
}

RescueExitOutput RescueExitMission::update(const CameraLineSnapshot& bottom,
    const ForwardLineSnapshot& forward, const RescueZoneSnapshot&,
    const Esp32TelemetrySnapshot& telemetry, std::uint64_t runSequence, Time now)
{
    if (phase_ == Phase::Failed) return output("rescue_exit_failed", failure_);
    if (phase_ == Phase::Completed)
        return output("rescue_exit_acquired", "Linha confirmada pela CAM0");
    selectedSector_ = -1;
    selectedConfidence_ = 0.0;
    if (!started_)
    {
        started_ = true;
        startedAt_ = progressAt_ = now;
        lastLeftCount_ = telemetry.leftEncoderCount;
        lastRightCount_ = telemetry.rightEncoderCount;
        lastUptimeMs_ = telemetry.esp32UptimeMs;
        lastBottomSequence_ = bottom.lineSequence;
        lastBottomTimestamp_ = bottom.lineTimestamp;
    }
    if (telemetry.esp32UptimeMs < lastUptimeMs_)
        return fail("ESP32 reiniciou durante a saída");
    lastUptimeMs_ = telemetry.esp32UptimeMs;
    if (now - startedAt_ >= std::chrono::milliseconds(config::kRescueExitTotalTimeoutMs))
        return fail("Tempo total da saída excedido");

    const bool sensorsReady = telemetry.readyForOperation() &&
        ImuTurnController::imuReady(telemetry) &&
        EncoderDistanceController::encodersReady(telemetry);
    // Conta somente avanço comandado com sensores atuais. Giros e pausas não entram na reta.
    if (movingForward_ && sensorsReady)
    {
        leftDistanceCm_ += std::abs(static_cast<double>(telemetry.leftEncoderCount) -
            static_cast<double>(lastLeftCount_)) / config::kEncoderCountsPerCentimeter;
        rightDistanceCm_ += std::abs(static_cast<double>(telemetry.rightEncoderCount) -
            static_cast<double>(lastRightCount_)) / config::kEncoderCountsPerCentimeter;
    }
    lastLeftCount_ = telemetry.leftEncoderCount;
    lastRightCount_ = telemetry.rightEncoderCount;
    movingForward_ = false;
    const bool newBottom = bottom.lineSequence > 0 &&
        bottom.lineSequence != lastBottomSequence_ && bottom.lineTimestamp > lastBottomTimestamp_;
    if (newBottom)
    {
        lastBottomSequence_ = bottom.lineSequence;
        lastBottomTimestamp_ = bottom.lineTimestamp;
    }
    if (!sensorsReady)
    {
        acquisitionFrames_ = 0;
        guidanceState_ = "WAITING_SENSORS";
        progressAt_ = now;
        if (!sensorsMissing_) { sensorsMissing_ = true; sensorsMissingSince_ = now; }
        if (now - sensorsMissingSince_ >= std::chrono::milliseconds(config::kRescueExitSensorTimeoutMs))
            return fail("ESP32, IMU ou encoders indisponíveis");
        return output("rescue_exit_waiting_sensors", "Parado: aguardando ESP32, IMU e encoders");
    }
    sensorsMissing_ = false;
    // Ausência de fita permite avanço; ausência de imagem atual sempre remove autoridade.
    const bool forwardReady = forward.exitAnalysisActive && runSequence > 0 &&
        forward.exitRunSequence == runSequence && std::isfinite(forward.ageMs) &&
        forward.ageMs >= 0.0 && forward.ageMs <= config::kRescueExitForwardStatusTimeoutMs;
    if (!forwardReady)
    {
        acquisitionFrames_ = 0;
        guidanceState_ = "WAITING_CAMERA";
        progressAt_ = now;
        if (!cameraMissing_) { cameraMissing_ = true; cameraMissingSince_ = now; }
        if (now - cameraMissingSince_ >= std::chrono::milliseconds(config::kRescueExitCameraRecoveryTimeoutMs))
            return fail("CAM1 indisponível após tentativa de reinício");
        return output("rescue_exit_waiting_camera", "Parado: aguardando reinício automático da CAM1");
    }
    cameraMissing_ = false;
    if (forward.cameraObscured) return fail("Câmera frontal obstruída durante a saída");

    if (phase_ == Phase::EntryAdvance)
    {
        // Reutiliza a proteção por encoders da entrada normal. O Fusion ainda
        // não assume o controle enquanto o robô cruza a sala de resgate.
        if (entryDistance_.idle())
            entryDistance_.start(config::kRescueCompletedEntryAdvanceCm,
                config::kRescueCompletedEntryPower, 1, now);
        const auto entry = entryDistance_.update(telemetry, now,
            "rescue_exit_remembered_entry", "Avançando na sala com resgate concluído");
        if (entry.failed) return fail(entry.status.action.c_str());
        if (!entry.completed)
            return output(entry.status.phase.c_str(), entry.status.action,
                entry.leftPower, entry.rightPower);
        // O heading ao fim da entrada é a referência do giro de 45° à direita.
        referenceHeadingDegrees_ = telemetry.yawZDeg;
        referenceValid_ = true;
        phase_ = Phase::Preparing;
        return output("rescue_exit_remembered_turn_starting",
            "Entrada concluída: iniciando giro à direita");
    }

    if (phase_ == Phase::Preparing)
    {
        // Sem uma entrada válida, não há alvo seguro para a saída normal.
        if (!referenceValid_) return fail("Yaw de entrada da sala de resgate indisponível");
        targetHeadingDegrees_ = std::remainder(
            referenceHeadingDegrees_ +
                (completedRescueRoute_ ? config::kRescueCompletedRightTurnDegrees
                                       : config::kRescueExitFromEntryYawDegrees), 360.0);
        const double turnDegrees = std::remainder(targetHeadingDegrees_ - telemetry.yawZDeg, 360.0);
        const double toleranceDegrees = completedRescueRoute_
            ? config::kBallApproachStartToleranceDegrees
            : config::kRescueExitFromEntryYawToleranceDegrees;
        if (std::abs(turnDegrees) <= toleranceDegrees)
            startStraight(telemetry, now);
        else
        {
            if (!turn_.start(std::abs(turnDegrees), turnDegrees < 0.0 ?
                    ImuTurnDirection::Left : ImuTurnDirection::Right,
                    telemetry, toleranceDegrees, 0, 0,
                    config::kRescueExitTurnPower, config::kRescueExitApproachTimeoutMs, now))
                return fail("Não foi possível iniciar o giro da saída fixa");
            phase_ = Phase::Turning;
        }
    }
    if (phase_ == Phase::Turning)
    {
        guidanceState_ = "TURNING";
        bottomBlocker_ = "BEFORE_STRAIGHT_GATE";
        const auto movement = turn_.update(telemetry, now);
        if (movement.result == ImuTurnResult::Failed) return fail(movement.action.c_str());
        if (movement.result != ImuTurnResult::Completed)
            return output("rescue_exit_direct_turning", movement.action,
                          movement.leftPower, movement.rightPower);
        startStraight(telemetry, now);
    }

    const double progressCm = std::min(leftDistanceCm_, rightDistanceCm_);
    if (progressCm >= lastProgressCm_ + config::kDriveDistanceMinimumProgressCounts /
                                           config::kEncoderCountsPerCentimeter)
    {
        lastProgressCm_ = progressCm;
        progressAt_ = now;
    }
    if (now - progressAt_ >= std::chrono::milliseconds(config::kRescueDistanceStallTimeoutMs))
        return fail("Rodas sem progresso durante a saída fixa");
    const double straightCm = completedRescueRoute_
        ? config::kRescueCompletedStraightCm
        : config::kRescueExitFrontGuidanceStartCm;
    if (phase_ == Phase::Straight && progressCm >= straightCm)
    {
        phase_ = Phase::FrontGuidance;
        fallbackStartCm_ = progressCm;
    }
    if (phase_ == Phase::Straight)
    {
        acquisitionFrames_ = 0;
        guidanceState_ = "INITIAL_STRAIGHT";
        bottomBlocker_ = "BEFORE_STRAIGHT_GATE";
        movingForward_ = true;
        return output("rescue_exit_initial_straight", "Avançando reto antes de habilitar o Fusion frontal",
                      config::kRescueExitExplorationPower, config::kRescueExitExplorationPower);
    }

    // A saída fixa não precisa classificar a topologia: um T também pode ser o percurso.
    // A CAM0 só assume com comando Fusion atual e quatro imagens novas consecutivas.
    const bool bottomValid = bottom.sourceFresh && bottom.normalSteeringValid &&
        bottom.lineControlSource == "fusion" &&
        std::isfinite(bottom.lineFollowerLeftPower) && std::isfinite(bottom.lineFollowerRightPower);
    if (!bottom.sourceFresh) bottomBlocker_ = "STALE";
    else if (bottom.lineControlSource != "fusion") bottomBlocker_ = "SOURCE_" + bottom.lineControlSource;
    else if (!bottomValid) bottomBlocker_ = "INVALID_COMMAND";
    else bottomBlocker_ = "CONFIRMING";
    if (!bottomValid) acquisitionFrames_ = 0;
    else if (newBottom) ++acquisitionFrames_;
    if (acquisitionFrames_ >= config::kRescueExitAcquisitionFrames)
    {
        phase_ = Phase::Completed;
        guidanceState_ = "CAM0";
        bottomBlocker_ = "NONE";
        return output("rescue_exit_acquired", "Linha confirmada; retomando o seguidor da CAM0",
                      bottom.lineFollowerLeftPower, bottom.lineFollowerRightPower);
    }

    const ExitCandidate* best = nullptr;
    for (std::size_t sector = 0; sector < forward.exitCandidates.size(); ++sector)
    {
        const auto& candidate = forward.exitCandidates[sector];
        if (usableCandidate(candidate) && (!best || candidate.score > best->score))
        {
            best = &candidate;
            selectedSector_ = static_cast<int>(sector);
            selectedConfidence_ = candidate.score;
        }
    }
    if (best || bottomValid) fallbackStartCm_ = progressCm;
    fallbackAdvanceCm_ = progressCm - fallbackStartCm_;
    if (fallbackAdvanceCm_ >= config::kRescueExitFallbackMaximumAdvanceCm)
        return fail("Limite de avanço sem Fusion válido atingido");
    if (!best)
    {
        // Uma imagem sem Fusion cancela imediatamente qualquer correção anterior.
        guidanceState_ = "STRAIGHT_NO_FUSION";
        movingForward_ = true;
        return output("rescue_exit_front_guidance", "Sem Fusion frontal válido; continuando reto",
                      config::kRescueExitExplorationPower, config::kRescueExitExplorationPower);
    }
    guidanceState_ = "CAM1";
    const auto powers = mapGuidancePowers(best->guidanceAngleDegrees);
    movingForward_ = true;
    return output("rescue_exit_front_guidance",
        "Corrigindo pelo Fusion frontal; aguardando confirmação da CAM0",
        powers.left, powers.right);
}
