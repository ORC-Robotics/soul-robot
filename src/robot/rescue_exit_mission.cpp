#include "obr/rescue_exit_mission.h"
#include "obr/config.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>

namespace
{
// Normaliza a diferença com sinal, incluindo a passagem por 0°/360°.
double signedAngle(double angle) { return std::remainder(angle, 360.0); }
}

void RescueExitMission::reset() { *this = RescueExitMission{}; }

bool RescueExitMission::rejected(double heading) const
{
    return std::any_of(rejected_.begin(), rejected_.end(), [&](const auto& item) {
        return ImuTurnController::angularDistanceDegrees(heading, item.heading) <=
            config::kRescueExitRejectedToleranceDegrees;
    });
}

bool RescueExitMission::startTurn(double degrees,
    const Esp32TelemetrySnapshot& telemetry, Time now)
{
    turn_.reset();
    return turn_.start(std::abs(degrees), degrees < 0 ? ImuTurnDirection::Left : ImuTurnDirection::Right,
        telemetry, config::kBallApproachStartToleranceDegrees, 0, 0,
        config::kRescueExitTurnPower, config::kRescueExitApproachTimeoutMs, now);
}

void RescueExitMission::reject(bool permanent, const char* reason, Time now)
{
    rejected_.push_back({targetHeading_, permanent || round_ == 2});
    failure_ = reason;
    reverseTargetCm_ = std::min(config::kRescueExitReverseMaximumCm,
                               std::min(advanceLeftCm_, advanceRightCm_));
    reversedCm_ = 0.0;
    // O recuo não inclui contagens dos pivôs e começa com preparação parada.
    if (reverseTargetCm_ > 0.0)
    {
        reverse_.start(reverseTargetCm_, config::kRescuePostDepositReversePower, -1, now);
        phase_ = Phase::Backing;
    }
    else
    {
        phase_ = Phase::Searching;
    }
    attempting_ = false;
    movingForward_ = false;
    candidateFrames_ = observedFrames_ = acquisitionFrames_ = scanSteps_ = 0;
    waitingFrame_ = true;
    observedAt_ = now;
}

RescueExitOutput RescueExitMission::fail(const char* reason)
{
    phase_ = Phase::Failed;
    failure_ = reason;
    movingForward_ = false;
    return output("rescue_exit_failed", reason);
}

RescueExitOutput RescueExitMission::output(const char* phase, const char* action,
    double left, double right)
{
    RescueExitOutput result;
    result.completed = phase_ == Phase::Completed;
    result.failed = phase_ == Phase::Failed;
    result.leftPower = std::clamp(left, config::kMinMotorOutput, config::kMaxMotorOutput);
    result.rightPower = std::clamp(right, config::kMinMotorOutput, config::kMaxMotorOutput);
    result.status.phase = phase;
    result.status.action = action;
    result.status.exitSector = sector_;
    result.status.exitConfidence = score_;
    result.status.exitHeadingDegrees = targetHeading_;
    result.status.exitRound = round_;
    result.status.exitLastFailure = failure_;
    result.status.exitAdvanceCm = std::min(advanceLeftCm_, advanceRightCm_);
    result.status.exitReverseCm = reversedCm_;
    result.status.targetDistanceCm = reverseTargetCm_;
    std::ostringstream headings;
    for (const auto& item : rejected_)
        headings << item.heading << (item.permanent ? " permanente; " : " temporário; ");
    result.status.exitRejections = headings.str();
    if (lastPhase_ != phase)
    {
        std::cout << "Rescue exit: " << phase << " heading=" << targetHeading_
                  << " round=" << round_ << " reason=" << action << '\n';
        lastPhase_ = phase;
    }
    return result;
}

RescueExitOutput RescueExitMission::update(const CameraLineSnapshot& bottom,
    const ForwardLineSnapshot& forward, const Esp32TelemetrySnapshot& telemetry,
    std::uint64_t runSequence, Time now)
{
    if (phase_ == Phase::Failed) return output("rescue_exit_failed", failure_.c_str());
    if (phase_ == Phase::Completed) return output("rescue_exit_acquired", "Saída confirmada pela CAM0");
    if (!started_)
    {
        started_ = true;
        startedAt_ = observedAt_ = now;
        lastForwardTimestamp_ = forward.timestamp;
        lastBottomTimestamp_ = bottom.lineTimestamp;
        lastSilverSequence_ = bottom.silverSequence;
        lastForwardSequence_ = forward.sequence;
        lastBottomSequence_ = bottom.lineSequence;
        lastLeft_ = telemetry.leftEncoderCount;
        lastRight_ = telemetry.rightEncoderCount;
        lastUptime_ = telemetry.esp32UptimeMs;
    }
    if (now - startedAt_ >= std::chrono::milliseconds(config::kRescueExitTotalTimeoutMs))
        return fail("Tempo total da busca excedido");

    const bool ready = telemetry.readyForOperation() && ImuTurnController::imuReady(telemetry) &&
        EncoderDistanceController::encodersReady(telemetry) && bottom.sourceFresh &&
        bottom.silverClassifierFresh && forward.sourceFresh && forward.exitAnalysisActive &&
        forward.exitRunSequence == runSequence;
    if (!ready)
    {
        movingForward_ = false;
        progressAt_ = now;
        candidateFrames_ = acquisitionFrames_ = observedFrames_ = 0;
        lastLeft_ = telemetry.leftEncoderCount;
        lastRight_ = telemetry.rightEncoderCount;
        if (!sensorsMissing_) { sensorsMissing_ = true; missingSince_ = now; }
        if (now - missingSince_ >= std::chrono::milliseconds(config::kRescueExitSensorTimeoutMs))
            return fail("Câmera, prata, IMU ou encoders indisponíveis");
        return output("rescue_exit_waiting_sensors", "Parado: aguardando sensores e visão da execução atual");
    }
    sensorsMissing_ = false;
    if (telemetry.esp32UptimeMs < lastUptime_) return fail("ESP32 reiniciou durante a busca");
    lastUptime_ = telemetry.esp32UptimeMs;
    if (movingForward_)
    {
        advanceLeftCm_ += std::abs(static_cast<double>(telemetry.leftEncoderCount - lastLeft_)) /
            config::kEncoderCountsPerCentimeter;
        advanceRightCm_ += std::abs(static_cast<double>(telemetry.rightEncoderCount - lastRight_)) /
            config::kEncoderCountsPerCentimeter;
    }
    lastLeft_ = telemetry.leftEncoderCount;
    lastRight_ = telemetry.rightEncoderCount;
    movingForward_ = false;
    const bool newForward = forward.timestamp > lastForwardTimestamp_ && forward.sequence != lastForwardSequence_;
    const bool newBottom = bottom.lineTimestamp > lastBottomTimestamp_ && bottom.lineSequence != lastBottomSequence_;
    if (newForward) { lastForwardTimestamp_ = forward.timestamp; lastForwardSequence_ = forward.sequence; }
    if (newBottom) { lastBottomTimestamp_ = bottom.lineTimestamp; lastBottomSequence_ = bottom.lineSequence; }

    if (phase_ == Phase::Backing)
    {
        const auto movement = reverse_.update(telemetry, now, "rescue_exit_backing", "Recuando da candidata rejeitada");
        reversedCm_ = movement.status.averageDistanceCm;
        if (movement.failed) return fail(movement.status.action.c_str());
        if (movement.completed)
        {
            phase_ = Phase::Searching;
            observedAt_ = now;
            waitingFrame_ = true;
        }
        return output("rescue_exit_backing", "Recuando da candidata rejeitada", movement.leftPower, movement.rightPower);
    }

    if (attempting_)
    {
        // Prata confirmada prevalece sobre qualquer geometria preta da CAM0.
        if (bottom.courseMarkerConfirmed && bottom.courseMarker == CourseMarker::Gray)
        {
            reject(true, "Entrada prata confirmada", now);
            return output("rescue_exit_rejected", failure_.c_str());
        }
        if (now - attemptAt_ >= std::chrono::milliseconds(config::kRescueExitApproachTimeoutMs))
        {
            reject(false, "Aproximação excedeu o tempo limite", now);
            return output("rescue_exit_rejected", failure_.c_str());
        }
        if (bottom.silverCandidateDetected)
        {
            progressAt_ = now;
            acquisitionFrames_ = 0;
            return output("rescue_exit_checking_silver", "Parado: confirmando possível prata");
        }
        if (phase_ == Phase::Approaching && newBottom)
        {
            const bool newSilver = bottom.silverSequence != lastSilverSequence_;
            lastSilverSequence_ = bottom.silverSequence;
            const bool valid = bottom.lineControlSource == "fusion" && bottom.normalSteeringValid &&
                               bottom.exitLineUnbranched;
            if (!valid) acquisitionFrames_ = 0;
            else if (newSilver) ++acquisitionFrames_;
            if (acquisitionFrames_ >= config::kRescueExitAcquisitionFrames)
            {
                phase_ = Phase::Completed;
                return output("rescue_exit_acquired", "Saída confirmada pela CAM0; retomando o percurso");
            }
        }
        if (forward.cameraObscured)
        {
            reject(false, "CAM1 obstruída", now);
            return output("rescue_exit_rejected", failure_.c_str());
        }
    }

    if (phase_ == Phase::Turning || phase_ == Phase::Aligning)
    {
        const auto turning = turn_.update(telemetry, now);
        if (turning.result == ImuTurnResult::Failed) return fail("Falha no giro da busca");
        if (turning.result == ImuTurnResult::Completed)
        {
            phase_ = attempting_ ? Phase::Approaching : Phase::Searching;
            observedAt_ = progressAt_ = lastSeenAt_ = now;
            waitingFrame_ = true;
        }
        return output("rescue_exit_turning", "Girando com a potência validada do resgate", turning.leftPower, turning.rightPower);
    }

    if (waitingFrame_)
    {
        if (now - observedAt_ < std::chrono::milliseconds(config::kRescueSearchSettlingMs) || !newForward)
            return output("rescue_exit_settling", "Parado: aguardando imagem posterior ao movimento");
        waitingFrame_ = false;
    }

    if (phase_ == Phase::Searching)
        return updateSearch(forward, telemetry, now, newForward);
    return updateApproach(forward, telemetry, now, newForward);
}

RescueExitOutput RescueExitMission::updateSearch(const ForwardLineSnapshot& forward,
    const Esp32TelemetrySnapshot& telemetry, Time now, bool newForward)
{
    if (!newForward) return output("rescue_exit_searching", "Aguardando nova observação dos setores");
    int best = -1;
    for (int index = 0; index < static_cast<int>(forward.exitCandidates.size()); ++index)
    {
        const auto& candidate = forward.exitCandidates[index];
        if (!candidate.visible || forward.cameraObscured || rejected(telemetry.yawZDeg + candidate.txDegrees)) continue;
        if (best < 0 || candidate.score > forward.exitCandidates[best].score ||
            (candidate.score == forward.exitCandidates[best].score && std::abs(candidate.txDegrees) < std::abs(forward.exitCandidates[best].txDegrees)))
            best = index;
    }
    ++observedFrames_;
    if (best >= 0)
    {
        const auto& candidate = forward.exitCandidates[best];
        const double heading = signedAngle(telemetry.yawZDeg + candidate.txDegrees);
        if (candidateFrames_ && ImuTurnController::angularDistanceDegrees(heading, confirmationHeading_) <=
                config::kBallApproachStartToleranceDegrees) ++candidateFrames_;
        else candidateFrames_ = 1;
        confirmationHeading_ = heading;
        sector_ = best;
        score_ = candidate.score;
        if (candidateFrames_ >= config::kRescueExitCandidateFrames)
        {
            targetHeading_ = trackingHeading_ = heading;
            advanceLeftCm_ = advanceRightCm_ = lastProgressCm_ = 0.0;
            acquisitionFrames_ = 0;
            attemptAt_ = lastSeenAt_ = progressAt_ = now;
            attempting_ = true;
            midLatched_ = nearLatched_ = false;
            reverseTargetCm_ = reversedCm_ = 0.0;
            phase_ = Phase::Approaching;
            if (std::abs(candidate.txDegrees) > config::kBallApproachStartToleranceDegrees)
            {
                if (!startTurn(candidate.txDegrees, telemetry, now)) return fail("Não foi possível orientar para a candidata");
                phase_ = Phase::Aligning;
            }
            return output("rescue_exit_candidate", "Candidata confirmada; iniciando aproximação");
        }
        return output("rescue_exit_searching", "Confirmando candidata preta em imagens novas");
    }
    candidateFrames_ = 0;
    if (observedFrames_ < config::kRescueExitCandidateFrames)
        return output("rescue_exit_searching", "Observando setores sem candidata confirmada");
    observedFrames_ = 0;
    if (scanSteps_ * config::kRescueExitScanDegrees >= 360.0)
    {
        if (round_ == 2) return fail("Duas rodadas sem saída válida");
        round_ = 2;
        scanSteps_ = 0;
        rejected_.erase(std::remove_if(rejected_.begin(), rejected_.end(), [](const auto& item) { return !item.permanent; }), rejected_.end());
        return output("rescue_exit_retry_round", "Liberando rejeições temporárias para a última rodada");
    }
    if (!startTurn(config::kRescueExitScanDegrees, telemetry, now)) return fail("Não foi possível iniciar a varredura");
    ++scanSteps_;
    phase_ = Phase::Turning;
    return output("rescue_exit_searching", "Próximo setor à direita");
}

RescueExitOutput RescueExitMission::updateApproach(const ForwardLineSnapshot& forward,
    const Esp32TelemetrySnapshot& telemetry, Time now, bool newForward)
{
    // Associa a observação à direção acompanhada; nunca escolhe o maior blob de novo.
    int match = -1;
    double bestError = config::kRescueExitRejectedToleranceDegrees;
    for (int index = 0; index < static_cast<int>(forward.exitCandidates.size()); ++index)
    {
        const auto& candidate = forward.exitCandidates[index];
        const double error = ImuTurnController::angularDistanceDegrees(
            telemetry.yawZDeg + candidate.txDegrees, trackingHeading_);
        if (candidate.visible && error <= bestError)
        {
            match = index;
            bestError = error;
        }
    }
    bool useful = match >= 0;
    if (useful)
    {
        const auto& candidate = forward.exitCandidates[match];
        // Quanto mais perto, maior a exigência: continuidade no MID e fita no NEAR.
        midLatched_ = midLatched_ || candidate.nearestBand >= 1;
        nearLatched_ = nearLatched_ || candidate.nearestBand >= 2;
        useful = (!midLatched_ || candidate.depthBands >= 2) &&
                 (!nearLatched_ || candidate.tapeValid);
    }
    if (!useful)
    {
        progressAt_ = now;
        if (now - lastSeenAt_ >= std::chrono::milliseconds(config::kRescueExitCandidateLostMs))
        {
            reject(false, "Candidata perdida ou sem continuidade", now);
            return output("rescue_exit_rejected", failure_.c_str());
        }
        return output("rescue_exit_lost", "Parado: aguardando recuperar a candidata");
    }
    const auto& candidate = forward.exitCandidates[match];
    if (newForward)
    {
        lastSeenAt_ = now;
        trackingHeading_ = signedAngle(telemetry.yawZDeg + candidate.txDegrees);
    }
    sector_ = match;
    score_ = candidate.score;
    if (std::abs(candidate.txDegrees) > config::kRescueZoneApproachFullHeadingErrorDegrees)
    {
        if (!startTurn(candidate.txDegrees, telemetry, now)) return fail("Não foi possível realinhar a candidata");
        phase_ = Phase::Aligning;
        return output("rescue_exit_aligning", "Parado: realinhando a candidata");
    }
    const double progress = std::min(advanceLeftCm_, advanceRightCm_);
    if (progress >= lastProgressCm_ + config::kDriveDistanceMinimumProgressCounts / config::kEncoderCountsPerCentimeter)
    {
        lastProgressCm_ = progress;
        progressAt_ = now;
    }
    if (now - progressAt_ >= std::chrono::milliseconds(config::kRescueDistanceStallTimeoutMs))
        return fail("Rodas sem progresso durante a aproximação");
    movingForward_ = true;
    const double correction = std::clamp(candidate.txDegrees / config::kRescueZoneApproachFullHeadingErrorDegrees, -1.0, 1.0) * config::kRescueExitCorrection;
    return output("rescue_exit_approaching", "Aproximando pela CAM1; CAM0 verifica o piso",
        config::kRescueExitApproachPower + correction, config::kRescueExitApproachPower - correction);
}
