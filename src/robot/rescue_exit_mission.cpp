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

// Prolonga somente o giro de ida ao corner, preservando seu sentido original.
double cornerPreAdvanceTurn(double degrees)
{
    if (std::abs(degrees) < 1e-6) return 0.0;
    return degrees + std::copysign(
        config::kRescueExitCornerPreAdvanceExtraDegrees, degrees);
}

struct GuidancePowers
{
    double left;
    double right;
};

// Aplica somente uma curva leve depois que a fita fica próxima na CAM1.
GuidancePowers mapGuidancePowers(double angleDegrees, bool steeringNear)
{
    if (!steeringNear)
        return {config::kRescueExitApproachPower, config::kRescueExitApproachPower};
    const double error = angleDegrees - 90.0;
    const double magnitude = std::abs(error);
    double strength = 0.0;
    if (magnitude > config::kRescueExitSteeringDeadbandDegrees)
    {
        const double span = config::kRescueExitSteeringFullDegrees -
                            config::kRescueExitSteeringDeadbandDegrees;
        const double progress = std::clamp(
            (magnitude - config::kRescueExitSteeringDeadbandDegrees) / span,
            0.0, 1.0);
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

}

bool RescueExitMission::buildGeometryCandidates(double greenHeading)
{
    const double delta = signedAngle(greenHeading - geometryReferenceHeading_);
    const double shortSide = config::kRescueExitCornerShortSeparationDegrees;
    const double longSide = config::kRescueExitCornerLongSeparationDegrees;
    const double diagonal = config::kRescueExitCornerDiagonalDegrees;
    const double tolerance = config::kRescueExitCornerGeometryToleranceDegrees;
    const std::array<double, 5> possibilities = {
        shortSide, -shortSide, longSide, -longSide, diagonal};
    int nearest = -1;
    double nearestError = tolerance;
    for (int index = 0; index < static_cast<int>(possibilities.size()); ++index)
    {
        const double error = std::abs(signedAngle(delta - possibilities[index]));
        if (error <= nearestError)
        {
            nearest = index;
            nearestError = error;
        }
    }

    if (nearest == 0)
        geometryCandidateHeadings_ = {
            signedAngle(geometryReferenceHeading_ + diagonal),
            signedAngle(geometryReferenceHeading_ - longSide)};
    else if (nearest == 1)
        geometryCandidateHeadings_ = {
            signedAngle(geometryReferenceHeading_ + diagonal),
            signedAngle(geometryReferenceHeading_ + longSide)};
    else if (nearest == 2)
        geometryCandidateHeadings_ = {
            signedAngle(geometryReferenceHeading_ + diagonal),
            signedAngle(geometryReferenceHeading_ - shortSide)};
    else if (nearest == 3)
        geometryCandidateHeadings_ = {
            signedAngle(geometryReferenceHeading_ + diagonal),
            signedAngle(geometryReferenceHeading_ + shortSide)};
    else if (nearest == 4)
    {
        // Na diagonal, o yaw sozinho não distingue a orientação do retângulo.
        // Esta dupla é testada primeiro pela própria estratégia geométrica.
        geometryCandidateHeadings_ = {
            signedAngle(geometryReferenceHeading_ + shortSide),
            signedAngle(geometryReferenceHeading_ - longSide)};
    }
    else return false;

    if (knownEntryHeadingValid_)
    {
        // A entrada salva é apenas uma preferência: tenta primeiro o corner
        // angularmente mais distante, sem proibir a outra possibilidade.
        const double firstDistance = ImuTurnController::angularDistanceDegrees(
            geometryCandidateHeadings_[0], knownEntryHeadingDegrees_);
        const double secondDistance = ImuTurnController::angularDistanceDegrees(
            geometryCandidateHeadings_[1], knownEntryHeadingDegrees_);
        if (secondDistance > firstDistance)
            std::swap(geometryCandidateHeadings_[0], geometryCandidateHeadings_[1]);
    }
    return true;
}

bool RescueExitMission::startNextGeometryProbe(
    const Esp32TelemetrySnapshot& telemetry, Time now)
{
    // Visita somente os outros três corners, sempre a partir do vermelho salvo.
    const std::array<double, 3> offsets = {
        config::kRescueExitCornerShortSeparationDegrees,
        config::kRescueExitCornerDiagonalDegrees,
        -config::kRescueExitCornerLongSeparationDegrees};
    if (geometryProbeIndex_ >= static_cast<int>(offsets.size())) return false;

    std::array<double, 3> headings{};
    for (std::size_t index = 0; index < offsets.size(); ++index)
        headings[index] = signedAngle(geometryReferenceHeading_ + offsets[index]);
    if (knownEntryHeadingValid_)
    {
        // A entrada salva apenas ordena a visita: o heading mais distante dela
        // vem primeiro, mas os três corners continuam obrigatoriamente disponíveis.
        std::stable_sort(headings.begin(), headings.end(), [&](double left, double right) {
            return ImuTurnController::angularDistanceDegrees(
                       left, knownEntryHeadingDegrees_) >
                   ImuTurnController::angularDistanceDegrees(
                       right, knownEntryHeadingDegrees_);
        });
    }
    const double nominalHeading = headings[geometryProbeIndex_++];
    geometryEmptyFrames_ = 0;
    // A classificação precisa olhar para o centro geométrico do corner.
    // Os 15° extras pertencem somente ao giro que antecede o avanço reto.
    const double turnDegrees = signedAngle(nominalHeading - telemetry.yawZDeg);
    targetHeading_ = nominalHeading;
    if (std::abs(turnDegrees) <= config::kBallApproachStartToleranceDegrees)
    {
        phase_ = Phase::GeometrySettling;
        geometryPhaseAt_ = now;
        return true;
    }
    if (!startTurn(turnDegrees, telemetry, now)) return false;
    phase_ = Phase::GeometryTurning;
    return true;
}

bool RescueExitMission::startGeometryCandidate(
    const Esp32TelemetrySnapshot& telemetry, Time now)
{
    while (geometryCandidateIndex_ <
               static_cast<int>(geometryCandidateHeadings_.size()) &&
           rejected(geometryCandidateHeadings_[geometryCandidateIndex_]))
        ++geometryCandidateIndex_;
    if (geometryCandidateIndex_ >=
        static_cast<int>(geometryCandidateHeadings_.size())) return false;

    const double nominalHeading =
        geometryCandidateHeadings_[geometryCandidateIndex_++];
    explorationAttempt_ = geometryCandidateIndex_;
    explorationAttemptTargetCm_ = config::kRescueExitCornerAdvanceCm;
    explorationAttemptLeftCm_ = explorationAttemptRightCm_ = 0.0;
    explorationTotalCm_ = explorationLastProgressCm_ = 0.0;
    advanceLeftCm_ = advanceRightCm_ = lastProgressCm_ = 0.0;
    geometryCandidateActive_ = true;
    cornerRecoveryUsed_ = false;
    cornerRecoveryDistance_.reset();
    progressAt_ = now;
    const double turnDegrees = cornerPreAdvanceTurn(
        signedAngle(nominalHeading - telemetry.yawZDeg));
    explorationHeading_ = targetHeading_ =
        signedAngle(telemetry.yawZDeg + turnDegrees);
    if (std::abs(turnDegrees) <= config::kBallApproachStartToleranceDegrees)
    {
        startExplorationAdvance(now);
        return true;
    }
    if (!startTurn(turnDegrees, telemetry, now)) return false;
    phase_ = Phase::ExplorationTurning;
    return true;
}

void RescueExitMission::startGeometryReturn(
    const char* reason, bool permanent,
    const Esp32TelemetrySnapshot& telemetry, Time now)
{
    explorationBlockReason_ = reason;
    rejected_.push_back({explorationHeading_, permanent ?
        config::kRescueExitSilverRejectedToleranceDegrees :
        config::kRescueExitRejectedToleranceDegrees, permanent});
    // Soma somente deslocamentos para frente medidos desde a origem geométrica.
    reverseTargetCm_ = std::min(explorationAttemptLeftCm_, explorationAttemptRightCm_) +
        std::min(advanceLeftCm_, advanceRightCm_);
    reversedCm_ = 0.0;
    movingExploration_ = movingForward_ = attempting_ = false;
    guidanceLatched_ = steeringNearLatched_ = false;
    resumeExplorationAfterReview_ = resumeExplorationTurn_ = false;
    geometryCandidateActive_ = false;
    geometryReturnTurnStarted_ = false;
    cornerRecoveryDistance_.reset();
    if (reverseTargetCm_ > 0.0)
    {
        reverse_.start(reverseTargetCm_, config::kRescueExitExplorationPower, -1, now);
        phase_ = Phase::GeometryBacking;
        return;
    }
    const double turnDegrees = signedAngle(geometryReferenceHeading_ - telemetry.yawZDeg);
    if (std::abs(turnDegrees) > config::kBallApproachStartToleranceDegrees &&
        startTurn(turnDegrees, telemetry, now))
        geometryReturnTurnStarted_ = true;
    phase_ = Phase::GeometryReturnTurning;
}

bool RescueExitMission::startCornerCollisionRecovery(
    const Esp32TelemetrySnapshot& telemetry, Time now)
{
    if (cornerRecoveryUsed_ || !geometryActive_ || !geometryCandidateActive_)
        return false;

    cornerRecoveryUsed_ = true;
    cornerRecoveryReturnHeading_ = explorationHeading_;
    const double visualError = lastGuidanceAngleDegrees_ - 90.0;
    if (guidanceLatched_ &&
        std::abs(visualError) > config::kRescueExitSteeringDeadbandDegrees)
    {
        cornerRecoveryTurnSign_ = visualError < 0.0 ? -1 : 1;
    }
    else
    {
        const double cornerOffset = signedAngle(
            explorationHeading_ - geometryReferenceHeading_);
        cornerRecoveryTurnSign_ = cornerOffset < 0.0 ? -1 : 1;
    }

    movingExploration_ = movingForward_ = attempting_ = false;
    reverseTargetCm_ = config::kRescueExitCornerCollisionReverseCm;
    reversedCm_ = 0.0;
    cornerRecoveryDistance_.start(
        config::kRescueExitCornerCollisionReverseCm,
        config::kRescueExitExplorationPower, -1, now);
    phase_ = Phase::CornerRecoveryBacking;
    progressAt_ = now;
    (void)telemetry;
    return true;
}

RescueExitOutput RescueExitMission::restartGeometry(
    const Esp32TelemetrySnapshot& telemetry, Time now, const char* reason)
{
    // Reinicia somente a estratégia geométrica. A entrada prata e os
    // triângulos já confirmados continuam proibidos entre as tentativas.
    rejected_.erase(std::remove_if(rejected_.begin(), rejected_.end(),
        [](const Rejection& rejection) { return !rejection.permanent; }),
        rejected_.end());
    turn_.reset();
    reverse_.reset();
    cornerRecoveryDistance_.reset();
    geometryCandidateActive_ = false;
    geometryReturnTurnStarted_ = false;
    geometryResumeProbesAfterReturn_ = false;
    cornerRecoveryUsed_ = false;
    attempting_ = movingForward_ = movingExploration_ = false;
    guidanceLatched_ = steeringNearLatched_ = false;
    geometryProbeIndex_ = geometryCandidateIndex_ = geometryEmptyFrames_ = 0;
    candidateFrames_ = observedFrames_ = acquisitionFrames_ = 0;
    explorationAttemptLeftCm_ = explorationAttemptRightCm_ = 0.0;
    advanceLeftCm_ = advanceRightCm_ = lastProgressCm_ = 0.0;
    failure_ = reason;

    const double turnDegrees = signedAngle(
        geometryReferenceHeading_ - telemetry.yawZDeg);
    if (std::abs(turnDegrees) > config::kBallApproachStartToleranceDegrees)
    {
        if (!startTurn(turnDegrees, telemetry, now))
            return fail("Não foi possível reiniciar a geometria no yaw central");
        phase_ = Phase::GeometryTurning;
        return output("rescue_exit_geometry_restarting",
                      "Reiniciando a geometria no yaw central salvo");
    }

    phase_ = Phase::GeometrySettling;
    geometryPhaseAt_ = now;
    return output("rescue_exit_geometry_restarting", reason);
}

RescueExitOutput RescueExitMission::updateGeometry(
    const RescueZoneSnapshot& zones,
    const Esp32TelemetrySnapshot& telemetry, Time now)
{
    if (phase_ == Phase::GeometryTurning)
    {
        const auto turning = turn_.update(telemetry, now);
        if (turning.result == ImuTurnResult::Failed)
        {
            if (turning.phase != "turn_correction_failed")
                return fail(turning.action.c_str());
            return restartGeometry(telemetry, now,
                "Correção angular esgotada; repetindo somente a geometria");
        }
        if (turning.result == ImuTurnResult::Completed)
        {
            phase_ = Phase::GeometrySettling;
            geometryPhaseAt_ = now;
            lastZoneSequence_ = zones.sequence;
        }
        return output("rescue_exit_geometry_turning",
                      "Girando para o centro exato do próximo corner",
                      turning.leftPower, turning.rightPower);
    }

    if (now - geometryPhaseAt_ <
        std::chrono::milliseconds(config::kRescueSearchSettlingMs))
        return output("rescue_exit_geometry_settling",
                      "Parado: estabilizando para observar o corner");

    if (zones.sourceFresh && zones.sequence != lastZoneSequence_)
    {
        lastZoneSequence_ = zones.sequence;
        geometryPhaseAt_ = now;

        // Verde e vermelho confirmados são corners proibidos. Candidatos ainda
        // não confirmados apenas mantêm o robô parado no heading atual.
        const bool greenConfirmed = zones.green.detected;
        const bool redConfirmed = zones.red.detected;
        if ((greenConfirmed || redConfirmed) && geometryProbeIndex_ > 0)
        {
            geometryEmptyFrames_ = 0;
            const auto& color = redConfirmed ? zones.red : zones.green;
            const double colorHeading = color.aimValid ? signedAngle(
                telemetry.yawZDeg + color.aimNormalized *
                    config::kRescueExitCornerCameraHorizontalFovDegrees * 0.5) :
                targetHeading_;
            rejected_.push_back({colorHeading,
                config::kRescueExitCornerGeometryToleranceDegrees, true});
            if (!startNextGeometryProbe(telemetry, now))
                return fail("Todos os corners foram verificados sem saída livre");
            return output("rescue_exit_geometry_turning",
                          redConfirmed ?
                              "Corner vermelho ignorado; indo ao próximo corner" :
                              "Corner verde ignorado; indo ao próximo corner");
        }

        if (geometryProbeIndex_ > 0)
        {
            // Evidência colorida ainda não confirmada nunca conta como frame
            // livre. O robô permanece parado até o filtro decidir a cor.
            if (zones.green.candidateDetected || zones.red.candidateDetected)
            {
                geometryEmptyFrames_ = 0;
                return output("rescue_exit_geometry_observing",
                              "Cor candidata no corner; aguardando classificação");
            }
            ++geometryEmptyFrames_;
            if (geometryEmptyFrames_ < config::kRescueExitCandidateFrames)
                return output("rescue_exit_geometry_observing",
                              "Sem triângulo neste frame; confirmando corner livre");
            // Ausência de triângulo torna o corner explorável. Fusion não é gate.
            geometryCandidateHeadings_[0] = targetHeading_;
            geometryCandidateHeadings_[1] = targetHeading_;
            geometryCandidateIndex_ = 0;
            geometryResumeProbesAfterReturn_ = true;
            if (!startGeometryCandidate(telemetry, now))
                return restartGeometry(telemetry, now,
                    "Corner livre rejeitado; repetindo somente a geometria");
            return output(phase_ == Phase::Exploring ?
                              "rescue_exit_corner_exploring" :
                              "rescue_exit_corner_turning",
                          "Sem triângulo: explorando o corner mesmo sem Fusion",
                          phase_ == Phase::Exploring ?
                              config::kRescueExitExplorationPower : 0.0,
                          phase_ == Phase::Exploring ?
                              config::kRescueExitExplorationPower : 0.0);
        }

        if (!startNextGeometryProbe(telemetry, now))
            return restartGeometry(telemetry, now,
                "Verde não confirmado; repetindo somente a geometria");
        return output(phase_ == Phase::GeometryTurning ?
                          "rescue_exit_geometry_turning" :
                          "rescue_exit_geometry_settling",
                      "Avançando para o próximo corner geométrico");
    }

    if (now - geometryPhaseAt_ >=
        std::chrono::milliseconds(config::kRescueExitCornerVisionTimeoutMs))
        return restartGeometry(telemetry, now,
            "Detector de triângulos sem leitura; repetindo somente a geometria");
    return output("rescue_exit_geometry_waiting",
                  "Parado: aguardando leitura nova do triângulo verde");
}

int RescueExitMission::explorationBin(double heading) const
{
    const double normalized = std::fmod(signedAngle(heading) + 360.0, 360.0);
    return static_cast<int>(std::floor(
        (normalized + config::kRescueExitScanDegrees * 0.5) /
        config::kRescueExitScanDegrees)) % static_cast<int>(explorationBins_.size());
}

void RescueExitMission::recordExplorationEvidence(
    const ForwardLineSnapshot& forward,
    const Esp32TelemetrySnapshot& telemetry)
{
    for (const auto& candidate : forward.exitCandidates)
    {
        const double heading = signedAngle(telemetry.yawZDeg + candidate.txDegrees);
        auto& bin = explorationBins_[explorationBin(heading)];
        bin.colorBlocked = bin.colorBlocked || candidate.blockedByColor;
        bin.grayBlocked = bin.grayBlocked || candidate.grayNoiseLikely;
        if (!candidate.visible || !candidate.guidanceValid || rejected(heading)) continue;
        ++bin.sightings;
        if (candidate.score >= bin.bestScore)
        {
            bin.bestScore = candidate.score;
            bin.bestHeading = heading;
        }
    }
}

bool RescueExitMission::startExploration(
    const Esp32TelemetrySnapshot& telemetry, Time now)
{
    int best = -1;
    for (int index = 0; index < static_cast<int>(explorationBins_.size()); ++index)
    {
        const auto& bin = explorationBins_[index];
        if (bin.colorBlocked || bin.grayBlocked || bin.sightings <= 0 ||
            rejected(bin.bestHeading)) continue;
        if (best < 0 || bin.sightings > explorationBins_[best].sightings ||
            (bin.sightings == explorationBins_[best].sightings &&
             bin.bestScore > explorationBins_[best].bestScore))
            best = index;
    }

    if (best >= 0)
    {
        explorationBaseHeading_ = explorationBins_[best].bestHeading;
    }
    else
    {
        // Sem pista parcial, usa o centro da maior faixa angular ainda não vetada.
        int bestRun = 0;
        for (int index = 0; index < static_cast<int>(explorationBins_.size()); ++index)
        {
            const double heading = index * config::kRescueExitScanDegrees;
            if (explorationBins_[index].colorBlocked || explorationBins_[index].grayBlocked ||
                rejected(heading)) continue;
            int run = 1;
            while (run < static_cast<int>(explorationBins_.size()))
            {
                const int next = (index + run) % static_cast<int>(explorationBins_.size());
                const double nextHeading = next * config::kRescueExitScanDegrees;
                if (explorationBins_[next].colorBlocked || explorationBins_[next].grayBlocked ||
                    rejected(nextHeading)) break;
                ++run;
            }
            if (run > bestRun)
            {
                bestRun = run;
                best = (index + (run - 1) / 2) % static_cast<int>(explorationBins_.size());
            }
        }
        if (best < 0) return false;
        explorationBaseHeading_ = bestRun == static_cast<int>(explorationBins_.size()) ?
            signedAngle(telemetry.yawZDeg) :
            signedAngle(best * config::kRescueExitScanDegrees);
    }

    const auto blocked = [&](int index) {
        return explorationBins_[index].colorBlocked || explorationBins_[index].grayBlocked;
    };
    int positiveBlocks = 0;
    int negativeBlocks = 0;
    for (int step = 1; step <= 2; ++step)
    {
        positiveBlocks += blocked(explorationBin(
            explorationBaseHeading_ + step * config::kRescueExitExplorationOffsetDegrees));
        negativeBlocks += blocked(explorationBin(
            explorationBaseHeading_ - step * config::kRescueExitExplorationOffsetDegrees));
    }
    explorationPositiveFirst_ = positiveBlocks <= negativeBlocks;
    explorationStarted_ = true;
    explorationOffsetIndex_ = 0;
    explorationAttempt_ = 0;
    explorationTotalCm_ = 0.0;
    return startNextExplorationTurn(telemetry, now);
}

bool RescueExitMission::startNextExplorationTurn(
    const Esp32TelemetrySnapshot& telemetry, Time now)
{
    if (explorationAttempt_ >= config::kRescueExitExplorationMaximumAttempts ||
        explorationTotalCm_ >= config::kRescueExitExplorationTotalCm)
        return false;

    while (explorationOffsetIndex_ < config::kRescueExitExplorationMaximumAttempts)
    {
        double offset = 0.0;
        if (explorationOffsetIndex_ == 1)
            offset = explorationPositiveFirst_ ? config::kRescueExitExplorationOffsetDegrees :
                                                  -config::kRescueExitExplorationOffsetDegrees;
        else if (explorationOffsetIndex_ == 2)
            offset = explorationPositiveFirst_ ? -config::kRescueExitExplorationOffsetDegrees :
                                                   config::kRescueExitExplorationOffsetDegrees;
        ++explorationOffsetIndex_;
        const double heading = signedAngle(explorationBaseHeading_ + offset);
        const auto& bin = explorationBins_[explorationBin(heading)];
        if (bin.colorBlocked || bin.grayBlocked || rejected(heading)) continue;

        explorationHeading_ = targetHeading_ = heading;
        ++explorationAttempt_;
        explorationAttemptTargetCm_ = std::min(
            config::kRescueExitExplorationAttemptCm,
            config::kRescueExitExplorationTotalCm - explorationTotalCm_);
        explorationAttemptLeftCm_ = explorationAttemptRightCm_ = 0.0;
        explorationLastProgressCm_ = 0.0;
        progressAt_ = now;
        const double turnDegrees = signedAngle(heading - telemetry.yawZDeg);
        if (std::abs(turnDegrees) <= config::kBallApproachStartToleranceDegrees)
        {
            startExplorationAdvance(now);
            return true;
        }
        if (!startTurn(turnDegrees, telemetry, now)) return false;
        phase_ = Phase::ExplorationTurning;
        return true;
    }
    return false;
}

void RescueExitMission::startExplorationAdvance(Time now)
{
    phase_ = Phase::Exploring;
    progressAt_ = now;
    explorationLastProgressCm_ = std::min(
        explorationAttemptLeftCm_, explorationAttemptRightCm_);
    candidateFrames_ = observedFrames_ = 0;
    waitingFrame_ = false;
}

void RescueExitMission::startExplorationRecovery(
    const char* reason, Time now, bool permanentlyRejectHeading)
{
    explorationBlockReason_ = reason;
    rejected_.push_back({explorationHeading_, permanentlyRejectHeading ?
        config::kRescueExitSilverRejectedToleranceDegrees :
        config::kRescueExitRejectedToleranceDegrees, permanentlyRejectHeading});
    explorationRecoveryTargetCm_ = std::min(
        config::kRescueExitExplorationRecoveryCm,
        std::min(explorationAttemptLeftCm_, explorationAttemptRightCm_));
    reverseTargetCm_ = explorationRecoveryTargetCm_;
    reversedCm_ = 0.0;
    movingExploration_ = false;
    movingForward_ = false;
    attempting_ = false;
    guidanceLatched_ = false;
    steeringNearLatched_ = false;
    resumeExplorationAfterReview_ = false;
    if (explorationRecoveryTargetCm_ > 0.0)
    {
        reverse_.start(explorationRecoveryTargetCm_,
                       config::kRescueExitExplorationPower, -1, now);
        phase_ = Phase::ExplorationBacking;
    }
    else
    {
        explorationSettleAt_ = now;
        phase_ = Phase::ExplorationSettling;
    }
}

RescueExitOutput RescueExitMission::updateExploration(
    const CameraLineSnapshot& bottom,
    const ForwardLineSnapshot& forward,
    const Esp32TelemetrySnapshot& telemetry,
    Time now,
    bool newForward,
    bool newBottom)
{
    const bool bottomReady = bottom.sourceFresh && bottom.silverClassifierFresh;
    if (bottomReady && bottom.courseMarkerConfirmed &&
        bottom.courseMarker == CourseMarker::Gray)
    {
        if (geometryActive_ && geometryCandidateActive_)
        {
            startGeometryReturn(
                "Entrada prata encontrada no corner", true, telemetry, now);
            return output("rescue_exit_corner_backing", explorationBlockReason_.c_str());
        }
        startExplorationRecovery("Entrada prata encontrada durante a exploração", now, true);
        return output("rescue_exit_exploration_blocked", explorationBlockReason_.c_str());
    }
    if (bottomReady && bottom.silverCandidateDetected)
    {
        progressAt_ = now;
        return output("rescue_exit_checking_silver",
                      "Parado: confirmando possível prata durante a exploração");
    }
    if (bottomReady && newBottom)
    {
        const bool validFusion = bottom.lineControlSource == "fusion" &&
            bottom.normalSteeringValid;
        if (!validFusion)
            acquisitionFrames_ = 0;
        else
            ++acquisitionFrames_;
        if (acquisitionFrames_ >= config::kRescueExitAcquisitionFrames)
        {
            // O Fusion pode guiar, mas a missão continua dona do controle até
            // atravessar uma janela física na qual a prata tem prioridade.
            phase_ = Phase::BottomValidating;
            bottomValidationStartLeft_ = telemetry.leftEncoderCount;
            bottomValidationStartRight_ = telemetry.rightEncoderCount;
            acquisitionFrames_ = 0;
            movingExploration_ = true;
            return output("rescue_exit_validating_bottom",
                          "Fusion inferior estável; procurando prata antes do handoff",
                          bottom.lineFollowerLeftPower,
                          bottom.lineFollowerRightPower);
        }
        if (validFusion)
        {
            movingExploration_ = true;
            return output("rescue_exit_checking_bottom",
                          "Mantendo a saída ativa enquanto a CAM0 verifica prata",
                          config::kRescueExitExplorationPower,
                          config::kRescueExitExplorationPower);
        }
    }
    if (forward.cameraObscured)
    {
        if (geometryActive_ && geometryCandidateActive_)
        {
            if (startCornerCollisionRecovery(telemetry, now))
                return output("rescue_exit_corner_recovery_backing",
                              "Parede detectada; preservando o corner e recuando 15 cm");
            startGeometryReturn(
                "Corner bloqueado pela proteção visual", false, telemetry, now);
            return output("rescue_exit_corner_backing", explorationBlockReason_.c_str());
        }
        startExplorationRecovery("CAM1 obstruída durante a exploração", now);
        return output("rescue_exit_exploration_blocked", explorationBlockReason_.c_str());
    }

    if (newForward && geometryActive_ && geometryCandidateActive_)
    {
        const auto best = std::max_element(
            forward.exitCandidates.begin(), forward.exitCandidates.end(),
            [](const auto& left, const auto& right) {
                return (left.visible && left.guidanceValid ? left.score : -1.0) <
                       (right.visible && right.guidanceValid ? right.score : -1.0);
            });
        if (best != forward.exitCandidates.end() && best->visible &&
            best->guidanceValid && !best->blockedByColor &&
            !best->grayNoiseLikely)
        {
            // Uma amostra ao entrar no corner basta para escolher o lado de
            // uma eventual recuperação; ela nunca corrige o avanço normal.
            lastGuidanceAngleDegrees_ = best->guidanceAngleDegrees;
            lastGuidanceAt_ = now;
            guidanceLatched_ = true;
        }
    }
    else if (newForward)
    {
        const bool candidateSeen = std::any_of(
            forward.exitCandidates.begin(), forward.exitCandidates.end(),
            [&](const auto& candidate) {
                return candidate.visible && candidate.guidanceValid &&
                       !candidate.blockedByColor && !candidate.grayNoiseLikely &&
                       !rejected(telemetry.yawZDeg + candidate.txDegrees);
            });
        if (candidateSeen)
        {
            resumeExplorationAfterReview_ = true;
            resumeExplorationTurn_ = false;
            phase_ = Phase::Searching;
            candidateFrames_ = observedFrames_ = 0;
            lastSeenAt_ = now;
            return updateSearch(forward, telemetry, now, true);
        }
    }

    const double progress = std::min(
        explorationAttemptLeftCm_, explorationAttemptRightCm_);
    if (progress >= explorationLastProgressCm_ +
            config::kDriveDistanceMinimumProgressCounts /
                config::kEncoderCountsPerCentimeter)
    {
        explorationLastProgressCm_ = progress;
        progressAt_ = now;
    }
    if (now - progressAt_ >=
        std::chrono::milliseconds(config::kRescueDistanceStallTimeoutMs))
    {
        if (geometryActive_ && geometryCandidateActive_)
        {
            if (startCornerCollisionRecovery(telemetry, now))
                return output("rescue_exit_corner_recovery_backing",
                              "Sem progresso; preservando o corner e recuando 15 cm");
            startGeometryReturn(
                "Corner bloqueado por falta de progresso", false, telemetry, now);
            return output("rescue_exit_corner_backing", explorationBlockReason_.c_str());
        }
        startExplorationRecovery("Rodas sem progresso durante a exploração", now);
        return output("rescue_exit_exploration_blocked", explorationBlockReason_.c_str());
    }
    if (progress >= explorationAttemptTargetCm_ ||
        explorationTotalCm_ >= config::kRescueExitExplorationTotalCm)
    {
        if (geometryActive_ && geometryCandidateActive_)
        {
            const bool forwardFusionFresh = guidanceLatched_ &&
                now - lastGuidanceAt_ <= std::chrono::milliseconds(
                    config::kRescueExitGuidanceHoldMs);
            if (forwardFusionFresh)
            {
                // Sessenta centímetros limitam somente a exploração sem visão.
                // Com a saída ainda visível, a CAM1 mantém o avanço reto até
                // a CAM0 assumir ou a proteção visual interromper o movimento.
                movingExploration_ = true;
                return output("rescue_exit_corner_exploring",
                              "Fusion frontal recente; avançando além de 60 cm até a CAM0",
                              config::kRescueExitExplorationPower,
                              config::kRescueExitExplorationPower);
            }
            startGeometryReturn(
                "Corner alcançado sem linha válida", false, telemetry, now);
            return output("rescue_exit_corner_backing", explorationBlockReason_.c_str());
        }
        explorationSettleAt_ = now;
        phase_ = Phase::ExplorationSettling;
        return output("rescue_exit_exploration_settling",
                      "Avanço exploratório concluído; reavaliando a CAM1");
    }

    movingExploration_ = true;
    return output(geometryCandidateActive_ ?
                      "rescue_exit_corner_exploring" : "rescue_exit_exploring",
                  geometryCandidateActive_ ?
                      "Avançando reto no corner; aguardando Fusion inferior" :
                      "Avançando até 30 cm enquanto procura Fusion",
                  config::kRescueExitExplorationPower,
                  config::kRescueExitExplorationPower);
}

void RescueExitMission::reset() { *this = RescueExitMission{}; }

void RescueExitMission::setKnownEntryHeading(double headingDegrees)
{
    knownEntryHeadingValid_ = std::isfinite(headingDegrees);
    if (knownEntryHeadingValid_)
        knownEntryHeadingDegrees_ = signedAngle(headingDegrees);
}

bool RescueExitMission::requiresRescueZoneDetection() const
{
    if (!started_) return true;
    if (!geometryActive_ || geometryCandidateActive_) return false;
    return phase_ == Phase::GeometrySettling ||
           phase_ == Phase::GeometryTurning ||
           phase_ == Phase::GeometryReturnTurning;
}

bool RescueExitMission::rejected(double heading) const
{
    return std::any_of(rejected_.begin(), rejected_.end(), [&](const auto& item) {
        return ImuTurnController::angularDistanceDegrees(heading, item.heading) <=
            item.toleranceDegrees;
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

void RescueExitMission::reject(bool permanent, const char* reason,
    const Esp32TelemetrySnapshot& telemetry, Time now, bool retrySame)
{
    if (geometryActive_ && geometryCandidateActive_)
    {
        startGeometryReturn(reason, permanent, telemetry, now);
        return;
    }
    if (retrySame)
    {
        retryPending_ = true;
        retryUsed_ = true;
        retryHeading_ = trackingHeading_;
    }
    else
    {
        rejected_.push_back({permanent ? trackingHeading_ : targetHeading_,
            permanent ? config::kRescueExitSilverRejectedToleranceDegrees :
                        config::kRescueExitRejectedToleranceDegrees,
            permanent || round_ == 2});
        retryPending_ = false;
    }
    failure_ = reason;
    // As duas rodas avançam; o menor encoder limita o recuo à distância medida.
    const double traveledCm = std::min(advanceLeftCm_, advanceRightCm_);
    reverseTargetCm_ = std::min(config::kRescueExitReverseMaximumCm, traveledCm);
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
    bottomMissing_ = false;
    guidanceLatched_ = false;
    steeringNearLatched_ = false;
    candidateFrames_ = observedFrames_ = acquisitionFrames_ = 0;
    waitingFrame_ = true;
    observedAt_ = now;
}

RescueExitOutput RescueExitMission::fail(const char* reason)
{
    phase_ = Phase::Failed;
    failure_ = reason;
    movingForward_ = false;
    movingExploration_ = false;
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
    result.status.exitExplorationHeadingDegrees = explorationHeading_;
    result.status.exitExplorationAttempt = explorationAttempt_;
    result.status.exitExplorationAdvanceCm = explorationTotalCm_;
    result.status.exitExplorationBlockReason = explorationBlockReason_;
    result.status.targetDistanceCm = reverseTargetCm_;
    std::ostringstream headings;
    for (const auto& item : rejected_)
        headings << item.heading << " ±" << item.toleranceDegrees
                 << (item.permanent ? " permanente; " : " temporário; ");
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
    const ForwardLineSnapshot& forward, const RescueZoneSnapshot& zones,
    const Esp32TelemetrySnapshot& telemetry,
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
        lastForwardSequence_ = forward.sequence;
        lastBottomSequence_ = bottom.lineSequence;
        lastZoneSequence_ = zones.sequence;
        lastLeft_ = telemetry.leftEncoderCount;
        lastRight_ = telemetry.rightEncoderCount;
        lastUptime_ = telemetry.esp32UptimeMs;
    }
    if (now - startedAt_ >= std::chrono::milliseconds(config::kRescueExitTotalTimeoutMs))
        return fail("Tempo total da busca excedido");

    // O timeout frontal global de 125 ms é menor que um frame a 8 FPS. A
    // saída usa uma janela própria, mas mantém todos os gates de movimento.
    const bool forwardAgeReady = std::isfinite(forward.ageMs) && forward.ageMs >= 0.0 &&
        forward.ageMs <= config::kRescueExitForwardStatusTimeoutMs;
    // Durante a aproximação, o próprio hold de 300 ms para e o limite de
    // 500 ms rejeita a rota. Assim, a janela de transporte não apaga o alvo
    // antes que essa recuperação controlada possa terminar.
    const bool forwardReady = (forwardAgeReady || phase_ == Phase::Approaching ||
                               phase_ == Phase::BottomValidating) &&
        forward.exitAnalysisActive && forward.exitRunSequence == runSequence;
    const bool ready = telemetry.readyForOperation() && ImuTurnController::imuReady(telemetry) &&
        EncoderDistanceController::encodersReady(telemetry) && forwardReady;
    if (!ready)
    {
        movingForward_ = false;
        progressAt_ = now;
        candidateFrames_ = acquisitionFrames_ = observedFrames_ = 0;
        guidanceLatched_ = false;
        steeringNearLatched_ = false;
        movingExploration_ = false;
        lastLeft_ = telemetry.leftEncoderCount;
        lastRight_ = telemetry.rightEncoderCount;
        if (!sensorsMissing_) { sensorsMissing_ = true; missingSince_ = now; }
        if (now - missingSince_ >= std::chrono::milliseconds(config::kRescueExitSensorTimeoutMs))
            return fail("CAM1, IMU ou encoders indisponíveis");
        return output("rescue_exit_waiting_sensors", "Parado: aguardando CAM1, IMU e encoders");
    }
    sensorsMissing_ = false;
    if (geometryActive_ && !geometryReferenceSaved_)
    {
        geometryReferenceSaved_ = true;
        geometryReferenceHeading_ = signedAngle(telemetry.yawZDeg);
        geometryPhaseAt_ = now;
        lastZoneSequence_ = zones.sequence;
        // A rotina começa depois da ré do depósito vermelho; esse corner já é proibido.
        rejected_.push_back({geometryReferenceHeading_,
            config::kRescueExitCornerGeometryToleranceDegrees, true});
        phase_ = Phase::GeometrySettling;
    }
    if (telemetry.esp32UptimeMs < lastUptime_) return fail("ESP32 reiniciou durante a busca");
    lastUptime_ = telemetry.esp32UptimeMs;
    const double leftDeltaCm = std::abs(static_cast<double>(
        telemetry.leftEncoderCount - lastLeft_)) / config::kEncoderCountsPerCentimeter;
    const double rightDeltaCm = std::abs(static_cast<double>(
        telemetry.rightEncoderCount - lastRight_)) / config::kEncoderCountsPerCentimeter;
    if (movingForward_)
    {
        advanceLeftCm_ += leftDeltaCm;
        advanceRightCm_ += rightDeltaCm;
    }
    if (movingExploration_)
    {
        explorationAttemptLeftCm_ += leftDeltaCm;
        explorationAttemptRightCm_ += rightDeltaCm;
        explorationTotalCm_ += std::min(leftDeltaCm, rightDeltaCm);
    }
    lastLeft_ = telemetry.leftEncoderCount;
    lastRight_ = telemetry.rightEncoderCount;
    movingForward_ = false;
    movingExploration_ = false;
    const bool newForward = forward.timestamp > lastForwardTimestamp_ && forward.sequence != lastForwardSequence_;
    const bool newBottom = bottom.lineTimestamp > lastBottomTimestamp_ && bottom.lineSequence != lastBottomSequence_;
    if (newForward) { lastForwardTimestamp_ = forward.timestamp; lastForwardSequence_ = forward.sequence; }
    if (newBottom) { lastBottomTimestamp_ = bottom.lineTimestamp; lastBottomSequence_ = bottom.lineSequence; }

    // A prata tem prioridade em qualquer fase da saída, inclusive durante
    // retorno e recuperação. Antes, esses estados podiam atravessar uma faixa
    // já confirmada pela CAM0 sem encaminhar a decisão para a geometria.
    const bool silverReady = bottom.sourceFresh && bottom.silverClassifierFresh;
    const bool silverConfirmed = silverReady && bottom.courseMarkerConfirmed &&
        bottom.courseMarker == CourseMarker::Gray;
    if (!silverReady || (!bottom.silverCandidateDetected && !silverConfirmed))
        silverBlockLatched_ = false;
    if (silverConfirmed)
    {
        acquisitionFrames_ = 0;
        if (!silverBlockLatched_)
        {
            const bool returningFromCorner = phase_ == Phase::GeometryBacking ||
                phase_ == Phase::GeometryReturnTurning ||
                phase_ == Phase::CornerRecoveryBacking ||
                phase_ == Phase::CornerRecoveryTurningAway ||
                phase_ == Phase::CornerRecoveryTurningBack;
            const double silverHeading =
                (geometryCandidateActive_ || returningFromCorner) ?
                    explorationHeading_ : signedAngle(telemetry.yawZDeg);
            rejected_.push_back({silverHeading,
                config::kRescueExitSilverRejectedToleranceDegrees, true});
            silverBlockLatched_ = true;
        }

        if (geometryCandidateActive_ &&
            (phase_ == Phase::Exploring || phase_ == Phase::Approaching ||
             phase_ == Phase::CornerRecoveryAdvancing ||
             phase_ == Phase::BottomValidating))
        {
            startGeometryReturn(
                "Entrada prata confirmada globalmente pela CAM0", true,
                telemetry, now);
            return output("rescue_exit_corner_backing",
                          explorationBlockReason_.c_str());
        }
        if (phase_ == Phase::BottomValidating)
        {
            startExplorationRecovery(
                "Entrada prata confirmada durante a validação da CAM0", now, true);
            return output("rescue_exit_exploration_blocked",
                          explorationBlockReason_.c_str());
        }
        return output("rescue_exit_checking_silver",
                      "Parado: entrada prata confirmada pela CAM0");
    }
    if (silverReady && bottom.silverCandidateDetected)
    {
        progressAt_ = now;
        acquisitionFrames_ = 0;
        return output("rescue_exit_checking_silver",
                      "Parado: confirmando possível prata pela CAM0");
    }

    if (phase_ == Phase::BottomValidating)
    {
        // Sem classificador recente, o Fusion não recebe autoridade para andar.
        // Isso preserva a prioridade da prata mesmo com a linha bem definida.
        const bool validBottomFusion = silverReady &&
            bottom.lineControlSource == "fusion" && bottom.normalSteeringValid;
        if (!validBottomFusion)
        {
            if (!bottomMissing_)
            {
                bottomMissing_ = true;
                bottomMissingSince_ = now;
            }
            if (now - bottomMissingSince_ >=
                std::chrono::milliseconds(config::kRescueExitSensorTimeoutMs))
                return fail("CAM0, Fusion inferior ou classificador de prata indisponível");
            return output("rescue_exit_validating_bottom",
                          "Parado: aguardando CAM0 e classificador de prata");
        }
        bottomMissing_ = false;

        if (forward.cameraObscured)
        {
            if (geometryActive_ && geometryCandidateActive_ &&
                startCornerCollisionRecovery(telemetry, now))
                return output("rescue_exit_corner_recovery_backing",
                              "Parede detectada durante a validação da CAM0");
            return fail("Câmera frontal obstruída durante a validação da saída");
        }

        const double leftValidationCm = std::abs(static_cast<double>(
            telemetry.leftEncoderCount - bottomValidationStartLeft_)) /
            config::kEncoderCountsPerCentimeter;
        const double rightValidationCm = std::abs(static_cast<double>(
            telemetry.rightEncoderCount - bottomValidationStartRight_)) /
            config::kEncoderCountsPerCentimeter;
        const double validationCm = std::min(leftValidationCm, rightValidationCm);
        if (validationCm >= config::kRescueExitBottomValidationAdvanceCm)
        {
            phase_ = Phase::Completed;
            return output("rescue_exit_acquired",
                          "Janela de prata concluída; retomando o percurso",
                          bottom.lineFollowerLeftPower,
                          bottom.lineFollowerRightPower);
        }

        if (geometryCandidateActive_)
            movingExploration_ = true;
        else
            movingForward_ = true;
        return output("rescue_exit_validating_bottom",
                      "CAM0 guiando; prata continua com prioridade",
                      bottom.lineFollowerLeftPower,
                      bottom.lineFollowerRightPower);
    }

    if (newForward && !forward.cameraObscured &&
        phase_ != Phase::Approaching && phase_ != Phase::Backing &&
        phase_ != Phase::ExplorationBacking)
        recordExplorationEvidence(forward, telemetry);

    if (phase_ == Phase::CornerRecoveryBacking)
    {
        const auto movement = cornerRecoveryDistance_.update(
            telemetry, now, "rescue_exit_corner_recovery_backing",
            "Recuando 15 cm para liberar a parede do corner");
        reversedCm_ = movement.status.averageDistanceCm;
        if (movement.failed)
            return fail("Falha na ré de recuperação do corner");
        if (movement.completed)
        {
            explorationAttemptLeftCm_ = std::max(
                0.0, explorationAttemptLeftCm_ - movement.status.leftDistanceCm);
            explorationAttemptRightCm_ = std::max(
                0.0, explorationAttemptRightCm_ - movement.status.rightDistanceCm);
            explorationTotalCm_ = std::max(
                0.0, explorationTotalCm_ -
                    std::min(movement.status.leftDistanceCm,
                             movement.status.rightDistanceCm));
            explorationLastProgressCm_ = std::min(
                explorationAttemptLeftCm_, explorationAttemptRightCm_);
            const double turnDegrees = cornerRecoveryTurnSign_ *
                config::kRescueExitCornerCollisionTurnDegrees;
            if (!startTurn(turnDegrees, telemetry, now))
                return fail("Não foi possível iniciar o desvio do corner");
            phase_ = Phase::CornerRecoveryTurningAway;
        }
        return output("rescue_exit_corner_recovery_backing",
                      "Criando espaço para tentar novamente o mesmo corner",
                      movement.leftPower, movement.rightPower);
    }

    if (phase_ == Phase::CornerRecoveryTurningAway)
    {
        const auto turning = turn_.update(telemetry, now);
        if (turning.result == ImuTurnResult::Failed)
            return fail("Falha no primeiro giro da recuperação do corner");
        if (turning.result == ImuTurnResult::Completed)
        {
            cornerRecoveryDistance_.start(
                config::kRescueExitCornerCollisionClearanceCm,
                config::kRescueExitExplorationPower, 1, now);
            phase_ = Phase::CornerRecoveryAdvancing;
            geometryPhaseAt_ = now;
        }
        return output("rescue_exit_corner_recovery_turning",
                      "Girando para o trecho indicado pela CAM1",
                      turning.leftPower, turning.rightPower);
    }

    if (phase_ == Phase::CornerRecoveryAdvancing)
    {
        if (forward.cameraObscured)
        {
            if (now - geometryPhaseAt_ <
                std::chrono::milliseconds(
                    config::kRescueExitForwardStatusTimeoutMs))
                return output("rescue_exit_corner_recovery_advancing",
                              "Parado: aguardando imagem após o desvio");
            startGeometryReturn(
                "Desvio do corner continuou obstruído", false, telemetry, now);
            return output("rescue_exit_corner_backing",
                          explorationBlockReason_.c_str());
        }
        const auto movement = cornerRecoveryDistance_.update(
            telemetry, now, "rescue_exit_corner_recovery_advancing",
            "Avançando 10 cm para sair da parede");
        if (movement.failed)
            return fail("Falha no avanço de recuperação do corner");
        if (movement.completed)
        {
            explorationAttemptLeftCm_ += movement.status.leftDistanceCm;
            explorationAttemptRightCm_ += movement.status.rightDistanceCm;
            explorationTotalCm_ += std::min(
                movement.status.leftDistanceCm,
                movement.status.rightDistanceCm);
            explorationLastProgressCm_ = std::min(
                explorationAttemptLeftCm_, explorationAttemptRightCm_);
            const double turnDegrees = signedAngle(
                cornerRecoveryReturnHeading_ - telemetry.yawZDeg);
            if (std::abs(turnDegrees) <=
                config::kBallApproachStartToleranceDegrees)
            {
                phase_ = Phase::Exploring;
                progressAt_ = now;
                movingExploration_ = true;
                return output("rescue_exit_corner_exploring",
                              "Parede contornada; retomando o mesmo corner",
                              config::kRescueExitExplorationPower,
                              config::kRescueExitExplorationPower);
            }
            if (!startTurn(turnDegrees, telemetry, now))
                return fail("Não foi possível restaurar o heading do corner");
            phase_ = Phase::CornerRecoveryTurningBack;
        }
        return output("rescue_exit_corner_recovery_advancing",
                      "Saindo da parede antes de restaurar o heading",
                      movement.leftPower, movement.rightPower);
    }

    if (phase_ == Phase::CornerRecoveryTurningBack)
    {
        const auto turning = turn_.update(telemetry, now);
        if (turning.result == ImuTurnResult::Failed)
            return fail("Falha ao restaurar a abertura do corner");
        if (turning.result == ImuTurnResult::Completed)
        {
            phase_ = Phase::Exploring;
            progressAt_ = now;
            movingExploration_ = true;
            return output("rescue_exit_corner_exploring",
                          "Heading restaurado; tentando novamente o mesmo corner",
                          config::kRescueExitExplorationPower,
                          config::kRescueExitExplorationPower);
        }
        return output("rescue_exit_corner_recovery_returning",
                      "Voltando a apontar para a abertura do corner",
                      turning.leftPower, turning.rightPower);
    }

    if (phase_ == Phase::GeometryBacking)
    {
        const auto movement = reverse_.update(
            telemetry, now, "rescue_exit_corner_backing",
            "Retornando ao ponto central pela distância medida");
        reversedCm_ = movement.status.averageDistanceCm;
        if (movement.failed) return fail(movement.status.action.c_str());
        if (movement.completed)
        {
            const double turnDegrees = signedAngle(
                geometryReferenceHeading_ - telemetry.yawZDeg);
            geometryReturnTurnStarted_ =
                std::abs(turnDegrees) > config::kBallApproachStartToleranceDegrees &&
                startTurn(turnDegrees, telemetry, now);
            phase_ = Phase::GeometryReturnTurning;
        }
        return output("rescue_exit_corner_backing",
                      "Voltando à origem antes do outro corner",
                      movement.leftPower, movement.rightPower);
    }

    if (phase_ == Phase::GeometryReturnTurning)
    {
        if (geometryReturnTurnStarted_)
        {
            const auto turning = turn_.update(telemetry, now);
            if (turning.result == ImuTurnResult::Failed)
            {
                if (turning.phase != "turn_correction_failed")
                    return fail(turning.action.c_str());
                return restartGeometry(telemetry, now,
                    "Retorno perdeu a referência; repetindo somente a geometria");
            }
            if (turning.result != ImuTurnResult::Completed)
                return output("rescue_exit_corner_return_turning",
                              "Restaurando o yaw salvo no ponto central",
                              turning.leftPower, turning.rightPower);
            geometryReturnTurnStarted_ = false;
        }
        if (geometryResumeProbesAfterReturn_)
        {
            geometryResumeProbesAfterReturn_ = false;
            if (startNextGeometryProbe(telemetry, now))
                return output(phase_ == Phase::GeometryTurning ?
                                  "rescue_exit_geometry_turning" :
                                  "rescue_exit_geometry_settling",
                              "Origem restaurada; verificando o próximo corner");
            return restartGeometry(telemetry, now,
                "Corners verificados; repetindo somente a geometria");
        }
        if (startGeometryCandidate(telemetry, now))
            return output(phase_ == Phase::Exploring ?
                              "rescue_exit_corner_exploring" :
                              "rescue_exit_corner_turning",
                          "Origem restaurada; seguindo para o outro corner",
                          phase_ == Phase::Exploring ?
                              config::kRescueExitExplorationPower : 0.0,
                          phase_ == Phase::Exploring ?
                              config::kRescueExitExplorationPower : 0.0);
        return restartGeometry(telemetry, now,
            "Corners candidatos testados; repetindo somente a geometria");
    }

    if (geometryActive_ && !geometryCandidateActive_ &&
        (phase_ == Phase::GeometrySettling || phase_ == Phase::GeometryTurning))
    {
        if (forward.cameraObscured)
        {
            turn_.reset();
            phase_ = Phase::GeometrySettling;
            geometryPhaseAt_ = now;
            return output("rescue_exit_geometry_obscured",
                          "Parado: câmera frontal obstruída durante a geometria");
        }
        // A geometria termina primeiro a classificação vermelho/verde. Uma
        // rota preta não pode cancelar essa checagem no meio do giro.
        return updateGeometry(zones, telemetry, now);
    }

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

    if (phase_ == Phase::ExplorationBacking)
    {
        const auto movement = reverse_.update(
            telemetry, now, "rescue_exit_exploration_backing",
            "Recuando da direção bloqueada durante a exploração");
        reversedCm_ = movement.status.averageDistanceCm;
        if (movement.failed) return fail(movement.status.action.c_str());
        if (movement.completed)
        {
            explorationSettleAt_ = now;
            phase_ = Phase::ExplorationSettling;
        }
        return output("rescue_exit_exploration_backing",
            "Recuando até 8 cm antes do próximo desvio",
            movement.leftPower, movement.rightPower);
    }

    if (phase_ == Phase::ExplorationSettling)
    {
        if (now - explorationSettleAt_ <
            std::chrono::milliseconds(config::kRescueDistanceSettleMs))
            return output("rescue_exit_exploration_settling",
                          "Parado: estabilizando antes da próxima exploração");

        // O robô mudou de posição; cores e cinza serão medidos novamente.
        explorationBins_ = {};
        if (newForward && !forward.cameraObscured)
        {
            recordExplorationEvidence(forward, telemetry);
            const bool candidateSeen = std::any_of(
                forward.exitCandidates.begin(), forward.exitCandidates.end(),
                [&](const auto& candidate) {
                    return candidate.visible && candidate.guidanceValid &&
                           !candidate.blockedByColor && !candidate.grayNoiseLikely &&
                           !rejected(telemetry.yawZDeg + candidate.txDegrees);
                });
            if (candidateSeen)
            {
                resumeExplorationAfterReview_ = true;
                resumeExplorationTurn_ = false;
                phase_ = Phase::Searching;
                candidateFrames_ = observedFrames_ = 0;
                return updateSearch(forward, telemetry, now, true);
            }
        }
        if (startNextExplorationTurn(telemetry, now))
        {
            if (phase_ == Phase::Exploring)
            {
                movingExploration_ = true;
                return output("rescue_exit_exploring",
                              "Iniciando a próxima tentativa exploratória",
                              config::kRescueExitExplorationPower,
                              config::kRescueExitExplorationPower);
            }
            return output("rescue_exit_exploration_turning",
                          "Girando para a próxima direção possível");
        }

        finalScan_ = true;
        round_ = 2;
        scanSteps_ = 0;
        phase_ = Phase::Searching;
        observedAt_ = now;
        waitingFrame_ = true;
        candidateFrames_ = observedFrames_ = 0;
        return output("rescue_exit_final_scan",
                      "Exploração concluída; iniciando a última varredura");
    }

    if (forward.cameraObscured && !attempting_ &&
        phase_ != Phase::Exploring && phase_ != Phase::ExplorationTurning)
    {
        turn_.reset();
        phase_ = Phase::Searching;
        candidateFrames_ = observedFrames_ = 0;
        resumeExplorationAfterReview_ = false;
        resumeExplorationTurn_ = false;
        return output("rescue_exit_obscured", "Parado: câmera frontal obstruída");
    }

    if (attempting_)
    {
        const bool bottomReady = bottom.sourceFresh && bottom.silverClassifierFresh;
        if (nearLatched_ && !bottomReady)
        {
            movingForward_ = false;
            progressAt_ = now;
            acquisitionFrames_ = 0;
            if (!bottomMissing_) { bottomMissing_ = true; bottomMissingSince_ = now; }
            if (now - bottomMissingSince_ >=
                std::chrono::milliseconds(config::kRescueExitSensorTimeoutMs))
                return fail("CAM0 ou classificador de prata indisponível em NEAR");
            return output("rescue_exit_waiting_bottom", "Parado: aguardando CAM0 e prata em NEAR");
        }
        else bottomMissing_ = false;
        // Prata confirmada prevalece sobre qualquer geometria preta da CAM0.
        if (bottomReady && bottom.courseMarkerConfirmed && bottom.courseMarker == CourseMarker::Gray)
        {
            reject(true, "Entrada prata confirmada", telemetry, now);
            return output((phase_ == Phase::GeometryBacking ||
                           phase_ == Phase::GeometryReturnTurning) ?
                              "rescue_exit_corner_backing" : "rescue_exit_rejected",
                          (phase_ == Phase::GeometryBacking ||
                           phase_ == Phase::GeometryReturnTurning) ?
                              explorationBlockReason_.c_str() : failure_.c_str());
        }
        if (now - attemptAt_ >= std::chrono::milliseconds(config::kRescueExitApproachTimeoutMs))
        {
            reject(false, "Aproximação excedeu o tempo limite", telemetry, now);
            return output((phase_ == Phase::GeometryBacking ||
                           phase_ == Phase::GeometryReturnTurning) ?
                              "rescue_exit_corner_backing" : "rescue_exit_rejected",
                          (phase_ == Phase::GeometryBacking ||
                           phase_ == Phase::GeometryReturnTurning) ?
                              explorationBlockReason_.c_str() : failure_.c_str());
        }
        if (forward.cameraObscured)
        {
            if (geometryActive_ && geometryCandidateActive_)
            {
                if (startCornerCollisionRecovery(telemetry, now))
                    return output("rescue_exit_corner_recovery_backing",
                                  "Parede detectada; tentando recuperar o mesmo corner");
                startGeometryReturn(
                    "Proteção visual interrompeu o corner", false, telemetry, now);
                return output("rescue_exit_corner_backing",
                              explorationBlockReason_.c_str());
            }
            if (!explorationStarted_)
            {
                explorationStarted_ = true;
                explorationBaseHeading_ = trackingHeading_;
                explorationOffsetIndex_ = 1;
                explorationAttempt_ = 1;
                explorationPositiveFirst_ = true;
            }
            explorationHeading_ = trackingHeading_;
            explorationAttemptLeftCm_ = advanceLeftCm_;
            explorationAttemptRightCm_ = advanceRightCm_;
            startExplorationRecovery("CAM1 obstruída durante a aproximação", now);
            return output("rescue_exit_exploration_blocked", explorationBlockReason_.c_str());
        }
        if (bottomReady && bottom.silverCandidateDetected)
        {
            progressAt_ = now;
            acquisitionFrames_ = 0;
            return output("rescue_exit_checking_silver", "Parado: confirmando possível prata");
        }
        if (phase_ == Phase::Approaching && bottomReady && newBottom)
        {
            // O próprio Fusion já valida a geometria. A prata é verificada
            // antes, portanto o primeiro frame novo pode assumir os motores.
            const bool valid = bottom.lineControlSource == "fusion" && bottom.normalSteeringValid;
            if (!valid) acquisitionFrames_ = 0;
            else ++acquisitionFrames_;
            if (acquisitionFrames_ >= config::kRescueExitAcquisitionFrames)
            {
                phase_ = Phase::BottomValidating;
                bottomValidationStartLeft_ = telemetry.leftEncoderCount;
                bottomValidationStartRight_ = telemetry.rightEncoderCount;
                acquisitionFrames_ = 0;
                movingForward_ = true;
                return output("rescue_exit_validating_bottom",
                    "Fusion inferior estável; procurando prata antes do handoff",
                    bottom.lineFollowerLeftPower, bottom.lineFollowerRightPower);
            }
        }
    }

    if (phase_ == Phase::Turning || phase_ == Phase::ExplorationTurning)
    {
        if (newForward && !forward.cameraObscured)
        {
            const bool candidateSeen = std::any_of(
                forward.exitCandidates.begin(), forward.exitCandidates.end(),
                [&](const auto& candidate) {
                    return candidate.visible && candidate.guidanceValid &&
                           !rejected(telemetry.yawZDeg + candidate.txDegrees);
                });
            if (candidateSeen)
            {
                // A primeira evidência freia a varredura antes dos três frames de confirmação.
                turn_.reset();
                resumeExplorationAfterReview_ = phase_ == Phase::ExplorationTurning;
                resumeExplorationTurn_ = resumeExplorationAfterReview_;
                phase_ = Phase::Searching;
                waitingFrame_ = false;
                candidateFrames_ = observedFrames_ = 0;
                return updateSearch(forward, telemetry, now, true);
            }
        }
        if (phase_ == Phase::ExplorationTurning && forward.cameraObscured)
        {
            turn_.reset();
            if (geometryActive_ && geometryCandidateActive_)
            {
                startGeometryReturn(
                    "Proteção visual bloqueou o giro para o corner",
                    false, telemetry, now);
                return output("rescue_exit_corner_returning",
                              explorationBlockReason_.c_str());
            }
            startExplorationRecovery("CAM1 obstruída durante o giro exploratório", now);
            return output("rescue_exit_exploration_blocked", explorationBlockReason_.c_str());
        }
        const auto turning = turn_.update(telemetry, now);
        if (turning.result == ImuTurnResult::Failed)
        {
            if (turning.phase != "turn_correction_failed") return fail(turning.action.c_str());
            if (phase_ == Phase::ExplorationTurning)
            {
                if (geometryActive_ && geometryCandidateActive_)
                {
                    startGeometryReturn(
                        "Falha no giro para o corner", false, telemetry, now);
                    return output("rescue_exit_corner_returning",
                                  explorationBlockReason_.c_str());
                }
                startExplorationRecovery("Falha no giro exploratório", now);
                return output("rescue_exit_exploration_blocked", explorationBlockReason_.c_str());
            }
            phase_ = Phase::Searching;
            observedAt_ = now;
            waitingFrame_ = true;
            candidateFrames_ = observedFrames_ = 0;
            return output("rescue_exit_searching", turning.action.c_str());
        }
        if (turning.result == ImuTurnResult::Completed)
        {
            if (phase_ == Phase::ExplorationTurning)
            {
                startExplorationAdvance(now);
                movingExploration_ = true;
                return output(geometryCandidateActive_ ?
                                  "rescue_exit_corner_exploring" :
                                  "rescue_exit_exploring",
                              geometryCandidateActive_ ?
                                  "Corner alcançado; avançando reto até a CAM0" :
                                  "Direção alcançada; iniciando avanço exploratório",
                              config::kRescueExitExplorationPower,
                              config::kRescueExitExplorationPower);
            }
            ++scanSteps_;
            phase_ = Phase::Searching;
            observedAt_ = progressAt_ = lastSeenAt_ = now;
            waitingFrame_ = true;
        }
        return output(phase_ == Phase::ExplorationTurning ?
                          "rescue_exit_exploration_turning" : "rescue_exit_turning",
                      phase_ == Phase::ExplorationTurning ?
                          "Girando para explorar a direção mais provável" :
                          "Girando com a potência validada do resgate",
                      turning.leftPower, turning.rightPower);
    }

    if (phase_ == Phase::Exploring)
        return updateExploration(bottom, forward, telemetry, now, newForward, newBottom);

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
    if (retryPending_ && now - observedAt_ >=
            std::chrono::milliseconds(config::kRescueExitReacquisitionWaitMs))
    {
        rejected_.push_back({retryHeading_, config::kRescueExitRejectedToleranceDegrees, false});
        retryPending_ = false;
    }
    const bool reviewingCandidate = candidateFrames_ > 0 &&
        now - lastSeenAt_ < std::chrono::milliseconds(config::kRescueExitCandidateReviewMs);
    int best = -1;
    for (int index = 0; index < static_cast<int>(forward.exitCandidates.size()); ++index)
    {
        const auto& candidate = forward.exitCandidates[index];
        const double heading = telemetry.yawZDeg + candidate.txDegrees;
        if (!candidate.visible || !candidate.guidanceValid || forward.cameraObscured ||
            rejected(heading) ||
            (reviewingCandidate && ImuTurnController::angularDistanceDegrees(
                heading, confirmationHeading_) >
                    config::kRescueExitCandidateHeadingToleranceDegrees) ||
            (retryPending_ && ImuTurnController::angularDistanceDegrees(
                heading, retryHeading_) > config::kRescueExitRejectedToleranceDegrees)) continue;
        if (best < 0 || candidate.score > forward.exitCandidates[best].score ||
            (candidate.score == forward.exitCandidates[best].score && std::abs(candidate.txDegrees) < std::abs(forward.exitCandidates[best].txDegrees)))
            best = index;
    }
    if (retryPending_ && best < 0)
        return output("rescue_exit_searching", "Parado: procurando novamente a mesma faixa");
    if (best >= 0)
    {
        observedFrames_ = 0;
        const auto& candidate = forward.exitCandidates[best];
        const double heading = signedAngle(telemetry.yawZDeg + candidate.txDegrees);
        if (candidateFrames_ && ImuTurnController::angularDistanceDegrees(heading, confirmationHeading_) <=
                config::kRescueExitCandidateHeadingToleranceDegrees) ++candidateFrames_;
        else candidateFrames_ = 1;
        confirmationHeading_ = heading;
        lastSeenAt_ = now;
        sector_ = best;
        score_ = candidate.score;
        if (candidateFrames_ >= config::kRescueExitCandidateFrames)
        {
            if (!retryPending_ || ImuTurnController::angularDistanceDegrees(
                    heading, retryHeading_) > config::kRescueExitRejectedToleranceDegrees)
                retryUsed_ = false;
            retryPending_ = false;
            resumeExplorationAfterReview_ = false;
            resumeExplorationTurn_ = false;
            targetHeading_ = trackingHeading_ = heading;
            advanceLeftCm_ = advanceRightCm_ = lastProgressCm_ = 0.0;
            acquisitionFrames_ = 0;
            attemptAt_ = lastSeenAt_ = lastGuidanceAt_ = progressAt_ = now;
            attempting_ = true;
            guidanceLatched_ = true;
            lastGuidanceAngleDegrees_ = candidate.guidanceAngleDegrees;
            steeringNearLatched_ = candidate.entryDepthNormalized >=
                config::kRescueExitSteeringStartDepth;
            midLatched_ = candidate.nearestBand >= 1;
            nearLatched_ = candidate.nearestBand >= 2;
            reverseTargetCm_ = reversedCm_ = 0.0;
            phase_ = Phase::Approaching;
            movingForward_ = true;
            if (geometryActive_ && geometryCandidateActive_)
            {
                // O corner geométrico já define a direção de avanço. A CAM1
                // mantém somente a proteção visual, sem corrigir os motores
                // antes de a CAM0 assumir o controle.
                phase_ = Phase::Exploring;
                movingForward_ = false;
                movingExploration_ = true;
                return output("rescue_exit_corner_exploring",
                    "Fusion frontal confirmada; mantendo avanço reto até a CAM0",
                    config::kRescueExitExplorationPower,
                    config::kRescueExitExplorationPower);
            }
            const auto powers = mapGuidancePowers(
                lastGuidanceAngleDegrees_, steeringNearLatched_);
            return output("rescue_exit_approaching",
                steeringNearLatched_ ? "Candidata próxima; correção leve pela CAM1" :
                                       "Candidata distante; avançando reto pela CAM1",
                powers.left, powers.right);
        }
        return output("rescue_exit_searching", "Confirmando candidata preta em imagens novas");
    }
    if (reviewingCandidate)
        return output("rescue_exit_searching", "Parado: reobservando candidata antes de girar");
    if (candidateFrames_ > 0)
    {
        candidateFrames_ = observedFrames_ = 0;
        if (resumeExplorationAfterReview_)
        {
            resumeExplorationAfterReview_ = false;
            if (resumeExplorationTurn_)
            {
                resumeExplorationTurn_ = false;
                const double remainingTurn = signedAngle(
                    explorationHeading_ - telemetry.yawZDeg);
                if (std::abs(remainingTurn) > config::kBallApproachStartToleranceDegrees)
                {
                    if (!startTurn(remainingTurn, telemetry, now))
                        return fail("Não foi possível retomar o giro exploratório");
                    phase_ = Phase::ExplorationTurning;
                    return output("rescue_exit_exploration_turning",
                                  "Candidata não confirmou; retomando o giro exploratório");
                }
            }
            phase_ = Phase::Exploring;
            progressAt_ = now;
            movingExploration_ = true;
            return output(geometryCandidateActive_ ?
                              "rescue_exit_corner_exploring" :
                              "rescue_exit_exploring",
                          "Candidata não confirmou; retomando o avanço restante",
                          config::kRescueExitExplorationPower,
                          config::kRescueExitExplorationPower);
        }
        return output("rescue_exit_searching", "Candidata não confirmada; observando o setor novamente");
    }
    ++observedFrames_;
    if (observedFrames_ < config::kRescueExitCandidateFrames)
        return output("rescue_exit_searching", "Observando setores sem candidata confirmada");
    observedFrames_ = 0;
    if (scanSteps_ * config::kRescueExitScanDegrees >= 360.0)
    {
        if (finalScan_ || explorationStarted_)
            return fail("Exploração e última varredura sem saída válida");
        if (!startExploration(telemetry, now))
            return fail("Todas as direções foram descartadas para exploração");
        if (phase_ == Phase::Exploring)
        {
            movingExploration_ = true;
            return output("rescue_exit_exploring",
                          "Varredura concluída; avançando na região mais provável",
                          config::kRescueExitExplorationPower,
                          config::kRescueExitExplorationPower);
        }
        return output(phase_ == Phase::ExplorationTurning ?
                          "rescue_exit_exploration_turning" : "rescue_exit_exploring",
                      "Varredura concluída; explorando a região mais provável");
    }
    if (!startTurn(config::kRescueExitScanDegrees, telemetry, now)) return fail("Não foi possível iniciar a varredura");
    phase_ = Phase::Turning;
    return output("rescue_exit_searching", "Próximo setor à direita");
}

RescueExitOutput RescueExitMission::updateApproach(const ForwardLineSnapshot& forward,
    const Esp32TelemetrySnapshot& telemetry, Time now, bool newForward)
{
    // Associa a observação à direção acompanhada; nunca escolhe o maior blob de novo.
    int match = -1;
    double bestError = config::kRescueExitTrackingToleranceDegrees;
    for (int index = 0; index < static_cast<int>(forward.exitCandidates.size()); ++index)
    {
        const auto& candidate = forward.exitCandidates[index];
        const double error = ImuTurnController::angularDistanceDegrees(
            telemetry.yawZDeg + candidate.txDegrees, trackingHeading_);
        if (candidate.visible && candidate.guidanceValid && error <= bestError)
        {
            match = index;
            bestError = error;
        }
    }
    const bool useful = newForward && forward.ageMs >= 0.0 &&
        forward.ageMs <= config::kRescueExitForwardStatusTimeoutMs &&
        match >= 0 && !forward.cameraObscured;
    if (useful)
    {
        const auto& candidate = forward.exitCandidates[match];
        // As faixas registram o início do handoff; continuidade e fita são só diagnóstico.
        midLatched_ = midLatched_ || candidate.nearestBand >= 1;
        nearLatched_ = nearLatched_ || candidate.nearestBand >= 2;
        lastSeenAt_ = lastGuidanceAt_ = now;
        trackingHeading_ = signedAngle(telemetry.yawZDeg + candidate.txDegrees);
        lastGuidanceAngleDegrees_ = candidate.guidanceAngleDegrees;
        steeringNearLatched_ = steeringNearLatched_ ||
            candidate.entryDepthNormalized >= config::kRescueExitSteeringStartDepth;
        guidanceLatched_ = true;
        sector_ = match;
        score_ = candidate.score;
    }
    const bool guidanceHeld = guidanceLatched_ && !forward.cameraObscured &&
        now - lastGuidanceAt_ <=
            std::chrono::milliseconds(config::kRescueExitGuidanceHoldMs);
    if (!useful && !guidanceHeld)
    {
        if (geometryActive_ && geometryCandidateActive_ &&
            !forward.cameraObscured)
        {
            // O corner foi escolhido pela geometria, não pela Fusion. Se a
            // orientação preta sumir, incorpora o trecho já percorrido e
            // continua reto até o limite geométrico, mantendo CAM0 e prata
            // com prioridade para interromper ou assumir o movimento.
            explorationAttemptLeftCm_ += advanceLeftCm_;
            explorationAttemptRightCm_ += advanceRightCm_;
            explorationTotalCm_ += std::min(advanceLeftCm_, advanceRightCm_);
            explorationLastProgressCm_ = std::min(
                explorationAttemptLeftCm_, explorationAttemptRightCm_);
            advanceLeftCm_ = advanceRightCm_ = lastProgressCm_ = 0.0;
            attempting_ = false;
            guidanceLatched_ = steeringNearLatched_ = false;
            acquisitionFrames_ = 0;
            phase_ = Phase::Exploring;
            progressAt_ = now;
            movingExploration_ = true;
            return output("rescue_exit_corner_exploring",
                          "Fusion perdida; continuando reto no corner geométrico",
                          config::kRescueExitExplorationPower,
                          config::kRescueExitExplorationPower);
        }
        progressAt_ = now;
        if (now - lastGuidanceAt_ >=
            std::chrono::milliseconds(config::kRescueExitReacquisitionWaitMs))
        {
            reject(false, "CAM1 e Fusion perdidos: aguardou reaquisição parado",
                   telemetry, now,
                   !retryUsed_);
            return output((phase_ == Phase::GeometryBacking ||
                           phase_ == Phase::GeometryReturnTurning) ?
                              "rescue_exit_corner_backing" : "rescue_exit_rejected",
                          (phase_ == Phase::GeometryBacking ||
                           phase_ == Phase::GeometryReturnTurning) ?
                              explorationBlockReason_.c_str() : failure_.c_str());
        }
        return output("rescue_exit_lost", "Parado: aguardando CAM1 ou Fusion inferior");
    }
    const double progress = std::min(advanceLeftCm_, advanceRightCm_);
    if (progress >= lastProgressCm_ + config::kDriveDistanceMinimumProgressCounts / config::kEncoderCountsPerCentimeter)
    {
        lastProgressCm_ = progress;
        progressAt_ = now;
    }
    if (now - progressAt_ >= std::chrono::milliseconds(config::kRescueDistanceStallTimeoutMs))
    {
        if (geometryActive_ && geometryCandidateActive_)
        {
            if (startCornerCollisionRecovery(telemetry, now))
                return output("rescue_exit_corner_recovery_backing",
                              "Sem progresso; tentando recuperar o mesmo corner");
            startGeometryReturn(
                "Rodas sem progresso durante o corner", false, telemetry, now);
            return output("rescue_exit_corner_backing",
                          explorationBlockReason_.c_str());
        }
        if (!explorationStarted_)
        {
            explorationStarted_ = true;
            explorationBaseHeading_ = trackingHeading_;
            explorationOffsetIndex_ = 1;
            explorationAttempt_ = 1;
            explorationPositiveFirst_ = true;
        }
        explorationHeading_ = trackingHeading_;
        explorationAttemptLeftCm_ = advanceLeftCm_;
        explorationAttemptRightCm_ = advanceRightCm_;
        startExplorationRecovery("Rodas sem progresso durante a aproximação", now);
        return output("rescue_exit_exploration_blocked", explorationBlockReason_.c_str());
    }
    movingForward_ = true;
    if (geometryActive_ && geometryCandidateActive_)
    {
        return output("rescue_exit_approaching",
            "Corner geométrico confirmado; avançando reto até a CAM0",
            config::kRescueExitExplorationPower,
            config::kRescueExitExplorationPower);
    }
    const auto powers = mapGuidancePowers(lastGuidanceAngleDegrees_, steeringNearLatched_);
    return output("rescue_exit_approaching",
        steeringNearLatched_ ? "Corrigindo levemente pela continuação próxima da CAM1" :
                               "Avançando reto até a faixa ficar próxima da CAM1",
        powers.left, powers.right);
}
