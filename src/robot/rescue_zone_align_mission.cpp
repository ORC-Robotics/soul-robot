#include "obr/rescue_zone_align_mission.h"

#include "obr/config.h"
#include "obr/imu_turn_controller.h"

#include <cmath>
#include <iostream>

namespace
{
const RescueZoneObservation& selectedZone(
    const RescueZoneSnapshot& zones,
    RescueZoneTargetColor color)
{
    return color == RescueZoneTargetColor::Red ? zones.red : zones.green;
}
}

RescueZoneAlignOutput RescueZoneAlignMission::update(
    const RescueZoneSnapshot& zones,
    const Esp32TelemetrySnapshot& telemetry,
    RescueZoneTargetColor targetColor,
    std::chrono::steady_clock::time_point now)
{
    const RescueZoneAlignOutput output = updatePhase(zones, telemetry, targetColor, now);
    if (output.status.phase != lastLoggedPhase_)
    {
        lastLoggedPhase_ = output.status.phase;
        const auto& zone = selectedZone(zones, targetColor);
        std::cout << "Rescue zone alignment: phase=" << output.status.phase
                  << " reason=" << output.status.action
                  << " sequence=" << zones.sequence
                  << " visionFresh=" << zones.sourceFresh
                  << " candidate=" << zone.candidateDetected << " detected=" << zone.detected
                  << " direction=" << output.status.rescueZoneAlignState
                  << " imuReady=" << ImuTurnController::imuReady(telemetry)
                  << " requested=" << output.leftPower << ',' << output.rightPower
                  << " applied=" << telemetry.appliedLeftPower << ',' << telemetry.appliedRightPower
                  << " encoderRates=" << telemetry.leftEncoderRate << ',' << telemetry.rightEncoderRate
                  << '\n';
    }
    return output;
}

RescueZoneAlignOutput RescueZoneAlignMission::updatePhase(
    const RescueZoneSnapshot& zones,
    const Esp32TelemetrySnapshot& telemetry,
    RescueZoneTargetColor targetColor,
    std::chrono::steady_clock::time_point now)
{
    if (!targetColorInitialized_ || targetColor_ != targetColor)
    {
        reset();
        targetColor_ = targetColor;
        targetColorInitialized_ = true;
        phaseStartedAt_ = now;
    }

    if (phase_ == Phase::Completed)
    {
        RescueZoneAlignOutput output = stoppedOutput(
            terminalPhase_, terminalAction_, 100.0);
        output.completed = true;
        return output;
    }

    if (phase_ == Phase::Pivoting)
    {
        return updatePivot(zones, telemetry, now);
    }
    if (phase_ == Phase::Settling)
    {
        return updateSettling(zones, telemetry, now);
    }
    return evaluate(zones, telemetry, now);
}

RescueZoneAlignOutput RescueZoneAlignMission::evaluate(
    const RescueZoneSnapshot& zones,
    const Esp32TelemetrySnapshot& telemetry,
    std::chrono::steady_clock::time_point now)
{
    if (!zones.sourceFresh)
    {
        return pause(
            "rescue_zone_align_waiting_vision",
            "ALIGN_ZONE pausado: aguardando snapshot GREEN/RED fresh");
    }
    if (!ImuTurnController::imuReady(telemetry))
    {
        return pause(
            "rescue_zone_align_waiting_imu",
            "ALIGN_ZONE pausado: aguardando IMU válida para salvar o heading");
    }

    const RescueZoneObservation& zone = selectedZone(zones, targetColor_);
    if (!observationConfirmed(zone))
    {
        return pause(
            "rescue_zone_align_waiting_zone",
            "ALIGN_ZONE parado: aguardando zona confirmada");
    }

    if (zone.geometryState == RescueZoneGeometryState::BoundsUnknown)
    {
        currentDirection_ = Direction::Unavailable;
        currentAimNormalized_ = std::numeric_limits<double>::quiet_NaN();
        return complete(
            "BEST_EFFORT_BOUNDS_UNKNOWN",
            "ALIGN_ZONE concluído em best effort: bounds desconhecidos",
            telemetry.yawZDeg);
    }

    if (zone.geometryState == RescueZoneGeometryState::LeftBoundOnly ||
        zone.geometryState == RescueZoneGeometryState::RightBoundOnly)
    {
        currentAimNormalized_ = std::numeric_limits<double>::quiet_NaN();
        centeredFrames_ = 0;
        // LEFT_BOUND_ONLY precisa revelar a borda direita; RIGHT_BOUND_ONLY
        // precisa revelar a esquerda. O pulso continua curto e sem setpoint IMU.
        const Direction direction =
            zone.geometryState == RescueZoneGeometryState::LeftBoundOnly
                ? Direction::Right
                : Direction::Left;
        currentDirection_ = direction;
        return startPivot(
            direction, true, zone.geometryState, zones, telemetry, now);
    }

    if (zone.geometryState != RescueZoneGeometryState::FullBounds ||
        !aimUsable(zone))
    {
        currentDirection_ = Direction::Unavailable;
        currentAimNormalized_ = std::numeric_limits<double>::quiet_NaN();
        return pause(
            "rescue_zone_align_waiting_aim",
            "ALIGN_ZONE parado: aguardando FULL_BOUNDS com aim válido");
    }

    updateAimTelemetry(zone);
    if (zones.sequence == lastEvaluatedSequence_)
    {
        return stoppedOutput(
            "rescue_zone_align_evaluating",
            "ALIGN_ZONE parado: aguardando observação nova");
    }
    lastEvaluatedSequence_ = zones.sequence;

    if (currentDirection_ == Direction::Center)
    {
        ++centeredFrames_;
        if (centeredFrames_ >= config::kRescueZoneAlignStableFrames)
        {
            return complete(
                "ALIGNED",
                "ALIGN_ZONE concluído: centro confirmado em dois frames",
                telemetry.yawZDeg);
        }
        return stoppedOutput(
            "rescue_zone_align_center",
            "Aim no centro: confirmando a segunda observação",
            50.0);
    }

    centeredFrames_ = 0;
    return startPivot(
        currentDirection_, false, RescueZoneGeometryState::FullBounds,
        zones, telemetry, now);
}

RescueZoneAlignOutput RescueZoneAlignMission::startPivot(
    Direction direction,
    bool usesPartialBound,
    RescueZoneGeometryState partialGeometry,
    const RescueZoneSnapshot& zones,
    const Esp32TelemetrySnapshot& telemetry,
    std::chrono::steady_clock::time_point now)
{
    activePivotDirection_ = direction;
    activePivotUsesPartialBound_ = usesPartialBound;
    activePartialGeometry_ = partialGeometry;
    phase_ = Phase::Pivoting;
    phaseStartedAt_ = now;
    return updatePivot(zones, telemetry, now);
}

RescueZoneAlignOutput RescueZoneAlignMission::updatePivot(
    const RescueZoneSnapshot& zones,
    const Esp32TelemetrySnapshot& telemetry,
    std::chrono::steady_clock::time_point now)
{
    if (!zones.sourceFresh)
    {
        beginSettling(zones, now);
        currentDirection_ = Direction::Unavailable;
        currentAimNormalized_ = std::numeric_limits<double>::quiet_NaN();
        centeredFrames_ = 0;
        return stoppedOutput(
            "rescue_zone_align_waiting_vision",
            "ALIGN_ZONE pausado: visão stale durante o micro-pivô");
    }
    if (!ImuTurnController::imuReady(telemetry))
    {
        beginSettling(zones, now);
        currentDirection_ = Direction::Unavailable;
        currentAimNormalized_ = std::numeric_limits<double>::quiet_NaN();
        centeredFrames_ = 0;
        return stoppedOutput(
            "rescue_zone_align_waiting_imu",
            "ALIGN_ZONE pausado: IMU inválida durante o micro-pivô");
    }

    const RescueZoneObservation& zone = selectedZone(zones, targetColor_);
    if (!observationConfirmed(zone))
    {
        beginSettling(zones, now);
        currentDirection_ = Direction::Unavailable;
        currentAimNormalized_ = std::numeric_limits<double>::quiet_NaN();
        centeredFrames_ = 0;
        return stoppedOutput(
            "rescue_zone_align_waiting_zone",
            "ALIGN_ZONE pausado: zona confirmada foi perdida");
    }
    if (zone.geometryState == RescueZoneGeometryState::BoundsUnknown)
    {
        currentDirection_ = Direction::Unavailable;
        currentAimNormalized_ = std::numeric_limits<double>::quiet_NaN();
        return complete(
            "BEST_EFFORT_BOUNDS_UNKNOWN",
            "ALIGN_ZONE concluído em best effort: bounds desconhecidos",
            telemetry.yawZDeg);
    }

    bool observationChanged = false;
    if (activePivotUsesPartialBound_)
    {
        currentAimNormalized_ = std::numeric_limits<double>::quiet_NaN();
        observationChanged = zone.geometryState != activePartialGeometry_;
    }
    else if (zone.geometryState != RescueZoneGeometryState::FullBounds)
    {
        observationChanged = true;
    }
    else if (!aimUsable(zone))
    {
        beginSettling(zones, now);
        currentDirection_ = Direction::Unavailable;
        currentAimNormalized_ = std::numeric_limits<double>::quiet_NaN();
        centeredFrames_ = 0;
        return stoppedOutput(
            "rescue_zone_align_waiting_aim",
            "ALIGN_ZONE pausado: aim ficou indisponível");
    }
    else
    {
        updateAimTelemetry(zone);
        observationChanged = currentDirection_ != activePivotDirection_;
    }

    if (observationChanged ||
        now - phaseStartedAt_ >= std::chrono::milliseconds(
            config::kRescueZoneAlignMicroPivotDurationMs))
    {
        beginSettling(zones, now);
        return stoppedOutput(
            "rescue_zone_align_braking",
            "Micro-pivô concluído: PWM zerado antes do novo frame");
    }

    RescueZoneAlignOutput output = stoppedOutput(
        "rescue_zone_align_pivoting",
        activePivotDirection_ == Direction::Left
            ? "Correção visual: micro-pivô para a esquerda"
            : "Correção visual: micro-pivô para a direita");
    const bool pivotLeft = activePivotDirection_ == Direction::Left;
    output.leftPower = pivotLeft ? -config::kRescueZoneAlignTurnPower
                                 : config::kRescueZoneAlignTurnPower;
    output.rightPower = -output.leftPower;
    return output;
}

RescueZoneAlignOutput RescueZoneAlignMission::updateSettling(
    const RescueZoneSnapshot& zones,
    const Esp32TelemetrySnapshot& telemetry,
    std::chrono::steady_clock::time_point now)
{
    if (!zones.sourceFresh)
    {
        currentDirection_ = Direction::Unavailable;
        currentAimNormalized_ = std::numeric_limits<double>::quiet_NaN();
        centeredFrames_ = 0;
        return stoppedOutput(
            "rescue_zone_align_waiting_vision",
            "ALIGN_ZONE pausado: visão stale durante o settling");
    }
    if (!ImuTurnController::imuReady(telemetry))
    {
        currentDirection_ = Direction::Unavailable;
        currentAimNormalized_ = std::numeric_limits<double>::quiet_NaN();
        centeredFrames_ = 0;
        return stoppedOutput(
            "rescue_zone_align_waiting_imu",
            "ALIGN_ZONE pausado: IMU inválida durante o settling");
    }

    const RescueZoneObservation& zone = selectedZone(zones, targetColor_);
    if (!observationConfirmed(zone))
    {
        return pause(
            "rescue_zone_align_waiting_zone",
            "ALIGN_ZONE pausado: zona confirmada foi perdida após o pulso");
    }
    if (zone.geometryState == RescueZoneGeometryState::BoundsUnknown)
    {
        currentDirection_ = Direction::Unavailable;
        currentAimNormalized_ = std::numeric_limits<double>::quiet_NaN();
        return complete(
            "BEST_EFFORT_BOUNDS_UNKNOWN",
            "ALIGN_ZONE concluído em best effort: bounds desconhecidos",
            telemetry.yawZDeg);
    }
    if (zone.geometryState == RescueZoneGeometryState::FullBounds)
    {
        if (!aimUsable(zone))
        {
            currentDirection_ = Direction::Unavailable;
            currentAimNormalized_ = std::numeric_limits<double>::quiet_NaN();
            return pause(
                "rescue_zone_align_waiting_aim",
                "ALIGN_ZONE pausado: aguardando aim válido após o pulso");
        }
        updateAimTelemetry(zone);
    }

    if (now - phaseStartedAt_ <
        std::chrono::milliseconds(config::kRescueZoneAlignSettleMs))
    {
        return stoppedOutput(
            "rescue_zone_align_settling",
            "Micro-pivô parado: aguardando settling fixo");
    }
    if (zones.sequence <= movementEndSequence_ ||
        zones.timestamp <= movementEndTimestamp_)
    {
        return stoppedOutput(
            "rescue_zone_align_waiting_new_frame",
            "Micro-pivô parado: aguardando frame posterior ao movimento");
    }

    phase_ = Phase::Evaluating;
    return evaluate(zones, telemetry, now);
}

RescueZoneAlignOutput RescueZoneAlignMission::complete(
    const std::string& reason,
    const std::string& action,
    double headingDegrees)
{
    phase_ = Phase::Completed;
    terminalPhase_ = "rescue_zone_align_completed";
    terminalAction_ = action;
    completionReason_ = reason;
    lockedHeading_ = headingDegrees;
    RescueZoneAlignOutput output = stoppedOutput(
        terminalPhase_, terminalAction_, 100.0);
    output.completed = true;
    return output;
}

RescueZoneAlignOutput RescueZoneAlignMission::pause(
    const std::string& phase,
    const std::string& action)
{
    phase_ = Phase::Evaluating;
    currentDirection_ = Direction::Unavailable;
    currentAimNormalized_ = std::numeric_limits<double>::quiet_NaN();
    centeredFrames_ = 0;
    return stoppedOutput(phase, action);
}

RescueZoneAlignOutput RescueZoneAlignMission::stoppedOutput(
    const std::string& phase,
    const std::string& action,
    double progressPercent) const
{
    RescueZoneAlignOutput output;
    output.status.phase = phase;
    output.status.action = action;
    output.status.progressPercent = progressPercent;
    output.status.rescueZoneAlignAimNormalized = currentAimNormalized_;
    output.status.rescueZoneAlignState = directionName(currentDirection_);
    output.status.rescueZoneAlignLockedHeading = lockedHeading_;
    output.status.rescueZoneAlignCompletionReason = completionReason_;
    return output;
}

void RescueZoneAlignMission::beginSettling(
    const RescueZoneSnapshot& zones,
    std::chrono::steady_clock::time_point now)
{
    phase_ = Phase::Settling;
    phaseStartedAt_ = now;
    movementEndSequence_ = zones.sequence;
    movementEndTimestamp_ = zones.timestamp;
}

void RescueZoneAlignMission::updateAimTelemetry(
    const RescueZoneObservation& zone)
{
    if (!aimUsable(zone))
    {
        currentAimNormalized_ = std::numeric_limits<double>::quiet_NaN();
        currentDirection_ = Direction::Unavailable;
        return;
    }
    currentAimNormalized_ = zone.aimNormalized;
    currentDirection_ = classifyAim(zone.aimNormalized);
}

bool RescueZoneAlignMission::observationConfirmed(
    const RescueZoneObservation& zone)
{
    return zone.candidateDetected && zone.detected &&
           zone.geometryState != RescueZoneGeometryState::NotDetected;
}

bool RescueZoneAlignMission::aimUsable(const RescueZoneObservation& zone)
{
    return zone.aimValid && std::isfinite(zone.aimNormalized) &&
           zone.aimNormalized >= -1.0 && zone.aimNormalized <= 1.0;
}

RescueZoneAlignMission::Direction RescueZoneAlignMission::classifyAim(
    double aimNormalized)
{
    if (aimNormalized < -config::kRescueZoneAlignDeadbandNormalized)
    {
        return Direction::Left;
    }
    if (aimNormalized > config::kRescueZoneAlignDeadbandNormalized)
    {
        return Direction::Right;
    }
    return Direction::Center;
}

const char* RescueZoneAlignMission::directionName(Direction direction)
{
    switch (direction)
    {
    case Direction::Left:
        return "LEFT";
    case Direction::Right:
        return "RIGHT";
    case Direction::Center:
        return "CENTER";
    case Direction::Unavailable:
    default:
        return "UNAVAILABLE";
    }
}

void RescueZoneAlignMission::reset()
{
    phase_ = Phase::Evaluating;
    targetColorInitialized_ = false;
    currentDirection_ = Direction::Unavailable;
    activePivotDirection_ = Direction::Unavailable;
    activePivotUsesPartialBound_ = false;
    activePartialGeometry_ = RescueZoneGeometryState::NotDetected;
    currentAimNormalized_ = std::numeric_limits<double>::quiet_NaN();
    lockedHeading_ = std::numeric_limits<double>::quiet_NaN();
    centeredFrames_ = 0;
    lastEvaluatedSequence_ = 0;
    movementEndSequence_ = 0;
    movementEndTimestamp_ = 0.0;
    terminalPhase_.clear();
    terminalAction_.clear();
    completionReason_.clear();
    lastLoggedPhase_.clear();
    phaseStartedAt_ = {};
}
