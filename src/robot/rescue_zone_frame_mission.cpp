#include "obr/rescue_zone_frame_mission.h"

#include "obr/config.h"

#include <algorithm>
#include <cmath>

namespace
{
const RescueZoneObservation& selectedZone(
    const RescueZoneSnapshot& zones,
    RescueZoneTargetColor color)
{
    return color == RescueZoneTargetColor::Red ? zones.red : zones.green;
}

bool observationUsable(const RescueZoneObservation& zone)
{
    return zone.candidateDetected && zone.detected &&
           zone.geometryState != RescueZoneGeometryState::NotDetected;
}

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

RescueZoneFrameOutput stoppedOutput(
    const std::string& phase,
    const std::string& action,
    double progressPercent = 0.0)
{
    RescueZoneFrameOutput output;
    output.status = makeStatus(phase, action, progressPercent);
    return output;
}
}

RescueZoneFrameOutput RescueZoneFrameMission::update(
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
    if (phaseStartedAt_ == std::chrono::steady_clock::time_point{})
    {
        phaseStartedAt_ = now;
    }

    if (phase_ == Phase::Completed)
    {
        RescueZoneFrameOutput output = stoppedOutput(
            "rescue_zone_frame_completed", completionAction_, 100.0);
        output.completed = true;
        output.status.rescueZoneFrameCompleted = true;
        output.status.rescueZoneFrameCompletionReason = completionReason_;
        return output;
    }
    if (phase_ == Phase::Failed)
    {
        RescueZoneFrameOutput output = stoppedOutput(
            "rescue_zone_frame_failed",
            "FRAME_ZONE interrompido por uma falha de segurança");
        output.failed = true;
        output.status.rescueZoneFrameFailed = true;
        return output;
    }

    if (phase_ == Phase::WaitingObservation)
    {
        if (!zones.sourceFresh)
        {
            if (now - phaseStartedAt_ > std::chrono::milliseconds(
                    config::kRescueZoneFrameStartupVisionTimeoutMs))
            {
                return fail(
                    "rescue_zone_frame_vision_timeout",
                    "FRAME_ZONE falhou: a CAM1 não publicou um frame recente");
            }
            return stoppedOutput(
                "rescue_zone_frame_waiting_vision",
                "FRAME_ZONE parado: aguardando visão recente da CAM1");
        }
        phase_ = Phase::Evaluating;
    }

    if (phase_ == Phase::Evaluating)
    {
        return evaluate(zones, telemetry, now);
    }
    if (phase_ == Phase::Turning)
    {
        return updateTurn(zones, telemetry, now);
    }
    return updateLateralSettling(zones, telemetry, now);
}

RescueZoneFrameOutput RescueZoneFrameMission::evaluate(
    const RescueZoneSnapshot& zones,
    const Esp32TelemetrySnapshot& telemetry,
    std::chrono::steady_clock::time_point now)
{
    if (!zones.sourceFresh)
    {
        return fail(
            "rescue_zone_frame_vision_stale",
            "FRAME_ZONE falhou: o frame da CAM1 ficou stale");
    }

    const RescueZoneObservation& zone = selectedZone(zones, targetColor_);
    if (!observationUsable(zone))
    {
        return stoppedOutput(
            "rescue_zone_frame_waiting_zone",
            std::string("FRAME_ZONE parado: aguardando zona ") +
                rescueZoneTargetColorName(targetColor_));
    }

    if (zone.geometryState == RescueZoneGeometryState::FullBounds)
    {
        if (zone.aimValid && std::isfinite(zone.aimX))
        {
            return complete(
                "FULL_BOUNDS",
                "FRAME_ZONE concluído: FULL_BOUNDS com aim válido");
        }
        return complete(
            "BEST_EFFORT_FULL_BOUNDS_NO_AIM",
            "FRAME_ZONE concluído em best effort: FULL_BOUNDS sem aim válido");
    }

    if (zone.geometryState == RescueZoneGeometryState::BoundsUnknown)
    {
        // Uma zona confirmada e próxima pode ocupar quase todo o frame. Recuar
        // apenas para revelar paredes brancas criaria um movimento desnecessário.
        return complete(
            "BEST_EFFORT_BOUNDS_UNKNOWN",
            "FRAME_ZONE concluído em best effort: bounds indisponíveis");
    }

    if (microPivotAttempts_ >= config::kRescueZoneFrameMaximumMicroPivots)
    {
        return complete(
            "BEST_EFFORT_PARTIAL_BOUND",
            "FRAME_ZONE concluído em best effort após micro-pivôs laterais");
    }

    if (!ImuTurnController::imuReady(telemetry))
    {
        return fail(
            "rescue_zone_frame_imu_stale",
            "FRAME_ZONE falhou: IMU sem amostra recente para o limite de segurança");
    }

    movementGeometryState_ = zone.geometryState;
    phaseStartedAt_ = now;
    ++microPivotAttempts_;

    // A referência angular é preservada entre pulsos. A IMU nunca define a
    // duração ou o setpoint do micro-pivô; ela limita apenas o giro acumulado.
    if (!pivotSafetyReferenceInitialized_)
    {
        pivotLastYawDegrees_ = telemetry.yawZDeg;
        pivotAccumulatedDegrees_ = 0.0;
        pivotOperationStartedAt_ = now;
        pivotSafetyReferenceInitialized_ = true;
    }
    phase_ = Phase::Turning;
    return updateTurn(zones, telemetry, now);
}

RescueZoneFrameOutput RescueZoneFrameMission::updateTurn(
    const RescueZoneSnapshot& zones,
    const Esp32TelemetrySnapshot& telemetry,
    std::chrono::steady_clock::time_point now)
{
    const RescueZoneObservation& zone = selectedZone(zones, targetColor_);
    if (!zones.sourceFresh || !observationUsable(zone))
    {
        return fail(
            "rescue_zone_frame_zone_lost",
            "FRAME_ZONE falhou: visão necessária foi perdida durante o giro");
    }
    if ((zone.geometryState == RescueZoneGeometryState::FullBounds &&
         zone.aimValid && std::isfinite(zone.aimX)) ||
        zone.geometryState != movementGeometryState_)
    {
        beginLateralSettling(zones, now);
        return stoppedOutput(
            "rescue_zone_frame_braking",
            "Geometria mudou: micro-pivô interrompido e PWM zerado");
    }
    if (now - pivotOperationStartedAt_ > std::chrono::milliseconds(
            config::kRescueZoneFrameTurnTimeoutMs))
    {
        return fail(
            "rescue_zone_frame_turn_timeout",
            "FRAME_ZONE falhou: a sequência de micro-pivôs excedeu o tempo global");
    }
    if (!ImuTurnController::imuReady(telemetry))
    {
        return fail(
            "rescue_zone_frame_imu_stale",
            "FRAME_ZONE falhou: IMU indisponível para o limite do pivot");
    }
    pivotAccumulatedDegrees_ += ImuTurnController::angularDistanceDegrees(
        pivotLastYawDegrees_, telemetry.yawZDeg);
    pivotLastYawDegrees_ = telemetry.yawZDeg;
    if (pivotAccumulatedDegrees_ >= config::kRescueZoneFrameMaximumPivotDegrees)
    {
        return fail(
            "rescue_zone_frame_pivot_angle_limit",
            "FRAME_ZONE falhou: os micro-pivôs atingiram o limite angular total");
    }
    if (now - phaseStartedAt_ >= std::chrono::milliseconds(
            config::kRescueZoneFrameMicroPivotDurationMs))
    {
        beginLateralSettling(zones, now);
        return stoppedOutput(
            "rescue_zone_frame_braking",
            "Micro-pivô concluído: PWM zerado antes de obter outro frame");
    }

    RescueZoneFrameOutput output;
    const bool pivotRight =
        movementGeometryState_ == RescueZoneGeometryState::LeftBoundOnly;
    output.leftPower = pivotRight ? config::kRescueZoneFrameTurnPower
                                  : -config::kRescueZoneFrameTurnPower;
    output.rightPower = -output.leftPower;
    output.status = makeStatus(
        "rescue_zone_frame_turning",
        "Executando micro-pivô temporizado para revelar a borda ausente",
        std::clamp(
            static_cast<double>(microPivotAttempts_) /
                config::kRescueZoneFrameMaximumMicroPivots * 100.0,
            0.0,
            100.0));
    return output;
}

RescueZoneFrameOutput RescueZoneFrameMission::updateLateralSettling(
    const RescueZoneSnapshot& zones,
    const Esp32TelemetrySnapshot& telemetry,
    std::chrono::steady_clock::time_point now)
{
    if (!zones.sourceFresh)
    {
        return fail(
            "rescue_zone_frame_vision_stale",
            "FRAME_ZONE falhou: a visão ficou stale após o micro-pivô");
    }
    if (now - phaseStartedAt_ > std::chrono::milliseconds(
            config::kRescueZoneFrameNewFrameTimeoutMs))
    {
        return fail(
            "rescue_zone_frame_new_frame_timeout",
            "FRAME_ZONE falhou: nenhum frame novo chegou após o micro-pivô");
    }
    if (now - phaseStartedAt_ < std::chrono::milliseconds(
            config::kRescueZoneFrameLateralSettleMs))
    {
        return stoppedOutput(
            "rescue_zone_frame_settling",
            "Micro-pivô parado: aguardando settling lateral fixo");
    }
    if (zones.sequence <= lateralMovementEndSequence_ ||
        zones.timestamp <= lateralMovementEndTimestamp_)
    {
        return stoppedOutput(
            "rescue_zone_frame_waiting_new_frame",
            "Micro-pivô parado: aguardando frame posterior ao movimento");
    }

    phase_ = Phase::Evaluating;
    phaseStartedAt_ = now;
    return evaluate(zones, telemetry, now);
}

RescueZoneFrameOutput RescueZoneFrameMission::complete(
    const std::string& reason,
    const std::string& action)
{
    phase_ = Phase::Completed;
    completionReason_ = reason;
    completionAction_ = action;
    RescueZoneFrameOutput output = stoppedOutput(
        "rescue_zone_frame_completed", action, 100.0);
    output.completed = true;
    output.status.rescueZoneFrameCompleted = true;
    output.status.rescueZoneFrameCompletionReason = reason;
    return output;
}

RescueZoneFrameOutput RescueZoneFrameMission::fail(
    const std::string& phase,
    const std::string& action)
{
    phase_ = Phase::Failed;
    RescueZoneFrameOutput output = stoppedOutput(phase, action);
    output.failed = true;
    output.status.rescueZoneFrameFailed = true;
    return output;
}

void RescueZoneFrameMission::beginLateralSettling(
    const RescueZoneSnapshot& zones,
    std::chrono::steady_clock::time_point now)
{
    phase_ = Phase::LateralSettling;
    phaseStartedAt_ = now;
    lateralMovementEndSequence_ = zones.sequence;
    lateralMovementEndTimestamp_ = zones.timestamp;
}

void RescueZoneFrameMission::reset()
{
    phase_ = Phase::WaitingObservation;
    targetColorInitialized_ = false;
    movementGeometryState_ = RescueZoneGeometryState::NotDetected;
    pivotLastYawDegrees_ = 0.0;
    pivotAccumulatedDegrees_ = 0.0;
    pivotSafetyReferenceInitialized_ = false;
    microPivotAttempts_ = 0;
    lateralMovementEndSequence_ = 0;
    lateralMovementEndTimestamp_ = 0.0;
    completionReason_.clear();
    completionAction_.clear();
    phaseStartedAt_ = {};
    pivotOperationStartedAt_ = {};
}
