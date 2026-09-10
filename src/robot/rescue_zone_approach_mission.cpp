#include "obr/rescue_zone_approach_mission.h"

#include "obr/config.h"
#include "obr/imu_turn_controller.h"

#include <algorithm>
#include <cmath>

RescueZoneApproachOutput RescueZoneApproachMission::update(
    double lockedHeadingDegrees,
    const Esp32TelemetrySnapshot& telemetry,
    std::chrono::steady_clock::time_point now,
    const RescueZoneSnapshot& zones,
    RescueZoneTargetColor targetColor)
{
    if (completed_ || failed_)
    {
        RescueZoneApproachOutput output = stoppedOutput(
            terminalPhase_, terminalAction_, telemetry,
            ImuTurnController::imuReady(telemetry)
                ? signedHeadingErrorDegrees(
                      lockedHeadingDegrees_, telemetry.yawZDeg)
                : std::numeric_limits<double>::quiet_NaN(),
            "STOP");
        output.completed = completed_;
        output.failed = failed_;
        return output;
    }

    if (!started_)
    {
        if (!std::isfinite(lockedHeadingDegrees))
        {
            return stopWithFailure(
                "rescue_zone_approach_no_heading",
                "APPROACH_ZONE não iniciado: lockedHeading indisponível",
                "LOCKED_HEADING_UNAVAILABLE",
                telemetry);
        }
        if (!ImuTurnController::imuReady(telemetry))
        {
            return stopWithFailure(
                "rescue_zone_approach_imu_stale",
                "APPROACH_ZONE não iniciado: IMU inválida ou stale",
                "IMU_STALE",
                telemetry);
        }
        if (!ultrasonicFresh(telemetry) || !ultrasonicValid(telemetry))
        {
            return stopWithFailure(
                "rescue_zone_approach_ultra_stale",
                "APPROACH_ZONE não iniciado: ULTRA inválido ou stale",
                "ULTRA_STALE",
                telemetry);
        }

        started_ = true;
        lockedHeadingDegrees_ = lockedHeadingDegrees;
        startedAt_ = now;
    }

    if (now - startedAt_ >
        std::chrono::milliseconds(config::kRescueZoneApproachTimeoutMs))
    {
        return stopWithFailure(
            "rescue_zone_approach_timeout",
            "APPROACH_ZONE interrompido: tempo global excedido",
            "TIMEOUT",
            telemetry);
    }
    if (!ImuTurnController::imuReady(telemetry))
    {
        return stopWithFailure(
            "rescue_zone_approach_imu_stale",
            "APPROACH_ZONE interrompido: IMU inválida ou stale",
            "IMU_STALE",
            telemetry);
    }
    if (!ultrasonicFresh(telemetry) || !ultrasonicValid(telemetry))
    {
        return stopWithFailure(
            "rescue_zone_approach_ultra_stale",
            "APPROACH_ZONE interrompido: ULTRA inválido ou stale",
            "ULTRA_STALE",
            telemetry);
    }

    const double distanceCm = telemetry.ultrasonicDistanceCm;
    const double headingErrorDegrees = signedHeadingErrorDegrees(
        lockedHeadingDegrees_, telemetry.yawZDeg);
    const RescueZoneObservation& targetZone =
        targetColor == RescueZoneTargetColor::Red ? zones.red : zones.green;
    const bool cameraStop =
        zones.sourceFresh &&
        (zones.cameraObscured ||
         targetZone.frameCoverage >=
             config::kRescueZoneApproachCameraStopCoverage);
    if (!finalAdvanceActive_ && cameraStop)
    {
        // A imagem sem detalhe ou tomada pela área indica contato muito próximo.
        // Ainda executa o mesmo avanço final lento antes de zerar os motores.
        finalAdvanceActive_ = true;
        finalAdvanceStartedAt_ = now;
        completionReason_ = "CAMERA_OBSCURED";
        RescueZoneApproachOutput output = stoppedOutput(
            "rescue_zone_approach_final_advance",
            "APPROACH_ZONE executando avanço final após obstrução da CAM1",
            telemetry,
            headingErrorDegrees,
            "FINAL");
        output.leftPower = config::kRescueZoneApproachNearPower;
        output.rightPower = config::kRescueZoneApproachNearPower;
        return output;
    }
    if (finalAdvanceActive_)
    {
        if (now - finalAdvanceStartedAt_ >=
            std::chrono::milliseconds(
                config::kRescueZoneApproachFinalAdvanceMs))
        {
            completed_ = true;
            terminalPhase_ = "rescue_zone_approach_completed";
            terminalAction_ =
                "APPROACH_ZONE concluído após avanço final temporizado";
            if (completionReason_.empty())
            {
                completionReason_ = "REACHED_DISTANCE";
            }
            RescueZoneApproachOutput output = stoppedOutput(
                terminalPhase_, terminalAction_, telemetry,
                headingErrorDegrees, "STOP");
            output.completed = true;
            return output;
        }

        RescueZoneApproachOutput output = stoppedOutput(
            "rescue_zone_approach_final_advance",
            "APPROACH_ZONE executando avanço final temporizado",
            telemetry,
            headingErrorDegrees,
            "FINAL");
        output.leftPower = config::kRescueZoneApproachNearPower;
        output.rightPower = config::kRescueZoneApproachNearPower;
        return output;
    }
    if (distanceCm <= config::kRescueZoneApproachStopDistanceCm)
    {
        // Após alcançar a distância alvo, avança por um intervalo curto e fixo.
        // A fase ignora novas faixas de distância e nunca aumenta a potência.
        finalAdvanceActive_ = true;
        finalAdvanceStartedAt_ = now;
        completionReason_ = "REACHED_DISTANCE";
        RescueZoneApproachOutput output = stoppedOutput(
            "rescue_zone_approach_final_advance",
            "APPROACH_ZONE executando avanço final temporizado",
            telemetry,
            headingErrorDegrees,
            "FINAL");
        output.leftPower = config::kRescueZoneApproachNearPower;
        output.rightPower = config::kRescueZoneApproachNearPower;
        return output;
    }

    if (distanceCm <= config::kRescueZoneApproachNearDistanceCm)
    {
        nearLatched_ = true;
    }

    double basePower = config::kRescueZoneApproachFarPower;
    std::string speedState = "FAR";
    if (nearLatched_ ||
        distanceCm <= config::kRescueZoneApproachNearDistanceCm)
    {
        // Depois de entrar na faixa próxima, um eco falso distante nunca
        // recupera as velocidades FAR ou MID durante esta aproximação.
        basePower = config::kRescueZoneApproachNearPower;
        speedState = "NEAR";
    }
    else if (distanceCm <= config::kRescueZoneApproachFarDistanceCm)
    {
        basePower = config::kRescueZoneApproachMidPower;
        speedState = "MID";
    }

    const double normalizedHeadingError = std::clamp(
        headingErrorDegrees /
            config::kRescueZoneApproachFullHeadingErrorDegrees,
        -1.0,
        1.0);
    const double correction =
        normalizedHeadingError *
        config::kRescueZoneApproachMaximumHeadingCorrection;

    RescueZoneApproachOutput output = stoppedOutput(
        "rescue_zone_approaching",
        "APPROACH_ZONE avançando com heading travado",
        telemetry,
        headingErrorDegrees,
        speedState);
    // O erro é alvo menos yaw atual. Pela convenção dos pivôs já validada,
    // positivo corrige à direita e negativo corrige à esquerda.
    output.leftPower = std::clamp(
        basePower + correction, 0.0, config::kMaxMotorOutput);
    output.rightPower = std::clamp(
        basePower - correction, 0.0, config::kMaxMotorOutput);
    return output;
}

void RescueZoneApproachMission::reset()
{
    started_ = false;
    nearLatched_ = false;
    finalAdvanceActive_ = false;
    completed_ = false;
    failed_ = false;
    lockedHeadingDegrees_ = std::numeric_limits<double>::quiet_NaN();
    terminalPhase_.clear();
    terminalAction_.clear();
    completionReason_.clear();
}

RescueZoneApproachOutput RescueZoneApproachMission::stopWithFailure(
    const std::string& phase,
    const std::string& action,
    const std::string& reason,
    const Esp32TelemetrySnapshot& telemetry)
{
    failed_ = true;
    terminalPhase_ = phase;
    terminalAction_ = action;
    completionReason_ = reason;
    const double headingErrorDegrees =
        std::isfinite(lockedHeadingDegrees_) &&
                ImuTurnController::imuReady(telemetry)
            ? signedHeadingErrorDegrees(
                  lockedHeadingDegrees_, telemetry.yawZDeg)
            : std::numeric_limits<double>::quiet_NaN();
    RescueZoneApproachOutput output = stoppedOutput(
        phase, action, telemetry, headingErrorDegrees, "STOP");
    output.failed = true;
    return output;
}

RescueZoneApproachOutput RescueZoneApproachMission::stoppedOutput(
    const std::string& phase,
    const std::string& action,
    const Esp32TelemetrySnapshot& telemetry,
    double headingErrorDegrees,
    const std::string& speedState) const
{
    RescueZoneApproachOutput output;
    output.status.phase = phase;
    output.status.action = action;
    output.status.progressPercent = completed_ ? 100.0 : 0.0;
    output.status.rescueZoneUltrasonicFresh = ultrasonicFresh(telemetry);
    output.status.rescueZoneUltrasonicValid = ultrasonicValid(telemetry);
    if (output.status.rescueZoneUltrasonicValid)
    {
        output.status.rescueZoneUltrasonicDistanceCm =
            telemetry.ultrasonicDistanceCm;
    }
    output.status.rescueZoneApproachLockedHeading = lockedHeadingDegrees_;
    output.status.rescueZoneApproachHeadingError = headingErrorDegrees;
    output.status.rescueZoneApproachNearLatched = nearLatched_;
    output.status.rescueZoneApproachSpeedState = speedState;
    output.status.rescueZoneApproachCompletionReason = completionReason_;
    return output;
}

bool RescueZoneApproachMission::ultrasonicFresh(
    const Esp32TelemetrySnapshot& telemetry)
{
    return telemetry.sensorFresh && telemetry.lastSensorAgeMs >= 0 &&
           telemetry.lastSensorAgeMs <=
               config::kRescueZoneUltrasonicFreshnessMs;
}

bool RescueZoneApproachMission::ultrasonicValid(
    const Esp32TelemetrySnapshot& telemetry)
{
    return std::isfinite(telemetry.ultrasonicDistanceCm) &&
           telemetry.ultrasonicDistanceCm >=
               config::kRescueZoneUltrasonicMinimumCm &&
           telemetry.ultrasonicDistanceCm <=
               config::kRescueZoneUltrasonicMaximumCm;
}

double RescueZoneApproachMission::signedHeadingErrorDegrees(
    double targetDegrees,
    double currentDegrees)
{
    double errorDegrees = std::fmod(targetDegrees - currentDegrees, 360.0);
    if (errorDegrees > 180.0)
    {
        errorDegrees -= 360.0;
    }
    else if (errorDegrees < -180.0)
    {
        errorDegrees += 360.0;
    }
    return errorDegrees;
}
