#include "obr/rescue_zone_triangle_mission.h"

#include <cmath>

void RescueZoneTriangleMission::markPhase(
    AutonomousStatus& status,
    const char* phase)
{
    status.rescueZoneTrianglePhase = phase;
}

RescueZoneTriangleOutput RescueZoneTriangleMission::update(
    const RescueZoneSnapshot& zones,
    const Esp32TelemetrySnapshot& telemetry,
    RescueZoneTargetColor targetColor,
    std::chrono::steady_clock::time_point now)
{
    RescueZoneTriangleOutput output;

    if (phase_ == Phase::Search)
    {
        const RescueZoneSearchOutput search =
            searchMission_.update(zones, targetColor);
        output.leftPower = search.leftPower;
        output.rightPower = search.rightPower;
        output.status = search.status;
        markPhase(output.status, "SEARCH");
        if (!search.completed)
        {
            return output;
        }
        phase_ = Phase::Align;
        alignMission_.reset();
        output.leftPower = 0.0;
        output.rightPower = 0.0;
        markPhase(output.status, "ALIGN");
        return output;
    }

    if (phase_ == Phase::Align)
    {
        const RescueZoneAlignOutput align =
            alignMission_.update(zones, telemetry, targetColor, now);
        output.leftPower = align.leftPower;
        output.rightPower = align.rightPower;
        output.failed = align.failed;
        output.status = align.status;
        markPhase(output.status, "ALIGN");
        if (align.failed)
        {
            phase_ = Phase::Failed;
            return output;
        }
        if (!align.completed)
        {
            return output;
        }

        lockedHeading_ = align.status.rescueZoneAlignLockedHeading;
        output.lockedHeading = lockedHeading_;
        output.lockedHeadingUpdated = std::isfinite(lockedHeading_);
        phase_ = Phase::Approach;
        approachMission_.reset();
        output.leftPower = 0.0;
        output.rightPower = 0.0;
        markPhase(output.status, "APPROACH");
        return output;
    }

    if (phase_ == Phase::Approach)
    {
        const RescueZoneApproachOutput approach = approachMission_.update(
            lockedHeading_, telemetry, now, zones, targetColor);
        output.leftPower = approach.leftPower;
        output.rightPower = approach.rightPower;
        output.failed = approach.failed;
        output.status = approach.status;
        markPhase(output.status, "APPROACH");
        if (approach.failed)
        {
            phase_ = Phase::Failed;
            return output;
        }
        if (!approach.completed)
        {
            return output;
        }
        phase_ = Phase::Success;
    }

    output.completed = phase_ == Phase::Success;
    output.failed = phase_ == Phase::Failed;
    output.status.phase = output.completed
                              ? "rescue_zone_triangle_success"
                              : "rescue_zone_triangle_failed";
    output.status.action = output.completed
                               ? "TRIÂNGULO concluído: área encontrada, alinhada e aproximada"
                               : "TRIÂNGULO interrompido por falha da etapa atual";
    output.status.progressPercent = output.completed ? 100.0 : 0.0;
    markPhase(output.status, output.completed ? "SUCCESS" : "FAILED");
    return output;
}

void RescueZoneTriangleMission::reset()
{
    phase_ = Phase::Search;
    searchMission_.reset();
    alignMission_.reset();
    approachMission_.reset();
    lockedHeading_ = std::numeric_limits<double>::quiet_NaN();
}

void RescueZoneTriangleMission::pause(std::chrono::steady_clock::time_point now)
{
    approachMission_.pause(now);
}

bool RescueZoneTriangleMission::requiresRescueZoneDetection() const
{
    return phase_ == Phase::Search || phase_ == Phase::Align ||
           phase_ == Phase::Approach;
}
