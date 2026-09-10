#pragma once

#include "obr/rescue_zone_align_mission.h"
#include "obr/rescue_zone_approach_mission.h"
#include "obr/rescue_zone_search_mission.h"

#include <chrono>
#include <limits>

struct RescueZoneTriangleOutput
{
    double leftPower = 0.0;
    double rightPower = 0.0;
    bool completed = false;
    bool failed = false;
    bool lockedHeadingUpdated = false;
    double lockedHeading = std::numeric_limits<double>::quiet_NaN();
    AutonomousStatus status;
};

// Encadeia SEARCH_ZONE, ALIGN_ZONE e APPROACH_ZONE sem reproduzir as decisões
// internas de percepção, alinhamento ou aproximação desses módulos.
class RescueZoneTriangleMission
{
public:
    RescueZoneTriangleOutput update(
        const RescueZoneSnapshot& zones,
        const Esp32TelemetrySnapshot& telemetry,
        RescueZoneTargetColor targetColor,
        std::chrono::steady_clock::time_point now =
            std::chrono::steady_clock::now());
    void reset();
    bool requiresRescueZoneDetection() const;

private:
    enum class Phase
    {
        Search,
        Align,
        Approach,
        Success,
        Failed
    };

    Phase phase_ = Phase::Search;
    RescueZoneSearchMission searchMission_;
    RescueZoneAlignMission alignMission_;
    RescueZoneApproachMission approachMission_;
    double lockedHeading_ = std::numeric_limits<double>::quiet_NaN();

    static void markPhase(AutonomousStatus& status, const char* phase);
};
