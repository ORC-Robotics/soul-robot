#include "obr/rescue_zone_search_mission.h"

#include "obr/config.h"

RescueZoneSearchOutput RescueZoneSearchMission::update(
    const RescueZoneSnapshot& zones,
    RescueZoneTargetColor targetColor)
{
    RescueZoneSearchOutput output;
    output.status.rescueZoneSearchTargetColor =
        rescueZoneTargetColorName(targetColor);

    if (completed_)
    {
        output.completed = true;
        output.status.phase = "rescue_zone_search_found";
        output.status.action = "SEARCH_ZONE concluído: área alvo encontrada";
        output.status.progressPercent = 100.0;
        output.status.rescueZoneSearchTargetDetected = true;
        output.status.rescueZoneSearchState = "FOUND";
        output.status.rescueZoneSearchCompletionReason = "FOUND";
        return output;
    }

    if (!zones.sourceFresh)
    {
        output.status.phase = "rescue_zone_search_waiting_vision";
        output.status.action = "SEARCH_ZONE pausado: aguardando visão fresh";
        output.status.rescueZoneSearchState = "SEARCHING";
        return output;
    }

    const RescueZoneObservation& target =
        targetColor == RescueZoneTargetColor::Red ? zones.red : zones.green;
    output.status.rescueZoneSearchTargetDetected = target.detected;
    if (target.detected)
    {
        completed_ = true;
        output.completed = true;
        output.status.phase = "rescue_zone_search_found";
        output.status.action = "SEARCH_ZONE concluído: área alvo encontrada";
        output.status.progressPercent = 100.0;
        output.status.rescueZoneSearchState = "FOUND";
        output.status.rescueZoneSearchCompletionReason = "FOUND";
        return output;
    }

    output.leftPower = config::kRescueZoneSearchTurnPower;
    output.rightPower = -config::kRescueZoneSearchTurnPower;
    output.status.phase = "rescue_zone_searching";
    output.status.action = "SEARCH_ZONE procurando a área selecionada";
    output.status.rescueZoneSearchState = "SEARCHING";
    return output;
}

void RescueZoneSearchMission::reset()
{
    completed_ = false;
}
