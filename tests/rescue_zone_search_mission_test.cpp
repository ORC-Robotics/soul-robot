#include "obr/config.h"
#include "obr/rescue_zone_search_mission.h"
#include "obr/robot_state.h"

#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

RescueZoneSnapshot freshZones(bool greenDetected, bool redDetected)
{
    RescueZoneSnapshot zones;
    zones.sourceFresh = true;
    zones.green.detected = greenDetected;
    zones.red.detected = redDetected;
    return zones;
}

void testSearchUsesOnlySelectedColor()
{
    RescueZoneSearchMission greenMission;
    RescueZoneSearchOutput output = greenMission.update(
        freshZones(false, true), RescueZoneTargetColor::Green);
    require(
        !output.completed &&
            output.leftPower == config::kRescueZoneSearchTurnPower &&
            output.rightPower == -config::kRescueZoneSearchTurnPower,
        "SEARCH GREEN deve ignorar RED e continuar o pivot a 0,72.");

    output = greenMission.update(
        freshZones(true, false), RescueZoneTargetColor::Green);
    require(
        output.completed && output.leftPower == 0.0 &&
            output.rightPower == 0.0 &&
            output.status.rescueZoneSearchState == "FOUND",
        "GREEN confirmado deve concluir imediatamente com PWM zero.");

    RescueZoneSearchMission redMission;
    output = redMission.update(
        freshZones(true, false), RescueZoneTargetColor::Red);
    require(
        !output.completed && output.leftPower > 0.0 && output.rightPower < 0.0,
        "SEARCH RED deve ignorar GREEN.");
    output = redMission.update(
        freshZones(false, true), RescueZoneTargetColor::Red);
    require(
        output.completed && output.leftPower == 0.0 &&
            output.rightPower == 0.0,
        "RED confirmado deve concluir imediatamente com PWM zero.");
}

void testStalePausesAndFreshResumes()
{
    RescueZoneSearchMission mission;
    RescueZoneSnapshot stale;
    RescueZoneSearchOutput output = mission.update(
        stale, RescueZoneTargetColor::Green);
    require(
        !output.completed && output.leftPower == 0.0 &&
            output.rightPower == 0.0,
        "Visão stale deve pausar SEARCH_ZONE sem concluir.");

    output = mission.update(
        freshZones(false, false), RescueZoneTargetColor::Green);
    require(
        !output.completed && output.leftPower > 0.0 && output.rightPower < 0.0,
        "SEARCH_ZONE deve retomar o pivot quando a visão voltar.");
}

void testEmergencyStopOverridesSearchPower()
{
    RobotState robotState;
    robotState.setAutonomousMission(AutonomousMission::RescueZoneSearch);
    robotState.startAutonomous();
    robotState.driveAutonomous(
        config::kRescueZoneSearchTurnPower,
        -config::kRescueZoneSearchTurnPower,
        false);
    robotState.emergencyStop();
    const RobotSnapshot snapshot = robotState.snapshot();
    require(
        snapshot.emergencyStop && snapshot.left == 0.0 &&
            snapshot.right == 0.0,
        "E-Stop deve zerar imediatamente o pivot do SEARCH_ZONE.");
}
}

int main()
{
    try
    {
        testSearchUsesOnlySelectedColor();
        testStalePausesAndFreshResumes();
        testEmergencyStopOverridesSearchPower();
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
    std::cout << "rescue_zone_search_mission_test: OK\n";
    return 0;
}
