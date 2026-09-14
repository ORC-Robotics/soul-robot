#include "obr/config.h"
#include "obr/rescue_zone_triangle_mission.h"

#include <chrono>
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

Esp32TelemetrySnapshot readyTelemetry(double distanceCm, double yawDegrees)
{
    Esp32TelemetrySnapshot telemetry;
    telemetry.serialOpen = true;
    telemetry.sensorFresh = true;
    telemetry.mpuOk = true;
    telemetry.motorSleepPinHigh = true;
    telemetry.lastSensorAgeMs = 10;
    telemetry.ultrasonicDistanceCm = distanceCm;
    telemetry.yawZDeg = yawDegrees;
    telemetry.gyroZDegPerSec = 0.0;
    return telemetry;
}

RescueZoneSnapshot foundUnknownGreen(std::uint64_t sequence)
{
    RescueZoneSnapshot zones;
    zones.sourceFresh = true;
    zones.sequence = sequence;
    zones.timestamp = static_cast<double>(sequence);
    zones.green.candidateDetected = true;
    zones.green.detected = true;
    zones.green.geometryState = RescueZoneGeometryState::BoundsUnknown;
    return zones;
}

RescueZoneSnapshot missingZones(std::uint64_t sequence)
{
    RescueZoneSnapshot zones;
    zones.sourceFresh = true;
    zones.sequence = sequence;
    zones.timestamp = static_cast<double>(sequence);
    return zones;
}

RescueZoneSnapshot foundUnknownRed(std::uint64_t sequence)
{
    RescueZoneSnapshot zones = missingZones(sequence);
    zones.red.candidateDetected = true;
    zones.red.detected = true;
    zones.red.geometryState = RescueZoneGeometryState::BoundsUnknown;
    return zones;
}

void testOrchestratesSearchAlignApproachAndSuccess()
{
    RescueZoneTriangleMission mission;
    const auto start = std::chrono::steady_clock::now();
    const Esp32TelemetrySnapshot telemetry = readyTelemetry(6.0, 37.0);
    require(
        mission.requiresRescueZoneDetection(),
        "TRIÂNGULO deve manter a visão ativa durante SEARCH.");

    RescueZoneTriangleOutput output = mission.update(
        foundUnknownGreen(1), telemetry, RescueZoneTargetColor::Green, start);
    require(
        !output.completed && output.leftPower == 0.0 &&
            output.status.rescueZoneTrianglePhase == "ALIGN",
        "FOUND deve transferir automaticamente o TRIÂNGULO para ALIGN.");
    require(
        mission.requiresRescueZoneDetection(),
        "TRIÂNGULO deve manter a visão ativa durante ALIGN.");

    output = mission.update(
        foundUnknownGreen(2), telemetry, RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(1));
    require(
        output.lockedHeadingUpdated && output.lockedHeading == 37.0 &&
            output.status.rescueZoneTrianglePhase == "APPROACH" &&
            output.leftPower == 0.0,
        "ALIGN concluído deve salvar heading e transferir para APPROACH.");
    require(
        mission.requiresRescueZoneDetection(),
        "TRIÂNGULO deve manter a visão durante APPROACH para detectar obstrução.");

    output = mission.update(
        foundUnknownGreen(3), telemetry, RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(2));
    require(
        !output.completed &&
            output.status.rescueZoneTrianglePhase == "APPROACH" &&
            output.leftPower == config::kRescueZoneApproachFinalAdvancePower,
        "APPROACH existente deve comandar sua aproximação sem lógica duplicada.");

    output = mission.update(
        foundUnknownGreen(4), telemetry, RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(
                    config::kRescueZoneApproachFinalAdvanceMs + 2));
    require(
        output.completed && !output.failed && output.leftPower == 0.0 &&
            output.rightPower == 0.0 &&
            output.status.rescueZoneTrianglePhase == "SUCCESS",
        "APPROACH concluído deve finalizar o TRIÂNGULO em SUCCESS parado.");
}

void testEmergencyStopOverridesEveryTrianglePhase()
{
    for (const std::string& phase : {"SEARCH", "ALIGN", "APPROACH"})
    {
        RobotState robotState;
        robotState.setAutonomousMission(AutonomousMission::RescueZoneTriangle);
        robotState.startAutonomous();
        robotState.driveAutonomous(0.72, -0.72, false);
        robotState.emergencyStop();
        const RobotSnapshot snapshot = robotState.snapshot();
        require(
            snapshot.emergencyStop && snapshot.left == 0.0 &&
                snapshot.right == 0.0,
            "E-Stop deve zerar o TRIÂNGULO durante " + phase + ".");
    }
}

void testLostZoneReturnsFromAlignToSearch()
{
    RescueZoneTriangleMission mission;
    const auto start = std::chrono::steady_clock::now();
    const Esp32TelemetrySnapshot telemetry = readyTelemetry(30.0, 0.0);

    RescueZoneTriangleOutput output = mission.update(
        foundUnknownRed(1), telemetry, RescueZoneTargetColor::Red, start);
    require(
        output.status.rescueZoneTrianglePhase == "ALIGN",
        "A zona encontrada deve iniciar ALIGN antes do teste de perda.");

    output = mission.update(
        missingZones(2), telemetry, RescueZoneTargetColor::Red,
        start + std::chrono::milliseconds(20));
    require(
        output.status.rescueZoneTrianglePhase == "SEARCH" &&
            output.status.phase == "rescue_zone_searching" &&
            output.leftPower == config::kRescueZoneSearchTurnPower &&
            output.rightPower == -config::kRescueZoneSearchTurnPower,
        "A perda confirmada durante ALIGN deve retomar SEARCH com movimento.");
}
}

int main()
{
    try
    {
        testOrchestratesSearchAlignApproachAndSuccess();
        testEmergencyStopOverridesEveryTrianglePhase();
        testLostZoneReturnsFromAlignToSearch();
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
    std::cout << "rescue_zone_triangle_mission_test: OK\n";
    return 0;
}
