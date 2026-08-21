#include "obr/main_mission.h"
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

Esp32TelemetrySnapshot readyTelemetry()
{
    Esp32TelemetrySnapshot telemetry;
    telemetry.serialOpen = true;
    telemetry.sensorFresh = true;
    telemetry.lastSensorAgeMs = 0;
    telemetry.motorSleepPinHigh = true;
    return telemetry;
}

CameraLineSnapshot freshVision(GreenInterpretation interpretation)
{
    CameraLineSnapshot snapshot;
    snapshot.sourceFresh = true;
    snapshot.lineSequence = 1;
    snapshot.greenCandidateCount =
        interpretation == GreenInterpretation::TurnAround180 ? 2 : 1;
    snapshot.greenPathBlackValid =
        interpretation == GreenInterpretation::Left ||
        interpretation == GreenInterpretation::Right ||
        interpretation == GreenInterpretation::TurnAround180;
    snapshot.greenConfirmed = true;
    snapshot.greenInterpretation = interpretation;
    return snapshot;
}

struct MissionFixture
{
    RobotState robotState;
    MainMission mission;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();

    MissionFixture()
    {
        robotState.setAutonomousMission(AutonomousMission::MainMission);
        robotState.startAutonomous();
    }

    RobotSnapshot update(
        const CameraLineSnapshot& snapshot,
        bool cameraReady = true)
    {
        mission.update(robotState, telemetry, cameraReady, snapshot);
        return robotState.snapshot();
    }
};

void requireStoppedByPendingFollower(
    const RobotSnapshot& snapshot,
    const std::string& context)
{
    require(
        snapshot.mode == "autonomous" &&
            snapshot.left == 0.0 && snapshot.right == 0.0,
        context + ": a percepção não pode movimentar os motores.");
    require(
        snapshot.autonomousStatus.phase == "line_follower_pending",
        context + ": a missão deve permanecer no seguidor pendente.");
}

void testNormalLineFollowerStaysStopped()
{
    MissionFixture fixture;
    requireStoppedByPendingFollower(
        fixture.update(freshVision(GreenInterpretation::None)),
        "Sem marcador verde");
}

void testEveryGreenClassificationStaysStopped()
{
    const GreenInterpretation interpretations[] = {
        GreenInterpretation::FalseMarker,
        GreenInterpretation::Ambiguous,
        GreenInterpretation::Left,
        GreenInterpretation::Right,
        GreenInterpretation::TurnAround180,
    };
    for (const GreenInterpretation interpretation : interpretations)
    {
        MissionFixture fixture;
        requireStoppedByPendingFollower(
            fixture.update(freshVision(interpretation)),
            "Classificação verde");
    }
}

void testUnavailableCameraStopsMission()
{
    MissionFixture fixture;
    const RobotSnapshot snapshot = fixture.update(
        freshVision(GreenInterpretation::Left), false);
    require(
        snapshot.mode == "stopped" &&
            snapshot.left == 0.0 && snapshot.right == 0.0 &&
            snapshot.autonomousStatus.phase == "camera_not_ready",
        "Câmera indisponível deve encerrar a missão com motores zerados.");
}
}

int main()
{
    try
    {
        testNormalLineFollowerStaysStopped();
        testEveryGreenClassificationStaysStopped();
        testUnavailableCameraStopsMission();
        std::cout << "main_mission_test: OK\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "main_mission_test: " << error.what() << '\n';
        return 1;
    }
}
