#include "obr/config.h"
#include "obr/main_mission.h"
#include "obr/robot_state.h"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

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
    telemetry.leftEncoderRate = 0.0;
    telemetry.rightEncoderRate = 0.0;
    return telemetry;
}

CameraLineSnapshot freshVision(std::uint64_t sequence)
{
    CameraLineSnapshot snapshot;
    snapshot.sourceFresh = true;
    snapshot.lineSequence = sequence;
    return snapshot;
}

CameraLineSnapshot greenMarker(
    std::uint64_t sequence,
    GreenTurnDecision decision)
{
    CameraLineSnapshot snapshot = freshVision(sequence);
    snapshot.greenNearSeen = true;
    snapshot.greenPathBlackValid = true;
    snapshot.greenCandidateDecision = decision;
    snapshot.greenCandidateFrames = 2;
    snapshot.greenConfirmed = true;
    snapshot.greenTurnDecision = decision;
    return snapshot;
}

CameraLineSnapshot strongGreenHandoff(
    std::uint64_t sequence,
    double leftPower,
    double rightPower)
{
    CameraLineSnapshot snapshot = freshVision(sequence);
    snapshot.greenManeuverNearValid = true;
    snapshot.greenManeuverTrajectoryValid = true;
    snapshot.greenManeuverLeftPower = leftPower;
    snapshot.greenManeuverRightPower = rightPower;
    return snapshot;
}

struct MissionFixture
{
    RobotState robotState;
    MainMission mission;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    std::uint64_t sequence = 0;

    MissionFixture()
    {
        robotState.setAutonomousMission(AutonomousMission::MainMission);
        robotState.startAutonomous();
    }

    void update(const CameraLineSnapshot& snapshot, bool cameraReady = true)
    {
        mission.update(robotState, telemetry, cameraReady, snapshot);
    }
};

void completeGreenTurnToVisualHandoff(
    MissionFixture& fixture,
    GreenTurnDecision decision)
{
    fixture.telemetry.mpuOk = true;
    fixture.update(greenMarker(++fixture.sequence, decision));
    fixture.update(freshVision(++fixture.sequence));
    fixture.telemetry.yawZDeg =
        decision == GreenTurnDecision::GuideLeft
            ? -config::kGreenDirectionalTurnTargetDegrees
            : config::kGreenDirectionalTurnTargetDegrees;
    fixture.telemetry.gyroZDegPerSec = 0.0;
    fixture.update(freshVision(++fixture.sequence));
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config::kTurn90SettleMs + 20));
    fixture.update(freshVision(++fixture.sequence));
    require(
        fixture.robotState.snapshot().autonomousStatus.phase ==
            "green_turn_visual_handoff",
        "O giro verde deve chegar ao handoff visual.");
}

void testNormalLineFollowerStaysStopped()
{
    MissionFixture fixture;
    fixture.update(freshVision(++fixture.sequence));
    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(
        snapshot.mode == "autonomous" &&
            snapshot.left == 0.0 && snapshot.right == 0.0,
        "O seguidor normal ainda não implementado deve manter os motores parados.");
    require(
        snapshot.autonomousStatus.phase == "line_follower_pending",
        "A telemetria deve identificar claramente o ponto ainda não implementado.");
}

void testUnconfirmedGreenDoesNotMoveNormalState()
{
    MissionFixture fixture;
    CameraLineSnapshot snapshot = freshVision(++fixture.sequence);
    snapshot.greenNearSeen = true;
    snapshot.greenPathBlackValid = true;
    snapshot.greenCandidateDecision = GreenTurnDecision::GuideLeft;
    snapshot.greenCandidateFrames = 1;
    fixture.update(snapshot);
    const RobotSnapshot result = fixture.robotState.snapshot();
    require(
        result.left == 0.0 && result.right == 0.0 &&
            result.autonomousStatus.phase == "line_follower_pending",
        "Verde não confirmado não pode criar movimento no estado normal.");
}

void testGreenApproachOverridesNormalStop()
{
    MissionFixture fixture;
    CameraLineSnapshot snapshot = freshVision(++fixture.sequence);
    snapshot.greenManeuverNearValid = true;
    snapshot.greenManeuverLeftPower = 0.69;
    snapshot.greenManeuverRightPower = 0.69;
    snapshot.greenPathBlackValid = true;
    snapshot.greenTurnDecision = GreenTurnDecision::Approach;
    fixture.update(snapshot);
    const RobotSnapshot result = fixture.robotState.snapshot();
    require(
        result.left == 0.69 && result.right == 0.69 &&
            result.autonomousStatus.phase == "green_approach",
        "A parada normal não pode sobrescrever a aproximação verde.");
}

void testDirectionalGreenTurnAndHandoffKeepPriority()
{
    MissionFixture fixture;
    completeGreenTurnToVisualHandoff(
        fixture,
        GreenTurnDecision::GuideLeft);

    fixture.update(strongGreenHandoff(
        ++fixture.sequence,
        0.86,
        0.61));
    const RobotSnapshot handoff = fixture.robotState.snapshot();
    require(
        handoff.mode == "autonomous" &&
            handoff.left == 0.86 && handoff.right == 0.61 &&
            handoff.autonomousStatus.phase ==
                "green_turn_handoff_completed",
        "O handoff verde deve aplicar a prévia validada no mesmo frame.");

    fixture.update(freshVision(++fixture.sequence));
    const RobotSnapshot normal = fixture.robotState.snapshot();
    require(
        normal.left == 0.0 && normal.right == 0.0 &&
            normal.autonomousStatus.phase == "line_follower_pending",
        "Depois da manobra verde, o estado normal deve voltar à parada segura.");
}

void testDoubleGreenStillStartsImuTurnAround()
{
    MissionFixture fixture;
    fixture.telemetry.mpuOk = true;
    fixture.update(greenMarker(
        ++fixture.sequence,
        GreenTurnDecision::TurnAround180));
    fixture.update(greenMarker(
        ++fixture.sequence,
        GreenTurnDecision::TurnAround180));
    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(
        snapshot.autonomousStatus.phase == "green_turning" &&
            snapshot.left > 0.0 && snapshot.right < 0.0,
        "O verde duplo deve continuar iniciando o retorno por IMU.");
}

void testUnavailableCameraStopsMission()
{
    MissionFixture fixture;
    fixture.update(freshVision(++fixture.sequence), false);
    const RobotSnapshot snapshot = fixture.robotState.snapshot();
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
        testUnconfirmedGreenDoesNotMoveNormalState();
        testGreenApproachOverridesNormalStop();
        testDirectionalGreenTurnAndHandoffKeepPriority();
        testDoubleGreenStillStartsImuTurnAround();
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
