#include "obr/config.h"
#include "obr/rescue_zone_align_mission.h"

#include <chrono>
#include <cmath>
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

Esp32TelemetrySnapshot readyTelemetry(double yawDegrees = 0.0)
{
    Esp32TelemetrySnapshot telemetry;
    telemetry.serialOpen = true;
    telemetry.sensorFresh = true;
    telemetry.mpuOk = true;
    telemetry.motorSleepPinHigh = true;
    telemetry.lastSensorAgeMs = 0;
    telemetry.yawZDeg = yawDegrees;
    telemetry.gyroZDegPerSec = 0.0;
    return telemetry;
}

RescueZoneSnapshot zoneSnapshot(
    bool aimValid,
    double aimNormalized,
    std::uint64_t sequence = 1)
{
    RescueZoneSnapshot snapshot;
    snapshot.sourceFresh = true;
    snapshot.sequence = sequence;
    snapshot.timestamp = static_cast<double>(sequence);
    snapshot.green.candidateDetected = true;
    snapshot.green.detected = true;
    snapshot.green.geometryState = RescueZoneGeometryState::FullBounds;
    snapshot.green.aimValid = aimValid;
    snapshot.green.aimX = aimValid ? 480.0 : NAN;
    snapshot.green.aimNormalized = aimValid ? aimNormalized : NAN;
    return snapshot;
}

RescueZoneSnapshot partialZoneSnapshot(
    RescueZoneGeometryState geometryState,
    std::uint64_t sequence = 1)
{
    RescueZoneSnapshot snapshot = zoneSnapshot(false, NAN, sequence);
    snapshot.green.geometryState = geometryState;
    return snapshot;
}

void testLeftAimPivotsLeft()
{
    RescueZoneAlignMission mission;
    RescueZoneAlignOutput output = mission.update(
        zoneSnapshot(true, -0.35),
        readyTelemetry(),
        RescueZoneTargetColor::Green);
    require(
        output.leftPower == -config::kRescueZoneAlignTurnPower &&
            output.rightPower == config::kRescueZoneAlignTurnPower &&
            output.status.rescueZoneAlignState == "LEFT",
        "Aim à esquerda deve executar o micro-pivô para a esquerda.");
}

void testRightAimPivotsRight()
{
    RescueZoneAlignMission mission;
    const RescueZoneAlignOutput output = mission.update(
        zoneSnapshot(true, 0.35),
        readyTelemetry(),
        RescueZoneTargetColor::Green);
    require(
        output.leftPower == config::kRescueZoneAlignTurnPower &&
            output.rightPower == -config::kRescueZoneAlignTurnPower &&
            output.status.rescueZoneAlignState == "RIGHT",
        "Aim à direita deve executar o micro-pivô para a direita.");
}

void testCenterDoesNotMoveAndRequiresDistinctFrames()
{
    const auto start = std::chrono::steady_clock::now();
    RescueZoneAlignMission mission;
    RescueZoneAlignOutput output = mission.update(
        zoneSnapshot(true, 0.10, 1),
        readyTelemetry(20.0),
        RescueZoneTargetColor::Green,
        start);
    require(
        !output.completed && output.leftPower == 0.0 &&
            output.rightPower == 0.0 &&
            output.status.rescueZoneAlignState == "CENTER",
        "O limite inclusivo da deadband deve permanecer parado.");

    output = mission.update(
        zoneSnapshot(true, 0.10, 1),
        readyTelemetry(21.0),
        RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(10));
    require(
        !output.completed,
        "Reprocessar o mesmo frame não pode confirmar o centro duas vezes.");

    output = mission.update(
        zoneSnapshot(true, -0.02, 2),
        readyTelemetry(22.5),
        RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(20));
    require(
        output.completed && !output.failed && output.leftPower == 0.0 &&
            output.rightPower == 0.0 &&
            output.status.rescueZoneAlignCompletionReason == "ALIGNED" &&
            output.status.rescueZoneAlignLockedHeading == 22.5,
        "Dois frames centrais devem concluir e salvar o yaw atual.");
}

void testInvalidAimWaitsStoppedUntilItBecomesUsable()
{
    RescueZoneAlignMission mission;
    RescueZoneAlignOutput output = mission.update(
        zoneSnapshot(false, NAN),
        readyTelemetry(-37.0),
        RescueZoneTargetColor::Green);
    require(
        !output.completed && !output.failed && output.leftPower == 0.0 &&
            output.rightPower == 0.0 &&
            output.status.rescueZoneAlignState == "UNAVAILABLE" &&
            output.status.phase == "rescue_zone_align_waiting_aim",
        "Aim inválido deve pausar sem concluir nem mover.");

    output = mission.update(
        zoneSnapshot(true, 0.30, 2),
        readyTelemetry(-37.0),
        RescueZoneTargetColor::Green);
    require(
        output.leftPower == config::kRescueZoneAlignTurnPower &&
            output.rightPower == -config::kRescueZoneAlignTurnPower,
        "Aim válido posterior deve retomar a centralização.");
}

void testRedTargetUsesRedAim()
{
    RescueZoneAlignMission mission;
    RescueZoneSnapshot zones = zoneSnapshot(true, -0.40);
    zones.red = zones.green;
    zones.red.aimNormalized = 0.40;
    zones.green = {};
    const RescueZoneAlignOutput output = mission.update(
        zones, readyTelemetry(), RescueZoneTargetColor::Red);
    require(
        output.leftPower == config::kRescueZoneAlignTurnPower &&
            output.rightPower == -config::kRescueZoneAlignTurnPower &&
            output.status.rescueZoneAlignState == "RIGHT",
        "A seleção RED deve usar somente o aimNormalized da zona vermelha.");
}

void testAimLossPausesActivePivotAndCanResume()
{
    const auto start = std::chrono::steady_clock::now();
    RescueZoneAlignMission mission;
    mission.update(
        zoneSnapshot(true, 0.40, 1),
        readyTelemetry(5.0),
        RescueZoneTargetColor::Green,
        start);
    RescueZoneAlignOutput output = mission.update(
        zoneSnapshot(false, NAN, 2),
        readyTelemetry(6.0),
        RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(30));
    require(
        !output.completed && !output.failed && output.leftPower == 0.0 &&
            output.rightPower == 0.0 &&
            output.status.phase == "rescue_zone_align_waiting_aim",
        "Perder aim durante o pulso deve apenas pausar imediatamente.");

    output = mission.update(
        zoneSnapshot(true, 0.30, 3),
        readyTelemetry(6.0),
        RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(140));
    require(
        output.leftPower == config::kRescueZoneAlignTurnPower &&
            output.rightPower == -config::kRescueZoneAlignTurnPower,
        "Aim válido em frame posterior à parada deve retomar o alinhamento.");
}

void testMicroPivotUsesFixedSettlingAndNewFrame()
{
    const auto start = std::chrono::steady_clock::now();
    RescueZoneAlignMission mission;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    RescueZoneAlignOutput output = mission.update(
        zoneSnapshot(true, -0.30, 1),
        telemetry,
        RescueZoneTargetColor::Green,
        start);
    require(output.leftPower < 0.0, "O primeiro pulso deve iniciar.");

    output = mission.update(
        zoneSnapshot(true, -0.30, 2),
        telemetry,
        RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(config::kRescueSearchPulseMs));
    require(
        output.leftPower == 0.0 && output.rightPower == 0.0,
        "O pulso deve terminar exatamente com PWM zero.");

    telemetry.leftEncoderRate = NAN;
    telemetry.rightEncoderRate = NAN;
    output = mission.update(
        zoneSnapshot(true, -0.30, 2),
        telemetry,
        RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(config::kRescueSearchPulseMs + config::kRescueSearchSettlingMs));
    require(
        output.leftPower == 0.0 && output.rightPower == 0.0 &&
            output.status.phase == "rescue_zone_align_waiting_new_frame",
        "Settling não deve consultar encoders nem reutilizar o frame antigo.");

    output = mission.update(
        zoneSnapshot(true, -0.30, 3),
        telemetry,
        RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(config::kRescueSearchPulseMs + config::kRescueSearchSettlingMs + 1));
    require(
        output.leftPower == -config::kRescueZoneAlignTurnPower &&
            output.rightPower == config::kRescueZoneAlignTurnPower,
        "Um frame posterior deve liberar o próximo micro-pivô.");
}

void testPartialBoundsKeepTryingAfterThreePivots()
{
    const auto start = std::chrono::steady_clock::now();
    RescueZoneAlignMission mission;
    const Esp32TelemetrySnapshot telemetry = readyTelemetry(31.0);
    RescueZoneAlignOutput output = mission.update(
        partialZoneSnapshot(RescueZoneGeometryState::LeftBoundOnly, 1),
        telemetry,
        RescueZoneTargetColor::Green,
        start);
    require(
        output.leftPower == config::kRescueZoneAlignTurnPower &&
            output.rightPower == -config::kRescueZoneAlignTurnPower,
        "LEFT_BOUND_ONLY deve procurar a borda direita com micro-pivô.");

    for (int attempt = 0; attempt < 3; ++attempt)
    {
        const int pulseEndMs = config::kRescueSearchPulseMs + attempt * (config::kRescueSearchPulseMs + config::kRescueSearchSettlingMs);
        output = mission.update(
            partialZoneSnapshot(
                RescueZoneGeometryState::LeftBoundOnly,
                static_cast<std::uint64_t>(2 + attempt * 2)),
            telemetry,
            RescueZoneTargetColor::Green,
            start + std::chrono::milliseconds(pulseEndMs));
        require(
            output.leftPower == 0.0 && output.rightPower == 0.0,
            "Cada tentativa de bounds deve terminar com PWM zero.");

        output = mission.update(
            partialZoneSnapshot(
                RescueZoneGeometryState::LeftBoundOnly,
                static_cast<std::uint64_t>(3 + attempt * 2)),
            telemetry,
            RescueZoneTargetColor::Green,
            start + std::chrono::milliseconds(pulseEndMs + config::kRescueSearchSettlingMs));
    }

    require(
        !output.completed && !output.failed &&
            output.leftPower == config::kRescueZoneAlignTurnPower &&
            output.rightPower == -config::kRescueZoneAlignTurnPower,
        "Bounds parciais devem iniciar o quarto micro-pivô sem best effort.");
}

void testPartialBoundsCanTransitionToFullAlignment()
{
    const auto start = std::chrono::steady_clock::now();
    RescueZoneAlignMission mission;
    const Esp32TelemetrySnapshot telemetry = readyTelemetry();
    mission.update(
        partialZoneSnapshot(RescueZoneGeometryState::RightBoundOnly, 1),
        telemetry,
        RescueZoneTargetColor::Green,
        start);

    RescueZoneAlignOutput output = mission.update(
        zoneSnapshot(true, 0.35, 2),
        telemetry,
        RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(40));
    require(
        output.leftPower == 0.0 && output.rightPower == 0.0,
        "FULL_BOUNDS deve interromper o pulso de busca antes de centralizar.");

    output = mission.update(
        zoneSnapshot(true, 0.35, 3),
        telemetry,
        RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(140));
    require(
        output.leftPower == config::kRescueZoneAlignTurnPower &&
            output.rightPower == -config::kRescueZoneAlignTurnPower,
        "Depois do settling, FULL_BOUNDS deve centralizar pelo aimNormalized.");
}

void testBoundsUnknownCompletesStopped()
{
    RescueZoneAlignMission mission;
    const RescueZoneAlignOutput output = mission.update(
        partialZoneSnapshot(RescueZoneGeometryState::BoundsUnknown),
        readyTelemetry(-14.0),
        RescueZoneTargetColor::Green);
    require(
        output.completed && !output.failed && output.leftPower == 0.0 &&
            output.rightPower == 0.0 &&
            output.status.rescueZoneAlignCompletionReason ==
                "BEST_EFFORT_BOUNDS_UNKNOWN" &&
            output.status.rescueZoneAlignLockedHeading == -14.0,
        "BOUNDS_UNKNOWN deve concluir parado, sem fabricar bordas.");
}

void testStaleVisionAndInvalidImuPauseAndResume()
{
    const auto start = std::chrono::steady_clock::now();
    RescueZoneAlignMission staleMission;
    RescueZoneSnapshot stale;
    RescueZoneAlignOutput output = staleMission.update(
        stale, readyTelemetry(), RescueZoneTargetColor::Green, start);
    require(
        !output.failed && output.leftPower == 0.0 && output.rightPower == 0.0 &&
            output.status.phase == "rescue_zone_align_waiting_vision",
        "O gate deve ter tempo para publicar o primeiro snapshot, sempre parado.");
    output = staleMission.update(
        stale,
        readyTelemetry(),
        RescueZoneTargetColor::Green,
        start + std::chrono::seconds(30));
    require(
        !output.failed && !output.completed && output.leftPower == 0.0 &&
            output.rightPower == 0.0,
        "Visão stale indefinidamente deve permanecer pausada e não falhar.");

    output = staleMission.update(
        zoneSnapshot(true, -0.30, 1),
        readyTelemetry(),
        RescueZoneTargetColor::Green,
        start + std::chrono::seconds(31));
    require(
        output.leftPower == -config::kRescueZoneAlignTurnPower &&
            output.rightPower == config::kRescueZoneAlignTurnPower,
        "Primeiro snapshot fresh deve retomar o alinhamento sem timeout.");

    RescueZoneAlignMission lostVisionMission;
    lostVisionMission.update(
        zoneSnapshot(true, 0.40),
        readyTelemetry(),
        RescueZoneTargetColor::Green,
        start);
    output = lostVisionMission.update(
        stale,
        readyTelemetry(),
        RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(10));
    require(
        !output.failed && !output.completed && output.leftPower == 0.0 &&
            output.rightPower == 0.0,
        "Stale depois de movimento deve pausar imediatamente sem abortar.");

    output = lostVisionMission.update(
        zoneSnapshot(true, 0.30, 2),
        readyTelemetry(),
        RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(120));
    require(
        output.leftPower == config::kRescueZoneAlignTurnPower &&
            output.rightPower == -config::kRescueZoneAlignTurnPower,
        "Frame fresh posterior à parada deve retomar após stale.");

    RescueZoneAlignMission imuMission;
    Esp32TelemetrySnapshot invalidImu = readyTelemetry();
    invalidImu.mpuOk = false;
    output = imuMission.update(
        zoneSnapshot(true, 0.40),
        invalidImu,
        RescueZoneTargetColor::Green,
        start);
    require(
        !output.failed && !output.completed && output.leftPower == 0.0 &&
            output.rightPower == 0.0 &&
            output.status.phase == "rescue_zone_align_waiting_imu",
        "IMU inválida deve pausar com motores zerados.");

    output = imuMission.update(
        zoneSnapshot(true, 0.40, 2),
        readyTelemetry(),
        RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(10));
    require(
        output.leftPower == config::kRescueZoneAlignTurnPower &&
            output.rightPower == -config::kRescueZoneAlignTurnPower,
        "IMU válida posterior deve retomar o alinhamento.");
}

void testLongRunningAlignmentHasNoGlobalTimeout()
{
    const auto start = std::chrono::steady_clock::now();
    RescueZoneAlignMission mission;
    mission.update(
        zoneSnapshot(true, 0.40, 1),
        readyTelemetry(),
        RescueZoneTargetColor::Green,
        start);
    RescueZoneAlignOutput output = mission.update(
        zoneSnapshot(true, 0.40, 2),
        readyTelemetry(9.0),
        RescueZoneTargetColor::Green,
        start + std::chrono::hours(1));
    require(
        !output.failed && !output.completed && output.leftPower == 0.0 &&
            output.rightPower == 0.0,
        "Execução longa deve apenas terminar o pulso atual, sem timeout.");

    output = mission.update(
        zoneSnapshot(true, 0.40, 3),
        readyTelemetry(9.0),
        RescueZoneTargetColor::Green,
        start + std::chrono::hours(1) + std::chrono::milliseconds(101));
    require(
        output.leftPower == config::kRescueZoneAlignTurnPower &&
            output.rightPower == -config::kRescueZoneAlignTurnPower,
        "Depois de uma hora, novo frame deve continuar liberando micro-pivô.");
}

void testEmergencyStopOverridesAlignCommand()
{
    RobotState robotState;
    robotState.setAutonomousMission(AutonomousMission::RescueZoneAlign);
    robotState.startAutonomous();
    robotState.driveAutonomous(-0.69, 0.69);
    robotState.emergencyStop();
    robotState.driveAutonomous(-0.69, 0.69);
    const RobotSnapshot snapshot = robotState.snapshot();
    require(
        snapshot.emergencyStop && snapshot.mode == "emergency" &&
            snapshot.left == 0.0 && snapshot.right == 0.0,
        "E-Stop deve prevalecer sobre qualquer pulso do ALIGN_ZONE.");
}
}

int main()
{
    try
    {
        testLeftAimPivotsLeft();
        testRightAimPivotsRight();
        testCenterDoesNotMoveAndRequiresDistinctFrames();
        testInvalidAimWaitsStoppedUntilItBecomesUsable();
        testRedTargetUsesRedAim();
        testAimLossPausesActivePivotAndCanResume();
        testMicroPivotUsesFixedSettlingAndNewFrame();
        testPartialBoundsKeepTryingAfterThreePivots();
        testPartialBoundsCanTransitionToFullAlignment();
        testBoundsUnknownCompletesStopped();
        testStaleVisionAndInvalidImuPauseAndResume();
        testLongRunningAlignmentHasNoGlobalTimeout();
        testEmergencyStopOverridesAlignCommand();
        std::cout << "rescue_zone_align_mission_test: OK\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "rescue_zone_align_mission_test: " << error.what() << '\n';
        return 1;
    }
}
