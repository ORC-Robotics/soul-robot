#include "obr/config.h"
#include "obr/rescue_zone_frame_mission.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
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
    telemetry.mpuOk = true;
    telemetry.motorSleepPinHigh = true;
    telemetry.lastSensorAgeMs = 0;
    telemetry.leftEncoderRate = 0.0;
    telemetry.rightEncoderRate = 0.0;
    telemetry.yawZDeg = 0.0;
    telemetry.gyroZDegPerSec = 0.0;
    return telemetry;
}

RescueZoneSnapshot zoneSnapshot(
    RescueZoneGeometryState geometryState,
    std::uint64_t sequence = 1)
{
    RescueZoneSnapshot snapshot;
    snapshot.sourceFresh = true;
    snapshot.sequence = sequence;
    snapshot.timestamp = static_cast<double>(sequence);
    snapshot.green.candidateDetected = true;
    snapshot.green.detected = true;
    snapshot.green.geometryState = geometryState;
    snapshot.green.aimValid =
        geometryState == RescueZoneGeometryState::FullBounds;
    snapshot.green.aimX = snapshot.green.aimValid ? 480.0 : NAN;
    return snapshot;
}

void testMissingZoneStaysStopped()
{
    RescueZoneFrameMission mission;
    RescueZoneSnapshot zones;
    zones.sourceFresh = true;
    zones.sequence = 1;

    const RescueZoneFrameOutput output = mission.update(
        zones, readyTelemetry(), RescueZoneTargetColor::Green);
    require(
        !output.completed && !output.failed && output.leftPower == 0.0 &&
            output.rightPower == 0.0,
        "Zona ausente deve manter o FRAME_ZONE parado sem inventar movimento.");
}

void testFullBoundsCompletesWithoutMoving()
{
    RescueZoneFrameMission mission;
    const RescueZoneFrameOutput output = mission.update(
        zoneSnapshot(RescueZoneGeometryState::FullBounds),
        readyTelemetry(),
        RescueZoneTargetColor::Green);
    require(
        output.completed && !output.failed && output.leftPower == 0.0 &&
            output.rightPower == 0.0 &&
            output.status.rescueZoneFrameCompletionReason == "FULL_BOUNDS",
        "FULL_BOUNDS com aim válido deve concluir parado com motivo explícito.");
}

void testFullBoundsWithoutAimCompletesBestEffort()
{
    RescueZoneFrameMission mission;
    RescueZoneSnapshot zones =
        zoneSnapshot(RescueZoneGeometryState::FullBounds);
    zones.green.aimValid = false;
    zones.green.aimX = NAN;
    const RescueZoneFrameOutput output = mission.update(
        zones, readyTelemetry(), RescueZoneTargetColor::Green);
    require(
        output.completed && !output.failed && output.leftPower == 0.0 &&
            output.rightPower == 0.0 &&
            output.status.rescueZoneFrameCompletionReason ==
                "BEST_EFFORT_FULL_BOUNDS_NO_AIM",
        "aimValid não deve continuar sendo uma obrigação para concluir.");
}

void testBoundsUnknownCompletesBestEffortWithoutReverseOrEncoders()
{
    RescueZoneFrameMission mission;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    telemetry.sensorFresh = false;
    telemetry.leftEncoderRate = NAN;
    telemetry.rightEncoderRate = NAN;
    telemetry.leftEncoderCount = 50000;
    telemetry.rightEncoderCount = -30000;

    const RescueZoneFrameOutput output = mission.update(
        zoneSnapshot(RescueZoneGeometryState::BoundsUnknown),
        telemetry,
        RescueZoneTargetColor::Green);
    require(
        output.completed && !output.failed && output.leftPower == 0.0 &&
            output.rightPower == 0.0 &&
            output.status.rescueZoneFrameCompletionReason ==
                "BEST_EFFORT_BOUNDS_UNKNOWN",
        "BOUNDS_UNKNOWN confirmado deve concluir parado e sem depender de encoder.");
}

void testSingleBoundsTurnInOppositeDirections()
{
    RescueZoneFrameMission leftMission;
    const RescueZoneFrameOutput leftOutput = leftMission.update(
        zoneSnapshot(RescueZoneGeometryState::LeftBoundOnly),
        readyTelemetry(),
        RescueZoneTargetColor::Green);
    require(
        std::abs(leftOutput.leftPower - 0.69) <= 1e-9 &&
            std::abs(leftOutput.rightPower + 0.69) <= 1e-9,
        "LEFT_BOUND_ONLY deve executar o micro-pivô para a direita.");

    RescueZoneFrameMission rightMission;
    const RescueZoneFrameOutput rightOutput = rightMission.update(
        zoneSnapshot(RescueZoneGeometryState::RightBoundOnly),
        readyTelemetry(),
        RescueZoneTargetColor::Green);
    require(
        std::abs(rightOutput.leftPower + 0.69) <= 1e-9 &&
            std::abs(rightOutput.rightPower - 0.69) <= 1e-9,
        "RIGHT_BOUND_ONLY deve executar o micro-pivô para a esquerda.");
}

void testLateralMicroPivotStopsAndRequiresNewFrame()
{
    const auto start = std::chrono::steady_clock::now();
    RescueZoneFrameMission mission;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    RescueZoneFrameOutput output = mission.update(
        zoneSnapshot(RescueZoneGeometryState::LeftBoundOnly, 1),
        telemetry,
        RescueZoneTargetColor::Green,
        start);
    require(
        output.leftPower == 0.69 && output.rightPower == -0.69,
        "O micro-pivô deve iniciar diretamente para a direita.");

    output = mission.update(
        zoneSnapshot(RescueZoneGeometryState::LeftBoundOnly, 2),
        telemetry,
        RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(80));
    require(
        !output.failed && output.leftPower == 0.0 && output.rightPower == 0.0 &&
            output.status.phase == "rescue_zone_frame_braking",
        "Ao completar 80 ms, o micro-pivô deve zerar o PWM.");

    output = mission.update(
        zoneSnapshot(RescueZoneGeometryState::LeftBoundOnly, 2),
        telemetry,
        RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(300));
    require(
        output.leftPower == 0.0 && output.rightPower == 0.0 &&
            output.status.phase == "rescue_zone_frame_waiting_new_frame",
        "Após o settling, a geometria anterior não pode iniciar outro pulso.");

    output = mission.update(
        zoneSnapshot(RescueZoneGeometryState::LeftBoundOnly, 3),
        telemetry,
        RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(310));
    require(
        output.leftPower == 0.69 && output.rightPower == -0.69,
        "Um frame novo no mesmo estado deve iniciar outro micro-pivô.");
}

void testPartialBoundCompletesBestEffortAfterConfiguredAttempts()
{
    require(
        config::kRescueZoneFrameMaximumMicroPivots == 3,
        "A regressão espera três tentativas laterais configuradas.");
    const auto start = std::chrono::steady_clock::now();
    RescueZoneFrameMission mission;
    const Esp32TelemetrySnapshot telemetry = readyTelemetry();

    RescueZoneFrameOutput output = mission.update(
        zoneSnapshot(RescueZoneGeometryState::LeftBoundOnly, 1),
        telemetry,
        RescueZoneTargetColor::Green,
        start);
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        const int pulseEndMs = 80 + attempt * 180;
        output = mission.update(
            zoneSnapshot(
                RescueZoneGeometryState::LeftBoundOnly,
                static_cast<std::uint64_t>(2 + attempt * 2)),
            telemetry,
            RescueZoneTargetColor::Green,
            start + std::chrono::milliseconds(pulseEndMs));
        require(
            output.leftPower == 0.0 && output.rightPower == 0.0,
            "Cada tentativa lateral deve terminar com PWM zerado.");

        output = mission.update(
            zoneSnapshot(
                RescueZoneGeometryState::LeftBoundOnly,
                static_cast<std::uint64_t>(3 + attempt * 2)),
            telemetry,
            RescueZoneTargetColor::Green,
            start + std::chrono::milliseconds(pulseEndMs + 100));
    }
    require(
        output.completed && !output.failed && output.leftPower == 0.0 &&
            output.rightPower == 0.0 &&
            output.status.rescueZoneFrameCompletionReason ==
                "BEST_EFFORT_PARTIAL_BOUND",
        "Bounds parciais devem concluir best effort após poucas tentativas.");
}

void testFullBoundsStopsActiveMicroPivotImmediately()
{
    const auto start = std::chrono::steady_clock::now();
    RescueZoneFrameMission mission;
    const Esp32TelemetrySnapshot telemetry = readyTelemetry();
    mission.update(
        zoneSnapshot(RescueZoneGeometryState::LeftBoundOnly, 1),
        telemetry,
        RescueZoneTargetColor::Green,
        start);
    RescueZoneFrameOutput output = mission.update(
        zoneSnapshot(RescueZoneGeometryState::FullBounds, 2),
        telemetry,
        RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(40));
    require(
        !output.failed && output.leftPower == 0.0 && output.rightPower == 0.0,
        "FULL_BOUNDS deve interromper imediatamente um micro-pivô ativo.");

    output = mission.update(
        zoneSnapshot(RescueZoneGeometryState::FullBounds, 3),
        telemetry,
        RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(140));
    require(
        output.completed &&
            output.status.rescueZoneFrameCompletionReason == "FULL_BOUNDS",
        "O frame novo FULL_BOUNDS deve concluir com o aim atual preservado.");
}

void testLateralPivotUsesImuOnlyAsMaximumAngleGuard()
{
    const auto start = std::chrono::steady_clock::now();
    RescueZoneFrameMission mission;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    mission.update(
        zoneSnapshot(RescueZoneGeometryState::RightBoundOnly, 1),
        telemetry,
        RescueZoneTargetColor::Green,
        start);

    telemetry.yawZDeg = 13.0;
    RescueZoneFrameOutput output = mission.update(
        zoneSnapshot(RescueZoneGeometryState::RightBoundOnly, 2),
        telemetry,
        RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(20));
    require(!output.failed, "A primeira parcela angular deve manter o pulso.");

    // O retorno a zero soma 26 graus percorridos, apesar do deslocamento líquido
    // nulo. A IMU atua somente como proteção da operação lateral acumulada.
    telemetry.yawZDeg = 0.0;
    output = mission.update(
        zoneSnapshot(RescueZoneGeometryState::RightBoundOnly, 3),
        telemetry,
        RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(40));
    require(
        output.failed && output.leftPower == 0.0 && output.rightPower == 0.0 &&
            output.status.phase == "rescue_zone_frame_pivot_angle_limit",
        "A IMU deve interromper somente no limite angular total de segurança.");
}

void testStaleVisionRemainsStopped()
{
    const auto start = std::chrono::steady_clock::now();
    RescueZoneFrameMission mission;
    RescueZoneSnapshot stale;
    RescueZoneFrameOutput output = mission.update(
        stale, readyTelemetry(), RescueZoneTargetColor::Green, start);
    require(
        !output.failed && output.leftPower == 0.0 && output.rightPower == 0.0,
        "A partida stale deve aguardar parada.");
    output = mission.update(
        stale,
        readyTelemetry(),
        RescueZoneTargetColor::Green,
        start + std::chrono::milliseconds(
                    config::kRescueZoneFrameStartupVisionTimeoutMs + 1));
    require(
        output.failed && output.leftPower == 0.0 && output.rightPower == 0.0,
        "Visão stale até o timeout deve falhar com motores parados.");
}

void testRedTargetIsIndependent()
{
    RescueZoneFrameMission mission;
    RescueZoneSnapshot zones =
        zoneSnapshot(RescueZoneGeometryState::FullBounds);
    zones.red = zones.green;
    zones.green = {};
    const RescueZoneFrameOutput output = mission.update(
        zones, readyTelemetry(), RescueZoneTargetColor::Red);
    require(output.completed, "A seleção RED deve avaliar somente a zona vermelha.");
}

void testTerminalStatusSurvivesSafeStop()
{
    RobotState robotState;
    robotState.setAutonomousMission(AutonomousMission::RescueZoneFrame);
    robotState.startAutonomous();
    robotState.stop();

    AutonomousStatus status;
    status.phase = "rescue_zone_frame_completed";
    status.action = "FRAME_ZONE concluído";
    status.progressPercent = 100.0;
    status.rescueZoneFrameCompleted = true;
    status.rescueZoneFrameCompletionReason = "BEST_EFFORT_PARTIAL_BOUND";
    robotState.updateAutonomousStatus(status);
    const RobotSnapshot snapshot = robotState.snapshot();
    require(
        snapshot.mode == "stopped" && snapshot.left == 0.0 &&
            snapshot.right == 0.0 &&
            snapshot.autonomousStatus.rescueZoneFrameCompleted &&
            snapshot.autonomousStatus.rescueZoneFrameCompletionReason ==
                "BEST_EFFORT_PARTIAL_BOUND",
        "A conclusão e seu motivo devem sobreviver à parada segura.");
}

void testCameraMonitorReadsCurrentTemporalObservation()
{
    const auto path = std::filesystem::temp_directory_path() /
                      "obr_rescue_zone_frame_status_test.json";
    const double timestamp = std::chrono::duration<double>(
                                 std::chrono::system_clock::now()
                                     .time_since_epoch())
                                 .count();
    std::ostringstream json;
    json << std::fixed << std::setprecision(6)
         << "{\"active\":true,\"timestamp\":" << timestamp
         << ",\"sequence\":42,"
         << "\"green\":{\"candidateDetected\":true,\"detected\":true,"
         << "\"geometryState\":\"FULL_BOUNDS\",\"aimValid\":true,"
         << "\"aimX\":480.0,\"greenMedianRgb\":[0,255,0]},"
         << "\"red\":{\"candidateDetected\":false,\"detected\":false,"
         << "\"geometryState\":\"NOT_DETECTED\",\"aimValid\":false,"
         << "\"aimX\":null},"
         << "\"ultrasonic\":{\"fresh\":true,\"valid\":true,"
         << "\"distanceCm\":43.7}}";
    {
        std::ofstream file(path, std::ios::trunc);
        file << json.str();
    }

    CameraMonitor monitor({}, path.string());
    const RescueZoneSnapshot snapshot = monitor.rescueZoneSnapshot();
    std::error_code removeError;
    std::filesystem::remove(path, removeError);
    require(
        snapshot.sourceFresh && snapshot.sequence == 42 &&
            snapshot.green.candidateDetected && snapshot.green.detected &&
            snapshot.green.geometryState ==
                RescueZoneGeometryState::FullBounds &&
            snapshot.green.aimValid && snapshot.green.aimX == 480.0 &&
            !snapshot.red.candidateDetected && !snapshot.red.detected,
        "CameraMonitor deve ler a observação temporal sem alterar o IPC.");
}
}

int main()
{
    try
    {
        testMissingZoneStaysStopped();
        testFullBoundsCompletesWithoutMoving();
        testFullBoundsWithoutAimCompletesBestEffort();
        testBoundsUnknownCompletesBestEffortWithoutReverseOrEncoders();
        testSingleBoundsTurnInOppositeDirections();
        testLateralMicroPivotStopsAndRequiresNewFrame();
        testPartialBoundCompletesBestEffortAfterConfiguredAttempts();
        testFullBoundsStopsActiveMicroPivotImmediately();
        testLateralPivotUsesImuOnlyAsMaximumAngleGuard();
        testStaleVisionRemainsStopped();
        testRedTargetIsIndependent();
        testTerminalStatusSurvivesSafeStop();
        testCameraMonitorReadsCurrentTemporalObservation();
        std::cout << "rescue_zone_frame_mission_test: OK\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "rescue_zone_frame_mission_test: " << error.what() << '\n';
        return 1;
    }
}
