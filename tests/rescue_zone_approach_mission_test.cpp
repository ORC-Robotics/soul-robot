#include "obr/config.h"
#include "obr/rescue_zone_approach_mission.h"
#include "obr/robot_state.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
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

Esp32TelemetrySnapshot readyTelemetry(
    double distanceCm,
    double yawDegrees = 0.0)
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

void testSpeedBandsAndHeadingCorrection()
{
    RescueZoneApproachMission mission;
    const auto start = std::chrono::steady_clock::now();

    RescueZoneApproachOutput output = mission.update(
        10.0, readyTelemetry(30.0, 10.0), start);
    require(
        output.status.rescueZoneApproachSpeedState == "FAR" &&
            output.leftPower == config::kRescueZoneApproachFarPower &&
            output.rightPower == config::kRescueZoneApproachFarPower,
        "Faixa FAR deve usar potência alta sem correção quando o heading coincide.");

    output = mission.update(
        10.0, readyTelemetry(20.0, 5.0),
        start + std::chrono::milliseconds(10));
    require(
        output.status.rescueZoneApproachSpeedState == "MID" &&
            output.leftPower > config::kRescueZoneApproachMidPower &&
            output.rightPower < config::kRescueZoneApproachMidPower,
        "Erro positivo deve corrigir para a direita durante o avanço MID.");

    output = mission.update(
        10.0, readyTelemetry(10.0, 15.0),
        start + std::chrono::milliseconds(20));
    require(
        output.status.rescueZoneApproachSpeedState == "NEAR" &&
            output.leftPower < config::kRescueZoneApproachNearPower &&
            output.rightPower > config::kRescueZoneApproachNearPower,
        "Erro negativo deve corrigir para a esquerda durante o avanço NEAR.");
}

void testNearLatchPreventsAccelerationAfterFalseJump()
{
    RescueZoneApproachMission mission;
    const auto start = std::chrono::steady_clock::now();
    RescueZoneApproachOutput output = mission.update(
        0.0, readyTelemetry(11.0), start);
    require(
        output.status.rescueZoneApproachNearLatched &&
            output.leftPower == config::kRescueZoneApproachNearPower,
        "Entrar em até 12 cm deve armar o nearLatched.");

    output = mission.update(
        0.0, readyTelemetry(80.0),
        start + std::chrono::milliseconds(10));
    require(
        output.status.rescueZoneApproachNearLatched &&
            output.status.rescueZoneApproachSpeedState == "NEAR" &&
            output.leftPower == config::kRescueZoneApproachNearPower &&
            output.rightPower == config::kRescueZoneApproachNearPower,
        "Salto distante após nearLatched nunca deve recuperar FAR ou MID.");
}

void testStopDistanceStartsTimedFinalAdvanceThenCompletes()
{
    RescueZoneApproachMission mission;
    const auto start = std::chrono::steady_clock::now();
    RescueZoneApproachOutput output = mission.update(
        -20.0, readyTelemetry(6.0, -20.0), start);
    require(
        !output.completed && !output.failed &&
            output.leftPower == config::kRescueZoneApproachFinalAdvancePower &&
            output.rightPower == config::kRescueZoneApproachFinalAdvancePower &&
            output.status.rescueZoneApproachSpeedState == "FINAL",
        "Até 6 cm deve iniciar o avanço final com a potência exclusiva de 75%.");

    output = mission.update(
        -20.0, readyTelemetry(6.0, -20.0),
        start + std::chrono::milliseconds(
                    config::kRescueZoneApproachFinalAdvanceMs - 1));
    require(
        !output.completed && output.leftPower ==
                                 config::kRescueZoneApproachFinalAdvancePower &&
            output.rightPower == config::kRescueZoneApproachFinalAdvancePower,
        "O avanço final deve continuar até completar os 1.500 ms.");

    output = mission.update(
        -20.0, readyTelemetry(6.0, -20.0),
        start + std::chrono::milliseconds(
                    config::kRescueZoneApproachFinalAdvanceMs));
    require(
        output.completed && !output.failed && output.leftPower == 0.0 &&
            output.rightPower == 0.0 &&
            output.status.rescueZoneApproachSpeedState == "STOP" &&
            output.status.rescueZoneApproachCompletionReason ==
                "REACHED_DISTANCE",
        "Ao completar 1.500 ms, a aproximação deve concluir com PWM zero.");
}

void testCameraObstructionRunsFinalAdvanceBeforeCompleting()
{
    const auto now = std::chrono::steady_clock::now();

    RescueZoneApproachMission coveredMission;
    RescueZoneSnapshot coveredZones;
    coveredZones.sourceFresh = true;
    coveredZones.red.frameCoverage =
        config::kRescueZoneApproachCameraStopCoverage;
    RescueZoneApproachOutput output = coveredMission.update(
        0.0, readyTelemetry(21.0), now, coveredZones,
        RescueZoneTargetColor::Red);
    require(
        !output.completed && !output.failed &&
            output.leftPower == config::kRescueZoneApproachFinalAdvancePower &&
            output.rightPower == config::kRescueZoneApproachFinalAdvancePower &&
            output.status.rescueZoneApproachSpeedState == "FINAL",
        "Zona alvo cobrindo 70% deve iniciar avanço final lento.");
    output = coveredMission.update(
        0.0, readyTelemetry(21.0),
        now + std::chrono::milliseconds(
                  config::kRescueZoneApproachFinalAdvanceMs),
        coveredZones, RescueZoneTargetColor::Red);
    require(
        output.completed && output.leftPower == 0.0 &&
            output.rightPower == 0.0 &&
            output.status.rescueZoneApproachCompletionReason ==
                "CAMERA_OBSCURED",
        "Após 1.500 ms, a parada visual deve concluir com PWM zero.");

    RescueZoneApproachMission obscuredMission;
    RescueZoneSnapshot obscuredZones;
    obscuredZones.sourceFresh = true;
    obscuredZones.cameraObscured = true;
    output = obscuredMission.update(
        0.0, readyTelemetry(21.0), now, obscuredZones,
        RescueZoneTargetColor::Green);
    require(
        !output.completed &&
            output.leftPower == config::kRescueZoneApproachFinalAdvancePower &&
            output.rightPower == config::kRescueZoneApproachFinalAdvancePower,
        "Frame escuro ou uniforme também deve executar o avanço de 1.500 ms.");
}

void testFinalAdvancePausesWithoutExtendingGlobalTimeout()
{
    using namespace std::chrono;
    const auto start = steady_clock::time_point{};
    RescueZoneApproachMission mission;
    auto telemetry = readyTelemetry(6.0);
    mission.update(0.0, telemetry, start);
    mission.pause(start + milliseconds(200));
    mission.pause(start + milliseconds(1500));
    auto output = mission.update(0.0, telemetry, start + milliseconds(2200));
    require(!output.completed && output.leftPower == 0.75,
            "Uma pausa longa não pode consumir o avanço obrigatório.");
    output = mission.update(0.0, telemetry, start + milliseconds(3499));
    require(!output.completed, "O avanço não pode terminar antes de 1.500 ms autorizados.");
    output = mission.update(0.0, telemetry, start + milliseconds(3500));
    require(output.completed && output.leftPower == 0.0 && output.rightPower == 0.0,
            "Somente o tempo autorizado completo deve liberar o depósito.");

    mission.reset();
    mission.update(0.0, telemetry, start);
    mission.pause(start + milliseconds(200));
    output = mission.update(0.0, telemetry,
                            start + milliseconds(config::kRescueZoneApproachTimeoutMs + 1));
    require(output.failed && !output.completed && output.leftPower == 0.0,
            "A pausa não pode renovar o timeout global de segurança.");
}

void testFinalAdvanceDoesNotBypassSensorFailures()
{
    using namespace std::chrono;
    const auto start = steady_clock::time_point{};
    for (int fault = 0; fault < 2; ++fault)
    {
        RescueZoneApproachMission mission;
        auto telemetry = readyTelemetry(6.0);
        mission.update(0.0, telemetry, start);
        if (fault == 0)
        {
            telemetry.ultrasonicDistanceCm = NAN;
        }
        else
        {
            telemetry.mpuOk = false;
        }
        auto output = mission.update(0.0, telemetry, start + milliseconds(100));
        require(output.failed && !output.completed &&
                    output.leftPower == 0.0 && output.rightPower == 0.0,
                "Sensor inválido durante o avanço deve bloquear o depósito.");
        output = mission.update(0.0, readyTelemetry(6.0), start + milliseconds(2000));
        require(output.failed && !output.completed,
                "A recuperação do sensor não pode transformar falha em sucesso.");
    }
}

void testMissingInputsAndRuntimeStaleFailStopped()
{
    RescueZoneApproachMission noHeadingMission;
    RescueZoneApproachOutput output = noHeadingMission.update(
        std::numeric_limits<double>::quiet_NaN(), readyTelemetry(30.0));
    require(
        output.failed && output.leftPower == 0.0 &&
            output.rightPower == 0.0,
        "Sem lockedHeading a aproximação não pode iniciar.");

    RescueZoneApproachMission staleMission;
    const auto start = std::chrono::steady_clock::now();
    output = staleMission.update(0.0, readyTelemetry(30.0), start);
    require(output.leftPower > 0.0 && output.rightPower > 0.0,
            "Entrada válida deve iniciar avanço.");
    Esp32TelemetrySnapshot stale = readyTelemetry(30.0);
    stale.lastSensorAgeMs = config::kRescueZoneUltrasonicFreshnessMs + 1;
    output = staleMission.update(
        0.0, stale, start + std::chrono::milliseconds(10));
    require(
        output.failed && output.leftPower == 0.0 &&
            output.rightPower == 0.0,
        "IMU ou ULTRA stale durante movimento deve falhar com PWM zero.");

    RescueZoneApproachMission invalidUltraMission;
    Esp32TelemetrySnapshot invalidUltra = readyTelemetry(500.0);
    output = invalidUltraMission.update(0.0, invalidUltra, start);
    require(
        output.failed && output.leftPower == 0.0 &&
            output.rightPower == 0.0 &&
            output.status.rescueZoneApproachCompletionReason == "ULTRA_STALE",
        "ULTRA fora da faixa válida não pode iniciar movimento.");
}

void testTimeoutAndEmergencyStopKeepZero()
{
    RescueZoneApproachMission mission;
    const auto start = std::chrono::steady_clock::now();
    mission.update(0.0, readyTelemetry(30.0), start);
    const RescueZoneApproachOutput output = mission.update(
        0.0,
        readyTelemetry(30.0),
        start + std::chrono::milliseconds(
                    config::kRescueZoneApproachTimeoutMs + 1));
    require(
        output.failed && output.leftPower == 0.0 &&
            output.rightPower == 0.0 &&
            output.status.rescueZoneApproachCompletionReason == "TIMEOUT",
        "Timeout deve falhar com PWM zero.");

    RobotState robotState;
    robotState.setAutonomousMission(AutonomousMission::RescueZoneApproach);
    robotState.startAutonomous();
    robotState.driveAutonomous(0.85, 0.85, false);
    robotState.emergencyStop();
    const RobotSnapshot snapshot = robotState.snapshot();
    require(
        snapshot.emergencyStop && snapshot.left == 0.0 &&
            snapshot.right == 0.0,
        "E-Stop deve prevalecer e zerar os dois comandos.");
}

void testAlignedHeadingPersistsForApproachSelection()
{
    RobotState robotState;
    robotState.setAutonomousMission(AutonomousMission::RescueZoneAlign);
    robotState.startAutonomous();
    require(
        robotState.setRescueZoneLockedHeading(37.5),
        "ALIGN_ZONE autônomo deve poder salvar heading válido.");
    robotState.stop();
    robotState.setAutonomousMission(AutonomousMission::RescueZoneApproach);
    require(
        robotState.snapshot().rescueZoneLockedHeading == 37.5,
        "Selecionar APPROACH_ZONE deve preservar o heading salvo pelo ALIGN_ZONE.");
}
}

int main()
{
    try
    {
        testSpeedBandsAndHeadingCorrection();
        testNearLatchPreventsAccelerationAfterFalseJump();
        testStopDistanceStartsTimedFinalAdvanceThenCompletes();
        testCameraObstructionRunsFinalAdvanceBeforeCompleting();
        testFinalAdvancePausesWithoutExtendingGlobalTimeout();
        testFinalAdvanceDoesNotBypassSensorFailures();
        testMissingInputsAndRuntimeStaleFailStopped();
        testTimeoutAndEmergencyStopKeepZero();
        testAlignedHeadingPersistsForApproachSelection();
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }

    std::cout << "Rescue zone approach mission tests passed.\n";
    return 0;
}
