#include "obr/obstacle_avoidance.h"

#include "obr/config.h"

#include <chrono>
#include <cmath>
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
    telemetry.mpuOk = true;
    telemetry.motorSleepPinHigh = true;
    telemetry.lastSensorAgeMs = 0;
    telemetry.ultrasonicDistanceCm = 6.0;
    telemetry.yawZDeg = 0.0;
    telemetry.gyroZDegPerSec = 0.0;
    return telemetry;
}

CameraLineSnapshot centeredLine()
{
    CameraLineSnapshot line;
    line.sourceFresh = true;
    line.lineNearDetected = true;
    line.lineNearFinePosition = 0.0;
    line.mediumTrusted = true;
    line.curveDiagnostics.mediumPosition = 0.0;
    return line;
}

void completeInitialReverse(
    ObstacleAvoidance& avoidance,
    Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line)
{
    const long long targetCounts = static_cast<long long>(std::ceil(
        config::kObstacleReverseDistanceCm *
        config::kEncoderCountsPerCentimeter));
    telemetry.leftEncoderCount -= targetCounts;
    telemetry.rightEncoderCount -= targetCounts;
    const ObstacleAvoidanceOutput completed =
        avoidance.update(telemetry, line, true);
    require(completed.phase == "obstacle_reverse_completed",
            "A ré inicial deve liberar a centralização após 5 cm.");
}

ObstacleAvoidanceOutput beginAndCenter(
    ObstacleAvoidance& avoidance,
    Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line)
{
    avoidance.update(telemetry, line, true);
    const ObstacleAvoidanceOutput detected =
        avoidance.update(telemetry, line, true);
    require(detected.leftPower == 0.0 && detected.rightPower == 0.0,
            "A confirmação do obstáculo deve zerar os motores.");
    const ObstacleAvoidanceOutput reversing =
        avoidance.update(telemetry, line, true);
    require(reversing.leftPower == -config::kObstacleReversePower &&
                reversing.rightPower == -config::kObstacleReversePower,
            "O desvio deve recuar antes da centralização.");
    completeInitialReverse(avoidance, telemetry, line);
    return avoidance.update(telemetry, line, true);
}

ObstacleAvoidanceOutput completeTurn(
    ObstacleAvoidance& avoidance,
    Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line,
    double targetYawDegrees)
{
    telemetry.yawZDeg = targetYawDegrees;
    telemetry.gyroZDegPerSec = 0.0;
    avoidance.update(telemetry, line, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kTurn90SettleMs + 20));
    ObstacleAvoidanceOutput output =
        avoidance.update(telemetry, line, true);
    if (output.phase != "obstacle_sampling_left" &&
        output.phase != "obstacle_sampling_right")
    {
        return output;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kObstacleClearanceSettleMs + 20));
    for (int sample = 0;
         sample < config::kObstacleClearanceRequiredSamples;
         ++sample)
    {
        ++telemetry.esp32UptimeMs;
        output = avoidance.update(telemetry, line, true);
    }
    return output;
}

ObstacleAvoidanceOutput sampleDuringTurn(
    ObstacleAvoidance& avoidance,
    Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line,
    double yawDegrees,
    double distanceCm)
{
    ++telemetry.esp32UptimeMs;
    telemetry.yawZDeg = yawDegrees;
    telemetry.gyroZDegPerSec = 20.0;
    telemetry.ultrasonicDistanceCm = distanceCm;
    return avoidance.update(telemetry, line, true);
}

ObstacleAvoidanceOutput completeSelectedForward(
    ObstacleAvoidance& avoidance,
    Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line)
{
    const long long targetCounts = static_cast<long long>(std::ceil(
        config::kObstacleSelectedForwardDistanceCm *
        config::kEncoderCountsPerCentimeter));
    telemetry.leftEncoderCount += targetCounts;
    telemetry.rightEncoderCount += targetCounts;
    return avoidance.update(telemetry, line, true);
}

ObstacleAvoidanceOutput advanceCurve(
    ObstacleAvoidance& avoidance,
    Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line,
    double distanceCm)
{
    const long long counts = static_cast<long long>(std::ceil(
        distanceCm * config::kEncoderCountsPerCentimeter));
    telemetry.leftEncoderCount += counts;
    telemetry.rightEncoderCount += counts;
    return avoidance.update(telemetry, line, true);
}

void sampleSideSweep(
    ObstacleAvoidance& avoidance,
    Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line,
    double directionSign,
    double first,
    double second,
    double third)
{
    sampleDuringTurn(avoidance, telemetry, line, directionSign * 15.0, first);
    sampleDuringTurn(avoidance, telemetry, line, directionSign * 30.0, second);
    sampleDuringTurn(avoidance, telemetry, line, directionSign * 40.0, third);
}

void testCenteringIsSharedAndPrecedesScan()
{
    ObstacleAvoidance avoidance;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    CameraLineSnapshot line = centeredLine();
    line.curveDiagnostics.mediumPosition = 0.6;

    avoidance.update(telemetry, line, true);
    avoidance.update(telemetry, line, true);
    avoidance.update(telemetry, line, true);
    completeInitialReverse(avoidance, telemetry, line);
    const ObstacleAvoidanceOutput centering =
        avoidance.update(telemetry, line, true);
    require(centering.phase == "obstacle_centering",
            "O desvio deve centralizar antes de medir os lados.");
    require(centering.leftPower == config::kGreenTurnAroundCenteringPower &&
                centering.rightPower ==
                    -config::kGreenTurnAroundCenteringPower,
            "O obstáculo deve usar a mesma decisão do retorno 180°.");

    line.curveDiagnostics.mediumPosition = 0.0;
    const ObstacleAvoidanceOutput turning =
        avoidance.update(telemetry, line, true);
    require(turning.leftPower == 0.0 && turning.rightPower == 0.0,
            "Ao centralizar, deve parar antes do primeiro giro.");
    require(std::abs(turning.yawBase) < 0.001,
            "O yaw base deve ser salvo depois da centralização.");
}

void testStableEndpointUsesMaximumAndSelectsRight()
{
    ObstacleAvoidance avoidance;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    const CameraLineSnapshot line = centeredLine();
    beginAndCenter(avoidance, telemetry, line);

    sampleDuringTurn(avoidance, telemetry, line, -5.0, 2.0);
    sampleSideSweep(avoidance, telemetry, line, -1.0, 300.0, 70.0, 75.0);
    ObstacleAvoidanceOutput output = completeTurn(
        avoidance, telemetry, line, -config::kObstacleClearanceScanDegrees);
    require(std::abs(output.leftClearance - 75.0) < 0.001,
            "A esquerda deve usar a leitura estabilizada perto de 60 graus.");

    output = completeTurn(avoidance, telemetry, line, 0.0);
    require(output.phase == "obstacle_measuring_right",
            "A direita deve começar somente depois do retorno ao yawBase.");
    sampleDuringTurn(avoidance, telemetry, line, 5.0, 3.0);
    sampleSideSweep(avoidance, telemetry, line, 1.0, 350.0, 82.0, 79.0);
    output = completeTurn(
        avoidance, telemetry, line, config::kObstacleClearanceScanDegrees);
    require(!output.completed && output.selectedSide == "RIGHT" &&
                output.phase == "obstacle_side_selected",
            "O lado com maior clearance deve ser selecionado.");
    require(std::abs(output.rightClearance - 79.0) < 0.001,
            "A direita deve ignorar o pico de movimento e usar o alvo estabilizado.");
    output = completeTurn(
        avoidance, telemetry, line, config::kObstacleSideApproachDegrees);
    require(output.phase == "obstacle_selected_forward_start",
            "Depois da medição a 60 graus, o avanço deve retornar a 45 graus.");
    output = avoidance.update(telemetry, line, true);
    require(output.leftPower == config::kObstacleSelectedForwardPower &&
                output.rightPower == config::kObstacleSelectedForwardPower,
            "O avanço deve partir em 0,75 no yaw selecionado.");
    telemetry.yawZDeg = config::kObstacleSideApproachDegrees - 5.0;
    output = avoidance.update(telemetry, line, true);
    require(output.leftPower > output.rightPower,
            "Erro positivo de heading deve corrigir suavemente para a direita.");
    output = completeSelectedForward(avoidance, telemetry, line);
    require(!output.completed && output.leftPower == 0.0 &&
                output.rightPower == 0.0,
            "Dez centímetros devem parar antes de iniciar a curva.");
    require(output.phase == "obstacle_curve_start",
            "A curva deve começar somente depois dos 10 cm.");

    telemetry.yawZDeg = -5.0;
    output = advanceCurve(
        avoidance,
        telemetry,
        line,
        config::kObstacleCurveDistanceCm * 0.5);
    require(output.leftPower > output.rightPower,
            "Depois do afastamento inicial, o yaw alvo deve avançar continuamente.");
    output = advanceCurve(
        avoidance,
        telemetry,
        line,
        config::kObstacleCurveDistanceCm * 0.5);
    require(!output.completed && output.leftPower == 0.0 &&
                output.rightPower == 0.0 &&
                output.phase == "obstacle_final_pivot_start",
            "A distância nominal deve parar antes do pivot final.");
    output = completeTurn(
        avoidance,
        telemetry,
        line,
        -5.0 - config::kObstacleFinalInwardPivotDegrees);
    require(output.completed && output.leftPower == 0.0 &&
                output.rightPower == 0.0,
            "O pivot final para dentro deve concluir com PWM zero.");
}

void testLeftSelectionReturnsToLeftYaw()
{
    ObstacleAvoidance avoidance;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    const CameraLineSnapshot line = centeredLine();
    beginAndCenter(avoidance, telemetry, line);
    sampleSideSweep(avoidance, telemetry, line, -1.0, 60.0, 40.0, 50.0);
    completeTurn(
        avoidance, telemetry, line, -config::kObstacleClearanceScanDegrees);
    completeTurn(avoidance, telemetry, line, 0.0);
    sampleSideSweep(avoidance, telemetry, line, 1.0, 10.0, 30.0, 20.0);
    ObstacleAvoidanceOutput output = completeTurn(
        avoidance, telemetry, line, config::kObstacleClearanceScanDegrees);
    require(!output.completed && output.selectedSide == "LEFT",
            "A maior folga esquerda deve iniciar o retorno à esquerda.");

    output = completeTurn(
        avoidance, telemetry, line, -config::kObstacleSideApproachDegrees);
    require(!output.completed && output.selectedSide == "LEFT" &&
                output.phase == "obstacle_selected_forward_start",
            "O avanço deve iniciar no yaw esquerdo selecionado.");
    output = completeSelectedForward(avoidance, telemetry, line);
    require(!output.completed && output.phase == "obstacle_curve_start",
            "O avanço esquerdo deve transferir para a curva.");
    telemetry.yawZDeg = config::kObstacleCurveEndOffsetDegrees;
    output = advanceCurve(
        avoidance,
        telemetry,
        line,
        config::kObstacleCurveDistanceCm);
    require(!output.completed &&
                output.phase == "obstacle_final_pivot_start",
            "A curva esquerda deve parar antes do pivot final.");
    output = completeTurn(
        avoidance,
        telemetry,
        line,
        config::kObstacleCurveEndOffsetDegrees +
            config::kObstacleFinalInwardPivotDegrees);
    require(output.completed,
            "O pivot final esquerdo deve concluir o desvio.");
    require(output.leftPower == 0.0 && output.rightPower == 0.0,
            "O posicionamento esquerdo deve terminar com PWM zero.");
}

void testPracticalTieUsesFixedSide()
{
    ObstacleAvoidance avoidance;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    const CameraLineSnapshot line = centeredLine();
    beginAndCenter(avoidance, telemetry, line);
    sampleSideSweep(avoidance, telemetry, line, -1.0, 30.0, 30.0, 30.0);
    completeTurn(
        avoidance, telemetry, line, -config::kObstacleClearanceScanDegrees);
    completeTurn(avoidance, telemetry, line, 0.0);
    sampleSideSweep(
        avoidance, telemetry, line, 1.0,
        30.0 + config::kObstacleClearanceTieCm,
        30.0 + config::kObstacleClearanceTieCm,
        30.0 + config::kObstacleClearanceTieCm);
    const ObstacleAvoidanceOutput output = completeTurn(
        avoidance, telemetry, line, config::kObstacleClearanceScanDegrees);
    const std::string expected =
        config::kObstacleDefaultSideIsRight ? "RIGHT" : "LEFT";
    require(output.selectedSide == expected,
            "O empate prático deve usar sempre o lado padrão.");
}

void testStartCanBeBlockedAndMissingLineStops()
{
    ObstacleAvoidance avoidance;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    CameraLineSnapshot line = centeredLine();
    avoidance.update(telemetry, line, false);
    avoidance.update(telemetry, line, false);
    require(!avoidance.active(),
            "Outra manobra deve poder bloquear o início do desvio.");

    line.sourceFresh = false;
    avoidance.update(telemetry, line, true);
    const ObstacleAvoidanceOutput waiting =
        avoidance.update(telemetry, line, true);
    require(waiting.hasControl && waiting.leftPower == 0.0 &&
                waiting.rightPower == 0.0,
            "Sem visão inferior, o robô deve permanecer parado.");
}
}

int main()
{
    try
    {
        testCenteringIsSharedAndPrecedesScan();
        testStableEndpointUsesMaximumAndSelectsRight();
        testLeftSelectionReturnsToLeftYaw();
        testPracticalTieUsesFixedSide();
        testStartCanBeBlockedAndMissingLineStops();
        std::cout << "obstacle_avoidance_test: OK\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "obstacle_avoidance_test: " << error.what() << '\n';
        return 1;
    }
}
