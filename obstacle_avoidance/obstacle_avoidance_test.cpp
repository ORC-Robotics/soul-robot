#include "obstacle_avoidance/obstacle_avoidance.h"

#include "obstacle_avoidance/config.h"
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
    telemetry.ultrasonicDistanceCm = 6.4;
    telemetry.leftEncoderRate = 0.0;
    telemetry.rightEncoderRate = 0.0;
    telemetry.yawZDeg = 0.0;
    telemetry.gyroZDegPerSec = 0.0;
    return telemetry;
}

ObstacleAvoidanceOutput completeTurn(
    ObstacleAvoidance& avoidance,
    Esp32TelemetrySnapshot& telemetry,
    double targetYawDegrees)
{
    telemetry.yawZDeg = targetYawDegrees;
    telemetry.gyroZDegPerSec = 0.0;
    avoidance.update(telemetry, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kTurn90SettleMs + 20));
    return avoidance.update(telemetry, true);
}

ObstacleAvoidanceOutput finishStageSettling(
    ObstacleAvoidance& avoidance,
    Esp32TelemetrySnapshot& telemetry)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(
        obstacle_config::kStageSettleMs + 20));
    return avoidance.update(telemetry, true);
}

ObstacleAvoidanceOutput completeDistance(
    ObstacleAvoidance& avoidance,
    Esp32TelemetrySnapshot& telemetry,
    double distanceCm)
{
    const long long targetCounts = static_cast<long long>(std::ceil(
        distanceCm * obstacle_config::kEncoderCountsPerCentimeter));
    telemetry.leftEncoderCount += targetCounts;
    telemetry.rightEncoderCount += targetCounts;
    return avoidance.update(telemetry, true);
}

void testObstacleNeedsTwoConsecutiveReadings()
{
    ObstacleAvoidance avoidance;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();

    const ObstacleAvoidanceOutput first =
        avoidance.update(telemetry, true);
    require(!first.hasControl && !avoidance.active(),
            "Uma leitura isolada não deve iniciar o desvio.");

    const ObstacleAvoidanceOutput second =
        avoidance.update(telemetry, true);
    require(second.hasControl && avoidance.active(),
            "Duas leituras próximas devem iniciar o desvio.");
    require(second.leftPower == 0.0 && second.rightPower == 0.0,
            "A detecção deve parar o robô antes de capturar o yaw.");

    const ObstacleAvoidanceOutput turning =
        finishStageSettling(avoidance, telemetry);
    require(turning.leftPower > 0.0 && turning.rightPower < 0.0,
            "Depois de estabilizar, a primeira etapa deve girar à direita.");
}

void testStartCanBeBlockedByAnotherManeuver()
{
    ObstacleAvoidance avoidance;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();

    for (int sample = 0; sample < 3; ++sample)
    {
        const ObstacleAvoidanceOutput output =
            avoidance.update(telemetry, false);
        require(!output.hasControl && !avoidance.active(),
                "Outra manobra ativa deve bloquear o início do desvio.");
    }
}

void testCompleteMeasuredRouteAndRecoverLine()
{
    ObstacleAvoidance avoidance;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();

    avoidance.update(telemetry, true);
    avoidance.update(telemetry, true);
    finishStageSettling(avoidance, telemetry);

    ObstacleAvoidanceOutput output = completeTurn(
        avoidance,
        telemetry,
        obstacle_config::kFirstRightTurnDegrees);
    require(output.phase == "obstacle_first_forward_start",
            "O giro à direita deve liberar a primeira reta.");

    output = completeDistance(
        avoidance,
        telemetry,
        obstacle_config::kFirstForwardDistanceCm);
    require(output.hasControl && output.leftPower == 0.0 &&
                output.rightPower == 0.0,
            "A primeira reta deve parar antes do giro à esquerda.");
    finishStageSettling(avoidance, telemetry);

    output = completeTurn(
        avoidance,
        telemetry,
        0.0);
    require(output.phase == "obstacle_second_forward_start",
            "O giro de 45 graus à esquerda deve liberar a segunda reta.");

    output = completeDistance(
        avoidance,
        telemetry,
        obstacle_config::kSecondForwardDistanceCm);
    require(output.phase == "obstacle_stage_completed",
            "A segunda reta deve iniciar o giro de 90 graus à esquerda.");
    finishStageSettling(avoidance, telemetry);

    output = completeTurn(
        avoidance,
        telemetry,
        -obstacle_config::kSecondLeftTurnDegrees);
    require(output.phase == "obstacle_third_forward_start",
            "O giro de 90 graus deve liberar a aproximação da linha.");

    output = completeDistance(
        avoidance,
        telemetry,
        obstacle_config::kThirdForwardDistanceCm);
    require(output.phase == "obstacle_stage_completed",
            "A terceira reta deve liberar a busca visual.");

    output = finishStageSettling(avoidance, telemetry);
    require(output.hasControl && output.leftPower > 0.0 &&
                output.rightPower < 0.0,
            "O último giro deve iniciar para a direita.");

    output = completeTurn(
        avoidance,
        telemetry,
        0.0);
    require(output.phase == "obstacle_reverse_start",
            "O giro final de 90 graus deve liberar a ré.");

    output = avoidance.update(telemetry, true);
    require(output.hasControl && output.leftPower < 0.0 &&
                output.rightPower < 0.0,
            "Depois do giro final, os dois lados devem andar em ré.");

    output = completeDistance(
        avoidance,
        telemetry,
        obstacle_config::kReverseDistanceCm);
    require(output.phase == "obstacle_stage_completed",
            "A ré deve parar depois de 5 cm medidos pelos encoders.");

    output = finishStageSettling(avoidance, telemetry);
    require(output.completed && !avoidance.active(),
            "A estabilização depois da ré deve concluir o desvio.");
}

void testLostEncoderStopsActiveRoute()
{
    ObstacleAvoidance avoidance;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();

    avoidance.update(telemetry, true);
    avoidance.update(telemetry, true);
    finishStageSettling(avoidance, telemetry);
    completeTurn(
        avoidance,
        telemetry,
        obstacle_config::kFirstRightTurnDegrees);

    telemetry.lastSensorAgeMs = obstacle_config::kEncoderFreshnessMs + 1;
    const ObstacleAvoidanceOutput output =
        avoidance.update(telemetry, true);
    require(output.failed && output.leftPower == 0.0 &&
                output.rightPower == 0.0,
            "Perder os encoders durante uma reta deve falhar com saída zero.");
}
}

int main()
{
    try
    {
        testObstacleNeedsTwoConsecutiveReadings();
        testStartCanBeBlockedByAnotherManeuver();
        testCompleteMeasuredRouteAndRecoverLine();
        testLostEncoderStopsActiveRoute();
        std::cout << "obstacle_avoidance_test: OK\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "obstacle_avoidance_test: " << error.what() << '\n';
        return 1;
    }
}
