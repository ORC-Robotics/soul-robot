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

ObstacleAvoidanceOutput completeInitialReverse(
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
    require(completed.phase == "obstacle_reverse_completed" ||
                completed.phase == "obstacle_side_selected",
            "A ré inicial deve liberar a próxima fase do desvio.");
    return completed;
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
    const ObstacleAvoidanceOutput reverseCompleted =
        completeInitialReverse(avoidance, telemetry, line);
    if (reverseCompleted.phase == "obstacle_side_selected")
    {
        return reverseCompleted;
    }
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

ObstacleAvoidanceOutput sampleCameraBlackDuringTurn(
    ObstacleAvoidance& avoidance,
    Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line,
    double yawDegrees,
    std::uint64_t sequence,
    bool visible,
    bool fresh = true)
{
    ++telemetry.esp32UptimeMs;
    telemetry.yawZDeg = yawDegrees;
    telemetry.gyroZDegPerSec = 20.0;
    ForwardLineSnapshot forwardLine;
    forwardLine.sourceFresh = fresh;
    forwardLine.obstacleBlackSequence = sequence;
    forwardLine.obstacleBlackVisible = visible;
    return avoidance.update(telemetry, line, true, forwardLine);
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

ObstacleAvoidanceOutput startCurveWithSelectedSide(
    ObstacleAvoidance& avoidance,
    Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line,
    bool selectRight)
{
    beginAndCenter(avoidance, telemetry, line);
    const double leftClearance = selectRight ? 20.0 : 80.0;
    const double rightClearance = selectRight ? 80.0 : 20.0;
    sampleSideSweep(
        avoidance,
        telemetry,
        line,
        -1.0,
        leftClearance,
        leftClearance,
        leftClearance);
    completeTurn(
        avoidance, telemetry, line, -config::kObstacleClearanceScanDegrees);
    completeTurn(avoidance, telemetry, line, 0.0);
    sampleSideSweep(
        avoidance,
        telemetry,
        line,
        1.0,
        rightClearance,
        rightClearance,
        rightClearance);
    ObstacleAvoidanceOutput output = completeTurn(
        avoidance, telemetry, line, config::kObstacleClearanceScanDegrees);
    require(output.selectedSide == (selectRight ? "RIGHT" : "LEFT"),
            "O preparo do teste deve selecionar o lado esperado.");
    output = completeTurn(
        avoidance,
        telemetry,
        line,
        selectRight ? config::kObstacleSideApproachDegrees
                    : -config::kObstacleSideApproachDegrees);
    output = completeSelectedForward(avoidance, telemetry, line);
    require(output.phase == "obstacle_curve_start",
            "O preparo do teste deve alcançar o início da curva.");
    return output;
}

CameraLineSnapshot fusionLine(
    std::uint64_t sequence, bool valid = true, bool continuationBand = false)
{
    CameraLineSnapshot line = centeredLine();
    line.lineSequence = sequence;
    line.lineControlSource = valid ? "fusion" : "search";
    line.normalSteeringValid = valid;
    line.obstacleContinuationBand = continuationBand;
    return line;
}

CameraLineSnapshot fusionSteeringLine(
    std::uint64_t sequence,
    double leftPower,
    double rightPower)
{
    CameraLineSnapshot line = fusionLine(sequence);
    line.lineFollowerLeftPower = leftPower;
    line.lineFollowerRightPower = rightPower;
    return line;
}

CameraLineSnapshot gapLostLine(
    std::uint64_t sequence,
    const std::string& decision)
{
    CameraLineSnapshot line = centeredLine();
    line.lineSequence = sequence;
    line.lineControlSource = "search";
    line.normalSteeringValid = false;
    line.gapValidationDecision = decision;
    return line;
}

ForwardLineSnapshot parabolaFrame(
    std::uint64_t sequence,
    std::uint64_t leftBlack,
    std::uint64_t rightBlack,
    bool nearVisible = false)
{
    ForwardLineSnapshot forward;
    forward.sourceFresh = true;
    forward.parabolaSequence = sequence;
    forward.parabolaLeftBlack = leftBlack;
    forward.parabolaRightBlack = rightBlack;
    forward.parabolaNearForwardVisible = nearVisible;
    return forward;
}

// Encerra a curva com o giro relativo para dentro, espelhado pelo lado escolhido.
ObstacleAvoidanceOutput completeExitPivot(
    ObstacleAvoidance& avoidance,
    Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line)
{
    const double startYaw = telemetry.yawZDeg;
    const auto waiting = avoidance.update(telemetry, line, true);
    require(waiting.phase == "obstacle_exit_pivot_wait" &&
                waiting.leftPower == 0.0 && waiting.rightPower == 0.0,
            "A pausa de saída deve manter os motores zerados.");
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kObstacleExitPivotWaitMs + 20));
    const auto starting = avoidance.update(telemetry, line, true);
    require(starting.phase == "obstacle_exit_pivot_start",
            "A pausa deve iniciar o giro de saída.");
    const auto turning = avoidance.update(telemetry, line, true);
    const double direction = starting.selectedSide == "LEFT" ? 1.0 : -1.0;
    require(turning.leftPower * direction > 0.0 &&
                turning.rightPower * direction < 0.0,
            "O giro nominal deve ser para dentro e espelhado pelo lado escolhido.");
    const auto output = completeTurn(
        avoidance, telemetry, line,
        startYaw + direction * config::kObstacleExitPivotDegrees);
    require(output.phase == "obstacle_exit_distance_forward_start" &&
                !output.completed && output.leftPower == 0.0 &&
                output.rightPower == 0.0,
            "O giro deve preparar o avanço configurado sem devolver o controle.");
    return output;
}

ObstacleAvoidanceOutput completeExitDistance(
    ObstacleAvoidance& avoidance,
    Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line)
{
    const long long counts = static_cast<long long>(std::ceil(
        config::kObstacleExitForwardDistanceCm *
        config::kEncoderCountsPerCentimeter));
    telemetry.leftEncoderCount += counts;
    telemetry.rightEncoderCount += counts;
    const auto output = avoidance.update(telemetry, line, true);
    require(output.phase == "obstacle_exit_fusion_turn_start" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "O avanço configurado deve parar antes do giro adicional.");
    return output;
}

ObstacleAvoidanceOutput completeExitFusionTurn(
    ObstacleAvoidance& avoidance,
    Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line)
{
    const double startYaw = telemetry.yawZDeg;
    const auto turning = avoidance.update(telemetry, line, true);
    const double direction = turning.selectedSide == "LEFT" ? 1.0 : -1.0;
    require(turning.leftPower == direction * config::kObstacleExitFusionTurnPower &&
                turning.rightPower == -direction * config::kObstacleExitFusionTurnPower,
            "O giro adicional deve ser espelhado para dentro do contorno.");
    telemetry.yawZDeg = startYaw +
        direction * config::kObstacleExitFusionTurnDegrees;
    const auto output = avoidance.update(telemetry, line, true);
    require(output.phase == "obstacle_exit_timed_forward_start" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "Dez graus sem Fusion devem preparar a procura reta temporizada.");
    return output;
}

ObstacleAvoidanceOutput reachExitTimedForward(
    ObstacleAvoidance& avoidance,
    Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line)
{
    completeExitDistance(avoidance, telemetry, line);
    return completeExitFusionTurn(avoidance, telemetry, line);
}

ObstacleAvoidanceOutput completeNominalObstacleWithParabolaBest(
    ObstacleAvoidance& avoidance,
    Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line,
    bool bestIsRight)
{
    startCurveWithSelectedSide(avoidance, telemetry, line, true);
    const ForwardLineSnapshot best = bestIsRight
        ? parabolaFrame(1, 2000, 21000)
        : parabolaFrame(1, 21000, 2000);
    ObstacleAvoidanceOutput output =
        avoidance.update(telemetry, line, true, best);
    require(output.rawBestParabolaSide == (bestIsRight ? "RIGHT" : "LEFT") &&
                output.bestParabolaSide == (bestIsRight ? "RIGHT" : "LEFT") &&
                output.bestParabolaScore == 19000,
            "O recovery deve preservar o lado bruto do melhor frame.");

    const long long curveCounts = static_cast<long long>(std::ceil(
        config::kObstacleCurveDistanceCm *
        config::kEncoderCountsPerCentimeter));
    telemetry.leftEncoderCount += curveCounts;
    telemetry.rightEncoderCount += curveCounts;
    output = avoidance.update(telemetry, line, true, best);
    require(output.phase == "obstacle_exit_pivot_wait",
            "Sem Fusion, a parábola nominal deve chegar à pausa de saída.");
    completeExitPivot(avoidance, telemetry, line);
    reachExitTimedForward(avoidance, telemetry, line);
    avoidance.update(telemetry, fusionLine(1), true);
    avoidance.update(telemetry, fusionLine(2), true);
    output = avoidance.update(telemetry, fusionLine(3), true);
    require(output.completed && output.phase == "obstacle_exit_reacquired" &&
                output.case3Armed && output.bestParabolaSideValid,
            "O reencontro deve preservar a memória e armar a proteção do caso 3.");
    return output;
}

ObstacleAvoidanceOutput startForcedLeftExitForward(
    ObstacleAvoidance& avoidance,
    Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line)
{
    ObstacleAvoidanceOutput output =
        beginAndCenter(avoidance, telemetry, line);
    require(output.selectedSide == "LEFT" &&
                output.selectedSideSource == "CONFIG",
            "O perfil forçado deve escolher LEFT sem depender da varredura.");

    output = completeTurn(
        avoidance,
        telemetry,
        line,
        -config::kObstacleSideApproachDegrees);
    require(output.phase == "obstacle_selected_forward_start",
            "O perfil esquerdo deve iniciar a primeira reta após o giro.");
    output = completeSelectedForward(avoidance, telemetry, line);
    require(output.phase == "obstacle_curve_start",
            "A primeira reta deve transferir o controle para a curva.");
    output = advanceCurve(
        avoidance,
        telemetry,
        line,
        config::kObstacleCurveDistanceCm);
    require(output.phase == "obstacle_exit_pivot_wait" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "A curva de 20 cm deve parar antes do giro de saída.");

    return completeExitPivot(avoidance, telemetry, line);
}

void testForcedLeftProfileReacquiresDuringStraightExit()
{
    ObstacleAvoidance avoidance;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    const CameraLineSnapshot line = centeredLine();
    startForcedLeftExitForward(avoidance, telemetry, line);

    ObstacleAvoidanceOutput output =
        avoidance.update(telemetry, line, true);
    require(output.phase == "obstacle_exit_distance_forward" &&
                output.leftPower == output.rightPower &&
                output.leftPower == config::kObstacleExitForwardPower,
            "Após a curva, a saída deve avançar a distância configurada.");

    completeExitDistance(avoidance, telemetry, line);
    avoidance.update(telemetry, fusionLine(1), true);
    avoidance.update(telemetry, fusionLine(2), true);
    output = avoidance.update(telemetry, fusionLine(3), true);
    require(output.completed &&
                output.phase == "obstacle_exit_reacquired" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "Três frames Fusion devem concluir a saída imediatamente e parada.");
}

void testForcedRightSkipsSideMeasurement()
{
    ObstacleAvoidance avoidance(config::ObstacleSideMode::Right);
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    const CameraLineSnapshot line = centeredLine();

    ObstacleAvoidanceOutput output =
        beginAndCenter(avoidance, telemetry, line);
    require(output.selectedSide == "RIGHT" &&
                output.selectedSideSource == "CONFIG" &&
                output.phase == "obstacle_side_selected",
            "O perfil fixo direito deve pular a comparação lateral.");

    output = completeTurn(
        avoidance,
        telemetry,
        line,
        config::kObstacleSideApproachDegrees);
    require(output.phase == "obstacle_selected_forward_start",
            "O perfil fixo direito deve avançar após o giro configurado.");
}

void testObstacleTurnAcceptsEncoderStopWithGyroBias()
{
    ObstacleAvoidance avoidance(config::ObstacleSideMode::Left);
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    const CameraLineSnapshot line = centeredLine();

    beginAndCenter(avoidance, telemetry, line);
    telemetry.yawZDeg = -config::kObstacleSideApproachDegrees;
    telemetry.gyroZDegPerSec = -7.0;
    telemetry.leftEncoderRate = 100.0;
    telemetry.rightEncoderRate = -100.0;
    ObstacleAvoidanceOutput output =
        avoidance.update(telemetry, line, true);
    require(output.phase == "obstacle_turn_settling",
            "O giro deve aguardar enquanto os encoders ainda indicam movimento.");

    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kTurn90SettleMs + 20));
    output = avoidance.update(telemetry, line, true);
    require(output.phase == "obstacle_turn_settling",
            "O viés do IMU não pode liberar um robô com rodas em movimento.");

    telemetry.leftEncoderRate = 0.0;
    telemetry.rightEncoderRate = 0.0;
    output = avoidance.update(telemetry, line, true);
    require(output.phase == "obstacle_selected_forward_start",
            "Os encoders parados devem liberar apenas o giro do obstáculo.");
}

void testExitDistanceRequiresBothEncoders()
{
    ObstacleAvoidance avoidance(config::ObstacleSideMode::Left);
    auto telemetry = readyTelemetry();
    const auto line = centeredLine();
    startForcedLeftExitForward(avoidance, telemetry, line);
    const long long counts = static_cast<long long>(std::ceil(
        config::kObstacleExitForwardDistanceCm * config::kEncoderCountsPerCentimeter));
    telemetry.leftEncoderCount += counts;
    auto output = avoidance.update(telemetry, line, true);
    require(output.phase == "obstacle_exit_distance_forward" &&
                output.progressPercent == 0.0 && output.leftPower > 0.0 &&
                output.rightPower > 0.0,
            "Uma roda sozinha não pode concluir o avanço configurado.");
    telemetry.rightEncoderCount += counts;
    output = avoidance.update(telemetry, line, true);
    require(output.phase == "obstacle_exit_fusion_turn_start" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "As duas rodas no alvo devem parar antes do giro adicional.");
}

void testFusionVotesCarryFromTurnToTimedForward()
{
    ObstacleAvoidance avoidance(config::ObstacleSideMode::Left);
    auto telemetry = readyTelemetry();
    const auto line = centeredLine();
    startForcedLeftExitForward(avoidance, telemetry, line);
    completeExitDistance(avoidance, telemetry, line);

    avoidance.update(telemetry, fusionLine(1), true);
    auto output = avoidance.update(telemetry, fusionLine(2), true);
    require(!output.completed && output.phase == "obstacle_exit_fusion_turn_right",
            "Dois votos durante o giro ainda não podem confirmar a faixa.");
    telemetry.yawZDeg += config::kObstacleExitFusionTurnDegrees;
    output = avoidance.update(telemetry, centeredLine(), true);
    require(output.phase == "obstacle_exit_timed_forward_start",
            "Dez graus sem o terceiro voto devem iniciar a reta temporizada.");
    output = avoidance.update(telemetry, fusionLine(3), true);
    require(output.completed && output.phase == "obstacle_exit_reacquired" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "O terceiro voto novo deve concluir a procura após a transição de fase.");
}

void testForcedLeftProfileSearchTimeoutStops()
{
    ObstacleAvoidance avoidance(config::ObstacleSideMode::Left);
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    const CameraLineSnapshot line = centeredLine();
    startForcedLeftExitForward(avoidance, telemetry, line);
    reachExitTimedForward(avoidance, telemetry, line);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kObstacleExitFusionForwardTimeoutMs + 20));
    ObstacleAvoidanceOutput output =
        avoidance.update(telemetry, line, true);
    require(output.phase == "obstacle_exit_search_right_start" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "Após a reta configurada sem faixa, deve parar antes da busca.");

    output = avoidance.update(telemetry, line, true);
    require(output.phase == "obstacle_exit_search_right" &&
                output.leftPower > 0.0 && output.rightPower < 0.0,
            "A busca final deve girar explicitamente para a direita.");

    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kObstacleExitSearchTimeoutMs + 20));
    output = avoidance.update(telemetry, line, true);
    require(output.failed &&
                output.phase == "obstacle_exit_search_timeout" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "Depois do timeout sem Fusion, a busca deve falhar parada.");
}

void testForcedLeftProfileReacquiresDuringRightSearch()
{
    ObstacleAvoidance avoidance(config::ObstacleSideMode::Left);
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    const CameraLineSnapshot line = centeredLine();
    startForcedLeftExitForward(avoidance, telemetry, line);
    reachExitTimedForward(avoidance, telemetry, line);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kObstacleExitFusionForwardTimeoutMs + 20));
    avoidance.update(telemetry, line, true);
    avoidance.update(telemetry, fusionLine(10), true);
    avoidance.update(telemetry, fusionLine(11), true);
    const ObstacleAvoidanceOutput output =
        avoidance.update(telemetry, fusionLine(12), true);

    require(output.completed &&
                output.phase == "obstacle_exit_reacquired" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "Três frames Fusion durante a busca à direita devem concluir parados.");
}

void testAutomaticNominalExitMirrorsBothSides()
{
    for (const bool selectRight : {false, true})
    {
        ObstacleAvoidance avoidance(config::ObstacleSideMode::Automatic);
        auto telemetry = readyTelemetry();
        const auto line = centeredLine();
        startCurveWithSelectedSide(avoidance, telemetry, line, selectRight);
        telemetry.yawZDeg = 0.0;
        auto output = advanceCurve(
            avoidance, telemetry, line, config::kObstacleCurveDistanceCm * 0.5);
        require(selectRight ? output.leftPower < output.rightPower
                            : output.leftPower > output.rightPower,
                "O yaw progressivo da curva deve espelhar o diferencial dos motores.");
        advanceCurve(avoidance, telemetry, line, config::kObstacleCurveDistanceCm * 0.5);
        completeExitPivot(avoidance, telemetry, line);
        output = avoidance.update(telemetry, line, true);
        require(output.phase == "obstacle_exit_distance_forward" &&
                    output.leftPower == config::kObstacleExitForwardPower &&
                    output.rightPower == config::kObstacleExitForwardPower,
                "Ambos os lados devem preservar o avanço configurado com comandos iguais.");
        completeExitDistance(avoidance, telemetry, line);
        const double turnStartYaw = telemetry.yawZDeg;
        output = avoidance.update(telemetry, line, true);
        const double direction = selectRight ? -1.0 : 1.0;
        require(output.leftPower == direction * config::kObstacleExitFusionTurnPower &&
                    output.rightPower == -direction * config::kObstacleExitFusionTurnPower,
                "O giro de 10 graus deve espelhar exatamente os comandos.");
        telemetry.yawZDeg = turnStartYaw +
            direction * config::kObstacleExitFusionTurnDegrees;
        output = avoidance.update(telemetry, line, true);
        require(output.phase == "obstacle_exit_timed_forward_start",
                "O giro adicional deve preparar a reta temporizada.");
        output = avoidance.update(telemetry, line, true);
        require(output.phase == "obstacle_exit_timed_forward" &&
                    output.leftPower == config::kObstacleExitForwardPower &&
                    output.rightPower == config::kObstacleExitForwardPower,
                "A procura reta de 1,5 segundo deve usar comandos iguais.");
        std::this_thread::sleep_for(std::chrono::milliseconds(
            config::kObstacleExitFusionForwardTimeoutMs + 20));
        output = avoidance.update(telemetry, line, true);
        require(output.phase == (selectRight ? "obstacle_exit_search_left_start"
                                            : "obstacle_exit_search_right_start") &&
                    output.leftPower == 0.0 && output.rightPower == 0.0,
                "A transição para a busca deve parar e indicar o lado espelhado.");
        output = avoidance.update(telemetry, line, true);
        require(output.leftPower == direction * config::kObstacleExitSearchPower &&
                    output.rightPower == -direction * config::kObstacleExitSearchPower,
                "A busca final deve espelhar exatamente os comandos dos motores.");
        avoidance.update(telemetry, fusionLine(1), true);
        output = avoidance.update(telemetry, fusionLine(1), true);
        require(!output.completed, "Repetir um frame não pode aumentar os votos.");
        avoidance.update(telemetry, fusionLine(2, false), true);
        avoidance.update(telemetry, fusionLine(3), true);
        avoidance.update(telemetry, fusionLine(4), true);
        output = avoidance.update(telemetry, fusionLine(5), true);
        require(output.completed && output.leftPower == 0.0 && output.rightPower == 0.0,
                "Três frames novos e consecutivos devem encerrar a busca em ambos os lados.");
    }
}

void testForcedSideStillAllowsEarlyRecovery()
{
    ObstacleAvoidance avoidance(config::ObstacleSideMode::Left);
    auto telemetry = readyTelemetry();
    const auto line = centeredLine();
    beginAndCenter(avoidance, telemetry, line);
    completeTurn(avoidance, telemetry, line, -config::kObstacleSideApproachDegrees);
    completeSelectedForward(avoidance, telemetry, line);
    avoidance.update(telemetry, fusionLine(1, true, true), true);
    avoidance.update(telemetry, fusionLine(2, true, true), true);
    auto output = avoidance.update(telemetry, fusionLine(3, true, true), true);
    require(output.phase == "obstacle_reacquire_forward" && !output.completed,
            "Forçar LEFT não pode desativar a recuperação antecipada da faixa.");
    const long long counts = static_cast<long long>(std::ceil(
        config::kObstacleReacquireForwardDistanceCm * config::kEncoderCountsPerCentimeter));
    telemetry.leftEncoderCount += counts;
    telemetry.rightEncoderCount += counts;
    avoidance.update(telemetry, fusionLine(3), true);
    output = avoidance.update(telemetry, fusionLine(4, false), true);
    require(output.leftPower == -config::kObstacleReacquireSearchPower &&
                output.rightPower == config::kObstacleReacquireSearchPower,
            "A sequência de avanço e giro deve buscar LEFT no contorno esquerdo.");
    avoidance.update(telemetry, fusionLine(5), true);
    avoidance.update(telemetry, fusionLine(6), true);
    output = avoidance.update(telemetry, fusionLine(7), true);
    require(output.completed && output.leftPower == 0.0 && output.rightPower == 0.0,
            "O reencontro antecipado esquerdo deve concluir parado.");
}

void testExitStopsWithStaleSensors()
{
    for (const bool staleVision : {false, true})
    {
        ObstacleAvoidance avoidance(config::ObstacleSideMode::Left);
        auto telemetry = readyTelemetry();
        auto line = centeredLine();
        startForcedLeftExitForward(avoidance, telemetry, line);
        if (staleVision)
        {
            line.sourceFresh = false;
        }
        else
        {
            telemetry.lastSensorAgeMs = config::kObstacleEncoderFreshnessMs + 1;
        }
        const auto output = avoidance.update(telemetry, line, true);
        require(output.failed && output.leftPower == 0.0 && output.rightPower == 0.0,
                "A reta de saída deve falhar parada quando visão ou sensores ficam antigos.");
    }
}

void testCenteringIsSharedAndPrecedesScan()
{
    ObstacleAvoidance avoidance(config::ObstacleSideMode::Automatic);
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
    ObstacleAvoidance avoidance(config::ObstacleSideMode::Automatic);
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
            "Depois da medição a 60 graus, o avanço deve retornar ao ângulo de aproximação.");
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
            "A distância configurada deve parar antes de iniciar a curva.");
    require(output.phase == "obstacle_curve_start",
            "A curva deve começar somente depois da reta inicial.");

    const double halfwayTargetYaw =
        (telemetry.yawZDeg - config::kObstacleCurveEndOffsetDegrees) * 0.5;
    telemetry.yawZDeg = halfwayTargetYaw - 5.0;
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
                output.phase == "obstacle_exit_pivot_wait",
            "A distância nominal deve parar antes do giro de saída.");
    completeExitPivot(avoidance, telemetry, line);
    reachExitTimedForward(avoidance, telemetry, line);
    avoidance.update(telemetry, fusionLine(1), true);
    avoidance.update(telemetry, fusionLine(2), true);
    output = avoidance.update(telemetry, fusionLine(3), true);
    require(output.completed && output.leftPower == 0.0 &&
                output.rightPower == 0.0,
            "O reencontro após o giro deve concluir com PWM zero.");
}

void testLeftSelectionReturnsToLeftYaw()
{
    ObstacleAvoidance avoidance(config::ObstacleSideMode::Automatic);
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
                output.phase == "obstacle_exit_pivot_wait",
            "A curva esquerda deve parar antes do giro de saída.");
    completeExitPivot(avoidance, telemetry, line);
    reachExitTimedForward(avoidance, telemetry, line);
    avoidance.update(telemetry, fusionLine(1), true);
    avoidance.update(telemetry, fusionLine(2), true);
    output = avoidance.update(telemetry, fusionLine(3), true);
    require(output.completed,
            "O reencontro esquerdo deve concluir o desvio.");
    require(output.leftPower == 0.0 && output.rightPower == 0.0,
            "O posicionamento esquerdo deve terminar com PWM zero.");
}

void testPracticalTieUsesFixedSide()
{
    ObstacleAvoidance avoidance(config::ObstacleSideMode::Automatic);
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

void testCameraBlackSelectsOnlyConfirmedSide()
{
    ObstacleAvoidance avoidance(config::ObstacleSideMode::Automatic);
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    const CameraLineSnapshot line = centeredLine();
    beginAndCenter(avoidance, telemetry, line);

    ObstacleAvoidanceOutput output = sampleCameraBlackDuringTurn(
        avoidance, telemetry, line, -5.0, 1, true);
    require(output.cameraBlackLeftFrames == 0,
            "A CAM1 deve ignorar preto antes de 10 graus.");

    output = sampleCameraBlackDuringTurn(
        avoidance, telemetry, line, -15.0, 2, true);
    require(output.cameraBlackLeftFrames == 1,
            "O primeiro frame novo na janela deve iniciar a confirmação.");
    output = sampleCameraBlackDuringTurn(
        avoidance, telemetry, line, -20.0, 2, true);
    require(output.cameraBlackLeftFrames == 1,
            "A mesma sequence da CAM1 não pode ser contada novamente.");
    sampleCameraBlackDuringTurn(
        avoidance, telemetry, line, -25.0, 3, true);
    output = sampleCameraBlackDuringTurn(
        avoidance, telemetry, line, -35.0, 4, true);
    require(output.cameraBlackLeft && output.cameraBlackLeftFrames == 3,
            "Três frames novos consecutivos devem confirmar LEFT.");

    output = sampleCameraBlackDuringTurn(
        avoidance, telemetry, line, -45.0, 5, false);
    require(output.cameraBlackLeft && output.cameraBlackLeftFrames == 3,
            "Depois de 40 graus, a evidência confirmada deve permanecer latched.");

    sampleSideSweep(avoidance, telemetry, line, -1.0, 20.0, 20.0, 20.0);
    completeTurn(
        avoidance, telemetry, line, -config::kObstacleClearanceScanDegrees);
    completeTurn(avoidance, telemetry, line, 0.0);

    sampleCameraBlackDuringTurn(
        avoidance, telemetry, line, 15.0, 6, false);
    sampleCameraBlackDuringTurn(
        avoidance, telemetry, line, 25.0, 7, false);
    output = sampleCameraBlackDuringTurn(
        avoidance, telemetry, line, 35.0, 8, false);
    require(!output.cameraBlackRight && output.cameraBlackRightFrames == 0,
            "Frames sem preto não devem confirmar RIGHT.");

    sampleSideSweep(avoidance, telemetry, line, 1.0, 80.0, 80.0, 80.0);
    output = completeTurn(
        avoidance, telemetry, line, config::kObstacleClearanceScanDegrees);
    require(output.selectedSide == "LEFT" &&
                output.selectedSideSource == "CAMERA_BLACK",
            "Uma confirmação exclusiva da CAM1 deve prevalecer sobre o ultrassônico.");
    require(output.rightClearance > output.leftClearance,
            "O teste deve manter o ultrassônico favorecendo o lado oposto.");
}

void testEarlyFusionRecoveryCompletesToTheSelectedSide()
{
    ObstacleAvoidance avoidance(config::ObstacleSideMode::Automatic);
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    const CameraLineSnapshot line = centeredLine();
    startCurveWithSelectedSide(avoidance, telemetry, line, true);

    avoidance.update(telemetry, fusionLine(1, true, true), true);
    avoidance.update(telemetry, fusionLine(1, true, true), true);
    avoidance.update(telemetry, fusionLine(2, true, true), true);
    ObstacleAvoidanceOutput output =
        avoidance.update(telemetry, fusionLine(3, true, true), true);
    require(output.phase == "obstacle_reacquire_forward" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "Três observações novas do Fusion devem cancelar a curva e o pivot final.");

    const long long recoveryCounts = static_cast<long long>(std::ceil(
        config::kObstacleReacquireForwardDistanceCm *
        config::kEncoderCountsPerCentimeter));
    telemetry.leftEncoderCount += recoveryCounts;
    telemetry.rightEncoderCount += recoveryCounts;
    output = avoidance.update(telemetry, fusionLine(3), true);
    require(output.phase == "obstacle_reacquire_search" &&
                std::abs(output.leftDistanceCm - 5.0) < 0.1 &&
                std::abs(output.rightDistanceCm - 5.0) < 0.1,
            "O avanço da recuperação deve terminar pelos encoders em 5 cm.");

    output = avoidance.update(telemetry, fusionLine(4), true);
    require(output.leftPower > 0.0 && output.rightPower < 0.0,
            "A busca RIGHT deve pivotar para a direita.");
    avoidance.update(telemetry, fusionLine(5), true);
    output = avoidance.update(telemetry, fusionLine(6), true);
    require(output.completed && output.phase == "obstacle_reacquired" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "Três novas observações estáveis devem concluir a recuperação parada.");
}

void testEarlyFusionRecoveryStopsAtAngularLimit()
{
    ObstacleAvoidance avoidance(config::ObstacleSideMode::Automatic);
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    const CameraLineSnapshot line = centeredLine();
    startCurveWithSelectedSide(avoidance, telemetry, line, false);

    avoidance.update(telemetry, fusionLine(20, true, true), true);
    avoidance.update(telemetry, fusionLine(21, true, true), true);
    avoidance.update(telemetry, fusionLine(22, true, true), true);
    const long long recoveryCounts = static_cast<long long>(std::ceil(
        config::kObstacleReacquireForwardDistanceCm *
        config::kEncoderCountsPerCentimeter));
    telemetry.leftEncoderCount += recoveryCounts;
    telemetry.rightEncoderCount += recoveryCounts;
    ObstacleAvoidanceOutput output =
        avoidance.update(telemetry, fusionLine(22), true);
    require(output.phase == "obstacle_reacquire_search",
            "O teste deve entrar na busca após o avanço curto.");

    telemetry.yawZDeg -= config::kObstacleReacquireSearchMaximumDegrees;
    output = avoidance.update(telemetry, fusionLine(23, false), true);
    require(output.failed && output.phase == "obstacle_reacquire_timeout" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "A busca LEFT deve falhar parada ao alcançar 100 graus.");
}

void testEarlyFusionRequiresConsecutiveContinuationBands()
{
    for (const bool selectRight : {false, true})
    {
        ObstacleAvoidance avoidance(config::ObstacleSideMode::Automatic);
        auto telemetry = readyTelemetry();
        startCurveWithSelectedSide(avoidance, telemetry, centeredLine(), selectRight);
        std::uint64_t sequence = 100;
        const auto expectCurve = [&](const CameraLineSnapshot& line) {
            const auto output = avoidance.update(telemetry, line, true);
            require(!output.failed && !output.completed &&
                        output.phase == "obstacle_curving",
                    "Sem confirmações consecutivas de Fusion e faixa, a curva deve continuar.");
        };
        for (int frame = 0; frame < config::kObstacleFusionReacquireConfirmationFrames + 1; ++frame)
        {
            expectCurve(fusionLine(++sequence));
        }
        for (int frame = 0; frame < config::kObstacleFusionReacquireConfirmationFrames; ++frame)
        {
            expectCurve(fusionLine(++sequence, true, true));
            expectCurve(fusionLine(++sequence, true, false));
        }
        for (int frame = 1; frame < config::kObstacleFusionReacquireConfirmationFrames; ++frame)
        {
            expectCurve(fusionLine(++sequence, true, true));
        }
        expectCurve(fusionLine(++sequence, false, true));
        for (int frame = 1; frame < config::kObstacleFusionReacquireConfirmationFrames; ++frame)
        {
            const auto line = fusionLine(++sequence, true, true);
            expectCurve(line);
            expectCurve(line);
        }
        const auto output = avoidance.update(telemetry, fusionLine(++sequence, true, true), true);
        require(output.phase == "obstacle_reacquire_forward" &&
                    output.leftPower == 0.0 && output.rightPower == 0.0,
                "Somente o número configurado de frames novos e consecutivos deve liberar o avanço.");
    }
}

void testParabolaFalseGapUsesBestSideAndReacquires()
{
    ObstacleAvoidance avoidance(config::ObstacleSideMode::Automatic);
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    const CameraLineSnapshot line = centeredLine();
    completeNominalObstacleWithParabolaBest(
        avoidance, telemetry, line, true);

    avoidance.update(telemetry, fusionLine(10), true);
    avoidance.update(telemetry, fusionLine(11), true);
    ObstacleAvoidanceOutput output =
        avoidance.update(telemetry, fusionLine(12), true);
    require(output.case3Armed && output.case3TimeRemainingMs > 0,
            "A primeira aquisição Fusion estável deve armar a janela do caso 3.");

    avoidance.update(telemetry, gapLostLine(13, "LOST"), true);
    avoidance.update(telemetry, gapLostLine(14, "GAP"), true);
    output = avoidance.update(telemetry, gapLostLine(15, "LOST"), true);
    require(output.phase == "obstacle_parabola_gaplost_validate" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "Três GAP/LOST novos dentro da janela devem parar para validar a CAM1.");

    avoidance.update(
        telemetry, gapLostLine(15, "LOST"), true,
        parabolaFrame(20, 0, 0, false));
    avoidance.update(
        telemetry, gapLostLine(15, "LOST"), true,
        parabolaFrame(21, 0, 0, true));
    output = avoidance.update(
        telemetry, gapLostLine(15, "LOST"), true,
        parabolaFrame(22, 0, 0, false));
    require(output.phase == "obstacle_parabola_reacquire_forward" &&
                output.nearForwardLineVotes == 1 &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "Um voto em três deve confirmar ausência frontal e preparar o avanço curto.");

    output = avoidance.update(telemetry, gapLostLine(16, "LOST"), true);
    require(output.phase == "obstacle_parabola_reacquire_forward" &&
                output.leftPower == config::kObstacleReacquireForwardPower &&
                output.rightPower == config::kObstacleReacquireForwardPower,
            "O caso 3 deve avançar reto com a potência do recovery do caso 2.");
    const long long clearanceCounts = static_cast<long long>(std::ceil(
        config::kObstacleParabolaReacquireForwardDistanceCm *
        config::kEncoderCountsPerCentimeter));
    telemetry.leftEncoderCount += clearanceCounts;
    telemetry.rightEncoderCount += clearanceCounts;
    telemetry.yawZDeg = 17.0;
    output = avoidance.update(telemetry, gapLostLine(16, "LOST"), true);
    require(output.phase == "obstacle_parabola_reacquire_search" &&
                std::abs(output.leftDistanceCm -
                    config::kObstacleParabolaReacquireForwardDistanceCm) < 0.1 &&
                std::abs(output.rightDistanceCm -
                    config::kObstacleParabolaReacquireForwardDistanceCm) < 0.1,
            "Os encoders devem encerrar a folga configurada antes do pivot.");

    output = avoidance.update(telemetry, gapLostLine(17, "LOST"), true);
    require(output.leftPower > 0.0 && output.rightPower < 0.0,
            "O bestParabolaSide RIGHT deve comandar pivot para a direita.");

    avoidance.update(telemetry, fusionLine(18), true);
    avoidance.update(telemetry, fusionLine(19), true);
    telemetry.yawZDeg += config::kObstacleParabolaRearBlockDegrees;
    output = avoidance.update(telemetry, fusionLine(20), true);
    require(!output.completed &&
                output.phase ==
                    "obstacle_parabola_reacquire_opposite_search" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "Ao alcançar 65 graus, o Fusion fora do setor deve ser rejeitado e o pivot deve parar.");

    telemetry.yawZDeg -= 10.0;
    output = avoidance.update(telemetry, fusionLine(21), true);
    require(output.leftPower < 0.0 && output.rightPower > 0.0,
            "Depois do bloqueio traseiro, a busca deve inverter para LEFT sem aceitar o Fusion antigo.");

    telemetry.yawZDeg -= config::kObstacleParabolaRearBlockDegrees;
    avoidance.update(telemetry, fusionLine(22), true);
    avoidance.update(telemetry, fusionLine(23), true);
    output = avoidance.update(telemetry, fusionLine(24), true);
    require(output.completed &&
                output.phase == "obstacle_parabola_reacquired" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "Somente três novos Fusion frontais devem concluir o caso 3 com PWM zero.");
}

void testParabolaForwardContinuationKeepsNormalGapHandling()
{
    ObstacleAvoidance avoidance(config::ObstacleSideMode::Automatic);
    auto telemetry = readyTelemetry();
    const auto line = centeredLine();
    completeNominalObstacleWithParabolaBest(avoidance, telemetry, line, false);
    avoidance.update(telemetry, gapLostLine(10, "GAP"), true);
    avoidance.update(telemetry, gapLostLine(11, "GAP"), true);
    auto output = avoidance.update(telemetry, gapLostLine(12, "GAP"), true);
    require(output.phase == "obstacle_parabola_gaplost_validate",
            "O GAP após o reencontro deve validar a continuação frontal.");
    avoidance.update(telemetry, gapLostLine(12, "GAP"), true,
                     parabolaFrame(20, 0, 0, true));
    output = avoidance.update(telemetry, gapLostLine(12, "GAP"), true,
                              parabolaFrame(20, 0, 0, true));
    require(!output.completed && output.nearForwardLineSamples == 1,
            "Um frame repetido da CAM1 não pode decidir a presença frontal.");
    avoidance.update(telemetry, gapLostLine(12, "GAP"), true,
                     parabolaFrame(21, 0, 0, false));
    output = avoidance.update(telemetry, gapLostLine(12, "GAP"), true,
                              parabolaFrame(22, 0, 0, true));
    require(output.completed && output.nearForwardLineVisible &&
                output.leftPower == 0.0 && output.rightPower == 0.0 &&
                !avoidance.active(),
            "Dois votos frontais em três devem manter o GAP normal sem pivot lateral.");
}

void testParabolaOppositeSearchStopsAtSameAngularLimit()
{
    ObstacleAvoidance avoidance(config::ObstacleSideMode::Automatic);
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    const CameraLineSnapshot line = centeredLine();
    completeNominalObstacleWithParabolaBest(
        avoidance, telemetry, line, true);

    avoidance.update(telemetry, fusionLine(40), true);
    avoidance.update(telemetry, fusionLine(41), true);
    avoidance.update(telemetry, fusionLine(42), true);
    avoidance.update(telemetry, gapLostLine(43, "LOST"), true);
    avoidance.update(telemetry, gapLostLine(44, "LOST"), true);
    avoidance.update(telemetry, gapLostLine(45, "LOST"), true);
    avoidance.update(
        telemetry, gapLostLine(45, "LOST"), true,
        parabolaFrame(50, 0, 0, false));
    avoidance.update(
        telemetry, gapLostLine(45, "LOST"), true,
        parabolaFrame(51, 0, 0, false));
    avoidance.update(
        telemetry, gapLostLine(45, "LOST"), true,
        parabolaFrame(52, 0, 0, false));

    const long long clearanceCounts = static_cast<long long>(std::ceil(
        config::kObstacleParabolaReacquireForwardDistanceCm *
        config::kEncoderCountsPerCentimeter));
    telemetry.leftEncoderCount += clearanceCounts;
    telemetry.rightEncoderCount += clearanceCounts;
    telemetry.yawZDeg = 10.0;
    avoidance.update(telemetry, gapLostLine(46, "LOST"), true);

    telemetry.yawZDeg += config::kObstacleParabolaRearBlockDegrees;
    ObstacleAvoidanceOutput output =
        avoidance.update(telemetry, gapLostLine(47, "LOST"), true);
    require(output.phase == "obstacle_parabola_reacquire_opposite_search",
            "O primeiro lado deve ser bloqueado em 65 graus.");

    telemetry.yawZDeg =
        10.0 - config::kObstacleParabolaRearBlockDegrees;
    output = avoidance.update(telemetry, gapLostLine(48, "LOST"), true);
    require(output.failed &&
                output.phase == "obstacle_parabola_reacquire_timeout" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "O lado oposto também deve parar com segurança em 65 graus.");
}

void testPostObstacleFusionCannotReturnToRearLine()
{
    ObstacleAvoidance avoidance(config::ObstacleSideMode::Automatic);
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    const CameraLineSnapshot line = centeredLine();
    completeNominalObstacleWithParabolaBest(
        avoidance, telemetry, line, false);
    const double postObstacleYaw = telemetry.yawZDeg;

    avoidance.update(telemetry, fusionSteeringLine(60, 0.78, -0.72), true);
    avoidance.update(telemetry, fusionSteeringLine(61, 0.78, -0.72), true);
    avoidance.update(telemetry, fusionSteeringLine(62, 0.78, -0.72), true);

    telemetry.yawZDeg = postObstacleYaw +
        config::kObstaclePostObstacleFusionReturnLimitDegrees - 1.0;
    ObstacleAvoidanceOutput output =
        avoidance.update(
            telemetry, fusionSteeringLine(63, 0.78, -0.72), true);
    require(!output.hasControl,
            "Antes de 35 graus, o Fusion normal deve continuar no controle.");

    telemetry.yawZDeg = postObstacleYaw +
        config::kObstaclePostObstacleFusionReturnLimitDegrees;
    output = avoidance.update(
        telemetry, fusionSteeringLine(64, 0.78, -0.72), true);
    require(output.hasControl &&
                output.phase == "obstacle_parabola_reacquire_search" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "Em 35 graus no lado errado, o obstáculo deve bloquear o Fusion imediatamente.");

    output = avoidance.update(
        telemetry, fusionSteeringLine(65, 0.78, -0.72), true);
    require(output.leftPower < 0.0 && output.rightPower > 0.0,
            "Após bloquear o retorno, o recovery deve pivotar para o bestParabolaSide LEFT.");

    avoidance.update(
        telemetry, fusionSteeringLine(66, 0.78, -0.72), true);
    avoidance.update(
        telemetry, fusionSteeringLine(67, 0.78, -0.72), true);
    output = avoidance.update(
        telemetry, fusionSteeringLine(68, 0.78, -0.72), true);
    require(!output.completed,
            "Fusion apontando contra o bestParabolaSide não pode concluir o recovery.");

    avoidance.update(
        telemetry, fusionSteeringLine(69, -0.72, 0.78), true);
    avoidance.update(
        telemetry, fusionSteeringLine(70, -0.72, 0.78), true);
    output = avoidance.update(
        telemetry, fusionSteeringLine(71, -0.72, 0.78), true);
    require(output.completed && output.phase == "obstacle_parabola_reacquired",
            "Três Fusion novos coerentes com LEFT devem concluir o recovery.");
}

void testParabolaCase3WindowExpiresWithoutRecovery()
{
    ObstacleAvoidance avoidance(config::ObstacleSideMode::Automatic);
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    const CameraLineSnapshot line = centeredLine();
    completeNominalObstacleWithParabolaBest(
        avoidance, telemetry, line, false);
    avoidance.update(telemetry, fusionLine(30), true);
    avoidance.update(telemetry, fusionLine(31), true);
    ObstacleAvoidanceOutput output =
        avoidance.update(telemetry, fusionLine(32), true);
    require(output.case3Armed,
            "A janela deve estar armada antes de testar sua expiração.");

    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    output = avoidance.update(telemetry, fusionLine(33), true);
    require(output.case3Armed,
            "Um novo Fusion dentro da janela não deve desarmar o caso 3.");
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kObstacleCase3FusionWindowMs - 1080));
    output = avoidance.update(telemetry, gapLostLine(34, "LOST"), true);
    require(!output.hasControl && !output.case3Armed &&
                !output.bestParabolaSideValid,
            "Após 2000 ms, a memória antiga não pode iniciar recovery lateral.");
}

void testStartCanBeBlockedAndMissingLineStops()
{
    ObstacleAvoidance avoidance(config::ObstacleSideMode::Automatic);
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

int main(int argc, char** argv)
{
    try
    {
        if (argc > 1 && std::string(argv[1]) == "--early-fusion-only")
        {
            testEarlyFusionRecoveryCompletesToTheSelectedSide();
            testEarlyFusionRecoveryStopsAtAngularLimit();
            testEarlyFusionRequiresConsecutiveContinuationBands();
            std::cout << "early_fusion_test: OK\n";
            return 0;
        }
        testCenteringIsSharedAndPrecedesScan();
        testStableEndpointUsesMaximumAndSelectsRight();
        testLeftSelectionReturnsToLeftYaw();
        testPracticalTieUsesFixedSide();
        testCameraBlackSelectsOnlyConfirmedSide();
        testEarlyFusionRecoveryCompletesToTheSelectedSide();
        testEarlyFusionRecoveryStopsAtAngularLimit();
        testEarlyFusionRequiresConsecutiveContinuationBands();
        testParabolaFalseGapUsesBestSideAndReacquires();
        testParabolaForwardContinuationKeepsNormalGapHandling();
        testParabolaOppositeSearchStopsAtSameAngularLimit();
        testPostObstacleFusionCannotReturnToRearLine();
        testParabolaCase3WindowExpiresWithoutRecovery();
        testForcedLeftProfileReacquiresDuringStraightExit();
        testForcedRightSkipsSideMeasurement();
        testObstacleTurnAcceptsEncoderStopWithGyroBias();
        testExitDistanceRequiresBothEncoders();
        testFusionVotesCarryFromTurnToTimedForward();
        testForcedLeftProfileReacquiresDuringRightSearch();
        testForcedLeftProfileSearchTimeoutStops();
        testAutomaticNominalExitMirrorsBothSides();
        testForcedSideStillAllowsEarlyRecovery();
        testExitStopsWithStaleSensors();
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
