#include "obr/config.h"
#include "obr/rescue_room_mission.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
using Clock = std::chrono::steady_clock;
constexpr std::uint64_t kRunSequence = 7;

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
    telemetry.pca9685Ok = true;
    telemetry.servoExtendedControlSupported = true;
    telemetry.servoHoldSupported = true;
    telemetry.servoHoldActive = true;
    telemetry.armServoEnabled = true;
    telemetry.wristServoEnabled = true;
    telemetry.gripperServoEnabled = true;
    telemetry.motorSleepPinHigh = true;
    telemetry.lastSensorAgeMs = 0;
    telemetry.leftEncoderRate = 0.0;
    telemetry.rightEncoderRate = 0.0;
    telemetry.yawZDeg = 0.0;
    telemetry.gyroZDegPerSec = 0.0;
    telemetry.esp32UptimeMs = 1;
    return telemetry;
}

ForwardBallSnapshot emptyFrame(
    std::uint64_t targetSequence,
    double timestamp)
{
    ForwardBallSnapshot ball;
    ball.sourceFresh = true;
    ball.targetSequence = targetSequence;
    ball.timestamp = timestamp;
    return ball;
}

ForwardBallSnapshot lockedSilver(
    std::uint64_t targetSequence,
    double timestamp)
{
    ForwardBallSnapshot ball = emptyFrame(targetSequence, timestamp);
    ball.detected = true;
    ball.candidateVisible = true;
    ball.targetLocked = true;
    ball.type = "silver_ball";
    ball.txDegrees = 0.0;
    ball.distanceCm = config::kBallApproachStopDistanceCm;
    ball.radiusPixels = 200.0;
    ball.visibleAreaPixels = 20000.0;
    return ball;
}

ForwardBallSnapshot lockedBlack(
    std::uint64_t targetSequence,
    double timestamp)
{
    ForwardBallSnapshot ball = lockedSilver(targetSequence, timestamp);
    ball.type = "black_ball";
    return ball;
}

RescueZoneSnapshot confirmedZone(bool green, std::uint64_t sequence)
{
    RescueZoneSnapshot zones;
    zones.sourceFresh = true;
    zones.sequence = sequence;
    RescueZoneObservation& zone = green ? zones.green : zones.red;
    zone.candidateDetected = true;
    zone.detected = true;
    // O caso sem duas bordas usa o modo best effort já coberto pelos testes
    // próprios do alinhamento e permite testar aqui apenas a ordem da sala.
    zone.geometryState = RescueZoneGeometryState::BoundsUnknown;
    return zones;
}

RescueRoomOutput updateMission(
    RescueRoomMission& mission,
    const ForwardBallSnapshot& ball,
    const RescueZoneSnapshot& zones,
    Esp32TelemetrySnapshot& telemetry,
    ServoPose& servoPose,
    Clock::time_point& now)
{
    RescueRoomOutput output = mission.update(
        ball,
        zones,
        telemetry,
        kRunSequence,
        0,
        servoPose,
        now);
    if (output.servoPoseRequested)
    {
        servoPose = output.servoPose;
    }
    return output;
}

RescueRoomOutput completeServoStage(
    RescueRoomMission& mission,
    const ForwardBallSnapshot& ball,
    Esp32TelemetrySnapshot& telemetry,
    ServoPose& servoPose,
    Clock::time_point& now,
    const std::string& expectedPhase)
{
    RescueRoomOutput output;
    for (int updateCount = 0; updateCount < 30; ++updateCount)
    {
        now += std::chrono::seconds(3);
        output = updateMission(
            mission, ball, {}, telemetry, servoPose, now);
        require(!output.failed, "A etapa de servo não deve falhar no fluxo nominal.");
        if (output.status.phase == expectedPhase)
        {
            return output;
        }
    }
    throw std::runtime_error(
        "A etapa de servo não alcançou a fase esperada: " + expectedPhase);
}

RescueRoomOutput completeDistanceStage(
    RescueRoomMission& mission,
    const ForwardBallSnapshot& ball,
    Esp32TelemetrySnapshot& telemetry,
    ServoPose& servoPose,
    Clock::time_point& now,
    double distanceCm)
{
    RescueRoomOutput output = updateMission(
        mission, ball, {}, telemetry, servoPose, now);
    require(!output.failed && output.leftPower != 0.0 &&
                output.rightPower != 0.0,
            "O deslocamento nominal deve aplicar potência nos dois lados.");
    if (distanceCm == config::kRescuePostCollectionReverseDistanceCm)
    {
        require(output.servoPoseRequested &&
                    output.servoPose.armDegrees == config::kServoRoutineArmPickupDegrees &&
                    output.servoPose.wristDegrees == config::kServoRoutineWristForwardDegrees &&
                    output.servoPose.gripperDegrees ==
                        config::kServoRoutineGripperRetentionDegrees,
                "A ré deve manter braço baixo, pulso imóvel e garra energizada em 5°.");
    }

    const long long counts = static_cast<long long>(std::ceil(
        distanceCm * config::kEncoderCountsPerCentimeter));
    telemetry.leftEncoderCount += counts;
    telemetry.rightEncoderCount += counts;
    ++telemetry.esp32UptimeMs;
    now += std::chrono::milliseconds(20);
    output = updateMission(
        mission, ball, {}, telemetry, servoPose, now);
    require(!output.failed && output.leftPower == 0.0 &&
                output.rightPower == 0.0,
            "A distância alcançada deve zerar o PWM antes da transição.");

    now += std::chrono::milliseconds(config::kRescueDistanceSettleMs);
    return updateMission(
        mission, ball, {}, telemetry, servoPose, now);
}

RescueRoomOutput collectVictim(
    RescueRoomMission& mission,
    const ForwardBallSnapshot& victim,
    Esp32TelemetrySnapshot& telemetry,
    ServoPose& servoPose,
    Clock::time_point& now)
{
    RescueRoomOutput output = updateMission(
        mission, victim, {}, telemetry, servoPose, now);
    require(output.status.phase == "rescue_victim_acquired",
            "A busca deve travar a vítima do tipo solicitado.");

    output = updateMission(
        mission, victim, {}, telemetry, servoPose, now);
    require(output.status.phase == "rescue_victim_aligned" &&
                !output.servoPoseRequested,
            "O YOLO deve alinhar antes de liberar a preparação dos servos.");

    output = completeServoStage(
        mission,
        victim,
        telemetry,
        servoPose,
        now,
        "rescue_capture_ready");
    output = updateMission(
        mission, victim, {}, telemetry, servoPose, now);
    require(output.status.phase == "rescue_victim_acquired",
            "A aproximação deve começar pelo alinhamento validado.");

    now += std::chrono::milliseconds(20);
    output = updateMission(
        mission, victim, {}, telemetry, servoPose, now);
    require(output.status.phase == "victim_collection_preparing",
            "A vítima próxima deve preparar o avanço final de 5 cm.");
    now += std::chrono::milliseconds(20);
    output = updateMission(
        mission, victim, {}, telemetry, servoPose, now);
    require(output.status.phase == "victim_collection_advancing",
            "A aproximação deve executar o avanço final por encoders.");

    const long long collectionCounts = static_cast<long long>(std::ceil(
        config::kVictimCollectionAdvanceDistanceCm *
        config::kEncoderCountsPerCentimeter));
    telemetry.leftEncoderCount += collectionCounts;
    telemetry.rightEncoderCount += collectionCounts;
    ++telemetry.esp32UptimeMs;
    now += std::chrono::milliseconds(20);
    output = updateMission(
        mission, victim, {}, telemetry, servoPose, now);
    require(output.status.phase == "victim_collection_settling",
            "O avanço final deve estabilizar antes de fechar a garra.");

    now += std::chrono::milliseconds(config::kVictimCollectionSettleMs);
    output = updateMission(
        mission, victim, {}, telemetry, servoPose, now);
    require(output.status.phase == "rescue_victim_reached",
            "O fechamento só pode começar depois da aproximação completa.");

    return completeServoStage(
        mission,
        victim,
        telemetry,
        servoPose,
        now,
        "rescue_collection_secured");
}

RescueRoomOutput reachDepositZone(
    RescueRoomMission& mission,
    const ForwardBallSnapshot& previousVictim,
    bool green,
    Esp32TelemetrySnapshot& telemetry,
    ServoPose& servoPose,
    Clock::time_point& now,
    bool pauseDuringAdvance = false)
{
    RescueRoomOutput output;
    for (std::uint64_t sequence = 1; sequence <= 4; ++sequence)
    {
        output = updateMission(
            mission,
            previousVictim,
            confirmedZone(green, sequence),
            telemetry,
            servoPose,
            now);
        require(!output.failed,
                "A busca nominal do triângulo não deve falhar.");
        if (sequence == 3 && pauseDuringAdvance)
        {
            now += std::chrono::milliseconds(200);
            telemetry.serialOpen = false;
            output = updateMission(mission, previousVictim, confirmedZone(green, 4),
                                   telemetry, servoPose, now);
            require(output.leftPower == 0.0 && output.rightPower == 0.0 &&
                        output.status.phase == "rescue_room_esp32_not_ready",
                    "A perda transitória da ESP32 deve pausar o avanço.");
            now += std::chrono::milliseconds(2000);
            telemetry.serialOpen = true;
            output = updateMission(mission, previousVictim, confirmedZone(green, 5),
                                   telemetry, servoPose, now);
            require(output.status.phase == "rescue_zone_approach_final_advance" &&
                        output.leftPower == config::kRescueZoneApproachFinalAdvancePower,
                    "O retorno da ESP32 deve retomar o avanço, sem antecipar o depósito.");
            now += std::chrono::milliseconds(config::kRescueZoneApproachFinalAdvanceMs - 201);
            output = updateMission(mission, previousVictim, confirmedZone(green, 6),
                                   telemetry, servoPose, now);
            require(output.status.phase == "rescue_zone_approach_final_advance",
                    "Com 1.499 ms autorizados, o depósito continua bloqueado.");
            now += std::chrono::milliseconds(1);
            continue;
        }
        now += sequence == 3
                   ? std::chrono::milliseconds(
                         config::kRescueZoneApproachFinalAdvanceMs)
                   : std::chrono::milliseconds(20);
    }
    require(output.status.phase == "rescue_deposit_ready",
            green
                ? "As vítimas vivas devem chegar ao triângulo verde."
                : "A vítima morta deve chegar ao triângulo vermelho.");
    return output;
}

void completeEntryAdvance(
    RescueRoomMission& mission,
    Esp32TelemetrySnapshot& telemetry,
    Clock::time_point& now,
    std::uint64_t targetSequence,
    double candidateTx = NAN)
{
    ForwardBallSnapshot entryFrame = emptyFrame(targetSequence, 1.0);
    entryFrame.candidateVisible = std::isfinite(candidateTx);
    entryFrame.candidateTxDegrees = candidateTx;
    RescueRoomOutput output = mission.update(
        entryFrame,
        {},
        telemetry,
        kRunSequence,
        0,
        {},
        now);
    require(output.status.phase == "rescue_entry_advancing" &&
                output.leftPower == config::kRescueEntryAdvancePower &&
                output.rightPower == config::kRescueEntryAdvancePower,
            "A entrada deve avançar 10 cm com o YOLO já ativo.");

    const long long counts = static_cast<long long>(std::ceil(
        config::kRescueEntryAdvanceDistanceCm *
        config::kEncoderCountsPerCentimeter));
    telemetry.leftEncoderCount += counts;
    telemetry.rightEncoderCount += counts;
    ++telemetry.esp32UptimeMs;
    now += std::chrono::milliseconds(20);
    output = mission.update(
        emptyFrame(targetSequence, 2.0), {}, telemetry, kRunSequence, 0, {}, now);
    require(output.status.phase == "rescue_distance_settling" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "A entrada deve zerar o PWM ao completar os dois encoders.");

    now += std::chrono::milliseconds(config::kRescueDistanceSettleMs);
    output = mission.update(
        emptyFrame(targetSequence, 3.0), {}, telemetry, kRunSequence, 0, {}, now);
    require(output.status.phase == "rescue_search_starting",
            "Sem vítima, o avanço deve entregar controle à varredura.");
}

void testSweepRemembersEntryCandidateAndDefaultsRight()
{
    for (double hint : {static_cast<double>(NAN), -20.0, 20.0})
    {
        RescueRoomMission mission;
        auto telemetry = readyTelemetry();
        auto now = Clock::time_point{};
        ServoPose pose;
        const auto sequence = mission.ballTargetSequence(kRunSequence);
        completeEntryAdvance(mission, telemetry, now, sequence, hint);
        const auto output = updateMission(
            mission, emptyFrame(sequence, 4.0), {}, telemetry, pose, now);
        require(output.status.phase == "rescue_search_sweep" &&
                    (hint < 0.0 ? output.leftPower < 0.0 : output.leftPower > 0.0),
                "A busca deve lembrar o lado visto na entrada; sem pista, começa à direita.");
        require(output.servoPoseRequested &&
                    output.servoPose.armDegrees ==
                        config::kServoRoutineArmStorageTransitionDegrees &&
                    output.servoPose.wristDegrees ==
                        config::kServoRoutineWristStorageClearanceDegrees &&
                    output.servoPose.gripperDegrees ==
                        config::kServoRoutineGripperClosedDegrees,
                "A busca deve manter braço e pulso recolhidos contra colisões.");
        mission.reset();
        telemetry = readyTelemetry();
        completeEntryAdvance(mission, telemetry, now, sequence);
        const auto restartedOutput = updateMission(
            mission, emptyFrame(sequence, 4.0), {}, telemetry, pose, now);
        require(restartedOutput.status.phase == "rescue_search_sweep" &&
                    restartedOutput.leftPower > 0.0 && restartedOutput.rightPower < 0.0,
                "Após reiniciar a missão sem pista, a busca deve voltar ao padrão à direita.");
    }
}

void testSweepTimeoutsReverseExpandAndStop()
{
    RescueRoomMission mission;
    auto telemetry = readyTelemetry();
    auto now = Clock::time_point{};
    ServoPose pose;
    const auto sequence = mission.ballTargetSequence(kRunSequence);
    completeEntryAdvance(mission, telemetry, now, sequence, -20.0);
    auto frame = emptyFrame(sequence, 10.0);
    for (int attempt = 0; attempt < 4; ++attempt)
    {
        auto output = updateMission(mission, frame, {}, telemetry, pose, now);
        require((attempt % 2 == 0 ? output.leftPower < 0.0 : output.leftPower > 0.0),
                "Cada timeout deve alternar o sentido antes de ampliar a varredura.");
        const int limitMs = attempt < 2 ? 3000 : 8000;
        now += std::chrono::milliseconds(limitMs - 1);
        output = updateMission(mission, frame, {}, telemetry, pose, now);
        require(output.leftPower != 0.0 && !output.failed,
                "A tentativa deve manter seu orçamento de 3 ou 8 segundos.");
        now += std::chrono::milliseconds(1);
        output = updateMission(mission, frame, {}, telemetry, pose, now);
        require(output.status.phase == "rescue_search_turn_timeout" &&
                    output.leftPower == 0.0 && output.rightPower == 0.0,
                "No limite exato, deve zerar o PWM antes de inverter.");
    }
    const auto output = updateMission(mission, frame, {}, telemetry, pose, now);
    require(output.failed && output.status.phase == "rescue_search_blocked" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "Quatro tentativas bloqueadas devem terminar sem novo giro.");
}

void testSweepUsesActualHeadingAfterPartialTurn()
{
    RescueRoomMission mission;
    auto telemetry = readyTelemetry();
    telemetry.yawZDeg = 170.0;
    auto now = Clock::time_point{};
    ServoPose pose;
    const auto sequence = mission.ballTargetSequence(kRunSequence);
    completeEntryAdvance(mission, telemetry, now, sequence, -20.0);
    auto frame = emptyFrame(sequence, 10.0);
    updateMission(mission, frame, {}, telemetry, pose, now);
    telemetry.yawZDeg = 160.0;
    now += std::chrono::milliseconds(3000);
    updateMission(mission, frame, {}, telemetry, pose, now);
    auto output = updateMission(mission, frame, {}, telemetry, pose, now);
    require(output.leftPower > 0.0, "Após giro parcial à esquerda, deve inverter.");
    telemetry.yawZDeg = -145.0;
    now += std::chrono::milliseconds(100);
    updateMission(mission, frame, {}, telemetry, pose, now);
    now += std::chrono::milliseconds(config::kTurn90SettleMs);
    output = updateMission(mission, frame, {}, telemetry, pose, now);
    require(output.status.phase == "rescue_search_endpoint",
            "O destino deve ser +45° da referência, incluindo a passagem por 180°.");
}

void testCandidateWaitDoesNotResetSweepTimeout()
{
    RescueRoomMission mission;
    auto telemetry = readyTelemetry();
    auto now = Clock::time_point{};
    ServoPose pose;
    const auto sequence = mission.ballTargetSequence(kRunSequence);
    completeEntryAdvance(mission, telemetry, now, sequence);
    auto candidate = emptyFrame(sequence, 10.0);
    candidate.candidateVisible = true;
    candidate.candidateTxDegrees = 20.0;
    auto output = updateMission(mission, candidate, {}, telemetry, pose, now);
    require(output.status.phase == "rescue_confirming_victim" && output.leftPower == 0.0,
            "Uma candidata sozinha nunca libera alinhamento ou coleta.");
    now += std::chrono::milliseconds(100);
    output = updateMission(mission, emptyFrame(sequence, 10.1), {}, telemetry, pose, now);
    require(output.status.phase == "rescue_reacquiring_candidate" &&
                output.leftPower > 0.0 && output.rightPower < 0.0,
            "Um frame intercalado deve retornar ao último ponto visto da candidata.");
    now += std::chrono::milliseconds(2900);
    output = updateMission(mission, candidate, {}, telemetry, pose, now);
    require(output.status.phase == "rescue_search_turn_timeout",
            "Candidata sem confirmação não pode reiniciar o orçamento.");
    telemetry.mpuOk = false;
    output = updateMission(mission, emptyFrame(sequence, 11.0), {}, telemetry, pose, now);
    require(output.leftPower == 0.0 && output.rightPower == 0.0,
            "IMU inválida deve bloquear a próxima tentativa.");
}

void testDelicateMotionUsesOneShortKick()
{
    RescueRoomMission mission;
    auto telemetry = readyTelemetry();
    auto now = Clock::time_point{};
    ServoPose pose;
    const auto sequence = mission.ballTargetSequence(kRunSequence);
    completeEntryAdvance(mission, telemetry, now, sequence);

    const auto frame = emptyFrame(sequence, 4.0);
    RescueRoomOutput output = updateMission(
        mission, frame, {}, telemetry, pose, now);
    require(
        std::abs(output.leftPower) == config::kRescueDelicateMotionKickPower &&
            std::abs(output.rightPower) == config::kRescueDelicateMotionKickPower,
        "O primeiro ciclo do giro deve usar o micropulso de 0,80.");

    now += std::chrono::milliseconds(
        config::kRescueDelicateMotionKickDurationMs);
    output = updateMission(mission, frame, {}, telemetry, pose, now);
    require(
        std::abs(output.leftPower) == config::kRescueSearchTurnPower &&
            std::abs(output.rightPower) == config::kRescueSearchTurnPower,
        "Após o micropulso, o mesmo giro deve retomar a potência original.");
}

void testRequiredVictimSearchBecomesContinuousAfterInitialSweep()
{
    RescueRoomMission mission;
    auto telemetry = readyTelemetry();
    auto now = Clock::time_point{};
    ServoPose pose;
    const auto sequence = mission.ballTargetSequence(kRunSequence);
    completeEntryAdvance(mission, telemetry, now, sequence, -20.0);

    double frameTimestamp = 10.0;
    const double endpoints[] = {-45.0, 45.0, -75.0, 75.0};
    for (const double endpoint : endpoints)
    {
        ForwardBallSnapshot frame = emptyFrame(sequence, frameTimestamp);
        RescueRoomOutput output = updateMission(
            mission, frame, {}, telemetry, pose, now);
        require(
            output.status.phase == "rescue_search_sweep",
            "A varredura inicial deve comandar cada heading antes da busca "
            "contínua.");

        telemetry.yawZDeg = endpoint;
        now += std::chrono::milliseconds(20);
        updateMission(mission, frame, {}, telemetry, pose, now);
        now += std::chrono::milliseconds(config::kTurn90SettleMs);
        output = updateMission(mission, frame, {}, telemetry, pose, now);
        require(
            output.status.phase == "rescue_search_endpoint" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "Cada heading inicial deve terminar parado antes do próximo.");
        frameTimestamp += 1.0;
    }

    RescueRoomOutput output = updateMission(
        mission,
        emptyFrame(sequence, frameTimestamp),
        {}, telemetry, pose, now);
    require(
        output.status.phase == "rescue_required_alive_search_continuous" &&
            output.leftPower == 0.0 && output.rightPower == 0.0,
        "Ao terminar os headings, a busca obrigatória deve preparar o giro "
        "contínuo.");

    output = updateMission(
        mission,
        emptyFrame(sequence, frameTimestamp + 1.0),
        {}, telemetry, pose, now);
    require(
        output.status.phase == "rescue_search_continuous" &&
            output.leftPower < 0.0 && output.rightPower > 0.0,
        "A busca pós-varredura deve girar continuamente para um único lado.");

    telemetry.yawZDeg = -120.0;
    now += std::chrono::seconds(10);
    output = updateMission(
        mission,
        emptyFrame(sequence, frameTimestamp + 2.0),
        {}, telemetry, pose, now);
    require(
        output.status.phase == "rescue_search_continuous" &&
            output.leftPower < 0.0 && output.rightPower > 0.0,
        "A busca contínua não deve inverter nem reutilizar os headings iniciais.");

    now += std::chrono::milliseconds(
        config::kRescueContinuousSearchStallTimeoutMs - 1);
    telemetry.yawZDeg = -106.0;
    output = updateMission(
        mission,
        emptyFrame(sequence, frameTimestamp + 3.0),
        {}, telemetry, pose, now);
    require(
        output.leftPower < 0.0 && output.rightPower > 0.0,
        "Antes de dois segundos dentro da faixa de 15 graus, a busca deve preservar o sentido.");

    now += std::chrono::milliseconds(1);
    output = updateMission(
        mission,
        emptyFrame(sequence, frameTimestamp + 4.0),
        {}, telemetry, pose, now);
    require(
        output.status.phase == "rescue_search_continuous" &&
            output.leftPower > 0.0 && output.rightPower < 0.0,
        "Após dois segundos dentro de 15 graus, a busca deve inverter o giro.");
}

void testReverseFailureNeverLiftsVictim()
{
    RescueRoomMission mission;
    auto telemetry = readyTelemetry();
    auto now = Clock::time_point{};
    ServoPose pose;
    const auto sequence = mission.ballTargetSequence(kRunSequence);
    completeEntryAdvance(mission, telemetry, now, sequence);
    const auto victim = lockedSilver(sequence, 10.0);
    collectVictim(mission, victim, telemetry, pose, now);
    auto output = updateMission(mission, victim, {}, telemetry, pose, now);
    require(output.leftPower < 0.0 && pose.armDegrees == config::kServoRoutineArmPickupDegrees,
            "Depois de prender, deve recuar com o braço baixo.");
    now += std::chrono::milliseconds(config::kRescueDistanceStallTimeoutMs);
    output = updateMission(mission, victim, {}, telemetry, pose, now);
    require(output.failed && output.status.phase == "rescue_distance_stall" &&
                output.leftPower == 0.0 && output.rightPower == 0.0 &&
                output.servoPoseRequested && pose.armDegrees == config::kServoRoutineArmPickupDegrees &&
                pose.gripperDegrees == config::kServoRoutineGripperRetentionDegrees,
            "Ré bloqueada deve falhar mantendo a vítima presa e o braço baixo.");
    now += std::chrono::seconds(10);
    output = updateMission(mission, victim, {}, telemetry, pose, now);
    require(output.failed && output.servoPoseRequested &&
                pose.armDegrees == config::kServoRoutineArmPickupDegrees,
            "A falha não pode liberar uma elevação tardia.");
}

void testEntryAdvanceDoesNotWaitForOrStopOnYolo()
{
    RescueRoomMission mission;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    RescueRoomOutput output = mission.update(
        {}, {}, telemetry, kRunSequence, 0, {}, Clock::time_point{});
    require(output.status.phase == "rescue_entry_advancing" && !output.failed,
            "O avanço mínimo deve começar mesmo sem resultado do YOLO.");
    require(output.leftPower > 0.0 && output.rightPower > 0.0,
            "Sem YOLO atual, a entrada deve concluir o avanço mínimo.");
    require(mission.requiresBallDetection() &&
                std::string(mission.ballTargetType()) == "silver_ball",
            "A entrada deve solicitar somente vítimas vivas.");

    ForwardBallSnapshot previousTarget = emptyFrame(
        mission.ballTargetSequence(kRunSequence) - 1,
        1.0);
    output = mission.update(
        previousTarget,
        {},
        telemetry,
        kRunSequence,
        0,
        {},
        Clock::time_point{});
    require(output.status.phase == "rescue_entry_advancing" &&
                output.leftPower > 0.0 && output.rightPower > 0.0,
            "Um frame de outra geração não deve bloquear o avanço mínimo.");

    auto victim = lockedSilver(mission.ballTargetSequence(kRunSequence), 2.0);
    victim.txDegrees = 20.0;
    telemetry.leftEncoderCount = static_cast<long long>(std::ceil(
        config::kRescueEntryAdvanceDistanceCm *
        config::kEncoderCountsPerCentimeter * 0.5));
    telemetry.rightEncoderCount = telemetry.leftEncoderCount;
    output = mission.update(victim, {}, telemetry, kRunSequence, 0, {},
                            Clock::time_point{} + std::chrono::milliseconds(20));
    require(output.status.phase == "rescue_entry_advancing" &&
                output.leftPower > 0.0 && output.rightPower > 0.0 &&
                !output.servoPoseRequested && !output.failed,
            "Uma vítima travada no meio da entrada não deve interromper o avanço.");

    telemetry.leftEncoderCount = static_cast<long long>(std::ceil(
        config::kRescueEntryAdvanceDistanceCm * config::kEncoderCountsPerCentimeter));
    telemetry.rightEncoderCount = telemetry.leftEncoderCount;
    mission.update({}, {}, telemetry, kRunSequence, 0, {},
                   Clock::time_point{} + std::chrono::milliseconds(40));
    output = mission.update({}, {}, telemetry, kRunSequence, 0, {},
        Clock::time_point{} + std::chrono::milliseconds(40 + config::kRescueDistanceSettleMs));
    require(output.status.phase == "rescue_search_starting" && !output.failed,
            "A entrada deve liberar a busca somente depois de concluir a distância.");
    output = mission.update(emptyFrame(mission.ballTargetSequence(kRunSequence), 3.0),
                            {}, telemetry, kRunSequence, 0, {},
        Clock::time_point{} + std::chrono::milliseconds(41 + config::kRescueDistanceSettleMs));
    require(output.status.phase == "rescue_search_sweep" && output.leftPower > 0.0,
            "Mesmo se a vítima sumir, a busca deve começar pelo lado direito memorizado.");
}

void testTransientEsp32LossPausesWithoutKillingMission()
{
    RescueRoomMission mission;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    const std::uint64_t targetSequence =
        mission.ballTargetSequence(kRunSequence);
    const ForwardBallSnapshot frame = emptyFrame(targetSequence, 1.0);
    telemetry.sensorFresh = false;

    RescueRoomOutput output = mission.update(
        frame, {}, telemetry, kRunSequence, 0, {}, Clock::time_point{});
    require(!output.failed &&
                output.status.phase == "rescue_room_esp32_not_ready" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "Uma perda transitória da ESP32 deve pausar, sem matar a missão.");

    telemetry.sensorFresh = true;
    output = mission.update(
        frame, {}, telemetry, kRunSequence, 0, {}, Clock::time_point{});
    require(!output.failed &&
                output.status.phase == "rescue_entry_advancing",
            "A missão deve continuar quando a telemetria segura voltar.");
}

void testPreparesOpenGripperBeforeApproach()
{
    RescueRoomMission mission;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    Clock::time_point now{};
    const std::uint64_t targetSequence =
        mission.ballTargetSequence(kRunSequence);
    completeEntryAdvance(mission, telemetry, now, targetSequence);

    RescueRoomOutput output = mission.update(
        lockedSilver(targetSequence, 4.0),
        {}, telemetry, kRunSequence, 0, {}, now);
    require(output.status.phase == "rescue_victim_acquired" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "A vítima confirmada deve interromper a varredura imediatamente.");

    output = mission.update(
        lockedSilver(targetSequence, 4.1),
        {}, telemetry, kRunSequence, 0, {}, now);
    require(output.status.phase == "rescue_victim_aligned" &&
                !output.servoPoseRequested,
            "O alinhamento inicial deve terminar antes da pose de coleta.");

    bool reachedOpenPickupPose = false;
    ServoPose currentPose{};
    for (int updateCount = 0; updateCount < 12; ++updateCount)
    {
        now += std::chrono::seconds(2);
        output = mission.update(
            lockedSilver(targetSequence, 5.0 + updateCount),
            {}, telemetry, kRunSequence, 0, currentPose, now);
        require(output.leftPower == 0.0 && output.rightPower == 0.0,
                "A preparação da garra nunca deve comandar tração.");
        if (output.servoPoseRequested &&
            output.status.phase == "servo_capture_arm_pickup")
        {
            reachedOpenPickupPose =
                output.servoPose.armDegrees ==
                    config::kServoRoutineArmPickupDegrees &&
                output.servoPose.gripperDegrees ==
                    config::kServoRoutineGripperFullyOpenDegrees;
        }
        if (output.servoPoseRequested)
        {
            currentPose = output.servoPose;
        }
        if (output.status.phase == "rescue_capture_ready")
        {
            break;
        }
    }
    require(reachedOpenPickupPose,
            "A aproximação deve receber a pose validada com a garra aberta.");

    output = mission.update(
        lockedSilver(targetSequence, 30.0),
        {}, telemetry, kRunSequence, 0,
        {config::kServoRoutineArmPickupDegrees,
         config::kServoRoutineWristForwardDegrees,
         config::kServoRoutineGripperFullyOpenDegrees},
        now + std::chrono::milliseconds(20));
    require(output.status.phase == "rescue_victim_acquired",
            "Depois da preparação, o YOLO deve iniciar o alinhamento da vítima.");
}

void testRunsRequiredVictimsInPriorityOrder()
{
    RescueRoomMission mission;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    telemetry.ultrasonicDistanceCm =
        config::kRescueZoneApproachStopDistanceCm;
    ServoPose servoPose{};
    Clock::time_point now{};

    std::uint64_t targetSequence = mission.ballTargetSequence(kRunSequence);
    completeEntryAdvance(mission, telemetry, now, targetSequence);

    ForwardBallSnapshot firstAlive = lockedSilver(targetSequence, 10.0);
    collectVictim(mission, firstAlive, telemetry, servoPose, now);
    RescueRoomOutput output = completeDistanceStage(
        mission,
        firstAlive,
        telemetry,
        servoPose,
        now,
        config::kRescuePostCollectionReverseDistanceCm);
    require(output.status.phase == "rescue_collection_lifting" &&
                servoPose.armDegrees == config::kServoRoutineArmPickupDegrees,
            "A ré deve terminar antes de solicitar elevação.");
    output = completeServoStage(mission, firstAlive, telemetry, servoPose, now,
                                "rescue_first_alive_storage");
    require(output.status.phase == "rescue_first_alive_storage",
            "A primeira vítima viva deve seguir para o armazenamento interno.");
    require(output.servoPose.armDegrees ==
                    config::kServoRoutineArmHomeDegrees &&
                output.servoPose.wristDegrees ==
                    config::kServoRoutineWristForwardDegrees &&
                output.servoPose.gripperDegrees ==
                    config::kServoRoutineGripperRetentionDegrees,
            "Após a elevação, a retenção deve usar a pose 15°/180°/5°.");

    output = completeServoStage(
        mission,
        firstAlive,
        telemetry,
        servoPose,
        now,
        "rescue_first_alive_stored");
    require(output.internalObjectStored &&
                std::string(mission.ballTargetType()) == "silver_ball",
            "Depois de armazenar a primeira prata, a segunda continua prioritária.");
    require(servoPose.armDegrees ==
                    config::kServoRoutineArmStorageTransitionDegrees &&
                servoPose.wristDegrees ==
                    config::kServoRoutineWristStorageClearanceDegrees &&
                servoPose.gripperDegrees ==
                    config::kServoRoutineGripperClosedDegrees,
            "O armazenamento deve terminar na pose recolhida usada pela busca.");

    targetSequence = mission.ballTargetSequence(kRunSequence);
    output = updateMission(
        mission, emptyFrame(targetSequence, 19.0), {}, telemetry, servoPose, now);
    require(output.status.phase == "rescue_search_continuous" &&
                output.leftPower != 0.0 &&
                output.rightPower == -output.leftPower,
            "Depois da primeira coleta, a busca seguinte deve girar continuamente sem repetir headings.");
    ForwardBallSnapshot secondAlive = lockedSilver(targetSequence, 20.0);
    collectVictim(mission, secondAlive, telemetry, servoPose, now);
    output = completeDistanceStage(
        mission,
        secondAlive,
        telemetry,
        servoPose,
        now,
        config::kRescuePostCollectionReverseDistanceCm);
    output = completeServoStage(mission, secondAlive, telemetry, servoPose, now,
                                "rescue_deposit_zone_starting");
    require(output.status.phase == "rescue_deposit_zone_starting" &&
                output.internalObjectStored,
            "A segunda prata deve iniciar o depósito mantendo a primeira guardada.");

    reachDepositZone(
        mission, secondAlive, true, telemetry, servoPose, now, true);
    output = completeServoStage(
        mission,
        secondAlive,
        telemetry,
        servoPose,
        now,
        "rescue_deposit_completed");
    require(!output.internalObjectStored,
            "O depósito verde deve liberar as duas pratas e limpar o armazenamento.");

    output = updateMission(
        mission, secondAlive, {}, telemetry, servoPose, now);
    require(!output.failed && output.status.phase == "rescue_deposit_reversing" &&
                output.leftPower < 0.0 && output.rightPower < 0.0,
            "Após as duas pratas, a ré de 20 cm deve começar.");
    const long long reverseCounts = static_cast<long long>(std::ceil(
        config::kRescuePostDepositReverseDistanceCm *
        config::kEncoderCountsPerCentimeter));
    const long long sideLeadCounts = static_cast<long long>(std::ceil(
        (config::kDriveDistanceMaximumSideDifferenceCm + 1.0) *
        config::kEncoderCountsPerCentimeter));
    telemetry.leftEncoderCount += sideLeadCounts;
    for (int sample = 0;
         sample < config::kDriveDistanceDifferenceConfirmationSamples;
         ++sample)
    {
        ++telemetry.esp32UptimeMs;
        now += std::chrono::milliseconds(20);
        output = updateMission(
            mission, secondAlive, {}, telemetry, servoPose, now);
        require(!output.failed &&
                    output.status.phase == "rescue_deposit_reversing" &&
                    output.leftPower < 0.0 && output.rightPower < 0.0,
                "A diferença entre encoders não deve encerrar a ré do resgate.");
    }
    telemetry.leftEncoderCount += reverseCounts - sideLeadCounts;
    telemetry.rightEncoderCount += reverseCounts;
    ++telemetry.esp32UptimeMs;
    now += std::chrono::milliseconds(20);
    output = updateMission(
        mission, secondAlive, {}, telemetry, servoPose, now);
    require(!output.failed && output.leftPower == 0.0 &&
                output.rightPower == 0.0,
            "A ré deve zerar os motores ao alcançar 20 cm em ambas as rodas.");
    now += std::chrono::milliseconds(config::kRescueDistanceSettleMs);
    output = updateMission(
        mission, secondAlive, {}, telemetry, servoPose, now);
    require(output.status.phase == "rescue_required_dead_search" &&
                std::string(mission.ballTargetType()) == "black_ball",
            "Somente depois das duas pratas a missão deve procurar a vítima preta.");

    targetSequence = mission.ballTargetSequence(kRunSequence);
    ForwardBallSnapshot dead = lockedBlack(targetSequence, 30.0);
    collectVictim(mission, dead, telemetry, servoPose, now);
    output = completeDistanceStage(
        mission,
        dead,
        telemetry,
        servoPose,
        now,
        config::kRescuePostCollectionReverseDistanceCm);
    output = completeServoStage(mission, dead, telemetry, servoPose, now,
                                "rescue_deposit_zone_starting");
    require(output.status.phase == "rescue_deposit_zone_starting" &&
                servoPose.armDegrees == config::kServoInitialAngleDegrees,
            "A vítima preta deve seguir diretamente ao depósito.");

    reachDepositZone(mission, dead, false, telemetry, servoPose, now);
    completeServoStage(
        mission,
        dead,
        telemetry,
        servoPose,
        now,
        "rescue_deposit_completed");
    output = completeDistanceStage(
        mission,
        dead,
        telemetry,
        servoPose,
        now,
        config::kRescueFinalDepositReverseDistanceCm);
    require(output.status.phase == "rescue_final_verification" &&
                std::string(mission.ballTargetType()) == "any",
            "Depois das entregas obrigatórias, uma única volta deve procurar qualquer vítima extra.");

    const auto extra = lockedSilver(mission.ballTargetSequence(kRunSequence), 40.0);
    output = collectVictim(mission, extra, telemetry, servoPose, now);
    require(servoPose.armDegrees == config::kServoRoutineArmPickupDegrees,
            "A vítima extra também deve permanecer baixa após fechar a garra.");
    output = completeDistanceStage(mission, extra, telemetry, servoPose, now,
                                   config::kRescuePostCollectionReverseDistanceCm);
    require(output.status.phase == "rescue_collection_lifting",
            "A vítima extra deve concluir a ré antes de levantar.");
    output = completeServoStage(mission, extra, telemetry, servoPose, now,
                                "rescue_deposit_zone_starting");
    require(servoPose.armDegrees == config::kServoInitialAngleDegrees,
            "A vítima extra sem armazenamento deve usar elevação para depósito direto.");
    reachDepositZone(mission, extra, true, telemetry, servoPose, now);
    output = completeServoStage(mission, extra, telemetry, servoPose, now,
                                "rescue_deposit_completed");
    require(output.status.action.find("ré de 40 cm") != std::string::npos,
            "A vítima viva da verificação final não selecionou a ré de 40 cm.");
    output = completeDistanceStage(mission, extra, telemetry, servoPose, now,
                                   config::kRescueFinalDepositReverseDistanceCm);
    require(output.status.phase == "rescue_final_verification" &&
                std::string(mission.ballTargetType()) == "any",
            "Após a ré final verde, a verificação deve procurar qualquer vítima extra.");

    const std::uint64_t finalSequence =
        mission.ballTargetSequence(kRunSequence);
    double frameTimestamp = 100.0;
    output = updateMission(
        mission, emptyFrame(finalSequence, frameTimestamp++), {},
        telemetry, servoPose, now);
    require(output.status.phase == "rescue_search_continuous",
            "A verificação final deve iniciar o único giro de 360°.");
    const double yawStep = output.leftPower < 0.0 ? -90.0 : 90.0;

    for (int quarterTurn = 0; quarterTurn < 4; ++quarterTurn)
    {
        telemetry.yawZDeg = std::remainder(
            telemetry.yawZDeg + yawStep, 360.0);
        now += std::chrono::milliseconds(100);
        output = updateMission(
            mission, emptyFrame(finalSequence, frameTimestamp++), {},
            telemetry, servoPose, now);
    }
    require(output.completed &&
                output.status.phase == "rescue_room_completed" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "Uma volta completa deve parar os motores e liberar a busca da saída.");
}

void testBlocksWristWhenServoOutputIsLostAfterFirstCapture()
{
    RescueRoomMission mission;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    ServoPose servoPose{};
    Clock::time_point now{};
    const std::uint64_t targetSequence =
        mission.ballTargetSequence(kRunSequence);
    completeEntryAdvance(mission, telemetry, now, targetSequence);

    const ForwardBallSnapshot firstAlive =
        lockedSilver(targetSequence, 40.0);
    collectVictim(mission, firstAlive, telemetry, servoPose, now);
    RescueRoomOutput output = completeDistanceStage(
        mission,
        firstAlive,
        telemetry,
        servoPose,
        now,
        config::kRescuePostCollectionReverseDistanceCm);
    output = completeServoStage(mission, firstAlive, telemetry, servoPose, now,
                                "rescue_first_alive_storage");

    telemetry.armServoEnabled = false;
    now += std::chrono::milliseconds(20);
    output = updateMission(
        mission, firstAlive, {}, telemetry, servoPose, now);
    require(output.failed &&
                output.status.phase == "rescue_servo_output_lost" &&
                !output.servoPoseRequested,
            "A perda de PWM do braço deve bloquear o pulso antes do armazenamento.");
}

void testRescueMemorySurvivesStopAndRejectsChangesWhileRunning()
{
    RobotState state;
    require(state.setRescueTestDeliveries(1, 0),
            "O robô parado deve aceitar uma entrega simulada.");
    state.startAutonomous();
    state.recordRescueDeliveries(2, 1);
    require(!state.setRescueTestDeliveries(0, 0),
            "Não é seguro alterar a memória durante a missão.");
    state.stop();
    const RobotSnapshot stopped = state.snapshot();
    require(stopped.rescueDeliveredAliveVictims == 2 &&
                stopped.rescueDeliveredDeadVictims == 1 &&
                stopped.rescueTestMemoryActive,
            "STOP deve preservar as entregas na RAM e indicar a simulação.");
    require(!state.setRescueTestDeliveries(3, 0) &&
                state.setRescueTestDeliveries(0, 0) &&
                !state.snapshot().rescueTestMemoryActive,
            "O ajuste deve respeitar os limites e permitir limpar a simulação.");
    RobotState restartedState;
    require(restartedState.snapshot().rescueDeliveredAliveVictims == 0 &&
                restartedState.snapshot().rescueDeliveredDeadVictims == 0,
            "Um novo processo deve começar sem entregas na memória.");
}

}

int main()
{
    try
    {
        testEntryAdvanceDoesNotWaitForOrStopOnYolo();
        testSweepRemembersEntryCandidateAndDefaultsRight();
        testDelicateMotionUsesOneShortKick();
        testSweepTimeoutsReverseExpandAndStop();
        testSweepUsesActualHeadingAfterPartialTurn();
        testCandidateWaitDoesNotResetSweepTimeout();
        testRequiredVictimSearchBecomesContinuousAfterInitialSweep();
        testReverseFailureNeverLiftsVictim();
        testTransientEsp32LossPausesWithoutKillingMission();
        testPreparesOpenGripperBeforeApproach();
        testRunsRequiredVictimsInPriorityOrder();
        testBlocksWristWhenServoOutputIsLostAfterFirstCapture();
        testRescueMemorySurvivesStopAndRejectsChangesWhileRunning();
    }
    catch (const std::exception& error)
    {
        std::cerr << "rescue_room_mission_test: " << error.what() << '\n';
        return 1;
    }

    std::cout << "rescue_room_mission_test: OK\n";
    return 0;
}
