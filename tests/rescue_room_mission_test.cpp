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
                    output.servoPose.gripperDegrees ==
                        config::kServoRoutineGripperRetentionDegrees,
                "A ré após a coleta deve manter a garra energizada em 5°.");
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
    Clock::time_point& now)
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
    std::uint64_t targetSequence)
{
    RescueRoomOutput output = mission.update(
        emptyFrame(targetSequence, 1.0),
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

void testWaitsForYoloBeforeEntryAdvance()
{
    RescueRoomMission mission;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    RescueRoomOutput output = mission.update(
        {}, {}, telemetry, kRunSequence, 0, {}, Clock::time_point{});
    require(output.status.phase == "rescue_entry_waiting_yolo",
            "O avanço não pode começar antes do primeiro frame do YOLO.");
    require(output.leftPower == 0.0 && output.rightPower == 0.0,
            "Sem YOLO atual, a entrada deve manter os motores parados.");
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
    require(output.status.phase == "rescue_entry_waiting_yolo" &&
                output.leftPower == 0.0 && output.rightPower == 0.0,
            "A entrada não pode reutilizar um frame de outra geração do YOLO.");
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
    require(output.status.phase == "rescue_first_alive_storage",
            "A primeira vítima viva deve seguir para o armazenamento interno.");
    require(output.servoPoseRequested &&
                output.servoPose.armDegrees ==
                    config::kServoRoutineArmHomeDegrees &&
                output.servoPose.wristDegrees ==
                    config::kServoRoutineWristForwardDegrees &&
                output.servoPose.gripperDegrees ==
                    config::kServoRoutineGripperRetentionDegrees,
            "A primeira ré deve conservar exatamente a pose 15°/180°/5°.");

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

    targetSequence = mission.ballTargetSequence(kRunSequence);
    ForwardBallSnapshot secondAlive = lockedSilver(targetSequence, 20.0);
    collectVictim(mission, secondAlive, telemetry, servoPose, now);
    output = completeDistanceStage(
        mission,
        secondAlive,
        telemetry,
        servoPose,
        now,
        config::kRescuePostCollectionReverseDistanceCm);
    require(output.status.phase == "rescue_deposit_zone_starting" &&
                output.internalObjectStored,
            "A segunda prata deve iniciar o depósito mantendo a primeira guardada.");

    reachDepositZone(
        mission, secondAlive, true, telemetry, servoPose, now);
    output = completeServoStage(
        mission,
        secondAlive,
        telemetry,
        servoPose,
        now,
        "rescue_deposit_completed");
    require(!output.internalObjectStored,
            "O depósito verde deve liberar as duas pratas e limpar o armazenamento.");

    output = completeDistanceStage(
        mission,
        secondAlive,
        telemetry,
        servoPose,
        now,
        config::kRescuePostDepositReverseDistanceCm);
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
    require(output.status.phase == "rescue_deposit_zone_starting",
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
        config::kRescueFinalDeadDepositReverseDistanceCm);
    require(output.status.phase == "rescue_final_verification" &&
                std::string(mission.ballTargetType()) == "silver_ball",
            "Depois das entregas obrigatórias, a checagem extra deve recomeçar pelas vivas.");
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
    require(output.status.phase == "rescue_first_alive_storage",
            "O teste deve chegar ao armazenamento da primeira vítima.");

    telemetry.armServoEnabled = false;
    now += std::chrono::milliseconds(20);
    output = updateMission(
        mission, firstAlive, {}, telemetry, servoPose, now);
    require(output.failed &&
                output.status.phase == "rescue_servo_output_lost" &&
                !output.servoPoseRequested,
            "A perda de PWM do braço deve bloquear o pulso antes do armazenamento.");
}
}

int main()
{
    try
    {
        testWaitsForYoloBeforeEntryAdvance();
        testTransientEsp32LossPausesWithoutKillingMission();
        testPreparesOpenGripperBeforeApproach();
        testRunsRequiredVictimsInPriorityOrder();
        testBlocksWristWhenServoOutputIsLostAfterFirstCapture();
    }
    catch (const std::exception& error)
    {
        std::cerr << "rescue_room_mission_test: " << error.what() << '\n';
        return 1;
    }

    std::cout << "rescue_room_mission_test: OK\n";
    return 0;
}
