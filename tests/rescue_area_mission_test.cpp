#include "obr/config.h"
#include "obr/rescue_area_mission.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
constexpr std::uint64_t kTargetSequence = 11;
using Clock = std::chrono::steady_clock;

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
    telemetry.lastSensorAgeMs = 0;
    telemetry.motorSleepPinHigh = true;
    telemetry.leftEncoderRate = 0.0;
    telemetry.rightEncoderRate = 0.0;
    telemetry.yawZDeg = 0.0;
    telemetry.esp32UptimeMs = 1;
    return telemetry;
}

ForwardBallSnapshot emptyFrame(double timestamp)
{
    ForwardBallSnapshot snapshot;
    snapshot.sourceFresh = true;
    snapshot.targetSequence = kTargetSequence;
    snapshot.timestamp = timestamp;
    return snapshot;
}

ForwardBallSnapshot candidateFrame(double timestamp)
{
    ForwardBallSnapshot snapshot = emptyFrame(timestamp);
    snapshot.candidateVisible = true;
    return snapshot;
}

ForwardBallSnapshot lockedVictim(
    double txDegrees = 0.0,
    double distanceCm = config::kBallApproachStopDistanceCm,
    double timestamp = 1.0)
{
    ForwardBallSnapshot snapshot = candidateFrame(timestamp);
    snapshot.detected = true;
    snapshot.targetLocked = true;
    snapshot.type = "black_ball";
    snapshot.txDegrees = txDegrees;
    snapshot.distanceCm = distanceCm;
    snapshot.radiusPixels = 268.8;
    snapshot.visibleAreaPixels = 20000.0;
    return snapshot;
}

void acquireNearVictim(
    RescueAreaMission& mission,
    Esp32TelemetrySnapshot& telemetry,
    Clock::time_point start)
{
    RescueAreaOutput output = mission.update(
        lockedVictim(),
        telemetry,
        kTargetSequence,
        false,
        true,
        start);
    require(output.status.phase == "rescue_victim_acquired",
            "O alvo travado deve interromper a busca antes do alinhamento.");
    require(output.leftPower == 0.0 && output.rightPower == 0.0,
            "A aquisição deve parar antes de alinhar.");

    output = mission.update(
        lockedVictim(0.0, config::kBallApproachStopDistanceCm, 2.0),
        telemetry,
        kTargetSequence,
        false,
        true,
        start + std::chrono::milliseconds(20));
    require(output.status.phase == "victim_collection_preparing",
            "Uma vítima próxima deve preparar o avanço final.");
}

void testSearchWaitsForFreshFramesAndCandidates()
{
    RescueAreaMission mission;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    const Clock::time_point start{};

    RescueAreaOutput output = mission.update(
        ForwardBallSnapshot{},
        telemetry,
        kTargetSequence,
        false,
        true,
        start);
    require(output.status.phase == "rescue_waiting_camera",
            "A busca deve aguardar uma leitura fresca da CAM1.");
    require(output.leftPower == 0.0 && output.rightPower == 0.0,
            "A câmera indisponível nunca deve autorizar movimento.");

    output = mission.update(
        emptyFrame(1.0),
        telemetry,
        kTargetSequence,
        false,
        true,
        start + std::chrono::milliseconds(1));
    require(output.status.phase == "rescue_search_pivot",
            "Sem vítima, a busca deve iniciar um micro-pivô.");
    require(output.leftPower == config::kRescueSearchTurnPower &&
                output.rightPower == -config::kRescueSearchTurnPower,
            "A busca deve usar a potência calibrada.");

    output = mission.update(
        emptyFrame(1.0),
        telemetry,
        kTargetSequence,
        false,
        true,
        start + std::chrono::milliseconds(
                    1 + config::kRescueSearchPulseMs));
    require(output.status.phase == "rescue_search_settling",
            "Depois do pulso, a imagem deve estabilizar com PWM zero.");
    require(output.leftPower == 0.0 && output.rightPower == 0.0,
            "A estabilização da busca deve manter o robô parado.");

    output = mission.update(
        emptyFrame(1.0),
        telemetry,
        kTargetSequence,
        false,
        true,
        start + std::chrono::milliseconds(
                    1 + config::kRescueSearchPulseMs +
                    config::kRescueSearchSettlingMs));
    require(output.status.phase == "rescue_search_waiting_frame",
            "Outro pulso deve exigir uma inferência posterior ao movimento.");

    output = mission.update(
        candidateFrame(2.0),
        telemetry,
        kTargetSequence,
        false,
        true,
        start + std::chrono::milliseconds(300));
    require(output.status.phase == "rescue_confirming_victim",
            "Uma candidata ainda não travada deve manter os motores parados.");
    require(output.leftPower == 0.0 && output.rightPower == 0.0,
            "A confirmação temporal não pode mover o robô.");
}

void testCollectionAdvanceCompletesByBothEncoders()
{
    RescueAreaMission mission;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    telemetry.leftEncoderCount = 100;
    telemetry.rightEncoderCount = 200;
    const Clock::time_point start{};

    acquireNearVictim(mission, telemetry, start);

    RescueAreaOutput output = mission.update(
        lockedVictim(0.0, 5.0, 3.0),
        telemetry,
        kTargetSequence,
        false,
        true,
        start + std::chrono::milliseconds(40));
    require(output.status.phase == "victim_collection_advancing",
            "Encoders parados devem liberar o avanço final.");
    require(output.leftPower > 0.0 && output.rightPower > 0.0,
            "A coleta deve avançar com potência positiva.");

    const long long requiredCounts = static_cast<long long>(std::ceil(
        config::kVictimCollectionAdvanceDistanceCm *
        config::kEncoderCountsPerCentimeter));
    telemetry.leftEncoderCount += requiredCounts;
    telemetry.rightEncoderCount += requiredCounts;
    ++telemetry.esp32UptimeMs;
    output = mission.update(
        lockedVictim(0.0, 5.0, 4.0),
        telemetry,
        kTargetSequence,
        false,
        true,
        start + std::chrono::milliseconds(60));
    require(output.status.phase == "victim_collection_settling",
            "Os dois encoders devem completar toda a distância de coleta.");
    require(output.leftPower == 0.0 && output.rightPower == 0.0,
            "A distância concluída deve zerar o PWM.");

    output = mission.update(
        lockedVictim(0.0, 5.0, 5.0),
        telemetry,
        kTargetSequence,
        false,
        true,
        start + std::chrono::milliseconds(
                    60 + config::kVictimCollectionSettleMs));
    require(output.completed && output.status.phase == "ball_reached",
            "O modo isolado deve concluir depois da estabilização da coleta.");
}

void testCollectionFailsWhenEncoderBecomesStale()
{
    RescueAreaMission mission;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    const Clock::time_point start{};
    acquireNearVictim(mission, telemetry, start);

    mission.update(
        lockedVictim(0.0, 5.0, 3.0),
        telemetry,
        kTargetSequence,
        false,
        true,
        start + std::chrono::milliseconds(40));

    telemetry.lastSensorAgeMs =
        config::kDriveDistanceEncoderFreshnessMs + 1;
    const RescueAreaOutput output = mission.update(
        lockedVictim(0.0, 5.0, 4.0),
        telemetry,
        kTargetSequence,
        false,
        true,
        start + std::chrono::milliseconds(60));
    require(output.failed &&
                output.status.phase == "victim_collection_encoder_lost",
            "A perda de encoders deve interromper o avanço final.");
    require(output.leftPower == 0.0 && output.rightPower == 0.0,
            "A falha dos encoders deve manter PWM zero.");
}

void testCollectionTreatsStallAndSideMismatchAsContact()
{
    const Clock::time_point start{};

    {
        RescueAreaMission mission;
        Esp32TelemetrySnapshot telemetry = readyTelemetry();
        acquireNearVictim(mission, telemetry, start);
        mission.update(
            lockedVictim(0.0, 5.0, 3.0),
            telemetry,
            kTargetSequence,
            false,
            true,
            start + std::chrono::milliseconds(40));

        RescueAreaOutput output = mission.update(
            lockedVictim(0.0, 5.0, 4.0),
            telemetry,
            kTargetSequence,
            false,
            true,
            start + std::chrono::milliseconds(
                        40 + config::kVictimCollectionStallTimeoutMs));
        require(!output.failed &&
                    output.status.phase == "victim_collection_settling",
                "Roda parada no avanço final deve confirmar contato.");
        require(output.leftPower == 0.0 && output.rightPower == 0.0,
                "O contato deve zerar o PWM imediatamente.");

        output = mission.update(
            lockedVictim(0.0, 5.0, 5.0),
            telemetry,
            kTargetSequence,
            false,
            true,
            start + std::chrono::milliseconds(
                        40 + config::kVictimCollectionStallTimeoutMs +
                        config::kVictimCollectionSettleMs));
        require(output.completed && output.status.phase == "ball_reached",
                "Contato por roda parada deve concluir a missão isolada.");
    }

    {
        RescueAreaMission mission;
        Esp32TelemetrySnapshot telemetry = readyTelemetry();
        acquireNearVictim(mission, telemetry, start);
        mission.update(
            lockedVictim(0.0, 5.0, 3.0),
            telemetry,
            kTargetSequence,
            false,
            true,
            start + std::chrono::milliseconds(40));

        RescueAreaOutput output;
        for (int sample = 0;
             sample < config::kVictimCollectionDifferenceConfirmationSamples;
             ++sample)
        {
            telemetry.leftEncoderCount += static_cast<long long>(
                std::ceil(
                    config::kVictimCollectionMaximumSideDifferenceCm *
                    config::kEncoderCountsPerCentimeter)) +
                1;
            telemetry.rightEncoderCount += 1;
            ++telemetry.esp32UptimeMs;
            output = mission.update(
                lockedVictim(0.0, 5.0, 4.0 + sample),
                telemetry,
                kTargetSequence,
                false,
                true,
                start + std::chrono::milliseconds(60 + sample * 20));
        }
        require(!output.failed &&
                    output.status.phase == "victim_collection_settling",
                "Divergência persistente deve confirmar resistência física.");
        require(output.leftPower == 0.0 && output.rightPower == 0.0,
                "A resistência física deve zerar o PWM imediatamente.");

        output = mission.update(
            lockedVictim(0.0, 5.0, 10.0),
            telemetry,
            kTargetSequence,
            false,
            true,
            start + std::chrono::milliseconds(
                        60 +
                        (config::kVictimCollectionDifferenceConfirmationSamples -
                         1) * 20 +
                        config::kVictimCollectionSettleMs));
        require(output.completed && output.status.phase == "ball_reached",
                "Contato por divergência deve concluir a missão isolada.");
    }
}

void testCollectionPreparationAndAdvanceTimeouts()
{
    const Clock::time_point start{};

    {
        RescueAreaMission mission;
        Esp32TelemetrySnapshot telemetry = readyTelemetry();
        acquireNearVictim(mission, telemetry, start);
        telemetry.leftEncoderRate =
            config::kBallAlignmentStationaryRateCountsPerSecond + 1.0;
        telemetry.rightEncoderRate = telemetry.leftEncoderRate;

        const RescueAreaOutput output = mission.update(
            lockedVictim(0.0, 5.0, 3.0),
            telemetry,
            kTargetSequence,
            false,
            true,
            start + std::chrono::milliseconds(
                        20 + config::kVictimCollectionPreparationTimeoutMs));
        require(
            output.failed &&
                output.status.phase ==
                    "victim_collection_preparation_timeout",
            "Rodas que não estabilizam devem cancelar a preparação da coleta.");
        require(output.leftPower == 0.0 && output.rightPower == 0.0,
                "O timeout de preparação deve manter PWM zero.");
    }

    {
        RescueAreaMission mission;
        Esp32TelemetrySnapshot telemetry = readyTelemetry();
        acquireNearVictim(mission, telemetry, start);
        mission.update(
            lockedVictim(0.0, 5.0, 3.0),
            telemetry,
            kTargetSequence,
            false,
            true,
            start + std::chrono::milliseconds(40));

        const int progressCounts = static_cast<int>(
            config::kDriveDistanceMinimumProgressCounts);
        for (int sample = 1; sample <= 3; ++sample)
        {
            telemetry.leftEncoderCount += progressCounts;
            telemetry.rightEncoderCount += progressCounts;
            ++telemetry.esp32UptimeMs;
            mission.update(
                lockedVictim(0.0, 5.0, 3.0 + sample),
                telemetry,
                kTargetSequence,
                false,
                true,
                start + std::chrono::milliseconds(40 + sample * 900));
        }

        const RescueAreaOutput output = mission.update(
            lockedVictim(0.0, 5.0, 7.0),
            telemetry,
            kTargetSequence,
            false,
            true,
            start + std::chrono::milliseconds(
                        40 + config::kVictimCollectionTimeoutMs));
        require(output.failed &&
                    output.status.phase == "victim_collection_timeout",
                "Progresso insuficiente deve respeitar o timeout total da coleta.");
        require(output.leftPower == 0.0 && output.rightPower == 0.0,
                "O timeout total da coleta deve zerar o PWM.");
    }
}
}

int main()
{
    try
    {
        testSearchWaitsForFreshFramesAndCandidates();
        testCollectionAdvanceCompletesByBothEncoders();
        testCollectionFailsWhenEncoderBecomesStale();
        testCollectionTreatsStallAndSideMismatchAsContact();
        testCollectionPreparationAndAdvanceTimeouts();
    }
    catch (const std::exception& error)
    {
        std::cerr << "rescue_area_mission_test: " << error.what() << '\n';
        return 1;
    }

    std::cout << "rescue_area_mission_test: OK\n";
    return 0;
}
