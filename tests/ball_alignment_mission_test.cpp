#include "obr/ball_alignment_mission.h"
#include "obr/config.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
constexpr std::uint64_t kTargetSequence = 7;
using Clock = std::chrono::steady_clock;

void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

ForwardBallSnapshot ball(
    double txDegrees,
    double distanceCm = 40.0,
    double timestamp = 1.0)
{
    ForwardBallSnapshot snapshot;
    snapshot.sourceFresh = true;
    snapshot.detected = true;
    snapshot.candidateVisible = true;
    snapshot.type = "black_ball";
    snapshot.txDegrees = txDegrees;
    snapshot.distanceCm = distanceCm;
    snapshot.radiusPixels = 71.7;
    snapshot.visibleAreaPixels = 15000.0;
    snapshot.targetSequence = kTargetSequence;
    snapshot.targetLocked = true;
    snapshot.timestamp = timestamp;
    return snapshot;
}

ForwardBallSnapshot missingBall(double timestamp = 2.0)
{
    ForwardBallSnapshot snapshot;
    snapshot.sourceFresh = true;
    snapshot.targetSequence = kTargetSequence;
    snapshot.targetLocked = true;
    snapshot.timestamp = timestamp;
    return snapshot;
}

Esp32TelemetrySnapshot stoppedTelemetry(double yawDegrees = 0.0)
{
    Esp32TelemetrySnapshot telemetry;
    telemetry.sensorFresh = true;
    telemetry.mpuOk = true;
    telemetry.lastSensorAgeMs = 0;
    telemetry.leftEncoderRate = 0.0;
    telemetry.rightEncoderRate = 0.0;
    telemetry.yawZDeg = yawDegrees;
    return telemetry;
}

Esp32TelemetrySnapshot movingPivotTelemetry(
    double yawDegrees,
    double direction)
{
    Esp32TelemetrySnapshot telemetry = stoppedTelemetry(yawDegrees);
    telemetry.appliedLeftPower =
        direction * config::kBallAlignmentStartPower;
    telemetry.appliedRightPower =
        -direction * config::kBallAlignmentStartPower;
    telemetry.leftEncoderRate =
        config::kBallAlignmentMovementMinimumRateCountsPerSecond + 1.0;
    telemetry.rightEncoderRate = -telemetry.leftEncoderRate;
    return telemetry;
}

void testRejectsOldTargetSequence()
{
    BallAlignmentMission mission;
    ForwardBallSnapshot oldTarget = ball(12.0);
    oldTarget.targetSequence = kTargetSequence - 1;

    const BallAlignmentOutput output = mission.update(
        oldTarget,
        stoppedTelemetry(),
        kTargetSequence,
        Clock::time_point{});

    require(output.leftPower == 0.0 && output.rightPower == 0.0,
            "Uma sequência antiga nunca pode mover o robô.");
    require(output.status.phase == "ball_alignment_waiting_target",
            "O alinhamento deve aguardar o alvo da execução atual.");
}

void testCoarsePulseStopsAtImuLimitAndUsesFineCorrection()
{
    BallAlignmentMission mission;
    const Clock::time_point start{};

    BallAlignmentOutput output = mission.update(
        ball(12.0),
        stoppedTelemetry(0.0),
        kTargetSequence,
        start);
    require(output.status.phase == "ball_alignment_correction_pulse",
            "Um erro grande deve iniciar um micro-pivô grosso.");
    require(
        std::abs(output.leftPower - config::kBallAlignmentStartPower) <
                0.0001 &&
            std::abs(output.rightPower +
                     config::kBallAlignmentStartPower) < 0.0001,
        "O micro-pivô grosso deve começar com a potência de partida.");

    Esp32TelemetrySnapshot applied = stoppedTelemetry(0.0);
    applied.appliedLeftPower = config::kBallAlignmentStartPower;
    applied.appliedRightPower = -config::kBallAlignmentStartPower;
    output = mission.update(
        ball(12.0, 40.0, 1.1),
        applied,
        kTargetSequence,
        start + std::chrono::milliseconds(20));
    require(
        std::abs(output.leftPower - config::kBallAlignmentStartPower) <
            0.0001,
        "Confirmar somente o PWM não deve iniciar o tempo útil do pulso.");

    output = mission.update(
        ball(12.0, 40.0, 1.2),
        movingPivotTelemetry(0.0, 1.0),
        kTargetSequence,
        start + std::chrono::milliseconds(200));
    require(
        std::abs(output.leftPower - config::kBallAlignmentCoarsePulsePower) <
                0.0001 &&
            std::abs(output.rightPower +
                     config::kBallAlignmentCoarsePulsePower) < 0.0001,
        "Os encoders devem iniciar o tempo útil na potência de pulso.");

    output = mission.update(
        ball(11.0, 40.0, 1.3),
        movingPivotTelemetry(2.0, 1.0),
        kTargetSequence,
        start + std::chrono::milliseconds(201));
    require(output.leftPower == 0.0 && output.rightPower == 0.0,
            "O limite da IMU deve cortar o pulso antes de ultrapassar o alvo.");
    require(output.status.phase == "ball_alignment_braking",
            "O corte por yaw deve iniciar a estabilização.");

    output = mission.update(
        ball(9.0, 40.0, 2.0),
        stoppedTelemetry(2.0),
        kTargetSequence,
        start + std::chrono::milliseconds(
                    201 + config::kBallAlignmentCrossingBrakeMs));
    require(output.status.phase == "ball_alignment_fine_correction",
            "Um erro residual deve usar a correção fina.");
    require(
        std::abs(output.leftPower - config::kBallAlignmentStartPower) <
                0.0001,
        "A correção fina também deve vencer a inércia antes do pulso útil.");
}

void testMotionConfirmationTimeoutFailsSafe()
{
    BallAlignmentMission mission;
    const Clock::time_point start{};

    mission.update(
        ball(15.0),
        stoppedTelemetry(),
        kTargetSequence,
        start);
    const BallAlignmentOutput output = mission.update(
        ball(15.0, 40.0, 1.1),
        stoppedTelemetry(),
        kTargetSequence,
        start + std::chrono::milliseconds(
                    config::kBallAlignmentPulseStartTimeoutMs));

    require(output.finished,
            "A falta de confirmação da ESP32 deve encerrar o alinhamento.");
    require(output.leftPower == 0.0 && output.rightPower == 0.0,
            "O timeout do micro-pivô deve manter PWM zero.");
    require(output.status.phase == "ball_alignment_motion_timeout",
            "A causa do timeout deve permanecer disponível na telemetria.");
}

void testApproachUsesVisionImuAndStopsAtDistance()
{
    BallAlignmentMission mission;
    const Clock::time_point start{};

    BallAlignmentOutput output = mission.update(
        ball(2.0, 40.0, 1.0),
        stoppedTelemetry(10.0),
        kTargetSequence,
        start);
    require(output.status.phase == "ball_alignment_braking",
            "A faixa central deve ser verificada com o robô parado.");

    for (int frame = 0; frame < config::kBallAlignmentStableFrames; ++frame)
    {
        output = mission.update(
            ball(2.0, 40.0, 2.0 + frame),
            stoppedTelemetry(10.0),
            kTargetSequence,
            start + std::chrono::milliseconds(
                        config::kBallAlignmentCrossingBrakeMs + frame * 20));
    }
    require(output.status.phase == "ball_approaching",
            "Frames alinhados e parados devem iniciar a aproximação.");
    require(output.leftPower == config::kBallApproachBasePower &&
                output.rightPower == config::kBallApproachBasePower,
            "A aproximação deve começar em linha reta.");

    output = mission.update(
        ball(6.0, 14.0, 10.0),
        stoppedTelemetry(10.0),
        kTargetSequence,
        start + std::chrono::milliseconds(200));
    require(output.leftPower > config::kBallApproachBasePower &&
                output.rightPower < config::kBallApproachBasePower,
            "A visão deve atualizar o heading desejado durante a aproximação.");

    output = mission.update(
        ball(6.0, 13.0, 10.0),
        stoppedTelemetry(12.4),
        kTargetSequence,
        start + std::chrono::milliseconds(220));
    require(
        std::abs(output.leftPower - config::kBallApproachBasePower) < 0.0001 &&
            std::abs(output.rightPower - config::kBallApproachBasePower) <
                0.0001,
        "A IMU deve encerrar a correção sem exigir outro frame do YOLO.");

    Esp32TelemetrySnapshot invalidImu = stoppedTelemetry(12.4);
    invalidImu.mpuOk = false;
    output = mission.update(
        ball(1.0, 10.0, 11.0),
        invalidImu,
        kTargetSequence,
        start + std::chrono::milliseconds(240));
    require(output.leftPower == 0.0 && output.rightPower == 0.0,
            "Uma IMU inválida deve parar a aproximação.");
    require(output.status.phase == "ball_approach_waiting_imu",
            "A espera pela IMU deve aparecer na telemetria.");

    output = mission.update(
        ball(0.5, config::kBallApproachStopDistanceCm, 12.0),
        stoppedTelemetry(12.4),
        kTargetSequence,
        start + std::chrono::milliseconds(260));
    require(output.finished && output.status.phase == "ball_reached",
            "A distância calibrada deve concluir a aproximação.");
    require(output.leftPower == 0.0 && output.rightPower == 0.0,
            "A chegada à vítima deve zerar os motores.");
}

void testTargetLossStopsAndTimesOut()
{
    BallAlignmentMission mission;
    const Clock::time_point start{};

    mission.update(
        ball(10.0),
        stoppedTelemetry(),
        kTargetSequence,
        start);
    BallAlignmentOutput output = mission.update(
        missingBall(),
        stoppedTelemetry(),
        kTargetSequence,
        start + std::chrono::milliseconds(10));
    require(output.leftPower == 0.0 && output.rightPower == 0.0,
            "A perda do alvo deve parar no mesmo ciclo.");
    require(output.status.phase == "ball_alignment_target_lost",
            "A perda temporária deve preservar o target lock.");

    output = mission.update(
        missingBall(3.0),
        stoppedTelemetry(),
        kTargetSequence,
        start + std::chrono::milliseconds(
                    10 + config::kBallAlignmentTargetLossTimeoutMs));
    require(output.finished,
            "A perda prolongada deve encerrar a tentativa atual.");
    require(output.status.phase == "ball_alignment_target_lost_timeout",
            "O timeout do alvo deve ser identificado.");
}

void testAlignmentOnlyStopsBeforeApproach()
{
    BallAlignmentMission mission;
    const Clock::time_point start{};
    BallAlignmentOutput output = mission.update(
        ball(0.0, 40.0, 1.0),
        stoppedTelemetry(),
        kTargetSequence,
        start,
        true);
    require(output.leftPower == 0.0 && output.rightPower == 0.0,
            "O alinhamento inicial não pode liberar avanço à vítima.");

    for (int frame = 2; frame <= 4; ++frame)
    {
        output = mission.update(
            ball(0.0, 40.0, static_cast<double>(frame)),
            stoppedTelemetry(),
            kTargetSequence,
            start + std::chrono::milliseconds(100 + frame * 20),
            true);
    }
    require(output.finished && output.status.phase == "ball_aligned",
            "Três frames centrais devem concluir somente o alinhamento inicial.");
    require(output.leftPower == 0.0 && output.rightPower == 0.0,
            "A conclusão do alinhamento deve manter a aproximação bloqueada.");
}
}

int main()
{
    try
    {
        testRejectsOldTargetSequence();
        testCoarsePulseStopsAtImuLimitAndUsesFineCorrection();
        testMotionConfirmationTimeoutFailsSafe();
        testApproachUsesVisionImuAndStopsAtDistance();
        testTargetLossStopsAndTimesOut();
        testAlignmentOnlyStopsBeforeApproach();
    }
    catch (const std::exception& error)
    {
        std::cerr << "ball_alignment_mission_test: " << error.what() << '\n';
        return 1;
    }

    std::cout << "ball_alignment_mission_test: OK\n";
    return 0;
}
