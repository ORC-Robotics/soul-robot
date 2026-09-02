#include "obr/ball_alignment_mission.h"
#include "obr/config.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>

namespace
{
constexpr std::uint64_t kTargetSequence = 7;

bool expect(bool condition, const std::string& message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        return false;
    }
    return true;
}

ForwardBallSnapshot ball(double txDegrees, double timestamp = 1.0)
{
    ForwardBallSnapshot snapshot;
    snapshot.sourceFresh = true;
    snapshot.detected = true;
    snapshot.type = "black_ball";
    snapshot.txDegrees = txDegrees;
    snapshot.distanceCm = 40.0;
    snapshot.radiusPixels = 71.7;
    snapshot.visibleAreaPixels = 15000.0;
    snapshot.targetSequence = kTargetSequence;
    snapshot.targetLocked = true;
    snapshot.timestamp = timestamp;
    return snapshot;
}

ForwardBallSnapshot missingBall()
{
    ForwardBallSnapshot snapshot;
    snapshot.sourceFresh = true;
    snapshot.targetSequence = kTargetSequence;
    snapshot.targetLocked = true;
    snapshot.timestamp = 2.0;
    return snapshot;
}

Esp32TelemetrySnapshot stoppedTelemetry()
{
    Esp32TelemetrySnapshot telemetry;
    telemetry.sensorFresh = true;
    telemetry.lastSensorAgeMs = 0;
    return telemetry;
}

Esp32TelemetrySnapshot movingRightTelemetry()
{
    Esp32TelemetrySnapshot telemetry = stoppedTelemetry();
    telemetry.leftEncoderRate = 45.0;
    telemetry.rightEncoderRate = -46.0;
    telemetry.appliedLeftPower = 0.70;
    telemetry.appliedRightPower = -0.70;
    return telemetry;
}
}

int main()
{
    bool passed = true;
    const auto start = std::chrono::steady_clock::time_point{};

    BallAlignmentMission mission;
    BallAlignmentOutput output = mission.update(
        ball(12.0), stoppedTelemetry(), kTargetSequence, start);
    passed &= expect(
        std::abs(output.leftPower - config::kBallAlignmentStartPower) < 0.0001 &&
            std::abs(output.rightPower + config::kBallAlignmentStartPower) <
                0.0001,
        "tx positivo deve iniciar o pivot para a direita com potência de partida");
    passed &= expect(output.status.phase == "ball_alignment_turning",
                     "o giro deve ser controlado pelo tx");

    output = mission.update(
        ball(6.0, 1.1),
        movingRightTelemetry(),
        kTargetSequence,
        start + std::chrono::milliseconds(20));
    passed &= expect(
        output.leftPower >= config::kBallAlignmentMinimumRunPower &&
            output.leftPower <= config::kBallAlignmentMaximumRunPower,
        "encoders em movimento devem liberar a faixa proporcional");

    output = mission.update(
        ball(-2.0, 1.2),
        movingRightTelemetry(),
        kTargetSequence,
        start + std::chrono::milliseconds(40));
    passed &= expect(output.leftPower == 0.0 && output.rightPower == 0.0,
                     "cruzar o centro deve zerar o PWM imediatamente");
    passed &= expect(output.status.phase == "ball_alignment_braking",
                     "o cruzamento deve iniciar a frenagem");

    output = mission.update(
        ball(-2.0, 1.3),
        stoppedTelemetry(),
        kTargetSequence,
        start + std::chrono::milliseconds(
                    40 + config::kBallAlignmentCrossingBrakeMs - 1));
    passed &= expect(output.leftPower == 0.0 && output.rightPower == 0.0,
                     "a frenagem deve durar 160 ms completos");

    output = mission.update(
        ball(-2.0, 1.4),
        stoppedTelemetry(),
        kTargetSequence,
        start + std::chrono::milliseconds(
                    40 + config::kBallAlignmentCrossingBrakeMs));
    passed &= expect(output.leftPower < 0.0 && output.rightPower > 0.0,
                     "após a frenagem, o tx deve permitir a correção oposta");
    output = mission.update(
        ball(-0.8, 1.5),
        stoppedTelemetry(),
        kTargetSequence,
        start + std::chrono::milliseconds(
                    40 + config::kBallAlignmentCrossingBrakeMs +
                    config::kBallAlignmentFineCorrectionPulseMs));
    passed &= expect(output.leftPower == 0.0 && output.rightPower == 0.0,
                     "o pulso fino deve terminar com uma nova frenagem");

    mission.reset();
    output = mission.update(
        ball(1.0), movingRightTelemetry(), kTargetSequence, start);
    passed &= expect(output.leftPower == 0.0 && output.rightPower == 0.0,
                     "entrar na tolerância deve zerar os motores imediatamente");
    passed &= expect(output.status.phase == "ball_alignment_braking",
                     "o primeiro frame central deve iniciar a verificação parada");
    output = mission.update(
        ball(0.8, 1.1),
        movingRightTelemetry(),
        kTargetSequence,
        start + std::chrono::milliseconds(
                    config::kBallAlignmentCrossingBrakeMs + 20));
    passed &= expect(output.status.phase == "ball_alignment_braking",
                     "a inércia confirmada pelos encoders deve impedir a conclusão");
    output = mission.update(
        ball(0.8, 1.2),
        stoppedTelemetry(),
        kTargetSequence,
        start + std::chrono::milliseconds(
                    config::kBallAlignmentCrossingBrakeMs + 40));
    passed &= expect(output.status.phase == "ball_alignment_verifying",
                     "o primeiro frame parado deve apenas iniciar a confirmação");
    output = mission.update(
        ball(0.7, 1.3),
        stoppedTelemetry(),
        kTargetSequence,
        start + std::chrono::milliseconds(
                    config::kBallAlignmentCrossingBrakeMs + 100));
    passed &= expect(output.status.phase == "ball_alignment_verifying",
                     "dois frames centrais ainda não devem concluir");
    output = mission.update(
        ball(0.6, 1.4),
        stoppedTelemetry(),
        kTargetSequence,
        start + std::chrono::milliseconds(
                    config::kBallAlignmentCrossingBrakeMs + 160));
    passed &= expect(
        output.leftPower == config::kBallApproachBasePower &&
            output.rightPower == config::kBallApproachBasePower,
        "três frames centrais e parados devem iniciar o avanço até a bola");
    passed &= expect(output.status.phase == "ball_approaching",
                     "o alinhamento confirmado deve iniciar a aproximação");
    ForwardBallSnapshot correctingBall = ball(6.0, 1.5);
    correctingBall.distanceCm = 14.0;
    output = mission.update(
        correctingBall,
        stoppedTelemetry(),
        kTargetSequence,
        start + std::chrono::milliseconds(
                    config::kBallAlignmentCrossingBrakeMs + 180));
    passed &= expect(
        output.leftPower > config::kBallApproachBasePower &&
            output.rightPower < config::kBallApproachBasePower &&
            output.rightPower > 0.0,
        "tx positivo deve corrigir para a direita enquanto o robô avança");
    passed &= expect(output.status.phase == "ball_approaching",
                     "a correção angular não deve pausar a aproximação");

    ForwardBallSnapshot closeBall = ball(0.6, 1.6);
    closeBall.distanceCm = config::kBallApproachStopDistanceCm;
    output = mission.update(
        closeBall,
        stoppedTelemetry(),
        kTargetSequence,
        start + std::chrono::milliseconds(
                    config::kBallAlignmentCrossingBrakeMs + 200));
    passed &= expect(output.leftPower == 0.0 && output.rightPower == 0.0,
                     "a distância segura deve parar a aproximação");
    passed &= expect(output.status.phase == "ball_reached",
                     "a distância segura deve concluir a aproximação");
    output = mission.update(
        ball(20.0, 2.0), stoppedTelemetry(), kTargetSequence, start);
    passed &= expect(output.leftPower == 0.0 && output.rightPower == 0.0,
                     "uma missão concluída deve permanecer parada após alcançar a bola");

    mission.reset();
    ForwardBallSnapshot oldSequence = ball(10.0);
    oldSequence.targetSequence = kTargetSequence - 1;
    output = mission.update(
        oldSequence, stoppedTelemetry(), kTargetSequence, start);
    passed &= expect(output.leftPower == 0.0 && output.rightPower == 0.0,
                     "uma sequência antiga nunca pode mover o robô");
    passed &= expect(output.status.phase == "ball_alignment_waiting_target",
                     "o controle deve aguardar o alvo da execução atual");

    mission.reset();
    output = mission.update(
        ball(8.0), stoppedTelemetry(), kTargetSequence, start);
    output = mission.update(
        missingBall(),
        stoppedTelemetry(),
        kTargetSequence,
        start + std::chrono::milliseconds(20));
    passed &= expect(output.leftPower == 0.0 && output.rightPower == 0.0,
                     "perder o alvo deve parar no mesmo ciclo");
    passed &= expect(output.status.phase == "ball_alignment_target_lost",
                     "a perda temporária deve procurar somente o alvo travado");
    output = mission.update(
        ball(5.0, 2.1),
        stoppedTelemetry(),
        kTargetSequence,
        start + std::chrono::milliseconds(500));
    passed &= expect(output.leftPower > 0.0 && output.rightPower < 0.0,
                     "o mesmo alvo pode retomar antes de um segundo");

    mission.reset();
    output = mission.update(
        ball(8.0), stoppedTelemetry(), kTargetSequence, start);
    output = mission.update(
        missingBall(),
        stoppedTelemetry(),
        kTargetSequence,
        start + std::chrono::milliseconds(10));
    output = mission.update(
        missingBall(),
        stoppedTelemetry(),
        kTargetSequence,
        start + std::chrono::milliseconds(
                    10 + config::kBallAlignmentTargetLossTimeoutMs));
    passed &= expect(output.leftPower == 0.0 && output.rightPower == 0.0,
                     "perda superior ao limite deve manter o robô parado");
    passed &= expect(
        output.status.phase == "ball_alignment_target_lost_timeout",
        "perda por um segundo deve encerrar a execução com falha");
    output = mission.update(
        ball(5.0, 3.0), stoppedTelemetry(), kTargetSequence, start);
    passed &= expect(output.leftPower == 0.0 && output.rightPower == 0.0,
                     "a falha deve persistir até uma nova execução");

    mission.reset();
    ForwardBallSnapshot stale = ball(15.0);
    stale.sourceFresh = false;
    output = mission.update(
        stale, stoppedTelemetry(), kTargetSequence, start);
    passed &= expect(output.leftPower == 0.0 && output.rightPower == 0.0,
                     "uma câmera expirada deve manter os motores parados");
    passed &= expect(output.status.phase == "ball_alignment_camera_stale",
                     "a câmera expirada deve aparecer no diagnóstico");

    if (!passed)
    {
        return 1;
    }
    std::cout << "ball_alignment_mission_test: OK\n";
    return 0;
}
