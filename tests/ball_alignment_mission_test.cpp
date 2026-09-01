#include "obr/ball_alignment_mission.h"
#include "obr/config.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

namespace
{
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
    snapshot.type = "silver_ball";
    snapshot.txDegrees = txDegrees;
    snapshot.distanceCm = 40.0;
    snapshot.radiusPixels = 71.7;
    snapshot.timestamp = timestamp;
    return snapshot;
}

Esp32TelemetrySnapshot stoppedTelemetry()
{
    Esp32TelemetrySnapshot telemetry;
    telemetry.sensorFresh = true;
    return telemetry;
}

Esp32TelemetrySnapshot movingRightTelemetry()
{
    Esp32TelemetrySnapshot telemetry;
    telemetry.sensorFresh = true;
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
    const Esp32TelemetrySnapshot stopped = stoppedTelemetry();

    BallAlignmentMission mission;
    BallAlignmentOutput output = mission.update(ball(12.0), stopped, start);
    passed &= expect(
        std::abs(output.leftPower - 0.70) < 0.0001 &&
            std::abs(output.rightPower + 0.70) < 0.0001,
        "o giro deve usar exatamente 0,70 de potência");
    passed &= expect(output.leftPower > 0.0 && output.rightPower < 0.0,
                     "tx positivo deve girar para a direita");
    passed &= expect(output.status.phase == "ball_alignment_turning",
                     "o primeiro comando deve iniciar o giro");

    output = mission.update(
        ball(12.0),
        movingRightTelemetry(),
        start + std::chrono::milliseconds(20));
    passed &= expect(
        std::abs(output.leftPower - 0.68) < 0.0001 &&
            std::abs(output.rightPower + 0.68) < 0.0001,
        "o giro deve cair para 0,68 após os encoders confirmarem movimento");

    output = mission.update(
        ball(7.0),
        movingRightTelemetry(),
        start + std::chrono::milliseconds(30));
    const double middlePower =
        (config::kBallAlignmentMinimumRunPower +
         config::kBallAlignmentMaximumRunPower) * 0.5;
    passed &= expect(
        std::abs(output.leftPower - middlePower) < 0.0001 &&
            std::abs(output.rightPower + middlePower) < 0.0001,
        "o erro intermediário deve produzir potência proporcional intermediária");

    output = mission.update(
        ball(2.1),
        movingRightTelemetry(),
        start + std::chrono::milliseconds(35));
    passed &= expect(
        output.leftPower >= config::kBallAlignmentMinimumRunPower &&
            output.leftPower < middlePower,
        "a potência deve se aproximar do piso quando o tx chega à zona morta");

    output = mission.update(
        ball(1.5), stopped, start + std::chrono::milliseconds(40));
    passed &= expect(output.leftPower == 0.0 && output.rightPower == 0.0,
                     "entrar em mais ou menos 2 graus deve cortar o giro");

    mission.reset();
    output = mission.update(ball(12.0), stopped, start);

    output = mission.update(
        ball(12.0),
        stopped,
        start + std::chrono::milliseconds(500));
    passed &= expect(
        std::abs(output.leftPower - 0.70) < 0.0001 &&
            std::abs(output.rightPower + 0.70) < 0.0001,
        "o giro deve continuar enquanto o tx estiver fora da zona morta");
    passed &= expect(output.status.phase == "ball_alignment_turning",
                     "não deve aguardar outro tx antes de alinhar");

    output = mission.update(
        ball(-4.0, 2.0),
        stopped,
        start + std::chrono::milliseconds(520));
    passed &= expect(output.leftPower == 0.0 && output.rightPower == 0.0,
                     "cruzar o centro deve cortar o PWM imediatamente");
    passed &= expect(output.status.phase == "ball_alignment_braking",
                     "a troca de sinal deve iniciar a frenagem");

    output = mission.update(
        ball(-4.0, 2.0),
        stopped,
        start + std::chrono::milliseconds(
                    520 + config::kBallAlignmentCrossingBrakeMs + 1));
    passed &= expect(output.leftPower == 0.0 && output.rightPower == 0.0,
                     "a frenagem deve exigir uma medição visual nova");

    output = mission.update(
        ball(-3.0, 3.0),
        stopped,
        start + std::chrono::milliseconds(
                    520 + config::kBallAlignmentCrossingBrakeMs + 21));
    passed &= expect(output.leftPower < 0.0 && output.rightPower > 0.0,
                     "uma medição nova pode iniciar a correção oposta");

    mission.reset();
    output = mission.update(ball(-8.0), stopped, start);
    passed &= expect(output.leftPower < 0.0 && output.rightPower > 0.0,
                     "tx negativo deve girar para a esquerda");

    mission.reset();
    output = mission.update(ball(2.0), stopped, start);
    passed &= expect(output.leftPower == 0.0 && output.rightPower == 0.0,
                     "a zona morta deve manter os motores parados");
    passed &= expect(output.status.phase == "ball_aligned",
                     "tx central deve ser informado como alinhado");

    mission.reset();
    output = mission.update(ball(2.1), stopped, start);
    passed &= expect(output.leftPower > 0.0 && output.rightPower < 0.0,
                     "tx acima de 2 graus deve sair da zona morta");

    ForwardBallSnapshot missing;
    missing.sourceFresh = true;
    output = mission.update(missing, stopped, start);
    passed &= expect(output.leftPower == 0.0 && output.rightPower == 0.0,
                     "perder a bola deve zerar os motores");

    ForwardBallSnapshot stale = ball(15.0);
    stale.sourceFresh = false;
    output = mission.update(stale, stopped, start);
    passed &= expect(output.leftPower == 0.0 && output.rightPower == 0.0,
                     "IPC antigo deve zerar os motores");
    passed &= expect(output.status.phase == "ball_alignment_camera_stale",
                     "IPC antigo deve aparecer na telemetria");

    if (!passed)
    {
        return 1;
    }
    std::cout << "ball_alignment_mission_test: OK\n";
    return 0;
}
