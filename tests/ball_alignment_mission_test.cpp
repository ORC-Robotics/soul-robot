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

ForwardBallSnapshot ball(double txDegrees)
{
    ForwardBallSnapshot snapshot;
    snapshot.sourceFresh = true;
    snapshot.detected = true;
    snapshot.type = "silver_ball";
    snapshot.txDegrees = txDegrees;
    snapshot.distanceCm = 40.0;
    snapshot.radiusPixels = 71.7;
    return snapshot;
}
}

int main()
{
    bool passed = true;
    const auto start = std::chrono::steady_clock::time_point{};

    BallAlignmentMission mission;
    BallAlignmentOutput output = mission.update(ball(12.0), start);
    passed &= expect(output.leftPower > 0.0 && output.rightPower < 0.0,
                     "tx positivo deve girar para a direita");
    passed &= expect(output.status.phase == "ball_alignment_turning",
                     "o primeiro comando deve iniciar um pulso");

    output = mission.update(
        ball(12.0),
        start + std::chrono::milliseconds(config::kBallAlignmentPulseMs + 1));
    passed &= expect(output.leftPower == 0.0 && output.rightPower == 0.0,
                     "o robô deve parar entre pulsos");
    passed &= expect(output.status.phase == "ball_alignment_settling",
                     "o pulso deve entrar em estabilização");

    mission.reset();
    output = mission.update(ball(-8.0), start);
    passed &= expect(output.leftPower < 0.0 && output.rightPower > 0.0,
                     "tx negativo deve girar para a esquerda");

    mission.reset();
    output = mission.update(ball(2.0), start);
    passed &= expect(output.leftPower == 0.0 && output.rightPower == 0.0,
                     "a zona morta deve manter os motores parados");
    passed &= expect(output.status.phase == "ball_aligned",
                     "tx central deve ser informado como alinhado");

    ForwardBallSnapshot missing;
    missing.sourceFresh = true;
    output = mission.update(missing, start);
    passed &= expect(output.leftPower == 0.0 && output.rightPower == 0.0,
                     "perder a bola deve zerar os motores");

    ForwardBallSnapshot stale = ball(15.0);
    stale.sourceFresh = false;
    output = mission.update(stale, start);
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
