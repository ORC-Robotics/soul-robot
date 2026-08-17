#include "obr/config.h"
#include "obr/main_mission.h"
#include "obr/robot_state.h"

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
    telemetry.lastSensorAgeMs = 0;
    telemetry.motorSleepPinHigh = true;
    telemetry.leftEncoderRate = 0.0;
    telemetry.rightEncoderRate = 0.0;
    return telemetry;
}

CameraLineSnapshot trackedLine(std::uint64_t sequence)
{
    CameraLineSnapshot line;
    line.sourceFresh = true;
    line.nearValid = true;
    line.farValid = true;
    line.leftPreview = 0.66;
    line.rightPreview = 0.66;
    line.lineSequence = sequence;
    return line;
}

CameraLineSnapshot greenMarker(
    std::uint64_t sequence,
    GreenTurnDecision decision)
{
    CameraLineSnapshot line = trackedLine(sequence);
    line.greenNearSeen = true;
    line.greenPathBlackValid = true;
    line.greenConfirmed = true;
    line.greenTurnDecision = decision;
    return line;
}

CameraLineSnapshot unconfirmedGreen(std::uint64_t sequence)
{
    CameraLineSnapshot line = trackedLine(sequence);
    line.greenNearSeen = true;
    line.greenPathBlackValid = true;
    return line;
}

CameraLineSnapshot greenWithWhiteAbove(std::uint64_t sequence)
{
    CameraLineSnapshot line = trackedLine(sequence);
    line.greenNearSeen = true;
    line.greenPathBlackValid = false;
    return line;
}

CameraLineSnapshot gapLine(std::uint64_t sequence, double alignmentError)
{
    CameraLineSnapshot line = trackedLine(sequence);
    line.gapCandidate = true;
    line.gapAlignmentValid = true;
    line.gapAlignmentError = alignmentError;
    return line;
}

CameraLineSnapshot lineLost(std::uint64_t sequence)
{
    CameraLineSnapshot line;
    line.sourceFresh = true;
    line.lineSequence = sequence;
    return line;
}

CameraLineSnapshot farLine(std::uint64_t sequence, double error)
{
    CameraLineSnapshot line = lineLost(sequence);
    line.farValid = true;
    line.farError = error;
    return line;
}

struct MissionFixture
{
    RobotState robotState;
    MainMission mission;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    std::uint64_t sequence = 0;

    MissionFixture()
    {
        robotState.setAutonomousMission(AutonomousMission::MainMission);
        robotState.startAutonomous();
    }

    void update(const CameraLineSnapshot& line, bool cameraReady = true)
    {
        mission.update(robotState, telemetry, cameraReady, line);
    }

    void enterCrossing(double alignmentError = 0.0)
    {
        for (int sample = 0; sample < config::kGapConfirmationSamples; ++sample)
        {
            update(gapLine(++sequence, alignmentError));
        }
        require(
            robotState.snapshot().autonomousStatus.phase == "crossing_gap",
            "Três frames deveriam iniciar o avanço reto do gap.");
    }

    void loseLine(long long leftCount, long long rightCount)
    {
        telemetry.leftEncoderCount = leftCount;
        telemetry.rightEncoderCount = rightCount;
        update(lineLost(++sequence));
    }

    void reachGapLimit()
    {
        const long long limitCounts = static_cast<long long>(std::ceil(
            config::kGapMaximumDistanceCm *
            config::kEncoderCountsPerCentimeter));
        telemetry.leftEncoderCount += limitCounts;
        telemetry.rightEncoderCount += limitCounts - 20;
        update(lineLost(++sequence));
        require(
            robotState.snapshot().autonomousStatus.phase ==
                "gap_searching_left",
            "O limite do gap deveria iniciar a busca pela linha.");
    }
};

void testGapRequiresConfirmationAndNeverRealigns()
{
    MissionFixture fixture;
    fixture.update(gapLine(++fixture.sequence, -0.80));
    fixture.update(gapLine(++fixture.sequence, -0.80));
    require(
        fixture.robotState.snapshot().autonomousStatus.phase ==
            "tracking_near",
        "Dois frames não podem confirmar um gap.");

    fixture.update(gapLine(++fixture.sequence, -0.80));
    const RobotSnapshot leftError = fixture.robotState.snapshot();
    require(leftError.autonomousStatus.phase == "crossing_gap",
            "O terceiro frame deveria iniciar a travessia.");
    require(leftError.left > 0.0 && leftError.right > 0.0 &&
                leftError.left == leftError.right,
            "Erro de alinhamento não pode provocar giro no gap.");

    MissionFixture rightFixture;
    rightFixture.enterCrossing(0.80);
    const RobotSnapshot rightError = rightFixture.robotState.snapshot();
    require(rightError.left > 0.0 && rightError.right > 0.0 &&
                rightError.left == rightError.right,
            "O gap deve seguir reto para qualquer erro de alinhamento.");
}

void testGapDistanceStartsOnlyAfterNearLoss()
{
    MissionFixture fixture;
    fixture.telemetry.leftEncoderCount = 400;
    fixture.telemetry.rightEncoderCount = 450;
    fixture.enterCrossing(0.70);

    fixture.telemetry.leftEncoderCount = 900;
    fixture.telemetry.rightEncoderCount = 950;
    fixture.update(gapLine(++fixture.sequence, -0.70));
    const RobotSnapshot approaching = fixture.robotState.snapshot();
    require(approaching.autonomousStatus.phase == "crossing_gap",
            "O robô deveria continuar reto antes de perder a linha.");
    require(approaching.autonomousStatus.leftDistanceCm == 0.0 &&
                approaching.autonomousStatus.rightDistanceCm == 0.0,
            "A aproximação ainda sobre a linha não pode consumir os 20 cm.");

    fixture.loseLine(900, 950);
    fixture.telemetry.leftEncoderCount += 100;
    fixture.telemetry.rightEncoderCount += 100;
    fixture.update(lineLost(++fixture.sequence));
    const RobotSnapshot crossing = fixture.robotState.snapshot();
    require(crossing.autonomousStatus.leftDistanceCm > 0.0 &&
                crossing.autonomousStatus.leftDistanceCm < 1.0,
            "A distância deveria ser medida a partir da perda da NEAR.");
}

void testGapSearchesAtMaximumDistance()
{
    MissionFixture fixture;
    fixture.enterCrossing();
    fixture.loseLine(1000, 1200);
    fixture.reachGapLimit();

    const RobotSnapshot searching = fixture.robotState.snapshot();
    require(searching.mode == "autonomous",
            "O limite de 20 cm deve manter a missão autônoma ativa.");
    require(searching.left == 0.0 && searching.right == 0.0,
            "A transição para a busca deve zerar os motores por um ciclo.");
    require(searching.autonomousStatus.phase == "gap_searching_left",
            "O limite do gap deveria iniciar a busca pela linha.");

    fixture.update(lineLost(++fixture.sequence));
    const RobotSnapshot rotating = fixture.robotState.snapshot();
    require(rotating.mode == "autonomous" && rotating.left < 0.0 &&
                rotating.right > 0.0,
            "Depois da transição, o robô deveria procurar a linha girando.");

    std::this_thread::sleep_for(std::chrono::milliseconds(1320));
    fixture.update(lineLost(++fixture.sequence));
    const RobotSnapshot waiting = fixture.robotState.snapshot();
    require(waiting.mode == "autonomous" && waiting.left == 0.0 &&
                waiting.right == 0.0 &&
                waiting.autonomousStatus.phase == "gap_search_waiting",
            "Ao terminar a busca, a missão deveria aguardar sem ser encerrada.");

    fixture.update(trackedLine(++fixture.sequence));
    const RobotSnapshot recovered = fixture.robotState.snapshot();
    require(recovered.mode == "autonomous" && recovered.left > 0.0 &&
                recovered.right > 0.0,
            "Uma nova linha deveria reativar o movimento automaticamente.");
}

void testGapReacquiresNearBeforeLimit()
{
    MissionFixture fixture;
    fixture.enterCrossing();
    fixture.loseLine(0, 0);
    for (int sample = 0; sample < config::kGapConfirmationSamples; ++sample)
    {
        fixture.telemetry.leftEncoderCount += 30;
        fixture.telemetry.rightEncoderCount += 30;
        fixture.update(trackedLine(++fixture.sequence));
    }

    require(
        fixture.robotState.snapshot().autonomousStatus.phase ==
            "reacquiring_near",
        "A NEAR confirmada deveria encerrar a travessia antes do limite.");
}

void testGapReacquiresFarBeforeLimit()
{
    MissionFixture fixture;
    fixture.enterCrossing();
    fixture.loseLine(0, 0);
    for (int sample = 0; sample < config::kGapConfirmationSamples; ++sample)
    {
        fixture.telemetry.leftEncoderCount += 30;
        fixture.telemetry.rightEncoderCount += 30;
        fixture.update(farLine(++fixture.sequence, -0.20));
    }

    require(fixture.robotState.snapshot().autonomousStatus.phase ==
                "recovering_far",
            "A FAR confirmada deveria encerrar a travessia antes do limite.");
}

void testGapUsesDisconnectedContinuation()
{
    MissionFixture fixture;
    fixture.enterCrossing();
    for (int sample = 0; sample < config::kGapConfirmationSamples; ++sample)
    {
        CameraLineSnapshot returned = gapLine(++fixture.sequence, 0.0);
        returned.gapReturnValid = true;
        returned.gapReturnError = 0.20;
        fixture.update(returned);
    }

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "recovering_far",
            "A continuação desconectada deveria encerrar o avanço reto.");
    require(snapshot.left > snapshot.right,
            "A continuação à direita deveria corrigir para a direita.");
}

void testGapStopsWhenEncoderBecomesStaleDuringCrossing()
{
    MissionFixture fixture;
    fixture.enterCrossing();
    fixture.telemetry.lastSensorAgeMs = config::kGapEncoderFreshnessMs + 1;
    fixture.update(lineLost(++fixture.sequence));

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "stopped" && snapshot.left == 0.0 &&
                snapshot.right == 0.0,
            "Encoder desatualizado deve parar a travessia.");
    require(snapshot.autonomousStatus.phase == "gap_encoder_lost",
            "A falha de encoder deveria permanecer na telemetria.");
}

void testGapStopsOnStall()
{
    MissionFixture fixture;
    fixture.enterCrossing();
    fixture.loseLine(0, 0);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config::kGapStallTimeoutMs + 20));
    fixture.update(lineLost(++fixture.sequence));

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "stopped" &&
                snapshot.autonomousStatus.phase == "gap_encoder_stall",
            "Ausência de progresso deveria parar o gap.");
}

void testGapSearchesOnTraversalTimeout()
{
    MissionFixture fixture;
    fixture.enterCrossing();
    fixture.loseLine(0, 0);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config::kGapTraversalTimeoutMs + 20));
    fixture.update(lineLost(++fixture.sequence));

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "autonomous" && snapshot.left == 0.0 &&
                snapshot.right == 0.0,
            "O tempo limite deveria trocar o avanço pela busca da linha.");
    require(snapshot.autonomousStatus.phase == "gap_searching_left",
            "A busca deveria permanecer visível na telemetria.");
}

void testGapStopsWhenCameraBecomesUnavailable()
{
    MissionFixture fixture;
    fixture.enterCrossing();
    fixture.update(lineLost(++fixture.sequence), false);

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "stopped" && snapshot.left == 0.0 &&
                snapshot.right == 0.0,
            "A perda da câmera deve zerar os motores.");
}

void testEmergencyStopWinsDuringCrossing()
{
    MissionFixture fixture;
    fixture.enterCrossing();
    fixture.loseLine(0, 0);
    fixture.robotState.emergencyStop();
    fixture.update(lineLost(++fixture.sequence));

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.emergencyStop && snapshot.mode == "emergency" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "O E-Stop deve ter prioridade durante a travessia do gap.");
}

void testGreenMarkersStartExpectedTurns()
{
    const struct
    {
        GreenTurnDecision decision;
        bool leftPositive;
        const char* description;
    } cases[] = {
        {GreenTurnDecision::Right80, true, "80 graus à direita"},
        {GreenTurnDecision::Left80, false, "80 graus à esquerda"},
        {GreenTurnDecision::TurnAround180, true, "180 graus à direita"},
    };

    for (const auto& testCase : cases)
    {
        MissionFixture fixture;
        fixture.telemetry.mpuOk = true;
        fixture.telemetry.yawZDeg = 0.0;
        fixture.telemetry.gyroZDegPerSec = 0.0;
        fixture.update(greenMarker(++fixture.sequence, testCase.decision));

        if (testCase.decision != GreenTurnDecision::TurnAround180)
        {
            require(
                fixture.robotState.snapshot().autonomousStatus.phase ==
                    "green_pre_turn_ready",
                "A curva de 80 graus deveria preparar o avanço de 5 cm.");
            const long long targetCounts = static_cast<long long>(std::ceil(
                config::kGreenPreTurnDistanceCm *
                config::kEncoderCountsPerCentimeter));
            fixture.telemetry.leftEncoderCount += targetCounts;
            fixture.telemetry.rightEncoderCount += targetCounts;
            fixture.update(greenMarker(++fixture.sequence, testCase.decision));
        }

        const RobotSnapshot snapshot = fixture.robotState.snapshot();
        require(snapshot.autonomousStatus.phase == "green_turning",
                std::string("O marcador deveria iniciar o giro de ") +
                    testCase.description + ".");
        require(testCase.leftPositive ? snapshot.left > 0.0
                                      : snapshot.left < 0.0,
                "O motor esquerdo recebeu o sentido incorreto.");
        require(testCase.leftPositive ? snapshot.right < 0.0
                                      : snapshot.right > 0.0,
                "O motor direito recebeu o sentido incorreto.");
    }
}

void testGreenWithWhiteAboveKeepsFollowingStraight()
{
    MissionFixture fixture;
    fixture.update(greenWithWhiteAbove(++fixture.sequence));

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "autonomous" && snapshot.left > 0.0 &&
                snapshot.right > 0.0 && snapshot.left == snapshot.right,
            "Verde com branco acima deveria manter o movimento reto.");
    require(snapshot.autonomousStatus.phase == "green_ignored_straight",
            "O painel deveria informar que o verde branco foi ignorado.");
}

void testRobotStopsWhileConfirmingGreen()
{
    MissionFixture fixture;
    fixture.update(unconfirmedGreen(++fixture.sequence));

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "autonomous" && snapshot.left == 0.0 &&
                snapshot.right == 0.0,
            "O robô deveria parar enquanto confirma o marcador verde.");
    require(snapshot.autonomousStatus.phase == "green_reading",
            "O painel deveria informar que a leitura verde está sendo confirmada.");
}

void testRobotResumesLineAfterUnconfirmedGreenReading()
{
    MissionFixture fixture;
    for (int sample = 0; sample < config::kGreenReadingMaximumSamples; ++sample)
    {
        fixture.update(unconfirmedGreen(++fixture.sequence));
    }

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "autonomous" && snapshot.left > 0.0 &&
                snapshot.right > 0.0,
            "Leitura verde sem decisão deveria devolver o segue-faixa.");
    require(snapshot.autonomousStatus.phase == "tracking_near",
            "O painel deveria voltar a indicar o seguimento da linha.");
}

void testGreen80TurnDrivesFiveCentimetersBeforeTurning()
{
    MissionFixture fixture;
    fixture.telemetry.mpuOk = true;
    fixture.telemetry.yawZDeg = 0.0;
    fixture.update(greenMarker(
        ++fixture.sequence, GreenTurnDecision::Right80));

    require(
        fixture.robotState.snapshot().autonomousStatus.phase ==
            "green_pre_turn_ready",
        "O verde confirmado deveria preparar o avanço antes da curva.");

    fixture.update(greenMarker(
        ++fixture.sequence, GreenTurnDecision::Right80));
    RobotSnapshot driving = fixture.robotState.snapshot();
    require(driving.autonomousStatus.phase == "green_pre_turn_driving" &&
                driving.left > 0.0 && driving.right > 0.0,
            "Antes do giro, as duas rodas deveriam avançar.");

    const long long targetCounts = static_cast<long long>(std::ceil(
        config::kGreenPreTurnDistanceCm *
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount += targetCounts;
    fixture.telemetry.rightEncoderCount += targetCounts;
    fixture.update(greenMarker(
        ++fixture.sequence, GreenTurnDecision::Right80));

    const RobotSnapshot turning = fixture.robotState.snapshot();
    require(turning.autonomousStatus.phase == "green_turning" &&
                turning.left > 0.0 && turning.right < 0.0,
            "Ao completar 5 cm, o giro de 80 graus deveria começar.");

    fixture.telemetry.yawZDeg = config::kGreenTurnTargetDegrees;
    fixture.telemetry.gyroZDegPerSec = 0.0;
    fixture.update(greenMarker(
        ++fixture.sequence, GreenTurnDecision::Right80));
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config::kTurn90SettleMs + 20));
    fixture.update(greenMarker(
        ++fixture.sequence, GreenTurnDecision::Right80));

    require(
        fixture.robotState.snapshot().autonomousStatus.phase ==
            "green_turn_completed",
        "Depois do giro, a missão deveria retornar ao segue-faixa.");
}
}

int main()
{
    try
    {
        testGapRequiresConfirmationAndNeverRealigns();
        testGapDistanceStartsOnlyAfterNearLoss();
        testGapSearchesAtMaximumDistance();
        testGapReacquiresNearBeforeLimit();
        testGapReacquiresFarBeforeLimit();
        testGapUsesDisconnectedContinuation();
        testGapStopsWhenEncoderBecomesStaleDuringCrossing();
        testGapStopsOnStall();
        testGapSearchesOnTraversalTimeout();
        testGapStopsWhenCameraBecomesUnavailable();
        testEmergencyStopWinsDuringCrossing();
        testGreenMarkersStartExpectedTurns();
        testGreenWithWhiteAboveKeepsFollowingStraight();
        testRobotStopsWhileConfirmingGreen();
        testRobotResumesLineAfterUnconfirmedGreenReading();
        testGreen80TurnDrivesFiveCentimetersBeforeTurning();
        std::cout << "16 testes da missão principal concluídos com sucesso."
                  << std::endl;
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Falha no teste da travessia de gap: " << error.what()
                  << std::endl;
        return 1;
    }
}
