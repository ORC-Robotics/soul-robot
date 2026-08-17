#include "obr/config.h"
#include "obr/main_mission.h"
#include "obr/robot_state.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

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

CameraLineSnapshot gapLine(std::uint64_t sequence, double alignmentError)
{
    CameraLineSnapshot line = trackedLine(sequence);
    line.gapCandidate = true;
    line.gapAlignmentValid = true;
    line.gapAlignmentError = alignmentError;
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

    void enterCrossing()
    {
        // Três frames confirmam a geometria e três confirmam o alinhamento.
        // O terceiro frame do candidato também é o primeiro do alinhamento.
        for (int sample = 0; sample < 5; ++sample)
        {
            update(gapLine(++sequence, 0.0));
        }
        require(
            robotState.snapshot().autonomousStatus.phase == "crossing_gap",
            "O gap alinhado deveria iniciar a travessia.");
    }
};

void testGapRequiresConfirmationAndAlignment()
{
    MissionFixture fixture;
    fixture.update(gapLine(++fixture.sequence, -0.30));
    fixture.update(gapLine(++fixture.sequence, -0.30));
    require(
        fixture.robotState.snapshot().autonomousStatus.phase == "tracking_near",
        "Dois frames não podem confirmar um gap.");

    fixture.update(gapLine(++fixture.sequence, -0.30));
    const RobotSnapshot aligning = fixture.robotState.snapshot();
    require(
        aligning.autonomousStatus.phase == "aligning_for_gap",
        "Três frames deveriam iniciar o alinhamento.");
    require(
        aligning.left < 0.0 && aligning.right > 0.0,
        "Erro à esquerda deveria produzir contrarrotação à esquerda.");

    fixture.update(gapLine(++fixture.sequence, 0.0));
    fixture.update(gapLine(++fixture.sequence, 0.0));
    require(
        fixture.robotState.snapshot().autonomousStatus.phase ==
            "aligning_for_gap",
        "Dois frames alinhados ainda não podem mover para frente.");
    fixture.update(gapLine(++fixture.sequence, 0.0));
    require(
        fixture.robotState.snapshot().autonomousStatus.phase == "crossing_gap",
        "Três frames alinhados deveriam liberar a travessia.");

    MissionFixture rightFixture;
    for (int sample = 0; sample < config::kGapConfirmationSamples; ++sample)
    {
        rightFixture.update(gapLine(++rightFixture.sequence, 0.30));
    }
    const RobotSnapshot aligningRight = rightFixture.robotState.snapshot();
    require(
        aligningRight.left > 0.0 && aligningRight.right < 0.0,
        "Erro à direita deveria produzir contrarrotação à direita.");
}

void testGapCancellationReturnsToTracking()
{
    MissionFixture fixture;
    for (int sample = 0; sample < 3; ++sample)
    {
        fixture.update(gapLine(++fixture.sequence, 0.30));
    }

    CameraLineSnapshot normal = trackedLine(++fixture.sequence);
    fixture.update(normal);
    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(
        snapshot.autonomousStatus.phase == "tracking_near",
        "Um candidato perdido deveria devolver o controle ao tracking.");
    require(snapshot.left > 0.0 && snapshot.right > 0.0,
            "O tracking deveria voltar a avançar sobre a fita válida.");
}

void testGapStopsAtMaximumWheelDistanceAndWaits()
{
    MissionFixture fixture;
    fixture.enterCrossing();
    const long long limitCounts = static_cast<long long>(std::ceil(
        config::kGapMaximumDistanceCm * config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount = limitCounts;
    fixture.telemetry.rightEncoderCount = limitCounts - 20;
    fixture.update(gapLine(++fixture.sequence, 0.0));

    const RobotSnapshot waiting = fixture.robotState.snapshot();
    require(waiting.mode == "autonomous",
            "O limite do gap deve manter a missão ativa.");
    require(waiting.autonomousStatus.phase == "gap_waiting",
            "O limite de 100 mm deveria entrar em espera.");
    require(waiting.left == 0.0 && waiting.right == 0.0,
            "A espera do gap deve manter os motores zerados.");

    for (int sample = 0; sample < config::kGapConfirmationSamples; ++sample)
    {
        CameraLineSnapshot returned = trackedLine(++fixture.sequence);
        fixture.update(returned);
    }
    require(
        fixture.robotState.snapshot().autonomousStatus.phase ==
            "reacquiring_near",
        "A espera deveria retomar após três frames NEAR válidos.");
}

void testGapReacquiresFarBeforeLimit()
{
    MissionFixture fixture;
    fixture.enterCrossing();
    for (int sample = 0; sample < config::kGapConfirmationSamples; ++sample)
    {
        fixture.telemetry.leftEncoderCount += 50;
        fixture.telemetry.rightEncoderCount += 50;
        CameraLineSnapshot returned = trackedLine(++fixture.sequence);
        returned.nearValid = false;
        returned.farValid = true;
        returned.farError = -0.20;
        fixture.update(returned);
    }

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "recovering_far",
            "A continuação FAR deveria encerrar a travessia.");
    require(snapshot.left > 0.0 || snapshot.right > 0.0,
            "A recuperação FAR deveria comandar uma correção controlada.");
}

void testGapUsesDisconnectedReturnBeforeNearDisappears()
{
    MissionFixture fixture;
    fixture.enterCrossing();
    for (int sample = 0; sample < config::kGapConfirmationSamples; ++sample)
    {
        fixture.telemetry.leftEncoderCount += 30;
        fixture.telemetry.rightEncoderCount += 30;
        CameraLineSnapshot returned = gapLine(++fixture.sequence, 0.0);
        returned.gapReturnValid = true;
        returned.gapReturnError = 0.20;
        fixture.update(returned);
    }

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "recovering_far",
            "A continuação desconectada deveria encerrar o avanço reto.");
    require(snapshot.left > snapshot.right,
            "A continuação à direita deveria comandar correção à direita.");

    CameraLineSnapshot stillVisible = gapLine(++fixture.sequence, 0.0);
    stillVisible.gapReturnValid = true;
    stillVisible.gapReturnError = 0.20;
    fixture.update(stillVisible);
    require(
        fixture.robotState.snapshot().autonomousStatus.action ==
            "Guiando pela continuação desconectada do gap",
        "A fita antiga na NEAR não pode substituir a continuação confirmada.");
}

void testGapStopsWhenEncoderBecomesStale()
{
    MissionFixture fixture;
    fixture.enterCrossing();
    fixture.telemetry.lastSensorAgeMs = config::kGapEncoderFreshnessMs + 1;
    fixture.update(gapLine(++fixture.sequence, 0.0));

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "stopped",
            "Encoder desatualizado deveria encerrar a missão.");
    require(snapshot.left == 0.0 && snapshot.right == 0.0,
            "Falha de encoder deve zerar os motores.");
    require(snapshot.autonomousStatus.phase == "gap_encoder_lost",
            "A falha do encoder deveria permanecer visível na telemetria.");
}

void testGapStopsWhenCameraBecomesUnavailable()
{
    MissionFixture fixture;
    fixture.enterCrossing();
    fixture.update(gapLine(++fixture.sequence, 0.0), false);

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "stopped",
            "A perda da câmera deveria encerrar a missão.");
    require(snapshot.left == 0.0 && snapshot.right == 0.0,
            "A perda da câmera deve zerar os motores.");
}
}

int main()
{
    try
    {
        testGapRequiresConfirmationAndAlignment();
        testGapCancellationReturnsToTracking();
        testGapStopsAtMaximumWheelDistanceAndWaits();
        testGapReacquiresFarBeforeLimit();
        testGapUsesDisconnectedReturnBeforeNearDisappears();
        testGapStopsWhenEncoderBecomesStale();
        testGapStopsWhenCameraBecomesUnavailable();
        std::cout << "7 testes da travessia de gap concluídos com sucesso."
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
