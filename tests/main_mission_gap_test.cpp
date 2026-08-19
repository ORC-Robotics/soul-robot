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

CameraLineSnapshot gapWithoutNear(std::uint64_t sequence)
{
    CameraLineSnapshot line = lineLost(sequence);
    line.gapCandidate = true;
    return line;
}

CameraLineSnapshot strongCurve(std::uint64_t sequence, double error)
{
    CameraLineSnapshot line = trackedLine(sequence);
    line.nearError = error;
    line.controlError = error;
    return line;
}

CameraLineSnapshot farOnlyLine(std::uint64_t sequence, double error)
{
    CameraLineSnapshot line = lineLost(sequence);
    line.farValid = true;
    line.farX = 320.0 * (1.0 + error);
    line.farError = error;
    line.controlError = error;
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
        update(gapLine(++sequence, alignmentError));
        require(
            robotState.snapshot().autonomousStatus.phase == "crossing_gap",
            "A primeira detecção deveria iniciar o avanço reto do gap.");
    }

    void loseLine(long long leftCount, long long rightCount)
    {
        telemetry.leftEncoderCount = leftCount;
        telemetry.rightEncoderCount = rightCount;
        update(lineLost(++sequence));
    }

};

void testGapStartsImmediatelyAndNeverRealigns()
{
    MissionFixture fixture;
    fixture.update(gapLine(++fixture.sequence, -0.80));
    const RobotSnapshot leftError = fixture.robotState.snapshot();
    require(leftError.autonomousStatus.phase == "crossing_gap",
            "A primeira detecção deveria iniciar a travessia.");
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

void testGapIgnoresEncoderStateAndDistance()
{
    MissionFixture fixture;
    fixture.telemetry.lastSensorAgeMs = 100000;
    fixture.telemetry.leftEncoderRate = std::nan("");
    fixture.telemetry.rightEncoderRate = std::nan("");
    fixture.enterCrossing();
    fixture.loseLine(1000000, -1000000);

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "autonomous" && snapshot.left > 0.0 &&
                snapshot.right > 0.0,
            "O estado dos encoders não deve interromper o GAP.");
    require(snapshot.autonomousStatus.phase == "crossing_gap",
            "Distância e telemetria dos encoders devem ser ignoradas no GAP.");
}

void testGapReacquiresNearOnFirstFrame()
{
    MissionFixture fixture;
    fixture.enterCrossing();
    fixture.loseLine(0, 0);
    fixture.update(trackedLine(++fixture.sequence));

    require(fixture.robotState.snapshot().autonomousStatus.phase ==
                "reacquiring_near",
            "A primeira NEAR válida deveria encerrar a travessia.");
}

void testGapStartsWithNearInvalid()
{
    MissionFixture fixture;
    fixture.update(gapWithoutNear(++fixture.sequence));

    require(fixture.robotState.snapshot().autonomousStatus.phase ==
                "crossing_gap",
            "O componente inferior deveria iniciar o gap sem NEAR válida.");
    fixture.update(trackedLine(++fixture.sequence));
    require(fixture.robotState.snapshot().autonomousStatus.phase ==
                "reacquiring_near",
            "A NEAR deve ser readquirida após o gap iniciado sem linha nela.");
}

void testGapUsesDisconnectedContinuationImmediately()
{
    MissionFixture fixture;
    fixture.enterCrossing();
    CameraLineSnapshot returned = gapLine(++fixture.sequence, 0.0);
    returned.gapReturnValid = true;
    returned.gapReturnError = 0.20;
    fixture.update(returned);

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "reacquiring_near",
            "A continuação desconectada deveria encerrar o avanço reto.");
    require(snapshot.left > snapshot.right,
            "A continuação à direita deveria corrigir para a direita.");
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

void testGapTimeoutStopsMotors()
{
    MissionFixture fixture;
    fixture.update(gapWithoutNear(++fixture.sequence));
    std::this_thread::sleep_for(std::chrono::milliseconds(2020));
    fixture.update(lineLost(++fixture.sequence));

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "stopped" && snapshot.left == 0.0 &&
                snapshot.right == 0.0,
            "Dois segundos sem linha devem zerar os motores no gap.");
}

void testStrongTurnUsesTwoNearSamplesAndThreeAlignedSamples()
{
    MissionFixture fixture;
    fixture.update(strongCurve(++fixture.sequence, 0.50));
    require(fixture.robotState.snapshot().autonomousStatus.phase ==
                "tracking_near",
            "Uma amostra forte ainda não deve iniciar a contrarrotação.");

    fixture.update(strongCurve(++fixture.sequence, 0.50));
    RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "turning_near" &&
                snapshot.left == 0.69 && snapshot.right == -0.69,
            "Duas amostras fortes da NEAR devem iniciar a contrarrotação.");

    fixture.update(strongCurve(++fixture.sequence, 0.0));
    fixture.update(strongCurve(++fixture.sequence, 0.0));
    require(fixture.robotState.snapshot().autonomousStatus.phase ==
                "turning_near",
            "Duas amostras alinhadas ainda devem manter a curva forte.");
    fixture.update(strongCurve(++fixture.sequence, 0.0));
    require(fixture.robotState.snapshot().autonomousStatus.phase ==
                "tracking_near",
            "Três amostras alinhadas devem encerrar a curva forte.");
}

void testStrongTurnStopsImmediatelyOnErrorSignChange()
{
    MissionFixture fixture;
    fixture.update(strongCurve(++fixture.sequence, -0.50));
    fixture.update(strongCurve(++fixture.sequence, -0.50));
    require(fixture.robotState.snapshot().autonomousStatus.phase ==
                "turning_near",
            "A curva forte à esquerda deveria estar ativa.");

    fixture.update(strongCurve(++fixture.sequence, 0.01));
    require(fixture.robotState.snapshot().autonomousStatus.phase ==
                "tracking_near",
            "A inversão do erro deve encerrar a curva no mesmo frame.");
}

void testFarOnlyUsesConservativeFallbackUntilNearReturns()
{
    MissionFixture fixture;
    fixture.update(farOnlyLine(++fixture.sequence, 0.30));

    RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "fallback_far" &&
                snapshot.left > snapshot.right && snapshot.right > 0.0,
            "A FAR isolada deve alinhar com correção limitada e positiva.");

    fixture.update(trackedLine(++fixture.sequence));
    require(fixture.robotState.snapshot().autonomousStatus.phase ==
                "reacquiring_near",
            "A NEAR deve reassumir por meio da reaquisição já existente.");
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

void testSingleGreenMarkersKeepContinuousGuidance()
{
    const struct
    {
        GreenTurnDecision decision;
        double leftPreview;
        double rightPreview;
        const char* description;
    } cases[] = {
        {GreenTurnDecision::Approach, 0.69, 0.69, "aproximação"},
        {GreenTurnDecision::GuideLeft, 0.69, 0.71, "guia à esquerda"},
        {GreenTurnDecision::GuideRight, 0.71, 0.69, "guia à direita"},
    };

    for (const auto& testCase : cases)
    {
        MissionFixture fixture;
        CameraLineSnapshot marker = greenMarker(
            ++fixture.sequence, testCase.decision);
        if (testCase.decision == GreenTurnDecision::Approach)
        {
            marker.greenConfirmed = false;
        }
        marker.leftPreview = testCase.leftPreview;
        marker.rightPreview = testCase.rightPreview;
        fixture.update(marker);

        const RobotSnapshot snapshot = fixture.robotState.snapshot();
        const std::string expectedPhase =
            testCase.decision == GreenTurnDecision::Approach
                ? "green_approach"
                : "green_guidance";
        require(snapshot.autonomousStatus.phase == expectedPhase,
                std::string("O marcador deveria manter o tracking em ") +
                    testCase.description + ".");
        require(snapshot.left == testCase.leftPreview &&
                    snapshot.right == testCase.rightPreview,
                "A missão deveria aplicar diretamente a prévia segura da visão.");
    }
}

void testInvalidOrUnconfirmedGreenDoesNotStopTracking()
{
    MissionFixture fixture;
    fixture.update(greenWithWhiteAbove(++fixture.sequence));

    RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "autonomous" && snapshot.left == 0.66 &&
                snapshot.right == 0.66,
            "Verde com branco acima deveria manter o movimento reto.");
    fixture.update(unconfirmedGreen(++fixture.sequence));
    snapshot = fixture.robotState.snapshot();
    require(snapshot.left == 0.66 && snapshot.right == 0.66,
            "A confirmação visual não pode interromper o segue-faixa.");
    require(snapshot.autonomousStatus.phase == "tracking_near",
            "Verde sem ação válida deveria permanecer no tracking.");
}

void completeGreenTurnAround(MissionFixture& fixture)
{
    fixture.telemetry.yawZDeg = config::kGreenTurnAroundTargetDegrees;
    fixture.telemetry.gyroZDegPerSec = 0.0;
    fixture.update(greenMarker(
        ++fixture.sequence, GreenTurnDecision::TurnAround180));
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config::kTurn90SettleMs + 20));
    fixture.update(greenMarker(
        ++fixture.sequence, GreenTurnDecision::TurnAround180));
}

void testDoubleGreenUsesImuAndEntersRecovery()
{
    MissionFixture fixture;
    fixture.telemetry.mpuOk = true;
    fixture.telemetry.yawZDeg = 0.0;
    fixture.telemetry.gyroZDegPerSec = 0.0;
    fixture.update(greenMarker(
        ++fixture.sequence, GreenTurnDecision::TurnAround180));

    RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "green_turning" &&
                snapshot.left > 0.0 && snapshot.right < 0.0,
            "O verde duplo deveria iniciar somente o retorno por IMU.");

    completeGreenTurnAround(fixture);
    snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "green_turn_completed",
            "O retorno concluído deveria preparar a recuperação da linha.");

    fixture.update(greenMarker(
        ++fixture.sequence, GreenTurnDecision::TurnAround180));
    snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase != "green_turning",
            "O latch não pode repetir o retorno sobre o mesmo marcador.");

    fixture.update(trackedLine(++fixture.sequence));
    snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "reacquiring_near" ||
                snapshot.autonomousStatus.phase == "tracking_near",
            "Depois do retorno, a missão deveria confirmar novamente a NEAR.");

    fixture.update(trackedLine(++fixture.sequence));
    fixture.update(trackedLine(++fixture.sequence));
    fixture.update(greenMarker(
        ++fixture.sequence, GreenTurnDecision::TurnAround180));
    require(fixture.robotState.snapshot().autonomousStatus.phase ==
                "green_turning",
            "Um novo verde duplo deveria ser aceito após o marcador anterior sair.");
}
}

int main()
{
    try
    {
        testGapStartsImmediatelyAndNeverRealigns();
        testGapIgnoresEncoderStateAndDistance();
        testGapReacquiresNearOnFirstFrame();
        testGapStartsWithNearInvalid();
        testGapUsesDisconnectedContinuationImmediately();
        testGapTimeoutStopsMotors();
        testGapStopsWhenCameraBecomesUnavailable();
        testEmergencyStopWinsDuringCrossing();
        testStrongTurnUsesTwoNearSamplesAndThreeAlignedSamples();
        testStrongTurnStopsImmediatelyOnErrorSignChange();
        testFarOnlyUsesConservativeFallbackUntilNearReturns();
        testSingleGreenMarkersKeepContinuousGuidance();
        testInvalidOrUnconfirmedGreenDoesNotStopTracking();
        testDoubleGreenUsesImuAndEntersRecovery();
        std::cout << "14 testes da missão principal concluídos com sucesso."
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
