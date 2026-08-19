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
    line.greenCandidateDecision = decision;
    line.greenCandidateFrames = 3;
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

CameraLineSnapshot corner90Line(
    std::uint64_t sequence,
    Corner90Direction direction)
{
    CameraLineSnapshot line = trackedLine(sequence);
    line.corner90Candidate = true;
    line.corner90Direction = direction;
    line.corner90Angle = direction == Corner90Direction::Left ? -90.0 : 90.0;
    line.corner90ConfirmFrames = 3;
    return line;
}

CameraLineSnapshot corner90ExitAlignedLine(std::uint64_t sequence)
{
    CameraLineSnapshot line = trackedLine(sequence);
    line.corner90ExitAlignment = true;
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
                snapshot.right == 0.0 &&
                snapshot.autonomousStatus.phase == "camera_not_ready",
            "A perda da câmera deve zerar os motores.");
}

void testMainMissionStopsWhenLineIpcIsNotFresh()
{
    MissionFixture fixture;
    fixture.update(trackedLine(++fixture.sequence));

    CameraLineSnapshot staleLine = trackedLine(++fixture.sequence);
    staleLine.sourceFresh = false;
    fixture.update(staleLine);

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "stopped" && snapshot.left == 0.0 &&
                snapshot.right == 0.0 &&
                snapshot.autonomousStatus.phase == "line_ipc_stale",
            "IPC visual ausente ou antigo deve impedir a Missão Principal de mover o robô.");
}

void testMainMissionStopsWhenEsp32BecomesUnavailable()
{
    MissionFixture fixture;
    fixture.telemetry.serialOpen = false;
    fixture.update(trackedLine(++fixture.sequence));

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "stopped" && snapshot.left == 0.0 &&
                snapshot.right == 0.0 &&
                snapshot.autonomousStatus.phase == "esp32_not_ready",
            "ESP32 indisponível deve zerar os motores e informar o motivo.");
}

void testGapTimeoutStopsMotors()
{
    MissionFixture fixture;
    fixture.update(gapWithoutNear(++fixture.sequence));
    std::this_thread::sleep_for(std::chrono::milliseconds(2020));
    fixture.update(lineLost(++fixture.sequence));

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "stopped" && snapshot.left == 0.0 &&
                snapshot.right == 0.0 &&
                snapshot.autonomousStatus.phase == "gap_timeout",
            "Dois segundos sem linha devem zerar os motores no gap.");
}

void testTotalLineLossTimeoutReportsReason()
{
    MissionFixture fixture;
    fixture.update(lineLost(++fixture.sequence));
    std::this_thread::sleep_for(std::chrono::milliseconds(1320));
    fixture.update(lineLost(++fixture.sequence));

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "stopped" && snapshot.left == 0.0 &&
                snapshot.right == 0.0 &&
                snapshot.autonomousStatus.phase == "line_lost_timeout",
            "Perda total da linha deve informar o motivo terminal.");
}

void testLargeControlErrorNeverUsesCounterRotation()
{
    MissionFixture fixture;
    CameraLineSnapshot curve = strongCurve(++fixture.sequence, 0.80);
    curve.leftPreview = 0.86;
    curve.rightPreview = 0.61;
    fixture.update(curve);
    curve.lineSequence = ++fixture.sequence;
    fixture.update(curve);

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "tracking_near" &&
                snapshot.left > 0.0 && snapshot.right > 0.0,
            "Erro alto sem geometria de cotovelo deve manter o diferencial contínuo.");
}

void testCorner90UsesGeometryAndExitsAfterTwoAlignedFrames()
{
    MissionFixture fixture;
    fixture.update(corner90Line(
        ++fixture.sequence, Corner90Direction::Right));
    RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "corner90_confirming" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "O primeiro frame forte deve parar para confirmar o cotovelo.");

    fixture.update(corner90Line(
        ++fixture.sequence, Corner90Direction::Right));
    snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "corner90_confirming" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "O segundo frame deve manter a parada curta de confirmação.");

    fixture.update(corner90Line(
        ++fixture.sequence, Corner90Direction::Right));
    snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "corner90_right" &&
                snapshot.left == config::kCorner90PivotStartPower &&
                snapshot.right == -config::kCorner90PivotStartPower,
            "Três frames geométricos devem iniciar o pivô à direita.");

    fixture.update(corner90ExitAlignedLine(++fixture.sequence));
    snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "corner90_right" &&
                snapshot.left == config::kCorner90PivotRunPower &&
                snapshot.right == -config::kCorner90PivotRunPower,
            "O pivô deve reduzir para o piso RUN na primeira leitura de saída.");

    fixture.update(corner90ExitAlignedLine(++fixture.sequence));
    snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "corner90_right",
            "A segunda leitura visual ainda deve manter o pivô.");

    fixture.update(corner90ExitAlignedLine(++fixture.sequence));
    snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "corner90_right",
            "A terceira leitura visual ainda deve manter o pivô.");

    fixture.update(corner90ExitAlignedLine(++fixture.sequence));
    snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "tracking_near" &&
                snapshot.left > 0.0 && snapshot.right > 0.0,
            "Quatro leituras visuais devem devolver o controle ao tracking.");
}

void testCorner90ConfirmationCancelsWeakOrInconsistentGeometry()
{
    MissionFixture weakFixture;
    CameraLineSnapshot weak = corner90Line(
        ++weakFixture.sequence, Corner90Direction::Left);
    weak.corner90Angle = -65.0;
    weakFixture.update(weak);
    require(weakFixture.robotState.snapshot().autonomousStatus.phase ==
                "tracking_near",
            "Ângulo abaixo de 70 graus não pode parar nem iniciar o cotovelo.");

    MissionFixture offsetFixture;
    CameraLineSnapshot offset = corner90Line(
        ++offsetFixture.sequence, Corner90Direction::Left);
    offset.nearError = 0.21;
    offsetFixture.update(offset);
    require(offsetFixture.robotState.snapshot().autonomousStatus.phase ==
                "tracking_near",
            "Linha acima da margem lateral não pode entrar na confirmação.");

    MissionFixture cancelFixture;
    cancelFixture.update(corner90Line(
        ++cancelFixture.sequence, Corner90Direction::Left));
    cancelFixture.update(trackedLine(++cancelFixture.sequence));
    const RobotSnapshot cancelled = cancelFixture.robotState.snapshot();
    require(cancelled.autonomousStatus.phase == "corner90_cancelled" &&
                cancelled.left > 0.0 && cancelled.right > 0.0,
            "Amostra incompatível deve cancelar o cotovelo e retomar o tracking.");
}

void testCorner90StopsWhenPivotDoesNotMove()
{
    MissionFixture fixture;
    fixture.update(corner90Line(
        ++fixture.sequence, Corner90Direction::Left));
    fixture.update(corner90Line(
        ++fixture.sequence, Corner90Direction::Left));
    fixture.update(corner90Line(
        ++fixture.sequence, Corner90Direction::Left));

    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kCorner90MotionConfirmationTimeoutMs + 50));
    fixture.update(corner90Line(
        ++fixture.sequence, Corner90Direction::Left));

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "stopped" &&
                snapshot.autonomousStatus.phase == "corner90_stall" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "Pivot sem giro confirmado deve parar os motores.");
}

void testCorner90StopsWhenVisionDoesNotReturn()
{
    MissionFixture fixture;
    fixture.telemetry.mpuOk = true;
    fixture.telemetry.gyroZDegPerSec = 20.0;
    fixture.update(corner90Line(
        ++fixture.sequence, Corner90Direction::Right));
    fixture.update(corner90Line(
        ++fixture.sequence, Corner90Direction::Right));
    fixture.update(corner90Line(
        ++fixture.sequence, Corner90Direction::Right));

    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kCorner90LineLossTimeoutMs + 50));
    fixture.update(lineLost(++fixture.sequence));

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "stopped" &&
                snapshot.autonomousStatus.phase == "corner90_line_lost" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "Pivot sem nova linha deve parar antes de girar indefinidamente.");
}

void testCorner90StopsAfterMaximumYaw()
{
    MissionFixture fixture;
    fixture.telemetry.mpuOk = true;
    fixture.telemetry.gyroZDegPerSec = 20.0;
    fixture.telemetry.yawZDeg = 0.0;
    fixture.update(corner90Line(
        ++fixture.sequence, Corner90Direction::Right));
    fixture.update(corner90Line(
        ++fixture.sequence, Corner90Direction::Right));
    fixture.update(corner90Line(
        ++fixture.sequence, Corner90Direction::Right));

    fixture.telemetry.yawZDeg = config::kCorner90MaximumYawDegrees + 1.0;
    fixture.update(corner90Line(
        ++fixture.sequence, Corner90Direction::Right));

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "stopped" &&
                snapshot.autonomousStatus.phase == "corner90_overturn" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "Pivot acima do limite angular deve parar os motores.");
}

void testValidVisualTrajectoryNeverUsesCounterRotation()
{
    MissionFixture fixture;
    CameraLineSnapshot trajectory = strongCurve(++fixture.sequence, 0.80);
    trajectory.trajectoryValid = true;
    trajectory.leftPreview = 0.86;
    trajectory.rightPreview = 0.61;
    fixture.update(trajectory);

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "tracking_near" &&
                snapshot.left == 0.86 && snapshot.right == 0.61,
            "Uma trajetória visual válida deve manter o diferencial contínuo sem contrarrotação.");
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

void testSingleGreenMarkersUseDirectedPivotAndAlignedExit()
{
    MissionFixture leftFixture;
    leftFixture.update(greenMarker(
        ++leftFixture.sequence, GreenTurnDecision::GuideLeft));
    RobotSnapshot snapshot = leftFixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "green_confirming" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "O primeiro verde forte deve parar para confirmar a direção.");
    leftFixture.update(greenMarker(
        ++leftFixture.sequence, GreenTurnDecision::GuideLeft));
    snapshot = leftFixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "green_confirming" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "O segundo verde forte deve manter os motores parados.");
    leftFixture.update(greenMarker(
        ++leftFixture.sequence, GreenTurnDecision::GuideLeft));
    snapshot = leftFixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "green_turn_left" &&
                snapshot.left == -config::kCorner90PivotStartPower &&
                snapshot.right == config::kCorner90PivotStartPower,
            "Verde à esquerda confirmado deve iniciar o pivô à esquerda.");

    leftFixture.update(corner90ExitAlignedLine(++leftFixture.sequence));
    leftFixture.update(corner90ExitAlignedLine(++leftFixture.sequence));
    snapshot = leftFixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "green_turn_left" &&
                snapshot.left == -config::kCorner90PivotRunPower &&
                snapshot.right == config::kCorner90PivotRunPower,
            "Após o verde sair, o pivô deve reduzir ao piso RUN antes da saída.");

    leftFixture.update(corner90ExitAlignedLine(++leftFixture.sequence));
    snapshot = leftFixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "tracking_near" &&
                snapshot.left > 0.0 && snapshot.right > 0.0,
            "Três imagens novas alinhadas devem devolver o verde ao tracking.");

    MissionFixture rightFixture;
    rightFixture.update(greenMarker(
        ++rightFixture.sequence, GreenTurnDecision::GuideRight));
    rightFixture.update(greenMarker(
        ++rightFixture.sequence, GreenTurnDecision::GuideRight));
    rightFixture.update(greenMarker(
        ++rightFixture.sequence, GreenTurnDecision::GuideRight));
    snapshot = rightFixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "green_turn_right" &&
                snapshot.left == config::kCorner90PivotStartPower &&
                snapshot.right == -config::kCorner90PivotStartPower,
            "Verde à direita confirmado deve iniciar o pivô à direita.");

    MissionFixture approachFixture;
    CameraLineSnapshot approach = greenMarker(
        ++approachFixture.sequence, GreenTurnDecision::Approach);
    approach.greenConfirmed = false;
    approach.leftPreview = 0.69;
    approach.rightPreview = 0.69;
    approachFixture.update(approach);
    snapshot = approachFixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "green_approach" &&
                snapshot.left == approach.leftPreview &&
                snapshot.right == approach.rightPreview,
            "A aproximação ao verde deve preservar a prévia reduzida da visão.");
}

void testGreenConfirmationCancelsInconsistentCandidateAndPreemptsCorner90()
{
    MissionFixture cancelFixture;
    cancelFixture.update(greenMarker(
        ++cancelFixture.sequence, GreenTurnDecision::GuideLeft));
    cancelFixture.update(trackedLine(++cancelFixture.sequence));
    RobotSnapshot snapshot = cancelFixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "green_cancelled" &&
                snapshot.left > 0.0 && snapshot.right > 0.0,
            "Verde inconsistente deve cancelar antes de iniciar o pivot.");

    MissionFixture priorityFixture;
    priorityFixture.update(corner90Line(
        ++priorityFixture.sequence, Corner90Direction::Right));
    priorityFixture.update(greenMarker(
        ++priorityFixture.sequence, GreenTurnDecision::GuideLeft));
    snapshot = priorityFixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "green_confirming" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "Candidato verde forte deve interromper a confirmação do Corner90.");
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
    fixture.update(greenMarker(
        ++fixture.sequence, GreenTurnDecision::TurnAround180));
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
    fixture.update(greenMarker(
        ++fixture.sequence, GreenTurnDecision::TurnAround180));
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
        testTotalLineLossTimeoutReportsReason();
        testGapStopsWhenCameraBecomesUnavailable();
        testMainMissionStopsWhenLineIpcIsNotFresh();
        testMainMissionStopsWhenEsp32BecomesUnavailable();
        testEmergencyStopWinsDuringCrossing();
        testLargeControlErrorNeverUsesCounterRotation();
        testCorner90UsesGeometryAndExitsAfterTwoAlignedFrames();
        testCorner90ConfirmationCancelsWeakOrInconsistentGeometry();
        testCorner90StopsWhenPivotDoesNotMove();
        testCorner90StopsWhenVisionDoesNotReturn();
        testCorner90StopsAfterMaximumYaw();
        testValidVisualTrajectoryNeverUsesCounterRotation();
        testFarOnlyUsesConservativeFallbackUntilNearReturns();
        testSingleGreenMarkersUseDirectedPivotAndAlignedExit();
        testGreenConfirmationCancelsInconsistentCandidateAndPreemptsCorner90();
        testInvalidOrUnconfirmedGreenDoesNotStopTracking();
        testDoubleGreenUsesImuAndEntersRecovery();
        std::cout << "23 testes da missão principal concluídos com sucesso."
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
