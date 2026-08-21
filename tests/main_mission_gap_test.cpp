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
using Corner90Direction = BlackLineGeometryDirection;

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
    line.greenCandidateFrames = 2;
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

CameraLineSnapshot acceptedGreenWithLineOccluded(
    std::uint64_t sequence,
    GreenTurnDecision decision)
{
    CameraLineSnapshot line;
    line.sourceFresh = true;
    line.lineSequence = sequence;
    line.greenNearSeen = true;
    line.greenPathBlackValid = true;
    line.greenCandidateDecision = decision;
    line.greenCandidateFrames = 2;
    line.greenConfirmed = true;
    line.greenTurnDecision = decision;
    return line;
}

CameraLineSnapshot unconfirmedStrongGreen(
    std::uint64_t sequence,
    GreenTurnDecision decision)
{
    CameraLineSnapshot line = trackedLine(sequence);
    line.greenNearSeen = true;
    line.greenPathBlackValid = true;
    line.greenCandidateDecision = decision;
    line.greenCandidateFrames = 1;
    return line;
}

CameraLineSnapshot strongTrackedLine(
    std::uint64_t sequence,
    double error,
    double leftPower,
    double rightPower)
{
    CameraLineSnapshot line = trackedLine(sequence);
    line.nearError = error;
    line.farValid = true;
    line.farError = error;
    line.trajectoryValid = true;
    line.leftPreview = leftPower;
    line.rightPreview = rightPower;
    return line;
}

CameraLineSnapshot trajectoryOnlyTrackedLine(
    std::uint64_t sequence,
    double error,
    double leftPower,
    double rightPower)
{
    CameraLineSnapshot line = trackedLine(sequence);
    line.nearError = error;
    line.trajectoryValid = true;
    line.leftPreview = leftPower;
    line.rightPreview = rightPower;
    return line;
}

CameraLineSnapshot nearFarRecoveryLine(
    std::uint64_t sequence,
    double nearError,
    double farError,
    double correction)
{
    CameraLineSnapshot line = trackedLine(sequence);
    line.nearError = nearError;
    line.farValid = true;
    line.farError = farError;
    line.targetCorrection = correction;
    line.trajectoryValid = false;
    return line;
}

CameraLineSnapshot corner90Line(
    std::uint64_t sequence,
    BlackLineGeometryDirection direction)
{
    CameraLineSnapshot line = trackedLine(sequence);
    line.blackLineGeometryCandidate = true;
    line.blackLineGeometryDirection = direction;
    line.blackLineGeometryAngleDegrees =
        direction == BlackLineGeometryDirection::Left ? -90.0 : 90.0;
    line.blackLineGeometryConfidence = 0.90;
    line.blackLineGeometryState = "candidate";
    return line;
}

CameraLineSnapshot corner90ExitAlignedLine(std::uint64_t sequence)
{
    CameraLineSnapshot line = trackedLine(sequence);
    line.trajectoryValid = true;
    line.fitSampleCount = 8;
    line.lookaheadY = 0.65;
    line.blackLineGeometryExitAlignment = true;
    return line;
}

CameraLineSnapshot corner90PrematureExitLine(std::uint64_t sequence)
{
    CameraLineSnapshot line = trackedLine(sequence);
    line.trajectoryValid = true;
    line.fitSampleCount = 8;
    line.fitB = 0.50;
    line.lookaheadY = 0.65;
    line.blackLineGeometryExitAlignment = true;
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

void completeGreenTurnToVisualHandoff(
    MissionFixture& fixture,
    GreenTurnDecision decision,
    double completedDegrees = config::kGreenDirectionalTurnTargetDegrees)
{
    fixture.telemetry.mpuOk = true;
    fixture.update(greenMarker(++fixture.sequence, decision));
    fixture.update(lineLost(++fixture.sequence));
    fixture.telemetry.yawZDeg =
        decision == GreenTurnDecision::GuideLeft
            ? -completedDegrees
            : completedDegrees;
    fixture.telemetry.gyroZDegPerSec = 0.0;
    fixture.update(lineLost(++fixture.sequence));
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config::kTurn90SettleMs + 20));
    fixture.update(lineLost(++fixture.sequence));
    require(fixture.robotState.snapshot().autonomousStatus.phase ==
                "green_turn_visual_handoff",
            "O giro verde concluído deve iniciar o handoff visual.");
}

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

void testLineLossWithoutStrongMemoryStopsImmediately()
{
    MissionFixture fixture;
    fixture.update(lineLost(++fixture.sequence));

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "stopped" && snapshot.left == 0.0 &&
                snapshot.right == 0.0 &&
                snapshot.autonomousStatus.phase == "line_recovery_unavailable",
            "Perda sem uma direção visual forte deve parar sem busca cega.");
}

void testLineRecoveryMemoryPreservesLeftStraightAndRightDirections()
{
    struct RecoveryCase
    {
        double error;
        double leftPower;
        double rightPower;
        const char* direction;
    };
    const RecoveryCase cases[] = {
        {-0.35, 0.61, 0.86, "esquerda"},
        {0.00, 0.70, 0.70, "reta"},
        {0.35, 0.86, 0.61, "direita"},
    };

    for (const RecoveryCase& recoveryCase : cases)
    {
        MissionFixture fixture;
        fixture.telemetry.leftEncoderRate = 100.0;
        fixture.telemetry.rightEncoderRate = 80.0;
        fixture.update(strongTrackedLine(
            ++fixture.sequence,
            recoveryCase.error,
            recoveryCase.leftPower,
            recoveryCase.rightPower));
        fixture.update(lineLost(++fixture.sequence));

        const RobotSnapshot snapshot = fixture.robotState.snapshot();
        require(snapshot.autonomousStatus.phase == "line_recovery_memory" &&
                    snapshot.left > 0.0 && snapshot.right > 0.0 &&
                    snapshot.left <= config::kLineRecoveryMemoryMaximumPower &&
                    snapshot.right <= config::kLineRecoveryMemoryMaximumPower &&
                    snapshot.autonomousStatus.action.find(recoveryCase.direction) !=
                        std::string::npos,
                "A memória visual deve manter direção e comandos somente à frente.");
    }
}

void testLineRecoveryReacquiresFarAndNear()
{
    MissionFixture farFixture;
    farFixture.telemetry.leftEncoderRate = 100.0;
    farFixture.telemetry.rightEncoderRate = 80.0;
    farFixture.update(strongTrackedLine(
        ++farFixture.sequence, 0.20, 0.78, 0.62));
    farFixture.update(lineLost(++farFixture.sequence));
    farFixture.update(farOnlyLine(++farFixture.sequence, 0.25));
    RobotSnapshot snapshot = farFixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "fallback_far" &&
                snapshot.autonomousStatus.action.find("FAR reencontrada") !=
                    std::string::npos,
            "A FAR atual deve cancelar a memória visual antes do limite físico.");

    MissionFixture nearFixture;
    nearFixture.telemetry.leftEncoderRate = 100.0;
    nearFixture.telemetry.rightEncoderRate = 80.0;
    nearFixture.update(strongTrackedLine(
        ++nearFixture.sequence, -0.20, 0.62, 0.78));
    nearFixture.update(lineLost(++nearFixture.sequence));
    nearFixture.update(trackedLine(++nearFixture.sequence));
    snapshot = nearFixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "reacquiring_near",
            "A NEAR atual deve retornar pela confirmação visual existente.");
}

void testLineRecoveryHonorsGapGreenCornerAndIpcPriorities()
{
    MissionFixture gapFixture;
    gapFixture.telemetry.leftEncoderRate = 100.0;
    gapFixture.telemetry.rightEncoderRate = 80.0;
    gapFixture.update(strongTrackedLine(
        ++gapFixture.sequence, 0.0, 0.70, 0.70));
    gapFixture.update(lineLost(++gapFixture.sequence));
    gapFixture.update(gapWithoutNear(++gapFixture.sequence));
    require(gapFixture.robotState.snapshot().autonomousStatus.phase ==
                "crossing_gap",
            "Um gap atual deve cancelar a memória visual sem avanço cego.");

    MissionFixture greenFixture;
    greenFixture.telemetry.mpuOk = true;
    greenFixture.telemetry.leftEncoderRate = 100.0;
    greenFixture.telemetry.rightEncoderRate = 80.0;
    greenFixture.update(strongTrackedLine(
        ++greenFixture.sequence, 0.0, 0.70, 0.70));
    greenFixture.update(lineLost(++greenFixture.sequence));
    CameraLineSnapshot green = greenMarker(
        ++greenFixture.sequence, GreenTurnDecision::GuideLeft);
    greenFixture.update(green);
    require(greenFixture.robotState.snapshot().autonomousStatus.phase ==
                "green_turn_45_left",
            "Um verde forte deve ter prioridade sobre a memória visual.");

    MissionFixture cornerFixture;
    cornerFixture.telemetry.leftEncoderRate = 100.0;
    cornerFixture.telemetry.rightEncoderRate = 80.0;
    cornerFixture.update(strongTrackedLine(
        ++cornerFixture.sequence, 0.0, 0.70, 0.70));
    cornerFixture.update(lineLost(++cornerFixture.sequence));
    cornerFixture.update(corner90Line(
        ++cornerFixture.sequence, Corner90Direction::Left));
    cornerFixture.update(corner90Line(
        ++cornerFixture.sequence, Corner90Direction::Left));
    require(cornerFixture.robotState.snapshot().autonomousStatus.phase ==
                "black_line_geometry_left",
            "A geometria forte deve interromper a memória e iniciar o pivot imediato.");

    MissionFixture ipcFixture;
    ipcFixture.telemetry.leftEncoderRate = 100.0;
    ipcFixture.telemetry.rightEncoderRate = 80.0;
    ipcFixture.update(strongTrackedLine(
        ++ipcFixture.sequence, 0.0, 0.70, 0.70));
    ipcFixture.update(lineLost(++ipcFixture.sequence));
    CameraLineSnapshot stale = lineLost(++ipcFixture.sequence);
    stale.sourceFresh = false;
    ipcFixture.update(stale);
    require(ipcFixture.robotState.snapshot().autonomousStatus.phase ==
                "line_ipc_stale",
            "IPC visual antigo deve interromper a memória visual imediatamente.");
}

void testLineRecoveryStopsForEncoderAndDistanceLimits()
{
    MissionFixture unavailableFixture;
    unavailableFixture.update(strongTrackedLine(
        ++unavailableFixture.sequence, 0.0, 0.70, 0.70));
    unavailableFixture.telemetry.leftEncoderRate = std::nan("");
    unavailableFixture.update(lineLost(++unavailableFixture.sequence));
    require(unavailableFixture.robotState.snapshot().autonomousStatus.phase ==
                "line_recovery_encoder_unavailable",
            "Encoder não finito deve bloquear o início da memória visual.");

    MissionFixture distanceFixture;
    distanceFixture.telemetry.leftEncoderRate = 100.0;
    distanceFixture.telemetry.rightEncoderRate = 1.0;
    distanceFixture.update(strongTrackedLine(
        ++distanceFixture.sequence, 0.0, 0.70, 0.70));
    distanceFixture.update(lineLost(++distanceFixture.sequence));
    distanceFixture.telemetry.leftEncoderCount = static_cast<long long>(
        std::ceil(config::kEncoderCountsPerCentimeter * 10.0));
    distanceFixture.telemetry.rightEncoderCount = 1;
    distanceFixture.update(lineLost(++distanceFixture.sequence));
    require(distanceFixture.robotState.snapshot().autonomousStatus.phase ==
                "line_recovery_distance_limit",
            "A roda mais rápida deve limitar a memória visual a 100 mm.");
}

void testLineRecoveryStopsWithoutAnyEncoderProgress()
{
    MissionFixture fixture;
    fixture.telemetry.leftEncoderRate = 100.0;
    fixture.telemetry.rightEncoderRate = 100.0;
    fixture.update(strongTrackedLine(
        ++fixture.sequence, 0.0, 0.70, 0.70));
    fixture.update(lineLost(++fixture.sequence));
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kLineRecoveryMemoryEncoderStallTimeoutMs + 30));
    fixture.update(lineLost(++fixture.sequence));

    require(fixture.robotState.snapshot().autonomousStatus.phase ==
                "line_recovery_encoder_stall",
            "Sem progresso total dos encoders a memória visual deve parar.");
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

void testVisualTurnUsesOneFrameAndExitsOnCurrentLine()
{
    MissionFixture fixture;
    fixture.update(corner90Line(
        ++fixture.sequence, Corner90Direction::Right));
    RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "black_line_geometry_right" &&
                snapshot.left == config::kCorner90PivotStartPower &&
                snapshot.right == -config::kCorner90PivotStartPower,
            "Uma geometria forte deve iniciar o pivot imediato à direita.");

    fixture.telemetry.leftEncoderRate = 100.0;
    fixture.telemetry.rightEncoderRate = 100.0;
    fixture.update(corner90Line(
        ++fixture.sequence, Corner90Direction::Right));
    snapshot = fixture.robotState.snapshot();
    require(snapshot.left == 0.70 && snapshot.right == -0.70,
            "Encoder em movimento não pode reduzir o pivot geométrico para 0,61.");

    fixture.update(corner90ExitAlignedLine(++fixture.sequence));
    snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "tracking_near" &&
                snapshot.left > 0.0 && snapshot.right > 0.0,
            "Uma linha nova deve devolver imediatamente ao tracking proporcional.");
}

void testVisualTurnRejectsTiltedExit()
{
    MissionFixture fixture;
    fixture.update(corner90Line(
        ++fixture.sequence, Corner90Direction::Left));

    fixture.telemetry.leftEncoderRate = 100.0;
    fixture.telemetry.rightEncoderRate = 100.0;
    fixture.update(corner90PrematureExitLine(++fixture.sequence));

    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "black_line_geometry_left" &&
                snapshot.left == -config::kCorner90PivotStartPower &&
                snapshot.right == config::kCorner90PivotStartPower,
            "Uma trajetória ainda inclinada não pode devolver o robô em reto.");
}

void testCorner90RejectsWeakOrOffCenterGeometry()
{
    MissionFixture weakFixture;
    CameraLineSnapshot weak = corner90Line(
        ++weakFixture.sequence, Corner90Direction::Left);
    weak.blackLineGeometryAngleDegrees = -29.0;
    weakFixture.update(weak);
    require(weakFixture.robotState.snapshot().autonomousStatus.phase ==
                "tracking_near",
            "Ângulo abaixo de 30 graus não pode iniciar o giro visual.");

    CameraLineSnapshot tooWide = corner90Line(
        ++weakFixture.sequence, Corner90Direction::Left);
    tooWide.blackLineGeometryAngleDegrees = -141.0;
    weakFixture.update(tooWide);
    require(weakFixture.robotState.snapshot().autonomousStatus.phase ==
                "tracking_near",
            "Ângulo acima de 140 graus não pode iniciar o giro visual.");

    MissionFixture offsetFixture;
    CameraLineSnapshot offset = corner90Line(
        ++offsetFixture.sequence, Corner90Direction::Left);
    offset.nearError = 0.21;
    offsetFixture.update(offset);
    require(offsetFixture.robotState.snapshot().autonomousStatus.phase ==
                "tracking_near",
            "Linha acima da margem lateral não pode iniciar o pivot.");

    MissionFixture strongFixture;
    strongFixture.update(corner90Line(
        ++strongFixture.sequence, Corner90Direction::Left));
    RobotSnapshot strong = strongFixture.robotState.snapshot();
    require(strong.autonomousStatus.phase == "black_line_geometry_left" &&
                strong.left == -config::kCorner90PivotStartPower &&
                strong.right == config::kCorner90PivotStartPower,
            "Geometria forte e centralizada deve iniciar o pivot imediato.");

    MissionFixture lowConfidenceFixture;
    CameraLineSnapshot lowConfidence = corner90Line(
        ++lowConfidenceFixture.sequence, Corner90Direction::Right);
    lowConfidence.blackLineGeometryConfidence = 0.74;
    lowConfidenceFixture.update(lowConfidence);
    require(lowConfidenceFixture.robotState.snapshot().autonomousStatus.phase ==
                "tracking_near",
            "Confiança abaixo do limite não pode iniciar o pivot.");
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
    // Os encoders simulam um pivot que está acontecendo. Assim o teste valida
    // especificamente o timeout visual, e não o watchdog de roda parada.
    fixture.telemetry.leftEncoderRate = 100.0;
    fixture.telemetry.rightEncoderRate = 100.0;
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

void testSingleGreenMarkersUseImu45AndVisualAcquire()
{
    MissionFixture leftFixture;
    leftFixture.telemetry.mpuOk = true;
    leftFixture.update(acceptedGreenWithLineOccluded(
        ++leftFixture.sequence, GreenTurnDecision::GuideLeft));
    RobotSnapshot snapshot = leftFixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "green_turn_45_left" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "O primeiro verde forte deve parar para confirmar a direção.");
    leftFixture.update(lineLost(++leftFixture.sequence));
    snapshot = leftFixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "green_turn_45_left" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "O segundo verde forte deve preparar o giro IMU de 45 graus.");

    leftFixture.update(lineLost(++leftFixture.sequence));
    snapshot = leftFixture.robotState.snapshot();
    require(snapshot.left == -config::kTurn90CommandPower &&
                snapshot.right == config::kTurn90CommandPower,
            "O giro verde esquerdo deve aplicar pivot simétrico na potência configurada.");

    leftFixture.telemetry.yawZDeg = -config::kGreenDirectionalTurnTargetDegrees;
    leftFixture.telemetry.gyroZDegPerSec = 0.0;
    leftFixture.update(trackedLine(++leftFixture.sequence));
    snapshot = leftFixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "green_turn_45_left" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "O giro verde deve frear antes de concluir o alvo angular.");

    std::this_thread::sleep_for(
        std::chrono::milliseconds(config::kTurn90SettleMs + 20));
    leftFixture.update(trackedLine(++leftFixture.sequence));
    snapshot = leftFixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "green_turn_visual_handoff" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "Após 45 graus, a missão deve priorizar o handoff visual parada.");

    leftFixture.update(trajectoryOnlyTrackedLine(
        ++leftFixture.sequence, 0.15, 0.68, 0.72));
    snapshot = leftFixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "tracking_near" &&
                snapshot.left == 0.68 && snapshot.right == 0.72,
            "Uma trajetória nova válida sem FAR deve devolver direto ao Pure Pursuit.");

    MissionFixture rightFixture;
    rightFixture.telemetry.mpuOk = true;
    rightFixture.update(greenMarker(
        ++rightFixture.sequence, GreenTurnDecision::GuideRight));
    snapshot = rightFixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "green_turn_45_right" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "Verde à direita confirmado deve preparar o giro IMU de 45 graus.");

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

void testGreen45AcceptsOrientationToleranceWithoutCorrectionPulses()
{
    MissionFixture fixture;
    const double acceptedDegrees =
        config::kGreenDirectionalTurnTargetDegrees -
        config::kGreenDirectionalTurnCompletionToleranceDegrees + 1.0;
    fixture.telemetry.mpuOk = true;
    fixture.update(greenMarker(
        ++fixture.sequence, GreenTurnDecision::GuideLeft));
    fixture.update(lineLost(++fixture.sequence));
    fixture.telemetry.yawZDeg = -acceptedDegrees;
    // A projeção de frenagem pede parada antes dos 45 graus; depois a leitura
    // estacionária confirma que a margem visual basta para encerrar o giro.
    fixture.telemetry.gyroZDegPerSec = 80.0;
    fixture.update(lineLost(++fixture.sequence));
    fixture.telemetry.gyroZDegPerSec = 0.0;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config::kTurn90SettleMs + 20));
    fixture.update(lineLost(++fixture.sequence));
    require(fixture.robotState.snapshot().autonomousStatus.phase ==
                "green_turn_visual_handoff",
            "O giro verde de orientação deve aceitar a margem visual sem "
            "pulsos de correção.");
}

void testGreenTurnUsesOneForwardProbeThenStops()
{
    MissionFixture fixture;
    fixture.telemetry.mpuOk = true;
    fixture.telemetry.leftEncoderRate = 100.0;
    fixture.telemetry.rightEncoderRate = 100.0;
    fixture.update(greenMarker(++fixture.sequence, GreenTurnDecision::GuideLeft));
    fixture.update(lineLost(++fixture.sequence));

    fixture.telemetry.yawZDeg = -config::kGreenDirectionalTurnTargetDegrees;
    fixture.telemetry.gyroZDegPerSec = 0.0;
    fixture.update(lineLost(++fixture.sequence));
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config::kTurn90SettleMs + 20));
    fixture.update(lineLost(++fixture.sequence));
    require(fixture.robotState.snapshot().autonomousStatus.phase ==
                "green_turn_visual_handoff",
            "O giro verde concluído deve iniciar o handoff visual.");

    std::this_thread::sleep_for(
        std::chrono::milliseconds(config::kGreenTurnAcquireTimeoutMs + 20));
    fixture.update(lineLost(++fixture.sequence));
    RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "green_turn_forward_probe" &&
                snapshot.left == config::kGreenTurnForwardProbePower &&
                snapshot.right == config::kGreenTurnForwardProbePower,
            "Sem rota em 800 ms, o verde deve iniciar uma única sonda reta.");

    fixture.telemetry.leftEncoderCount = static_cast<long long>(std::ceil(
        config::kGreenTurnForwardProbeDistanceMm / 10.0 *
        config::kEncoderCountsPerCentimeter));
    fixture.update(lineLost(++fixture.sequence));
    snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "stopped" &&
                snapshot.autonomousStatus.phase == "green_turn_line_not_found" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "A sonda verde deve parar ao atingir 20 mm sem rota nova.");
}

void testGreenTurnNearFarRouteCancelsForwardProbe()
{
    MissionFixture fixture;
    fixture.telemetry.leftEncoderRate = 100.0;
    fixture.telemetry.rightEncoderRate = 100.0;
    completeGreenTurnToVisualHandoff(fixture, GreenTurnDecision::GuideLeft);

    std::this_thread::sleep_for(
        std::chrono::milliseconds(config::kGreenTurnAcquireTimeoutMs + 20));
    fixture.update(lineLost(++fixture.sequence));
    require(fixture.robotState.snapshot().autonomousStatus.phase ==
                "green_turn_forward_probe",
            "Sem rota visual, o teste deve iniciar a sonda pós-verde.");

    // Reproduz a situação da pista: NEAR e FAR enxergam a faixa inclinada,
    // enquanto o fit ainda não tem amostras suficientes para ser válido.
    fixture.update(nearFarRecoveryLine(
        ++fixture.sequence, 0.21, -0.54, -0.12));
    const RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.mode == "autonomous" &&
                snapshot.autonomousStatus.phase == "reacquiring_near" &&
                snapshot.left > 0.0 && snapshot.right > snapshot.left,
            "NEAR/FAR atuais devem cancelar a sonda e corrigir a rota, mesmo sem fit.");
}

void testGreenTurnVisualHandoffIgnoresGreenAndCorner90()
{
    MissionFixture fixture;
    completeGreenTurnToVisualHandoff(fixture, GreenTurnDecision::GuideLeft);

    fixture.update(corner90Line(++fixture.sequence, Corner90Direction::Right));
    RobotSnapshot snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "green_turn_visual_handoff" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "Um cotovelo residual não pode interromper o handoff pós-verde.");

    fixture.update(greenMarker(++fixture.sequence, GreenTurnDecision::GuideRight));
    snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "green_turn_visual_handoff" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "Um verde confirmado no cooldown não pode iniciar nova manobra.");

    fixture.update(trajectoryOnlyTrackedLine(
        ++fixture.sequence, -0.10, 0.64, 0.73));
    snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "tracking_near" &&
                snapshot.left == 0.64 && snapshot.right == 0.73,
            "A rota forte deve ter prioridade sobre verde e Corner90 no cooldown.");

    fixture.update(greenMarker(++fixture.sequence, GreenTurnDecision::GuideRight));
    snapshot = fixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "tracking_near" &&
                snapshot.left == 0.66 && snapshot.right == 0.66,
            "O cooldown deve ignorar novo verde mesmo após voltar ao tracking.");

    MissionFixture gapFixture;
    completeGreenTurnToVisualHandoff(gapFixture, GreenTurnDecision::GuideRight);
    gapFixture.update(gapLine(++gapFixture.sequence, 0.0));
    snapshot = gapFixture.robotState.snapshot();
    require(snapshot.mode == "stopped" &&
                snapshot.autonomousStatus.phase == "green_turn_line_not_found",
            "Um gap deve manter prioridade de segurança no handoff pós-verde.");
}

void testAcceptedGreenPreemptsCorner90WithoutRawCancellation()
{
    MissionFixture memoryFixture;
    memoryFixture.telemetry.leftEncoderRate = 100.0;
    memoryFixture.telemetry.rightEncoderRate = 100.0;
    memoryFixture.update(strongTrackedLine(
        ++memoryFixture.sequence, 0.0, 0.70, 0.70));
    memoryFixture.update(lineLost(++memoryFixture.sequence));
    CameraLineSnapshot rawGreen = lineLost(++memoryFixture.sequence);
    rawGreen.greenNearSeen = true;
    rawGreen.greenPathBlackValid = true;
    rawGreen.greenCandidateDecision = GreenTurnDecision::GuideLeft;
    rawGreen.greenCandidateFrames = 1;
    memoryFixture.update(rawGreen);
    require(memoryFixture.robotState.snapshot().autonomousStatus.phase ==
                "line_recovery_memory",
            "Candidato verde bruto não pode apagar a memória visual.");

    MissionFixture cancelFixture;
    cancelFixture.update(unconfirmedStrongGreen(
        ++cancelFixture.sequence, GreenTurnDecision::GuideLeft));
    cancelFixture.update(trackedLine(++cancelFixture.sequence));
    RobotSnapshot snapshot = cancelFixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "tracking_near" &&
                snapshot.left > 0.0 && snapshot.right > 0.0,
            "Verde inconsistente deve cancelar antes de iniciar o pivot.");

    MissionFixture priorityFixture;
    priorityFixture.telemetry.mpuOk = true;
    priorityFixture.update(corner90Line(
        ++priorityFixture.sequence, Corner90Direction::Right));
    priorityFixture.update(greenMarker(
        ++priorityFixture.sequence, GreenTurnDecision::GuideLeft));
    snapshot = priorityFixture.robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "green_turn_45_left" &&
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
        testLineLossWithoutStrongMemoryStopsImmediately();
        testLineRecoveryMemoryPreservesLeftStraightAndRightDirections();
        testLineRecoveryReacquiresFarAndNear();
        testLineRecoveryHonorsGapGreenCornerAndIpcPriorities();
        testLineRecoveryStopsForEncoderAndDistanceLimits();
        testLineRecoveryStopsWithoutAnyEncoderProgress();
        testGapStopsWhenCameraBecomesUnavailable();
        testMainMissionStopsWhenLineIpcIsNotFresh();
        testMainMissionStopsWhenEsp32BecomesUnavailable();
        testEmergencyStopWinsDuringCrossing();
        testLargeControlErrorNeverUsesCounterRotation();
        testVisualTurnUsesOneFrameAndExitsOnCurrentLine();
        testVisualTurnRejectsTiltedExit();
        testCorner90RejectsWeakOrOffCenterGeometry();
        testCorner90StopsWhenPivotDoesNotMove();
        testCorner90StopsWhenVisionDoesNotReturn();
        testValidVisualTrajectoryNeverUsesCounterRotation();
        testFarOnlyUsesConservativeFallbackUntilNearReturns();
        testSingleGreenMarkersUseImu45AndVisualAcquire();
        testGreen45AcceptsOrientationToleranceWithoutCorrectionPulses();
        testGreenTurnUsesOneForwardProbeThenStops();
        testGreenTurnNearFarRouteCancelsForwardProbe();
        testGreenTurnVisualHandoffIgnoresGreenAndCorner90();
        testAcceptedGreenPreemptsCorner90WithoutRawCancellation();
        testInvalidOrUnconfirmedGreenDoesNotStopTracking();
        testDoubleGreenUsesImuAndEntersRecovery();
        std::cout << "29 testes da missão principal concluídos com sucesso."
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
