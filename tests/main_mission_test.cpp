#include "obr/config.h"
#include "obr/main_mission.h"
#include "obr/robot_state.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
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

bool closeTo(double actual, double expected)
{
    return std::abs(actual - expected) <= 1e-9;
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
    telemetry.gyroZDegPerSec = 0.0;
    return telemetry;
}

CameraLineSnapshot freshVision(
    GreenInterpretation interpretation,
    bool lineNearDetected = true)
{
    CameraLineSnapshot snapshot;
    snapshot.sourceFresh = true;
    snapshot.lineFollowerLeftPower = 0.68;
    snapshot.lineFollowerRightPower = 0.67;
    snapshot.lineNearDetected = lineNearDetected;
    snapshot.lineNearFinePosition = lineNearDetected
                                        ? 0.0
                                        : std::numeric_limits<double>::quiet_NaN();
    snapshot.lineSequence = 1;
    snapshot.greenCandidateCount =
        interpretation == GreenInterpretation::TurnAround180 ? 2 : 1;
    snapshot.greenPathBlackValid =
        interpretation == GreenInterpretation::Left ||
        interpretation == GreenInterpretation::Right ||
        interpretation == GreenInterpretation::TurnAround180;
    snapshot.greenConfirmed = interpretation != GreenInterpretation::None;
    snapshot.greenInterpretation = interpretation;
    return snapshot;
}

CameraLineSnapshot bottomVision(
    std::uint64_t sequence,
    bool farTrusted,
    bool mediumTrusted,
    const std::string& trustedDirection,
    bool normalSteeringValid = true)
{
    CameraLineSnapshot snapshot = freshVision(GreenInterpretation::None);
    snapshot.lineSequence = sequence;
    snapshot.farTrusted = farTrusted;
    snapshot.mediumTrusted = mediumTrusted;
    snapshot.trustedDirection = trustedDirection;
    double trustedPosition = 0.0;
    if (trustedDirection == "LEFT")
    {
        trustedPosition = -0.60;
    }
    else if (trustedDirection == "RIGHT")
    {
        trustedPosition = 0.60;
    }
    if (farTrusted)
    {
        snapshot.curveDiagnostics.farBandPosition = trustedPosition;
    }
    if (mediumTrusted)
    {
        snapshot.curveDiagnostics.mediumPosition = trustedPosition;
    }
    snapshot.curveDiagnostics.lineState = "LINE";
    snapshot.curveDiagnostics.virtualState = "NORMAL";
    if (farTrusted || mediumTrusted)
    {
        snapshot.lineControlSource =
            normalSteeringValid ? "virtual" : "virtual-sensor-recovery";
        snapshot.curveDiagnostics.finalSteering =
            normalSteeringValid
                ? 0.10
                : std::numeric_limits<double>::quiet_NaN();
    }
    else
    {
        // Simula o recovery inferior que já estaria ativo sem a câmera frontal.
        snapshot.lineControlSource = "virtual-blind-search";
        snapshot.lineFollowerLeftPower = 0.78;
        snapshot.lineFollowerRightPower = -0.72;
        snapshot.curveDiagnostics.finalSteering =
            std::numeric_limits<double>::quiet_NaN();
    }
    snapshot.normalSteeringValid =
        normalSteeringValid && (farTrusted || mediumTrusted);
    return snapshot;
}

CameraLineSnapshot gapVision(std::uint64_t sequence)
{
    CameraLineSnapshot snapshot = bottomVision(
        sequence, false, false, "NONE");
    snapshot.curveDiagnostics.lineState = "GAP";
    snapshot.lineControlSource = "gap-forward";
    snapshot.lineFollowerLeftPower = 0.75;
    snapshot.lineFollowerRightPower = 0.75;
    snapshot.normalSteeringValid = false;
    return snapshot;
}

CameraLineSnapshot pendingLineLoss(std::uint64_t sequence)
{
    CameraLineSnapshot snapshot = bottomVision(
        sequence, false, false, "NONE");
    snapshot.lineControlSource = "virtual-search-wait";
    snapshot.lineFollowerLeftPower = 0.0;
    snapshot.lineFollowerRightPower = 0.0;
    return snapshot;
}

ForwardLineSnapshot forwardVision(
    std::uint64_t sequence,
    double position,
    bool sourceFresh = true)
{
    ForwardLineSnapshot snapshot;
    snapshot.sourceFresh = sourceFresh;
    snapshot.visible = true;
    snapshot.position = position;
    snapshot.confidence = 0.01;
    snapshot.sequence = sequence;
    snapshot.timestamp = 1.0;

    const double limitedError = std::clamp(position, -0.36, 0.36);
    const double strength = std::min(1.0, std::abs(limitedError) / 0.36);
    const double outerPower = 0.75 + strength * (0.82 - 0.75);
    const double innerPower = 0.75 - strength * (0.75 - 0.66);
    if (limitedError < 0.0)
    {
        snapshot.normalLeftPower = innerPower;
        snapshot.normalRightPower = outerPower;
    }
    else if (limitedError > 0.0)
    {
        snapshot.normalLeftPower = outerPower;
        snapshot.normalRightPower = innerPower;
    }
    else
    {
        snapshot.normalLeftPower = 0.75;
        snapshot.normalRightPower = 0.75;
    }
    return snapshot;
}

ForwardLineSnapshot emptyForwardVision(
    std::uint64_t sequence,
    bool sourceFresh = true)
{
    ForwardLineSnapshot snapshot;
    snapshot.sourceFresh = sourceFresh;
    snapshot.visible = false;
    snapshot.sequence = sequence;
    snapshot.timestamp = 1.0;
    return snapshot;
}

CameraLineSnapshot virtualBlindVision(
    std::uint64_t sequence,
    bool backup)
{
    CameraLineSnapshot snapshot = bottomVision(
        sequence, false, false, "NONE");
    snapshot.lineControlSource = backup
                                     ? "virtual-blind-search-backup"
                                     : "virtual-blind-search";
    snapshot.lineFollowerLeftPower = backup ? -0.69 : 0.78;
    snapshot.lineFollowerRightPower = backup ? -0.69 : -0.72;
    return snapshot;
}

CameraLineSnapshot forwardAssistSearchVision(std::uint64_t sequence)
{
    CameraLineSnapshot snapshot = bottomVision(
        sequence, false, false, "NONE");
    snapshot.lineControlSource = "virtual-no-line";
    snapshot.lineFollowerLeftPower = 0.0;
    snapshot.lineFollowerRightPower = 0.0;
    return snapshot;
}

struct MissionFixture
{
    RobotState robotState;
    MainMission mission;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();

    MissionFixture()
    {
        robotState.setAutonomousMission(AutonomousMission::MainMission);
        robotState.startAutonomous();
    }

    RobotSnapshot update(
        const CameraLineSnapshot& snapshot,
        bool cameraReady = true,
        const ForwardLineSnapshot& forwardSnapshot = {})
    {
        mission.update(
            robotState,
            telemetry,
            cameraReady,
            snapshot,
            forwardSnapshot);
        return robotState.snapshot();
    }
};

void requireFollowingLine(
    const RobotSnapshot& snapshot,
    const std::string& context)
{
    require(
        snapshot.mode == "autonomous" &&
            closeTo(snapshot.left, 0.68) && closeTo(snapshot.right, 0.67),
        context + ": o segue-linha deve manter autoridade.");
    require(
        snapshot.autonomousStatus.phase == "line_following",
        context + ": a fase deve continuar no segue-linha.");
}

RobotSnapshot startReturnImu(
    MissionFixture& fixture,
    const CameraLineSnapshot& returnVision)
{
    CameraLineSnapshot alignedVision = returnVision;
    alignedVision.lineNearDetected = true;
    alignedVision.lineNearFinePosition = 0.0;
    alignedVision.mediumTrusted = true;
    alignedVision.curveDiagnostics.mediumPosition = 0.0;

    fixture.update(alignedVision);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kGreenTurnAroundRecognitionDelayMs + 20));
    fixture.update(alignedVision);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kGreenTurnAroundPostCenteringDelayMs + 20));
    fixture.update(alignedVision);

    const long long targetCounts = static_cast<long long>(std::ceil(
        config::kGreenTurnAroundForwardDistanceCm *
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount = targetCounts;
    fixture.telemetry.rightEncoderCount = targetCounts;
    fixture.update(returnVision);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kGreenTurnAroundForwardSettleMs + 20));
    return fixture.update(returnVision);
}

RobotSnapshot startReturnForward(
    MissionFixture& fixture,
    CameraLineSnapshot returnVision)
{
    returnVision.lineNearDetected = true;
    returnVision.lineNearFinePosition = 0.0;
    returnVision.mediumTrusted = true;
    returnVision.curveDiagnostics.mediumPosition = 0.0;

    RobotSnapshot snapshot = fixture.update(returnVision);
    require(
        snapshot.autonomousStatus.phase == "turnaround_recognition_delay" &&
            snapshot.left == 0.0 && snapshot.right == 0.0,
        "O retorno deve parar antes de iniciar o alinhamento.");

    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kGreenTurnAroundRecognitionDelayMs + 20));
    snapshot = fixture.update(returnVision);
    require(
        snapshot.autonomousStatus.phase == "turnaround_centered_delay" &&
            snapshot.left == 0.0 && snapshot.right == 0.0,
        "NEAR e MEDIUM alinhados devem iniciar a segunda pausa.");

    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kGreenTurnAroundPostCenteringDelayMs + 20));
    return fixture.update(returnVision);
}

void testNormalLineFollowerCommandsMotors()
{
    MissionFixture fixture;
    requireFollowingLine(
        fixture.update(freshVision(GreenInterpretation::None)),
        "Sem marcador verde");
}

CameraLineSnapshot normalLineVision(double leftPower, double rightPower)
{
    CameraLineSnapshot snapshot = freshVision(GreenInterpretation::None);
    snapshot.lineFollowerLeftPower = leftPower;
    snapshot.lineFollowerRightPower = rightPower;
    snapshot.lineControlSource = "fusion";
    snapshot.normalSteeringValid = true;
    snapshot.farTrusted = true;
    snapshot.curveDiagnostics.lineState = "LINE";
    snapshot.curveDiagnostics.virtualState = "NORMAL";
    return snapshot;
}

void testNormalLineFollowerCompensatesRampPower()
{
    MissionFixture uphillFixture;
    uphillFixture.telemetry.rampAngleDeg =
        config::kLineFollowingUphillThresholdDeg;
    RobotSnapshot snapshot = uphillFixture.update(normalLineVision(0.70, 0.76));
    require(
        closeTo(snapshot.left, 0.85) && closeTo(snapshot.right, 0.91) &&
            !snapshot.encoderSynchronizationAllowed,
        "Subida deve somar o mesmo offset aos dois lados do segue-linha NORMAL.");

    MissionFixture downhillFixture;
    downhillFixture.telemetry.rampAngleDeg =
        config::kLineFollowingDownhillThresholdDeg;
    snapshot = downhillFixture.update(normalLineVision(0.70, 0.76));
    require(
        closeTo(snapshot.left, 0.65) && closeTo(snapshot.right, 0.71) &&
            !snapshot.encoderSynchronizationAllowed,
        "Descida deve reduzir igualmente os dois lados do segue-linha NORMAL.");

    MissionFixture levelFixture;
    levelFixture.telemetry.rampAngleDeg = 2.0;
    snapshot = levelFixture.update(normalLineVision(0.70, 0.76));
    require(
        closeTo(snapshot.left, 0.70) && closeTo(snapshot.right, 0.76) &&
            !snapshot.encoderSynchronizationAllowed,
        "Inclinação entre os limiares não deve alterar o segue-linha.");
}

void testRampCompensationRequiresFreshValidImu()
{
    MissionFixture invalidFixture;
    invalidFixture.telemetry.rampAngleDeg = 12.0;
    invalidFixture.telemetry.mpuOk = false;
    RobotSnapshot snapshot = invalidFixture.update(normalLineVision(0.70, 0.76));
    require(
        closeTo(snapshot.left, 0.70) && closeTo(snapshot.right, 0.76) &&
            !snapshot.encoderSynchronizationAllowed,
        "IMU inválida não pode alterar a potência do segue-linha.");

    MissionFixture staleFixture;
    staleFixture.telemetry.rampAngleDeg = 12.0;
    staleFixture.telemetry.lastSensorAgeMs = config::kTurn90ImuFreshnessMs + 1;
    snapshot = staleFixture.update(normalLineVision(0.70, 0.76));
    require(
        closeTo(snapshot.left, 0.70) && closeTo(snapshot.right, 0.76) &&
            !snapshot.encoderSynchronizationAllowed,
        "Inclinação stale não pode alterar a potência do segue-linha.");
}

void testRampCompensationPreservesSpecialLineCommands()
{
    MissionFixture pivotFixture;
    pivotFixture.telemetry.rampAngleDeg = 12.0;
    RobotSnapshot snapshot = pivotFixture.update(normalLineVision(-0.30, 0.85));
    require(
        closeTo(snapshot.left, -0.30) && closeTo(snapshot.right, 0.85) &&
            !snapshot.encoderSynchronizationAllowed,
        "Pivot Fusion não pode receber compensação de rampa.");

    MissionFixture greenFixture;
    greenFixture.telemetry.rampAngleDeg = 12.0;
    CameraLineSnapshot green = normalLineVision(0.70, 0.76);
    green.curveDiagnostics.lineState = "GREEN";
    snapshot = greenFixture.update(green);
    require(
        closeTo(snapshot.left, 0.70) && closeTo(snapshot.right, 0.76) &&
            snapshot.encoderSynchronizationAllowed,
        "GREEN não pode receber compensação de rampa.");

    MissionFixture recoveryFixture;
    recoveryFixture.telemetry.rampAngleDeg = 12.0;
    CameraLineSnapshot recovery = normalLineVision(0.70, 0.76);
    recovery.curveDiagnostics.virtualState = "REORIENT_LEFT";
    snapshot = recoveryFixture.update(recovery);
    require(
        closeTo(snapshot.left, 0.70) && closeTo(snapshot.right, 0.76) &&
            snapshot.encoderSynchronizationAllowed,
        "Recovery não pode receber compensação de rampa.");
}

void testRampCompensationClampsFinalMotorCommands()
{
    MissionFixture fixture;
    fixture.telemetry.rampAngleDeg = 12.0;
    const RobotSnapshot snapshot = fixture.update(normalLineVision(0.90, 0.98));
    require(
        closeTo(snapshot.left, 1.0) && closeTo(snapshot.right, 1.0),
        "Compensação de subida deve respeitar o clamp final dos motores.");
}

void testNonReturnGreenDoesNotStartSequence()
{
    const GreenInterpretation interpretations[] = {
        GreenInterpretation::FalseMarker,
        GreenInterpretation::Ambiguous,
    };
    for (const GreenInterpretation interpretation : interpretations)
    {
        MissionFixture fixture;
        requireFollowingLine(
            fixture.update(freshVision(interpretation)),
            "Classificação diferente de retorno");
    }
}

void testLateralGreenUsesCameraCommandWithoutImuGate()
{
    const GreenInterpretation interpretations[] = {
        GreenInterpretation::Left,
        GreenInterpretation::Right,
    };

    for (const GreenInterpretation interpretation : interpretations)
    {
        MissionFixture fixture;
        fixture.telemetry.mpuOk = false;
        CameraLineSnapshot greenVision = freshVision(interpretation);
        greenVision.curveDiagnostics.lineState = "GREEN";

        requireFollowingLine(
            fixture.update(greenVision),
            "Verde lateral deve usar imediatamente o comando da câmera");
    }
}

void testReturnWaitsForRequiredSensors()
{
    MissionFixture fixture;
    fixture.telemetry.mpuOk = false;
    const RobotSnapshot snapshot = fixture.update(
        freshVision(GreenInterpretation::TurnAround180));
    require(
        snapshot.mode == "autonomous" && snapshot.left == 0.0 &&
            snapshot.right == 0.0,
        "Retorno sem IMU pronta deve manter os motores parados.");
    require(
        snapshot.autonomousStatus.phase == "turnaround_waiting_sensors",
        "Retorno sem IMU deve aguardar sensores.");
}

void testImuFailureAlwaysStopsReturn()
{
    MissionFixture fixture;
    const CameraLineSnapshot returnVision = freshVision(
        GreenInterpretation::TurnAround180,
        false);
    startReturnImu(fixture, returnVision);

    const double turnSign = config::kGreenTurnAroundTurnsRight ? 1.0 : -1.0;
    fixture.telemetry.yawZDeg = turnSign * 120.0;
    fixture.telemetry.mpuOk = false;
    const RobotSnapshot snapshot = fixture.update(returnVision);

    require(
        snapshot.mode == "stopped" && snapshot.left == 0.0 &&
            snapshot.right == 0.0 &&
            snapshot.autonomousStatus.phase == "turn_imu_lost",
        "Falha da IMU deve parar o retorno mesmo após bastante progresso.");
}

void testUnequalEncoderDistancesDoNotInterruptForwardStage()
{
    MissionFixture fixture;
    const CameraLineSnapshot returnVision = freshVision(
        GreenInterpretation::TurnAround180,
        false);
    startReturnForward(fixture, returnVision);

    fixture.telemetry.leftEncoderCount = static_cast<long long>(std::ceil(
        8.0 * config::kEncoderCountsPerCentimeter));
    fixture.telemetry.rightEncoderCount = static_cast<long long>(std::ceil(
        1.0 * config::kEncoderCountsPerCentimeter));
    const RobotSnapshot snapshot = fixture.update(returnVision);
    require(
        snapshot.mode == "autonomous" &&
            snapshot.autonomousStatus.phase == "turnaround_forward" &&
            closeTo(snapshot.left, config::kGreenTurnAroundForwardPower) &&
            closeTo(snapshot.right, config::kGreenTurnAroundForwardPower) &&
            snapshot.encoderSynchronizationAllowed,
        "Diferença entre encoders não deve mais interromper o avanço.");
}

void testMissingEncoderDataStopsAfterConfiguredSecond()
{
    MissionFixture fixture;
    const CameraLineSnapshot returnVision = freshVision(
        GreenInterpretation::TurnAround180,
        false);
    startReturnForward(fixture, returnVision);

    fixture.telemetry.lastSensorAgeMs =
        config::kGreenTurnAroundEncoderDataTimeoutMs + 1;
    const RobotSnapshot snapshot = fixture.update(returnVision);
    require(
        snapshot.mode == "stopped" && snapshot.left == 0.0 &&
            snapshot.right == 0.0,
        "Telemetria de encoder ausente deve parar o avanço.");
    require(
        snapshot.autonomousStatus.phase == "turnaround_encoder_lost",
        "A parada deve identificar a ausência dos dados dos encoders.");
}

void testReturnRunsConfiguredSequenceAndRestoresFollower()
{
    MissionFixture fixture;
    CameraLineSnapshot returnVision = freshVision(
        GreenInterpretation::TurnAround180,
        false);
    // Reproduz o comando assimétrico que podia existir no frame verde.
    // O retorno deve descartá-lo, parar e só avançar após as etapas iniciais.
    returnVision.lineFollowerLeftPower = -0.78;
    returnVision.lineFollowerRightPower = 0.78;

    RobotSnapshot snapshot = startReturnForward(fixture, returnVision);
    require(
        snapshot.autonomousStatus.phase == "turnaround_forward" &&
            closeTo(snapshot.left, config::kGreenTurnAroundForwardPower) &&
            closeTo(snapshot.right, config::kGreenTurnAroundForwardPower),
        "Após as duas pausas, o retorno deve iniciar o avanço configurado.");

    const long long targetCounts = static_cast<long long>(std::ceil(
        config::kGreenTurnAroundForwardDistanceCm *
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount = targetCounts;
    fixture.telemetry.rightEncoderCount = targetCounts;
    snapshot = fixture.update(returnVision);
    require(
        snapshot.autonomousStatus.phase == "turnaround_forward_settling" &&
            snapshot.left == 0.0 && snapshot.right == 0.0,
        "Após a distância configurada o retorno deve parar antes do giro.");

    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kGreenTurnAroundForwardSettleMs + 20));
    snapshot = fixture.update(returnVision);
    const double expectedTurnSign =
        config::kGreenTurnAroundTurnsRight ? 1.0 : -1.0;
    require(
        snapshot.autonomousStatus.phase == "turnaround_imu" &&
            closeTo(
                snapshot.left,
                expectedTurnSign * config::kTurn90CommandPower) &&
            closeTo(
                snapshot.right,
                -expectedTurnSign * config::kTurn90CommandPower),
        "Depois do avanço o retorno deve iniciar o giro configurado por IMU.");

    fixture.telemetry.yawZDeg =
        expectedTurnSign * config::kGreenTurnAroundImuDegrees;
    snapshot = fixture.update(returnVision);
    require(
        snapshot.autonomousStatus.phase == "turnaround_imu" &&
            snapshot.left == 0.0 && snapshot.right == 0.0,
        "Ao alcançar 150 graus o controlador deve estabilizar o giro.");

    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kTurn90SettleMs + 20));
    snapshot = fixture.update(returnVision);
    require(
        snapshot.autonomousStatus.phase == "turnaround_searching_line" &&
            closeTo(
                snapshot.left,
                expectedTurnSign *
                    config::kGreenTurnAroundLineSearchPower) &&
            closeTo(
                snapshot.right,
                -expectedTurnSign *
                    config::kGreenTurnAroundLineSearchPower),
        "Após estabilizar em 150 graus o pivot deve buscar a linha.");

    CameraLineSnapshot recoveredVision = freshVision(
        GreenInterpretation::None,
        true);
    for (int frame = 1;
         frame < config::kGreenTurnAroundLineReacquireFrames;
         ++frame)
    {
        snapshot = fixture.update(recoveredVision);
        require(
            snapshot.autonomousStatus.phase ==
                "turnaround_searching_line",
            "Uma leitura isolada do NEAR não deve encerrar o pivot.");
    }
    snapshot = fixture.update(recoveredVision);
    requireFollowingLine(snapshot, "Linha próxima recuperada");
}

void testReturnStopsBeforeAndAfterCentering()
{
    MissionFixture fixture;
    CameraLineSnapshot returnVision = freshVision(
        GreenInterpretation::TurnAround180,
        true);
    returnVision.lineNearFinePosition = 0.75;
    returnVision.mediumTrusted = true;
    returnVision.curveDiagnostics.mediumPosition = 0.75;

    RobotSnapshot snapshot = fixture.update(returnVision);
    require(
        snapshot.autonomousStatus.phase == "turnaround_recognition_delay" &&
            snapshot.left == 0.0 && snapshot.right == 0.0,
        "O reconhecimento do retorno deve zerar os motores por um segundo.");

    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kGreenTurnAroundRecognitionDelayMs + 20));
    snapshot = fixture.update(returnVision);
    require(
        snapshot.autonomousStatus.phase == "turnaround_centering" &&
            closeTo(
                snapshot.left,
                config::kGreenTurnAroundCenteringPower) &&
            closeTo(
                snapshot.right,
                -config::kGreenTurnAroundCenteringPower),
        "Depois da primeira pausa, o alinhamento deve girar no próprio eixo.");

    returnVision.lineNearFinePosition = 0.0;
    returnVision.curveDiagnostics.mediumPosition = 0.0;
    snapshot = fixture.update(returnVision);
    require(
        snapshot.autonomousStatus.phase == "turnaround_centered_delay" &&
            snapshot.left == 0.0 && snapshot.right == 0.0,
        "Ao alinhar NEAR e MEDIUM, os motores devem zerar novamente.");

    snapshot = fixture.update(returnVision);
    require(
        snapshot.autonomousStatus.phase == "turnaround_post_centering_delay" &&
            snapshot.left == 0.0 && snapshot.right == 0.0,
        "Durante a segunda espera o robô deve permanecer parado.");

    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kGreenTurnAroundPostCenteringDelayMs + 20));
    snapshot = fixture.update(returnVision);
    require(
        snapshot.autonomousStatus.phase == "turnaround_forward" &&
            closeTo(snapshot.left, config::kGreenTurnAroundForwardPower) &&
            closeTo(snapshot.right, config::kGreenTurnAroundForwardPower),
        "Somente após a segunda pausa o retorno deve avançar.");
}

void testReturnCenteringRequiresBothNearAndMedium()
{
    MissionFixture fixture;
    CameraLineSnapshot returnVision = freshVision(
        GreenInterpretation::TurnAround180,
        true);
    returnVision.lineNearFinePosition = 0.0;
    returnVision.mediumTrusted = true;
    returnVision.curveDiagnostics.mediumPosition = -0.75;
    fixture.update(returnVision);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kGreenTurnAroundRecognitionDelayMs + 20));

    RobotSnapshot snapshot = fixture.update(returnVision);
    require(
        snapshot.autonomousStatus.phase == "turnaround_centering" &&
            closeTo(
                snapshot.left,
                -config::kGreenTurnAroundCenteringPower) &&
            closeTo(
                snapshot.right,
                config::kGreenTurnAroundCenteringPower),
        "MEDIUM desalinhado deve manter o pivot mesmo com NEAR central.");

    returnVision.lineNearFinePosition = 0.75;
    returnVision.curveDiagnostics.mediumPosition = 0.0;
    snapshot = fixture.update(returnVision);
    require(
        snapshot.autonomousStatus.phase == "turnaround_centering" &&
            closeTo(
                snapshot.left,
                config::kGreenTurnAroundCenteringPower) &&
            closeTo(
                snapshot.right,
                -config::kGreenTurnAroundCenteringPower),
        "NEAR desalinhado deve manter o pivot mesmo com MEDIUM central.");

    returnVision.lineNearDetected = false;
    returnVision.lineNearFinePosition =
        std::numeric_limits<double>::quiet_NaN();
    snapshot = fixture.update(returnVision);
    require(
        snapshot.autonomousStatus.phase ==
                "turnaround_centering_waiting_line" &&
            snapshot.left == 0.0 && snapshot.right == 0.0,
        "Sem direção lateral válida, o alinhamento deve esperar parado.");

    returnVision.lineNearDetected = true;
    returnVision.lineNearFinePosition = 0.0;
    snapshot = fixture.update(returnVision);
    require(
        snapshot.autonomousStatus.phase == "turnaround_centered_delay" &&
            snapshot.left == 0.0 && snapshot.right == 0.0,
        "A segunda pausa só pode começar com NEAR e MEDIUM centralizados.");
}

void testReturnCenteringTimeoutReleasesConfiguredSequence()
{
    MissionFixture fixture;
    CameraLineSnapshot returnVision = freshVision(
        GreenInterpretation::TurnAround180,
        true);
    returnVision.lineNearFinePosition = 0.75;
    returnVision.mediumTrusted = true;
    returnVision.curveDiagnostics.mediumPosition = 0.75;
    fixture.update(returnVision);

    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kGreenTurnAroundRecognitionDelayMs + 20));
    fixture.update(returnVision);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kGreenTurnAroundCenteringTimeoutMs + 20));
    RobotSnapshot snapshot = fixture.update(returnVision);
    require(
        snapshot.autonomousStatus.phase ==
                "turnaround_centering_timeout_delay" &&
            snapshot.left == 0.0 && snapshot.right == 0.0,
        "O timeout deve parar o pivot antes da segunda espera.");

    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kGreenTurnAroundPostCenteringDelayMs + 20));
    snapshot = fixture.update(returnVision);
    require(
        snapshot.autonomousStatus.phase == "turnaround_forward" &&
            closeTo(snapshot.left, config::kGreenTurnAroundForwardPower) &&
            closeTo(snapshot.right, config::kGreenTurnAroundForwardPower),
        "Após timeout e segunda pausa, a sequência deve prosseguir.");
}

void testVisualSearchStopsAtAngularLimit()
{
    MissionFixture fixture;
    const CameraLineSnapshot returnVision = freshVision(
        GreenInterpretation::TurnAround180,
        false);
    startReturnImu(fixture, returnVision);

    const double turnSign = config::kGreenTurnAroundTurnsRight ? 1.0 : -1.0;
    fixture.telemetry.yawZDeg =
        turnSign * config::kGreenTurnAroundImuDegrees;
    fixture.update(returnVision);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kTurn90SettleMs + 20));
    fixture.update(returnVision);

    double limitedYaw = turnSign *
                        (config::kGreenTurnAroundImuDegrees +
                         config::kGreenTurnAroundLineSearchMaximumDegrees);
    if (limitedYaw > 180.0)
    {
        limitedYaw -= 360.0;
    }
    else if (limitedYaw < -180.0)
    {
        limitedYaw += 360.0;
    }
    fixture.telemetry.yawZDeg = limitedYaw;
    const RobotSnapshot snapshot = fixture.update(returnVision);

    require(
        snapshot.mode == "stopped" && snapshot.left == 0.0 &&
            snapshot.right == 0.0 &&
            snapshot.autonomousStatus.phase ==
                "turnaround_line_search_angle_limit",
        "A busca visual deve parar antes de completar outra volta.");
}

void testVisualSearchAcceptsValidatedFusionWithoutNear()
{
    MissionFixture fixture;
    const CameraLineSnapshot returnVision = freshVision(
        GreenInterpretation::TurnAround180,
        false);
    startReturnImu(fixture, returnVision);

    const double turnSign = config::kGreenTurnAroundTurnsRight ? 1.0 : -1.0;
    fixture.telemetry.yawZDeg =
        turnSign * config::kGreenTurnAroundImuDegrees;
    fixture.update(returnVision);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kTurn90SettleMs + 20));
    fixture.update(returnVision);

    CameraLineSnapshot fusionVision = freshVision(
        GreenInterpretation::None,
        false);
    fusionVision.lineControlSource = "fusion";
    fusionVision.normalSteeringValid = true;
    fusionVision.lineFollowerLeftPower = 0.72;
    fusionVision.lineFollowerRightPower = 0.72;

    RobotSnapshot snapshot = fixture.update(fusionVision);
    require(
        snapshot.autonomousStatus.phase == "turnaround_searching_line",
        "Uma confirmação Fusion isolada não deve encerrar o pivot.");

    snapshot = fixture.update(fusionVision);
    require(
        snapshot.mode == "autonomous" &&
            snapshot.autonomousStatus.phase == "line_following" &&
            closeTo(snapshot.left, fusionVision.lineFollowerLeftPower) &&
            closeTo(snapshot.right, fusionVision.lineFollowerRightPower),
        "Duas confirmações Fusion válidas devem devolver o controle ao seguidor.");
}

void testVisualSearchRejectsUnvalidatedFusionWithoutNear()
{
    MissionFixture fixture;
    const CameraLineSnapshot returnVision = freshVision(
        GreenInterpretation::TurnAround180,
        false);
    startReturnImu(fixture, returnVision);

    const double turnSign = config::kGreenTurnAroundTurnsRight ? 1.0 : -1.0;
    fixture.telemetry.yawZDeg =
        turnSign * config::kGreenTurnAroundImuDegrees;
    fixture.update(returnVision);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kTurn90SettleMs + 20));
    fixture.update(returnVision);

    CameraLineSnapshot invalidFusion = freshVision(
        GreenInterpretation::None,
        false);
    invalidFusion.lineControlSource = "fusion";
    invalidFusion.normalSteeringValid = false;

    RobotSnapshot snapshot = fixture.update(invalidFusion);
    snapshot = fixture.update(invalidFusion);
    require(
        snapshot.mode == "autonomous" &&
            snapshot.autonomousStatus.phase == "turnaround_searching_line" &&
            closeTo(
                snapshot.left,
                turnSign * config::kGreenTurnAroundLineSearchPower) &&
            closeTo(
                snapshot.right,
                -turnSign * config::kGreenTurnAroundLineSearchPower),
        "Fusion sem validação não pode encerrar a busca visual.");
}

void testForwardValidatorNeverOverridesBottomCommands()
{
    // Inclui steering NORMAL, curva com roda interna em ré, GAP e ambos os
    // movimentos do recovery existente. A frontal nunca substitui esses valores.
    for (const std::string state : {"PRESENT", "UNCERTAIN", "ABSENT"})
    {
        for (int scenario = 0; scenario < 6; ++scenario)
        {
            MissionFixture fixture;
            fixture.update(bottomVision(1, true, true, "LEFT"));
            CameraLineSnapshot bottom = bottomVision(2, true, true, "LEFT");
            if (scenario == 1)
            {
                bottom.lineControlSource = "fusion";
                bottom.lineFollowerLeftPower = -0.70;
                bottom.lineFollowerRightPower = 0.83;
                bottom.normalSteeringValid = true;
            }
            else if (scenario == 2)
            {
                bottom = gapVision(2);
                bottom.gapValidationDecision = "GAP";
            }
            else if (scenario == 3 || scenario == 4)
            {
                bottom = virtualBlindVision(2, scenario == 3);
                bottom.gapValidationDecision = "LOST";
            }
            else if (scenario == 5)
            {
                bottom = pendingLineLoss(2);
                bottom.gapValidationDecision = "CHECKING";
            }
            ForwardLineSnapshot forward = forwardVision(1, 0.90);
            forward.pathState = state;
            forward.confidence = 0.99;
            forward.referenceValid = true;
            forward.visible = state != "ABSENT";
            const RobotSnapshot result = fixture.update(bottom, true, forward);
            require(closeTo(result.left, bottom.lineFollowerLeftPower) &&
                        closeTo(result.right, bottom.lineFollowerRightPower),
                    "Frontal não pode substituir comando inferior em " + state);
            require(!result.autonomousStatus.forwardAssistEntryAllowed &&
                        result.autonomousStatus.gapValidationDecision == bottom.gapValidationDecision,
                    "Validador deve expor decisão sem possuir autoridade de motor.");
            require(!forward.normalCommandValid(),
                    "Potências frontais antigas nunca devem ser executáveis.");
        }
    }
}

void testStaleForwardDoesNotBlockNativeRecoveryOrNormalLine()
{
    MissionFixture fixture;
    const ForwardLineSnapshot stale = forwardVision(1, 0.9, false);
    fixture.update(bottomVision(1, true, true, "LEFT"));
    const CameraLineSnapshot lost = virtualBlindVision(2, true);
    RobotSnapshot result = fixture.update(lost, true, stale);
    require(closeTo(result.left, lost.lineFollowerLeftPower) &&
                closeTo(result.right, lost.lineFollowerRightPower),
            "Frontal antiga não pode bloquear indefinidamente o recovery inferior.");
    CameraLineSnapshot recovered = bottomVision(3, true, true, "RIGHT");
    result = fixture.update(recovered, true, stale);
    require(closeTo(result.left, recovered.lineFollowerLeftPower) &&
                closeTo(result.right, recovered.lineFollowerRightPower) &&
                result.autonomousStatus.forwardAssistState == "BOTTOM",
            "Inferior recuperada deve manter autoridade completa no primeiro frame.");
}

void testGapAndGreenKeepNativeAuthorityWithoutImu()
{
    for (bool green : {false, true})
    {
        MissionFixture fixture;
        fixture.telemetry.mpuOk = false;
        CameraLineSnapshot bottom = gapVision(1);
        if (green)
        {
            bottom.curveDiagnostics.lineState = "GREEN";
            bottom.lineControlSource = "fusion-green";
            bottom.lineFollowerLeftPower = -0.70;
            bottom.lineFollowerRightPower = 0.80;
        }
        const RobotSnapshot result = fixture.update(bottom, true, forwardVision(1, -0.7));
        require(closeTo(result.left, bottom.lineFollowerLeftPower) &&
                    closeTo(result.right, bottom.lineFollowerRightPower),
                "A validação frontal não cria gate de IMU para GAP ou verde inferior.");
    }
}

void testForwardIpcAcceptsGeometryWithoutPowersAndRejectsInvalidSources()
{
    const auto path = std::filesystem::temp_directory_path() /
        ("obr-forward-validation-" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()) + ".json");
    CameraMonitor monitor(path.string());
    const auto write = [&](int version, const std::string& state, bool present, double age)
    {
        const double now = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        std::ofstream output(path);
        output << std::setprecision(16)
               << "{\"forwardLineVisible\":true,\"forwardLinePosition\":0.2,"
               << "\"forwardLineConfidence\":1.0,\"forwardPathConfidence\":1.0,"
               << "\"forwardLineSequence\":1,\"forwardLineTimestamp\":" << now - age
               << ",\"forwardPathVersion\":" << version
               << ",\"forwardPathState\":\"" << state << "\""
               << ",\"forwardLinePresent\":" << (present ? "true" : "false") << "}";
    };
    write(2, "PRESENT", true, 0.0);
    auto source = monitor.forwardLineSnapshot();
    require(source.sourceFresh && source.pathState == "PRESENT" &&
                source.lineObservationValid() && !source.normalCommandValid(),
            "IPC geométrico deve ser aceito sem potências nem referência inferior.");
    for (const auto invalid : {0, 1, 2, 3, 4})
    {
        write(invalid == 0 ? 1 : 2, invalid == 1 ? "BAD" : "PRESENT",
              invalid != 2, invalid == 3 ? 2.0 : invalid == 4 ? -1.0 : 0.0);
        require(!monitor.forwardLineSnapshot().sourceFresh,
                "IPC antigo, estado inválido, presença inconsistente ou timestamp inválido deve falhar fechado.");
    }
    std::filesystem::remove(path);
    require(!monitor.forwardLineSnapshot().sourceFresh,
            "Arquivo ausente não pode reutilizar evidência frontal antiga.");
}

void testUnavailableCameraStopsMission()
{
    MissionFixture fixture;
    const RobotSnapshot snapshot = fixture.update(
        freshVision(GreenInterpretation::Left), false);
    require(
        snapshot.mode == "stopped" &&
            snapshot.left == 0.0 && snapshot.right == 0.0 &&
            snapshot.autonomousStatus.phase == "camera_not_ready",
        "Câmera indisponível deve encerrar a missão com motores zerados.");
}

void testObstacleTakesControlAndSurvivesCameraLoss()
{
    MissionFixture fixture;
    fixture.telemetry.ultrasonicDistanceCm = 6.0;

    RobotSnapshot snapshot = fixture.update(
        freshVision(GreenInterpretation::None));
    requireFollowingLine(snapshot, "Primeira confirmação do obstáculo");

    snapshot = fixture.update(freshVision(GreenInterpretation::None));
    require(
        snapshot.mode == "autonomous" && snapshot.left == 0.0 &&
            snapshot.right == 0.0 &&
            snapshot.autonomousStatus.phase == "obstacle_detected",
        "Duas leituras ultrassônicas devem parar antes de iniciar o desvio.");

    snapshot = fixture.update(
        freshVision(GreenInterpretation::None), false);
    require(
        snapshot.mode == "autonomous" && snapshot.left == 0.0 &&
            snapshot.right == 0.0 &&
            snapshot.autonomousStatus.phase == "obstacle_settling",
        "O desvio iniciado deve manter autoridade se a câmera ficar indisponível.");
}

void testConfirmedCourseMarkersControlOnlyExpectedPhase()
{
    MissionFixture fixture;
    CameraLineSnapshot red = freshVision(GreenInterpretation::None);
    red.courseMarkerConfirmed = true;
    red.courseMarker = CourseMarker::Red;

    RobotSnapshot snapshot = fixture.update(red);
    requireFollowingLine(
        snapshot,
        "A faixa vermelha não deve encerrar o percurso inicial");
    require(
        !fixture.mission.requiresRescueVision(),
        "O percurso inicial não deve ligar a visão de resgate.");

    CameraLineSnapshot gray = freshVision(GreenInterpretation::None);
    gray.courseMarkerConfirmed = true;
    gray.courseMarker = CourseMarker::Gray;
    snapshot = fixture.update(gray);
    require(
        snapshot.mode == "autonomous" && snapshot.left == 0.0 &&
            snapshot.right == 0.0 &&
            snapshot.autonomousStatus.phase == "rescue_area_entering",
        "A faixa cinza confirmada deve entrar no resgate com os motores parados.");
    require(
        fixture.mission.requiresRescueVision(),
        "A fase de resgate deve solicitar a visão frontal de vítimas.");

    fixture.mission.reset();
    require(
        !fixture.mission.requiresRescueVision(),
        "O reset deve restaurar o primeiro percurso sem visão pesada.");
}
}

int main()
{
    try
    {
        testNormalLineFollowerCommandsMotors();
        testNormalLineFollowerCompensatesRampPower();
        testRampCompensationRequiresFreshValidImu();
        testRampCompensationPreservesSpecialLineCommands();
        testRampCompensationClampsFinalMotorCommands();
        testNonReturnGreenDoesNotStartSequence();
        testLateralGreenUsesCameraCommandWithoutImuGate();
        testReturnWaitsForRequiredSensors();
        testImuFailureAlwaysStopsReturn();
        testUnequalEncoderDistancesDoNotInterruptForwardStage();
        testMissingEncoderDataStopsAfterConfiguredSecond();
        testReturnRunsConfiguredSequenceAndRestoresFollower();
        testReturnStopsBeforeAndAfterCentering();
        testReturnCenteringRequiresBothNearAndMedium();
        testReturnCenteringTimeoutReleasesConfiguredSequence();
        testVisualSearchStopsAtAngularLimit();
        testVisualSearchAcceptsValidatedFusionWithoutNear();
        testVisualSearchRejectsUnvalidatedFusionWithoutNear();
        testForwardValidatorNeverOverridesBottomCommands();
        testStaleForwardDoesNotBlockNativeRecoveryOrNormalLine();
        testGapAndGreenKeepNativeAuthorityWithoutImu();
        testForwardIpcAcceptsGeometryWithoutPowersAndRejectsInvalidSources();
        testUnavailableCameraStopsMission();
        testObstacleTakesControlAndSurvivesCameraLoss();
        testConfirmedCourseMarkersControlOnlyExpectedPhase();
        std::cout << "main_mission_test: OK\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "main_mission_test: " << error.what() << '\n';
        return 1;
    }
}
