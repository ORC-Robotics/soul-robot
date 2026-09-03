#include "obr/config.h"
#include "obr/main_mission.h"
#include "obr/robot_state.h"

#include <algorithm>
#include <chrono>
#include <cmath>
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

RobotSnapshot startForwardSearch(
    MissionFixture& fixture,
    const std::string& direction)
{
    fixture.update(bottomVision(1, true, true, direction));
    fixture.update(pendingLineLoss(2));
    return fixture.update(bottomVision(3, false, false, "NONE"));
}

void testForwardAssistStaysOnBottomWhileTrusted()
{
    MissionFixture fixture;
    const RobotSnapshot snapshot = fixture.update(
        bottomVision(1, true, true, "LEFT"));
    require(
        snapshot.autonomousStatus.forwardAssistState == "BOTTOM" &&
            closeTo(snapshot.left, 0.68) && closeTo(snapshot.right, 0.67),
        "FAR/MEDIUM trusted devem manter autoridade integral da bottom.");
}

void testForwardAssistSearchConfirmsLossWithRecentTrustedDirection()
{
    MissionFixture fixture;
    fixture.update(bottomVision(1, true, true, "LEFT"));
    const RobotSnapshot waiting = fixture.update(pendingLineLoss(2));
    require(
        waiting.autonomousStatus.forwardAssistState == "BOTTOM" &&
            waiting.autonomousStatus.forwardAssistEntryBlocker ==
                "WAITING_LOSS_CONFIRMATION" &&
            closeTo(waiting.left, 0.0) && closeTo(waiting.right, 0.0),
        "Uma perda isolada deve aguardar confirmação antes de SEARCH_SPIN.");

    const RobotSnapshot snapshot = fixture.update(
        bottomVision(3, false, false, "NONE"));
    require(
        snapshot.autonomousStatus.forwardAssistState == "SEARCH_SPIN" &&
            snapshot.autonomousStatus.forwardAssistDirection == "LEFT" &&
            closeTo(
                snapshot.left,
                -config::kForwardAssistSearchSpinPower) &&
            closeTo(
                snapshot.right,
                config::kForwardAssistSearchSpinPower),
        "A perda consecutiva após LEFT trusted deve iniciar SEARCH_SPIN LEFT.");

    MissionFixture noDirectionFixture;
    noDirectionFixture.update(bottomVision(1, true, true, "NONE"));
    const RobotSnapshot noDirection = noDirectionFixture.update(
        pendingLineLoss(2));
    require(
        noDirection.autonomousStatus.forwardAssistState == "BOTTOM" &&
            !noDirection.autonomousStatus.forwardAssistEntryAllowed &&
            noDirection.autonomousStatus.forwardAssistEntryBlocker ==
                "NO_LATCHED_DIRECTION",
        "Trust sem direção lateral confiável não pode iniciar a busca frontal.");
}

void testForwardAssistNonePreservesRightLatchUntilLoss()
{
    MissionFixture fixture;

    RobotSnapshot snapshot = fixture.update(
        bottomVision(1, false, true, "RIGHT"));
    require(
        snapshot.autonomousStatus.forwardAssistLatchedDirection == "RIGHT",
        "Uma direção RIGHT trusted deve ser memorizada.");

    snapshot = fixture.update(bottomVision(2, false, true, "NONE"));
    require(
        snapshot.autonomousStatus.forwardAssistLatchedDirection == "RIGHT",
        "Um frame trusted sem nova direção não pode apagar o latch RIGHT.");

    snapshot = fixture.update(bottomVision(3, false, true, "NONE"));
    require(
        snapshot.autonomousStatus.forwardAssistLatchedDirection == "RIGHT",
        "Vários frames NONE devem preservar o último lado RIGHT confiável.");

    snapshot = fixture.update(pendingLineLoss(4));
    require(
        snapshot.autonomousStatus.forwardAssistState == "BOTTOM" &&
            snapshot.autonomousStatus.forwardAssistEntryBlocker ==
                "WAITING_LOSS_CONFIRMATION",
        "O primeiro frame perdido deve preservar BOTTOM e o latch RIGHT.");

    snapshot = fixture.update(bottomVision(5, false, false, "NONE"));
    require(
        snapshot.autonomousStatus.forwardAssistState == "SEARCH_SPIN" &&
            snapshot.autonomousStatus.forwardAssistDirection == "RIGHT" &&
            snapshot.autonomousStatus.forwardAssistLatchedDirection ==
                "RIGHT" &&
            !snapshot.autonomousStatus.forwardAssistFarTrusted &&
            !snapshot.autonomousStatus.forwardAssistMediumTrusted &&
            snapshot.autonomousStatus.forwardAssistEntryAllowed &&
            snapshot.autonomousStatus.forwardAssistEntryBlocker == "NONE" &&
            closeTo(
                snapshot.left,
                config::kForwardAssistSearchSpinPower) &&
            closeTo(
                snapshot.right,
                -config::kForwardAssistSearchSpinPower),
        "A perda de FAR/MEDIUM deve iniciar SEARCH_SPIN RIGHT com o latch "
        "preservado durante os frames NONE.");
}

void testForwardAssistValidDirectionReplacesLatchAndNonePreservesIt()
{
    MissionFixture fixture;

    fixture.update(bottomVision(1, true, true, "RIGHT"));
    RobotSnapshot snapshot = fixture.update(
        bottomVision(2, true, true, "LEFT"));
    require(
        snapshot.autonomousStatus.forwardAssistLatchedDirection == "LEFT",
        "Uma nova direção LEFT trusted deve substituir o latch RIGHT.");

    snapshot = fixture.update(bottomVision(3, true, true, "NONE"));
    require(
        snapshot.autonomousStatus.forwardAssistLatchedDirection == "LEFT",
        "O primeiro frame NONE deve preservar o novo latch LEFT.");

    snapshot = fixture.update(bottomVision(4, true, true, "NONE"));
    require(
        snapshot.autonomousStatus.forwardAssistLatchedDirection == "LEFT",
        "Vários frames NONE devem preservar o último lado LEFT confiável.");

    snapshot = fixture.update(pendingLineLoss(5));
    require(
        snapshot.autonomousStatus.forwardAssistState == "BOTTOM" &&
            snapshot.autonomousStatus.forwardAssistEntryBlocker ==
                "WAITING_LOSS_CONFIRMATION",
        "O primeiro frame perdido deve preservar BOTTOM e o latch LEFT.");

    snapshot = fixture.update(bottomVision(6, false, false, "NONE"));
    require(
        snapshot.autonomousStatus.forwardAssistState == "SEARCH_SPIN" &&
            snapshot.autonomousStatus.forwardAssistDirection == "LEFT" &&
            snapshot.autonomousStatus.forwardAssistLatchedDirection ==
                "LEFT" &&
            snapshot.autonomousStatus.forwardAssistEntryAllowed &&
            snapshot.autonomousStatus.forwardAssistEntryBlocker == "NONE" &&
            closeTo(
                snapshot.left,
                -config::kForwardAssistSearchSpinPower) &&
            closeTo(
                snapshot.right,
                config::kForwardAssistSearchSpinPower),
        "Depois de RIGHT para LEFT, a perda deve iniciar SEARCH_SPIN LEFT.");
}

void testForwardAssistFarDirectionWinsWithoutExistingLatch()
{
    MissionFixture farLeftFixture;
    CameraLineSnapshot farLeft = bottomVision(
        1, true, true, "RIGHT");
    farLeft.curveDiagnostics.farBandPosition = -0.60;
    farLeft.curveDiagnostics.mediumPosition = 0.60;
    RobotSnapshot snapshot = farLeftFixture.update(farLeft);
    require(
        snapshot.autonomousStatus.forwardAssistLatchedDirection == "LEFT",
        "FAR LEFT deve vencer MEDIUM RIGHT ao criar o latch.");

    MissionFixture farRightFixture;
    CameraLineSnapshot farRight = bottomVision(
        1, true, true, "LEFT");
    farRight.curveDiagnostics.farBandPosition = 0.60;
    farRight.curveDiagnostics.mediumPosition = -0.60;
    snapshot = farRightFixture.update(farRight);
    require(
        snapshot.autonomousStatus.forwardAssistLatchedDirection == "RIGHT",
        "FAR RIGHT deve vencer MEDIUM LEFT ao criar o latch.");
}

void testForwardAssistConflictPreservesExistingLatch()
{
    MissionFixture fixture;
    fixture.update(bottomVision(1, true, true, "LEFT"));

    CameraLineSnapshot conflict = bottomVision(
        2, true, true, "RIGHT");
    conflict.curveDiagnostics.farBandPosition = -0.60;
    conflict.curveDiagnostics.mediumPosition = 0.60;
    RobotSnapshot snapshot = fixture.update(conflict);
    require(
        snapshot.autonomousStatus.forwardAssistLatchedDirection == "LEFT",
        "FAR LEFT com MEDIUM RIGHT deve preservar o latch LEFT.");

    conflict = bottomVision(3, true, true, "LEFT");
    conflict.curveDiagnostics.farBandPosition = 0.60;
    conflict.curveDiagnostics.mediumPosition = -0.60;
    snapshot = fixture.update(conflict);
    require(
        snapshot.autonomousStatus.forwardAssistLatchedDirection == "LEFT",
        "Direções FAR/MEDIUM conflitantes não podem inverter um latch existente.");
}

void testForwardAssistMediumFlipRequiresTwoConsecutiveFrames()
{
    MissionFixture fixture;
    fixture.update(bottomVision(1, true, true, "LEFT"));

    RobotSnapshot snapshot = fixture.update(
        bottomVision(2, false, true, "RIGHT"));
    require(
        snapshot.autonomousStatus.forwardAssistLatchedDirection == "LEFT",
        "Um único frame MEDIUM RIGHT não pode inverter o latch LEFT.");

    snapshot = fixture.update(bottomVision(3, false, true, "RIGHT"));
    require(
        snapshot.autonomousStatus.forwardAssistLatchedDirection == "RIGHT",
        "Dois frames MEDIUM RIGHT consecutivos devem confirmar a inversão.");

    CameraLineSnapshot farLeft = bottomVision(
        4, true, false, "LEFT");
    snapshot = fixture.update(farLeft);
    require(
        snapshot.autonomousStatus.forwardAssistLatchedDirection == "LEFT",
        "FAR LEFT válido deve atualizar imediatamente um latch RIGHT.");

    MissionFixture sequenceGapFixture;
    sequenceGapFixture.update(bottomVision(10, true, true, "LEFT"));
    sequenceGapFixture.update(bottomVision(11, false, true, "RIGHT"));
    snapshot = sequenceGapFixture.update(
        bottomVision(13, false, true, "RIGHT"));
    require(
        snapshot.autonomousStatus.forwardAssistLatchedDirection == "LEFT",
        "Um salto de sequence deve reiniciar a confirmação MEDIUM em 1/2.");

    snapshot = sequenceGapFixture.update(
        bottomVision(14, false, true, "RIGHT"));
    require(
        snapshot.autonomousStatus.forwardAssistLatchedDirection == "RIGHT",
        "Somente o próximo frame MEDIUM consecutivo deve completar 2/2.");
}

void testForwardAssistNoneBreaksMediumFlipButPreservesLatch()
{
    MissionFixture fixture;
    fixture.update(bottomVision(1, true, true, "LEFT"));
    fixture.update(bottomVision(2, false, true, "RIGHT"));

    RobotSnapshot snapshot = fixture.update(
        bottomVision(3, false, true, "NONE"));
    require(
        snapshot.autonomousStatus.forwardAssistLatchedDirection == "LEFT",
        "NONE deve preservar LEFT e interromper a confirmação MEDIUM.");

    snapshot = fixture.update(bottomVision(4, false, true, "RIGHT"));
    require(
        snapshot.autonomousStatus.forwardAssistLatchedDirection == "LEFT",
        "Após NONE, a confirmação MEDIUM deve recomeçar em 1/2.");

    snapshot = fixture.update(pendingLineLoss(5));
    require(
        snapshot.autonomousStatus.forwardAssistState == "BOTTOM" &&
            snapshot.autonomousStatus.forwardAssistEntryBlocker ==
                "WAITING_LOSS_CONFIRMATION",
        "O primeiro frame perdido não pode iniciar busca após um flip incompleto.");

    snapshot = fixture.update(bottomVision(6, false, false, "NONE"));
    require(
        snapshot.autonomousStatus.forwardAssistState == "SEARCH_SPIN" &&
            snapshot.autonomousStatus.forwardAssistDirection == "LEFT" &&
            snapshot.autonomousStatus.forwardAssistLatchedDirection ==
                "LEFT" &&
            snapshot.autonomousStatus.forwardAssistEntryAllowed &&
            snapshot.autonomousStatus.forwardAssistEntryBlocker == "NONE" &&
            closeTo(
                snapshot.left,
                -config::kForwardAssistSearchSpinPower) &&
            closeTo(
                snapshot.right,
                config::kForwardAssistSearchSpinPower),
        "A perda completa deve usar o latch LEFT preservado.");
}

void testForwardAssistKeepsValidNormalSteeringAcrossSequenceGap()
{
    MissionFixture fixture;
    CameraLineSnapshot trusted = bottomVision(
        40, false, true, "LEFT");
    trusted.curveDiagnostics.mediumPosition = -0.60;
    fixture.update(trusted);

    CameraLineSnapshot lost = bottomVision(
        43, false, false, "NONE");
    lost.lineNearDetected = true;
    lost.lineNearFinePosition = -0.72;
    lost.curveDiagnostics.nearFinePosition = -0.72;
    lost.curveDiagnostics.mediumPosition =
        std::numeric_limits<double>::quiet_NaN();
    lost.curveDiagnostics.farBandPosition =
        std::numeric_limits<double>::quiet_NaN();
    lost.curveDiagnostics.headingAngleDeg =
        std::numeric_limits<double>::quiet_NaN();
    lost.curveDiagnostics.finalSteering = 0.0;
    lost.lineControlSource = "virtual";
    lost.normalSteeringValid = true;
    lost.lineFollowerLeftPower = 0.75;
    lost.lineFollowerRightPower = 0.75;

    const RobotSnapshot snapshot = fixture.update(lost);
    require(
        snapshot.autonomousStatus.forwardAssistState == "BOTTOM" &&
            snapshot.autonomousStatus.forwardAssistLatchedDirection ==
                "LEFT" &&
            !snapshot.autonomousStatus.forwardAssistFarTrusted &&
            !snapshot.autonomousStatus.forwardAssistMediumTrusted &&
            !snapshot.autonomousStatus.forwardAssistEntryAllowed &&
            snapshot.autonomousStatus.forwardAssistEntryBlocker ==
                "NORMAL_STEERING_VALID" &&
            closeTo(snapshot.left, lost.lineFollowerLeftPower) &&
            closeTo(snapshot.right, lost.lineFollowerRightPower),
        "Steering NORMAL válido deve manter BOTTOM mesmo com salto de sequence.");
}

void testForwardAssistKeepsValidFusionPivotBeforeSearch()
{
    MissionFixture fixture;
    fixture.update(bottomVision(1, false, true, "LEFT"));

    CameraLineSnapshot fusionPivot = bottomVision(
        2, false, false, "NONE");
    fusionPivot.lineControlSource = "fusion";
    fusionPivot.normalSteeringValid = true;
    fusionPivot.curveDiagnostics.finalSteering = -1.0;
    fusionPivot.lineFollowerLeftPower = -0.72;
    fusionPivot.lineFollowerRightPower = 0.78;

    const RobotSnapshot snapshot = fixture.update(fusionPivot);
    require(
        snapshot.autonomousStatus.forwardAssistState == "BOTTOM" &&
            snapshot.autonomousStatus.forwardAssistEntryBlocker ==
                "NORMAL_STEERING_VALID" &&
            closeTo(snapshot.left, fusionPivot.lineFollowerLeftPower) &&
            closeTo(snapshot.right, fusionPivot.lineFollowerRightPower),
        "Pivot contínuo Fusion válido deve manter BOTTOM antes de SEARCH_SPIN.");
}

void testForwardLineStopsSpinImmediatelyAtEightAndThirtyDegrees()
{
    for (const double foundAtDegrees : {8.0, 30.0})
    {
        MissionFixture fixture;
        startForwardSearch(fixture, "LEFT");
        fixture.telemetry.yawZDeg = -foundAtDegrees;
        const ForwardLineSnapshot forward = forwardVision(1, -0.32);
        const CameraLineSnapshot gapCandidate = gapVision(3);
        const RobotSnapshot snapshot = fixture.update(
            gapCandidate,
            true,
            forward);
        require(
            snapshot.autonomousStatus.forwardAssistState ==
                    "FORWARD_FOLLOW" &&
                snapshot.autonomousStatus.forwardAssistGapCandidate &&
                snapshot.left > 0.0 && snapshot.right > 0.0 &&
                closeTo(snapshot.left, forward.normalLeftPower) &&
                closeTo(snapshot.right, forward.normalRightPower),
            "O primeiro frame frontal válido deve trocar SPIN por avanço no mesmo ciclo.");
    }
}

void testForwardSearchStopsAtWrappedSixtyFiveDegreeLimit()
{
    MissionFixture fixture;
    fixture.telemetry.yawZDeg = 170.0;
    startForwardSearch(fixture, "RIGHT");
    fixture.telemetry.yawZDeg = -125.0;
    const CameraLineSnapshot recovery = gapVision(3);
    const RobotSnapshot snapshot = fixture.update(recovery);
    require(
        snapshot.autonomousStatus.forwardAssistState == "BOTTOM" &&
            snapshot.autonomousStatus.forwardAssistEntryBlocker ==
                "SEARCH_ANGLE_LIMIT" &&
            snapshot.autonomousStatus.forwardAssistGapCandidate &&
            closeTo(snapshot.left, recovery.lineFollowerLeftPower) &&
            closeTo(snapshot.right, recovery.lineFollowerRightPower),
        "O limite de 65 graus deve considerar wrap-around e liberar GAP/recovery.");
}

void testBottomRecoveryRequiresTwoStableFramesDuringSearch()
{
    MissionFixture fixture;
    startForwardSearch(fixture, "RIGHT");
    const CameraLineSnapshot firstRecovered = bottomVision(
        4, false, true, "RIGHT");
    RobotSnapshot snapshot = fixture.update(firstRecovered);
    require(
        snapshot.autonomousStatus.forwardAssistState == "SEARCH_SPIN" &&
            snapshot.autonomousStatus.bottomStableFrames == 1 &&
            closeTo(
                snapshot.left,
                config::kForwardAssistSearchSpinPower) &&
            closeTo(
                snapshot.right,
                -config::kForwardAssistSearchSpinPower),
        "Um frame MEDIUM trusted não pode cancelar SEARCH_SPIN.");

    const CameraLineSnapshot secondRecovered = bottomVision(
        5, false, true, "RIGHT");
    snapshot = fixture.update(secondRecovered);
    require(
        snapshot.autonomousStatus.forwardAssistState == "BOTTOM" &&
            closeTo(snapshot.left, secondRecovered.lineFollowerLeftPower) &&
            closeTo(snapshot.right, secondRecovered.lineFollowerRightPower),
        "Dois frames MEDIUM trusted com steering normal devem devolver BOTTOM.");
}

void testBottomRecoveryFlickerResetsCounterDuringSearch()
{
    MissionFixture fixture;
    startForwardSearch(fixture, "LEFT");

    RobotSnapshot snapshot = fixture.update(
        bottomVision(4, false, true, "LEFT"));
    require(
        snapshot.autonomousStatus.forwardAssistState == "SEARCH_SPIN" &&
            snapshot.autonomousStatus.bottomStableFrames == 1,
        "O primeiro frame trusted deve registrar BOTTOM_STABLE 1/2.");

    snapshot = fixture.update(bottomVision(5, false, false, "NONE"));
    require(
        snapshot.autonomousStatus.forwardAssistState == "SEARCH_SPIN" &&
            snapshot.autonomousStatus.bottomStableFrames == 0 &&
            snapshot.autonomousStatus.forwardAssistDirection == "LEFT" &&
            closeTo(
                snapshot.left,
                -config::kForwardAssistSearchSpinPower) &&
            closeTo(
                snapshot.right,
                config::kForwardAssistSearchSpinPower),
        "Perder trust depois de 1/2 deve zerar o contador e manter SEARCH LEFT.");
}

void testSearchKeepsAuthorityOverGapAndTotalBottomLoss()
{
    MissionFixture gapFixture;
    gapFixture.telemetry.yawZDeg = 10.0;
    startForwardSearch(gapFixture, "LEFT");
    gapFixture.telemetry.yawZDeg = 22.0;
    const RobotSnapshot gap = gapFixture.update(gapVision(3));
    require(
        gap.autonomousStatus.forwardAssistState == "SEARCH_SPIN" &&
            gap.autonomousStatus.forwardAssistDirection == "LEFT" &&
            gap.autonomousStatus.forwardAssistLatchedDirection == "LEFT" &&
            gap.autonomousStatus.forwardAssistGapCandidate &&
            gap.autonomousStatus.bottomStableFrames == 0 &&
            closeTo(gap.autonomousStatus.forwardAssistYawDeltaDeg, 12.0) &&
            closeTo(
                gap.left,
                -config::kForwardAssistSearchSpinPower) &&
            closeTo(
                gap.right,
                config::kForwardAssistSearchSpinPower),
        "GAP posterior não pode expulsar SEARCH nem substituir seus motores.");

    MissionFixture lossFixture;
    startForwardSearch(lossFixture, "RIGHT");
    CameraLineSnapshot noBottomSensors = bottomVision(
        3, false, false, "NONE");
    noBottomSensors.lineNearDetected = false;
    noBottomSensors.lineNearFinePosition =
        std::numeric_limits<double>::quiet_NaN();
    noBottomSensors.curveDiagnostics.nearFinePosition =
        std::numeric_limits<double>::quiet_NaN();
    const RobotSnapshot totalLoss = lossFixture.update(noBottomSensors);
    require(
        totalLoss.autonomousStatus.forwardAssistState == "SEARCH_SPIN" &&
            !totalLoss.autonomousStatus.forwardAssistGapCandidate &&
            totalLoss.autonomousStatus.forwardAssistDirection == "RIGHT" &&
            closeTo(
                totalLoss.left,
                config::kForwardAssistSearchSpinPower) &&
            closeTo(
                totalLoss.right,
                -config::kForwardAssistSearchSpinPower),
        "Perda total de NEAR/FAR/MEDIUM deve manter o SEARCH já iniciado.");
}

void testForwardFollowUsesOnlyNormalForwardPowerAndTwoStableFrames()
{
    MissionFixture fixture;
    startForwardSearch(fixture, "RIGHT");
    ForwardLineSnapshot forward = forwardVision(1, 1.0);
    RobotSnapshot snapshot = fixture.update(
        bottomVision(4, false, false, "NONE"), true, forward);
    require(
        snapshot.autonomousStatus.forwardAssistState == "FORWARD_FOLLOW" &&
            snapshot.left >= config::kForwardAssistNormalMinimumPower &&
            snapshot.right >= config::kForwardAssistNormalMinimumPower &&
            snapshot.left <= config::kForwardAssistNormalMaximumPower &&
            snapshot.right <= config::kForwardAssistNormalMaximumPower,
        "FORWARD_FOLLOW deve permanecer à frente e dentro do NORMAL.");

    snapshot = fixture.update(
        bottomVision(5, true, false, "RIGHT"), true, forward);
    require(
        snapshot.autonomousStatus.forwardAssistState == "FORWARD_FOLLOW" &&
            snapshot.autonomousStatus.bottomStableFrames == 1,
        "Um frame inferior estável ainda não deve retirar a frontal.");

    const CameraLineSnapshot secondStable = bottomVision(
        6, true, false, "RIGHT");
    snapshot = fixture.update(secondStable, true, forward);
    require(
        snapshot.autonomousStatus.forwardAssistState == "BOTTOM" &&
            closeTo(snapshot.left, secondStable.lineFollowerLeftPower) &&
            closeTo(snapshot.right, secondStable.lineFollowerRightPower),
        "Dois frames inferiores estáveis devem devolver a autoridade à bottom.");
}

void testBottomStableFramesMustHaveConsecutiveSequences()
{
    MissionFixture fixture;
    startForwardSearch(fixture, "RIGHT");
    const ForwardLineSnapshot forward = forwardVision(1, 0.15);
    fixture.update(
        bottomVision(4, false, false, "NONE"), true, forward);

    fixture.update(bottomVision(5, true, false, "RIGHT"), true, forward);
    RobotSnapshot snapshot = fixture.update(
        bottomVision(7, true, false, "RIGHT"), true, forward);
    require(
        snapshot.autonomousStatus.forwardAssistState == "FORWARD_FOLLOW" &&
            snapshot.autonomousStatus.bottomStableFrames == 1,
        "Uma lacuna de sequence deve reiniciar a confirmação da bottom.");

    const CameraLineSnapshot consecutive = bottomVision(
        8, true, false, "RIGHT");
    snapshot = fixture.update(consecutive, true, forward);
    require(
        snapshot.autonomousStatus.forwardAssistState == "BOTTOM" &&
            closeTo(snapshot.left, consecutive.lineFollowerLeftPower) &&
            closeTo(snapshot.right, consecutive.lineFollowerRightPower),
        "Somente o segundo sequence consecutivo deve devolver a autoridade.");
}

void testForwardLossResumesSameSearchAttempt()
{
    MissionFixture fixture;
    fixture.telemetry.yawZDeg = 10.0;
    startForwardSearch(fixture, "LEFT");
    fixture.telemetry.yawZDeg = 22.0;
    fixture.update(
        bottomVision(4, false, false, "NONE"),
        true,
        forwardVision(1, 0.20));

    fixture.telemetry.yawZDeg = 25.0;
    ForwardLineSnapshot stale = forwardVision(1, 0.20, false);
    const RobotSnapshot snapshot = fixture.update(
        bottomVision(4, false, false, "NONE"), true, stale);
    require(
        snapshot.autonomousStatus.forwardAssistState == "SEARCH_SPIN" &&
            snapshot.autonomousStatus.forwardAssistDirection == "LEFT" &&
            closeTo(snapshot.autonomousStatus.forwardAssistYawDeltaDeg, 15.0) &&
            closeTo(
                snapshot.left,
                -config::kForwardAssistSearchSpinPower),
        "A perda frontal deve retomar direção, yawOrigin e orçamento originais.");
}

void testStaleForwardReadingNeverStopsSearch()
{
    MissionFixture fixture;
    startForwardSearch(fixture, "RIGHT");
    const RobotSnapshot snapshot = fixture.update(
        bottomVision(4, false, false, "NONE"),
        true,
        forwardVision(9, -0.40, false));
    require(
        snapshot.autonomousStatus.forwardAssistState == "SEARCH_SPIN" &&
            closeTo(
                snapshot.left,
                config::kForwardAssistSearchSpinPower) &&
            closeTo(
                snapshot.right,
                -config::kForwardAssistSearchSpinPower),
        "Leitura frontal stale deve ser ignorada durante SEARCH.");
}

void testGreenAndExistingCriticalTurnCancelForwardAuthority()
{
    MissionFixture greenFixture;
    startForwardSearch(greenFixture, "LEFT");
    CameraLineSnapshot greenPriority = bottomVision(
        3, false, false, "NONE");
    greenPriority.curveDiagnostics.lineState = "GREEN";
    greenPriority.lineFollowerLeftPower = 0.70;
    greenPriority.lineFollowerRightPower = 0.69;
    const RobotSnapshot green = greenFixture.update(greenPriority);
    require(
        green.autonomousStatus.forwardAssistState == "BOTTOM" &&
            closeTo(green.left, greenPriority.lineFollowerLeftPower) &&
            closeTo(green.right, greenPriority.lineFollowerRightPower),
        "GREEN deve continuar acima da autoridade frontal.");

    MissionFixture criticalFixture;
    criticalFixture.update(bottomVision(1, true, true, "LEFT"));
    CameraLineSnapshot hardCorner = bottomVision(
        2, false, false, "NONE");
    hardCorner.lineControlSource = "virtual";
    hardCorner.lineFollowerLeftPower = -0.72;
    hardCorner.lineFollowerRightPower = 0.72;
    const RobotSnapshot critical = criticalFixture.update(hardCorner);
    require(
        critical.autonomousStatus.forwardAssistState == "BOTTOM" &&
            !critical.autonomousStatus.forwardAssistEntryAllowed &&
            critical.autonomousStatus.forwardAssistEntryBlocker ==
                "BOTTOM_CRITICAL_TURN" &&
            closeTo(critical.left, hardCorner.lineFollowerLeftPower) &&
            closeTo(critical.right, hardCorner.lineFollowerRightPower),
        "HARD CORNER/PIVOT/SPIN inferior não pode ser substituído pela frontal.");
}

void testGapWithoutSearchPreservesExistingBehavior()
{
    MissionFixture fixture;
    const CameraLineSnapshot gapCommand = gapVision(1);
    const RobotSnapshot snapshot = fixture.update(gapCommand);
    require(
        snapshot.autonomousStatus.forwardAssistState == "BOTTOM" &&
            snapshot.autonomousStatus.forwardAssistGapCandidate &&
            snapshot.autonomousStatus.forwardAssistEntryBlocker ==
                "LINE_NOT_NORMAL" &&
            closeTo(snapshot.left, gapCommand.lineFollowerLeftPower) &&
            closeTo(snapshot.right, gapCommand.lineFollowerRightPower) &&
            snapshot.encoderSynchronizationAllowed,
        "GAP sem SEARCH ativo deve manter sua autoridade e seus motores atuais.");
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
        testForwardAssistStaysOnBottomWhileTrusted();
        testForwardAssistSearchConfirmsLossWithRecentTrustedDirection();
        testForwardAssistNonePreservesRightLatchUntilLoss();
        testForwardAssistValidDirectionReplacesLatchAndNonePreservesIt();
        testForwardAssistFarDirectionWinsWithoutExistingLatch();
        testForwardAssistConflictPreservesExistingLatch();
        testForwardAssistMediumFlipRequiresTwoConsecutiveFrames();
        testForwardAssistNoneBreaksMediumFlipButPreservesLatch();
        testForwardAssistKeepsValidNormalSteeringAcrossSequenceGap();
        testForwardAssistKeepsValidFusionPivotBeforeSearch();
        testForwardLineStopsSpinImmediatelyAtEightAndThirtyDegrees();
        testForwardSearchStopsAtWrappedSixtyFiveDegreeLimit();
        testBottomRecoveryRequiresTwoStableFramesDuringSearch();
        testBottomRecoveryFlickerResetsCounterDuringSearch();
        testSearchKeepsAuthorityOverGapAndTotalBottomLoss();
        testForwardFollowUsesOnlyNormalForwardPowerAndTwoStableFrames();
        testBottomStableFramesMustHaveConsecutiveSequences();
        testForwardLossResumesSameSearchAttempt();
        testStaleForwardReadingNeverStopsSearch();
        testGreenAndExistingCriticalTurnCancelForwardAuthority();
        testGapWithoutSearchPreservesExistingBehavior();
        testUnavailableCameraStopsMission();
        std::cout << "main_mission_test: OK\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "main_mission_test: " << error.what() << '\n';
        return 1;
    }
}
