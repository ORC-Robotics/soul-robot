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
    fixture.update(returnVision);
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

void testNormalLineFollowerCommandsMotors()
{
    MissionFixture fixture;
    requireFollowingLine(
        fixture.update(freshVision(GreenInterpretation::None)),
        "Sem marcador verde");
}

void testNonReturnGreenDoesNotStartSequence()
{
    const GreenInterpretation interpretations[] = {
        GreenInterpretation::FalseMarker,
        GreenInterpretation::Ambiguous,
        GreenInterpretation::Left,
        GreenInterpretation::Right,
    };
    for (const GreenInterpretation interpretation : interpretations)
    {
        MissionFixture fixture;
        requireFollowingLine(
            fixture.update(freshVision(interpretation)),
            "Classificação diferente de retorno");
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
    fixture.update(returnVision);

    fixture.telemetry.leftEncoderCount = static_cast<long long>(std::ceil(
        8.0 * config::kEncoderCountsPerCentimeter));
    fixture.telemetry.rightEncoderCount = static_cast<long long>(std::ceil(
        1.0 * config::kEncoderCountsPerCentimeter));
    const RobotSnapshot snapshot = fixture.update(returnVision);
    require(
        snapshot.mode == "autonomous" &&
            snapshot.autonomousStatus.phase == "turnaround_forward" &&
            closeTo(snapshot.left, config::kGreenTurnAroundForwardPower) &&
            closeTo(snapshot.right, config::kGreenTurnAroundForwardPower),
        "Diferença entre encoders não deve mais interromper o avanço.");
}

void testMissingEncoderDataStopsAfterConfiguredSecond()
{
    MissionFixture fixture;
    const CameraLineSnapshot returnVision = freshVision(
        GreenInterpretation::TurnAround180,
        false);
    fixture.update(returnVision);

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
    // O retorno deve descartá-lo e assumir o avanço reto imediatamente.
    returnVision.lineFollowerLeftPower = -0.78;
    returnVision.lineFollowerRightPower = 0.78;

    RobotSnapshot snapshot = fixture.update(returnVision);
    require(
        snapshot.autonomousStatus.phase == "turnaround_forward" &&
            closeTo(snapshot.left, config::kGreenTurnAroundForwardPower) &&
            closeTo(snapshot.right, config::kGreenTurnAroundForwardPower),
        "O retorno deve avançar reto no mesmo ciclo da detecção.");

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

void testReturnSkipsCenteringRegardlessOfNearPosition()
{
    for (const double nearPosition : {-0.75, 0.75})
    {
        MissionFixture fixture;
        CameraLineSnapshot returnVision = freshVision(
            GreenInterpretation::TurnAround180,
            true);
        returnVision.lineNearFinePosition = nearPosition;

        const RobotSnapshot snapshot = fixture.update(returnVision);
        require(
            snapshot.autonomousStatus.phase == "turnaround_forward" &&
                closeTo(
                    snapshot.left,
                    config::kGreenTurnAroundForwardPower) &&
                closeTo(
                    snapshot.right,
                    config::kGreenTurnAroundForwardPower),
            "O retorno deve ignorar o NEAR e avançar sem centralização.");
    }
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

RobotSnapshot startForwardSearch(
    MissionFixture& fixture,
    const std::string& direction)
{
    fixture.update(bottomVision(1, true, true, direction));
    return fixture.update(bottomVision(2, false, false, "NONE"));
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

void testForwardAssistSearchUsesImmediatelyPreviousTrustedDirection()
{
    MissionFixture fixture;
    const RobotSnapshot snapshot = startForwardSearch(fixture, "LEFT");
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
        bottomVision(2, false, false, "NONE"));
    require(
        noDirection.autonomousStatus.forwardAssistState == "BOTTOM",
        "Trust sem direção lateral confiável não pode iniciar a busca frontal.");
}

void testForwardLineStopsSpinImmediatelyAtEightAndThirtyDegrees()
{
    for (const double foundAtDegrees : {8.0, 30.0})
    {
        MissionFixture fixture;
        startForwardSearch(fixture, "LEFT");
        fixture.telemetry.yawZDeg = -foundAtDegrees;
        const ForwardLineSnapshot forward = forwardVision(1, -0.32);
        const RobotSnapshot snapshot = fixture.update(
            bottomVision(2, false, false, "NONE"),
            true,
            forward);
        require(
            snapshot.autonomousStatus.forwardAssistState ==
                    "FORWARD_FOLLOW" &&
                snapshot.left > 0.0 && snapshot.right > 0.0 &&
                closeTo(snapshot.left, forward.normalLeftPower) &&
                closeTo(snapshot.right, forward.normalRightPower),
            "O primeiro frame frontal válido deve trocar SPIN por avanço no mesmo ciclo.");
    }
}

void testForwardSearchStopsAtWrappedFortyFiveDegreeLimit()
{
    MissionFixture fixture;
    fixture.telemetry.yawZDeg = 170.0;
    startForwardSearch(fixture, "RIGHT");
    fixture.telemetry.yawZDeg = -145.0;
    const CameraLineSnapshot recovery = bottomVision(
        2, false, false, "NONE");
    const RobotSnapshot snapshot = fixture.update(recovery);
    require(
        snapshot.autonomousStatus.forwardAssistState == "BOTTOM" &&
            closeTo(snapshot.left, recovery.lineFollowerLeftPower) &&
            closeTo(snapshot.right, recovery.lineFollowerRightPower),
        "O limite de 45 graus deve considerar wrap-around e devolver ao recovery.");
}

void testBottomRecoveryStopsSearchImmediately()
{
    MissionFixture fixture;
    startForwardSearch(fixture, "RIGHT");
    const CameraLineSnapshot recovered = bottomVision(
        3, true, false, "RIGHT");
    const RobotSnapshot snapshot = fixture.update(recovered);
    require(
        snapshot.autonomousStatus.forwardAssistState == "BOTTOM" &&
            closeTo(snapshot.left, recovered.lineFollowerLeftPower) &&
            closeTo(snapshot.right, recovered.lineFollowerRightPower),
        "FAR trusted durante SEARCH deve parar o SPIN no mesmo ciclo.");
}

void testForwardFollowUsesOnlyNormalForwardPowerAndTwoStableFrames()
{
    MissionFixture fixture;
    startForwardSearch(fixture, "RIGHT");
    ForwardLineSnapshot forward = forwardVision(1, 1.0);
    RobotSnapshot snapshot = fixture.update(
        bottomVision(2, false, false, "NONE"), true, forward);
    require(
        snapshot.autonomousStatus.forwardAssistState == "FORWARD_FOLLOW" &&
            snapshot.left >= config::kForwardAssistNormalMinimumPower &&
            snapshot.right >= config::kForwardAssistNormalMinimumPower &&
            snapshot.left <= config::kForwardAssistNormalMaximumPower &&
            snapshot.right <= config::kForwardAssistNormalMaximumPower,
        "FORWARD_FOLLOW deve permanecer à frente e dentro do NORMAL.");

    snapshot = fixture.update(
        bottomVision(3, true, false, "RIGHT"), true, forward);
    require(
        snapshot.autonomousStatus.forwardAssistState == "FORWARD_FOLLOW" &&
            snapshot.autonomousStatus.bottomStableFrames == 1,
        "Um frame inferior estável ainda não deve retirar a frontal.");

    const CameraLineSnapshot secondStable = bottomVision(
        4, true, false, "RIGHT");
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
        bottomVision(2, false, false, "NONE"), true, forward);

    fixture.update(bottomVision(3, true, false, "RIGHT"), true, forward);
    RobotSnapshot snapshot = fixture.update(
        bottomVision(5, true, false, "RIGHT"), true, forward);
    require(
        snapshot.autonomousStatus.forwardAssistState == "FORWARD_FOLLOW" &&
            snapshot.autonomousStatus.bottomStableFrames == 1,
        "Uma lacuna de sequence deve reiniciar a confirmação da bottom.");

    const CameraLineSnapshot consecutive = bottomVision(
        6, true, false, "RIGHT");
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
        bottomVision(2, false, false, "NONE"),
        true,
        forwardVision(1, 0.20));

    fixture.telemetry.yawZDeg = 25.0;
    ForwardLineSnapshot stale = forwardVision(1, 0.20, false);
    const RobotSnapshot snapshot = fixture.update(
        bottomVision(2, false, false, "NONE"), true, stale);
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
        bottomVision(2, false, false, "NONE"),
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

void testGreenGapAndExistingCriticalTurnCancelForwardAuthority()
{
    for (const std::string lineState : {"GREEN", "GAP"})
    {
        MissionFixture fixture;
        startForwardSearch(fixture, "LEFT");
        CameraLineSnapshot higherPriority = bottomVision(
            3, false, false, "NONE");
        higherPriority.curveDiagnostics.lineState = lineState;
        higherPriority.lineFollowerLeftPower = 0.70;
        higherPriority.lineFollowerRightPower = 0.69;
        const RobotSnapshot snapshot = fixture.update(higherPriority);
        require(
            snapshot.autonomousStatus.forwardAssistState == "BOTTOM" &&
                closeTo(snapshot.left, higherPriority.lineFollowerLeftPower) &&
                closeTo(snapshot.right, higherPriority.lineFollowerRightPower),
            lineState + " deve cancelar a autoridade frontal.");
    }

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
            closeTo(critical.left, hardCorner.lineFollowerLeftPower) &&
            closeTo(critical.right, hardCorner.lineFollowerRightPower),
        "HARD CORNER/PIVOT/SPIN inferior não pode ser substituído pela frontal.");
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
        testNonReturnGreenDoesNotStartSequence();
        testReturnWaitsForRequiredSensors();
        testImuFailureAlwaysStopsReturn();
        testUnequalEncoderDistancesDoNotInterruptForwardStage();
        testMissingEncoderDataStopsAfterConfiguredSecond();
        testReturnRunsConfiguredSequenceAndRestoresFollower();
        testReturnSkipsCenteringRegardlessOfNearPosition();
        testVisualSearchStopsAtAngularLimit();
        testForwardAssistStaysOnBottomWhileTrusted();
        testForwardAssistSearchUsesImmediatelyPreviousTrustedDirection();
        testForwardLineStopsSpinImmediatelyAtEightAndThirtyDegrees();
        testForwardSearchStopsAtWrappedFortyFiveDegreeLimit();
        testBottomRecoveryStopsSearchImmediately();
        testForwardFollowUsesOnlyNormalForwardPowerAndTwoStableFrames();
        testBottomStableFramesMustHaveConsecutiveSequences();
        testForwardLossResumesSameSearchAttempt();
        testStaleForwardReadingNeverStopsSearch();
        testGreenGapAndExistingCriticalTurnCancelForwardAuthority();
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
