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
    bool lineNearDetected = true,
    bool lineNearAnyDetected = false)
{
    CameraLineSnapshot snapshot;
    snapshot.sourceFresh = true;
    snapshot.lineFollowerLeftPower = 0.68;
    snapshot.lineFollowerRightPower = 0.67;
    snapshot.lineNearDetected = lineNearDetected;
    snapshot.lineNearAnyDetected =
        lineNearDetected || lineNearAnyDetected;
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
        bool cameraReady = true)
    {
        mission.update(robotState, telemetry, cameraReady, snapshot);
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
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kGreenTurnAroundRecognitionStopMs + 20));
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
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kGreenTurnAroundRecognitionStopMs + 20));
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
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kGreenTurnAroundRecognitionStopMs + 20));
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

    RobotSnapshot snapshot = fixture.update(returnVision);
    require(
        snapshot.autonomousStatus.phase ==
                "turnaround_recognized_stopping" &&
            snapshot.left == 0.0 && snapshot.right == 0.0,
        "O retorno reconhecido deve primeiro parar os dois motores.");

    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kGreenTurnAroundRecognitionStopMs + 20));
    snapshot = fixture.update(returnVision);
    require(
        snapshot.autonomousStatus.phase == "turnaround_forward" &&
            closeTo(snapshot.left, config::kGreenTurnAroundForwardPower) &&
            closeTo(snapshot.right, config::kGreenTurnAroundForwardPower),
        "Depois da parada, o retorno deve avançar com os dois motores.");

    const long long targetCounts = static_cast<long long>(std::ceil(
        config::kGreenTurnAroundForwardDistanceCm *
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount = targetCounts;
    fixture.telemetry.rightEncoderCount = targetCounts;
    snapshot = fixture.update(returnVision);
    require(
        snapshot.autonomousStatus.phase == "turnaround_forward_settling" &&
            snapshot.left == 0.0 && snapshot.right == 0.0,
        "Após a distância configurada, o retorno deve parar antes do giro.");

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
        "Ao alcançar 125 graus o controlador deve estabilizar o giro.");

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
        "Após estabilizar em 125 graus o pivot deve buscar a linha.");

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

void testVisualSearchResumesLineFollowingAtAngularLimit()
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
        snapshot.mode == "autonomous" &&
            closeTo(snapshot.left, returnVision.lineFollowerLeftPower) &&
            closeTo(snapshot.right, returnVision.lineFollowerRightPower) &&
            snapshot.autonomousStatus.phase == "line_following",
        "A busca visual deve devolver o controle ao segue-linha em 200 graus.");
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

void testVisualSearchSlowsAndExtendsAfterSeeingNearSide()
{
    MissionFixture fixture;
    const CameraLineSnapshot returnVision = freshVision(
        GreenInterpretation::TurnAround180,
        false,
        false);
    startReturnImu(fixture, returnVision);

    const double turnSign = config::kGreenTurnAroundTurnsRight ? 1.0 : -1.0;
    fixture.telemetry.yawZDeg =
        turnSign * config::kGreenTurnAroundImuDegrees;
    fixture.update(returnVision);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kTurn90SettleMs + 20));
    fixture.update(returnVision);

    const auto searchYaw = [turnSign](double additionalDegrees)
    {
        double yaw = turnSign *
                     (config::kGreenTurnAroundImuDegrees +
                      additionalDegrees);
        if (yaw > 180.0)
        {
            yaw -= 360.0;
        }
        else if (yaw < -180.0)
        {
            yaw += 360.0;
        }
        return yaw;
    };

    CameraLineSnapshot sideVision = freshVision(
        GreenInterpretation::None,
        false,
        true);
    fixture.telemetry.yawZDeg = searchYaw(40.0);
    RobotSnapshot snapshot = fixture.update(sideVision);
    require(
        snapshot.autonomousStatus.phase == "turnaround_searching_line" &&
            closeTo(
                std::abs(snapshot.left),
                config::kGreenTurnAroundLineApproachPower) &&
            closeTo(
                std::abs(snapshot.right),
                config::kGreenTurnAroundLineApproachPower),
        "A linha lateral deve reduzir a potência sem encerrar o retorno.");

    fixture.telemetry.yawZDeg = searchYaw(65.0);
    snapshot = fixture.update(freshVision(
        GreenInterpretation::None,
        false,
        false));
    require(
        snapshot.autonomousStatus.phase == "turnaround_searching_line",
        "A aproximação deve continuar enquanto o total estiver abaixo de 200 graus.");

    const CameraLineSnapshot centeredVision = freshVision(
        GreenInterpretation::None,
        true,
        true);
    fixture.update(centeredVision);
    snapshot = fixture.update(centeredVision);
    requireFollowingLine(snapshot, "Linha centralizada após retorno");
}

void testVisualSearchResumesAtTwoHundredDegreesAfterSeeingNearSide()
{
    MissionFixture fixture;
    const CameraLineSnapshot returnVision = freshVision(
        GreenInterpretation::TurnAround180,
        false,
        false);
    startReturnImu(fixture, returnVision);

    const double turnSign = config::kGreenTurnAroundTurnsRight ? 1.0 : -1.0;
    fixture.telemetry.yawZDeg =
        turnSign * config::kGreenTurnAroundImuDegrees;
    fixture.update(returnVision);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kTurn90SettleMs + 20));
    fixture.update(returnVision);

    CameraLineSnapshot sideVision = freshVision(
        GreenInterpretation::None,
        false,
        true);
    fixture.telemetry.yawZDeg = turnSign * 170.0;
    fixture.update(sideVision);

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
        snapshot.mode == "autonomous" &&
            closeTo(snapshot.left, returnVision.lineFollowerLeftPower) &&
            closeTo(snapshot.right, returnVision.lineFollowerRightPower) &&
            snapshot.autonomousStatus.phase == "line_following",
        "A aproximação lateral deve retomar o segue-linha em aproximadamente 200 graus.");
}

void testActiveObstacleIgnoresCameraFailure()
{
    MissionFixture fixture;
    CameraLineSnapshot vision = freshVision(GreenInterpretation::None);
    fixture.telemetry.ultrasonicDistanceCm = 6.4;

    // Duas leituras consecutivas confirmam o obstáculo e tornam a sequência
    // exclusiva antes de simular qualquer falha da câmera inferior.
    fixture.update(vision);
    RobotSnapshot snapshot = fixture.update(vision);
    require(
        snapshot.mode == "autonomous" &&
            snapshot.autonomousStatus.phase == "obstacle_detected",
        "O obstáculo deve ser confirmado antes da falha visual simulada.");

    snapshot = fixture.update(vision, false);
    require(
        snapshot.mode == "autonomous" && snapshot.left == 0.0 &&
            snapshot.right == 0.0 &&
            snapshot.autonomousStatus.phase == "obstacle_settling",
        "Câmera indisponível não deve cancelar um desvio já iniciado.");

    vision.sourceFresh = false;
    snapshot = fixture.update(vision, true);
    require(
        snapshot.mode == "autonomous" &&
            snapshot.autonomousStatus.phase == "obstacle_settling",
        "IPC visual antigo não deve cancelar um desvio já iniciado.");
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
        testVisualSearchResumesLineFollowingAtAngularLimit();
        testVisualSearchSlowsAndExtendsAfterSeeingNearSide();
        testVisualSearchResumesAtTwoHundredDegreesAfterSeeingNearSide();
        testUnavailableCameraStopsMission();
        testActiveObstacleIgnoresCameraFailure();
        std::cout << "main_mission_test: OK\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "main_mission_test: " << error.what() << '\n';
        return 1;
    }
}
