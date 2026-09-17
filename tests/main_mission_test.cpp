#include "obr/config.h"
#include "obr/line_course_mission.h"
#include "obr/rescue_area_mission.h"
#include "obr/rescue_zone_triangle_mission.h"
#include "obr/rescue_exit_mission.h"
#include "obr/servo_routine.h"
#include "obr/silver_entry_maneuver.h"
// Expõe apenas as fases destas duas missões no teste para preparar a transição
// final sem executar uma coleta completa nem adicionar acesso no código do robô.
#define private public
#include "obr/main_mission.h"
#undef private
#include "obr/mission_controller.h"
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

void testBallIpcCandidateDirectionIsOptionalAndCannotConfirmTarget()
{
    const auto path = std::filesystem::temp_directory_path() /
                      "obr_rescue_candidate_test.json";
    CameraMonitor monitor({}, {}, path.string());
    const auto publish = [&](const std::string& hint, bool detected, double ageSeconds = 0.0) {
        std::ofstream file(path);
        file << std::setprecision(17)
             << "{\"active\":true,\"timestamp\":"
             << std::chrono::duration<double>(
                    std::chrono::system_clock::now().time_since_epoch()).count() - ageSeconds
             << ",\"targetSequence\":42,\"ballCandidateVisible\":true,\"ballDetected\":"
             << (detected ? "true" : "false")
             << ",\"targetLocked\":" << (detected ? "true" : "false")
             << ",\"ballType\":\"silver_ball\",\"ballTxDegrees\":10,"
                "\"ballDistanceCm\":20,\"ballRadiusPixels\":50,\"visibleAreaPixels\":500"
             << hint << '}';
    };
    publish(",\"candidateTxDegrees\":-20", false);
    auto ball = monitor.forwardBallSnapshot();
    require(ball.sourceFresh && ball.candidateVisible && !ball.detected &&
                !ball.targetLocked && ball.candidateTxDegrees == -20.0,
            "A candidata deve publicar direção sem liberar um alvo confirmado.");
    for (const std::string hint : {"", ",\"candidateTxDegrees\":null",
                                  ",\"candidateTxDegrees\":90"})
    {
        publish(hint, true);
        ball = monitor.forwardBallSnapshot();
        require(ball.detected && ball.targetLocked && std::isnan(ball.candidateTxDegrees),
                "Campo opcional ausente ou inválido não deve rejeitar uma detecção válida.");
    }
    publish(",\"candidateTxDegrees\":-20", false, 2.0);
    ball = monitor.forwardBallSnapshot();
    require(!ball.sourceFresh && std::isnan(ball.candidateTxDegrees),
            "Uma candidata antiga não pode orientar a busca atual.");
    std::filesystem::remove(path);
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
        interpretation == GreenInterpretation::None
            ? 0
            : (interpretation == GreenInterpretation::TurnAround180 ? 2 : 1);
    snapshot.greenPathBlackValid =
        interpretation == GreenInterpretation::Left ||
        interpretation == GreenInterpretation::Right ||
        interpretation == GreenInterpretation::TurnAround180;
    snapshot.greenPairCompatible =
        interpretation == GreenInterpretation::TurnAround180;
    snapshot.greenConfirmed = interpretation != GreenInterpretation::None;
    snapshot.greenRawInterpretation = interpretation;
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

void testRescueExitWristTransitionPreservesOtherChannels()
{
    for (const bool armRequested : {false, true})
    {
        for (const bool gripperRequested : {false, true})
        {
            MissionFixture fixture;
            ServoPose pose{config::kServoRoutineArmHomeDegrees + 10.0, 45.0, 73.0};
            require(fixture.robotState.setAutonomousServoPose(pose),
                    "A pose inicial deve ser aceita.");
            fixture.robotState.setAutonomousServoOutputEnabled(ServoId::Arm, armRequested);
            fixture.robotState.setAutonomousServoOutputEnabled(ServoId::Gripper, gripperRequested);
            fixture.mission.phase_ = MainMission::Phase::RescueArea;

            const auto before = fixture.robotState.snapshot();
            auto after = fixture.update({});
            require(after.servoPose.armDegrees == pose.armDegrees &&
                        after.servoPose.wristDegrees == pose.wristDegrees &&
                        after.servoPose.gripperDegrees == pose.gripperDegrees &&
                        after.servoCommandSequence == before.servoCommandSequence &&
                        !fixture.mission.requiresExitVision(),
                    "Antes da conclusão, a espera do resgate deve preservar a pose.");

            fixture.mission.rescueRoomMission_.phase_ = RescueRoomMission::Phase::Completed;
            after = fixture.update({});
            require(fixture.mission.requiresExitVision() &&
                        after.autonomousStatus.phase == "rescue_exit_starting" &&
                        after.servoPose.armDegrees == pose.armDegrees &&
                        after.servoPose.gripperDegrees == pose.gripperDegrees &&
                        after.servoPose.wristDegrees == config::kServoRoutineWristInternalDegrees &&
                        after.armServoRequested == armRequested &&
                        after.gripperServoRequested == gripperRequested &&
                        after.wristServoRequested &&
                        after.servoCommandSequence == before.servoCommandSequence + 1,
                    "A transição deve alterar só o pulso, preservando alvos e canais não relacionados.");

            const auto transitionSequence = after.servoCommandSequence;
            after = fixture.update({});
            require(after.servoCommandSequence == transitionSequence &&
                        after.armServoRequested == armRequested &&
                        after.gripperServoRequested == gripperRequested,
                    "A busca da saída não deve repetir o comando de servo.");
        }
    }
}

void testRescueExitWristUsesLatestRescuePose()
{
    MissionFixture fixture;
    fixture.mission.phase_ = MainMission::Phase::RescueArea;
    auto& room = fixture.mission.rescueRoomMission_;
    room.phase_ = RescueRoomMission::Phase::SearchVictim;
    room.finalVerification_ = true;
    room.finalSearchStarted_ = true;
    room.finalSearchStartedAt_ = std::chrono::steady_clock::now();
    room.finalSearchAccumulatedDegrees_ = config::kRescueFinalVictimSearchDegrees;
    require(fixture.robotState.setAutonomousServoPose(
                {config::kServoRoutineArmHomeDegrees + 5.0, 90.0, 73.0}),
            "A pose anterior à varredura deve ser aceita.");
    ForwardBallSnapshot ball;
    ball.sourceFresh = true;
    ball.targetSequence = fixture.mission.rescueBallTargetSequence(
        fixture.robotState.snapshot().autonomousRunSequence);
    fixture.mission.update(fixture.robotState, fixture.telemetry, true, {}, {}, ball,
                           fixture.robotState.snapshot().autonomousRunSequence);
    const auto after = fixture.robotState.snapshot();
    require(fixture.mission.requiresExitVision() &&
                after.servoPose.armDegrees == config::kServoRoutineArmStorageTransitionDegrees &&
                after.servoPose.gripperDegrees == config::kServoRoutineGripperClosedDegrees &&
                after.servoPose.wristDegrees == config::kServoRoutineWristInternalDegrees,
            "A transição deve preservar a pose publicada pela última atualização do resgate.");
}

void testRescueExitWristPreservesMechanicalGuard()
{
    MissionFixture fixture;
    ServoPose pose{config::kServoRoutineArmHomeDegrees, 45.0, 73.0};
    require(fixture.robotState.setAutonomousServoPose(pose), "A pose elevada deve ser aceita.");
    pose.armDegrees = config::kServoRoutineArmHomeDegrees - 1.0;
    require(fixture.robotState.setAutonomousServoPose(pose), "Baixar o braço sem girar o pulso deve ser permitido.");
    fixture.robotState.setAutonomousServoOutputEnabled(ServoId::Arm, false);
    fixture.robotState.setAutonomousServoOutputEnabled(ServoId::Gripper, false);
    const auto before = fixture.robotState.snapshot();
    fixture.mission.phase_ = MainMission::Phase::RescueArea;
    fixture.mission.rescueRoomMission_.phase_ = RescueRoomMission::Phase::Completed;
    const auto after = fixture.update({});
    require(fixture.mission.requiresExitVision() &&
                after.servoPose.armDegrees == pose.armDegrees &&
                after.servoPose.wristDegrees == pose.wristDegrees &&
                after.servoPose.gripperDegrees == pose.gripperDegrees &&
                !after.armServoRequested && !after.gripperServoRequested &&
                after.servoCommandSequence == before.servoCommandSequence,
            "A proteção mecânica deve rejeitar o pulso sem alterar pose ou solicitações.");
}

void testObstacleContinuationBandIpcFailsSafe()
{
    require(!CameraLineSnapshot{}.obstacleContinuationBand,
            "Um snapshot vazio deve bloquear a recuperação antecipada.");
    const auto path = std::filesystem::temp_directory_path() / "obr_obstacle_band_test.json";
    CameraMonitor monitor({}, {}, {}, path.string());
    std::uint64_t sequence = 0;
    const auto publish = [&](const std::string& field, double ageSeconds = 0.0) {
        std::ofstream file(path);
        file << std::setprecision(17)
             << "{\"lineFollowerLeftPower\":0.5,\"lineFollowerRightPower\":0.5,"
                "\"lineNearDetected\":true,\"greenPathBlackValid\":false,"
                "\"greenPairCompatible\":true,"
                "\"greenCandidateCount\":0,\"greenConfirmed\":false,"
                "\"greenRawInterpretation\":\"DIREITA\","
                "\"greenInterpretation\":\"SEM_DECISAO\",\"lineControlSource\":\"fusion\","
                "\"finalSteering\":0,\"lineSequence\":" << ++sequence
             << ",\"lineTimestamp\":" << std::chrono::duration<double>(
                    std::chrono::system_clock::now().time_since_epoch()).count() - ageSeconds
             << field << '}';
    };
    for (const std::string field : {"", ",\"obstacleContinuationBand\":false",
                                   ",\"obstacleContinuationBand\":null",
                                   ",\"obstacleContinuationBand\":1",
                                   ",\"obstacleContinuationBand\":\"true\"",
                                   ",\"obstacleContinuationBand\":trueX"})
    {
        publish(",\"obstacleContinuationBand\":true");
        require(monitor.lineSnapshot().obstacleContinuationBand,
                "O booleano true deve ser aceito no IPC atual.");
        publish(field);
        const auto line = monitor.lineSnapshot();
        require(line.sourceFresh && line.normalSteeringValid &&
                    line.greenPairCompatible &&
                    line.greenRawInterpretation == GreenInterpretation::Right &&
                    !line.obstacleContinuationBand,
                "Campo ausente ou inválido deve ser falso sem alterar Fusion válido.");
    }
    for (const std::string field : {"", ",\"greenFrontRoiValid\":false",
                                   ",\"greenFrontRoiValid\":null",
                                   ",\"greenFrontRoiValid\":1",
                                   ",\"greenFrontRoiValid\":\"true\""})
    {
        publish(",\"greenFrontRoiValid\":true");
        const auto upperValid = monitor.lineSnapshot();
        require(upperValid.sourceFresh && upperValid.greenFrontRoiValid &&
                    !upperValid.greenPathBlackValid && !upperValid.greenConfirmed,
                "Preto superior válido deve ser independente de decisão e confirmação.");
        publish(field);
        require(!monitor.lineSnapshot().greenFrontRoiValid,
                "Preto superior ausente ou inválido não deve reutilizar o cache.");
    }
    publish(",\"greenFrontRoiValid\":true",
            config::kCameraLineStatusTimeoutMs / 1000.0 + 1.0);
    require(!monitor.lineSnapshot().greenFrontRoiValid,
            "Preto superior de um frame vencido não deve conceder espera.");
    publish(",\"obstacleContinuationBand\":true",
            config::kCameraLineStatusTimeoutMs / 1000.0 + 1.0);
    require(!monitor.lineSnapshot().obstacleContinuationBand,
            "Um frame vencido não deve manter a faixa completa.");
    publish(",\"obstacleContinuationBand\":true");
    require(monitor.lineSnapshot().obstacleContinuationBand, "O cache deve conter uma faixa válida.");
    std::filesystem::remove(path);
    require(!monitor.lineSnapshot().obstacleContinuationBand,
            "IPC ausente não deve reutilizar a faixa do cache.");
}

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
        config::kLineFollowingUphillThresholdDeg - 0.1;
    RobotSnapshot snapshot = uphillFixture.update(normalLineVision(0.75, 0.75));
    require(
        closeTo(snapshot.left, 0.75) && closeTo(snapshot.right, 0.75) &&
            !snapshot.encoderSynchronizationAllowed,
        "Abaixo de 4 graus, a subida não deve alterar a potência.");

    MissionFixture steepUphillFixture;
    steepUphillFixture.telemetry.rampAngleDeg =
        config::kLineFollowingSteepUphillThresholdDeg;
    snapshot = steepUphillFixture.update(normalLineVision(0.75, 0.75));
    require(
        closeTo(snapshot.left, 0.85) && closeTo(snapshot.right, 0.85) &&
            !snapshot.encoderSynchronizationAllowed,
        "A partir de 4 graus, a subida deve liberar até 0,85.");

    MissionFixture downhillFixture;
    downhillFixture.telemetry.rampAngleDeg =
        config::kLineFollowingDownhillThresholdDeg;
    snapshot = downhillFixture.update(normalLineVision(0.70, 0.76));
    require(
        closeTo(snapshot.left, 0.65) && closeTo(snapshot.right, 0.71) &&
            !snapshot.encoderSynchronizationAllowed,
        "Descida deve reduzir igualmente os dois lados do segue-linha NORMAL.");

    MissionFixture levelFixture;
    levelFixture.telemetry.rampAngleDeg =
        config::kLineFollowingUphillThresholdDeg - 0.1;
    snapshot = levelFixture.update(normalLineVision(0.70, 0.76));
    require(
        closeTo(snapshot.left, 0.70) && closeTo(snapshot.right, 0.76) &&
            !snapshot.encoderSynchronizationAllowed,
        "Inclinação imediatamente abaixo de 4 graus não deve acelerar.");
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
        closeTo(snapshot.left, 0.85) && closeTo(snapshot.right, 0.85),
        "Compensação de subida forte deve respeitar o teto de 0,85.");
}

void testNonReturnGreenDoesNotStartSequence()
{
    MissionFixture falseFixture;
    RobotSnapshot falseSnapshot = falseFixture.update(
        freshVision(GreenInterpretation::FalseMarker));
    require(falseSnapshot.autonomousStatus.phase == "green_confirming",
            "Verde falso deve aguardar a janela curta antes do descarte.");
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kGreenConfirmationDecisionWaitMs + 20));
    CameraLineSnapshot laterFalse = freshVision(GreenInterpretation::FalseMarker);
    laterFalse.lineSequence = 2;
    requireFollowingLine(
        falseFixture.update(laterFalse),
        "Verde falso persistente");

    MissionFixture ambiguousFixture;
    const RobotSnapshot ambiguous = ambiguousFixture.update(
        freshVision(GreenInterpretation::Ambiguous));
    require(
        closeTo(ambiguous.left, 0.675) && closeTo(ambiguous.right, 0.675) &&
            ambiguous.autonomousStatus.phase == "green_confirming",
        "Candidato ambíguo deve manter a base e avançar reto na confirmação.");
}

void testLateralGreenWaitsForMeasuredConfirmation()
{
    const GreenInterpretation interpretations[] = {
        GreenInterpretation::Left,
        GreenInterpretation::Right,
    };

    for (const GreenInterpretation interpretation : interpretations)
    {
        MissionFixture fixture;
        CameraLineSnapshot greenVision = freshVision(interpretation);
        greenVision.curveDiagnostics.lineState = "GREEN";
        greenVision.lineFollowerLeftPower = 0.90;
        greenVision.lineFollowerRightPower = 0.80;

        RobotSnapshot snapshot = fixture.update(greenVision);
        require(
            closeTo((snapshot.left + snapshot.right) * 0.5,
                    config::kGreenConfirmationMaximumBasePower) &&
                snapshot.autonomousStatus.phase == "green_confirming",
            "Verde lateral deve limitar a base sem iniciar o giro antecipadamente.");

        require(closeTo(snapshot.left, snapshot.right) &&
                    !snapshot.encoderSynchronizationAllowed,
                "Os 30 mm devem ignorar o steering e usar rumo próprio.");
        fixture.telemetry.yawZDeg = 3.0;
        snapshot = fixture.update(greenVision);
        require(snapshot.left < snapshot.right &&
                    closeTo((snapshot.left + snapshot.right) * 0.5,
                            config::kGreenConfirmationMaximumBasePower),
                "A confirmação deve corrigir yaw sem aumentar a base.");
        fixture.telemetry.yawZDeg = -3.0;
        snapshot = fixture.update(greenVision);
        require(snapshot.left > snapshot.right,
                "A confirmação deve corrigir também o desvio oposto.");
        fixture.telemetry.yawZDeg = 0.0;

        const long long confirmationCounts = static_cast<long long>(std::ceil(
            config::kGreenConfirmationMaximumDistanceCm *
            config::kEncoderCountsPerCentimeter));
        fixture.telemetry.leftEncoderCount = confirmationCounts;
        fixture.telemetry.rightEncoderCount = confirmationCounts;
        snapshot = fixture.update(greenVision);
        require(
            closeTo(snapshot.left, config::kGreenLateralForwardPower) &&
                closeTo(snapshot.right, config::kGreenLateralForwardPower) &&
                snapshot.autonomousStatus.phase ==
                    (interpretation == GreenInterpretation::Left
                         ? "green_forward_left"
                         : "green_forward_right"),
            "LEFT/RIGHT só deve travar depois dos 30 mm de confirmação.");

        fixture.telemetry.yawZDeg = 3.0;
        snapshot = fixture.update(greenVision);
        require(
            snapshot.autonomousStatus.phase ==
                (interpretation == GreenInterpretation::Left
                     ? "green_forward_left"
                     : "green_forward_right") &&
                snapshot.left < snapshot.right &&
                !snapshot.encoderSynchronizationAllowed,
            "Yaw à direita deve corrigir a reta verde para a esquerda sem sincronismo duplicado.");

        fixture.telemetry.yawZDeg = -3.0;
        snapshot = fixture.update(greenVision);
        require(snapshot.left > snapshot.right,
                "Yaw à esquerda deve corrigir a reta verde para a direita.");
    }
}

void testGreenConfirmationPreservesLowBaseAndStopsOnImuLoss()
{
    MissionFixture fixture;
    CameraLineSnapshot green = freshVision(GreenInterpretation::Right);
    green.lineFollowerLeftPower = 0.18;
    green.lineFollowerRightPower = 0.02;
    RobotSnapshot snapshot = fixture.update(green);
    require(closeTo(snapshot.left, 0.10) && closeTo(snapshot.right, 0.10),
            "A confirmação não deve aumentar uma base já baixa.");
    fixture.telemetry.yawZDeg = 20.0;
    snapshot = fixture.update(green);
    require(snapshot.left >= 0.0 && snapshot.right >= 0.0 &&
                closeTo((snapshot.left + snapshot.right) * 0.5, 0.10),
            "A correção da confirmação não deve inverter roda nem aumentar a base.");
    fixture.telemetry.mpuOk = false;
    snapshot = fixture.update(green);
    require(snapshot.autonomousStatus.phase == "green_confirmation_imu_lost" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "Perder o rumo durante os 30 mm deve parar as duas rodas.");
}

void testTurnAroundOverridesProvisionalLateralGreen()
{
    MissionFixture fixture;
    RobotSnapshot snapshot = fixture.update(
        freshVision(GreenInterpretation::Left));
    require(snapshot.autonomousStatus.phase == "green_confirming",
            "LEFT provisório deve permanecer na janela de confirmação.");

    snapshot = fixture.update(
        freshVision(GreenInterpretation::TurnAround180));
    require(
        snapshot.autonomousStatus.phase == "turnaround_recognition_delay" &&
            snapshot.left == 0.0 && snapshot.right == 0.0,
        "O retorno de 180° deve substituir LEFT antes dos 30 mm.");
}

void testOppositeGreenDoesNotRestartOrPromoteLatchedEvent()
{
    MissionFixture fixture;
    CameraLineSnapshot rightCandidate =
        freshVision(GreenInterpretation::Right);
    rightCandidate.greenConfirmed = false;
    rightCandidate.greenPathBlackValid = true;
    rightCandidate.greenInterpretation = GreenInterpretation::None;
    CameraLineSnapshot right = freshVision(GreenInterpretation::Right);
    CameraLineSnapshot left = freshVision(GreenInterpretation::Left);

    RobotSnapshot snapshot = fixture.update(rightCandidate);
    require(snapshot.autonomousStatus.phase == "green_confirming",
            "RIGHT deve abrir uma única janela de confirmação.");

    const long long oneCentimeter = static_cast<long long>(std::ceil(
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount = oneCentimeter;
    fixture.telemetry.rightEncoderCount = oneCentimeter;
    snapshot = fixture.update(right);
    require(snapshot.autonomousStatus.phase == "green_confirming",
            "RIGHT confirmado deve atualizar o evento já aberto sem reiniciá-lo.");

    const long long twoCentimeters = static_cast<long long>(std::ceil(
        2.0 * config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount = twoCentimeters;
    fixture.telemetry.rightEncoderCount = twoCentimeters;
    snapshot = fixture.update(left);
    require(
        snapshot.autonomousStatus.phase == "green_confirming" &&
            snapshot.autonomousStatus.progressPercent >= 60.0,
        "LEFT distante não pode reiniciar distância nem zerar o evento RIGHT.");
    require(snapshot.autonomousStatus.phase != "turnaround_recognition_delay",
            "RIGHT histórico seguido de LEFT não pode criar um falso 180°.");

    CameraLineSnapshot incompatiblePair =
        freshVision(GreenInterpretation::TurnAround180);
    incompatiblePair.greenPairCompatible = false;
    snapshot = fixture.update(incompatiblePair);
    require(
        snapshot.autonomousStatus.phase == "green_confirming" &&
            snapshot.autonomousStatus.progressPercent >= 60.0,
        "Um RETORNO_180 sem par geométrico compatível deve permanecer no evento atual.");

    const long long confirmationCounts = static_cast<long long>(std::ceil(
        config::kGreenConfirmationMaximumDistanceCm *
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount = confirmationCounts;
    fixture.telemetry.rightEncoderCount = confirmationCounts;
    snapshot = fixture.update(left);
    require(
        snapshot.autonomousStatus.phase == "green_forward_right",
        "O evento latched deve preservar RIGHT mesmo se outro LEFT aparecer adiante.");
}

void testConfirmedLateralGreenCannotReturnToFalse()
{
    MissionFixture fixture;
    RobotSnapshot snapshot = fixture.update(
        freshVision(GreenInterpretation::Right));
    require(snapshot.autonomousStatus.phase == "green_confirming",
            "RIGHT confirmado deve permanecer na confirmação medida.");

    const long long oneCentimeter = static_cast<long long>(std::ceil(
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount = oneCentimeter;
    fixture.telemetry.rightEncoderCount = oneCentimeter;
    snapshot = fixture.update(freshVision(GreenInterpretation::FalseMarker));
    require(snapshot.autonomousStatus.phase == "green_confirming",
            "FALSE não pode apagar RIGHT depois de sua confirmação.");

    const long long twoCentimeters = static_cast<long long>(std::ceil(
        2.0 * config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount = twoCentimeters;
    fixture.telemetry.rightEncoderCount = twoCentimeters;
    snapshot = fixture.update(freshVision(GreenInterpretation::None));
    require(
        snapshot.autonomousStatus.phase == "green_confirming" &&
            snapshot.autonomousStatus.progressPercent >= 60.0,
        "Ausência temporária não pode apagar nem reiniciar RIGHT confirmado.");

    const long long confirmationCounts = static_cast<long long>(std::ceil(
        config::kGreenConfirmationMaximumDistanceCm *
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount = confirmationCounts;
    fixture.telemetry.rightEncoderCount = confirmationCounts;
    snapshot = fixture.update(freshVision(GreenInterpretation::None));
    require(snapshot.autonomousStatus.phase == "green_forward_right",
            "RIGHT confirmado deve concluir mesmo sem verde nos últimos frames.");
}

void testRetainedLateralConfirmationPrecedesMissingCandidate()
{
    CameraLineSnapshot retainedRight =
        freshVision(GreenInterpretation::Right);
    retainedRight.greenCandidateCount = 0;
    retainedRight.greenPathBlackValid = false;
    retainedRight.greenRawInterpretation = GreenInterpretation::None;

    MissionFixture retentionOnlyFixture;
    requireFollowingLine(
        retentionOnlyFixture.update(retainedRight),
        "Confirmação retida sem evento ativo");

    MissionFixture fixture;
    CameraLineSnapshot rightCandidate =
        freshVision(GreenInterpretation::Right);
    rightCandidate.greenConfirmed = false;
    rightCandidate.greenPathBlackValid = true;
    rightCandidate.greenInterpretation = GreenInterpretation::None;

    RobotSnapshot snapshot = fixture.update(rightCandidate);
    require(snapshot.autonomousStatus.phase == "green_confirming",
            "RIGHT provisório deve abrir a janela de confirmação.");

    snapshot = fixture.update(retainedRight);
    require(
        snapshot.autonomousStatus.phase == "green_confirming",
        "A confirmação retida deve ser consumida antes da ausência do candidato.");

    const long long confirmationCounts = static_cast<long long>(std::ceil(
        config::kGreenConfirmationMaximumDistanceCm *
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount = confirmationCounts;
    fixture.telemetry.rightEncoderCount = confirmationCounts;
    snapshot = fixture.update(freshVision(GreenInterpretation::None));
    require(
        snapshot.autonomousStatus.phase == "green_forward_right",
        "RIGHT retido e confirmado deve concluir a janela e iniciar os 100 mm.");
}

void testTransientCandidateLossDuringBrakingKeepsCurrentEvent()
{
    CameraLineSnapshot rightCandidate =
        freshVision(GreenInterpretation::Right);
    rightCandidate.greenConfirmed = false;
    rightCandidate.greenPathBlackValid = true;
    rightCandidate.greenInterpretation = GreenInterpretation::None;

    MissionFixture fixture;
    fixture.telemetry.leftEncoderRate = -3000.0;
    fixture.telemetry.rightEncoderRate = 3000.0;
    RobotSnapshot snapshot = fixture.update(rightCandidate);
    require(snapshot.autonomousStatus.phase == "green_confirmation_stopping",
            "RIGHT provisório deve abrir a confirmação e iniciar a frenagem.");

    snapshot = fixture.update(freshVision(GreenInterpretation::FalseMarker));
    require(snapshot.autonomousStatus.phase == "green_confirmation_stopping",
            "FALSE transitório não pode apagar um RIGHT válido durante a frenagem.");

    CameraLineSnapshot retainedRight =
        freshVision(GreenInterpretation::Right);
    retainedRight.greenCandidateCount = 0;
    retainedRight.greenPathBlackValid = false;
    retainedRight.greenRawInterpretation = GreenInterpretation::None;
    snapshot = fixture.update(retainedRight);
    require(snapshot.autonomousStatus.phase == "green_confirmation_stopping",
            "A confirmação RIGHT retida deve ser consumida pelo evento ainda ativo.");

    fixture.telemetry.leftEncoderRate = 0.0;
    fixture.telemetry.rightEncoderRate = 0.0;
    snapshot = fixture.update(freshVision(GreenInterpretation::None));
    require(snapshot.autonomousStatus.phase == "green_confirming",
            "Após parar, o evento RIGHT confirmado deve iniciar os 30 mm.");

    const long long confirmationCounts = static_cast<long long>(std::ceil(
        config::kGreenConfirmationMaximumDistanceCm *
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount = confirmationCounts;
    fixture.telemetry.rightEncoderCount = confirmationCounts;
    snapshot = fixture.update(freshVision(GreenInterpretation::None));
    require(snapshot.autonomousStatus.phase == "green_forward_right",
            "RIGHT confirmado deve seguir obrigatoriamente para os 100 mm.");
}

void testBrakingDoesNotConsumeVisualDiscardWindow()
{
    MissionFixture fixture;
    CameraLineSnapshot provisional = freshVision(GreenInterpretation::FalseMarker);
    provisional.greenConfirmed = false;
    fixture.telemetry.leftEncoderRate = -3000.0;
    fixture.telemetry.rightEncoderRate = 3000.0;
    fixture.update(provisional);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kGreenConfirmationDecisionWaitMs + 20));
    ++provisional.lineSequence;
    RobotSnapshot snapshot = fixture.update(provisional);
    require(snapshot.autonomousStatus.phase == "green_confirmation_stopping" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "FALSE durante a frenagem não pode consumir a janela visual.");

    fixture.telemetry.leftEncoderRate = 0.0;
    fixture.telemetry.rightEncoderRate = 0.0;
    snapshot = fixture.update(provisional);
    require(snapshot.autonomousStatus.phase == "green_confirming",
            "O avanço deve começar com um prazo visual novo após parar.");
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kGreenConfirmationDecisionWaitMs + 20));
    snapshot = fixture.update(provisional);
    require(snapshot.autonomousStatus.phase == "green_confirming",
            "A mesma imagem que iniciou o avanço não pode descartar o evento.");

    const long long confirmationCounts = static_cast<long long>(std::ceil(
        config::kGreenConfirmationMaximumDistanceCm *
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount = confirmationCounts;
    fixture.telemetry.rightEncoderCount = confirmationCounts;
    ++provisional.lineSequence;
    snapshot = fixture.update(provisional);
    require(snapshot.autonomousStatus.phase == "green_confirming" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "Ao atingir 30 mm, FALSE não pode antecipar a espera parada.");

    CameraLineSnapshot retainedRight = freshVision(GreenInterpretation::Right);
    retainedRight.lineSequence = provisional.lineSequence + 1;
    retainedRight.greenCandidateCount = 0;
    retainedRight.greenPathBlackValid = false;
    retainedRight.greenRawInterpretation = GreenInterpretation::None;
    snapshot = fixture.update(retainedRight);
    require(snapshot.autonomousStatus.phase == "green_forward_right",
            "RIGHT retido deve confirmar o evento preservado após a frenagem.");
}

void testValidBlackCandidateGetsBoundedExtraConfirmationWait()
{
    for (const auto finalDecision : {GreenInterpretation::Right,
                                    GreenInterpretation::Left,
                                    GreenInterpretation::FalseMarker,
                                    GreenInterpretation::Ambiguous})
    {
        MissionFixture fixture;
        CameraLineSnapshot candidate = freshVision(GreenInterpretation::Ambiguous);
        candidate.greenConfirmed = false;
        candidate.greenInterpretation = GreenInterpretation::Ambiguous;
        candidate.greenPathBlackValid = false;
        candidate.greenFrontRoiValid = true;
        fixture.update(candidate);
        const long long counts = static_cast<long long>(std::ceil(
            config::kGreenConfirmationMaximumDistanceCm * config::kEncoderCountsPerCentimeter));
        fixture.telemetry.leftEncoderCount = counts;
        fixture.telemetry.rightEncoderCount = counts;
        ++candidate.lineSequence;
        fixture.update(candidate);
        // Reproduz a perda da ROI antes de o prazo inicial terminar.
        candidate.greenFrontRoiValid = false;
        candidate.greenCandidateCount = 0;
        candidate.greenRawInterpretation = GreenInterpretation::FalseMarker;
        candidate.greenInterpretation = GreenInterpretation::FalseMarker;
        candidate.greenConfirmed = true;
        ++candidate.lineSequence;
        fixture.update(candidate);
        std::this_thread::sleep_for(std::chrono::milliseconds(
            config::kGreenConfirmationDecisionWaitMs + 20));
        ++candidate.lineSequence;
        RobotSnapshot snapshot = fixture.update(candidate);
        require(snapshot.autonomousStatus.phase == "green_confirming" &&
                    snapshot.left == 0.0 && snapshot.right == 0.0,
                "Preto válido anterior deve preservar a espera mesmo com FALSE após perder a ROI.");
        // Mesmo depois da concessão, FALSE não pode antecipar o teto da espera.
        ++candidate.lineSequence;
        snapshot = fixture.update(candidate);
        require(snapshot.autonomousStatus.phase == "green_confirming" &&
                    snapshot.left == 0.0 && snapshot.right == 0.0,
                "A espera extra deve permanecer ativa após outro snapshot FALSE.");
        if (finalDecision == GreenInterpretation::Left ||
            finalDecision == GreenInterpretation::Right)
        {
            candidate.greenConfirmed = true;
            candidate.greenInterpretation = finalDecision;
            candidate.greenCandidateCount = 0;
            candidate.greenPathBlackValid = false;
            candidate.greenFrontRoiValid = false;
            ++candidate.lineSequence;
            snapshot = fixture.update(candidate);
            require(snapshot.autonomousStatus.phase ==
                        (finalDecision == GreenInterpretation::Right
                             ? "green_forward_right" : "green_forward_left"),
                    "O lado confirmado na espera extra deve iniciar a manobra sem novos pixels verdes.");
        }
        else
        {
            candidate.greenInterpretation = finalDecision;
            candidate.greenRawInterpretation = finalDecision;
            candidate.greenConfirmed = finalDecision == GreenInterpretation::FalseMarker;
            std::this_thread::sleep_for(std::chrono::milliseconds(
                config::kGreenConfirmationExtraWaitMs + 20));
            ++candidate.lineSequence;
            snapshot = fixture.update(candidate);
            require(snapshot.autonomousStatus.phase == "line_following",
                    "Novos candidatos não podem renovar indefinidamente a espera extra.");
        }
    }
}

void testGreenAcceptsOnlyConfirmedSideLineBeforeMinimumYaw()
{
    for (const auto direction : {GreenInterpretation::Left, GreenInterpretation::Right})
    {
      for (const bool referenceAvailable : {true, false})
      {
        MissionFixture fixture;
        CameraLineSnapshot green = freshVision(direction);
        const double sign = direction == GreenInterpretation::Right ? 1.0 : -1.0;
        green.curveDiagnostics.lineState = "GREEN";
        green.lineControlSource = "fusion";
        green.mediumTrusted = true;
        green.curveDiagnostics.mediumPosition = sign * 0.5;
        green.curveDiagnostics.headingAngleDeg = referenceAvailable
            ? sign * 6.2 : std::numeric_limits<double>::quiet_NaN();
        // Cruza a fronteira do yaw para verificar a compensação nos dois lados.
        fixture.telemetry.yawZDeg = sign * 179.0;
        fixture.update(green);
        const long long confirmationCounts = static_cast<long long>(std::ceil(
            config::kGreenConfirmationMaximumDistanceCm * config::kEncoderCountsPerCentimeter));
        fixture.telemetry.leftEncoderCount = confirmationCounts;
        fixture.telemetry.rightEncoderCount = confirmationCounts;
        fixture.update(green);
        const long long forwardCounts = static_cast<long long>(std::ceil(
            config::kGreenLateralForwardDistanceCm * config::kEncoderCountsPerCentimeter));
        fixture.telemetry.leftEncoderCount += forwardCounts;
        fixture.telemetry.rightEncoderCount += forwardCounts;
        fixture.update(green);
        std::this_thread::sleep_for(std::chrono::milliseconds(config::kRescueDistanceSettleMs + 20));
        fixture.update(green);
        fixture.telemetry.yawZDeg = std::remainder(sign * 189.0, 360.0);
        green.lineControlSource = "fusion-green";
        green.mediumTrusted = true;
        green.curveDiagnostics.finalSteering = sign * 0.3;
        const std::string searchPhase = direction == GreenInterpretation::Right
            ? "green_searching_right" : "green_searching_left";
        for (const double position : {0.0, -sign * 0.5})
        {
            ++green.lineSequence;
            green.curveDiagnostics.mediumPosition = position;
            require(fixture.update(green).autonomousStatus.phase == searchPhase,
                    "CENTER e lado oposto não dispensam o yaw mínimo.");
        }
        green.curveDiagnostics.mediumPosition = sign * 0.5;
        require(fixture.update(green).autonomousStatus.phase == searchPhase,
                "Reutilizar o snapshot não dispensa o yaw mínimo.");
        ++green.lineSequence;
        green.lineControlSource = "green-direction-hold";
        require(fixture.update(green).autonomousStatus.phase == searchPhase,
                "Hold sem Fusion atual não dispensa o yaw mínimo.");
        ++green.lineSequence;
        green.lineControlSource = "fusion-green";
        green.curveDiagnostics.finalSteering = -sign * 0.3;
        require(fixture.update(green).autonomousStatus.phase == searchPhase,
                "Fusion apontando para o lado oposto não dispensa o yaw mínimo.");
        ++green.lineSequence;
        green.curveDiagnostics.finalSteering = sign * 0.3;
        green.curveDiagnostics.headingAngleDeg = sign * (6.2 - 10.0);
        require(fixture.update(green).autonomousStatus.phase == searchPhase,
                "A mesma faixa frontal vista de lado não pode dispensar o giro mínimo.");
        ++green.lineSequence;
        green.curveDiagnostics.headingAngleDeg = sign * (14.0 - 10.0);
        require(fixture.update(green).autonomousStatus.phase == searchPhase,
                "A mudança de projeção de 6,2° para 14° não comprova um ramo lateral.");

        green.curveDiagnostics.headingAngleDeg = sign * (6.2 + 40.0 - 10.0);
        for (int frame = 1; frame <= config::kGreenEarlyBranchStableFrames; ++frame)
        {
            ++green.lineSequence;
            const RobotSnapshot snapshot = fixture.update(green);
            if (referenceAvailable && frame == config::kGreenEarlyBranchStableFrames)
            {
                require(snapshot.autonomousStatus.phase ==
                            (direction == GreenInterpretation::Right
                                 ? "green_reverse_right" : "green_reverse_left") &&
                            snapshot.left < 0.0 && snapshot.right < 0.0,
                        "Um ramo distinto e estável no lado confirmado deve dispensar os 30°.");
            }
            else
            {
                require(snapshot.autonomousStatus.phase == searchPhase,
                        "Uma imagem isolada ou referência ausente não pode liberar antecipadamente.");
                require(fixture.update(green).autonomousStatus.phase == searchPhase,
                        "Repetir a mesma imagem não acumula estabilidade do ramo.");
                if (referenceAvailable && frame == 1)
                {
                    const double validHeading = green.curveDiagnostics.headingAngleDeg;
                    ++green.lineSequence;
                    green.curveDiagnostics.headingAngleDeg =
                        std::numeric_limits<double>::quiet_NaN();
                    require(fixture.update(green).autonomousStatus.phase == searchPhase,
                            "Um heading ausente deve interromper a sequência de evidências.");
                    ++green.lineSequence;
                    green.curveDiagnostics.headingAngleDeg = validHeading;
                    require(fixture.update(green).autonomousStatus.phase == searchPhase,
                            "Depois de perder a geometria, um frame válido ainda não basta.");
                }
            }
        }
        if (!referenceAvailable)
        {
            fixture.telemetry.yawZDeg = std::remainder(
                sign * (179.0 + config::kGreenLateralMinimumYawDegrees), 360.0);
            ++green.lineSequence;
            const RobotSnapshot snapshot = fixture.update(green);
            require(snapshot.autonomousStatus.phase ==
                        (direction == GreenInterpretation::Right
                             ? "green_reverse_right" : "green_reverse_left"),
                    "Após os 30°, a referência ausente não altera a aquisição normal da faixa.");
        }
      }
    }
}

void testConfirmationStartsAfterInitialStopAndMeasuresRealDistance()
{
    CameraLineSnapshot rightCandidate =
        freshVision(GreenInterpretation::Right);
    rightCandidate.greenConfirmed = false;
    rightCandidate.greenPathBlackValid = false;
    rightCandidate.greenInterpretation = GreenInterpretation::None;

    MissionFixture fixture;
    fixture.telemetry.leftEncoderRate = -3000.0;
    fixture.telemetry.rightEncoderRate = 3000.0;
    RobotSnapshot snapshot = fixture.update(rightCandidate);
    require(
        snapshot.autonomousStatus.phase == "green_confirmation_stopping" &&
            snapshot.left == 0.0 && snapshot.right == 0.0,
        "O primeiro candidato deve parar o robô antes de medir os 30 mm.");

    const long long brakingCounts = static_cast<long long>(std::ceil(
        2.0 * config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount = -brakingCounts;
    fixture.telemetry.rightEncoderCount = brakingCounts;
    fixture.telemetry.leftEncoderRate = 0.0;
    fixture.telemetry.rightEncoderRate = 0.0;
    snapshot = fixture.update(rightCandidate);
    require(
        snapshot.autonomousStatus.phase == "green_confirming" &&
            snapshot.left != 0.0 && snapshot.right != 0.0,
        "A confirmação deve avançar somente depois de registrar a parada.");

    CameraLineSnapshot retainedRight =
        freshVision(GreenInterpretation::Right);
    retainedRight.greenCandidateCount = 0;
    retainedRight.greenPathBlackValid = false;
    retainedRight.greenRawInterpretation = GreenInterpretation::None;

    const long long confirmationCounts = static_cast<long long>(std::ceil(
        config::kGreenConfirmationMaximumDistanceCm *
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount =
        -brakingCounts - confirmationCounts;
    fixture.telemetry.rightEncoderCount =
        brakingCounts + confirmationCounts;
    snapshot = fixture.update(rightCandidate);
    require(
        snapshot.autonomousStatus.phase == "green_confirming" &&
            snapshot.left == 0.0 && snapshot.right == 0.0,
        "Somente 30 mm após a parada devem interromper o avanço.");

    snapshot = fixture.update(retainedRight);
    require(
        snapshot.autonomousStatus.phase == "green_forward_right",
        "A decisão visual após os 30 mm reais deve iniciar os 100 mm.");
}

void testLateralGreenCentersOnlyWithCompleteLocalGeometry()
{
    MissionFixture fixture;
    CameraLineSnapshot green = freshVision(GreenInterpretation::Right);
    green.lineControlSource = "fusion-green";
    green.mediumTrusted = true;
    green.farTrusted = true;
    green.curveDiagnostics.lineState = "GREEN";
    green.curveDiagnostics.mediumPosition = 0.0;
    green.curveDiagnostics.farBandPosition = 0.0;

    fixture.update(green);
    const long long confirmationCounts = static_cast<long long>(std::ceil(
        config::kGreenConfirmationMaximumDistanceCm *
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount = confirmationCounts;
    fixture.telemetry.rightEncoderCount = confirmationCounts;
    fixture.update(green);

    const long long forwardCounts = static_cast<long long>(std::ceil(
        config::kGreenLateralForwardDistanceCm *
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount += forwardCounts / 2;
    fixture.telemetry.rightEncoderCount += forwardCounts / 2;
    RobotSnapshot snapshot = fixture.update(green);
    require(
        snapshot.autonomousStatus.phase == "green_forward_right",
        "Cinco centímetros não devem concluir o novo avanço de 10 cm.");

    fixture.telemetry.leftEncoderCount = confirmationCounts + forwardCounts;
    fixture.telemetry.rightEncoderCount = confirmationCounts + forwardCounts;
    fixture.update(green);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kRescueDistanceSettleMs + 20));
    snapshot = fixture.update(green);
    require(
        snapshot.autonomousStatus.phase == "green_searching_right" &&
            snapshot.left > 0.0 && snapshot.right < 0.0,
        "RIGHT deve girar fisicamente para a direita e ignorar a linha antes de 30°.");

    fixture.telemetry.yawZDeg =
        config::kGreenLateralMinimumYawDegrees - 1.0;
    CameraLineSnapshot beforeMinimum = green;
    beforeMinimum.lineSequence = 2;
    snapshot = fixture.update(beforeMinimum);
    require(snapshot.autonomousStatus.phase == "green_searching_right",
            "A busca deve continuar abaixo do yaw mínimo.");

    fixture.telemetry.yawZDeg = config::kGreenLateralMinimumYawDegrees;
    snapshot = fixture.update(beforeMinimum);
    require(snapshot.autonomousStatus.phase == "green_searching_right",
            "Um snapshot antigo não pode encerrar a procura após 30°.");

    CameraLineSnapshot foundGreen = green;
    foundGreen.lineSequence = 3;
    snapshot = fixture.update(foundGreen);
    require(snapshot.autonomousStatus.phase == "green_centering_right",
            "Um frame novo com NEAR, MID e FAR deve iniciar GREEN_CENTERING.");

    for (int frame = 0; frame < config::kGreenCenteringRequiredFrames; ++frame)
    {
        ++foundGreen.lineSequence;
        snapshot = fixture.update(foundGreen);
    }
    require(
        snapshot.autonomousStatus.phase == "green_reverse_right" &&
            snapshot.left < 0.0 && snapshot.right < 0.0 &&
            !snapshot.encoderSynchronizationAllowed,
        "Frames centralizados devem iniciar a ré medida antes do seguidor normal.");

    fixture.telemetry.yawZDeg += 3.0;
    snapshot = fixture.update(foundGreen);
    require(snapshot.left < snapshot.right &&
                snapshot.autonomousStatus.phase == "green_reverse_right",
            "Yaw à direita deve corrigir a ré para a esquerda.");

    const long long reverseCounts = static_cast<long long>(std::ceil(
        config::kGreenLateralReverseDistanceCm *
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount -= reverseCounts;
    fixture.telemetry.rightEncoderCount -= reverseCounts;
    snapshot = fixture.update(foundGreen);
    require(snapshot.left == 0.0 && snapshot.right == 0.0,
            "A ré deve parar ao atingir 5 cm pelos encoders.");
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kRescueDistanceSettleMs + 20));
    snapshot = fixture.update(foundGreen);
    require(snapshot.autonomousStatus.phase == "green_reverse_complete" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "A conclusão da ré não pode sobrepor o seguidor no mesmo ciclo.");

    CameraLineSnapshot normalFusion = freshVision(GreenInterpretation::None);
    normalFusion.lineSequence = foundGreen.lineSequence + 1;
    normalFusion.lineControlSource = "fusion";
    normalFusion.normalSteeringValid = true;
    normalFusion.curveDiagnostics.lineState = "LINE";
    normalFusion.curveDiagnostics.virtualState = "NORMAL";
    normalFusion.curveDiagnostics.finalSteering = 0.0;
    normalFusion.lineFollowerLeftPower = 0.73;
    normalFusion.lineFollowerRightPower = 0.69;
    snapshot = fixture.update(normalFusion);
    require(snapshot.autonomousStatus.phase == "line_following" &&
                closeTo(snapshot.left, normalFusion.lineFollowerLeftPower) &&
                closeTo(snapshot.right, normalFusion.lineFollowerRightPower),
            "O Fusion deve reassumir somente depois da ré concluída.");
}

void testLateralGreenSkipsCenteringWhenGeometryIsIncomplete()
{
    MissionFixture fixture;
    CameraLineSnapshot green = freshVision(GreenInterpretation::Right);
    green.lineControlSource = "green-direction-hold";
    green.mediumTrusted = false;
    green.farTrusted = false;
    green.curveDiagnostics.lineState = "GREEN";

    fixture.update(green);
    const long long confirmationCounts = static_cast<long long>(std::ceil(
        config::kGreenConfirmationMaximumDistanceCm *
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount = confirmationCounts;
    fixture.telemetry.rightEncoderCount = confirmationCounts;
    fixture.update(green);

    const long long forwardCounts = static_cast<long long>(std::ceil(
        config::kGreenLateralForwardDistanceCm *
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount += forwardCounts;
    fixture.telemetry.rightEncoderCount += forwardCounts;
    fixture.update(green);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kRescueDistanceSettleMs + 20));
    fixture.update(green);

    fixture.telemetry.yawZDeg = config::kGreenLateralMinimumYawDegrees;
    RobotSnapshot snapshot = fixture.update(green);
    require(
        snapshot.autonomousStatus.phase == "green_searching_right" &&
            snapshot.left > 0.0 && snapshot.right < 0.0,
        "Após 30°, RIGHT continua o pivot enquanto a faixa não aparecer.");

    CameraLineSnapshot noPath = green;
    noPath.lineSequence = 2;
    snapshot = fixture.update(noPath);
    require(
        snapshot.autonomousStatus.phase == "green_searching_right" &&
            snapshot.left > 0.0 && snapshot.right < 0.0,
        "Frame novo sem MID/FAR não pode encerrar a procura.");

    fixture.telemetry.yawZDeg =
        config::kGreenLateralMinimumYawDegrees + 15.0;
    snapshot = fixture.update(noPath);
    require(
        snapshot.autonomousStatus.phase == "green_searching_right" &&
            snapshot.left > 0.0 && snapshot.right < 0.0,
        "O pivot deve continuar além de 30° enquanto a faixa não aparecer.");

    CameraLineSnapshot foundMid = noPath;
    foundMid.lineSequence = 3;
    foundMid.mediumTrusted = true;
    foundMid.curveDiagnostics.mediumPosition = 0.0;
    snapshot = fixture.update(foundMid);
    require(
        snapshot.autonomousStatus.phase == "green_reverse_right" &&
            snapshot.left < 0.0 && snapshot.right < 0.0,
        "MID encontrada sem FAR deve pular a centralização e iniciar a ré.");
}

void testLeftGreenReversesOnlyAfterVisualLine()
{
    MissionFixture fixture;
    CameraLineSnapshot green = freshVision(GreenInterpretation::Left);
    green.lineControlSource = "green-direction-hold";
    green.curveDiagnostics.lineState = "GREEN";

    fixture.update(green);
    const long long confirmationCounts = static_cast<long long>(std::ceil(
        config::kGreenConfirmationMaximumDistanceCm *
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount = confirmationCounts;
    fixture.telemetry.rightEncoderCount = confirmationCounts;
    fixture.update(green);

    const long long forwardCounts = static_cast<long long>(std::ceil(
        config::kGreenLateralForwardDistanceCm *
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount += forwardCounts;
    fixture.telemetry.rightEncoderCount += forwardCounts;
    fixture.update(green);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kRescueDistanceSettleMs + 20));
    fixture.update(green);

    fixture.telemetry.yawZDeg = -config::kGreenLateralMinimumYawDegrees;
    RobotSnapshot snapshot = fixture.update(green);
    require(snapshot.autonomousStatus.phase == "green_searching_left" &&
                snapshot.left < 0.0 && snapshot.right > 0.0,
            "LEFT deve continuar procurando a faixa após o giro mínimo.");

    CameraLineSnapshot foundMid = green;
    foundMid.lineSequence = 2;
    foundMid.mediumTrusted = true;
    foundMid.curveDiagnostics.mediumPosition = -0.4;
    snapshot = fixture.update(foundMid);
    require(snapshot.autonomousStatus.phase == "green_reverse_left" &&
                snapshot.left < 0.0 && snapshot.right < 0.0,
            "MID encontrada deve iniciar a mesma ré de 5 cm para LEFT.");

    fixture.telemetry.mpuOk = false;
    snapshot = fixture.update(foundMid);
    require(snapshot.autonomousStatus.phase == "green_reverse_imu_lost" &&
                snapshot.left == 0.0 && snapshot.right == 0.0,
            "Perder a IMU durante a ré deve parar as duas rodas.");
}

void testGreenReverseIgnoresBlindLinePlaceholder()
{
    MissionFixture fixture;
    CameraLineSnapshot green = freshVision(GreenInterpretation::Right);
    green.lineControlSource = "green-direction-hold";
    green.curveDiagnostics.lineState = "GREEN";

    fixture.update(green);
    const long long confirmationCounts = static_cast<long long>(std::ceil(
        config::kGreenConfirmationMaximumDistanceCm *
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount = confirmationCounts;
    fixture.telemetry.rightEncoderCount = confirmationCounts;
    fixture.update(green);

    const long long forwardCounts = static_cast<long long>(std::ceil(
        config::kGreenLateralForwardDistanceCm *
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount += forwardCounts;
    fixture.telemetry.rightEncoderCount += forwardCounts;
    fixture.update(green);
    std::this_thread::sleep_for(std::chrono::milliseconds(
        config::kRescueDistanceSettleMs + 20));
    fixture.update(green);

    fixture.telemetry.yawZDeg = config::kGreenLateralMinimumYawDegrees;
    fixture.update(green);

    CameraLineSnapshot waiting = freshVision(GreenInterpretation::None, false);
    waiting.lineSequence = 2;
    waiting.lineControlSource = "virtual-search-wait";
    waiting.curveDiagnostics.lineState = "LINE";
    waiting.lineFollowerLeftPower = 0.0;
    waiting.lineFollowerRightPower = 0.0;
    const RobotSnapshot snapshot = fixture.update(waiting);
    require(snapshot.autonomousStatus.phase == "green_searching_right" &&
                snapshot.left > 0.0 && snapshot.right < 0.0,
            "LINE de espera sem faixa física não pode encerrar o pivot nem iniciar ré.");

    CameraLineSnapshot foundMid = green;
    foundMid.lineSequence = 3;
    foundMid.mediumTrusted = true;
    foundMid.curveDiagnostics.mediumPosition = 0.4;
    const RobotSnapshot reverse = fixture.update(foundMid);
    require(reverse.autonomousStatus.phase == "green_reverse_right" &&
                reverse.left < 0.0 && reverse.right < 0.0,
            "A MID encontrada depois do placeholder deve iniciar a ré medida.");
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
        "O reconhecimento do retorno deve zerar os motores durante a pausa configurada.");

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

void testTransientLineIpcLossPausesAndResumesMission()
{
    MissionFixture fixture;
    CameraLineSnapshot staleLine = freshVision(GreenInterpretation::None);
    staleLine.sourceFresh = false;

    RobotSnapshot snapshot = fixture.update(staleLine, true);
    require(
        snapshot.mode == "autonomous" && snapshot.left == 0.0 &&
            snapshot.right == 0.0 &&
            snapshot.autonomousStatus.phase == "line_ipc_waiting",
        "IPC visual antigo deve pausar com motores zerados sem encerrar a missão.");

    snapshot = fixture.update(freshVision(GreenInterpretation::None), true);
    requireFollowingLine(snapshot, "Retorno do IPC visual");
}

void testObstaclePausesIfBottomCameraBecomesUnavailable()
{
    MissionFixture fixture;
    fixture.telemetry.ultrasonicDistanceCm = 5.0;

    RobotSnapshot snapshot = fixture.update(
        freshVision(GreenInterpretation::None));
    requireFollowingLine(snapshot, "Primeira confirmação do obstáculo");

    snapshot = fixture.update(freshVision(GreenInterpretation::None));
    require(
        snapshot.mode == "autonomous" && snapshot.left == 0.0 &&
            snapshot.right == 0.0 &&
            snapshot.autonomousStatus.phase == "obstacle_detected",
        "Duas leituras ultrassônicas devem parar antes de iniciar o desvio.");

    // Conclui a ré pelos encoders antes de avaliar a falta de visão na
    // centralização; sem isso, o teste ainda estaria na etapa anterior.
    const long long reverseCounts = static_cast<long long>(std::ceil(
        config::kObstacleReverseDistanceCm * config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount -= reverseCounts;
    fixture.telemetry.rightEncoderCount -= reverseCounts;
    fixture.update(freshVision(GreenInterpretation::None));

    CameraLineSnapshot staleLine = freshVision(GreenInterpretation::None);
    staleLine.sourceFresh = false;
    snapshot = fixture.update(staleLine, false);
    require(
        snapshot.mode == "autonomous" && snapshot.left == 0.0 &&
            snapshot.right == 0.0 &&
            snapshot.autonomousStatus.phase ==
                "obstacle_centering_waiting_line",
        "A centralização do desvio deve pausar com PWM zero sem perder a manobra.");
}

void testObstacleRespectsStopAndSafetyPriority()
{
    for (int scenario = 0; scenario < 3; ++scenario)
    {
        MissionFixture fixture;
        fixture.telemetry.ultrasonicDistanceCm = 5.0;
        const auto line = freshVision(GreenInterpretation::None);
        fixture.update(line);
        fixture.update(line);
        const auto reversing = fixture.update(line);
        require(reversing.left < 0.0 && reversing.right < 0.0,
                "O teste deve interromper um desvio que já está movimentando os motores.");
        if (scenario == 0)
        {
            fixture.robotState.emergencyStop();
        }
        else if (scenario == 1)
        {
            fixture.robotState.stop();
        }
        else
        {
            fixture.telemetry.sensorFresh = false;
        }
        const auto stopped = fixture.update(line);
        require(stopped.mode != "autonomous" && stopped.left == 0.0 && stopped.right == 0.0,
                "E-Stop, Parar e perda da ESP32 devem interromper o desvio com motores zerados.");
    }
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
    gray.silverCandidateDetected = true;
    snapshot = fixture.update(gray);
    require(
        snapshot.mode == "autonomous" &&
            closeTo(snapshot.left, config::kSilverEntryAdvancePower) &&
            closeTo(snapshot.right, config::kSilverEntryAdvancePower) &&
            snapshot.autonomousStatus.phase == "silver_entry_advancing",
        "A candidata cinza deve iniciar o avanço reto de até 5 cm.");

    gray.courseMarkerConfirmed = true;
    gray.courseMarker = CourseMarker::Gray;
    snapshot = fixture.update(gray);
    require(
        closeTo(snapshot.left, -config::kSilverEntryReversePower) &&
            closeTo(snapshot.right, -config::kSilverEntryReversePower) &&
            snapshot.autonomousStatus.phase == "silver_entry_backing_up",
        "A faixa cinza confirmada deve recuar antes de alinhar.");

    const long long reverseCounts = static_cast<long long>(std::ceil(
        config::kSilverEntryReverseDistanceCm *
        config::kEncoderCountsPerCentimeter));
    fixture.telemetry.leftEncoderCount = reverseCounts;
    fixture.telemetry.rightEncoderCount = reverseCounts;
    snapshot = fixture.update(gray);
    require(
        snapshot.autonomousStatus.phase == "silver_entry_aligning_line" &&
            snapshot.left == 0.0 && snapshot.right == 0.0,
        "A primeira leitura central deve confirmar a linha inferior.");

    snapshot = fixture.update(gray);
    require(
        snapshot.mode == "autonomous" && snapshot.left == 0.0 &&
            snapshot.right == 0.0 &&
            snapshot.autonomousStatus.phase == "rescue_area_entering",
        "Duas leituras centrais devem confirmar a área de resgate.");
    require(
        fixture.mission.requiresRescueVision(),
        "A faixa cinza deve ligar automaticamente a visão de vítimas.");

    snapshot = fixture.update(gray);
    require(
        snapshot.mode == "autonomous" && snapshot.left == 0.0 &&
            snapshot.right == 0.0 &&
            snapshot.autonomousStatus.phase == "rescue_entry_waiting_yolo",
        "Depois da faixa cinza, o robô deve aguardar o primeiro frame do YOLO.");

    fixture.mission.reset();
    require(
        !fixture.mission.requiresRescueVision(),
        "O reset deve restaurar o primeiro percurso sem visão pesada.");

    MissionFixture rejectedFixture;
    CameraLineSnapshot candidate = freshVision(GreenInterpretation::None);
    candidate.silverCandidateDetected = true;
    snapshot = rejectedFixture.update(candidate);
    require(
        snapshot.autonomousStatus.phase == "silver_entry_advancing",
        "A candidata deve abrir a janela de avanço.");

    const long long entryCounts = static_cast<long long>(std::ceil(
        config::kSilverEntryAdvanceDistanceCm *
        config::kEncoderCountsPerCentimeter));
    rejectedFixture.telemetry.leftEncoderCount = entryCounts;
    rejectedFixture.telemetry.rightEncoderCount = entryCounts;
    snapshot = rejectedFixture.update(freshVision(GreenInterpretation::None));
    requireFollowingLine(
        snapshot,
        "Sem confirmação dentro de 5 cm");
    require(
        !rejectedFixture.mission.requiresRescueVision(),
        "Uma candidata não confirmada não deve iniciar o resgate.");

    MissionFixture reverseFixture;
    CameraLineSnapshot reverseCandidate = freshVision(GreenInterpretation::None);
    reverseCandidate.silverCandidateDetected = true;
    reverseFixture.update(reverseCandidate);

    CameraLineSnapshot noLine = reverseCandidate;
    noLine.courseMarkerConfirmed = true;
    noLine.courseMarker = CourseMarker::Gray;
    noLine.lineNearDetected = false;
    noLine.lineNearFinePosition = std::numeric_limits<double>::quiet_NaN();
    snapshot = reverseFixture.update(noLine);
    require(
        closeTo(snapshot.left, -config::kSilverEntryReversePower) &&
            closeTo(snapshot.right, -config::kSilverEntryReversePower) &&
            snapshot.autonomousStatus.phase == "silver_entry_backing_up",
        "Sem linha preta, a entrada deve iniciar uma ré curta.");

    const long long delayedReverseCounts = static_cast<long long>(std::ceil(
        config::kSilverEntryReverseDistanceCm *
        config::kEncoderCountsPerCentimeter));
    reverseFixture.telemetry.leftEncoderCount = delayedReverseCounts;
    reverseFixture.telemetry.rightEncoderCount = delayedReverseCounts;
    snapshot = reverseFixture.update(noLine);
    require(
        snapshot.left == 0.0 && snapshot.right == 0.0 &&
            snapshot.autonomousStatus.phase == "silver_entry_waiting_line",
        "Depois da ré sem linha, o robô deve aguardar parado.");

    CameraLineSnapshot alignedLine = noLine;
    alignedLine.lineNearDetected = true;
    alignedLine.lineNearFinePosition = 0.0;
    snapshot = reverseFixture.update(alignedLine);
    require(
        snapshot.autonomousStatus.phase == "silver_entry_aligning_line",
        "A primeira leitura NEAR central deve aguardar confirmação.");
    snapshot = reverseFixture.update(alignedLine);
    require(
        snapshot.autonomousStatus.phase == "rescue_area_entering" &&
            snapshot.left == 0.0 && snapshot.right == 0.0,
        "A linha recuperada e centralizada deve confirmar a área de resgate.");
}

void testRescueAlignmentModeKeepsExistingMotorAuthority()
{
    RobotState robotState;
    MissionController controller;
    robotState.setAutonomousMission(AutonomousMission::RescueArea);
    robotState.startAutonomous();

    const RobotSnapshot started = robotState.snapshot();
    ForwardBallSnapshot ball;
    ball.sourceFresh = true;
    ball.detected = true;
    ball.type = "black_ball";
    ball.txDegrees = 18.0;
    ball.distanceCm = 24.0;
    ball.radiusPixels = 30.0;
    ball.visibleAreaPixels = 2500.0;
    ball.targetSequence = started.autonomousRunSequence;
    ball.targetLocked = true;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    controller.update(
        robotState, telemetry, false, {}, {}, ball);

    RobotSnapshot snapshot = robotState.snapshot();
    require(
        snapshot.mode == "autonomous" && snapshot.left == 0.0 &&
            snapshot.right == 0.0 &&
            snapshot.autonomousStatus.phase == "rescue_victim_acquired",
        "O alvo confirmado deve parar antes de iniciar o alinhamento.");

    controller.update(
        robotState, telemetry, false, {}, {}, ball);
    snapshot = robotState.snapshot();
    require(
        snapshot.mode == "autonomous" && snapshot.left > 0.0 &&
            snapshot.right < 0.0 &&
            snapshot.autonomousStatus.phase ==
                "ball_alignment_correction_pulse",
        "O resgate deve transferir o alvo travado ao alinhamento validado.");
}

void testCornerYawModeOnlyTurnsAndStopsOnImuLoss()
{
    RobotState robotState;
    MissionController controller;
    robotState.setAutonomousMission(AutonomousMission::RescueCornerYawTest);
    robotState.startAutonomous();
    RobotSnapshot snapshot = robotState.snapshot();
    require(!snapshot.armServoRequested && !snapshot.wristServoRequested &&
                !snapshot.gripperServoRequested,
            "O teste de yaw não deve acionar servos.");
    require(!controller.requiresExitVision(snapshot) &&
                !controller.requiresRescueZoneDetection(snapshot),
            "O teste de yaw não deve ligar a visão de saída ou triângulos.");

    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    telemetry.yawZDeg = -120.0;
    controller.update(robotState, telemetry, false, {}, {}, {});
    snapshot = robotState.snapshot();
    require(snapshot.autonomousStatus.phase == "corner_yaw_turning" &&
                snapshot.left > 0.0 && snapshot.right < 0.0,
            "O primeiro alvo deve comandar apenas pivô à direita.");

    telemetry.mpuOk = false;
    controller.update(robotState, telemetry, false, {}, {}, {});
    snapshot = robotState.snapshot();
    require(snapshot.mode == "stopped" && snapshot.left == 0.0 &&
                snapshot.right == 0.0 &&
                snapshot.autonomousStatus.phase == "corner_yaw_failed",
            "A perda da IMU deve encerrar o teste com motores parados.");
}

void testRescueZoneDetectionOnlyKeepsMotorsStopped()
{
    RobotState robotState;
    MissionController controller;
    robotState.setAutonomousMission(
        AutonomousMission::RescueZoneDetection);
    robotState.startAutonomous();

    RobotSnapshot snapshot = robotState.snapshot();
    require(
        controller.requiresRescueZoneDetection(snapshot),
        "O modo isolado deve abrir o gate das áreas de resgate.");
    require(
        !controller.requiresForwardBallDetection(snapshot),
        "O modo das áreas não deve restaurar o detector de vítimas.");
    require(
        !snapshot.armServoRequested && !snapshot.wristServoRequested &&
            !snapshot.gripperServoRequested,
        "A validação visual não deve acionar os servos.");

    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    telemetry.ultrasonicDistanceCm = 43.7;

    robotState.driveAutonomous(0.8, -0.8);
    controller.update(
        robotState,
        telemetry,
        false,
        CameraLineSnapshot{},
        ForwardLineSnapshot{},
        ForwardBallSnapshot{});
    snapshot = robotState.snapshot();
    require(
        snapshot.mode == "autonomous" && closeTo(snapshot.left, 0.0) &&
            closeTo(snapshot.right, 0.0),
        "A percepção das áreas deve renovar potência zero a cada ciclo.");
    require(
        snapshot.autonomousStatus.phase == "rescue_zone_detection_active",
        "O modo isolado deve publicar uma fase de diagnóstico própria.");
    require(
        snapshot.autonomousStatus.rescueZoneUltrasonicFresh &&
            snapshot.autonomousStatus.rescueZoneUltrasonicValid &&
            closeTo(
                snapshot.autonomousStatus.rescueZoneUltrasonicDistanceCm,
                43.7),
        "O status do modo deve expor a distância frontal válida e atual.");

    telemetry.lastSensorAgeMs =
        config::kRescueZoneUltrasonicFreshnessMs + 1;
    controller.update(
        robotState,
        telemetry,
        false,
        CameraLineSnapshot{},
        ForwardLineSnapshot{},
        ForwardBallSnapshot{});
    snapshot = robotState.snapshot();
    require(
        !snapshot.autonomousStatus.rescueZoneUltrasonicFresh &&
            snapshot.autonomousStatus.rescueZoneUltrasonicValid &&
            closeTo(snapshot.left, 0.0) && closeTo(snapshot.right, 0.0),
        "Uma leitura stale deve ser marcada no status sem liberar os motores.");

    telemetry.lastSensorAgeMs = 0;
    telemetry.ultrasonicDistanceCm =
        config::kRescueZoneUltrasonicMaximumCm + 1.0;
    controller.update(
        robotState,
        telemetry,
        false,
        CameraLineSnapshot{},
        ForwardLineSnapshot{},
        ForwardBallSnapshot{});
    snapshot = robotState.snapshot();
    require(
        snapshot.autonomousStatus.rescueZoneUltrasonicFresh &&
            !snapshot.autonomousStatus.rescueZoneUltrasonicValid &&
            closeTo(snapshot.left, 0.0) && closeTo(snapshot.right, 0.0),
        "Uma leitura fora da faixa deve ser invalidada sem liberar os motores.");

    robotState.start();
    snapshot = robotState.snapshot();
    require(
        snapshot.mode == "manual" &&
            controller.requiresRescueZoneDetection(snapshot),
        "O modo Manual deve manter a percepção da área selecionada ativa.");
    robotState.drive(0.35, -0.25);
    controller.update(
        robotState,
        telemetry,
        false,
        CameraLineSnapshot{},
        ForwardLineSnapshot{},
        ForwardBallSnapshot{});
    snapshot = robotState.snapshot();
    require(
        snapshot.mode == "manual" && closeTo(snapshot.left, 0.35) &&
            closeTo(snapshot.right, -0.25),
        "A percepção das áreas não deve zerar comandos manuais.");

    robotState.stop();
    require(
        !controller.requiresRescueZoneDetection(robotState.snapshot()),
        "Stop deve fechar imediatamente o gate das áreas de resgate.");
}

void testRescueZoneAlignKeepsDetectionGateActive()
{
    RobotState robotState;
    MissionController controller;
    robotState.setAutonomousMission(AutonomousMission::RescueZoneAlign);
    robotState.setRescueZoneTargetColor(RescueZoneTargetColor::Green);
    robotState.startAutonomous();

    require(
        controller.requiresRescueZoneDetection(robotState.snapshot()),
        "ALIGN_ZONE autônomo deve manter o snapshot GREEN/RED sendo atualizado.");

    robotState.start();
    require(
        controller.requiresRescueZoneDetection(robotState.snapshot()),
        "ALIGN_ZONE selecionado deve preservar o overlay também no modo Manual.");

    robotState.emergencyStop();
    require(
        !controller.requiresRescueZoneDetection(robotState.snapshot()),
        "E-Stop deve fechar o gate visual e manter os motores protegidos.");
}

void testRescueZoneApproachUsesStoredHeadingWithCameraStopGate()
{
    RobotState robotState;
    MissionController controller;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    telemetry.ultrasonicDistanceCm = 30.0;

    robotState.setAutonomousMission(AutonomousMission::RescueZoneAlign);
    robotState.startAutonomous();
    require(
        robotState.setRescueZoneLockedHeading(12.0),
        "ALIGN_ZONE deve salvar o heading que libera a aproximação isolada.");
    robotState.stop();
    robotState.setAutonomousMission(AutonomousMission::RescueZoneApproach);
    robotState.startAutonomous();

    require(
        controller.requiresRescueZoneDetection(robotState.snapshot()),
        "APPROACH_ZONE deve manter a CAM1 ativa para a parada por obstrução.");
    controller.update(
        robotState,
        telemetry,
        false,
        CameraLineSnapshot{},
        ForwardLineSnapshot{},
        ForwardBallSnapshot{});
    RobotSnapshot snapshot = robotState.snapshot();
    require(
        snapshot.mode == "autonomous" && snapshot.left > 0.0 &&
            snapshot.right > 0.0 &&
            snapshot.autonomousStatus.rescueZoneApproachSpeedState == "FAR",
        "APPROACH_ZONE deve avançar sem depender da visão quando as entradas são válidas.");

    telemetry.lastSensorAgeMs =
        config::kRescueZoneUltrasonicFreshnessMs + 1;
    controller.update(
        robotState,
        telemetry,
        false,
        CameraLineSnapshot{},
        ForwardLineSnapshot{},
        ForwardBallSnapshot{});
    snapshot = robotState.snapshot();
    require(
        snapshot.mode == "stopped" && closeTo(snapshot.left, 0.0) &&
            closeTo(snapshot.right, 0.0) &&
            snapshot.autonomousStatus.phase ==
                "rescue_zone_approach_imu_stale",
        "Stale real deve encerrar APPROACH_ZONE com os motores zerados.");
}

void testRescueZoneTriangleGateFollowsInternalPhase()
{
    RobotState robotState;
    MissionController controller;
    Esp32TelemetrySnapshot telemetry = readyTelemetry();
    telemetry.ultrasonicDistanceCm = 20.0;
    RescueZoneSnapshot zones;
    zones.sourceFresh = true;
    zones.sequence = 1;
    zones.timestamp = 1.0;
    zones.green.detected = true;
    zones.green.geometryState = RescueZoneGeometryState::BoundsUnknown;

    robotState.setAutonomousMission(AutonomousMission::RescueZoneTriangle);
    robotState.setRescueZoneTargetColor(RescueZoneTargetColor::Green);
    robotState.startAutonomous();
    require(
        controller.requiresRescueZoneDetection(robotState.snapshot()),
        "TRIÂNGULO deve abrir o gate visual durante SEARCH.");

    controller.update(
        robotState, telemetry, false, CameraLineSnapshot{},
        ForwardLineSnapshot{}, ForwardBallSnapshot{}, zones);
    require(
        controller.requiresRescueZoneDetection(robotState.snapshot()),
        "FOUND deve manter o gate visual aberto para ALIGN.");

    zones.sequence = 2;
    zones.timestamp = 2.0;
    controller.update(
        robotState, telemetry, false, CameraLineSnapshot{},
        ForwardLineSnapshot{}, ForwardBallSnapshot{}, zones);
    require(
        controller.requiresRescueZoneDetection(robotState.snapshot()),
        "ALIGN concluído deve manter o gate visual para proteger APPROACH.");
}
}

// Valida a mesma transição de saída na missão principal e no seletor isolado.
void testExitAcquisitionRestoresFollower()
{
    for (const auto selected : {AutonomousMission::MainMission, AutonomousMission::RescueExit})
    {
        RobotState state;
        state.setAutonomousMission(selected);
        state.startAutonomous();
        auto telemetry = readyTelemetry();
        telemetry.ultrasonicDistanceCm = 100.0;
        MainMission main;
        main.reset(true);
        MissionController controller;
        auto bottom = freshVision(GreenInterpretation::None);
        bottom.silverClassifierFresh = bottom.exitLineUnbranched = bottom.normalSteeringValid = true;
        bottom.lineControlSource = "fusion";
        ForwardLineSnapshot forward;
        forward.sourceFresh = forward.exitAnalysisActive = true;
        forward.exitRunSequence = state.snapshot().autonomousRunSequence;
        forward.exitCandidates[2] = {true, 30.0, 0.5, 2, 1, true, true, 90.0};
        bool acquired = false;
        for (int frame = 1; frame <= 20; ++frame)
        {
            bottom.lineTimestamp = forward.timestamp = frame;
            bottom.lineSequence = bottom.silverSequence = forward.sequence = frame;
            if (selected == AutonomousMission::MainMission)
                main.update(state, telemetry, true, bottom, forward);
            else
                controller.update(state, telemetry, true, bottom, forward, {});
            const auto snapshot = state.snapshot();
            acquired = acquired || snapshot.autonomousStatus.phase == "rescue_exit_acquired";
            if (snapshot.autonomousStatus.phase == "rescue_exit_acquired")
                require(closeTo(snapshot.left, bottom.lineFollowerLeftPower) &&
                            closeTo(snapshot.right, bottom.lineFollowerRightPower),
                        "O handoff deve aplicar o Fusion inferior sem ciclo parado");
            if (acquired && snapshot.autonomousStatus.phase == "line_following")
            {
                requireFollowingLine(snapshot, "Saída deve devolver o controle à CAM0");
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
        }
        require(acquired, "A integração não confirmou a saída");
        requireFollowingLine(state.snapshot(), "Percurso posterior ao resgate");
        // Prata posterior não pode reabrir a sequência de entrada no resgate.
        bottom.silverCandidateDetected = bottom.courseMarkerConfirmed = true;
        bottom.courseMarker = CourseMarker::Gray;
        if (selected == AutonomousMission::MainMission)
            main.update(state, telemetry, true, bottom, forward);
        else
            controller.update(state, telemetry, true, bottom, forward, {});
        requireFollowingLine(state.snapshot(), "Prata no percurso final");
        require(!controller.requiresExitVision(state.snapshot()), "Gate da saída continuou aberto após aquisição");
    }
}

void testExitFailureDiagnosticSurvivesStop()
{
    RobotState state;
    state.setAutonomousMission(AutonomousMission::RescueExit);
    state.startAutonomous();
    state.stop();
    AutonomousStatus failure;
    failure.phase = "rescue_exit_failed";
    failure.exitLastFailure = "Câmera indisponível";
    state.updateAutonomousStatus(failure);
    require(state.snapshot().autonomousStatus.exitLastFailure == failure.exitLastFailure,
            "Stop apagou o diagnóstico de falha da saída");
}

void testPostExitRecoveryUsesFrontDirectionAndReturnsToBottom()
{
    RobotState state;
    state.setAutonomousMission(AutonomousMission::MainMission);
    state.startAutonomous();
    LineCourseMission mission;
    auto telemetry = readyTelemetry();
    const auto frontLeft = forwardVision(1, -0.70);

    auto lost = virtualBlindVision(1, false);
    lost.gapValidationDecision = "LOST";
    mission.update(state, telemetry, true, lost, frontLeft, true);
    lost.lineSequence = 2;
    mission.update(state, telemetry, true, lost, frontLeft, true);
    auto snapshot = state.snapshot();
    require(closeTo(snapshot.left, -config::kForwardAssistSearchSpinPower) &&
                closeTo(snapshot.right, config::kForwardAssistSearchSpinPower) &&
                snapshot.autonomousStatus.phase == "forward_line_recovery",
            "Perda da CAM0 não girou para o lado indicado pela CAM1");

    auto recovered = bottomVision(3, true, true, "LEFT");
    mission.update(state, telemetry, true, recovered, frontLeft, true);
    recovered.lineSequence = 4;
    mission.update(state, telemetry, true, recovered, frontLeft, true);
    snapshot = state.snapshot();
    require(closeTo(snapshot.left, recovered.lineFollowerLeftPower) &&
                closeTo(snapshot.right, recovered.lineFollowerRightPower),
            "CAM0 recuperada não reassumiu após dois frames normais");
}

void testRescueExitKeepsVisionGateActiveInManualMode()
{
    RobotState state;
    MissionController controller;
    state.setAutonomousMission(AutonomousMission::RescueExit);

    state.start();
    require(
        controller.requiresExitVision(state.snapshot()),
        "SAÍDA selecionada deve preservar o overlay no modo Manual");

    state.drive(0.35, -0.25);
    const auto manual = state.snapshot();
    require(
        closeTo(manual.left, 0.35) && closeTo(manual.right, -0.25),
        "O gate visual da saída não deve substituir os comandos manuais");

    state.stop();
    require(
        !controller.requiresExitVision(state.snapshot()),
        "Stop deve fechar imediatamente o gate visual da saída");

    state.start();
    state.emergencyStop();
    require(
        !controller.requiresExitVision(state.snapshot()),
        "E-Stop deve fechar o gate visual da saída");
}

void testExitIpcRejectsInvalidEvidenceWithoutBreakingNormalVision()
{
    const auto directory = std::filesystem::temp_directory_path();
    const auto frontPath = directory / "obr_exit_front_test.json";
    const auto bottomPath = directory / "obr_exit_bottom_test.json";
    CameraMonitor monitor(frontPath.string(), {}, {}, bottomPath.string());
    const auto publish = [&](double silverAge, const std::string& angle, bool exitActive,
                             const std::string& guidanceAngle = "78") {
        const double now = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        std::ofstream front(frontPath);
        front << std::setprecision(17)
              << "{\"forwardPathVersion\":2,\"forwardPathState\":\"ABSENT\","
                 "\"forwardLinePresent\":false,\"forwardLineVisible\":false,"
                 "\"forwardPathConfidence\":0,\"forwardLineSequence\":12,"
                 "\"forwardLineTimestamp\":" << now
              << ",\"exitAnalysisActive\":" << (exitActive ? "true" : "false")
              << ",\"exitRunSequence\":7,\"exitCameraObscured\":true,\"exitCandidates\":{";
        for (int sector = 0; sector < 5; ++sector)
        {
            if (sector) front << ',';
            front << "\"sector" << sector << "\":{\"visible\":true,\"txDegrees\":" << angle
                  << ",\"score\":0.2,\"depthBands\":2,\"nearestBand\":1,\"tapeValid\":false,"
                     "\"guidanceValid\":true,\"guidanceAngleDegrees\":" << guidanceAngle
                  << ",\"entryAngleDegrees\":90,\"entryOffsetNormalized\":0,"
                     "\"entryDepthNormalized\":0.75,"
                     "\"blockedByColor\":false,\"grayNoiseLikely\":false,"
                     "\"solidBlack\":true,"
                     "\"entryPoint\":{\"x\":480,\"y\":400}}";
        }
        front << "}}";
        std::ofstream bottom(bottomPath);
        bottom << std::setprecision(17)
               << "{\"lineFollowerLeftPower\":0.7,\"lineFollowerRightPower\":0.7,"
                  "\"lineNearDetected\":true,\"greenPathBlackValid\":false,\"greenCandidateCount\":0,"
                  "\"greenConfirmed\":false,\"greenInterpretation\":\"SEM_DECISAO\","
                  "\"lineSequence\":12,\"lineTimestamp\":" << now
               << ",\"silverClassifierAvailable\":true,\"silverSequence\":10,\"silverTimestamp\":" << now - silverAge
               << ",\"exitLineUnbranched\":true,\"silverCandidateDetected\":false,"
                  "\"courseMarkerConfirmed\":false,\"courseMarker\":\"NONE\"}";
    };
    publish(0.0, "-12", true);
    const auto front = monitor.forwardLineSnapshot();
    require(front.sourceFresh && front.exitAnalysisActive && front.cameraObscured &&
            front.exitRunSequence == 7 && front.exitCandidates[3].txDegrees == -12.0,
            "Contrato frontal da saída não foi lido corretamente");
    require(monitor.lineSnapshot().silverClassifierFresh && monitor.lineSnapshot().exitLineUnbranched,
            "Contrato inferior da saída não foi lido corretamente");
    publish(0.3, "-12", true);
    require(monitor.lineSnapshot().sourceFresh && monitor.lineSnapshot().silverClassifierFresh,
            "Janela própria de 500 ms da prata dependeu do timeout de 125 ms da CAM0");
    publish(1.0, "-12", true);
    require(monitor.lineSnapshot().sourceFresh && !monitor.lineSnapshot().silverClassifierFresh,
            "Classificador vencido deve bloquear saída sem invalidar o segue-faixa");
    for (const auto angle : {"null", "200", "NaN"})
    {
        publish(0.0, angle, true);
        const auto invalid = monitor.forwardLineSnapshot();
        require(invalid.sourceFresh && !invalid.exitAnalysisActive,
                "Candidata inválida não pode autorizar a saída nem derrubar o contrato normal");
    }
    for (const auto guidanceAngle : {"null", "181", "NaN"})
    {
        publish(0.0, "0", true, guidanceAngle);
        const auto invalid = monitor.forwardLineSnapshot();
        require(invalid.sourceFresh && !invalid.exitAnalysisActive,
                "Ângulo near/far inválido autorizou a saída");
    }
    publish(0.0, "0", false);
    require(!monitor.forwardLineSnapshot().exitAnalysisActive, "Gate fechado liberou candidata");
    {
        std::ifstream file(bottomPath);
        std::ostringstream content;
        content << file.rdbuf();
        auto invalidDecision = content.str();
        const std::string field = "\"silverCandidateDetected\":false";
        invalidDecision.replace(invalidDecision.find(field), field.size(), "\"silverCandidateDetected\":null");
        file.close();
        std::ofstream invalid(bottomPath);
        invalid << invalidDecision;
    }
    const auto invalidBottom = monitor.lineSnapshot();
    require(invalidBottom.sourceFresh && !invalidBottom.silverClassifierFresh,
            "Decisão inválida de prata foi interpretada como ausência de prata");
    std::filesystem::remove(frontPath);
    std::filesystem::remove(bottomPath);
}

// A conclusão bloqueia a execução atual; o START não rearma a mesma faixa.
void testRedFinishStopsAndRearms()
{
    RobotState state;
    CameraLineSnapshot camera;
    camera.sourceFresh = camera.redValid = camera.redConfirmed = true;
    camera.redRatio = 0.2;
    camera.lineSequence = 1;
    camera.lineTimestamp = 1.0;
    state.startAutonomous();
    state.driveAutonomous(0.3, 0.3);
    require(state.observeRedFinish(camera), "Red must finish the run");
    state.driveAutonomous(0.4, 0.4);
    state.driveRawDiagnostic(0.4, 0.4);
    state.updateAutonomousStatus({"running", "Residual action"});
    state.enforceCommandTimeout(std::chrono::milliseconds(0));
    state.stop();
    require(state.snapshot().missionFinished && state.snapshot().left == 0.0 &&
            state.snapshot().right == 0.0 && !state.snapshot().armServoRequested &&
            state.snapshot().autonomousStatus.phase == "mission_finished",
            "Residual commands must preserve completed stop");
    require(!state.setAutonomousServoPose({}), "Completed run must reject servo pose");
    require(state.tryStartAutonomous(), "Physical START must allow another run");
    require(!state.snapshot().missionFinished, "START must clear completion");
    camera.lineSequence++; camera.lineTimestamp += 0.02;
    require(!state.observeRedFinish(camera), "Same red patch must remain disarmed");
    state.driveAutonomous(0.2, 0.2);
    require(state.snapshot().left > 0.0, "New run must be able to leave red");
    camera.redConfirmed = false; camera.redClearConfirmed = true; camera.redRatio = 0.0;
    camera.sourceFresh = false;
    camera.lineSequence++; camera.lineTimestamp += 0.02;
    state.observeRedFinish(camera);
    camera.sourceFresh = true; camera.redConfirmed = true;
    camera.redClearConfirmed = false; camera.redRatio = 0.2;
    camera.lineSequence++; camera.lineTimestamp += 0.02;
    require(!state.observeRedFinish(camera), "Stale absence cannot rearm");
    camera.redConfirmed = false; camera.redClearConfirmed = true; camera.redRatio = 0.0;
    camera.lineSequence++; camera.lineTimestamp += 0.02;
    state.observeRedFinish(camera);
    camera.redConfirmed = true; camera.redClearConfirmed = false; camera.redRatio = 0.2;
    require(!state.observeRedFinish(camera), "Repeated IPC cannot confirm");
    camera.lineSequence++; camera.lineTimestamp += 0.02;
    require(state.observeRedFinish(camera), "New patch must finish after rearm");
    state.setAutonomousMission(AutonomousMission::DriveDistance);
    require(!state.snapshot().missionFinished && state.snapshot().mode == "stopped",
            "Mission selection must clear completion without movement");
    state.start();
    state.drive(0.2, 0.2);
    require(state.snapshot().left > 0.0, "Manual START must remain available");
    state.emergencyStop();
    require(!state.tryStartAutonomous() && state.snapshot().left == 0.0,
            "Physical START must not clear emergency stop");
}

void testRedFinishWaitsUntilRescueEnds()
{
    RobotState state;
    CameraLineSnapshot camera;
    camera.sourceFresh = camera.redValid = camera.redConfirmed = true;
    camera.redRatio = 0.2;
    camera.lineSequence = 1;
    camera.lineTimestamp = 1.0;
    state.startAutonomous();
    state.updateAutonomousStatus({"rescue_exit_corner_backing", "Voltando ao corner"});

    require(!state.observeRedFinish(camera) && !state.snapshot().missionFinished,
            "Red must not finish while the rescue mission is active");

    state.updateAutonomousStatus({"line_following", "Seguindo linha"});
    ++camera.lineSequence;
    camera.lineTimestamp += 0.02;
    require(state.observeRedFinish(camera),
            "Red must work again immediately after leaving the rescue phase");
}

int main(int argc, char** argv)
{
    try
    {
        if (argc > 1 && std::string(argv[1]) == "--obstacle-only")
        {
            testObstacleContinuationBandIpcFailsSafe();
            testObstaclePausesIfBottomCameraBecomesUnavailable();
            testObstacleRespectsStopAndSafetyPriority();
            std::cout << "obstacle_integration_test: OK\n";
            return 0;
        }
        testRescueExitWristTransitionPreservesOtherChannels();
        testRescueExitWristUsesLatestRescuePose();
        testRescueExitWristPreservesMechanicalGuard();
        testObstacleContinuationBandIpcFailsSafe();
        if (argc > 1 && std::string(argv[1]) == "--localized-fixes-only")
        {
            std::cout << "localized_fixes_test: OK\n";
            return 0;
        }
        testRedFinishStopsAndRearms();
        testRedFinishWaitsUntilRescueEnds();
        if (argc > 1 && std::string(argv[1]) == "--red-only")
        {
            std::cout << "red_finish_test: OK\n";
            return 0;
        }
        if (argc > 1 && std::string(argv[1]) == "--green-only")
        {
            testNonReturnGreenDoesNotStartSequence();
            testLateralGreenWaitsForMeasuredConfirmation();
            testGreenConfirmationPreservesLowBaseAndStopsOnImuLoss();
            testTurnAroundOverridesProvisionalLateralGreen();
            testOppositeGreenDoesNotRestartOrPromoteLatchedEvent();
            testConfirmedLateralGreenCannotReturnToFalse();
            testRetainedLateralConfirmationPrecedesMissingCandidate();
            testTransientCandidateLossDuringBrakingKeepsCurrentEvent();
            testBrakingDoesNotConsumeVisualDiscardWindow();
            testValidBlackCandidateGetsBoundedExtraConfirmationWait();
            testGreenAcceptsOnlyConfirmedSideLineBeforeMinimumYaw();
            testConfirmationStartsAfterInitialStopAndMeasuresRealDistance();
            testLateralGreenCentersOnlyWithCompleteLocalGeometry();
            testLateralGreenSkipsCenteringWhenGeometryIsIncomplete();
            testLeftGreenReversesOnlyAfterVisualLine();
            testGreenReverseIgnoresBlindLinePlaceholder();
            std::cout << "green_maneuver_test: OK\n";
            return 0;
        }
        testExitAcquisitionRestoresFollower();
        testCornerYawModeOnlyTurnsAndStopsOnImuLoss();
        testExitFailureDiagnosticSurvivesStop();
        testRescueExitKeepsVisionGateActiveInManualMode();
        testExitIpcRejectsInvalidEvidenceWithoutBreakingNormalVision();
        if (argc > 1 && std::string(argv[1]) == "--exit-only")
        {
            std::cout << "rescue_exit_integration_test: OK\n";
            return 0;
        }
        testBallIpcCandidateDirectionIsOptionalAndCannotConfirmTarget();
        testNormalLineFollowerCommandsMotors();
        testNormalLineFollowerCompensatesRampPower();
        testRampCompensationRequiresFreshValidImu();
        testRampCompensationPreservesSpecialLineCommands();
        testRampCompensationClampsFinalMotorCommands();
        testNonReturnGreenDoesNotStartSequence();
        testLateralGreenWaitsForMeasuredConfirmation();
        testGreenConfirmationPreservesLowBaseAndStopsOnImuLoss();
        testTurnAroundOverridesProvisionalLateralGreen();
        testOppositeGreenDoesNotRestartOrPromoteLatchedEvent();
        testConfirmedLateralGreenCannotReturnToFalse();
        testRetainedLateralConfirmationPrecedesMissingCandidate();
        testTransientCandidateLossDuringBrakingKeepsCurrentEvent();
        testBrakingDoesNotConsumeVisualDiscardWindow();
        testValidBlackCandidateGetsBoundedExtraConfirmationWait();
        testGreenAcceptsOnlyConfirmedSideLineBeforeMinimumYaw();
        testConfirmationStartsAfterInitialStopAndMeasuresRealDistance();
        testLateralGreenCentersOnlyWithCompleteLocalGeometry();
        testLateralGreenSkipsCenteringWhenGeometryIsIncomplete();
        testLeftGreenReversesOnlyAfterVisualLine();
        testGreenReverseIgnoresBlindLinePlaceholder();
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
        testPostExitRecoveryUsesFrontDirectionAndReturnsToBottom();
        testGapAndGreenKeepNativeAuthorityWithoutImu();
        testForwardIpcAcceptsGeometryWithoutPowersAndRejectsInvalidSources();
        testUnavailableCameraStopsMission();
        testTransientLineIpcLossPausesAndResumesMission();
        testObstaclePausesIfBottomCameraBecomesUnavailable();
        testObstacleRespectsStopAndSafetyPriority();
        testConfirmedCourseMarkersControlOnlyExpectedPhase();
        testRescueAlignmentModeKeepsExistingMotorAuthority();
        testRescueZoneDetectionOnlyKeepsMotorsStopped();
        testRescueZoneAlignKeepsDetectionGateActive();
        testRescueZoneApproachUsesStoredHeadingWithCameraStopGate();
        testRescueZoneTriangleGateFollowsInternalPhase();
        std::cout << "main_mission_test: OK\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "main_mission_test: " << error.what() << '\n';
        return 1;
    }
}
