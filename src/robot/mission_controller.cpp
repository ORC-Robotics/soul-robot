#include "obr/mission_controller.h"

#include "obr/config.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

namespace
{
AutonomousStatus makeAutonomousStatus(
    const std::string& phase,
    const std::string& action,
    double progressPercent = 0.0)
{
    AutonomousStatus status;
    status.phase = phase;
    status.action = action;
    status.progressPercent = progressPercent;
    return status;
}

AutonomousStatus makeDistanceStatus(
    const std::string& phase,
    const std::string& action,
    double targetDistanceCm,
    double leftDistanceCm,
    double rightDistanceCm,
    double progressPercent)
{
    AutonomousStatus status = makeAutonomousStatus(
        phase, action, progressPercent);
    status.targetDistanceCm = targetDistanceCm;
    status.leftDistanceCm = leftDistanceCm;
    status.rightDistanceCm = rightDistanceCm;
    status.averageDistanceCm = (leftDistanceCm + rightDistanceCm) * 0.5;
    return status;
}

struct DriveDistanceCommand
{
    double left = config::kDriveDistanceBaseCommandPower;
    double right = config::kDriveDistanceBaseCommandPower;
};

DriveDistanceCommand calculateDriveDistanceCommand(
    double leftDistanceCm,
    double rightDistanceCm)
{
    const double distanceDifferenceCm = leftDistanceCm - rightDistanceCm;
    const double differenceMagnitude = std::abs(distanceDifferenceCm);
    if (differenceMagnitude <= config::kDriveDistanceBalanceDeadbandCm)
    {
        return {};
    }

    const double correction = std::clamp(
        (differenceMagnitude - config::kDriveDistanceBalanceDeadbandCm) *
            config::kDriveDistanceBalanceGainPerCm,
        0.0,
        config::kDriveDistanceMaximumBalanceCorrection);
    DriveDistanceCommand command;
    if (distanceDifferenceCm > 0.0)
    {
        // O lado esquerdo avançou mais: desacelera-o e reforça o direito.
        command.left = std::clamp(
            config::kDriveDistanceBaseCommandPower - correction,
            config::kDriveDistanceMinimumCommandPower,
            config::kDriveDistanceMaximumCommandPower);
        command.right = std::clamp(
            config::kDriveDistanceBaseCommandPower + correction,
            config::kDriveDistanceMinimumCommandPower,
            config::kDriveDistanceMaximumCommandPower);
    }
    else
    {
        // O lado direito avançou mais: aplica a mesma correção no sentido oposto.
        command.left = std::clamp(
            config::kDriveDistanceBaseCommandPower + correction,
            config::kDriveDistanceMinimumCommandPower,
            config::kDriveDistanceMaximumCommandPower);
        command.right = std::clamp(
            config::kDriveDistanceBaseCommandPower - correction,
            config::kDriveDistanceMinimumCommandPower,
            config::kDriveDistanceMaximumCommandPower);
    }
    return command;
}
}

bool MissionController::requiresExitVision(const RobotSnapshot& snapshot) const
{
    if (snapshot.emergencyStop)
    {
        return false;
    }

    // No modo Manual, a missão isolada mantém apenas o diagnóstico visual
    // da saída. Este gate não concede autoridade aos motores: o movimento
    // continua dependendo exclusivamente dos comandos manuais do RobotState.
    if (snapshot.mode == "manual")
    {
        return snapshot.autonomousMission == AutonomousMission::RescueExit;
    }

    return snapshot.mode == "autonomous" &&
           (snapshot.autonomousMission == AutonomousMission::MainMission ||
            snapshot.autonomousMission == AutonomousMission::RescueExit) &&
           mainMission_.requiresExitVision();
}

bool MissionController::requiresForwardBallDetection(
    const RobotSnapshot& snapshot) const
{
    if (snapshot.mode != "autonomous" || snapshot.emergencyStop)
    {
        return false;
    }
    if (snapshot.autonomousMission == AutonomousMission::RescueArea)
    {
        return true;
    }
    return snapshot.autonomousMission == AutonomousMission::MainMission &&
           mainMission_.requiresRescueVision();
}

bool MissionController::requiresRescueZoneDetection(
    const RobotSnapshot& snapshot) const
{
    // A percepção permanece ativa no Manual para permitir o enquadramento
    // controlado pelo operador. O gate não concede autoridade aos motores:
    // somente RobotState::drive() aceita movimento nesse modo.
    const bool operationMode =
        snapshot.mode == "autonomous" || snapshot.mode == "manual";
    if (!operationMode || snapshot.emergencyStop)
    {
        return false;
    }
    if (snapshot.autonomousMission == AutonomousMission::RescueZoneTriangle)
    {
        return snapshot.mode == "manual" ||
               rescueZoneTriangleMission_.requiresRescueZoneDetection();
    }
    if (snapshot.mode == "autonomous" &&
        (snapshot.autonomousMission == AutonomousMission::MainMission ||
         snapshot.autonomousMission == AutonomousMission::RescueExit))
    {
        // O modo isolado reutiliza a saída fixa da missão principal.
        // Nessa etapa, a rota já está definida e não exige detector de triângulos.
        return mainMission_.requiresRescueZoneDetection();
    }
    return snapshot.autonomousMission ==
               AutonomousMission::RescueZoneDetection ||
           snapshot.autonomousMission == AutonomousMission::RescueZoneSearch ||
           snapshot.autonomousMission == AutonomousMission::RescueZoneAlign ||
           snapshot.autonomousMission == AutonomousMission::RescueZoneApproach;
}

std::uint64_t MissionController::forwardBallTargetSequence(
    const RobotSnapshot& snapshot) const
{
    if (snapshot.autonomousMission == AutonomousMission::MainMission)
    {
        return mainMission_.rescueBallTargetSequence(
            snapshot.autonomousRunSequence);
    }
    return snapshot.autonomousRunSequence;
}

const char* MissionController::forwardBallTargetType(
    const RobotSnapshot& snapshot) const
{
    if (snapshot.autonomousMission == AutonomousMission::MainMission)
    {
        return mainMission_.rescueBallTargetType();
    }
    // O modo isolado continua aceitando ambos os tipos para diagnóstico.
    return "any";
}

void MissionController::update(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    bool cameraReady,
    const CameraLineSnapshot& cameraLineSnapshot,
    const ForwardLineSnapshot& forwardLineSnapshot,
    const ForwardBallSnapshot& forwardBallSnapshot,
    const RescueZoneSnapshot& rescueZoneSnapshot)
{
    const RobotSnapshot snapshot = robotState.snapshot();
    if (snapshot.missionFinished || snapshot.mode != "autonomous")
    {
        resetMissionState();
        activeAutonomousRunSequence_ = 0;
        return;
    }

    if (activeAutonomousRunSequence_ != snapshot.autonomousRunSequence)
    {
        // Cada partida recebe um identificador para nunca reutilizar uma fase
        // interna de uma execução anterior depois de Stop seguido de Auto.
        resetMissionState();
        activeAutonomousRunSequence_ = snapshot.autonomousRunSequence;
        if (snapshot.autonomousMission == AutonomousMission::RescueExit)
            mainMission_.reset(true);
    }

    switch (snapshot.autonomousMission)
    {
    case AutonomousMission::TurnRight90:
        distancePhase_ = DistancePhase::Idle;
        updateTurnRight90(robotState, esp32Telemetry);
        return;
    case AutonomousMission::RescueCornerYawTest:
        distancePhase_ = DistancePhase::Idle;
        updateCornerYawTest(robotState, esp32Telemetry);
        return;
    case AutonomousMission::RescueExitWithReverse:
        distancePhase_ = DistancePhase::Idle;
        updateCornerYawTestWithReverse(robotState, esp32Telemetry);
        return;
    case AutonomousMission::DriveDistance:
        testTurnController_.reset();
        rescueAreaMission_.reset();
        obstacleAvoidanceTest_.reset();
        updateDriveDistance(
            robotState, esp32Telemetry, snapshot.driveDistanceTargetCm);
        return;
    case AutonomousMission::RescueArea:
        testTurnController_.reset();
        obstacleAvoidanceTest_.reset();
        distancePhase_ = DistancePhase::Idle;
        mainMission_.reset();
        updateRescueArea(
            robotState, esp32Telemetry, forwardBallSnapshot);
        return;
    case AutonomousMission::RescueZoneDetection:
        testTurnController_.reset();
        rescueAreaMission_.reset();
        obstacleAvoidanceTest_.reset();
        distancePhase_ = DistancePhase::Idle;
        mainMission_.reset();
        updateRescueZoneDetection(robotState, esp32Telemetry);
        return;
    case AutonomousMission::RescueZoneAlign:
        testTurnController_.reset();
        rescueAreaMission_.reset();
        obstacleAvoidanceTest_.reset();
        distancePhase_ = DistancePhase::Idle;
        mainMission_.reset();
        updateRescueZoneAlign(
            robotState, snapshot, esp32Telemetry, rescueZoneSnapshot);
        return;
    case AutonomousMission::RescueZoneSearch:
        testTurnController_.reset();
        rescueAreaMission_.reset();
        obstacleAvoidanceTest_.reset();
        distancePhase_ = DistancePhase::Idle;
        mainMission_.reset();
        updateRescueZoneSearch(robotState, snapshot, rescueZoneSnapshot);
        return;
    case AutonomousMission::RescueZoneApproach:
        testTurnController_.reset();
        rescueAreaMission_.reset();
        obstacleAvoidanceTest_.reset();
        distancePhase_ = DistancePhase::Idle;
        mainMission_.reset();
        updateRescueZoneApproach(
            robotState, snapshot, esp32Telemetry, rescueZoneSnapshot);
        return;
    case AutonomousMission::RescueZoneTriangle:
        testTurnController_.reset();
        rescueAreaMission_.reset();
        obstacleAvoidanceTest_.reset();
        distancePhase_ = DistancePhase::Idle;
        mainMission_.reset();
        updateRescueZoneTriangle(
            robotState, snapshot, esp32Telemetry, rescueZoneSnapshot);
        return;
    case AutonomousMission::ObstacleAvoidance:
        testTurnController_.reset();
        rescueAreaMission_.reset();
        distancePhase_ = DistancePhase::Idle;
        mainMission_.reset();
        updateObstacleAvoidance(
            robotState,
            esp32Telemetry,
            cameraLineSnapshot,
            forwardLineSnapshot);
        return;
    case AutonomousMission::ServoInitialize:
        updateServoRoutine(
            robotState, ServoRoutineKind::Initialize, snapshot, esp32Telemetry);
        return;
    case AutonomousMission::ServoCapture:
        updateServoRoutine(
            robotState, ServoRoutineKind::Capture, snapshot, esp32Telemetry);
        return;
    case AutonomousMission::ServoInternalStorage:
        updateServoRoutine(
            robotState, ServoRoutineKind::InternalStorage, snapshot,
            esp32Telemetry);
        return;
    case AutonomousMission::ServoDeposit:
        updateServoRoutine(
            robotState, ServoRoutineKind::Deposit, snapshot, esp32Telemetry);
        return;
    case AutonomousMission::ServoFullSequence:
        updateServoRoutine(
            robotState, ServoRoutineKind::FullSequence, snapshot,
            esp32Telemetry);
        return;
    case AutonomousMission::ServoFullSequenceTwo:
        updateServoRoutine(
            robotState, ServoRoutineKind::FullSequenceTwo, snapshot,
            esp32Telemetry);
        return;
    case AutonomousMission::RescueExit:
    case AutonomousMission::MainMission:
    default:
        testTurnController_.reset();
        rescueAreaMission_.reset();
        obstacleAvoidanceTest_.reset();
        distancePhase_ = DistancePhase::Idle;
        mainMission_.update(
            robotState,
            esp32Telemetry,
            cameraReady,
            cameraLineSnapshot,
            forwardLineSnapshot,
            forwardBallSnapshot,
            rescueZoneSnapshot,
            activeAutonomousRunSequence_);
        return;
    }
}

void MissionController::updateServoRoutine(
    RobotState& robotState,
    ServoRoutineKind kind,
    const RobotSnapshot& snapshot,
    const Esp32TelemetrySnapshot& esp32Telemetry)
{
    // As rotinas nunca comandam tração. O zero é renovado em todos os ciclos
    // para preservar também o watchdog normal dos motores.
    robotState.driveAutonomous(0.0, 0.0);

    if (!esp32Telemetry.readyForOperation() || !esp32Telemetry.pca9685Ok)
    {
        // Sem confirmação recente do driver, continuar contando o tempo poderia
        // pular um movimento que nunca chegou ao mecanismo.
        robotState.stop();
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "servo_driver_lost",
            "Rotina interrompida: PCA9685 ou comunicação indisponível"));
        return;
    }

    const ServoRoutineOutput output = servoRoutine_.update(
        kind,
        snapshot.autonomousRunSequence,
        snapshot.servoRoutineConfirmationSequence,
        snapshot.servoPose,
        std::chrono::steady_clock::now());
    if (output.poseRequested)
    {
        robotState.setAutonomousServoPose(output.pose);
    }
    if (output.releaseGripper)
    {
        robotState.setAutonomousServoOutputEnabled(ServoId::Gripper, false);
    }
    robotState.setServoRoutineInternalObjectStored(
        output.internalObjectStored);

    AutonomousStatus status = makeAutonomousStatus(
        output.phase, output.action, output.progressPercent);
    status.servoRoutineWaitingForConfirmation =
        output.waitingForConfirmation;
    robotState.updateAutonomousStatus(status);
}

void MissionController::updateRescueArea(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    const ForwardBallSnapshot& forwardBallSnapshot)
{
    const RescueAreaOutput output = rescueAreaMission_.update(
        forwardBallSnapshot,
        esp32Telemetry,
        activeAutonomousRunSequence_,
        false,
        true);
    // A ausência ou expiração da visão produz zero neste mesmo ciclo.
    // O RobotState ainda aplica clamp, E-Stop e timeout antes dos motores.
    robotState.driveAutonomous(output.leftPower, output.rightPower);
    if (output.completed || output.failed)
    {
        // Conclusão e falha são estados terminais: parar encerra a missão e
        // impede que a visão pesada continue consumindo CPU sem necessidade.
        robotState.stop();
    }
    robotState.updateAutonomousStatus(output.status);
}

void MissionController::updateRescueZoneDetection(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry)
{
    // Este modo valida somente percepção e renova zero em todos os ciclos.
    // Nenhum valor do IPC das zonas possui autoridade sobre a tração.
    robotState.driveAutonomous(0.0, 0.0);
    AutonomousStatus status = makeAutonomousStatus(
        "rescue_zone_detection_active",
        "Detectando áreas verde e vermelha; motores parados");
    status.rescueZoneUltrasonicFresh =
        esp32Telemetry.sensorFresh && esp32Telemetry.lastSensorAgeMs >= 0 &&
        esp32Telemetry.lastSensorAgeMs <=
            config::kRescueZoneUltrasonicFreshnessMs;
    status.rescueZoneUltrasonicValid =
        std::isfinite(esp32Telemetry.ultrasonicDistanceCm) &&
        esp32Telemetry.ultrasonicDistanceCm >=
            config::kRescueZoneUltrasonicMinimumCm &&
        esp32Telemetry.ultrasonicDistanceCm <=
            config::kRescueZoneUltrasonicMaximumCm;
    if (status.rescueZoneUltrasonicValid)
    {
        status.rescueZoneUltrasonicDistanceCm =
            esp32Telemetry.ultrasonicDistanceCm;
    }
    robotState.updateAutonomousStatus(status);
}

void MissionController::updateRescueZoneAlign(
    RobotState& robotState,
    const RobotSnapshot& snapshot,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    const RescueZoneSnapshot& rescueZoneSnapshot)
{
    RescueZoneAlignOutput output = rescueZoneAlignMission_.update(
        rescueZoneSnapshot,
        esp32Telemetry,
        snapshot.rescueZoneTargetColor);
    // O ultrassônico continua disponível somente para telemetria e overlay.
    // ALIGN_ZONE não consulta essa distância para decidir ou comandar motores.
    output.status.rescueZoneUltrasonicFresh =
        esp32Telemetry.sensorFresh && esp32Telemetry.lastSensorAgeMs >= 0 &&
        esp32Telemetry.lastSensorAgeMs <=
            config::kRescueZoneUltrasonicFreshnessMs;
    output.status.rescueZoneUltrasonicValid =
        std::isfinite(esp32Telemetry.ultrasonicDistanceCm) &&
        esp32Telemetry.ultrasonicDistanceCm >=
            config::kRescueZoneUltrasonicMinimumCm &&
        esp32Telemetry.ultrasonicDistanceCm <=
            config::kRescueZoneUltrasonicMaximumCm;
    if (output.status.rescueZoneUltrasonicValid)
    {
        output.status.rescueZoneUltrasonicDistanceCm =
            esp32Telemetry.ultrasonicDistanceCm;
    }

    robotState.driveAutonomous(output.leftPower, output.rightPower);
    robotState.updateAutonomousStatus(output.status);
    if (output.completed || output.failed)
    {
        if (output.completed && std::isfinite(
                                    output.status.rescueZoneAlignLockedHeading))
        {
            robotState.setRescueZoneLockedHeading(
                output.status.rescueZoneAlignLockedHeading);
        }
        // Toda saída terminal encerra o modo depois de renovar PWM zero.
        robotState.stop();
        robotState.updateAutonomousStatus(output.status);
    }
}

void MissionController::updateRescueZoneSearch(
    RobotState& robotState,
    const RobotSnapshot& snapshot,
    const RescueZoneSnapshot& rescueZoneSnapshot)
{
    const RescueZoneSearchOutput output = rescueZoneSearchMission_.update(
        rescueZoneSnapshot, snapshot.rescueZoneTargetColor);
    // O pivot visual não usa encoders; o zero publicado em stale ou FOUND
    // continua passando pelo mesmo watchdog e E-Stop dos demais modos.
    robotState.driveAutonomous(output.leftPower, output.rightPower, false);
    robotState.updateAutonomousStatus(output.status);
    if (output.completed)
    {
        robotState.stop();
        robotState.updateAutonomousStatus(output.status);
    }
}

void MissionController::updateRescueZoneApproach(
    RobotState& robotState,
    const RobotSnapshot& snapshot,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    const RescueZoneSnapshot& rescueZoneSnapshot)
{
    const RescueZoneApproachOutput output = rescueZoneApproachMission_.update(
        snapshot.rescueZoneLockedHeading,
        esp32Telemetry,
        std::chrono::steady_clock::now(),
        rescueZoneSnapshot,
        snapshot.rescueZoneTargetColor);
    // A correção diferencial é intencional; o sincronismo por encoder não
    // participa da aproximação orientada pelo heading.
    robotState.driveAutonomous(
        output.leftPower, output.rightPower, false);
    robotState.updateAutonomousStatus(output.status);
    if (output.completed || output.failed)
    {
        // Toda saída terminal renova PWM zero antes de encerrar o modo.
        robotState.stop();
        robotState.updateAutonomousStatus(output.status);
    }
}

void MissionController::updateRescueZoneTriangle(
    RobotState& robotState,
    const RobotSnapshot& snapshot,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    const RescueZoneSnapshot& rescueZoneSnapshot)
{
    const RescueZoneTriangleOutput output = rescueZoneTriangleMission_.update(
        rescueZoneSnapshot,
        esp32Telemetry,
        snapshot.rescueZoneTargetColor);
    robotState.driveAutonomous(output.leftPower, output.rightPower, false);
    if (output.lockedHeadingUpdated)
    {
        robotState.setRescueZoneLockedHeading(output.lockedHeading);
    }
    robotState.updateAutonomousStatus(output.status);
    if (output.completed || output.failed)
    {
        robotState.stop();
        robotState.updateAutonomousStatus(output.status);
    }
}

void MissionController::updateObstacleAvoidance(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    const CameraLineSnapshot& cameraLineSnapshot,
    const ForwardLineSnapshot& forwardLineSnapshot)
{
    const ObstacleAvoidanceOutput output = obstacleAvoidanceTest_.update(
        esp32Telemetry,
        cameraLineSnapshot,
        true,
        forwardLineSnapshot);
    robotState.driveAutonomous(output.leftPower, output.rightPower);

    AutonomousStatus status = makeAutonomousStatus(
        output.phase,
        output.action,
        output.progressPercent);
    status.targetDistanceCm = output.targetDistanceCm;
    status.leftDistanceCm = output.leftDistanceCm;
    status.rightDistanceCm = output.rightDistanceCm;
    status.averageDistanceCm =
        (output.leftDistanceCm + output.rightDistanceCm) * 0.5;
    status.obstacleYawBase = output.yawBase;
    status.obstacleLeftClearance = output.leftClearance;
    status.obstacleRightClearance = output.rightClearance;
    status.obstacleSelectedSide = output.selectedSide;
    status.cameraBlackLeft = output.cameraBlackLeft;
    status.cameraBlackRight = output.cameraBlackRight;
    status.cameraBlackLeftFrames = output.cameraBlackLeftFrames;
    status.cameraBlackRightFrames = output.cameraBlackRightFrames;
    status.selectedSideSource = output.selectedSideSource;
    status.rawBestParabolaSide = output.rawBestParabolaSide;
    status.bestParabolaSide = output.bestParabolaSide;
    status.bestParabolaScore = output.bestParabolaScore;
    status.bestParabolaLeftBlack = output.bestParabolaLeftBlack;
    status.bestParabolaRightBlack = output.bestParabolaRightBlack;
    status.bestParabolaSequence = output.bestParabolaSequence;
    status.bestParabolaSideValid = output.bestParabolaSideValid;
    status.nearForwardLineVisible = output.nearForwardLineVisible;
    status.nearForwardLineVotes = output.nearForwardLineVotes;
    status.nearForwardLineSamples = output.nearForwardLineSamples;
    status.case3Armed = output.case3Armed;
    status.case3FusionAcquireTime = output.case3FusionAcquireTime;
    status.case3TimeRemainingMs = output.case3TimeRemainingMs;
    if (output.completed || output.failed)
    {
        // O modo isolado termina parado como as demais ferramentas de teste.
        robotState.stop();
        robotState.updateAutonomousStatus(status);
        return;
    }
    robotState.updateAutonomousStatus(status);
}

void MissionController::updateTurnRight90(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry)
{
    if (!testTurnController_.active())
    {
        if (!ImuTurnController::imuReady(esp32Telemetry))
        {
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeAutonomousStatus(
                "waiting_imu", "Aguardando referência angular do MPU6050"));
            return;
        }
        testTurnController_.start(
            config::kTurn90TargetDegrees,
            ImuTurnDirection::Right,
            esp32Telemetry);
    }

    const ImuTurnOutput output = testTurnController_.update(esp32Telemetry);
    robotState.driveAutonomous(output.leftPower, output.rightPower);
    if (output.result == ImuTurnResult::Completed)
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "completed", "Giro de 90° concluído", 100.0));
        return;
    }

    AutonomousStatus status = makeAutonomousStatus(
        output.phase, output.action, output.progressPercent);
    if (output.result == ImuTurnResult::Failed)
    {
        robotState.stop();
        robotState.updateAutonomousStatus(status);
        return;
    }
    robotState.updateAutonomousStatus(status);
}

void MissionController::updateCornerYawTest(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& telemetry)
{
    // Os três alvos são relativos ao yaw capturado no início, tanto no
    // triângulo vermelho quanto no verde. Este teste nunca avança em linha reta.
    constexpr std::array<double, 3> offsets = {
        config::kRescueExitFirstStraightYawDegrees,
        config::kRescueExitSecondStraightYawDegrees,
        config::kRescueExitThirdStraightYawDegrees};
    const auto now = std::chrono::steady_clock::now();
    robotState.driveAutonomous(0.0, 0.0);

    if (!cornerYawReferenceValid_)
    {
        if (!telemetry.readyForOperation() ||
            !ImuTurnController::imuReady(telemetry))
        {
            robotState.updateAutonomousStatus(makeAutonomousStatus(
                "corner_yaw_waiting_imu", "Parado: aguardando IMU e ESP32"));
            return;
        }
        cornerYawReferenceDegrees_ = telemetry.yawZDeg;
        cornerYawReferenceValid_ = true;
    }

    if (!telemetry.readyForOperation() ||
        !ImuTurnController::imuReady(telemetry))
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "corner_yaw_failed", "Teste interrompido: IMU ou ESP32 indisponível"));
        return;
    }

    if (cornerYawIndex_ >= static_cast<int>(offsets.size()))
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "corner_yaw_completed", "Três direções verificadas; motores parados", 100.0));
        return;
    }

    const int offsetDegrees = static_cast<int>(offsets[cornerYawIndex_]);
    const std::string target = "Alvo " + std::to_string(cornerYawIndex_ + 1) +
        "/3 (" + (offsetDegrees > 0 ? "+" : "") +
        std::to_string(offsetDegrees) + "°): ";

    if (cornerYawPhase_ == CornerYawPhase::Holding)
    {
        if (now - cornerYawHoldStartedAt_ <
            std::chrono::milliseconds(config::kRescueCornerYawHoldMs))
        {
            robotState.updateAutonomousStatus(makeAutonomousStatus(
                "corner_yaw_holding", target + "parado por 2 segundos"));
            return;
        }
        ++cornerYawIndex_;
        if (cornerYawIndex_ == static_cast<int>(offsets.size()))
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeAutonomousStatus(
                "corner_yaw_completed", "Três direções verificadas; motores parados", 100.0));
            return;
        }
        cornerYawPhase_ = CornerYawPhase::Ready;
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "corner_yaw_next", "Pausa concluída; preparando o próximo yaw"));
        return;
    }

    if (cornerYawPhase_ == CornerYawPhase::Ready)
    {
        const double targetYaw = std::remainder(
            cornerYawReferenceDegrees_ + offsets[cornerYawIndex_], 360.0);
        const double turnDegrees = std::remainder(
            targetYaw - telemetry.yawZDeg, 360.0);
        if (std::abs(turnDegrees) <= config::kBallApproachStartToleranceDegrees)
        {
            cornerYawPhase_ = CornerYawPhase::Holding;
            cornerYawHoldStartedAt_ = now;
            robotState.updateAutonomousStatus(makeAutonomousStatus(
                "corner_yaw_holding", target + "já alinhado; parado por 2 segundos"));
            return;
        }
        if (!testTurnController_.start(
                std::abs(turnDegrees),
                turnDegrees < 0.0 ? ImuTurnDirection::Left : ImuTurnDirection::Right,
                telemetry,
                config::kBallApproachStartToleranceDegrees,
                0, 0,
                config::kRescueExitTurnPower,
                config::kRescueExitApproachTimeoutMs,
                now))
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeAutonomousStatus(
                "corner_yaw_failed", target + "não foi possível iniciar o giro"));
            return;
        }
        cornerYawPhase_ = CornerYawPhase::Turning;
    }

    const ImuTurnOutput turn = testTurnController_.update(telemetry, now);
    if (turn.result == ImuTurnResult::Failed)
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "corner_yaw_failed", target + turn.action));
        return;
    }
    if (turn.result == ImuTurnResult::Completed)
    {
        cornerYawPhase_ = CornerYawPhase::Holding;
        cornerYawHoldStartedAt_ = now;
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "corner_yaw_holding", target + "parado por 2 segundos"));
        return;
    }
    robotState.driveAutonomous(turn.leftPower, turn.rightPower);
    robotState.updateAutonomousStatus(makeAutonomousStatus(
        "corner_yaw_turning", target + turn.action, turn.progressPercent));
}

void MissionController::updateCornerYawTestWithReverse(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& telemetry)
{
    const auto now = std::chrono::steady_clock::now();
    if (!cornerYawInitialReverseCompleted_)
    {
        if (cornerYawInitialReverse_.idle())
        {
            cornerYawInitialReverse_.start(
                config::kRescueFinalDepositReverseDistanceCm,
                config::kRescuePostDepositReversePower, -1, now);
        }

        const EncoderDistanceOutput reverse = cornerYawInitialReverse_.update(
            telemetry, now, "corner_yaw_initial_reverse",
            "Recuando 40 cm antes de testar os yaws das quinas");
        robotState.driveAutonomous(reverse.leftPower, reverse.rightPower);
        robotState.updateAutonomousStatus(reverse.status);
        if (reverse.failed)
        {
            robotState.stop();
            robotState.updateAutonomousStatus(reverse.status);
            return;
        }
        if (reverse.completed)
        {
            cornerYawInitialReverseCompleted_ = true;
            cornerYawReferenceValid_ = false;
            robotState.driveAutonomous(0.0, 0.0);
        }
        return;
    }

    updateCornerYawTest(robotState, telemetry);
}

void MissionController::updateDriveDistance(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    double targetDistanceCm)
{
    const auto now = std::chrono::steady_clock::now();
    const bool encoderReady = esp32Telemetry.sensorFresh &&
                              esp32Telemetry.lastSensorAgeMs >= 0 &&
                              esp32Telemetry.lastSensorAgeMs <=
                                  config::kDriveDistanceEncoderFreshnessMs &&
                              std::isfinite(esp32Telemetry.leftEncoderRate) &&
                              std::isfinite(esp32Telemetry.rightEncoderRate);

    if (distancePhase_ == DistancePhase::Idle)
    {
        if (!std::isfinite(targetDistanceCm) ||
            targetDistanceCm < config::kDriveDistanceMinimumTargetCm ||
            targetDistanceCm > config::kDriveDistanceMaximumTargetCm)
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeDistanceStatus(
                "distance_invalid_target",
                "Distância solicitada fora da faixa segura",
                targetDistanceCm, 0.0, 0.0, 0.0));
            return;
        }

        if (!encoderReady)
        {
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeDistanceStatus(
                "waiting_encoders",
                "Aguardando telemetria recente dos encoders",
                targetDistanceCm, 0.0, 0.0, 0.0));
            return;
        }

        distancePhase_ = DistancePhase::Driving;
        distanceStartLeftCount_ = esp32Telemetry.leftEncoderCount;
        distanceStartRightCount_ = esp32Telemetry.rightEncoderCount;
        activeDistanceTargetCm_ = targetDistanceCm;
        distanceStartedAt_ = now;
        distancePhaseStartedAt_ = now;
        distanceLastProgressAt_ = now;
        lastDistanceProgressCounts_ = 0.0;
        distanceCorrectionPulseCount_ = 0;
        distanceDifferenceSamples_ = 0;
        distanceLastDifferenceUptimeMs_ = 0;
    }

    const double leftCounts = std::abs(static_cast<double>(
        esp32Telemetry.leftEncoderCount - distanceStartLeftCount_));
    const double rightCounts = std::abs(static_cast<double>(
        esp32Telemetry.rightEncoderCount - distanceStartRightCount_));
    const double leftDistanceCm =
        leftCounts / config::kEncoderCountsPerCentimeter;
    const double rightDistanceCm =
        rightCounts / config::kEncoderCountsPerCentimeter;
    const double minimumDistanceCm = std::min(leftDistanceCm, rightDistanceCm);
    const double minimumCounts = std::min(leftCounts, rightCounts);
    const double targetCounts =
        activeDistanceTargetCm_ * config::kEncoderCountsPerCentimeter;
    const double progressPercent = std::clamp(
        minimumDistanceCm / activeDistanceTargetCm_ * 100.0, 0.0, 100.0);
    const DriveDistanceCommand driveCommand = calculateDriveDistanceCommand(
        leftDistanceCm, rightDistanceCm);

    if (now - distanceStartedAt_ >
        std::chrono::milliseconds(config::kDriveDistanceTimeoutMs))
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeDistanceStatus(
            "distance_timeout", "Percurso interrompido pelo tempo limite",
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm,
            progressPercent));
        distancePhase_ = DistancePhase::Idle;
        return;
    }

    if (!encoderReady)
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeDistanceStatus(
            "distance_encoder_lost",
            "Percurso interrompido: encoders sem dados recentes",
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm,
            progressPercent));
        distancePhase_ = DistancePhase::Idle;
        return;
    }

    const bool movementPhase = distancePhase_ == DistancePhase::Driving ||
                               distancePhase_ == DistancePhase::CorrectionPulse;
    const bool newEncoderSample =
        esp32Telemetry.esp32UptimeMs != distanceLastDifferenceUptimeMs_;
    if (movementPhase && newEncoderSample)
    {
        distanceLastDifferenceUptimeMs_ = esp32Telemetry.esp32UptimeMs;
        const double sideDifferenceCm =
            std::abs(leftDistanceCm - rightDistanceCm);
        if (sideDifferenceCm > config::kDriveDistanceMaximumSideDifferenceCm)
        {
            ++distanceDifferenceSamples_;
        }
        else
        {
            distanceDifferenceSamples_ = 0;
        }
    }
    if (movementPhase && distanceDifferenceSamples_ >=
                             config::kDriveDistanceDifferenceConfirmationSamples)
    {
        // Não tenta corrigir uma diferença grande como se fosse um ajuste fino.
        // Parar cedo evita que um motor fraco, roda travada ou encoder incoerente
        // transforme o teste reto em giro no próprio eixo.
        robotState.stop();
        robotState.updateAutonomousStatus(makeDistanceStatus(
            "distance_encoder_mismatch",
            "Percurso interrompido: diferença excessiva entre os lados",
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm,
            progressPercent));
        distancePhase_ = DistancePhase::Idle;
        return;
    }
    if (movementPhase &&
        minimumCounts >= lastDistanceProgressCounts_ +
                             config::kDriveDistanceMinimumProgressCounts)
    {
        lastDistanceProgressCounts_ = minimumCounts;
        distanceLastProgressAt_ = now;
    }
    if (movementPhase &&
        now - distanceLastProgressAt_ >
            std::chrono::milliseconds(config::kDriveDistanceStallTimeoutMs))
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeDistanceStatus(
            "distance_encoder_stall",
            "Percurso interrompido: um lado não avançou",
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm,
            progressPercent));
        distancePhase_ = DistancePhase::Idle;
        return;
    }

    if (distancePhase_ == DistancePhase::Driving)
    {
        const double predictionSeconds =
            config::kDriveDistanceBrakePredictionSeconds +
            esp32Telemetry.lastSensorAgeMs / 1000.0;
        const double projectedLeftCounts =
            leftCounts + std::abs(esp32Telemetry.leftEncoderRate) * predictionSeconds;
        const double projectedRightCounts =
            rightCounts + std::abs(esp32Telemetry.rightEncoderRate) * predictionSeconds;

        if (std::min(projectedLeftCounts, projectedRightCounts) >= targetCounts)
        {
            robotState.driveAutonomous(0.0, 0.0);
            distancePhase_ = DistancePhase::Settling;
            distancePhaseStartedAt_ = now;
            robotState.updateAutonomousStatus(makeDistanceStatus(
                "distance_settling",
                "PWM zerado: aguardando o percurso estabilizar",
                activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm,
                progressPercent));
            return;
        }

        robotState.driveAutonomous(
            driveCommand.left,
            driveCommand.right);
        robotState.updateAutonomousStatus(makeDistanceStatus(
            "driving_distance", "Avançando até a distância selecionada",
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm,
            progressPercent));
        return;
    }

    if (distancePhase_ == DistancePhase::CorrectionPulse)
    {
        if (minimumDistanceCm >=
            activeDistanceTargetCm_ - config::kDriveDistanceToleranceCm)
        {
            robotState.driveAutonomous(0.0, 0.0);
            distancePhase_ = DistancePhase::Settling;
            distancePhaseStartedAt_ = now;
            return;
        }

        if (now - distancePhaseStartedAt_ <
            std::chrono::milliseconds(config::kDriveDistanceCorrectionPulseMs))
        {
            robotState.driveAutonomous(
                driveCommand.left,
                driveCommand.right);
            robotState.updateAutonomousStatus(makeDistanceStatus(
                "distance_correction",
                "Aplicando correção curta de distância",
                activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm,
                progressPercent));
            return;
        }

        robotState.driveAutonomous(0.0, 0.0);
        distancePhase_ = DistancePhase::Settling;
        distancePhaseStartedAt_ = now;
    }

    robotState.driveAutonomous(0.0, 0.0);
    if (now - distancePhaseStartedAt_ <
        std::chrono::milliseconds(config::kDriveDistanceSettleMs))
    {
        robotState.updateAutonomousStatus(makeDistanceStatus(
            "distance_settling", "Aguardando os encoders estabilizarem",
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm,
            progressPercent));
        return;
    }

    if (minimumDistanceCm >=
        activeDistanceTargetCm_ - config::kDriveDistanceToleranceCm)
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeDistanceStatus(
            "distance_completed", "Distância concluída e registrada",
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm, 100.0));
        distancePhase_ = DistancePhase::Idle;
        return;
    }

    if (distanceCorrectionPulseCount_ >=
        config::kDriveDistanceMaximumCorrectionPulses)
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeDistanceStatus(
            "distance_correction_failed",
            "Percurso parado: correções insuficientes",
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm,
            progressPercent));
        distancePhase_ = DistancePhase::Idle;
        return;
    }

    ++distanceCorrectionPulseCount_;
    distancePhase_ = DistancePhase::CorrectionPulse;
    distancePhaseStartedAt_ = now;
    distanceLastProgressAt_ = now;
    lastDistanceProgressCounts_ = minimumCounts;
    robotState.driveAutonomous(
        driveCommand.left,
        driveCommand.right);
    robotState.updateAutonomousStatus(makeDistanceStatus(
        "distance_correction", "Completando os centímetros restantes",
        activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm,
        progressPercent));
}

void MissionController::resetMissionState()
{
    mainMission_.reset();
    rescueAreaMission_.reset();
    rescueZoneAlignMission_.reset();
    rescueZoneApproachMission_.reset();
    rescueZoneSearchMission_.reset();
    rescueZoneTriangleMission_.reset();
    obstacleAvoidanceTest_.reset();
    testTurnController_.reset();
    cornerYawInitialReverse_.reset();
    cornerYawPhase_ = CornerYawPhase::Ready;
    cornerYawReferenceDegrees_ = 0.0;
    cornerYawReferenceValid_ = false;
    cornerYawInitialReverseCompleted_ = false;
    cornerYawIndex_ = 0;
    servoRoutine_.resetExecution();
    distancePhase_ = DistancePhase::Idle;
    activeDistanceTargetCm_ = 0.0;
    lastDistanceProgressCounts_ = 0.0;
    distanceCorrectionPulseCount_ = 0;
    distanceDifferenceSamples_ = 0;
    distanceLastDifferenceUptimeMs_ = 0;
}
