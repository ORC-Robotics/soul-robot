#include "obr/line_course_mission.h"

#include "obr/config.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

namespace
{
AutonomousStatus makeMainMissionStatus(
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

AutonomousStatus makeObstacleStatus(
    const ObstacleAvoidanceOutput& output)
{
    AutonomousStatus status = makeMainMissionStatus(
        output.phase, output.action, output.progressPercent);
    status.targetDistanceCm = output.targetDistanceCm;
    status.leftDistanceCm = output.leftDistanceCm;
    status.rightDistanceCm = output.rightDistanceCm;
    status.averageDistanceCm =
        (output.leftDistanceCm + output.rightDistanceCm) * 0.5;
    status.obstacleYawBase = output.yawBase;
    status.obstacleLeftClearance = output.leftClearance;
    status.obstacleRightClearance = output.rightClearance;
    status.obstacleSelectedSide = output.selectedSide;
    return status;
}
}

void LineCourseMission::reset()
{
    obstacleAvoidance_.reset();
    greenTurnAroundManeuver_.reset();
    forwardLineAssist_.reset();
}

void LineCourseMission::update(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    bool cameraReady,
    const CameraLineSnapshot& cameraLineSnapshot,
    const ForwardLineSnapshot& forwardLineSnapshot)
{
    const RobotSnapshot robotSnapshot = robotState.snapshot();
    if (robotSnapshot.mode != "autonomous")
    {
        return;
    }

    // E-Stop, calibração e perda da ESP32 continuam acima de qualquer manobra.
    // Sem essa telemetria não existe uma forma segura de manter os motores ativos.
    if (!esp32Telemetry.readyForOperation())
    {
        reset();
        robotState.stop();
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            "esp32_not_ready",
            "Missão interrompida: ESP32 sem telemetria pronta"));
        std::cout << "MainMission stopped: esp32_not_ready" << std::endl;
        return;
    }

    // O desvio só pode iniciar fora do retorno verde. Depois de iniciado, ele
    // mantém autoridade até escolher o lado e parar no yaw correspondente.
    const ObstacleAvoidanceOutput obstacleOutput = obstacleAvoidance_.update(
        esp32Telemetry,
        cameraLineSnapshot,
        !greenTurnAroundManeuver_.active() && cameraReady &&
            cameraLineSnapshot.sourceFresh);
    if (obstacleOutput.failed)
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeObstacleStatus(obstacleOutput));
        std::cout << "Obstacle avoidance stopped: "
                  << obstacleOutput.phase << std::endl;
        return;
    }
    if (obstacleOutput.hasControl)
    {
        // O desvio substitui temporariamente tanto a câmera inferior quanto o
        // Forward Assist; nenhum dos dois atualiza os motores durante a manobra.
        forwardLineAssist_.reset();
        robotState.driveAutonomous(
            obstacleOutput.leftPower,
            obstacleOutput.rightPower);
        robotState.updateAutonomousStatus(makeObstacleStatus(obstacleOutput));
        return;
    }

    // A câmera inferior participa da centralização inicial e volta a ser
    // obrigatória para o segue-linha quando o módulo devolve o controle.
    if (!cameraReady || !cameraLineSnapshot.sourceFresh)
    {
        const std::string phase =
            !cameraReady ? "camera_not_ready" : "line_ipc_stale";
        const std::string action = !cameraReady
                                       ? "Missão interrompida: câmera inferior indisponível"
                                       : "Missão interrompida: IPC visual ausente ou antigo";
        reset();
        robotState.stop();
        robotState.updateAutonomousStatus(
            makeMainMissionStatus(phase, action));
        std::cout << "MainMission stopped: " << phase << std::endl;
        return;
    }

    if (greenTurnAroundManeuver_.shouldBlockForwardAssist(cameraLineSnapshot))
    {
        // O retorno verde nunca compartilha seu estado com a assistência frontal.
        forwardLineAssist_.reset();
    }
    if (greenTurnAroundManeuver_.update(
            robotState,
            esp32Telemetry,
            cameraLineSnapshot))
    {
        return;
    }

    if (forwardLineAssist_.update(
            robotState,
            esp32Telemetry,
            cameraLineSnapshot,
            forwardLineSnapshot))
    {
        return;
    }

    double leftPower = cameraLineSnapshot.lineFollowerLeftPower;
    double rightPower = cameraLineSnapshot.lineFollowerRightPower;
    const bool normalLineFollowing =
        cameraLineSnapshot.curveDiagnostics.lineState == "LINE" &&
        cameraLineSnapshot.curveDiagnostics.virtualState == "NORMAL";
    const bool rampTelemetryReady =
        esp32Telemetry.sensorFresh && esp32Telemetry.mpuOk &&
        esp32Telemetry.lastSensorAgeMs >= 0 &&
        esp32Telemetry.lastSensorAgeMs <= config::kTurn90ImuFreshnessMs &&
        std::isfinite(esp32Telemetry.rampAngleDeg);
    if (normalLineFollowing && rampTelemetryReady &&
        leftPower > 0.0 && rightPower > 0.0)
    {
        double rampPowerOffset = 0.0;
        double rampMaximumPower = config::kMaxMotorOutput;
        if (esp32Telemetry.rampAngleDeg >=
            config::kLineFollowingSteepUphillThresholdDeg)
        {
            rampPowerOffset =
                config::kLineFollowingSteepUphillPowerOffset;
            rampMaximumPower =
                config::kLineFollowingSteepUphillMaximumPower;
        }
        else if (esp32Telemetry.rampAngleDeg >=
            config::kLineFollowingUphillThresholdDeg)
        {
            rampPowerOffset = config::kLineFollowingUphillPowerOffset;
            rampMaximumPower = config::kLineFollowingUphillMaximumPower;
        }
        else if (esp32Telemetry.rampAngleDeg <=
                 config::kLineFollowingDownhillThresholdDeg)
        {
            rampPowerOffset = config::kLineFollowingDownhillPowerOffset;
        }

        // O mesmo offset preserva o diferencial do Fusion até que a roda
        // externa alcance o teto seguro definido para cada faixa da rampa.
        leftPower = std::clamp(
            leftPower + rampPowerOffset,
            config::kMinMotorOutput,
            rampMaximumPower);
        rightPower = std::clamp(
            rightPower + rampPowerOffset,
            config::kMinMotorOutput,
            rampMaximumPower);
    }
    robotState.driveAutonomous(
        leftPower,
        rightPower,
        !(
            normalLineFollowing &&
            cameraLineSnapshot.normalSteeringValid));
    robotState.updateAutonomousStatus(forwardLineAssist_.status(
        "line_following",
        "Seguindo a linha pela câmera inferior",
        forwardLineSnapshot));
}
