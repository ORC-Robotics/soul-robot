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
    return status;
}
}

void LineCourseMission::reset()
{
    obstacleAvoidance_.reset();
    greenManeuver_.reset();
    forwardLineAssist_.reset();
}

void LineCourseMission::update(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    bool cameraReady,
    const CameraLineSnapshot& cameraLineSnapshot,
    const ForwardLineSnapshot& forwardLineSnapshot,
    bool allowForwardLostRecovery)
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

    // Qualquer candidato verde novo assume prioridade antes do obstáculo.
    // Uma fonte ausente ou antiga nunca pode iniciar nem sustentar movimento.
    if (greenManeuver_.active() &&
        (!cameraReady || !cameraLineSnapshot.sourceFresh))
    {
        greenManeuver_.reset();
        if (cameraReady)
        {
            robotState.driveAutonomous(0.0, 0.0);
        }
        else
        {
            reset();
            robotState.stop();
        }
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            cameraReady ? "line_ipc_waiting" : "camera_not_ready",
            cameraReady
                ? "Manobra verde pausada: aguardando uma leitura visual nova"
                : "Missão interrompida: câmera inferior indisponível"));
        return;
    }
    if (cameraReady && cameraLineSnapshot.sourceFresh &&
        greenManeuver_.shouldBlockForwardAssist(cameraLineSnapshot))
    {
        forwardLineAssist_.reset();
    }
    if (cameraReady && cameraLineSnapshot.sourceFresh &&
        greenManeuver_.update(
            robotState,
            esp32Telemetry,
            cameraLineSnapshot))
    {
        // Um verde novo invalida qualquer desvio parcialmente iniciado. Depois
        // da curva, o obstáculo deve ser detectado novamente com dados atuais.
        obstacleAvoidance_.reset();
        return;
    }

    // O desvio só inicia fora de uma sequência verde e mantém autoridade
    // durante o contorno, a saída e as recuperações que ainda estejam ativas.
    const ObstacleAvoidanceOutput obstacleOutput = obstacleAvoidance_.update(
        esp32Telemetry,
        cameraLineSnapshot,
        !greenManeuver_.active() && cameraReady &&
            cameraLineSnapshot.sourceFresh,
        forwardLineSnapshot);
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
    if (!cameraReady)
    {
        reset();
        robotState.stop();
        robotState.updateAutonomousStatus(
            makeMainMissionStatus(
                "camera_not_ready",
                "Missão interrompida: câmera inferior indisponível"));
        std::cout << "MainMission stopped: camera_not_ready" << std::endl;
        return;
    }

    if (!cameraLineSnapshot.sourceFresh)
    {
        // Uma escrita atrasada do IPC não deve apagar toda a missão. Os motores
        // permanecem zerados até uma amostra nova devolver o controle à câmera.
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(
            makeMainMissionStatus(
                "line_ipc_waiting",
                "Pausado: aguardando uma leitura visual nova"));
        return;
    }

    if (forwardLineAssist_.update(
            robotState,
            esp32Telemetry,
            cameraLineSnapshot,
            forwardLineSnapshot,
            allowForwardLostRecovery))
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
    AutonomousStatus status = forwardLineAssist_.status(
        "line_following",
        "Seguindo a linha pela câmera inferior",
        forwardLineSnapshot);
    status.rawBestParabolaSide = obstacleOutput.rawBestParabolaSide;
    status.bestParabolaSide = obstacleOutput.bestParabolaSide;
    status.bestParabolaScore = obstacleOutput.bestParabolaScore;
    status.bestParabolaLeftBlack = obstacleOutput.bestParabolaLeftBlack;
    status.bestParabolaRightBlack = obstacleOutput.bestParabolaRightBlack;
    status.bestParabolaSequence = obstacleOutput.bestParabolaSequence;
    status.bestParabolaSideValid = obstacleOutput.bestParabolaSideValid;
    status.nearForwardLineVisible = obstacleOutput.nearForwardLineVisible;
    status.nearForwardLineVotes = obstacleOutput.nearForwardLineVotes;
    status.nearForwardLineSamples = obstacleOutput.nearForwardLineSamples;
    status.case3Armed = obstacleOutput.case3Armed;
    status.case3FusionAcquireTime = obstacleOutput.case3FusionAcquireTime;
    status.case3TimeRemainingMs = obstacleOutput.case3TimeRemainingMs;
    robotState.updateAutonomousStatus(status);
}
