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
    status.obstacleWaitSecondsRemaining = output.waitSecondsRemaining;
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
    obstacleRecoveryWaiting_ = false;
    obstacleRecoveryFusionFrames_ = 0;
    obstacleRecoveryLastLineSequence_ = 0;
    obstacleRecoveryCause_.clear();
    cameraRecoveryEligible_ = false;
    cameraRecoveryWaiting_ = false;
    cameraRecoveryFrames_ = 0;
    cameraRecoveryLastLineSequence_ = 0;
    cameraRecoveryRunSequence_ = 0;
    cameraRecoveryStartedAt_ = {};
}

bool LineCourseMission::updateCameraRecovery(
    RobotState& robotState,
    const RobotSnapshot& robotSnapshot,
    bool cameraReady,
    const CameraLineSnapshot& cameraLineSnapshot)
{
    if (cameraRecoveryRunSequence_ != robotSnapshot.autonomousRunSequence)
    {
        // Uma partida nova não herda a espera nem os votos da execução anterior.
        cameraRecoveryWaiting_ = false;
        cameraRecoveryEligible_ = false;
        cameraRecoveryFrames_ = 0;
        cameraRecoveryRunSequence_ = robotSnapshot.autonomousRunSequence;
    }

    if (!cameraRecoveryWaiting_)
    {
        const std::string& phase = robotSnapshot.autonomousStatus.phase;
        // O atraso breve do IPC pode preceder a perda do status geral. Nesse
        // intervalo, preserva somente a elegibilidade já adquirida em movimento.
        const bool eligiblePhase =
            phase == "line_following" || phase == "green_confirming";
        if (!eligiblePhase && phase != "line_ipc_waiting")
        {
            cameraRecoveryEligible_ = false;
        }
        else if (eligiblePhase && cameraReady && cameraLineSnapshot.sourceFresh)
        {
            cameraRecoveryEligible_ = cameraLineSnapshot.lineSequence > 0 &&
                cameraLineSnapshot.normalSteeringValid &&
                cameraLineSnapshot.lineControlSource == "fusion" &&
                cameraLineSnapshot.curveDiagnostics.lineState == "LINE" &&
                cameraLineSnapshot.curveDiagnostics.virtualState == "NORMAL";
        }
        if (cameraReady || cameraLineSnapshot.sourceFresh ||
            !cameraLineSnapshot.activeStreamDelayed || !cameraRecoveryEligible_ ||
            obstacleAvoidance_.active() ||
            obstacleRecoveryWaiting_ || forwardLineAssist_.active())
        {
            return false;
        }

        cameraRecoveryWaiting_ = true;
        cameraRecoveryFrames_ = 0;
        cameraRecoveryLastLineSequence_ = cameraLineSnapshot.lineSequence;
        cameraRecoveryRunSequence_ = robotSnapshot.autonomousRunSequence;
        cameraRecoveryStartedAt_ = std::chrono::steady_clock::now();
        robotState.driveAutonomous(0.0, 0.0);
        // A confirmação verde anterior perdeu validade. A cena será reavaliada
        // após a recuperação, sem continuar uma manobra com comandos antigos.
        greenManeuver_.reset();
        forwardLineAssist_.reset();
        std::cout << "Bottom camera recovery waiting: motors stopped" << std::endl;
    }

    if (!cameraReady && !cameraLineSnapshot.activeStreamDelayed)
    {
        // Desligamento, falha declarada ou IPC inválido mantêm a parada original.
        // Não aguardamos a câmera voltar quando o motivo já não é um atraso.
        cameraRecoveryWaiting_ = false;
        cameraRecoveryEligible_ = false;
        cameraRecoveryFrames_ = 0;
        return false;
    }

    robotState.driveAutonomous(0.0, 0.0);
    const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - cameraRecoveryStartedAt_).count();
    if (elapsedMs >= config::kBottomCameraRecoveryTimeoutMs)
    {
        reset();
        robotState.stop();
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            "camera_not_ready",
            "Missão interrompida: câmera inferior não recuperou dentro do prazo"));
        std::cout << "MainMission stopped: bottom_camera_recovery_timeout" << std::endl;
        return true;
    }

    const bool reliableLine = cameraReady && cameraLineSnapshot.sourceFresh &&
        cameraLineSnapshot.lineControlSource == "fusion" &&
        cameraLineSnapshot.normalSteeringValid &&
        cameraLineSnapshot.curveDiagnostics.lineState == "LINE" &&
        cameraLineSnapshot.curveDiagnostics.virtualState == "NORMAL" &&
        std::isfinite(cameraLineSnapshot.lineFollowerLeftPower) &&
        std::isfinite(cameraLineSnapshot.lineFollowerRightPower) &&
        std::abs(cameraLineSnapshot.lineFollowerLeftPower) <= 1.0 &&
        std::abs(cameraLineSnapshot.lineFollowerRightPower) <= 1.0;
    if (!reliableLine)
    {
        cameraRecoveryFrames_ = 0;
        // Uma leitura rejeitada não pode ser reapresentada como um voto novo.
        cameraRecoveryLastLineSequence_ = std::max(
            cameraRecoveryLastLineSequence_, cameraLineSnapshot.lineSequence);
    }
    else if (cameraLineSnapshot.lineSequence > cameraRecoveryLastLineSequence_)
    {
        cameraRecoveryLastLineSequence_ = cameraLineSnapshot.lineSequence;
        ++cameraRecoveryFrames_;
    }

    if (cameraRecoveryFrames_ >= config::kBottomCameraRecoveryRequiredFrames)
    {
        cameraRecoveryWaiting_ = false;
        cameraRecoveryEligible_ = false;
        cameraRecoveryFrames_ = 0;
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            "camera_recovery_ready",
            "Câmera inferior recuperada: reavaliando a faixa antes de retomar"));
        std::cout << "Bottom camera recovery ready: new Fusion frames confirmed" << std::endl;
    }
    else
    {
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            "camera_recovery_waiting",
            "Pausado: aguardando a recuperação da câmera inferior"));
    }
    return true;
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
        cameraRecoveryWaiting_ = false;
        cameraRecoveryEligible_ = false;
        cameraRecoveryFrames_ = 0;
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

    // Esta recuperação só trata a perda temporária da CAM0 nas fases permitidas.
    // ESP32, E-Stop e calibração continuam sendo validados antes da espera.
    if (updateCameraRecovery(
            robotState, robotSnapshot, cameraReady, cameraLineSnapshot))
    {
        return;
    }

    // Qualquer candidato verde novo assume prioridade antes do obstáculo.
    // Uma fonte ausente ou antiga nunca pode iniciar nem sustentar movimento.
    if (greenManeuver_.active() &&
        (!cameraReady || !cameraLineSnapshot.sourceFresh))
    {
        if (cameraReady && greenManeuver_.waitingForStallRecovery())
        {
            // Uma falha transitória do IPC não pode cancelar a espera pela faixa.
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "green_stall_waiting_line",
                "Manobra verde pausada: aguardando faixa Fusion confiável"));
            return;
        }
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

    if (obstacleRecoveryWaiting_)
    {
        // Após uma falha do desvio, não volta a acelerar perto do obstáculo.
        // A missão continua ativa e retoma ao confirmar a faixa e frente livre.
        const bool frontClear = esp32Telemetry.lastSensorAgeMs >= 0 &&
            esp32Telemetry.lastSensorAgeMs <= config::kObstacleUltrasonicFreshnessMs &&
            std::isfinite(esp32Telemetry.ultrasonicDistanceCm) &&
            esp32Telemetry.ultrasonicDistanceCm >= config::kObstacleRearmDistanceCm &&
            esp32Telemetry.ultrasonicDistanceCm <= 400.0;
        const bool lineConfirmed = cameraReady && cameraLineSnapshot.sourceFresh &&
            cameraLineSnapshot.lineControlSource == "fusion" &&
            cameraLineSnapshot.normalSteeringValid &&
            cameraLineSnapshot.curveDiagnostics.lineState == "LINE" &&
            cameraLineSnapshot.curveDiagnostics.virtualState == "NORMAL";
        const bool imuReady = ImuTurnController::imuReady(esp32Telemetry);
        if (frontClear && lineConfirmed && imuReady &&
            cameraLineSnapshot.lineSequence > obstacleRecoveryLastLineSequence_)
        {
            ++obstacleRecoveryFusionFrames_;
            obstacleRecoveryLastLineSequence_ = cameraLineSnapshot.lineSequence;
        }
        else if (!frontClear || !lineConfirmed || !imuReady)
        {
            obstacleRecoveryFusionFrames_ = 0;
        }
        robotState.driveAutonomous(0.0, 0.0);
        if (obstacleRecoveryFusionFrames_ >=
            config::kObstacleRecoveryRequiredFusionFrames)
        {
            obstacleRecoveryWaiting_ = false;
            obstacleRecoveryFusionFrames_ = 0;
            obstacleAvoidance_.reset();
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "obstacle_recovery_ready",
                "Desvio abortado: faixa e frente livre confirmadas; retomando seguidor"));
        }
        else
        {
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "obstacle_recovery_waiting",
                "Desvio pausado (" + obstacleRecoveryCause_ +
                    "): aguardando faixa Fusion e frente livre"));
        }
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
        obstacleRecoveryWaiting_ = true;
        obstacleRecoveryFusionFrames_ = 0;
        obstacleRecoveryLastLineSequence_ = cameraLineSnapshot.lineSequence;
        obstacleRecoveryCause_ = obstacleOutput.phase;
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeObstacleStatus(obstacleOutput));
        std::cout << "Obstacle avoidance paused for recovery: "
                  << obstacleOutput.phase << std::endl;
        return;
    }
    if (config::kWaveBonusAfterObstacleEnabled && obstacleOutput.completed &&
        obstacleOutput.phase == "obstacle_exit_reacquired")
    {
        // A saída confirmada ocorre uma vez por desvio. Pausa a tração antes
        // de voltar ao seguidor, sem apagar a fase da missão principal.
        forwardLineAssist_.reset();
        robotState.requestWaveBonus();
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
