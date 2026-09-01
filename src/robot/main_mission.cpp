#include "obr/main_mission.h"

#include "obr/config.h"

#include <algorithm>
#include <chrono>
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

AutonomousStatus makeTurnAroundForwardStatus(
    const std::string& phase,
    const std::string& action,
    double leftDistanceCm,
    double rightDistanceCm,
    double progressPercent)
{
    AutonomousStatus status = makeMainMissionStatus(
        phase, action, progressPercent);
    status.targetDistanceCm = config::kGreenTurnAroundForwardDistanceCm;
    status.leftDistanceCm = leftDistanceCm;
    status.rightDistanceCm = rightDistanceCm;
    status.averageDistanceCm = (leftDistanceCm + rightDistanceCm) * 0.5;
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
    return status;
}

bool turnAroundDetected(const CameraLineSnapshot& snapshot)
{
    return snapshot.greenConfirmed && snapshot.greenPathBlackValid &&
           snapshot.greenCandidateCount == 2 &&
           snapshot.greenInterpretation == GreenInterpretation::TurnAround180;
}

bool turnAroundEncodersReady(const Esp32TelemetrySnapshot& telemetry)
{
    return telemetry.sensorFresh && telemetry.lastSensorAgeMs >= 0 &&
           telemetry.lastSensorAgeMs <=
               config::kGreenTurnAroundEncoderDataTimeoutMs &&
           std::isfinite(telemetry.leftEncoderRate) &&
           std::isfinite(telemetry.rightEncoderRate);
}

ImuTurnDirection configuredTurnDirection()
{
    return config::kGreenTurnAroundTurnsRight
               ? ImuTurnDirection::Right
               : ImuTurnDirection::Left;
}

double configuredTurnSign()
{
    return config::kGreenTurnAroundTurnsRight ? 1.0 : -1.0;
}

double shortestAngularDistanceDegrees(double first, double second)
{
    double difference = std::fmod(std::abs(second - first), 360.0);
    if (difference > 180.0)
    {
        difference = 360.0 - difference;
    }
    return difference;
}
}

void MainMission::reset()
{
    turnAroundPhase_ = TurnAroundPhase::Idle;
    obstacleAvoidance_.reset();
    turnAroundController_.reset();
    turnAroundArmed_ = true;
    forwardStartLeftCount_ = 0;
    forwardStartRightCount_ = 0;
    lineReacquireFrames_ = 0;
    lineSearchSawNearLine_ = false;
    lineSearchStartYawDegrees_ = 0.0;
}

void MainMission::update(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    bool cameraReady,
    const CameraLineSnapshot& cameraLineSnapshot)
{
    const RobotSnapshot robotSnapshot = robotState.snapshot();
    if (robotSnapshot.mode != "autonomous" ||
        robotSnapshot.autonomousMission != AutonomousMission::MainMission)
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

    const auto now = std::chrono::steady_clock::now();

    // O desvio só pode iniciar fora do retorno verde. Depois de iniciado, ele
    // mantém autoridade até terminar a ré final ou falhar com os motores zerados.
    const ObstacleAvoidanceOutput obstacleOutput = obstacleAvoidance_.update(
        esp32Telemetry,
        turnAroundPhase_ == TurnAroundPhase::Idle);
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
        robotState.driveAutonomous(
            obstacleOutput.leftPower,
            obstacleOutput.rightPower);
        robotState.updateAutonomousStatus(makeObstacleStatus(obstacleOutput));
        return;
    }

    // A câmera não participa dos giros nem dos deslocamentos do desvio. Sua
    // indisponibilidade só é avaliada quando o módulo devolve a autoridade.
    // Assim, linha, verde ou IPC visual antigo não interrompem uma sequência
    // de obstáculo que já foi confirmada pelo ultrassônico.
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

    if (obstacleOutput.completed)
    {
        robotState.driveAutonomous(
            cameraLineSnapshot.lineFollowerLeftPower,
            cameraLineSnapshot.lineFollowerRightPower);
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            "line_following",
            "Desvio concluído: seguindo novamente a linha",
            100.0));
        return;
    }

    const bool detected180 = turnAroundDetected(cameraLineSnapshot);
    if (turnAroundPhase_ == TurnAroundPhase::Idle && !detected180)
    {
        // Um retorno concluído só pode disparar novamente depois que os dois
        // verdes realmente saírem da percepção confirmada.
        turnAroundArmed_ = true;
    }

    if (turnAroundPhase_ == TurnAroundPhase::Idle && detected180 &&
        turnAroundArmed_)
    {
        if (!turnAroundEncodersReady(esp32Telemetry) ||
            !ImuTurnController::imuReady(esp32Telemetry))
        {
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_waiting_sensors",
                "Retorno detectado: aguardando encoders e MPU6050"));
            return;
        }

        turnAroundArmed_ = false;
        // A parada tem duração configurada. Os encoders só são referenciados
        // depois da estabilização, imediatamente antes do avanço.
        turnAroundPhase_ = TurnAroundPhase::RecognizedStopping;
        phaseStartedAt_ = now;
        lineReacquireFrames_ = 0;
        lineSearchSawNearLine_ = false;
        lineSearchStartYawDegrees_ = 0.0;
    }

    if (turnAroundPhase_ == TurnAroundPhase::RecognizedStopping)
    {
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            "turnaround_recognized_stopping",
            "Retorno 180° reconhecido: parando antes de avançar"));

        if (now - phaseStartedAt_ < std::chrono::milliseconds(
                config::kGreenTurnAroundRecognitionStopMs))
        {
            return;
        }

        if (!turnAroundEncodersReady(esp32Telemetry) ||
            !ImuTurnController::imuReady(esp32Telemetry))
        {
            return;
        }

        forwardStartLeftCount_ = esp32Telemetry.leftEncoderCount;
        forwardStartRightCount_ = esp32Telemetry.rightEncoderCount;
        turnAroundPhase_ = TurnAroundPhase::DrivingForward;
        phaseStartedAt_ = now;
    }

    if (turnAroundPhase_ == TurnAroundPhase::Idle)
    {
        robotState.driveAutonomous(
            cameraLineSnapshot.lineFollowerLeftPower,
            cameraLineSnapshot.lineFollowerRightPower);
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            "line_following", "Seguindo a linha pela câmera inferior"));
        return;
    }

    if (turnAroundPhase_ == TurnAroundPhase::DrivingForward)
    {
        const double leftCounts = std::abs(static_cast<double>(
            esp32Telemetry.leftEncoderCount - forwardStartLeftCount_));
        const double rightCounts = std::abs(static_cast<double>(
            esp32Telemetry.rightEncoderCount - forwardStartRightCount_));
        const double leftDistanceCm =
            leftCounts / config::kEncoderCountsPerCentimeter;
        const double rightDistanceCm =
            rightCounts / config::kEncoderCountsPerCentimeter;
        const double minimumDistanceCm =
            std::min(leftDistanceCm, rightDistanceCm);
        const double progressPercent = std::clamp(
            minimumDistanceCm /
                config::kGreenTurnAroundForwardDistanceCm * 100.0,
            0.0,
            100.0);

        if (!turnAroundEncodersReady(esp32Telemetry))
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeTurnAroundForwardStatus(
                "turnaround_encoder_lost",
                "Retorno interrompido: encoders sem dados recentes",
                leftDistanceCm,
                rightDistanceCm,
                progressPercent));
            return;
        }
        if (now - phaseStartedAt_ > std::chrono::milliseconds(
                config::kGreenTurnAroundForwardSafetyTimeoutMs))
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeTurnAroundForwardStatus(
                "turnaround_forward_timeout",
                "Retorno interrompido pelo limite absoluto de segurança",
                leftDistanceCm,
                rightDistanceCm,
                progressPercent));
            return;
        }

        const double predictionSeconds =
            config::kDriveDistanceBrakePredictionSeconds +
            esp32Telemetry.lastSensorAgeMs / 1000.0;
        const double projectedLeftCounts =
            leftCounts + std::abs(esp32Telemetry.leftEncoderRate) *
                             predictionSeconds;
        const double projectedRightCounts =
            rightCounts + std::abs(esp32Telemetry.rightEncoderRate) *
                              predictionSeconds;
        const double targetCounts =
            config::kGreenTurnAroundForwardDistanceCm *
            config::kEncoderCountsPerCentimeter;
        if (std::min(projectedLeftCounts, projectedRightCounts) >= targetCounts)
        {
            turnAroundPhase_ = TurnAroundPhase::ForwardSettling;
            phaseStartedAt_ = now;
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeTurnAroundForwardStatus(
                "turnaround_forward_settling",
                "Avanço concluído: aguardando o robô estabilizar",
                leftDistanceCm,
                rightDistanceCm,
                progressPercent));
            return;
        }

        robotState.driveAutonomous(
            config::kGreenTurnAroundForwardPower,
            config::kGreenTurnAroundForwardPower);
        robotState.updateAutonomousStatus(makeTurnAroundForwardStatus(
            "turnaround_forward",
            "Retorno 180°: avançando a distância configurada pelos encoders",
            leftDistanceCm,
            rightDistanceCm,
            progressPercent));
        return;
    }

    if (turnAroundPhase_ == TurnAroundPhase::ForwardSettling)
    {
        robotState.driveAutonomous(0.0, 0.0);
        if (now - phaseStartedAt_ < std::chrono::milliseconds(
                config::kGreenTurnAroundForwardSettleMs))
        {
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_forward_settling",
                "Aguardando a parada antes do giro por IMU"));
            return;
        }
        if (!turnAroundController_.start(
                config::kGreenTurnAroundImuDegrees,
                configuredTurnDirection(),
                esp32Telemetry,
                config::kGreenTurnAroundImuToleranceDegrees))
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turn_imu_lost",
                "Retorno interrompido: MPU6050 sem referência válida"));
            return;
        }
        turnAroundPhase_ = TurnAroundPhase::TurningByImu;
    }

    if (turnAroundPhase_ == TurnAroundPhase::TurningByImu)
    {
        const ImuTurnOutput output =
            turnAroundController_.update(esp32Telemetry);
        if (output.result == ImuTurnResult::Failed)
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                output.phase, output.action, output.progressPercent));
            return;
        }
        if (output.result == ImuTurnResult::Completed)
        {
            // A busca visual começa somente após a IMU concluir e estabilizar
            // o giro inicial. Isso evita trocar cedo para um pivot sem alvo angular.
            turnAroundPhase_ = TurnAroundPhase::SearchingLine;
            phaseStartedAt_ = now;
            lineReacquireFrames_ = 0;
            lineSearchSawNearLine_ = false;
            lineSearchStartYawDegrees_ = esp32Telemetry.yawZDeg;
        }
        else
        {
            robotState.driveAutonomous(output.leftPower, output.rightPower);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_imu", output.action, output.progressPercent));
            return;
        }
    }

    if (turnAroundPhase_ == TurnAroundPhase::SearchingLine)
    {
        if (!ImuTurnController::imuReady(esp32Telemetry))
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turn_imu_lost",
                "Retorno interrompido: IMU perdida durante a busca da linha"));
            return;
        }

        // A fita pode entrar primeiro por uma lateral da faixa NEAR. Memorizar
        // essa aproximação reduz o pivot, mas não altera o limite de 200°.
        if (cameraLineSnapshot.lineNearAnyDetected)
        {
            lineSearchSawNearLine_ = true;
        }

        if (cameraLineSnapshot.lineNearDetected)
        {
            ++lineReacquireFrames_;
        }
        else
        {
            lineReacquireFrames_ = 0;
        }

        if (lineReacquireFrames_ >=
            config::kGreenTurnAroundLineReacquireFrames)
        {
            turnAroundPhase_ = TurnAroundPhase::Idle;
            turnAroundController_.reset();
            robotState.driveAutonomous(
                cameraLineSnapshot.lineFollowerLeftPower,
                cameraLineSnapshot.lineFollowerRightPower);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "line_following",
                "Retorno 180° concluído: linha próxima recuperada",
                100.0));
            return;
        }

        const double lineSearchDegrees = shortestAngularDistanceDegrees(
            lineSearchStartYawDegrees_, esp32Telemetry.yawZDeg);
        if (lineSearchDegrees >=
            config::kGreenTurnAroundLineSearchMaximumDegrees)
        {
            // O limite encerra somente a manobra de retorno. A câmera está
            // recente e volta a comandar o segue-linha sem permitir mais giro.
            turnAroundPhase_ = TurnAroundPhase::Idle;
            turnAroundController_.reset();
            robotState.driveAutonomous(
                cameraLineSnapshot.lineFollowerLeftPower,
                cameraLineSnapshot.lineFollowerRightPower);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "line_following",
                "Retorno encerrado em aproximadamente 200°: retomando o segue-linha",
                100.0));
            return;
        }
        if (now - phaseStartedAt_ > std::chrono::milliseconds(
                config::kGreenTurnAroundLineSearchTimeoutMs))
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_line_search_timeout",
                "Retorno interrompido: linha não encontrada no tempo seguro"));
            return;
        }

        const double searchPower =
            lineSearchSawNearLine_
                ? config::kGreenTurnAroundLineApproachPower
                : config::kGreenTurnAroundLineSearchPower;
        const double leftPower = configuredTurnSign() * searchPower;
        robotState.driveAutonomous(leftPower, -leftPower);
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            "turnaround_searching_line",
            "Continuando o giro até a linha próxima reaparecer"));
    }
}
