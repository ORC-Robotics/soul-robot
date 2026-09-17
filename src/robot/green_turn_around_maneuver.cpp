#include "obr/green_turn_around_maneuver.h"

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

bool turnAroundDetected(const CameraLineSnapshot& snapshot)
{
    return config::kGreenTurnAroundEnabled && snapshot.greenConfirmed &&
           snapshot.greenPathBlackValid && snapshot.greenPairCompatible &&
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

void GreenTurnAroundManeuver::reset()
{
    phase_ = Phase::Idle;
    turnController_.reset();
    lineCenteringController_.reset();
    armed_ = true;
    forwardStartLeftCount_ = 0;
    forwardStartRightCount_ = 0;
    lineReacquireFrames_ = 0;
    lineSearchStartYawDegrees_ = 0.0;
}

bool GreenTurnAroundManeuver::active() const
{
    return phase_ != Phase::Idle;
}

bool GreenTurnAroundManeuver::shouldBlockForwardAssist(
    const CameraLineSnapshot& cameraLineSnapshot) const
{
    return active() || turnAroundDetected(cameraLineSnapshot);
}

bool GreenTurnAroundManeuver::update(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    const CameraLineSnapshot& cameraLineSnapshot)
{
    const auto now = std::chrono::steady_clock::now();

    const auto startTurnAroundForward = [&]()
    {
        if (!turnAroundEncodersReady(esp32Telemetry) ||
            !ImuTurnController::imuReady(esp32Telemetry))
        {
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_waiting_sensors",
                "Retorno pausado: aguardando encoders e MPU6050"));
            return false;
        }

        phase_ = Phase::DrivingForward;
        forwardStartLeftCount_ = esp32Telemetry.leftEncoderCount;
        forwardStartRightCount_ = esp32Telemetry.rightEncoderCount;
        phaseStartedAt_ = now;
        return true;
    };
    // A ré começa com PWM zero para não carregar o giro para o recuo.
    const auto startTurnAroundReverse = [&]()
    {
        turnController_.reset();
        phase_ = Phase::DrivingReverse;
        forwardStartLeftCount_ = esp32Telemetry.leftEncoderCount;
        forwardStartRightCount_ = esp32Telemetry.rightEncoderCount;
        phaseStartedAt_ = now;
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            "turnaround_reverse_starting",
            "Linha recuperada: iniciando a ré de 5 cm"));
    };
    const bool detected180 = turnAroundDetected(cameraLineSnapshot);
    if (phase_ == Phase::Idle && !detected180)
    {
        // Um retorno concluído só pode disparar novamente depois que os dois
        // verdes realmente saírem da percepção confirmada.
        armed_ = true;
    }

    if (phase_ == Phase::Idle && detected180 &&
        armed_)
    {
        if (!turnAroundEncodersReady(esp32Telemetry) ||
            !ImuTurnController::imuReady(esp32Telemetry))
        {
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_waiting_sensors",
                "Retorno detectado: aguardando encoders e MPU6050"));
            return true;
        }

        armed_ = false;
        lineReacquireFrames_ = 0;
        lineSearchStartYawDegrees_ = 0.0;
        // O retorno assume os motores no mesmo ciclo da confirmação verde.
        // A primeira pausa elimina qualquer comando residual do segue-linha.
        phase_ = Phase::RecognitionDelay;
        phaseStartedAt_ = now;
    }

    if (phase_ == Phase::Idle)
    {
        return false;
    }

    if (phase_ == Phase::RecognitionDelay)
    {
        robotState.driveAutonomous(0.0, 0.0);
        if (now - phaseStartedAt_ < std::chrono::milliseconds(
                config::kGreenTurnAroundRecognitionDelayMs))
        {
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_recognition_delay",
                "Retorno 180° reconhecido: aguardando antes do alinhamento"));
            return true;
        }

        phase_ = Phase::Centering;
        phaseStartedAt_ = now;
        lineCenteringController_.start(now);
    }

    if (phase_ == Phase::Centering)
    {
        const LineCenteringOutput centering =
            lineCenteringController_.update(cameraLineSnapshot, now);
        if (centering.completed)
        {
            // O segundo intervalo sempre começa com os motores zerados. Mesmo
            // no timeout, isso interrompe o giro antes do avanço por encoder.
            phase_ = Phase::PostCenteringDelay;
            phaseStartedAt_ = now;
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                centering.timedOut
                    ? "turnaround_centering_timeout_delay"
                    : "turnaround_centered_delay",
                centering.timedOut
                    ? "Alinhamento expirou: aguardando antes do avanço"
                    : "NEAR e MEDIUM alinhados: aguardando antes do avanço"));
            return true;
        }
        if (centering.state == "WAITING_LINE")
        {
            // Sem posição lateral confiável, permanecer parado é mais seguro
            // do que escolher um lado e iniciar um giro cego.
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_centering_waiting_line",
                "Aguardando NEAR e MEDIUM válidos para concluir o alinhamento"));
            return true;
        }
        robotState.driveAutonomous(
            centering.leftPower, centering.rightPower);
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            "turnaround_centering",
            "Retorno 180°: alinhando NEAR e MEDIUM antes do avanço"));
        return true;
    }

    if (phase_ == Phase::PostCenteringDelay)
    {
        robotState.driveAutonomous(0.0, 0.0);
        if (now - phaseStartedAt_ < std::chrono::milliseconds(
                config::kGreenTurnAroundPostCenteringDelayMs))
        {
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_post_centering_delay",
                "Alinhamento encerrado: aguardando antes do avanço"));
            return true;
        }
        if (!startTurnAroundForward())
        {
            return true;
        }
    }

    if (phase_ == Phase::DrivingForward)
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
            return true;
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
            return true;
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
            phase_ = Phase::ForwardSettling;
            phaseStartedAt_ = now;
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeTurnAroundForwardStatus(
                "turnaround_forward_settling",
                "Avanço concluído: aguardando o robô estabilizar",
                leftDistanceCm,
                rightDistanceCm,
                progressPercent));
            return true;
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
        return true;
    }

    if (phase_ == Phase::ForwardSettling)
    {
        robotState.driveAutonomous(0.0, 0.0);
        if (now - phaseStartedAt_ < std::chrono::milliseconds(
                config::kGreenTurnAroundForwardSettleMs))
        {
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_forward_settling",
                "Aguardando a parada antes do giro por IMU"));
            return true;
        }
        if (!turnController_.start(
                config::kGreenTurnAroundImuDegrees,
                configuredTurnDirection(),
                esp32Telemetry,
                config::kGreenTurnAroundImuToleranceDegrees,
                0, 0, 0.0,
                config::kGreenTurnAroundImuTimeoutMs))
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turn_imu_lost",
                "Retorno interrompido: MPU6050 sem referência válida"));
            return true;
        }
        phase_ = Phase::TurningByImu;
    }

    if (phase_ == Phase::TurningByImu)
    {
        const ImuTurnOutput output =
            turnController_.update(esp32Telemetry);
        const bool fusionLineRecovered =
            cameraLineSnapshot.lineControlSource == "fusion" &&
            cameraLineSnapshot.normalSteeringValid;
        const bool lineRecovered =
            cameraLineSnapshot.lineNearDetected || fusionLineRecovered;

        if (output.phase == "turn_settling" && lineRecovered)
        {
            // A faixa recuperada encerra o giro e libera somente a ré curta.
            startTurnAroundReverse();
            return true;
        }
        const bool angularCorrectionsExhausted =
            output.result == ImuTurnResult::Failed &&
            output.phase == "turn_correction_failed";
        if (output.result == ImuTurnResult::Failed && !angularCorrectionsExhausted)
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                output.phase, output.action, output.progressPercent));
            return true;
        }
        if (output.result == ImuTurnResult::Completed || angularCorrectionsExhausted)
        {
            // O ângulo inicial é uma referência para o retorno, não seu objetivo
            // final. Após estabilizar e esgotar as correções, a câmera assume a
            // busca da faixa sem encerrar a missão. Falhas da IMU ainda param;
            // a busca visual mantém seus limites de tempo e deslocamento angular.
            phase_ = Phase::SearchingLine;
            phaseStartedAt_ = now;
            lineReacquireFrames_ = 0;
            lineSearchStartYawDegrees_ = esp32Telemetry.yawZDeg;
            if (angularCorrectionsExhausted)
            {
                std::cout << "Turnaround: angular corrections exhausted; continuing visual line search"
                          << std::endl;
            }
        }
        else
        {
            robotState.driveAutonomous(output.leftPower, output.rightPower);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_imu", output.action, output.progressPercent));
            return true;
        }
    }

    if (phase_ == Phase::SearchingLine)
    {
        if (!ImuTurnController::imuReady(esp32Telemetry))
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turn_imu_lost",
                "Retorno interrompido: IMU perdida durante a busca da linha"));
            return true;
        }

        const bool fusionLineRecovered =
            cameraLineSnapshot.lineControlSource == "fusion" &&
            cameraLineSnapshot.normalSteeringValid;
        const bool lineRecovered =
            cameraLineSnapshot.lineNearDetected || fusionLineRecovered;
        if (lineRecovered)
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
            startTurnAroundReverse();
            return true;
        }

        const double lineSearchDegrees = shortestAngularDistanceDegrees(
            lineSearchStartYawDegrees_, esp32Telemetry.yawZDeg);
        if (lineSearchDegrees >=
            config::kGreenTurnAroundLineSearchMaximumDegrees)
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_line_search_angle_limit",
                "Retorno interrompido: limite angular da busca visual atingido"));
            return true;
        }
        if (now - phaseStartedAt_ > std::chrono::milliseconds(
                config::kGreenTurnAroundLineSearchTimeoutMs))
        {
            robotState.stop();
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_line_search_timeout",
                "Retorno interrompido: linha não encontrada no tempo seguro"));
            return true;
        }

        const double leftPower =
            configuredTurnSign() * config::kGreenTurnAroundLineSearchPower;
        robotState.driveAutonomous(leftPower, -leftPower);
        robotState.updateAutonomousStatus(makeMainMissionStatus(
            "turnaround_searching_line",
            "Continuando o giro até a linha próxima reaparecer"));
    }

    if (phase_ == Phase::DrivingReverse)
    {
        const double leftDistanceCm = std::abs(static_cast<double>(
            esp32Telemetry.leftEncoderCount - forwardStartLeftCount_)) /
            config::kEncoderCountsPerCentimeter;
        const double rightDistanceCm = std::abs(static_cast<double>(
            esp32Telemetry.rightEncoderCount - forwardStartRightCount_)) /
            config::kEncoderCountsPerCentimeter;
        const bool reverseCompleted =
            std::min(leftDistanceCm, rightDistanceCm) >=
                config::kGreenTurnAroundReverseDistanceCm;
        const bool reverseTimedOut = now - phaseStartedAt_ >=
            std::chrono::milliseconds(config::kGreenTurnAroundReverseTimeoutMs);
        // Sem encoders atuais ou após o limite, cancela apenas a ré opcional.
        // Não mantém um comando de recuo antigo nem encerra a missão inteira.
        if (reverseCompleted || reverseTimedOut ||
            !turnAroundEncodersReady(esp32Telemetry))
        {
            phase_ = Phase::Idle;
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeMainMissionStatus(
                "turnaround_reverse_complete",
                reverseCompleted ? "Ré concluída: retomando o segue-linha"
                                 : "Ré encerrada: retomando o segue-linha",
                100.0));
            return true;
        }
        robotState.driveAutonomous(
            -config::kGreenTurnAroundReversePower,
            -config::kGreenTurnAroundReversePower);
        AutonomousStatus status = makeMainMissionStatus(
            "turnaround_reverse", "Retorno 180°: recuando após recuperar a faixa",
            std::clamp(std::min(leftDistanceCm, rightDistanceCm) /
                           config::kGreenTurnAroundReverseDistanceCm * 100.0,
                       0.0, 100.0));
        status.targetDistanceCm = config::kGreenTurnAroundReverseDistanceCm;
        status.leftDistanceCm = leftDistanceCm;
        status.rightDistanceCm = rightDistanceCm;
        status.averageDistanceCm = (leftDistanceCm + rightDistanceCm) * 0.5;
        robotState.updateAutonomousStatus(status);
    }

    return true;
}
