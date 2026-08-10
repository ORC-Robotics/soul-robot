#include "obr/line_follower.h"

#include "obr/config.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

namespace
{
double currentUnixSeconds()
{
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration<double>(now).count();
}

AutonomousStatus makeAutonomousStatus(
    const std::string& phase,
    const std::string& action,
    bool lineDetected = false,
    double rawLineError = 0.0,
    double filteredLineError = 0.0,
    double steeringCorrection = 0.0,
    double progressPercent = 0.0)
{
    AutonomousStatus status;
    status.phase = phase;
    status.action = action;
    status.lineDetected = lineDetected;
    status.rawLineError = rawLineError;
    status.filteredLineError = filteredLineError;
    status.steeringCorrection = steeringCorrection;
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
        phase, action, false, 0.0, 0.0, 0.0, progressPercent);
    status.targetDistanceCm = targetDistanceCm;
    status.leftDistanceCm = leftDistanceCm;
    status.rightDistanceCm = rightDistanceCm;
    status.averageDistanceCm = (leftDistanceCm + rightDistanceCm) * 0.5;
    return status;
}
}

void LineFollower::update(RobotState& robotState, const Esp32TelemetrySnapshot& esp32Telemetry)
{
    const RobotSnapshot snapshot = robotState.snapshot();
    if (snapshot.mode != "autonomous")
    {
        resetMissionState();
        activeAutonomousRunSequence_ = 0;
        return;
    }

    if (activeAutonomousRunSequence_ != snapshot.autonomousRunSequence)
    {
        // Cada nova partida recebe um identificador. Assim, Stop seguido de Auto
        // entre dois ciclos nunca reutiliza yaw ou fase da execução anterior.
        resetMissionState();
        activeAutonomousRunSequence_ = snapshot.autonomousRunSequence;
    }

    if (snapshot.autonomousMission == AutonomousMission::TurnRight90)
    {
        updateTurnRight90(robotState, esp32Telemetry);
        return;
    }

    if (snapshot.autonomousMission == AutonomousMission::DriveDistance)
    {
        updateDriveDistance(robotState, esp32Telemetry, snapshot.driveDistanceTargetCm);
        return;
    }

    turn90Phase_ = Turn90Phase::Idle;
    turn90CorrectionPulseCount_ = 0;
    distancePhase_ = DistancePhase::Idle;

    const auto now = std::chrono::steady_clock::now();
    if (phase_ != Phase::Following)
    {
        updateGreenManeuver(robotState, now);
        return;
    }

    const CameraStatus status = readCameraStatus();
    if (!status.valid || !status.active || !isFresh(status))
    {
        // Se a câmera falhar ou o JSON ficar antigo, a ação segura é parar.
        resetLineControl();
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "waiting_camera", "Aguardando dados válidos da câmera"));
        return;
    }

    if (isGreenAction(status.greenAction) && now >= greenCooldownUntil_)
    {
        startGreenManeuver(status.greenAction, now);
        updateGreenManeuver(robotState, now);
        return;
    }

    followLine(robotState, status);
}

void LineFollower::updateTurnRight90(
    RobotState& robotState, const Esp32TelemetrySnapshot& esp32Telemetry)
{
    const auto now = std::chrono::steady_clock::now();
    const bool imuReady = esp32Telemetry.sensorFresh && esp32Telemetry.mpuOk &&
                          esp32Telemetry.lastSensorAgeMs >= 0 &&
                          esp32Telemetry.lastSensorAgeMs <= config::kTurn90ImuFreshnessMs &&
                          std::isfinite(esp32Telemetry.yawZDeg) &&
                          std::isfinite(esp32Telemetry.gyroZDegPerSec);

    if (turn90Phase_ == Turn90Phase::Idle)
    {
        if (!imuReady)
        {
            // Sem uma referência angular válida, a missão não pode iniciar.
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeAutonomousStatus(
                "waiting_imu", "Aguardando referência angular do MPU6050"));
            return;
        }

        turn90Phase_ = Turn90Phase::Turning;
        turn90StartYawDegrees_ = esp32Telemetry.yawZDeg;
        turn90StartedAt_ = now;
        turn90PhaseStartedAt_ = now;
        turn90CorrectionPulseCount_ = 0;
        turn90CorrectionDirection_ = 1.0;
        std::cout << "Turn-right-90 mission started at yaw="
                  << turn90StartYawDegrees_ << " deg\n";
    }

    const auto elapsed = now - turn90StartedAt_;
    if (elapsed > std::chrono::milliseconds(config::kTurn90TimeoutMs))
    {
        // O timeout evita manter os motores ativos se o ângulo parar de mudar.
        robotState.stop();
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "turn_timeout", "Giro interrompido pelo tempo limite"));
        turn90Phase_ = Turn90Phase::Idle;
        std::cout << "Turn-right-90 mission stopped by timeout\n";
        return;
    }

    if (!imuReady)
    {
        // Depois que o giro começa, perder sua referência angular encerra a
        // missão. Retomar sozinho poderia usar uma posição que mudou sem medição.
        robotState.stop();
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "turn_imu_lost", "Giro interrompido: amostra recente do MPU6050 indisponível"));
        turn90Phase_ = Turn90Phase::Idle;
        std::cout << "Turn-right-90 mission stopped after losing IMU data\n";
        return;
    }

    const double turnedDegrees = angularDistanceDegrees(
        turn90StartYawDegrees_, esp32Telemetry.yawZDeg);
    const double remainingDegrees = config::kTurn90TargetDegrees - turnedDegrees;
    const double progressPercent = std::clamp(
        turnedDegrees / config::kTurn90TargetDegrees * 100.0, 0.0, 100.0);

    if (turn90Phase_ == Turn90Phase::Turning)
    {
        const double predictionSeconds = config::kTurn90BrakePredictionSeconds +
                                         esp32Telemetry.lastSensorAgeMs / 1000.0;
        const double brakeLeadDegrees = std::clamp(
            std::abs(esp32Telemetry.gyroZDegPerSec) * predictionSeconds,
            config::kTurn90StopToleranceDegrees,
            config::kTurn90MaximumBrakeLeadDegrees);

        if (remainingDegrees <= brakeLeadDegrees)
        {
            // O DRV8833 fica habilitado, mas recebe PWM zero antes do alvo. A
            // pausa permite medir quanto a inércia ainda moveu o robô.
            robotState.driveAutonomous(0.0, 0.0);
            turn90Phase_ = Turn90Phase::Settling;
            turn90PhaseStartedAt_ = now;
            robotState.updateAutonomousStatus(makeAutonomousStatus(
                "turn_settling", "PWM zerado: aguardando o giro estabilizar",
                false, 0.0, 0.0, 0.0, progressPercent));
            return;
        }

        // O comando lógico de 0,01 passa pelo perfil operacional. Isso garante
        // a partida com aproximadamente 0,65 / -0,67 de PWM efetivo.
        robotState.driveAutonomous(
            config::kTurn90CommandPower, -config::kTurn90CommandPower);
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "turning_right_90", "Girando 90° à direita com comando lógico de 0,01",
            false, 0.0, 0.0, config::kTurn90CommandPower, progressPercent));
        return;
    }

    if (turn90Phase_ == Turn90Phase::CorrectionPulse)
    {
        const bool targetReached =
            std::abs(remainingDegrees) <= config::kTurn90StopToleranceDegrees;
        const bool targetCrossedDuringPulse =
            remainingDegrees * turn90CorrectionDirection_ <= 0.0;
        if (targetReached || targetCrossedDuringPulse)
        {
            robotState.driveAutonomous(0.0, 0.0);
            turn90Phase_ = Turn90Phase::Settling;
            turn90PhaseStartedAt_ = now;
            robotState.updateAutonomousStatus(makeAutonomousStatus(
                "turn_settling", "Alvo alcançado: aguardando o giro estabilizar",
                false, 0.0, 0.0, 0.0, progressPercent));
            return;
        }

        const auto pulseElapsed = now - turn90PhaseStartedAt_;
        if (pulseElapsed < std::chrono::milliseconds(config::kTurn90CorrectionPulseMs))
        {
            robotState.driveAutonomous(
                turn90CorrectionDirection_ * config::kTurn90CommandPower,
                -turn90CorrectionDirection_ * config::kTurn90CommandPower);
            robotState.updateAutonomousStatus(makeAutonomousStatus(
                "turn_correction",
                std::string(turn90CorrectionDirection_ > 0.0
                                ? "Completando ângulo com correção curta nº "
                                : "Reduzindo excesso com correção reversa nº ") +
                    std::to_string(turn90CorrectionPulseCount_),
                false, 0.0, 0.0,
                turn90CorrectionDirection_ * config::kTurn90CommandPower,
                progressPercent));
            return;
        }

        robotState.driveAutonomous(0.0, 0.0);
        turn90Phase_ = Turn90Phase::Settling;
        turn90PhaseStartedAt_ = now;
    }

    // A estabilização é não bloqueante: o loop continua cuidando do
    // E-Stop, do botão físico, da UART e do timeout enquanto o PWM permanece zero.
    robotState.driveAutonomous(0.0, 0.0);
    const auto settleElapsed = now - turn90PhaseStartedAt_;
    const bool angularMotionStopped =
        std::abs(esp32Telemetry.gyroZDegPerSec) <= config::kTurn90StationaryRateDegPerSec;
    if (settleElapsed < std::chrono::milliseconds(config::kTurn90SettleMs) ||
        !angularMotionStopped)
    {
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "turn_settling", "Aguardando a leitura angular estabilizar",
            false, 0.0, 0.0, 0.0, progressPercent));
        return;
    }

    if (std::abs(remainingDegrees) <= config::kTurn90StopToleranceDegrees)
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "completed", "Giro de 90° concluído", false, 0.0, 0.0, 0.0, 100.0));
        turn90Phase_ = Turn90Phase::Idle;
        std::cout << "Turn-right-90 mission completed at "
                  << turnedDegrees << " deg\n";
        return;
    }

    if (turn90CorrectionPulseCount_ >= config::kTurn90MaximumCorrectionPulses)
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "turn_correction_failed", "Giro parado: correções não alcançaram 90°",
            false, 0.0, 0.0, 0.0, progressPercent));
        turn90Phase_ = Turn90Phase::Idle;
        std::cout << "Turn-right-90 mission stopped after correction limit at "
                  << turnedDegrees << " deg\n";
        return;
    }

    ++turn90CorrectionPulseCount_;
    turn90CorrectionDirection_ = remainingDegrees > 0.0 ? 1.0 : -1.0;
    turn90Phase_ = Turn90Phase::CorrectionPulse;
    turn90PhaseStartedAt_ = now;
    robotState.driveAutonomous(
        turn90CorrectionDirection_ * config::kTurn90CommandPower,
        -turn90CorrectionDirection_ * config::kTurn90CommandPower);
    robotState.updateAutonomousStatus(makeAutonomousStatus(
        "turn_correction",
        turn90CorrectionDirection_ > 0.0
            ? "Aplicando correção para completar o ângulo"
            : "Aplicando correção reversa para reduzir o excesso",
        false, 0.0, 0.0,
        turn90CorrectionDirection_ * config::kTurn90CommandPower,
        progressPercent));
}

void LineFollower::updateDriveDistance(
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
                "distance_invalid_target", "Distância solicitada fora da faixa segura",
                targetDistanceCm, 0.0, 0.0, 0.0));
            return;
        }

        if (!encoderReady)
        {
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeDistanceStatus(
                "waiting_encoders", "Aguardando telemetria recente dos encoders",
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
        std::cout << "Drive-distance mission started: target="
                  << activeDistanceTargetCm_ << " cm, counts/cm="
                  << config::kEncoderCountsPerCentimeter << "\n";
    }

    const double leftCounts = std::abs(
        static_cast<double>(esp32Telemetry.leftEncoderCount - distanceStartLeftCount_));
    const double rightCounts = std::abs(
        static_cast<double>(esp32Telemetry.rightEncoderCount - distanceStartRightCount_));
    const double leftDistanceCm = leftCounts / config::kEncoderCountsPerCentimeter;
    const double rightDistanceCm = rightCounts / config::kEncoderCountsPerCentimeter;
    const double minimumDistanceCm = std::min(leftDistanceCm, rightDistanceCm);
    const double targetCounts =
        activeDistanceTargetCm_ * config::kEncoderCountsPerCentimeter;
    const double progressPercent = std::clamp(
        minimumDistanceCm / activeDistanceTargetCm_ * 100.0, 0.0, 100.0);

    if (now - distanceStartedAt_ >
        std::chrono::milliseconds(config::kDriveDistanceTimeoutMs))
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeDistanceStatus(
            "distance_timeout", "Percurso interrompido pelo tempo limite",
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm, progressPercent));
        distancePhase_ = DistancePhase::Idle;
        std::cout << "Drive-distance mission stopped by timeout\n";
        return;
    }

    if (!encoderReady)
    {
        // Sem amostras recentes, a Raspberry não consegue saber quanto o robô
        // percorreu. Continuar poderia ultrapassar indefinidamente o alvo.
        robotState.stop();
        robotState.updateAutonomousStatus(makeDistanceStatus(
            "distance_encoder_lost", "Percurso interrompido: encoders sem dados recentes",
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm, progressPercent));
        distancePhase_ = DistancePhase::Idle;
        std::cout << "Drive-distance mission stopped after losing encoder telemetry\n";
        return;
    }

    const bool movementPhase = distancePhase_ == DistancePhase::Driving ||
                               distancePhase_ == DistancePhase::CorrectionPulse;
    const double minimumCounts = std::min(leftCounts, rightCounts);
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
        // Os dois lados precisam avançar. Se um encoder parar de responder, usar
        // somente o outro poderia dobrar a distância real antes da parada.
        robotState.stop();
        robotState.updateAutonomousStatus(makeDistanceStatus(
            "distance_encoder_stall", "Percurso interrompido: um lado não avançou",
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm, progressPercent));
        distancePhase_ = DistancePhase::Idle;
        std::cout << "Drive-distance mission stopped after encoder stall\n";
        return;
    }

    if (distancePhase_ == DistancePhase::Driving)
    {
        const double predictionSeconds = config::kDriveDistanceBrakePredictionSeconds +
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
                "distance_settling", "PWM zerado: aguardando o percurso estabilizar",
                activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm, progressPercent));
            return;
        }

        robotState.driveAutonomous(
            config::kDriveDistanceCommandPower,
            config::kDriveDistanceCommandPower);
        robotState.updateAutonomousStatus(makeDistanceStatus(
            "driving_distance", "Avançando até a distância selecionada",
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm, progressPercent));
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
                config::kDriveDistanceCommandPower,
                config::kDriveDistanceCommandPower);
            robotState.updateAutonomousStatus(makeDistanceStatus(
                "distance_correction", "Aplicando correção curta de distância",
                activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm, progressPercent));
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
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm, progressPercent));
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
        std::cout << "Drive-distance mission completed: left=" << leftDistanceCm
                  << " cm, right=" << rightDistanceCm << " cm\n";
        return;
    }

    if (distanceCorrectionPulseCount_ >=
        config::kDriveDistanceMaximumCorrectionPulses)
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeDistanceStatus(
            "distance_correction_failed", "Percurso parado: correções insuficientes",
            activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm, progressPercent));
        distancePhase_ = DistancePhase::Idle;
        return;
    }

    ++distanceCorrectionPulseCount_;
    distancePhase_ = DistancePhase::CorrectionPulse;
    distancePhaseStartedAt_ = now;
    distanceLastProgressAt_ = now;
    lastDistanceProgressCounts_ = minimumCounts;
    robotState.driveAutonomous(
        config::kDriveDistanceCommandPower,
        config::kDriveDistanceCommandPower);
    robotState.updateAutonomousStatus(makeDistanceStatus(
        "distance_correction", "Completando os centímetros restantes",
        activeDistanceTargetCm_, leftDistanceCm, rightDistanceCm, progressPercent));
}

void LineFollower::resetMissionState()
{
    phase_ = Phase::Following;
    activeGreenAction_ = "NENHUM";
    resetLineControl();
    turn90Phase_ = Turn90Phase::Idle;
    turn90CorrectionPulseCount_ = 0;
    turn90CorrectionDirection_ = 1.0;
    distancePhase_ = DistancePhase::Idle;
    activeDistanceTargetCm_ = 0.0;
    lastDistanceProgressCounts_ = 0.0;
    distanceCorrectionPulseCount_ = 0;
}

void LineFollower::resetLineControl()
{
    lastLineError_ = 0.0;
    filteredLineError_ = 0.0;
    lastSteeringCorrection_ = 0.0;
    lastCameraTimestampSeconds_ = 0.0;
    lineErrorFilterInitialized_ = false;
}

bool LineFollower::cameraReady() const
{
    const CameraStatus status = readCameraStatus();
    return status.valid && status.active && status.fps > 0.0 && isFresh(status);
}

LineFollower::CameraStatus LineFollower::readCameraStatus() const
{
    std::ifstream file(config::kCameraStatusPath);
    CameraStatus status;
    if (!file)
    {
        return status;
    }

    std::ostringstream content;
    content << file.rdbuf();
    const std::string json = content.str();
    if (json.empty())
    {
        return status;
    }

    status.valid = true;
    status.active = getJsonBool(json, "active", true);
    status.lineDetected = getJsonBool(json, "lineDetected", false);
    status.fps = getJsonNumber(json, "fps", 0.0);
    status.lineError = getJsonNumber(json, "lineError", 0.0);
    status.timestampSeconds = getJsonNumber(json, "timestamp", 0.0);
    status.greenAction = getJsonString(json, "greenAction", "NENHUM");
    return status;
}

void LineFollower::followLine(RobotState& robotState, const CameraStatus& status)
{
    if (status.lineDetected && !std::isfinite(status.lineError))
    {
        // Um erro visual inválido não pode chegar à mistura dos motores.
        resetLineControl();
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "invalid_vision", "Erro visual inválido: motores parados"));
        return;
    }

    if (!status.lineDetected)
    {
        // Em curvas de 90 graus, a linha pode sair da imagem por alguns ciclos.
        // O robô gira para o último lado conhecido, mas ainda para se o JSON da
        // câmera ficar antigo, porque dados antigos não são seguros.
        if (std::abs(lastLineError_) > config::kLineFollowerLostLineDeadbandPixels)
        {
            const double turnPower = config::kLineFollowerLostLineTurnPower;
            if (lastLineError_ > 0.0)
            {
                robotState.driveAutonomous(turnPower, -turnPower);
                robotState.updateAutonomousStatus(makeAutonomousStatus(
                    "searching_right", "Linha perdida: procurando à direita",
                    false, status.lineError, filteredLineError_, turnPower));
            }
            else
            {
                robotState.driveAutonomous(-turnPower, turnPower);
                robotState.updateAutonomousStatus(makeAutonomousStatus(
                    "searching_left", "Linha perdida: procurando à esquerda",
                    false, status.lineError, filteredLineError_, -turnPower));
            }
            return;
        }

        // A busca em frente também usa o mínimo operacional. Reduzir a base pela
        // metade produziria um comando incapaz de mover os quatro motores.
        robotState.driveAutonomous(config::kLineFollowerBasePower, config::kLineFollowerBasePower);
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "searching_forward", "Linha perdida: avançando para reencontrá-la"));
        return;
    }

    if (!lineErrorFilterInitialized_)
    {
        filteredLineError_ = status.lineError;
        lineErrorFilterInitialized_ = true;
    }
    else if (status.timestampSeconds != lastCameraTimestampSeconds_)
    {
        filteredLineError_ += config::kLineFollowerErrorFilterAlpha *
                              (status.lineError - filteredLineError_);
    }
    lastCameraTimestampSeconds_ = status.timestampSeconds;
    lastLineError_ = filteredLineError_;

    const double controlledError = std::abs(filteredLineError_) <
                                           config::kLineFollowerErrorDeadbandPixels
                                       ? 0.0
                                       : filteredLineError_;
    const double targetCorrection = std::clamp(
        controlledError * config::kLineFollowerTurnGain,
        -config::kLineFollowerMaxTurnCorrection,
        config::kLineFollowerMaxTurnCorrection);

    // Limita a rapidez da mudança para que ruído visual não alterne o torque
    // entre os lados e faça as rodas de borracha tremerem.
    const double correctionStep = std::clamp(
        targetCorrection - lastSteeringCorrection_,
        -config::kLineFollowerCorrectionSlewPerCycle,
        config::kLineFollowerCorrectionSlewPerCycle);
    lastSteeringCorrection_ += correctionStep;

    // A base de 0,65 é mantida no lado interno da curva. Somente o lado externo
    // acelera, evitando que uma correção faça um conjunto cair abaixo da faixa
    // em que os motores conseguem girar de forma confiável.
    double left = config::kLineFollowerBasePower;
    double right = config::kLineFollowerBasePower;
    if (lastSteeringCorrection_ > 0.0)
    {
        left = std::min(
            config::kLineFollowerBasePower + lastSteeringCorrection_,
            config::kOperationalMaximumReferencePower);
    }
    else if (lastSteeringCorrection_ < 0.0)
    {
        right = std::min(
            config::kLineFollowerBasePower - lastSteeringCorrection_,
            config::kOperationalMaximumReferencePower);
    }
    robotState.driveAutonomous(left, right);

    std::string phase = "following_straight";
    std::string action = "Seguindo a linha em frente";
    if (lastSteeringCorrection_ > 0.001)
    {
        phase = "correcting_right";
        action = "Corrigindo trajetória para a direita";
    }
    else if (lastSteeringCorrection_ < -0.001)
    {
        phase = "correcting_left";
        action = "Corrigindo trajetória para a esquerda";
    }
    robotState.updateAutonomousStatus(makeAutonomousStatus(
        phase, action, true, status.lineError, filteredLineError_, lastSteeringCorrection_));
}

void LineFollower::startGreenManeuver(const std::string& action, std::chrono::steady_clock::time_point now)
{
    resetLineControl();
    activeGreenAction_ = action;
    phase_ = Phase::ApproachingGreen;
    phaseUntil_ = now + std::chrono::milliseconds(config::kGreenApproachMs);
    std::cout << "Green action detected: " << action << "\n";
}

void LineFollower::updateGreenManeuver(RobotState& robotState, std::chrono::steady_clock::time_point now)
{
    if (phase_ == Phase::ApproachingGreen)
    {
        if (now < phaseUntil_)
        {
            // Avança devagar para alinhar o centro do robô com a interseção.
            robotState.driveAutonomous(config::kLineFollowerBasePower, config::kLineFollowerBasePower);
            robotState.updateAutonomousStatus(makeAutonomousStatus(
                "approaching_green", "Avançando para alinhar com a marca verde"));
            return;
        }

        phase_ = Phase::TurningGreen;
        const int turnMs = activeGreenAction_ == "MEIA VOLTA" ? config::kGreenUTurnMs : config::kGreenTurnMs;
        phaseUntil_ = now + std::chrono::milliseconds(turnMs);
    }

    if (phase_ == Phase::TurningGreen)
    {
        if (now < phaseUntil_)
        {
            if (activeGreenAction_ == "ESQUERDA")
            {
                robotState.driveAutonomous(-config::kGreenTurnPower, config::kGreenTurnPower);
                robotState.updateAutonomousStatus(makeAutonomousStatus(
                    "green_turn_left", "Executando decisão verde: esquerda",
                    false, 0.0, 0.0, -config::kGreenTurnPower));
            }
            else
            {
                // Direita e meia-volta usam o mesmo sentido inicial de giro.
                robotState.driveAutonomous(config::kGreenTurnPower, -config::kGreenTurnPower);
                const bool uTurn = activeGreenAction_ == "MEIA VOLTA";
                robotState.updateAutonomousStatus(makeAutonomousStatus(
                    uTurn ? "green_u_turn" : "green_turn_right",
                    uTurn ? "Executando decisão verde: meia-volta"
                          : "Executando decisão verde: direita",
                    false, 0.0, 0.0, config::kGreenTurnPower));
            }
            return;
        }

        phase_ = Phase::Following;
        activeGreenAction_ = "NENHUM";
        greenCooldownUntil_ = now + std::chrono::milliseconds(config::kGreenCooldownMs);
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "green_completed", "Decisão verde concluída"));
    }
}

bool LineFollower::isFresh(const CameraStatus& status)
{
    if (status.timestampSeconds <= 0.0)
    {
        return false;
    }

    const double ageMs = (currentUnixSeconds() - status.timestampSeconds) * 1000.0;
    return ageMs >= 0.0 && ageMs <= config::kCameraStatusTimeoutMs;
}

bool LineFollower::isGreenAction(const std::string& action)
{
    return action == "ESQUERDA" || action == "DIREITA" || action == "MEIA VOLTA";
}

double LineFollower::angularDistanceDegrees(double first, double second)
{
    // Normaliza a diferença para [-180, 180]. Isso mantém a medição correta
    // quando o yaw atravessa a transição entre +180 e -180 graus.
    double difference = std::fmod(second - first + 540.0, 360.0) - 180.0;
    return std::abs(difference);
}

double LineFollower::getJsonNumber(const std::string& json, const std::string& key, double fallback)
{
    const std::string pattern = "\"" + key + "\":";
    size_t start = json.find(pattern);
    if (start == std::string::npos)
    {
        return fallback;
    }

    start += pattern.size();
    const size_t end = json.find_first_of(",}", start);
    if (end == std::string::npos)
    {
        return fallback;
    }

    try
    {
        return std::stod(json.substr(start, end - start));
    }
    catch (const std::exception&)
    {
        return fallback;
    }
}

bool LineFollower::getJsonBool(const std::string& json, const std::string& key, bool fallback)
{
    const std::string pattern = "\"" + key + "\":";
    size_t start = json.find(pattern);
    if (start == std::string::npos)
    {
        return fallback;
    }

    start += pattern.size();
    while (start < json.size() && json[start] == ' ')
    {
        ++start;
    }

    if (json.compare(start, 4, "true") == 0)
    {
        return true;
    }
    if (json.compare(start, 5, "false") == 0)
    {
        return false;
    }
    return fallback;
}

std::string LineFollower::getJsonString(const std::string& json, const std::string& key, const std::string& fallback)
{
    const std::string pattern = "\"" + key + "\":";
    size_t start = json.find(pattern);
    if (start == std::string::npos)
    {
        return fallback;
    }

    start = json.find('"', start + pattern.size());
    if (start == std::string::npos)
    {
        return fallback;
    }

    const size_t end = json.find('"', start + 1);
    if (end == std::string::npos)
    {
        return fallback;
    }

    return json.substr(start + 1, end - start - 1);
}
