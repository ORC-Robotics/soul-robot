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
    const CameraStatus status = readCameraStatus();
    if (!status.valid || !status.active || !isFresh(status))
    {
        // Se a câmera falhar ou o JSON ficar antigo, a ação segura é parar.
        clearVisionEvents();
        resetLineControl();
        navigationState_ = NavigationState::Following;
        initialLineAcquired_ = false;
        initialLineValidFrames_ = 0;
        reacquireStationary_ = false;
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "waiting_camera", "Aguardando dados válidos da câmera"));
        return;
    }

    if (!cameraNumbersValid(status))
    {
        // NaN ou infinito em qualquer campo usado para movimento invalida toda
        // a decisão do quadro; nenhum valor antigo pode continuar comandando.
        clearVisionEvents();
        resetLineControl();
        navigationState_ = NavigationState::Following;
        initialLineAcquired_ = false;
        initialLineValidFrames_ = 0;
        reacquireStationary_ = false;
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeAutonomousStatus(
            "invalid_vision", "Geometria visual inválida: motores parados"));
        return;
    }

    updateMainMission(robotState, esp32Telemetry, status, now);
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
        // a partida com aproximadamente 0,65 / -0,65 de PWM efetivo. O giro
        // oposto não usa o sincronismo automático dos encoders.
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
    navigationState_ = NavigationState::Following;
    activeManeuver_ = ManeuverKind::None;
    greenPhase_ = GreenPhase::None;
    activeGreenAction_ = "NENHUM";
    clearVisionEvents();
    resetLineControl();
    reacquireStationary_ = false;
    initialLineAcquired_ = false;
    initialLineValidFrames_ = 0;
    reacquireValidFrames_ = 0;
    lineLostFrames_ = 0;
    lastSearchDirection_ = 0;
    pendingCornerDirection_ = "NONE";
    cornerTranslationEncoderInitialized_ = false;
    cornerTranslationSettling_ = false;
    lastCornerTranslationProgressCounts_ = 0.0;
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
    filteredLineError_ = 0.0;
    filteredHeadingError_ = 0.0;
    lastSteeringCorrection_ = 0.0;
    lastCameraTimestampSeconds_ = 0.0;
    maneuverStartCameraTimestampSeconds_ = 0.0;
    lineErrorFilterInitialized_ = false;
    turnDepartedOldPath_ = false;
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
    status.currentPathValid = getJsonBool(
        json, "currentPathValid", status.lineDetected);
    status.fps = getJsonNumber(json, "fps", 0.0);
    status.lineError = getJsonNumber(json, "lineError", 0.0);
    status.positionErrorPixels = getJsonNumber(json, "positionErrorPixels", status.lineError);
    status.headingErrorDegrees = getJsonNumber(json, "headingErrorDegrees", 0.0);
    status.pathConfidence = getJsonNumber(
        json, "pathConfidence", status.lineDetected ? 1.0 : 0.0);
    status.nearPathX = getJsonNumber(json, "nearPathX", 0.0);
    status.midPathX = getJsonNumber(json, "midPathX", 0.0);
    status.farPathX = getJsonNumber(json, "farPathX", 0.0);
    status.previewEventDetected = getJsonBool(json, "previewEventDetected", false);
    status.previewEventType = getJsonString(json, "previewEventType", "NONE");
    status.previewEventDirection = getJsonString(json, "previewEventDirection", "NONE");
    status.previewEventProximity = getJsonNumber(json, "previewEventProximity", 0.0);
    status.previewEventConfidence = getJsonNumber(json, "previewEventConfidence", 0.0);
    status.timestampSeconds = getJsonNumber(json, "timestamp", 0.0);
    status.greenAction = getJsonString(json, "greenAction", "NENHUM");
    status.greenProximity = getJsonNumber(json, "greenProximity", 0.0);
    status.greenConfidence = getJsonNumber(json, "greenConfidence", 0.0);
    return status;
}

void LineFollower::updateMainMission(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    const CameraStatus& status,
    std::chrono::steady_clock::time_point now)
{
    const bool newCameraSample = status.timestampSeconds != lastCameraTimestampSeconds_;
    if (newCameraSample)
    {
        updateVisionTrackers(status, esp32Telemetry, now);
        lastCameraTimestampSeconds_ = status.timestampSeconds;
    }

    if (navigationState_ == NavigationState::AdvancingToCorner ||
        navigationState_ == NavigationState::ReversingAfterCorner)
    {
        updateCornerTranslation(robotState, esp32Telemetry, status, now, newCameraSample);
        return;
    }

    if (navigationState_ == NavigationState::ExecutingTurn)
    {
        updateExecutingTurn(robotState, esp32Telemetry, status, now, newCameraSample);
        return;
    }

    if (navigationState_ == NavigationState::Reacquiring)
    {
        updateReacquiring(robotState, status, now, newCameraSample);
        return;
    }

    const bool pathUsable = status.lineDetected && status.currentPathValid &&
                            status.pathConfidence >= config::kLineFollowerMinimumPathConfidence;

    if (!initialLineAcquired_)
    {
        if (newCameraSample)
        {
            initialLineValidFrames_ = pathUsable
                                          ? initialLineValidFrames_ + 1
                                          : 0;
        }

        if (initialLineValidFrames_ < config::kInitialLineAcquireFrames)
        {
            // A partida permanece parada até a câmera mostrar uma raiz frontal
            // consistente. Nenhuma direção de busca nasce de um quadro isolado.
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeNavigationStatus(
                "acquiring_initial_line",
                "Confirmando a linha inicial antes de liberar os motores", status));
            return;
        }

        initialLineAcquired_ = true;
        navigationState_ = NavigationState::Following;
        lastSearchDirection_ = 0;
        const double acquiredCameraTimestampSeconds = lastCameraTimestampSeconds_;
        resetLineControl();
        lastCameraTimestampSeconds_ = acquiredCameraTimestampSeconds;
    }

    const TrackedEvent* event = activeNextEvent();
    const double actionProximity = event != nullptr && event->type == "GREEN"
                                       ? config::kGreenActionProximity
                                       : config::kEventActionProximity;
    const double approachProximity = event != nullptr && event->type == "GREEN"
                                         ? config::kGreenApproachProximity
                                         : config::kEventApproachProximity;

    // Em uma curva de 90 graus, a raiz reta pode sair da imagem antes que o
    // Corner Anchor alcance a zona de ação. Um corner já confirmado e latched
    // tem prioridade sobre a busca genérica de linha, desde que já tenha
    // alcançado a zona de aproximação. Eventos verdes não usam esta exceção,
    // pois a manobra deles inclui um avanço cego antes do giro.
    const bool confirmedCornerReachedTransition =
        !pathUsable && event != nullptr && event->type == "CORNER" &&
        event->confirmed && event->latched && isCornerDirection(event->direction) &&
        event->proximity >= config::kEventApproachProximity;
    if (confirmedCornerReachedTransition)
    {
        startCornerAdvance(event->direction, now);
        updateCornerTranslation(robotState, esp32Telemetry, status, now, newCameraSample);
        return;
    }

    if (!pathUsable)
    {
        if (navigationState_ != NavigationState::LineLost)
        {
            navigationState_ = NavigationState::LineLost;
            lineLostStartedAt_ = now;
            lineLostFrames_ = 0;
        }
        updateLineLost(robotState, status, now, newCameraSample);
        return;
    }

    if (navigationState_ == NavigationState::LineLost)
    {
        // Uma linha nova encerra a busca limitada. Os filtros reiniciam nesta
        // geometria para não reutilizar erro do trecho que desapareceu.
        navigationState_ = NavigationState::Following;
        lineErrorFilterInitialized_ = false;
        lineLostFrames_ = 0;
    }

    if (event != nullptr && event->confirmed && event->latched &&
        event->proximity >= actionProximity)
    {
        if (event->type == "GREEN")
        {
            startGreenManeuver(event->direction, now);
            updateExecutingTurn(robotState, esp32Telemetry, status, now, newCameraSample);
        }
        else
        {
            startCornerAdvance(event->direction, now);
            updateCornerTranslation(robotState, esp32Telemetry, status, now, newCameraSample);
        }
        return;
    }

    if (event != nullptr && event->confirmed && event->latched &&
        event->proximity >= approachProximity)
    {
        navigationState_ = NavigationState::ApproachingEvent;
    }
    else if (navigationState_ == NavigationState::ApproachingEvent &&
             (event == nullptr || event->proximity < config::kEventExitProximity))
    {
        navigationState_ = NavigationState::Following;
    }

    if (navigationState_ == NavigationState::ApproachingEvent)
    {
        followCurrentPath(
            robotState, status, config::kLineFollowerApproachPower,
            "approaching_event", "Evento confirmado: seguindo a linha atual até a zona de ação",
            newCameraSample);
        return;
    }

    followCurrentPath(
        robotState, status, config::kLineFollowerStraightPower, "", "", newCameraSample);
}

void LineFollower::updateVisionTrackers(
    const CameraStatus& status,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    std::chrono::steady_clock::time_point now)
{
    if (!initialLineAcquired_)
    {
        // Eventos só começam a acumular confirmação depois que a linha inicial
        // foi validada. Isso evita um corner falso durante a abertura da câmera.
        clearVisionEvents();
        return;
    }

    const bool retainLatchedEvent = navigationState_ == NavigationState::AdvancingToCorner ||
                                    navigationState_ == NavigationState::ExecutingTurn ||
                                    navigationState_ == NavigationState::ReversingAfterCorner ||
                                    navigationState_ == NavigationState::Reacquiring;
    // Preview e verde só podem ganhar confirmação quando o CurrentPath do mesmo
    // status também é utilizável. Isso impede que um fragmento preto distante
    // seja memorizado como evento enquanto a linha próxima está perdida.
    const bool currentPathUsable = status.lineDetected && status.currentPathValid &&
                                   status.pathConfidence >=
                                       config::kLineFollowerMinimumPathConfidence;
    const bool cornerSeen = currentPathUsable &&
                            now >= cornerCooldownUntil_ &&
                            status.previewEventDetected &&
                            status.previewEventType == "CORNER" &&
                            isCornerDirection(status.previewEventDirection) &&
                            status.previewEventConfidence >= config::kEventMinimumConfidence;

    if (cornerSeen && (!cornerEvent_.latched ||
                       cornerEvent_.direction == status.previewEventDirection))
    {
        if (!cornerEvent_.latched && cornerEvent_.direction != status.previewEventDirection)
        {
            cornerEvent_ = {};
        }
        cornerEvent_.type = "CORNER";
        cornerEvent_.direction = status.previewEventDirection;
        cornerEvent_.proximity = status.previewEventProximity;
        cornerEvent_.confidence = status.previewEventConfidence;
        cornerEvent_.misses = 0;
        ++cornerEvent_.hits;
        if (!cornerEvent_.latched && cornerEvent_.hits >= config::kEventConfirmFrames)
        {
            cornerEvent_.confirmed = true;
            cornerEvent_.latched = true;
            cornerEvent_.latchedAt = now;
            cornerEvent_.latchedLeftEncoderCount = esp32Telemetry.leftEncoderCount;
            cornerEvent_.latchedRightEncoderCount = esp32Telemetry.rightEncoderCount;
            std::cout << "Corner latched: direction=" << cornerEvent_.direction
                      << " proximity=" << cornerEvent_.proximity << "\n";
        }
    }
    else
    {
        ++cornerEvent_.misses;
        const bool expiredCandidate = !cornerEvent_.latched &&
                                      cornerEvent_.misses > config::kEventMaxMissFrames;
        const bool expiredLatch = cornerEvent_.latched && !retainLatchedEvent &&
                                  (cornerEvent_.misses > config::kEventLatchedMaxMissFrames ||
                                   now - cornerEvent_.latchedAt >
                                       std::chrono::milliseconds(config::kEventMaximumAgeMs));
        if (expiredCandidate || expiredLatch)
        {
            cornerEvent_ = {};
        }
    }

    const bool greenSeen = currentPathUsable &&
                           now >= greenCooldownUntil_ &&
                           isGreenAction(status.greenAction) &&
                           status.greenConfidence >= config::kGreenMinimumConfidence;
    if (greenSeen && (!greenEvent_.latched || greenEvent_.direction == status.greenAction))
    {
        if (!greenEvent_.latched && greenEvent_.direction != status.greenAction)
        {
            greenEvent_ = {};
        }
        greenEvent_.type = "GREEN";
        greenEvent_.direction = status.greenAction;
        greenEvent_.proximity = status.greenProximity;
        greenEvent_.confidence = status.greenConfidence;
        greenEvent_.misses = 0;
        ++greenEvent_.hits;
        if (!greenEvent_.latched && greenEvent_.hits >= config::kGreenConfirmFrames)
        {
            greenEvent_.confirmed = true;
            greenEvent_.latched = true;
            greenEvent_.latchedAt = now;
            greenEvent_.latchedLeftEncoderCount = esp32Telemetry.leftEncoderCount;
            greenEvent_.latchedRightEncoderCount = esp32Telemetry.rightEncoderCount;
            std::cout << "Green event latched: action=" << greenEvent_.direction
                      << " proximity=" << greenEvent_.proximity << "\n";
        }
    }
    else
    {
        ++greenEvent_.misses;
        const bool expiredCandidate = !greenEvent_.latched &&
                                      greenEvent_.misses > config::kGreenMaxMissFrames;
        const bool expiredLatch = greenEvent_.latched && !retainLatchedEvent &&
                                  (greenEvent_.misses > config::kGreenLatchedMaxMissFrames ||
                                   now - greenEvent_.latchedAt >
                                       std::chrono::milliseconds(config::kGreenMaximumAgeMs));
        if (expiredCandidate || expiredLatch)
        {
            greenEvent_ = {};
        }
    }
}

void LineFollower::followCurrentPath(
    RobotState& robotState,
    const CameraStatus& status,
    double basePower,
    const std::string& forcedPhase,
    const std::string& forcedAction,
    bool newCameraSample)
{
    if (!lineErrorFilterInitialized_)
    {
        filteredLineError_ = status.positionErrorPixels;
        filteredHeadingError_ = status.headingErrorDegrees;
        lineErrorFilterInitialized_ = true;
    }
    else if (newCameraSample)
    {
        filteredLineError_ += config::kLineFollowerPositionFilterAlpha *
                              (status.positionErrorPixels - filteredLineError_);
        filteredHeadingError_ += config::kLineFollowerHeadingFilterAlpha *
                                 (status.headingErrorDegrees - filteredHeadingError_);
    }
    const auto removeDeadband = [](double value, double deadband)
    {
        const double magnitude = std::abs(value);
        if (magnitude <= deadband)
        {
            return 0.0;
        }
        return std::copysign(magnitude - deadband, value);
    };
    const double controlledPosition = removeDeadband(
        filteredLineError_, config::kLineFollowerErrorDeadbandPixels);
    const double controlledHeading = removeDeadband(
        filteredHeadingError_, config::kLineFollowerHeadingDeadbandDegrees);
    const double positionCorrection =
        controlledPosition * config::kLineFollowerPositionGain;
    double headingCorrection =
        controlledHeading * config::kLineFollowerHeadingGain;
    if (positionCorrection * headingCorrection < 0.0)
    {
        // A inclinação ajuda a antecipar o caminho, mas nunca pode inverter a
        // correção necessária para recolocar o centro do robô sobre a linha.
        const double maximumOpposingHeading =
            std::abs(positionCorrection) *
            config::kLineFollowerOpposingHeadingLimitRatio;
        headingCorrection = std::clamp(
            headingCorrection,
            -maximumOpposingHeading,
            maximumOpposingHeading);
    }
    const double targetCorrection = std::clamp(
        positionCorrection + headingCorrection,
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
    double left = basePower;
    double right = basePower;
    if (lastSteeringCorrection_ > 0.0)
    {
        left = std::min(
            basePower + lastSteeringCorrection_,
            config::kOperationalMaximumReferencePower);
        lastSearchDirection_ = 1;
    }
    else if (lastSteeringCorrection_ < 0.0)
    {
        right = std::min(
            basePower - lastSteeringCorrection_,
            config::kOperationalMaximumReferencePower);
        lastSearchDirection_ = -1;
    }
    else
    {
        lastSearchDirection_ = 0;
    }
    robotState.driveAutonomous(left, right);

    std::string phase = forcedPhase.empty() ? "following_straight" : forcedPhase;
    std::string action = forcedAction.empty() ? "Seguindo CurrentPath por posição e heading" : forcedAction;
    if (forcedPhase.empty() && lastSteeringCorrection_ > 0.001)
    {
        phase = "correcting_right";
        action = "Posição da linha pede correção suave para a direita";
    }
    else if (forcedPhase.empty() && lastSteeringCorrection_ < -0.001)
    {
        phase = "correcting_left";
        action = "Posição da linha pede correção suave para a esquerda";
    }
    robotState.updateAutonomousStatus(makeNavigationStatus(
        phase, action, status, lastSteeringCorrection_));
}

void LineFollower::updateLineLost(
    RobotState& robotState,
    const CameraStatus& status,
    std::chrono::steady_clock::time_point now,
    bool newCameraSample)
{
    if (newCameraSample)
    {
        ++lineLostFrames_;
    }

    if (now - lineLostStartedAt_ >
        std::chrono::milliseconds(config::kLineLostSearchTimeoutMs))
    {
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeNavigationStatus(
            "line_search_timeout", "Busca encerrada pelo tempo limite: motores parados",
            status));
        return;
    }

    // A primeira busca repete o diferencial realmente usado no último quadro.
    // Se a perda persistir, o mesmo sentido recebe o diferencial mínimo completo.
    // Os dois lados continuam avançando; esta rotina nunca executa um corner.
    double searchCorrection = lastSteeringCorrection_;
    if (lineLostFrames_ > config::kLineLostGraceFrames && lastSearchDirection_ != 0)
    {
        searchCorrection = std::copysign(
            std::max(
                std::abs(searchCorrection),
                config::kLineLostMinimumSteeringCorrection),
            static_cast<double>(lastSearchDirection_));
    }

    double left = config::kLineFollowerBasePower;
    double right = config::kLineFollowerBasePower;
    if (searchCorrection > 0.0)
    {
        left = std::min(
            left + searchCorrection,
            config::kOperationalMaximumReferencePower);
    }
    else if (searchCorrection < 0.0)
    {
        right = std::min(
            right - searchCorrection,
            config::kOperationalMaximumReferencePower);
    }
    robotState.driveAutonomous(left, right);

    if (lastSearchDirection_ > 0)
    {
        robotState.updateAutonomousStatus(makeNavigationStatus(
            lineLostFrames_ <= config::kLineLostGraceFrames
                ? "line_lost_grace"
                : "searching_right",
            "Linha perdida: mantendo o último arco para a direita",
            status, searchCorrection));
    }
    else if (lastSearchDirection_ < 0)
    {
        robotState.updateAutonomousStatus(makeNavigationStatus(
            lineLostFrames_ <= config::kLineLostGraceFrames
                ? "line_lost_grace"
                : "searching_left",
            "Linha perdida: mantendo o último arco para a esquerda",
            status, searchCorrection));
    }
    else
    {
        robotState.updateAutonomousStatus(makeNavigationStatus(
            "searching_forward",
            "Linha perdida após trajetória central: procurando à frente",
            status, 0.0));
    }
}

void LineFollower::startCornerAdvance(
    const std::string& direction,
    std::chrono::steady_clock::time_point now)
{
    pendingCornerDirection_ = direction;
    navigationState_ = NavigationState::AdvancingToCorner;
    cornerTranslationEncoderInitialized_ = false;
    cornerTranslationSettling_ = false;
    cornerTranslationStartedAt_ = now;
    cornerTranslationLastProgressAt_ = now;
    lastCornerTranslationProgressCounts_ = 0.0;
    std::cout << "Advancing before visual corner: direction=" << direction
              << ", target=" << config::kCornerAdvanceDistanceCm << " cm\n";
}

void LineFollower::startCornerReverse(std::chrono::steady_clock::time_point now)
{
    navigationState_ = NavigationState::ReversingAfterCorner;
    cornerTranslationEncoderInitialized_ = false;
    cornerTranslationSettling_ = false;
    cornerTranslationStartedAt_ = now;
    cornerTranslationLastProgressAt_ = now;
    lastCornerTranslationProgressCounts_ = 0.0;
    phaseUntil_ = now +
                  std::chrono::milliseconds(config::kCornerReverseStartSettleMs);
    std::cout << "Reversing after visual corner: target="
              << config::kCornerReverseDistanceCm << " cm\n";
}

void LineFollower::updateCornerTranslation(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    const CameraStatus& status,
    std::chrono::steady_clock::time_point now,
    bool newCameraSample)
{
    const bool reversing = navigationState_ == NavigationState::ReversingAfterCorner;
    const double targetDistanceCm = reversing
                                        ? config::kCornerReverseDistanceCm
                                        : config::kCornerAdvanceDistanceCm;
    const bool encoderReady = esp32Telemetry.sensorFresh &&
                              esp32Telemetry.lastSensorAgeMs >= 0 &&
                              esp32Telemetry.lastSensorAgeMs <=
                                  config::kDriveDistanceEncoderFreshnessMs &&
                              std::isfinite(esp32Telemetry.leftEncoderRate) &&
                              std::isfinite(esp32Telemetry.rightEncoderRate);

    if (!encoderReady)
    {
        // Sem telemetria recente, não é possível limitar o deslocamento. A
        // missão é encerrada para impedir avanço ou ré indefinidos.
        robotState.stop();
        AutonomousStatus encoderLostStatus = makeNavigationStatus(
            reversing ? "corner_reverse_encoder_lost" : "corner_advance_encoder_lost",
            reversing
                ? "Ré do corner interrompida: encoders sem dados recentes"
                : "Avanço do corner interrompido: encoders sem dados recentes",
            status);
        encoderLostStatus.targetDistanceCm = targetDistanceCm;
        robotState.updateAutonomousStatus(encoderLostStatus);
        std::cout << "Corner translation stopped after losing encoder telemetry\n";
        return;
    }

    if (reversing && !cornerTranslationEncoderInitialized_ && now < phaseUntil_)
    {
        // O giro recebe alguns ciclos de PWM zero antes de registrar a origem
        // da ré. Assim, rotação residual não entra na distância dos encoders.
        robotState.driveAutonomous(0.0, 0.0);
        AutonomousStatus preparingStatus = makeNavigationStatus(
            "corner_reverse_preparing",
            "Novo segmento encontrado: estabilizando antes da ré", status);
        preparingStatus.targetDistanceCm = targetDistanceCm;
        robotState.updateAutonomousStatus(preparingStatus);
        return;
    }

    if (!cornerTranslationEncoderInitialized_)
    {
        cornerTranslationStartLeftCount_ = esp32Telemetry.leftEncoderCount;
        cornerTranslationStartRightCount_ = esp32Telemetry.rightEncoderCount;
        cornerTranslationLastProgressAt_ = now;
        cornerTranslationEncoderInitialized_ = true;
    }

    const double leftCounts = std::abs(static_cast<double>(
        esp32Telemetry.leftEncoderCount - cornerTranslationStartLeftCount_));
    const double rightCounts = std::abs(static_cast<double>(
        esp32Telemetry.rightEncoderCount - cornerTranslationStartRightCount_));
    const double minimumCounts = std::min(leftCounts, rightCounts);
    const double leftDistanceCm = leftCounts / config::kEncoderCountsPerCentimeter;
    const double rightDistanceCm = rightCounts / config::kEncoderCountsPerCentimeter;
    const double minimumDistanceCm = std::min(leftDistanceCm, rightDistanceCm);
    const double progressPercent = std::clamp(
        minimumDistanceCm / targetDistanceCm * 100.0, 0.0, 100.0);

    auto makeTranslationStatus = [&](const std::string& phase, const std::string& action)
    {
        AutonomousStatus translationStatus = makeNavigationStatus(phase, action, status);
        translationStatus.progressPercent = progressPercent;
        translationStatus.targetDistanceCm = targetDistanceCm;
        translationStatus.leftDistanceCm = leftDistanceCm;
        translationStatus.rightDistanceCm = rightDistanceCm;
        translationStatus.averageDistanceCm = (leftDistanceCm + rightDistanceCm) * 0.5;
        return translationStatus;
    };

    if (now - cornerTranslationStartedAt_ >
        std::chrono::milliseconds(config::kCornerTranslationTimeoutMs))
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeTranslationStatus(
            reversing ? "corner_reverse_timeout" : "corner_advance_timeout",
            reversing
                ? "Ré do corner interrompida pelo tempo limite"
                : "Avanço do corner interrompido pelo tempo limite"));
        std::cout << "Corner translation stopped by timeout\n";
        return;
    }

    if (cornerTranslationSettling_)
    {
        robotState.driveAutonomous(0.0, 0.0);
        if (now < phaseUntil_)
        {
            robotState.updateAutonomousStatus(makeTranslationStatus(
                reversing ? "corner_reverse_settling" : "corner_advance_settling",
                reversing
                    ? "Ré concluída: estabilizando para recalcular a linha"
                    : "Avanço concluído: estabilizando antes do giro"));
            return;
        }

        if (reversing)
        {
            beginReacquiring(now, true);
            updateReacquiring(robotState, status, now, newCameraSample);
        }
        else
        {
            startCornerManeuver(pendingCornerDirection_, now);
            updateExecutingTurn(
                robotState, esp32Telemetry, status, now, newCameraSample);
        }
        return;
    }

    if (minimumDistanceCm >= targetDistanceCm)
    {
        // O PWM fica zerado durante alguns ciclos reais antes da próxima fase.
        // Isso separa a inércia do deslocamento da rotação ou do novo steering.
        robotState.driveAutonomous(0.0, 0.0);
        cornerTranslationSettling_ = true;
        phaseUntil_ = now +
                      std::chrono::milliseconds(config::kCornerTranslationSettleMs);
        robotState.updateAutonomousStatus(makeTranslationStatus(
            reversing ? "corner_reverse_settling" : "corner_advance_settling",
            reversing
                ? "Ré concluída: estabilizando para recalcular a linha"
                : "Avanço concluído: estabilizando antes do giro"));
        std::cout << (reversing ? "Corner reverse completed: left="
                                : "Corner advance completed: left=")
                  << leftDistanceCm
                  << " cm, right=" << rightDistanceCm << " cm\n";
        return;
    }

    if (minimumCounts >= lastCornerTranslationProgressCounts_ +
                             config::kDriveDistanceMinimumProgressCounts)
    {
        lastCornerTranslationProgressCounts_ = minimumCounts;
        cornerTranslationLastProgressAt_ = now;
    }
    if (now - cornerTranslationLastProgressAt_ >
        std::chrono::milliseconds(config::kDriveDistanceStallTimeoutMs))
    {
        // O menor avanço representa o lado mais lento. Se ele não progride,
        // continuar o deslocamento poderia mover somente metade da tração.
        robotState.stop();
        robotState.updateAutonomousStatus(makeTranslationStatus(
            reversing ? "corner_reverse_encoder_stall"
                      : "corner_advance_encoder_stall",
            reversing
                ? "Ré do corner interrompida: um lado não avançou"
                : "Avanço do corner interrompido: um lado não avançou"));
        std::cout << "Corner translation stopped after encoder stall\n";
        return;
    }

    const double translationPower = reversing
                                        ? -config::kLineFollowerApproachPower
                                        : config::kLineFollowerApproachPower;
    robotState.driveAutonomous(
        translationPower,
        translationPower);
    robotState.updateAutonomousStatus(makeTranslationStatus(
        reversing ? "corner_reverse" : "corner_advance",
        reversing
            ? "Curva concluída: recuando a distância configurada"
            : "Corner confirmado: avançando a distância configurada"));
}

void LineFollower::startCornerManeuver(
    const std::string& direction,
    std::chrono::steady_clock::time_point now)
{
    activeManeuver_ = direction == "LEFT"
                          ? ManeuverKind::CornerLeft
                          : ManeuverKind::CornerRight;
    pendingCornerDirection_ = "NONE";
    cornerTranslationEncoderInitialized_ = false;
    cornerTranslationSettling_ = false;
    navigationState_ = NavigationState::ExecutingTurn;
    maneuverStartedAt_ = now;
    maneuverStartCameraTimestampSeconds_ = lastCameraTimestampSeconds_;
    turnDepartedOldPath_ = false;
    reacquireValidFrames_ = 0;
    std::cout << "Executing visual corner: " << direction << "\n";
}

void LineFollower::startGreenManeuver(
    const std::string& action,
    std::chrono::steady_clock::time_point now)
{
    activeGreenAction_ = action;
    activeManeuver_ = action == "ESQUERDA"
                          ? ManeuverKind::GreenLeft
                          : action == "MEIA VOLTA" ? ManeuverKind::GreenUTurn
                                                   : ManeuverKind::GreenRight;
    greenPhase_ = GreenPhase::Approaching;
    navigationState_ = NavigationState::ExecutingTurn;
    maneuverStartedAt_ = now;
    maneuverStartCameraTimestampSeconds_ = lastCameraTimestampSeconds_;
    turnDepartedOldPath_ = false;
    phaseUntil_ = now + std::chrono::milliseconds(config::kGreenApproachMs);
    reacquireValidFrames_ = 0;
    std::cout << "Executing confirmed green action: " << action << "\n";
}

void LineFollower::updateExecutingTurn(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    const CameraStatus& status,
    std::chrono::steady_clock::time_point now,
    bool newCameraSample)
{
    if (now - maneuverStartedAt_ >
        std::chrono::milliseconds(config::kCornerTurnTimeoutMs))
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeNavigationStatus(
            "corner_turn_timeout", "Giro visual interrompido pelo tempo limite", status));
        std::cout << "Visual turn stopped by timeout\n";
        return;
    }

    if (greenPhase_ == GreenPhase::Approaching)
    {
        if (now < phaseUntil_)
        {
            robotState.driveAutonomous(
                config::kLineFollowerApproachPower,
                config::kLineFollowerApproachPower);
            robotState.updateAutonomousStatus(makeNavigationStatus(
                "approaching_green", "Verde confirmado: alinhando na zona de ação", status));
            return;
        }
        greenPhase_ = GreenPhase::Turning;
        const int turnMs = activeManeuver_ == ManeuverKind::GreenUTurn
                               ? config::kGreenUTurnMs
                               : config::kGreenTurnMs;
        phaseUntil_ = now + std::chrono::milliseconds(turnMs);
    }

    if (greenPhase_ == GreenPhase::Turning)
    {
        if (now < phaseUntil_)
        {
            if (activeManeuver_ == ManeuverKind::GreenLeft)
            {
                robotState.driveAutonomous(
                    -config::kLineFollowerCounterTurnPower,
                    config::kGreenTurnPower);
                robotState.updateAutonomousStatus(makeNavigationStatus(
                    "green_turn_left", "Executando decisão verde: esquerda",
                    status, -config::kGreenTurnPower));
            }
            else
            {
                robotState.driveAutonomous(
                    config::kGreenTurnPower,
                    -config::kLineFollowerCounterTurnPower);
                const bool uTurn = activeManeuver_ == ManeuverKind::GreenUTurn;
                robotState.updateAutonomousStatus(makeNavigationStatus(
                    uTurn ? "green_u_turn" : "green_turn_right",
                    uTurn ? "Executando decisão verde: meia-volta"
                          : "Executando decisão verde: direita",
                    status, config::kGreenTurnPower));
            }
            return;
        }
        beginReacquiring(now);
        // A readquisição decide o comando no mesmo ciclo. Assim não existe um
        // frame artificial de PWM zero entre o giro e a nova linha.
        updateReacquiring(robotState, status, now, newCameraSample);
        return;
    }

    const bool minimumTurnElapsed = now - maneuverStartedAt_ >=
                                    std::chrono::milliseconds(config::kCornerTurnMinimumMs);
    const bool newGeometryAfterStart =
        status.timestampSeconds > maneuverStartCameraTimestampSeconds_;
    const bool lineReady = status.lineDetected && status.currentPathValid &&
                           status.pathConfidence >= config::kReacquireMinimumPathConfidence &&
                           std::abs(status.positionErrorPixels) <=
                               config::kReacquireMaxPositionErrorPixels &&
                           std::abs(status.headingErrorDegrees) <=
                               config::kReacquireMaxHeadingErrorDegrees;
    if (!lineReady)
    {
        // O novo segmento só pode ser aceito depois que a geometria antiga
        // realmente saiu dos limites. Isso evita encerrar o giro no primeiro frame.
        turnDepartedOldPath_ = true;
    }
    if (minimumTurnElapsed && newGeometryAfterStart && turnDepartedOldPath_ && lineReady)
    {
        // O novo segmento encerra a rotação, mas o eixo traseiro ainda precisa
        // ser reposicionado. A readquisição só começa depois da ré por encoder.
        startCornerReverse(now);
        updateCornerTranslation(
            robotState, esp32Telemetry, status, now, newCameraSample);
        return;
    }

    if (activeManeuver_ == ManeuverKind::CornerLeft)
    {
        // Os lados giram em sentidos opostos para executar o corner no próprio
        // eixo. As rodas omni dianteiras evitam o antigo arraste lateral.
        robotState.driveAutonomous(
            -config::kLineFollowerCounterTurnPower,
            config::kCornerTurnPower);
        robotState.updateAutonomousStatus(makeNavigationStatus(
            "executing_corner_left", "Girando à esquerda até encontrar o novo segmento",
            status, -config::kCornerTurnPower));
    }
    else
    {
        robotState.driveAutonomous(
            config::kCornerTurnPower,
            -config::kLineFollowerCounterTurnPower);
        robotState.updateAutonomousStatus(makeNavigationStatus(
            "executing_corner_right", "Girando à direita até encontrar o novo segmento",
            status, config::kCornerTurnPower));
    }
}

void LineFollower::beginReacquiring(
    std::chrono::steady_clock::time_point now,
    bool waitStationary)
{
    navigationState_ = NavigationState::Reacquiring;
    reacquireStartedAt_ = now;
    reacquireValidFrames_ = 0;
    reacquireStationary_ = waitStationary;
}

void LineFollower::updateReacquiring(
    RobotState& robotState,
    const CameraStatus& status,
    std::chrono::steady_clock::time_point now,
    bool newCameraSample)
{
    if (now - reacquireStartedAt_ >
        std::chrono::milliseconds(config::kReacquireTimeoutMs))
    {
        robotState.stop();
        robotState.updateAutonomousStatus(makeNavigationStatus(
            "reacquire_timeout", "Readquisição falhou: motores parados", status));
        std::cout << "Visual reacquisition stopped by timeout\n";
        return;
    }

    const bool lineReady = status.lineDetected && status.currentPathValid &&
                           status.pathConfidence >= config::kReacquireMinimumPathConfidence &&
                           std::abs(status.positionErrorPixels) <=
                               config::kReacquireMaxPositionErrorPixels &&
                           std::abs(status.headingErrorDegrees) <=
                               config::kReacquireMaxHeadingErrorDegrees;
    if (lineReady)
    {
        if (newCameraSample)
        {
            ++reacquireValidFrames_;
        }

        if (reacquireStationary_ &&
            reacquireValidFrames_ < config::kReacquireFrames)
        {
            // Depois da ré, a geometria é confirmada com o robô parado. O
            // steering só volta a atuar após três status coerentes da câmera.
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeNavigationStatus(
                "reacquiring_stationary",
                "Linha encontrada após a ré: calculando a nova trajetória",
                status));
            return;
        }

        followCurrentPath(
            robotState, status, config::kLineFollowerCornerPower,
            "reacquiring", "Confirmando o novo segmento da linha", newCameraSample);

        if (reacquireValidFrames_ >= config::kReacquireFrames)
        {
            navigationState_ = NavigationState::Following;
            activeManeuver_ = ManeuverKind::None;
            greenPhase_ = GreenPhase::None;
            activeGreenAction_ = "NENHUM";
            greenCooldownUntil_ = now + std::chrono::milliseconds(config::kGreenCooldownMs);
            cornerCooldownUntil_ = now + std::chrono::milliseconds(config::kEventCooldownMs);
            reacquireStationary_ = false;
            clearVisionEvents();
            resetLineControl();
            std::cout << "Visual line reacquired\n";
        }
        return;
    }

    reacquireValidFrames_ = 0;
    if (reacquireStationary_)
    {
        // Sem uma trajetória válida após a ré, mover novamente poderia afastar
        // o robô da branch correta. O timeout de readquisição encerra a missão.
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeNavigationStatus(
            "waiting_line_after_corner",
            "Ré concluída: aguardando uma trajetória válida", status));
        return;
    }

    const bool turnLeft = activeManeuver_ == ManeuverKind::CornerLeft ||
                          activeManeuver_ == ManeuverKind::GreenLeft;
    if (turnLeft)
    {
        robotState.driveAutonomous(
            -config::kLineFollowerCounterTurnPower,
            config::kCornerTurnPower);
        robotState.updateAutonomousStatus(makeNavigationStatus(
            "reacquiring", "Procurando o novo segmento à esquerda", status,
            -config::kCornerTurnPower));
    }
    else
    {
        robotState.driveAutonomous(
            config::kCornerTurnPower,
            -config::kLineFollowerCounterTurnPower);
        robotState.updateAutonomousStatus(makeNavigationStatus(
            "reacquiring", "Procurando o novo segmento à direita", status,
            config::kCornerTurnPower));
    }
}

AutonomousStatus LineFollower::makeNavigationStatus(
    const std::string& phase,
    const std::string& action,
    const CameraStatus& status,
    double steeringCorrection) const
{
    AutonomousStatus result = makeAutonomousStatus(
        phase, action, status.lineDetected, status.positionErrorPixels,
        filteredLineError_, steeringCorrection);
    result.rawHeadingError = status.headingErrorDegrees;
    result.filteredHeadingError = filteredHeadingError_;
    result.pathConfidence = status.pathConfidence;
    result.navigationState = navigationStateName(navigationState_);
    result.greenAction = status.greenAction;
    result.greenProximity = status.greenProximity;
    result.greenConfidence = status.greenConfidence;

    const TrackedEvent* event = activeNextEvent();
    if (event != nullptr)
    {
        result.nextEventType = event->type;
        result.nextEventDirection = event->direction;
        result.nextEventProximity = event->proximity;
        result.nextEventConfidence = event->confidence;
        result.progressPercent = event->proximity * 100.0;
    }
    return result;
}

const LineFollower::TrackedEvent* LineFollower::activeNextEvent() const
{
    // Verde confirmado tem prioridade sobre o corner geométrico porque ele
    // representa uma regra explícita da prova na mesma interseção.
    if (greenEvent_.confirmed || greenEvent_.latched)
    {
        return &greenEvent_;
    }
    if (cornerEvent_.confirmed || cornerEvent_.latched)
    {
        return &cornerEvent_;
    }
    return nullptr;
}

void LineFollower::clearVisionEvents()
{
    cornerEvent_ = {};
    greenEvent_ = {};
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

bool LineFollower::isCornerDirection(const std::string& direction)
{
    return direction == "LEFT" || direction == "RIGHT";
}

bool LineFollower::cameraNumbersValid(const CameraStatus& status)
{
    const bool basicValuesValid = std::isfinite(status.fps) &&
                                  std::isfinite(status.timestampSeconds) &&
                                  std::isfinite(status.positionErrorPixels) &&
                                  std::isfinite(status.headingErrorDegrees) &&
                                  std::isfinite(status.pathConfidence) &&
                                  status.pathConfidence >= 0.0 &&
                                  status.pathConfidence <= 1.0;
    if (!basicValuesValid)
    {
        return false;
    }

    if (status.lineDetected &&
        (!std::isfinite(status.nearPathX) ||
         !std::isfinite(status.midPathX) ||
         !std::isfinite(status.farPathX)))
    {
        return false;
    }

    if (status.previewEventDetected &&
        (!std::isfinite(status.previewEventProximity) ||
         !std::isfinite(status.previewEventConfidence) ||
         status.previewEventProximity < 0.0 || status.previewEventProximity > 1.0 ||
         status.previewEventConfidence < 0.0 || status.previewEventConfidence > 1.0))
    {
        return false;
    }

    if (isGreenAction(status.greenAction) &&
        (!std::isfinite(status.greenProximity) ||
         !std::isfinite(status.greenConfidence) ||
         status.greenProximity < 0.0 || status.greenProximity > 1.0 ||
         status.greenConfidence < 0.0 || status.greenConfidence > 1.0))
    {
        return false;
    }
    return true;
}

const char* LineFollower::navigationStateName(NavigationState state)
{
    switch (state)
    {
    case NavigationState::ApproachingEvent:
        return "approaching_event";
    case NavigationState::AdvancingToCorner:
        return "advancing_to_corner";
    case NavigationState::ExecutingTurn:
        return "executing_turn";
    case NavigationState::ReversingAfterCorner:
        return "reversing_after_corner";
    case NavigationState::Reacquiring:
        return "reacquiring";
    case NavigationState::LineLost:
        return "line_lost";
    case NavigationState::Following:
    default:
        return "following";
    }
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
