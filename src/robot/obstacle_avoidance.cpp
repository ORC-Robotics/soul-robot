#include "obr/obstacle_avoidance.h"

#include "obr/config.h"

#include <algorithm>
#include <cmath>

namespace
{
bool ultrasonicReadingIsValid(const Esp32TelemetrySnapshot& telemetry)
{
    return telemetry.sensorFresh && telemetry.lastSensorAgeMs >= 0 &&
           telemetry.lastSensorAgeMs <=
               config::kObstacleUltrasonicFreshnessMs &&
           std::isfinite(telemetry.ultrasonicDistanceCm) &&
           telemetry.ultrasonicDistanceCm >= 2.0 &&
           telemetry.ultrasonicDistanceCm <= 400.0;
}
}

ObstacleAvoidanceOutput ObstacleAvoidance::update(
    const Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line,
    bool allowStart)
{
    switch (phase_)
    {
    case Phase::Idle:
        return updateIdle(telemetry, line, allowStart);
    case Phase::Centering:
        return updateCentering(telemetry, line);
    case Phase::TurningLeftForMeasurement:
    case Phase::ReturningToBaseBeforeRight:
    case Phase::TurningRightForMeasurement:
    case Phase::PositioningSelectedSide:
    case Phase::FinalInwardPivot:
        return updateTurn(telemetry);
    case Phase::DrivingSelectedHeading:
        return updateSelectedForward(telemetry);
    case Phase::CurvingAroundObstacle:
        return updateCurve(telemetry);
    }
    return {};
}

void ObstacleAvoidance::reset()
{
    phase_ = Phase::Idle;
    turnController_.reset();
    lineCenteringController_.reset();
    armed_ = true;
    obstacleConfirmationSamples_ = 0;
    rearmConfirmationSamples_ = 0;
    yawBase_ = std::numeric_limits<double>::quiet_NaN();
    leftClearance_ = std::numeric_limits<double>::quiet_NaN();
    rightClearance_ = std::numeric_limits<double>::quiet_NaN();
    selectedSide_ = "NONE";
    clearanceSamples_.clear();
    lastSampleUptimeMs_ = -1;
    forwardStartLeftCount_ = 0;
    forwardStartRightCount_ = 0;
    selectedHeadingYaw_ = std::numeric_limits<double>::quiet_NaN();
    curveStartLeftCount_ = 0;
    curveStartRightCount_ = 0;
    curveStartYaw_ = std::numeric_limits<double>::quiet_NaN();
    curveEndYaw_ = std::numeric_limits<double>::quiet_NaN();
}

bool ObstacleAvoidance::active() const
{
    return phase_ != Phase::Idle;
}

ObstacleAvoidanceOutput ObstacleAvoidance::updateIdle(
    const Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line,
    bool allowStart)
{
    if (!ultrasonicReadingIsValid(telemetry))
    {
        obstacleConfirmationSamples_ = 0;
        rearmConfirmationSamples_ = 0;
        return {};
    }
    if (!armed_)
    {
        if (telemetry.ultrasonicDistanceCm >= config::kObstacleRearmDistanceCm)
        {
            ++rearmConfirmationSamples_;
            if (rearmConfirmationSamples_ >=
                config::kObstacleRearmConfirmationSamples)
            {
                armed_ = true;
                rearmConfirmationSamples_ = 0;
            }
        }
        else
        {
            rearmConfirmationSamples_ = 0;
        }
        return {};
    }
    if (!allowStart ||
        telemetry.ultrasonicDistanceCm > config::kObstacleDetectionDistanceCm)
    {
        obstacleConfirmationSamples_ = 0;
        return {};
    }

    ++obstacleConfirmationSamples_;
    if (obstacleConfirmationSamples_ <
        config::kObstacleDetectionConfirmationSamples)
    {
        return {};
    }
    if (!line.sourceFresh || !ImuTurnController::imuReady(telemetry))
    {
        return output(
            "obstacle_waiting_sensors",
            "Obstáculo detectado: aguardando linha inferior e MPU6050");
    }

    armed_ = false;
    obstacleConfirmationSamples_ = 0;
    phase_ = Phase::Centering;
    lineCenteringController_.start();
    return output(
        "obstacle_detected",
        "Obstáculo confirmado: parando antes da centralização");
}

ObstacleAvoidanceOutput ObstacleAvoidance::updateCentering(
    const Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line)
{
    if (!line.sourceFresh)
    {
        return output(
            "obstacle_centering_waiting_line",
            "CENTRALIZANDO: aguardando visão inferior fresh");
    }
    const LineCenteringOutput centering =
        lineCenteringController_.update(line);
    if (centering.timedOut)
    {
        return fail(
            "obstacle_centering_timeout",
            "Desvio interrompido: centralização da linha expirou");
    }
    if (!centering.completed)
    {
        return output(
            centering.state == "WAITING_LINE"
                ? "obstacle_centering_waiting_line"
                : "obstacle_centering",
            centering.state == "WAITING_LINE"
                ? "CENTRALIZANDO: aguardando NEAR e MEDIUM válidos"
                : "CENTRALIZANDO na linha inferior",
            centering.leftPower,
            centering.rightPower);
    }
    if (!ImuTurnController::imuReady(telemetry))
    {
        return fail(
            "obstacle_imu_lost",
            "Desvio interrompido: IMU inválida ao salvar yawBase");
    }

    yawBase_ = telemetry.yawZDeg;
    if (!startTurnToYaw(
            Phase::TurningLeftForMeasurement,
            normalizedYaw(yawBase_ - config::kObstacleSideScanDegrees),
            telemetry))
    {
        return fail(
            "obstacle_turn_start_failed",
            "Desvio interrompido: não foi possível mirar o lado esquerdo");
    }
    return output(
        "obstacle_centered",
        "Linha centralizada: yawBase salvo; preparando medição esquerda");
}

ObstacleAvoidanceOutput ObstacleAvoidance::updateTurn(
    const Esp32TelemetrySnapshot& telemetry)
{
    const ImuTurnOutput turn = turnController_.update(telemetry);
    const bool collectingLeft = phase_ == Phase::TurningLeftForMeasurement;
    const bool collectingRight = phase_ == Phase::TurningRightForMeasurement;
    if ((turn.phase == "turning" || turn.phase == "turn_correction") &&
        (collectingLeft || collectingRight))
    {
        collectClearanceDuringTurn(telemetry, collectingLeft);
    }
    if (turn.result == ImuTurnResult::Failed)
    {
        return fail("obstacle_" + turn.phase, "Desvio: " + turn.action);
    }
    if (turn.result != ImuTurnResult::Completed)
    {
        if (collectingLeft || collectingRight)
        {
            return output(
                collectingLeft ? "obstacle_measuring_left"
                               : "obstacle_measuring_right",
                collectingLeft
                    ? "MEDINDO ESQUERDA continuamente durante o giro"
                    : "MEDINDO DIREITA continuamente durante o giro",
                turn.leftPower,
                turn.rightPower);
        }
        return output(
            "obstacle_" + turn.phase,
            "Desvio: " + turn.action,
            turn.leftPower,
            turn.rightPower);
    }

    turnController_.reset();
    if (phase_ == Phase::TurningLeftForMeasurement)
    {
        if (!finishClearanceMeasurement(true))
        {
            return fail(
                "obstacle_left_clearance_unavailable",
                "Desvio interrompido: nenhuma leitura válida na varredura esquerda");
        }
        if (!startTurnToYaw(
                Phase::ReturningToBaseBeforeRight, yawBase_, telemetry))
        {
            return fail(
                "obstacle_turn_start_failed",
                "Desvio interrompido: não foi possível retornar ao yawBase");
        }
        return output(
            "obstacle_left_measured",
            "MEDIÇÃO ESQUERDA concluída; retornando ao yawBase");
    }
    if (phase_ == Phase::ReturningToBaseBeforeRight)
    {
        clearanceSamples_.clear();
        lastSampleUptimeMs_ = -1;
        if (!startTurnToYaw(
                Phase::TurningRightForMeasurement,
                normalizedYaw(yawBase_ + config::kObstacleSideScanDegrees),
                telemetry))
        {
            return fail(
                "obstacle_turn_start_failed",
                "Desvio interrompido: não foi possível iniciar a varredura direita");
        }
        return output(
            "obstacle_measuring_right",
            "MEDINDO DIREITA durante o giro de yawBase até 45 graus");
    }
    if (phase_ == Phase::TurningRightForMeasurement)
    {
        if (!finishClearanceMeasurement(false))
        {
            return fail(
                "obstacle_right_clearance_unavailable",
                "Desvio interrompido: nenhuma leitura válida na varredura direita");
        }
        const double difference = leftClearance_ - rightClearance_;
        if (std::abs(difference) <= config::kObstacleClearanceTieCm)
        {
            selectedSide_ =
                config::kObstacleDefaultSideIsRight ? "RIGHT" : "LEFT";
        }
        else
        {
            selectedSide_ = difference > 0.0 ? "LEFT" : "RIGHT";
        }

        const double targetYaw = normalizedYaw(
            yawBase_ + (selectedSide_ == "RIGHT" ? 1.0 : -1.0) *
                           config::kObstacleSideScanDegrees);
        if (std::abs(signedYawError(targetYaw, telemetry.yawZDeg)) <=
            config::kObstacleTurnToleranceDegrees)
        {
            return startSelectedForward(telemetry);
        }
        if (!startTurnToYaw(
                Phase::PositioningSelectedSide, targetYaw, telemetry))
        {
            return fail(
                "obstacle_turn_start_failed",
                "Desvio interrompido: não foi possível posicionar o lado escolhido");
        }
        return output(
            "obstacle_side_selected",
            "LADO ESCOLHIDO: " + selectedSide_ + "; posicionando yaw inicial");
    }
    if (phase_ == Phase::FinalInwardPivot)
    {
        phase_ = Phase::Idle;
        ObstacleAvoidanceOutput result = output(
            "obstacle_completed",
            "Desvio nominal concluído: pivot final para dentro finalizado");
        result.completed = true;
        result.progressPercent = 100.0;
        return result;
    }
    return startSelectedForward(telemetry);
}

void ObstacleAvoidance::collectClearanceDuringTurn(
    const Esp32TelemetrySnapshot& telemetry,
    bool measuringLeft)
{
    const double yawDelta = signedYawError(telemetry.yawZDeg, yawBase_);
    const double sideAngle = measuringLeft ? -yawDelta : yawDelta;
    if (sideAngle < config::kObstacleClearanceIgnoreDegrees ||
        sideAngle > config::kObstacleSideScanDegrees ||
        !ultrasonicReadingIsValid(telemetry) ||
        telemetry.esp32UptimeMs == lastSampleUptimeMs_)
    {
        return;
    }
    clearanceSamples_.push_back(telemetry.ultrasonicDistanceCm);
    lastSampleUptimeMs_ = telemetry.esp32UptimeMs;
}

bool ObstacleAvoidance::finishClearanceMeasurement(bool measuringLeft)
{
    if (clearanceSamples_.empty())
    {
        return false;
    }
    const double clearance = minimumClearance(clearanceSamples_);
    if (measuringLeft)
    {
        leftClearance_ = clearance;
    }
    else
    {
        rightClearance_ = clearance;
    }
    clearanceSamples_.clear();
    lastSampleUptimeMs_ = -1;
    return true;
}

bool ObstacleAvoidance::startTurnToYaw(
    Phase phase,
    double targetYawDegrees,
    const Esp32TelemetrySnapshot& telemetry)
{
    if (!ImuTurnController::imuReady(telemetry))
    {
        return false;
    }
    const double error = signedYawError(targetYawDegrees, telemetry.yawZDeg);
    if (std::abs(error) <= config::kObstacleTurnToleranceDegrees)
    {
        return false;
    }
    if (!turnController_.start(
            std::abs(error),
            error > 0.0 ? ImuTurnDirection::Right : ImuTurnDirection::Left,
            telemetry,
            config::kObstacleTurnToleranceDegrees,
            config::kObstacleTurnCorrectionPulseMs,
            config::kObstacleTurnMaximumCorrectionPulses,
            config::kObstacleTurnCommandPower,
            config::kObstacleTurnTimeoutMs))
    {
        return false;
    }
    phase_ = phase;
    return true;
}

ObstacleAvoidanceOutput ObstacleAvoidance::startSelectedForward(
    const Esp32TelemetrySnapshot& telemetry)
{
    turnController_.reset();
    phase_ = Phase::DrivingSelectedHeading;
    selectedHeadingYaw_ = telemetry.yawZDeg;
    forwardStartLeftCount_ = telemetry.leftEncoderCount;
    forwardStartRightCount_ = telemetry.rightEncoderCount;
    forwardStartedAt_ = std::chrono::steady_clock::now();
    ObstacleAvoidanceOutput result = output(
        "obstacle_selected_forward_start",
        "LADO ESCOLHIDO: " + selectedSide_ +
            "; iniciando avanço de 10 cm no yaw selecionado");
    result.targetDistanceCm = config::kObstacleSelectedForwardDistanceCm;
    return result;
}

ObstacleAvoidanceOutput ObstacleAvoidance::updateSelectedForward(
    const Esp32TelemetrySnapshot& telemetry)
{
    const double leftCounts = std::abs(static_cast<double>(
        telemetry.leftEncoderCount - forwardStartLeftCount_));
    const double rightCounts = std::abs(static_cast<double>(
        telemetry.rightEncoderCount - forwardStartRightCount_));
    const double leftDistanceCm =
        leftCounts / config::kEncoderCountsPerCentimeter;
    const double rightDistanceCm =
        rightCounts / config::kEncoderCountsPerCentimeter;
    const double usedDistanceCm = std::min(leftDistanceCm, rightDistanceCm);
    const double progressPercent = std::clamp(
        usedDistanceCm / config::kObstacleSelectedForwardDistanceCm * 100.0,
        0.0,
        100.0);

    if (!telemetry.sensorFresh || telemetry.lastSensorAgeMs < 0 ||
        telemetry.lastSensorAgeMs > config::kObstacleEncoderFreshnessMs ||
        !std::isfinite(telemetry.leftEncoderRate) ||
        !std::isfinite(telemetry.rightEncoderRate) ||
        !ImuTurnController::imuReady(telemetry))
    {
        return fail(
            "obstacle_selected_forward_sensors_lost",
            "Avanço interrompido: encoders ou IMU sem dados recentes");
    }
    if (std::chrono::steady_clock::now() - forwardStartedAt_ >
        std::chrono::milliseconds(config::kObstacleDistanceSafetyTimeoutMs))
    {
        return fail(
            "obstacle_selected_forward_timeout",
            "Avanço interrompido pelo tempo limite de segurança");
    }

    const double predictionSeconds =
        config::kObstacleBrakePredictionSeconds +
        telemetry.lastSensorAgeMs / 1000.0;
    const double projectedLeftCounts =
        leftCounts + std::abs(telemetry.leftEncoderRate) * predictionSeconds;
    const double projectedRightCounts =
        rightCounts + std::abs(telemetry.rightEncoderRate) * predictionSeconds;
    const double targetCounts =
        config::kObstacleSelectedForwardDistanceCm *
        config::kEncoderCountsPerCentimeter;
    if (std::min(projectedLeftCounts, projectedRightCounts) >= targetCounts)
    {
        ObstacleAvoidanceOutput result = startCurve(telemetry);
        result.targetDistanceCm = config::kObstacleSelectedForwardDistanceCm;
        result.leftDistanceCm = leftDistanceCm;
        result.rightDistanceCm = rightDistanceCm;
        return result;
    }

    const double headingError =
        signedYawError(selectedHeadingYaw_, telemetry.yawZDeg);
    const double normalizedError = std::clamp(
        headingError /
            config::kObstacleSelectedForwardFullHeadingErrorDegrees,
        -1.0,
        1.0);
    const double correction =
        normalizedError *
        config::kObstacleSelectedForwardMaximumHeadingCorrection;
    ObstacleAvoidanceOutput result = output(
        "obstacle_selected_forward",
        "Avançando 10 cm com correção suave pelo yaw selecionado",
        std::clamp(
            config::kObstacleSelectedForwardPower + correction,
            0.0,
            config::kMaxMotorOutput),
        std::clamp(
            config::kObstacleSelectedForwardPower - correction,
            0.0,
            config::kMaxMotorOutput));
    result.progressPercent = progressPercent;
    result.targetDistanceCm = config::kObstacleSelectedForwardDistanceCm;
    result.leftDistanceCm = leftDistanceCm;
    result.rightDistanceCm = rightDistanceCm;
    return result;
}

ObstacleAvoidanceOutput ObstacleAvoidance::startCurve(
    const Esp32TelemetrySnapshot& telemetry)
{
    phase_ = Phase::CurvingAroundObstacle;
    curveStartLeftCount_ = telemetry.leftEncoderCount;
    curveStartRightCount_ = telemetry.rightEncoderCount;
    // A curva parte do yaw real ao fim dos 10 cm; o heading anterior serviu
    // apenas para manter a primeira reta e não deve ser presumido novamente.
    curveStartYaw_ = telemetry.yawZDeg;
    const double oppositeOffset =
        selectedSide_ == "LEFT" ? config::kObstacleCurveEndOffsetDegrees
                                : -config::kObstacleCurveEndOffsetDegrees;
    curveEndYaw_ = normalizedYaw(yawBase_ + oppositeOffset);
    curveStartedAt_ = std::chrono::steady_clock::now();
    ObstacleAvoidanceOutput result = output(
        "obstacle_curve_start",
        "Avanço de 10 cm concluído: iniciando curva suave");
    result.targetDistanceCm = config::kObstacleCurveDistanceCm;
    return result;
}

ObstacleAvoidanceOutput ObstacleAvoidance::updateCurve(
    const Esp32TelemetrySnapshot& telemetry)
{
    const double leftCounts = std::abs(static_cast<double>(
        telemetry.leftEncoderCount - curveStartLeftCount_));
    const double rightCounts = std::abs(static_cast<double>(
        telemetry.rightEncoderCount - curveStartRightCount_));
    const double leftDistanceCm =
        leftCounts / config::kEncoderCountsPerCentimeter;
    const double rightDistanceCm =
        rightCounts / config::kEncoderCountsPerCentimeter;
    const double progress = std::clamp(
        std::min(leftDistanceCm, rightDistanceCm) /
            config::kObstacleCurveDistanceCm,
        0.0,
        1.0);

    if (!telemetry.sensorFresh || telemetry.lastSensorAgeMs < 0 ||
        telemetry.lastSensorAgeMs > config::kObstacleEncoderFreshnessMs ||
        !std::isfinite(telemetry.leftEncoderRate) ||
        !std::isfinite(telemetry.rightEncoderRate) ||
        !ImuTurnController::imuReady(telemetry))
    {
        return fail(
            "obstacle_curve_sensors_lost",
            "Curva interrompida: encoders ou IMU sem dados recentes");
    }
    if (std::chrono::steady_clock::now() - curveStartedAt_ >
        std::chrono::milliseconds(config::kObstacleDistanceSafetyTimeoutMs))
    {
        return fail(
            "obstacle_curve_timeout",
            "Curva interrompida pelo tempo limite de segurança");
    }
    const double totalYawChange = signedYawError(curveEndYaw_, curveStartYaw_);
    const double targetYaw = normalizedYaw(
        curveStartYaw_ + totalYawChange * progress);
    const double headingError = signedYawError(targetYaw, telemetry.yawZDeg);
    if (progress >= 1.0)
    {
        const ImuTurnDirection direction =
            selectedSide_ == "LEFT" ? ImuTurnDirection::Right
                                    : ImuTurnDirection::Left;
        if (!turnController_.start(
                config::kObstacleFinalInwardPivotDegrees,
                direction,
                telemetry,
                config::kObstacleTurnToleranceDegrees,
                config::kObstacleTurnCorrectionPulseMs,
                config::kObstacleTurnMaximumCorrectionPulses,
                config::kObstacleTurnCommandPower,
                config::kObstacleTurnTimeoutMs))
        {
            return fail(
                "obstacle_final_pivot_start_failed",
                "Desvio interrompido: não foi possível iniciar o pivot final");
        }
        phase_ = Phase::FinalInwardPivot;
        ObstacleAvoidanceOutput result = output(
            "obstacle_final_pivot_start",
            "Curva concluída: iniciando pivot final de 20 graus para dentro");
        result.targetDistanceCm = config::kObstacleCurveDistanceCm;
        result.leftDistanceCm = leftDistanceCm;
        result.rightDistanceCm = rightDistanceCm;
        return result;
    }

    const double correction = std::clamp(
        headingError / config::kObstacleCurveFullHeadingErrorDegrees,
        -1.0,
        1.0) * config::kObstacleCurveMaximumHeadingCorrection;
    ObstacleAvoidanceOutput result = output(
        "obstacle_curving",
        "Executando curva suave com yaw alvo progressivo",
        std::clamp(
            config::kObstacleCurveBasePower + correction,
            0.0,
            config::kMaxMotorOutput),
        std::clamp(
            config::kObstacleCurveBasePower - correction,
            0.0,
            config::kMaxMotorOutput));
    result.progressPercent = progress * 100.0;
    result.targetDistanceCm = config::kObstacleCurveDistanceCm;
    result.leftDistanceCm = leftDistanceCm;
    result.rightDistanceCm = rightDistanceCm;
    return result;
}

ObstacleAvoidanceOutput ObstacleAvoidance::output(
    const std::string& phase,
    const std::string& action,
    double leftPower,
    double rightPower) const
{
    ObstacleAvoidanceOutput result;
    result.hasControl = true;
    result.leftPower = leftPower;
    result.rightPower = rightPower;
    result.phase = phase;
    result.action = action;
    result.yawBase = yawBase_;
    result.leftClearance = leftClearance_;
    result.rightClearance = rightClearance_;
    result.selectedSide = selectedSide_;
    return result;
}

ObstacleAvoidanceOutput ObstacleAvoidance::fail(
    const std::string& phase,
    const std::string& action)
{
    turnController_.reset();
    lineCenteringController_.reset();
    phase_ = Phase::Idle;
    obstacleConfirmationSamples_ = 0;
    ObstacleAvoidanceOutput result = output(phase, action);
    result.failed = true;
    return result;
}

double ObstacleAvoidance::minimumClearance(const std::vector<double>& samples)
{
    return *std::min_element(samples.begin(), samples.end());
}

double ObstacleAvoidance::normalizedYaw(double yawDegrees)
{
    double normalized = std::fmod(yawDegrees, 360.0);
    if (normalized > 180.0)
    {
        normalized -= 360.0;
    }
    else if (normalized < -180.0)
    {
        normalized += 360.0;
    }
    return normalized;
}

double ObstacleAvoidance::signedYawError(
    double targetDegrees,
    double currentDegrees)
{
    return normalizedYaw(targetDegrees - currentDegrees);
}
