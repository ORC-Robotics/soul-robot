#include "obr/obstacle_avoidance.h"

#include "obr/config.h"

#include <algorithm>
#include <cmath>
#include <iostream>

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

ObstacleAvoidance::ObstacleAvoidance(bool forceLeftSide)
    : forceLeftSide_(forceLeftSide)
{
}

ObstacleAvoidanceOutput ObstacleAvoidance::update(
    const Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line,
    bool allowStart,
    const ForwardLineSnapshot& forwardLine)
{
    switch (phase_)
    {
    case Phase::Idle:
        return updateIdle(telemetry, line, allowStart, forwardLine);
    case Phase::ReversingBeforeCentering:
        return updateInitialReverse(telemetry);
    case Phase::Centering:
        return updateCentering(telemetry, line);
    case Phase::TurningLeftForMeasurement:
    case Phase::ReturningToBaseBeforeRight:
    case Phase::TurningRightForMeasurement:
    case Phase::PositioningSelectedSide:
    case Phase::ExitPivotRight:
    case Phase::FinalInwardPivot:
        return updateTurn(telemetry, line, forwardLine);
    case Phase::SamplingLeftClearance:
        return updateClearanceSampling(telemetry, true);
    case Phase::SamplingRightClearance:
        return updateClearanceSampling(telemetry, false);
    case Phase::DrivingSelectedHeading:
        return updateSelectedForward(telemetry);
    case Phase::CurvingAroundObstacle:
        return updateCurve(telemetry, line, forwardLine);
    case Phase::ExitPivotWait:
        return updateExitPivotWait(telemetry);
    case Phase::ExitForward:
        return updateExitForward(telemetry, line);
    case Phase::ExitSearchRight:
        return updateExitSearchRight(telemetry, line);
    case Phase::ReacquireForward:
        return updateReacquireForward(telemetry, line);
    case Phase::ReacquireSearch:
        return updateReacquireSearch(telemetry, line);
    case Phase::ParabolaGapLostValidate:
        return updateParabolaGapLostValidation(
            telemetry, line, forwardLine);
    case Phase::ParabolaReacquireForward:
        return updateParabolaReacquireForward(telemetry, line);
    case Phase::ParabolaReacquireSearch:
        return updateParabolaReacquireSearch(telemetry, line);
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
    resetCameraBlackEvidence();
    clearanceSamples_.clear();
    maximumClearanceAngleDegrees_ =
        std::numeric_limits<double>::quiet_NaN();
    lastSampleUptimeMs_ = -1;
    forwardStartLeftCount_ = 0;
    forwardStartRightCount_ = 0;
    reverseStartLeftCount_ = 0;
    reverseStartRightCount_ = 0;
    selectedHeadingYaw_ = std::numeric_limits<double>::quiet_NaN();
    curveStartLeftCount_ = 0;
    curveStartRightCount_ = 0;
    curveStartYaw_ = std::numeric_limits<double>::quiet_NaN();
    curveEndYaw_ = std::numeric_limits<double>::quiet_NaN();
    exitPivotWaitStartedAt_ = {};
    exitForwardStartedAt_ = {};
    exitSearchStartedAt_ = {};
    fusionReacquireFrames_ = 0;
    lastFusionLineSequence_ = 0;
    reacquireForwardStartLeftCount_ = 0;
    reacquireForwardStartRightCount_ = 0;
    reacquireSearchStartYaw_ = std::numeric_limits<double>::quiet_NaN();
    clearCase3Evidence();
}

bool ObstacleAvoidance::active() const
{
    return phase_ != Phase::Idle;
}

ObstacleAvoidanceOutput ObstacleAvoidance::updateIdle(
    const Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line,
    bool allowStart,
    const ForwardLineSnapshot& forwardLine)
{
    const ObstacleAvoidanceOutput case3Output =
        updateCase3Idle(telemetry, line, forwardLine);
    if (case3Output.hasControl)
    {
        return case3Output;
    }
    if (!ultrasonicReadingIsValid(telemetry))
    {
        obstacleConfirmationSamples_ = 0;
        rearmConfirmationSamples_ = 0;
        return case3Output;
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
        return case3Output;
    }
    if (!allowStart ||
        telemetry.ultrasonicDistanceCm > config::kObstacleDetectionDistanceCm)
    {
        obstacleConfirmationSamples_ = 0;
        return case3Output;
    }

    ++obstacleConfirmationSamples_;
    if (obstacleConfirmationSamples_ <
        config::kObstacleDetectionConfirmationSamples)
    {
        return case3Output;
    }
    if (!line.sourceFresh || !ImuTurnController::imuReady(telemetry))
    {
        return output(
            "obstacle_waiting_sensors",
            "Obstáculo detectado: aguardando linha inferior e MPU6050");
    }

    armed_ = false;
    obstacleConfirmationSamples_ = 0;
    resetCameraBlackEvidence();
    clearCase3Evidence();
    phase_ = Phase::ReversingBeforeCentering;
    reverseStartLeftCount_ = telemetry.leftEncoderCount;
    reverseStartRightCount_ = telemetry.rightEncoderCount;
    reverseStartedAt_ = std::chrono::steady_clock::now();
    return output(
        "obstacle_detected",
        "Obstáculo confirmado: parando antes da centralização");
}

ObstacleAvoidanceOutput ObstacleAvoidance::startCentering(
    const std::string& action)
{
    phase_ = Phase::Centering;
    lineCenteringController_.start();
    ObstacleAvoidanceOutput result = output(
        "obstacle_reverse_completed",
        action);
    result.progressPercent = 100.0;
    result.targetDistanceCm = config::kObstacleReverseDistanceCm;
    return result;
}

ObstacleAvoidanceOutput ObstacleAvoidance::updateInitialReverse(
    const Esp32TelemetrySnapshot& telemetry)
{
    const double leftDistanceCm = std::abs(static_cast<double>(
        telemetry.leftEncoderCount - reverseStartLeftCount_)) /
        config::kEncoderCountsPerCentimeter;
    const double rightDistanceCm = std::abs(static_cast<double>(
        telemetry.rightEncoderCount - reverseStartRightCount_)) /
        config::kEncoderCountsPerCentimeter;
    const double usedDistanceCm = std::max(leftDistanceCm, rightDistanceCm);
    const double progressPercent = std::clamp(
        usedDistanceCm / config::kObstacleReverseDistanceCm * 100.0,
        0.0,
        100.0);

    const bool encoderDataFresh =
        telemetry.sensorFresh && telemetry.lastSensorAgeMs >= 0 &&
        telemetry.lastSensorAgeMs <= config::kObstacleEncoderFreshnessMs;
    if (encoderDataFresh &&
        usedDistanceCm >= config::kObstacleReverseDistanceCm)
    {
        ObstacleAvoidanceOutput result = startCentering(
            "Ré inicial concluída: iniciando centralização");
        result.leftDistanceCm = leftDistanceCm;
        result.rightDistanceCm = rightDistanceCm;
        return result;
    }

    if (!encoderDataFresh ||
        std::chrono::steady_clock::now() - reverseStartedAt_ >=
            std::chrono::milliseconds(
                config::kObstacleInitialReverseMaximumMs))
    {
        ObstacleAvoidanceOutput result = startCentering(
            "Ré inicial encerrada sem bloquear o desvio; iniciando centralização");
        result.leftDistanceCm = leftDistanceCm;
        result.rightDistanceCm = rightDistanceCm;
        return result;
    }

    ObstacleAvoidanceOutput result = output(
        "obstacle_initial_reverse",
        "Recuando 2 cm antes de iniciar o desvio",
        -config::kObstacleReversePower,
        -config::kObstacleReversePower);
    result.progressPercent = progressPercent;
    result.targetDistanceCm = config::kObstacleReverseDistanceCm;
    result.leftDistanceCm = leftDistanceCm;
    result.rightDistanceCm = rightDistanceCm;
    return result;
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
    if (forceLeftSide_)
    {
        selectedSide_ = "LEFT";
        selectedSideSource_ = "CONFIG";
        const double targetYaw = normalizedYaw(
            yawBase_ - config::kObstacleSideApproachDegrees);
        if (!startTurnToYaw(
                Phase::PositioningSelectedSide, targetYaw, telemetry))
        {
            return fail(
                "obstacle_turn_start_failed",
                "Desvio interrompido: não foi possível iniciar o perfil esquerdo");
        }
        return output(
            "obstacle_side_selected",
            "Perfil fixo: LEFT; posicionando yaw inicial");
    }
    if (!startTurnToYaw(
            Phase::TurningLeftForMeasurement,
            normalizedYaw(yawBase_ - config::kObstacleClearanceScanDegrees),
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
    const Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line,
    const ForwardLineSnapshot& forwardLine)
{
    const bool collectingLeft = phase_ == Phase::TurningLeftForMeasurement;
    const bool collectingRight = phase_ == Phase::TurningRightForMeasurement;
    const std::string cameraConfirmation =
        collectingLeft || collectingRight
            ? observeCameraBlack(telemetry, forwardLine, collectingLeft)
            : "";
    if (phase_ == Phase::FinalInwardPivot &&
        case3AwaitingFusionAcquire_ &&
        observeCase3FusionAcquire(line))
    {
        armCase3FusionWindow();
    }
    const ImuTurnOutput turn = turnController_.update(telemetry);
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
                !cameraConfirmation.empty()
                    ? cameraConfirmation
                    : collectingLeft
                          ? "MIRANDO ESQUERDA; medindo somente com o robô estabilizado"
                          : "MIRANDO DIREITA; medindo somente com o robô estabilizado",
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
        return startClearanceSampling(true);
    }
    if (phase_ == Phase::ReturningToBaseBeforeRight)
    {
        clearanceSamples_.clear();
        lastSampleUptimeMs_ = -1;
        if (!startTurnToYaw(
                Phase::TurningRightForMeasurement,
                normalizedYaw(yawBase_ + config::kObstacleClearanceScanDegrees),
                telemetry))
        {
            return fail(
                "obstacle_turn_start_failed",
                "Desvio interrompido: não foi possível iniciar a varredura direita");
        }
        return output(
            "obstacle_measuring_right",
            "MIRANDO DIREITA; a medição ocorrerá com o robô estabilizado");
    }
    if (phase_ == Phase::TurningRightForMeasurement)
    {
        return startClearanceSampling(false);
    }
    if (phase_ == Phase::ExitPivotRight)
    {
        phase_ = Phase::ExitForward;
        exitForwardStartedAt_ = std::chrono::steady_clock::now();
        fusionReacquireFrames_ = 0;
        lastFusionLineSequence_ = line.lineSequence;
        return output(
            "obstacle_exit_forward_start",
            "Giro de saída concluído: seguindo reto para procurar a faixa");
    }
    if (phase_ == Phase::FinalInwardPivot)
    {
        phase_ = Phase::Idle;
        case3PostObstacleYaw_ = telemetry.yawZDeg;
        ObstacleAvoidanceOutput result = output(
            "obstacle_completed",
            "Desvio nominal concluído: pivot final para dentro finalizado");
        result.completed = true;
        result.progressPercent = 100.0;
        return result;
    }
    return startSelectedForward(telemetry);
}

ObstacleAvoidanceOutput ObstacleAvoidance::startClearanceSampling(
    bool measuringLeft)
{
    clearanceSamples_.clear();
    maximumClearanceAngleDegrees_ =
        std::numeric_limits<double>::quiet_NaN();
    lastSampleUptimeMs_ = -1;
    clearanceSamplingStartedAt_ = std::chrono::steady_clock::now();
    phase_ = measuringLeft ? Phase::SamplingLeftClearance
                           : Phase::SamplingRightClearance;
    return output(
        measuringLeft ? "obstacle_sampling_left"
                      : "obstacle_sampling_right",
        measuringLeft
            ? "ROBÔ PARADO: estabilizando o ultrassônico à esquerda"
            : "ROBÔ PARADO: estabilizando o ultrassônico à direita");
}

ObstacleAvoidanceOutput ObstacleAvoidance::updateClearanceSampling(
    const Esp32TelemetrySnapshot& telemetry,
    bool measuringLeft)
{
    const auto elapsed = std::chrono::steady_clock::now() -
                         clearanceSamplingStartedAt_;
    if (elapsed > std::chrono::milliseconds(
                      config::kObstacleClearanceSamplingTimeoutMs))
    {
        return fail(
            measuringLeft ? "obstacle_left_clearance_unavailable"
                          : "obstacle_right_clearance_unavailable",
            "Desvio interrompido: amostras estáveis insuficientes do ultrassônico");
    }
    if (elapsed < std::chrono::milliseconds(
                      config::kObstacleClearanceSettleMs))
    {
        return output(
            measuringLeft ? "obstacle_sampling_left"
                          : "obstacle_sampling_right",
            "ROBÔ PARADO: aguardando o eco do giro anterior sair");
    }

    collectStableClearance(telemetry, measuringLeft);
    if (clearanceSamples_.size() < static_cast<std::size_t>(
                                       config::kObstacleClearanceRequiredSamples))
    {
        return output(
            measuringLeft ? "obstacle_sampling_left"
                          : "obstacle_sampling_right",
            "ROBÔ PARADO: coletando ultrassônico estável " +
                std::to_string(clearanceSamples_.size()) + "/" +
                std::to_string(config::kObstacleClearanceRequiredSamples));
    }

    if (!finishClearanceMeasurement(measuringLeft))
    {
        return fail(
            measuringLeft ? "obstacle_left_clearance_unavailable"
                          : "obstacle_right_clearance_unavailable",
            "Desvio interrompido: nenhuma leitura ultrassônica estável");
    }
    if (!measuringLeft)
    {
        return continueAfterRightMeasurement(telemetry);
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
        "MEDIÇÃO ESTÁVEL ESQUERDA concluída; retornando ao yawBase");
}

ObstacleAvoidanceOutput ObstacleAvoidance::continueAfterRightMeasurement(
    const Esp32TelemetrySnapshot& telemetry)
{
    const double difference = leftClearance_ - rightClearance_;
    if (cameraBlackLeft_ != cameraBlackRight_)
    {
        selectedSide_ = cameraBlackLeft_ ? "LEFT" : "RIGHT";
        selectedSideSource_ = "CAMERA_BLACK";
    }
    else if (std::abs(difference) <= config::kObstacleClearanceTieCm)
    {
        selectedSide_ =
            config::kObstacleDefaultSideIsRight ? "RIGHT" : "LEFT";
        selectedSideSource_ = "ULTRASONIC";
    }
    else
    {
        selectedSide_ = difference > 0.0 ? "LEFT" : "RIGHT";
        selectedSideSource_ = "ULTRASONIC";
    }
    std::cout << "Obstacle side decision leftMaximumCm=" << leftClearance_
              << " rightMaximumCm=" << rightClearance_
              << " differenceCm=" << difference
              << " selected=" << selectedSide_
              << " source=" << selectedSideSource_ << '\n';

    const double targetYaw = normalizedYaw(
        yawBase_ + (selectedSide_ == "RIGHT" ? 1.0 : -1.0) *
                       config::kObstacleSideApproachDegrees);
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
        "LADO ESCOLHIDO: " + selectedSide_ + " via " +
            selectedSideSource_ + "; posicionando yaw inicial");
}

void ObstacleAvoidance::collectStableClearance(
    const Esp32TelemetrySnapshot& telemetry,
    bool measuringLeft)
{
    const double yawDelta = signedYawError(telemetry.yawZDeg, yawBase_);
    const double sideAngle = measuringLeft ? -yawDelta : yawDelta;
    const double stableWindowStartDegrees = std::max(
        config::kObstacleClearanceIgnoreDegrees,
        config::kObstacleClearanceScanDegrees -
            config::kObstacleTurnToleranceDegrees);
    const double stableWindowEndDegrees =
        config::kObstacleClearanceScanDegrees +
        config::kObstacleTurnToleranceDegrees;
    if (sideAngle < stableWindowStartDegrees ||
        sideAngle > stableWindowEndDegrees ||
        std::abs(telemetry.gyroZDegPerSec) >
            config::kTurn90StationaryRateDegPerSec ||
        !ultrasonicReadingIsValid(telemetry) ||
        telemetry.esp32UptimeMs == lastSampleUptimeMs_)
    {
        return;
    }
    if (clearanceSamples_.empty() ||
        telemetry.ultrasonicDistanceCm >
            maximumClearance(clearanceSamples_))
    {
        maximumClearanceAngleDegrees_ = sideAngle;
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
    // Compara o maior eco obtido perto do alvo lateral e com o robô parado.
    // Isso reproduz a leitura manual e evita picos capturados durante o giro.
    const double clearance = maximumClearance(clearanceSamples_);
    std::cout << "Obstacle stable clearance side="
              << (measuringLeft ? "LEFT" : "RIGHT")
              << " samples=" << clearanceSamples_.size()
              << " maximumCm=" << clearance
              << " maximumAngleDeg=" << maximumClearanceAngleDegrees_
              << '\n';
    if (measuringLeft)
    {
        leftClearance_ = clearance;
    }
    else
    {
        rightClearance_ = clearance;
    }
    clearanceSamples_.clear();
    maximumClearanceAngleDegrees_ =
        std::numeric_limits<double>::quiet_NaN();
    lastSampleUptimeMs_ = -1;
    return true;
}

std::string ObstacleAvoidance::observeCameraBlack(
    const Esp32TelemetrySnapshot& telemetry,
    const ForwardLineSnapshot& forwardLine,
    bool measuringLeft)
{
    if (!forwardLine.sourceFresh ||
        forwardLine.obstacleBlackSequence == 0 ||
        forwardLine.obstacleBlackSequence == lastCameraBlackSequence_)
    {
        return {};
    }
    // Todo frame novo é consumido uma única vez. Fora da janela ele é apenas
    // descartado, sem apagar uma confirmação obtida anteriormente.
    lastCameraBlackSequence_ = forwardLine.obstacleBlackSequence;
    const double yawDelta = signedYawError(telemetry.yawZDeg, yawBase_);
    const double sideAngle = measuringLeft ? -yawDelta : yawDelta;
    if (sideAngle < config::kObstacleCameraBlackMinimumAngleDegrees ||
        sideAngle > config::kObstacleCameraBlackMaximumAngleDegrees)
    {
        return {};
    }

    int& frames = measuringLeft ? cameraBlackLeftFrames_
                                : cameraBlackRightFrames_;
    bool& confirmed = measuringLeft ? cameraBlackLeft_
                                    : cameraBlackRight_;
    if (!forwardLine.obstacleBlackVisible)
    {
        if (!confirmed)
        {
            frames = 0;
        }
        return {};
    }
    if (confirmed)
    {
        return {};
    }

    frames = std::min(
        frames + 1,
        config::kObstacleCameraBlackConfirmationFrames);
    if (frames < config::kObstacleCameraBlackConfirmationFrames)
    {
        return {};
    }
    confirmed = true;
    const std::string side = measuringLeft ? "LEFT" : "RIGHT";
    std::cout << "CAM1 confirmou faixa " << side << '\n';
    return "CAM1 confirmou faixa " + side;
}

void ObstacleAvoidance::resetCameraBlackEvidence()
{
    cameraBlackLeft_ = false;
    cameraBlackRight_ = false;
    cameraBlackLeftFrames_ = 0;
    cameraBlackRightFrames_ = 0;
    lastCameraBlackSequence_ = 0;
    selectedSideSource_ = "UNDECIDED";
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
            "; iniciando avanço no yaw selecionado");
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
        "Avançando com correção suave pelo yaw selecionado",
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
    // A curva parte do yaw real ao fim da primeira reta; o heading anterior serviu
    // apenas para manter a primeira reta e não deve ser presumido novamente.
    curveStartYaw_ = telemetry.yawZDeg;
    const double oppositeOffset =
        selectedSide_ == "LEFT" ? config::kObstacleCurveEndOffsetDegrees
                                : -config::kObstacleCurveEndOffsetDegrees;
    curveEndYaw_ = normalizedYaw(yawBase_ + oppositeOffset);
    curveStartedAt_ = std::chrono::steady_clock::now();
    fusionReacquireFrames_ = 0;
    lastFusionLineSequence_ = 0;
    lastParabolaSequence_ = 0;
    ObstacleAvoidanceOutput result = output(
        "obstacle_curve_start",
        "Avanço inicial concluído: iniciando curva suave");
    result.targetDistanceCm = config::kObstacleCurveDistanceCm;
    return result;
}

ObstacleAvoidanceOutput ObstacleAvoidance::updateCurve(
    const Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line,
    const ForwardLineSnapshot& forwardLine)
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
    if (!forceLeftSide_)
    {
        observeParabolaFrame(forwardLine);
        // O perfil calibrado completa os 20 cm antes de procurar a saída. No
        // modo adaptativo antigo, o Fusion ainda pode antecipar a recuperação.
        if (line.sourceFresh && line.lineSequence != 0 &&
            line.lineSequence != lastFusionLineSequence_ &&
            !line.obstacleContinuationBand)
        {
            lastFusionLineSequence_ = line.lineSequence;
            fusionReacquireFrames_ = 0;
        }
        if (observeFreshFusion(line) && line.obstacleContinuationBand)
        {
            clearCase3Evidence();
            phase_ = Phase::ReacquireForward;
            reacquireForwardStartLeftCount_ = telemetry.leftEncoderCount;
            reacquireForwardStartRightCount_ = telemetry.rightEncoderCount;
            reacquireForwardStartedAt_ = std::chrono::steady_clock::now();
            fusionReacquireFrames_ = 0;
            ObstacleAvoidanceOutput result = output(
                "obstacle_reacquire_forward",
                "Fusion detectado durante parábola");
            result.targetDistanceCm =
                config::kObstacleReacquireForwardDistanceCm;
            return result;
        }
    }
    const double totalYawChange = signedYawError(curveEndYaw_, curveStartYaw_);
    const double targetYaw = normalizedYaw(
        curveStartYaw_ + totalYawChange * progress);
    const double headingError = signedYawError(targetYaw, telemetry.yawZDeg);
    if (progress >= 1.0)
    {
        if (!forceLeftSide_)
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
            case3AwaitingFusionAcquire_ = bestParabolaSideValid_;
            case3FusionAcquireFrames_ = 0;
            lastCase3LineSequence_ = line.lineSequence;
            ObstacleAvoidanceOutput result = output(
                "obstacle_final_pivot_start",
                "Curva concluída: iniciando pivot final para dentro");
            result.targetDistanceCm = config::kObstacleCurveDistanceCm;
            result.leftDistanceCm = leftDistanceCm;
            result.rightDistanceCm = rightDistanceCm;
            return result;
        }

        phase_ = Phase::ExitPivotWait;
        exitPivotWaitStartedAt_ = std::chrono::steady_clock::now();
        ObstacleAvoidanceOutput result = output(
            "obstacle_exit_pivot_wait",
            "Curva concluída: aguardando antes do giro de saída");
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

ObstacleAvoidanceOutput ObstacleAvoidance::updateExitPivotWait(
    const Esp32TelemetrySnapshot& telemetry)
{
    if (!ImuTurnController::imuReady(telemetry))
    {
        return fail(
            "obstacle_exit_pivot_sensors_lost",
            "Giro de saída interrompido: IMU sem dados recentes");
    }
    if (std::chrono::steady_clock::now() - exitPivotWaitStartedAt_ <
        std::chrono::milliseconds(config::kObstacleExitPivotWaitMs))
    {
        return output(
            "obstacle_exit_pivot_wait",
            "Aguardando a inércia da curva terminar");
    }

    const double targetYaw = normalizedYaw(
        telemetry.yawZDeg + config::kObstacleExitPivotRightDegrees);
    if (!startTurnToYaw(Phase::ExitPivotRight, targetYaw, telemetry))
    {
        return fail(
            "obstacle_exit_pivot_start_failed",
            "Não foi possível iniciar o giro de 25 graus para a direita");
    }
    return output(
        "obstacle_exit_pivot_start",
        "Iniciando giro de 25 graus para a direita");
}

ObstacleAvoidanceOutput ObstacleAvoidance::updateExitForward(
    const Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line)
{
    if (!line.sourceFresh)
    {
        return fail(
            "obstacle_exit_forward_vision_lost",
            "Saída interrompida: visão inferior sem dados recentes");
    }
    if (!telemetry.sensorFresh || telemetry.lastSensorAgeMs < 0 ||
        telemetry.lastSensorAgeMs > config::kObstacleEncoderFreshnessMs ||
        !std::isfinite(telemetry.leftEncoderRate) ||
        !std::isfinite(telemetry.rightEncoderRate) ||
        !ImuTurnController::imuReady(telemetry))
    {
        return fail(
            "obstacle_exit_forward_sensors_lost",
            "Saída interrompida: encoders ou IMU sem dados recentes");
    }
    if (observeFreshFusion(line))
    {
        return completeExit(
            "obstacle_exit_reacquired",
            "Faixa confirmada durante o avanço de saída");
    }

    if (std::chrono::steady_clock::now() - exitForwardStartedAt_ >=
        std::chrono::milliseconds(config::kObstacleExitStraightTimeoutMs))
    {
        phase_ = Phase::ExitSearchRight;
        exitSearchStartedAt_ = std::chrono::steady_clock::now();
        fusionReacquireFrames_ = 0;
        lastFusionLineSequence_ = line.lineSequence;
        return output(
            "obstacle_exit_search_right_start",
            "Faixa ausente após 1,3 segundo: preparando busca à direita");
    }

    return output(
        "obstacle_exit_forward",
        "Seguindo reto por até 1,3 segundo para encontrar a faixa",
        config::kObstacleSelectedForwardPower,
        config::kObstacleSelectedForwardPower);
}

ObstacleAvoidanceOutput ObstacleAvoidance::updateExitSearchRight(
    const Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line)
{
    if (!line.sourceFresh)
    {
        return fail(
            "obstacle_exit_search_vision_lost",
            "Busca interrompida: visão inferior sem dados recentes");
    }
    if (!ImuTurnController::imuReady(telemetry))
    {
        return fail(
            "obstacle_exit_search_sensors_lost",
            "Busca interrompida: IMU sem dados recentes");
    }
    if (observeFreshFusion(line))
    {
        return completeExit(
            "obstacle_exit_reacquired",
            "Faixa confirmada durante a busca à direita");
    }
    if (std::chrono::steady_clock::now() - exitSearchStartedAt_ >=
        std::chrono::milliseconds(
            config::kObstacleExitSearchRightTimeoutMs))
    {
        return fail(
            "obstacle_exit_search_timeout",
            "Faixa não encontrada após dois segundos de busca à direita");
    }

    return output(
        "obstacle_exit_search_right",
        "Girando explicitamente à direita para procurar a faixa",
        config::kObstacleTurnCommandPower,
        -config::kObstacleTurnCommandPower);
}

ObstacleAvoidanceOutput ObstacleAvoidance::completeExit(
    const std::string& phase,
    const std::string& action)
{
    turnController_.reset();
    lineCenteringController_.reset();
    clearCase3Evidence();
    phase_ = Phase::Idle;
    ObstacleAvoidanceOutput result = output(phase, action);
    result.completed = true;
    result.progressPercent = 100.0;
    return result;
}

ObstacleAvoidanceOutput ObstacleAvoidance::updateReacquireForward(
    const Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line)
{
    const double leftCounts = std::abs(static_cast<double>(
        telemetry.leftEncoderCount - reacquireForwardStartLeftCount_));
    const double rightCounts = std::abs(static_cast<double>(
        telemetry.rightEncoderCount - reacquireForwardStartRightCount_));
    const double leftDistanceCm =
        leftCounts / config::kEncoderCountsPerCentimeter;
    const double rightDistanceCm =
        rightCounts / config::kEncoderCountsPerCentimeter;
    const double usedDistanceCm = std::min(leftDistanceCm, rightDistanceCm);

    if (!telemetry.sensorFresh || telemetry.lastSensorAgeMs < 0 ||
        telemetry.lastSensorAgeMs > config::kObstacleEncoderFreshnessMs ||
        !std::isfinite(telemetry.leftEncoderRate) ||
        !std::isfinite(telemetry.rightEncoderRate))
    {
        return fail(
            "obstacle_reacquire_forward_sensors_lost",
            "Recovery interrompido: encoders sem dados recentes");
    }
    if (std::chrono::steady_clock::now() - reacquireForwardStartedAt_ >
        std::chrono::milliseconds(config::kObstacleDistanceSafetyTimeoutMs))
    {
        return fail(
            "obstacle_reacquire_forward_timeout",
            "Recovery interrompido pelo tempo limite de segurança");
    }

    const double predictionSeconds =
        config::kObstacleBrakePredictionSeconds +
        telemetry.lastSensorAgeMs / 1000.0;
    const double projectedLeftCounts =
        leftCounts + std::abs(telemetry.leftEncoderRate) * predictionSeconds;
    const double projectedRightCounts =
        rightCounts + std::abs(telemetry.rightEncoderRate) * predictionSeconds;
    const double targetCounts =
        config::kObstacleReacquireForwardDistanceCm *
        config::kEncoderCountsPerCentimeter;
    if (std::min(projectedLeftCounts, projectedRightCounts) >= targetCounts)
    {
        if (!ImuTurnController::imuReady(telemetry))
        {
            return fail(
                "obstacle_reacquire_search_sensors_lost",
                "Recovery interrompido: IMU indisponível para buscar a linha");
        }
        phase_ = Phase::ReacquireSearch;
        reacquireSearchStartYaw_ = telemetry.yawZDeg;
        reacquireSearchStartedAt_ = std::chrono::steady_clock::now();
        fusionReacquireFrames_ = 0;
        lastFusionLineSequence_ = line.lineSequence;
        ObstacleAvoidanceOutput result = output(
            "obstacle_reacquire_search",
            "Recovery +5cm; buscando linha " + selectedSide_);
        result.progressPercent = 100.0;
        result.targetDistanceCm =
            config::kObstacleReacquireForwardDistanceCm;
        result.leftDistanceCm = leftDistanceCm;
        result.rightDistanceCm = rightDistanceCm;
        return result;
    }

    ObstacleAvoidanceOutput result = output(
        "obstacle_reacquire_forward",
        "Recovery +5cm",
        config::kObstacleSelectedForwardPower,
        config::kObstacleSelectedForwardPower);
    result.progressPercent = std::clamp(
        usedDistanceCm / config::kObstacleReacquireForwardDistanceCm * 100.0,
        0.0,
        100.0);
    result.targetDistanceCm = config::kObstacleReacquireForwardDistanceCm;
    result.leftDistanceCm = leftDistanceCm;
    result.rightDistanceCm = rightDistanceCm;
    return result;
}

ObstacleAvoidanceOutput ObstacleAvoidance::updateReacquireSearch(
    const Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line)
{
    if (!ImuTurnController::imuReady(telemetry))
    {
        return fail(
            "obstacle_reacquire_search_sensors_lost",
            "Recovery interrompido: IMU perdida durante a busca da linha");
    }
    if (observeFreshFusion(line))
    {
        phase_ = Phase::Idle;
        ObstacleAvoidanceOutput result = output(
            "obstacle_reacquired",
            "Linha reacquirida");
        result.completed = true;
        result.progressPercent = 100.0;
        return result;
    }

    const double yawChange = signedYawError(
        telemetry.yawZDeg, reacquireSearchStartYaw_);
    if (std::abs(yawChange) >=
            config::kObstacleReacquireSearchMaximumDegrees ||
        std::chrono::steady_clock::now() - reacquireSearchStartedAt_ >
            std::chrono::milliseconds(config::kObstacleTurnTimeoutMs))
    {
        return fail(
            "obstacle_reacquire_timeout",
            "Recovery timeout");
    }

    const double direction = selectedSide_ == "LEFT" ? -1.0 : 1.0;
    return output(
        "obstacle_reacquire_search",
        "Buscando linha " + selectedSide_,
        direction * config::kObstacleTurnCommandPower,
        -direction * config::kObstacleTurnCommandPower);
}

bool ObstacleAvoidance::observeFreshFusion(
    const CameraLineSnapshot& line)
{
    if (!line.sourceFresh || line.lineSequence == 0 ||
        line.lineSequence == lastFusionLineSequence_)
    {
        return false;
    }
    lastFusionLineSequence_ = line.lineSequence;
    const bool fusionValid =
        line.lineControlSource == "fusion" && line.normalSteeringValid;
    if (!fusionValid)
    {
        fusionReacquireFrames_ = 0;
        return false;
    }
    fusionReacquireFrames_ = std::min(
        fusionReacquireFrames_ + 1,
        config::kObstacleFusionReacquireConfirmationFrames);
    return fusionReacquireFrames_ >=
           config::kObstacleFusionReacquireConfirmationFrames;
}

void ObstacleAvoidance::observeParabolaFrame(
    const ForwardLineSnapshot& forwardLine)
{
    if (!forwardLine.sourceFresh || forwardLine.parabolaSequence == 0 ||
        forwardLine.parabolaSequence == lastParabolaSequence_)
    {
        return;
    }
    lastParabolaSequence_ = forwardLine.parabolaSequence;
    const std::uint64_t left = forwardLine.parabolaLeftBlack;
    const std::uint64_t right = forwardLine.parabolaRightBlack;
    const std::uint64_t winner = std::max(left, right);
    const std::uint64_t loser = std::min(left, right);
    if (winner < config::kObstacleParabolaMinimumBlackPixels ||
        static_cast<double>(winner) <
            static_cast<double>(loser) *
                config::kObstacleParabolaMinimumDominance)
    {
        return;
    }

    const std::uint64_t score = winner - loser;
    if (score <= bestParabolaScore_)
    {
        return;
    }
    rawBestParabolaSide_ = left > right ? "LEFT" : "RIGHT";
    bestParabolaSide_ = rawBestParabolaSide_;
    bestParabolaScore_ = score;
    bestParabolaLeftBlack_ = left;
    bestParabolaRightBlack_ = right;
    bestParabolaSequence_ = forwardLine.parabolaSequence;
    bestParabolaSideValid_ = true;
    std::cout << "Parabola raw best " << rawBestParabolaSide_
              << " recovery side=" << bestParabolaSide_
              << " score=" << bestParabolaScore_ << '\n';
}

bool ObstacleAvoidance::observeCase3FusionAcquire(
    const CameraLineSnapshot& line)
{
    if (!line.sourceFresh || line.lineSequence == 0 ||
        line.lineSequence == lastCase3LineSequence_)
    {
        return false;
    }
    lastCase3LineSequence_ = line.lineSequence;
    const bool fusionValid =
        line.lineControlSource == "fusion" && line.normalSteeringValid;
    if (!fusionValid)
    {
        case3FusionAcquireFrames_ = 0;
        return false;
    }
    case3FusionAcquireFrames_ = std::min(
        case3FusionAcquireFrames_ + 1,
        config::kObstacleFusionReacquireConfirmationFrames);
    return case3FusionAcquireFrames_ >=
           config::kObstacleFusionReacquireConfirmationFrames;
}

void ObstacleAvoidance::armCase3FusionWindow()
{
    if (case3Armed_ || !bestParabolaSideValid_)
    {
        return;
    }
    case3AwaitingFusionAcquire_ = false;
    case3Armed_ = true;
    case3FusionAcquireTime_ = std::chrono::steady_clock::now();
    case3GapLostFrames_ = 0;
}

ObstacleAvoidanceOutput ObstacleAvoidance::startParabolaRearFusionRecovery(
    const Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line)
{
    phase_ = Phase::ParabolaReacquireSearch;
    case3AwaitingFusionAcquire_ = false;
    case3Armed_ = false;
    case3GapLostFrames_ = 0;
    parabolaSearchStartYaw_ = telemetry.yawZDeg;
    parabolaSearchStartedAt_ = std::chrono::steady_clock::now();
    parabolaSearchOppositeSide_ = false;
    parabolaRearFusionRecovery_ = true;
    fusionReacquireFrames_ = 0;
    lastFusionLineSequence_ = line.lineSequence;
    return output(
        "obstacle_parabola_reacquire_search",
        "Rear Fusion blocked; searching " + bestParabolaSide_);
}

ObstacleAvoidanceOutput ObstacleAvoidance::updateCase3Idle(
    const Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line,
    const ForwardLineSnapshot& forwardLine)
{
    if (case3AwaitingFusionAcquire_ && observeCase3FusionAcquire(line))
    {
        armCase3FusionWindow();
    }
    if (!case3Armed_)
    {
        if (!case3AwaitingFusionAcquire_)
        {
            return {};
        }
        ObstacleAvoidanceOutput result = output("", "");
        result.hasControl = false;
        return result;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - case3FusionAcquireTime_ >=
        std::chrono::milliseconds(config::kObstacleCase3FusionWindowMs))
    {
        clearCase3Evidence();
        return {};
    }

    const bool fusionValid =
        line.sourceFresh && line.lineControlSource == "fusion" &&
        line.normalSteeringValid;
    if (fusionValid && ImuTurnController::imuReady(telemetry) &&
        std::isfinite(case3PostObstacleYaw_))
    {
        const double bestDirection =
            bestParabolaSide_ == "LEFT" ? -1.0 : 1.0;
        const double yawFromObstacleCompletion = signedYawError(
            telemetry.yawZDeg, case3PostObstacleYaw_);
        const double wrongDirectionDegrees =
            yawFromObstacleCompletion * -bestDirection;
        if (wrongDirectionDegrees >=
            config::kObstaclePostObstacleFusionReturnLimitDegrees)
        {
            // O Fusion ainda está válido, mas já virou para o lado oposto ao
            // recovery salvo pela CAM1. Assume os motores antes de voltar ao
            // obstáculo e rejeita essa mesma faixa durante o retorno frontal.
            return startParabolaRearFusionRecovery(telemetry, line);
        }
    }

    if (line.sourceFresh && line.lineSequence > 0 &&
        line.lineSequence != lastCase3LineSequence_)
    {
        lastCase3LineSequence_ = line.lineSequence;
        const bool fusionValid =
            line.lineControlSource == "fusion" && line.normalSteeringValid;
        const bool gapOrLost =
            line.gapValidationDecision == "GAP" ||
            line.gapValidationDecision == "LOST";
        if (fusionValid)
        {
            case3GapLostFrames_ = 0;
        }
        else if (gapOrLost)
        {
            case3GapLostFrames_ = std::min(
                case3GapLostFrames_ + 1,
                config::kObstacleParabolaGapLostConfirmationFrames);
        }
        else
        {
            case3GapLostFrames_ = 0;
        }
    }

    if (case3GapLostFrames_ >=
            config::kObstacleParabolaGapLostConfirmationFrames &&
        bestParabolaSideValid_)
    {
        phase_ = Phase::ParabolaGapLostValidate;
        case3Armed_ = false;
        nearForwardLineVisible_ = false;
        nearForwardLineVotes_ = 0;
        nearForwardLineSamples_ = 0;
        lastNearValidationSequence_ = forwardLine.parabolaSequence;
        return output(
            "obstacle_parabola_gaplost_validate",
            "GAP/LOST validating CAM1 NEAR");
    }

    ObstacleAvoidanceOutput result = output("", "");
    result.hasControl = false;
    return result;
}

ObstacleAvoidanceOutput ObstacleAvoidance::updateParabolaGapLostValidation(
    const Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line,
    const ForwardLineSnapshot& forwardLine)
{
    if (forwardLine.sourceFresh && forwardLine.parabolaSequence > 0 &&
        forwardLine.parabolaSequence != lastNearValidationSequence_)
    {
        lastNearValidationSequence_ = forwardLine.parabolaSequence;
        ++nearForwardLineSamples_;
        if (forwardLine.parabolaNearForwardVisible)
        {
            ++nearForwardLineVotes_;
        }
    }
    if (nearForwardLineSamples_ <
        config::kObstacleParabolaNearValidationFrames)
    {
        return output(
            "obstacle_parabola_gaplost_validate",
            "GAP/LOST validating CAM1 NEAR");
    }

    nearForwardLineVisible_ =
        nearForwardLineVotes_ >= config::kObstacleParabolaNearRequiredVotes;
    if (nearForwardLineVisible_ || !bestParabolaSideValid_)
    {
        phase_ = Phase::Idle;
        ObstacleAvoidanceOutput result = output(
            "obstacle_parabola_gaplost_validate",
            nearForwardLineVisible_
                ? "Forward continuation detected: normal GAP/LOST"
                : "No parabola evidence: normal GAP/LOST");
        result.completed = true;
        clearCase3Evidence();
        return result;
    }
    if (!ImuTurnController::imuReady(telemetry))
    {
        ObstacleAvoidanceOutput result = fail(
            "obstacle_parabola_reacquire_timeout",
            "Parabola reacquire timeout");
        clearCase3Evidence();
        return result;
    }

    phase_ = Phase::ParabolaReacquireForward;
    parabolaReacquireForwardStartLeftCount_ = telemetry.leftEncoderCount;
    parabolaReacquireForwardStartRightCount_ = telemetry.rightEncoderCount;
    parabolaReacquireForwardStartedAt_ = std::chrono::steady_clock::now();
    fusionReacquireFrames_ = 0;
    lastFusionLineSequence_ = line.lineSequence;
    return output(
        "obstacle_parabola_reacquire_forward",
        "False GAP/LOST confirmed");
}

ObstacleAvoidanceOutput ObstacleAvoidance::updateParabolaReacquireForward(
    const Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line)
{
    const double leftCounts = std::abs(static_cast<double>(
        telemetry.leftEncoderCount -
        parabolaReacquireForwardStartLeftCount_));
    const double rightCounts = std::abs(static_cast<double>(
        telemetry.rightEncoderCount -
        parabolaReacquireForwardStartRightCount_));
    const double leftDistanceCm =
        leftCounts / config::kEncoderCountsPerCentimeter;
    const double rightDistanceCm =
        rightCounts / config::kEncoderCountsPerCentimeter;
    const double usedDistanceCm = std::min(leftDistanceCm, rightDistanceCm);

    if (!telemetry.sensorFresh || telemetry.lastSensorAgeMs < 0 ||
        telemetry.lastSensorAgeMs > config::kObstacleEncoderFreshnessMs ||
        !std::isfinite(telemetry.leftEncoderRate) ||
        !std::isfinite(telemetry.rightEncoderRate))
    {
        ObstacleAvoidanceOutput result = fail(
            "obstacle_parabola_reacquire_forward_sensors_lost",
            "Recovery do caso 3 interrompido: encoders sem dados recentes");
        clearCase3Evidence();
        return result;
    }
    if (std::chrono::steady_clock::now() -
            parabolaReacquireForwardStartedAt_ >
        std::chrono::milliseconds(config::kObstacleDistanceSafetyTimeoutMs))
    {
        ObstacleAvoidanceOutput result = fail(
            "obstacle_parabola_reacquire_forward_timeout",
            "Recovery do caso 3 interrompido pelo tempo limite de segurança");
        clearCase3Evidence();
        return result;
    }

    // Usa a mesma compensação de frenagem do avanço curto do caso 2. A menor
    // distância das duas rodas impede concluir se apenas uma roda avançou.
    const double predictionSeconds =
        config::kObstacleBrakePredictionSeconds +
        telemetry.lastSensorAgeMs / 1000.0;
    const double projectedLeftCounts =
        leftCounts + std::abs(telemetry.leftEncoderRate) * predictionSeconds;
    const double projectedRightCounts =
        rightCounts + std::abs(telemetry.rightEncoderRate) * predictionSeconds;
    const double targetCounts =
        config::kObstacleParabolaReacquireForwardDistanceCm *
        config::kEncoderCountsPerCentimeter;
    if (std::min(projectedLeftCounts, projectedRightCounts) >= targetCounts)
    {
        if (!ImuTurnController::imuReady(telemetry))
        {
            ObstacleAvoidanceOutput result = fail(
                "obstacle_parabola_reacquire_search_sensors_lost",
                "Recovery do caso 3 interrompido: IMU indisponível para buscar a linha");
            clearCase3Evidence();
            return result;
        }
        phase_ = Phase::ParabolaReacquireSearch;
        // O limite angular começa somente após os 2 cm, imediatamente antes
        // do primeiro comando de pivot para o lado salvo pela CAM1.
        parabolaSearchStartYaw_ = telemetry.yawZDeg;
        parabolaSearchStartedAt_ = std::chrono::steady_clock::now();
        parabolaSearchOppositeSide_ = false;
        fusionReacquireFrames_ = 0;
        lastFusionLineSequence_ = line.lineSequence;
        ObstacleAvoidanceOutput result = output(
            "obstacle_parabola_reacquire_search",
            "Searching parabola exit " + bestParabolaSide_);
        result.progressPercent = 100.0;
        result.targetDistanceCm =
            config::kObstacleParabolaReacquireForwardDistanceCm;
        result.leftDistanceCm = leftDistanceCm;
        result.rightDistanceCm = rightDistanceCm;
        return result;
    }

    ObstacleAvoidanceOutput result = output(
        "obstacle_parabola_reacquire_forward",
        "Recovery clearance +2cm",
        config::kObstacleSelectedForwardPower,
        config::kObstacleSelectedForwardPower);
    result.progressPercent = std::clamp(
        usedDistanceCm /
            config::kObstacleParabolaReacquireForwardDistanceCm * 100.0,
        0.0,
        100.0);
    result.targetDistanceCm =
        config::kObstacleParabolaReacquireForwardDistanceCm;
    result.leftDistanceCm = leftDistanceCm;
    result.rightDistanceCm = rightDistanceCm;
    return result;
}

ObstacleAvoidanceOutput ObstacleAvoidance::updateParabolaReacquireSearch(
    const Esp32TelemetrySnapshot& telemetry,
    const CameraLineSnapshot& line)
{
    if (!ImuTurnController::imuReady(telemetry))
    {
        ObstacleAvoidanceOutput result = fail(
            "obstacle_parabola_reacquire_timeout",
            "Parabola reacquire timeout");
        clearCase3Evidence();
        return result;
    }
    const double yawChange = signedYawError(
        telemetry.yawZDeg, parabolaSearchStartYaw_);
    const double initialDirection =
        bestParabolaSide_ == "LEFT" ? -1.0 : 1.0;
    const double directedYawChange = yawChange * initialDirection;
    if (std::chrono::steady_clock::now() - parabolaSearchStartedAt_ >
            std::chrono::milliseconds(config::kObstacleTurnTimeoutMs))
    {
        ObstacleAvoidanceOutput result = fail(
            "obstacle_parabola_reacquire_timeout",
            "Parabola reacquire timeout");
        clearCase3Evidence();
        return result;
    }

    if (parabolaRearFusionRecovery_)
    {
        const double recoveryYawChange = yawChange * initialDirection;
        if (recoveryYawChange >=
            config::kObstacleReacquireSearchMaximumDegrees)
        {
            ObstacleAvoidanceOutput result = fail(
                "obstacle_parabola_reacquire_timeout",
                "Parabola rear-Fusion recovery timeout");
            clearCase3Evidence();
            return result;
        }

        const bool fusionValid =
            line.sourceFresh && line.lineControlSource == "fusion" &&
            line.normalSteeringValid;
        const bool fusionSupportsBestSide = fusionValid &&
            (bestParabolaSide_ == "LEFT"
                 ? line.lineFollowerLeftPower <= line.lineFollowerRightPower
                 : line.lineFollowerRightPower <= line.lineFollowerLeftPower);
        if (!fusionSupportsBestSide)
        {
            // Rejeita a faixa que causou o giro oposto. O Fusion volta a votar
            // assim que seu próprio steering concorda com o lado salvo pela CAM1.
            fusionReacquireFrames_ = 0;
            lastFusionLineSequence_ = line.lineSequence;
        }
        else if (observeFreshFusion(line))
        {
            phase_ = Phase::Idle;
            ObstacleAvoidanceOutput result = output(
                "obstacle_parabola_reacquired",
                "Parabola line reacquired");
            result.completed = true;
            result.progressPercent = 100.0;
            clearCase3Evidence();
            return result;
        }

        return output(
            "obstacle_parabola_reacquire_search",
            "Rear Fusion blocked; searching " + bestParabolaSide_,
            initialDirection * config::kObstacleTurnCommandPower,
            -initialDirection * config::kObstacleTurnCommandPower);
    }

    if (!parabolaSearchOppositeSide_ &&
        directedYawChange >= config::kObstacleParabolaRearBlockDegrees)
    {
        // A partir de 65 graus a câmera pode enxergar a faixa deixada para trás.
        // Esse frame é consumido sem votar e a busca recomeça no lado oposto.
        parabolaSearchOppositeSide_ = true;
        fusionReacquireFrames_ = 0;
        lastFusionLineSequence_ = line.lineSequence;
        const std::string oppositeSide =
            bestParabolaSide_ == "LEFT" ? "RIGHT" : "LEFT";
        return output(
            "obstacle_parabola_reacquire_opposite_search",
            "Rear line blocked; searching opposite " + oppositeSide);
    }

    if (parabolaSearchOppositeSide_ &&
        directedYawChange <= -config::kObstacleParabolaRearBlockDegrees)
    {
        ObstacleAvoidanceOutput result = fail(
            "obstacle_parabola_reacquire_timeout",
            "Parabola reacquire timeout");
        clearCase3Evidence();
        return result;
    }

    const bool fusionBlockedBySearchSector =
        directedYawChange >= config::kObstacleParabolaRearBlockDegrees ||
        (parabolaSearchOppositeSide_ && directedYawChange > 0.0);
    if (fusionBlockedBySearchSector)
    {
        // Depois da inversão, ignora o Fusion até cruzar o yaw inicial e entrar
        // realmente no lado oposto. Assim a faixa traseira não volta a votar.
        fusionReacquireFrames_ = 0;
        lastFusionLineSequence_ = line.lineSequence;
    }
    else if (observeFreshFusion(line))
    {
        phase_ = Phase::Idle;
        ObstacleAvoidanceOutput result = output(
            "obstacle_parabola_reacquired",
            "Parabola line reacquired");
        result.completed = true;
        result.progressPercent = 100.0;
        clearCase3Evidence();
        return result;
    }

    const double direction = parabolaSearchOppositeSide_
        ? -initialDirection
        : initialDirection;
    const std::string searchSide = parabolaSearchOppositeSide_
        ? (bestParabolaSide_ == "LEFT" ? "RIGHT" : "LEFT")
        : bestParabolaSide_;
    return output(
        parabolaSearchOppositeSide_
            ? "obstacle_parabola_reacquire_opposite_search"
            : "obstacle_parabola_reacquire_search",
        "Searching parabola exit " + searchSide,
        direction * config::kObstacleTurnCommandPower,
        -direction * config::kObstacleTurnCommandPower);
}

void ObstacleAvoidance::clearCase3Evidence()
{
    rawBestParabolaSide_ = "NONE";
    bestParabolaSide_ = "NONE";
    bestParabolaScore_ = 0;
    bestParabolaLeftBlack_ = 0;
    bestParabolaRightBlack_ = 0;
    bestParabolaSequence_ = 0;
    bestParabolaSideValid_ = false;
    lastParabolaSequence_ = 0;
    case3AwaitingFusionAcquire_ = false;
    case3Armed_ = false;
    case3FusionAcquireFrames_ = 0;
    case3GapLostFrames_ = 0;
    lastCase3LineSequence_ = 0;
    nearForwardLineVisible_ = false;
    nearForwardLineVotes_ = 0;
    nearForwardLineSamples_ = 0;
    lastNearValidationSequence_ = 0;
    parabolaReacquireForwardStartLeftCount_ = 0;
    parabolaReacquireForwardStartRightCount_ = 0;
    parabolaSearchStartYaw_ = std::numeric_limits<double>::quiet_NaN();
    parabolaSearchOppositeSide_ = false;
    case3PostObstacleYaw_ = std::numeric_limits<double>::quiet_NaN();
    parabolaRearFusionRecovery_ = false;
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
    result.cameraBlackLeft = cameraBlackLeft_;
    result.cameraBlackRight = cameraBlackRight_;
    result.cameraBlackLeftFrames = cameraBlackLeftFrames_;
    result.cameraBlackRightFrames = cameraBlackRightFrames_;
    result.selectedSideSource = selectedSideSource_;
    result.rawBestParabolaSide = rawBestParabolaSide_;
    result.bestParabolaSide = bestParabolaSide_;
    result.bestParabolaScore = bestParabolaScore_;
    result.bestParabolaLeftBlack = bestParabolaLeftBlack_;
    result.bestParabolaRightBlack = bestParabolaRightBlack_;
    result.bestParabolaSequence = bestParabolaSequence_;
    result.bestParabolaSideValid = bestParabolaSideValid_;
    result.nearForwardLineVisible = nearForwardLineVisible_;
    result.nearForwardLineVotes = nearForwardLineVotes_;
    result.nearForwardLineSamples = nearForwardLineSamples_;
    result.case3Armed = case3Armed_;
    if (case3Armed_)
    {
        result.case3FusionAcquireTime =
            std::chrono::duration<double>(
                case3FusionAcquireTime_.time_since_epoch()).count();
        result.case3TimeRemainingMs = std::max<long long>(
            0,
            config::kObstacleCase3FusionWindowMs -
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() -
                    case3FusionAcquireTime_).count());
    }
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

double ObstacleAvoidance::maximumClearance(const std::vector<double>& samples)
{
    return *std::max_element(samples.begin(), samples.end());
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
