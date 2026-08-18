#include "obr/main_mission.h"

#include "obr/config.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

namespace
{
// Valores experimentais usados somente durante a recuperação pela FAR.
constexpr double kFarBaseSpeed = 0.65;
constexpr double kFarDeadzone = 0.10;
constexpr double kFarProportionalGain = 0.30;
constexpr double kFarMaximumCorrection = 0.15;
constexpr double kFarExtremeError = 0.75;

// Um erro deste tamanho registra uma direção útil sem apagar a memória no centro.
constexpr double kSignificantDirectionError = 0.20;
constexpr double kNearReacquireMaxAbsError = 0.40;

// A histerese evita alternar entre tracking e pivô perto do mesmo limiar.
// Somente sequências novas da câmera avançam as confirmações de entrada e saída.
constexpr double kAheadStrongTurnEnterError = 0.25;
constexpr double kStrongSteeringEnterError = 0.40;
constexpr double kStrongSteeringExitError = 0.12;
constexpr double kStrongSteeringDirectionMinimum = 0.05;
constexpr int kAheadStrongTurnEnterSamples = 2;
constexpr int kAheadStrongTurnExitSamples = 3;
// Potência exclusiva da contrarrotação antecipada. As recuperações
// permanecem limitadas pela base de 0,65 definida acima.
constexpr double kAheadStrongTurnPower = 0.65;

// Os limites impedem que o robô procure indefinidamente por uma linha perdida.
constexpr auto kNearRecoveryTimeout = std::chrono::milliseconds(3000);
constexpr auto kTotalLossTimeout = std::chrono::milliseconds(1300);
constexpr int kNearSamplesToConfirmRecovery = 3;

struct MotorCommand
{
    double left = 0.0;
    double right = 0.0;
};

AutonomousStatus makeLineStatus(
    const std::string& phase,
    const std::string& action)
{
    AutonomousStatus status;
    status.phase = phase;
    status.action = action;
    return status;
}

MotorCommand calculateOneWheelPivotCommand(double error)
{
    return error < 0.0
               ? MotorCommand{0.0, kFarBaseSpeed}
               : MotorCommand{kFarBaseSpeed, 0.0};
}

MotorCommand calculateCounterRotationCommand(bool turnLeft, double power)
{
    return turnLeft
               ? MotorCommand{-power, power}
               : MotorCommand{power, -power};
}

MotorCommand calculateNearReacquisitionCommand(double correction)
{
    // A reaquisição preserva a base validada de 0,65 mesmo quando o perfil
    // inferior publica uma prévia mais rápida para o tracking normal.
    const double safeCorrection = std::clamp(
        correction, -kFarMaximumCorrection, kFarMaximumCorrection);
    if (safeCorrection > 0.0)
    {
        return {kFarBaseSpeed + safeCorrection, kFarBaseSpeed};
    }
    if (safeCorrection < 0.0)
    {
        return {kFarBaseSpeed, kFarBaseSpeed + std::abs(safeCorrection)};
    }
    return {kFarBaseSpeed, kFarBaseSpeed};
}

MotorCommand calculateFarRecoveryCommand(double farError)
{
    const double safeFarError = std::clamp(farError, -1.0, 1.0);
    const double errorMagnitude = std::abs(safeFarError);

    if (errorMagnitude >= kFarExtremeError)
    {
        return calculateOneWheelPivotCommand(safeFarError);
    }

    if (errorMagnitude <= kFarDeadzone)
    {
        return {kFarBaseSpeed, kFarBaseSpeed};
    }

    const double normalizedMagnitude =
        (errorMagnitude - kFarDeadzone) / (1.0 - kFarDeadzone);
    const double correction = std::min(
        kFarMaximumCorrection,
        kFarProportionalGain * normalizedMagnitude);

    if (safeFarError < 0.0)
    {
        return {kFarBaseSpeed, kFarBaseSpeed + correction};
    }
    return {kFarBaseSpeed + correction, kFarBaseSpeed};
}
}

void MainMission::reset()
{
    state_ = LineFollowState::TrackingNear;
    greenTurnController_.reset();
    greenDecisionLatched_ = false;
    lastSignificantDirection_ = LineDirection::Unknown;
    searchDirection_ = LineDirection::Unknown;
    lastValidError_ = 0.0;
    lastProcessedLineSequence_ = 0;
    hasProcessedLineSequence_ = false;
    consecutiveNearValidSamples_ = 0;
    aheadStrongTurnActive_ = false;
    aheadStrongTurnDirection_ = LineDirection::Unknown;
    aheadStrongTurnEnterSamples_ = 0;
    aheadStrongTurnExitSamples_ = 0;
    aheadStrongTurnLastLineSequence_ = 0;
    aheadStrongTurnHasLineSequence_ = false;
    aheadStrongTurnLastHeadingError_ = 0.0;
    nearRecoveryActive_ = false;
    totalLossActive_ = false;
    nearLostAt_ = {};
    totalLossStartedAt_ = {};
    resetGapTracking();
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

    if (!esp32Telemetry.readyForOperation() || !cameraReady ||
        !cameraLineSnapshot.sourceFresh)
    {
        // Uma fonte obrigatória indisponível encerra a execução e zera os motores.
        robotState.stop();
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const bool newLineSample =
        !hasProcessedLineSequence_ ||
        cameraLineSnapshot.lineSequence != lastProcessedLineSequence_;
    if (newLineSample)
    {
        hasProcessedLineSequence_ = true;
        lastProcessedLineSequence_ = cameraLineSnapshot.lineSequence;

        if (cameraLineSnapshot.nearValid)
        {
            updateDirectionMemory(cameraLineSnapshot.nearError);
        }
        else if (cameraLineSnapshot.farValid)
        {
            updateDirectionMemory(cameraLineSnapshot.farError);
        }
    }

    // O verde duplo tem prioridade sobre seguimento, curva antecipada e gap.
    // A manobra usa o IMU e nunca comanda GPIO diretamente.
    if (updateGreenTurn(
            robotState,
            esp32Telemetry,
            cameraLineSnapshot,
            newLineSample))
    {
        return;
    }

    const bool greenApproachActive =
        cameraLineSnapshot.greenPathBlackValid &&
        cameraLineSnapshot.greenTurnDecision == GreenTurnDecision::Approach;
    const bool greenGuidanceActive =
        cameraLineSnapshot.greenConfirmed &&
        (cameraLineSnapshot.greenTurnDecision ==
             GreenTurnDecision::GuideLeft ||
         cameraLineSnapshot.greenTurnDecision ==
             GreenTurnDecision::GuideRight);
    if (cameraLineSnapshot.nearValid &&
        (greenApproachActive || greenGuidanceActive))
    {
        // Somente um verde reconhecido pode substituir a prévia normal. Sem
        // verde, o segue-linha conserva exatamente o comportamento anterior.
        aheadStrongTurnActive_ = false;
        aheadStrongTurnDirection_ = LineDirection::Unknown;
        aheadStrongTurnEnterSamples_ = 0;
        aheadStrongTurnExitSamples_ = 0;
        nearRecoveryActive_ = false;
        totalLossActive_ = false;
        transitionTo(LineFollowState::TrackingNear);
        robotState.driveAutonomous(
            cameraLineSnapshot.leftPreview,
            cameraLineSnapshot.rightPreview);
        robotState.updateAutonomousStatus(makeLineStatus(
            greenApproachActive ? "green_approach" : "green_guidance",
            greenApproachActive
                ? "Aproximando do marcador verde com velocidade reduzida"
                : "Seguindo o alvo deslocado pelo marcador verde"));
        return;
    }

    const bool gapStateActive = state_ == LineFollowState::CrossingGap;
    if (!gapStateActive &&
        state_ == LineFollowState::TrackingNear &&
        newLineSample && cameraLineSnapshot.gapCandidate &&
        !aheadStrongTurnActive_ && !nearRecoveryActive_ && !totalLossActive_ &&
        std::abs(cameraLineSnapshot.controlError) <
            kAheadStrongTurnEnterError &&
        std::abs(cameraLineSnapshot.nearError) < kStrongSteeringEnterError)
    {
        // O GAP é a última prioridade de percurso: curva e recuperação sempre
        // vencem. A primeira detecção inicia o avanço reto sem realinhamento.
        aheadStrongTurnActive_ = false;
        aheadStrongTurnDirection_ = LineDirection::Unknown;
        aheadStrongTurnEnterSamples_ = 0;
        aheadStrongTurnExitSamples_ = 0;
        nearRecoveryActive_ = false;
        totalLossActive_ = false;
        gapNearLossObserved_ = false;
        transitionTo(LineFollowState::CrossingGap);
        robotState.driveAutonomous(
            config::kGapDriveCommandPower,
            config::kGapDriveCommandPower);
        robotState.updateAutonomousStatus(makeLineStatus(
            "crossing_gap",
            "Gap detectado: seguindo reto até reencontrar a linha"));
        return;
    }

    if (state_ == LineFollowState::CrossingGap)
    {
        if (newLineSample && !gapNearLossObserved_ &&
            !cameraLineSnapshot.nearValid)
        {
            // A perda da NEAR separa a fita anterior de uma linha reencontrada.
            // O GAP não usa distância, tempo nem telemetria dos encoders.
            gapNearLossObserved_ = true;
        }

        if (newLineSample && gapNearLossObserved_ &&
            cameraLineSnapshot.nearValid)
        {
            resetGapTracking();
            nearRecoveryActive_ = true;
            transitionTo(LineFollowState::ReacquiringNear);
            const MotorCommand command =
                std::abs(cameraLineSnapshot.nearError) >
                        kNearReacquireMaxAbsError
                    ? calculateOneWheelPivotCommand(
                          cameraLineSnapshot.nearError)
                    : calculateNearReacquisitionCommand(
                          cameraLineSnapshot.correction);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeLineStatus(
                "reacquiring_near",
                "Linha reencontrada após o gap"));
            return;
        }
        if (newLineSample && cameraLineSnapshot.gapReturnValid)
        {
            const double returnError = cameraLineSnapshot.gapReturnError;
            resetGapTracking();
            nearRecoveryActive_ = true;
            nearLostAt_ = now;
            transitionTo(LineFollowState::RecoveringFar);
            const MotorCommand command =
                calculateFarRecoveryCommand(returnError);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeLineStatus(
                "recovering_far",
                "Continuação do gap encontrada pela visão"));
            return;
        }
        if (newLineSample && gapNearLossObserved_ &&
            cameraLineSnapshot.farValid)
        {
            resetGapTracking();
            nearRecoveryActive_ = true;
            nearLostAt_ = now;
            transitionTo(LineFollowState::RecoveringFar);
            const MotorCommand command =
                calculateFarRecoveryCommand(cameraLineSnapshot.farError);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeLineStatus(
                "recovering_far",
                "Linha reencontrada pela FAR após o gap"));
            return;
        }

        robotState.driveAutonomous(
            config::kGapDriveCommandPower,
            config::kGapDriveCommandPower);
        robotState.updateAutonomousStatus(makeLineStatus(
            "crossing_gap",
            "Atravessando gap até reencontrar a linha"));
        return;
    }

    // Este detector possui sequência própria para nunca contar o mesmo frame
    // novamente quando o loop C++ roda mais rápido que o processo de visão.
    const bool newAheadStrongTurnSample =
        !aheadStrongTurnHasLineSequence_ ||
        cameraLineSnapshot.lineSequence != aheadStrongTurnLastLineSequence_;
    if (newAheadStrongTurnSample)
    {
        aheadStrongTurnHasLineSequence_ = true;
        aheadStrongTurnLastLineSequence_ = cameraLineSnapshot.lineSequence;
    }

    const auto logAheadStrongTurnEvent =
        [&](const char* event,
            const char* exitReason,
            double headingError,
            const MotorCommand& command)
    {
        std::cout << "MainMission ahead strong turn " << event << ":"
                  << " lineSequence=" << cameraLineSnapshot.lineSequence
                  << " nearError=" << cameraLineSnapshot.nearError
                  << " farError=" << cameraLineSnapshot.farError
                  << " headingError=" << headingError
                  << " controlError=" << cameraLineSnapshot.controlError
                  << " direction="
                  << (aheadStrongTurnDirection_ == LineDirection::Left
                          ? "left"
                          : "right")
                  << " leftCommand=" << command.left
                  << " rightCommand=" << command.right
                  << " exitReason=" << exitReason
                  << std::endl;
    };

    const auto exitAheadStrongTurn =
        [&](const char* exitReason,
            double headingError,
            bool returnToTracking)
    {
        const MotorCommand command =
            calculateCounterRotationCommand(
                aheadStrongTurnDirection_ == LineDirection::Left,
                kAheadStrongTurnPower);
        logAheadStrongTurnEvent(
            "exited", exitReason, headingError, command);
        aheadStrongTurnActive_ = false;
        aheadStrongTurnDirection_ = LineDirection::Unknown;
        aheadStrongTurnEnterSamples_ = 0;
        aheadStrongTurnExitSamples_ = 0;
        if (returnToTracking)
        {
            transitionTo(LineFollowState::TrackingNear);
        }
    };

    const bool aheadStrongTurnBandsValid =
        cameraLineSnapshot.nearValid && cameraLineSnapshot.farValid;
    if (aheadStrongTurnBandsValid)
    {
        const double headingError =
            cameraLineSnapshot.farError - cameraLineSnapshot.nearError;
        const double steeringError = cameraLineSnapshot.controlError;
        aheadStrongTurnLastHeadingError_ = headingError;

        if (aheadStrongTurnActive_)
        {
            if (newAheadStrongTurnSample)
            {
                const bool steeringSignCrossed =
                    std::abs(steeringError) > kStrongSteeringExitError &&
                    ((aheadStrongTurnDirection_ == LineDirection::Left &&
                      steeringError > 0.0) ||
                     (aheadStrongTurnDirection_ == LineDirection::Right &&
                      steeringError < 0.0));
                if (steeringSignCrossed)
                {
                    exitAheadStrongTurn(
                        "steering_sign_crossed", headingError, true);
                }
                else if (std::abs(steeringError) <= kStrongSteeringExitError)
                {
                    ++aheadStrongTurnExitSamples_;
                }
                else
                {
                    aheadStrongTurnExitSamples_ = 0;
                }
            }

            if (aheadStrongTurnActive_ &&
                aheadStrongTurnExitSamples_ >= kAheadStrongTurnExitSamples)
            {
                exitAheadStrongTurn("control_aligned", headingError, true);
            }

            if (aheadStrongTurnActive_)
            {
                const MotorCommand command =
                    calculateCounterRotationCommand(
                        aheadStrongTurnDirection_ == LineDirection::Left,
                        kAheadStrongTurnPower);
                transitionTo(LineFollowState::TurningAhead);
                robotState.driveAutonomous(command.left, command.right);
                robotState.updateAutonomousStatus(makeLineStatus(
                    "turning_ahead", "Curva forte antecipada pela AHEAD"));
                return;
            }
        }
        else if (newAheadStrongTurnSample)
        {
            const bool strongHeadingDemand =
                std::abs(headingError) >= kAheadStrongTurnEnterError;
            const bool strongSteeringDemand =
                std::abs(steeringError) >= kStrongSteeringEnterError;
            const bool directionIsUsable =
                std::abs(steeringError) >= kStrongSteeringDirectionMinimum;
            const bool shouldEnterStrongTurn =
                directionIsUsable &&
                (strongHeadingDemand || strongSteeringDemand);
            if (shouldEnterStrongTurn)
            {
                const LineDirection sampleDirection =
                    steeringError < 0.0
                        ? LineDirection::Left
                        : LineDirection::Right;
                if (sampleDirection == aheadStrongTurnDirection_)
                {
                    ++aheadStrongTurnEnterSamples_;
                }
                else
                {
                    aheadStrongTurnDirection_ = sampleDirection;
                    aheadStrongTurnEnterSamples_ = 1;
                }
            }
            else
            {
                aheadStrongTurnEnterSamples_ = 0;
                aheadStrongTurnDirection_ = LineDirection::Unknown;
            }

            if (aheadStrongTurnEnterSamples_ >= kAheadStrongTurnEnterSamples)
            {
                aheadStrongTurnActive_ = true;
                aheadStrongTurnExitSamples_ = 0;
                const MotorCommand command =
                    calculateCounterRotationCommand(
                        aheadStrongTurnDirection_ == LineDirection::Left,
                        kAheadStrongTurnPower);
                logAheadStrongTurnEvent(
                    "entered", "none", headingError, command);

                transitionTo(LineFollowState::TurningAhead);
                robotState.driveAutonomous(command.left, command.right);
                robotState.updateAutonomousStatus(makeLineStatus(
                    "turning_ahead", "Curva forte antecipada pela AHEAD"));
                return;
            }
        }
    }
    else
    {
        if (aheadStrongTurnActive_)
        {
            const char* exitReason = cameraLineSnapshot.nearValid
                                         ? "far_lost"
                                         : "near_lost";
            exitAheadStrongTurn(
                exitReason,
                aheadStrongTurnLastHeadingError_,
                cameraLineSnapshot.nearValid);
        }
        else
        {
            aheadStrongTurnEnterSamples_ = 0;
            aheadStrongTurnExitSamples_ = 0;
            aheadStrongTurnDirection_ = LineDirection::Unknown;
        }
    }

    if (cameraLineSnapshot.nearValid)
    {
        const bool requiresNearReacquisition =
            nearRecoveryActive_ || state_ != LineFollowState::TrackingNear;
        nearRecoveryActive_ = false;
        nearLostAt_ = {};
        totalLossActive_ = false;
        totalLossStartedAt_ = {};

        if (!requiresNearReacquisition)
        {
            consecutiveNearValidSamples_ = 0;
            transitionTo(LineFollowState::TrackingNear);
            robotState.driveAutonomous(
                cameraLineSnapshot.leftPreview,
                cameraLineSnapshot.rightPreview);
            robotState.updateAutonomousStatus(makeLineStatus(
                "tracking_near", "Seguindo pela NEAR"));
            return;
        }

        transitionTo(LineFollowState::ReacquiringNear);
        if (std::abs(cameraLineSnapshot.nearError) >
            kNearReacquireMaxAbsError)
        {
            consecutiveNearValidSamples_ = 0;

            // Usa o mesmo pivô controlado de um lado empregado pela FAR extrema.
            // Erro negativo aponta para a esquerda; erro positivo, para a direita.
            const MotorCommand command =
                calculateOneWheelPivotCommand(cameraLineSnapshot.nearError);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeLineStatus(
                "reacquiring_near", "Readquirindo NEAR: alinhando ao centro"));
            return;
        }

        if (newLineSample &&
            consecutiveNearValidSamples_ < kNearSamplesToConfirmRecovery)
        {
            ++consecutiveNearValidSamples_;
        }

        const MotorCommand command =
            calculateNearReacquisitionCommand(cameraLineSnapshot.correction);
        robotState.driveAutonomous(command.left, command.right);
        robotState.updateAutonomousStatus(makeLineStatus(
            "reacquiring_near",
            "Readquirindo NEAR " +
                std::to_string(consecutiveNearValidSamples_) + "/3"));

        if (consecutiveNearValidSamples_ >= kNearSamplesToConfirmRecovery)
        {
            totalLossActive_ = false;
            consecutiveNearValidSamples_ = 0;
            searchDirection_ = LineDirection::Unknown;
            totalLossStartedAt_ = {};
            transitionTo(LineFollowState::TrackingNear);
        }
        return;
    }

    consecutiveNearValidSamples_ = 0;
    if (!nearRecoveryActive_)
    {
        // Este tempo mede somente a ausência contínua da NEAR. Qualquer nova
        // amostra NEAR válida encerra esta contagem antes da reaquisição.
        nearRecoveryActive_ = true;
        nearLostAt_ = now;
        searchDirection_ = LineDirection::Unknown;
    }
    if (now - nearLostAt_ >= kNearRecoveryTimeout &&
        !cameraLineSnapshot.farValid)
    {
        // A parada é terminal: somente uma nova partida poderá mover o robô.
        robotState.stop();
        std::cout << "MainMission stopped: NEAR recovery timeout ("
                  << kNearRecoveryTimeout.count() << " ms)"
                  << " nearValid=" << std::boolalpha
                  << cameraLineSnapshot.nearValid
                  << " farValid=" << cameraLineSnapshot.farValid
                  << std::noboolalpha
                  << " nearError=" << cameraLineSnapshot.nearError
                  << " state=" << stateName(state_) << std::endl;
        return;
    }

    if (cameraLineSnapshot.farValid)
    {
        totalLossActive_ = false;
        totalLossStartedAt_ = {};
        transitionTo(LineFollowState::RecoveringFar);

        const MotorCommand command =
            calculateFarRecoveryCommand(cameraLineSnapshot.farError);
        robotState.driveAutonomous(command.left, command.right);
        robotState.updateAutonomousStatus(makeLineStatus(
            "recovering_far", "Recuperando pela FAR"));
        return;
    }

    if (!totalLossActive_)
    {
        totalLossActive_ = true;
        totalLossStartedAt_ = now;
    }
    if (now - totalLossStartedAt_ >= kTotalLossTimeout)
    {
        // Sem qualquer linha visível, o giro também possui limite independente.
        robotState.stop();
        std::cout << "MainMission stopped: total line loss timeout ("
                  << kTotalLossTimeout.count() << " ms)" << std::endl;
        return;
    }

    const LineDirection direction = chooseSearchDirection();
    if (direction == LineDirection::Right)
    {
        transitionTo(LineFollowState::SearchingRight);
        const MotorCommand command =
            calculateCounterRotationCommand(false, kFarBaseSpeed);
        robotState.driveAutonomous(command.left, command.right);
        robotState.updateAutonomousStatus(makeLineStatus(
            "searching_right", "Procurando linha à direita"));
        return;
    }

    transitionTo(LineFollowState::SearchingLeft);
    const MotorCommand command =
        calculateCounterRotationCommand(true, kFarBaseSpeed);
    robotState.driveAutonomous(command.left, command.right);
    robotState.updateAutonomousStatus(makeLineStatus(
        "searching_left", "Procurando linha à esquerda"));
}

void MainMission::transitionTo(LineFollowState nextState)
{
    if (state_ == nextState)
    {
        return;
    }

    state_ = nextState;
    std::cout << "MainMission state: " << stateName(state_);
    if (state_ == LineFollowState::ReacquiringNear ||
        state_ == LineFollowState::TrackingNear)
    {
        std::cout << " nearError=" << lastValidError_;
    }
    std::cout << std::endl;
}

bool MainMission::updateGreenTurn(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    const CameraLineSnapshot& cameraLineSnapshot,
    bool newLineSample)
{
    if (state_ == LineFollowState::TurningAtGreenMarker)
    {
        const ImuTurnOutput output = greenTurnController_.update(esp32Telemetry);
        robotState.driveAutonomous(output.leftPower, output.rightPower);

        AutonomousStatus status;
        status.phase = output.phase == "turning"
                           ? "green_turning"
                           : output.phase;
        status.action = output.action;
        status.progressPercent = output.progressPercent;
        if (output.result == ImuTurnResult::Completed)
        {
            // Depois do retorno, a linha pode estar fora da NEAR. A recuperação
            // existente reassume o controle sem usar tempos fixos de giro.
            nearRecoveryActive_ = true;
            nearLostAt_ = std::chrono::steady_clock::now();
            totalLossActive_ = false;
            totalLossStartedAt_ = {};
            searchDirection_ = LineDirection::Unknown;
            transitionTo(LineFollowState::ReacquiringNear);
            robotState.updateAutonomousStatus(makeLineStatus(
                "green_turn_completed",
                "Retorno verde concluído: procurando a linha"));
            return true;
        }
        if (output.result == ImuTurnResult::Failed)
        {
            // Falha de IMU ou tempo limite encerra a missão com motores zerados.
            robotState.stop();
            robotState.updateAutonomousStatus(status);
            return true;
        }
        robotState.updateAutonomousStatus(status);
        return true;
    }

    if (newLineSample && !cameraLineSnapshot.greenNearSeen &&
        !cameraLineSnapshot.greenConfirmed)
    {
        // Só libera outro retorno depois que o marcador anterior sair por
        // completo da cena, evitando repetir o giro sobre o mesmo verde duplo.
        greenDecisionLatched_ = false;
    }
    if (greenDecisionLatched_)
    {
        return false;
    }

    const bool confirmedTurnAround =
        cameraLineSnapshot.greenConfirmed &&
        cameraLineSnapshot.greenPathBlackValid &&
        cameraLineSnapshot.greenTurnDecision ==
            GreenTurnDecision::TurnAround180;
    if (!confirmedTurnAround)
    {
        // APPROACH, LEFT e RIGHT já estão incorporados às prévias publicadas
        // pela visão. Eles não iniciam uma manobra paralela neste módulo.
        return false;
    }

    if (!ImuTurnController::imuReady(esp32Telemetry))
    {
        // Sem referência angular recente, o robô espera parado sobre o marcador.
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeLineStatus(
            "green_turn_waiting_imu",
            "Aguardando IMU para retorno de 180° à direita"));
        return true;
    }

    greenDecisionLatched_ = true;
    aheadStrongTurnActive_ = false;
    nearRecoveryActive_ = false;
    totalLossActive_ = false;
    resetGapTracking();

    if (!greenTurnController_.start(
            config::kGreenTurnAroundTargetDegrees,
            ImuTurnDirection::Right,
            esp32Telemetry))
    {
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeLineStatus(
            "green_turn_waiting_imu",
            "Aguardando IMU para retorno de 180° à direita"));
        return true;
    }

    transitionTo(LineFollowState::TurningAtGreenMarker);

    const ImuTurnOutput output = greenTurnController_.update(esp32Telemetry);
    robotState.driveAutonomous(output.leftPower, output.rightPower);
    AutonomousStatus status;
    status.phase = "green_turning";
    status.action = output.action;
    status.progressPercent = output.progressPercent;
    robotState.updateAutonomousStatus(status);
    return true;
}

void MainMission::resetGapTracking()
{
    gapNearLossObserved_ = false;
}

void MainMission::updateDirectionMemory(double error)
{
    lastValidError_ = error;
    if (std::abs(error) < kSignificantDirectionError)
    {
        return;
    }

    lastSignificantDirection_ =
        error < 0.0 ? LineDirection::Left : LineDirection::Right;
}

MainMission::LineDirection MainMission::chooseSearchDirection()
{
    if (searchDirection_ != LineDirection::Unknown)
    {
        return searchDirection_;
    }

    if (lastSignificantDirection_ != LineDirection::Unknown)
    {
        searchDirection_ = lastSignificantDirection_;
    }
    else if (lastValidError_ > 0.0)
    {
        searchDirection_ = LineDirection::Right;
    }
    else
    {
        // Erro negativo procura à esquerda; zero também escolhe esquerda
        // deterministicamente para impedir alternância durante a tentativa.
        searchDirection_ = LineDirection::Left;
    }
    return searchDirection_;
}

const char* MainMission::stateName(LineFollowState state)
{
    switch (state)
    {
    case LineFollowState::TrackingNear:
        return "TrackingNear";
    case LineFollowState::TurningAtGreenMarker:
        return "TurningAtGreenMarker";
    case LineFollowState::TurningAhead:
        return "TurningAhead";
    case LineFollowState::CrossingGap:
        return "CrossingGap";
    case LineFollowState::ReacquiringNear:
        return "ReacquiringNear";
    case LineFollowState::RecoveringFar:
        return "RecoveringFar";
    case LineFollowState::SearchingLeft:
        return "SearchingLeft";
    case LineFollowState::SearchingRight:
        return "SearchingRight";
    }
    return "Unknown";
}
