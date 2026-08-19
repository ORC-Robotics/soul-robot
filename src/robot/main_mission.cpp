#include "obr/main_mission.h"

#include "obr/config.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

namespace
{
// Valores usados no alinhamento seguro ao reencontrar a linha.
constexpr double kLineRecoveryBaseSpeed =
    config::kOperationalMinimumMotorPower;
constexpr double kLineRecoveryDeadzone = 0.10;
constexpr double kLineRecoveryProportionalGain = 0.30;
constexpr double kLineRecoveryMaximumCorrection = 0.15;
constexpr double kLineRecoveryExtremeError = 0.75;

// Um erro deste tamanho registra uma direção útil sem apagar a memória no centro.
constexpr double kSignificantDirectionError = 0.20;
constexpr double kNearReacquireMaxAbsError = 0.40;

// A histerese evita alternar entre tracking e pivô perto do mesmo limiar.
// Somente sequências novas da câmera avançam as confirmações de entrada e saída.
constexpr double kStrongSteeringEnterError = 0.40;
constexpr double kStrongSteeringExitError = 0.12;
constexpr double kStrongSteeringDirectionMinimum = 0.05;
constexpr int kStrongTurnEnterSamples = 2;
constexpr int kStrongTurnExitSamples = 3;
// Potência exclusiva da contrarrotação antecipada. As recuperações
// permanecem limitadas pelo piso mecânico de 0,69 definido em config.h.
constexpr double kStrongTurnPower = config::kOperationalMinimumMotorPower;

// Os limites impedem que o robô procure indefinidamente por uma linha perdida.
constexpr auto kTotalLossTimeout = std::chrono::milliseconds(1300);
// O robô nunca pode continuar avançando indefinidamente sem reencontrar a linha.
constexpr auto kGapTimeout = std::chrono::milliseconds(2000);
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
               ? MotorCommand{0.0, kLineRecoveryBaseSpeed}
               : MotorCommand{kLineRecoveryBaseSpeed, 0.0};
}

MotorCommand calculateCounterRotationCommand(bool turnLeft, double power)
{
    return turnLeft
               ? MotorCommand{-power, power}
               : MotorCommand{power, -power};
}

MotorCommand calculateNearReacquisitionCommand(double correction)
{
    // A reaquisição preserva a base validada de 0,69 mesmo quando o perfil
    // inferior publica uma prévia mais rápida para o tracking normal.
    const double safeCorrection = std::clamp(
        correction,
        -kLineRecoveryMaximumCorrection,
        kLineRecoveryMaximumCorrection);
    if (safeCorrection > 0.0)
    {
        return {kLineRecoveryBaseSpeed + safeCorrection,
                kLineRecoveryBaseSpeed};
    }
    if (safeCorrection < 0.0)
    {
        return {kLineRecoveryBaseSpeed,
                kLineRecoveryBaseSpeed + std::abs(safeCorrection)};
    }
    return {kLineRecoveryBaseSpeed, kLineRecoveryBaseSpeed};
}

MotorCommand calculateLineRecoveryCommand(double lineError)
{
    const double safeLineError = std::clamp(lineError, -1.0, 1.0);
    const double errorMagnitude = std::abs(safeLineError);

    if (errorMagnitude >= kLineRecoveryExtremeError)
    {
        return calculateOneWheelPivotCommand(safeLineError);
    }

    if (errorMagnitude <= kLineRecoveryDeadzone)
    {
        return {kLineRecoveryBaseSpeed, kLineRecoveryBaseSpeed};
    }

    const double normalizedMagnitude =
        (errorMagnitude - kLineRecoveryDeadzone) /
        (1.0 - kLineRecoveryDeadzone);
    const double correction = std::min(
        kLineRecoveryMaximumCorrection,
        kLineRecoveryProportionalGain * normalizedMagnitude);

    if (safeLineError < 0.0)
    {
        return {kLineRecoveryBaseSpeed,
                kLineRecoveryBaseSpeed + correction};
    }
    return {kLineRecoveryBaseSpeed + correction,
            kLineRecoveryBaseSpeed};
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
    strongTurnActive_ = false;
    strongTurnDirection_ = LineDirection::Unknown;
    strongTurnEnterSamples_ = 0;
    strongTurnExitSamples_ = 0;
    strongTurnLastLineSequence_ = 0;
    strongTurnHasLineSequence_ = false;
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
        strongTurnActive_ = false;
        strongTurnDirection_ = LineDirection::Unknown;
        strongTurnEnterSamples_ = 0;
        strongTurnExitSamples_ = 0;
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
    const bool strongTurnDemand =
        cameraLineSnapshot.nearValid &&
        std::abs(cameraLineSnapshot.controlError) >=
            kStrongSteeringEnterError;
    if (!gapStateActive &&
        state_ == LineFollowState::TrackingNear &&
        newLineSample && cameraLineSnapshot.gapCandidate &&
        !strongTurnActive_ && !strongTurnDemand &&
        !nearRecoveryActive_ && !totalLossActive_)
    {
        // O gap tem prioridade menor que uma curva forte já visível. A
        // detecção pode chegar com a NEAR vazia, pois usa a faixa inferior.
        strongTurnActive_ = false;
        strongTurnDirection_ = LineDirection::Unknown;
        strongTurnEnterSamples_ = 0;
        strongTurnExitSamples_ = 0;
        nearRecoveryActive_ = false;
        totalLossActive_ = false;
        gapNearLossObserved_ = !cameraLineSnapshot.nearValid;
        gapStartedAt_ = now;
        transitionTo(LineFollowState::CrossingGap);
    }

    if (state_ == LineFollowState::CrossingGap)
    {
        if (gapStartedAt_ != std::chrono::steady_clock::time_point{} &&
            now - gapStartedAt_ >= kGapTimeout)
        {
            // Sem linha ou continuação confirmada, avançar além deste limite
            // seria inseguro. A parada é terminal para esta execução.
            robotState.stop();
            std::cout << "MainMission stopped: gap timeout ("
                      << kGapTimeout.count() << " ms)" << std::endl;
            resetGapTracking();
            return;
        }

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
            transitionTo(LineFollowState::ReacquiringNear);
            const MotorCommand command =
                calculateLineRecoveryCommand(returnError);
            robotState.driveAutonomous(command.left, command.right);
            robotState.updateAutonomousStatus(makeLineStatus(
                "reacquiring_near",
                "Continuação do gap encontrada pela visão"));
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

    // Este detector conta somente frames novos para manter a histerese
    // independente da frequência do loop principal.
    const bool newStrongTurnSample =
        !strongTurnHasLineSequence_ ||
        cameraLineSnapshot.lineSequence != strongTurnLastLineSequence_;
    if (newStrongTurnSample)
    {
        strongTurnHasLineSequence_ = true;
        strongTurnLastLineSequence_ = cameraLineSnapshot.lineSequence;
    }

    const auto logStrongTurnEvent =
        [&](const char* event,
            const char* exitReason,
            const MotorCommand& command)
    {
        std::cout << "MainMission NEAR strong turn " << event << ":"
                  << " lineSequence=" << cameraLineSnapshot.lineSequence
                  << " nearError=" << cameraLineSnapshot.nearError
                  << " controlError=" << cameraLineSnapshot.controlError
                  << " direction="
                  << (strongTurnDirection_ == LineDirection::Left
                          ? "left"
                          : "right")
                  << " leftCommand=" << command.left
                  << " rightCommand=" << command.right
                  << " exitReason=" << exitReason
                  << std::endl;
    };

    const auto exitStrongTurn = [&](const char* exitReason,
                                    bool returnToTracking)
    {
        const MotorCommand command = calculateCounterRotationCommand(
            strongTurnDirection_ == LineDirection::Left,
            kStrongTurnPower);
        logStrongTurnEvent("exited", exitReason, command);
        strongTurnActive_ = false;
        strongTurnDirection_ = LineDirection::Unknown;
        strongTurnEnterSamples_ = 0;
        strongTurnExitSamples_ = 0;
        if (returnToTracking)
        {
            transitionTo(LineFollowState::TrackingNear);
        }
    };

    if (cameraLineSnapshot.nearValid)
    {
        const double steeringError = cameraLineSnapshot.controlError;
        if (strongTurnActive_)
        {
            if (newStrongTurnSample)
            {
                const bool steeringSignCrossed =
                    ((strongTurnDirection_ == LineDirection::Left &&
                      steeringError > 0.0) ||
                     (strongTurnDirection_ == LineDirection::Right &&
                      steeringError < 0.0));
                if (steeringSignCrossed)
                {
                    exitStrongTurn("steering_sign_crossed", true);
                }
                else if (std::abs(steeringError) <=
                         kStrongSteeringExitError)
                {
                    ++strongTurnExitSamples_;
                }
                else
                {
                    strongTurnExitSamples_ = 0;
                }
            }

            if (strongTurnActive_ &&
                strongTurnExitSamples_ >= kStrongTurnExitSamples)
            {
                exitStrongTurn("control_aligned", true);
            }

            if (strongTurnActive_)
            {
                const MotorCommand command = calculateCounterRotationCommand(
                    strongTurnDirection_ == LineDirection::Left,
                    kStrongTurnPower);
                transitionTo(LineFollowState::TurningNear);
                robotState.driveAutonomous(command.left, command.right);
                robotState.updateAutonomousStatus(makeLineStatus(
                    "turning_near", "Curva forte antecipada pela NEAR"));
                return;
            }
        }
        else if (newStrongTurnSample)
        {
            const bool shouldEnterStrongTurn =
                std::abs(steeringError) >= kStrongSteeringEnterError &&
                std::abs(steeringError) >=
                    kStrongSteeringDirectionMinimum;
            if (shouldEnterStrongTurn)
            {
                const LineDirection sampleDirection =
                    steeringError < 0.0
                        ? LineDirection::Left
                        : LineDirection::Right;
                if (sampleDirection == strongTurnDirection_)
                {
                    ++strongTurnEnterSamples_;
                }
                else
                {
                    strongTurnDirection_ = sampleDirection;
                    strongTurnEnterSamples_ = 1;
                }
            }
            else
            {
                strongTurnEnterSamples_ = 0;
                strongTurnDirection_ = LineDirection::Unknown;
            }

            if (strongTurnEnterSamples_ >= kStrongTurnEnterSamples)
            {
                strongTurnActive_ = true;
                strongTurnExitSamples_ = 0;
                const MotorCommand command = calculateCounterRotationCommand(
                    strongTurnDirection_ == LineDirection::Left,
                    kStrongTurnPower);
                logStrongTurnEvent("entered", "none", command);
                transitionTo(LineFollowState::TurningNear);
                robotState.driveAutonomous(command.left, command.right);
                robotState.updateAutonomousStatus(makeLineStatus(
                    "turning_near", "Curva forte antecipada pela NEAR"));
                return;
            }
        }
    }
    else if (strongTurnActive_)
    {
        exitStrongTurn("near_lost", false);
    }
    else
    {
        strongTurnEnterSamples_ = 0;
        strongTurnExitSamples_ = 0;
        strongTurnDirection_ = LineDirection::Unknown;
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

            // O pivô de uma roda reduz o risco de ultrapassar a linha durante
            // um erro extremo de reaquisição.
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
        nearRecoveryActive_ = true;
        nearLostAt_ = now;
        searchDirection_ = LineDirection::Unknown;
    }
    if (cameraLineSnapshot.farValid)
    {
        // A FAR isolada não volta ao tracking normal. Ela apenas mantém uma
        // correção limitada até a NEAR reaparecer e confirmar a linha próxima.
        totalLossActive_ = false;
        totalLossStartedAt_ = {};
        transitionTo(LineFollowState::ReacquiringNear);
        const MotorCommand command =
            calculateLineRecoveryCommand(cameraLineSnapshot.farError);
        robotState.driveAutonomous(command.left, command.right);
        robotState.updateAutonomousStatus(makeLineStatus(
            "fallback_far", "NEAR ausente: alinhando pela FAR"));
        return;
    }
    if (!totalLossActive_)
    {
        // Sem qualquer banda válida, este limite curto impede contrarrotação
        // indefinida sem linha visível.
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
            calculateCounterRotationCommand(false, kLineRecoveryBaseSpeed);
        robotState.driveAutonomous(command.left, command.right);
        robotState.updateAutonomousStatus(makeLineStatus(
            "searching_right", "Procurando linha à direita"));
        return;
    }

    transitionTo(LineFollowState::SearchingLeft);
    const MotorCommand command =
        calculateCounterRotationCommand(true, kLineRecoveryBaseSpeed);
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
    strongTurnActive_ = false;
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
    gapStartedAt_ = {};
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
    case LineFollowState::TurningNear:
        return "TurningNear";
    case LineFollowState::CrossingGap:
        return "CrossingGap";
    case LineFollowState::ReacquiringNear:
        return "ReacquiringNear";
    case LineFollowState::SearchingLeft:
        return "SearchingLeft";
    case LineFollowState::SearchingRight:
        return "SearchingRight";
    }
    return "Unknown";
}
