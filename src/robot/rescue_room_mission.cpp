#include "obr/rescue_room_mission.h"

#include "obr/config.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

namespace
{
AutonomousStatus makeStatus(
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

bool approachReturnedToSearch(const AutonomousStatus& status)
{
    return status.phase == "rescue_reacquiring_victim" ||
           status.phase == "rescue_search_pivot" ||
           status.phase == "rescue_search_settling" ||
           status.phase == "rescue_search_waiting_frame";
}

bool initialAlignmentMustRestart(const AutonomousStatus& status)
{
    return status.phase == "ball_alignment_target_lost_timeout" ||
           status.phase == "ball_alignment_camera_stale_timeout";
}

bool allServoOutputsEnabled(const Esp32TelemetrySnapshot& telemetry)
{
    return telemetry.armServoEnabled && telemetry.wristServoEnabled &&
           telemetry.gripperServoEnabled;
}
}

RescueRoomOutput RescueRoomMission::update(
    const ForwardBallSnapshot& ball,
    const RescueZoneSnapshot& zones,
    const Esp32TelemetrySnapshot& telemetry,
    std::uint64_t autonomousRunSequence,
    unsigned long long servoConfirmationSequence,
    const ServoPose& currentServoPose,
    std::chrono::steady_clock::time_point now)
{
    RescueRoomOutput output = updateStep(
        ball,
        zones,
        telemetry,
        autonomousRunSequence,
        servoConfirmationSequence,
        currentServoPose,
        now);
    applyDelicateMotionKick(output, now);
    return output;
}

RescueRoomOutput RescueRoomMission::updateStep(
    const ForwardBallSnapshot& ball,
    const RescueZoneSnapshot& zones,
    const Esp32TelemetrySnapshot& telemetry,
    std::uint64_t autonomousRunSequence,
    unsigned long long servoConfirmationSequence,
    const ServoPose& currentServoPose,
    std::chrono::steady_clock::time_point now)
{
    RescueRoomOutput output;
    output.internalObjectStored = storedAliveVictim_;

    if (phase_ == Phase::Completed)
    {
        output.completed = true;
        output.status = makeStatus(
            "rescue_room_completed",
            "Resgate concluído: nenhuma vítima adicional encontrada",
            100.0);
        return output;
    }
    if (phase_ == Phase::Failed)
    {
        applyCollectionRetention(output);
        output.failed = true;
        output.status = failureStatus_;
        return output;
    }
    if (!telemetry.readyForOperation())
    {
        triangleMission_.pause(now);
        applyCollectionRetention(output);
        // Uma perda transitória para o robô sem descartar toda a sequência.
        // Quando a telemetria voltar, a etapa atual continua do ponto seguro.
        output.status = makeStatus(
            "rescue_room_esp32_not_ready",
            "Resgate pausado: aguardando a ESP32 voltar a ficar pronta");
        return output;
    }
    if (servoMotionStarted_ &&
        (!telemetry.servoHoldSupported || !telemetry.servoHoldActive))
    {
        output.failed = true;
        output.status = makeStatus(
            "rescue_servo_hold_lost",
            "Resgate interrompido: proteção contra FULL_OFF não confirmada");
        fail(output.status);
        return output;
    }
    if (servoOutputsConfirmed_ && !allServoOutputsEnabled(telemetry))
    {
        // Depois da primeira confirmação, qualquer canal OFF invalida a posição
        // mecânica do braço. O pulso nunca pode avançar nessa condição.
        output.failed = true;
        output.status = makeStatus(
            "rescue_servo_output_lost",
            "Resgate interrompido: um servo perdeu PWM; pulso bloqueado");
        fail(output.status);
        return output;
    }

    const std::uint64_t expectedTargetSequence =
        ballTargetSequence(autonomousRunSequence);

    if (requiresBallDetection() && ball.sourceFresh &&
        ball.targetSequence == expectedTargetSequence)
    {
        const double tx = matchesVictim(ball, desiredVictimType_, expectedTargetSequence)
                              ? ball.txDegrees : ball.candidateTxDegrees;
        if ((ball.candidateVisible || ball.detected) && std::isfinite(tx) &&
            std::abs(tx) <= 45.0 && tx != 0.0)
        {
            candidateSide_ = tx < 0.0 ? -1 : 1;
            candidateHeadingDegrees_ = std::remainder(telemetry.yawZDeg + tx, 360.0);
            candidateHeadingValid_ = true;
        }
    }

    if (phase_ == Phase::EntryAdvance)
    {
        // O deslocamento só começa depois que o primeiro resultado do YOLO
        // confirma que o gate realmente abriu. Durante a espera, o PWM é zero.
        if (distanceController_.idle() &&
            (!ball.sourceFresh ||
             ball.targetSequence != expectedTargetSequence))
        {
            output.status = makeStatus(
                "rescue_entry_waiting_yolo",
                "Parado: aguardando o YOLO da busca atual antes do avanço de 10 cm");
            return output;
        }
        if (distanceController_.idle())
        {
            startDistance(
                config::kRescueEntryAdvanceDistanceCm,
                config::kRescueEntryAdvancePower,
                1,
                now);
        }
        output = updateDistance(
            telemetry,
            now,
            "rescue_entry_advancing",
            "Avançando 10 cm com o detector de vítimas ativo");
        output.internalObjectStored = storedAliveVictim_;
        if (output.failed)
        {
            fail(output.status);
            return output;
        }
        if (!output.completed)
        {
            return output;
        }

        distanceController_.reset();
        if (matchesVictim(ball, desiredVictimType_, expectedTargetSequence))
        {
            initialAlternatingSweepAllowed_ = false;
            carriedVictimType_ = desiredVictimType_;
            initialVictimAlignmentMission_.reset();
            phase_ = Phase::AlignVictim;
            output.completed = false;
            output.status = makeStatus(
                "rescue_victim_acquired",
                "Vítima viva encontrada durante a entrada; iniciando alinhamento");
            return output;
        }
        phase_ = Phase::SearchVictim;
        sweepStep_ = SweepStep::First45;
        sweepTurnStarted_ = false;
        waitingForSweepFrame_ = false;
        output.completed = false;
        output.status = makeStatus(
            "rescue_search_starting",
            "Avanço concluído: iniciando varredura da vítima viva");
        return output;
    }

    if (phase_ == Phase::SearchVictim)
    {
        return updateSearch(ball, telemetry, expectedTargetSequence, now);
    }

    if (phase_ == Phase::AlignVictim)
    {
        const BallAlignmentOutput alignment =
            initialVictimAlignmentMission_.update(
                ball,
                telemetry,
                expectedTargetSequence,
                now,
                true);
        output.leftPower = alignment.leftPower;
        output.rightPower = alignment.rightPower;
        output.status = alignment.status;
        output.internalObjectStored = storedAliveVictim_;
        if (!alignment.finished)
        {
            return output;
        }
        if (initialAlignmentMustRestart(alignment.status))
        {
            initialVictimAlignmentMission_.reset();
            startVictimSearch(desiredVictimType_, finalVerification_);
            output.leftPower = 0.0;
            output.rightPower = 0.0;
            output.status = makeStatus(
                "rescue_reacquiring_victim",
                "Alvo perdido no alinhamento inicial; reiniciando a busca");
            return output;
        }
        if (alignment.status.phase != "ball_aligned")
        {
            fail(alignment.status);
            output.failed = true;
            return output;
        }

        initialVictimAlignmentMission_.reset();
        phase_ = Phase::PrepareCapture;
        output.status = makeStatus(
            "rescue_victim_aligned",
            "Alinhamento confirmado; preparando a garra aberta");
        return output;
    }

    if (phase_ == Phase::PrepareCapture)
    {
        output = updateServo(
            ServoRoutineKind::PrepareCapture,
            telemetry,
            autonomousRunSequence,
            servoConfirmationSequence,
            currentServoPose,
            now);
        if (output.failed)
        {
            fail(output.status);
            return output;
        }
        if (output.completed)
        {
            servoRoutine_.resetExecution();
            victimApproachMission_.reset();
            phase_ = Phase::ApproachVictim;
            output.completed = false;
            output.status = makeStatus(
                "rescue_capture_ready",
                "Garra aberta na posição de coleta; iniciando aproximação pelo YOLO");
        }
        return output;
    }

    if (phase_ == Phase::ApproachVictim)
    {
        const RescueAreaOutput approach = victimApproachMission_.update(
            ball,
            telemetry,
            expectedTargetSequence,
            false,
            true,
            now);
        output.leftPower = approach.leftPower;
        output.rightPower = approach.rightPower;
        output.status = approach.status;
        output.failed = approach.failed;
        output.internalObjectStored = storedAliveVictim_;
        if (approach.failed &&
            approach.status.phase == "ball_alignment_camera_stale_timeout")
        {
            // A câmera pode perder alguns frames durante a aproximação. Isso
            // cancela qualquer PWM e reinicia a busca, sem encerrar a missão.
            victimApproachMission_.reset();
            startVictimSearch(desiredVictimType_, finalVerification_);
            output.failed = false;
            output.leftPower = 0.0;
            output.rightPower = 0.0;
            output.status = makeStatus(
                "rescue_reacquiring_victim",
                "YOLO perdido: reiniciando a busca da mesma vítima");
            return output;
        }
        if (approach.failed)
        {
            fail(approach.status);
            return output;
        }
        if (approachReturnedToSearch(approach.status))
        {
            victimApproachMission_.reset();
            startVictimSearch(desiredVictimType_, finalVerification_);
            output.leftPower = 0.0;
            output.rightPower = 0.0;
            output.status = makeStatus(
                "rescue_reacquiring_victim",
                "Alvo perdido: reiniciando a varredura limitada");
            return output;
        }
        if (approach.completed)
        {
            victimApproachMission_.reset();
            phase_ = Phase::SecureCapture;
            output.completed = false;
            output.status = makeStatus(
                "rescue_victim_reached",
                "Aproximação concluída; fechando a garra");
        }
        return output;
    }

    if (phase_ == Phase::SecureCapture)
    {
        output = updateServo(
            ServoRoutineKind::GripForReverse,
            telemetry,
            autonomousRunSequence,
            servoConfirmationSequence,
            currentServoPose,
            now);
        if (output.failed)
        {
            fail(output.status);
            return output;
        }
        if (output.completed)
        {
            // A vítima já está presa, mas o braço continua em 103° e o pulso
            // permanece imóvel. Somente uma ré concluída libera a elevação.
            const bool willUseInternalStorage =
                carriedVictimType_ == VictimType::Alive &&
                (collectedAliveVictims_ == 0 || storedAliveVictim_);
            liftRoutineKind_ = willUseInternalStorage
                ? ServoRoutineKind::LiftAfterReverse
                : ServoRoutineKind::LiftAfterReverseForDirectDeposit;
            collectionRetentionPose_ = output.servoPose;
            collectionRetentionPose_.gripperDegrees =
                config::kServoRoutineGripperRetentionDegrees;
            collectionRetentionActive_ = true;
            servoRoutine_.resetExecution();
            if (carriedVictimType_ == VictimType::Alive)
            {
                ++collectedAliveVictims_;
            }
            startDistance(
                config::kRescuePostCollectionReverseDistanceCm,
                config::kRescuePostCollectionReversePower,
                -1,
                now);
            phase_ = Phase::ReverseAfterCollection;
            output.completed = false;
            output.status = makeStatus(
                "rescue_collection_secured",
                "Vítima presa; iniciando ré de até 15 cm");
        }
        return output;
    }

    if (phase_ == Phase::ReverseAfterCollection)
    {
        output = updateDistance(
            telemetry,
            now,
            "rescue_collection_reversing",
            "Recuando até 15 cm após a coleta");
        applyCollectionRetention(output);
        output.internalObjectStored = storedAliveVictim_;
        if (output.failed)
        {
            fail(output.status);
            return output;
        }
        if (!output.completed)
        {
            return output;
        }

        distanceController_.reset();
        output.completed = false;
        phase_ = Phase::LiftAfterCollection;
        output.status = makeStatus(
            "rescue_collection_lifting", "Ré concluída; liberando a elevação do braço");
        return output;
    }

    if (phase_ == Phase::LiftAfterCollection)
    {
        output = updateServo(
            liftRoutineKind_, telemetry, autonomousRunSequence,
            servoConfirmationSequence, currentServoPose, now);
        if (output.failed)
        {
            applyCollectionRetention(output);
            fail(output.status);
            return output;
        }
        // Atualiza a retenção conforme a elevação é solicitada para nunca
        // reaplicar a pose baixa durante o deslocamento até o triângulo.
        collectionRetentionPose_ = output.servoPose;
        if (!output.completed)
        {
            return output;
        }
        servoRoutine_.resetExecution();
        output.completed = false;
        candidateSide_ = -1;
        if (carriedVictimType_ == VictimType::Alive &&
            collectedAliveVictims_ == 1 && !storedAliveVictim_)
        {
            phase_ = Phase::StoreFirstAlive;
            output.status = makeStatus(
                "rescue_first_alive_storage",
                "Ré concluída; armazenando internamente a primeira vítima viva");
            return output;
        }

        triangleMission_.reset();
        phase_ = Phase::FindDepositZone;
        output.status = makeStatus(
            "rescue_deposit_zone_starting",
            carriedVictimType_ == VictimType::Alive
                ? "Procurando o triângulo verde"
                : "Procurando o triângulo vermelho");
        return output;
    }

    if (phase_ == Phase::StoreFirstAlive)
    {
        collectionRetentionActive_ = false;
        output = updateServo(
            ServoRoutineKind::InternalStorage,
            telemetry,
            autonomousRunSequence,
            servoConfirmationSequence,
            currentServoPose,
            now);
        if (output.failed)
        {
            fail(output.status);
            return output;
        }
        if (output.completed)
        {
            servoRoutine_.resetExecution();
            storedAliveVictim_ = true;
            startVictimSearch(VictimType::Alive, false);
            output.completed = false;
            output.internalObjectStored = true;
            output.status = makeStatus(
                "rescue_first_alive_stored",
                "Primeira vítima viva armazenada; procurando a segunda");
        }
        return output;
    }

    if (phase_ == Phase::FindDepositZone)
    {
        const RescueZoneTargetColor targetColor =
            carriedVictimType_ == VictimType::Alive
                ? RescueZoneTargetColor::Green
                : RescueZoneTargetColor::Red;
        const RescueZoneTriangleOutput triangle = triangleMission_.update(
            zones,
            telemetry,
            targetColor,
            now);
        output.leftPower = triangle.leftPower;
        output.rightPower = triangle.rightPower;
        output.status = triangle.status;
        output.failed = triangle.failed;
        output.internalObjectStored = storedAliveVictim_;
        applyCollectionRetention(output);
        if (triangle.failed)
        {
            fail(triangle.status);
            return output;
        }
        if (triangle.completed)
        {
            triangleMission_.reset();
            depositRoutineKind_ =
                carriedVictimType_ == VictimType::Alive && storedAliveVictim_
                    ? ServoRoutineKind::DepositCarriedAndStored
                    : ServoRoutineKind::Deposit;
            phase_ = Phase::DepositVictims;
            output.completed = false;
            output.status = makeStatus(
                "rescue_deposit_ready",
                "Triângulo alcançado; iniciando a entrega das vítimas");
        }
        return output;
    }

    if (phase_ == Phase::DepositVictims)
    {
        collectionRetentionActive_ = false;
        output = updateServo(
            depositRoutineKind_,
            telemetry,
            autonomousRunSequence,
            servoConfirmationSequence,
            currentServoPose,
            now);
        if (output.failed)
        {
            fail(output.status);
            return output;
        }
        if (output.completed)
        {
            servoRoutine_.resetExecution();
            if (carriedVictimType_ == VictimType::Alive)
            {
                deliveredAliveVictims_ += storedAliveVictim_ ? 2 : 1;
                storedAliveVictim_ = false;
            }
            else
            {
                ++deliveredDeadVictims_;
            }
            const bool requiredDeadDeliveryCompleted =
                carriedVictimType_ == VictimType::Dead &&
                deliveredAliveVictims_ >= 2 &&
                deliveredDeadVictims_ == 1 &&
                !finalVerification_;
            // A última vítima pode ser depositada no vermelho ou no verde.
            // Nos dois casos, 40 cm deixam a busca da saída longe do triângulo.
            const bool finalDepositReverse =
                requiredDeadDeliveryCompleted || finalVerification_;
            postDepositReverseDistanceCm_ =
                finalDepositReverse
                    ? config::kRescueFinalDepositReverseDistanceCm
                    : config::kRescuePostDepositReverseDistanceCm;
            startDistance(
                postDepositReverseDistanceCm_,
                config::kRescuePostDepositReversePower,
                -1,
                now);
            phase_ = Phase::ReverseAfterDeposit;
            output.completed = false;
            output.internalObjectStored = false;
            output.status = makeStatus(
                "rescue_deposit_completed",
                finalDepositReverse
                    ? "Entrega final concluída; iniciando ré de 40 cm"
                    : "Entrega concluída; iniciando ré de 20 cm");
        }
        return output;
    }

    if (phase_ == Phase::ReverseAfterDeposit)
    {
        output = updateDistance(
            telemetry,
            now,
            "rescue_deposit_reversing",
            postDepositReverseDistanceCm_ ==
                    config::kRescueFinalDepositReverseDistanceCm
                ? "Recuando 40 cm antes de continuar a verificação final"
                : "Recuando 20 cm para liberar o triângulo");
        if (output.failed)
        {
            fail(output.status);
            return output;
        }
        if (!output.completed)
        {
            return output;
        }

        distanceController_.reset();
        output.completed = false;
        if (deliveredAliveVictims_ < 2)
        {
            startVictimSearch(VictimType::Alive, false);
            output.status = makeStatus(
                "rescue_required_alive_search",
                "Procurando a próxima vítima viva obrigatória");
        }
        else if (deliveredDeadVictims_ < 1)
        {
            startVictimSearch(VictimType::Dead, false);
            output.status = makeStatus(
                "rescue_required_dead_search",
                "Duas vítimas vivas entregues; procurando a vítima morta");
        }
        else
        {
            startVictimSearch(VictimType::Alive, true);
            output.status = makeStatus(
                "rescue_final_verification",
                "Entregas obrigatórias concluídas; verificando vítimas vivas extras");
        }
        return output;
    }

    output.failed = true;
    output.status = makeStatus(
        "rescue_room_invalid_phase",
        "Resgate interrompido: estado interno inválido");
    fail(output.status);
    return output;
}

void RescueRoomMission::applyDelicateMotionKick(
    RescueRoomOutput& output,
    std::chrono::steady_clock::time_point now)
{
    constexpr double kStraightTolerance = 0.001;
    const auto direction = [=](double power) {
        if (power > kStraightTolerance) return 1;
        if (power < -kStraightTolerance) return -1;
        return 0;
    };

    const int leftDirection = direction(output.leftPower);
    const int rightDirection = direction(output.rightPower);
    const bool encoderStraightMotion =
        output.status.phase == "rescue_entry_advancing" ||
        output.status.phase == "rescue_collection_reversing" ||
        output.status.phase == "rescue_deposit_reversing";
    const bool straightMotion =
        encoderStraightMotion ||
        (leftDirection != 0 && leftDirection == rightDirection &&
         std::abs(std::abs(output.leftPower) - std::abs(output.rightPower)) <=
             kStraightTolerance);
    const bool delicateMotion =
        (leftDirection != 0 || rightDirection != 0) && !straightMotion;

    if (!delicateMotion)
    {
        // Paradas e retas encerram o trecho. Um novo giro receberá somente um
        // novo impulso, em vez de renovar 0,80 continuamente a cada ciclo.
        delicateMotionActive_ = false;
        delicateMotionLeftDirection_ = 0;
        delicateMotionRightDirection_ = 0;
        delicateMotionKickDeadline_ = {};
        return;
    }

    const bool directionChanged =
        leftDirection != delicateMotionLeftDirection_ ||
        rightDirection != delicateMotionRightDirection_;
    if (!delicateMotionActive_ || directionChanged)
    {
        delicateMotionActive_ = true;
        delicateMotionLeftDirection_ = leftDirection;
        delicateMotionRightDirection_ = rightDirection;
        delicateMotionKickDeadline_ =
            now + std::chrono::milliseconds(
                      config::kRescueDelicateMotionKickDurationMs);
    }

    if (now >= delicateMotionKickDeadline_)
    {
        return;
    }

    // Zero continua zero em pivôs de uma roda. Nos demais lados, 0,80 funciona
    // como piso temporário e nunca reduz um comando que já seja mais forte.
    if (leftDirection != 0)
    {
        output.leftPower = std::copysign(
            std::max(std::abs(output.leftPower),
                     config::kRescueDelicateMotionKickPower),
            output.leftPower);
    }
    if (rightDirection != 0)
    {
        output.rightPower = std::copysign(
            std::max(std::abs(output.rightPower),
                     config::kRescueDelicateMotionKickPower),
            output.rightPower);
    }
}

void RescueRoomMission::advanceSweepStep()
{
    switch (sweepStep_)
    {
    case SweepStep::First45: sweepStep_ = SweepStep::Opposite45; break;
    case SweepStep::Opposite45: sweepStep_ = SweepStep::First75; break;
    case SweepStep::First75: sweepStep_ = SweepStep::Opposite75; break;
    case SweepStep::Opposite75: sweepStep_ = SweepStep::Finished; break;
    case SweepStep::Finished: break;
    }
    sweepAttemptStarted_ = false;
    sweepTurnStarted_ = false;
    waitingForSweepFrame_ = false;
    sweepTurnController_.reset();
}

RescueRoomOutput RescueRoomMission::updateSearch(
    const ForwardBallSnapshot& ball,
    const Esp32TelemetrySnapshot& telemetry,
    std::uint64_t expectedTargetSequence,
    std::chrono::steady_clock::time_point now)
{
    RescueRoomOutput output;
    output.internalObjectStored = storedAliveVictim_;
    if (matchesVictim(ball, desiredVictimType_, expectedTargetSequence))
    {
        // Depois que qualquer vítima foi encontrada, uma eventual busca
        // posterior nunca volta aos headings alternados da entrada.
        initialAlternatingSweepAllowed_ = false;
        continuousSearchProgressWatchActive_ = false;
        sweepTurnController_.reset();
        carriedVictimType_ = desiredVictimType_;
        initialVictimAlignmentMission_.reset();
        phase_ = Phase::AlignVictim;
        output.status = makeStatus(
            "rescue_victim_acquired", "Vítima confirmada; motores parados antes do alinhamento");
        return output;
    }

    const bool wideSweep = sweepStep_ == SweepStep::First75 ||
                           sweepStep_ == SweepStep::Opposite75;
    const int timeoutMs = wideSweep ? config::kRescueVictimSecondSweepTimeoutMs
                                    : config::kRescueVictimFirstSweepTimeoutMs;
    // A espera por uma candidata não renova o orçamento. Mesmo com pausas,
    // uma tentativa bloqueada termina sem insistir indefinidamente na parede.
    if (!continuousSearchActive_ && sweepAttemptStarted_ &&
        !waitingForSweepFrame_ &&
        now - sweepAttemptStartedAt_ >= std::chrono::milliseconds(timeoutMs))
    {
        ++sweepTimeoutCount_;
        advanceSweepStep();
        output.status = makeStatus(
            "rescue_search_turn_timeout", "Tempo do giro esgotado; parado antes da próxima tentativa");
        std::cout << "Rescue sweep timeout: limitMs=" << timeoutMs
                  << " yaw=" << telemetry.yawZDeg << '\n';
        return output;
    }
    if (!ball.sourceFresh || ball.targetSequence != expectedTargetSequence ||
        !ImuTurnController::imuReady(telemetry))
    {
        // Após a pausa, recalcula o ângulo restante pela posição real da IMU.
        // O tempo parado por falta de sensores não caracteriza colisão.
        continuousSearchProgressWatchActive_ = false;
        sweepTurnController_.reset();
        sweepTurnStarted_ = false;
        output.status = makeStatus(
            "rescue_search_waiting_sensors", "Busca parada: aguardando YOLO e IMU atuais");
        return output;
    }
    if (ball.candidateVisible)
    {
        if (!sweepAttemptStarted_)
        {
            sweepAttemptStarted_ = true;
            sweepAttemptStartedAt_ = now;
        }
        if (sweepStep_ == SweepStep::First45) sweepFirstSide_ = candidateSide_;
        candidateConfirmationActive_ = true;
        candidateLastSeenAt_ = now;
    }
    if (candidateConfirmationActive_ &&
        now - candidateLastSeenAt_ < std::chrono::milliseconds(config::kRescueVictimCandidateHoldMs))
    {
        // A busca para intencionalmente enquanto confirma ou recupera a
        // candidata; essa espera não deve disparar a inversão antitravamento.
        continuousSearchProgressWatchActive_ = false;
        sweepTurnController_.reset();
        sweepTurnStarted_ = false;
        if (!ball.candidateVisible && candidateHeadingValid_)
        {
            // Se a candidata piscar, retorna ao último heading visto em vez de retomar a varredura.
            const double remaining = std::remainder(
                candidateHeadingDegrees_ - telemetry.yawZDeg, 360.0);
            if (std::abs(remaining) > config::kBallApproachStartToleranceDegrees)
            {
                output.leftPower = remaining < 0.0
                                       ? -config::kRescueSearchTurnPower
                                       : config::kRescueSearchTurnPower;
                output.rightPower = -output.leftPower;
                output.status = makeStatus(
                    "rescue_reacquiring_candidate",
                    "Retornando ao último ponto visto da candidata");
                return output;
            }
        }
        output.status = makeStatus(
            "rescue_confirming_victim", "Motores parados: confirmando a candidata atual");
        return output;
    }
    candidateConfirmationActive_ = false;
    if (continuousSearchActive_)
    {
        bool reversedAfterStall = false;
        if (!continuousSearchProgressWatchActive_)
        {
            continuousSearchProgressWatchActive_ = true;
            continuousSearchProgressYaw_ = telemetry.yawZDeg;
            continuousSearchProgressStartedAt_ = now;
        }
        else
        {
            // std::remainder preserva a menor diferença também ao cruzar
            // +180°/-180°, evitando um falso progresso de quase 360°.
            const double angularProgress = std::abs(std::remainder(
                telemetry.yawZDeg - continuousSearchProgressYaw_, 360.0));
            if (angularProgress >=
                config::kRescueContinuousSearchMinimumProgressDegrees)
            {
                continuousSearchProgressYaw_ = telemetry.yawZDeg;
                continuousSearchProgressStartedAt_ = now;
            }
            else if (now - continuousSearchProgressStartedAt_ >=
                     std::chrono::milliseconds(
                         config::kRescueContinuousSearchStallTimeoutMs))
            {
                // Pouco avanço angular com os motores comandados indica que o
                // robô pode estar pressionando uma parede. O sentido oposto é
                // aplicado imediatamente e recebe uma nova janela de progresso.
                sweepFirstSide_ = -sweepFirstSide_;
                continuousSearchProgressYaw_ = telemetry.yawZDeg;
                continuousSearchProgressStartedAt_ = now;
                reversedAfterStall = true;
                std::cout
                    << "Rescue continuous search reversed: progressDegrees="
                    << angularProgress
                    << " limitDegrees="
                    << config::kRescueContinuousSearchMinimumProgressDegrees
                    << " timeoutMs="
                    << config::kRescueContinuousSearchStallTimeoutMs << '\n';
            }
        }

        // Depois da varredura angular, mantém o sentido enquanto a IMU
        // confirma progresso. Se ficar preso, o watchdog escolhe o lado oposto.
        output.leftPower = sweepFirstSide_ * config::kRescueSearchTurnPower;
        output.rightPower = -output.leftPower;
        output.status = makeStatus(
            "rescue_search_continuous",
            reversedAfterStall
                ? (sweepFirstSide_ < 0
                       ? "Pouco avanço na IMU; busca invertida para a esquerda"
                       : "Pouco avanço na IMU; busca invertida para a direita")
                : (sweepFirstSide_ < 0
                       ? "Busca contínua de vítima girando para a esquerda"
                       : "Busca contínua de vítima girando para a direita"));
        return output;
    }
    if (waitingForSweepFrame_)
    {
        if (ball.timestamp <= sweepFrameTimestamp_)
        {
            output.status = makeStatus(
                "rescue_search_waiting_frame", "Parado: aguardando frame posterior ao giro");
            return output;
        }
        advanceSweepStep();
    }
    if (sweepStep_ == SweepStep::Finished)
    {
        // São quatro destinos: os dois lados em 45° e os dois lados em 75°.
        if (sweepTimeoutCount_ == 4)
        {
            output.failed = true;
            output.status = makeStatus(
                "rescue_search_blocked", "Busca interrompida: as quatro tentativas excederam o tempo limite");
            fail(output.status);
            return output;
        }
        if (finalVerification_ && desiredVictimType_ == VictimType::Alive)
        {
            startVictimSearch(VictimType::Dead, true);
            output.status = makeStatus(
                "rescue_final_dead_verification", "Nenhuma vítima viva extra; verificando vítimas mortas extras");
            return output;
        }
        if (finalVerification_)
        {
            phase_ = Phase::Completed;
            output.completed = true;
            output.status = makeStatus(
                "rescue_room_completed", "Nenhuma vítima adicional encontrada; missão concluída", 100.0);
            return output;
        }
        const VictimType missingType = desiredVictimType_;
        initialAlternatingSweepAllowed_ = false;
        continuousSearchActive_ = true;
        continuousSearchProgressWatchActive_ = false;
        sweepTurnController_.reset();
        sweepTurnStarted_ = false;
        waitingForSweepFrame_ = false;
        output.status = makeStatus(
            missingType == VictimType::Alive ? "rescue_required_alive_search_continuous"
                                            : "rescue_required_dead_search_continuous",
            "Vítima obrigatória ainda não confirmada; iniciando busca contínua");
        return output;
    }
    if (!sweepReferenceSet_)
    {
        sweepReferenceSet_ = true;
        sweepReferenceYaw_ = telemetry.yawZDeg;
        sweepFirstSide_ = candidateSide_;
    }
    if (!sweepAttemptStarted_)
    {
        sweepAttemptStarted_ = true;
        sweepAttemptStartedAt_ = now;
    }
    const bool opposite = sweepStep_ == SweepStep::Opposite45 ||
                          sweepStep_ == SweepStep::Opposite75;
    const double limit = (sweepStep_ == SweepStep::First75 ||
                          sweepStep_ == SweepStep::Opposite75)
                             ? config::kRescueVictimSecondSweepDegrees
                             : config::kRescueVictimFirstSweepDegrees;
    const double targetYaw = sweepReferenceYaw_ +
                             sweepFirstSide_ * (opposite ? -limit : limit);
    const double remaining = std::remainder(targetYaw - telemetry.yawZDeg, 360.0);
    const auto reachedEndpoint = [&]() {
        sweepTurnController_.reset();
        sweepTurnStarted_ = false;
        waitingForSweepFrame_ = true;
        sweepFrameTimestamp_ = ball.timestamp;
        output.leftPower = 0.0;
        output.rightPower = 0.0;
        output.status = makeStatus(
            "rescue_search_endpoint", "Limite alcançado; aguardando imagem com o robô parado");
    };
    if (!sweepTurnStarted_)
    {
        if (std::abs(remaining) <= config::kRescueVictimSweepToleranceDegrees)
        {
            reachedEndpoint();
            return output;
        }
        const int legTimeoutMs = limit == config::kRescueVictimSecondSweepDegrees
                                     ? config::kRescueVictimSecondSweepTimeoutMs
                                     : config::kRescueVictimFirstSweepTimeoutMs;
        if (!sweepTurnController_.start(
                std::abs(remaining), remaining < 0.0 ? ImuTurnDirection::Left : ImuTurnDirection::Right,
                telemetry, config::kRescueVictimSweepToleranceDegrees,
                config::kTurn90CorrectionPulseMs, -1, config::kRescueSearchTurnPower,
                legTimeoutMs, now))
        {
            output.status = makeStatus("rescue_search_imu_unavailable", "Busca parada: giro indisponível");
            return output;
        }
        sweepTurnStarted_ = true;
    }
    const ImuTurnOutput turn = sweepTurnController_.update(telemetry, now);
    output.leftPower = turn.leftPower;
    output.rightPower = turn.rightPower;
    output.status = makeStatus(
        "rescue_search_sweep", "Procurando vítima no heading " + std::to_string(targetYaw),
        turn.progressPercent);
    if (turn.result == ImuTurnResult::Completed)
    {
        // Confere o destino absoluto: o controlador de giro mede a magnitude
        // percorrida, mas um deslocamento no sentido errado não conclui a busca.
        if (std::abs(remaining) <= config::kRescueVictimSweepToleranceDegrees)
        {
            reachedEndpoint();
        }
        else
        {
            sweepTurnStarted_ = false;
        }
    }
    else if (turn.result == ImuTurnResult::Failed)
    {
        sweepTurnController_.reset();
        sweepTurnStarted_ = false;
        output.status = makeStatus("rescue_search_turn_paused", turn.action);
    }
    return output;
}

RescueRoomOutput RescueRoomMission::updateServo(
    ServoRoutineKind kind,
    const Esp32TelemetrySnapshot& telemetry,
    std::uint64_t autonomousRunSequence,
    unsigned long long confirmationSequence,
    const ServoPose& currentPose,
    std::chrono::steady_clock::time_point now)
{
    RescueRoomOutput output;
    output.internalObjectStored = storedAliveVictim_;
    if (!telemetry.pca9685Ok)
    {
        output.failed = true;
        output.status = makeStatus(
            "rescue_servo_driver_lost",
            "Resgate interrompido: PCA9685 indisponível");
        return output;
    }
    if (!telemetry.servoHoldSupported)
    {
        output.failed = true;
        output.status = makeStatus(
            "rescue_servo_hold_unsupported",
            "Resgate bloqueado: atualize a ESP32 para impedir FULL_OFF");
        return output;
    }
    if (!telemetry.servoHoldActive)
    {
        output.status = makeStatus(
            "rescue_servo_hold_waiting",
            "Parado: aguardando a ESP32 travar o PWM dos servos");
        return output;
    }

    const bool outputsEnabled = allServoOutputsEnabled(telemetry);
    if (servoMotionStarted_ && !servoOutputsConfirmed_)
    {
        if (outputsEnabled)
        {
            servoOutputsConfirmed_ = true;
        }
        else if (now >= servoEnableDeadline_)
        {
            output.failed = true;
            output.status = makeStatus(
                "rescue_servo_enable_timeout",
                "Resgate interrompido: servos não confirmaram PWM ativo");
            return output;
        }
        else
        {
            // Repete a pose inicial sem avançar o relógio da sequência. Isso
            // cobre o atraso entre o comando UART e a próxima telemetria.
            output.servoPoseRequested = true;
            output.servoPose = pendingServoPose_;
            output.status = makeStatus(
                "rescue_servo_enable_waiting",
                "Mantendo pose: aguardando os três canais ativos");
            return output;
        }
    }

    const ServoRoutineOutput servo = servoRoutine_.update(
        kind,
        autonomousRunSequence,
        confirmationSequence,
        currentPose,
        now);
    output.servoPoseRequested = servo.poseRequested;
    output.releaseGripper = servo.releaseGripper;
    output.servoPose = servo.pose;
    output.completed = servo.completed;
    output.failed = servo.failed;
    output.status = makeStatus(
        servo.phase,
        servo.action,
        servo.progressPercent);
    output.status.servoRoutineWaitingForConfirmation =
        servo.waitingForConfirmation;
    if (servo.poseRequested)
    {
        pendingServoPose_ = servo.pose;
        if (!servoMotionStarted_)
        {
            servoMotionStarted_ = true;
            servoOutputsConfirmed_ = outputsEnabled;
            servoEnableDeadline_ =
                now + std::chrono::milliseconds(
                          config::kRescueServoEnableConfirmationTimeoutMs);
        }
    }
    return output;
}

void RescueRoomMission::startDistance(
    double targetCm, double power, int directionSign,
    std::chrono::steady_clock::time_point now)
{
    distanceController_.start(targetCm, power, directionSign, now);
}

RescueRoomOutput RescueRoomMission::updateDistance(
    const Esp32TelemetrySnapshot& telemetry,
    std::chrono::steady_clock::time_point now,
    const char* phase, const char* action)
{
    const auto movement = distanceController_.update(telemetry, now, phase, action);
    RescueRoomOutput output;
    output.internalObjectStored = storedAliveVictim_;
    output.leftPower = movement.leftPower;
    output.rightPower = movement.rightPower;
    output.completed = movement.completed;
    output.failed = movement.failed;
    output.status = movement.status;
    return output;
}

void RescueRoomMission::startVictimSearch(
    VictimType type,
    bool finalVerification)
{
    if (desiredVictimType_ != type)
    {
        candidateSide_ = -1;
    }
    desiredVictimType_ = type;
    finalVerification_ = finalVerification;
    ++ballTargetGeneration_;
    sweepStep_ = SweepStep::First45;
    sweepReferenceSet_ = false;
    sweepAttemptStarted_ = false;
    sweepTimeoutCount_ = 0;
    sweepTurnStarted_ = false;
    waitingForSweepFrame_ = false;
    // Apenas a primeira busca sem vítima na entrada usa headings alternados.
    // Após encontrar ou recolher qualquer vítima, toda nova busca gira direto.
    continuousSearchActive_ = !initialAlternatingSweepAllowed_;
    if (continuousSearchActive_) sweepFirstSide_ = candidateSide_ < 0 ? -1 : 1;
    continuousSearchProgressWatchActive_ = false;
    candidateConfirmationActive_ = false;
    candidateHeadingValid_ = false;
    sweepFrameTimestamp_ = 0.0;
    sweepTurnController_.reset();
    initialVictimAlignmentMission_.reset();
    victimApproachMission_.reset();
    phase_ = Phase::SearchVictim;
}

void RescueRoomMission::applyCollectionRetention(
    RescueRoomOutput& output) const
{
    if (!collectionRetentionActive_)
    {
        return;
    }

    // A pose completa mantém braço e pulso no último alvo e renova também a
    // autorização da garra. O único passo que deixa de usar 5° é a soltura
    // explícita em 90° dentro da rotina de armazenamento ou depósito.
    output.servoPoseRequested = true;
    output.servoPose = collectionRetentionPose_;
}

void RescueRoomMission::fail(const AutonomousStatus& status)
{
    phase_ = Phase::Failed;
    failureStatus_ = status;
    sweepTurnController_.reset();
    initialVictimAlignmentMission_.reset();
    victimApproachMission_.reset();
    triangleMission_.reset();
}

bool RescueRoomMission::matchesVictim(
    const ForwardBallSnapshot& ball,
    VictimType type,
    std::uint64_t expectedTargetSequence)
{
    return ball.sourceFresh && ball.detected && ball.targetLocked &&
           ball.targetSequence == expectedTargetSequence &&
           std::isfinite(ball.txDegrees) &&
           ball.type == (type == VictimType::Alive
                             ? "silver_ball"
                             : "black_ball");
}

bool RescueRoomMission::requiresBallDetection() const
{
    return phase_ == Phase::EntryAdvance ||
           phase_ == Phase::SearchVictim ||
           phase_ == Phase::AlignVictim ||
           phase_ == Phase::PrepareCapture ||
           phase_ == Phase::ApproachVictim;
}

bool RescueRoomMission::requiresRescueZoneDetection() const
{
    return phase_ == Phase::FindDepositZone &&
           triangleMission_.requiresRescueZoneDetection();
}

std::uint64_t RescueRoomMission::ballTargetSequence(
    std::uint64_t autonomousRunSequence) const
{
    // O bloco por partida reserva 65.536 buscas distintas. Isso mantém a
    // geração monotônica mesmo quando a arena contém vítimas extras.
    return autonomousRunSequence * 65536ULL + ballTargetGeneration_;
}

const char* RescueRoomMission::ballTargetType() const
{
    return desiredVictimType_ == VictimType::Alive
               ? "silver_ball"
               : "black_ball";
}

void RescueRoomMission::reset()
{
    phase_ = Phase::EntryAdvance;
    desiredVictimType_ = VictimType::Alive;
    carriedVictimType_ = VictimType::Alive;
    candidateSide_ = -1;
    sweepFirstSide_ = -1;
    sweepReferenceYaw_ = 0.0;
    liftRoutineKind_ = ServoRoutineKind::LiftAfterReverse;
    sweepStep_ = SweepStep::First45;
    sweepReferenceSet_ = false;
    sweepAttemptStarted_ = false;
    sweepTimeoutCount_ = 0;
    sweepTurnStarted_ = false;
    waitingForSweepFrame_ = false;
    initialAlternatingSweepAllowed_ = true;
    continuousSearchActive_ = false;
    continuousSearchProgressWatchActive_ = false;
    continuousSearchProgressYaw_ = 0.0;
    continuousSearchProgressStartedAt_ = {};
    candidateConfirmationActive_ = false;
    candidateHeadingValid_ = false;
    finalVerification_ = false;
    storedAliveVictim_ = false;
    collectionRetentionActive_ = false;
    servoMotionStarted_ = false;
    servoOutputsConfirmed_ = false;
    collectedAliveVictims_ = 0;
    deliveredAliveVictims_ = 0;
    deliveredDeadVictims_ = 0;
    ballTargetGeneration_ = 0;
    sweepFrameTimestamp_ = 0.0;
    postDepositReverseDistanceCm_ =
        config::kRescuePostDepositReverseDistanceCm;
    collectionRetentionPose_ = {};
    pendingServoPose_ = {};
    servoEnableDeadline_ = {};
    delicateMotionActive_ = false;
    delicateMotionLeftDirection_ = 0;
    delicateMotionRightDirection_ = 0;
    delicateMotionKickDeadline_ = {};
    depositRoutineKind_ = ServoRoutineKind::Deposit;
    sweepTurnController_.reset();
    initialVictimAlignmentMission_.reset();
    victimApproachMission_.reset();
    triangleMission_.reset();
    servoRoutine_.resetExecution();
    distanceController_.reset();
    failureStatus_ = {};
}
