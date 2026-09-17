#include "obr/servo_routine.h"

#include "obr/config.h"

#include <chrono>

ServoRoutineOutput ServoRoutine::update(
    ServoRoutineKind kind,
    unsigned long long runSequence,
    unsigned long long confirmationSequence,
    const ServoPose& initialPose,
    std::chrono::steady_clock::time_point now)
{
    if (activeRunSequence_ != runSequence || activeKind_ != kind || steps_.empty())
    {
        start(kind, runSequence, confirmationSequence, initialPose);
    }
    if (failed_)
    {
        return currentOutput();
    }

    while (stepIndex_ < steps_.size())
    {
        const Step& step = steps_[stepIndex_];
        if (step.action == StepAction::MarkInternalStorage)
        {
            internalObjectStored_ = true;
            ++stepIndex_;
            continue;
        }
        if (step.action == StepAction::ClearInternalStorage)
        {
            internalObjectStored_ = false;
            ++stepIndex_;
            continue;
        }
        if (step.action == StepAction::Complete)
        {
            stepIndex_ = steps_.size();
            stepStarted_ = false;
            break;
        }

        if (step.action == StepAction::WaitForOpenConfirmation ||
            step.action == StepAction::WaitForCloseConfirmation)
        {
            if (!stepStarted_)
            {
                stepStarted_ = true;
                confirmationSequenceAtWait_ = confirmationSequence;
                if (step.action == StepAction::WaitForOpenConfirmation)
                {
                    stepDeadline_ = now + std::chrono::milliseconds(
                                             config::kServoRoutineConfirmationTimeoutMs);
                }
            }
            else if (confirmationSequence > confirmationSequenceAtWait_)
            {
                ++stepIndex_;
                stepStarted_ = false;
                continue;
            }
            else if (step.action == StepAction::WaitForOpenConfirmation &&
                     now >= stepDeadline_)
            {
                failed_ = true;
            }
            return currentOutput();
        }

        if (!stepStarted_)
        {
            stepStarted_ = true;
            stepDeadline_ = now + std::chrono::milliseconds(step.durationMs);
            switch (step.action)
            {
            case StepAction::ApplyInitialPose:
                pose_ = {};
                break;
            case StepAction::HoldCurrentPose:
                break;
            case StepAction::MoveArm:
                pose_.armDegrees = step.angleDegrees;
                break;
            case StepAction::MoveWrist:
                pose_.wristDegrees = step.angleDegrees;
                break;
            case StepAction::MoveGripper:
                pose_.gripperDegrees = step.angleDegrees;
                break;
            case StepAction::ReleaseGripper:
            case StepAction::WaitForOpenConfirmation:
            case StepAction::WaitForCloseConfirmation:
            case StepAction::MarkInternalStorage:
            case StepAction::ClearInternalStorage:
            case StepAction::Complete:
                break;
            }
        }

        if (now >= stepDeadline_)
        {
            ++stepIndex_;
            stepStarted_ = false;
            continue;
        }
        return currentOutput();
    }

    return currentOutput();
}

void ServoRoutine::resetExecution()
{
    steps_.clear();
    activeRunSequence_ = 0;
    stepIndex_ = 0;
    stepStarted_ = false;
    failed_ = false;
}

void ServoRoutine::start(
    ServoRoutineKind kind,
    unsigned long long runSequence,
    unsigned long long confirmationSequence,
    const ServoPose& initialPose)
{
    steps_.clear();
    pose_ = initialPose;
    activeKind_ = kind;
    activeRunSequence_ = runSequence;
    confirmationSequenceAtWait_ = confirmationSequence;
    stepIndex_ = 0;
    stepStarted_ = false;
    failed_ = false;

    const auto appendArm = [this](double angle, const char* phase, const char* description)
    {
        appendMove(StepAction::MoveArm, angle, config::kServoRoutineArmStepMs,
                   phase, description);
    };
    const auto appendWrist = [this](double angle, const char* phase, const char* description)
    {
        appendMove(StepAction::MoveWrist, angle, config::kServoRoutineWristStepMs,
                   phase, description);
    };
    const auto appendGripper = [this](
                                   double angle,
                                   int durationMs,
                                   const char* phase,
                                   const char* description)
    {
        appendMove(StepAction::MoveGripper, angle, durationMs, phase, description);
    };
    const auto appendGripWithRetention = [&appendGripper](
                                             const char* pressPhase,
                                             const char* retentionPhase)
    {
        appendGripper(
            config::kServoRoutineGripperClosedDegrees,
            config::kServoRoutineGripperPressMs,
            pressPhase, "Apertando a vítima com a garra em 0°");
        appendGripper(
            config::kServoRoutineGripperRetentionDegrees,
            config::kServoRoutineGripperStepMs,
            retentionPhase, "Mantendo a vítima presa com a garra em 5°");
    };
    const auto appendFirstCapture = [
                                        &appendArm,
                                        &appendWrist,
                                        &appendGripper,
                                        &appendGripWithRetention](double finalArmDegrees)
    {
        // O braço chega primeiro a 15° para liberar mecanicamente o pulso.
        appendArm(config::kServoRoutineArmHomeDegrees,
                  "servo_capture_arm_clearance", "Elevando braço para 15°");
        appendWrist(config::kServoRoutineWristForwardDegrees,
                    "servo_capture_wrist_forward", "Movendo pulso para 180°");
        appendGripper(config::kServoRoutineGripperFullyOpenDegrees,
                      config::kServoRoutineGripperStepMs,
                      "servo_capture_gripper_open", "Abrindo garra completamente em 180°");
        appendArm(config::kServoRoutineArmPickupDegrees,
                  "servo_capture_arm_pickup", "Movendo braço para 103°");
        appendGripWithRetention(
            "servo_capture_gripper_press", "servo_capture_gripper_retention");
        appendArm(finalArmDegrees, "servo_capture_arm_finish",
                  finalArmDegrees == 0.0
                      ? "Movendo braço para 0°"
                      : "Retornando braço para 15°");
    };
    const auto appendStorage = [
                                   this,
                                   &appendArm,
                                   &appendWrist,
                                   &appendGripper](bool finishReadyForCapture)
    {
        appendWrist(config::kServoRoutineWristInternalDegrees,
                    "servo_store_wrist_internal", "Movendo pulso para 0°");
        appendGripper(config::kServoRoutineGripperDepositDegrees,
                      config::kServoRoutineGripperStepMs,
                      "servo_store_gripper_release",
                      "Abrindo garra em 90° para armazenar a vítima");
        // O braço afasta primeiro a garra do compartimento. Somente em 50° o
        // pulso recebe a folga necessária para girar com segurança até 45°.
        appendArm(config::kServoRoutineArmStorageClearanceDegrees,
                  "servo_store_arm_clearance", "Elevando braço para 50°");
        appendWrist(config::kServoRoutineWristStorageClearanceDegrees,
                    "servo_store_wrist_clearance", "Movendo pulso para 45°");
        appendGripper(config::kServoRoutineGripperClosedDegrees,
                      config::kServoRoutineGripperStepMs,
                      "servo_store_gripper_close", "Fechando garra vazia em 0°");
        appendArm(config::kServoRoutineArmStorageTransitionDegrees,
                  "servo_store_arm_transition", "Movendo braço para 20°");
        if (finishReadyForCapture)
        {
            appendWrist(config::kServoRoutineWristForwardDegrees,
                        "servo_store_wrist_forward", "Movendo pulso para 180°");
            appendArm(config::kServoInitialAngleDegrees,
                      "servo_store_arm_ready", "Movendo braço para 0°");
        }
        steps_.push_back({StepAction::MarkInternalStorage});
    };

    if (kind == ServoRoutineKind::Initialize)
    {
        pose_ = {
            config::kAutonomousInitialArmAngleDegrees,
            config::kServoInitialAngleDegrees,
            config::kServoInitialAngleDegrees};
        steps_.push_back({
            StepAction::HoldCurrentPose, 0.0,
            config::kServoRoutineInitialPoseMs,
            "servo_initial_pose", "Estabilizando braço em 15° e demais servos em 0°"});
        steps_.push_back({StepAction::Complete});
        return;
    }

    steps_.push_back({
        StepAction::HoldCurrentPose, 0.0, config::kServoRoutineResumePoseMs,
        "servo_resume_pose", "Estabilizando a pose antes da sequência"});

    if (kind == ServoRoutineKind::Wave)
    {
        // Normaliza uma pose anterior antes do gesto. O braço em 15° libera
        // o pulso; a garra conserva seu ângulo e não solta uma vítima carregada.
        appendArm(config::kServoRoutineArmHomeDegrees,
                  "servo_wave_clearance", "Liberando o pulso com braço em home");
        appendWrist(config::kServoRoutineWristInternalDegrees,
                    "servo_wave_wrist_home", "Preparando o pulso em 0°");
        appendArm(config::kServoWaveArmDegrees,
                  "servo_wave_arm", "Posicionando o braço para o tchauzinho");
        for (int repetition = 0; repetition < config::kServoWaveRepetitions; ++repetition)
        {
            appendMove(StepAction::MoveWrist, config::kServoWaveWristDegrees,
                       config::kServoWaveWristStepMs,
                       "servo_wave_out", "Tchauzinho: movendo o pulso para 30°");
            appendMove(StepAction::MoveWrist, config::kServoRoutineWristInternalDegrees,
                       config::kServoWaveWristStepMs,
                       "servo_wave_back", "Tchauzinho: retornando o pulso para 0°");
        }
        appendArm(config::kServoRoutineArmHomeDegrees,
                  "servo_wave_home", "Tchauzinho concluído: retornando o braço para home");
    }
    else if (kind == ServoRoutineKind::PrepareCapture)
    {
        // Esta etapa é o prefixo já validado da coleta. O orquestrador pausa
        // aqui para o YOLO aproximar o robô antes de a garra ser fechada.
        appendArm(config::kServoRoutineArmHomeDegrees,
                  "servo_capture_arm_clearance", "Elevando braço para 15°");
        appendWrist(config::kServoRoutineWristForwardDegrees,
                    "servo_capture_wrist_forward", "Movendo pulso para 180°");
        appendGripper(config::kServoRoutineGripperFullyOpenDegrees,
                      config::kServoRoutineGripperStepMs,
                      "servo_capture_gripper_open", "Abrindo garra completamente em 180°");
        appendArm(config::kServoRoutineArmPickupDegrees,
                  "servo_capture_arm_pickup", "Movendo braço para 103°");
    }
    else if (kind == ServoRoutineKind::GripForReverse)
    {
        // Conserva braço e pulso na coleta até o orquestrador concluir a ré.
        appendGripWithRetention(
            "servo_capture_gripper_press", "servo_capture_gripper_retention");
    }
    else if (kind == ServoRoutineKind::LiftAfterReverse ||
             kind == ServoRoutineKind::LiftAfterReverseForDirectDeposit)
    {
        const bool direct = kind == ServoRoutineKind::LiftAfterReverseForDirectDeposit;
        appendArm(direct ? config::kServoInitialAngleDegrees
                         : config::kServoRoutineArmHomeDegrees,
                  "servo_capture_arm_finish", "Elevando braço após concluir a ré");
    }
    else if (kind == ServoRoutineKind::SecureCapture)
    {
        // Esta etapa é o sufixo já validado da coleta e conserva os mesmos
        // ângulos, tempos e ordem usados pela rotina completa.
        appendGripWithRetention(
            "servo_capture_gripper_press", "servo_capture_gripper_retention");
        appendArm(config::kServoRoutineArmHomeDegrees,
                  "servo_capture_arm_finish", "Retornando braço para 15°");
    }
    else if (kind == ServoRoutineKind::SecureCaptureForDirectDeposit)
    {
        // A sequência direta validada termina a coleta com o braço em 0°.
        // Esta variante expõe esse mesmo sufixo sem modificar nenhum ângulo.
        appendGripWithRetention(
            "servo_capture_gripper_press", "servo_capture_gripper_retention");
        appendArm(config::kServoInitialAngleDegrees,
                  "servo_capture_arm_finish", "Movendo braço para 0°");
    }
    else if (kind == ServoRoutineKind::Capture)
    {
        appendFirstCapture(config::kServoRoutineArmHomeDegrees);
    }
    else if (kind == ServoRoutineKind::InternalStorage)
    {
        // A missão começa outra busca nesta pose recolhida. Não avançar até
        // 0°/180° evita expor o conjunto enquanto o robô gira pela sala.
        appendStorage(false);
    }
    else if (kind == ServoRoutineKind::Deposit)
    {
        appendArm(config::kServoRoutineArmHomeDegrees,
                  "servo_deposit_arm_clearance", "Elevando braço para 15°");
        appendWrist(config::kServoRoutineWristForwardDegrees,
                    "servo_deposit_wrist_forward", "Movendo pulso para 180°");
        appendGripper(config::kServoRoutineGripperDepositDegrees,
                      config::kServoRoutineGripperStepMs,
                      "servo_deposit_gripper_open", "Abrindo garra em 90° no depósito");
        appendGripper(config::kServoRoutineGripperClosedDegrees,
                      config::kServoRoutineGripperStepMs,
                      "servo_deposit_gripper_close", "Fechando garra vazia em 0°");
        appendArm(config::kServoRoutineArmHomeDegrees,
                  "servo_deposit_arm_home", "Retornando braço para 15°");
        appendWrist(config::kServoRoutineWristInternalDegrees,
                    "servo_deposit_wrist_home", "Retornando pulso para 0°");
    }
    else if (kind == ServoRoutineKind::DepositCarriedAndStored)
    {
        // Corresponde exatamente ao trecho final já validado da sequência
        // completa: entrega a vítima da garra e depois retira a armazenada.
        appendGripper(config::kServoRoutineGripperDepositDegrees,
                      config::kServoRoutineGripperStepMs,
                      "servo_second_deposit", "Abrindo garra em 90° no depósito");
        appendWrist(config::kServoRoutineWristStoredApproachDegrees,
                    "servo_stored_wrist_approach", "Movendo pulso para 65°");
        appendArm(config::kServoRoutineArmStoredPickupDegrees,
                  "servo_stored_arm_approach", "Movendo braço para 65°");
        appendWrist(config::kServoRoutineWristInternalDegrees,
                    "servo_stored_wrist_internal", "Movendo pulso para 0°");
        appendArm(config::kServoRoutineArmHomeDegrees,
                  "servo_stored_arm_pickup", "Movendo braço para 15°");
        appendGripWithRetention(
            "servo_stored_gripper_press", "servo_stored_gripper_retention");
        appendArm(config::kServoRoutineArmStoredCarryDegrees,
                  "servo_stored_arm_carry", "Movendo braço para 25°");
        appendWrist(config::kServoRoutineWristForwardDegrees,
                    "servo_stored_wrist_deposit", "Movendo pulso para 180°");
        appendGripper(config::kServoRoutineGripperDepositDegrees,
                      config::kServoRoutineGripperStepMs,
                      "servo_stored_deposit", "Abrindo garra em 90° no depósito");
        appendArm(config::kServoRoutineArmHomeDegrees,
                  "servo_final_arm_home", "Retornando braço para 15°");
        appendWrist(config::kServoRoutineWristInternalDegrees,
                    "servo_final_wrist_home", "Retornando pulso para 0°");
        appendGripper(config::kServoRoutineGripperClosedDegrees,
                      config::kServoRoutineGripperStepMs,
                      "servo_final_gripper_home", "Fechando garra vazia em 0°");
        steps_.push_back({StepAction::ClearInternalStorage});
    }
    else if (kind == ServoRoutineKind::FullSequence)
    {
        // Sequência completa com armazenamento da primeira vítima.
        steps_.push_back({StepAction::ClearInternalStorage});
        appendFirstCapture(config::kServoRoutineArmHomeDegrees);
        appendStorage(true);
        appendGripper(config::kServoRoutineGripperFullyOpenDegrees,
                      config::kServoRoutineGripperStepMs,
                      "servo_second_gripper_open", "Abrindo garra completamente em 180°");
        appendArm(config::kServoRoutineArmPickupDegrees,
                  "servo_second_arm_pickup", "Movendo braço para 103°");
        appendGripWithRetention(
            "servo_second_gripper_press", "servo_second_gripper_retention");
        appendArm(config::kServoRoutineArmHomeDegrees,
                  "servo_second_arm_carry", "Retornando braço para 15°");
        appendGripper(config::kServoRoutineGripperDepositDegrees,
                      config::kServoRoutineGripperStepMs,
                      "servo_second_deposit", "Abrindo garra em 90° no depósito");
        appendWrist(config::kServoRoutineWristStoredApproachDegrees,
                    "servo_stored_wrist_approach", "Movendo pulso para 65°");
        appendArm(config::kServoRoutineArmStoredPickupDegrees,
                  "servo_stored_arm_approach", "Movendo braço para 65°");
        appendWrist(config::kServoRoutineWristInternalDegrees,
                    "servo_stored_wrist_internal", "Movendo pulso para 0°");
        appendArm(config::kServoRoutineArmHomeDegrees,
                  "servo_stored_arm_pickup", "Movendo braço para 15°");
        appendGripWithRetention(
            "servo_stored_gripper_press", "servo_stored_gripper_retention");
        appendArm(config::kServoRoutineArmStoredCarryDegrees,
                  "servo_stored_arm_carry", "Movendo braço para 25°");
        appendWrist(config::kServoRoutineWristForwardDegrees,
                    "servo_stored_wrist_deposit", "Movendo pulso para 180°");
        appendGripper(config::kServoRoutineGripperDepositDegrees,
                      config::kServoRoutineGripperStepMs,
                      "servo_stored_deposit", "Abrindo garra em 90° no depósito");
        appendArm(config::kServoRoutineArmHomeDegrees,
                  "servo_final_arm_home", "Retornando braço para 15°");
        appendWrist(config::kServoRoutineWristInternalDegrees,
                    "servo_final_wrist_home", "Retornando pulso para 0°");
        appendGripper(config::kServoRoutineGripperClosedDegrees,
                      config::kServoRoutineGripperStepMs,
                      "servo_final_gripper_home", "Fechando garra vazia em 0°");
        steps_.push_back({StepAction::ClearInternalStorage});
    }
    else
    {
        // Sequência completa 2: coleta e entrega sem armazenamento interno.
        steps_.push_back({StepAction::ClearInternalStorage});
        appendFirstCapture(config::kServoInitialAngleDegrees);
        appendGripper(config::kServoRoutineGripperDepositDegrees,
                      config::kServoRoutineGripperStepMs,
                      "servo_direct_deposit", "Abrindo garra em 90° no depósito");
        appendGripper(config::kServoRoutineGripperClosedDegrees,
                      config::kServoRoutineGripperStepMs,
                      "servo_direct_gripper_home", "Fechando garra vazia em 0°");
        appendArm(config::kServoRoutineArmHomeDegrees,
                  "servo_direct_arm_home", "Retornando braço para 15°");
        appendWrist(config::kServoRoutineWristInternalDegrees,
                    "servo_direct_wrist_home", "Retornando pulso para 0°");
    }

    steps_.push_back({StepAction::Complete});
}

void ServoRoutine::appendMove(
    StepAction action,
    double angleDegrees,
    int durationMs,
    const char* phase,
    const char* description)
{
    steps_.push_back({action, angleDegrees, durationMs, phase, description});
}

ServoRoutineOutput ServoRoutine::currentOutput() const
{
    ServoRoutineOutput output;
    output.pose = pose_;
    output.internalObjectStored = internalObjectStored_;
    output.failed = failed_;

    if (failed_)
    {
        output.phase = "servo_confirmation_timeout";
        output.action = "Rotina interrompida: confirmação não recebida em 2 segundos";
        output.progressPercent = steps_.empty()
                                     ? 0.0
                                     : 100.0 * static_cast<double>(stepIndex_) /
                                           static_cast<double>(steps_.size());
        return output;
    }

    if (stepIndex_ >= steps_.size())
    {
        output.completed = true;
        output.phase = "servo_completed";
        output.action = "Rotina de servos concluída";
        output.progressPercent = 100.0;
        return output;
    }

    const Step& step = steps_[stepIndex_];
    output.phase = step.phase;
    output.action = step.description;
    output.progressPercent = 100.0 * static_cast<double>(stepIndex_) /
                             static_cast<double>(steps_.size());
    output.waitingForConfirmation =
        step.action == StepAction::WaitForOpenConfirmation ||
        step.action == StepAction::WaitForCloseConfirmation;
    output.releaseGripper = step.action == StepAction::ReleaseGripper;
    output.poseRequested = step.action == StepAction::ApplyInitialPose ||
                           step.action == StepAction::HoldCurrentPose ||
                           step.action == StepAction::MoveArm ||
                           step.action == StepAction::MoveWrist ||
                           step.action == StepAction::MoveGripper;
    return output;
}
