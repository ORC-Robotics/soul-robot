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

    if (kind == ServoRoutineKind::Initialize)
    {
        steps_.push_back({
            StepAction::ApplyInitialPose, 0.0,
            config::kServoRoutineInitialPoseMs,
            "servo_initial_pose", "Estabilizando braço, pulso e garra em 0°"});
        steps_.push_back({StepAction::Complete});
        return;
    }

    steps_.push_back({
        StepAction::HoldCurrentPose, 0.0,
        config::kServoRoutineResumePoseMs,
        "servo_resume_pose", "Reativando a pose anterior antes do movimento"});

    appendMove(
        StepAction::MoveArm, config::kServoRoutineArmTransferDegrees,
        config::kServoRoutineArmStepMs,
        "servo_arm_transfer", "Movendo braço para 10°");

    if (kind == ServoRoutineKind::Capture)
    {
        appendMove(
            StepAction::MoveWrist, config::kServoRoutineWristForwardDegrees,
            config::kServoRoutineWristStepMs,
            "servo_wrist_forward", "Movendo pulso para 174°");
        appendMove(
            StepAction::MoveGripper, config::kServoRoutineGripperOpenDegrees,
            config::kServoRoutineGripperStepMs,
            "servo_gripper_open", "Abrindo garra em 20°");
        appendMove(
            StepAction::MoveArm, config::kServoRoutineArmPickupDegrees,
            config::kServoRoutineArmStepMs,
            "servo_arm_pickup", "Movendo braço para 105°");
        appendMove(
            StepAction::MoveGripper, config::kServoRoutineGripperClosedDegrees,
            config::kServoRoutineGripperPressMs,
            "servo_gripper_press", "Pressionando objeto com a garra em 3°");
        steps_.push_back({
            StepAction::ReleaseGripper, 0.0, config::kMainLoopPeriodMs * 2,
            "servo_gripper_release", "Removendo esforço contínuo da garra"});
        steps_.push_back({StepAction::Complete});
        return;
    }

    if (kind == ServoRoutineKind::InternalStorage)
    {
        appendMove(
            StepAction::MoveWrist, config::kServoRoutineWristInternalDegrees,
            config::kServoRoutineWristStepMs,
            "servo_wrist_internal", "Movendo pulso para 0°");
        appendMove(
            StepAction::MoveGripper, config::kServoRoutineGripperOpenDegrees,
            config::kServoRoutineGripperStepMs,
            "servo_store_release", "Abrindo garra em 20° no armazenamento");
        appendMove(
            StepAction::MoveArm, config::kServoRoutineArmPickupDegrees,
            config::kServoRoutineArmStepMs,
            "servo_ready_next", "Movendo braço para 105° para nova captura");
        steps_.push_back({StepAction::MarkInternalStorage});
        steps_.push_back({
            StepAction::WaitForCloseConfirmation, 0.0, 0,
            "servo_wait_close", "Aguardando confirmação para fechar a garra"});
        appendMove(
            StepAction::MoveGripper, config::kServoRoutineGripperClosedDegrees,
            config::kServoRoutineGripperPressMs,
            "servo_next_gripper_press", "Pressionando o novo objeto com a garra em 3°");
        steps_.push_back({
            StepAction::ReleaseGripper, 0.0, config::kMainLoopPeriodMs * 2,
            "servo_gripper_release", "Removendo esforço contínuo da garra"});
        steps_.push_back({StepAction::Complete});
        return;
    }

    appendMove(
        StepAction::MoveWrist, config::kServoRoutineWristForwardDegrees,
        config::kServoRoutineWristStepMs,
        "servo_wrist_deposit", "Movendo pulso para 174°");
    steps_.push_back({
        StepAction::WaitForOpenConfirmation, 0.0,
        config::kServoRoutineConfirmationTimeoutMs,
        "servo_wait_open", "Aguardando confirmação para abrir a garra"});
    appendMove(
        StepAction::MoveGripper, config::kServoRoutineGripperOpenDegrees,
        config::kServoRoutineGripperStepMs,
        "servo_deposit_open", "Abrindo garra em 20° para depositar");

    if (internalObjectStored_)
    {
        appendMove(
            StepAction::MoveArm, config::kServoRoutineArmPickupDegrees,
            config::kServoRoutineArmStepMs,
            "servo_stored_arm_pickup", "Movendo braço para 105°");
        appendMove(
            StepAction::MoveWrist, config::kServoRoutineWristInternalDegrees,
            config::kServoRoutineWristStepMs,
            "servo_stored_wrist_internal", "Corrigindo pulso para 0°");
        appendMove(
            StepAction::MoveGripper, config::kServoRoutineGripperInternalOpenDegrees,
            config::kServoRoutineGripperStepMs,
            "servo_stored_gripper_open", "Abrindo garra em 10°");
        appendMove(
            StepAction::MoveArm, config::kServoRoutineArmTransferDegrees,
            config::kServoRoutineArmStepMs,
            "servo_stored_arm_internal", "Movendo braço para 10° no armazenamento");
        appendMove(
            StepAction::MoveGripper, config::kServoRoutineGripperClosedDegrees,
            config::kServoRoutineGripperPressMs,
            "servo_stored_gripper_close", "Fechando garra no objeto armazenado");
        appendMove(
            StepAction::MoveArm, config::kServoRoutineArmStoredObjectLiftDegrees,
            config::kServoRoutineArmStepMs,
            "servo_stored_arm_lift", "Elevando braço para 60°");
        appendMove(
            StepAction::MoveWrist, config::kServoRoutineWristForwardDegrees,
            config::kServoRoutineWristStepMs,
            "servo_stored_wrist_deposit", "Movendo pulso para 174°");
        appendMove(
            StepAction::MoveArm, config::kServoRoutineArmTransferDegrees,
            config::kServoRoutineArmStepMs,
            "servo_stored_arm_deposit", "Movendo braço para 10° no depósito");
        steps_.push_back({
            StepAction::WaitForOpenConfirmation, 0.0,
            config::kServoRoutineConfirmationTimeoutMs,
            "servo_wait_stored_open",
            "Aguardando confirmação para depositar o objeto armazenado"});
        appendMove(
            StepAction::MoveGripper, config::kServoRoutineGripperOpenDegrees,
            config::kServoRoutineGripperStepMs,
            "servo_stored_deposit_open", "Abrindo garra em 20° para depositar");
        steps_.push_back({StepAction::ClearInternalStorage});
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
