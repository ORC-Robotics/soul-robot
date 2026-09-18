#include "obr/config.h"
#include "obr/servo_routine.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

bool closeTo(double actual, double expected)
{
    return std::abs(actual - expected) < 0.001;
}

struct ObservedStep
{
    std::string phase;
    ServoPose pose;
};

std::vector<ObservedStep> runRoutine(
    ServoRoutineKind kind,
    unsigned long long sequence,
    const ServoPose& initialPose,
    ServoRoutineOutput& finalOutput)
{
    ServoRoutine routine;
    auto now = std::chrono::steady_clock::time_point{};
    std::vector<ObservedStep> observed;
    finalOutput = routine.update(kind, sequence, 0, initialPose, now);

    for (int updateCount = 0;
         updateCount < 80 && !finalOutput.completed && !finalOutput.failed;
         ++updateCount)
    {
        require(!finalOutput.waitingForConfirmation,
                "Debug servo routines must not wait for mission conditions");
        require(!finalOutput.releaseGripper,
                "The gripper must keep an active holding angle");
        if (finalOutput.poseRequested)
        {
            observed.push_back({finalOutput.phase, finalOutput.pose});
        }
        now += std::chrono::seconds(10);
        finalOutput = routine.update(kind, sequence, 0, initialPose, now);
    }

    require(finalOutput.completed && !finalOutput.failed,
            "Servo routine should complete without external input");
    return observed;
}

const ServoPose& poseAt(
    const std::vector<ObservedStep>& steps,
    const std::string& phase)
{
    for (const ObservedStep& step : steps)
    {
        if (step.phase == phase)
        {
            return step.pose;
        }
    }
    throw std::runtime_error("Missing phase: " + phase);
}

void requireWristClearance(const std::vector<ObservedStep>& steps)
{
    for (std::size_t index = 1; index < steps.size(); ++index)
    {
        if (!closeTo(steps[index - 1].pose.wristDegrees,
                     steps[index].pose.wristDegrees))
        {
            require(
                steps[index - 1].pose.armDegrees >=
                    config::kServoRoutineArmHomeDegrees,
                "Wrist moved before the arm reached the 15-degree clearance");
        }
    }
}

void requirePhaseOrder(
    const std::vector<ObservedStep>& steps,
    const std::vector<std::string>& expectedPhases)
{
    std::size_t expectedIndex = 0;
    for (const ObservedStep& step : steps)
    {
        if (expectedIndex < expectedPhases.size() &&
            step.phase == expectedPhases[expectedIndex])
        {
            ++expectedIndex;
        }
    }
    require(expectedIndex == expectedPhases.size(),
            "Servo phases did not follow the required order");
}
}

int main()
{
    try
    {
        ServoRoutineOutput output;
        const ServoPose home{
            config::kServoRoutineArmHomeDegrees,
            config::kServoRoutineWristInternalDegrees,
            config::kServoRoutineGripperClosedDegrees};

        auto steps = runRoutine(ServoRoutineKind::Initialize, 1, {}, output);
        require(closeTo(steps.front().pose.armDegrees, 15.0) &&
                    closeTo(steps.front().pose.wristDegrees, 0.0) &&
                    closeTo(steps.front().pose.gripperDegrees, 0.0),
                "Initialize should apply the 15/0/0 home pose");

        steps = runRoutine(ServoRoutineKind::Capture, 2, home, output);
        require(closeTo(poseAt(steps, "servo_capture_gripper_open").gripperDegrees, 180.0) &&
                    closeTo(poseAt(steps, "servo_capture_arm_pickup").armDegrees, 103.0) &&
                    closeTo(poseAt(steps, "servo_capture_gripper_press").gripperDegrees, 0.0) &&
                    closeTo(poseAt(steps, "servo_capture_gripper_retention").gripperDegrees, 5.0) &&
                    closeTo(poseAt(steps, "servo_capture_arm_finish").armDegrees, 15.0),
                "Capture should use the new pickup and retention poses");
        requireWristClearance(steps);

        steps = runRoutine(ServoRoutineKind::PrepareCapture, 20, home, output);
        requirePhaseOrder(steps, {
            "servo_capture_arm_clearance",
            "servo_capture_wrist_forward",
            "servo_capture_gripper_open",
            "servo_capture_arm_pickup"});
        require(closeTo(steps.back().pose.armDegrees, 103.0) &&
                    closeTo(steps.back().pose.wristDegrees, 180.0) &&
                    closeTo(steps.back().pose.gripperDegrees, 180.0),
                "PrepareCapture must stop at the validated open pickup pose");

        const ServoPose prepared{
            config::kServoRoutineArmPickupDegrees,
            config::kServoRoutineWristForwardDegrees,
            config::kServoRoutineGripperFullyOpenDegrees};
        steps = runRoutine(ServoRoutineKind::GripForReverse, 30, prepared, output);
        for (const auto& step : steps)
        {
            require(closeTo(step.pose.armDegrees, 103.0) && closeTo(step.pose.wristDegrees, 180.0),
                    "Prender para a ré não pode mover braço ou pulso.");
        }
        requirePhaseOrder(steps, {"servo_capture_gripper_press", "servo_capture_gripper_retention"});
        require(closeTo(output.pose.gripperDegrees, 5.0), "A garra deve terminar em retenção.");
        const ServoPose retained = output.pose;
        steps = runRoutine(ServoRoutineKind::LiftAfterReverse, 31, retained, output);
        require(closeTo(output.pose.armDegrees, 15.0) && closeTo(output.pose.gripperDegrees, 5.0),
                "A elevação para armazenamento deve manter a vítima presa.");
        steps = runRoutine(ServoRoutineKind::LiftAfterReverseForDirectDeposit, 32, retained, output);
        require(closeTo(output.pose.armDegrees, 0.0) && closeTo(output.pose.gripperDegrees, 5.0),
                "A elevação direta deve conservar o destino de 0° e a retenção.");
        steps = runRoutine(ServoRoutineKind::SecureCapture, 21, prepared, output);
        requirePhaseOrder(steps, {
            "servo_capture_gripper_press",
            "servo_capture_gripper_retention",
            "servo_capture_arm_finish"});
        require(closeTo(steps.back().pose.armDegrees, 15.0) &&
                    closeTo(steps.back().pose.gripperDegrees, 5.0),
                "SecureCapture must retain the victim with the validated angles");
        require(closeTo(
                    poseAt(steps, "servo_capture_arm_finish").gripperDegrees,
                    config::kServoRoutineGripperRetentionDegrees),
                "The gripper must remain at 5 degrees while the arm rises");

        steps = runRoutine(
            ServoRoutineKind::SecureCaptureForDirectDeposit,
            23,
            prepared,
            output);
        require(closeTo(steps.back().pose.armDegrees, 0.0) &&
                    closeTo(steps.back().pose.gripperDegrees, 5.0),
                "Direct capture must preserve the validated 0-degree finish");

        const ServoPose captured{
            config::kServoRoutineArmHomeDegrees,
            config::kServoRoutineWristForwardDegrees,
            config::kServoRoutineGripperRetentionDegrees};
        steps = runRoutine(ServoRoutineKind::InternalStorage, 3, captured, output);
        require(closeTo(poseAt(steps, "servo_store_wrist_internal").wristDegrees, 0.0) &&
                    closeTo(poseAt(steps, "servo_store_wrist_internal").gripperDegrees, 5.0) &&
                    closeTo(poseAt(steps, "servo_store_gripper_release").gripperDegrees, 90.0) &&
                    closeTo(poseAt(steps, "servo_store_arm_clearance").armDegrees, 50.0) &&
                    closeTo(poseAt(steps, "servo_store_wrist_clearance").wristDegrees, 45.0) &&
                    closeTo(poseAt(steps, "servo_store_arm_transition").armDegrees, 20.0) &&
                    closeTo(output.pose.armDegrees, 20.0) &&
                    closeTo(output.pose.wristDegrees, 45.0) &&
                    closeTo(output.pose.gripperDegrees, 0.0) &&
                    output.internalObjectStored,
                "Storage must finish tucked before the next victim search");
        requireWristClearance(steps);

        const ServoPose blackCarried{
            config::kServoInitialAngleDegrees,
            config::kServoRoutineWristForwardDegrees,
            config::kServoRoutineGripperRetentionDegrees};
        steps = runRoutine(
            ServoRoutineKind::DepositCarriedKeepingStored,
            33,
            blackCarried,
            output);
        requirePhaseOrder(steps, {
            "servo_deposit_gripper_open",
            "servo_deposit_gripper_close",
            "servo_deposit_arm_ready_for_stored"});
        for (const auto& step : steps)
        {
            require(!closeTo(step.pose.wristDegrees,
                             config::kServoRoutineWristInternalDegrees),
                    "Com prata armazenada, o pulso não pode voltar a 0° após depositar a preta.");
        }
        require(closeTo(output.pose.armDegrees,
                        config::kServoRoutineArmHomeDegrees) &&
                    closeTo(output.pose.wristDegrees,
                            config::kServoRoutineWristForwardDegrees) &&
                    closeTo(output.pose.gripperDegrees,
                            config::kServoRoutineGripperClosedDegrees),
                "O depósito da preta deve terminar na pose da segunda prata.");
        requireWristClearance(steps);

        steps = runRoutine(ServoRoutineKind::FullSequence, 4, home, output);
        require(closeTo(poseAt(steps, "servo_second_gripper_open").gripperDegrees, 180.0) &&
                    closeTo(poseAt(steps, "servo_second_gripper_retention").gripperDegrees, 5.0) &&
                    closeTo(poseAt(steps, "servo_second_deposit").gripperDegrees, 90.0) &&
                    closeTo(poseAt(steps, "servo_stored_wrist_approach").wristDegrees, 65.0) &&
                    closeTo(poseAt(steps, "servo_stored_arm_approach").armDegrees, 65.0) &&
                    closeTo(poseAt(steps, "servo_stored_gripper_retention").gripperDegrees, 5.0) &&
                    closeTo(poseAt(steps, "servo_stored_arm_carry").armDegrees, 25.0) &&
                    closeTo(poseAt(steps, "servo_stored_deposit").gripperDegrees, 90.0) &&
                    closeTo(poseAt(steps, "servo_final_arm_home").armDegrees, 15.0) &&
                    closeTo(poseAt(steps, "servo_final_wrist_home").wristDegrees, 0.0) &&
                    closeTo(poseAt(steps, "servo_final_gripper_home").gripperDegrees, 0.0) &&
                    !output.internalObjectStored,
                "Full sequence should execute the complete storage path and finish home");
        requireWristClearance(steps);
        requirePhaseOrder(steps, {
            "servo_capture_arm_clearance",
            "servo_capture_wrist_forward",
            "servo_capture_gripper_open",
            "servo_capture_arm_pickup",
            "servo_capture_gripper_press",
            "servo_capture_gripper_retention",
            "servo_capture_arm_finish",
            "servo_store_wrist_internal",
            "servo_store_gripper_release",
            "servo_store_arm_clearance",
            "servo_store_wrist_clearance",
            "servo_store_gripper_close",
            "servo_store_arm_transition",
            "servo_store_wrist_forward",
            "servo_store_arm_ready",
            "servo_second_gripper_open",
            "servo_second_arm_pickup",
            "servo_second_gripper_press",
            "servo_second_gripper_retention",
            "servo_second_arm_carry",
            "servo_second_deposit",
            "servo_stored_wrist_approach",
            "servo_stored_arm_approach",
            "servo_stored_wrist_internal",
            "servo_stored_arm_pickup",
            "servo_stored_gripper_press",
            "servo_stored_gripper_retention",
            "servo_stored_arm_carry",
            "servo_stored_wrist_deposit",
            "servo_stored_deposit",
            "servo_final_arm_home",
            "servo_final_wrist_home",
            "servo_final_gripper_home"});

        const ServoPose secondCaptured{
            config::kServoRoutineArmHomeDegrees,
            config::kServoRoutineWristForwardDegrees,
            config::kServoRoutineGripperRetentionDegrees};
        steps = runRoutine(
            ServoRoutineKind::DepositCarriedAndStored,
            22,
            secondCaptured,
            output);
        requirePhaseOrder(steps, {
            "servo_second_deposit",
            "servo_stored_wrist_approach",
            "servo_stored_arm_approach",
            "servo_stored_wrist_internal",
            "servo_stored_arm_pickup",
            "servo_stored_gripper_press",
            "servo_stored_gripper_retention",
            "servo_stored_arm_carry",
            "servo_stored_wrist_deposit",
            "servo_stored_deposit",
            "servo_final_arm_home",
            "servo_final_wrist_home",
            "servo_final_gripper_home"});

        const ServoPose storedDeliveryStart{
            config::kServoRoutineArmHomeDegrees,
            config::kServoRoutineWristForwardDegrees,
            config::kServoRoutineGripperClosedDegrees};
        steps = runRoutine(
            ServoRoutineKind::DepositStored,
            34,
            storedDeliveryStart,
            output);
        requirePhaseOrder(steps, {
            "servo_stored_wrist_approach",
            "servo_stored_arm_approach",
            "servo_stored_wrist_internal",
            "servo_stored_arm_pickup",
            "servo_stored_gripper_press",
            "servo_stored_gripper_retention",
            "servo_stored_arm_carry",
            "servo_stored_wrist_deposit",
            "servo_stored_deposit"});

        steps = runRoutine(ServoRoutineKind::FullSequenceTwo, 5, home, output);
        require(closeTo(poseAt(steps, "servo_capture_gripper_open").gripperDegrees, 180.0) &&
                    closeTo(poseAt(steps, "servo_capture_gripper_retention").gripperDegrees, 5.0) &&
                    closeTo(poseAt(steps, "servo_capture_arm_finish").armDegrees, 0.0) &&
                    closeTo(poseAt(steps, "servo_direct_deposit").gripperDegrees, 90.0) &&
                    closeTo(poseAt(steps, "servo_direct_gripper_home").gripperDegrees, 0.0) &&
                    closeTo(poseAt(steps, "servo_direct_arm_home").armDegrees, 15.0) &&
                    closeTo(poseAt(steps, "servo_direct_wrist_home").wristDegrees, 0.0),
                "Full sequence two should deliver directly and finish home");
        requireWristClearance(steps);
        requirePhaseOrder(steps, {
            "servo_capture_arm_clearance",
            "servo_capture_wrist_forward",
            "servo_capture_gripper_open",
            "servo_capture_arm_pickup",
            "servo_capture_gripper_press",
            "servo_capture_gripper_retention",
            "servo_capture_arm_finish",
            "servo_direct_deposit",
            "servo_direct_gripper_home",
            "servo_direct_arm_home",
            "servo_direct_wrist_home"});
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
