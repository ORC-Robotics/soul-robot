#include "obr/config.h"
#include "obr/servo_routine.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

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
}

int main()
{
    using Clock = std::chrono::steady_clock;
    try
    {
        ServoRoutine routine;
        Clock::time_point now{};

        const ServoPose nonZeroPose{105.0, 174.0, 20.0};
        ServoRoutineOutput output = routine.update(
            ServoRoutineKind::Initialize, 1, 0, nonZeroPose, now);
        require(output.poseRequested &&
                    closeTo(output.pose.armDegrees, 0.0) &&
                    closeTo(output.pose.wristDegrees, 0.0) &&
                    closeTo(output.pose.gripperDegrees, 0.0),
                "Initialization should apply the complete zero-degree pose");
        now += std::chrono::milliseconds(config::kServoRoutineInitialPoseMs);
        output = routine.update(
            ServoRoutineKind::Initialize, 1, 0, nonZeroPose, now);
        require(output.completed,
                "Initialization should complete after the zero pose settles");

        routine.resetExecution();
        output = routine.update(ServoRoutineKind::Capture, 2, 0, {}, now);
        require(output.poseRequested &&
                    closeTo(output.pose.armDegrees, 0.0) &&
                    closeTo(output.pose.wristDegrees, 0.0) &&
                    closeTo(output.pose.gripperDegrees, 0.0),
                "Capture should first reapply the previous pose");
        now += std::chrono::milliseconds(config::kServoRoutineResumePoseMs);
        output = routine.update(ServoRoutineKind::Capture, 2, 0, {}, now);
        require(closeTo(output.pose.armDegrees, 10.0),
                "Capture should move the arm after restoring the pose");
        now += std::chrono::milliseconds(config::kServoRoutineArmStepMs);
        output = routine.update(ServoRoutineKind::Capture, 2, 0, {}, now);
        require(output.poseRequested && closeTo(output.pose.wristDegrees, 174.0),
                "Capture should move the wrist after the arm step");
        now += std::chrono::milliseconds(config::kServoRoutineWristStepMs);
        output = routine.update(ServoRoutineKind::Capture, 2, 0, {}, now);
        require(output.poseRequested && closeTo(output.pose.gripperDegrees, 20.0),
                "Capture should open the gripper after positioning the wrist");
        now += std::chrono::milliseconds(config::kServoRoutineGripperStepMs);
        output = routine.update(ServoRoutineKind::Capture, 2, 0, {}, now);
        require(output.poseRequested && closeTo(output.pose.armDegrees, 105.0),
                "Capture should lower the arm only after opening the gripper");
        now += std::chrono::milliseconds(config::kServoRoutineArmStepMs);
        output = routine.update(ServoRoutineKind::Capture, 2, 0, {}, now);
        require(output.poseRequested && closeTo(output.pose.gripperDegrees, 3.0),
                "Capture should finish with the short three-degree press");
        now += std::chrono::milliseconds(config::kServoRoutineGripperPressMs);
        output = routine.update(ServoRoutineKind::Capture, 2, 0, {}, now);
        require(output.releaseGripper,
                "Capture should remove only the gripper signal after the press");
        now += std::chrono::milliseconds(config::kMainLoopPeriodMs * 2);
        output = routine.update(ServoRoutineKind::Capture, 2, 0, {}, now);
        require(output.completed,
                "Capture should complete after releasing the gripper signal");

        routine.resetExecution();
        const ServoPose capturedPose{105.0, 174.0, 3.0};
        output = routine.update(
            ServoRoutineKind::InternalStorage, 3, 0, capturedPose, now);
        require(closeTo(output.pose.armDegrees, 105.0) &&
                    closeTo(output.pose.wristDegrees, 174.0) &&
                    closeTo(output.pose.gripperDegrees, 3.0),
                "Storage should reapply the complete captured pose first");
        now += std::chrono::milliseconds(config::kServoRoutineResumePoseMs);
        output = routine.update(
            ServoRoutineKind::InternalStorage, 3, 0, capturedPose, now);
        require(closeTo(output.pose.armDegrees, 10.0) &&
                    closeTo(output.pose.gripperDegrees, 3.0),
                "Storage should keep the gripper closed while moving the arm");
        now += std::chrono::milliseconds(config::kServoRoutineArmStepMs);
        output = routine.update(
            ServoRoutineKind::InternalStorage, 3, 0, capturedPose, now);
        require(closeTo(output.pose.wristDegrees, 0.0),
                "Storage should point the wrist inward");
        now += std::chrono::milliseconds(config::kServoRoutineWristStepMs);
        output = routine.update(
            ServoRoutineKind::InternalStorage, 3, 0, capturedPose, now);
        require(closeTo(output.pose.gripperDegrees, 20.0),
                "Storage should release the object internally");
        now += std::chrono::milliseconds(config::kServoRoutineGripperStepMs);
        output = routine.update(
            ServoRoutineKind::InternalStorage, 3, 0, capturedPose, now);
        require(closeTo(output.pose.armDegrees, 105.0),
                "Storage should return the arm for another capture");
        now += std::chrono::milliseconds(config::kServoRoutineArmStepMs);
        output = routine.update(
            ServoRoutineKind::InternalStorage, 3, 0, capturedPose, now);
        require(output.waitingForConfirmation && output.internalObjectStored,
                "Storage should remember the object and wait to close on the next one");
        now += std::chrono::milliseconds(
            config::kServoRoutineConfirmationTimeoutMs * 2);
        output = routine.update(
            ServoRoutineKind::InternalStorage, 3, 0, capturedPose, now);
        require(output.waitingForConfirmation && !output.failed,
                "Storage closing request should wait without the deposit timeout");
        output = routine.update(
            ServoRoutineKind::InternalStorage, 3, 1, capturedPose, now);
        require(output.poseRequested && closeTo(output.pose.gripperDegrees, 3.0),
                "Storage confirmation should apply the short closing press");
        now += std::chrono::milliseconds(config::kServoRoutineGripperPressMs);
        output = routine.update(
            ServoRoutineKind::InternalStorage, 3, 1, capturedPose, now);
        require(output.releaseGripper,
                "Storage should remove gripper PWM after capturing the next object");
        now += std::chrono::milliseconds(config::kMainLoopPeriodMs * 2);
        output = routine.update(
            ServoRoutineKind::InternalStorage, 3, 1, capturedPose, now);
        require(output.completed && output.internalObjectStored,
                "Storage completion should retain the internal inventory state");

        routine.resetExecution();
        const ServoPose readyForNextCapture{105.0, 0.0, 20.0};
        output = routine.update(
            ServoRoutineKind::Deposit, 4, 1, readyForNextCapture, now);
        require(closeTo(output.pose.armDegrees, 105.0) &&
                    closeTo(output.pose.wristDegrees, 0.0) &&
                    closeTo(output.pose.gripperDegrees, 20.0),
                "Deposit should first restore the pose from the previous routine");
        now += std::chrono::milliseconds(config::kServoRoutineResumePoseMs);
        output = routine.update(
            ServoRoutineKind::Deposit, 4, 1, readyForNextCapture, now);
        now += std::chrono::milliseconds(config::kServoRoutineArmStepMs);
        output = routine.update(
            ServoRoutineKind::Deposit, 4, 1, readyForNextCapture, now);
        now += std::chrono::milliseconds(config::kServoRoutineWristStepMs);
        output = routine.update(
            ServoRoutineKind::Deposit, 4, 0, readyForNextCapture, now);
        require(output.waitingForConfirmation,
                "Deposit should wait before opening the first object");

        output = routine.update(
            ServoRoutineKind::Deposit, 4, 2, readyForNextCapture, now);
        require(output.poseRequested && closeTo(output.pose.gripperDegrees, 20.0),
                "Deposit confirmation should open the first object");

        const int storedPathDurations[] = {
            config::kServoRoutineGripperStepMs,
            config::kServoRoutineArmStepMs,
            config::kServoRoutineWristStepMs,
            config::kServoRoutineGripperStepMs,
            config::kServoRoutineArmStepMs,
            config::kServoRoutineGripperPressMs,
            config::kServoRoutineArmStepMs,
            config::kServoRoutineWristStepMs,
            config::kServoRoutineArmStepMs};
        for (const int durationMs : storedPathDurations)
        {
            now += std::chrono::milliseconds(durationMs);
            output = routine.update(
                ServoRoutineKind::Deposit, 4, 2, readyForNextCapture, now);
        }
        require(output.waitingForConfirmation && output.internalObjectStored,
                "Stored-object deposit should require a second opening confirmation");

        output = routine.update(
            ServoRoutineKind::Deposit, 4, 3, readyForNextCapture, now);
        require(output.poseRequested && closeTo(output.pose.gripperDegrees, 20.0),
                "Second confirmation should open the stored object");
        now += std::chrono::milliseconds(config::kServoRoutineGripperStepMs);
        output = routine.update(
            ServoRoutineKind::Deposit, 4, 3, readyForNextCapture, now);
        require(output.completed && !output.internalObjectStored,
                "Depositing the stored object should clear the inventory state");

        ServoRoutine timeoutRoutine;
        Clock::time_point timeoutNow{};
        output = timeoutRoutine.update(
            ServoRoutineKind::Deposit, 1, 0, capturedPose, timeoutNow);
        timeoutNow += std::chrono::milliseconds(config::kServoRoutineResumePoseMs);
        output = timeoutRoutine.update(
            ServoRoutineKind::Deposit, 1, 0, capturedPose, timeoutNow);
        timeoutNow += std::chrono::milliseconds(config::kServoRoutineArmStepMs);
        output = timeoutRoutine.update(
            ServoRoutineKind::Deposit, 1, 0, capturedPose, timeoutNow);
        timeoutNow += std::chrono::milliseconds(config::kServoRoutineWristStepMs);
        output = timeoutRoutine.update(
            ServoRoutineKind::Deposit, 1, 0, capturedPose, timeoutNow);
        require(output.waitingForConfirmation,
                "Deposit should expose its confirmation gate");
        timeoutNow += std::chrono::milliseconds(
            config::kServoRoutineConfirmationTimeoutMs);
        output = timeoutRoutine.update(
            ServoRoutineKind::Deposit, 1, 0, capturedPose, timeoutNow);
        require(output.failed && !output.poseRequested,
                "Deposit timeout should stop without opening the gripper");
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
