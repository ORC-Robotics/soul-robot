#include "obr/servo_controller.h"

#include <cmath>
#include <iostream>
#include <limits>

namespace
{
bool require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << message << '\n';
        return false;
    }
    return true;
}
}

int main()
{
    bool ok = true;
    ok &= require(ServoController::isValidAngle(0.0),
                  "Zero degrees should be accepted");
    ok &= require(ServoController::isValidAngle(90.5),
                  "Angles inside the range should be accepted");
    ok &= require(ServoController::isValidAngle(180.0),
                  "180 degrees should be accepted");
    ok &= require(!ServoController::isValidAngle(-0.1),
                  "Negative angles should be rejected");
    ok &= require(!ServoController::isValidAngle(180.1),
                  "Angles above 180 degrees should be rejected");
    ok &= require(!ServoController::isValidAngle(
                      std::numeric_limits<double>::quiet_NaN()),
                  "NaN should be rejected");
    ok &= require(!ServoController::isValidAngle(
                      std::numeric_limits<double>::infinity()),
                  "Infinity should be rejected");
    ok &= require(ServoController::isValidCalibrationPulse(500),
                  "The lower absolute calibration pulse should be accepted");
    ok &= require(ServoController::isValidCalibrationPulse(2500),
                  "The upper absolute calibration pulse should be accepted");
    ok &= require(!ServoController::isValidCalibrationPulse(499) &&
                      !ServoController::isValidCalibrationPulse(2501),
                  "Calibration pulses outside the absolute range should be rejected");
    ok &= require(ServoController::isValidCalibrationEndpoints(700, 2300),
                  "A normal calibration span should be accepted");
    ok &= require(ServoController::isValidCalibrationEndpoints(2300, 700),
                  "Reversed endpoints should be accepted and represent inversion");
    ok &= require(!ServoController::isValidCalibrationEndpoints(1400, 1500),
                  "Endpoints that are too close should be rejected");
    ok &= require(std::abs(ServoController::moveAngleToward(0.0, 90.0, 1.8) - 1.8) < 0.001,
                  "The wrist ramp should limit a positive step");
    ok &= require(std::abs(ServoController::moveAngleToward(90.0, 0.0, 1.8) - 88.2) < 0.001,
                  "The wrist ramp should limit a negative step");
    ok &= require(std::abs(ServoController::moveAngleToward(89.0, 90.0, 1.8) - 90.0) < 0.001,
                  "The wrist ramp should stop exactly at the target");

    RobotState robotState;
    ok &= require(!robotState.setManualServoAngle(ServoId::Arm, 45.0),
                  "Stopped mode should reject manual servo commands");
    robotState.start();
    RobotSnapshot snapshot = robotState.snapshot();
    ok &= require(snapshot.armServoRequested && snapshot.wristServoRequested &&
                      snapshot.gripperServoRequested &&
                      std::abs(snapshot.servoPose.armDegrees) < 0.001 &&
                      std::abs(snapshot.servoPose.wristDegrees) < 0.001 &&
                      std::abs(snapshot.servoPose.gripperDegrees) < 0.001,
                  "Manual start should request the initial zero-degree pose");
    ok &= require(robotState.setManualServoAngle(ServoId::Arm, 45.0),
                  "Manual mode should accept a valid arm angle");
    snapshot = robotState.snapshot();
    ok &= require(snapshot.armServoRequested &&
                      std::abs(snapshot.servoPose.armDegrees - 45.0) < 0.001,
                  "RobotState should publish the requested arm angle");
    ok &= require(robotState.setManualServoAngle(ServoId::Gripper, 35.0),
                  "Manual mode should accept a gripper holding angle");
    robotState.enforceManualServoTimeout(std::chrono::milliseconds(-1));
    snapshot = robotState.snapshot();
    ok &= require(!snapshot.armServoRequested &&
                      !snapshot.wristServoRequested &&
                      !snapshot.gripperServoRequested,
                  "Manual servo commands should expire together");
    ok &= require(robotState.setManualServoAngle(ServoId::Wrist, 45.0),
                  "Manual servo control should recover after its timeout");
    snapshot = robotState.snapshot();
    ok &= require(snapshot.armServoRequested && snapshot.wristServoRequested &&
                      snapshot.gripperServoRequested &&
                      std::abs(snapshot.servoPose.gripperDegrees - 35.0) < 0.001,
                  "Moving the wrist after a timeout should restore the complete stored pose");
    robotState.enforceManualServoTimeout(std::chrono::milliseconds(-1));
    const unsigned long long sequenceAfterSecondTimeout =
        robotState.snapshot().servoCommandSequence;
    ok &= require(robotState.setManualServoAngle(ServoId::Wrist, 45.0),
                  "An unchanged wrist target should reactivate an expired pose");
    snapshot = robotState.snapshot();
    ok &= require(snapshot.armServoRequested && snapshot.wristServoRequested &&
                      snapshot.gripperServoRequested &&
                      snapshot.servoCommandSequence > sequenceAfterSecondTimeout,
                  "Reactivation should publish the complete pose even when the target is unchanged");
    robotState.stop();
    snapshot = robotState.snapshot();
    ok &= require(!snapshot.armServoRequested &&
                      !snapshot.wristServoRequested &&
                      !snapshot.gripperServoRequested,
                  "Stop should clear every servo request");
    ok &= require(robotState.beginServoCalibration(),
                  "Stopped mode should allow dedicated servo calibration");
    snapshot = robotState.snapshot();
    ok &= require(snapshot.servoCalibrationActive &&
                      snapshot.mode == "servo_calibration",
                  "Servo calibration should use its own visible robot mode");
    ok &= require(!robotState.setManualServoAngle(ServoId::Arm, 90.0),
                  "Normal angle commands should be rejected during calibration");
    robotState.stop();
    snapshot = robotState.snapshot();
    ok &= require(!snapshot.servoCalibrationActive &&
                      snapshot.mode == "stopped",
                  "Stop should end servo calibration immediately");

    robotState.startAutonomous();
    const ServoPose pose{120.0, 45.0, 20.0};
    ok &= require(robotState.setAutonomousServoPose(pose),
                  "Autonomous mode should accept a valid complete pose");
    snapshot = robotState.snapshot();
    ok &= require(snapshot.armServoRequested && snapshot.wristServoRequested &&
                      snapshot.gripperServoRequested,
                  "An autonomous pose should request all three servos");
    ok &= require(robotState.setAutonomousServoOutputEnabled(
                      ServoId::Gripper, false),
                  "Autonomous routines should be able to release the gripper output");
    snapshot = robotState.snapshot();
    ok &= require(snapshot.armServoRequested && snapshot.wristServoRequested &&
                      !snapshot.gripperServoRequested,
                  "Releasing the gripper should preserve arm and wrist requests");
    robotState.emergencyStop();
    snapshot = robotState.snapshot();
    ok &= require(!snapshot.armServoRequested &&
                      !snapshot.wristServoRequested &&
                      !snapshot.gripperServoRequested,
                  "Emergency stop should clear every servo request");

    RobotState confirmationState;
    confirmationState.setAutonomousMission(AutonomousMission::ServoDeposit);
    confirmationState.startAutonomous();
    ok &= require(!confirmationState.confirmServoRoutineAction(),
                  "Deposit confirmation should be rejected outside its waiting step");
    AutonomousStatus waitingStatus;
    waitingStatus.servoRoutineWaitingForConfirmation = true;
    confirmationState.updateAutonomousStatus(waitingStatus);
    const unsigned long long confirmationSequence =
        confirmationState.snapshot().servoRoutineConfirmationSequence;
    ok &= require(confirmationState.confirmServoRoutineAction() &&
                      confirmationState.snapshot().servoRoutineConfirmationSequence ==
                          confirmationSequence + 1,
                  "Deposit confirmation should publish exactly one request");
    confirmationState.setServoRoutineInternalObjectStored(true);
    confirmationState.stop();
    ok &= require(confirmationState.snapshot().servoRoutineInternalObjectStored,
                  "Internal storage state should persist between servo routines");

    RobotState chainedRoutineState;
    chainedRoutineState.start();
    chainedRoutineState.setManualServoAngle(ServoId::Arm, 105.0);
    chainedRoutineState.setManualServoAngle(ServoId::Wrist, 174.0);
    chainedRoutineState.setManualServoAngle(ServoId::Gripper, 3.0);
    chainedRoutineState.stop();
    chainedRoutineState.setAutonomousMission(AutonomousMission::ServoInternalStorage);
    chainedRoutineState.startAutonomous();
    snapshot = chainedRoutineState.snapshot();
    ok &= require(std::abs(snapshot.servoPose.armDegrees - 105.0) < 0.001 &&
                      std::abs(snapshot.servoPose.wristDegrees - 174.0) < 0.001 &&
                      std::abs(snapshot.servoPose.gripperDegrees - 3.0) < 0.001,
                  "Starting a chained servo routine should preserve the previous pose");
    return ok ? 0 : 1;
}
