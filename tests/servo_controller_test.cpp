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
    robotState.enforceManualServoTimeout(std::chrono::milliseconds(-1));
    snapshot = robotState.snapshot();
    ok &= require(!snapshot.armServoRequested,
                  "Manual servo commands should expire independently");
    ok &= require(robotState.setManualServoAngle(ServoId::Arm, 45.0),
                  "Manual servo control should recover after its timeout");
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
    robotState.emergencyStop();
    snapshot = robotState.snapshot();
    ok &= require(!snapshot.armServoRequested &&
                      !snapshot.wristServoRequested &&
                      !snapshot.gripperServoRequested,
                  "Emergency stop should clear every servo request");
    return ok ? 0 : 1;
}
