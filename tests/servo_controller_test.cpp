#include "obr/servo_controller.h"
#include "obr/camera_monitor.h"

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
    ok &= require(!snapshot.armServoRequested && !snapshot.wristServoRequested &&
                      !snapshot.gripperServoRequested,
                  "Manual start should preserve the current servo pose");
    ok &= require(!robotState.setManualServoAngle(ServoId::Wrist, 45.0),
                  "Manual mode should reject wrist motion below arm clearance");
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
    snapshot = robotState.snapshot();
    ok &= require(snapshot.armServoRequested && snapshot.wristServoRequested &&
                      snapshot.gripperServoRequested &&
                      std::abs(snapshot.servoPose.armDegrees -
                               config::kAutonomousInitialArmAngleDegrees) < 0.001 &&
                      std::abs(snapshot.servoPose.wristDegrees) < 0.001 &&
                      std::abs(snapshot.servoPose.gripperDegrees) < 0.001,
                  "Autonomous start should request the 15/0/0-degree pose");
    ok &= require(robotState.setAutonomousServoPose({0.0, 0.0, 0.0}),
                  "Autonomous mode should allow lowering the arm without wrist motion");
    ok &= require(!robotState.setAutonomousServoPose({0.0, 45.0, 0.0}),
                  "Autonomous mode should reject wrist motion below arm clearance");
    ok &= require(robotState.setAutonomousServoPose(
                      {config::kServoRoutineArmHomeDegrees, 0.0, 0.0}),
                  "Autonomous mode should allow restoring arm clearance first");
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
    chainedRoutineState.setManualServoAngle(
        ServoId::Wrist, config::kServoRoutineWristForwardDegrees);
    chainedRoutineState.setManualServoAngle(
        ServoId::Gripper, config::kServoRoutineGripperClosedDegrees);
    chainedRoutineState.stop();
    chainedRoutineState.setAutonomousMission(AutonomousMission::ServoInternalStorage);
    chainedRoutineState.startAutonomous();
    snapshot = chainedRoutineState.snapshot();
    ok &= require(
        std::abs(snapshot.servoPose.armDegrees -
                 config::kAutonomousInitialArmAngleDegrees) < 0.001 &&
            std::abs(snapshot.servoPose.wristDegrees) < 0.001 &&
            std::abs(snapshot.servoPose.gripperDegrees) < 0.001,
        "Starting a servo routine should first request the autonomous initial pose");
    RobotState waveState;
    waveState.setAutonomousMission(AutonomousMission::ServoWave);
    waveState.startAutonomous();
    waveState.driveAutonomous(0.7, 0.7);
    ok &= require(waveState.requestWaveBonus(),
                  "A missão autônoma deve aceitar um bônus solicitado uma vez.");
    waveState.driveAutonomous(0.7, 0.7);
    snapshot = waveState.snapshot();
    ok &= require(snapshot.waveBonusRequested && snapshot.left == 0.0 &&
                      snapshot.right == 0.0,
                  "O bônus deve bloquear qualquer novo comando de tração.");
    ok &= require(waveState.setWaveBonusServoPose({105.0, 30.0, 0.0}),
                  "O bônus deve permitir a pose dos servos no modo autônomo.");
    waveState.emergencyStop();
    snapshot = waveState.snapshot();
    ok &= require(!snapshot.waveBonusRequested && snapshot.left == 0.0 &&
                      snapshot.right == 0.0 && !snapshot.armServoRequested &&
                      !waveState.setWaveBonusServoPose({15.0, 0.0, 0.0}),
                  "A emergência deve cancelar o gesto e desligar os servos.");

    RobotState finishWaveState;
    finishWaveState.setAutonomousMission(AutonomousMission::MainMission);
    finishWaveState.startAutonomous();
    CameraLineSnapshot red;
    red.sourceFresh = true;
    red.redValid = true;
    red.redConfirmed = true;
    red.redRatio = 0.2;
    red.lineSequence = 1;
    red.lineTimestamp = 1.0;
    RobotState overlappingWaveState;
    overlappingWaveState.setAutonomousMission(AutonomousMission::MainMission);
    overlappingWaveState.startAutonomous();
    ok &= require(overlappingWaveState.requestWaveBonus() &&
                      !overlappingWaveState.observeRedFinish(red) &&
                      !overlappingWaveState.snapshot().missionFinished,
                  "A chegada deve aguardar o fim de um tchauzinho já iniciado.");
    overlappingWaveState.completeWaveBonus();
    ok &= require(overlappingWaveState.observeRedFinish(red),
                  "A mesma faixa vermelha deve ser aceita ao concluir o bônus.");
    ok &= require(finishWaveState.observeRedFinish(red),
                  "O marcador vermelho deve encerrar a missão e bloquear a tração.");
    ok &= require(finishWaveState.requestWaveBonus(),
                  "Um bônus solicitado após a chegada deve ativar somente os servos.");
    finishWaveState.driveAutonomous(0.8, 0.8);
    ok &= require(finishWaveState.setWaveBonusServoPose({105.0, 30.0, 0.0}) &&
                      !finishWaveState.setAutonomousServoPose({105.0, 30.0, 0.0}),
                  "A chegada deve aceitar apenas a pose do bônus, nunca a autoridade comum.");
    snapshot = finishWaveState.snapshot();
    ok &= require(snapshot.missionFinished && snapshot.mode == "stopped" &&
                      snapshot.left == 0.0 && snapshot.right == 0.0 &&
                      !finishWaveState.tryStartAutonomous(),
                  "O bônus da chegada não pode reativar motores ou nova missão.");
    finishWaveState.stop();
    snapshot = finishWaveState.snapshot();
    ok &= require(snapshot.missionFinished && !snapshot.waveBonusRequested &&
                      !snapshot.armServoRequested,
                  "Parar deve cancelar o bônus sem destravar a chegada.");

    return ok ? 0 : 1;
}
