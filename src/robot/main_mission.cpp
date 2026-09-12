#include "obr/main_mission.h"

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

}

void MainMission::reset()
{
    phase_ = Phase::InitialLineCourse;
    lineCourseMission_.reset();
    silverEntryManeuver_.reset();
    rescueRoomMission_.reset();
}

bool MainMission::requiresRescueVision() const
{
    return phase_ == Phase::RescueArea &&
           rescueRoomMission_.requiresBallDetection();
}

bool MainMission::requiresRescueZoneDetection() const
{
    return phase_ == Phase::RescueArea &&
           rescueRoomMission_.requiresRescueZoneDetection();
}

std::uint64_t MainMission::rescueBallTargetSequence(
    std::uint64_t autonomousRunSequence) const
{
    return rescueRoomMission_.ballTargetSequence(autonomousRunSequence);
}

const char* MainMission::rescueBallTargetType() const
{
    return rescueRoomMission_.ballTargetType();
}

void MainMission::update(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    bool cameraReady,
    const CameraLineSnapshot& cameraLineSnapshot,
    const ForwardLineSnapshot& forwardLineSnapshot)
{
    const RobotSnapshot snapshot = robotState.snapshot();
    update(
        robotState,
        esp32Telemetry,
        cameraReady,
        cameraLineSnapshot,
        forwardLineSnapshot,
        ForwardBallSnapshot{},
        RescueZoneSnapshot{},
        snapshot.autonomousRunSequence);
}

void MainMission::update(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    bool cameraReady,
    const CameraLineSnapshot& cameraLineSnapshot,
    const ForwardLineSnapshot& forwardLineSnapshot,
    const ForwardBallSnapshot& forwardBallSnapshot,
    std::uint64_t autonomousRunSequence)
{
    update(
        robotState,
        esp32Telemetry,
        cameraReady,
        cameraLineSnapshot,
        forwardLineSnapshot,
        forwardBallSnapshot,
        RescueZoneSnapshot{},
        autonomousRunSequence);
}

void MainMission::update(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    bool cameraReady,
    const CameraLineSnapshot& cameraLineSnapshot,
    const ForwardLineSnapshot& forwardLineSnapshot,
    const ForwardBallSnapshot& forwardBallSnapshot,
    const RescueZoneSnapshot& rescueZoneSnapshot,
    std::uint64_t autonomousRunSequence)
{
    const RobotSnapshot snapshot = robotState.snapshot();
    if (snapshot.mode != "autonomous" ||
        snapshot.autonomousMission != AutonomousMission::MainMission)
    {
        return;
    }

    if (phase_ == Phase::Completed)
    {
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeStatus(
            "main_mission_completed",
            "Missão concluída: resgate finalizado e robô parado",
            100.0));
        return;
    }
    if (phase_ == Phase::Failed)
    {
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeStatus(
            "main_mission_failed",
            "Missão interrompida com falha"));
        return;
    }

    if (phase_ == Phase::InitialLineCourse && cameraReady)
    {
        const SilverEntryOutput silverEntry = silverEntryManeuver_.update(
            cameraLineSnapshot,
            esp32Telemetry);
        if (silverEntry.completed)
        {
            // A faixa cinza entrega autoridade diretamente à rotina completa.
            // O primeiro avanço ainda aguarda o gate do YOLO publicar um frame.
            lineCourseMission_.reset();
            silverEntryManeuver_.reset();
            rescueRoomMission_.reset();
            phase_ = Phase::RescueArea;
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeStatus(
                "rescue_area_entering",
                "Faixa cinza confirmada: ligando o YOLO para entrar no resgate"));
            return;
        }
        if (silverEntry.hasControl)
        {
            lineCourseMission_.reset();
            robotState.driveAutonomous(
                silverEntry.leftPower,
                silverEntry.rightPower);
            robotState.updateAutonomousStatus(silverEntry.status);
            return;
        }
    }

    if (phase_ == Phase::RescueArea)
    {
        const RescueRoomOutput output = rescueRoomMission_.update(
            forwardBallSnapshot,
            rescueZoneSnapshot,
            esp32Telemetry,
            autonomousRunSequence,
            snapshot.servoRoutineConfirmationSequence,
            snapshot.servoPose);
        robotState.driveAutonomous(output.leftPower, output.rightPower);
        if (output.servoPoseRequested)
        {
            robotState.setAutonomousServoPose(output.servoPose);
        }
        if (output.releaseGripper)
        {
            robotState.setAutonomousServoOutputEnabled(ServoId::Gripper, false);
        }
        robotState.setServoRoutineInternalObjectStored(
            output.internalObjectStored);
        robotState.updateAutonomousStatus(output.status);
        if (output.failed)
        {
            phase_ = Phase::Failed;
            robotState.stop();
            // stop() aplica primeiro a saída segura e substitui o texto do
            // painel. Publicar a falha novamente preserva a causa específica.
            robotState.updateAutonomousStatus(output.status);
            return;
        }
        if (output.completed)
        {
            phase_ = Phase::Completed;
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeStatus(
                "main_mission_completed",
                "Resgate concluído: robô parado antes da futura busca da saída",
                100.0));
        }
        return;
    }

    lineCourseMission_.update(
        robotState,
        esp32Telemetry,
        cameraReady,
        cameraLineSnapshot,
        forwardLineSnapshot);
}
