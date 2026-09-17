#include "obr/main_mission.h"
#include "obr/config.h"

#include <cmath>
#include <iostream>

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

void MainMission::reset(bool startAtExit)
{
    phase_ = startAtExit ? Phase::ExitSearch : Phase::InitialLineCourse;
    lineCourseMission_.reset();
    silverEntryManeuver_.reset();
    silverSuppressedUntilClear_ = false;
    silverClearFrames_ = 0;
    lastSilverClearSequence_ = 0;
    rescueRoomMission_.reset();
    rescueExitMission_.reset();
    exitReferenceInitialized_ = false;
}

bool MainMission::requiresExitVision() const
{
    return phase_ == Phase::ExitSearch;
}

bool MainMission::requiresRescueVision() const
{
    return phase_ == Phase::RescueArea &&
           rescueRoomMission_.requiresBallDetection();
}

bool MainMission::requiresRescueZoneDetection() const
{
    return (phase_ == Phase::ExitSearch &&
            rescueExitMission_.requiresRescueZoneDetection()) ||
           (phase_ == Phase::RescueArea &&
            rescueRoomMission_.requiresRescueZoneDetection());
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
        (snapshot.autonomousMission != AutonomousMission::MainMission &&
         snapshot.autonomousMission != AutonomousMission::RescueExit))
    {
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
        // Verde, retorno de 180°, obstáculo e recuperações têm prioridade.
        // Uma confirmação de prata feita nesses frames não pode disparar depois.
        const bool obstacleNear = esp32Telemetry.sensorFresh &&
            esp32Telemetry.lastSensorAgeMs >= 0 &&
            esp32Telemetry.lastSensorAgeMs <= config::kObstacleUltrasonicFreshnessMs &&
            std::isfinite(esp32Telemetry.ultrasonicDistanceCm) &&
            esp32Telemetry.ultrasonicDistanceCm >= 2.0 &&
            esp32Telemetry.ultrasonicDistanceCm <=
                config::kObstacleDetectionDistanceCm;
        const bool lineManeuverPending = cameraLineSnapshot.sourceFresh &&
            cameraLineSnapshot.greenCandidateCount > 0;
        const bool suppressSilver = lineCourseMission_.maneuverActive() ||
            lineManeuverPending || obstacleNear;
        if (suppressSilver)
        {
            silverSuppressedUntilClear_ = true;
            silverClearFrames_ = 0;
            lastSilverClearSequence_ = cameraLineSnapshot.silverSequence;
            silverEntryManeuver_.reset();
        }
        else if (silverSuppressedUntilClear_ &&
                 cameraLineSnapshot.silverClassifierFresh)
        {
            if (cameraLineSnapshot.silverSequence < lastSilverClearSequence_)
            {
                // Reinício da câmera exige novas leituras, não a sequência antiga.
                lastSilverClearSequence_ = cameraLineSnapshot.silverSequence;
                silverClearFrames_ = 0;
            }
            else if (cameraLineSnapshot.silverSequence > lastSilverClearSequence_)
            {
                lastSilverClearSequence_ = cameraLineSnapshot.silverSequence;
                silverClearFrames_ =
                    !cameraLineSnapshot.silverCandidateDetected &&
                            !cameraLineSnapshot.courseMarkerConfirmed
                        ? silverClearFrames_ + 1
                        : 0;
                if (silverClearFrames_ >= config::kSilverAfterManeuverClearFrames)
                {
                    silverSuppressedUntilClear_ = false;
                }
            }
        }

        if (!silverSuppressedUntilClear_)
        {
            const SilverEntryOutput silverEntry = silverEntryManeuver_.update(
                cameraLineSnapshot,
                esp32Telemetry);
            if (silverEntry.completed)
            {
                // A faixa cinza entrega autoridade diretamente à rotina completa.
                // O primeiro avanço ainda aguarda o gate do YOLO publicar um frame.
                const bool enteredWithoutLine =
                    silverEntry.status.phase == "silver_entry_line_timeout";
                lineCourseMission_.reset();
                silverEntryManeuver_.reset();
                rescueRoomMission_.reset();
                phase_ = Phase::RescueArea;
                robotState.driveAutonomous(0.0, 0.0);
                robotState.updateAutonomousStatus(makeStatus(
                    "rescue_area_entering",
                    enteredWithoutLine
                        ? "Faixa cinza confirmada: resgate iniciado sem linha NEAR após 2,5 s"
                        : "Faixa cinza confirmada: ligando o YOLO para entrar no resgate"));
                if (enteredWithoutLine)
                {
                    std::cout << "Silver entry continued without NEAR line after 2500 ms" << std::endl;
                }
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
            // Recolhe somente o pulso após o resgate, preservando a pose atual
            // do braço e da garra e a proteção mecânica de RobotState.
            ServoPose exitPose = robotState.snapshot().servoPose;
            exitPose.wristDegrees = config::kServoRoutineWristInternalDegrees;
            robotState.setAutonomousServoPose(exitPose, true);
            phase_ = Phase::ExitSearch;
            rescueExitMission_.reset();
            rescueExitMission_.setTriangleReferenceHeading(
                rescueRoomMission_.lastTriangleHeadingDegrees());
            exitReferenceInitialized_ = true;
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeStatus(
                "rescue_exit_starting",
                "Varredura final concluída; iniciando a rota fixa da saída",
                0.0));
        }
        return;
    }

    if (phase_ == Phase::ExitSearch)
    {
        // O teste isolado reutiliza o heading salvo; sem ele, a saída captura o yaw inicial.
        if (!exitReferenceInitialized_)
        {
            rescueExitMission_.setTriangleReferenceHeading(snapshot.rescueZoneLockedHeading);
            exitReferenceInitialized_ = true;
        }
        const auto exit = rescueExitMission_.update(
            cameraLineSnapshot, forwardLineSnapshot, rescueZoneSnapshot,
            esp32Telemetry, autonomousRunSequence);
        robotState.driveAutonomous(exit.leftPower, exit.rightPower);
        if (exit.failed)
        {
            phase_ = Phase::Failed;
            robotState.stop();
        }
        else if (exit.completed)
        {
            phase_ = Phase::FinalLineCourse;
            lineCourseMission_.reset();
        }
        robotState.updateAutonomousStatus(exit.status);
        return;
    }

    lineCourseMission_.update(
        robotState,
        esp32Telemetry,
        cameraReady,
        cameraLineSnapshot,
        forwardLineSnapshot,
        phase_ == Phase::FinalLineCourse);
}
