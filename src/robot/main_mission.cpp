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

bool confirmedMarker(
    const CameraLineSnapshot& cameraLineSnapshot,
    CourseMarker expectedMarker)
{
    return cameraLineSnapshot.sourceFresh &&
           cameraLineSnapshot.courseMarkerConfirmed &&
           cameraLineSnapshot.courseMarker == expectedMarker;
}
}

void MainMission::reset()
{
    phase_ = Phase::InitialLineCourse;
    lineCourseMission_.reset();
    rescueAreaMission_.reset();
}

bool MainMission::requiresRescueVision() const
{
    return phase_ == Phase::RescueArea;
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
            "Missão concluída: faixa vermelha confirmada",
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

    if (phase_ == Phase::InitialLineCourse &&
        cameraReady &&
        confirmedMarker(cameraLineSnapshot, CourseMarker::Gray))
    {
        // A faixa cinza termina o primeiro percurso. O resgate começa parado
        // para que o gate da visão frontal seja aberto no próximo ciclo.
        lineCourseMission_.reset();
        rescueAreaMission_.reset();
        phase_ = Phase::RescueArea;
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeStatus(
            "rescue_area_entering",
            "Faixa cinza confirmada: iniciando a área de resgate"));
        return;
    }

    if (phase_ == Phase::RescueArea)
    {
        const RescueAreaOutput output = rescueAreaMission_.update(
            forwardBallSnapshot,
            esp32Telemetry,
            autonomousRunSequence,
            cameraLineSnapshot.sourceFresh &&
                cameraLineSnapshot.rescueExitConfirmed,
            false);
        robotState.driveAutonomous(output.leftPower, output.rightPower);
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
            // O segundo percurso recebe estado limpo; nenhuma busca, direção
            // verde ou manobra anterior pode atravessar a saída do resgate.
            rescueAreaMission_.reset();
            lineCourseMission_.reset();
            phase_ = Phase::FinalLineCourse;
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeStatus(
                "final_line_course_entering",
                "Saída do resgate confirmada: retomando o percurso de linha"));
        }
        return;
    }

    if (phase_ == Phase::FinalLineCourse &&
        cameraReady &&
        confirmedMarker(cameraLineSnapshot, CourseMarker::Red))
    {
        lineCourseMission_.reset();
        phase_ = Phase::Completed;
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeStatus(
            "main_mission_completed",
            "Faixa vermelha confirmada: missão concluída",
            100.0));
        return;
    }

    lineCourseMission_.update(
        robotState,
        esp32Telemetry,
        cameraReady,
        cameraLineSnapshot,
        forwardLineSnapshot);
}
