#include "obr/rescue_area_mission.h"

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

bool ballAlignmentFailed(const AutonomousStatus& status)
{
    return status.phase == "ball_alignment_target_lost_timeout" ||
           status.phase == "ball_alignment_camera_stale_timeout" ||
           status.phase == "ball_alignment_motion_timeout" ||
           status.phase == "ball_alignment_esp32_not_ready";
}
}

RescueAreaOutput RescueAreaMission::update(
    const ForwardBallSnapshot& forwardBallSnapshot,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    std::uint64_t autonomousRunSequence,
    bool rescueExitConfirmed,
    bool finishAfterVictim)
{
    RescueAreaOutput output;
    if (failed_)
    {
        output.failed = true;
        output.status = failureStatus_;
        return output;
    }

    if (!esp32Telemetry.readyForOperation())
    {
        reset();
        output.failed = true;
        output.status = makeStatus(
            "rescue_esp32_not_ready",
            "Resgate interrompido: ESP32 não está pronta");
        return output;
    }

    if (!victimReached_)
    {
        const BallAlignmentOutput alignment = ballAlignmentMission_.update(
            forwardBallSnapshot,
            esp32Telemetry,
            autonomousRunSequence);
        output.leftPower = alignment.leftPower;
        output.rightPower = alignment.rightPower;
        output.status = alignment.status;
        if (!alignment.finished)
        {
            return output;
        }
        if (ballAlignmentFailed(alignment.status) ||
            alignment.status.phase != "ball_reached")
        {
            failed_ = true;
            failureStatus_ = alignment.status;
            output.failed = true;
            return output;
        }
        victimReached_ = true;
    }

    if (finishAfterVictim || rescueExitConfirmed)
    {
        output.completed = true;
        output.status = makeStatus(
            rescueExitConfirmed ? "rescue_exit_confirmed" : "ball_reached",
            rescueExitConfirmed
                ? "Saída da área de resgate confirmada"
                : "Bola alcançada; teste isolado concluído",
            100.0);
        return output;
    }

    // A parada é intencional: o percurso de procura da saída ainda não possui
    // uma estratégia física validada e não deve ser substituído por movimento cego.
    output.status = makeStatus(
        "rescue_waiting_exit_program",
        "Vítima alcançada: aguardando a estratégia validada de saída");
    return output;
}

void RescueAreaMission::reset()
{
    ballAlignmentMission_.reset();
    victimReached_ = false;
    failed_ = false;
    failureStatus_ = {};
}
