#include "obr/robot_state.h"

#include "obr/config.h"

#include <algorithm>
#include <cmath>

namespace
{
double clampMotorCommand(double command)
{
    if (!std::isfinite(command))
    {
        return 0.0;
    }

    return std::clamp(command, config::kMinMotorOutput, config::kMaxMotorOutput);
}
}

const char* autonomousMissionName(AutonomousMission mission)
{
    switch (mission)
    {
    case AutonomousMission::DriveDistance:
        return "drive_distance";
    case AutonomousMission::TurnRight90:
        return "turn_right_90";
    case AutonomousMission::MainMission:
    default:
        return "main_mission";
    }
}

RobotSnapshot RobotState::snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

void RobotState::start()
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_.emergencyStop = false;
    state_.mode = "manual";
    state_.left = 0.0;
    state_.right = 0.0;
    state_.rawMotorCommand = false;
    state_.autonomousStatus = {"manual", "Controle manual ativo"};
    lastCommand_ = std::chrono::steady_clock::now();
}

void RobotState::startAutonomous()
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_.emergencyStop = false;
    state_.mode = "autonomous";
    state_.left = 0.0;
    state_.right = 0.0;
    state_.rawMotorCommand = false;
    ++state_.autonomousRunSequence;
    state_.autonomousStatus = {"starting", "Inicializando missão"};
    lastCommand_ = std::chrono::steady_clock::now();
}

bool RobotState::tryStartAutonomous()
{
    std::lock_guard<std::mutex> lock(mutex_);

    // O botão físico não libera E-Stop. A condição é revalidada dentro
    // do mutex para que uma emergência concorrente nunca seja apagada pela partida.
    if (state_.emergencyStop || state_.mode != "stopped")
    {
        return false;
    }

    state_.mode = "autonomous";
    state_.left = 0.0;
    state_.right = 0.0;
    state_.rawMotorCommand = false;
    ++state_.autonomousRunSequence;
    state_.autonomousStatus = {"starting", "Inicializando missão"};
    lastCommand_ = std::chrono::steady_clock::now();
    return true;
}

void RobotState::setAutonomousMission(AutonomousMission mission)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // Trocar a missão sempre para o robô. Isso impede que uma nova estratégia
    // assuma os motores no meio de um movimento iniciado pela missão anterior.
    state_.mode = state_.emergencyStop ? "emergency" : "stopped";
    state_.left = 0.0;
    state_.right = 0.0;
    state_.rawMotorCommand = false;
    state_.autonomousMission = mission;
    state_.autonomousStatus = {"ready", "Missão selecionada e pronta"};
    lastCommand_ = std::chrono::steady_clock::now();
}

bool RobotState::setDriveDistanceTargetCm(double targetCm)
{
    if (!std::isfinite(targetCm) ||
        targetCm < config::kDriveDistanceMinimumTargetCm ||
        targetCm > config::kDriveDistanceMaximumTargetCm)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    state_.driveDistanceTargetCm = targetCm;
    return true;
}

void RobotState::stop()
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_.mode = "stopped";
    state_.left = 0.0;
    state_.right = 0.0;
    state_.rawMotorCommand = false;
    state_.autonomousStatus = {"stopped", "Missão parada"};
    lastCommand_ = std::chrono::steady_clock::now();
}

void RobotState::emergencyStop()
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_.mode = "emergency";
    state_.emergencyStop = true;
    state_.left = 0.0;
    state_.right = 0.0;
    state_.rawMotorCommand = false;
    state_.autonomousStatus = {"emergency", "Parada de emergência ativa"};
    lastCommand_ = std::chrono::steady_clock::now();
}

void RobotState::drive(double left, double right)
{
    std::lock_guard<std::mutex> lock(mutex_);
    lastCommand_ = std::chrono::steady_clock::now();

    if (state_.emergencyStop || state_.mode != "manual")
    {
        // Comandos de movimento só são aceitos depois do Start.
        // Isso impede que o dashboard tire o robô do modo parado por acidente.
        state_.left = 0.0;
        state_.right = 0.0;
        state_.rawMotorCommand = false;
        return;
    }

    state_.left = clampMotorCommand(left);
    state_.right = clampMotorCommand(right);
    state_.rawMotorCommand = false;
}

void RobotState::driveRawDiagnostic(double left, double right)
{
    std::lock_guard<std::mutex> lock(mutex_);
    lastCommand_ = std::chrono::steady_clock::now();

    if (state_.emergencyStop || state_.mode != "manual")
    {
        // O diagnóstico direto continua bloqueado fora do modo manual para não
        // permitir que o dashboard contorne a parada ou o E-Stop.
        state_.left = 0.0;
        state_.right = 0.0;
        state_.rawMotorCommand = false;
        return;
    }

    state_.left = clampMotorCommand(left);
    state_.right = clampMotorCommand(right);
    state_.rawMotorCommand = true;
}

void RobotState::driveAutonomous(double left, double right)
{
    std::lock_guard<std::mutex> lock(mutex_);
    lastCommand_ = std::chrono::steady_clock::now();

    if (state_.emergencyStop || state_.mode != "autonomous")
    {
        // O controlador autônomo só pode mover o robô quando o modo autônomo
        // foi ativado explicitamente pelo dashboard.
        return;
    }

    state_.left = clampMotorCommand(left);
    state_.right = clampMotorCommand(right);
    state_.rawMotorCommand = false;
}

void RobotState::updateAutonomousStatus(const AutonomousStatus& status)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // Uma atualização atrasada do controlador autônomo não pode substituir no painel
    // um estado manual, parado ou de emergência que acabou de ser solicitado.
    const bool terminalMissionStatus = status.phase == "completed" ||
                                       status.phase == "turn_timeout" ||
                                       status.phase == "turn_imu_lost" ||
                                       status.phase == "turn_correction_failed" ||
                                       status.phase == "distance_completed" ||
                                       status.phase == "distance_timeout" ||
                                       status.phase == "distance_encoder_lost" ||
                                       status.phase == "distance_encoder_stall" ||
                                       status.phase == "distance_encoder_mismatch" ||
                                       status.phase == "distance_correction_failed" ||
                                       status.phase == "distance_invalid_target" ||
                                       status.phase == "esp32_not_ready" ||
                                       status.phase == "camera_not_ready" ||
                                       status.phase == "line_ipc_stale";
    if (state_.mode != "autonomous" && !terminalMissionStatus)
    {
        return;
    }
    state_.autonomousStatus = status;

    // Evita que valores inválidos prejudiquem o JSON enviado continuamente ao dashboard.
    if (!std::isfinite(state_.autonomousStatus.progressPercent))
    {
        state_.autonomousStatus.progressPercent = 0.0;
    }
    state_.autonomousStatus.progressPercent = std::clamp(
        state_.autonomousStatus.progressPercent, 0.0, 100.0);

    if (!std::isfinite(state_.autonomousStatus.targetDistanceCm))
    {
        state_.autonomousStatus.targetDistanceCm = 0.0;
    }
    if (!std::isfinite(state_.autonomousStatus.leftDistanceCm))
    {
        state_.autonomousStatus.leftDistanceCm = 0.0;
    }
    if (!std::isfinite(state_.autonomousStatus.rightDistanceCm))
    {
        state_.autonomousStatus.rightDistanceCm = 0.0;
    }
    if (!std::isfinite(state_.autonomousStatus.averageDistanceCm))
    {
        state_.autonomousStatus.averageDistanceCm = 0.0;
    }
}

void RobotState::enforceCommandTimeout(std::chrono::milliseconds timeout)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto age = std::chrono::steady_clock::now() - lastCommand_;

    if ((state_.mode == "manual" || state_.mode == "autonomous") && age > timeout)
    {
        state_.left = 0.0;
        state_.right = 0.0;
        state_.rawMotorCommand = false;
    }
}
