#include "obr/robot_state.h"

#include "obr/config.h"
#include "obr/camera_monitor.h"
#include <iostream>

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

bool validServoAngle(double angleDegrees)
{
    return std::isfinite(angleDegrees) &&
           angleDegrees >= config::kServoMinimumAngleDegrees &&
           angleDegrees <= config::kServoMaximumAngleDegrees;
}

bool sameServoAngle(double firstDegrees, double secondDegrees)
{
    return std::abs(firstDegrees - secondDegrees) < 0.001;
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
    case AutonomousMission::RescueZoneDetection:
        return "rescue_zone_detection";
    case AutonomousMission::RescueZoneSearch:
        return "rescue_zone_search";
    case AutonomousMission::RescueZoneAlign:
        return "rescue_zone_align";
    case AutonomousMission::RescueZoneApproach:
        return "rescue_zone_approach";
    case AutonomousMission::RescueZoneTriangle:
        return "rescue_zone_triangle";
    case AutonomousMission::RescueExit:
        return "rescue_exit";
    case AutonomousMission::RescueExitWithReverse:
        return "rescue_exit_with_reverse";
    case AutonomousMission::RescueCornerYawTest:
        return "rescue_corner_yaw_test";
    case AutonomousMission::RescueArea:
        return "rescue_area";
    case AutonomousMission::ObstacleAvoidance:
        return "obstacle_avoidance";
    case AutonomousMission::ServoInitialize:
        return "servo_initialize";
    case AutonomousMission::ServoCapture:
        return "servo_capture";
    case AutonomousMission::ServoInternalStorage:
        return "servo_internal_storage";
    case AutonomousMission::ServoDeposit:
        return "servo_deposit";
    case AutonomousMission::ServoFullSequence:
        return "servo_full_sequence";
    case AutonomousMission::ServoFullSequenceTwo:
        return "servo_full_sequence_two";
    case AutonomousMission::MainMission:
    default:
        return "main_mission";
    }
}

const char* rescueZoneTargetColorName(RescueZoneTargetColor color)
{
    return color == RescueZoneTargetColor::Red ? "red" : "green";
}

RobotSnapshot RobotState::snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    RobotSnapshot snapshot = state_;
    snapshot.commandAgeMs =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - lastCommand_)
            .count();
    snapshot.commandTimedOut =
        (snapshot.mode == "manual" || snapshot.mode == "autonomous") &&
        snapshot.commandAgeMs > config::kCommandTimeoutMs;
    return snapshot;
}

bool RobotState::observeRedFinish(const CameraLineSnapshot& camera)
{
    std::lock_guard<std::mutex> lock(mutex_);
    // O teste pode começar sobre o triângulo vermelho e comanda apenas a ré
    // calibrada e os pivôs dos yaws de referência.
    // A chegada vermelha do percurso não deve encerrar essa calibração.
    if (state_.mode == "autonomous" &&
        (state_.autonomousMission == AutonomousMission::RescueCornerYawTest ||
         state_.autonomousMission == AutonomousMission::RescueExitWithReverse))
        return false;
    state_.redValid = camera.sourceFresh && camera.redValid &&
        std::isfinite(camera.redRatio) && camera.redRatio >= 0.0 && camera.redRatio <= 1.0;
    if (!state_.redValid || !std::isfinite(camera.lineTimestamp) ||
        camera.lineTimestamp <= lastRedTimestamp_ || camera.lineSequence == 0 ||
        camera.lineSequence == lastRedSequence_)
        return false;
    lastRedTimestamp_ = camera.lineTimestamp;
    lastRedSequence_ = camera.lineSequence;
    state_.redRatio = camera.redRatio;
    if (!redFinishArmed_)
    {
        if (camera.redClearConfirmed && !camera.redConfirmed &&
            camera.redRatio < config::kRedFinishMinRatio)
        {
            redFinishArmed_ = true;
            std::cout << "Red finish detection rearmed after leaving the marker\n";
        }
        return false;
    }
    if (!camera.redConfirmed || camera.redClearConfirmed ||
        camera.redRatio < config::kRedFinishMinRatio || state_.missionFinished)
        return false;

    // Durante o desvio, o vermelho continua sendo processado, mas não pode
    // encerrar a missão antes que a manobra devolva o controle.
    const bool obstacleAvoidanceActive =
        state_.autonomousStatus.phase.rfind("obstacle_", 0) == 0;
    if (state_.mode == "autonomous" && obstacleAvoidanceActive)
        return false;

    // A área de resgate não possui chegada vermelha. Essa leitura continua
    // atualizada, mas só pode encerrar a missão depois do retorno ao percurso.
    if (state_.autonomousStatus.phase.rfind("rescue_", 0) == 0)
        return false;

    // A trava é independente do E-Stop e cancela a execução inteira sob o mutex.
    // O STOP físico é enviado pelo runtime antes de qualquer alerta ou missão.
    redFinishArmed_ = false;
    state_.missionFinished = true;
    state_.mode = state_.emergencyStop ? "emergency" : "stopped";
    state_.left = state_.right = 0.0;
    state_.rawMotorCommand = false;
    state_.encoderSynchronizationAllowed = true;
    state_.servoCalibrationActive = false;
    disableServosLocked();
    state_.autonomousStatus = {"mission_finished", "Chegada confirmada: Vermelho", 100.0};
    lastCommand_ = std::chrono::steady_clock::now();
    std::cout << "Mission finished: red marker confirmed; motors stopped\n";
    return true;
}

void RobotState::start()
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_.missionFinished = false;
    state_.emergencyStop = false;
    state_.mode = "manual";
    state_.left = 0.0;
    state_.right = 0.0;
    state_.rawMotorCommand = false;
    state_.encoderSynchronizationAllowed = true;
    state_.servoCalibrationActive = false;
    // Entrar no modo Manual não publica uma pose. Assim, os servos conservam
    // a posição atual até que o operador envie um comando explícito.
    lastManualServoCommand_ = std::chrono::steady_clock::now();
    state_.autonomousStatus = {"manual", "Controle manual ativo"};
    lastCommand_ = std::chrono::steady_clock::now();
}

void RobotState::startAutonomous()
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_.missionFinished = false;
    state_.emergencyStop = false;
    state_.mode = "autonomous";
    state_.left = 0.0;
    state_.right = 0.0;
    state_.rawMotorCommand = false;
    state_.encoderSynchronizationAllowed = true;
    state_.servoCalibrationActive = false;
    if (state_.autonomousMission == AutonomousMission::RescueZoneDetection ||
        state_.autonomousMission == AutonomousMission::RescueZoneSearch ||
        state_.autonomousMission == AutonomousMission::RescueZoneAlign ||
        state_.autonomousMission == AutonomousMission::RescueZoneApproach ||
        state_.autonomousMission == AutonomousMission::RescueZoneTriangle ||
        state_.autonomousMission == AutonomousMission::RescueExitWithReverse ||
        state_.autonomousMission == AutonomousMission::RescueCornerYawTest)
    {
        // Estes modos de teste não precisam mover os servos.
        // Mantê-los desligados evita movimentos mecânicos não solicitados.
        disableServosLocked();
    }
    else
    {
        requestInitialServoPoseLocked();
    }
    ++state_.autonomousRunSequence;
    state_.autonomousStatus = {"starting", "Inicializando missão"};
    lastCommand_ = std::chrono::steady_clock::now();
}

bool RobotState::tryStartAutonomous()
{
    std::lock_guard<std::mutex> lock(mutex_);

    // O botão físico não libera E-Stop. A condição é revalidada dentro
    // do mutex para que uma emergência concorrente nunca seja apagada pela partida.
    if (state_.emergencyStop || state_.mode != "stopped" ||
        state_.servoCalibrationActive)
    {
        return false;
    }

    state_.missionFinished = false;
    state_.mode = "autonomous";
    state_.left = 0.0;
    state_.right = 0.0;
    state_.rawMotorCommand = false;
    state_.encoderSynchronizationAllowed = true;
    state_.servoCalibrationActive = false;
    if (state_.autonomousMission == AutonomousMission::RescueZoneDetection ||
        state_.autonomousMission == AutonomousMission::RescueZoneSearch ||
        state_.autonomousMission == AutonomousMission::RescueZoneAlign ||
        state_.autonomousMission == AutonomousMission::RescueZoneApproach ||
        state_.autonomousMission == AutonomousMission::RescueZoneTriangle ||
        state_.autonomousMission == AutonomousMission::RescueExitWithReverse ||
        state_.autonomousMission == AutonomousMission::RescueCornerYawTest)
    {
        // A partida física aplica a mesma condição segura do dashboard:
        // os servos permanecem desligados nesses modos de teste.
        disableServosLocked();
    }
    else
    {
        requestInitialServoPoseLocked();
    }
    ++state_.autonomousRunSequence;
    state_.autonomousStatus = {"starting", "Inicializando missão"};
    lastCommand_ = std::chrono::steady_clock::now();
    return true;
}

void RobotState::setAutonomousMission(AutonomousMission mission)
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_.missionFinished = false;

    // Trocar a missão sempre para o robô. Isso impede que uma nova estratégia
    // assuma os motores no meio de um movimento iniciado pela missão anterior.
    state_.mode = state_.emergencyStop ? "emergency" : "stopped";
    state_.left = 0.0;
    state_.right = 0.0;
    state_.rawMotorCommand = false;
    state_.encoderSynchronizationAllowed = true;
    state_.servoCalibrationActive = false;
    disableServosLocked();
    state_.autonomousMission = mission;
    if (mission == AutonomousMission::RescueZoneAlign ||
        mission == AutonomousMission::RescueZoneTriangle)
    {
        // Uma nova tentativa de alinhamento invalida o heading anterior.
        // Somente uma conclusão nova pode liberar o APPROACH_ZONE novamente.
        state_.rescueZoneLockedHeading =
            std::numeric_limits<double>::quiet_NaN();
    }
    state_.autonomousStatus = {"ready", "Missão selecionada e pronta"};
    lastCommand_ = std::chrono::steady_clock::now();
}

bool RobotState::setRescueTestDeliveries(int alive, int dead)
{
    std::lock_guard<std::mutex> lock(mutex_);
    // A simulação só é aceita com os motores parados. Não altera servos,
    // E-Stop nem autoriza a partida autônoma.
    if (state_.mode != "stopped" || state_.emergencyStop ||
        alive < 0 || alive > 2 || dead < 0 || dead > 1)
    {
        return false;
    }
    state_.rescueDeliveredAliveVictims = alive;
    state_.rescueDeliveredDeadVictims = dead;
    state_.rescueTestMemoryActive = alive != 0 || dead != 0;
    return true;
}

void RobotState::recordRescueDeliveries(int alive, int dead)
{
    std::lock_guard<std::mutex> lock(mutex_);
    // Só entregas confirmadas pela missão podem aumentar a memória real.
    if (state_.mode == "autonomous" &&
        state_.autonomousMission == AutonomousMission::MainMission &&
        alive >= state_.rescueDeliveredAliveVictims &&
        dead >= state_.rescueDeliveredDeadVictims)
    {
        state_.rescueDeliveredAliveVictims = alive;
        state_.rescueDeliveredDeadVictims = dead;
    }
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

void RobotState::setRescueZoneTargetColor(RescueZoneTargetColor color)
{
    std::lock_guard<std::mutex> lock(mutex_);
    state_.missionFinished = false;
    // Alterar o alvo para o robô antes da próxima partida. Assim, uma manobra
    // iniciada para uma cor nunca continua usando a seleção nova no meio do giro.
    state_.mode = state_.emergencyStop ? "emergency" : "stopped";
    state_.left = 0.0;
    state_.right = 0.0;
    state_.rawMotorCommand = false;
    state_.encoderSynchronizationAllowed = true;
    state_.servoCalibrationActive = false;
    disableServosLocked();
    state_.rescueZoneTargetColor = color;
    state_.autonomousStatus = {"ready", "Cor alvo da zona selecionada"};
    lastCommand_ = std::chrono::steady_clock::now();
}

bool RobotState::setRescueZoneLockedHeading(double headingDegrees)
{
    if (!std::isfinite(headingDegrees))
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.emergencyStop || state_.mode != "autonomous" ||
        (state_.autonomousMission != AutonomousMission::RescueZoneAlign &&
         state_.autonomousMission != AutonomousMission::RescueZoneTriangle))
    {
        return false;
    }

    // O heading validado pelo ALIGN_ZONE permanece disponível após Stop e
    // seleção do APPROACH_ZONE. Nenhuma outra missão pode sobrescrevê-lo.
    state_.rescueZoneLockedHeading = headingDegrees;
    return true;
}

void RobotState::stop()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.missionFinished) return;
    state_.mode = "stopped";
    state_.left = 0.0;
    state_.right = 0.0;
    state_.rawMotorCommand = false;
    state_.encoderSynchronizationAllowed = true;
    state_.servoCalibrationActive = false;
    disableServosLocked();
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
    state_.encoderSynchronizationAllowed = true;
    state_.servoCalibrationActive = false;
    disableServosLocked();
    state_.autonomousStatus = {"emergency", "Parada de emergência ativa"};
    lastCommand_ = std::chrono::steady_clock::now();
}

void RobotState::drive(double left, double right)
{
    std::lock_guard<std::mutex> lock(mutex_);
    lastCommand_ = std::chrono::steady_clock::now();

    if (state_.missionFinished || state_.emergencyStop || state_.mode != "manual")
    {
        // Comandos de movimento só são aceitos depois do Start.
        // Isso impede que o dashboard tire o robô do modo parado por acidente.
        state_.left = 0.0;
        state_.right = 0.0;
        state_.rawMotorCommand = false;
        state_.encoderSynchronizationAllowed = true;
        return;
    }

    state_.left = clampMotorCommand(left);
    state_.right = clampMotorCommand(right);
    state_.rawMotorCommand = false;
    state_.encoderSynchronizationAllowed = true;
}

void RobotState::driveRawDiagnostic(double left, double right)
{
    std::lock_guard<std::mutex> lock(mutex_);
    lastCommand_ = std::chrono::steady_clock::now();

    if (state_.missionFinished || state_.emergencyStop || state_.mode != "manual")
    {
        // O diagnóstico direto continua bloqueado fora do modo manual para não
        // permitir que o dashboard contorne a parada ou o E-Stop.
        state_.left = 0.0;
        state_.right = 0.0;
        state_.rawMotorCommand = false;
        state_.encoderSynchronizationAllowed = true;
        return;
    }

    state_.left = clampMotorCommand(left);
    state_.right = clampMotorCommand(right);
    state_.rawMotorCommand = true;
    state_.encoderSynchronizationAllowed = true;
}

void RobotState::driveAutonomous(
    double left,
    double right,
    bool encoderSynchronizationAllowed)
{
    std::lock_guard<std::mutex> lock(mutex_);
    lastCommand_ = std::chrono::steady_clock::now();

    if (state_.missionFinished || state_.emergencyStop || state_.mode != "autonomous")
    {
        // O controlador autônomo só pode mover o robô quando o modo autônomo
        // foi ativado explicitamente pelo dashboard.
        return;
    }

    state_.left = clampMotorCommand(left);
    state_.right = clampMotorCommand(right);
    state_.rawMotorCommand = false;
    state_.encoderSynchronizationAllowed = encoderSynchronizationAllowed;
}

bool RobotState::setManualServoAngle(ServoId servo, double angleDegrees)
{
    if (!validServoAngle(angleDegrees))
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.missionFinished || state_.emergencyStop || state_.mode != "manual")
    {
        return false;
    }

    if (servo == ServoId::Wrist &&
        !sameServoAngle(state_.servoPose.wristDegrees, angleDegrees) &&
        state_.servoPose.armDegrees < config::kServoRoutineArmHomeDegrees)
    {
        // O pulso pode colidir com a estrutura quando o braço está abaixo de
        // 15°. O operador deve elevar o braço antes de solicitar a rotação.
        return false;
    }

    // A renovação do servo possui relógio próprio. Ela não pode impedir o
    // watchdog dos motores de zerar um comando de tração antigo.
    lastManualServoCommand_ = std::chrono::steady_clock::now();

    const bool completePoseWasRequested = state_.armServoRequested &&
                                          state_.wristServoRequested &&
                                          state_.gripperServoRequested;
    bool targetChanged = false;

    switch (servo)
    {
    case ServoId::Arm:
        targetChanged = !sameServoAngle(state_.servoPose.armDegrees, angleDegrees);
        state_.servoPose.armDegrees = angleDegrees;
        break;
    case ServoId::Wrist:
        targetChanged = !sameServoAngle(state_.servoPose.wristDegrees, angleDegrees);
        state_.servoPose.wristDegrees = angleDegrees;
        break;
    case ServoId::Gripper:
        targetChanged = !sameServoAngle(state_.servoPose.gripperDegrees, angleDegrees);
        state_.servoPose.gripperDegrees = angleDegrees;
        break;
    }

    // Qualquer ajuste manual renova a pose completa já memorizada. Assim, após
    // o watchdog remover os sinais, mover o pulso também volta a energizar a
    // garra no seu último alvo e evita que a inércia a desloque livremente.
    state_.armServoRequested = true;
    state_.wristServoRequested = true;
    state_.gripperServoRequested = true;

    if (!targetChanged && completePoseWasRequested)
    {
        return true;
    }

    ++state_.servoCommandSequence;
    return true;
}

bool RobotState::setAutonomousServoPose(const ServoPose& pose, bool wristOnly)
{
    if (!validServoAngle(pose.armDegrees) ||
        !validServoAngle(pose.wristDegrees) ||
        !validServoAngle(pose.gripperDegrees))
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.missionFinished || state_.emergencyStop || state_.mode != "autonomous")
    {
        return false;
    }

    if (!sameServoAngle(state_.servoPose.wristDegrees, pose.wristDegrees) &&
        state_.servoPose.armDegrees < config::kServoRoutineArmHomeDegrees)
    {
        // As rotinas devem concluir primeiro a elevação do braço. Rejeitar
        // esta pose impede que uma alteração futura gire o pulso sem folga.
        return false;
    }

    if (state_.armServoRequested && state_.wristServoRequested &&
        state_.gripperServoRequested &&
        sameServoAngle(state_.servoPose.armDegrees, pose.armDegrees) &&
        sameServoAngle(state_.servoPose.wristDegrees, pose.wristDegrees) &&
        sameServoAngle(state_.servoPose.gripperDegrees, pose.gripperDegrees))
    {
        return true;
    }

    // Uma programação predefinida publica a pose inteira de uma vez. O módulo
    // de saída mantém essa atualização agrupada também no protocolo UART.
    // A transição para a saída altera só o pulso sob o mesmo mutex, sem
    // reativar canais desligados nem publicar uma pose intermediária.
    if (wristOnly)
    {
        state_.servoPose.wristDegrees = pose.wristDegrees;
    }
    else
    {
        state_.servoPose = pose;
        state_.armServoRequested = true;
        state_.gripperServoRequested = true;
    }
    state_.wristServoRequested = true;
    ++state_.servoCommandSequence;
    return true;
}

bool RobotState::setAutonomousServoOutputEnabled(ServoId servo, bool enabled)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.missionFinished || state_.emergencyStop || state_.mode != "autonomous")
    {
        return false;
    }

    bool* requested = nullptr;
    switch (servo)
    {
    case ServoId::Arm:
        requested = &state_.armServoRequested;
        break;
    case ServoId::Wrist:
        requested = &state_.wristServoRequested;
        break;
    case ServoId::Gripper:
        requested = &state_.gripperServoRequested;
        break;
    }

    if (*requested == enabled)
    {
        return true;
    }

    // A rotina pode retirar somente o esforço contínuo da garra. O controlador
    // reaplica os outros canais após limpar as saídas no protocolo existente.
    *requested = enabled;
    ++state_.servoCommandSequence;
    return true;
}

bool RobotState::confirmServoRoutineAction()
{
    std::lock_guard<std::mutex> lock(mutex_);
    const bool confirmationAllowed =
        !state_.emergencyStop && state_.mode == "autonomous" &&
        (state_.autonomousMission == AutonomousMission::ServoDeposit ||
         state_.autonomousMission == AutonomousMission::ServoFullSequence ||
         state_.autonomousMission == AutonomousMission::ServoFullSequenceTwo ||
         state_.autonomousMission == AutonomousMission::ServoInternalStorage) &&
        state_.autonomousStatus.servoRoutineWaitingForConfirmation;
    if (!confirmationAllowed)
    {
        return false;
    }

    ++state_.servoRoutineConfirmationSequence;
    return true;
}

void RobotState::setServoRoutineInternalObjectStored(bool stored)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.mode == "autonomous" && !state_.emergencyStop)
    {
        // O inventário é apenas um registro lógico para escolher a sequência
        // de depósito. Ele permanece entre Stop e a próxima rotina.
        state_.servoRoutineInternalObjectStored = stored;
    }
}

void RobotState::disableServos()
{
    std::lock_guard<std::mutex> lock(mutex_);
    disableServosLocked();
}

bool RobotState::beginServoCalibration()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.emergencyStop || state_.mode != "stopped")
    {
        return false;
    }

    // Este modo mantém somente os motores em zero e permite que o controlador
    // dedicado envie um pulso bruto a um único servo. O botão Parar encerra o
    // modo ao chamar stop() e remove essa autorização imediatamente.
    state_.missionFinished = false;
    state_.servoCalibrationActive = true;
    state_.mode = "servo_calibration";
    state_.left = 0.0;
    state_.right = 0.0;
    state_.rawMotorCommand = false;
    state_.encoderSynchronizationAllowed = true;
    disableServosLocked();
    state_.autonomousStatus = {
        "servo_calibration", "Calibração de servo em bancada"};
    lastCommand_ = std::chrono::steady_clock::now();
    return true;
}

void RobotState::endServoCalibration()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.servoCalibrationActive)
    {
        return;
    }

    state_.servoCalibrationActive = false;
    state_.mode = state_.emergencyStop ? "emergency" : "stopped";
    state_.left = 0.0;
    state_.right = 0.0;
    state_.rawMotorCommand = false;
    state_.encoderSynchronizationAllowed = true;
    disableServosLocked();
    state_.autonomousStatus = {"stopped", "Calibração de servo encerrada"};
    lastCommand_ = std::chrono::steady_clock::now();
}

void RobotState::updateAutonomousStatus(const AutonomousStatus& status)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.missionFinished) return;

    // Uma atualização atrasada do controlador autônomo não pode substituir no painel
    // um estado manual, parado ou de emergência que acabou de ser solicitado.
    const bool terminalObstacleStatus =
        status.phase == "obstacle_turn_start_failed" ||
        status.phase == "obstacle_turn_timeout" ||
        status.phase == "obstacle_turn_imu_lost" ||
        status.phase == "obstacle_turn_correction_failed" ||
        status.phase == "obstacle_centering_timeout" ||
        status.phase == "obstacle_imu_lost" ||
        status.phase == "obstacle_side_selection_completed" ||
        status.phase == "obstacle_selected_forward_completed" ||
        status.phase == "obstacle_selected_forward_sensors_lost" ||
        status.phase == "obstacle_selected_forward_timeout" ||
        status.phase == "obstacle_curve_completed" ||
        status.phase == "obstacle_curve_sensors_lost" ||
        status.phase == "obstacle_curve_timeout" ||
        status.phase == "obstacle_completed" ||
        status.phase == "obstacle_encoder_lost" ||
        status.phase == "obstacle_distance_timeout";
    const bool terminalRescueZoneAlignStatus =
        status.phase == "rescue_zone_align_completed";
    const bool terminalRescueZoneApproachStatus =
        status.phase == "rescue_zone_approach_completed" ||
        status.phase == "rescue_zone_approach_no_heading" ||
        status.phase == "rescue_zone_approach_imu_stale" ||
        status.phase == "rescue_zone_approach_ultra_stale" ||
        status.phase == "rescue_zone_approach_timeout";
    const bool terminalRescueZoneSearchStatus =
        status.phase == "rescue_zone_search_found";
    const bool terminalRescueZoneTriangleStatus =
        status.phase == "rescue_zone_triangle_success" ||
        status.phase == "rescue_zone_triangle_failed";
    const bool terminalMissionStatus = status.phase == "corner_yaw_completed" ||
                                       status.phase == "corner_yaw_failed" ||
                                       status.phase == "rescue_exit_failed" ||
                                       status.phase == "completed" ||
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
                                       status.phase == "servo_driver_lost" ||
                                       status.phase == "turnaround_forward_timeout" ||
                                       status.phase == "turnaround_encoder_lost" ||
                                       status.phase == "turnaround_line_search_angle_limit" ||
                                       status.phase == "turnaround_line_search_timeout" ||
                                       status.phase == "esp32_not_ready" ||
                                       status.phase == "camera_not_ready" ||
                                       status.phase == "line_ipc_stale" ||
                                       status.phase == "rescue_esp32_not_ready" ||
                                       status.phase ==
                                           "victim_collection_preparation_timeout" ||
                                       status.phase ==
                                           "victim_collection_encoder_lost" ||
                                       status.phase ==
                                           "victim_collection_encoder_mismatch" ||
                                       status.phase ==
                                           "victim_collection_stall" ||
                                       status.phase ==
                                           "victim_collection_timeout" ||
                                       status.phase == "ball_reached" ||
                                       status.phase ==
                                           "ball_alignment_target_lost_timeout" ||
                                       status.phase ==
                                           "ball_alignment_motion_timeout" ||
                                       terminalObstacleStatus ||
                                       terminalRescueZoneAlignStatus ||
                                       terminalRescueZoneApproachStatus ||
                                       terminalRescueZoneSearchStatus ||
                                       terminalRescueZoneTriangleStatus;
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
    if (!std::isfinite(state_.autonomousStatus.forwardAssistYawDeltaDeg))
    {
        state_.autonomousStatus.forwardAssistYawDeltaDeg = 0.0;
    }
    state_.autonomousStatus.forwardAssistYawDeltaDeg = std::clamp(
        state_.autonomousStatus.forwardAssistYawDeltaDeg,
        0.0,
        180.0);
    if (!std::isfinite(state_.autonomousStatus.forwardLinePosition))
    {
        state_.autonomousStatus.forwardLinePosition = 0.0;
        state_.autonomousStatus.forwardLineVisible = false;
    }
    state_.autonomousStatus.forwardLinePosition = std::clamp(
        state_.autonomousStatus.forwardLinePosition,
        -1.0,
        1.0);
    state_.autonomousStatus.bottomStableFrames = std::max(
        0,
        state_.autonomousStatus.bottomStableFrames);
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

void RobotState::enforceManualServoTimeout(std::chrono::milliseconds timeout)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto age = std::chrono::steady_clock::now() - lastManualServoCommand_;
    const bool anyServoRequested = state_.armServoRequested ||
                                   state_.wristServoRequested ||
                                   state_.gripperServoRequested;

    // No modo autônomo, a própria missão encerra a pose com stop(), E-Stop ou
    // disableServos(). No modo Manual, a perda do painel expira esta autorização.
    if (state_.mode == "manual" && anyServoRequested && age > timeout)
    {
        disableServosLocked();
    }
}

void RobotState::requestInitialServoPoseLocked()
{
    // O Autônomo parte com o braço em 15° e os demais servos em 0°.
    // Enquanto o robô estiver parado, disableServosLocked() remove os pulsos.
    state_.servoPose = {
        config::kAutonomousInitialArmAngleDegrees,
        config::kServoInitialAngleDegrees,
        config::kServoInitialAngleDegrees};
    state_.armServoRequested = true;
    state_.wristServoRequested = true;
    state_.gripperServoRequested = true;
    ++state_.servoCommandSequence;
    lastManualServoCommand_ = std::chrono::steady_clock::now();
}

void RobotState::disableServosLocked()
{
    if (!state_.armServoRequested && !state_.wristServoRequested &&
        !state_.gripperServoRequested)
    {
        return;
    }

    state_.armServoRequested = false;
    state_.wristServoRequested = false;
    state_.gripperServoRequested = false;
    ++state_.servoCommandSequence;
}
