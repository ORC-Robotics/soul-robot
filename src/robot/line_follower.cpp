#include "obr/line_follower.h"

#include "obr/config.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <sstream>
#include <string>

namespace
{
bool readJsonNumber(
    const std::string& json,
    const std::string& key,
    double& value)
{
    const std::string marker = "\"" + key + "\":";
    const std::size_t markerPosition = json.find(marker);
    if (markerPosition == std::string::npos)
    {
        return false;
    }

    const std::size_t valueStart = markerPosition + marker.size();
    const std::size_t valueEnd = json.find_first_of(",}", valueStart);
    if (valueEnd == std::string::npos)
    {
        return false;
    }

    try
    {
        std::size_t parsedLength = 0;
        const std::string token = json.substr(valueStart, valueEnd - valueStart);
        value = std::stod(token, &parsedLength);
        return token.find_first_not_of(" \t\r\n", parsedLength) == std::string::npos;
    }
    catch (const std::exception&)
    {
        return false;
    }
}

bool readJsonBool(
    const std::string& json,
    const std::string& key,
    bool& value)
{
    const std::string marker = "\"" + key + "\":";
    std::size_t valueStart = json.find(marker);
    if (valueStart == std::string::npos)
    {
        return false;
    }

    valueStart += marker.size();
    while (valueStart < json.size() &&
           (json[valueStart] == ' ' || json[valueStart] == '\t'))
    {
        ++valueStart;
    }

    const auto tokenIsComplete = [&json](std::size_t tokenEnd)
    {
        while (tokenEnd < json.size() &&
               (json[tokenEnd] == ' ' || json[tokenEnd] == '\t' ||
                json[tokenEnd] == '\r' || json[tokenEnd] == '\n'))
        {
            ++tokenEnd;
        }
        return tokenEnd < json.size() &&
               (json[tokenEnd] == ',' || json[tokenEnd] == '}');
    };

    if (json.compare(valueStart, 4, "true") == 0 &&
        tokenIsComplete(valueStart + 4))
    {
        value = true;
        return true;
    }
    if (json.compare(valueStart, 5, "false") == 0 &&
        tokenIsComplete(valueStart + 5))
    {
        value = false;
        return true;
    }
    return false;
}

bool readJsonString(
    const std::string& json,
    const std::string& key,
    std::string& value)
{
    const std::string marker = "\"" + key + "\":";
    std::size_t valueStart = json.find(marker);
    if (valueStart == std::string::npos)
    {
        return false;
    }

    valueStart += marker.size();
    while (valueStart < json.size() &&
           (json[valueStart] == ' ' || json[valueStart] == '\t'))
    {
        ++valueStart;
    }
    if (valueStart >= json.size() || json[valueStart] != '"')
    {
        return false;
    }

    const std::size_t valueEnd = json.find('"', valueStart + 1);
    if (valueEnd == std::string::npos)
    {
        return false;
    }

    value = json.substr(valueStart + 1, valueEnd - valueStart - 1);
    std::size_t tokenEnd = valueEnd + 1;
    while (tokenEnd < json.size() &&
           (json[tokenEnd] == ' ' || json[tokenEnd] == '\t' ||
            json[tokenEnd] == '\r' || json[tokenEnd] == '\n'))
    {
        ++tokenEnd;
    }
    const bool tokenIsComplete = tokenEnd < json.size() &&
                                 (json[tokenEnd] == ',' || json[tokenEnd] == '}');
    const bool valueIsKnown = value == "LEFT" || value == "RIGHT" ||
                              value == "BOTH" || value == "NONE";
    return tokenIsComplete && valueIsKnown;
}

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

void stopForVisionFailure(
    RobotState& robotState,
    const std::string& phase,
    const std::string& action)
{
    // Qualquer falha da visão zera os dois lados no mesmo ciclo de controle.
    // O robô não tenta adivinhar a posição da linha usando um comando antigo.
    robotState.driveAutonomous(0.0, 0.0);
    robotState.updateAutonomousStatus(makeStatus(phase, action));
}
}

LineFollower::CameraStatus LineFollower::readCameraStatus() const
{
    CameraStatus status;
    std::ifstream file(config::kCameraStatusPath);
    if (!file)
    {
        return status;
    }

    std::ostringstream content;
    content << file.rdbuf();
    const std::string json = content.str();
    status.valid = readJsonBool(json, "active", status.active) &&
                   readJsonBool(json, "lineDetected", status.lineDetected) &&
                   readJsonBool(json, "turn90Ahead", status.turn90Ahead) &&
                   readJsonString(json, "turn90Direction", status.turn90Direction) &&
                   readJsonBool(json, "farLineDetected", status.farLineDetected) &&
                   readJsonNumber(json, "fps", status.framesPerSecond) &&
                   readJsonNumber(json, "lineError", status.lineError) &&
                   readJsonNumber(json, "timestamp", status.timestampSeconds);
    if (status.valid)
    {
        // O booleano e a direção devem descrever o mesmo resultado da visão.
        status.valid = status.turn90Ahead
                           ? status.turn90Direction != "NONE"
                           : status.turn90Direction == "NONE";
    }
    return status;
}

bool LineFollower::isFresh(const CameraStatus& status)
{
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const double currentSeconds = std::chrono::duration<double>(now).count();
    const double ageMilliseconds =
        (currentSeconds - status.timestampSeconds) * 1000.0;
    return std::isfinite(ageMilliseconds) && ageMilliseconds >= 0.0 &&
           ageMilliseconds <= config::kCameraStatusTimeoutMs;
}

void LineFollower::reset()
{
    phase_ = Phase::Following;
    turnDirection_ = TurnDirection::None;
    approachStartLeftCount_ = 0;
    approachStartRightCount_ = 0;
    lastApproachProgressCounts_ = 0.0;
    originalLineLost_ = false;
    turnTriggerArmed_ = true;
    phaseStartedAt_ = {};
    lastApproachProgressAt_ = {};
    turnCooldownUntil_ = {};
}

bool LineFollower::encodersReady(
    const Esp32TelemetrySnapshot& esp32Telemetry)
{
    return esp32Telemetry.sensorFresh &&
           esp32Telemetry.lastSensorAgeMs >= 0 &&
           esp32Telemetry.lastSensorAgeMs <=
               config::kDriveDistanceEncoderFreshnessMs &&
           std::isfinite(esp32Telemetry.leftEncoderRate) &&
           std::isfinite(esp32Telemetry.rightEncoderRate);
}

void LineFollower::startTurnApproach(
    TurnDirection direction,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    std::chrono::steady_clock::time_point now)
{
    phase_ = Phase::AdvancingToTurn;
    turnDirection_ = direction;
    turnTriggerArmed_ = false;
    approachStartLeftCount_ = esp32Telemetry.leftEncoderCount;
    approachStartRightCount_ = esp32Telemetry.rightEncoderCount;
    lastApproachProgressCounts_ = 0.0;
    originalLineLost_ = false;
    phaseStartedAt_ = now;
    lastApproachProgressAt_ = now;
}

void LineFollower::updateTurnApproach(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    const CameraStatus& cameraStatus,
    std::chrono::steady_clock::time_point now)
{
    if (!encodersReady(esp32Telemetry))
    {
        robotState.stop();
        reset();
        robotState.updateAutonomousStatus(makeStatus(
            "line_turn_encoder_lost",
            "Manobra interrompida: encoders sem dados recentes"));
        return;
    }

    if (phase_ == Phase::SettlingBeforeTurn)
    {
        robotState.driveAutonomous(0.0, 0.0);
        if (now - phaseStartedAt_ <
            std::chrono::milliseconds(config::kDriveDistanceSettleMs))
        {
            robotState.updateAutonomousStatus(makeStatus(
                "line_turn_settling",
                "Avanço concluído: aguardando o robô estabilizar",
                100.0));
            return;
        }

        phase_ = Phase::TurningToLine;
        phaseStartedAt_ = now;
        originalLineLost_ = !cameraStatus.lineDetected;
        updateTurningToLine(robotState, cameraStatus, now);
        return;
    }

    if (now - phaseStartedAt_ >
        std::chrono::milliseconds(config::kLineFollowerTurnApproachTimeoutMs))
    {
        robotState.stop();
        reset();
        robotState.updateAutonomousStatus(makeStatus(
            "line_turn_approach_timeout",
            "Manobra interrompida: avanço de 10 cm excedeu o tempo máximo"));
        return;
    }

    const double leftCounts = std::abs(static_cast<double>(
        esp32Telemetry.leftEncoderCount - approachStartLeftCount_));
    const double rightCounts = std::abs(static_cast<double>(
        esp32Telemetry.rightEncoderCount - approachStartRightCount_));
    const double minimumCounts = std::min(leftCounts, rightCounts);
    const double targetCounts = config::kLineFollowerTurnApproachDistanceCm *
                                config::kEncoderCountsPerCentimeter;
    const double progressPercent = std::clamp(
        minimumCounts / targetCounts * 100.0, 0.0, 100.0);

    if (minimumCounts >= lastApproachProgressCounts_ +
                             config::kDriveDistanceMinimumProgressCounts)
    {
        lastApproachProgressCounts_ = minimumCounts;
        lastApproachProgressAt_ = now;
    }
    if (now - lastApproachProgressAt_ >
        std::chrono::milliseconds(config::kDriveDistanceStallTimeoutMs))
    {
        robotState.stop();
        reset();
        robotState.updateAutonomousStatus(makeStatus(
            "line_turn_encoder_stall",
            "Manobra interrompida: um lado não avançou durante os 10 cm",
            progressPercent));
        return;
    }

    if (minimumCounts < targetCounts)
    {
        robotState.driveAutonomous(
            config::kLineFollowerMotorPower,
            config::kLineFollowerMotorPower);
        robotState.updateAutonomousStatus(makeStatus(
            "line_turn_approach",
            "Avançando 10 cm antes do giro de 90°",
            progressPercent));
        return;
    }

    robotState.driveAutonomous(0.0, 0.0);
    phase_ = Phase::SettlingBeforeTurn;
    phaseStartedAt_ = now;
    robotState.updateAutonomousStatus(makeStatus(
        "line_turn_settling",
        "Avanço de 10 cm concluído: PWM zerado",
        100.0));
}

void LineFollower::updateTurningToLine(
    RobotState& robotState,
    const CameraStatus& cameraStatus,
    std::chrono::steady_clock::time_point now)
{
    if (now - phaseStartedAt_ >
        std::chrono::milliseconds(config::kLineFollowerTurnSearchTimeoutMs))
    {
        robotState.stop();
        reset();
        robotState.updateAutonomousStatus(makeStatus(
            "line_turn_search_timeout",
            "Manobra interrompida: linha não reencontrada durante o giro"));
        return;
    }

    if (!cameraStatus.lineDetected)
    {
        originalLineLost_ = true;
    }
    else if (originalLineLost_)
    {
        phase_ = Phase::Following;
        turnDirection_ = TurnDirection::None;
        turnCooldownUntil_ = now +
                             std::chrono::milliseconds(
                                 config::kLineFollowerTurnCooldownMs);
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(makeStatus(
            "line_turn_completed",
            "Linha reencontrada: giro de 90° concluído"));
        return;
    }

    const double motorPower = config::kLineFollowerMotorPower;
    if (turnDirection_ == TurnDirection::Right)
    {
        // O giro usa sentidos opostos para procurar a nova linha sem avançar.
        robotState.driveAutonomous(motorPower, -motorPower);
        robotState.updateAutonomousStatus(makeStatus(
            "line_turn_right",
            "Girando à direita até reencontrar a linha"));
    }
    else
    {
        robotState.driveAutonomous(-motorPower, motorPower);
        robotState.updateAutonomousStatus(makeStatus(
            "line_turn_left",
            "Girando à esquerda até reencontrar a linha"));
    }
}

void LineFollower::update(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry)
{
    const CameraStatus visionStatus = readCameraStatus();
    if (!visionStatus.valid)
    {
        reset();
        stopForVisionFailure(
            robotState,
            "vision_invalid",
            "Visão inválida: motores parados");
        return;
    }

    const bool cameraHealthy = visionStatus.active &&
                               visionStatus.framesPerSecond > 0.0 &&
                               std::isfinite(visionStatus.framesPerSecond) &&
                               visionStatus.timestampSeconds > 0.0 &&
                               isFresh(visionStatus);
    if (!cameraHealthy)
    {
        reset();
        stopForVisionFailure(
            robotState,
            "camera_unavailable",
            "Câmera inválida ou desatualizada: motores parados");
        return;
    }

    const bool lineErrorValid = !visionStatus.lineDetected ||
                                (std::isfinite(visionStatus.lineError) &&
                                 visionStatus.lineError >=
                                     config::kLineFollowerMinimumError &&
                                 visionStatus.lineError <=
                                     config::kLineFollowerMaximumError);
    if (!lineErrorValid)
    {
        reset();
        stopForVisionFailure(
            robotState,
            "vision_invalid",
            "Erro da linha inválido: motores parados");
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (phase_ == Phase::AdvancingToTurn ||
        phase_ == Phase::SettlingBeforeTurn)
    {
        updateTurnApproach(robotState, esp32Telemetry, visionStatus, now);
        return;
    }
    if (phase_ == Phase::TurningToLine)
    {
        updateTurningToLine(robotState, visionStatus, now);
        return;
    }

    if (!visionStatus.turn90Ahead)
    {
        // Uma nova manobra só pode ser armada depois que o alerta anterior sair da ROI.
        turnTriggerArmed_ = true;
    }

    const bool rightTurnAhead = visionStatus.turn90Ahead &&
                                visionStatus.turn90Direction == "RIGHT";
    const bool leftTurnAhead = visionStatus.turn90Ahead &&
                               visionStatus.turn90Direction == "LEFT";
    const bool directionalTurnAhead = rightTurnAhead || leftTurnAhead;
    if (turnTriggerArmed_ && directionalTurnAhead &&
        !visionStatus.farLineDetected &&
        now >= turnCooldownUntil_)
    {
        if (!encodersReady(esp32Telemetry))
        {
            robotState.driveAutonomous(0.0, 0.0);
            robotState.updateAutonomousStatus(makeStatus(
                "line_turn_waiting_encoders",
                "Curva detectada: aguardando encoders recentes"));
            return;
        }

        startTurnApproach(
            rightTurnAhead ? TurnDirection::Right : TurnDirection::Left,
            esp32Telemetry,
            now);
        updateTurnApproach(robotState, esp32Telemetry, visionStatus, now);
        return;
    }

    const bool noLineInAnyRegion = !visionStatus.lineDetected &&
                                   !visionStatus.turn90Ahead &&
                                   !visionStatus.farLineDetected;
    if (noLineInAnyRegion)
    {
        // Com a câmera válida e recente, ausência nas três ROIs manda seguir reto.
        // Uma falha de captura continua sendo tratada antes e sempre zera os motores.
        robotState.driveAutonomous(
            config::kLineFollowerMotorPower,
            config::kLineFollowerMotorPower);
        robotState.updateAutonomousStatus(makeStatus(
            "searching_forward",
            "Nenhuma ROI encontrou linha: avançando em frente"));
        return;
    }

    if (!visionStatus.lineDetected)
    {
        stopForVisionFailure(
            robotState,
            "line_not_in_control_roi",
            "Linha ausente na ROI inferior: motores parados");
        return;
    }

    const double motorPower = config::kLineFollowerMotorPower;
    double leftPower = motorPower;
    double rightPower = motorPower;
    std::string phase = "following_straight";
    std::string action = "Linha centralizada: seguindo em frente";

    if (visionStatus.lineError > config::kLineFollowerDeadband)
    {
        // Para virar à direita, somente o motor esquerdo permanece ligado.
        leftPower = motorPower;
        rightPower = 0.0;
        phase = "following_right";
        action = "Linha à direita: somente o motor esquerdo está ligado";
    }
    else if (visionStatus.lineError < -config::kLineFollowerDeadband)
    {
        // Para virar à esquerda, somente o motor direito permanece ligado.
        leftPower = 0.0;
        rightPower = motorPower;
        phase = "following_left";
        action = "Linha à esquerda: somente o motor direito está ligado";
    }

    if (visionStatus.turn90Ahead)
    {
        action += "; curva de 90° ou cruzamento detectado à frente";
    }
    if (visionStatus.farLineDetected)
    {
        action += "; linha detectada na ROI distante";
    }

    robotState.driveAutonomous(leftPower, rightPower);
    robotState.updateAutonomousStatus(makeStatus(phase, action));
}
