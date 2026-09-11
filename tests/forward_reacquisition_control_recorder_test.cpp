#include "obr/forward_reacquisition_control_recorder.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace
{
void require(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

std::string readFile(const std::filesystem::path& path)
{
    std::ifstream input(path);
    std::ostringstream contents;
    contents << input.rdbuf();
    return contents.str();
}
}

int main()
{
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() /
        "obr_forward_reacquisition_control_recorder_test";
    std::error_code cleanupError;
    std::filesystem::remove_all(root, cleanupError);
    std::filesystem::create_directories(root);

    RobotSnapshot robot;
    robot.mode = "autonomous";
    robot.autonomousStatus.phase = "obstacle_curving";
    robot.autonomousStatus.action = "Contornando obstáculo";
    robot.autonomousStatus.obstacleYawBase =
        std::numeric_limits<double>::quiet_NaN();
    robot.autonomousStatus.obstacleLeftClearance = 42.5;
    robot.autonomousStatus.obstacleRightClearance =
        std::numeric_limits<double>::infinity();
    robot.autonomousStatus.obstacleSelectedSide = "LEFT";

    MotorSynchronizationSnapshot motor;
    motor.correctedLeftPower = 0.71;
    motor.correctedRightPower = 0.63;
    Esp32TelemetrySnapshot telemetry;
    telemetry.sensorFresh = true;
    telemetry.lastSensorAgeMs = 1;
    telemetry.yawZDeg = 12.5;
    telemetry.gyroZDegPerSec = -3.25;
    telemetry.ultrasonicDistanceCm = 18.0;
    telemetry.leftEncoderCount = 100;
    telemetry.rightEncoderCount = 90;
    ForwardLineSnapshot forward;
    forward.sourceFresh = true;
    forward.sequence = 77;
    forward.timestamp = 1234.5;
    forward.ageMs = 4.0;

    const auto disabledControl = root / "disabled-control.json";
    {
        ForwardReacquisitionControlRecorder recorder(
            disabledControl.string(), (root / "sessions").string());
        recorder.record(robot, motor, telemetry, forward, "obstacle_avoidance");
    }
    require(
        !std::filesystem::exists(root / "sessions" / "off" / "control.jsonl"),
        "Gravador desligado criou dados.");

    const std::string session = "20260911T120000Z_obstacle_exit_left";
    const auto sessionDirectory = root / "sessions" / session;
    std::filesystem::create_directories(sessionDirectory);
    const auto controlPath = root / "control.json";
    {
        std::ofstream control(controlPath);
        control << "{\"active\":true,\"session\":\"" << session
                << "\",\"label\":\"obstacle_exit_left\"}";
    }

    const RobotSnapshot robotBefore = robot;
    const MotorSynchronizationSnapshot motorBefore = motor;
    {
        ForwardReacquisitionControlRecorder recorder(
            controlPath.string(), (root / "sessions").string());
        recorder.record(robot, motor, telemetry, forward, "obstacle_avoidance");
    }
    require(robot.left == robotBefore.left && robot.right == robotBefore.right,
            "O gravador alterou o comando solicitado.");
    require(
        motor.correctedLeftPower == motorBefore.correctedLeftPower &&
            motor.correctedRightPower == motorBefore.correctedRightPower,
        "O gravador alterou o comando final dos motores.");

    const std::string jsonl = readFile(sessionDirectory / "control.jsonl");
    require(!jsonl.empty(), "A sessão ativa não gravou controle.");
    require(jsonl.find("NaN") == std::string::npos,
            "O JSON contém NaN.");
    require(jsonl.find("Infinity") == std::string::npos,
            "O JSON contém Infinity.");
    require(
        jsonl.find("\"finalLeftCommand\":0.71") != std::string::npos &&
            jsonl.find("\"finalRightCommand\":0.63") != std::string::npos,
        "O JSON não preservou o comando final.");
    require(
        jsonl.find("\"sequence\":77") != std::string::npos &&
            jsonl.find("\"yawBase\":null") != std::string::npos &&
            jsonl.find("\"rightClearanceCm\":null") != std::string::npos,
        "O JSON não preservou correlação frontal ou números estritos.");

    // Um destino inválido força erro de escrita; a chamada continua sem exceção
    // e os comandos entregues por referência const permanecem idênticos.
    const auto invalidRoot = root / "not-a-directory";
    {
        std::ofstream file(invalidRoot);
        file << "x";
    }
    {
        ForwardReacquisitionControlRecorder recorder(
            controlPath.string(), invalidRoot.string());
        recorder.record(robot, motor, telemetry, forward, "obstacle_avoidance");
    }
    require(
        motor.correctedLeftPower == 0.71 && motor.correctedRightPower == 0.63,
        "Falha de gravação interferiu no comando final.");

    std::filesystem::remove_all(root, cleanupError);
    return 0;
}
