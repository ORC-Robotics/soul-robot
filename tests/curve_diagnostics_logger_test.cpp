#include "obr/curve_diagnostics_logger.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace
{
bool require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << message << '\n';
    }
    return condition;
}
}

int main()
{
    const std::filesystem::path outputPath =
        std::filesystem::temp_directory_path() /
        "obr_curve_diagnostics_logger_test.csv";

    {
        CurveDiagnosticsLogger logger(outputPath.string());

        CameraLineSnapshot camera;
        camera.sourceFresh = true;
        camera.lineTimestamp = 123.456;
        camera.lineSequence = 42;
        camera.curveDiagnostics.farAngle75 = 47.5;
        camera.curveDiagnostics.farConsensus = "RIGHT";
        camera.curveDiagnostics.farConfirmFrames = 3;
        camera.curveDiagnostics.curveIntent = "RIGHT";
        camera.curveDiagnostics.curveIntentConfirmFrames = 2;
        camera.curveDiagnostics.curveIntentReleaseFrames = 0;
        camera.curveDiagnostics.pathAmbiguous = "false";
        camera.curveDiagnostics.virtualState = "NORMAL";
        camera.curveDiagnostics.lineState = "LINE";

        RobotSnapshot robot;
        robot.mode = "autonomous";
        robot.autonomousMission = AutonomousMission::MainMission;
        robot.autonomousStatus.phase = "line_following";

        MotorSynchronizationSnapshot motors;
        motors.correctedLeftPower = 0.73;
        motors.correctedRightPower = 0.64;

        logger.record(camera, robot, motors);
        logger.record(camera, robot, motors);

        // GAP/verde e outras fases não podem aumentar o CSV de curvas normais.
        camera.lineSequence = 43;
        camera.curveDiagnostics.lineState = "GAP";
        logger.record(camera, robot, motors);
    }

    std::ifstream output(outputPath);
    std::string header;
    std::string row;
    std::string unexpectedRow;
    std::getline(output, header);
    std::getline(output, row);
    std::getline(output, unexpectedRow);

    bool ok = true;
    ok &= require(
        header.find("timestamp_ms,frame_id,farAngle60") == 0,
        "Cabeçalho do diagnóstico não possui os campos iniciais esperados.");
    ok &= require(
        header.find("leftMotor,rightMotor,pathAmbiguous,vstate,lineState") !=
            std::string::npos,
        "Cabeçalho do diagnóstico não possui os campos finais esperados.");
    ok &= require(
        row.find("123456,42,NaN,47.5") == 0,
        "A linha não preservou timestamp, frame e métrica inválida.");
    ok &= require(
        row.find(",0.73,0.64,\"false\",\"NORMAL\",\"LINE\"") !=
            std::string::npos,
        "A linha não contém os comandos finais enviados aos motores.");
    ok &= require(
        unexpectedRow.empty(),
        "Frame duplicado ou fora de LINE foi registrado no CSV.");

    std::error_code removeError;
    std::filesystem::remove(outputPath, removeError);
    return ok ? 0 : 1;
}
