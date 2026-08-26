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
        camera.lineControlSource = "virtual-reorient";
        camera.curveDiagnostics.nearPosition = 0.12;
        camera.curveDiagnostics.mediumPosition = -0.08;
        camera.curveDiagnostics.farBandPosition = 0.04;
        camera.curveDiagnostics.headingAngleDeg = 5.5;
        camera.curveDiagnostics.finalSteering = 0.18;
        camera.curveDiagnostics.virtualState = "REORIENT_RIGHT";
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

        // GAP permanece no baseline para comparar a futura câmera frontal.
        camera.lineSequence = 43;
        camera.lineTimestamp = 123.5;
        camera.lineControlSource = "gap-blind-search";
        camera.curveDiagnostics.finalSteering = -1.0;
        camera.curveDiagnostics.virtualState = "NORMAL";
        camera.curveDiagnostics.lineState = "GAP";
        logger.record(camera, robot, motors);

        // Verde, IPC antigo e fases externas ao seguidor não entram no CSV.
        camera.lineSequence = 44;
        camera.curveDiagnostics.lineState = "GREEN";
        logger.record(camera, robot, motors);
        camera.lineSequence = 45;
        camera.sourceFresh = false;
        camera.curveDiagnostics.lineState = "LINE";
        logger.record(camera, robot, motors);
        camera.sourceFresh = true;
        camera.lineSequence = 46;
        robot.autonomousStatus.phase = "green_maneuver";
        logger.record(camera, robot, motors);
    }

    std::ifstream output(outputPath);
    std::string header;
    std::string lineRow;
    std::string gapRow;
    std::string unexpectedRow;
    std::getline(output, header);
    std::getline(output, lineRow);
    std::getline(output, gapRow);
    std::getline(output, unexpectedRow);

    bool ok = true;
    ok &= require(
        header ==
            "timestamp_ms,frame_id,nearPosition,mediumPosition,"
            "farBandPosition,headingAngleDeg,finalSteering,leftMotor,"
            "rightMotor,controlSource,vstate,lineState",
        "Cabeçalho do baseline visual não corresponde ao contrato esperado.");
    ok &= require(
        lineRow ==
            "123456,42,0.12,-0.08,0.04,5.5,0.18,0.73,0.64,"
            "\"virtual-reorient\",\"REORIENT_RIGHT\",\"LINE\"",
        "Frame LINE não preservou baseline, motores, fonte e REORIENT.");
    ok &= require(
        gapRow ==
            "123500,43,0.12,-0.08,0.04,5.5,-1,0.73,0.64,"
            "\"gap-blind-search\",\"NORMAL\",\"GAP\"",
        "Frame GAP relevante não foi preservado no CSV.");
    ok &= require(
        unexpectedRow.empty(),
        "Frame duplicado, verde, antigo ou de outra fase foi registrado.");

    std::error_code removeError;
    std::filesystem::remove(outputPath, removeError);
    return ok ? 0 : 1;
}
