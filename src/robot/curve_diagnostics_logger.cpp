#include "obr/curve_diagnostics_logger.h"

#include "obr/config.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>

namespace
{
constexpr const char* kCsvHeader =
    "timestamp_ms,frame_id,"
    "farAngle60,farAngle75,farAngle90,farAngleSpread,farPathAngleDeg,"
    "farConsensus,farConfirmFrames,"
    "curveIntent,curveIntentConfirmFrames,curveIntentReleaseFrames,"
    "dynamicTargetAngleDeg,dynamicLookaheadPx,"
    "nearPosition,mediumPosition,farBandPosition,headingAngleDeg,"
    "baseVirtualSteering,hybridSteering,finalSteering,"
    "leftMotor,rightMotor,pathAmbiguous,vstate,lineState";

std::string diagnosticNumber(double value)
{
    if (!std::isfinite(value))
    {
        return "NaN";
    }

    std::ostringstream text;
    text << std::setprecision(10) << value;
    return text.str();
}

std::string diagnosticFrames(int value)
{
    return value >= 0 ? std::to_string(value) : "INVALID";
}

std::string csvText(const std::string& value)
{
    std::string escaped = "\"";
    for (const char character : value)
    {
        if (character == '\"')
        {
            escaped += "\"\"";
        }
        else if (character != '\r' && character != '\n')
        {
            escaped += character;
        }
    }
    escaped += '\"';
    return escaped;
}
}

CurveDiagnosticsLogger::CurveDiagnosticsLogger(const std::string& outputPath)
{
    // O arquivo é recriado a cada partida para que cada teste tenha uma sessão
    // independente e possa ser copiado sem separar execuções antigas.
    output_.open(outputPath, std::ios::out | std::ios::trunc);
    if (!output_)
    {
        std::cerr << "Curve diagnostics unavailable at " << outputPath << '\n';
        return;
    }

    output_ << kCsvHeader << '\n';
    output_.flush();
    enabled_ = true;
    worker_ = std::thread(&CurveDiagnosticsLogger::workerLoop, this);
}

CurveDiagnosticsLogger::~CurveDiagnosticsLogger()
{
    if (!worker_.joinable())
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopRequested_ = true;
    }
    condition_.notify_one();
    worker_.join();
}

void CurveDiagnosticsLogger::record(
    const CameraLineSnapshot& cameraLineSnapshot,
    const RobotSnapshot& robotSnapshot,
    const MotorSynchronizationSnapshot& finalMotorCommand)
{
    if (!enabled_ ||
        !isRelevantLineFrame(cameraLineSnapshot, robotSnapshot))
    {
        return;
    }
    if (hasLastFrame_ &&
        cameraLineSnapshot.lineSequence == lastFrameId_)
    {
        return;
    }

    hasLastFrame_ = true;
    lastFrameId_ = cameraLineSnapshot.lineSequence;
    std::string row = buildCsvRow(cameraLineSnapshot, finalMotorCommand);

    bool flushBatch = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pendingRows_.push_back(std::move(row));
        flushBatch = pendingRows_.size() >=
                     static_cast<std::size_t>(
                         config::kCurveDiagnosticsFlushFrames);
    }
    if (flushBatch)
    {
        condition_.notify_one();
    }
}

bool CurveDiagnosticsLogger::isRelevantLineFrame(
    const CameraLineSnapshot& cameraLineSnapshot,
    const RobotSnapshot& robotSnapshot)
{
    return cameraLineSnapshot.sourceFresh &&
           cameraLineSnapshot.curveDiagnostics.lineState == "LINE" &&
           robotSnapshot.mode == "autonomous" &&
           robotSnapshot.autonomousMission == AutonomousMission::MainMission &&
           robotSnapshot.autonomousStatus.phase == "line_following";
}

std::string CurveDiagnosticsLogger::buildCsvRow(
    const CameraLineSnapshot& cameraLineSnapshot,
    const MotorSynchronizationSnapshot& finalMotorCommand)
{
    const CameraCurveDiagnostics& diagnostics =
        cameraLineSnapshot.curveDiagnostics;
    const double timestampMs = cameraLineSnapshot.lineTimestamp * 1000.0;

    std::ostringstream row;
    row << diagnosticNumber(timestampMs) << ','
        << cameraLineSnapshot.lineSequence << ','
        << diagnosticNumber(diagnostics.farAngle60) << ','
        << diagnosticNumber(diagnostics.farAngle75) << ','
        << diagnosticNumber(diagnostics.farAngle90) << ','
        << diagnosticNumber(diagnostics.farAngleSpread) << ','
        << diagnosticNumber(diagnostics.farPathAngleDeg) << ','
        << csvText(diagnostics.farConsensus) << ','
        << diagnosticFrames(diagnostics.farConfirmFrames) << ','
        << csvText(diagnostics.curveIntent) << ','
        << diagnosticFrames(diagnostics.curveIntentConfirmFrames) << ','
        << diagnosticFrames(diagnostics.curveIntentReleaseFrames) << ','
        << diagnosticNumber(diagnostics.dynamicTargetAngleDeg) << ','
        << diagnosticNumber(diagnostics.dynamicLookaheadPx) << ','
        << diagnosticNumber(diagnostics.nearPosition) << ','
        << diagnosticNumber(diagnostics.mediumPosition) << ','
        << diagnosticNumber(diagnostics.farBandPosition) << ','
        << diagnosticNumber(diagnostics.headingAngleDeg) << ','
        << diagnosticNumber(diagnostics.baseVirtualSteering) << ','
        << diagnosticNumber(diagnostics.hybridSteering) << ','
        << diagnosticNumber(diagnostics.finalSteering) << ','
        // Estes são os valores pós-piso/sincronismo enviados pela Raspberry.
        << diagnosticNumber(finalMotorCommand.correctedLeftPower) << ','
        << diagnosticNumber(finalMotorCommand.correctedRightPower) << ','
        << csvText(diagnostics.pathAmbiguous) << ','
        << csvText(diagnostics.virtualState) << ','
        << csvText(diagnostics.lineState);
    return row.str();
}

void CurveDiagnosticsLogger::workerLoop()
{
    const auto flushInterval = std::chrono::milliseconds(
        config::kCurveDiagnosticsFlushIntervalMs);
    while (true)
    {
        std::vector<std::string> rows;
        bool stopAfterBatch = false;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait_for(lock, flushInterval, [this]() {
                return stopRequested_ ||
                       pendingRows_.size() >=
                           static_cast<std::size_t>(
                               config::kCurveDiagnosticsFlushFrames);
            });
            if (pendingRows_.empty())
            {
                if (stopRequested_)
                {
                    break;
                }
                continue;
            }
            rows.swap(pendingRows_);
            stopAfterBatch = stopRequested_;
        }

        for (const std::string& row : rows)
        {
            output_ << row << '\n';
        }
        output_.flush();
        if (!output_ && !writeFailureReported_)
        {
            // Uma falha de diagnóstico é reportada, mas nunca interrompe motores.
            std::cerr << "Curve diagnostics CSV write failed\n";
            writeFailureReported_ = true;
        }
        if (stopAfterBatch)
        {
            break;
        }
    }
}
