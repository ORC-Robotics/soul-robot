#include "obr/forward_reacquisition_control_recorder.h"

#include "obr/config.h"

#include <chrono>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>

namespace
{
constexpr std::size_t kMaximumPendingSamples = 512;
constexpr std::size_t kFlushBatchSamples = 25;
constexpr auto kControlPollInterval = std::chrono::milliseconds(100);

bool safeSessionName(const std::string& value)
{
    if (value.empty())
    {
        return false;
    }
    for (const unsigned char character : value)
    {
        if (!(std::isalnum(character) || character == '_' || character == '-'))
        {
            return false;
        }
    }
    return true;
}

bool jsonBoolean(const std::string& json, const std::string& key, bool& value)
{
    const std::string marker = "\"" + key + "\"";
    std::size_t position = json.find(marker);
    if (position == std::string::npos)
    {
        return false;
    }
    position = json.find(':', position + marker.size());
    if (position == std::string::npos)
    {
        return false;
    }
    position = json.find_first_not_of(" \t\r\n", position + 1);
    if (json.compare(position, 4, "true") == 0)
    {
        value = true;
        return true;
    }
    if (json.compare(position, 5, "false") == 0)
    {
        value = false;
        return true;
    }
    return false;
}

bool jsonString(
    const std::string& json,
    const std::string& key,
    std::string& value)
{
    const std::string marker = "\"" + key + "\"";
    std::size_t position = json.find(marker);
    if (position == std::string::npos)
    {
        return false;
    }
    position = json.find(':', position + marker.size());
    position = json.find('"', position == std::string::npos ? position : position + 1);
    if (position == std::string::npos)
    {
        return false;
    }
    const std::size_t end = json.find('"', position + 1);
    if (end == std::string::npos)
    {
        return false;
    }
    value = json.substr(position + 1, end - position - 1);
    return safeSessionName(value);
}

std::string escapedJsonString(const std::string& value)
{
    std::ostringstream escaped;
    escaped << '"';
    for (const unsigned char character : value)
    {
        switch (character)
        {
        case '"': escaped << "\\\""; break;
        case '\\': escaped << "\\\\"; break;
        case '\b': escaped << "\\b"; break;
        case '\f': escaped << "\\f"; break;
        case '\n': escaped << "\\n"; break;
        case '\r': escaped << "\\r"; break;
        case '\t': escaped << "\\t"; break;
        default:
            if (character < 0x20)
            {
                escaped << "\\u00" << std::hex << std::setw(2)
                        << std::setfill('0') << static_cast<int>(character)
                        << std::dec << std::setfill(' ');
            }
            else
            {
                escaped << static_cast<char>(character);
            }
        }
    }
    escaped << '"';
    return escaped.str();
}

void appendNumber(std::ostringstream& json, double value)
{
    if (std::isfinite(value))
    {
        json << std::setprecision(15) << value;
    }
    else
    {
        json << "null";
    }
}

void appendUartMotorNumber(std::ostringstream& json, double value)
{
    // Esp32Bridge envia MOTOR com três casas decimais. Usar a mesma forma
    // registra o número que entrou efetivamente na mensagem UART.
    if (!std::isfinite(value))
    {
        json << "0.000";
        return;
    }
    json << std::fixed << std::setprecision(3) << value << std::defaultfloat;
}
}

ForwardReacquisitionControlRecorder::ForwardReacquisitionControlRecorder(
    std::string controlPath,
    std::string sessionRoot)
    : controlPath_(std::move(controlPath)),
      sessionRoot_(std::move(sessionRoot))
{
    refreshControl();
    try
    {
        worker_ = std::thread(
            &ForwardReacquisitionControlRecorder::workerLoop, this);
        workerAvailable_ = true;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Forward reacquisition control recorder unavailable: "
                  << error.what() << '\n';
    }
}

ForwardReacquisitionControlRecorder::~ForwardReacquisitionControlRecorder()
{
    if (!workerAvailable_ || !worker_.joinable())
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

void ForwardReacquisitionControlRecorder::record(
    const RobotSnapshot& robotSnapshot,
    const MotorSynchronizationSnapshot& finalMotorCommand,
    const Esp32TelemetrySnapshot& telemetry,
    const ForwardLineSnapshot& forwardLineSnapshot,
    const std::string& commandSource) noexcept
{
    try
    {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock() || !workerAvailable_ || !recordingActive_ ||
            pendingSamples_.size() >= kMaximumPendingSamples)
        {
            return;
        }

        Sample sample;
        sample.timestamp = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        sample.robot = robotSnapshot;
        sample.motor = finalMotorCommand;
        sample.telemetry = telemetry;
        sample.forwardLine = forwardLineSnapshot;
        sample.commandSource = commandSource;
        sample.session = activeSession_;
        pendingSamples_.push_back(std::move(sample));
        const bool flushBatch = pendingSamples_.size() >= kFlushBatchSamples;
        lock.unlock();
        if (flushBatch)
        {
            condition_.notify_one();
        }
    }
    catch (...)
    {
        // Falta de memória ou qualquer erro diagnóstico não chega ao controle.
    }
}

void ForwardReacquisitionControlRecorder::refreshControl() noexcept
{
    bool active = false;
    std::string session;
    try
    {
        std::ifstream input(controlPath_);
        std::ostringstream contents;
        contents << input.rdbuf();
        const std::string json = contents.str();
        active = input.good() || input.eof();
        active = active && jsonBoolean(json, "active", active) && active &&
                 jsonString(json, "session", session);
    }
    catch (...)
    {
        active = false;
        session.clear();
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (active && session != failedSession_)
    {
        failureReported_ = false;
    }
    recordingActive_ = active && session != failedSession_;
    activeSession_ = recordingActive_ ? session : std::string{};
}

void ForwardReacquisitionControlRecorder::workerLoop() noexcept
{
    while (true)
    {
        refreshControl();
        std::deque<Sample> samples;
        bool shouldStop = false;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait_for(lock, kControlPollInterval, [this] {
                return stopRequested_ ||
                       pendingSamples_.size() >= kFlushBatchSamples;
            });
            samples.swap(pendingSamples_);
            shouldStop = stopRequested_;
        }

        if (!samples.empty() && !writeSamples(samples))
        {
            std::lock_guard<std::mutex> lock(mutex_);
            recordingActive_ = false;
            failedSession_ = samples.front().session;
            if (!failureReported_)
            {
                std::cerr
                    << "Forward reacquisition control recording disabled after write failure\n";
                failureReported_ = true;
            }
        }
        if (shouldStop)
        {
            return;
        }
    }
}

bool ForwardReacquisitionControlRecorder::writeSamples(
    const std::deque<Sample>& samples) noexcept
{
    try
    {
        std::ofstream output;
        std::string openSession;
        for (const Sample& sample : samples)
        {
            if (!safeSessionName(sample.session))
            {
                return false;
            }
            if (sample.session != openSession)
            {
                output.close();
                const std::filesystem::path outputPath =
                    std::filesystem::path(sessionRoot_) /
                    sample.session / "control.jsonl";
                output.open(outputPath, std::ios::out | std::ios::app);
                if (!output)
                {
                    return false;
                }
                openSession = sample.session;
            }
            output << buildJsonLine(sample) << '\n';
            if (!output)
            {
                return false;
            }
        }
        output.flush();
        return static_cast<bool>(output);
    }
    catch (...)
    {
        return false;
    }
}

std::string ForwardReacquisitionControlRecorder::buildJsonLine(
    const Sample& sample)
{
    const bool obstacleActive =
        sample.robot.mode == "autonomous" &&
        sample.robot.autonomousStatus.phase.rfind("obstacle_", 0) == 0;
    const bool encodersAvailable =
        sample.telemetry.sensorFresh && sample.telemetry.lastSensorAgeMs >= 0;

    std::ostringstream json;
    json << '{' << "\"timestamp\":";
    appendNumber(json, sample.timestamp);
    json << ",\"mode\":" << escapedJsonString(sample.robot.mode)
         << ",\"autonomousMission\":"
         << escapedJsonString(
                autonomousMissionName(sample.robot.autonomousMission))
         << ",\"autonomousPhase\":"
         << escapedJsonString(sample.robot.autonomousStatus.phase)
         << ",\"action\":"
         << escapedJsonString(sample.robot.autonomousStatus.action)
         << ",\"movementCommandSource\":"
         << escapedJsonString(sample.commandSource)
         << ",\"finalLeftCommand\":";
    appendUartMotorNumber(json, sample.motor.correctedLeftPower);
    json << ",\"finalRightCommand\":";
    appendUartMotorNumber(json, sample.motor.correctedRightPower);
    json << ",\"yawDegrees\":";
    appendNumber(json, sample.telemetry.yawZDeg);
    json << ",\"gyroZDegreesPerSecond\":";
    appendNumber(json, sample.telemetry.gyroZDegPerSec);
    json << ",\"ultrasonicDistanceCm\":";
    if (sample.telemetry.ultrasonicDistanceCm >= 0.0)
    {
        appendNumber(json, sample.telemetry.ultrasonicDistanceCm);
    }
    else
    {
        json << "null";
    }
    json << ",\"obstacle\":{"
         << "\"active\":" << (obstacleActive ? "true" : "false")
         << ",\"phase\":";
    if (obstacleActive)
    {
        json << escapedJsonString(sample.robot.autonomousStatus.phase);
    }
    else
    {
        json << "null";
    }
    json << ",\"yawBase\":";
    appendNumber(json, sample.robot.autonomousStatus.obstacleYawBase);
    json << ",\"leftClearanceCm\":";
    appendNumber(json, sample.robot.autonomousStatus.obstacleLeftClearance);
    json << ",\"rightClearanceCm\":";
    appendNumber(json, sample.robot.autonomousStatus.obstacleRightClearance);
    json << ",\"selectedSide\":"
         << escapedJsonString(
                sample.robot.autonomousStatus.obstacleSelectedSide)
         << ",\"cameraBlackLeft\":"
         << (sample.robot.autonomousStatus.cameraBlackLeft ? "true" : "false")
         << ",\"cameraBlackRight\":"
         << (sample.robot.autonomousStatus.cameraBlackRight ? "true" : "false")
         << ",\"cameraBlackLeftFrames\":"
         << sample.robot.autonomousStatus.cameraBlackLeftFrames
         << ",\"cameraBlackRightFrames\":"
         << sample.robot.autonomousStatus.cameraBlackRightFrames
         << ",\"selectedSideSource\":"
         << escapedJsonString(sample.robot.autonomousStatus.selectedSideSource)
         << ",\"rawBestParabolaSide\":"
         << escapedJsonString(sample.robot.autonomousStatus.rawBestParabolaSide)
         << ",\"bestParabolaSide\":"
         << escapedJsonString(sample.robot.autonomousStatus.bestParabolaSide)
         << ",\"bestParabolaScore\":"
         << sample.robot.autonomousStatus.bestParabolaScore
         << ",\"bestParabolaLeftBlack\":"
         << sample.robot.autonomousStatus.bestParabolaLeftBlack
         << ",\"bestParabolaRightBlack\":"
         << sample.robot.autonomousStatus.bestParabolaRightBlack
         << ",\"bestParabolaSequence\":"
         << sample.robot.autonomousStatus.bestParabolaSequence
         << ",\"bestParabolaSideValid\":"
         << (sample.robot.autonomousStatus.bestParabolaSideValid ? "true" : "false")
         << ",\"nearForwardLineVisible\":"
         << (sample.robot.autonomousStatus.nearForwardLineVisible ? "true" : "false")
         << ",\"nearForwardLineVotes\":"
         << sample.robot.autonomousStatus.nearForwardLineVotes
         << ",\"nearForwardLineSamples\":"
         << sample.robot.autonomousStatus.nearForwardLineSamples
         << ",\"case3Armed\":"
         << (sample.robot.autonomousStatus.case3Armed ? "true" : "false")
         << ",\"case3FusionAcquireTime\":"
         << sample.robot.autonomousStatus.case3FusionAcquireTime
         << ",\"case3TimeRemainingMs\":"
         << sample.robot.autonomousStatus.case3TimeRemainingMs
         << "},\"encoders\":{"
         << "\"available\":" << (encodersAvailable ? "true" : "false")
         << ",\"leftCount\":" << sample.telemetry.leftEncoderCount
         << ",\"rightCount\":" << sample.telemetry.rightEncoderCount
         << ",\"leftDistanceCm\":";
    if (encodersAvailable)
    {
        appendNumber(
            json,
            static_cast<double>(sample.telemetry.leftEncoderCount) /
                config::kEncoderCountsPerCentimeter);
    }
    else
    {
        json << "null";
    }
    json << ",\"rightDistanceCm\":";
    if (encodersAvailable)
    {
        appendNumber(
            json,
            static_cast<double>(sample.telemetry.rightEncoderCount) /
                config::kEncoderCountsPerCentimeter);
    }
    else
    {
        json << "null";
    }
    json << "},\"forwardLineSnapshot\":{"
         << "\"sourceFresh\":"
         << (sample.forwardLine.sourceFresh ? "true" : "false")
         << ",\"sequence\":" << sample.forwardLine.sequence
         << ",\"timestamp\":";
    appendNumber(json, sample.forwardLine.timestamp);
    json << ",\"ageMs\":";
    appendNumber(json, sample.forwardLine.ageMs);
    json << ",\"pathState\":"
         << escapedJsonString(sample.forwardLine.pathState)
         << ",\"present\":"
         << (sample.forwardLine.present ? "true" : "false")
         << ",\"position\":";
    appendNumber(json, sample.forwardLine.position);
    json << ",\"confidence\":";
    appendNumber(json, sample.forwardLine.confidence);
    json << "}}";
    return json.str();
}
