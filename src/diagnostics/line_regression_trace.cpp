#include "obr/line_regression_trace.h"

#include "obr/config.h"

#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

namespace
{
// Sessenta segundos cobrem 300 amostras mesmo quando a visão opera perto de 10 Hz.
constexpr auto kMaximumTraceDuration = std::chrono::seconds(60);
constexpr int kMaximumTraceSamples = 300;
// Estes limites espelham a implementação auditada sem participar do controle.
constexpr double kTurningControlEntryThreshold = 0.40;
constexpr double kTurningDirectionMinimum = 0.05;

std::vector<std::string> splitCsvLine(const std::string& line)
{
    std::vector<std::string> values;
    std::istringstream input(line);
    std::string value;
    while (std::getline(input, value, ','))
    {
        values.push_back(value);
    }
    return values;
}

bool parseBoolean(const std::string& text, bool& value)
{
    std::string normalized = text;
    while (!normalized.empty() &&
           (normalized.back() == '\r' || normalized.back() == '\n' ||
            normalized.back() == ' ' || normalized.back() == '\t'))
    {
        normalized.pop_back();
    }

    if (normalized == "True" || normalized == "true" || normalized == "1")
    {
        value = true;
        return true;
    }
    if (normalized == "False" || normalized == "false" || normalized == "0")
    {
        value = false;
        return true;
    }
    return false;
}
}

void LineRegressionTrace::update(
    const CameraLineSnapshot& cameraLineSnapshot,
    const RobotSnapshot& robotSnapshot,
    const MotorSynchronizationSnapshot& motorSynchronization)
{
    const auto now = std::chrono::steady_clock::now();
    const bool requested = requestExists();
    if (!requested && !active_)
    {
        return;
    }
    if (!active_)
    {
        start(now);
        if (!active_)
        {
            return;
        }
    }

    const double controlLoopHz =
        lastControlLoopAt_.time_since_epoch().count() == 0
            ? 0.0
            : 1.0 / std::chrono::duration<double>(
                        now - lastControlLoopAt_).count();
    lastControlLoopAt_ = now;

    VisionSample visionSample;
    const bool newSequence =
        !hasRecordedLineSequence_ ||
        cameraLineSnapshot.lineSequence != lastRecordedLineSequence_;
    if (newSequence && readVisionSample(visionSample) &&
        visionSample.lineSequence == cameraLineSnapshot.lineSequence)
    {
        const std::string missionState = missionStateName(
            robotSnapshot.autonomousStatus.phase);
        const double controlError = cameraLineSnapshot.controlError;
        const bool controlDemand =
            cameraLineSnapshot.nearValid &&
            std::abs(controlError) >= kTurningControlEntryThreshold;
        const bool directionUsable =
            cameraLineSnapshot.nearValid &&
            std::abs(controlError) >= kTurningDirectionMinimum;
        const bool turningDemand =
            directionUsable && controlDemand;
        const std::string sampleDirection = controlError < 0.0
                                                ? "left"
                                                : "right";
        const bool wasTurning = previousState_ == "TurningNear";
        const std::string previousDirection = turningCandidateDirection_;
        if (!wasTurning)
        {
            if (!turningDemand)
            {
                turningEntryCount_ = 0;
                turningCandidateDirection_ = "unknown";
            }
            else if (turningCandidateDirection_ == sampleDirection)
            {
                ++turningEntryCount_;
            }
            else
            {
                turningCandidateDirection_ = sampleDirection;
                turningEntryCount_ = 1;
            }
        }

        std::string transitionReason = "steady";
        if (missionState != previousState_)
        {
            if (missionState == "TurningNear")
            {
                transitionReason = "control_threshold";
            }
            else if (wasTurning)
            {
                const bool signCrossed =
                    ((previousDirection == "left" && controlError > 0.0) ||
                     (previousDirection == "right" && controlError < 0.0));
                if (!cameraLineSnapshot.nearValid)
                {
                    transitionReason = "near_lost";
                }
                else if (signCrossed)
                {
                    transitionReason = "steering_sign_crossed";
                }
                else
                {
                    transitionReason = "control_aligned";
                }
                turningEntryCount_ = 0;
                turningCandidateDirection_ = "unknown";
            }
            else
            {
                transitionReason = robotSnapshot.autonomousStatus.action;
            }
        }

        const double finalLeft = motorSynchronization.correctedLeftPower;
        const double finalRight = motorSynchronization.correctedRightPower;
        // Na convenção lógica validada, sinais iguais avançam e sinais opostos
        // giram. A inversão elétrica, quando necessária, ocorre só no firmware.
        const double linearComponent = (finalLeft + finalRight) * 0.5;
        const double angularComponent = (finalRight - finalLeft) * 0.5;
        const double monotonicTimestamp =
            std::chrono::duration<double>(now.time_since_epoch()).count();

        output_ << std::fixed << std::setprecision(6)
                << monotonicTimestamp << ','
                << cameraLineSnapshot.lineSequence << ','
                << visionSample.frameTimestamp << ','
                << cameraLineSnapshot.ageMs << ','
                << visionSample.captureHz << ','
                << visionSample.processingHz << ','
                << visionSample.ipcHz << ','
                << controlLoopHz << ','
                << visionSample.mjpegHz << ','
                << visionSample.captureMs << ','
                << visionSample.lineDetectionMs << ','
                << visionSample.greenMaskMs << ','
                << visionSample.greenContoursMs << ','
                << visionSample.topologyMs << ','
                << visionSample.greenProcessingMs << ','
                << visionSample.overlayMs << ','
                << visionSample.mjpegMs << ','
                << visionSample.ipcMs << ','
                << visionSample.totalVisionMs << ','
                << (cameraLineSnapshot.nearValid ? "true" : "false") << ','
                << cameraLineSnapshot.nearX << ','
                << cameraLineSnapshot.nearError << ','
                << (cameraLineSnapshot.farValid ? "true" : "false") << ','
                << cameraLineSnapshot.farX << ','
                << cameraLineSnapshot.farError << ','
                << cameraLineSnapshot.lateralError << ','
                << cameraLineSnapshot.headingError << ','
                << cameraLineSnapshot.adaptivePreview << ','
                << cameraLineSnapshot.previewError << ','
                << cameraLineSnapshot.pTerm << ','
                << cameraLineSnapshot.filteredDerivative << ','
                << cameraLineSnapshot.dTerm << ','
                << cameraLineSnapshot.controlError << ','
                << cameraLineSnapshot.previewFactor << ','
                << cameraLineSnapshot.kControl << ','
                << cameraLineSnapshot.kNear << ','
                << cameraLineSnapshot.kFar << ','
                << cameraLineSnapshot.correction << ','
                << csvText(visionSample.greenRaw) << ','
                << (visionSample.greenConfirmed ? "true" : "false") << ','
                << csvText(missionState) << ','
                << csvText(transitionReason) << ','
                << turningEntryCount_ << ','
                << csvText(turningCandidateDirection_) << ','
                << robotSnapshot.left << ','
                << robotSnapshot.right << ','
                << finalLeft << ','
                << finalRight << ','
                << (motorSynchronization.eligible ? "true" : "false") << ','
                << (motorSynchronization.active ? "true" : "false") << ','
                << linearComponent << ','
                << angularComponent << '\n';
        output_.flush();
        lastRecordedLineSequence_ = cameraLineSnapshot.lineSequence;
        hasRecordedLineSequence_ = true;
        ++sampleCount_;
        previousState_ = missionState;
    }

    if (!requested || sampleCount_ >= kMaximumTraceSamples ||
        now - startedAt_ >= kMaximumTraceDuration)
    {
        finish();
    }
}

void LineRegressionTrace::start(std::chrono::steady_clock::time_point now)
{
    output_.open(config::kLineRegressionTracePath, std::ios::trunc);
    if (!output_)
    {
        std::cerr << "Line regression trace could not open output file\n";
        std::remove(config::kLineRegressionTraceRequestPath);
        return;
    }
    output_ << "monotonicTimestamp,lineSequence,frameTimestamp,frameAgeMs,"
               "captureHz,processingHz,ipcPublishHz,mainMissionControlLoopHz,"
               "mjpegOutputHz,captureMs,lineDetectionMs,greenMaskMs,"
               "greenContoursAndFiltersMs,topologyMs,greenProcessingMs,"
               "overlayMs,mjpegPublishMs,ipcPublishMs,totalVisionMs,nearValid,"
               "nearX,nearError,farValid,farX,farError,lateralError,headingError,"
               "adaptivePreview,previewError,pTerm,filteredDerivative,dTerm,"
               "controlError,preview,kControl,kNear,kFar,correction,"
               "greenRaw,greenConfirmed,mainMissionState,transitionReason,"
               "turningNearEntryCount,candidateDirection,"
               "requestedLeft,requestedRight,finalLeft,finalRight,syncEligible,"
               "syncActive,linearComponent,angularComponent\n";
    output_.flush();
    active_ = true;
    startedAt_ = now;
    lastControlLoopAt_ = {};
    hasRecordedLineSequence_ = false;
    sampleCount_ = 0;
    previousState_.clear();
    turningCandidateDirection_ = "unknown";
    turningEntryCount_ = 0;
    std::cout << "Line regression trace started\n";
}

void LineRegressionTrace::finish()
{
    if (!active_)
    {
        return;
    }
    output_.close();
    active_ = false;
    std::remove(config::kLineRegressionTraceRequestPath);
    std::remove(config::kLineRegressionVisionSamplePath);
    std::cout << "Line regression trace finished: samples="
              << sampleCount_ << " path="
              << config::kLineRegressionTracePath << '\n';
}

bool LineRegressionTrace::requestExists()
{
    std::ifstream request(config::kLineRegressionTraceRequestPath);
    return request.good();
}

bool LineRegressionTrace::readVisionSample(VisionSample& sample)
{
    std::ifstream input(config::kLineRegressionVisionSamplePath);
    std::string line;
    if (!input || !std::getline(input, line))
    {
        return false;
    }
    const std::vector<std::string> values = splitCsvLine(line);
    if (values.size() != 18)
    {
        return false;
    }
    try
    {
        sample.lineSequence = std::stoull(values[0]);
        sample.frameTimestamp = std::stod(values[1]);
        sample.captureHz = std::stod(values[2]);
        sample.processingHz = std::stod(values[3]);
        sample.ipcHz = std::stod(values[4]);
        sample.mjpegHz = std::stod(values[5]);
        sample.captureMs = std::stod(values[6]);
        sample.lineDetectionMs = std::stod(values[7]);
        sample.greenMaskMs = std::stod(values[8]);
        sample.greenContoursMs = std::stod(values[9]);
        sample.topologyMs = std::stod(values[10]);
        sample.greenProcessingMs = std::stod(values[11]);
        sample.overlayMs = std::stod(values[12]);
        sample.mjpegMs = std::stod(values[13]);
        sample.ipcMs = std::stod(values[14]);
        sample.totalVisionMs = std::stod(values[15]);
        sample.greenRaw = values[16];
        return parseBoolean(values[17], sample.greenConfirmed);
    }
    catch (const std::exception&)
    {
        return false;
    }
}

std::string LineRegressionTrace::missionStateName(const std::string& phase)
{
    if (phase == "tracking_near")
    {
        return "TrackingNear";
    }
    if (phase == "turning_near")
    {
        return "TurningNear";
    }
    if (phase == "reacquiring_near")
    {
        return "ReacquiringNear";
    }
    if (phase == "fallback_far")
    {
        return "ReacquiringNear";
    }
    if (phase == "searching_left")
    {
        return "SearchingLeft";
    }
    if (phase == "searching_right")
    {
        return "SearchingRight";
    }
    return phase;
}

std::string LineRegressionTrace::csvText(const std::string& value)
{
    std::string escaped = value;
    std::size_t position = 0;
    while ((position = escaped.find('"', position)) != std::string::npos)
    {
        escaped.insert(position, 1, '"');
        position += 2;
    }
    return '"' + escaped + '"';
}
