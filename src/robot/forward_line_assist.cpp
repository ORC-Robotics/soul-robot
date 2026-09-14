#include "obr/forward_line_assist.h"
#include "obr/config.h"

#include <algorithm>
#include <cmath>

void ForwardLineAssist::reset()
{
    *this = ForwardLineAssist{};
}

bool ForwardLineAssist::update(
    RobotState& robotState,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    const CameraLineSnapshot& cameraLineSnapshot,
    const ForwardLineSnapshot& forwardLineSnapshot,
    bool allowRecovery)
{
    bottom_ = cameraLineSnapshot;
    if (!allowRecovery)
    {
        recoveryDirectionSign_ = 0;
        bottomLossFrames_ = bottomStableFrames_ = 0;
        recovering_ = false;
        lastBottomSequence_ = cameraLineSnapshot.lineSequence;
        return false;
    }

    if (forwardLineSnapshot.lineObservationValid() &&
        std::abs(forwardLineSnapshot.position) >=
            config::kForwardAssistDirectionPositionThreshold)
    {
        recoveryDirectionSign_ = forwardLineSnapshot.position < 0.0 ? -1 : 1;
    }

    const bool newBottom = cameraLineSnapshot.lineSequence > 0 &&
        cameraLineSnapshot.lineSequence != lastBottomSequence_;
    if (newBottom) lastBottomSequence_ = cameraLineSnapshot.lineSequence;
    const bool bottomNormal = cameraLineSnapshot.normalSteeringValid &&
        cameraLineSnapshot.gapValidationDecision == "NORMAL";

    if (recovering_)
    {
        if (newBottom && bottomNormal) ++bottomStableFrames_;
        else if (newBottom) bottomStableFrames_ = 0;
        if (bottomStableFrames_ >= config::kForwardAssistBottomStableFrames)
        {
            recovering_ = false;
            bottomLossFrames_ = bottomStableFrames_ = 0;
            return false;
        }
    }
    else
    {
        if (newBottom && cameraLineSnapshot.gapValidationDecision == "LOST")
            ++bottomLossFrames_;
        else if (newBottom)
            bottomLossFrames_ = 0;
        if (bottomLossFrames_ >= config::kForwardAssistBottomLossFrames &&
            recoveryDirectionSign_ != 0)
        {
            recovering_ = true;
            bottomStableFrames_ = 0;
            recoveryStartYawDegrees_ = esp32Telemetry.yawZDeg;
            recoveryStartedAt_ = std::chrono::steady_clock::now();
        }
    }

    if (!recovering_) return false;

    const bool angularLimitReached = esp32Telemetry.sensorFresh &&
        esp32Telemetry.mpuOk && std::isfinite(esp32Telemetry.yawZDeg) &&
        std::abs(std::remainder(
            esp32Telemetry.yawZDeg - recoveryStartYawDegrees_, 360.0)) >=
            config::kForwardAssistMaximumSearchDegrees;
    const bool timeoutReached = std::chrono::steady_clock::now() -
        recoveryStartedAt_ >=
            std::chrono::milliseconds(config::kForwardAssistRecoveryTimeoutMs);
    if (angularLimitReached || timeoutReached)
    {
        robotState.driveAutonomous(0.0, 0.0);
        robotState.updateAutonomousStatus(status(
            "forward_line_recovery_limit",
            "CAM1 não recuperou a linha dentro do limite seguro",
            forwardLineSnapshot));
        return true;
    }

    const double leftPower = recoveryDirectionSign_ < 0 ?
        -config::kForwardAssistSearchSpinPower :
         config::kForwardAssistSearchSpinPower;
    robotState.driveAutonomous(leftPower, -leftPower);
    robotState.updateAutonomousStatus(status(
        "forward_line_recovery",
        recoveryDirectionSign_ < 0 ?
            "CAM0 perdeu a linha; buscando à esquerda indicada pela CAM1" :
            "CAM0 perdeu a linha; buscando à direita indicada pela CAM1",
        forwardLineSnapshot));
    return true;
}

AutonomousStatus ForwardLineAssist::status(
    const std::string& phase,
    const std::string& action,
    const ForwardLineSnapshot& forward) const
{
    AutonomousStatus result;
    result.phase = phase;
    result.action = action;
    result.forwardAssistState = recovering_ ? "FRONT_RECOVERY" :
        (bottom_.gapValidationDecision == "NORMAL" ?
             "BOTTOM" : bottom_.gapValidationDecision);
    result.forwardAssistLatchedDirection = recoveryDirectionSign_ < 0 ? "LEFT" :
        (recoveryDirectionSign_ > 0 ? "RIGHT" : bottom_.trustedDirection);
    result.forwardAssistFarTrusted = bottom_.farTrusted;
    result.forwardAssistMediumTrusted = bottom_.mediumTrusted;
    result.forwardAssistGapCandidate = bottom_.curveDiagnostics.lineState == "GAP";
    result.forwardAssistEntryAllowed = recovering_;
    result.forwardAssistEntryBlocker = recovering_ ? "NONE" : "BOTTOM_ACTIVE";
    result.forwardLineVisible = forward.lineObservationValid();
    result.forwardLinePosition = result.forwardLineVisible ? std::clamp(forward.position, -1.0, 1.0) : 0.0;
    result.forwardPathConfidence = forward.sourceFresh ? forward.confidence : 0.0;
    result.forwardPathState = forward.sourceFresh ? forward.pathState : "UNCERTAIN";
    result.gapValidationDecision = bottom_.gapValidationDecision;
    result.nearLineState = bottom_.nearLineState;
    result.bottomLineControlSource = bottom_.lineControlSource;
    return result;
}
