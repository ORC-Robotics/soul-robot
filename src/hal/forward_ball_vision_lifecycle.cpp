#include "obr/forward_ball_vision_lifecycle.h"

#include <iostream>

ForwardBallVisionLifecycle::ForwardBallVisionLifecycle(
    CameraMonitor& cameraMonitor)
    : cameraMonitor_(cameraMonitor)
{
}

void ForwardBallVisionLifecycle::begin()
{
    requestedTargetSequence_ = std::numeric_limits<std::uint64_t>::max();
    requestedTargetType_.clear();
    enabled_ = false;
    controlWriteFailureLogged_ = false;
    targetWriteFailureLogged_ = false;
    controlKnown_ = cameraMonitor_.setForwardBallDetectionEnabled(false);
}

void ForwardBallVisionLifecycle::update(
    bool detectionRequired,
    std::uint64_t targetSequence,
    const std::string& targetType)
{
    bool targetReady = !detectionRequired;
    if (detectionRequired &&
        requestedTargetSequence_ == targetSequence &&
        requestedTargetType_ == targetType)
    {
        targetReady = true;
    }
    else if (detectionRequired &&
             cameraMonitor_.requestForwardBallTarget(
                 targetSequence,
                 targetType))
    {
        requestedTargetSequence_ = targetSequence;
        requestedTargetType_ = targetType;
        targetReady = true;
        targetWriteFailureLogged_ = false;
    }
    else if (detectionRequired && !targetWriteFailureLogged_)
    {
        std::cerr << "Victim target sequence could not be published\n";
        targetWriteFailureLogged_ = true;
    }

    const bool shouldEnable = detectionRequired && targetReady;
    if (!controlKnown_ || shouldEnable != enabled_)
    {
        if (cameraMonitor_.setForwardBallDetectionEnabled(shouldEnable))
        {
            enabled_ = shouldEnable;
            controlKnown_ = true;
            controlWriteFailureLogged_ = false;
            std::cout << "Victim detection "
                      << (enabled_ ? "enabled" : "disabled")
                      << " by rescue-area gate\n";
        }
        else if (!controlWriteFailureLogged_)
        {
            controlKnown_ = false;
            std::cerr << "Victim detection gate could not be published\n";
            controlWriteFailureLogged_ = true;
        }
    }
}

ForwardBallSnapshot ForwardBallVisionLifecycle::snapshot() const
{
    return enabled_
               ? cameraMonitor_.forwardBallSnapshot()
               : ForwardBallSnapshot{};
}

void ForwardBallVisionLifecycle::stop()
{
    cameraMonitor_.setForwardBallDetectionEnabled(false);
    enabled_ = false;
}
