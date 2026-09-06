#pragma once

#include "obr/camera_monitor.h"

#include <cstdint>
#include <limits>

// Controla o gate e a identidade da execução que pode consumir a visão pesada
// de vítimas. Fora do resgate, o detector permanece explicitamente desligado.
class ForwardBallVisionLifecycle
{
public:
    explicit ForwardBallVisionLifecycle(CameraMonitor& cameraMonitor);

    void begin();
    void update(
        bool detectionRequired,
        std::uint64_t autonomousRunSequence);
    ForwardBallSnapshot snapshot() const;
    void stop();

private:
    CameraMonitor& cameraMonitor_;
    std::uint64_t requestedTargetSequence_ =
        std::numeric_limits<std::uint64_t>::max();
    bool controlKnown_ = false;
    bool enabled_ = false;
    bool controlWriteFailureLogged_ = false;
    bool targetWriteFailureLogged_ = false;
};
