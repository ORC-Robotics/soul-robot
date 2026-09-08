#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"
#include "obr/robot_state.h"

#include <cstdint>
#include <string>

// Mantém o estado temporal usado para transferir o controle entre as câmeras.
// A assistência só assume os motores depois dos mesmos gates já validados.
class ForwardLineAssist
{
public:
    void reset();
    bool update(
        RobotState& robotState,
        const Esp32TelemetrySnapshot& esp32Telemetry,
        const CameraLineSnapshot& cameraLineSnapshot,
        const ForwardLineSnapshot& forwardLineSnapshot);
    AutonomousStatus status(
        const std::string& phase,
        const std::string& action,
        const ForwardLineSnapshot& forwardLineSnapshot) const;

private:
    enum class State
    {
        Bottom,
        SearchSpin,
        ForwardFollow
    };

    enum class Direction
    {
        None,
        Left,
        Right
    };

    State state_ = State::Bottom;
    Direction direction_ = Direction::None;
    double yawOriginDegrees_ = 0.0;
    double yawDeltaDegrees_ = 0.0;
    int bottomStableFrames_ = 0;
    int bottomLostFrames_ = 0;
    bool hasPreviousBottomFrame_ = false;
    std::uint64_t previousBottomSequence_ = 0;
    bool previousBottomTrusted_ = false;
    bool previousBottomLineNormal_ = false;
    Direction latchedBottomDirection_ = Direction::None;
    Direction mediumFlipCandidateDirection_ = Direction::None;
    int mediumFlipConfirmationFrames_ = 0;
    bool farTrusted_ = false;
    bool mediumTrusted_ = false;
    bool gapCandidate_ = false;
    bool entryAllowed_ = false;
    std::string entryBlocker_ = "WAITING_TRUST";

    void updateLatchedBottomDirection(
        Direction farDirection,
        Direction mediumDirection,
        bool consecutiveBottomFrame);
};
