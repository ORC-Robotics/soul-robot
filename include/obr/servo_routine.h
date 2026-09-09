#pragma once

#include "obr/servo_types.h"

#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

enum class ServoRoutineKind
{
    Initialize,
    Capture,
    InternalStorage,
    Deposit,
    FullSequence,
    FullSequenceTwo
};

struct ServoRoutineOutput
{
    ServoPose pose;
    bool poseRequested = false;
    bool releaseGripper = false;
    bool waitingForConfirmation = false;
    bool internalObjectStored = false;
    bool completed = false;
    bool failed = false;
    std::string phase = "servo_idle";
    std::string action = "Rotina de servos aguardando início";
    double progressPercent = 0.0;
};

// Executa ações predefinidas em ordem, alterando somente um servo por passo.
// Os tempos são explícitos porque os servos não fornecem posição física.
class ServoRoutine
{
public:
    ServoRoutineOutput update(
        ServoRoutineKind kind,
        unsigned long long runSequence,
        unsigned long long confirmationSequence,
        const ServoPose& initialPose,
        std::chrono::steady_clock::time_point now);
    void resetExecution();

private:
    enum class StepAction
    {
        ApplyInitialPose,
        HoldCurrentPose,
        MoveArm,
        MoveWrist,
        MoveGripper,
        ReleaseGripper,
        WaitForOpenConfirmation,
        WaitForCloseConfirmation,
        MarkInternalStorage,
        ClearInternalStorage,
        Complete
    };

    struct Step
    {
        StepAction action;
        double angleDegrees = 0.0;
        int durationMs = 0;
        const char* phase = "servo_step";
        const char* description = "Executando passo da rotina";
    };

    std::vector<Step> steps_;
    ServoPose pose_{};
    ServoRoutineKind activeKind_ = ServoRoutineKind::Capture;
    unsigned long long activeRunSequence_ = 0;
    unsigned long long confirmationSequenceAtWait_ = 0;
    std::size_t stepIndex_ = 0;
    bool stepStarted_ = false;
    bool failed_ = false;
    bool internalObjectStored_ = false;
    std::chrono::steady_clock::time_point stepDeadline_{};

    void start(
        ServoRoutineKind kind,
        unsigned long long runSequence,
        unsigned long long confirmationSequence,
        const ServoPose& initialPose);
    void appendMove(
        StepAction action,
        double angleDegrees,
        int durationMs,
        const char* phase,
        const char* description);
    ServoRoutineOutput currentOutput() const;
};
