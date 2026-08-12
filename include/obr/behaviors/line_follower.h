#pragma once

#include "obr/camera_monitor.h"

#include <chrono>

struct LineFollowerOutput
{
    bool valid = false;
    bool recovering = false;
    bool searchTimedOut = false;
    bool sharpTurning = false;
    bool sharpAligning = false;
    bool sharpTurnTimedOut = false;
    int searchDirection = 0;
    double errorNormalized = 0.0;
    double correction = 0.0;
    double leftPower = 0.0;
    double rightPower = 0.0;
};

// Converte o erro horizontal da única ROI inferior em comando diferencial.
// Ele memoriza somente o último lado significativo para uma procura curta.
// Não existe previsão geométrica, integral, derivada ou classificação de curva.
class LineFollower
{
public:
    void reset();
    LineFollowerOutput calculate(const CameraSnapshot& camera);

private:
    int lastDirection_ = 0;
    double lastCorrectionMagnitude_ = 0.0;
    bool searchActive_ = false;
    std::chrono::steady_clock::time_point searchStartedAt_{};
    bool sharpTurnActive_ = false;
    int sharpTurnDirection_ = 0;
    int sharpTurnCandidateDirection_ = 0;
    int sharpTurnCandidateFrames_ = 0;
    int sharpTurnAlignedFrames_ = 0;
    long long lastProcessedLineSampleSequence_ = 0;
    std::chrono::steady_clock::time_point sharpTurnStartedAt_{};

    static void applyDifferential(
        LineFollowerOutput& output,
        double correction);
    static void applySharpTurn(
        LineFollowerOutput& output,
        int direction);
};
