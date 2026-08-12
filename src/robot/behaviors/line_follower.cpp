#include "obr/behaviors/line_follower.h"

#include "obr/config.h"

#include <algorithm>
#include <chrono>
#include <cmath>

void LineFollower::reset()
{
    lastDirection_ = 0;
    lastCorrectionMagnitude_ = 0.0;
    searchActive_ = false;
    sharpTurnActive_ = false;
    sharpTurnDirection_ = 0;
    sharpTurnCandidateDirection_ = 0;
    sharpTurnCandidateFrames_ = 0;
    sharpTurnAlignedFrames_ = 0;
    lastProcessedLineSampleSequence_ = 0;
}

LineFollowerOutput LineFollower::calculate(const CameraSnapshot& camera)
{
    LineFollowerOutput output;
    if (!camera.fresh)
    {
        // Status antigo não pode manter nem iniciar a procura visual.
        reset();
        return output;
    }

    const auto now = std::chrono::steady_clock::now();
    const bool newLineSample = camera.lineSampleSequence > 0 &&
                               camera.lineSampleSequence !=
                                   lastProcessedLineSampleSequence_;
    if (newLineSample)
    {
        lastProcessedLineSampleSequence_ = camera.lineSampleSequence;
    }

    if (!sharpTurnActive_ && newLineSample)
    {
        if (camera.lineDetected && camera.sharpTurnCandidate &&
            camera.sharpTurnDirection != 0)
        {
            if (sharpTurnCandidateDirection_ == camera.sharpTurnDirection)
            {
                ++sharpTurnCandidateFrames_;
            }
            else
            {
                sharpTurnCandidateDirection_ = camera.sharpTurnDirection;
                sharpTurnCandidateFrames_ = 1;
            }
        }
        else
        {
            sharpTurnCandidateDirection_ = 0;
            sharpTurnCandidateFrames_ = 0;
        }

        if (sharpTurnCandidateFrames_ >= config::kSharpTurnConfirmFrames)
        {
            sharpTurnActive_ = true;
            sharpTurnDirection_ = sharpTurnCandidateDirection_;
            sharpTurnStartedAt_ = now;
            sharpTurnAlignedFrames_ = 0;
            sharpTurnCandidateFrames_ = 0;
            searchActive_ = false;
        }
    }

    if (sharpTurnActive_)
    {
        const auto sharpTurnElapsed = now - sharpTurnStartedAt_;
        output.errorNormalized = camera.lineDetected
                                     ? camera.lineErrorNormalized
                                     : 0.0;
        const bool alignedWithNewSegment =
            camera.lineDetected && !camera.sharpTurnCandidate &&
            std::abs(camera.lineErrorNormalized) <=
                config::kSharpTurnExitMaximumError &&
            camera.lineAngleFromVerticalDegrees <=
                config::kSharpTurnExitMaximumAngleFromVerticalDegrees;
        if (newLineSample)
        {
            sharpTurnAlignedFrames_ = alignedWithNewSegment
                                          ? sharpTurnAlignedFrames_ + 1
                                          : 0;
        }

        const bool minimumTurnFinished =
            sharpTurnElapsed >=
            std::chrono::milliseconds(config::kSharpTurnMinimumMs);
        if (minimumTurnFinished &&
            sharpTurnAlignedFrames_ >= config::kSharpTurnAlignedFrames)
        {
            sharpTurnActive_ = false;
            sharpTurnDirection_ = 0;
            sharpTurnAlignedFrames_ = 0;
        }
        else if (sharpTurnElapsed >
                 std::chrono::milliseconds(config::kSharpTurnTimeoutMs))
        {
            output.sharpTurnTimedOut = true;
            output.searchDirection = sharpTurnDirection_;
            return output;
        }
        else if (minimumTurnFinished && alignedWithNewSegment &&
                 sharpTurnAlignedFrames_ > 0)
        {
            // O primeiro frame alinhado zera o PWM. Permanecer girando durante
            // a confirmação faria o robô atravessar o centro da nova linha.
            output.valid = true;
            output.sharpAligning = true;
            output.searchDirection = sharpTurnDirection_;
            return output;
        }
        else
        {
            output.valid = true;
            output.sharpTurning = true;
            output.searchDirection = sharpTurnDirection_;
            applySharpTurn(output, sharpTurnDirection_);
            return output;
        }
    }

    if (camera.lineDetected)
    {
        output.valid = true;
        output.errorNormalized = std::clamp(
            camera.lineErrorNormalized, -1.0, 1.0);
        output.correction = config::kLineFollowerKp * output.errorNormalized;
        if (std::abs(output.errorNormalized) >=
            config::kLineFollowerDirectionMemoryDeadband)
        {
            lastDirection_ = output.errorNormalized > 0.0 ? 1 : -1;
        }
        lastCorrectionMagnitude_ = std::abs(output.correction);
        searchActive_ = false;
        applyDifferential(output, output.correction);
        return output;
    }

    if (lastDirection_ == 0)
    {
        // Sem erro anterior significativo não existe um lado confiável para procurar.
        return output;
    }

    if (!searchActive_)
    {
        searchActive_ = true;
        searchStartedAt_ = now;
    }

    const auto elapsed = now - searchStartedAt_;
    if (elapsed > std::chrono::milliseconds(config::kLineLostSearchTimeoutMs))
    {
        output.searchTimedOut = true;
        output.searchDirection = lastDirection_;
        return output;
    }

    const double elapsedSeconds =
        std::chrono::duration<double>(elapsed).count();
    const double startingCorrection = std::max(
        lastCorrectionMagnitude_, config::kLineLostInitialCorrection);
    const double correctionMagnitude = std::clamp(
        startingCorrection +
            config::kLineLostCorrectionRampPerSecond * elapsedSeconds,
        config::kLineLostInitialCorrection,
        config::kLineLostMaximumCorrection);

    output.valid = true;
    output.recovering = true;
    output.searchDirection = lastDirection_;
    output.correction = correctionMagnitude * lastDirection_;
    applyDifferential(output, output.correction);
    return output;
}

void LineFollower::applyDifferential(
    LineFollowerOutput& output,
    double correction)
{
    // Erro positivo significa linha à direita: o lado esquerdo acelera e o
    // direito desacelera para recentralizar a linha na única ROI inferior.
    output.leftPower = std::clamp(
        config::kLineFollowerBaseSpeed + correction,
        config::kMinMotorOutput,
        config::kMaxMotorOutput);
    output.rightPower = std::clamp(
        config::kLineFollowerBaseSpeed - correction,
        config::kMinMotorOutput,
        config::kMaxMotorOutput);
}

void LineFollower::applySharpTurn(
    LineFollowerOutput& output,
    int direction)
{
    // Direita: lado esquerdo avança e direito recua. Esquerda faz o oposto.
    // Os dois lados participam do pivô para não produzir outra trajetória diagonal.
    output.leftPower = config::kSharpTurnPower * direction;
    output.rightPower = -config::kSharpTurnPower * direction;
}
