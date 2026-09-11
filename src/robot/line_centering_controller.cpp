#include "obr/line_centering_controller.h"

#include "obr/config.h"

#include <cmath>

void LineCenteringController::start(
    std::chrono::steady_clock::time_point now)
{
    started_ = true;
    startedAt_ = now;
}

LineCenteringOutput LineCenteringController::update(
    const CameraLineSnapshot& line,
    std::chrono::steady_clock::time_point now) const
{
    LineCenteringOutput output;
    if (!started_)
    {
        return output;
    }

    const bool nearPositionValid =
        line.lineNearDetected && std::isfinite(line.lineNearFinePosition) &&
        std::abs(line.lineNearFinePosition) <= 1.0;
    const double mediumPosition = line.curveDiagnostics.mediumPosition;
    const bool mediumPositionValid =
        line.mediumTrusted && std::isfinite(mediumPosition) &&
        std::abs(mediumPosition) <= 1.0;
    const bool bothCentered =
        nearPositionValid && mediumPositionValid &&
        std::abs(line.lineNearFinePosition) <=
            config::kGreenTurnAroundCenteringTolerance &&
        std::abs(mediumPosition) <=
            config::kGreenTurnAroundCenteringTolerance;
    output.timedOut =
        now - startedAt_ >= std::chrono::milliseconds(
                                config::kGreenTurnAroundCenteringTimeoutMs);
    if (bothCentered || output.timedOut)
    {
        output.completed = true;
        output.state = output.timedOut ? "TIMEOUT" : "CENTERED";
        return output;
    }

    double alignmentPosition = 0.0;
    bool alignmentDirectionValid = false;
    if (mediumPositionValid &&
        std::abs(mediumPosition) >
            config::kGreenTurnAroundCenteringTolerance)
    {
        // MEDIUM orienta a faixa futura; NEAR remove o deslocamento restante.
        alignmentPosition = mediumPosition;
        alignmentDirectionValid = true;
    }
    else if (nearPositionValid &&
             std::abs(line.lineNearFinePosition) >
                 config::kGreenTurnAroundCenteringTolerance)
    {
        alignmentPosition = line.lineNearFinePosition;
        alignmentDirectionValid = true;
    }

    if (!alignmentDirectionValid)
    {
        return output;
    }

    const double turnSign = alignmentPosition > 0.0 ? 1.0 : -1.0;
    output.leftPower =
        turnSign * config::kGreenTurnAroundCenteringPower;
    output.rightPower = -output.leftPower;
    output.state = "CENTERING";
    return output;
}

void LineCenteringController::reset()
{
    started_ = false;
}
