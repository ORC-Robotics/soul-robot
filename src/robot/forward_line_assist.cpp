#include "obr/forward_line_assist.h"

#include <algorithm>

void ForwardLineAssist::reset()
{
    bottom_ = {};
}

bool ForwardLineAssist::update(
    RobotState&,
    const Esp32TelemetrySnapshot&,
    const CameraLineSnapshot& cameraLineSnapshot,
    const ForwardLineSnapshot&)
{
    bottom_ = cameraLineSnapshot;
    // Retornar false mantém o caminho existente de LineCourseMission. Nenhum
    // frame frontal, inclusive PRESENT, substitui as potências inferiores.
    return false;
}

AutonomousStatus ForwardLineAssist::status(
    const std::string& phase,
    const std::string& action,
    const ForwardLineSnapshot& forward) const
{
    AutonomousStatus result;
    result.phase = phase;
    result.action = action;
    result.forwardAssistState = bottom_.gapValidationDecision == "NORMAL"
                                    ? "BOTTOM" : bottom_.gapValidationDecision;
    result.forwardAssistLatchedDirection = bottom_.trustedDirection;
    result.forwardAssistFarTrusted = bottom_.farTrusted;
    result.forwardAssistMediumTrusted = bottom_.mediumTrusted;
    result.forwardAssistGapCandidate = bottom_.curveDiagnostics.lineState == "GAP";
    result.forwardAssistEntryAllowed = false;
    result.forwardAssistEntryBlocker = "VALIDATOR_ONLY";
    result.forwardLineVisible = forward.lineObservationValid();
    result.forwardLinePosition = result.forwardLineVisible ? std::clamp(forward.position, -1.0, 1.0) : 0.0;
    result.forwardPathConfidence = forward.sourceFresh ? forward.confidence : 0.0;
    result.forwardPathState = forward.sourceFresh ? forward.pathState : "UNCERTAIN";
    result.gapValidationDecision = bottom_.gapValidationDecision;
    result.nearLineState = bottom_.nearLineState;
    result.bottomLineControlSource = bottom_.lineControlSource;
    return result;
}
