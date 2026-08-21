#include "obr/camera_monitor.h"

#include "obr/config.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <fstream>
#include <sstream>
#include <string>

namespace
{
double currentUnixSeconds()
{
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration<double>(now).count();
}

double getJsonNumber(
    const std::string& json,
    const std::string& key,
    double fallback)
{
    const std::string marker = "\"" + key + "\":";
    const std::size_t markerPosition = json.find(marker);
    if (markerPosition == std::string::npos)
    {
        return fallback;
    }

    const std::size_t valueStart = markerPosition + marker.size();
    const std::size_t valueEnd = json.find_first_of(",}", valueStart);
    try
    {
        return std::stod(json.substr(valueStart, valueEnd - valueStart));
    }
    catch (const std::exception&)
    {
        return fallback;
    }
}

bool getJsonBool(
    const std::string& json,
    const std::string& key,
    bool fallback)
{
    const std::string marker = "\"" + key + "\":";
    std::size_t valueStart = json.find(marker);
    if (valueStart == std::string::npos)
    {
        return fallback;
    }
    valueStart += marker.size();
    while (valueStart < json.size() &&
           (json[valueStart] == ' ' || json[valueStart] == '\t'))
    {
        ++valueStart;
    }
    if (json.compare(valueStart, 4, "true") == 0)
    {
        return true;
    }
    if (json.compare(valueStart, 5, "false") == 0)
    {
        return false;
    }
    return fallback;
}

bool findJsonValue(
    const std::string& json,
    const std::string& key,
    std::string& value)
{
    const std::string marker = "\"" + key + "\":";
    std::size_t valueStart = json.find(marker);
    if (valueStart == std::string::npos)
    {
        return false;
    }
    valueStart += marker.size();
    while (valueStart < json.size() &&
           (json[valueStart] == ' ' || json[valueStart] == '\t' ||
            json[valueStart] == '\r' || json[valueStart] == '\n'))
    {
        ++valueStart;
    }

    std::size_t valueEnd = json.find_first_of(",}", valueStart);
    if (valueEnd == std::string::npos)
    {
        return false;
    }
    while (valueEnd > valueStart &&
           (json[valueEnd - 1] == ' ' || json[valueEnd - 1] == '\t' ||
            json[valueEnd - 1] == '\r' || json[valueEnd - 1] == '\n'))
    {
        --valueEnd;
    }
    if (valueEnd == valueStart)
    {
        return false;
    }

    value = json.substr(valueStart, valueEnd - valueStart);
    return true;
}

bool tryGetJsonNumber(
    const std::string& json,
    const std::string& key,
    double& value)
{
    std::string text;
    if (!findJsonValue(json, key, text))
    {
        return false;
    }

    try
    {
        std::size_t parsedCharacters = 0;
        const double parsedValue = std::stod(text, &parsedCharacters);
        if (parsedCharacters != text.size())
        {
            return false;
        }
        value = parsedValue;
        return true;
    }
    catch (const std::exception&)
    {
        return false;
    }
}

bool tryGetJsonBool(
    const std::string& json,
    const std::string& key,
    bool& value)
{
    std::string text;
    if (!findJsonValue(json, key, text))
    {
        return false;
    }
    if (text == "true")
    {
        value = true;
        return true;
    }
    if (text == "false")
    {
        value = false;
        return true;
    }
    return false;
}

bool tryGetJsonString(
    const std::string& json,
    const std::string& key,
    std::string& value)
{
    const std::string marker = "\"" + key + "\":";
    std::size_t valueStart = json.find(marker);
    if (valueStart == std::string::npos)
    {
        return false;
    }
    valueStart += marker.size();
    while (valueStart < json.size() &&
           (json[valueStart] == ' ' || json[valueStart] == '\t' ||
            json[valueStart] == '\r' || json[valueStart] == '\n'))
    {
        ++valueStart;
    }
    if (valueStart >= json.size() || json[valueStart] != '"')
    {
        return false;
    }
    const std::size_t valueEnd = json.find('"', valueStart + 1);
    if (valueEnd == std::string::npos)
    {
        return false;
    }
    value = json.substr(valueStart + 1, valueEnd - valueStart - 1);
    return true;
}

bool parseGreenTurnDecision(
    const std::string& interpretation,
    GreenTurnDecision& decision)
{
    if (interpretation == "SEM_DECISAO" || interpretation == "AMBIGUO" ||
        interpretation == "VERDE_FALSO_NO_SENTIDO_ATUAL")
    {
        decision = GreenTurnDecision::None;
        return true;
    }
    if (interpretation == "APROXIMACAO")
    {
        decision = GreenTurnDecision::Approach;
        return true;
    }
    if (interpretation == "ESQUERDA")
    {
        decision = GreenTurnDecision::GuideLeft;
        return true;
    }
    if (interpretation == "DIREITA")
    {
        decision = GreenTurnDecision::GuideRight;
        return true;
    }
    if (interpretation == "RETORNO_180")
    {
        decision = GreenTurnDecision::TurnAround180;
        return true;
    }
    return false;
}

bool isDirectionalGreenTurn(GreenTurnDecision decision)
{
    return decision == GreenTurnDecision::GuideLeft ||
           decision == GreenTurnDecision::GuideRight;
}

bool parseBlackLineGeometryDirection(
    const std::string& text,
    BlackLineGeometryDirection& direction)
{
    if (text == "NONE")
    {
        direction = BlackLineGeometryDirection::None;
        return true;
    }
    if (text == "LEFT")
    {
        direction = BlackLineGeometryDirection::Left;
        return true;
    }
    if (text == "RIGHT")
    {
        direction = BlackLineGeometryDirection::Right;
        return true;
    }
    return false;
}

bool isBlackLineGeometryState(const std::string& state)
{
    return state == "idle" || state == "candidate" ||
           state == "pivoting" || state == "exit_aligned" ||
           state == "rearming";
}

bool parseExtremeCurveDirection(
    const std::string& text,
    ExtremeCurveDirection& direction)
{
    if (text == "NONE")
    {
        direction = ExtremeCurveDirection::None;
        return true;
    }
    if (text == "LEFT")
    {
        direction = ExtremeCurveDirection::Left;
        return true;
    }
    if (text == "RIGHT")
    {
        direction = ExtremeCurveDirection::Right;
        return true;
    }
    return false;
}

bool tryGetJsonUnsignedInteger(
    const std::string& json,
    const std::string& key,
    std::uint64_t& value)
{
    std::string text;
    if (!findJsonValue(json, key, text) || text.empty())
    {
        return false;
    }
    for (const char character : text)
    {
        if (character < '0' || character > '9')
        {
            return false;
        }
    }

    try
    {
        std::size_t parsedCharacters = 0;
        const unsigned long long parsedValue = std::stoull(text, &parsedCharacters);
        if (parsedCharacters != text.size())
        {
            return false;
        }
        value = static_cast<std::uint64_t>(parsedValue);
        return true;
    }
    catch (const std::exception&)
    {
        return false;
    }
}

bool isNormalizedValue(double value)
{
    return std::isfinite(value) && value >= -1.0 && value <= 1.0;
}

CameraLineSnapshot unavailableLineSnapshot(
    const CameraLineSnapshot& cachedSnapshot,
    bool hasCachedSnapshot)
{
    CameraLineSnapshot snapshot = hasCachedSnapshot
                                      ? cachedSnapshot
                                      : CameraLineSnapshot{};
    snapshot.sourceFresh = false;
    snapshot.nearValid = false;
    snapshot.nearX = 0.0;
    snapshot.nearError = 0.0;
    snapshot.farValid = false;
    snapshot.farX = 0.0;
    snapshot.farError = 0.0;
    snapshot.lateralError = 0.0;
    snapshot.headingError = 0.0;
    snapshot.trajectoryValid = false;
    snapshot.fitA = 0.0;
    snapshot.fitB = 0.0;
    snapshot.fitC = 0.0;
    snapshot.fitQuality = 0.0;
    snapshot.fitRmsError = 0.0;
    snapshot.fitSampleCount = 0;
    snapshot.lookaheadX = 0.0;
    snapshot.lookaheadY = 0.0;
    snapshot.curvature = 0.0;
    snapshot.extremeCurveCandidate = false;
    snapshot.extremeCurveDirection = ExtremeCurveDirection::None;
    snapshot.extremeCurveCurvature = 0.0;
    snapshot.extremeCurveConfirmFrames = 0;
    snapshot.blackLineGeometryCandidate = false;
    snapshot.blackLineGeometryDirection = BlackLineGeometryDirection::None;
    snapshot.blackLineGeometryAngleDegrees = 0.0;
    snapshot.blackLineGeometryConfidence = 0.0;
    snapshot.blackLineGeometryState = "idle";
    snapshot.blackLineGeometryExitAlignment = false;
    snapshot.adaptivePreview = 0.0;
    snapshot.previewError = 0.0;
    snapshot.pTerm = 0.0;
    snapshot.filteredDerivative = 0.0;
    snapshot.dTerm = 0.0;
    snapshot.controlError = 0.0;
    snapshot.previewFactor = 0.0;
    snapshot.kControl = 0.0;
    snapshot.kNear = 0.0;
    snapshot.kFar = 0.0;
    snapshot.correction = 0.0;
    snapshot.leftPreview = 0.0;
    snapshot.rightPreview = 0.0;
    snapshot.gapCandidate = false;
    snapshot.gapAlignmentValid = false;
    snapshot.gapAlignmentError = 0.0;
    snapshot.gapReturnValid = false;
    snapshot.gapReturnError = 0.0;
    snapshot.greenNearSeen = false;
    snapshot.greenPathBlackValid = false;
    snapshot.greenCandidateDecision = GreenTurnDecision::None;
    snapshot.greenCandidateFrames = 0;
    snapshot.greenConfirmed = false;
    snapshot.greenTurnDecision = GreenTurnDecision::None;
    if (hasCachedSnapshot)
    {
        snapshot.ageMs =
            (currentUnixSeconds() - snapshot.lineTimestamp) * 1000.0;
        if (!std::isfinite(snapshot.ageMs))
        {
            snapshot.ageMs = 0.0;
        }
    }
    return snapshot;
}
}

bool CameraMonitor::ready() const
{
    std::ifstream file(config::kCameraStatusPath);
    if (!file)
    {
        return false;
    }

    std::ostringstream content;
    content << file.rdbuf();
    const std::string json = content.str();
    const bool active = getJsonBool(json, "active", false);
    const double fps = getJsonNumber(json, "fps", 0.0);
    const double timestamp = getJsonNumber(json, "timestamp", 0.0);
    const double ageMs = (currentUnixSeconds() - timestamp) * 1000.0;

    // Um arquivo antigo não prova que o processo de captura continua vivo.
    return active && fps > 0.0 && std::isfinite(ageMs) && ageMs >= 0.0 &&
           ageMs <= config::kCameraStatusTimeoutMs;
}

CameraLineSnapshot CameraMonitor::lineSnapshot()
{
    try
    {
        std::ifstream file(config::kCameraLineStatusPath);
        if (!file)
        {
            return unavailableLineSnapshot(
                cachedLineSnapshot_, hasCachedLineSnapshot_);
        }

        std::ostringstream content;
        content << file.rdbuf();
        if (file.bad())
        {
            return unavailableLineSnapshot(
                cachedLineSnapshot_, hasCachedLineSnapshot_);
        }
        const std::string json = content.str();

        CameraLineSnapshot candidate;
        std::string greenInterpretation;
        std::string greenRawInterpretation;
        std::string blackLineGeometryDirection;
        std::string extremeCurveDirection;
        if (!tryGetJsonBool(json, "nearValid", candidate.nearValid) ||
            !tryGetJsonNumber(json, "nearX", candidate.nearX) ||
            !tryGetJsonNumber(json, "nearError", candidate.nearError) ||
            !tryGetJsonBool(json, "farValid", candidate.farValid) ||
            !tryGetJsonNumber(json, "farX", candidate.farX) ||
            !tryGetJsonNumber(json, "farError", candidate.farError) ||
            !tryGetJsonNumber(
                json, "lateralError", candidate.lateralError) ||
            !tryGetJsonNumber(
                json, "headingError", candidate.headingError) ||
            !tryGetJsonBool(
                json, "trajectoryValid", candidate.trajectoryValid) ||
            !tryGetJsonNumber(json, "fitA", candidate.fitA) ||
            !tryGetJsonNumber(json, "fitB", candidate.fitB) ||
            !tryGetJsonNumber(json, "fitC", candidate.fitC) ||
            !tryGetJsonNumber(
                json, "fitQuality", candidate.fitQuality) ||
            !tryGetJsonNumber(
                json, "fitRmsError", candidate.fitRmsError) ||
            !tryGetJsonUnsignedInteger(
                json, "fitSampleCount", candidate.fitSampleCount) ||
            !tryGetJsonNumber(
                json, "lookaheadX", candidate.lookaheadX) ||
            !tryGetJsonNumber(
                json, "lookaheadY", candidate.lookaheadY) ||
            !tryGetJsonNumber(
                json, "curvature", candidate.curvature) ||
            !tryGetJsonBool(
                json,
                "extremeCurveCandidate",
                candidate.extremeCurveCandidate) ||
            !tryGetJsonString(
                json, "extremeCurveDirection", extremeCurveDirection) ||
            !parseExtremeCurveDirection(
                extremeCurveDirection, candidate.extremeCurveDirection) ||
            !tryGetJsonNumber(
                json,
                "extremeCurveCurvature",
                candidate.extremeCurveCurvature) ||
            !tryGetJsonUnsignedInteger(
                json,
                "extremeCurveConfirmFrames",
                candidate.extremeCurveConfirmFrames) ||
            !tryGetJsonBool(
                json,
                "blackLineGeometryCandidate",
                candidate.blackLineGeometryCandidate) ||
            !tryGetJsonString(
                json,
                "blackLineGeometryDirection",
                blackLineGeometryDirection) ||
            !parseBlackLineGeometryDirection(
                blackLineGeometryDirection,
                candidate.blackLineGeometryDirection) ||
            !tryGetJsonNumber(
                json,
                "blackLineGeometryAngleDegrees",
                candidate.blackLineGeometryAngleDegrees) ||
            !tryGetJsonNumber(
                json,
                "blackLineGeometryConfidence",
                candidate.blackLineGeometryConfidence) ||
            !tryGetJsonString(
                json,
                "blackLineGeometryState",
                candidate.blackLineGeometryState) ||
            !tryGetJsonBool(
                json,
                "blackLineGeometryExitAlignment",
                candidate.blackLineGeometryExitAlignment) ||
            !tryGetJsonNumber(
                json, "adaptivePreview", candidate.adaptivePreview) ||
            !tryGetJsonNumber(
                json, "previewError", candidate.previewError) ||
            !tryGetJsonNumber(json, "pTerm", candidate.pTerm) ||
            !tryGetJsonNumber(
                json, "filteredDerivative", candidate.filteredDerivative) ||
            !tryGetJsonNumber(json, "dTerm", candidate.dTerm) ||
            !tryGetJsonNumber(json, "controlError", candidate.controlError) ||
            !tryGetJsonNumber(
                json, "preview", candidate.previewFactor) ||
            !tryGetJsonNumber(json, "kControl", candidate.kControl) ||
            !tryGetJsonNumber(json, "kNear", candidate.kNear) ||
            !tryGetJsonNumber(json, "kFar", candidate.kFar) ||
            !tryGetJsonNumber(
                json, "targetCorrection", candidate.targetCorrection) ||
            !tryGetJsonNumber(
                json, "appliedCorrection", candidate.appliedCorrection) ||
            !tryGetJsonNumber(
                json, "steerRateUsed", candidate.steerRateUsed) ||
            !tryGetJsonNumber(json, "correction", candidate.correction) ||
            !tryGetJsonNumber(json, "leftPreview", candidate.leftPreview) ||
            !tryGetJsonNumber(json, "rightPreview", candidate.rightPreview) ||
            !tryGetJsonBool(json, "gapCandidate", candidate.gapCandidate) ||
            !tryGetJsonBool(
                json, "gapAlignmentValid", candidate.gapAlignmentValid) ||
            !tryGetJsonNumber(
                json, "gapAlignmentError", candidate.gapAlignmentError) ||
            !tryGetJsonBool(
                json, "gapReturnValid", candidate.gapReturnValid) ||
            !tryGetJsonNumber(
                json, "gapReturnError", candidate.gapReturnError) ||
            !tryGetJsonBool(
                json, "greenNearSeen", candidate.greenNearSeen) ||
            !tryGetJsonBool(
                json, "greenPathBlackValid", candidate.greenPathBlackValid) ||
            !tryGetJsonString(
                json, "greenRawInterpretation", greenRawInterpretation) ||
            !parseGreenTurnDecision(
                greenRawInterpretation, candidate.greenCandidateDecision) ||
            !tryGetJsonUnsignedInteger(
                json, "greenConsecutiveSamples", candidate.greenCandidateFrames) ||
            !tryGetJsonBool(
                json, "greenConfirmed", candidate.greenConfirmed) ||
            !tryGetJsonString(
                json, "greenInterpretation", greenInterpretation) ||
            !parseGreenTurnDecision(
                greenInterpretation, candidate.greenTurnDecision) ||
            !tryGetJsonNumber(json, "lineTimestamp", candidate.lineTimestamp) ||
            !tryGetJsonUnsignedInteger(
                json, "lineSequence", candidate.lineSequence))
        {
            return unavailableLineSnapshot(
                cachedLineSnapshot_, hasCachedLineSnapshot_);
        }

        const bool valuesValid =
            std::isfinite(candidate.nearX) && candidate.nearX >= 0.0 &&
            isNormalizedValue(candidate.nearError) &&
            std::isfinite(candidate.farX) && candidate.farX >= 0.0 &&
            isNormalizedValue(candidate.farError) &&
            isNormalizedValue(candidate.lateralError) &&
            isNormalizedValue(candidate.headingError) &&
            std::isfinite(candidate.fitA) && candidate.fitA >= -10.0 &&
            candidate.fitA <= 10.0 &&
            std::isfinite(candidate.fitB) && candidate.fitB >= -10.0 &&
            candidate.fitB <= 10.0 &&
            std::isfinite(candidate.fitC) && candidate.fitC >= -2.0 &&
            candidate.fitC <= 2.0 &&
            std::isfinite(candidate.fitQuality) &&
            candidate.fitQuality >= 0.0 && candidate.fitQuality <= 1.0 &&
            std::isfinite(candidate.fitRmsError) &&
            candidate.fitRmsError >= 0.0 && candidate.fitRmsError <= 0.10 &&
            candidate.fitSampleCount <= 64 &&
            std::isfinite(candidate.lookaheadX) &&
            candidate.lookaheadX >= -2.0 && candidate.lookaheadX <= 2.0 &&
            std::isfinite(candidate.lookaheadY) &&
            candidate.lookaheadY >= 0.0 && candidate.lookaheadY <= 1.10 &&
            std::isfinite(candidate.curvature) &&
            candidate.curvature >= -10.0 && candidate.curvature <= 10.0 &&
            std::isfinite(candidate.extremeCurveCurvature) &&
            candidate.extremeCurveCurvature >= -10.0 &&
            candidate.extremeCurveCurvature <= 10.0 &&
            candidate.extremeCurveConfirmFrames <= 2 &&
            (!candidate.extremeCurveCandidate ||
             candidate.extremeCurveDirection != ExtremeCurveDirection::None) &&
            std::isfinite(candidate.blackLineGeometryAngleDegrees) &&
            candidate.blackLineGeometryAngleDegrees >= -180.0 &&
            candidate.blackLineGeometryAngleDegrees <= 180.0 &&
            std::isfinite(candidate.blackLineGeometryConfidence) &&
            candidate.blackLineGeometryConfidence >= 0.0 &&
            candidate.blackLineGeometryConfidence <= 1.0 &&
            isBlackLineGeometryState(candidate.blackLineGeometryState) &&
            (!candidate.blackLineGeometryCandidate ||
             candidate.blackLineGeometryDirection !=
                 BlackLineGeometryDirection::None) &&
            (!candidate.trajectoryValid ||
             (candidate.nearValid && candidate.fitSampleCount >= 4 &&
              candidate.lookaheadY > 0.0)) &&
            std::isfinite(candidate.adaptivePreview) &&
            candidate.adaptivePreview >= 0.0 &&
            candidate.adaptivePreview <= 1.0 &&
            isNormalizedValue(candidate.previewError) &&
            std::isfinite(candidate.pTerm) && candidate.pTerm >= -2.0 &&
            candidate.pTerm <= 2.0 &&
            std::isfinite(candidate.filteredDerivative) &&
            candidate.filteredDerivative >= -5.0 &&
            candidate.filteredDerivative <= 5.0 &&
            isNormalizedValue(candidate.dTerm) &&
            isNormalizedValue(candidate.controlError) &&
            std::isfinite(candidate.previewFactor) &&
            candidate.previewFactor >= 0.0 && candidate.previewFactor <= 1.0 &&
            std::isfinite(candidate.kControl) && candidate.kControl >= 0.0 &&
            candidate.kControl <= 2.0 &&
            std::isfinite(candidate.kNear) && candidate.kNear >= 0.0 &&
            candidate.kNear <= 2.0 &&
            std::isfinite(candidate.kFar) && candidate.kFar >= 0.0 &&
            candidate.kFar <= 2.0 &&
            isNormalizedValue(candidate.targetCorrection) &&
            isNormalizedValue(candidate.appliedCorrection) &&
            std::isfinite(candidate.steerRateUsed) &&
            candidate.steerRateUsed >= 0.0 && candidate.steerRateUsed <= 10.0 &&
            isNormalizedValue(candidate.correction) &&
            isNormalizedValue(candidate.leftPreview) &&
            isNormalizedValue(candidate.rightPreview) &&
            isNormalizedValue(candidate.gapAlignmentError) &&
            isNormalizedValue(candidate.gapReturnError) &&
            (!candidate.gapAlignmentValid || candidate.gapCandidate) &&
            (!candidate.gapReturnValid || candidate.gapCandidate) &&
            (!candidate.greenConfirmed ||
             candidate.greenTurnDecision != GreenTurnDecision::None) &&
            candidate.greenCandidateFrames <= 1000 &&
            std::isfinite(candidate.lineTimestamp);
        if (!valuesValid)
        {
            return unavailableLineSnapshot(
                cachedLineSnapshot_, hasCachedLineSnapshot_);
        }

        if (!candidate.nearValid)
        {
            candidate.nearX = 0.0;
            candidate.nearError = 0.0;
            candidate.lateralError = 0.0;
        }
        if (!candidate.farValid)
        {
            candidate.farX = 0.0;
            candidate.farError = 0.0;
        }
        if (!candidate.nearValid || !candidate.farValid)
        {
            candidate.headingError = 0.0;
            candidate.previewError = 0.0;
        }
        if (!candidate.trajectoryValid)
        {
            candidate.fitA = 0.0;
            candidate.fitB = 0.0;
            candidate.fitC = 0.0;
            candidate.fitQuality = 0.0;
            candidate.fitRmsError = 0.0;
            candidate.fitSampleCount = 0;
            candidate.lookaheadX = 0.0;
            candidate.lookaheadY = 0.0;
            candidate.curvature = 0.0;
        }
        if (!candidate.extremeCurveCandidate)
        {
            candidate.extremeCurveDirection = ExtremeCurveDirection::None;
            candidate.extremeCurveCurvature = 0.0;
            candidate.extremeCurveConfirmFrames = 0;
        }
        if (!candidate.blackLineGeometryCandidate)
        {
            candidate.blackLineGeometryDirection =
                BlackLineGeometryDirection::None;
            candidate.blackLineGeometryAngleDegrees = 0.0;
            candidate.blackLineGeometryConfidence = 0.0;
            if (candidate.blackLineGeometryState == "candidate")
            {
                candidate.blackLineGeometryState = "idle";
            }
        }
        if (!candidate.nearValid && !candidate.farValid)
        {
            candidate.controlError = 0.0;
            candidate.correction = 0.0;
            candidate.leftPreview = 0.0;
            candidate.rightPreview = 0.0;
        }
        if (!candidate.gapCandidate)
        {
            candidate.gapAlignmentValid = false;
            candidate.gapAlignmentError = 0.0;
            candidate.gapReturnValid = false;
            candidate.gapReturnError = 0.0;
        }
        if (!candidate.gapAlignmentValid)
        {
            candidate.gapAlignmentError = 0.0;
        }
        if (!candidate.gapReturnValid)
        {
            candidate.gapReturnError = 0.0;
        }
        if (!candidate.greenConfirmed &&
            candidate.greenTurnDecision != GreenTurnDecision::Approach)
        {
            candidate.greenTurnDecision = GreenTurnDecision::None;
        }
        if (!candidate.greenPathBlackValid ||
            (!isDirectionalGreenTurn(candidate.greenCandidateDecision) &&
             candidate.greenCandidateDecision != GreenTurnDecision::TurnAround180))
        {
            candidate.greenCandidateDecision = GreenTurnDecision::None;
            candidate.greenCandidateFrames = 0;
        }

        if (!hasCachedLineSnapshot_ ||
            candidate.lineSequence != cachedLineSnapshot_.lineSequence)
        {
            cachedLineSnapshot_ = candidate;
            hasCachedLineSnapshot_ = true;
        }

        CameraLineSnapshot snapshot = cachedLineSnapshot_;
        snapshot.ageMs =
            (currentUnixSeconds() - snapshot.lineTimestamp) * 1000.0;
        snapshot.sourceFresh =
            std::isfinite(snapshot.ageMs) && snapshot.ageMs >= 0.0 &&
            snapshot.ageMs <= config::kCameraLineStatusTimeoutMs;
        if (!snapshot.sourceFresh)
        {
            snapshot.nearValid = false;
            snapshot.nearX = 0.0;
            snapshot.nearError = 0.0;
            snapshot.farValid = false;
            snapshot.farX = 0.0;
            snapshot.farError = 0.0;
            snapshot.lateralError = 0.0;
            snapshot.headingError = 0.0;
            snapshot.trajectoryValid = false;
            snapshot.fitA = 0.0;
            snapshot.fitB = 0.0;
            snapshot.fitC = 0.0;
            snapshot.fitQuality = 0.0;
            snapshot.fitRmsError = 0.0;
            snapshot.fitSampleCount = 0;
            snapshot.lookaheadX = 0.0;
            snapshot.lookaheadY = 0.0;
            snapshot.curvature = 0.0;
            snapshot.blackLineGeometryCandidate = false;
            snapshot.blackLineGeometryDirection =
                BlackLineGeometryDirection::None;
            snapshot.blackLineGeometryAngleDegrees = 0.0;
            snapshot.blackLineGeometryConfidence = 0.0;
            snapshot.blackLineGeometryState = "idle";
            snapshot.blackLineGeometryExitAlignment = false;
            snapshot.adaptivePreview = 0.0;
            snapshot.previewError = 0.0;
            snapshot.pTerm = 0.0;
            snapshot.filteredDerivative = 0.0;
            snapshot.dTerm = 0.0;
            snapshot.controlError = 0.0;
            snapshot.previewFactor = 0.0;
            snapshot.kControl = 0.0;
            snapshot.kNear = 0.0;
            snapshot.kFar = 0.0;
            snapshot.correction = 0.0;
            snapshot.leftPreview = 0.0;
            snapshot.rightPreview = 0.0;
            snapshot.gapCandidate = false;
            snapshot.gapAlignmentValid = false;
            snapshot.gapAlignmentError = 0.0;
            snapshot.gapReturnValid = false;
            snapshot.gapReturnError = 0.0;
            snapshot.greenNearSeen = false;
            snapshot.greenPathBlackValid = false;
            snapshot.greenCandidateDecision = GreenTurnDecision::None;
            snapshot.greenCandidateFrames = 0;
            snapshot.greenConfirmed = false;
            snapshot.greenTurnDecision = GreenTurnDecision::None;
        }
        return snapshot;
    }
    catch (...)
    {
        // Uma falha de IPC não pode interromper o loop principal nem alterar motores.
        return unavailableLineSnapshot(
            cachedLineSnapshot_, hasCachedLineSnapshot_);
    }
}
