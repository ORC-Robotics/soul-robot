#include "obr/camera_monitor.h"

#include "obr/config.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace
{
double currentUnixSeconds()
{
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration<double>(now).count();
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

bool tryGetJsonObject(
    const std::string& json,
    const std::string& key,
    std::string& object)
{
    const std::string marker = "\"" + key + "\":";
    std::size_t start = json.find(marker);
    if (start == std::string::npos)
    {
        return false;
    }
    start = json.find('{', start + marker.size());
    if (start == std::string::npos)
    {
        return false;
    }

    int depth = 0;
    bool insideString = false;
    bool escaped = false;
    for (std::size_t index = start; index < json.size(); ++index)
    {
        const char character = json[index];
        if (insideString)
        {
            if (escaped)
            {
                escaped = false;
            }
            else if (character == '\\')
            {
                escaped = true;
            }
            else if (character == '"')
            {
                insideString = false;
            }
            continue;
        }
        if (character == '"')
        {
            insideString = true;
        }
        else if (character == '{')
        {
            ++depth;
        }
        else if (character == '}' && --depth == 0)
        {
            object = json.substr(start, index - start + 1);
            return true;
        }
    }
    return false;
}

bool parseRescueZoneGeometryState(
    const std::string& value,
    RescueZoneGeometryState& state)
{
    if (value == "NOT_DETECTED")
    {
        state = RescueZoneGeometryState::NotDetected;
        return true;
    }
    if (value == "BOUNDS_UNKNOWN")
    {
        state = RescueZoneGeometryState::BoundsUnknown;
        return true;
    }
    if (value == "LEFT_BOUND_ONLY")
    {
        state = RescueZoneGeometryState::LeftBoundOnly;
        return true;
    }
    if (value == "RIGHT_BOUND_ONLY")
    {
        state = RescueZoneGeometryState::RightBoundOnly;
        return true;
    }
    if (value == "FULL_BOUNDS")
    {
        state = RescueZoneGeometryState::FullBounds;
        return true;
    }
    return false;
}

bool parseRescueZoneObservation(
    const std::string& json,
    RescueZoneObservation& observation)
{
    std::string geometryState;
    if (!tryGetJsonBool(
            json, "candidateDetected", observation.candidateDetected) ||
        !tryGetJsonBool(json, "detected", observation.detected) ||
        !tryGetJsonString(json, "geometryState", geometryState) ||
        !parseRescueZoneGeometryState(
            geometryState, observation.geometryState) ||
        !tryGetJsonBool(json, "aimValid", observation.aimValid) ||
        !tryGetJsonNumber(json, "frameCoverage", observation.frameCoverage) ||
        !std::isfinite(observation.frameCoverage) ||
        observation.frameCoverage < 0.0 ||
        observation.frameCoverage > 1.0)
    {
        return false;
    }
    if (observation.aimValid &&
        (!tryGetJsonNumber(json, "aimX", observation.aimX) ||
         !std::isfinite(observation.aimX) ||
         !tryGetJsonNumber(
             json, "aimNormalized", observation.aimNormalized) ||
         !std::isfinite(observation.aimNormalized) ||
         observation.aimNormalized < -1.0 ||
         observation.aimNormalized > 1.0))
    {
        return false;
    }
    return true;
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
        const unsigned long long parsedValue =
            std::stoull(text, &parsedCharacters);
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

double getJsonNumber(
    const std::string& json,
    const std::string& key,
    double fallback)
{
    double value = fallback;
    return tryGetJsonNumber(json, key, value) ? value : fallback;
}

bool getJsonBool(
    const std::string& json,
    const std::string& key,
    bool fallback)
{
    bool value = fallback;
    return tryGetJsonBool(json, key, value) ? value : fallback;
}

bool parseGreenInterpretation(
    const std::string& interpretation,
    GreenInterpretation& result)
{
    if (interpretation == "SEM_DECISAO")
    {
        result = GreenInterpretation::None;
        return true;
    }
    if (interpretation == "VERDE_FALSO")
    {
        result = GreenInterpretation::FalseMarker;
        return true;
    }
    if (interpretation == "AMBIGUO")
    {
        result = GreenInterpretation::Ambiguous;
        return true;
    }
    if (interpretation == "ESQUERDA")
    {
        result = GreenInterpretation::Left;
        return true;
    }
    if (interpretation == "DIREITA")
    {
        result = GreenInterpretation::Right;
        return true;
    }
    if (interpretation == "RETORNO_180")
    {
        result = GreenInterpretation::TurnAround180;
        return true;
    }
    return false;
}

bool parseCourseMarker(
    const std::string& marker,
    CourseMarker& result)
{
    if (marker == "NONE")
    {
        result = CourseMarker::None;
        return true;
    }
    if (marker == "GRAY")
    {
        result = CourseMarker::Gray;
        return true;
    }
    if (marker == "RED")
    {
        result = CourseMarker::Red;
        return true;
    }
    return false;
}

bool isNormalizedValue(double value)
{
    return std::isfinite(value) && value >= -1.0 && value <= 1.0;
}

bool isTrustedDirection(const std::string& direction)
{
    return direction == "NONE" || direction == "LEFT" ||
           direction == "RIGHT";
}

CameraLineSnapshot unavailableLineSnapshot(
    const CameraLineSnapshot& cachedSnapshot,
    bool hasCachedSnapshot)
{
    CameraLineSnapshot snapshot =
        hasCachedSnapshot ? cachedSnapshot : CameraLineSnapshot{};
    snapshot.sourceFresh = false;
    snapshot.lineFollowerLeftPower = 0.0;
    snapshot.lineFollowerRightPower = 0.0;
    snapshot.lineControlSource = "unavailable";
    snapshot.lineNearDetected = false;
    snapshot.lineNearFinePosition =
        std::numeric_limits<double>::quiet_NaN();
    snapshot.farTrusted = false;
    snapshot.mediumTrusted = false;
    snapshot.normalSteeringValid = false;
    snapshot.trustedDirection = "NONE";
    snapshot.greenPathBlackValid = false;
    snapshot.greenCandidateCount = 0;
    snapshot.greenConfirmed = false;
    snapshot.greenInterpretation = GreenInterpretation::None;
    snapshot.courseMarkerConfirmed = false;
    snapshot.courseMarker = CourseMarker::None;
    snapshot.silverCandidateDetected = false;
    snapshot.rescueExitConfirmed = false;
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


ForwardLineSnapshot unavailableForwardLineSnapshot(
    const ForwardLineSnapshot& cachedSnapshot,
    bool hasCachedSnapshot)
{
    ForwardLineSnapshot snapshot = hasCachedSnapshot
                                       ? cachedSnapshot
                                       : ForwardLineSnapshot{};
    snapshot.sourceFresh = false;
    snapshot.visible = false;
    snapshot.position = std::numeric_limits<double>::quiet_NaN();
    snapshot.normalLeftPower = 0.0;
    snapshot.normalRightPower = 0.0;
    if (hasCachedSnapshot)
    {
        snapshot.ageMs =
            (currentUnixSeconds() - snapshot.timestamp) * 1000.0;
        if (!std::isfinite(snapshot.ageMs))
        {
            snapshot.ageMs = 0.0;
        }
    }
    return snapshot;
}
}

bool ForwardLineSnapshot::lineObservationValid() const
{
    return sourceFresh && visible && sequence > 0 &&
           isNormalizedValue(position);
}

bool ForwardLineSnapshot::normalCommandValid() const
{
    // Contrato mantido para consumidores antigos: a frontal não comanda motores.
    return false;
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
        std::string lineControlSource;
        if (!tryGetJsonNumber(
                json,
                "lineFollowerLeftPower",
                candidate.lineFollowerLeftPower) ||
            !tryGetJsonNumber(
                json,
                "lineFollowerRightPower",
                candidate.lineFollowerRightPower) ||
            !tryGetJsonBool(
                json, "lineNearDetected", candidate.lineNearDetected) ||
            !tryGetJsonBool(
                json,
                "greenPathBlackValid",
                candidate.greenPathBlackValid) ||
            !tryGetJsonUnsignedInteger(
                json,
                "greenCandidateCount",
                candidate.greenCandidateCount) ||
            !tryGetJsonBool(
                json, "greenConfirmed", candidate.greenConfirmed) ||
            !tryGetJsonString(
                json, "greenInterpretation", greenInterpretation) ||
            !parseGreenInterpretation(
                greenInterpretation,
                candidate.greenInterpretation) ||
            !tryGetJsonNumber(
                json, "lineTimestamp", candidate.lineTimestamp) ||
            !tryGetJsonUnsignedInteger(
                json, "lineSequence", candidate.lineSequence))
        {
            return unavailableLineSnapshot(
                cachedLineSnapshot_, hasCachedLineSnapshot_);
        }

        // Os marcadores são opcionais até o detector físico ser calibrado.
        // Um processo antigo nunca inicia resgate nem conclui a missão por engano.
        bool courseMarkerConfirmed = false;
        if (tryGetJsonBool(
                json, "courseMarkerConfirmed", courseMarkerConfirmed))
        {
            candidate.courseMarkerConfirmed = courseMarkerConfirmed;
        }
        std::string courseMarker;
        if (tryGetJsonString(json, "courseMarker", courseMarker))
        {
            if (!parseCourseMarker(courseMarker, candidate.courseMarker))
            {
                return unavailableLineSnapshot(
                    cachedLineSnapshot_, hasCachedLineSnapshot_);
            }
        }
        if (candidate.courseMarkerConfirmed &&
            candidate.courseMarker == CourseMarker::None)
        {
            return unavailableLineSnapshot(
                cachedLineSnapshot_, hasCachedLineSnapshot_);
        }
        tryGetJsonBool(
            json, "silverCandidateDetected", candidate.silverCandidateDetected);
        tryGetJsonBool(
            json, "rescueExitConfirmed", candidate.rescueExitConfirmed);

        // A origem do controle é opcional para manter compatibilidade com um
        // processo de câmera antigo. Este campo serve somente ao diagnóstico.
        if (tryGetJsonString(json, "lineControlSource", lineControlSource))
        {
            candidate.lineControlSource = lineControlSource;
        }

        // A posição fina é opcional para manter compatibilidade com processos
        // de câmera antigos. Sem ela, a missão simplesmente pula a correção.
        tryGetJsonNumber(
            json, "nearFinePosition", candidate.lineNearFinePosition);

        // O baseline abaixo é opcional e nunca invalida o comando visual.
        // Ausência ou valor inválido permanece como NaN/INVALID no CSV.
        CameraCurveDiagnostics& diagnostics = candidate.curveDiagnostics;
        diagnostics.nearFinePosition = candidate.lineNearFinePosition;
        tryGetJsonNumber(
            json, "mediumPosition", diagnostics.mediumPosition);
        tryGetJsonNumber(
            json, "farBandPosition", diagnostics.farBandPosition);
        tryGetJsonNumber(
            json, "headingAngleDeg", diagnostics.headingAngleDeg);
        tryGetJsonNumber(
            json, "finalSteering", diagnostics.finalSteering);
        tryGetJsonString(json, "vstate", diagnostics.virtualState);
        tryGetJsonString(json, "lineState", diagnostics.lineState);
        std::string decision;
        std::string nearState;
        if (tryGetJsonString(json, "nearLineState", nearState) &&
            (nearState == "PRESENT" || nearState == "LOST" || nearState == "UNKNOWN"))
        {
            candidate.nearLineState = nearState;
        }
        if (tryGetJsonString(json, "gapValidationDecision", decision))
        {
            if (decision != "NORMAL" && decision != "CHECKING" &&
                decision != "GAP" && decision != "LOST")
            {
                return unavailableLineSnapshot(cachedLineSnapshot_, hasCachedLineSnapshot_);
            }
            candidate.gapValidationDecision = decision;
        }

        // Os gates trusted são opcionais somente para compatibilidade com uma
        // câmera antiga. Ausência mantém ambos falsos e bloqueia a assistência.
        tryGetJsonBool(json, "farTrusted", candidate.farTrusted);
        tryGetJsonBool(json, "mediumTrusted", candidate.mediumTrusted);
        std::string trustedDirection;
        if (tryGetJsonString(json, "trustedDirection", trustedDirection))
        {
            if (!isTrustedDirection(trustedDirection))
            {
                return unavailableLineSnapshot(
                    cachedLineSnapshot_, hasCachedLineSnapshot_);
            }
            candidate.trustedDirection = trustedDirection;
        }
        // O virtual permanece limitado à faixa positiva NORMAL. O Fusion pode
        // comandar a roda interna em ré enquanto seu target atual continua
        // válido; GREEN, GAP, recovery e fontes críticas continuam vetados.
        const bool virtualNormalCommand =
            candidate.lineControlSource == "virtual" &&
            candidate.lineFollowerLeftPower >=
                config::kForwardAssistNormalMinimumPower &&
            candidate.lineFollowerLeftPower <=
                config::kForwardAssistNormalMaximumPower &&
            candidate.lineFollowerRightPower >=
                config::kForwardAssistNormalMinimumPower &&
            candidate.lineFollowerRightPower <=
                config::kForwardAssistNormalMaximumPower;
        const bool fusionNormalCommand =
            candidate.lineControlSource == "fusion" &&
            candidate.lineFollowerLeftPower >=
                config::kForwardAssistFusionMinimumPower &&
            candidate.lineFollowerLeftPower <=
                config::kForwardAssistFusionMaximumPower &&
            candidate.lineFollowerRightPower >=
                config::kForwardAssistFusionMinimumPower &&
            candidate.lineFollowerRightPower <=
                config::kForwardAssistFusionMaximumPower;
        candidate.normalSteeringValid =
            (virtualNormalCommand || fusionNormalCommand) &&
            isNormalizedValue(diagnostics.finalSteering);
        // Validade do comando NORMAL não afirma presença física de fita.
        // O gate local é decidido no Python e nunca é vetado por este campo.

        const bool valuesValid =
            isNormalizedValue(candidate.lineFollowerLeftPower) &&
            isNormalizedValue(candidate.lineFollowerRightPower) &&
            candidate.greenCandidateCount <= 1000 &&
            std::isfinite(candidate.lineTimestamp);
        if (!valuesValid)
        {
            return unavailableLineSnapshot(
                cachedLineSnapshot_, hasCachedLineSnapshot_);
        }

        candidate.ageMs =
            (currentUnixSeconds() - candidate.lineTimestamp) * 1000.0;
        candidate.sourceFresh =
            std::isfinite(candidate.ageMs) &&
            candidate.ageMs >= 0.0 &&
            candidate.ageMs <= config::kCameraLineStatusTimeoutMs;
        if (!candidate.sourceFresh)
        {
            return unavailableLineSnapshot(
                candidate, true);
        }

        cachedLineSnapshot_ = candidate;
        hasCachedLineSnapshot_ = true;
        return candidate;
    }
    catch (const std::exception&)
    {
        return unavailableLineSnapshot(
            cachedLineSnapshot_, hasCachedLineSnapshot_);
    }
}

ForwardLineSnapshot CameraMonitor::forwardLineSnapshot()
{
    try
    {
        std::ifstream file(forwardLineStatusPath_.empty()
                               ? config::kForwardLineStatusPath : forwardLineStatusPath_);
        if (!file)
        {
            return unavailableForwardLineSnapshot(
                cachedForwardLineSnapshot_,
                hasCachedForwardLineSnapshot_);
        }
        std::ostringstream content;
        content << file.rdbuf();
        if (file.bad())
        {
            return unavailableForwardLineSnapshot(
                cachedForwardLineSnapshot_,
                hasCachedForwardLineSnapshot_);
        }
        const std::string json = content.str();

        ForwardLineSnapshot candidate;
        std::uint64_t pathVersion = 0;
        if (!tryGetJsonUnsignedInteger(json, "forwardPathVersion", pathVersion) || pathVersion != 2 ||
            !tryGetJsonString(json, "forwardPathState", candidate.pathState) ||
            !tryGetJsonBool(json, "forwardLinePresent", candidate.present) ||
            !tryGetJsonBool(
                json, "forwardLineVisible", candidate.visible) ||
            !tryGetJsonNumber(
                json, "forwardPathConfidence", candidate.confidence) ||
            !tryGetJsonUnsignedInteger(
                json, "forwardLineSequence", candidate.sequence) ||
            !tryGetJsonNumber(
                json, "forwardLineTimestamp", candidate.timestamp))
        {
            return unavailableForwardLineSnapshot(
                cachedForwardLineSnapshot_,
                hasCachedForwardLineSnapshot_);
        }

        if (candidate.visible &&
            !tryGetJsonNumber(json, "forwardLinePosition", candidate.position))
        {
            return unavailableForwardLineSnapshot(
                cachedForwardLineSnapshot_,
                hasCachedForwardLineSnapshot_);
        }

        // Campos obstacleBlack são aditivos. Um produtor antigo ou uma leitura
        // inválida não pode derrubar o contrato frontal usado por GAP/LOST.
        const bool obstacleBlackValid =
            tryGetJsonBool(
                json, "obstacleBlackVisible", candidate.obstacleBlackVisible) &&
            tryGetJsonUnsignedInteger(
                json,
                "obstacleBlackPixelCount",
                candidate.obstacleBlackPixelCount) &&
            tryGetJsonNumber(
                json, "obstacleBlackRatio", candidate.obstacleBlackRatio) &&
            tryGetJsonUnsignedInteger(
                json,
                "obstacleBlackLargestComponent",
                candidate.obstacleBlackLargestComponent) &&
            tryGetJsonUnsignedInteger(
                json,
                "obstacleBlackSequence",
                candidate.obstacleBlackSequence) &&
            std::isfinite(candidate.obstacleBlackRatio) &&
            candidate.obstacleBlackRatio >= 0.0 &&
            candidate.obstacleBlackRatio <= 1.0;
        if (!obstacleBlackValid)
        {
            candidate.obstacleBlackVisible = false;
            candidate.obstacleBlackPixelCount = 0;
            candidate.obstacleBlackRatio = 0.0;
            candidate.obstacleBlackLargestComponent = 0;
            candidate.obstacleBlackSequence = 0;
        }

        // O caso 3 recebe apenas contagens simples da CAM1. Ausência desses
        // campos mantém o desvio e todos os consumidores globais inalterados.
        const bool parabolaBlackValid =
            tryGetJsonUnsignedInteger(
                json, "parabolaLeftBlack", candidate.parabolaLeftBlack) &&
            tryGetJsonUnsignedInteger(
                json, "parabolaRightBlack", candidate.parabolaRightBlack) &&
            tryGetJsonUnsignedInteger(
                json, "parabolaSequence", candidate.parabolaSequence) &&
            tryGetJsonUnsignedInteger(
                json,
                "parabolaNearForwardBlack",
                candidate.parabolaNearForwardBlack) &&
            tryGetJsonUnsignedInteger(
                json,
                "parabolaNearForwardLargest",
                candidate.parabolaNearForwardLargest) &&
            tryGetJsonBool(
                json,
                "parabolaNearForwardVisible",
                candidate.parabolaNearForwardVisible);
        if (!parabolaBlackValid)
        {
            candidate.parabolaLeftBlack = 0;
            candidate.parabolaRightBlack = 0;
            candidate.parabolaSequence = 0;
            candidate.parabolaNearForwardBlack = 0;
            candidate.parabolaNearForwardLargest = 0;
            candidate.parabolaNearForwardVisible = false;
        }

        const bool valuesValid =
            candidate.sequence > 0 &&
            (candidate.pathState == "PRESENT" || candidate.pathState == "UNCERTAIN" ||
             candidate.pathState == "ABSENT") &&
            (candidate.present == (candidate.pathState == "PRESENT")) &&
            (!candidate.present || (candidate.visible && candidate.confidence == 1.0)) &&
            std::isfinite(candidate.timestamp) &&
            candidate.timestamp >= 0.0 &&
            std::isfinite(candidate.confidence) &&
            candidate.confidence >= 0.0 &&
            candidate.confidence <= 1.0 &&
            (!candidate.visible || isNormalizedValue(candidate.position));
        if (!valuesValid)
        {
            return unavailableForwardLineSnapshot(
                cachedForwardLineSnapshot_,
                hasCachedForwardLineSnapshot_);
        }

        candidate.ageMs =
            (currentUnixSeconds() - candidate.timestamp) * 1000.0;
        candidate.sourceFresh =
            std::isfinite(candidate.ageMs) && candidate.ageMs >= 0.0 &&
            candidate.ageMs <= config::kForwardLineStatusTimeoutMs;
        if (!candidate.sourceFresh)
        {
            return unavailableForwardLineSnapshot(candidate, true);
        }

        cachedForwardLineSnapshot_ = candidate;
        hasCachedForwardLineSnapshot_ = true;
        return candidate;
    }
    catch (const std::exception&)
    {
        return unavailableForwardLineSnapshot(
            cachedForwardLineSnapshot_,
            hasCachedForwardLineSnapshot_);
    }
}

ForwardBallSnapshot CameraMonitor::forwardBallSnapshot() const
{
    ForwardBallSnapshot snapshot;
    try
    {
        std::ifstream file(config::kForwardBallStatusPath);
        if (!file)
        {
            return snapshot;
        }
        std::ostringstream content;
        content << file.rdbuf();
        if (file.bad())
        {
            return snapshot;
        }
        const std::string json = content.str();
        const bool active = getJsonBool(json, "active", false);
        if (!tryGetJsonBool(json, "ballDetected", snapshot.detected) ||
            !tryGetJsonBool(
                json, "ballCandidateVisible", snapshot.candidateVisible) ||
            !tryGetJsonNumber(json, "timestamp", snapshot.timestamp) ||
            !tryGetJsonUnsignedInteger(
                json, "targetSequence", snapshot.targetSequence) ||
            !tryGetJsonBool(json, "targetLocked", snapshot.targetLocked))
        {
            return ForwardBallSnapshot{};
        }

        snapshot.ageMs = (currentUnixSeconds() - snapshot.timestamp) * 1000.0;
        snapshot.sourceFresh = active && std::isfinite(snapshot.ageMs) &&
                               snapshot.ageMs >= 0.0 &&
                               snapshot.ageMs <=
                                   config::kForwardBallStatusTimeoutMs;
        if (!snapshot.sourceFresh || !snapshot.detected)
        {
            snapshot.detected = false;
            return snapshot;
        }

        if (!tryGetJsonString(json, "ballType", snapshot.type) ||
            !tryGetJsonNumber(json, "ballTxDegrees", snapshot.txDegrees) ||
            !tryGetJsonNumber(json, "ballDistanceCm", snapshot.distanceCm) ||
            !tryGetJsonNumber(json, "ballRadiusPixels", snapshot.radiusPixels) ||
            !tryGetJsonNumber(
                json, "visibleAreaPixels", snapshot.visibleAreaPixels))
        {
            return ForwardBallSnapshot{};
        }
        const bool valuesValid = !snapshot.type.empty() &&
                                 std::isfinite(snapshot.txDegrees) &&
                                 std::abs(snapshot.txDegrees) <= 45.0 &&
                                 std::isfinite(snapshot.distanceCm) &&
                                 snapshot.distanceCm > 0.0 &&
                                 std::isfinite(snapshot.radiusPixels) &&
                                 snapshot.radiusPixels > 0.0 &&
                                 std::isfinite(snapshot.visibleAreaPixels) &&
                                 snapshot.visibleAreaPixels > 0.0 &&
                                 snapshot.targetLocked;
        if (!valuesValid)
        {
            return ForwardBallSnapshot{};
        }
        return snapshot;
    }
    catch (const std::exception&)
    {
        return ForwardBallSnapshot{};
    }
}

RescueZoneSnapshot CameraMonitor::rescueZoneSnapshot() const
{
    RescueZoneSnapshot snapshot;
    try
    {
        const std::string& configuredPath = rescueZoneStatusPath_.empty()
                                                ? config::kRescueZoneStatusPath
                                                : rescueZoneStatusPath_;
        std::ifstream file(configuredPath);
        if (!file)
        {
            return snapshot;
        }
        std::ostringstream content;
        content << file.rdbuf();
        if (file.bad())
        {
            return snapshot;
        }

        const std::string json = content.str();
        bool active = false;
        std::string greenJson;
        std::string redJson;
        if (!tryGetJsonBool(json, "active", active) ||
            !tryGetJsonNumber(json, "timestamp", snapshot.timestamp) ||
            !tryGetJsonUnsignedInteger(json, "sequence", snapshot.sequence) ||
            !tryGetJsonBool(json, "cameraObscured", snapshot.cameraObscured) ||
            !tryGetJsonObject(json, "green", greenJson) ||
            !tryGetJsonObject(json, "red", redJson) ||
            !parseRescueZoneObservation(greenJson, snapshot.green) ||
            !parseRescueZoneObservation(redJson, snapshot.red))
        {
            return RescueZoneSnapshot{};
        }

        snapshot.ageMs = (currentUnixSeconds() - snapshot.timestamp) * 1000.0;
        snapshot.sourceFresh = active && snapshot.sequence > 0 &&
                               std::isfinite(snapshot.ageMs) &&
                               snapshot.ageMs >= 0.0 &&
                               snapshot.ageMs <=
                                   config::kRescueZoneStatusTimeoutMs;
        if (!snapshot.sourceFresh)
        {
            snapshot.green = {};
            snapshot.red = {};
        }
        return snapshot;
    }
    catch (const std::exception&)
    {
        return RescueZoneSnapshot{};
    }
}

bool CameraMonitor::setForwardBallDetectionEnabled(bool enabled) const
{
#ifdef _WIN32
    (void)enabled;
    return true;
#else
    {
        std::ofstream control(
            config::kForwardBallDetectionTemporaryControlPath,
            std::ios::trunc);
        if (!control)
        {
            return false;
        }
        control << (enabled ? "1\n" : "0\n");
        if (!control)
        {
            return false;
        }
    }
    if (std::rename(
            config::kForwardBallDetectionTemporaryControlPath,
            config::kForwardBallDetectionControlPath) != 0)
    {
        std::remove(config::kForwardBallDetectionTemporaryControlPath);
        return false;
    }
    return true;
#endif
}

bool CameraMonitor::requestForwardBallTargetSequence(
    std::uint64_t sequence) const
{
#ifdef _WIN32
    (void)sequence;
    return true;
#else
    {
        std::ofstream control(
            config::kForwardBallTargetSequenceTemporaryControlPath,
            std::ios::trunc);
        if (!control)
        {
            return false;
        }
        control << sequence << '\n';
        if (!control)
        {
            return false;
        }
    }
    if (std::rename(
            config::kForwardBallTargetSequenceTemporaryControlPath,
            config::kForwardBallTargetSequenceControlPath) != 0)
    {
        std::remove(config::kForwardBallTargetSequenceTemporaryControlPath);
        return false;
    }
    return true;
#endif
}

bool CameraMonitor::publishRescueZoneDetectionInput(
    bool enabled,
    bool ultrasonicFresh,
    bool ultrasonicValid,
    double ultrasonicDistanceCm) const
{
#ifdef _WIN32
    (void)enabled;
    (void)ultrasonicFresh;
    (void)ultrasonicValid;
    (void)ultrasonicDistanceCm;
    return true;
#else
    // O mesmo arquivo atômico transporta o gate e a profundidade frontal.
    // O Python ainda revalida idade, faixa e flags antes de desenhar a leitura.
    {
        std::ofstream control(
            config::kRescueZoneDetectionTemporaryControlPath,
            std::ios::trunc);
        if (!control)
        {
            return false;
        }
        control << "{\"enabled\":" << (enabled ? "true" : "false")
                << ",\"ultrasonicFresh\":"
                << (ultrasonicFresh ? "true" : "false")
                << ",\"ultrasonicValid\":"
                << (ultrasonicValid ? "true" : "false")
                << ",\"ultrasonicDistanceCm\":";
        if (ultrasonicValid && std::isfinite(ultrasonicDistanceCm))
        {
            control << ultrasonicDistanceCm;
        }
        else
        {
            control << "null";
        }
        // A precisão padrão arredonda o Unix time atual em centenas de
        // segundos. Isso faria a CAM1 rejeitar imediatamente o heartbeat como
        // stale e esconder o overlay das áreas.
        control << ",\"timestamp\":" << std::fixed << std::setprecision(6)
                << currentUnixSeconds() << "}\n";
        if (!control)
        {
            return false;
        }
    }
    if (std::rename(
            config::kRescueZoneDetectionTemporaryControlPath,
            config::kRescueZoneDetectionControlPath) != 0)
    {
        std::remove(config::kRescueZoneDetectionTemporaryControlPath);
        return false;
    }
    if (!enabled)
    {
        // O C++ também invalida o resultado para que STOP e E-Stop não
        // dependam do próximo frame da câmera para apagar uma leitura antiga.
        std::remove(config::kRescueZoneStatusPath);
    }
    return true;
#endif
}
