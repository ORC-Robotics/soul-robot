#include "obr/camera_monitor.h"

#include "obr/config.h"

#include <chrono>
#include <cmath>
#include <cstdio>
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

bool isNormalizedValue(double value)
{
    return std::isfinite(value) && value >= -1.0 && value <= 1.0;
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
    snapshot.lineNearAnyDetected = false;
    snapshot.greenPathBlackValid = false;
    snapshot.greenCandidateCount = 0;
    snapshot.greenConfirmed = false;
    snapshot.greenInterpretation = GreenInterpretation::None;
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

        // A origem do controle é opcional para manter compatibilidade com um
        // processo de câmera antigo. Este campo serve somente ao diagnóstico.
        if (tryGetJsonString(json, "lineControlSource", lineControlSource))
        {
            candidate.lineControlSource = lineControlSource;
        }
        // Processos de câmera antigos não publicam a faixa NEAR completa.
        // Nesse caso, o centro continua sendo o fallback seguro e compatível.
        candidate.lineNearAnyDetected = candidate.lineNearDetected;
        tryGetJsonBool(
            json, "lineNearAnyDetected", candidate.lineNearAnyDetected);

        // O baseline abaixo é opcional e nunca invalida o comando visual.
        // Ausência ou valor inválido permanece como NaN/INVALID no CSV.
        CameraCurveDiagnostics& diagnostics = candidate.curveDiagnostics;
        tryGetJsonNumber(
            json, "nearFinePosition", diagnostics.nearFinePosition);
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
