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
    snapshot.nearError = 0.0;
    snapshot.controlError = 0.0;
    snapshot.correction = 0.0;
    snapshot.leftPreview = 0.0;
    snapshot.rightPreview = 0.0;
    snapshot.farValid = false;
    snapshot.farError = 0.0;
    snapshot.farArea = 0.0;
    snapshot.centerDeltaValid = false;
    snapshot.centerDeltaPx = 0.0;
    snapshot.gapCandidate = false;
    snapshot.gapAlignmentValid = false;
    snapshot.gapAlignmentError = 0.0;
    snapshot.gapReturnValid = false;
    snapshot.gapReturnError = 0.0;
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
        if (!tryGetJsonBool(json, "nearValid", candidate.nearValid) ||
            !tryGetJsonNumber(json, "nearError", candidate.nearError) ||
            !tryGetJsonNumber(json, "controlError", candidate.controlError) ||
            !tryGetJsonNumber(json, "correction", candidate.correction) ||
            !tryGetJsonNumber(json, "leftPreview", candidate.leftPreview) ||
            !tryGetJsonNumber(json, "rightPreview", candidate.rightPreview) ||
            !tryGetJsonBool(json, "farValid", candidate.farValid) ||
            !tryGetJsonNumber(json, "farError", candidate.farError) ||
            !tryGetJsonNumber(json, "farArea", candidate.farArea) ||
            !tryGetJsonBool(
                json, "centerDeltaValid", candidate.centerDeltaValid) ||
            !tryGetJsonNumber(
                json, "centerDeltaPx", candidate.centerDeltaPx) ||
            !tryGetJsonBool(json, "gapCandidate", candidate.gapCandidate) ||
            !tryGetJsonBool(
                json, "gapAlignmentValid", candidate.gapAlignmentValid) ||
            !tryGetJsonNumber(
                json, "gapAlignmentError", candidate.gapAlignmentError) ||
            !tryGetJsonBool(
                json, "gapReturnValid", candidate.gapReturnValid) ||
            !tryGetJsonNumber(
                json, "gapReturnError", candidate.gapReturnError) ||
            !tryGetJsonNumber(json, "lineTimestamp", candidate.lineTimestamp) ||
            !tryGetJsonUnsignedInteger(
                json, "lineSequence", candidate.lineSequence))
        {
            return unavailableLineSnapshot(
                cachedLineSnapshot_, hasCachedLineSnapshot_);
        }

        const bool valuesValid =
            isNormalizedValue(candidate.nearError) &&
            isNormalizedValue(candidate.controlError) &&
            isNormalizedValue(candidate.correction) &&
            isNormalizedValue(candidate.leftPreview) &&
            isNormalizedValue(candidate.rightPreview) &&
            isNormalizedValue(candidate.farError) &&
            std::isfinite(candidate.farArea) && candidate.farArea >= 0.0 &&
            std::isfinite(candidate.centerDeltaPx) &&
            (!candidate.centerDeltaValid ||
             (candidate.nearValid && candidate.farValid)) &&
            isNormalizedValue(candidate.gapAlignmentError) &&
            isNormalizedValue(candidate.gapReturnError) &&
            (!candidate.gapCandidate || candidate.nearValid) &&
            (!candidate.gapAlignmentValid || candidate.gapCandidate) &&
            (!candidate.gapReturnValid || candidate.gapCandidate) &&
            std::isfinite(candidate.lineTimestamp);
        if (!valuesValid)
        {
            return unavailableLineSnapshot(
                cachedLineSnapshot_, hasCachedLineSnapshot_);
        }

        if (!candidate.nearValid)
        {
            candidate.nearError = 0.0;
            candidate.controlError = 0.0;
            candidate.correction = 0.0;
            candidate.leftPreview = 0.0;
            candidate.rightPreview = 0.0;
        }
        if (!candidate.farValid)
        {
            candidate.farError = 0.0;
            candidate.farArea = 0.0;
        }
        if (!candidate.centerDeltaValid)
        {
            candidate.centerDeltaPx = 0.0;
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
            snapshot.nearError = 0.0;
            snapshot.controlError = 0.0;
            snapshot.correction = 0.0;
            snapshot.leftPreview = 0.0;
            snapshot.rightPreview = 0.0;
            snapshot.farValid = false;
            snapshot.farError = 0.0;
            snapshot.farArea = 0.0;
            snapshot.centerDeltaValid = false;
            snapshot.centerDeltaPx = 0.0;
            snapshot.gapCandidate = false;
            snapshot.gapAlignmentValid = false;
            snapshot.gapAlignmentError = 0.0;
            snapshot.gapReturnValid = false;
            snapshot.gapReturnError = 0.0;
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
