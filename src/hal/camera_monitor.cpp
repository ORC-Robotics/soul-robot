#include "obr/camera_monitor.h"

#include "obr/config.h"

#include <chrono>
#include <cmath>
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
