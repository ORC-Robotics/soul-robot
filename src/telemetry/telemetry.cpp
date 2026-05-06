#include "obr/telemetry.h"

#include <fstream>
#include <string>
#include <sstream>

Telemetry::Telemetry()
    : previous_(readCpuSample())
{
}

TelemetrySample Telemetry::read()
{
    CpuSample current = readCpuSample();
    TelemetrySample sample;
    sample.cpuUsage = calculateCpuUsage(previous_, current);
    sample.temperature = readCpuTemperature();
    sample.ramUsage = readRamUsage();
    previous_ = current;
    return sample;
}

Telemetry::CpuSample Telemetry::readCpuSample()
{
    std::ifstream file("/proc/stat");

    std::string cpu;
    unsigned long long user = 0;
    unsigned long long nice = 0;
    unsigned long long system = 0;
    unsigned long long idle = 0;
    unsigned long long iowait = 0;
    unsigned long long irq = 0;
    unsigned long long softirq = 0;
    unsigned long long steal = 0;

    file >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;

    const unsigned long long idleAll = idle + iowait;
    const unsigned long long total = user + nice + system + idle + iowait + irq + softirq + steal;

    return {idleAll, total};
}

double Telemetry::calculateCpuUsage(const CpuSample& previous, const CpuSample& current)
{
    const unsigned long long idleDelta = current.idle - previous.idle;
    const unsigned long long totalDelta = current.total - previous.total;

    if (totalDelta == 0)
    {
        return 0.0;
    }

    return 100.0 * (1.0 - static_cast<double>(idleDelta) / totalDelta);
}

double Telemetry::readCpuTemperature()
{
    std::ifstream file("/sys/class/thermal/thermal_zone0/temp");
    int tempMilliCelsius = 0;
    file >> tempMilliCelsius;
    return tempMilliCelsius / 1000.0;
}

double Telemetry::readRamUsage()
{
    std::ifstream file("/proc/meminfo");
    std::string line;
    unsigned long long totalRam = 0;
    unsigned long long availableRam = 0;

    while (std::getline(file, line))
    {
        if (line.find("MemTotal:") == 0)
        {
            std::istringstream iss(line.substr(9));
            iss >> totalRam;
        }
        else if (line.find("MemAvailable:") == 0)
        {
            std::istringstream iss(line.substr(13));
            iss >> availableRam;
            break;
        }
    }

    if (totalRam == 0)
    {
        return 0.0;
    }

    // Calcula o uso de RAM em porcentagem
    unsigned long long usedRam = totalRam - availableRam;
    return 100.0 * static_cast<double>(usedRam) / totalRam;
}
