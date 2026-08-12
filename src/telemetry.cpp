#include "telemetry.h"

#include <fstream>
#include <string>

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
