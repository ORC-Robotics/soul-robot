#pragma once

struct TelemetrySample
{
    double cpuUsage = 0.0;
    double temperature = 0.0;
};

class Telemetry
{
public:
    Telemetry();

    TelemetrySample read();

private:
    struct CpuSample
    {
        unsigned long long idle = 0;
        unsigned long long total = 0;
    };

    CpuSample previous_;

    static CpuSample readCpuSample();
    static double calculateCpuUsage(const CpuSample& previous, const CpuSample& current);
    static double readCpuTemperature();
};
