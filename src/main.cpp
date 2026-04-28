#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

bool running = true;

void handleSignal(int)
{
    running = false;
}

struct CpuSample
{
    unsigned long long idle;
    unsigned long long total;
};

CpuSample readCpuSample()
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

    unsigned long long idleAll = idle + iowait;
    unsigned long long total = user + nice + system + idle + iowait + irq + softirq + steal;

    return {idleAll, total};
}

double calculateCpuUsage(const CpuSample& previous, const CpuSample& current)
{
    unsigned long long idleDelta = current.idle - previous.idle;
    unsigned long long totalDelta = current.total - previous.total;

    if (totalDelta == 0)
    {
        return 0.0;
    }

    return 100.0 * (1.0 - static_cast<double>(idleDelta) / totalDelta);
}

double readCpuTemperature()
{
    std::ifstream file("/sys/class/thermal/thermal_zone0/temp");

    int tempMilliCelsius = 0;
    file >> tempMilliCelsius;

    return tempMilliCelsius / 1000.0;
}

int main()
{
    std::signal(SIGINT, handleSignal);

    std::cout << "Robot C++ iniciado na Raspberry Pi.\n";
    std::cout << "Pressione Ctrl+C para parar.\n\n";

    CpuSample previous = readCpuSample();

    while (running)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        CpuSample current = readCpuSample();

        double cpuUsage = calculateCpuUsage(previous, current);
        double temperature = readCpuTemperature();

        previous = current;

        std::cout << "CPU: " << cpuUsage << "% | Temp: " << temperature << " °C\n";
    }

    std::cout << "\nEncerrando.\n";

    return 0;
}
