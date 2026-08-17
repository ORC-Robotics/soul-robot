#include "obr/gpio.h"

#include "obr/config.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <system_error>
#include <thread>

GpioPin::GpioPin(int bcmPin)
    : bcmPin_(bcmPin), gpioNumber_(detectMainGpioBase() + bcmPin)
{
}

GpioPin::~GpioPin()
{
    write(false);
}

bool GpioPin::beginOutput()
{
#ifdef _WIN32
    return true;
#else
    if (!writeFile("/sys/class/gpio/export", std::to_string(gpioNumber_)))
    {
        std::ifstream existing(gpioPath("direction"));
        if (!existing.good())
        {
            std::cerr << "GPIO BCM " << bcmPin_ << " export failed\n";
            return false;
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(config::kGpioExportDelayMs));

    if (!writeFile(gpioPath("direction"), "out"))
    {
        std::cerr << "GPIO BCM " << bcmPin_ << " direction failed\n";
        return false;
    }

    return write(false);
#endif
}

bool GpioPin::write(bool high)
{
#ifdef _WIN32
    (void)high;
    return true;
#else
    return writeFile(gpioPath("value"), high ? "1" : "0");
#endif
}

int GpioPin::pin() const
{
    return bcmPin_;
}

std::string GpioPin::gpioPath(const std::string& file) const
{
    std::ostringstream path;
    path << "/sys/class/gpio/gpio" << gpioNumber_ << "/" << file;
    return path.str();
}

int GpioPin::detectMainGpioBase()
{
#ifdef _WIN32
    return 0;
#else
    // A base global do GPIO varia entre os modelos e versões do kernel.
    // Na Raspberry Pi 5, os pinos do conector pertencem ao controlador RP1.
    const std::filesystem::path gpioClassPath("/sys/class/gpio");
    std::error_code error;
    for (const auto& entry : std::filesystem::directory_iterator(gpioClassPath, error))
    {
        const std::string chipName = entry.path().filename().string();
        if (chipName.rfind("gpiochip", 0) != 0)
        {
            continue;
        }

        std::ifstream labelFile(entry.path() / "label");
        std::string label;
        std::getline(labelFile, label);
        if (label.find("pinctrl") == std::string::npos)
        {
            continue;
        }

        std::ifstream baseFile(entry.path() / "base");
        int base = 0;
        if (baseFile >> base)
        {
            return base;
        }
    }

    // Kernels antigos expõem os GPIOs do conector a partir de zero.
    return 0;
#endif
}

bool GpioPin::writeFile(const std::string& path, const std::string& value)
{
    std::ofstream file(path);
    if (!file.good())
    {
        return false;
    }

    file << value;
    return file.good();
}
