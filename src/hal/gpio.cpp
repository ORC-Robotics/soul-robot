#include "obr/gpio.h"

#include "obr/config.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
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
    std::ifstream chip("/sys/class/gpio/gpiochip512/label");
    std::string label;
    chip >> label;

    if (label.find("pinctrl") != std::string::npos)
    {
        return 512;
    }

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
