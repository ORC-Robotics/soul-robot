#include "obr/gpio.h"

#include "obr/config.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>

GpioPin::GpioPin(int bcmPin)
    : pin_(bcmPin)
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
    if (!writeFile("/sys/class/gpio/export", std::to_string(pin_)))
    {
        std::ifstream existing(gpioPath("direction"));
        if (!existing.good())
        {
            std::cerr << "GPIO " << pin_ << " export failed\n";
            return false;
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(config::kGpioExportDelayMs));

    if (!writeFile(gpioPath("direction"), "out"))
    {
        std::cerr << "GPIO " << pin_ << " direction failed\n";
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
    return pin_;
}

std::string GpioPin::gpioPath(const std::string& file) const
{
    std::ostringstream path;
    path << "/sys/class/gpio/gpio" << pin_ << "/" << file;
    return path.str();
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
