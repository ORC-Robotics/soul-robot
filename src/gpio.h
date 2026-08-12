#pragma once

#include <string>

class GpioPin
{
public:
    explicit GpioPin(int bcmPin);
    ~GpioPin();

    GpioPin(const GpioPin&) = delete;
    GpioPin& operator=(const GpioPin&) = delete;

    bool beginOutput();
    bool write(bool high);
    int pin() const;

private:
    int pin_;

    std::string gpioPath(const std::string& file) const;
    static bool writeFile(const std::string& path, const std::string& value);
};
