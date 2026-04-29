#pragma once

#include <string>

// Encapsula um pino GPIO da Raspberry Pi usando a interface /sys/class/gpio.
// Todas as escritas passam por esta classe para facilitar auditoria de hardware.
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
