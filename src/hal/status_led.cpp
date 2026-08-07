#include "obr/status_led.h"

#include <iostream>

StatusLed::StatusLed(int bcmPin)
    : pin_(bcmPin)
{
}

bool StatusLed::begin()
{
    initialized_ = pin_.beginOutput();
    ready_ = false;

    if (!initialized_)
    {
        std::cerr << "System ready LED initialization failed on BCM GPIO " << pin_.pin() << "\n";
    }

    return initialized_;
}

void StatusLed::setReady(bool ready)
{
    if (!initialized_ || ready == ready_.load())
    {
        return;
    }

    if (!pin_.write(ready))
    {
        std::cerr << "System ready LED write failed on BCM GPIO " << pin_.pin() << "\n";
        return;
    }

    ready_ = ready;
    std::cout << "System ready LED " << (ready ? "on" : "off") << "\n";
}

bool StatusLed::isReady() const
{
    return ready_.load();
}

void StatusLed::off()
{
    if (!initialized_)
    {
        return;
    }

    pin_.write(false);
    ready_ = false;
}
