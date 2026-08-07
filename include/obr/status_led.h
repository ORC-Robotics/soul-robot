#pragma once

#include "obr/gpio.h"

#include <atomic>

// Controla o LED que indica quando a Raspberry Pi e a ESP32 estão comunicando.
// A classe evita escritas repetidas no GPIO durante o loop rápido do robô.
class StatusLed
{
public:
    explicit StatusLed(int bcmPin);

    bool begin();
    void setReady(bool ready);
    bool isReady() const;
    void off();

private:
    GpioPin pin_;
    bool initialized_ = false;
    std::atomic<bool> ready_{false};
};
