#pragma once

#include "obr/camera_monitor.h"

#include <chrono>
#include <string>

struct LineCenteringOutput
{
    double leftPower = 0.0;
    double rightPower = 0.0;
    bool completed = false;
    bool timedOut = false;
    std::string state = "WAITING_LINE";
};

// Centraliza NEAR e MEDIUM com a mesma decisão usada antes do retorno de 180°.
// Os consumidores escolhem apenas como reagir à conclusão ou ao timeout.
class LineCenteringController
{
public:
    void start(
        std::chrono::steady_clock::time_point now =
            std::chrono::steady_clock::now());
    LineCenteringOutput update(
        const CameraLineSnapshot& line,
        std::chrono::steady_clock::time_point now =
            std::chrono::steady_clock::now()) const;
    void reset();

private:
    bool started_ = false;
    std::chrono::steady_clock::time_point startedAt_{};
};
