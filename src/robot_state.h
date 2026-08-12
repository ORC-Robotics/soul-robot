#pragma once

#include <chrono>
#include <mutex>
#include <string>

struct RobotSnapshot
{
    std::string mode = "stopped";
    bool emergencyStop = false;
    double left = 0.0;
    double right = 0.0;
};

class RobotState
{
public:
    RobotSnapshot snapshot() const;

    void start();
    void stop();
    void emergencyStop();
    void drive(double left, double right);
    void enforceCommandTimeout(std::chrono::milliseconds timeout);

private:
    mutable std::mutex mutex_;
    RobotSnapshot state_;
    std::chrono::steady_clock::time_point lastCommand_ = std::chrono::steady_clock::now();
};
