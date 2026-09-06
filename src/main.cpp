#include "obr/robot_application.h"

#include <atomic>
#include <csignal>

namespace
{
std::atomic<bool> running(true);

void handleSignal(int)
{
    running = false;
}
}

int main()
{
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    RobotApplication application;
    return application.run(running);
}
