#include "obr/config.h"
#include "obr/dashboard_server.h"
#include "obr/esp32_bridge.h"
#include "obr/line_follower.h"
#include "obr/motor_controller.h"
#include "obr/robot_state.h"
#include "obr/telemetry.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

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

    RobotState robotState;
    Telemetry telemetry;
    Esp32Bridge esp32;
    LineFollower lineFollower;
    MotorController motors(esp32);
    DashboardServer dashboard(robotState, telemetry, esp32);

    motors.begin();

    if (!dashboard.start())
    {
        motors.stop();
        return 1;
    }

    std::cout << "OBR robot dashboard running\n";
    std::cout << "Open http://raspberrypi.local:" << config::kDashboardPort << " in a browser\n";

    while (running)
    {
        lineFollower.update(robotState);
        motors.apply(robotState.snapshot());
        std::this_thread::sleep_for(std::chrono::milliseconds(config::kMainLoopPeriodMs));
    }

    dashboard.stop();
    motors.stop();
    return 0;
}
