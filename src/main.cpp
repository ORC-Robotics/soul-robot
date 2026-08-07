#include "obr/config.h"
#include "obr/dashboard_server.h"
#include "obr/esp32_bridge.h"
#include "obr/line_follower.h"
#include "obr/motor_controller.h"
#include "obr/robot_state.h"
#include "obr/status_led.h"
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
    StatusLed readyLed(config::kRaspberryReadyLedPin);
    DashboardServer dashboard(robotState, telemetry, esp32, readyLed);

    readyLed.begin();
    motors.begin();

    if (!dashboard.start())
    {
        motors.stop();
        return 1;
    }

    std::cout << "OBR robot dashboard running\n";
    std::cout << "Open http://raspberrypi.local:" << config::kDashboardPort << " in a browser\n";

    unsigned long long handledStartButtonPressSequence = 0;
    bool tractionFaultWasActive = false;

    while (running)
    {
        const Esp32TelemetrySnapshot esp32Telemetry = esp32.telemetrySnapshot();
        if (esp32Telemetry.tractionFaultActive && !tractionFaultWasActive)
        {
            // A ESP32 já zerou os quatro PWMs. A Raspberry também abandona o
            // modo atual para impedir que comandos antigos tentem religá-los.
            robotState.stop();
            std::cerr << "Traction fault stopped the robot; explicit rearm required\n";
        }
        tractionFaultWasActive = esp32Telemetry.tractionFaultActive;
        if (esp32Telemetry.calibrationActive)
        {
            // A calibração física também força a Raspberry para o modo parado.
            // Assim, um comando antigo não volta a mover o robô ao final do processo.
            robotState.stop();
        }

        const bool cameraReady = lineFollower.cameraReady();
        if (esp32Telemetry.startButtonPressSequence != handledStartButtonPressSequence)
        {
            handledStartButtonPressSequence = esp32Telemetry.startButtonPressSequence;
            const RobotSnapshot stateBeforeStart = robotState.snapshot();
            const bool startAllowed = stateBeforeStart.mode == "stopped" &&
                                      !stateBeforeStart.emergencyStop &&
                                      esp32Telemetry.readyForOperation() && cameraReady;
            if (startAllowed && esp32.sendClearEmergencyStop())
            {
                // O toque físico inicia a missão já selecionada no dashboard.
                // CLEAR_ESTOP também remove a trava deixada por uma calibração,
                // mas só é enviado depois de confirmar que nenhum E-Stop está ativo.
                robotState.startAutonomous();
                std::cout << "Physical Start button launched autonomous mission: "
                          << autonomousMissionName(stateBeforeStart.autonomousMission) << "\n";
            }
            else
            {
                std::cout << "Physical Start button ignored: system is not ready or robot is not stopped\n";
            }
        }

        lineFollower.update(robotState, esp32Telemetry);

        // Zera comandos antigos antes de enviá-los à ESP32.
        // Isso impede que uma queda do dashboard mantenha o último movimento ativo.
        robotState.enforceCommandTimeout(std::chrono::milliseconds(config::kCommandTimeoutMs));

        const RobotSnapshot robotSnapshot = robotState.snapshot();
        readyLed.setReady(esp32Telemetry.readyForOperation() &&
                          !robotSnapshot.emergencyStop && cameraReady);
        motors.apply(robotSnapshot);
        std::this_thread::sleep_for(std::chrono::milliseconds(config::kMainLoopPeriodMs));
    }

    dashboard.stop();
    readyLed.off();
    motors.stop();
    return 0;
}
