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
#include <cmath>
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

bool turn90ImuReady(const Esp32TelemetrySnapshot& telemetry)
{
    // O giro de 90 graus não depende da câmera, mas só pode iniciar com yaw
    // e velocidade angular recentes e válidos.
    return telemetry.sensorFresh && telemetry.mpuOk &&
           telemetry.lastSensorAgeMs >= 0 &&
           telemetry.lastSensorAgeMs <= config::kTurn90ImuFreshnessMs &&
           std::isfinite(telemetry.yawZDeg) &&
           std::isfinite(telemetry.gyroZDegPerSec);
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
    bool previousStartButtonPressed = false;
    bool consumeNextStartButtonShortPress = false;
    while (running)
    {
        const Esp32TelemetrySnapshot esp32Telemetry = esp32.telemetrySnapshot();
        if (esp32Telemetry.calibrationActive)
        {
            // A calibração física também força a Raspberry para o modo parado.
            // Assim, um comando antigo não volta a mover o robô ao final do processo.
            robotState.stop();
            consumeNextStartButtonShortPress = false;
        }

        const bool cameraReady = lineFollower.cameraReady();
        const bool startButtonPressedEdge =
            esp32Telemetry.startButtonPressed && !previousStartButtonPressed;
        const bool startButtonReleasedEdge =
            !esp32Telemetry.startButtonPressed && previousStartButtonPressed;
        previousStartButtonPressed = esp32Telemetry.startButtonPressed;

        if (startButtonPressedEdge && !esp32Telemetry.calibrationActive)
        {
            const RobotSnapshot stateAtButtonPress = robotState.snapshot();
            if ((stateAtButtonPress.mode == "manual" ||
                 stateAtButtonPress.mode == "autonomous") &&
                !stateAtButtonPress.emergencyStop)
            {
                // A borda de pressão para o robô sem esperar o botão ser solto.
                // O evento SHORT posterior é consumido para não reiniciar a missão.
                robotState.stop();
                consumeNextStartButtonShortPress = true;
                std::cout << "Physical Start button press stopped the robot immediately\n";
            }
        }

        if (esp32Telemetry.startButtonPressSequence != handledStartButtonPressSequence)
        {
            handledStartButtonPressSequence = esp32Telemetry.startButtonPressSequence;
            if (consumeNextStartButtonShortPress)
            {
                consumeNextStartButtonShortPress = false;
                std::cout << "Physical Start short event consumed after stopping the robot\n";
            }
            else
            {
                const RobotSnapshot stateBeforeStart = robotState.snapshot();
                if ((stateBeforeStart.mode == "manual" ||
                     stateBeforeStart.mode == "autonomous") &&
                    !stateBeforeStart.emergencyStop)
                {
                    // Um toque curto ainda funciona como Stop se a borda de
                    // pressão não apareceu entre duas amostras da telemetria.
                    robotState.stop();
                    std::cout << "Physical Start button stopped the robot\n";
                }
                else
                {
                    const bool missionReady =
                        stateBeforeStart.autonomousMission == AutonomousMission::TurnRight90
                            ? turn90ImuReady(esp32Telemetry)
                            : cameraReady;
                    const bool startAllowed = stateBeforeStart.mode == "stopped" &&
                                              !stateBeforeStart.emergencyStop &&
                                              esp32Telemetry.readyForOperation() && missionReady;
                    const bool clearSent = startAllowed && esp32.sendClearEmergencyStop();
                    const bool started = clearSent && robotState.tryStartAutonomous();
                    if (started)
                    {
                        // O toque físico inicia a missão já selecionada no dashboard.
                        // CLEAR_ESTOP também remove a trava deixada por uma calibração,
                        // enquanto a transição atômica impede liberar um E-Stop concorrente.
                        std::cout << "Physical Start button launched autonomous mission: "
                                  << autonomousMissionName(stateBeforeStart.autonomousMission) << "\n";
                    }
                    else
                    {
                        if (robotState.snapshot().emergencyStop)
                        {
                            // Se o E-Stop chegou durante a tentativa, restaura também
                            // a trava local da ESP32 depois de qualquer CLEAR já enviado.
                            esp32.sendEmergencyStop();
                        }
                        std::cout << "Physical Start button ignored: selected mission is not ready or E-Stop is active\n";
                    }
                }
            }
        }

        if (startButtonReleasedEdge && consumeNextStartButtonShortPress)
        {
            // Pressões menores que 80 ms e pressões longas não geram SHORT.
            // Se nenhum evento foi consumido até a telemetria da soltura, libera
            // a próxima pressão para ela não herdar este estado.
            consumeNextStartButtonShortPress = false;
            std::cout << "Physical Start release produced no short event to consume\n";
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
