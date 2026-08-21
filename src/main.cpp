#include "obr/config.h"
#include "obr/camera_monitor.h"
#include "obr/dashboard_server.h"
#include "obr/esp32_bridge.h"
#include "obr/mission_controller.h"
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

bool driveDistanceEncodersReady(const Esp32TelemetrySnapshot& telemetry)
{
    // A missão de distância usa os dois encoders e não depende da câmera.
    return telemetry.sensorFresh && telemetry.lastSensorAgeMs >= 0 &&
           telemetry.lastSensorAgeMs <= config::kDriveDistanceEncoderFreshnessMs &&
           std::isfinite(telemetry.leftEncoderRate) &&
           std::isfinite(telemetry.rightEncoderRate);
}

bool selectedMissionReady(
    AutonomousMission mission,
    const Esp32TelemetrySnapshot& telemetry,
    bool cameraReady,
    const CameraLineSnapshot& cameraLineSnapshot)
{
    if (mission == AutonomousMission::TurnRight90)
    {
        return turn90ImuReady(telemetry);
    }
    if (mission == AutonomousMission::DriveDistance)
    {
        return driveDistanceEncodersReady(telemetry);
    }
    return cameraReady && cameraLineSnapshot.sourceFresh;
}
}

int main()
{
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    RobotState robotState;
    Telemetry telemetry;
    Esp32Bridge esp32;
    CameraMonitor cameraMonitor;
    MissionController missionController;
    MotorController motors(esp32);
    StatusLed readyLed(config::kRaspberryReadyLedPin);
    DashboardServer dashboard(robotState, telemetry, esp32, motors, readyLed);

    readyLed.begin();
    motors.begin();

    if (!dashboard.start())
    {
        motors.stop();
        return 1;
    }

    std::cout << "OBR robot dashboard running\n";
#ifdef _WIN32
    // A compilação para Windows é usada somente para visualizar o dashboard localmente.
    std::cout << "Open http://127.0.0.1:" << config::kDashboardPort << " in a browser\n";
#else
    std::cout << "Open http://raspberrypi.local:" << config::kDashboardPort << " in a browser\n";
#endif

    unsigned long long handledStartButtonPressSequence = 0;
    bool previousStartButtonPressed = false;
    bool consumeNextStartButtonShortPress = false;
    bool startupComplete = false;
    bool systemDisplayStatusSent = false;
    bool lastSystemDisplayReady = false;
    auto lastSystemDisplayStatusTime = std::chrono::steady_clock::now();
    auto lastCameraLineDiagnosticTime =
        std::chrono::steady_clock::now() - std::chrono::seconds(1);
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

        const bool cameraReady = cameraMonitor.ready();
        const CameraLineSnapshot cameraLineSnapshot = cameraMonitor.lineSnapshot();
        const auto cameraLineDiagnosticTime = std::chrono::steady_clock::now();
        if (cameraLineDiagnosticTime - lastCameraLineDiagnosticTime >=
            std::chrono::seconds(1))
        {
            // Este log apenas mostra a medição; as decisões de movimento são
            // aplicadas separadamente pela Missão Principal.
            std::cout << std::boolalpha
                      << "Camera line sourceFresh=" << cameraLineSnapshot.sourceFresh
                      << " lineSequence=" << cameraLineSnapshot.lineSequence
                      << " ageMs=" << cameraLineSnapshot.ageMs
                      << " greenConfirmed="
                      << cameraLineSnapshot.greenConfirmed
                      << " normalLeft="
                      << cameraLineSnapshot.lineFollowerLeftPower
                      << " normalRight="
                      << cameraLineSnapshot.lineFollowerRightPower
                      << std::noboolalpha << std::endl;
            lastCameraLineDiagnosticTime = cameraLineDiagnosticTime;
        }
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
                    const bool missionReady = selectedMissionReady(
                        stateBeforeStart.autonomousMission,
                        esp32Telemetry,
                        cameraReady,
                        cameraLineSnapshot);
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

        missionController.update(
            robotState,
            esp32Telemetry,
            cameraReady,
            cameraLineSnapshot);

        // Zera comandos antigos antes de enviá-los à ESP32.
        // Isso impede que uma queda do dashboard mantenha o último movimento ativo.
        robotState.enforceCommandTimeout(std::chrono::milliseconds(config::kCommandTimeoutMs));

        const RobotSnapshot robotSnapshot = robotState.snapshot();
        const bool systemReady = esp32Telemetry.readyForOperation() &&
                                 !robotSnapshot.emergencyStop && cameraReady;
        readyLed.setReady(systemReady);

        // A conclusão do boot fica travada até o processo reiniciar. Uma falha
        // posterior da câmera apaga o LED, mas não transforma operação em boot.
        startupComplete = startupComplete || systemReady;
        const auto now = std::chrono::steady_clock::now();
        const bool heartbeatDue =
            now - lastSystemDisplayStatusTime >=
            std::chrono::milliseconds(config::kRaspberrySystemStatusHeartbeatMs);
        if (!systemDisplayStatusSent ||
            startupComplete != lastSystemDisplayReady || heartbeatDue)
        {
            const bool statusSent = startupComplete
                                        ? esp32.sendSystemReady()
                                        : esp32.sendSystemStarting();
            if (statusSent)
            {
                systemDisplayStatusSent = true;
                lastSystemDisplayReady = startupComplete;
                lastSystemDisplayStatusTime = now;
            }
        }
        motors.apply(robotSnapshot);
        std::this_thread::sleep_for(std::chrono::milliseconds(config::kMainLoopPeriodMs));
    }

    // Se o serviço for reiniciado de forma limpa, a OLED informa imediatamente
    // que a Raspberry voltou ao processo de inicialização.
    esp32.sendSystemStarting();
    dashboard.stop();
    readyLed.off();
    motors.stop();
    return 0;
}
