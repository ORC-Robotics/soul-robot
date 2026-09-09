#include "obr/robot_application.h"

#include "obr/config.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

namespace
{
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
    if (mission == AutonomousMission::ServoInitialize ||
        mission == AutonomousMission::ServoCapture ||
        mission == AutonomousMission::ServoInternalStorage ||
        mission == AutonomousMission::ServoDeposit)
    {
        // As rotinas não usam câmera nem sensores de navegação, mas dependem
        // da ponte e do PCA9685 prontos para aplicar cada passo com segurança.
        return telemetry.readyForOperation() && telemetry.pca9685Ok;
    }
    if (mission == AutonomousMission::TurnRight90)
    {
        return turn90ImuReady(telemetry);
    }
    if (mission == AutonomousMission::DriveDistance)
    {
        return driveDistanceEncodersReady(telemetry);
    }
    if (mission == AutonomousMission::RescueDetection ||
        mission == AutonomousMission::RescueArea)
    {
        // A visão pesada é ligada somente depois da partida. A missão começa
        // parada e aguarda um IPC frontal recente antes de mover os motores.
        return true;
    }
    if (mission == AutonomousMission::ObstacleAvoidance)
    {
        // O teste isolado usa ultrassom, IMU e encoders da ESP32.
        return telemetry.readyForOperation();
    }
    return cameraReady && cameraLineSnapshot.sourceFresh;
}

const char* motorDirectionName(double power)
{
    if (power > 0.0)
    {
        return "forward";
    }
    if (power < 0.0)
    {
        return "reverse";
    }
    return "stopped";
}

const char* autonomousCommandSourceName(AutonomousMission mission)
{
    if (mission == AutonomousMission::ServoInitialize ||
        mission == AutonomousMission::ServoCapture ||
        mission == AutonomousMission::ServoInternalStorage ||
        mission == AutonomousMission::ServoDeposit)
    {
        return "servos";
    }
    if (mission == AutonomousMission::TurnRight90)
    {
        return "imu";
    }
    if (mission == AutonomousMission::DriveDistance)
    {
        return "encoders";
    }
    if (mission == AutonomousMission::RescueDetection)
    {
        return "forward_ball_detection";
    }
    if (mission == AutonomousMission::RescueArea)
    {
        return "forward_ball_tx";
    }
    if (mission == AutonomousMission::ObstacleAvoidance)
    {
        return "ultrasonic_imu_encoders";
    }
    return "camera";
}
}

RobotApplication::RobotApplication()
    : servos_(esp32_),
      forwardBallVision_(cameraMonitor_),
      motors_(esp32_),
      oledEvents_(esp32_),
      curveDiagnosticsLogger_(config::kCurveDiagnosticsPath),
      readyLed_(config::kRaspberryReadyLedPin),
      dashboard_(
          robotState_,
          telemetry_,
          esp32_,
          motors_,
          servos_,
          readyLed_)
{
}

int RobotApplication::run(const std::atomic<bool>& running)
{
    RobotState& robotState = robotState_;
    Telemetry& telemetry = telemetry_;
    Esp32Bridge& esp32 = esp32_;
    ServoController& servos = servos_;
    CameraMonitor& cameraMonitor = cameraMonitor_;
    ForwardBallVisionLifecycle& forwardBallVision = forwardBallVision_;
    MissionController& missionController = missionController_;
    MotorController& motors = motors_;
    OledEventNotifier& oledEvents = oledEvents_;
    CurveDiagnosticsLogger& curveDiagnosticsLogger =
        curveDiagnosticsLogger_;
    StatusLed& readyLed = readyLed_;
    DashboardServer& dashboard = dashboard_;

    readyLed.begin();
    motors.begin();
    forwardBallVision.begin();

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
    bool autonomousMotorTraceInitialized = false;
    std::uint64_t lastAutonomousMotorTraceLineSequence = 0;
    long long lastAutonomousMotorTraceEsp32UptimeMs = -1;
    double lastAutonomousMotorTraceLeft = 0.0;
    double lastAutonomousMotorTraceRight = 0.0;
    AutonomousMission lastAutonomousMotorTraceMission =
        AutonomousMission::MainMission;
    std::string lastAutonomousMotorTracePhase;
    auto lastAutonomousMotorTraceTime =
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

        const RobotSnapshot stateAtLoopStart = robotState.snapshot();
        const bool rescueAreaActive =
            missionController.requiresForwardBallDetection(stateAtLoopStart);
        forwardBallVision.update(
            rescueAreaActive,
            stateAtLoopStart.autonomousRunSequence);

        const bool cameraReady = cameraMonitor.ready();
        const CameraLineSnapshot cameraLineSnapshot = cameraMonitor.lineSnapshot();
        const ForwardLineSnapshot forwardLineSnapshot =
            cameraMonitor.forwardLineSnapshot();
        const ForwardBallSnapshot forwardBallSnapshot =
            forwardBallVision.snapshot();
        const bool oledEventDisplayAvailable = esp32Telemetry.sensorFresh &&
                                               esp32Telemetry.oledOk &&
                                               esp32Telemetry.raspberrySystemReady;
        oledEvents.updateLineEvents(
            cameraLineSnapshot, oledEventDisplayAvailable);
        const auto cameraLineDiagnosticTime = std::chrono::steady_clock::now();
        if (cameraLineDiagnosticTime - lastCameraLineDiagnosticTime >=
            std::chrono::seconds(1))
        {
            // Este log apenas mostra a percepção. O alerta da OLED e qualquer
            // manobra continuam em módulos próprios, fora do caminho de log.
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
            if (stateAtButtonPress.servoCalibrationActive ||
                esp32Telemetry.servoCalibrationActive)
            {
                // O botão físico também funciona como parada durante o ajuste:
                // o pulso é removido antes de qualquer nova ação de partida.
                esp32.sendServoCalibrationEnd();
                robotState.stop();
                consumeNextStartButtonShortPress = true;
                std::cout << "Physical Start button stopped servo calibration\n";
            }
            else if ((stateAtButtonPress.mode == "manual" ||
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
            cameraLineSnapshot,
            forwardLineSnapshot,
            forwardBallSnapshot);

        // Zera comandos antigos antes de enviá-los à ESP32.
        // Isso impede que uma queda do dashboard mantenha o último movimento ativo.
        robotState.enforceCommandTimeout(std::chrono::milliseconds(config::kCommandTimeoutMs));
        robotState.enforceManualServoTimeout(
            std::chrono::milliseconds(config::kManualServoCommandTimeoutMs));

        const RobotSnapshot robotSnapshot = robotState.snapshot();
        const bool obstacleDetourActive =
            robotSnapshot.mode == "autonomous" &&
            robotSnapshot.autonomousStatus.phase.rfind("obstacle_", 0) == 0;
        oledEvents.updateObstacleDetour(
            obstacleDetourActive,
            oledEventDisplayAvailable);
        const bool rescueVisionRequired =
            missionController.requiresForwardBallDetection(robotSnapshot);
        const bool selectedPerceptionReady =
            rescueVisionRequired
                ? forwardBallSnapshot.sourceFresh
                : selectedMissionReady(
                      robotSnapshot.autonomousMission,
                      esp32Telemetry,
                      cameraReady,
                      cameraLineSnapshot);
        const bool systemReady = esp32Telemetry.readyForOperation() &&
                                 !robotSnapshot.emergencyStop &&
                                 selectedPerceptionReady;
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
        servos.apply(robotSnapshot);

        const MotorSynchronizationSnapshot finalMotorCommand =
            motors.synchronizationSnapshot();
        curveDiagnosticsLogger.record(
            cameraLineSnapshot,
            robotSnapshot,
            finalMotorCommand);
        const Esp32TelemetrySnapshot finalEsp32Telemetry =
            esp32.telemetrySnapshot();
        const auto motorTraceTime = std::chrono::steady_clock::now();
        const bool autonomousTraceActive =
            robotSnapshot.mode == "autonomous";
        if (autonomousTraceActive || autonomousMotorTraceInitialized)
        {
            const double finalLeft = finalMotorCommand.correctedLeftPower;
            const double finalRight = finalMotorCommand.correctedRightPower;
            const bool traceChanged =
                !autonomousMotorTraceInitialized ||
                cameraLineSnapshot.lineSequence !=
                    lastAutonomousMotorTraceLineSequence ||
                finalEsp32Telemetry.esp32UptimeMs !=
                    lastAutonomousMotorTraceEsp32UptimeMs ||
                std::abs(finalLeft - lastAutonomousMotorTraceLeft) > 0.0005 ||
                std::abs(finalRight - lastAutonomousMotorTraceRight) > 0.0005 ||
                robotSnapshot.autonomousMission !=
                    lastAutonomousMotorTraceMission ||
                robotSnapshot.autonomousStatus.phase !=
                    lastAutonomousMotorTracePhase ||
                !autonomousTraceActive;
            const bool traceHeartbeatDue =
                motorTraceTime - lastAutonomousMotorTraceTime >=
                std::chrono::milliseconds(500);
            if (traceChanged || traceHeartbeatDue)
            {
                // O trace registra apenas dados já decididos. Os motores de um
                // mesmo lado compartilham o PWM, por isso os campos dianteiro e
                // traseiro repetem o comando final enviado pela Raspberry.
                const auto timestampUs =
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
                const char* priority = "autonomous";
                if (robotSnapshot.emergencyStop ||
                    finalEsp32Telemetry.emergencyStopActive)
                {
                    priority = "emergency_stop";
                }
                else if (finalEsp32Telemetry.calibrationActive)
                {
                    priority = "calibration";
                }
                else if (!autonomousTraceActive)
                {
                    priority = "mode_exit";
                }
                else if (finalEsp32Telemetry.motorWatchdogTimedOut)
                {
                    priority = "esp32_watchdog";
                }
                std::ostringstream trace;
                trace << std::fixed << std::setprecision(3)
                      << "AUTONOMOUS_MOTOR_TRACE"
                      << " timestampUs=" << timestampUs
                      << " mode=" << robotSnapshot.mode
                      << " mission="
                      << autonomousMissionName(robotSnapshot.autonomousMission)
                      << " phase=" << robotSnapshot.autonomousStatus.phase
                      << " commandSource="
                      << autonomousCommandSourceName(
                             robotSnapshot.autonomousMission)
                      << " visionSource="
                      << cameraLineSnapshot.lineControlSource
                      << " visionSequence="
                      << cameraLineSnapshot.lineSequence
                      << " visionFresh=" << cameraLineSnapshot.sourceFresh
                      << " visionLeft="
                      << cameraLineSnapshot.lineFollowerLeftPower
                      << " visionRight="
                      << cameraLineSnapshot.lineFollowerRightPower
                      << " requestedLeft=" << robotSnapshot.left
                      << " requestedRight=" << robotSnapshot.right
                      << " commandAgeMs=" << robotSnapshot.commandAgeMs
                      << " commandTimedOut="
                      << robotSnapshot.commandTimedOut
                      << " frontLeft=" << finalLeft
                      << " rearLeft=" << finalLeft
                      << " frontRight=" << finalRight
                      << " rearRight=" << finalRight
                      << " leftDirection=" << motorDirectionName(finalLeft)
                      << " rightDirection=" << motorDirectionName(finalRight)
                      << " leftPwm=" << std::abs(finalLeft)
                      << " rightPwm=" << std::abs(finalRight)
                      << " raw=" << robotSnapshot.rawMotorCommand
                      << " syncEligible=" << finalMotorCommand.eligible
                      << " syncActive=" << finalMotorCommand.active
                      << " uartOpen=" << finalEsp32Telemetry.serialOpen
                      << " esp32TelemetryFresh="
                      << finalEsp32Telemetry.sensorFresh
                      << " driverEnabled="
                      << finalEsp32Telemetry.motorSleepPinHigh
                      << " esp32AppliedLeft="
                      << finalEsp32Telemetry.appliedLeftPower
                      << " esp32AppliedRight="
                      << finalEsp32Telemetry.appliedRightPower
                      << " esp32LeftPwm="
                      << std::abs(finalEsp32Telemetry.appliedLeftPower)
                      << " esp32RightPwm="
                      << std::abs(finalEsp32Telemetry.appliedRightPower)
                      << " esp32Estop="
                      << finalEsp32Telemetry.emergencyStopActive
                      << " calibration="
                      << finalEsp32Telemetry.calibrationActive
                      << " watchdogAgeMs="
                      << finalEsp32Telemetry.motorCommandAgeMs
                      << " watchdogTimedOut="
                      << finalEsp32Telemetry.motorWatchdogTimedOut
                      << " uartSource="
                      << finalEsp32Telemetry.motorControlSource
                      << " priority=" << priority << '\n';
                std::cout << trace.str();

                autonomousMotorTraceInitialized = autonomousTraceActive;
                lastAutonomousMotorTraceLineSequence =
                    cameraLineSnapshot.lineSequence;
                lastAutonomousMotorTraceEsp32UptimeMs =
                    finalEsp32Telemetry.esp32UptimeMs;
                lastAutonomousMotorTraceLeft = finalLeft;
                lastAutonomousMotorTraceRight = finalRight;
                lastAutonomousMotorTraceMission =
                    robotSnapshot.autonomousMission;
                lastAutonomousMotorTracePhase =
                    robotSnapshot.autonomousStatus.phase;
                lastAutonomousMotorTraceTime = motorTraceTime;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(config::kMainLoopPeriodMs));
    }

    // Se o serviço for reiniciado de forma limpa, a OLED informa imediatamente
    // que a Raspberry voltou ao processo de inicialização.
    servos.disableAll();
    forwardBallVision.stop();
    esp32.sendSystemStarting();
    dashboard.stop();
    readyLed.off();
    motors.stop();
    return 0;
}
