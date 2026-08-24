#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

// Cópia das leituras enviadas pela ESP32 pela UART.
// O dashboard usa esta estrutura para mostrar se os sensores estão recentes.
struct Esp32TelemetrySnapshot
{
    bool serialOpen = false;
    bool sensorFresh = false;
    bool mpuOk = false;
    long long lastSensorAgeMs = -1;
    double ultrasonicDistanceCm = -1.0;
    double gyroZDegPerSec = 0.0;
    double yawZDeg = 0.0;
    double accelX = 0.0;
    double accelY = 0.0;
    double accelZ = 0.0;
    double gyroXDegPerSec = 0.0;
    double gyroYDegPerSec = 0.0;
    double rampAngleDeg = 0.0;
    double imuTemperatureCelsius = 0.0;
    double batteryVoltage = 0.0;
    long long batteryAdcMillivolts = 0;
    long long leftEncoderCount = 0;
    long long rightEncoderCount = 0;
    double leftEncoderRate = 0.0;
    double rightEncoderRate = 0.0;
    double appliedLeftPower = 0.0;
    double appliedRightPower = 0.0;
    bool startButtonPressed = false;
    unsigned long long startButtonPressSequence = 0;
    bool pca9685Ok = false;
    bool oledOk = false;
    bool remoteOledActive = false;
    bool raspberrySystemReady = false;
    bool motorSleepPinHigh = false;
    bool emergencyStopActive = false;
    bool calibrationActive = false;
    long long motorCommandAgeMs = -1;
    bool motorWatchdogTimedOut = false;
    std::string motorControlSource = "unknown";
    bool calibrationStatusKnown = false;
    bool lastCalibrationSucceeded = false;
    long long esp32UptimeMs = 0;

    // Indica se a ESP32 está comunicando e se o driver pode operar com segurança.
    // Falta de telemetria, nSLEEP baixo ou E-Stop local impedem o estado pronto.
    bool readyForOperation() const
    {
        return serialOpen && sensorFresh && motorSleepPinHigh &&
               !emergencyStopActive && !calibrationActive;
    }
};

// Faz a comunicação UART entre a Raspberry Pi e a ESP32.
// A Raspberry envia comandos de motor já validados, e a ESP32 devolve sensores.
class Esp32Bridge
{
public:
    Esp32Bridge();
    ~Esp32Bridge();

    bool begin();
    void stop();

    bool sendMotorCommand(double left, double right, bool emergencyStop);
    bool sendStop();
    bool sendEmergencyStop();
    bool sendClearEmergencyStop();
    bool sendResetEncoders();
    bool sendCalibrateSensors();
    bool sendSystemStarting();
    bool sendSystemReady();
    bool sendOledMessage(const std::string& title, const std::string& firstLine,
                         const std::string& secondLine, int durationMs);
    bool clearOledMessage();

    Esp32TelemetrySnapshot telemetrySnapshot() const;

private:
    mutable std::mutex telemetryMutex_;
    std::mutex writeMutex_;
    std::atomic<bool> running_{false};
    std::atomic<bool> serialOpen_{false};
    std::thread readThread_;
    Esp32TelemetrySnapshot telemetry_;
    bool hasSensorSample_ = false;
    std::chrono::steady_clock::time_point lastSensorTime_{};
    int serialFd_ = -1;

    bool openSerialPort();
    void closeSerialPort();
    bool writeLine(const std::string& line);
    void readLoop();
    void handleLine(const std::string& line);
    bool parseSensorLine(const std::string& line);

    static double safeMotorPower(double command);
};
