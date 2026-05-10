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
