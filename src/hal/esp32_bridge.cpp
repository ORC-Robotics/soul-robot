#include "obr/esp32_bridge.h"

#include "obr/config.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

#ifndef _WIN32
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace
{
bool startsWith(const std::string& text, const std::string& prefix)
{
    return text.rfind(prefix, 0) == 0;
}

std::vector<std::string> splitCsv(const std::string& text)
{
    std::vector<std::string> values;
    std::stringstream stream(text);
    std::string value;

    while (std::getline(stream, value, ','))
    {
        values.push_back(value);
    }

    return values;
}

#ifndef _WIN32
speed_t baudRateConstant(int baudRate)
{
    if (baudRate == 9600)
    {
        return B9600;
    }

    if (baudRate == 57600)
    {
        return B57600;
    }

    return B115200;
}
#endif
}

Esp32Bridge::Esp32Bridge() = default;

Esp32Bridge::~Esp32Bridge()
{
    stop();
}

bool Esp32Bridge::begin()
{
    if (running_)
    {
        return serialOpen_;
    }

    if (!openSerialPort())
    {
        return false;
    }

    running_ = true;
    readThread_ = std::thread(&Esp32Bridge::readLoop, this);
    sendStop();
    return true;
}

void Esp32Bridge::stop()
{
    if (!running_ && !serialOpen_)
    {
        return;
    }

    sendStop();
    running_ = false;

    if (readThread_.joinable())
    {
        readThread_.join();
    }

    closeSerialPort();
}

bool Esp32Bridge::sendMotorCommand(double left, double right, bool emergencyStop)
{
    double safeLeft = emergencyStop ? 0.0 : safeMotorPower(left);
    double safeRight = emergencyStop ? 0.0 : safeMotorPower(right);

    if (std::abs(safeLeft) < config::kMotorDeadband)
    {
        safeLeft = 0.0;
    }
    if (std::abs(safeRight) < config::kMotorDeadband)
    {
        safeRight = 0.0;
    }

    // Nunca envia somente um lado. A ESP32 repete esta validação, mas a
    // Raspberry já transmite um giro válido para manter o protocolo previsível.
    if ((safeLeft == 0.0) != (safeRight == 0.0))
    {
        if (safeLeft == 0.0)
        {
            safeLeft = -safeRight;
        }
        else
        {
            safeRight = -safeLeft;
        }
    }

    std::ostringstream command;
    command << std::fixed << std::setprecision(3)
            << "MOTOR," << safeLeft << "," << safeRight << "," << (emergencyStop ? 1 : 0) << "\n";

    return writeLine(command.str());
}

bool Esp32Bridge::sendStop()
{
    return writeLine("STOP\n");
}

bool Esp32Bridge::sendEmergencyStop()
{
    return writeLine("ESTOP\n");
}

bool Esp32Bridge::sendClearEmergencyStop()
{
    // A ESP32 mantém o E-Stop travado. Somente uma ação explícita de partida
    // pode liberar a trava, e os motores continuam zerados durante a liberação.
    return writeLine("CLEAR_ESTOP\n");
}

bool Esp32Bridge::sendResetEncoders()
{
    return writeLine("RESET_ENCODERS\n");
}

bool Esp32Bridge::sendCalibrateSensors()
{
    // A ESP32 para os motores antes de apagar referências e recalibrar o IMU.
    return writeLine("CALIBRATE_SENSORS\n");
}

Esp32TelemetrySnapshot Esp32Bridge::telemetrySnapshot() const
{
    std::lock_guard<std::mutex> lock(telemetryMutex_);
    Esp32TelemetrySnapshot snapshot = telemetry_;
    snapshot.serialOpen = serialOpen_;

    if (hasSensorSample_)
    {
        const auto age = std::chrono::steady_clock::now() - lastSensorTime_;
        snapshot.lastSensorAgeMs = std::chrono::duration_cast<std::chrono::milliseconds>(age).count();
        snapshot.sensorFresh = snapshot.serialOpen &&
                               snapshot.lastSensorAgeMs <= config::kEsp32TelemetryTimeoutMs;
    }
    else
    {
        snapshot.lastSensorAgeMs = -1;
        snapshot.sensorFresh = false;
    }

    return snapshot;
}

bool Esp32Bridge::openSerialPort()
{
#ifdef _WIN32
    std::cerr << "ESP32 UART is not opened on Windows. Deploy to the Raspberry Pi to use /dev/serial0.\n";
    serialOpen_ = false;
    return false;
#else
    serialFd_ = open(config::kEsp32SerialPort, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serialFd_ < 0)
    {
        std::cerr << "ESP32 UART open failed on " << config::kEsp32SerialPort << ": "
                  << std::strerror(errno) << "\n";
        serialOpen_ = false;
        return false;
    }

    termios options = {};
    if (tcgetattr(serialFd_, &options) != 0)
    {
        std::cerr << "ESP32 UART tcgetattr failed: " << std::strerror(errno) << "\n";
        closeSerialPort();
        return false;
    }

    cfmakeraw(&options);
    const speed_t baudRate = baudRateConstant(config::kEsp32SerialBaudRate);
    cfsetispeed(&options, baudRate);
    cfsetospeed(&options, baudRate);

    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (tcsetattr(serialFd_, TCSANOW, &options) != 0)
    {
        std::cerr << "ESP32 UART tcsetattr failed: " << std::strerror(errno) << "\n";
        closeSerialPort();
        return false;
    }

    tcflush(serialFd_, TCIOFLUSH);
    serialOpen_ = true;
    std::cout << "ESP32 UART opened on " << config::kEsp32SerialPort
              << " at " << config::kEsp32SerialBaudRate << " bps\n";
    return true;
#endif
}

void Esp32Bridge::closeSerialPort()
{
#ifdef _WIN32
    serialOpen_ = false;
#else
    if (serialFd_ >= 0)
    {
        close(serialFd_);
        serialFd_ = -1;
    }

    serialOpen_ = false;
#endif
}

bool Esp32Bridge::writeLine(const std::string& line)
{
    if (!serialOpen_)
    {
        return false;
    }

#ifdef _WIN32
    (void)line;
    return false;
#else
    std::lock_guard<std::mutex> lock(writeMutex_);
    size_t sent = 0;

    while (sent < line.size())
    {
        const ssize_t result = write(serialFd_, line.data() + sent, line.size() - sent);
        if (result > 0)
        {
            sent += static_cast<size_t>(result);
            continue;
        }

        if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        std::cerr << "ESP32 UART write failed: " << std::strerror(errno) << "\n";
        serialOpen_ = false;
        return false;
    }

    return true;
#endif
}

void Esp32Bridge::readLoop()
{
    std::string buffer;

    while (running_)
    {
#ifdef _WIN32
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
#else
        char data[128] = {};
        const ssize_t bytesRead = read(serialFd_, data, sizeof(data));

        if (bytesRead > 0)
        {
            buffer.append(data, static_cast<size_t>(bytesRead));

            size_t newline = buffer.find('\n');
            while (newline != std::string::npos)
            {
                std::string line = buffer.substr(0, newline);
                if (!line.empty() && line.back() == '\r')
                {
                    line.pop_back();
                }

                handleLine(line);
                buffer.erase(0, newline + 1);
                newline = buffer.find('\n');
            }
        }
        else if (bytesRead < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        {
            std::cerr << "ESP32 UART read failed: " << std::strerror(errno) << "\n";
            serialOpen_ = false;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
#endif
    }
}

void Esp32Bridge::handleLine(const std::string& line)
{
    if (line.empty())
    {
        return;
    }

    if (line == "READY")
    {
        std::cout << "ESP32 ready\n";
        return;
    }

    if (startsWith(line, "ERR,"))
    {
        std::cerr << "ESP32 reported error: " << line.substr(4) << "\n";
        return;
    }

    if (line == "CALIBRATION,START")
    {
        std::lock_guard<std::mutex> lock(telemetryMutex_);
        telemetry_.calibrationActive = true;
        telemetry_.calibrationStatusKnown = true;
        telemetry_.lastCalibrationSucceeded = false;
        std::cout << "ESP32 sensor calibration started\n";
        return;
    }

    if (line == "START_BUTTON,SHORT")
    {
        std::lock_guard<std::mutex> lock(telemetryMutex_);
        ++telemetry_.startButtonPressSequence;
        std::cout << "ESP32 short Start-button press received\n";
        return;
    }

    if (startsWith(line, "TRACTION_RECOVERY,") || startsWith(line, "TRACTION_FAULT,"))
    {
        std::lock_guard<std::mutex> lock(telemetryMutex_);
        telemetry_.tractionRecoveryActive = true;
        telemetry_.tractionRecoverySide = line.find("LEFT") != std::string::npos ? 1 : 2;
        std::cout << "ESP32 traction recovery: "
                  << (telemetry_.tractionRecoverySide == 1 ? "boosting left side" : "boosting right side")
                  << "\n";
        return;
    }

    if (line == "CALIBRATION,DONE" || line == "CALIBRATION,FAILED")
    {
        std::lock_guard<std::mutex> lock(telemetryMutex_);
        telemetry_.calibrationActive = false;
        telemetry_.calibrationStatusKnown = true;
        telemetry_.lastCalibrationSucceeded = line == "CALIBRATION,DONE";
        std::cout << (telemetry_.lastCalibrationSucceeded
                          ? "ESP32 sensor calibration completed\n"
                          : "ESP32 sensor calibration failed\n");
        return;
    }

    if (startsWith(line, "SENSOR,"))
    {
        if (!parseSensorLine(line))
        {
            std::cerr << "ESP32 sensor line ignored: " << line << "\n";
        }
        return;
    }
}

bool Esp32Bridge::parseSensorLine(const std::string& line)
{
    const std::vector<std::string> values = splitCsv(line);
    if (values.size() < 8)
    {
        return false;
    }

    try
    {
        Esp32TelemetrySnapshot next;
        next.serialOpen = serialOpen_;
        next.sensorFresh = true;
        next.ultrasonicDistanceCm = std::stod(values[1]);
        next.gyroZDegPerSec = std::stod(values[2]);
        next.yawZDeg = std::stod(values[3]);
        next.accelX = std::stod(values[4]);
        next.accelY = std::stod(values[5]);
        next.accelZ = std::stod(values[6]);
        next.mpuOk = std::stoi(values[7]) != 0;

        // Os campos após o MPU6050 são opcionais para manter compatibilidade
        // com firmwares antigos. O firmware principal atual envia todos eles.
        if (values.size() >= 17)
        {
            next.batteryVoltage = std::stod(values[8]);
            next.leftEncoderCount = std::stoll(values[9]);
            next.rightEncoderCount = std::stoll(values[10]);
            next.startButtonPressed = std::stoi(values[11]) != 0;
            next.pca9685Ok = std::stoi(values[12]) != 0;
            next.appliedLeftPower = std::stod(values[13]);
            next.appliedRightPower = std::stod(values[14]);
            next.leftEncoderRate = std::stod(values[15]);
            next.rightEncoderRate = std::stod(values[16]);
        }

        if (values.size() >= 26)
        {
            next.rampAngleDeg = std::stod(values[17]);
            next.gyroXDegPerSec = std::stod(values[18]);
            next.gyroYDegPerSec = std::stod(values[19]);
            next.imuTemperatureCelsius = std::stod(values[20]);
            next.oledOk = std::stoi(values[21]) != 0;
            next.motorSleepPinHigh = std::stoi(values[22]) != 0;
            next.emergencyStopActive = std::stoi(values[23]) != 0;
            next.batteryAdcMillivolts = std::stoll(values[24]);
            next.esp32UptimeMs = std::stoll(values[25]);
        }

        if (values.size() >= 27)
        {
            next.calibrationActive = std::stoi(values[26]) != 0;
        }

        if (values.size() >= 29)
        {
            next.tractionRecoveryActive = std::stoi(values[27]) != 0;
            next.tractionRecoverySide = std::stoi(values[28]);
        }

        std::lock_guard<std::mutex> lock(telemetryMutex_);
        next.startButtonPressSequence = telemetry_.startButtonPressSequence;
        next.calibrationStatusKnown = telemetry_.calibrationStatusKnown;
        next.lastCalibrationSucceeded = telemetry_.lastCalibrationSucceeded;
        if (values.size() < 29)
        {
            next.tractionRecoveryActive = telemetry_.tractionRecoveryActive;
            next.tractionRecoverySide = telemetry_.tractionRecoverySide;
        }
        if (values.size() < 27)
        {
            next.calibrationActive = telemetry_.calibrationActive;
        }
        telemetry_ = next;
        hasSensorSample_ = true;
        lastSensorTime_ = std::chrono::steady_clock::now();
        return true;
    }
    catch (const std::exception&)
    {
        return false;
    }
}

double Esp32Bridge::safeMotorPower(double command)
{
    if (!std::isfinite(command))
    {
        return 0.0;
    }

    return std::clamp(command, config::kMinMotorOutput, config::kMaxMotorOutput);
}
