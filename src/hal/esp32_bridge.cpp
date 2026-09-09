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

std::string sanitizeOledText(const std::string& text, size_t maximumLength)
{
    std::string sanitized;
    sanitized.reserve(std::min(text.size(), maximumLength));
    for (const unsigned char character : text)
    {
        if (character >= 32 && character <= 126)
        {
            sanitized.push_back(static_cast<char>(character));
            if (sanitized.size() == maximumLength)
            {
                break;
            }
        }
    }
    return sanitized;
}

std::string hexEncodeOledField(const std::string& text)
{
    if (text.empty())
    {
        return "-";
    }

    constexpr char kHexDigits[] = "0123456789ABCDEF";
    std::string encoded;
    encoded.reserve(text.size() * 2);
    for (const unsigned char character : text)
    {
        encoded.push_back(kHexDigits[character >> 4]);
        encoded.push_back(kHexDigits[character & 0x0f]);
    }
    return encoded;
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
    sendSystemStarting();
    return true;
}

void Esp32Bridge::stop()
{
    if (!running_ && !serialOpen_)
    {
        return;
    }

    sendSystemStarting();
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
    const double safeLeft = emergencyStop ? 0.0 : safeMotorPower(left);
    const double safeRight = emergencyStop ? 0.0 : safeMotorPower(right);

    std::ostringstream command;
    command << std::fixed << std::setprecision(3)
            << "MOTOR," << safeLeft << "," << safeRight << ","
            << (emergencyStop ? 1 : 0) << "\n";

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

bool Esp32Bridge::sendSystemStarting()
{
    // Reiniciar o serviço da Raspberry devolve a OLED à animação de boot.
    // Esse comando não altera GPIO, modo do robô nem saídas de motor.
    return writeLine("SYSTEM_STARTING\n");
}

bool Esp32Bridge::sendSystemReady()
{
    // O comando funciona como heartbeat. A ESP32 exige renovações periódicas
    // para não manter a tela normal se o processo principal deixar de executar.
    return writeLine("SYSTEM_READY\n");
}

bool Esp32Bridge::sendServoAngle(ServoId servo, double angleDegrees)
{
    if (!std::isfinite(angleDegrees) ||
        angleDegrees < config::kServoMinimumAngleDegrees ||
        angleDegrees > config::kServoMaximumAngleDegrees)
    {
        return false;
    }

    std::ostringstream command;
    command << std::fixed << std::setprecision(1)
            << "SERVO," << servoProtocolName(servo) << ','
            << angleDegrees << "\n";
    return writeLine(command.str());
}

bool Esp32Bridge::sendServoPose(const ServoPose& pose)
{
    const auto validAngle = [](double angleDegrees)
    {
        return std::isfinite(angleDegrees) &&
               angleDegrees >= config::kServoMinimumAngleDegrees &&
               angleDegrees <= config::kServoMaximumAngleDegrees;
    };
    if (!validAngle(pose.armDegrees) ||
        !validAngle(pose.wristDegrees) ||
        !validAngle(pose.gripperDegrees))
    {
        return false;
    }

    // Uma única mensagem permite que a ESP32 valide a pose inteira antes de
    // atualizar braço, pulso e garra em uma programação predefinida.
    std::ostringstream command;
    command << std::fixed << std::setprecision(1)
            << "SERVO_POSE," << pose.armDegrees << ','
            << pose.wristDegrees << ',' << pose.gripperDegrees << "\n";
    return writeLine(command.str());
}

bool Esp32Bridge::sendServoSlew(
    ServoId servo, double targetAngleDegrees,
    double maximumSpeedDegreesPerSecond)
{
    if (!std::isfinite(targetAngleDegrees) ||
        targetAngleDegrees < config::kServoMinimumAngleDegrees ||
        targetAngleDegrees > config::kServoMaximumAngleDegrees ||
        !std::isfinite(maximumSpeedDegreesPerSecond) ||
        maximumSpeedDegreesPerSecond <
            config::kServoSlewMinimumSpeedDegreesPerSecond ||
        maximumSpeedDegreesPerSecond >
            config::kServoSlewMaximumSpeedDegreesPerSecond)
    {
        return false;
    }

    // O comando opcional leva alvo e limite definidos pela Raspberry. Nenhuma
    // rotina atual o envia, portanto SERVO e SERVO_POSE mantêm o comportamento.
    std::ostringstream command;
    command << std::fixed << std::setprecision(1)
            << "SERVO_SLEW," << servoProtocolName(servo) << ','
            << targetAngleDegrees << ','
            << maximumSpeedDegreesPerSecond << "\n";
    return writeLine(command.str());
}

bool Esp32Bridge::sendDisableServo(ServoId servo)
{
    std::ostringstream command;
    command << "SERVO_DISABLE," << servoProtocolName(servo) << "\n";
    return writeLine(command.str());
}

bool Esp32Bridge::sendDisableAllServos()
{
    return writeLine("SERVO_DISABLE_ALL\n");
}

bool Esp32Bridge::sendServoCalibrationBegin()
{
    return writeLine("SERVO_CAL_BEGIN\n");
}

bool Esp32Bridge::sendServoCalibrationEnd()
{
    return writeLine("SERVO_CAL_END\n");
}

bool Esp32Bridge::sendServoCalibrationDisableOutput()
{
    return writeLine("SERVO_CAL_DISABLE\n");
}

bool Esp32Bridge::sendServoCalibrationPulse(ServoId servo, int pulseUs)
{
    if (pulseUs < config::kServoCalibrationAbsoluteMinimumPulseUs ||
        pulseUs > config::kServoCalibrationAbsoluteMaximumPulseUs)
    {
        return false;
    }

    std::ostringstream command;
    command << "SERVO_CAL_PULSE," << servoProtocolName(servo) << ','
            << pulseUs << "\n";
    return writeLine(command.str());
}

bool Esp32Bridge::sendServoCalibrationSave(ServoId servo, int pulseAtZeroUs,
                                           int pulseAt180Us)
{
    if (pulseAtZeroUs < config::kServoCalibrationAbsoluteMinimumPulseUs ||
        pulseAtZeroUs > config::kServoCalibrationAbsoluteMaximumPulseUs ||
        pulseAt180Us < config::kServoCalibrationAbsoluteMinimumPulseUs ||
        pulseAt180Us > config::kServoCalibrationAbsoluteMaximumPulseUs)
    {
        return false;
    }

    const int spanUs = std::abs(pulseAt180Us - pulseAtZeroUs);
    if (spanUs < config::kServoCalibrationMinimumSpanUs)
    {
        return false;
    }

    // Os dois pulsos representam diretamente as posições lógicas. A ESP32
    // deduz o sentido e persiste a faixa sem exigir cálculos no dashboard.
    std::ostringstream command;
    command << "SERVO_CAL_SAVE," << servoProtocolName(servo) << ','
            << pulseAtZeroUs << ',' << pulseAt180Us << "\n";
    return writeLine(command.str());
}

bool Esp32Bridge::sendOledMessage(const std::string& title,
                                  const std::string& firstLine,
                                  const std::string& secondLine,
                                  int durationMs)
{
    // O texto é reduzido a ASCII porque a fonte padrão do SSD1306 não possui
    // suporte confiável a UTF-8. O hexadecimal protege os separadores da UART.
    const std::string safeTitle = sanitizeOledText(
        title, static_cast<size_t>(config::kRemoteOledTitleMaxLength));
    const std::string safeFirstLine = sanitizeOledText(
        firstLine, static_cast<size_t>(config::kRemoteOledLineMaxLength));
    const std::string safeSecondLine = sanitizeOledText(
        secondLine, static_cast<size_t>(config::kRemoteOledLineMaxLength));
    if (safeTitle.empty() && safeFirstLine.empty() && safeSecondLine.empty())
    {
        std::cerr << "OLED message ignored: all text fields are empty\n";
        return false;
    }

    const int safeDurationMs = std::clamp(
        durationMs, config::kRemoteOledMinimumDurationMs,
        config::kRemoteOledMaximumDurationMs);
    std::ostringstream command;
    command << "OLED," << safeDurationMs << ','
            << hexEncodeOledField(safeTitle) << ','
            << hexEncodeOledField(safeFirstLine) << ','
            << hexEncodeOledField(safeSecondLine) << "\n";
    return writeLine(command.str());
}

bool Esp32Bridge::sendOledLargeMessage(const std::string& primaryText,
                                       const std::string& secondaryText,
                                       int durationMs)
{
    // O texto principal será ampliado pela ESP32 dentro da região azul do OLED.
    // ASCII evita caracteres que a fonte nativa do SSD1306 não consegue exibir.
    const std::string safePrimaryText = sanitizeOledText(
        primaryText, static_cast<size_t>(config::kRemoteOledLineMaxLength));
    const std::string safeSecondaryText = sanitizeOledText(
        secondaryText, static_cast<size_t>(config::kRemoteOledLineMaxLength));
    if (safePrimaryText.empty())
    {
        std::cerr << "Large OLED message ignored: primary text is empty\n";
        return false;
    }

    const int safeDurationMs = std::clamp(
        durationMs, config::kRemoteOledMinimumDurationMs,
        config::kRemoteOledMaximumDurationMs);
    std::ostringstream command;
    command << "OLED_BIG," << safeDurationMs << ','
            << hexEncodeOledField(safePrimaryText) << ','
            << hexEncodeOledField(safeSecondaryText) << "\n";
    return writeLine(command.str());
}

bool Esp32Bridge::clearOledMessage()
{
    return writeLine("OLED_CLEAR\n");
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

    if (line == "OLED,OK" || line == "OLED,CLEARED")
    {
        std::cout << (line == "OLED,OK"
                          ? "ESP32 OLED remote message accepted\n"
                          : "ESP32 OLED returned to the local screen\n");
        return;
    }

    if (startsWith(line, "SERVO_CAL,"))
    {
        std::cout << "ESP32 servo calibration: " << line.substr(10) << "\n";
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

        if (values.size() >= 28)
        {
            next.remoteOledActive = std::stoi(values[27]) != 0;
        }
        if (values.size() >= 29)
        {
            next.raspberrySystemReady = std::stoi(values[28]) != 0;
        }
        if (values.size() >= 32)
        {
            next.motorCommandAgeMs = std::stoll(values[29]);
            next.motorWatchdogTimedOut = std::stoi(values[30]) != 0;
            const int motorControlSource = std::stoi(values[31]);
            if (motorControlSource == 1)
            {
                next.motorControlSource = "dashboard";
            }
            else if (motorControlSource == 2)
            {
                next.motorControlSource = "raspberry";
            }
            else if (motorControlSource == 0)
            {
                next.motorControlSource = "none";
            }
        }

        if (values.size() >= 41)
        {
            next.armServoAngleDegrees = std::stod(values[32]);
            next.armServoPulseUs = std::stoll(values[33]);
            next.armServoEnabled = std::stoi(values[34]) != 0;
            next.wristServoAngleDegrees = std::stod(values[35]);
            next.wristServoPulseUs = std::stoll(values[36]);
            next.wristServoEnabled = std::stoi(values[37]) != 0;
            next.gripperServoAngleDegrees = std::stod(values[38]);
            next.gripperServoPulseUs = std::stoll(values[39]);
            next.gripperServoEnabled = std::stoi(values[40]) != 0;
        }

        if (values.size() >= 52)
        {
            next.servoCalibrationSupported = true;
            next.servoCalibrationActive = std::stoi(values[41]) != 0;
            next.servoCalibrationSelectedIndex = std::stoi(values[42]);
            next.armServoMinimumPulseUs = std::stoi(values[43]);
            next.armServoMaximumPulseUs = std::stoi(values[44]);
            next.armServoInverted = std::stoi(values[45]) != 0;
            next.wristServoMinimumPulseUs = std::stoi(values[46]);
            next.wristServoMaximumPulseUs = std::stoi(values[47]);
            next.wristServoInverted = std::stoi(values[48]) != 0;
            next.gripperServoMinimumPulseUs = std::stoi(values[49]);
            next.gripperServoMaximumPulseUs = std::stoi(values[50]);
            next.gripperServoInverted = std::stoi(values[51]) != 0;
        }

        if (values.size() >= 61)
        {
            // Os campos finais pertencem à capacidade opcional SERVO_SLEW.
            // A ausência deles preserva compatibilidade com o firmware atual.
            next.servoExtendedControlSupported = true;
            next.armServoTargetAngleDegrees = std::stod(values[52]);
            next.armServoSlewRateDegreesPerSecond = std::stod(values[53]);
            next.armServoSlewActive = std::stoi(values[54]) != 0;
            next.wristServoTargetAngleDegrees = std::stod(values[55]);
            next.wristServoSlewRateDegreesPerSecond = std::stod(values[56]);
            next.wristServoSlewActive = std::stoi(values[57]) != 0;
            next.gripperServoTargetAngleDegrees = std::stod(values[58]);
            next.gripperServoSlewRateDegreesPerSecond = std::stod(values[59]);
            next.gripperServoSlewActive = std::stoi(values[60]) != 0;
        }

        std::lock_guard<std::mutex> lock(telemetryMutex_);
        next.startButtonPressSequence = telemetry_.startButtonPressSequence;
        next.calibrationStatusKnown = telemetry_.calibrationStatusKnown;
        next.lastCalibrationSucceeded = telemetry_.lastCalibrationSucceeded;
        if (values.size() < 27)
        {
            next.calibrationActive = telemetry_.calibrationActive;
        }
        if (values.size() < 28)
        {
            next.remoteOledActive = telemetry_.remoteOledActive;
        }
        if (values.size() < 29)
        {
            next.raspberrySystemReady = telemetry_.raspberrySystemReady;
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

const char* Esp32Bridge::servoProtocolName(ServoId servo)
{
    switch (servo)
    {
    case ServoId::Arm:
        return "ARM";
    case ServoId::Wrist:
        return "WRIST";
    case ServoId::Gripper:
        return "GRIPPER";
    }
    return "UNKNOWN";
}
