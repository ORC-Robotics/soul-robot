#include "obr/dashboard_server.h"

#include "obr/config.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <thread>

namespace
{
// O serviço inicia na raiz do projeto, onde o deploy também copia os recursos visuais.
constexpr const char* kDashboardLogoPath = "assets/dashboard-logo.png";
constexpr const char* kDashboardFaviconPath = "assets/soul-sync-favicon.png";

std::string lowerCopy(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return text;
}

bool parseServoName(const std::string& name, ServoId& servo)
{
    if (name == "arm")
    {
        servo = ServoId::Arm;
        return true;
    }
    if (name == "wrist")
    {
        servo = ServoId::Wrist;
        return true;
    }
    if (name == "gripper")
    {
        servo = ServoId::Gripper;
        return true;
    }
    return false;
}
}

DashboardServer::DashboardServer(RobotState& robotState, Telemetry& telemetry, Esp32Bridge& esp32,
                                 MotorController& motors, ServoController& servos,
                                 StatusLed& readyLed)
    : robotState_(robotState), telemetry_(telemetry), esp32_(esp32),
      motors_(motors), servos_(servos), readyLed_(readyLed)
{
}

DashboardServer::~DashboardServer()
{
    stop();
}

bool DashboardServer::start()
{
    if (!initializeSockets())
    {
        std::cerr << "Failed to initialize sockets\n";
        return false;
    }

    server_ = socket(AF_INET, SOCK_STREAM, 0);
    if (isInvalidSocket(server_))
    {
        std::cerr << "Failed to create socket\n";
        cleanupSockets();
        return false;
    }

    int opt = 1;
#ifdef _WIN32
    const char* reuseOpt = reinterpret_cast<const char*>(&opt);
#else
    const void* reuseOpt = &opt;
#endif
    setsockopt(server_, SOL_SOCKET, SO_REUSEADDR, reuseOpt, sizeof(opt));

    sockaddr_in address = {};
    address.sin_family = AF_INET;
#ifdef _WIN32
    // No Windows, o executável serve apenas como prévia local sem hardware.
    // Restringir ao loopback impede o acesso ao painel por outros computadores da rede.
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
#else
    address.sin_addr.s_addr = INADDR_ANY;
#endif
    address.sin_port = htons(config::kDashboardPort);

    if (bind(server_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
    {
        std::cerr << "Failed to bind port " << config::kDashboardPort << "\n";
        closeSocket(server_);
        cleanupSockets();
        return false;
    }

    if (listen(server_, 8) < 0)
    {
        std::cerr << "Failed to listen on port " << config::kDashboardPort << "\n";
        closeSocket(server_);
        cleanupSockets();
        return false;
    }

    running_ = true;
    acceptThread_ = std::thread(&DashboardServer::acceptLoop, this);
    telemetryThread_ = std::thread(&DashboardServer::telemetryLoop, this);

    return true;
}

void DashboardServer::stop()
{
    if (!running_)
    {
        return;
    }

    running_ = false;
    shutdownSocket(server_);

    {
        std::lock_guard<std::mutex> lock(clientsMutex_);
        for (SocketHandle client : clients_)
        {
            closeSocket(client);
        }
        clients_.clear();
    }

    if (acceptThread_.joinable())
    {
        acceptThread_.join();
    }

    if (telemetryThread_.joinable())
    {
        telemetryThread_.join();
    }

    cleanupSockets();
}

bool DashboardServer::initializeSockets()
{
#if defined(_WIN32) && !defined(__INTELLISENSE__)
    WSADATA data;
    return WSAStartup(MAKEWORD(2, 2), &data) == 0;
#else
    return true;
#endif
}

void DashboardServer::cleanupSockets()
{
#if defined(_WIN32) && !defined(__INTELLISENSE__)
    WSACleanup();
#endif
}

bool DashboardServer::isInvalidSocket(SocketHandle socketHandle) const
{
#ifdef _WIN32
    return socketHandle == INVALID_SOCKET;
#else
    return socketHandle < 0;
#endif
}

void DashboardServer::closeSocket(SocketHandle socketHandle)
{
#if defined(_WIN32) && defined(__INTELLISENSE__)
    (void)socketHandle;
#elif defined(_WIN32)
    closesocket(socketHandle);
#else
    close(socketHandle);
#endif
}

void DashboardServer::shutdownSocket(SocketHandle socketHandle)
{
#if defined(_WIN32) && defined(__INTELLISENSE__)
    (void)socketHandle;
#elif defined(_WIN32)
    shutdown(socketHandle, SD_BOTH);
    closesocket(socketHandle);
#else
    shutdown(socketHandle, SHUT_RDWR);
    close(socketHandle);
#endif
}

void DashboardServer::acceptLoop()
{
    while (running_)
    {
        sockaddr_in clientAddress = {};
        SocketLength clientLength = sizeof(clientAddress);
        SocketHandle client = accept(server_, reinterpret_cast<sockaddr*>(&clientAddress), &clientLength);

        if (!isInvalidSocket(client))
        {
            std::thread(&DashboardServer::handleClient, this, client).detach();
        }
    }
}

void DashboardServer::telemetryLoop()
{
    int telemetryLogCounter = 0;
    auto previousTelemetryTime = std::chrono::steady_clock::time_point{};
    double previousWorkMs = 0.0;

    while (running_)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(config::kTelemetryPeriodMs));

        const auto workStartedAt = std::chrono::steady_clock::now();
        const double effectiveIntervalMs =
            previousTelemetryTime.time_since_epoch().count() == 0
                ? 0.0
                : std::chrono::duration<double, std::milli>(
                      workStartedAt - previousTelemetryTime)
                      .count();
        previousTelemetryTime = workStartedAt;

        TelemetrySample sample = telemetry_.read();
        const long long generatedUnixMs =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();
        std::size_t websocketClientCount = 0;
        {
            std::lock_guard<std::mutex> lock(clientsMutex_);
            websocketClientCount = clients_.size();
        }
        const std::uint64_t sequence = ++telemetrySequence_;
        // O pacote publica o custo do ciclo anterior porque só é possível
        // incluir o tempo completo depois que o broadcast daquele ciclo termina.
        std::string json = buildTelemetryJson(
            sample,
            sequence,
            generatedUnixMs,
            effectiveIntervalMs,
            previousWorkMs,
            websocketClientCount);
        broadcast(json);
        previousWorkMs = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - workStartedAt)
                             .count();

        ++telemetryLogCounter;
        if (telemetryLogCounter >= config::kTelemetryLogEverySamples)
        {
            telemetryLogCounter = 0;
            std::cout << "Telemetry " << json << "\n";
        }
    }
}

void DashboardServer::handleClient(SocketHandle client)
{
    char buffer[4096] = {};
    SocketResult bytesRead = recv(client, buffer, sizeof(buffer) - 1, 0);
    if (bytesRead <= 0)
    {
        closeSocket(client);
        return;
    }

    std::string request(buffer, bytesRead);
    if (lowerCopy(request).find("upgrade: websocket") != std::string::npos)
    {
        handleWebSocket(client, request);
        return;
    }

    if (request.find("GET /camera.jpg") == 0)
    {
        if (!sendCameraFrame(client))
        {
            sendHttpNotFound(client);
        }
        closeSocket(client);
        return;
    }

    if (request.find("GET /camera-stream.mjpg") == 0)
    {
        if (!proxyCameraStream(client, request, config::kCameraStreamPort,
                               config::kCameraStreamPath, true))
        {
            sendHttpNotFound(client);
        }
        closeSocket(client);
        return;
    }

    if (request.find("HEAD /camera-stream.mjpg") == 0)
    {
        if (!sendCameraStreamHead(client, config::kCameraStreamPort))
        {
            sendHttpNotFound(client);
        }
        closeSocket(client);
        return;
    }

    if (request.find("GET /camera-status.json") == 0)
    {
        sendCameraStatus(client, config::kCameraStatusPath);
        closeSocket(client);
        return;
    }

    if (request.find("GET /forward-camera-stream.mjpg") == 0)
    {
        if (!proxyCameraStream(client, request, config::kForwardCameraStreamPort,
                               config::kForwardCameraStreamPath, false))
        {
            sendHttpNotFound(client);
        }
        closeSocket(client);
        return;
    }

    if (request.find("HEAD /forward-camera-stream.mjpg") == 0)
    {
        if (!sendCameraStreamHead(client, config::kForwardCameraStreamPort))
        {
            sendHttpNotFound(client);
        }
        closeSocket(client);
        return;
    }

    if (request.find("GET /forward-camera-status.json") == 0)
    {
        sendCameraStatus(client, config::kForwardCameraStatusPath);
        closeSocket(client);
        return;
    }

    if (request.find("GET /dashboard-logo.png") == 0)
    {
        if (!sendStaticFile(client, kDashboardLogoPath, "image/png"))
        {
            sendHttpNotFound(client);
        }
        closeSocket(client);
        return;
    }

    if (request.find("GET /soul-sync-favicon.png") == 0)
    {
        if (!sendStaticFile(client, kDashboardFaviconPath, "image/png"))
        {
            sendHttpNotFound(client);
        }
        closeSocket(client);
        return;
    }

    sendHttpResponse(client, dashboardHtml(), "text/html; charset=utf-8");
    closeSocket(client);
}

void DashboardServer::handleWebSocket(SocketHandle client, const std::string& request)
{
    std::string key = getHeaderValue(request, "Sec-WebSocket-Key: ");
    std::string accept = base64Encode(sha1(key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"));

    std::ostringstream response;
    response << "HTTP/1.1 101 Switching Protocols\r\n"
             << "Upgrade: websocket\r\n"
             << "Connection: Upgrade\r\n"
             << "Sec-WebSocket-Accept: " << accept << "\r\n\r\n";

    std::string text = response.str();
    send(client, text.c_str(), static_cast<int>(text.size()), 0);
    addWebSocketClient(client);

    std::string payload;
    while (running_ && readWebSocketFrame(client, payload))
    {
        handleCommand(payload);
    }

    removeWebSocketClient(client);
    closeSocket(client);
}

void DashboardServer::addWebSocketClient(SocketHandle client)
{
    std::lock_guard<std::mutex> lock(clientsMutex_);
    clients_.push_back(client);
}

void DashboardServer::removeWebSocketClient(SocketHandle client)
{
    std::lock_guard<std::mutex> lock(clientsMutex_);
    clients_.erase(std::remove(clients_.begin(), clients_.end(), client), clients_.end());
}

void DashboardServer::broadcast(const std::string& message)
{
    std::lock_guard<std::mutex> lock(clientsMutex_);

    for (auto it = clients_.begin(); it != clients_.end();)
    {
        if (sendWebSocketText(*it, message))
        {
            ++it;
        }
        else
        {
            closeSocket(*it);
            it = clients_.erase(it);
        }
    }
}

void DashboardServer::endServoCalibrationIfActive()
{
    const RobotSnapshot state = robotState_.snapshot();
    const Esp32TelemetrySnapshot esp32 = esp32_.telemetrySnapshot();
    if (state.servoCalibrationActive || esp32.servoCalibrationActive)
    {
        // O encerramento é enviado antes da próxima troca de modo para remover
        // o pulso imediatamente. O STOP subsequente permanece como segunda trava.
        if (!servos_.endCalibration())
        {
            std::cerr << "Servo calibration end command was not sent to ESP32\n";
        }
    }
    robotState_.endServoCalibration();
}

void DashboardServer::handleCommand(const std::string& message)
{
    if (message.find("\"command\":\"estop\"") != std::string::npos)
    {
        endServoCalibrationIfActive();
        robotState_.emergencyStop();
        std::cout << "Emergency stop received\n";
    }
    else if (message.find("\"command\":\"start\"") != std::string::npos)
    {
        endServoCalibrationIfActive();
        esp32_.sendClearEmergencyStop();
        robotState_.start();
        std::cout << "Start received\n";
    }
    else if (message.find("\"command\":\"auto\"") != std::string::npos)
    {
        endServoCalibrationIfActive();
        esp32_.sendClearEmergencyStop();
        robotState_.startAutonomous();
        std::cout << "Autonomous start received\n";
    }
    else if (message.find("\"command\":\"set_autonomous_mission\"") != std::string::npos)
    {
        endServoCalibrationIfActive();
        if (message.find("\"mission\":\"main_mission\"") != std::string::npos)
        {
            robotState_.setAutonomousMission(AutonomousMission::MainMission);
            std::cout << "Autonomous mission selected: main_mission\n";
        }
        else if (message.find("\"mission\":\"turn_right_90\"") != std::string::npos)
        {
            robotState_.setAutonomousMission(AutonomousMission::TurnRight90);
            std::cout << "Autonomous mission selected: turn_right_90\n";
        }
        else if (message.find("\"mission\":\"drive_distance\"") != std::string::npos)
        {
            const double targetCm = getJsonNumber(
                message, "distanceCm", config::kDriveDistanceDefaultTargetCm);
            if (robotState_.setDriveDistanceTargetCm(targetCm))
            {
                robotState_.setAutonomousMission(AutonomousMission::DriveDistance);
                std::cout << "Autonomous mission selected: drive_distance, target="
                          << targetCm << " cm\n";
            }
            else
            {
                // Alvos inválidos não podem substituir a missão segura atual.
                std::cerr << "Invalid drive-distance target ignored\n";
            }
        }
        else if (message.find("\"mission\":\"rescue_area\"") != std::string::npos)
        {
            // Selecionar a etapa não liga o detector. O gate só abre depois
            // que o modo autônomo inicia, e fecha novamente em Stop ou E-Stop.
            robotState_.setAutonomousMission(AutonomousMission::RescueArea);
            std::cout << "Autonomous mission selected: rescue_area\n";
        }
        else if (message.find("\"mission\":\"rescue_zone_detection\"") != std::string::npos)
        {
            robotState_.setAutonomousMission(
                AutonomousMission::RescueZoneDetection);
            std::cout << "Autonomous mission selected: rescue_zone_detection\n";
        }
        else if (message.find("\"mission\":\"rescue_zone_frame\"") != std::string::npos)
        {
            const bool targetRed =
                message.find("\"targetColor\":\"red\"") != std::string::npos;
            const bool targetGreen =
                message.find("\"targetColor\":\"green\"") != std::string::npos;
            if (!targetRed && !targetGreen)
            {
                std::cerr << "FRAME_ZONE target color must be green or red\n";
                return;
            }
            robotState_.setAutonomousMission(AutonomousMission::RescueZoneFrame);
            robotState_.setRescueZoneTargetColor(
                targetRed ? RescueZoneTargetColor::Red
                          : RescueZoneTargetColor::Green);
            std::cout << "Autonomous mission selected: rescue_zone_frame, target="
                      << (targetRed ? "red" : "green") << '\n';
        }
        else if (message.find("\"mission\":\"obstacle_avoidance\"") != std::string::npos)
        {
            robotState_.setAutonomousMission(
                AutonomousMission::ObstacleAvoidance);
            std::cout << "Autonomous mission selected: obstacle_avoidance\n";
        }
        else if (message.find("\"mission\":\"servo_initialize\"") != std::string::npos)
        {
            robotState_.setAutonomousMission(AutonomousMission::ServoInitialize);
            std::cout << "Autonomous mission selected: servo_initialize\n";
        }
        else if (message.find("\"mission\":\"servo_capture\"") != std::string::npos)
        {
            robotState_.setAutonomousMission(AutonomousMission::ServoCapture);
            std::cout << "Autonomous mission selected: servo_capture\n";
        }
        else if (message.find("\"mission\":\"servo_internal_storage\"") != std::string::npos)
        {
            robotState_.setAutonomousMission(
                AutonomousMission::ServoInternalStorage);
            std::cout << "Autonomous mission selected: servo_internal_storage\n";
        }
        else if (message.find("\"mission\":\"servo_deposit\"") != std::string::npos)
        {
            robotState_.setAutonomousMission(AutonomousMission::ServoDeposit);
            std::cout << "Autonomous mission selected: servo_deposit\n";
        }
        else if (message.find("\"mission\":\"servo_full_sequence\"") != std::string::npos)
        {
            robotState_.setAutonomousMission(
                AutonomousMission::ServoFullSequence);
            std::cout << "Autonomous mission selected: servo_full_sequence\n";
        }
        else if (message.find("\"mission\":\"servo_full_sequence_two\"") != std::string::npos)
        {
            robotState_.setAutonomousMission(
                AutonomousMission::ServoFullSequenceTwo);
            std::cout << "Autonomous mission selected: servo_full_sequence_two\n";
        }
        else
        {
            // Missões desconhecidas são ignoradas para nunca executar um
            // comportamento diferente daquele selecionado pelo operador.
            std::cerr << "Invalid autonomous mission ignored\n";
        }
    }
    else if (message.find("\"command\":\"servo_routine_confirm\"") != std::string::npos)
    {
        if (!robotState_.confirmServoRoutineAction())
        {
            std::cerr << "Servo routine confirmation ignored: no step is waiting\n";
        }
    }
    else if (message.find("\"command\":\"stop\"") != std::string::npos)
    {
        endServoCalibrationIfActive();
        robotState_.stop();
        motors_.stop();
        std::cout << "Stop received\n";
    }
    else if (message.find("\"command\":\"calibrate\"") != std::string::npos)
    {
        // A calibração nunca preserva um comando de movimento anterior.
        // A ESP32 também trava motores e servos até uma nova partida explícita.
        endServoCalibrationIfActive();
        robotState_.stop();
        if (!esp32_.sendCalibrateSensors())
        {
            std::cerr << "Sensor calibration command was not sent to ESP32\n";
        }
        else
        {
            std::cout << "Sensor calibration requested\n";
        }
    }
    else if (message.find("\"command\":\"oled_large_message\"") != std::string::npos)
    {
        const std::string primaryText = getJsonString(message, "primaryText", "");
        const std::string secondaryText = getJsonString(message, "secondaryText", "");
        const double requestedDurationMs = getJsonNumber(
            message, "durationMs", config::kRemoteOledMaximumDurationMs);
        const int durationMs = static_cast<int>(std::clamp(
            requestedDurationMs,
            static_cast<double>(config::kRemoteOledMinimumDurationMs),
            static_cast<double>(config::kRemoteOledMaximumDurationMs)));
        // O layout grande usa o mesmo caminho dos alertas automáticos. Assim,
        // o painel testa exatamente a fonte adaptativa e o pulso gravados na ESP32.
        if (!esp32_.sendOledLargeMessage(
                primaryText, secondaryText, durationMs))
        {
            std::cerr << "Large OLED message was not sent to ESP32\n";
        }
    }
    else if (message.find("\"command\":\"oled_message\"") != std::string::npos)
    {
        const std::string title = getJsonString(message, "title", "");
        const std::string firstLine = getJsonString(message, "firstLine", "");
        const std::string secondLine = getJsonString(message, "secondLine", "");
        const double requestedDurationMs = getJsonNumber(
            message, "durationMs", config::kRemoteOledMaximumDurationMs);
        const int durationMs = static_cast<int>(std::clamp(
            requestedDurationMs,
            static_cast<double>(config::kRemoteOledMinimumDurationMs),
            static_cast<double>(config::kRemoteOledMaximumDurationMs)));
        if (!esp32_.sendOledMessage(title, firstLine, secondLine, durationMs))
        {
            std::cerr << "OLED message was not sent to ESP32\n";
        }
    }
    else if (message.find("\"command\":\"oled_clear\"") != std::string::npos)
    {
        if (!esp32_.clearOledMessage())
        {
            std::cerr << "OLED clear command was not sent to ESP32\n";
        }
    }
    else if (message.find("\"command\":\"servo_disable_all\"") != std::string::npos)
    {
        robotState_.disableServos();
    }
    else if (message.find("\"command\":\"servo_calibration_begin\"") != std::string::npos)
    {
        const RobotSnapshot state = robotState_.snapshot();
        const Esp32TelemetrySnapshot esp32 = esp32_.telemetrySnapshot();
        const bool calibrationAllowed =
            (state.mode == "stopped" || state.mode == "manual") &&
            !state.emergencyStop &&
            esp32.sensorFresh && esp32.pca9685Ok &&
            esp32.raspberrySystemReady &&
            esp32.servoCalibrationSupported &&
            !esp32.emergencyStopActive && !esp32.calibrationActive &&
            !esp32.servoCalibrationActive;
        if (!calibrationAllowed)
        {
            std::cerr << "Servo calibration begin ignored: system is not in a safe state\n";
        }
        else
        {
            // A transição local ocorre primeiro para que o laço principal passe
            // a renovar somente MOTOR 0,0 antes de qualquer pulso de bancada.
            robotState_.stop();
            if (!robotState_.beginServoCalibration() ||
                !servos_.beginCalibration())
            {
                robotState_.endServoCalibration();
                motors_.stop();
                std::cerr << "Servo calibration could not be started\n";
            }
            else
            {
                std::cout << "Servo calibration started\n";
            }
        }
    }
    else if (message.find("\"command\":\"servo_calibration_end\"") != std::string::npos)
    {
        endServoCalibrationIfActive();
        robotState_.stop();
        motors_.stop();
        std::cout << "Servo calibration stopped\n";
    }
    else if (message.find("\"command\":\"servo_calibration_disable\"") != std::string::npos)
    {
        const RobotSnapshot state = robotState_.snapshot();
        if (state.servoCalibrationActive &&
            !servos_.disableCalibrationOutput())
        {
            std::cerr << "Servo calibration output could not be disabled\n";
        }
    }
    else if (message.find("\"command\":\"servo_calibration_pulse\"") != std::string::npos)
    {
        const std::string servoName = getJsonString(message, "servo", "");
        const double requestedPulseUs = getJsonNumber(
            message, "pulseUs", std::numeric_limits<double>::quiet_NaN());
        ServoId servo = ServoId::Arm;
        const bool pulseIsInteger = std::isfinite(requestedPulseUs) &&
                                    std::floor(requestedPulseUs) == requestedPulseUs;
        const bool valueFitsInt = pulseIsInteger &&
                                  requestedPulseUs >= std::numeric_limits<int>::min() &&
                                  requestedPulseUs <= std::numeric_limits<int>::max();
        const int pulseUs = valueFitsInt ? static_cast<int>(requestedPulseUs) : 0;
        const RobotSnapshot state = robotState_.snapshot();
        const Esp32TelemetrySnapshot esp32 = esp32_.telemetrySnapshot();
        const bool calibrationControlAllowed =
            state.servoCalibrationActive && !state.emergencyStop &&
            esp32.sensorFresh && esp32.pca9685Ok &&
            esp32.raspberrySystemReady && esp32.servoCalibrationActive &&
            !esp32.emergencyStopActive && !esp32.calibrationActive;
        if (!parseServoName(servoName, servo) || !valueFitsInt ||
            !ServoController::isValidCalibrationPulse(pulseUs) ||
            !calibrationControlAllowed ||
            !servos_.setCalibrationPulse(servo, pulseUs))
        {
            std::cerr << "Servo calibration pulse ignored\n";
        }
    }
    else if (message.find("\"command\":\"servo_calibration_save\"") != std::string::npos)
    {
        const std::string servoName = getJsonString(message, "servo", "");
        const double requestedZeroUs = getJsonNumber(
            message, "pulseAtZeroUs", std::numeric_limits<double>::quiet_NaN());
        const double requested180Us = getJsonNumber(
            message, "pulseAt180Us", std::numeric_limits<double>::quiet_NaN());
        ServoId servo = ServoId::Arm;
        const bool valuesAreIntegers =
            std::isfinite(requestedZeroUs) && std::isfinite(requested180Us) &&
            std::floor(requestedZeroUs) == requestedZeroUs &&
            std::floor(requested180Us) == requested180Us;
        const bool valuesFitInt =
            valuesAreIntegers &&
            requestedZeroUs >= std::numeric_limits<int>::min() &&
            requestedZeroUs <= std::numeric_limits<int>::max() &&
            requested180Us >= std::numeric_limits<int>::min() &&
            requested180Us <= std::numeric_limits<int>::max();
        const int pulseAtZeroUs = valuesFitInt ? static_cast<int>(requestedZeroUs) : 0;
        const int pulseAt180Us = valuesFitInt ? static_cast<int>(requested180Us) : 0;
        const RobotSnapshot state = robotState_.snapshot();
        const Esp32TelemetrySnapshot esp32 = esp32_.telemetrySnapshot();
        const bool calibrationControlAllowed =
            state.servoCalibrationActive && !state.emergencyStop &&
            esp32.sensorFresh && esp32.pca9685Ok &&
            esp32.raspberrySystemReady && esp32.servoCalibrationActive &&
            !esp32.emergencyStopActive && !esp32.calibrationActive;
        if (!parseServoName(servoName, servo) || !valuesFitInt ||
            !ServoController::isValidCalibrationEndpoints(
                pulseAtZeroUs, pulseAt180Us) ||
            !calibrationControlAllowed ||
            !servos_.saveCalibration(servo, pulseAtZeroUs, pulseAt180Us))
        {
            std::cerr << "Servo calibration save ignored\n";
        }
    }
    else if (message.find("\"command\":\"servo_angle\"") != std::string::npos)
    {
        const std::string servoName = getJsonString(message, "servo", "");
        const double angleDegrees = getJsonNumber(
            message, "angleDegrees", std::numeric_limits<double>::quiet_NaN());
        ServoId servo = ServoId::Arm;
        const bool knownServo = parseServoName(servoName, servo);

        const RobotSnapshot state = robotState_.snapshot();
        const Esp32TelemetrySnapshot esp32 = esp32_.telemetrySnapshot();
        const bool manualControlAllowed =
            state.mode == "manual" && !state.emergencyStop &&
            esp32.sensorFresh && esp32.pca9685Ok &&
            esp32.raspberrySystemReady && !esp32.emergencyStopActive &&
            !esp32.calibrationActive && !esp32.servoCalibrationActive &&
            !state.servoCalibrationActive;
        if (!knownServo)
        {
            std::cerr << "Servo command ignored: invalid name\n";
        }
        else if (!manualControlAllowed)
        {
            // O painel só movimenta mecanismos no modo Manual. As rotinas
            // autônomas publicam poses validadas no RobotState.
            std::cerr << "Servo command ignored: manual control is not available\n";
        }
        else if (!robotState_.setManualServoAngle(servo, angleDegrees))
        {
            std::cerr << "Servo command ignored: invalid angle or state\n";
        }
    }
    else if (message.find("\"command\":\"set_forward_camera\"") != std::string::npos)
    {
        bool enabled = false;
        if (!getJsonBool(message, "enabled", enabled))
        {
            std::cerr << "Forward camera command ignored: enabled must be boolean\n";
        }
        else if (!setForwardCameraEnabled(enabled))
        {
            std::cerr << "Forward camera state could not be written\n";
        }
        else
        {
            std::cout << "Forward camera requested: "
                      << (enabled ? "enabled" : "disabled") << "\n";
        }
    }
    else if (message.find("\"command\":\"set_line_camera\"") != std::string::npos)
    {
        bool enabled = false;
        if (!getJsonBool(message, "enabled", enabled))
        {
            std::cerr << "Line camera command ignored: enabled must be boolean\n";
        }
        else if (!setLineCameraEnabled(enabled))
        {
            std::cerr << "Line camera state could not be written\n";
        }
        else
        {
            std::cout << "Line camera requested: "
                      << (enabled ? "enabled" : "disabled") << "\n";
        }
    }
    else if (message.find("\"command\":\"drive_raw\"") != std::string::npos)
    {
        const double left = getJsonNumber(message, "left", 0.0);
        const double right = getJsonNumber(message, "right", 0.0);
        robotState_.driveRawDiagnostic(left, right);
        std::cout << "Raw diagnostic drive left=" << left << " right=" << right << "\n";
    }
    else if (message.find("\"command\":\"drive\"") != std::string::npos)
    {
        double left = getJsonNumber(message, "left", 0.0);
        double right = getJsonNumber(message, "right", 0.0);
        robotState_.drive(left, right);
        std::cout << "Drive left=" << left << " right=" << right << "\n";
    }
}

std::string DashboardServer::buildTelemetryJson(
    const TelemetrySample& sample,
    std::uint64_t sequence,
    long long generatedUnixMs,
    double effectiveIntervalMs,
    double previousWorkMs,
    std::size_t websocketClientCount) const
{
    RobotSnapshot state = robotState_.snapshot();
    Esp32TelemetrySnapshot esp32 = esp32_.telemetrySnapshot();
    MotorSynchronizationSnapshot motorSync = motors_.synchronizationSnapshot();

    std::ostringstream json;
    json << std::fixed << std::setprecision(2)
         << "{\"cpu\":" << sample.cpuUsage
         << ",\"temperature\":" << sample.temperature
         << ",\"ram\":" << sample.ramUsage
         << ",\"telemetrySequence\":" << sequence
         << ",\"telemetryGeneratedUnixMs\":" << generatedUnixMs
         << ",\"telemetryConfiguredPeriodMs\":" << config::kTelemetryPeriodMs
         << ",\"telemetryEffectiveIntervalMs\":" << effectiveIntervalMs
         << ",\"telemetryServerWorkMs\":" << previousWorkMs
         << ",\"websocketClientCount\":" << websocketClientCount
         << ",\"mode\":\"" << state.mode << "\""
         << ",\"autonomousMission\":\"" << autonomousMissionName(state.autonomousMission) << "\""
         << ",\"rescueZoneTargetColor\":\""
         << rescueZoneTargetColorName(state.rescueZoneTargetColor) << "\""
         << ",\"autonomousRunSequence\":" << state.autonomousRunSequence
         << ",\"rawMotorCommand\":" << (state.rawMotorCommand ? "true" : "false")
         << ",\"autonomousPhase\":\"" << state.autonomousStatus.phase << "\""
         << ",\"autonomousAction\":\"" << state.autonomousStatus.action << "\""
         << ",\"autonomousProgressPercent\":" << state.autonomousStatus.progressPercent
         << ",\"driveDistanceTargetCm\":" << state.driveDistanceTargetCm
         << ",\"autonomousTargetDistanceCm\":" << state.autonomousStatus.targetDistanceCm
         << ",\"autonomousLeftDistanceCm\":" << state.autonomousStatus.leftDistanceCm
         << ",\"autonomousRightDistanceCm\":" << state.autonomousStatus.rightDistanceCm
         << ",\"autonomousAverageDistanceCm\":" << state.autonomousStatus.averageDistanceCm
         << ",\"servoRoutineWaitingForConfirmation\":"
         << (state.autonomousStatus.servoRoutineWaitingForConfirmation ? "true" : "false")
         << ",\"servoRoutineInternalObjectStored\":"
         << (state.servoRoutineInternalObjectStored ? "true" : "false")
         << ",\"forwardAssistState\":\"" << state.autonomousStatus.forwardAssistState << "\""
         << ",\"forwardAssistDirection\":\"" << state.autonomousStatus.forwardAssistDirection << "\""
         << ",\"forwardAssistLatchedDirection\":\"" << state.autonomousStatus.forwardAssistLatchedDirection << "\""
         << ",\"forwardAssistEntryBlocker\":\"" << state.autonomousStatus.forwardAssistEntryBlocker << "\""
         << ",\"forwardAssistYawDeltaDeg\":" << state.autonomousStatus.forwardAssistYawDeltaDeg
         << ",\"forwardAssistFarTrusted\":" << (state.autonomousStatus.forwardAssistFarTrusted ? "true" : "false")
         << ",\"forwardAssistMediumTrusted\":" << (state.autonomousStatus.forwardAssistMediumTrusted ? "true" : "false")
         << ",\"forwardAssistGapCandidate\":" << (state.autonomousStatus.forwardAssistGapCandidate ? "true" : "false")
         << ",\"forwardAssistEntryAllowed\":" << (state.autonomousStatus.forwardAssistEntryAllowed ? "true" : "false")
         << ",\"forwardAssistYawLimitDeg\":" << config::kForwardAssistMaximumSearchDegrees
         << ",\"bottomStableRequiredFrames\":" << config::kForwardAssistBottomStableFrames
         << ",\"forwardLineVisible\":" << (state.autonomousStatus.forwardLineVisible ? "true" : "false")
         << ",\"forwardPathConfidence\":" << state.autonomousStatus.forwardPathConfidence
         << ",\"forwardPathState\":\"" << state.autonomousStatus.forwardPathState << "\""
         << ",\"gapValidationDecision\":\"" << state.autonomousStatus.gapValidationDecision << "\""
         << ",\"nearLineState\":\"" << state.autonomousStatus.nearLineState << "\""
         << ",\"lineControlSource\":" << std::quoted(state.autonomousStatus.bottomLineControlSource)
         << ",\"forwardLinePosition\":" << state.autonomousStatus.forwardLinePosition
         << ",\"bottomStableFrames\":" << state.autonomousStatus.bottomStableFrames
         << ",\"rescueZoneUltrasonicFresh\":"
         << (state.autonomousStatus.rescueZoneUltrasonicFresh ? "true" : "false")
         << ",\"rescueZoneUltrasonicValid\":"
         << (state.autonomousStatus.rescueZoneUltrasonicValid ? "true" : "false")
         << ",\"rescueZoneUltrasonicDistanceCm\":"
         << state.autonomousStatus.rescueZoneUltrasonicDistanceCm
         << ",\"rescueZoneFrameCompleted\":"
         << (state.autonomousStatus.rescueZoneFrameCompleted ? "true" : "false")
         << ",\"rescueZoneFrameFailed\":"
         << (state.autonomousStatus.rescueZoneFrameFailed ? "true" : "false")
         << ",\"rescueZoneFrameCompletionReason\":"
         << std::quoted(state.autonomousStatus.rescueZoneFrameCompletionReason)
          << ",\"left\":" << state.left
          << ",\"right\":" << state.right
         << ",\"requestedLeft\":" << state.left
         << ",\"requestedRight\":" << state.right
         << ",\"emergency\":" << (state.emergencyStop ? "true" : "false")
         << ",\"esp32SerialOpen\":" << (esp32.serialOpen ? "true" : "false")
         << ",\"esp32SensorFresh\":" << (esp32.sensorFresh ? "true" : "false")
         << ",\"systemReady\":" << (readyLed_.isReady() ? "true" : "false")
         << ",\"esp32LastSensorAgeMs\":" << esp32.lastSensorAgeMs
         << ",\"mpuOk\":" << (esp32.mpuOk ? "true" : "false")
         << ",\"ultrasonicDistanceCm\":" << esp32.ultrasonicDistanceCm
         << ",\"gyroZDegPerSec\":" << esp32.gyroZDegPerSec
         << ",\"yawZDeg\":" << esp32.yawZDeg
         << ",\"accelX\":" << esp32.accelX
         << ",\"accelY\":" << esp32.accelY
         << ",\"accelZ\":" << esp32.accelZ
         << ",\"gyroXDegPerSec\":" << esp32.gyroXDegPerSec
         << ",\"gyroYDegPerSec\":" << esp32.gyroYDegPerSec
         << ",\"rampAngleDeg\":" << esp32.rampAngleDeg
         << ",\"imuTemperatureCelsius\":" << esp32.imuTemperatureCelsius
         << ",\"batteryVoltage\":" << esp32.batteryVoltage
         << ",\"batteryAdcMillivolts\":" << esp32.batteryAdcMillivolts
         << ",\"leftEncoderCount\":" << esp32.leftEncoderCount
         << ",\"rightEncoderCount\":" << esp32.rightEncoderCount
         << ",\"leftEncoderRate\":" << esp32.leftEncoderRate
         << ",\"rightEncoderRate\":" << esp32.rightEncoderRate
         << ",\"esp32AppliedLeftPower\":" << esp32.appliedLeftPower
         << ",\"esp32AppliedRightPower\":" << esp32.appliedRightPower
         << ",\"commandedLeft\":" << motorSync.correctedLeftPower
         << ",\"commandedRight\":" << motorSync.correctedRightPower
         << ",\"appliedLeft\":" << esp32.appliedLeftPower
         << ",\"appliedRight\":" << esp32.appliedRightPower
         << ",\"leftMotorStarting\":"
         << (motorSync.leftMotorStarting ? "true" : "false")
         << ",\"rightMotorStarting\":"
         << (motorSync.rightMotorStarting ? "true" : "false")
         << ",\"leftMotorRunning\":"
         << (motorSync.leftMotorRunning ? "true" : "false")
         << ",\"rightMotorRunning\":"
         << (motorSync.rightMotorRunning ? "true" : "false")
         << ",\"leftMotorConfirmationSamples\":"
         << motorSync.leftMotorConfirmationSamples
         << ",\"rightMotorConfirmationSamples\":"
         << motorSync.rightMotorConfirmationSamples
         << ",\"motorSyncEligible\":" << (motorSync.eligible ? "true" : "false")
         << ",\"motorSyncActive\":" << (motorSync.active ? "true" : "false")
         << ",\"motorSyncEncoderDataValid\":"
         << (motorSync.encoderDataValid ? "true" : "false")
         << ",\"motorSyncCorrectionApplied\":"
         << (motorSync.correctionApplied ? "true" : "false")
         << ",\"motorSyncDirection\":" << motorSync.direction
         << ",\"motorSyncValidSamples\":" << motorSync.validSamples
         << ",\"motorSyncLeftScale\":" << motorSync.leftScale
         << ",\"motorSyncRightScale\":" << motorSync.rightScale
         << ",\"motorSyncLeftEfficiency\":" << motorSync.filteredLeftEfficiency
         << ",\"motorSyncRightEfficiency\":" << motorSync.filteredRightEfficiency
         << ",\"motorSyncCorrectedLeftPower\":" << motorSync.correctedLeftPower
         << ",\"motorSyncCorrectedRightPower\":" << motorSync.correctedRightPower
         << ",\"startButtonPressed\":" << (esp32.startButtonPressed ? "true" : "false")
         << ",\"pca9685Ok\":" << (esp32.pca9685Ok ? "true" : "false")
         << ",\"armServoAngleDegrees\":" << esp32.armServoAngleDegrees
         << ",\"armServoPulseUs\":" << esp32.armServoPulseUs
         << ",\"armServoEnabled\":" << (esp32.armServoEnabled ? "true" : "false")
         << ",\"wristServoAngleDegrees\":" << esp32.wristServoAngleDegrees
         << ",\"wristServoPulseUs\":" << esp32.wristServoPulseUs
         << ",\"wristServoEnabled\":" << (esp32.wristServoEnabled ? "true" : "false")
         << ",\"gripperServoAngleDegrees\":" << esp32.gripperServoAngleDegrees
         << ",\"gripperServoPulseUs\":" << esp32.gripperServoPulseUs
         << ",\"gripperServoEnabled\":" << (esp32.gripperServoEnabled ? "true" : "false")
         << ",\"armServoTargetAngleDegrees\":" << esp32.armServoTargetAngleDegrees
         << ",\"armServoSlewRateDegreesPerSecond\":"
         << esp32.armServoSlewRateDegreesPerSecond
         << ",\"armServoSlewActive\":" << (esp32.armServoSlewActive ? "true" : "false")
         << ",\"wristServoTargetAngleDegrees\":" << esp32.wristServoTargetAngleDegrees
         << ",\"wristServoSlewRateDegreesPerSecond\":"
         << esp32.wristServoSlewRateDegreesPerSecond
         << ",\"wristServoSlewActive\":" << (esp32.wristServoSlewActive ? "true" : "false")
         << ",\"gripperServoTargetAngleDegrees\":" << esp32.gripperServoTargetAngleDegrees
         << ",\"gripperServoSlewRateDegreesPerSecond\":"
         << esp32.gripperServoSlewRateDegreesPerSecond
         << ",\"gripperServoSlewActive\":" << (esp32.gripperServoSlewActive ? "true" : "false")
         << ",\"servoExtendedControlSupported\":"
         << (esp32.servoExtendedControlSupported ? "true" : "false")
         << ",\"servoCalibrationRequested\":"
         << (state.servoCalibrationActive ? "true" : "false")
         << ",\"servoCalibrationSupported\":"
         << (esp32.servoCalibrationSupported ? "true" : "false")
         << ",\"servoCalibrationActive\":"
         << (esp32.servoCalibrationActive ? "true" : "false")
         << ",\"servoCalibrationSelectedIndex\":"
         << esp32.servoCalibrationSelectedIndex
         << ",\"armServoMinimumPulseUs\":" << esp32.armServoMinimumPulseUs
         << ",\"armServoMaximumPulseUs\":" << esp32.armServoMaximumPulseUs
         << ",\"armServoInverted\":" << (esp32.armServoInverted ? "true" : "false")
         << ",\"wristServoMinimumPulseUs\":" << esp32.wristServoMinimumPulseUs
         << ",\"wristServoMaximumPulseUs\":" << esp32.wristServoMaximumPulseUs
         << ",\"wristServoInverted\":" << (esp32.wristServoInverted ? "true" : "false")
         << ",\"gripperServoMinimumPulseUs\":" << esp32.gripperServoMinimumPulseUs
         << ",\"gripperServoMaximumPulseUs\":" << esp32.gripperServoMaximumPulseUs
         << ",\"gripperServoInverted\":" << (esp32.gripperServoInverted ? "true" : "false")
         << ",\"servoCalibrationAbsoluteMinimumPulseUs\":"
         << config::kServoCalibrationAbsoluteMinimumPulseUs
         << ",\"servoCalibrationAbsoluteMaximumPulseUs\":"
         << config::kServoCalibrationAbsoluteMaximumPulseUs
         << ",\"servoCalibrationMinimumSpanUs\":"
         << config::kServoCalibrationMinimumSpanUs
         << ",\"oledOk\":" << (esp32.oledOk ? "true" : "false")
         << ",\"esp32RemoteOledActive\":" << (esp32.remoteOledActive ? "true" : "false")
         << ",\"esp32RaspberrySystemReady\":" << (esp32.raspberrySystemReady ? "true" : "false")
         << ",\"oledNavigationAlertDurationMs\":"
         << config::kOledNavigationAlertDurationMs
         << ",\"motorSleepPinHigh\":" << (esp32.motorSleepPinHigh ? "true" : "false")
         << ",\"esp32EmergencyStop\":" << (esp32.emergencyStopActive ? "true" : "false")
         << ",\"esp32CalibrationActive\":" << (esp32.calibrationActive ? "true" : "false")
         << ",\"esp32CalibrationStatusKnown\":" << (esp32.calibrationStatusKnown ? "true" : "false")
         << ",\"esp32LastCalibrationSucceeded\":" << (esp32.lastCalibrationSucceeded ? "true" : "false")
         << ",\"esp32UptimeMs\":" << esp32.esp32UptimeMs
         << ",\"raspberryCommandTimeoutMs\":" << config::kCommandTimeoutMs
         << ",\"esp32MotorCommandTimeoutMs\":" << config::kEsp32MotorCommandTimeoutMs
         << ",\"motorStartMinimumPower\":" << config::kMotorStartMinimumPower
         << ",\"motorRunMinimumPower\":" << config::kMotorRunMinimumPower
         << ",\"motorRunConfirmationRequiredSamples\":"
         << config::kMotorRunConfirmationSamples
         << ",\"operationalMaximumReferencePower\":" << config::kOperationalMaximumReferencePower
         << ",\"encoderSyncWarmupSamples\":" << config::kEncoderSyncWarmupSamples
         << ",\"encoderCountsPerCentimeter\":" << config::kEncoderCountsPerCentimeter
         << "}";

    return json.str();
}

std::string DashboardServer::dashboardHtml()
{
  return R"HTML(<!doctype html>
<html lang="pt-BR">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta name="theme-color" content="#000000">
  <link rel="icon" type="image/png" sizes="128x128" href="/soul-sync-favicon.png?v=1">
  <title>Soul Sync · OBR 2026</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      --bg-page: #000000;
      --bg-primary: #0d0d0d;
      --bg-card: #151515;
      --bg-card-hover: #1c1c1c;
      --bg-control: #1f1f1f;
      --bg-overlay: #0d0d0de8;
      --border-primary: #2a2a2a;
      --border-subtle: #202020;
      --text-primary: #f2f2f2;
      --text-secondary: #b4b4b4;
      --text-muted: #777777;
      --text-technical: #8a8a8a;
      --text-disabled: #555555;
      --interactive-hover: #262626;
      --interactive-active: #303030;
      --focus-ring: #666666;
      --semantic-success: #22c55e;
      --semantic-warning: #eab308;
      --semantic-danger: #ef4444;
      --semantic-danger-hover: #c93636;
      --text-on-warning: #111827;
      --dashboard-scale: .8;
      --chart-primary: #f2f2f2;
      --chart-secondary: #b4b4b4;
      --chart-tertiary: #777777;
      --space-1: 4px;
      --space-2: 8px;
      --space-3: 12px;
      --space-4: 16px;
      --space-5: 20px;
      --space-6: 24px;
      --space-7: 32px;
      --page-max-width: 1420px;
      --page-inline-space: 18px;
      --page-block-space: var(--space-6);
      --layout-gap: var(--space-3);
      --card-padding: var(--space-4);
      --control-height: 44px;
      --button-height: 48px;
      --accordion-height: 40px;
      --bg: var(--bg-page);
      --surface: var(--bg-primary);
      --surface-2: var(--bg-card);
      --line: var(--border-primary);
      --line-soft: var(--border-subtle);
      --text: var(--text-primary);
      --muted: var(--text-technical);
      --yellow: var(--semantic-warning);
      --green: var(--semantic-success);
      --danger: var(--semantic-danger);
    }
    * { box-sizing: border-box; }
    body { margin: 0; min-height: 100vh; color: var(--text); background: var(--bg-page); zoom: var(--dashboard-scale); }
    button, input { font: inherit; }
    .shell { width: min(var(--page-max-width), calc(100% - var(--page-inline-space) - var(--page-inline-space))); margin: 0 auto; padding: var(--page-block-space) 0; display: grid; gap: var(--space-4); }
    .topbar { display: flex; align-items: center; justify-content: space-between; gap: var(--space-5); margin: 0; }
    .brand { display: flex; align-items: center; gap: var(--space-3); }
    .brand-mark { width: 52px; height: 52px; flex: 0 0 52px; }
    .brand-mark img { display: block; width: 100%; height: 100%; object-fit: contain; }
    .eyebrow { display: block; margin-bottom: var(--space-1); color: var(--text-secondary); font-size: .72rem; font-weight: 850; letter-spacing: .18em; text-transform: uppercase; }
    h1 { margin: 0; font-size: clamp(1.5rem, 3vw, 2.35rem); line-height: 1; letter-spacing: -.045em; }
    .status-cluster { display: flex; flex-wrap: wrap; justify-content: flex-end; gap: var(--space-2); }
    .status-pill { min-height: 37px; display: inline-flex; align-items: center; gap: var(--space-2); padding: var(--space-2) var(--space-3); border: 1px solid var(--line); border-radius: 999px; background: var(--bg-primary); color: var(--muted); font-size: .8rem; font-weight: 800; }
    .status-pill::before { content: ""; width: 7px; height: 7px; border-radius: 50%; background: var(--text-disabled); }
    .status-pill.ok { color: var(--green); border-color: var(--green); background: var(--bg-primary); }
    .status-pill.ok::before { background: var(--green); }
    .status-pill.warn { color: var(--yellow); border-color: var(--yellow); background: var(--bg-primary); }
    .status-pill.warn::before { background: var(--yellow); }
    .status-pill.danger { color: var(--danger); border-color: var(--danger); background: var(--bg-primary); }
    .status-pill.danger::before { background: var(--danger); }
    .mode-navigation { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: var(--space-1); margin: 0; padding: var(--space-1); border: 1px solid var(--line); border-radius: 14px; background: var(--bg-primary); box-shadow: none; }
    .mode-tab { min-height: var(--control-height); padding: var(--space-2) var(--space-4); border-color: transparent; background: transparent; color: var(--muted); font-size: .73rem; letter-spacing: .12em; text-transform: uppercase; }
    .mode-tab:hover { background: var(--bg-card-hover); }
    .mode-tab.active { color: var(--bg-primary); border-color: var(--text-primary); background: var(--text-primary); box-shadow: none; }
    .mode-tab:focus-visible { outline: 2px solid var(--focus-ring); outline-offset: 2px; }
    .dashboard-mode[hidden] { display: none; }
    .dashboard-mode { min-width: 0; display: grid; gap: var(--layout-gap); }
    .mode-heading { display: flex; align-items: end; justify-content: space-between; gap: var(--space-5); margin: 0; }
    .mode-heading h2 { margin: 0; font-size: 1.25rem; }
    .mode-heading p { max-width: 620px; margin: 0; color: var(--muted); font-size: .78rem; line-height: 1.45; text-align: right; }
    .hero-grid { display: grid; grid-template-columns: minmax(0, 1.05fr) minmax(0, 1.45fr) minmax(0, 1.15fr) minmax(0, 1.35fr); gap: var(--layout-gap); margin: 0; }
    .hero-grid > *, .main-grid > *, .telemetry-grid > *, .operation-cockpit > *, .diagnostic-tools > * { min-width: 0; }
    .card { position: relative; min-width: 0; border: 1px solid var(--line); border-radius: 16px; background: var(--bg-card); box-shadow: none; overflow: hidden; }
    .hero-card { min-height: 140px; padding: var(--card-padding); }
    .hero-card::after { display: none; }
    .card-label { margin: 0 0 var(--space-2); color: var(--muted); font-size: .72rem; font-weight: 850; letter-spacing: .13em; text-transform: uppercase; }
    .hero-value { margin: 0; font-size: clamp(1.8rem, 3.3vw, 2.9rem); line-height: 1; font-weight: 900; letter-spacing: -.05em; }
    .hero-value.yellow { color: var(--yellow); }
    .hero-value.status-good { color: var(--green); text-shadow: none; }
    .hero-value.status-warn { color: var(--yellow); text-shadow: none; }
    .hero-value.status-bad { color: var(--danger); text-shadow: none; }
    .hero-detail { margin: var(--space-2) 0 0; color: var(--muted); font-size: .82rem; }
    .battery-track { height: 8px; margin-top: var(--space-3); padding: 2px; border: 1px solid var(--border-primary); border-radius: 99px; background: var(--bg-control); }
    .battery-fill { width: 0; height: 100%; border-radius: inherit; background: var(--text-secondary); box-shadow: none; transition: width .25s ease; }
    .raspberry-card { display: flex; flex-direction: column; }
    .system-metrics { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: var(--space-2); }
    .system-metric { --metric: var(--text-secondary); --metric-surface: var(--bg-card); --metric-border: var(--border-primary); --metric-label: var(--text-muted); position: relative; min-width: 0; padding: var(--space-2); border: 1px solid var(--metric-border); border-radius: 11px; background: var(--metric-surface); box-shadow: none; overflow: hidden; transition: border-color .2s, background .2s; }
    .system-metric::after { display: none; }
    .system-metric.cpu, .system-metric.ram, .system-metric.temperature { --metric: var(--text-secondary); --metric-surface: var(--bg-card); --metric-border: var(--border-primary); --metric-label: var(--text-muted); }
    .system-metric[data-level="warn"] { --metric: var(--yellow); --metric-border: var(--yellow); --metric-label: var(--yellow); }
    .system-metric[data-level="danger"] { --metric: var(--danger); --metric-border: var(--danger); --metric-label: var(--danger); box-shadow: none; }
    .metric-header { position: relative; z-index: 1; display: flex; align-items: center; justify-content: space-between; gap: var(--space-1); color: var(--metric-label); font-size: .61rem; font-weight: 850; letter-spacing: .1em; text-transform: uppercase; }
    .metric-dot { width: 6px; height: 6px; flex: 0 0 auto; border-radius: 50%; background: var(--metric); box-shadow: none; }
    .system-metric strong { position: relative; z-index: 1; display: block; margin-top: var(--space-1); color: var(--text); font-size: clamp(1.05rem, 1.6vw, 1.4rem); line-height: 1; letter-spacing: -.035em; white-space: nowrap; font-variant-numeric: tabular-nums; }
    .metric-track { position: relative; z-index: 1; height: 4px; margin-top: var(--space-2); border-radius: 99px; background: var(--bg-control); overflow: hidden; }
    .metric-fill { width: 0; height: 100%; border-radius: inherit; background: var(--metric); box-shadow: none; transition: width .3s ease, background .2s; }
    .raspberry-role { display: flex; align-items: center; gap: var(--space-2); margin: var(--space-2) 0 0; color: var(--muted); font-size: .68rem; }
    .raspberry-role::before { content: ""; width: 5px; height: 5px; flex: 0 0 auto; border-radius: 50%; background: var(--text-muted); box-shadow: none; }
    .main-grid { display: grid; grid-template-columns: minmax(0, 1.65fr) minmax(360px, .85fr); gap: var(--layout-gap); margin: 0; align-items: start; }
    .side-stack { display: grid; gap: var(--layout-gap); min-width: 0; }
    .section-card { padding: var(--card-padding); }
    .section-header { display: flex; align-items: center; justify-content: space-between; gap: var(--space-3); margin: 0 0 var(--space-3); }
    .section-title { display: flex; align-items: center; gap: var(--space-2); margin: 0; font-size: .86rem; letter-spacing: .1em; text-transform: uppercase; }
    .section-title::before { content: ""; width: 3px; height: 16px; border-radius: 4px; background: var(--text-secondary); box-shadow: none; }
    .camera-workspace-header { align-items: flex-start; }
    .camera-heading { min-width: 150px; }
    .camera-heading p { margin: var(--space-2) 0 0 var(--space-3); color: var(--muted); font-size: .68rem; line-height: 1.35; }
    .camera-meta { display: flex; flex-wrap: wrap; justify-content: flex-end; gap: var(--space-2); }
    .meta-chip { padding: var(--space-1) var(--space-2); border: 1px solid var(--line); border-radius: 7px; background: var(--bg-primary); color: var(--muted); font-size: .7rem; }
    .meta-chip strong { color: var(--text); font-variant-numeric: tabular-nums; }
    .camera-view-selector { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: var(--space-1); margin: 0 0 var(--space-2); padding: var(--space-1); border: 1px solid var(--line); border-radius: 11px; background: var(--bg-primary); }
    .camera-view-button { min-height: 38px; padding: 6px 10px; border-color: transparent; border-radius: 8px; background: transparent; color: var(--muted); font-size: .68rem; letter-spacing: .09em; text-transform: uppercase; }
    .camera-view-button:hover { background: var(--bg-card-hover); }
    .camera-view-button.active { color: var(--text-primary); border-color: var(--border-primary); background: var(--interactive-active); box-shadow: none; }
    .camera-view-button:focus-visible { outline: 2px solid var(--focus-ring); outline-offset: 2px; }
    .dataset-capture-strip { display: flex; align-items: baseline; flex-wrap: wrap; gap: var(--space-1) var(--space-2); min-height: 30px; margin: 0 0 var(--space-2); padding: 6px var(--space-2); border: 1px solid var(--line-soft); border-radius: 8px; background: var(--bg-primary); color: var(--muted); }
    .dataset-capture-strip strong { color: var(--text-muted); font-size: .59rem; font-weight: 900; letter-spacing: .08em; text-transform: uppercase; }
    .dataset-capture-strip span { min-width: 0; font-size: .62rem; font-variant-numeric: tabular-nums; overflow-wrap: anywhere; }
    .dataset-capture-strip[data-state="active"] strong { color: var(--green); }
    .dataset-capture-strip[data-state="stale"] strong { color: var(--yellow); }
    .camera-feed-grid { display: grid; grid-template-columns: minmax(0, 1fr); gap: var(--space-2); }
    .camera-feed-grid[data-view="dual"] { grid-template-columns: minmax(0, 1fr); grid-template-rows: repeat(2, minmax(0, 1fr)); }
    .camera-feed-card { min-width: 0; padding: var(--space-2); border: 1px solid var(--line-soft); border-radius: 13px; background: var(--bg-primary); }
    .camera-feed-header { display: flex; align-items: center; justify-content: space-between; gap: var(--space-2); min-height: var(--space-7); padding: 0 0 var(--space-2); }
    .camera-feed-identity { display: flex; align-items: center; flex-wrap: wrap; gap: var(--space-2); min-width: 0; }
    .camera-feed-header strong { font-size: .72rem; letter-spacing: .06em; text-transform: uppercase; }
    .camera-mode-selector { display: inline-flex; gap: 2px; padding: 2px; border: 1px solid var(--line); border-radius: 7px; background: var(--bg-primary); }
    .camera-mode-button { min-height: 24px; padding: 3px 7px; border: 0; border-radius: 5px; background: transparent; color: var(--muted); font-size: .56rem; font-weight: 850; letter-spacing: .06em; }
    .camera-mode-button:hover { color: var(--text); background: var(--interactive-hover); }
    .camera-mode-button.active { color: var(--text-primary); background: var(--interactive-active); box-shadow: none; }
    .camera-mode-button:focus-visible { outline: 2px solid var(--focus-ring); outline-offset: 1px; }
    .camera-status { display: inline-flex; align-items: center; gap: 6px; color: var(--muted); font-size: .59rem; font-weight: 900; letter-spacing: .07em; text-transform: uppercase; }
    .camera-status::before { content: ""; width: 6px; height: 6px; border-radius: 50%; background: var(--text-disabled); box-shadow: none; }
    .camera-status.online { color: var(--green); }
    .camera-status.online::before { background: var(--green); box-shadow: none; }
    .camera-status.loading { color: var(--yellow); }
    .camera-status.loading::before { background: var(--yellow); box-shadow: none; }
    .camera-status.offline { color: var(--danger); }
    .camera-status.offline::before { background: var(--danger); box-shadow: none; }
    .camera-status.disabled, .camera-status.unconfigured { color: var(--text-muted); }
    .camera-status.disabled::before { background: var(--text-disabled); box-shadow: none; }
    .camera-feed-actions { display: flex; align-items: center; justify-content: flex-end; gap: 8px; }
    .camera-power-button { min-height: 28px; padding: 4px 8px; border-radius: 7px; color: var(--text-secondary); background: var(--bg-control); font-size: .57rem; letter-spacing: .07em; }
    .camera-power-button.active { color: var(--green); border-color: var(--green); background: var(--bg-control); }
    .camera-frame { position: relative; aspect-ratio: 4 / 3; border: 1px solid var(--border-primary); border-radius: 11px; overflow: hidden; background: var(--bg-page); }
    .camera-feed-card[data-camera-id="forward"] .camera-frame { aspect-ratio: 16 / 9; }
    .camera-frame img { display: block; width: 100%; height: 100%; object-fit: contain; }
    .camera-frame.offline img { opacity: 0; }
    .camera-message { position: absolute; inset: 0; display: grid; place-items: center; color: var(--muted); text-align: center; padding: var(--space-5); }
    .camera-frame.online .camera-message { display: none; }
    .camera-placeholder { display: grid; align-content: center; justify-items: center; gap: var(--space-2); width: 100%; height: 100%; padding: var(--space-6); text-align: center; }
    .camera-placeholder strong { color: var(--text); font-size: .86rem; letter-spacing: .11em; }
    .camera-placeholder p { margin: 0; color: var(--muted); font-size: .72rem; line-height: 1.45; }
    .camera-placeholder .planned-use { margin-top: var(--space-2); color: var(--text-secondary); font-size: .62rem; font-weight: 850; letter-spacing: .08em; text-transform: uppercase; }
    .camera-telemetry-label { margin: var(--space-3) 0 0; color: var(--text-secondary); font-size: .61rem; font-weight: 850; letter-spacing: .1em; text-transform: uppercase; }
    .camera-diagnostics { min-width: 0; display: grid; gap: var(--layout-gap); }
    .camera-diagnostic-list { display: grid; gap: var(--space-1); margin: 0; }
    .camera-diagnostic-item { display: flex; align-items: baseline; justify-content: space-between; gap: var(--space-2); min-width: 0; }
    .camera-diagnostic-item dt { color: var(--muted); font-size: .62rem; }
    .camera-diagnostic-item dd { margin: 0; color: var(--text); font-size: .69rem; font-weight: 800; font-variant-numeric: tabular-nums; overflow-wrap: anywhere; }
    .diagnostic-tools { display: grid; grid-template-columns: minmax(0, 1fr); gap: var(--layout-gap); margin: 0; align-items: start; }
    .diagnostic-oled-card { padding: 0; }
    .diagnostic-oled-card .oled-editor { border: 0; border-radius: 0; background: transparent; }
    .diagnostic-communication-alert { display: grid; grid-template-columns: minmax(190px, .7fr) minmax(0, 1.3fr); align-items: center; gap: var(--space-5); margin: 0; padding: var(--space-3) var(--card-padding); border: 1px solid var(--danger); border-radius: 12px; background: var(--bg-card); }
    .diagnostic-communication-alert[hidden] { display: none; }
    .diagnostic-communication-alert strong { display: block; color: var(--danger); font-size: 1rem; letter-spacing: .08em; }
    .diagnostic-communication-alert span { display: block; margin-top: var(--space-1); color: var(--danger); font-size: .72rem; }
    .diagnostic-communication-alert p { margin: 0; color: var(--text-secondary); font-size: .72rem; line-height: 1.45; }
    .diagnostic-subsystem.communication { grid-column: 1 / -1; min-height: 0; }
    .diagnostic-subsystem .section-header { align-items: center; }
    .subsystem-status { flex: 0 0 auto; padding: 4px 7px; border: 1px solid var(--border-primary); border-radius: 999px; color: var(--text-secondary); background: var(--bg-primary); font-size: .56rem; font-weight: 900; letter-spacing: .07em; text-transform: uppercase; }
    .subsystem-status.ok { color: var(--green); border-color: var(--green); background: var(--bg-primary); }
    .subsystem-status.warn { color: var(--yellow); border-color: var(--yellow); background: var(--bg-primary); }
    .subsystem-status.danger { color: var(--danger); border-color: var(--danger); background: var(--bg-primary); }
    .diagnostic-subsystem[data-telemetry="missing"] { border-color: var(--border-subtle); }
    .diagnostic-subsystem[data-telemetry="missing"] .big-reading { border-color: var(--border-subtle); background: var(--bg-primary); }
    .diagnostic-subsystem[data-telemetry="missing"] .big-reading strong,
    .diagnostic-subsystem[data-telemetry="missing"] .telemetry-row strong { color: var(--text-muted); }
    .diagnostic-maintenance-heading { margin: var(--space-3) 0 0; padding-top: var(--space-4); border-top: 1px solid var(--line-soft); }
    .operation-overview { display: grid; grid-template-columns: minmax(0, 1.15fr) minmax(0, .85fr) minmax(0, .75fr) minmax(0, 2.15fr) minmax(0, .85fr); align-items: stretch; margin: 0; border: 1px solid var(--line); border-radius: 14px; background: var(--bg-card); box-shadow: none; overflow: hidden; }
    .operation-overview > div { min-width: 0; display: flex; flex-direction: column; justify-content: center; gap: var(--space-1); padding: var(--space-3); border-right: 1px solid var(--line-soft); }
    .operation-overview > div:last-child { border-right: 0; }
    .operation-overview span, .operation-overview-label { color: var(--muted); font-size: .58rem; font-weight: 850; letter-spacing: .1em; text-transform: uppercase; }
    .operation-overview strong { color: var(--text); font-size: 1.03rem; line-height: 1.05; font-variant-numeric: tabular-nums; white-space: nowrap; }
    .operation-overview-mode small { color: var(--muted); font-size: .62rem; line-height: 1.25; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
    .operation-overview .cockpit-mode-value { color: var(--text-primary); font-size: 1.42rem; letter-spacing: -.035em; }
    .cockpit-mode-value[data-mode="manual"], .cockpit-mode-value[data-mode="autonomous"] { color: var(--text-primary); }
    .cockpit-mode-value[data-mode="emergency"] { color: var(--danger); }
    .operation-overview-item.battery strong { color: var(--text-primary); }
    .operation-overview strong.status-good { color: var(--green); }
    .operation-overview strong.status-warn { color: var(--yellow); }
    .operation-overview strong.status-bad { color: var(--danger); }
    .operation-meter { height: 3px; margin-top: var(--space-1); border-radius: 99px; background: var(--bg-control); overflow: hidden; }
    .operation-meter > div { width: 0; height: 100%; border-radius: inherit; background: var(--text-secondary); box-shadow: none; transition: width .25s ease; }
    .operation-raspberry { gap: var(--space-2) !important; }
    .operation-system-metrics { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: var(--space-2); }
    .operation-system-metric { min-width: 0; display: grid; grid-template-columns: auto 1fr; align-items: baseline; gap: var(--space-1); }
    .operation-system-metric span { font-size: .53rem; }
    .operation-system-metric strong { font-size: .82rem; text-align: right; }
    .operation-system-metric > i { grid-column: 1 / -1; height: 2px; border-radius: 99px; background: var(--bg-control); overflow: hidden; }
    .operation-system-metric > i > i { display: block; width: 0; height: 100%; background: var(--text-secondary); transition: width .25s ease; }
    .operation-system-metric[data-level="warn"] > i > i { background: var(--yellow); }
    .operation-system-metric[data-level="danger"] > i > i { background: var(--danger); }
    .operation-cockpit { display: grid; grid-template-columns: minmax(0, 2fr) minmax(360px, 1fr); gap: var(--layout-gap); align-items: stretch; }
    .operation-camera-panel { align-self: start; width: 100%; padding: var(--card-padding); }
    .operation-camera-header, .operation-control-header { display: flex; align-items: flex-start; justify-content: space-between; gap: var(--space-3); margin: 0 0 var(--space-3); }
    .operation-camera-header h2, .operation-control-header h2 { margin: 0; font-size: 1.08rem; letter-spacing: -.02em; }
    .operation-camera-header .eyebrow, .operation-control-header .eyebrow { margin-bottom: var(--space-1); font-size: .58rem; }
    .operation-camera-panel .camera-meta { align-self: center; }
    .operation-camera-panel .meta-chip { padding: 2px 5px; border: 0; background: transparent; font-size: .61rem; }
    .operation-camera-panel .camera-view-selector { width: min(330px, 100%); margin: 0 0 var(--space-2); padding: var(--space-1); }
    .operation-camera-panel .camera-view-button { min-height: 31px; }
    .operation-camera-panel .camera-feed-grid { aspect-ratio: 4 / 3; grid-auto-rows: minmax(0, 1fr); align-items: stretch; }
    .operation-camera-panel .camera-feed-card { display: grid; grid-template-rows: auto minmax(0, 1fr); min-height: 0; height: 100%; padding: 0; border: 0; background: var(--bg-primary); overflow: hidden; }
    .operation-camera-panel .camera-feed-header { padding: var(--space-2); }
    .operation-camera-panel .camera-frame,
    .operation-camera-panel .camera-feed-card[data-camera-id="forward"] .camera-frame { min-height: 0; height: 100%; aspect-ratio: auto; border: 0; border-radius: 8px; background: var(--bg-page); }
    .camera-details { margin: 0 0 var(--space-2); border-top: 1px solid var(--line-soft); border-bottom: 1px solid var(--line-soft); }
    .camera-details summary { display: flex; align-items: center; justify-content: space-between; min-height: var(--accordion-height); color: var(--muted); font-size: .58rem; font-weight: 850; letter-spacing: .08em; text-transform: uppercase; cursor: pointer; list-style: none; }
    .camera-details summary::-webkit-details-marker { display: none; }
    .camera-details summary::after { content: "+"; color: var(--text-secondary); font-size: .9rem; }
    .camera-details[open] summary::after { content: "−"; }
    .camera-technical-metadata { display: flex; flex-wrap: wrap; gap: var(--space-1) var(--space-3); padding: 0 0 var(--space-3); }
    .camera-technical-metadata .meta-chip { padding: 0; border: 0; background: transparent; font-size: .59rem; }
    .camera-hud { position: absolute; inset: var(--space-2); z-index: 3; display: flex; flex-direction: column; justify-content: space-between; gap: var(--space-2); pointer-events: none; }
    .camera-hud[hidden], .camera-frame:not(.online) .camera-hud { display: none; }
    .camera-hud-header { display: flex; flex-direction: column; align-items: flex-end; justify-content: flex-start; gap: var(--space-1); }
    .camera-hud-header > * { padding: 5px 8px; border: 1px solid var(--border-primary); border-radius: 6px; color: var(--text); background: var(--bg-overlay); font-size: .62rem; font-weight: 900; letter-spacing: .07em; }
    .camera-hud-line.valid { color: var(--green); border-color: var(--green); }
    .camera-hud-line.invalid { color: var(--danger); border-color: var(--danger); }
    .camera-hud-line.searching { color: var(--yellow); border-color: var(--yellow); }
    .camera-hud-values { display: grid; grid-template-columns: .8fr .8fr repeat(3, 1fr) 1.45fr; border: 1px solid var(--border-primary); border-radius: 7px; background: var(--bg-overlay); overflow: hidden; }
    .camera-hud-value { min-width: 0; display: flex; flex-direction: column; gap: 2px; padding: 7px 9px; border-right: 1px solid var(--border-subtle); }
    .camera-hud-value:last-child { border-right: 0; }
    .camera-hud-value > span { color: var(--text-muted); font-size: .5rem; font-weight: 850; letter-spacing: .08em; text-transform: uppercase; }
    .camera-hud-value strong { color: var(--text); font-size: .76rem; line-height: 1.05; font-variant-numeric: tabular-nums; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
    .camera-hud-value small { color: var(--muted); font-size: .5rem; }
    .camera-hud-value.pipeline { grid-column: span 2; }
    .camera-hud-value.green-status strong,
    .camera-hud-value.path-status strong { color: var(--text-primary); }
    .camera-hud-value.speed strong { color: var(--text-primary); font-size: .67rem; }
    .camera-message { align-content: center; justify-items: center; gap: 6px; }
    .camera-message strong { color: var(--text); font-size: .78rem; letter-spacing: .08em; text-transform: uppercase; }
    .camera-message span { max-width: 320px; color: var(--muted); font-size: .66rem; line-height: 1.4; }
    .camera-frame.loading .camera-message strong { color: var(--yellow); }
    .camera-frame.offline .camera-message strong { color: var(--danger); }
    .operation-control-panel { align-self: stretch; min-height: 0; display: flex; flex-direction: column; padding: var(--card-padding); overflow: hidden; contain: size; }
    .operation-control-main { min-width: 0; flex: 0 0 auto; display: grid; gap: var(--layout-gap); }
    .operation-control-header { align-items: center; margin-bottom: 0; }
    .operation-control-header .machine-badge { max-width: 48%; }
    .operation-mission-summary, .operation-drive-summary { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: var(--space-3); padding: var(--space-3) 0; border-top: 1px solid var(--line-soft); border-bottom: 1px solid var(--line-soft); }
    .operation-mission-summary span, .operation-drive-summary span { display: block; color: var(--muted); font-size: .57rem; font-weight: 800; letter-spacing: .08em; text-transform: uppercase; }
    .operation-mission-summary strong, .operation-drive-summary strong { display: block; margin-top: 5px; color: var(--text); font-size: .9rem; line-height: 1.2; font-variant-numeric: tabular-nums; overflow-wrap: anywhere; }
    .operation-drive-summary strong { font-size: 1.05rem; }
    .operation-drive-summary strong.accent { color: var(--text-primary); }
    .operation-machine-action { min-height: 0; padding: 2px 0; border: 0; background: transparent; }
    .operation-machine-action strong { margin-top: 5px; font-size: 1.13rem; }
    .operation-progress { margin: 0; }
    .operation-mission-selector { padding: 0; border: 0; background: transparent; box-shadow: none; }
    .operation-mission-selector > label::after { display: none; }
    .operation-mission-selector select { min-height: var(--control-height); }
    .operation-mode-buttons { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    .operation-mode-buttons button { min-height: 52px; }
    .operation-accordions { min-width: 0; min-height: 0; flex: 0 1 auto; margin-top: auto; border-top: 1px solid var(--line-soft); overflow-x: hidden; overflow-y: auto; overscroll-behavior: contain; }
    .cockpit-details { border: 0; }
    .cockpit-details + .cockpit-details { border-top: 1px solid var(--line-soft); }
    .cockpit-details summary { display: flex; align-items: center; justify-content: space-between; min-height: var(--accordion-height); color: var(--muted); font-size: .63rem; font-weight: 850; letter-spacing: .08em; text-transform: uppercase; cursor: pointer; list-style: none; }
    .cockpit-details summary::-webkit-details-marker { display: none; }
    .cockpit-details summary::after { content: "+"; color: var(--text-secondary); font-size: 1rem; }
    .cockpit-details[open] summary::after { content: "−"; }
    .cockpit-details-body { display: grid; gap: var(--space-2); padding: 0 0 var(--space-3); }
    .operation-detail-metrics { margin-top: 0; }
    .manual-control-body .keyboard-panel, .manual-control-body .drive-control { border-color: var(--border-subtle); }
    .mission-state-card { --state-color: var(--text-secondary); background: var(--bg-card); }
    .mission-state-card[data-tone="active"] { --state-color: var(--text-primary); }
    .mission-state-card[data-tone="warn"] { --state-color: var(--yellow); }
    .mission-state-card[data-tone="danger"] { --state-color: var(--danger); }
    .mission-state-card .section-title::before { background: var(--state-color); box-shadow: none; }
    .machine-badge { display: inline-flex; align-items: center; gap: var(--space-2); max-width: 52%; padding: var(--space-2); border: 1px solid var(--state-color); border-radius: 999px; color: var(--state-color); background: var(--bg-primary); font-size: .62rem; font-weight: 900; letter-spacing: .08em; text-transform: uppercase; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
    .machine-badge::before { content: ""; width: 7px; height: 7px; flex: 0 0 auto; border-radius: 50%; background: var(--state-color); box-shadow: none; }
    .machine-action { min-height: 72px; padding: var(--space-3); border: 1px solid var(--line); border-radius: 12px; background: var(--bg-primary); }
    .machine-action span { display: block; color: var(--muted); font-size: .62rem; font-weight: 850; letter-spacing: .11em; text-transform: uppercase; }
    .machine-action strong { display: block; margin-top: 7px; color: var(--text); font-size: 1.05rem; line-height: 1.25; }
    .machine-progress { margin: var(--space-3) 0; }
    .machine-progress-label { display: flex; justify-content: space-between; gap: 10px; margin-bottom: 6px; color: var(--muted); font-size: .65rem; }
    .machine-progress-label strong { color: var(--state-color); font-variant-numeric: tabular-nums; }
    .machine-progress-track { height: 6px; border-radius: 99px; background: var(--bg-control); overflow: hidden; }
    .machine-progress-fill { width: 0; height: 100%; border-radius: inherit; background: var(--state-color); box-shadow: none; transition: width .18s ease; }
    .machine-flow { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: var(--space-1); margin: 0 0 var(--space-3); }
    .machine-step { position: relative; min-width: 0; padding: 8px 3px; border: 1px solid var(--line-soft); border-radius: 8px; color: var(--text-muted); background: var(--bg-primary); text-align: center; font-size: .55rem; font-weight: 850; letter-spacing: .055em; text-transform: uppercase; transition: .2s; }
    .machine-step.active { color: var(--bg-primary); border-color: var(--state-color); background: var(--state-color); box-shadow: none; }
    .machine-metrics { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: var(--space-2); }
    .machine-metric { min-width: 0; padding: var(--space-2); border: 1px solid var(--line-soft); border-radius: 9px; background: var(--bg-primary); }
    .machine-metric span { display: block; color: var(--muted); font-size: .58rem; letter-spacing: .06em; text-transform: uppercase; }
    .machine-metric strong { display: block; margin-top: 5px; color: var(--text); font-size: .82rem; font-variant-numeric: tabular-nums; overflow-wrap: anywhere; }
    .machine-metric strong.accent { color: var(--state-color); }
    .command-card { display: flex; flex-direction: column; gap: var(--layout-gap); }
    .command-card .requested-drive { grid-template-columns: minmax(0, 1fr); }
    .mission-selector { padding: var(--space-3); border: 1px solid var(--border-primary); border-radius: 12px; background: var(--bg-primary); box-shadow: none; }
    .mission-selector > label { display: flex; align-items: center; justify-content: space-between; gap: var(--space-2); margin: 0 0 var(--space-2); color: var(--text-secondary); font-size: .68rem; font-weight: 850; letter-spacing: .1em; text-transform: uppercase; }
    .mission-selector > label::after { content: "PADRÃO: PRINCIPAL"; padding: 3px 6px; border: 1px solid var(--border-primary); border-radius: 99px; color: var(--muted); background: var(--bg-primary); font-size: .55rem; letter-spacing: .06em; }
    .mission-selector select { width: 100%; min-height: var(--control-height); padding: 0 var(--space-7) 0 var(--space-3); border: 1px solid var(--border-primary); border-radius: 9px; outline: none; color: var(--text); background: var(--bg-control); font: inherit; font-size: .78rem; font-weight: 850; letter-spacing: .035em; cursor: pointer; }
    .mission-selector select:focus { border-color: var(--focus-ring); box-shadow: 0 0 0 2px var(--focus-ring); }
    .mission-selector option { color: var(--text); background: var(--bg-control); }
    .mission-hint { display: block; margin-top: var(--space-2); color: var(--muted); font-size: .66rem; line-height: 1.35; }
    .oled-editor { border: 1px solid var(--border-primary); border-radius: 12px; background: var(--bg-primary); overflow: hidden; }
    .oled-editor summary { display: flex; align-items: center; justify-content: space-between; gap: var(--space-2); min-height: var(--accordion-height); padding: var(--space-3); color: var(--text-secondary); font-size: .7rem; font-weight: 900; letter-spacing: .09em; text-transform: uppercase; cursor: pointer; list-style: none; }
    .oled-editor summary::-webkit-details-marker { display: none; }
    .oled-editor summary::after { content: "+"; color: var(--text-secondary); font-size: 1.1rem; line-height: 1; }
    .oled-editor[open] summary::after { content: "−"; }
    .oled-summary-state { margin-left: auto; padding: 3px 6px; border: 1px solid var(--border-primary); border-radius: 99px; color: var(--muted); background: var(--bg-primary); font-size: .54rem; letter-spacing: .06em; }
    .oled-editor-body { display: grid; gap: var(--space-3); padding: 0 var(--space-3) var(--space-3); border-top: 1px solid var(--line-soft); }
    .oled-live-state { display: flex; align-items: center; justify-content: space-between; gap: var(--space-4); margin-top: var(--space-3); padding: var(--space-3); border: 1px solid var(--line-soft); border-radius: 10px; background: var(--bg-primary); }
    .oled-live-state div { flex: 0 0 auto; }
    .oled-live-state span { display: block; color: var(--muted); font-size: .57rem; font-weight: 800; letter-spacing: .08em; text-transform: uppercase; }
    .oled-live-state strong { display: block; margin-top: 4px; color: var(--text); font-size: .82rem; letter-spacing: .04em; }
    .oled-editor-state { margin: 0; color: var(--muted); font-size: .65rem; line-height: 1.4; text-align: right; }
    .oled-section { display: grid; gap: var(--space-2); padding-top: var(--space-3); border-top: 1px solid var(--line-soft); }
    .oled-section-heading { display: flex; align-items: end; justify-content: space-between; gap: var(--space-3); }
    .oled-section-heading strong { color: var(--text-secondary); font-size: .68rem; letter-spacing: .08em; text-transform: uppercase; }
    .oled-section-heading span { color: var(--muted); font-size: .61rem; line-height: 1.35; text-align: right; }
    .oled-preset-grid { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: var(--space-2); }
    .oled-preset-grid.mission-states { grid-template-columns: repeat(5, minmax(0, 1fr)); }
    .oled-preset { min-height: 58px; display: grid; align-content: center; gap: 3px; padding: var(--space-2); border-color: var(--border-primary); background: var(--bg-control); color: var(--text-primary); text-align: left; }
    .oled-preset:hover:not(:disabled) { border-color: var(--focus-ring); background: var(--interactive-hover); }
    .oled-preset strong { font-size: .72rem; letter-spacing: .06em; }
    .oled-preset span { color: var(--muted); font-size: .58rem; font-weight: 750; line-height: 1.25; }
    .oled-layout-selector { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 2px; padding: 2px; border: 1px solid var(--border-primary); border-radius: 9px; background: var(--bg-primary); }
    .oled-layout-button { min-height: 34px; border: 0; background: transparent; color: var(--muted); font-size: .66rem; }
    .oled-layout-button.active { color: var(--bg-primary); background: var(--text-primary); }
    .oled-fields { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: var(--space-2); }
    .oled-fields[hidden] { display: none; }
    .oled-field { min-width: 0; }
    .oled-field.wide { grid-column: 1 / -1; }
    .oled-field label { display: block; margin-bottom: var(--space-1); color: var(--muted); font-size: .59rem; font-weight: 800; letter-spacing: .07em; text-transform: uppercase; }
    .oled-field input { width: 100%; min-width: 0; min-height: var(--control-height); padding: var(--space-2); border: 1px solid var(--border-primary); border-radius: 8px; outline: none; color: var(--text); background: var(--bg-control); font: inherit; font-size: .78rem; }
    .oled-field input:focus { border-color: var(--focus-ring); box-shadow: 0 0 0 2px var(--focus-ring); }
    .oled-layout-hint { margin: 0; color: var(--muted); font-size: .62rem; line-height: 1.4; }
    .oled-actions { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: var(--space-2); }
    .oled-actions button { min-height: var(--control-height); font-size: .72rem; }
    .oled-actions .oled-clear { border-color: var(--border-primary); background: var(--bg-control); color: var(--text-primary); }
    .distance-mission-settings { margin: var(--space-2) 0 0; padding: var(--space-2); border: 1px solid var(--border-primary); border-radius: 9px; background: var(--bg-primary); }
    .distance-mission-settings[hidden] { display: none; }
    .distance-input-label { display: block; margin-bottom: var(--space-2); color: var(--muted); font-size: .62rem; font-weight: 800; letter-spacing: .07em; text-transform: uppercase; }
    .distance-input { display: grid; grid-template-columns: minmax(0, 1fr) auto; align-items: center; gap: var(--space-2); }
    .distance-input input { width: 100%; min-width: 0; min-height: var(--control-height); padding: var(--space-2); border: 1px solid var(--border-primary); border-radius: 8px; outline: none; color: var(--text); background: var(--bg-control); font: inherit; font-size: 1rem; font-weight: 900; font-variant-numeric: tabular-nums; }
    .distance-input input:focus { border-color: var(--focus-ring); box-shadow: 0 0 0 2px var(--focus-ring); }
    .distance-input strong { color: var(--text-primary); font-size: .75rem; }
    .distance-calibration { display: block; margin-top: var(--space-2); color: var(--text-muted); font-size: .59rem; line-height: 1.35; }
    .mode-buttons { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: var(--space-2); }
    button { min-height: var(--button-height); border: 1px solid var(--border-primary); border-radius: 10px; color: var(--text-primary); background: var(--bg-control); font-weight: 850; cursor: pointer; transition: transform .1s, border-color .1s, background .1s; }
    button:hover { transform: translateY(-1px); border-color: var(--focus-ring); background: var(--interactive-hover); }
    button:focus-visible { outline: 2px solid var(--focus-ring); outline-offset: 2px; }
    button.active { color: var(--text-primary); border-color: var(--focus-ring); background: var(--interactive-active); }
    button.warning { border-color: var(--yellow); background: var(--yellow); color: var(--text-on-warning); }
    button.warning:hover { border-color: var(--yellow); background: var(--yellow); }
    button.danger { border-color: var(--danger); background: var(--danger); color: var(--text-primary); }
    button.danger:hover { border-color: var(--danger); background: var(--semantic-danger-hover); }
    button:disabled { cursor: not-allowed; opacity: .48; transform: none; }
    button.warning:disabled { opacity: 1; }
    .drive-control { padding: var(--space-3); border: 1px solid var(--line-soft); border-radius: 12px; background: var(--bg-primary); }
    .drive-control label { display: flex; justify-content: space-between; gap: var(--space-3); margin: 0 0 var(--space-2); color: var(--muted); font-size: .76rem; font-weight: 750; text-transform: uppercase; letter-spacing: .07em; }
    .drive-control output { color: var(--text); font-variant-numeric: tabular-nums; }
    input[type="range"] { width: 100%; accent-color: var(--text-primary); }
    .manual-speed-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: var(--space-2); }
    .manual-speed-grid .drive-control { border-color: var(--border-primary); background: var(--bg-primary); }
    .manual-test-note { grid-column: 1 / -1; margin: 0; color: var(--muted); font-size: .72rem; line-height: 1.45; }
    .servo-control-card { display: grid; gap: var(--space-3); }
    .servo-control-grid { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: var(--space-2); }
    .servo-control { min-width: 0; padding: var(--space-3); border: 1px solid var(--border-primary); border-radius: 12px; background: var(--bg-primary); }
    .servo-control label { display: flex; align-items: center; justify-content: space-between; gap: var(--space-2); color: var(--text-secondary); font-size: .72rem; font-weight: 850; letter-spacing: .07em; text-transform: uppercase; }
    .servo-control label output { color: var(--text); font-size: .9rem; font-variant-numeric: tabular-nums; }
    .servo-control input[type="range"] { margin: var(--space-3) 0; }
    .servo-angle-entry { display: grid; grid-template-columns: minmax(0, 1fr) auto; align-items: center; gap: var(--space-2); }
    .servo-angle-entry input { width: 100%; min-width: 0; min-height: var(--control-height); padding: 0 var(--space-2); border: 1px solid var(--border-primary); border-radius: 8px; outline: none; color: var(--text); background: var(--bg-control); font: inherit; font-variant-numeric: tabular-nums; }
    .servo-angle-entry input:focus { border-color: var(--focus-ring); box-shadow: 0 0 0 2px var(--focus-ring); }
    .servo-angle-entry span { color: var(--muted); font-size: .8rem; font-weight: 800; }
    .servo-output-state { display: block; min-height: 1.2em; margin-top: var(--space-2); color: var(--muted); font-size: .66rem; line-height: 1.35; }
    .servo-actions { display: flex; align-items: center; justify-content: space-between; gap: var(--space-3); }
    .servo-actions p { margin: 0; color: var(--muted); font-size: .7rem; line-height: 1.4; }
    .servo-actions button { flex: 0 0 auto; min-height: var(--control-height); }
    .servo-calibration { border-top: 1px solid var(--line-soft); }
    .servo-calibration summary { display: flex; align-items: center; justify-content: space-between; gap: var(--space-3); padding-top: var(--space-3); color: var(--text-secondary); cursor: pointer; font-size: .72rem; font-weight: 850; letter-spacing: .07em; text-transform: uppercase; }
    .servo-calibration-body { display: grid; gap: var(--space-3); padding-top: var(--space-3); }
    .servo-calibration-warning { margin: 0; padding: var(--space-3); border: 1px solid var(--yellow); border-radius: 10px; color: var(--text-secondary); background: var(--bg-primary); font-size: .7rem; line-height: 1.45; }
    .servo-calibration-toolbar { display: flex; flex-wrap: wrap; align-items: end; gap: var(--space-2); }
    .servo-calibration-toolbar label, .servo-calibration-endpoint label { display: grid; gap: var(--space-1); color: var(--muted); font-size: .66rem; font-weight: 800; }
    .servo-calibration-toolbar select, .servo-calibration-endpoint input { min-height: var(--control-height); padding: 0 var(--space-2); border: 1px solid var(--border-primary); border-radius: 8px; color: var(--text); background: var(--bg-control); }
    .servo-calibration-workspace { display: grid; gap: var(--space-3); padding: var(--space-3); border: 1px solid var(--border-primary); border-radius: 12px; background: var(--bg-primary); }
    .servo-calibration-workspace[hidden] { display: none; }
    .servo-calibration-readout { display: flex; align-items: baseline; justify-content: space-between; gap: var(--space-2); }
    .servo-calibration-readout strong { font-size: 1.25rem; font-variant-numeric: tabular-nums; }
    .servo-calibration-adjustments { display: grid; grid-template-columns: repeat(4, auto) minmax(90px, 1fr) auto; gap: var(--space-1); }
    .servo-calibration-adjustments input { width: 100%; min-width: 0; min-height: var(--control-height); padding: 0 var(--space-2); border: 1px solid var(--border-primary); border-radius: 8px; color: var(--text); background: var(--bg-control); text-align: center; font-variant-numeric: tabular-nums; }
    .servo-calibration-endpoints { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: var(--space-2); }
    .servo-calibration-endpoint { display: grid; grid-template-columns: minmax(0, 1fr) auto; align-items: end; gap: var(--space-1); }
    .servo-calibration-endpoint label { min-width: 0; }
    .servo-calibration-actions { display: flex; flex-wrap: wrap; gap: var(--space-2); }
    .servo-calibration-actions button { min-height: var(--control-height); }
    .servo-calibration-note { margin: 0; color: var(--muted); font-size: .67rem; line-height: 1.4; }
    .requested-drive { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: var(--space-2); }
    .request-value { padding: var(--space-3); border: 1px solid var(--line-soft); border-radius: 10px; background: var(--bg-primary); }
    .request-value span { display: block; color: var(--muted); font-size: .67rem; text-transform: uppercase; }
    .request-adjustment { display: grid; grid-template-columns: 34px minmax(0, 1fr) 34px; gap: var(--space-1); margin-top: var(--space-1); }
    .request-value input { width: 100%; min-width: 0; padding: 5px 4px; border: 1px solid var(--border-primary); border-radius: 7px; outline: none; color: var(--text); background: var(--bg-control); font: inherit; font-size: 1.12rem; font-weight: 900; text-align: center; font-variant-numeric: tabular-nums; }
    .request-value input:focus { border-color: var(--focus-ring); box-shadow: 0 0 0 2px var(--focus-ring); }
    .trim-button { min-height: 34px; padding: 0; border-radius: 7px; font-size: .74rem; }
    .keyboard-panel { display: flex; align-items: center; justify-content: space-between; gap: var(--space-3); padding: var(--space-3); border: 1px solid var(--line-soft); border-radius: 12px; background: var(--bg-primary); }
    .keyboard-copy strong { display: block; font-size: .78rem; letter-spacing: .06em; text-transform: uppercase; }
    .keyboard-copy span { display: block; max-width: 190px; margin-top: 4px; color: var(--muted); font-size: .7rem; line-height: 1.35; }
    .keys { flex: 0 0 auto; display: grid; grid-template-columns: repeat(3, 30px); grid-template-rows: repeat(2, 30px); gap: 4px; }
    .keycap { display: grid; place-items: center; border: 1px solid var(--border-primary); border-radius: 6px; background: var(--bg-control); color: var(--text-secondary); font-size: .72rem; font-weight: 900; box-shadow: none; }
    .keycap.w { grid-column: 2; }
    .keycap.a { grid-column: 1; grid-row: 2; }
    .keycap.s { grid-column: 2; grid-row: 2; }
    .keycap.d { grid-column: 3; grid-row: 2; }
    .keycap.active { border-color: var(--focus-ring); background: var(--interactive-active); color: var(--text-primary); box-shadow: none; }
    .safety-note { margin-top: auto; padding: var(--space-3); border-left: 3px solid var(--yellow); border-radius: 7px; background: var(--bg-primary); color: var(--yellow); font-size: .75rem; line-height: 1.45; }
    .telemetry-heading { display: flex; align-items: end; justify-content: space-between; gap: var(--space-5); margin: 0; }
    .telemetry-heading h2 { margin: 0; font-size: 1.2rem; }
    .telemetry-heading p { margin: 0; color: var(--muted); font-size: .78rem; }
    .telemetry-grid { display: grid; grid-template-columns: repeat(12, minmax(0, 1fr)); gap: var(--layout-gap); align-items: stretch; }
    .telemetry-card { grid-column: span 6; min-height: 0; padding: var(--card-padding); }
    .telemetry-card.wide { grid-column: span 6; }
    .diagnostic-third { grid-column: span 4; }
    .big-pair { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: var(--space-2); margin: 0 0 var(--space-3); }
    .big-reading { padding: var(--space-3); border: 1px solid var(--line-soft); border-radius: 11px; background: var(--bg-primary); }
    .big-reading span { display: block; color: var(--muted); font-size: .68rem; text-transform: uppercase; letter-spacing: .06em; }
    .big-reading strong { display: block; margin-top: var(--space-2); font-size: 1.55rem; font-variant-numeric: tabular-nums; }
    .telemetry-list { display: grid; gap: 0; }
    .telemetry-row { min-height: 36px; display: flex; align-items: center; justify-content: space-between; gap: var(--space-3); border-bottom: 1px solid var(--line-soft); font-size: .78rem; }
    .telemetry-row:last-child { border-bottom: 0; }
    .telemetry-row span { color: var(--muted); }
    .telemetry-row strong { min-width: 0; text-align: right; overflow-wrap: anywhere; font-variant-numeric: tabular-nums; }
    .axis { color: var(--text-primary); font-weight: 900; }
    .motor-bar { height: 5px; margin-top: var(--space-2); border-radius: 99px; background: var(--bg-control); overflow: hidden; }
    .motor-bar > div { width: 0; height: 100%; border-radius: inherit; background: var(--text-secondary); transition: width .15s; }
    .state-good { color: var(--green); }
    .state-warn { color: var(--yellow); }
    .state-bad { color: var(--danger); }
    .state-neutral { color: var(--text-muted); }
    footer { width: 100%; display: flex; justify-content: space-between; gap: var(--space-5); margin: 0; padding: var(--space-4) 0 0; border-top: 1px solid var(--line-soft); color: var(--text-muted); font-size: .72rem; }
    @media (max-width: 1050px) {
      .hero-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .main-grid { grid-template-columns: 1fr; }
      .operation-overview { grid-template-columns: repeat(3, minmax(0, 1fr)); }
      .operation-overview > div:nth-child(3) { border-right: 0; }
      .operation-overview > div { border-bottom: 1px solid var(--line-soft); }
      .operation-overview > div:nth-child(n + 4) { border-bottom: 0; }
      .operation-cockpit { grid-template-columns: 1fr; align-items: start; }
      .operation-control-panel { align-self: auto; contain: none; overflow: visible; }
      .operation-accordions { flex: 0 0 auto; margin-top: 0; overflow: visible; }
      .diagnostic-tools { grid-template-columns: 1fr; }
      .telemetry-card, .telemetry-card.wide,
      .diagnostic-third { grid-column: span 12; }
      .oled-preset-grid, .oled-preset-grid.mission-states { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    }
    @media (max-width: 680px) {
      html, body { overflow-x: hidden; }
      :root { --page-inline-space: 10px; --page-block-space: var(--space-4); }
      .topbar { align-items: flex-start; flex-direction: column; }
      .status-cluster { justify-content: flex-start; }
      .mode-navigation { grid-template-columns: 1fr; }
      .mode-tab { min-height: 40px; }
      .mode-heading { align-items: flex-start; flex-direction: column; }
      .mode-heading p { text-align: left; }
      .hero-grid { grid-template-columns: 1fr; }
      .hero-card { min-height: 124px; }
      .operation-overview { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .operation-overview > div { border-right: 1px solid var(--line-soft); border-bottom: 0; }
      .operation-overview > div:nth-child(-n + 4) { border-bottom: 1px solid var(--line-soft); }
      .operation-overview > div:nth-child(even) { border-right: 0; }
      .operation-overview > div:last-child { grid-column: 1 / -1; border-right: 0; }
      .operation-camera-header { flex-direction: column; }
      .operation-camera-panel .camera-meta { align-self: stretch; justify-content: flex-start; }
      .camera-hud-values { grid-template-columns: repeat(3, minmax(0, 1fr)); }
      .camera-hud-value:nth-child(3) { border-right: 0; }
      .camera-hud-value:nth-child(-n + 3) { border-bottom: 1px solid var(--border-subtle); }
      .operation-mission-summary, .operation-drive-summary { gap: var(--space-2); }
      .operation-mode-buttons { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .diagnostic-communication-alert { grid-template-columns: 1fr; gap: var(--space-2); }
      .section-header, .telemetry-heading { align-items: flex-start; flex-direction: column; }
      .camera-meta { justify-content: flex-start; }
      .camera-feed-grid[data-view="dual"] { grid-template-columns: 1fr; }
      .camera-view-button { min-width: 0; padding-inline: var(--space-1); }
      .camera-message, .safety-note { overflow-wrap: anywhere; }
      .keyboard-copy span { max-width: none; }
      .manual-speed-grid { grid-template-columns: 1fr; }
      .servo-control-grid { grid-template-columns: 1fr; }
      .servo-actions { align-items: stretch; flex-direction: column; }
      .servo-calibration-adjustments { grid-template-columns: repeat(4, minmax(0, 1fr)); }
      .servo-calibration-adjustments input { grid-column: 1 / 4; }
      .servo-calibration-endpoints { grid-template-columns: 1fr; }
      .oled-live-state, .oled-section-heading { align-items: flex-start; flex-direction: column; }
      .oled-editor-state, .oled-section-heading span { text-align: left; }
      .oled-preset-grid, .oled-preset-grid.mission-states, .oled-fields, .oled-actions { grid-template-columns: 1fr; }
      .oled-field.wide { grid-column: auto; }
      .telemetry-row { align-items: flex-start; flex-wrap: wrap; padding: var(--space-2) 0; }
      footer { flex-direction: column; }
    }
  </style>
</head>
<body>
  <main class="shell">
    <header class="topbar">
      <div class="brand">
        <div class="brand-mark"><img src="/dashboard-logo.png?v=1" width="52" height="52" alt="Logo Soul Sync"></div>
        <div><span class="eyebrow">OBR · Temporada 2026</span><h1>Soul Sync</h1></div>
      </div>
      <div class="status-cluster">
        <div id="connection" class="status-pill">PAINEL OFFLINE</div>
        <div id="esp32Link" class="status-pill">ESP32 OFFLINE</div>
        <div id="safetyStatus" class="status-pill warn">ROBÔ PARADO</div>
      </div>
    </header>

    <nav class="mode-navigation" role="tablist" aria-label="Áreas do dashboard">
      <button id="operationModeButton" type="button" class="mode-tab active" role="tab" data-dashboard-mode="operation" aria-controls="operationMode" aria-selected="true">Operação</button>
      <button id="diagnosticsModeButton" type="button" class="mode-tab" role="tab" data-dashboard-mode="diagnostics" aria-controls="diagnosticsMode" aria-selected="false">Diagnóstico</button>
    </nav>

    <section id="operationMode" class="dashboard-mode" role="tabpanel" aria-labelledby="operationModeButton" data-dashboard-panel="operation">
      <section class="operation-overview" aria-label="Resumo operacional">
        <div class="operation-overview-mode">
          <span>Modo atual</span>
          <strong id="mode" class="cockpit-mode-value">--</strong>
          <small id="modeDetail">Aguardando estado da Raspberry</small>
        </div>
        <div class="operation-overview-item battery">
          <span>Bateria</span>
          <strong id="batteryVoltage">--.-- V</strong>
          <div class="operation-meter"><div id="batteryFill"></div></div>
        </div>
        <div class="operation-overview-item">
          <span>ESP32</span>
          <strong id="esp32Status" class="status-bad">OFFLINE</strong>
        </div>
        <div class="operation-raspberry" aria-label="Estado da Raspberry Pi">
          <span class="operation-overview-label">Raspberry Pi</span>
          <div class="operation-system-metrics">
            <div id="cpuMetric" class="operation-system-metric"><span>CPU</span><strong id="cpu">--%</strong><i><i id="cpuFill"></i></i></div>
            <div id="ramMetric" class="operation-system-metric"><span>RAM</span><strong id="ram">--%</strong><i><i id="ramFill"></i></i></div>
            <div id="temperatureMetric" class="operation-system-metric"><span>Temp.</span><strong id="temp">-- °C</strong><i><i id="temperatureFill"></i></i></div>
          </div>
        </div>
        <div class="operation-overview-item">
          <span>FPS câmera</span>
          <strong id="operationCameraFps">-- FPS</strong>
        </div>
      </section>

      <section class="operation-cockpit">
        <section class="card operation-camera-panel">
          <div class="operation-camera-header">
            <div>
              <span class="eyebrow">Visão principal</span>
              <h2 id="operationCameraName">Câmera inferior</h2>
            </div>
            <div id="cameraMetadata" class="camera-meta" aria-live="polite"></div>
          </div>
          <div class="camera-view-selector" role="group" aria-label="Visualização das câmeras">
            <button type="button" class="camera-view-button active" data-camera-view="downward" aria-pressed="true">Inferior</button>
            <button type="button" class="camera-view-button" data-camera-view="forward" aria-pressed="false">Frontal</button>
            <button type="button" class="camera-view-button" data-camera-view="dual" aria-pressed="false">Dupla</button>
          </div>
          <details class="camera-details">
            <summary>Detalhes da câmera</summary>
            <div id="cameraTechnicalMetadata" class="camera-technical-metadata"></div>
          </details>
          <div id="datasetCaptureStatus" class="dataset-capture-strip" data-state="paused" aria-live="polite">
            <strong id="datasetCaptureState">Dataset pausado</strong>
            <span id="datasetCaptureDetails">Nenhuma coleta configurada.</span>
          </div>
          <div id="cameraFeeds" class="camera-feed-grid" data-view="downward" aria-live="polite"></div>
          <div id="downwardCameraTelemetry" class="camera-hud" aria-label="Estado da visão inferior" hidden>
            <div class="camera-hud-header">
              <strong id="cameraLineFollowerState" class="camera-hud-line">CONTROLE PENDENTE</strong>
              <strong id="forwardAssistDiagnostic">NEAR UNKNOWN | FWD UNCERTAIN | DECISION NORMAL | SOURCE UNAVAILABLE | CONTROLE INFERIOR</strong>
              <span id="cameraHudFps">-- FPS</span>
            </div>
            <div class="camera-hud-values">
              <div class="camera-hud-value green-status"><span>Verde</span><strong id="operationGreenInterpretation">SEM DECISÃO</strong><small id="operationGreenConfirmed">NÃO CONFIRMADO</small></div>
              <div class="camera-hud-value path-status"><span>Faixa associada</span><strong id="operationGreenPathBlackValid">NÃO</strong><small id="operationLineSequence">—</small></div>
              <div class="camera-hud-value"><span>Reparo da máscara</span><strong id="operationSpecularRepair">—</strong></div>
              <div class="camera-hud-value speed"><span>Speed</span><strong id="machineEncoderSpeed">-- / -- cont/s</strong></div>
              <div class="camera-hud-value pipeline"><span>Pipeline</span><strong id="operationPipelinePrimary">BIN — · SPEC — ms</strong><small id="operationPipelineSecondary">MORPH — · CONT — ms</small></div>
              <div class="camera-hud-value"><span>Entrada prata</span><strong id="operationSilverShadowState">INDISPONÍVEL</strong><small id="operationSilverShadowDetails">—</small></div>
            </div>
          </div>
        </section>

        <aside id="missionStateCard" class="card operation-control-panel mission-state-card" data-tone="idle">
          <div class="operation-control-main">
            <div class="operation-control-header">
              <div><span class="eyebrow">Controle da prova</span><h2>Missão e comando</h2></div>
              <span id="machineStateBadge" class="machine-badge">PARADO</span>
            </div>

            <div class="operation-mission-summary">
              <div><span>Missão atual</span><strong id="machineMission">Principal</strong></div>
              <div><span>Comportamento</span><strong id="machineBehavior">NENHUM</strong></div>
            </div>
            <div class="machine-action operation-machine-action">
              <span>Ação atual</span>
              <strong id="machineAction">Missão parada</strong>
            </div>
            <div class="machine-progress operation-progress">
              <div class="machine-progress-label"><span>Progresso da etapa</span><strong id="machineProgressValue">0%</strong></div>
              <div class="machine-progress-track"><div id="machineProgressFill" class="machine-progress-fill"></div></div>
            </div>
            <div class="operation-drive-summary">
              <div><span>Velocidade E / D</span><strong id="machineAppliedSpeed">-- / --</strong></div>
              <div><span>Comando E / D</span><strong id="machineRequestedSpeed" class="accent">0.00 / 0.00</strong></div>
            </div>

            <div class="mission-selector operation-mission-selector">
              <label for="autonomousMission">Missão autônoma</label>
              <select id="autonomousMission">
                <option value="main_mission" selected>MISSÃO PRINCIPAL</option>
                <option value="turn_right_90">GIRO 90° À DIREITA</option>
                <option value="drive_distance">PERCORRER DISTÂNCIA</option>
                <option value="rescue_zone_detection">ÁREAS DE RESGATE</option>
                <option value="rescue_zone_frame">FRAME_ZONE · ENQUADRAR ÁREA</option>
                <option value="rescue_area">RESGATE · DETECTAR + ALINHAR/IR ATRÁS</option>
                <option value="obstacle_avoidance">DESVIO DE OBSTÁCULO</option>
                <option value="servo_initialize">SERVOS · POSE HOME 15/0/0</option>
                <option value="servo_capture">SERVOS · CAPTURA</option>
                <option value="servo_internal_storage">SERVOS · ARMAZENAMENTO INTERNO</option>
                <option value="servo_deposit">SERVOS · DEPÓSITO</option>
                <option value="servo_full_sequence">SERVOS · SEQUÊNCIA COM ARMAZENAMENTO</option>
                <option value="servo_full_sequence_two">SERVOS · SEQUÊNCIA SEM ARMAZENAMENTO</option>
              </select>
              <span id="missionHint" class="mission-hint">Segue-faixa com retorno verde e desvio ultrassônico de obstáculo.</span>
              <div id="distanceMissionSettings" class="distance-mission-settings" hidden>
                <span class="distance-input-label">Distância alvo</span>
                <div class="distance-input">
                  <input id="distanceTargetCm" type="number" min="1" max="300" step="1" value="20" inputmode="decimal" aria-label="Distância alvo em centímetros">
                  <strong>cm</strong>
                </div>
                <small class="distance-calibration">Calibração real: 3600 contagens = 18,7 cm · 192,51 cont/cm.</small>
              </div>
              <div id="rescueZoneFrameSettings" class="distance-mission-settings" hidden>
                <span class="distance-input-label">Cor alvo obrigatória</span>
                <select id="rescueZoneTargetColor" aria-label="Cor alvo do FRAME_ZONE">
                  <option value="green" selected>GREEN</option>
                  <option value="red">RED</option>
                </select>
                <small class="distance-calibration">O modo enquadra somente a cor escolhida e termina parado.</small>
              </div>
              <div id="servoRoutineSettings" class="distance-mission-settings" hidden>
                <span id="servoRoutineStorageState" class="distance-input-label">Armazenamento interno vazio</span>
                <button id="servoRoutineConfirmButton" type="button" class="warning" disabled>Confirmar próximo passo</button>
                <small class="distance-calibration">O armazenamento aguarda o pedido de fechamento. No depósito, cada abertura deve ser confirmada em até 2 segundos.</small>
              </div>
            </div>

            <div class="mode-buttons operation-mode-buttons">
              <button id="manualButton" onclick="sendCommand('start')">Manual</button>
              <button id="autoButton" onclick="startAutonomousMission()">Autônomo</button>
              <button id="calibrationButton" class="warning" onclick="sendCommand('calibrate')" aria-label="Calibrar sensores; mantenha o robô parado" title="Mantenha o robô parado durante a calibração" disabled>Calibrar</button>
              <button id="stopButton" class="danger operation-stop" onclick="sendCommand('stop')">Parar</button>
            </div>
          </div>

          <div class="operation-accordions">
            <details class="cockpit-details">
              <summary>Detalhes da missão</summary>
              <div class="cockpit-details-body">
                <div class="machine-flow" aria-label="Fluxo da máquina de estados">
                  <span id="machineStepPerception" class="machine-step">Percepção</span>
                  <span id="machineStepDecision" class="machine-step">Decisão</span>
                  <span id="machineStepMotion" class="machine-step">Movimento</span>
                  <span id="machineStepFeedback" class="machine-step">Feedback</span>
                </div>
                <div class="machine-metrics operation-detail-metrics">
                  <div class="machine-metric"><span>Alvo de distância</span><strong id="machineDistanceTarget">-- cm</strong></div>
                  <div class="machine-metric"><span>Distância E / D</span><strong id="machineDistanceSides">-- / -- cm</strong></div>
                  <div class="machine-metric"><span>Média percorrida</span><strong id="machineDistanceAverage">-- cm</strong></div>
                  <div class="machine-metric"><span>Modo</span><strong id="machineMode">PARADO</strong></div>
                </div>
              </div>
            </details>

            <details class="cockpit-details">
              <summary>Controle manual detalhado</summary>
              <div class="cockpit-details-body manual-control-body">
                <div class="keyboard-panel">
                  <div class="keyboard-copy"><strong>Controle WASD</strong><span id="keyboardState">Ative o modo Manual para usar o teclado.</span></div>
                  <div class="keys" aria-label="Teclas de movimento">
                    <span id="keyW" class="keycap w">W</span><span id="keyA" class="keycap a">A</span><span id="keyS" class="keycap s">S</span><span id="keyD" class="keycap d">D</span>
                  </div>
                </div>
                <div class="manual-speed-grid">
                  <div class="drive-control">
                    <label for="manualDrivePower"><span>Velocidade reta de teste</span><output id="manualDrivePowerValue">0.69</output></label>
                    <input id="manualDrivePower" type="range" min="0.05" max="1.00" step="0.01" value="0.69">
                  </div>
                  <div class="drive-control">
                    <label for="manualTurnPower"><span>Velocidade em curva de teste</span><output id="manualTurnPowerValue">0.69</output></label>
                    <input id="manualTurnPower" type="range" min="0.05" max="1.00" step="0.01" value="0.69">
                  </div>
                  <p class="manual-test-note">O controle manual usa o perfil operacional. Em reta, o sincronismo pelos encoders corrige a diferença entre os lados.</p>
                </div>
              </div>
            </details>
          </div>
        </aside>
      </section>
    </section>

    <section id="diagnosticsMode" class="dashboard-mode" role="tabpanel" aria-labelledby="diagnosticsModeButton" data-dashboard-panel="diagnostics" hidden>
      <div class="mode-heading">
        <div><span class="eyebrow">Inspeção técnica</span><h2>Diagnóstico de hardware</h2></div>
        <p>Motores, encoders, sensores, barramentos, watchdogs e ferramentas de manutenção.</p>
      </div>

      <section id="esp32CommunicationAlert" class="diagnostic-communication-alert" role="status" hidden>
        <div><strong id="esp32CommunicationAlertTitle">ESP32 OFFLINE</strong><span id="esp32CommunicationAlertAge">Sem telemetria recente</span></div>
        <p>Subsistemas afetados: motores / encoders / IMU / sensores / periféricos.</p>
      </section>

      <section class="telemetry-grid diagnostic-subsystems">
      <article id="communicationSubsystem" class="card telemetry-card diagnostic-subsystem communication" data-telemetry="missing">
        <div class="section-header"><h3 class="section-title">Comunicação</h3><span id="communicationSubsystemStatus" class="subsystem-status">sem telemetria</span></div>
        <div class="telemetry-list">
          <div class="telemetry-row"><span>ESP32</span><strong id="diagnosticEsp32Status" class="state-neutral">sem telemetria</strong></div>
          <div class="telemetry-row"><span>Idade da telemetria</span><strong id="telemetryAge">-- ms</strong></div>
          <div class="telemetry-row"><span>UART Raspberry</span><strong id="serialState">--</strong></div>
          <div class="telemetry-row"><span>Configuração UART</span><strong>115200 bps · TX GPIO14 / RX GPIO15</strong></div>
          <div class="telemetry-row"><span>TX / RX ESP32</span><strong>GPIO1 / GPIO3</strong></div>
          <div class="telemetry-row"><span>Uptime ESP32</span><strong id="diagnosticUptime">--</strong></div>
          <div class="telemetry-row"><span>Timeout de comando · Raspberry</span><strong id="raspberryCommandTimeout">-- ms</strong></div>
          <div class="telemetry-row"><span>LED de sistema · BCM GPIO26</span><strong id="readyLedState">--</strong></div>
        </div>
      </article>

      <article id="tractionSubsystem" class="card telemetry-card diagnostic-subsystem" data-telemetry="missing">
        <div class="section-header"><h3 class="section-title">Tração</h3><span id="tractionSubsystemStatus" class="subsystem-status">sem telemetria</span></div>
        <div class="big-pair">
          <div class="big-reading"><span>Esquerda · 2 motores</span><strong id="appliedLeft">0.00</strong><div class="motor-bar"><div id="leftMotorBar"></div></div></div>
          <div class="big-reading"><span>Direita · 2 motores</span><strong id="appliedRight">0.00</strong><div class="motor-bar"><div id="rightMotorBar"></div></div></div>
        </div>
        <div class="telemetry-list">
           <div class="telemetry-row"><span>Comando Raspberry · esquerda / direita</span><strong><span id="requestedLeft">0.00</span> / <span id="requestedRight">0.00</span></strong></div>
           <div class="telemetry-row"><span>Comando final · esquerda / direita</span><strong><span id="commandedLeft">0.00</span> / <span id="commandedRight">0.00</span></strong></div>
           <div class="telemetry-row"><span>Estado dos pisos</span><strong id="motorMotionState">--</strong></div>
          <div class="telemetry-row"><span>Driver</span><strong>2 × DRV8833 · 4 motores</strong></div>
          <div class="telemetry-row"><span>DRV8833 nSLEEP · GPIO26</span><strong id="sleepState">--</strong></div>
          <div class="telemetry-row"><span>Perfil de potência</span><strong id="motorCommandProfile">--</strong></div>
          <div class="telemetry-row"><span>E-Stop local da ESP32</span><strong id="esp32Estop">--</strong></div>
          <div class="telemetry-row"><span>Watchdog de motor · ESP32</span><strong id="esp32MotorTimeout">-- ms</strong></div>
        </div>
      </article>

      <article id="encoderSubsystem" class="card telemetry-card diagnostic-subsystem" data-telemetry="missing">
        <div class="section-header"><h3 class="section-title">Encoders</h3><span id="encoderSubsystemStatus" class="subsystem-status">sem telemetria</span></div>
        <div class="big-pair">
          <div class="big-reading"><span>Contagem esquerda</span><strong id="leftEncoderCount">--</strong></div>
          <div class="big-reading"><span>Contagem direita</span><strong id="rightEncoderCount">--</strong></div>
        </div>
        <div class="telemetry-list">
          <div class="telemetry-row"><span>Taxa esquerda</span><strong id="leftEncoderRate">-- cont/s</strong></div>
          <div class="telemetry-row"><span>Taxa direita</span><strong id="rightEncoderRate">-- cont/s</strong></div>
          <div class="telemetry-row"><span>Posição esquerda desde o último reset</span><strong id="leftEncoderPosition">-- cm</strong></div>
          <div class="telemetry-row"><span>Posição direita desde o último reset</span><strong id="rightEncoderPosition">-- cm</strong></div>
          <div class="telemetry-row"><span>Calibração usada pela missão</span><strong id="machineEncoderCalibration">-- cont/cm</strong></div>
          <div class="telemetry-row"><span>Canais</span><strong>GPIO19/21 · GPIO22/23</strong></div>
        </div>
      </article>

      <article id="imuSubsystem" class="card telemetry-card diagnostic-subsystem diagnostic-third" data-telemetry="missing">
        <div class="section-header"><h3 class="section-title">IMU · MPU6050</h3><span id="imuSubsystemStatus" class="subsystem-status">sem telemetria</span></div>
        <div class="big-pair">
          <div class="big-reading"><span>Giro integrado</span><strong id="yawZ">-- °</strong></div>
          <div class="big-reading"><span>Inclinação da rampa</span><strong id="rampAngle">-- °</strong></div>
        </div>
        <div class="telemetry-list">
          <div class="telemetry-row"><span>MPU6050</span><strong id="mpuState">--</strong></div>
          <div class="telemetry-row"><span>Giroscópio X / Y / Z</span><strong><b class="axis">X</b> <span id="gyroX">--</span> &nbsp; <b class="axis">Y</b> <span id="gyroY">--</span> &nbsp; <b class="axis">Z</b> <span id="gyroZ">--</span> °/s</strong></div>
          <div class="telemetry-row"><span>Aceleração X / Y / Z</span><strong><b class="axis">X</b> <span id="accelX">--</span> &nbsp; <b class="axis">Y</b> <span id="accelY">--</span> &nbsp; <b class="axis">Z</b> <span id="accelZ">--</span> m/s²</strong></div>
          <div class="telemetry-row"><span>Temperatura do IMU</span><strong id="imuTemperature">-- °C</strong></div>
        </div>
      </article>

      <article id="sensorSubsystem" class="card telemetry-card diagnostic-subsystem diagnostic-third" data-telemetry="missing">
        <div class="section-header"><h3 class="section-title">Sensores</h3><span id="sensorFreshState" class="subsystem-status">sem telemetria</span></div>
        <div class="big-reading"><span>Ultrassônico frontal</span><strong id="ultrasonic">-- cm</strong></div>
        <div class="telemetry-list">
          <div class="telemetry-row"><span>Bateria ADC · GPIO36</span><strong id="batteryAdc">-- mV</strong></div>
          <div class="telemetry-row"><span>Start button · GPIO27</span><strong id="startButtonState">--</strong></div>
          <div class="telemetry-row"><span>Calibração · segure Start por 5 s</span><strong id="calibrationState">--</strong></div>
        </div>
      </article>

      <article id="peripheralSubsystem" class="card telemetry-card diagnostic-subsystem diagnostic-third" data-telemetry="missing">
        <div class="section-header"><h3 class="section-title">Periféricos / I2C</h3><span id="peripheralSubsystemStatus" class="subsystem-status">sem telemetria</span></div>
        <div class="telemetry-list">
          <div class="telemetry-row"><span>PCA9685 · 0x40</span><strong id="pcaState">--</strong></div>
          <div class="telemetry-row"><span>OLED SSD1306</span><strong id="oledState">--</strong></div>
          <div class="telemetry-row"><span>Inicialização geral na OLED</span><strong id="oledBootState">--</strong></div>
          <div class="telemetry-row"><span>Página enviada pela Raspberry</span><strong id="oledRemoteTelemetry">--</strong></div>
          <div class="telemetry-row"><span>Barramento I2C</span><strong>GPIO14 / GPIO13</strong></div>
        </div>
      </article>
      </section>

      <div class="telemetry-heading diagnostic-maintenance-heading">
        <div><span class="eyebrow">Ações explícitas</span><h2>Manutenção</h2></div>
        <p>Ferramentas mantidas separadas da leitura passiva dos subsistemas.</p>
      </div>

      <section class="diagnostic-tools">
        <article id="independentMotorControl" class="card section-card command-card">
          <div class="section-header"><h3 class="section-title">Acionamento independente dos motores</h3></div>
          <div class="requested-drive">
            <div class="request-value"><span>Lado esquerdo · ajuste exato</span><div class="request-adjustment"><button class="trim-button" data-side="left" data-delta="-0.01" aria-label="Reduzir lado esquerdo em 0,01">−.01</button><input id="leftValue" aria-label="Potência exata do lado esquerdo" type="number" min="-1" max="1" step="0.01" value="0.00" inputmode="decimal"><button class="trim-button" data-side="left" data-delta="0.01" aria-label="Aumentar lado esquerdo em 0,01">+.01</button></div></div>
            <div class="request-value"><span>Lado direito · ajuste exato</span><div class="request-adjustment"><button class="trim-button" data-side="right" data-delta="-0.01" aria-label="Reduzir lado direito em 0,01">−.01</button><input id="rightValue" aria-label="Potência exata do lado direito" type="number" min="-1" max="1" step="0.01" value="0.00" inputmode="decimal"><button class="trim-button" data-side="right" data-delta="0.01" aria-label="Aumentar lado direito em 0,01">+.01</button></div></div>
          </div>
          <div class="safety-note">Ative o modo Manual antes do teste e mantenha as rodas suspensas. Cada lado usa PWM direto; clamp, E-Stop e timeouts continuam ativos.</div>
        </article>

        <article id="servoControl" class="card section-card servo-control-card">
          <div class="section-header"><h3 class="section-title">Braço, pulso e garra</h3><span id="servoControlStatus" class="subsystem-status">bloqueado</span></div>
          <div class="servo-control-grid">
            <div class="servo-control">
              <label for="armServoSlider"><span>Braço · canal 0</span><output id="armServoValue">0°</output></label>
              <input id="armServoSlider" data-servo-slider="arm" type="range" min="0" max="180" step="1" value="0" disabled>
              <div class="servo-angle-entry"><input id="armServoInput" data-servo-input="arm" type="number" min="0" max="180" step="1" value="0" inputmode="numeric" aria-label="Ângulo exato do braço" disabled><span>graus</span></div>
              <small id="armServoState" class="servo-output-state">Sem sinal confirmado.</small>
            </div>
            <div class="servo-control">
              <label for="wristServoSlider"><span>Pulso · canal 1</span><output id="wristServoValue">0°</output></label>
              <input id="wristServoSlider" data-servo-slider="wrist" type="range" min="0" max="180" step="1" value="0" disabled>
              <div class="servo-angle-entry"><input id="wristServoInput" data-servo-input="wrist" type="number" min="0" max="180" step="1" value="0" inputmode="numeric" aria-label="Ângulo exato do pulso" disabled><span>graus</span></div>
              <small id="wristServoState" class="servo-output-state">Sem sinal confirmado.</small>
            </div>
            <div class="servo-control">
              <label for="gripperServoSlider"><span>Garra · canal 2</span><output id="gripperServoValue">0°</output></label>
              <input id="gripperServoSlider" data-servo-slider="gripper" type="range" min="0" max="180" step="1" value="0" disabled>
              <div class="servo-angle-entry"><input id="gripperServoInput" data-servo-input="gripper" type="number" min="0" max="180" step="1" value="0" inputmode="numeric" aria-label="Ângulo exato da garra" disabled><span>graus</span></div>
              <small id="gripperServoState" class="servo-output-state">Sem sinal confirmado.</small>
            </div>
          </div>
          <div class="servo-actions">
            <p>Ative o modo Manual. O botão Parar e a perda da Raspberry removem os três sinais; a trava de emergência do software permanece como redundância.</p>
            <button id="disableServosButton" type="button" class="warning" onclick="disableAllServos()" disabled>Desligar sinais</button>
          </div>
          <details id="servoCalibrationPanel" class="servo-calibration">
            <summary><span>Calibrar pulsos sem regravar</span><span id="servoCalibrationStatus" class="subsystem-status">aguardando firmware</span></summary>
            <div class="servo-calibration-body">
              <p class="servo-calibration-warning">O botão Parar sempre encerra este modo e desliga todos os pulsos. Comece em 1500 µs, avance em passos pequenos e pare antes de ruído, vibração, aquecimento ou batente mecânico.</p>
              <div class="servo-calibration-toolbar">
                <label for="servoCalibrationTarget">Servo em teste
                  <select id="servoCalibrationTarget" disabled>
                    <option value="arm">Braço · MG995 · canal 0</option>
                    <option value="wrist">Pulso · MG90S · canal 1</option>
                    <option value="gripper">Garra · MG90S · canal 2</option>
                  </select>
                </label>
                <button id="servoCalibrationBeginButton" type="button" class="warning" disabled>Entrar na calibração</button>
                <button id="servoCalibrationEndButton" type="button" class="danger" disabled>Parar calibração</button>
              </div>
              <div id="servoCalibrationWorkspace" class="servo-calibration-workspace" hidden>
                <div class="servo-calibration-readout"><span>Pulso bruto solicitado</span><strong id="servoCalibrationPulseValue">1500 µs</strong></div>
                <input id="servoCalibrationPulseSlider" type="range" min="500" max="2500" step="10" value="1500" disabled>
                <div class="servo-calibration-adjustments">
                  <button type="button" data-servo-pulse-delta="-100" disabled>−100</button>
                  <button type="button" data-servo-pulse-delta="-10" disabled>−10</button>
                  <button type="button" data-servo-pulse-delta="10" disabled>+10</button>
                  <button type="button" data-servo-pulse-delta="100" disabled>+100</button>
                  <input id="servoCalibrationPulseInput" type="number" min="500" max="2500" step="10" value="1500" inputmode="numeric" aria-label="Pulso bruto do servo em microssegundos" disabled>
                  <button id="servoCalibrationApplyButton" type="button" disabled>Aplicar pulso</button>
                </div>
                <div class="servo-calibration-endpoints">
                  <div class="servo-calibration-endpoint">
                    <label for="servoPulseAtZeroInput">Posição lógica 0°
                      <input id="servoPulseAtZeroInput" type="number" min="500" max="2500" step="10" value="500" inputmode="numeric" disabled>
                    </label>
                    <button id="captureServoZeroButton" type="button" disabled>Usar atual</button>
                  </div>
                  <div class="servo-calibration-endpoint">
                    <label for="servoPulseAt180Input">Posição lógica 180°
                      <input id="servoPulseAt180Input" type="number" min="500" max="2500" step="10" value="2500" inputmode="numeric" disabled>
                    </label>
                    <button id="captureServo180Button" type="button" disabled>Usar atual</button>
                  </div>
                </div>
                <div class="servo-calibration-actions">
                  <button id="servoCalibrationDisableButton" type="button" class="warning" disabled>Desligar pulso</button>
                  <button id="servoCalibrationSaveButton" type="button" disabled>Salvar neste servo</button>
                </div>
                <p id="servoCalibrationMessage" class="servo-calibration-note">Entrar na calibração não movimenta o servo. O primeiro pulso só sai ao usar Aplicar.</p>
              </div>
            </div>
          </details>
        </article>

        <article class="card section-card diagnostic-oled-card">
          <details class="oled-editor">
            <summary><span>OLED · estados e mensagens</span><span id="oledEditorSummaryState" class="oled-summary-state">SEM TELEMETRIA</span></summary>
            <div class="oled-editor-body">
              <div class="oled-live-state">
                <div><span>Estado físico atual</span><strong id="oledCurrentState">AGUARDANDO TELEMETRIA</strong></div>
                <p id="oledRemoteStatus" class="oled-editor-state">Aguardando telemetria da OLED.</p>
              </div>

              <section class="oled-section" aria-labelledby="oledAutomaticAlertsTitle">
                <div class="oled-section-heading"><strong id="oledAutomaticAlertsTitle">Alertas automáticos atuais</strong><span>Envia o mesmo layout grande e pulsante usado durante a missão.</span></div>
                <div class="oled-preset-grid">
                  <button type="button" class="oled-preset" data-oled-send data-oled-preset data-primary="VERDE" data-secondary="ESQUERDA" data-preset-label="Verde · esquerda" disabled><strong>VERDE</strong><span>ESQUERDA</span></button>
                  <button type="button" class="oled-preset" data-oled-send data-oled-preset data-primary="VERDE" data-secondary="DIREITA" data-preset-label="Verde · direita" disabled><strong>VERDE</strong><span>DIREITA</span></button>
                  <button type="button" class="oled-preset" data-oled-send data-oled-preset data-primary="VERDE" data-secondary="180 GRAUS" data-preset-label="Verde · retorno de 180 graus" disabled><strong>VERDE</strong><span>180 GRAUS</span></button>
                  <button type="button" class="oled-preset" data-oled-send data-oled-preset data-primary="GAP" data-secondary="CONFIRMADO" data-preset-label="GAP confirmado" disabled><strong>GAP</strong><span>CONFIRMADO</span></button>
                  <button type="button" class="oled-preset" data-oled-send data-oled-preset data-primary="CINZA" data-secondary="CONFIRMADO" data-preset-label="Faixa cinza confirmada" disabled><strong>CINZA</strong><span>CONFIRMADO</span></button>
                  <button type="button" class="oled-preset" data-oled-send data-oled-preset data-primary="DESVIO" data-secondary="" data-preset-label="Desvio de obstáculo" disabled><strong>DESVIO</strong><span>SEM DETALHE</span></button>
                </div>
              </section>

              <section class="oled-section" aria-labelledby="oledMissionStatesTitle">
                <div class="oled-section-heading"><strong id="oledMissionStatesTitle">Estados da missão principal</strong><span>Atalhos para simular visualmente as cinco fases atuais; não são alertas automáticos.</span></div>
                <div class="oled-preset-grid mission-states">
                  <button type="button" class="oled-preset" data-oled-send data-oled-preset data-primary="LINHA" data-secondary="PERCURSO INICIAL" data-preset-label="Percurso inicial" disabled><strong>LINHA</strong><span>PERCURSO INICIAL</span></button>
                  <button type="button" class="oled-preset" data-oled-send data-oled-preset data-primary="RESGATE" data-secondary="AREA DE RESGATE" data-preset-label="Área de resgate" disabled><strong>RESGATE</strong><span>ÁREA DE RESGATE</span></button>
                  <button type="button" class="oled-preset" data-oled-send data-oled-preset data-primary="LINHA" data-secondary="PERCURSO FINAL" data-preset-label="Percurso final" disabled><strong>LINHA</strong><span>PERCURSO FINAL</span></button>
                  <button type="button" class="oled-preset" data-oled-send data-oled-preset data-primary="MISSAO" data-secondary="CONCLUIDA" data-preset-label="Missão concluída" disabled><strong>MISSÃO</strong><span>CONCLUÍDA</span></button>
                  <button type="button" class="oled-preset" data-oled-send data-oled-preset data-primary="MISSAO" data-secondary="INTERROMPIDA" data-preset-label="Missão interrompida" disabled><strong>MISSÃO</strong><span>INTERROMPIDA</span></button>
                </div>
              </section>

              <section class="oled-section" aria-labelledby="oledCustomMessageTitle">
                <div class="oled-section-heading"><strong id="oledCustomMessageTitle">Mensagem personalizada</strong><span>Escolha entre a página comum e o texto grande adaptativo.</span></div>
                <div class="oled-layout-selector" role="group" aria-label="Layout da mensagem OLED">
                  <button type="button" class="oled-layout-button active" data-oled-layout="standard" aria-pressed="true">Título + duas linhas</button>
                  <button type="button" class="oled-layout-button" data-oled-layout="large" aria-pressed="false">Texto grande + detalhe</button>
                </div>
                <div id="oledStandardFields" class="oled-fields">
                  <div class="oled-field wide"><label for="oledTitle">Título · até 12 caracteres</label><input id="oledTitle" maxlength="12" value="OBR 2026" placeholder="OBR 2026"></div>
                  <div class="oled-field"><label for="oledFirstLine">Linha 1 · até 20</label><input id="oledFirstLine" maxlength="20" value="ROBO PRONTO" placeholder="ROBO PRONTO"></div>
                  <div class="oled-field"><label for="oledSecondLine">Linha 2 · até 20</label><input id="oledSecondLine" maxlength="20" value="AGUARDANDO" placeholder="AGUARDANDO"></div>
                </div>
                <div id="oledLargeFields" class="oled-fields" hidden>
                  <div class="oled-field"><label for="oledPrimaryText">Texto principal · até 20</label><input id="oledPrimaryText" maxlength="20" value="VERDE" placeholder="VERDE"></div>
                  <div class="oled-field"><label for="oledSecondaryText">Detalhe opcional · até 20</label><input id="oledSecondaryText" maxlength="20" value="ESQUERDA" placeholder="ESQUERDA"></div>
                </div>
                <div class="oled-fields">
                  <div class="oled-field wide"><label for="oledDurationSeconds">Duração · 0,5 a 30 segundos</label><input id="oledDurationSeconds" type="number" min="0.5" max="30" step="0.5" value="10" inputmode="decimal"></div>
                </div>
                <p id="oledLayoutHint" class="oled-layout-hint">A página comum mostra título, duas linhas e o tempo restante.</p>
              </section>
              <div class="oled-actions">
                <button id="oledShowButton" type="button" data-oled-send onclick="showOledMessage()" disabled>Mostrar página comum</button>
                <button id="oledClearButton" type="button" class="oled-clear" onclick="clearOledMessage()" disabled>Tela padrão</button>
              </div>
            </div>
          </details>
        </article>
      </section>
    </section>

    <footer><span>OBR 2026 · Plataforma de controle e telemetria</span><span>Raspberry Pi ↔ UART ↔ ESP32</span></footer>
  </main>

  <script>
    let ws;
    let manualEnabled = false;
    const driveKeyCodes = ["KeyW", "KeyA", "KeyS", "KeyD"];
    const pressedDriveKeys = new Set();
    const element = id => document.getElementById(id);
    const dashboardModeButtons = Array.from(document.querySelectorAll("[data-dashboard-mode]"));
    const dashboardModePanels = Array.from(document.querySelectorAll("[data-dashboard-panel]"));
    const cameras = {
      downward: {
        id: "downward",
        name: "Câmera inferior",
        role: "Segue-faixa e detecção de verde",
        streamUrl: "/camera-stream.mjpg",
        statusUrl: "/camera-status.json",
        displayMode: "real",
        status: "INICIANDO",
        enabled: true,
        active: false,
        transitioning: false,
        requestedEnabled: true,
        transitionDeadlineMs: 0,
        error: "",
        metadata: { fps: "--", resolution: "--", sensor: "--", crop: "--", format: "--" }
      },
      forward: {
        id: "forward",
        name: "Câmera frontal",
        role: "Resgate e percepção frontal",
        streamUrl: "/forward-camera-stream.mjpg",
        statusUrl: "/forward-camera-status.json",
        status: "INICIANDO",
        enabled: true,
        active: false,
        transitioning: false,
        requestedEnabled: true,
        transitionDeadlineMs: 0,
        error: "",
        metadata: { fps: "0.0", resolution: "960×540", sensor: "1920×1080 10-bit", crop: "--", format: "--" },
        ball: { enabled: false, detected: false, type: "", position: "nenhuma", distanceCm: null, extrapolated: false, angleDegrees: null, centerX: null, centerY: null, radiusPixels: null, diameterPixels: null, circularity: null, topClipped: false, detectionMethod: "", processingMs: null }
      }
    };
    const connection = element("connection");
    const manualDrivePower = element("manualDrivePower");
    const manualTurnPower = element("manualTurnPower");
    const leftValue = element("leftValue");
    const rightValue = element("rightValue");
    const cameraFeeds = element("cameraFeeds");
    const cameraMetadata = element("cameraMetadata");
    const cameraTechnicalMetadata = element("cameraTechnicalMetadata");
    const operationCameraName = element("operationCameraName");
    const operationCameraFps = element("operationCameraFps");
    const datasetCaptureStatus = element("datasetCaptureStatus");
    const datasetCaptureState = element("datasetCaptureState");
    const datasetCaptureDetails = element("datasetCaptureDetails");
    const cameraHudFps = element("cameraHudFps");
    const downwardCameraTelemetry = element("downwardCameraTelemetry");
    const forwardAssistDiagnostic = element("forwardAssistDiagnostic");
    const cameraViewButtons = Array.from(document.querySelectorAll("[data-camera-view]"));
    const operationCameraDiagnosticFields = {
      lineFollowerState: element("cameraLineFollowerState"),
      greenInterpretation: element("operationGreenInterpretation"),
      greenConfirmed: element("operationGreenConfirmed"),
      greenPathBlackValid: element("operationGreenPathBlackValid"),
      lineSequence: element("operationLineSequence"),
      specularRepair: element("operationSpecularRepair"),
      pipelinePrimary: element("operationPipelinePrimary"),
      pipelineSecondary: element("operationPipelineSecondary"),
      silverShadowState: element("operationSilverShadowState"),
      silverShadowDetails: element("operationSilverShadowDetails")
    };
    const autonomousMission = element("autonomousMission");
    const distanceTargetCm = element("distanceTargetCm");
    const rescueZoneTargetColor = element("rescueZoneTargetColor");
    const oledTitle = element("oledTitle");
    const oledFirstLine = element("oledFirstLine");
    const oledSecondLine = element("oledSecondLine");
    const oledPrimaryText = element("oledPrimaryText");
    const oledSecondaryText = element("oledSecondaryText");
    const oledDurationSeconds = element("oledDurationSeconds");
    const oledStandardFields = element("oledStandardFields");
    const oledLargeFields = element("oledLargeFields");
    const oledLayoutButtons = Array.from(
      document.querySelectorAll("[data-oled-layout]"));
    const oledSendButtons = Array.from(
      document.querySelectorAll("[data-oled-send]"));
    const oledPresetButtons = Array.from(
      document.querySelectorAll("[data-oled-preset]"));
    let selectedOledLayout = "standard";
    let oledAutomaticAlertDurationMs = 2500;
    const servoControls = {
      arm: {
        slider: element("armServoSlider"), input: element("armServoInput"),
        value: element("armServoValue"), state: element("armServoState"),
        angleField: "armServoAngleDegrees", pulseField: "armServoPulseUs",
        enabledField: "armServoEnabled", minimumField: "armServoMinimumPulseUs",
        maximumField: "armServoMaximumPulseUs", invertedField: "armServoInverted",
        index: 0, requestedAngleDegrees: null
      },
      wrist: {
        slider: element("wristServoSlider"), input: element("wristServoInput"),
        value: element("wristServoValue"), state: element("wristServoState"),
        angleField: "wristServoAngleDegrees", pulseField: "wristServoPulseUs",
        enabledField: "wristServoEnabled", minimumField: "wristServoMinimumPulseUs",
        maximumField: "wristServoMaximumPulseUs", invertedField: "wristServoInverted",
        index: 1, requestedAngleDegrees: null
      },
      gripper: {
        slider: element("gripperServoSlider"), input: element("gripperServoInput"),
        value: element("gripperServoValue"), state: element("gripperServoState"),
        angleField: "gripperServoAngleDegrees", pulseField: "gripperServoPulseUs",
        enabledField: "gripperServoEnabled", minimumField: "gripperServoMinimumPulseUs",
        maximumField: "gripperServoMaximumPulseUs", invertedField: "gripperServoInverted",
        index: 2, requestedAngleDegrees: null
      }
    };
    const servoCalibrationTarget = element("servoCalibrationTarget");
    const servoCalibrationWorkspace = element("servoCalibrationWorkspace");
    const servoCalibrationPulseSlider = element("servoCalibrationPulseSlider");
    const servoCalibrationPulseInput = element("servoCalibrationPulseInput");
    const servoPulseAtZeroInput = element("servoPulseAtZeroInput");
    const servoPulseAt180Input = element("servoPulseAt180Input");
    const servoCalibrationAdjustmentButtons = Array.from(
      document.querySelectorAll("[data-servo-pulse-delta]"));
    const servoKeepAlivePeriodMs = 500;
    const servoSendTimers = new Map();
    let servoControlAllowed = false;
    let servoCalibrationControlAllowed = false;
    let servoCalibrationOutputEnabled = false;
    let servoCalibrationPulseTimer = null;
    let latestServoTelemetry = null;
    let loadedServoCalibrationProfile = "";
    let requestedLeft = 0;
    let requestedRight = 0;
    let rawDiagnosticDrive = false;
    const manualTestMinimumPower = 0.05;
    let manualMaximumPower = 1.0;
    let activeCameraView = "downward";
    let activeDashboardMode = "operation";
    let latestCameraDiagnosticData = null;
    let cameraRenderGeneration = 0;
    const mountedCameraImages = new Map();
    const mountedCameraStatuses = new Map();
    const mountedCameraModeButtons = [];
    const cameraReconnectTimers = new Map();
    let lastCameraStatusTimestamp = null;
    let lastCameraStatusChangeAtMs = 0;
    let lastCameraMetadataSignature = "";
    // O histórico cobre dez segundos e nunca ultrapassa cem amostras.
    // O intervalo mínimo impede que uma fonte futura mais rápida aumente o custo visual.

    function selectDashboardMode(mode) {
      activeDashboardMode = mode;
      dashboardModePanels.forEach(panel => {
        panel.hidden = panel.dataset.dashboardPanel !== mode;
      });
      dashboardModeButtons.forEach(button => {
        const selected = button.dataset.dashboardMode === mode;
        button.classList.toggle("active", selected);
        button.setAttribute("aria-selected", selected ? "true" : "false");
        button.tabIndex = selected ? 0 : -1;
      });
      renderCurrentCameraDiagnostics();
    }

    function setPill(target, text, state) {
      target.textContent = text;
      target.className = `status-pill ${state || ""}`.trim();
    }

    function setTextIfChanged(target, text) {
      if (target && target.textContent !== text) target.textContent = text;
    }

    function setState(id, ok, goodText = "online", badText = "indisponível") {
      const target = element(id);
      target.textContent = ok ? goodText : badText;
      target.className = ok ? "state-good" : "state-bad";
    }

    function setNeutralState(id, text = "sem telemetria") {
      const target = element(id);
      target.textContent = text;
      target.className = "state-neutral";
    }

    function setTelemetryValue(id, available, value) {
      const target = element(id);
      target.textContent = available ? value : "sem telemetria";
      target.className = available ? "" : "state-neutral";
    }

    function setSubsystemStatus(id, text, state = "neutral") {
      const target = element(id);
      target.textContent = text;
      target.className = `subsystem-status ${state}`;
    }

    function setSubsystemTelemetry(id, available) {
      element(id).dataset.telemetry = available ? "available" : "missing";
    }

    function formatNumber(value, digits = 1) {
      const number = Number(value);
      return Number.isFinite(number) ? number.toFixed(digits) : "--";
    }

    function formatPipelineTiming(value) {
      return typeof value === "number" && Number.isFinite(value) && value >= 0
        ? formatNumber(value, 2)
        : "—";
    }

    function formatUptime(milliseconds) {
      const totalSeconds = Math.max(0, Math.floor(Number(milliseconds) / 1000));
      if (!Number.isFinite(totalSeconds)) return "--";
      const hours = Math.floor(totalSeconds / 3600);
      const minutes = Math.floor((totalSeconds % 3600) / 60);
      const seconds = totalSeconds % 60;
      return `${hours}h ${String(minutes).padStart(2, "0")}m ${String(seconds).padStart(2, "0")}s`;
    }

    function updateMode(data) {
      const mode = String(data.mode || "stopped");
      const labels = { manual: "MANUAL", autonomous: "AUTÔNOMO", stopped: "PARADO", emergency: "EMERGÊNCIA", servo_calibration: "CALIBRAÇÃO DE SERVO" };
      const modeDisplay = element("mode");
      modeDisplay.textContent = labels[mode] || mode.toUpperCase();
      modeDisplay.dataset.mode = mode;
      const selectedMission = String(data.autonomousMission || "main_mission");
      const autonomousDetail = selectedMission === "turn_right_90"
        ? "Executando giro de 90° à direita"
          : selectedMission === "drive_distance"
            ? "Percorrendo a distância selecionada pelos encoders"
          : selectedMission === "rescue_zone_detection"
            ? "Detectando as áreas verde e vermelha; Autônomo mantém os motores parados"
          : selectedMission === "rescue_zone_frame"
            ? `Enquadrando a área ${String(data.rescueZoneTargetColor || "green").toUpperCase()} em passos curtos`
          : selectedMission === "rescue_area"
            ? "Detectando, alinhando e aproximando da vítima mais próxima"
          : selectedMission === "obstacle_avoidance"
            ? "Executando o desvio ultrassônico de obstáculo"
          : selectedMission.startsWith("servo_")
            ? "Executando movimentos sequenciais de braço, pulso e garra"
          : "Missão principal em execução";
      element("modeDetail").textContent = mode === "manual" ? "Comandos humanos habilitados" : mode === "autonomous" ? autonomousDetail : mode === "servo_calibration" ? "Motores zerados · Parar desliga os pulsos" : mode === "emergency" ? "Movimento bloqueado pela trava de segurança" : "Saídas de motor zeradas";
      ["manualButton", "autoButton", "stopButton"].forEach(id => element(id).classList.remove("active"));
      if (mode === "manual") element("manualButton").classList.add("active");
      if (mode === "autonomous") element("autoButton").classList.add("active");
      if (mode === "stopped") element("stopButton").classList.add("active");
      if (mode !== "manual" && manualEnabled) {
        manualEnabled = false;
        resetKeyboardState();
        resetDrive();
      }
      updateKeyboardIndicators();
    }

    function updateAutonomousMission(data) {
      const mission = String(data.autonomousMission || "main_mission");
      if (document.activeElement !== autonomousMission) autonomousMission.value = mission;
      if (document.activeElement !== distanceTargetCm) {
        distanceTargetCm.value = formatNumber(data.driveDistanceTargetCm, 1);
      }
      element("distanceMissionSettings").hidden = mission !== "drive_distance";
      element("rescueZoneFrameSettings").hidden = mission !== "rescue_zone_frame";
      if (document.activeElement !== rescueZoneTargetColor) {
        rescueZoneTargetColor.value = String(data.rescueZoneTargetColor || "green");
      }
      element("servoRoutineSettings").hidden = !mission.startsWith("servo_");
      element("servoRoutineStorageState").textContent =
        data.servoRoutineInternalObjectStored === true
          ? "Objeto registrado no armazenamento interno"
          : "Armazenamento interno vazio";
      const waitingForServoConfirmation =
        data.mode === "autonomous" &&
        data.servoRoutineWaitingForConfirmation === true;
      element("servoRoutineConfirmButton").disabled = !waitingForServoConfirmation;
      element("servoRoutineConfirmButton").textContent =
        data.autonomousPhase === "servo_wait_close"
          ? "Confirmar fechamento da garra"
          : "Confirmar abertura da garra";
      element("missionHint").textContent = mission === "turn_right_90"
        ? "Usa o MPU6050, comando 0,01 com perfil operacional e frenagem preditiva."
          : mission === "drive_distance"
            ? "Avança os dois lados até o alvo medido pelos encoders."
          : mission === "rescue_zone_detection"
            ? "Mantém a percepção ativa no Manual para enquadramento; no Autônomo, a tração permanece zerada."
          : mission === "rescue_zone_frame"
            ? "Tenta melhorar o enquadramento com até três micro-pivôs; aceita best effort, não recua, não alinha nem avança."
          : mission === "rescue_area"
            ? "Preserva o alinhamento e a aproximação para receber as detecções do futuro modelo."
          : mission === "obstacle_avoidance"
            ? "Executa isoladamente a mesma manobra ultrassônica usada no percurso de linha."
          : mission === "servo_initialize"
            ? "Aplica e estabiliza a pose Home: braço 15°, pulso 0° e garra 0°."
          : mission === "servo_capture"
            ? "Executa a coleta e mantém a garra energizada no ângulo de retenção de 5°."
          : mission === "servo_internal_storage"
            ? "Executa os passos de armazenamento interno sem aguardar condições externas."
          : mission === "servo_deposit"
            ? "Executa os passos de depósito diretamente para validação em bancada."
          : mission === "servo_full_sequence"
            ? "Coleta duas vítimas, armazena a primeira e deposita ambas passo a passo."
          : mission === "servo_full_sequence_two"
            ? "Coleta uma vítima e realiza o depósito sem armazenamento interno."
          : "Segue-faixa com retorno verde e desvio ultrassônico de obstáculo.";
    }

    function updateStateMachine(data) {
      const phases = {
        stopped: ["PARADO", "idle", "machineStepFeedback"],
        manual: ["CONTROLE MANUAL", "active", "machineStepDecision"],
        ready: ["MISSÃO PRONTA", "idle", "machineStepDecision"],
        starting: ["INICIALIZANDO", "warn", "machineStepPerception"],
        emergency: ["EMERGÊNCIA", "danger", "machineStepFeedback"],
        calibrating: ["CALIBRANDO", "warn", "machineStepFeedback"],
        servo_calibration: ["CALIBRANDO SERVO", "warn", "machineStepFeedback"],
        servo_initial_pose: ["SERVOS: POSIÇÃO INICIAL", "active", "machineStepMotion"],
        servo_resume_pose: ["SERVOS: REATIVANDO POSE", "warn", "machineStepFeedback"],
        servo_wait_open: ["SERVOS: CONFIRMAR ABERTURA", "warn", "machineStepDecision"],
        servo_wait_stored_open: ["SERVOS: CONFIRMAR ABERTURA", "warn", "machineStepDecision"],
        servo_wait_close: ["SERVOS: CONFIRMAR FECHAMENTO", "warn", "machineStepDecision"],
        servo_confirmation_timeout: ["SERVOS: CONFIRMAÇÃO EXPIRADA", "danger", "machineStepFeedback"],
        servo_driver_lost: ["SERVOS: DRIVER INDISPONÍVEL", "danger", "machineStepFeedback"],
        servo_completed: ["SERVOS: ROTINA CONCLUÍDA", "active", "machineStepFeedback"],
        waiting_esp32: ["ESP32 OFFLINE", "danger", "machineStepFeedback"],
        main_waiting_behaviors: ["ESTRUTURA PRONTA", "idle", "machineStepDecision"],
        waiting_imu: ["AGUARDANDO IMU", "warn", "machineStepPerception"],
        line_follower_pending: ["SEGUE-FAIXA PENDENTE", "warn", "machineStepDecision"],
        line_following: ["SEGUINDO LINHA", "active", "machineStepMotion"],
        turnaround_waiting_sensors: ["RETORNO: SENSORES", "warn", "machineStepPerception"],
        turnaround_forward: ["RETORNO: AVANÇO", "active", "machineStepMotion"],
        turnaround_forward_settling: ["RETORNO: ESTABILIZANDO", "warn", "machineStepFeedback"],
        turnaround_imu: ["RETORNO: GIRO IMU", "active", "machineStepMotion"],
        turnaround_searching_line: ["RETORNO: BUSCANDO LINHA", "active", "machineStepPerception"],
        turnaround_forward_timeout: ["RETORNO: TIMEOUT", "danger", "machineStepFeedback"],
        turnaround_encoder_lost: ["RETORNO: ENCODERS OFFLINE", "danger", "machineStepFeedback"],
        turnaround_line_search_angle_limit: ["RETORNO: LIMITE ANGULAR", "danger", "machineStepFeedback"],
        turnaround_line_search_timeout: ["RETORNO: LINHA AUSENTE", "danger", "machineStepFeedback"],
        turning_right_90: ["GIRO DE 90°", "active", "machineStepMotion"],
        turn_settling: ["ESTABILIZANDO GIRO", "warn", "machineStepFeedback"],
        turn_correction: ["CORRIGINDO GIRO", "active", "machineStepMotion"],
        completed: ["MISSÃO CONCLUÍDA", "active", "machineStepFeedback"],
        turn_timeout: ["TEMPO LIMITE", "danger", "machineStepFeedback"],
        turn_imu_lost: ["IMU PERDIDA", "danger", "machineStepFeedback"],
        turn_correction_failed: ["CORREÇÃO INSUFICIENTE", "danger", "machineStepFeedback"],
        waiting_encoders: ["AGUARDANDO ENCODERS", "warn", "machineStepPerception"],
        driving_distance: ["PERCORRENDO DISTÂNCIA", "active", "machineStepMotion"],
        distance_settling: ["ESTABILIZANDO DISTÂNCIA", "warn", "machineStepFeedback"],
        distance_correction: ["CORRIGINDO DISTÂNCIA", "active", "machineStepMotion"],
        distance_completed: ["DISTÂNCIA CONCLUÍDA", "active", "machineStepFeedback"],
        distance_timeout: ["TEMPO LIMITE", "danger", "machineStepFeedback"],
        distance_encoder_lost: ["ENCODERS OFFLINE", "danger", "machineStepFeedback"],
        distance_encoder_stall: ["SEM AVANÇO", "danger", "machineStepFeedback"],
        distance_encoder_mismatch: ["LADOS DESBALANCEADOS", "danger", "machineStepFeedback"],
        distance_correction_failed: ["CORREÇÃO INSUFICIENTE", "danger", "machineStepFeedback"],
        distance_invalid_target: ["ALVO INVÁLIDO", "danger", "machineStepFeedback"],
        obstacle_detected: ["OBSTÁCULO CONFIRMADO", "warn", "machineStepFeedback"],
        obstacle_waiting_sensors: ["DESVIO: SENSORES", "warn", "machineStepPerception"],
        obstacle_settling: ["DESVIO: ESTABILIZANDO", "warn", "machineStepFeedback"],
        obstacle_turning: ["DESVIO: GIRANDO", "active", "machineStepMotion"],
        obstacle_turn_settling: ["DESVIO: ESTABILIZANDO GIRO", "warn", "machineStepFeedback"],
        obstacle_turn_correction: ["DESVIO: CORRIGINDO GIRO", "active", "machineStepMotion"],
        obstacle_first_forward_start: ["DESVIO: RETA 1", "active", "machineStepMotion"],
        obstacle_first_forward: ["DESVIO: RETA 1", "active", "machineStepMotion"],
        obstacle_second_forward_start: ["DESVIO: RETA 2", "active", "machineStepMotion"],
        obstacle_second_forward: ["DESVIO: RETA 2", "active", "machineStepMotion"],
        obstacle_third_forward_start: ["DESVIO: RETA 3", "active", "machineStepMotion"],
        obstacle_third_forward: ["DESVIO: RETA 3", "active", "machineStepMotion"],
        obstacle_reverse_start: ["DESVIO: RÉ", "active", "machineStepMotion"],
        obstacle_reversing: ["DESVIO: RÉ", "active", "machineStepMotion"],
        obstacle_stage_completed: ["DESVIO: ETAPA CONCLUÍDA", "warn", "machineStepFeedback"],
        obstacle_completed: ["DESVIO CONCLUÍDO", "active", "machineStepFeedback"],
        obstacle_encoder_lost: ["DESVIO: ENCODERS OFFLINE", "danger", "machineStepFeedback"],
        obstacle_distance_timeout: ["DESVIO: TIMEOUT", "danger", "machineStepFeedback"],
        obstacle_turn_start_failed: ["DESVIO: FALHA NO GIRO", "danger", "machineStepFeedback"],
        obstacle_turn_timeout: ["DESVIO: TIMEOUT DO GIRO", "danger", "machineStepFeedback"],
        obstacle_turn_imu_lost: ["DESVIO: IMU PERDIDA", "danger", "machineStepFeedback"],
        obstacle_turn_correction_failed: ["DESVIO: CORREÇÃO INSUFICIENTE", "danger", "machineStepFeedback"],
        rescue_esp32_not_ready: ["RESGATE: ESP32 OFFLINE", "danger", "machineStepFeedback"],
        rescue_zone_detection_active: ["ÁREAS: DETECÇÃO ATIVA", "active", "machineStepPerception"],
        rescue_zone_frame_waiting_vision: ["FRAME_ZONE: AGUARDANDO CAM1", "warn", "machineStepPerception"],
        rescue_zone_frame_waiting_zone: ["FRAME_ZONE: AGUARDANDO ZONA", "warn", "machineStepPerception"],
        rescue_zone_frame_turning: ["FRAME_ZONE: MICRO-PIVÔ", "active", "machineStepMotion"],
        rescue_zone_frame_braking: ["FRAME_ZONE: FRENANDO", "warn", "machineStepFeedback"],
        rescue_zone_frame_settling: ["FRAME_ZONE: ESTABILIZANDO", "warn", "machineStepFeedback"],
        rescue_zone_frame_waiting_new_frame: ["FRAME_ZONE: NOVO FRAME", "warn", "machineStepPerception"],
        rescue_zone_frame_completed: ["FRAME_ZONE CONCLUÍDO", "active", "machineStepFeedback"],
        rescue_zone_frame_failed: ["FRAME_ZONE FALHOU", "danger", "machineStepFeedback"],
        ball_alignment_waiting_target: ["RESGATE: PROCURANDO VÍTIMA", "warn", "machineStepPerception"],
        ball_alignment_camera_stale: ["RESGATE: VISÃO OFFLINE", "danger", "machineStepFeedback"],
        ball_alignment_target_lost: ["RESGATE: ALVO PERDIDO", "warn", "machineStepPerception"],
        ball_alignment_target_lost_timeout: ["RESGATE: TIMEOUT DO ALVO", "danger", "machineStepFeedback"],
        ball_alignment_turning: ["RESGATE: ALINHANDO", "active", "machineStepMotion"],
        ball_alignment_braking: ["RESGATE: ESTABILIZANDO", "warn", "machineStepFeedback"],
        ball_alignment_verifying: ["RESGATE: CONFIRMANDO", "warn", "machineStepPerception"],
        ball_alignment_fine_correction: ["RESGATE: CORREÇÃO FINA", "active", "machineStepMotion"],
        ball_approach_waiting_distance: ["RESGATE: SEM DISTÂNCIA", "warn", "machineStepPerception"],
        ball_approaching: ["RESGATE: APROXIMANDO", "active", "machineStepMotion"],
        ball_reached: ["RESGATE: VÍTIMA ALCANÇADA", "active", "machineStepFeedback"]
      };
      let phase = String(data.autonomousPhase || "stopped");
      let action = String(data.autonomousAction || "Aguardando estado da missão");
      if (data.emergency === true || data.esp32EmergencyStop === true) {
        phase = "emergency";
        action = "Movimento bloqueado pela parada de emergência";
      } else if (data.esp32CalibrationActive === true) {
        phase = "calibrating";
        action = "Calibrando sensores: mantenha o robô parado";
      } else if (data.servoCalibrationRequested === true || data.servoCalibrationActive === true) {
        phase = "servo_calibration";
        action = "Ajustando pulsos de servo: motores parados; Parar desliga a saída";
      } else if (data.mode === "autonomous" && data.esp32SerialOpen !== true) {
        phase = "waiting_esp32";
        action = "Missão sem comunicação com a ESP32: motores protegidos";
      }
      const rescueZoneFrameFailure =
        data.rescueZoneFrameFailed === true && phase.startsWith("rescue_zone_frame_");
      const phaseInfo = rescueZoneFrameFailure
        ? ["FRAME_ZONE FALHOU", "danger", "machineStepFeedback"]
        : phases[phase] || [phase.replaceAll("_", " ").toUpperCase(), "idle", "machineStepDecision"];
      const card = element("missionStateCard");
      card.dataset.tone = phaseInfo[1];
      element("machineStateBadge").textContent = phaseInfo[0];
      element("machineAction").textContent = action;

      ["machineStepPerception", "machineStepDecision", "machineStepMotion", "machineStepFeedback"]
        .forEach(id => element(id).classList.toggle("active", id === phaseInfo[2]));

      const progress = Math.max(0, Math.min(100, Number(data.autonomousProgressPercent) || 0));
      element("machineProgressValue").textContent = `${formatNumber(progress, 0)}%`;
      element("machineProgressFill").style.width = `${progress}%`;

      const mission = String(data.autonomousMission || "main_mission");
      element("machineMission").textContent = mission === "turn_right_90"
        ? "Giro 90° à direita"
          : mission === "drive_distance"
            ? "Percorrer distância"
          : mission === "rescue_zone_detection" ? "Áreas de Resgate"
          : mission === "rescue_zone_frame" ? `FRAME_ZONE · ${String(data.rescueZoneTargetColor || "green").toUpperCase()}`
          : mission === "rescue_area" ? "Resgate · Detectar e seguir"
          : mission === "obstacle_avoidance" ? "Desvio de obstáculo"
          : mission === "servo_initialize" ? "Servos · Inicialização"
          : mission === "servo_capture" ? "Servos · Captura"
          : mission === "servo_internal_storage" ? "Servos · Armazenamento"
          : mission === "servo_deposit" ? "Servos · Depósito"
          : mission === "servo_full_sequence" ? "Servos · Com armazenamento"
          : mission === "servo_full_sequence_two" ? "Servos · Sem armazenamento"
          : "Missão principal";
      const turnAroundActive = phase.startsWith("turnaround_");
      const obstacleActive = phase.startsWith("obstacle_");
      element("machineBehavior").textContent = mission === "turn_right_90"
        ? "TESTE DE GIRO"
          : mission === "drive_distance"
            ? "TESTE DE DISTÂNCIA"
          : mission === "rescue_zone_detection"
            ? "DETECÇÃO DE ÁREAS"
          : mission === "rescue_zone_frame"
            ? "ENQUADRAMENTO DE ÁREA"
          : mission === "rescue_area"
            ? "ALINHAMENTO DE VÍTIMA"
          : mission === "obstacle_avoidance"
            ? "TESTE DE OBSTÁCULO"
          : mission.startsWith("servo_")
            ? "ROTINA DE SERVOS"
          : obstacleActive ? "DESVIO DE OBSTÁCULO"
          : turnAroundActive ? "RETORNO 180°" : "SEGUE-LINHA";
      element("machineRequestedSpeed").textContent = `${formatNumber(data.left, 2)} / ${formatNumber(data.right, 2)}`;
      const fresh = data.esp32SensorFresh === true;
      element("machineAppliedSpeed").textContent = fresh
        ? `${formatNumber(data.esp32AppliedLeftPower, 2)} / ${formatNumber(data.esp32AppliedRightPower, 2)}`
        : "-- / --";
      setTextIfChanged(
        element("machineEncoderSpeed"),
        fresh
          ? `${formatNumber(data.leftEncoderRate, 0)} / ${formatNumber(data.rightEncoderRate, 0)} cont/s`
          : "-- / -- cont/s"
      );
      const distanceMission = mission === "drive_distance";
      element("machineDistanceTarget").textContent = distanceMission ? `${formatNumber(data.driveDistanceTargetCm, 1)} cm` : "-- cm";
      element("machineDistanceSides").textContent = distanceMission
        ? `${formatNumber(data.autonomousLeftDistanceCm, 1)} / ${formatNumber(data.autonomousRightDistanceCm, 1)} cm`
        : "-- / -- cm";
      element("machineDistanceAverage").textContent = distanceMission ? `${formatNumber(data.autonomousAverageDistanceCm, 1)} cm` : "-- cm";
      element("machineEncoderCalibration").textContent = distanceMission ? `${formatNumber(data.encoderCountsPerCentimeter, 2)} cont/cm` : "-- cont/cm";
      const modeLabels = { manual: "MANUAL", autonomous: "AUTÔNOMO", stopped: "PARADO", emergency: "EMERGÊNCIA", servo_calibration: "CALIBRAÇÃO DE SERVO" };
      element("machineMode").textContent = modeLabels[data.mode] || String(data.mode || "--").toUpperCase();
    }

    function updateSystemMetric(metricId, fillId, value, warningValue, dangerValue) {
      const safeValue = Number.isFinite(Number(value)) ? Number(value) : 0;
      const metric = element(metricId);
      metric.dataset.level = safeValue >= dangerValue ? "danger" : (safeValue >= warningValue ? "warn" : "normal");
      element(fillId).style.width = `${Math.max(0, Math.min(100, safeValue))}%`;
    }

    function updateDiagnosticCommunicationState(data, fresh, serialOpen, calibrating, mpuOk, localEmergency, age) {
      const alert = element("esp32CommunicationAlert");
      alert.hidden = fresh;
      element("esp32CommunicationAlertTitle").textContent = serialOpen
        ? "TELEMETRIA DA ESP32 DESATUALIZADA"
        : "ESP32 OFFLINE";
      element("esp32CommunicationAlertAge").textContent = Number.isFinite(age) && age >= 0
        ? `Sem telemetria há ${formatNumber(age, 0)} ms`
        : "Sem telemetria recente";

      const communicationText = calibrating && fresh
        ? "calibrando"
        : fresh ? "online" : serialOpen ? "sem telemetria" : "offline";
      const communicationState = calibrating && fresh
        ? "warn"
        : fresh ? "ok" : serialOpen ? "warn" : "danger";
      setSubsystemStatus("communicationSubsystemStatus", communicationText, communicationState);
      setSubsystemTelemetry("communicationSubsystem", true);

      const diagnosticEsp32Status = element("diagnosticEsp32Status");
      diagnosticEsp32Status.textContent = calibrating && fresh
        ? "CALIBRANDO"
        : fresh ? "ONLINE" : serialOpen ? "SEM TELEMETRIA" : "OFFLINE";
      diagnosticEsp32Status.className = calibrating && fresh
        ? "state-warn"
        : fresh ? "state-good" : serialOpen ? "state-warn" : "state-bad";

      ["tractionSubsystem", "encoderSubsystem", "imuSubsystem", "sensorSubsystem", "peripheralSubsystem"]
        .forEach(id => setSubsystemTelemetry(id, fresh));

      if (!fresh) {
        setSubsystemStatus("tractionSubsystemStatus", "sem telemetria");
        setSubsystemStatus("encoderSubsystemStatus", "sem telemetria");
        setSubsystemStatus("imuSubsystemStatus", "sem telemetria");
        setSubsystemStatus("sensorFreshState", "sem telemetria");
        setSubsystemStatus("peripheralSubsystemStatus", "sem telemetria");
        return;
      }

      setSubsystemStatus("tractionSubsystemStatus", localEmergency ? "E-STOP ativo" : "online", localEmergency ? "danger" : "ok");
      setSubsystemStatus("encoderSubsystemStatus", "online", "ok");
      setSubsystemStatus("imuSubsystemStatus", calibrating ? "calibrando" : mpuOk ? "online" : "verificar", calibrating ? "warn" : mpuOk ? "ok" : "danger");
      setSubsystemStatus("sensorFreshState", calibrating ? "calibrando" : "online", calibrating ? "warn" : "ok");
      const peripheralsOk = data.pca9685Ok === true && data.oledOk === true;
      setSubsystemStatus("peripheralSubsystemStatus", peripheralsOk ? "online" : "verificar", peripheralsOk ? "ok" : "warn");
    }

    function updateEsp32Telemetry(data) {
      const receivedMinimumPower = Number(data.motorStartMinimumPower);
      const receivedMaximumPower = Number(data.operationalMaximumReferencePower);
      if (Number.isFinite(receivedMinimumPower) && Number.isFinite(receivedMaximumPower) &&
          receivedMinimumPower > 0 && receivedMaximumPower >= receivedMinimumPower) {
        manualMaximumPower = receivedMaximumPower;
        manualDrivePower.min = manualTestMinimumPower.toFixed(2);
        manualDrivePower.max = manualMaximumPower.toFixed(2);
        manualTurnPower.min = manualTestMinimumPower.toFixed(2);
        manualTurnPower.max = manualMaximumPower.toFixed(2);
        manualDrivePower.value = clampManualPower(manualDrivePower.value).toFixed(2);
        manualTurnPower.value = clampManualPower(manualTurnPower.value).toFixed(2);
        element("manualDrivePowerValue").textContent = manualDrivePower.value;
        element("manualTurnPowerValue").textContent = manualTurnPower.value;
      }
      const fresh = data.esp32SensorFresh === true;
      const serialOpen = data.esp32SerialOpen === true;
      const calibrating = data.esp32CalibrationActive === true;
      const servoCalibrating = data.servoCalibrationRequested === true ||
        data.servoCalibrationActive === true;
      const mpuOk = fresh && !calibrating && data.mpuOk === true;
      const localEmergency = data.esp32EmergencyStop === true;
      const systemEmergency = data.emergency === true || localEmergency;
      const battery = Number(data.batteryVoltage);
      const distance = Number(data.ultrasonicDistanceCm);
      const age = Number(data.esp32LastSensorAgeMs);

      updateDiagnosticCommunicationState(data, fresh, serialOpen, calibrating, mpuOk, localEmergency, age);

      setPill(element("esp32Link"), calibrating ? "ESP32 CALIBRANDO SENSORES" : (servoCalibrating ? "ESP32 CALIBRANDO SERVO" : (serialOpen ? (fresh ? "ESP32 SINCRONIZADA" : "ESP32 SEM TELEMETRIA") : "ESP32 OFFLINE")), (calibrating || servoCalibrating) ? "warn" : (fresh ? "ok" : (serialOpen ? "warn" : "danger")));
      setPill(element("safetyStatus"), systemEmergency ? "TRAVA DE SEGURANÇA ATIVA" : ((calibrating || servoCalibrating) ? "CALIBRAÇÃO · MOTORES PARADOS" : (data.mode === "stopped" ? "ROBÔ PARADO" : "MOVIMENTO AUTORIZADO")), systemEmergency ? "danger" : ((calibrating || servoCalibrating || data.mode === "stopped") ? "warn" : "ok"));
      const esp32Status = element("esp32Status");
      esp32Status.textContent = calibrating ? "CALIBRANDO SENSORES" : (servoCalibrating ? "CALIBRANDO SERVO" : (fresh ? "ONLINE" : (serialOpen ? "SEM DADOS" : "OFFLINE")));
      esp32Status.className = (calibrating || servoCalibrating) ? "status-warn" : (fresh ? "status-good" : (serialOpen ? "status-warn" : "status-bad"));
      element("telemetryAge").textContent = age >= 0 ? `${formatNumber(age, 0)} ms` : "-- ms";
      element("serialState").textContent = serialOpen ? "aberta" : "fechada";
      element("serialState").className = serialOpen ? "state-good" : "state-bad";
      element("readyLedState").textContent = data.systemReady === true ? "aceso · sistema pronto" : "apagado · aguardando";
      element("readyLedState").className = data.systemReady === true ? "state-good" : "state-warn";
      element("raspberryCommandTimeout").textContent = `${formatNumber(data.raspberryCommandTimeoutMs, 0)} ms`;
      element("esp32MotorTimeout").textContent = `${formatNumber(data.esp32MotorCommandTimeoutMs, 0)} ms`;

      element("batteryVoltage").textContent = fresh ? `${formatNumber(battery, 2)} V` : "--.-- V";
      setTelemetryValue("batteryAdc", fresh, `${formatNumber(data.batteryAdcMillivolts, 0)} mV`);
      const batteryPercent = fresh ? Math.max(0, Math.min(1, (battery - 10.5) / 3.5)) : 0;
      element("batteryFill").style.width = `${batteryPercent * 100}%`;

      setTelemetryValue("diagnosticUptime", fresh, formatUptime(data.esp32UptimeMs));
      element("requestedLeft").textContent = formatNumber(data.requestedLeft, 2);
      element("requestedRight").textContent = formatNumber(data.requestedRight, 2);
      setTelemetryValue("commandedLeft", fresh, formatNumber(data.commandedLeft, 2));
      setTelemetryValue("commandedRight", fresh, formatNumber(data.commandedRight, 2));
      const motorCommandProfile = element("motorCommandProfile");
      motorCommandProfile.textContent = data.rawMotorCommand === true
        ? "ajuste individual direto · sem piso operacional e sem sincronismo"
        : `partida ${formatNumber(data.motorStartMinimumPower, 2)} · execução ${formatNumber(data.motorRunMinimumPower, 2)} · sincronismo somente em reta`;
      motorCommandProfile.className = data.rawMotorCommand === true ? "state-warn" : "state-good";
      setTelemetryValue("appliedLeft", fresh, formatNumber(data.appliedLeft, 2));
      setTelemetryValue("appliedRight", fresh, formatNumber(data.appliedRight, 2));
      const motorMotionState = element("motorMotionState");
      const leftMotion = data.leftMotorRunning === true ? "RUNNING" : (data.leftMotorStarting === true ? "STARTING" : "STOPPED");
      const rightMotion = data.rightMotorRunning === true ? "RUNNING" : (data.rightMotorStarting === true ? "STARTING" : "STOPPED");
      const requiredMotionSamples = Number(data.motorRunConfirmationRequiredSamples) || 0;
      motorMotionState.textContent = fresh
        ? `E ${leftMotion} (${Number(data.leftMotorConfirmationSamples) || 0}/${requiredMotionSamples}) · D ${rightMotion} (${Number(data.rightMotorConfirmationSamples) || 0}/${requiredMotionSamples})`
        : "sem telemetria";
      motorMotionState.className = fresh && leftMotion === "RUNNING" && rightMotion === "RUNNING" ? "state-good" : "state-warn";
      element("leftMotorBar").style.width = fresh ? `${Math.min(100, Math.abs(Number(data.esp32AppliedLeftPower)) * 100)}%` : "0%";
      element("rightMotorBar").style.width = fresh ? `${Math.min(100, Math.abs(Number(data.esp32AppliedRightPower)) * 100)}%` : "0%";
      if (fresh) {
        setState("sleepState", data.motorSleepPinHigh === true, "HIGH · habilitado", "LOW · verificar");
        setState("esp32Estop", !localEmergency, "liberado", "ATIVO");
      } else {
        setNeutralState("sleepState");
        setNeutralState("esp32Estop");
      }

      setTelemetryValue("leftEncoderCount", fresh, formatNumber(data.leftEncoderCount, 0));
      setTelemetryValue("rightEncoderCount", fresh, formatNumber(data.rightEncoderCount, 0));
      setTelemetryValue("leftEncoderRate", fresh, `${formatNumber(data.leftEncoderRate, 0)} cont/s`);
      setTelemetryValue("rightEncoderRate", fresh, `${formatNumber(data.rightEncoderRate, 0)} cont/s`);
      const countsPerCentimeter = Number(data.encoderCountsPerCentimeter);
      const encoderScaleValid = fresh && Number.isFinite(countsPerCentimeter) && countsPerCentimeter > 0;
      setTelemetryValue("leftEncoderPosition", fresh, encoderScaleValid
        ? `${formatNumber(Number(data.leftEncoderCount) / countsPerCentimeter, 1)} cm`
        : "escala indisponível");
      setTelemetryValue("rightEncoderPosition", fresh, encoderScaleValid
        ? `${formatNumber(Number(data.rightEncoderCount) / countsPerCentimeter, 1)} cm`
        : "escala indisponível");
      setTelemetryValue("machineEncoderCalibration", fresh, encoderScaleValid
        ? `${formatNumber(countsPerCentimeter, 2)} cont/cm`
        : "escala indisponível");

      const imuUnavailableText = calibrating ? "calibrando" : "indisponível";
      setTelemetryValue("yawZ", fresh, mpuOk ? `${formatNumber(data.yawZDeg, 1)} °` : imuUnavailableText);
      setTelemetryValue("rampAngle", fresh, mpuOk ? `${formatNumber(data.rampAngleDeg, 1)} °` : imuUnavailableText);
      setTelemetryValue("gyroX", fresh, mpuOk ? formatNumber(data.gyroXDegPerSec, 1) : imuUnavailableText);
      setTelemetryValue("gyroY", fresh, mpuOk ? formatNumber(data.gyroYDegPerSec, 1) : imuUnavailableText);
      setTelemetryValue("gyroZ", fresh, mpuOk ? formatNumber(data.gyroZDegPerSec, 1) : imuUnavailableText);
      setTelemetryValue("accelX", fresh, mpuOk ? formatNumber(data.accelX, 1) : imuUnavailableText);
      setTelemetryValue("accelY", fresh, mpuOk ? formatNumber(data.accelY, 1) : imuUnavailableText);
      setTelemetryValue("accelZ", fresh, mpuOk ? formatNumber(data.accelZ, 1) : imuUnavailableText);
      setTelemetryValue("imuTemperature", fresh, mpuOk ? `${formatNumber(data.imuTemperatureCelsius, 1)} °C` : imuUnavailableText);

      if (fresh) {
        element("ultrasonic").textContent = distance >= 0 ? `${formatNumber(distance, 1)} cm` : "sem eco";
        element("ultrasonic").className = distance >= 0 ? "state-good" : "state-warn";
        setState("startButtonState", data.startButtonPressed === true, "pressionado", "solto");
        if (data.startButtonPressed !== true) element("startButtonState").className = "";
      } else {
        setNeutralState("ultrasonic");
        setNeutralState("startButtonState");
      }
      const calibrationButton = element("calibrationButton");
      calibrationButton.disabled = !fresh || calibrating || servoCalibrating;
      calibrationButton.textContent = calibrating ? "Calibrando…" : "Calibrar";
      if (calibrating) {
        element("calibrationState").textContent = "em andamento";
        element("calibrationState").className = "state-warn";
      } else if (fresh && data.esp32CalibrationStatusKnown === true) {
        const calibrationOk = data.esp32LastCalibrationSucceeded === true;
        element("calibrationState").textContent = calibrationOk ? "concluída" : "falhou";
        element("calibrationState").className = calibrationOk ? "state-good" : "state-bad";
      } else {
        if (fresh) {
          element("calibrationState").textContent = "aguardando";
          element("calibrationState").className = "";
        } else {
          setNeutralState("calibrationState");
        }
      }
      if (fresh) {
        setState("mpuState", mpuOk, "online", calibrating ? "calibrando" : "indisponível");
        if (calibrating) element("mpuState").className = "state-warn";
        setState("pcaState", data.pca9685Ok === true, "online", "indisponível");
        setState("oledState", data.oledOk === true, "online", "indisponível");
      } else {
        setNeutralState("mpuState");
        setNeutralState("pcaState");
        setNeutralState("oledState");
      }
      updateServoControls(data, fresh, calibrating, systemEmergency);
      updateServoCalibration(data, fresh, calibrating, systemEmergency);
      const oledAvailable = fresh && data.oledOk === true;
      const oledBootReady = oledAvailable && data.esp32RaspberrySystemReady === true;
      const remoteOledActive = oledBootReady && data.esp32RemoteOledActive === true;
      const receivedOledAlertDurationMs = Number(
        data.oledNavigationAlertDurationMs);
      if (Number.isFinite(receivedOledAlertDurationMs) &&
          receivedOledAlertDurationMs >= 500 &&
          receivedOledAlertDurationMs <= 30000) {
        oledAutomaticAlertDurationMs = receivedOledAlertDurationMs;
      }
      const oledCanReceive = oledBootReady && !systemEmergency && !calibrating;
      const oledStateLabel = !fresh
        ? "SEM TELEMETRIA"
        : !oledAvailable
          ? "INDISPONÍVEL"
          : systemEmergency
            ? "EMERGÊNCIA"
            : calibrating
              ? "CALIBRANDO"
              : !oledBootReady
                ? "INICIALIZANDO"
                : remoteOledActive ? "MENSAGEM REMOTA" : "TELA PADRÃO";
      const oledStateClass = !fresh
        ? "state-neutral"
        : !oledAvailable || systemEmergency
          ? "state-bad"
          : calibrating || !oledBootReady
            ? "state-warn"
            : "state-good";
      const oledCurrentState = element("oledCurrentState");
      const oledEditorSummaryState = element("oledEditorSummaryState");
      oledCurrentState.textContent = oledStateLabel;
      oledCurrentState.className = oledStateClass;
      oledEditorSummaryState.textContent = oledStateLabel;
      oledEditorSummaryState.className = `oled-summary-state ${oledStateClass}`;
      oledSendButtons.forEach(button => { button.disabled = !oledCanReceive; });
      element("oledClearButton").disabled = !oledAvailable || !remoteOledActive;
      element("oledRemoteStatus").textContent = !fresh
        ? "Sem telemetria da ESP32."
        : !oledAvailable
        ? "OLED indisponível."
        : systemEmergency
          ? "A página local de emergência tem prioridade sobre qualquer mensagem remota."
        : calibrating
          ? "A calibração dos sensores controla a OLED até terminar."
        : !oledBootReady
          ? "Inicializando Raspberry, câmera e serviços do robô."
        : remoteOledActive
          ? "Mensagem da Raspberry em exibição; depois do prazo, a tela padrão retorna."
          : "Tela padrão ativa: bateria, giro e inclinação.";
      element("oledRemoteStatus").className = `oled-editor-state ${oledStateClass}`;
      element("oledBootState").textContent = !fresh
        ? "sem telemetria"
        : !oledAvailable ? "indisponível"
        : oledBootReady ? "concluída · tela liberada" : "em andamento · animação ativa";
      element("oledBootState").className = !fresh ? "state-neutral" : oledBootReady ? "state-good" : "state-warn";
      element("oledRemoteTelemetry").textContent = !fresh
        ? "sem telemetria"
        : !oledAvailable ? "indisponível"
        : !oledBootReady ? "bloqueada durante o boot" : remoteOledActive ? "ativa · temporária" : "inativa · tela local";
      element("oledRemoteTelemetry").className = !fresh
        ? "state-neutral" : !oledAvailable ? "state-warn" : remoteOledActive ? "state-good" : "";
    }

    function connect() {
      ws = new WebSocket(`ws://${location.host}/ws`);
      ws.onopen = () => setPill(connection, "PAINEL ONLINE", "ok");
      ws.onclose = () => { manualEnabled = false; resetKeyboardState(); resetDrive(); disableServoControlsLocally(); stopServoCalibrationLocally(); setPill(connection, "PAINEL OFFLINE", "danger"); window.setTimeout(connect, 1000); };
      ws.onmessage = event => {
        const data = JSON.parse(event.data);
        updateMode(data);
        updateAutonomousMission(data);
        updateStateMachine(data);
        updateForwardAssistDiagnostic(data);
        element("cpu").textContent = `${formatNumber(data.cpu, 1)}%`;
        element("temp").textContent = `${formatNumber(data.temperature, 1)} °C`;
        element("ram").textContent = `${formatNumber(data.ram, 1)}%`;
        updateSystemMetric("cpuMetric", "cpuFill", data.cpu, 70, 90);
        updateSystemMetric("ramMetric", "ramFill", data.ram, 75, 90);
        updateSystemMetric("temperatureMetric", "temperatureFill", data.temperature, 65, 80);
        updateEsp32Telemetry(data);
      };
    }

    function send(payload) {
      if (!ws || ws.readyState !== WebSocket.OPEN) return false;
      ws.send(JSON.stringify(payload));
      return true;
    }

    function clampServoAngle(value) {
      return Math.round(Math.max(0, Math.min(180, Number(value) || 0)));
    }

    function showRequestedServoAngle(servoName, value, rememberRequest = true) {
      const control = servoControls[servoName];
      if (!control) return 0;
      const angleDegrees = clampServoAngle(value);
      if (rememberRequest) control.requestedAngleDegrees = angleDegrees;
      control.slider.value = String(angleDegrees);
      control.input.value = String(angleDegrees);
      control.value.textContent = `${angleDegrees}°`;
      return angleDegrees;
    }

    function sendServoAngle(servoName, value, immediate = false) {
      const angleDegrees = showRequestedServoAngle(servoName, value);
      if (!servoControlAllowed) return;

      const pendingTimer = servoSendTimers.get(servoName);
      if (pendingTimer) window.clearTimeout(pendingTimer);
      const transmit = () => {
        servoSendTimers.delete(servoName);
        send({ command: "servo_angle", servo: servoName, angleDegrees });
      };
      if (immediate) transmit();
      else servoSendTimers.set(servoName, window.setTimeout(transmit, 50));
    }

    function disableAllServos() {
      servoSendTimers.forEach(timer => window.clearTimeout(timer));
      servoSendTimers.clear();
      send({ command: "servo_disable_all" });
    }

    function disableServoControlsLocally() {
      servoControlAllowed = false;
      Object.values(servoControls).forEach(control => {
        control.slider.disabled = true;
        control.input.disabled = true;
      });
      element("disableServosButton").disabled = true;
      setSubsystemStatus("servoControlStatus", "bloqueado", "warn");
    }

    function updateServoControls(data, fresh, calibrating, systemEmergency) {
      const pcaReady = fresh && data.pca9685Ok === true;
      const raspberryReady = data.esp32RaspberrySystemReady === true;
      const servoCalibrating = data.servoCalibrationRequested === true ||
        data.servoCalibrationActive === true;
      servoControlAllowed = pcaReady && raspberryReady &&
        data.mode === "manual" && !calibrating && !servoCalibrating &&
        !systemEmergency;
      Object.entries(servoControls).forEach(([servoName, control]) => {
        control.slider.disabled = !servoControlAllowed;
        control.input.disabled = !servoControlAllowed;
        const angleDegrees = clampServoAngle(data[control.angleField]);
        const userEditing = document.activeElement === control.slider ||
          document.activeElement === control.input || servoSendTimers.has(servoName);
        const enabled = fresh && data[control.enabledField] === true;
        control.outputEnabled = enabled;
        if (!servoControlAllowed || !enabled) {
          control.requestedAngleDegrees = null;
        }
        // Durante a rampa, angleDegrees é a referência intermediária aplicada.
        // Copiá-la para o campo faria o keepalive substituir o alvo do operador.
        if (!userEditing && control.requestedAngleDegrees === null) {
          showRequestedServoAngle(servoName, angleDegrees, false);
        }
        const pulseUs = Math.max(0, Number(data[control.pulseField]) || 0);
        control.state.textContent = enabled
          ? `Saída ativa · pedido ${clampServoAngle(control.input.value)}° · aplicado ${angleDegrees}° · ${pulseUs.toFixed(0)} µs.`
          : `Sinal desligado · último alvo ${angleDegrees}°.`;
        control.state.className = `servo-output-state ${enabled ? "state-good" : "state-neutral"}`;
      });
      element("disableServosButton").disabled = !pcaReady || servoCalibrating;

      if (!fresh) setSubsystemStatus("servoControlStatus", "sem telemetria", "warn");
      else if (!pcaReady) setSubsystemStatus("servoControlStatus", "PCA indisponível", "danger");
      else if (systemEmergency) setSubsystemStatus("servoControlStatus", "E-STOP ativo", "danger");
      else if (calibrating) setSubsystemStatus("servoControlStatus", "calibrando", "warn");
      else if (servoCalibrating) setSubsystemStatus("servoControlStatus", "calibração de pulso", "warn");
      else if (!raspberryReady) setSubsystemStatus("servoControlStatus", "aguardando sistema", "warn");
      else if (data.mode !== "manual") setSubsystemStatus("servoControlStatus", "ative Manual", "warn");
      else setSubsystemStatus("servoControlStatus", "controle liberado", "ok");
    }

    function servoCalibrationPulseBounds() {
      const minimum = Number(latestServoTelemetry?.servoCalibrationAbsoluteMinimumPulseUs);
      const maximum = Number(latestServoTelemetry?.servoCalibrationAbsoluteMaximumPulseUs);
      return {
        minimum: Number.isFinite(minimum) ? minimum : 500,
        maximum: Number.isFinite(maximum) ? maximum : 2500
      };
    }

    function clampServoCalibrationPulse(value) {
      const bounds = servoCalibrationPulseBounds();
      return Math.round(Math.max(bounds.minimum, Math.min(bounds.maximum, Number(value) || 1500)));
    }

    function showServoCalibrationPulse(value) {
      const pulseUs = clampServoCalibrationPulse(value);
      servoCalibrationPulseSlider.value = String(pulseUs);
      servoCalibrationPulseInput.value = String(pulseUs);
      element("servoCalibrationPulseValue").textContent = `${pulseUs} µs`;
      return pulseUs;
    }

    function selectedServoCalibrationControl() {
      return servoControls[servoCalibrationTarget.value] || servoControls.arm;
    }

    function loadSelectedServoCalibration(force = false) {
      if (!latestServoTelemetry) return;
      const control = selectedServoCalibrationControl();
      const minimumPulseUs = Number(latestServoTelemetry[control.minimumField]);
      const maximumPulseUs = Number(latestServoTelemetry[control.maximumField]);
      if (!Number.isFinite(minimumPulseUs) || !Number.isFinite(maximumPulseUs) ||
          maximumPulseUs <= minimumPulseUs) return;

      const inverted = latestServoTelemetry[control.invertedField] === true;
      const pulseAtZeroUs = inverted ? maximumPulseUs : minimumPulseUs;
      const pulseAt180Us = inverted ? minimumPulseUs : maximumPulseUs;
      const profileSignature = `${servoCalibrationTarget.value}:${pulseAtZeroUs}:${pulseAt180Us}`;
      if (!force && profileSignature === loadedServoCalibrationProfile) return;
      const editingEndpoints = document.activeElement === servoPulseAtZeroInput ||
        document.activeElement === servoPulseAt180Input;
      if (force || !editingEndpoints) {
        servoPulseAtZeroInput.value = String(Math.round(pulseAtZeroUs));
        servoPulseAt180Input.value = String(Math.round(pulseAt180Us));
      }
      if (force || !servoCalibrationOutputEnabled) {
        showServoCalibrationPulse((pulseAtZeroUs + pulseAt180Us) / 2);
      }
      if (!force && loadedServoCalibrationProfile &&
          loadedServoCalibrationProfile.startsWith(`${servoCalibrationTarget.value}:`)) {
        element("servoCalibrationMessage").textContent =
          `Perfil confirmado na ESP32: 0° = ${Math.round(pulseAtZeroUs)} µs · 180° = ${Math.round(pulseAt180Us)} µs.`;
      }
      loadedServoCalibrationProfile = profileSignature;
    }

    function clearServoCalibrationPulseTimer() {
      if (servoCalibrationPulseTimer) window.clearTimeout(servoCalibrationPulseTimer);
      servoCalibrationPulseTimer = null;
    }

    function sendServoCalibrationPulse(value, immediate = false) {
      const pulseUs = showServoCalibrationPulse(value);
      clearServoCalibrationPulseTimer();
      if (!servoCalibrationControlAllowed) return;
      if (!servoCalibrationOutputEnabled && !immediate) return;

      const transmit = () => {
        servoCalibrationPulseTimer = null;
        servoCalibrationOutputEnabled = true;
        send({
          command: "servo_calibration_pulse",
          servo: servoCalibrationTarget.value,
          pulseUs
        });
      };
      if (immediate) transmit();
      else servoCalibrationPulseTimer = window.setTimeout(transmit, 60);
    }

    function disableServoCalibrationOutput() {
      clearServoCalibrationPulseTimer();
      servoCalibrationOutputEnabled = false;
      send({ command: "servo_calibration_disable" });
      element("servoCalibrationMessage").textContent = "Pulso desligado; a calibração continua aberta.";
    }

    function stopServoCalibrationLocally() {
      clearServoCalibrationPulseTimer();
      servoCalibrationControlAllowed = false;
      servoCalibrationOutputEnabled = false;
    }

    function beginServoCalibration() {
      stopServoCalibrationLocally();
      element("servoCalibrationMessage").textContent = "Entrada solicitada. Nenhum pulso será aplicado automaticamente.";
      send({ command: "servo_calibration_begin" });
    }

    function endServoCalibration() {
      stopServoCalibrationLocally();
      send({ command: "servo_calibration_end" });
      element("servoCalibrationMessage").textContent = "Parada solicitada; todos os pulsos serão desligados.";
    }

    function captureServoCalibrationEndpoint(input) {
      input.value = String(showServoCalibrationPulse(servoCalibrationPulseInput.value));
    }

    function saveServoCalibration() {
      if (!servoCalibrationControlAllowed) return;
      const pulseAtZeroUs = clampServoCalibrationPulse(servoPulseAtZeroInput.value);
      const pulseAt180Us = clampServoCalibrationPulse(servoPulseAt180Input.value);
      servoPulseAtZeroInput.value = String(pulseAtZeroUs);
      servoPulseAt180Input.value = String(pulseAt180Us);
      const minimumSpanUs = Number(latestServoTelemetry?.servoCalibrationMinimumSpanUs) || 200;
      if (Math.abs(pulseAt180Us - pulseAtZeroUs) < minimumSpanUs) {
        element("servoCalibrationMessage").textContent = `Os extremos precisam diferir pelo menos ${minimumSpanUs} µs.`;
        return;
      }
      send({
        command: "servo_calibration_save",
        servo: servoCalibrationTarget.value,
        pulseAtZeroUs,
        pulseAt180Us
      });
      servoCalibrationOutputEnabled = false;
      element("servoCalibrationMessage").textContent = "Salvamento solicitado. O ESP32 desligará o pulso e confirmará os novos limites na telemetria.";
    }

    function updateServoCalibration(data, fresh, calibrating, systemEmergency) {
      latestServoTelemetry = data;
      const supported = fresh && data.servoCalibrationSupported === true;
      const active = supported && data.servoCalibrationActive === true &&
        data.servoCalibrationRequested === true;
      const starting = supported && data.servoCalibrationRequested === true && !active;
      const pcaReady = data.pca9685Ok === true;
      const raspberryReady = data.esp32RaspberrySystemReady === true;
      const beginAllowed = supported && pcaReady && raspberryReady &&
        !calibrating && !systemEmergency &&
        (data.mode === "stopped" || data.mode === "manual") &&
        !active && !starting;

      element("servoCalibrationBeginButton").disabled = !beginAllowed;
      element("servoCalibrationEndButton").disabled = !active && !starting;
      servoCalibrationTarget.disabled = !supported || calibrating || systemEmergency;
      servoCalibrationWorkspace.hidden = !active;
      servoCalibrationControlAllowed = active && pcaReady && raspberryReady &&
        !calibrating && !systemEmergency;

      const workspaceControls = [
        servoCalibrationPulseSlider, servoCalibrationPulseInput,
        servoPulseAtZeroInput, servoPulseAt180Input,
        element("servoCalibrationApplyButton"),
        element("captureServoZeroButton"), element("captureServo180Button"),
        element("servoCalibrationDisableButton"),
        element("servoCalibrationSaveButton"),
        ...servoCalibrationAdjustmentButtons
      ];
      workspaceControls.forEach(control => {
        control.disabled = !servoCalibrationControlAllowed;
      });

      if (active) {
        const selectedControl = selectedServoCalibrationControl();
        const selectedByEsp = Number(data.servoCalibrationSelectedIndex) === selectedControl.index;
        servoCalibrationOutputEnabled = selectedByEsp && data[selectedControl.enabledField] === true;
        const userEditingPulse = document.activeElement === servoCalibrationPulseSlider ||
          document.activeElement === servoCalibrationPulseInput || servoCalibrationPulseTimer;
        if (servoCalibrationOutputEnabled && !userEditingPulse) {
          showServoCalibrationPulse(data[selectedControl.pulseField]);
        }
        loadSelectedServoCalibration(false);
      } else {
        stopServoCalibrationLocally();
      }

      if (!fresh) setSubsystemStatus("servoCalibrationStatus", "sem telemetria", "warn");
      else if (!supported) setSubsystemStatus("servoCalibrationStatus", "grave o firmware novo", "danger");
      else if (systemEmergency) setSubsystemStatus("servoCalibrationStatus", "bloqueada", "danger");
      else if (calibrating) setSubsystemStatus("servoCalibrationStatus", "sensores calibrando", "warn");
      else if (!pcaReady) setSubsystemStatus("servoCalibrationStatus", "PCA indisponível", "danger");
      else if (starting) setSubsystemStatus("servoCalibrationStatus", "iniciando", "warn");
      else if (active) setSubsystemStatus("servoCalibrationStatus", "ativa · Parar desliga", "warn");
      else setSubsystemStatus("servoCalibrationStatus", "pronta", "ok");
    }

    function sendCommand(command) {
      if (["start", "auto", "stop", "calibrate"].includes(command)) {
        stopServoCalibrationLocally();
      }
      if (manualEnabled) resetDrive();
      manualEnabled = false;
      resetKeyboardState();
      resetDrive();
      send({ command });
      manualEnabled = command === "start";
      updateKeyboardIndicators();
    }

    function sanitizeOledText(value, maximumLength) {
      return String(value || "")
        .normalize("NFD")
        .replace(/[\u0300-\u036f]/g, "")
        .replace(/[^\x20-\x7E]/g, "")
        .slice(0, maximumLength);
    }

    function oledDurationMilliseconds() {
      const durationSeconds = Math.max(0.5, Math.min(30, Number(oledDurationSeconds.value) || 10));
      oledDurationSeconds.value = durationSeconds.toFixed(1);
      return Math.round(durationSeconds * 1000);
    }

    function setOledEditorFeedback(message, tone = "warn") {
      element("oledRemoteStatus").textContent = message;
      element("oledRemoteStatus").className = `oled-editor-state state-${tone}`;
    }

    function selectOledLayout(layout) {
      if (!["standard", "large"].includes(layout)) return;
      selectedOledLayout = layout;
      const large = layout === "large";
      oledStandardFields.hidden = large;
      oledLargeFields.hidden = !large;
      oledLayoutButtons.forEach(button => {
        const selected = button.dataset.oledLayout === layout;
        button.classList.toggle("active", selected);
        button.setAttribute("aria-pressed", selected ? "true" : "false");
      });
      element("oledShowButton").textContent = large
        ? "Mostrar texto grande"
        : "Mostrar página comum";
      element("oledLayoutHint").textContent = large
        ? "A ESP32 escolhe a maior fonte que cabe, mantém o detalhe abaixo e pulsa o contraste."
        : "A página comum mostra título, duas linhas e o tempo restante.";
    }

    function sendOledLargeMessage(primaryText, secondaryText, durationMs, feedback) {
      const primary = sanitizeOledText(primaryText, 20);
      const secondary = sanitizeOledText(secondaryText, 20);
      oledPrimaryText.value = primary;
      oledSecondaryText.value = secondary;
      if (!primary.trim()) {
        setOledEditorFeedback("Digite o texto principal antes de enviar.");
        return false;
      }
      const sent = send({
        command: "oled_large_message",
        primaryText: primary,
        secondaryText: secondary,
        durationMs
      });
      setOledEditorFeedback(
        sent ? feedback : "Dashboard sem conexão com o robô.",
        sent ? "warn" : "bad"
      );
      return sent;
    }

    function simulateOledPreset(button) {
      selectOledLayout("large");
      const durationMs = oledAutomaticAlertDurationMs;
      oledDurationSeconds.value = (durationMs / 1000).toFixed(1);
      sendOledLargeMessage(
        button.dataset.primary,
        button.dataset.secondary,
        durationMs,
        `Simulação enviada: ${button.dataset.presetLabel}.`
      );
    }

    function showOledMessage() {
      const durationMs = oledDurationMilliseconds();
      if (selectedOledLayout === "large") {
        sendOledLargeMessage(
          oledPrimaryText.value,
          oledSecondaryText.value,
          durationMs,
          "Texto grande enviado; aguardando confirmação na telemetria."
        );
        return;
      }

      const title = sanitizeOledText(oledTitle.value, 12);
      const firstLine = sanitizeOledText(oledFirstLine.value, 20);
      const secondLine = sanitizeOledText(oledSecondLine.value, 20);
      oledTitle.value = title;
      oledFirstLine.value = firstLine;
      oledSecondLine.value = secondLine;
      if (!title.trim() && !firstLine.trim() && !secondLine.trim()) {
        setOledEditorFeedback("Digite pelo menos um texto antes de enviar.");
        return;
      }
      const sent = send({
        command: "oled_message",
        title,
        firstLine,
        secondLine,
        durationMs
      });
      setOledEditorFeedback(
        sent
          ? "Página comum enviada; aguardando confirmação na telemetria."
          : "Dashboard sem conexão com o robô.",
        sent ? "warn" : "bad"
      );
    }

    function clearOledMessage() {
      const sent = send({ command: "oled_clear" });
      setOledEditorFeedback(
        sent
          ? "Retorno à tela padrão solicitado."
          : "Dashboard sem conexão com o robô.",
        sent ? "warn" : "bad"
      );
    }

    function selectAutonomousMission() {
      manualEnabled = false;
      resetKeyboardState();
      resetDrive();
      const targetCm = Math.max(1, Math.min(300, Number(distanceTargetCm.value) || 20));
      distanceTargetCm.value = formatNumber(targetCm, 1);
      element("distanceMissionSettings").hidden = autonomousMission.value !== "drive_distance";
      element("rescueZoneFrameSettings").hidden = autonomousMission.value !== "rescue_zone_frame";
      element("servoRoutineSettings").hidden = !autonomousMission.value.startsWith("servo_");
      send({
        command: "set_autonomous_mission",
        mission: autonomousMission.value,
        distanceCm: targetCm,
        targetColor: rescueZoneTargetColor.value
      });
    }

    function confirmServoRoutineAction() {
      send({ command: "servo_routine_confirm" });
      element("servoRoutineConfirmButton").disabled = true;
    }

    function startAutonomousMission() {
      // O WebSocket preserva a ordem: primeiro confirma missão e alvo, depois inicia.
      selectAutonomousMission();
      sendCommand("auto");
    }

    function clamp(value) { return Math.max(-1, Math.min(1, value)); }

    function clampManualPower(value) {
      return Math.max(manualTestMinimumPower, Math.min(manualMaximumPower, Number(value) || manualTestMinimumPower));
    }

    function scaleOperationalAxis(axis, maximumPower) {
      const safeAxis = clamp(axis);
      if (Math.abs(safeAxis) < 0.0001) return 0;
      const magnitude = manualTestMinimumPower +
        (clampManualPower(maximumPower) - manualTestMinimumPower) * Math.abs(safeAxis);
      return Math.sign(safeAxis) * magnitude;
    }

    function updateManualPowerSettings() {
      manualDrivePower.value = clampManualPower(manualDrivePower.value).toFixed(2);
      manualTurnPower.value = clampManualPower(manualTurnPower.value).toFixed(2);
      element("manualDrivePowerValue").textContent = manualDrivePower.value;
      element("manualTurnPowerValue").textContent = manualTurnPower.value;
      try {
        localStorage.setItem("obrManualDrivePower", manualDrivePower.value);
        localStorage.setItem("obrManualTurnPower", manualTurnPower.value);
      } catch {}
      if (manualEnabled && pressedDriveKeys.size > 0) applyKeyboardDrive();
    }

    function restoreManualPowerSettings() {
      try {
        manualDrivePower.value = localStorage.getItem("obrManualDrivePower") || "0.69";
        manualTurnPower.value = localStorage.getItem("obrManualTurnPower") || "0.69";
      } catch {
        manualDrivePower.value = "0.69";
        manualTurnPower.value = "0.69";
      }
      updateManualPowerSettings();
    }

    function sendCurrentDrive() {
      if (manualEnabled) {
        send({ command: rawDiagnosticDrive ? "drive_raw" : "drive", left: requestedLeft, right: requestedRight });
      }
    }

    function updateDriveFromMixer(throttleValue, turnValue) {
      const safeThrottleValue = clamp(Number(throttleValue));
      const safeTurnValue = clamp(Number(turnValue));
      const selectedPower = Math.abs(safeTurnValue) > 0.0001
        ? clampManualPower(manualTurnPower.value)
        : clampManualPower(manualDrivePower.value);

      // O giro tem prioridade sobre frente e ré. Isso impede que combinações
      // como W+A zerem um lado e façam o robô curvar com metade da tração.
      const turning = Math.abs(safeTurnValue) > 0.0001;
      const leftAxis = turning ? safeTurnValue : safeThrottleValue;
      const rightAxis = turning ? -safeTurnValue : safeThrottleValue;

      requestedLeft = scaleOperationalAxis(leftAxis, selectedPower);
      requestedRight = scaleOperationalAxis(rightAxis, selectedPower);
      // O controle manual comum usa o perfil START/RUN e permite o sincronismo
      // pelos encoders durante a marcha reta. O ajuste independente permanece direto.
      rawDiagnosticDrive = false;
      leftValue.value = requestedLeft.toFixed(2);
      rightValue.value = requestedRight.toFixed(2);
      sendCurrentDrive();
    }

    function updateDriveFromExactInputs() {
      const parsedLeft = Number(leftValue.value);
      const parsedRight = Number(rightValue.value);
      if (!Number.isFinite(parsedLeft) || !Number.isFinite(parsedRight)) {
        leftValue.value = requestedLeft.toFixed(2);
        rightValue.value = requestedRight.toFixed(2);
        return;
      }
      requestedLeft = clamp(parsedLeft);
      requestedRight = clamp(parsedRight);
      rawDiagnosticDrive = true;
      leftValue.value = requestedLeft.toFixed(2);
      rightValue.value = requestedRight.toFixed(2);
      sendCurrentDrive();
    }

    function adjustExactSide(side, delta) {
      const target = side === "left" ? leftValue : rightValue;
      const currentValue = Number(target.value);
      if (!Number.isFinite(currentValue)) return;
      target.value = (Math.round(clamp(currentValue + delta) * 100) / 100).toFixed(2);
      updateDriveFromExactInputs();
    }

    function resetDrive() {
      requestedLeft = 0;
      requestedRight = 0;
      rawDiagnosticDrive = false;
      leftValue.value = "0.00";
      rightValue.value = "0.00";
      sendCurrentDrive();
    }

    function updateKeyboardIndicators() {
      driveKeyCodes.forEach(code => element(code.replace("Key", "key")).classList.toggle("active", pressedDriveKeys.has(code)));
      element("keyboardState").textContent = !manualEnabled
        ? "Ative o modo Manual para usar o teclado."
        : (pressedDriveKeys.size ? "Teclado comandando com os limites selecionados." : "W/S: frente e ré · A/D: giro dos dois lados com prioridade.");
    }

    function resetKeyboardState() {
      pressedDriveKeys.clear();
      updateKeyboardIndicators();
    }

    function applyKeyboardDrive() {
      const forward = (pressedDriveKeys.has("KeyW") ? 1 : 0) - (pressedDriveKeys.has("KeyS") ? 1 : 0);
      const turnValue = (pressedDriveKeys.has("KeyD") ? 1 : 0) - (pressedDriveKeys.has("KeyA") ? 1 : 0);
      updateDriveFromMixer(forward, turnValue);
    }

    function stopDriveOnFocusLoss() {
      resetKeyboardState();
      if (manualEnabled) resetDrive();
    }

    function visibleCameraIds(view = activeCameraView) {
      if (view === "dual") return ["forward", "downward"];
      return [view];
    }

    function cameraIsVisible(cameraId) {
      return visibleCameraIds().includes(cameraId);
    }

    function cameraStatusClass(status) {
      if (status === "ONLINE") return "online";
      if (status === "OFFLINE" || status === "ERRO" || status === "FALHA") return "offline";
      if (status === "CARREGANDO" || status === "INICIANDO" || status === "DESLIGANDO") return "loading";
      if (status === "DESLIGADA" || status === "PARADA") return "disabled";
      return "unconfigured";
    }

    function updateDatasetCaptureStatus(data) {
      const counts = data.datasetCaptureCounts && typeof data.datasetCaptureCounts === "object"
        ? data.datasetCaptureCounts : {};
      const safeCount = value => {
        const number = Number(value);
        return Number.isFinite(number) && number >= 0 ? Math.floor(number) : 0;
      };
      const camera = typeof data.datasetCaptureCamera === "string" ? data.datasetCaptureCamera : "";
      const session = typeof data.datasetCaptureSession === "string" ? data.datasetCaptureSession : "";
      const label = typeof data.datasetCaptureLabel === "string" ? data.datasetCaptureLabel : "";
      const fps = Number(data.datasetCaptureFps);
      const dropped = safeCount(data.datasetCaptureDropped);
      const lastSavedAt = Number(data.datasetCaptureLastSavedAt);
      const updatedAt = Number(data.datasetCaptureUpdatedAt);
      const active = data.datasetCaptureActive === true;
      const updateAgeSeconds = Date.now() / 1000 - updatedAt;
      const stale = data.datasetCaptureStale === true || (active && (
        !Number.isFinite(updatedAt) || updatedAt <= 0 || updateAgeSeconds < 0 || updateAgeSeconds > 4.0
      ));
      const configured = Boolean(camera || session || label);

      datasetCaptureStatus.dataset.state = stale ? "stale" : (active ? "active" : "paused");
      datasetCaptureState.textContent = stale
        ? "Dataset sem atualização"
        : (active ? "Dataset capturando" : "Dataset pausado");

      if (!configured) {
        datasetCaptureDetails.textContent = "Nenhuma coleta configurada.";
        datasetCaptureStatus.removeAttribute("title");
        return;
      }

      const details = [
        (camera || "--").toUpperCase(),
        (label || "--").toUpperCase(),
      ];
      if (!stale) {
        details.push(Number.isFinite(fps) && fps > 0 ? `${fps.toFixed(1)} FPS` : "-- FPS");
      }
      details.push(`B ${safeCount(counts.black)}`);
      details.push(`O ${safeCount(counts.other)}`);
      details.push(`S ${safeCount(counts.silver)}`);
      if (!stale) details.push(`${dropped} perdidas`);
      details.push(session || "--");
      datasetCaptureDetails.textContent = details.join(" · ");
      if (Number.isFinite(lastSavedAt) && lastSavedAt > 0) {
        datasetCaptureStatus.title = `Último JPEG salvo: ${new Date(lastSavedAt * 1000).toLocaleString("pt-BR")}`;
      } else {
        datasetCaptureStatus.removeAttribute("title");
      }
    }

    function setCameraStatus(cameraId, status) {
      const camera = cameras[cameraId];
      camera.status = status;
      const statusElement = mountedCameraStatuses.get(cameraId);
      if (statusElement) {
        statusElement.textContent = status;
        statusElement.className = `camera-status ${cameraStatusClass(status)}`;
      }
      if (cameraId === "downward") updateLineCameraButtons();
      renderCameraMetadata();
    }

    function addCameraMetadata(target, label, value) {
      const chip = document.createElement("span");
      chip.className = "meta-chip";
      chip.append(`${label} `);
      const content = document.createElement("strong");
      content.textContent = value;
      chip.appendChild(content);
      target.appendChild(chip);
    }

    function cameraFpsText(camera) {
      const fps = Number(camera.metadata.fps);
      return camera.status === "ONLINE" && Number.isFinite(fps) && fps > 0
        ? `${fps.toFixed(1)} FPS`
        : "-- FPS";
    }

    function updateOperationCameraFps() {
      const selectedFps = activeCameraView === "dual"
        ? `${cameraFpsText(cameras.downward)} / ${cameraFpsText(cameras.forward)}`
        : cameraFpsText(cameras[activeCameraView]);
      setTextIfChanged(operationCameraFps, selectedFps);
      setTextIfChanged(cameraHudFps, cameraFpsText(cameras.downward));
    }

    function renderCameraMetadata() {
      updateOperationCameraFps();
      const visibleCameras = visibleCameraIds().map(cameraId => cameras[cameraId]);
      const selectedName = activeCameraView === "dual"
        ? "Câmeras inferior e frontal"
        : cameras[activeCameraView].name;
      const statusText = visibleCameras
        .map(camera => activeCameraView === "dual" ? `${camera.name.replace("Câmera ", "")} ${camera.status}` : camera.status)
        .join(" · ");
      const metadataSignature = JSON.stringify({
        view: activeCameraView,
        statusText,
        cameras: visibleCameras.map(camera => ({
          id: camera.id,
          role: camera.role,
          streamUrl: camera.streamUrl,
          metadata: camera.metadata
        }))
      });
      setTextIfChanged(operationCameraName, selectedName);
      if (metadataSignature === lastCameraMetadataSignature) return;
      lastCameraMetadataSignature = metadataSignature;
      cameraMetadata.replaceChildren();
      cameraTechnicalMetadata.replaceChildren();
      addCameraMetadata(cameraMetadata, "Status", statusText);
      addCameraMetadata(cameraMetadata, "FPS", operationCameraFps.textContent);
      visibleCameras.forEach(camera => {
        const prefix = activeCameraView === "dual" ? `${camera.name.replace("Câmera ", "")} · ` : "";
        addCameraMetadata(cameraTechnicalMetadata, `${prefix}Papel`, camera.role);
        addCameraMetadata(cameraTechnicalMetadata, `${prefix}Resolução`, camera.metadata.resolution);
        addCameraMetadata(cameraTechnicalMetadata, `${prefix}Sensor`, camera.metadata.sensor);
        addCameraMetadata(cameraTechnicalMetadata, `${prefix}Crop`, camera.metadata.crop);
        addCameraMetadata(cameraTechnicalMetadata, `${prefix}Formato`, camera.metadata.format);
        addCameraMetadata(cameraTechnicalMetadata, `${prefix}Stream`, camera.streamUrl || "NÃO CONFIGURADO");
      });
    }

    function clearCameraReconnect(cameraId) {
      window.clearTimeout(cameraReconnectTimers.get(cameraId));
      cameraReconnectTimers.delete(cameraId);
    }

    function cameraStreamUrl(camera) {
      if (!camera.streamUrl) return null;
      const separator = camera.streamUrl.includes("?") ? "&" : "?";
      const modeParameter = camera.displayMode
        ? `mode=${encodeURIComponent(camera.displayMode)}&`
        : "";
      return `${camera.streamUrl}${separator}${modeParameter}ts=${Date.now()}`;
    }

    function setCameraFrameMessage(frame, title, detail) {
      const message = frame.querySelector(".camera-message");
      if (!message) return;
      const titleElement = message.querySelector("strong");
      const detailElement = message.querySelector("span");
      setTextIfChanged(titleElement, title);
      setTextIfChanged(detailElement, detail);
    }

    function connectCameraImage(camera, image, frame, generation) {
      if (generation !== cameraRenderGeneration || !cameraIsVisible(camera.id)) return;
      clearCameraReconnect(camera.id);
      frame.className = "camera-frame loading";
      setCameraFrameMessage(frame, "AGUARDANDO STREAM", "Conectando à câmera selecionada.");
      if (camera.id !== "downward") setCameraStatus(camera.id, "CARREGANDO");
      // O mesmo elemento é reutilizado; remover o src encerra a conexão MJPEG
      // anterior antes que o modo escolhido abra uma nova conexão única.
      image.removeAttribute("src");
      window.requestAnimationFrame(() => {
        if (generation !== cameraRenderGeneration || !cameraIsVisible(camera.id)) return;
        image.src = cameraStreamUrl(camera);
      });
    }

    function selectDownwardCameraMode(mode) {
      if (!["real", "line"].includes(mode)) return;
      const camera = cameras.downward;
      camera.displayMode = mode;
      mountedCameraModeButtons.forEach(button => {
        const selected = button.dataset.cameraMode === mode;
        button.classList.toggle("active", selected);
        button.setAttribute("aria-pressed", selected ? "true" : "false");
      });
      const image = mountedCameraImages.get("downward");
      const frame = image?.closest(".camera-frame");
      if (image && frame) connectCameraImage(camera, image, frame, cameraRenderGeneration);
    }

    function buildCameraModeSelector(camera) {
      const selector = document.createElement("div");
      selector.className = "camera-mode-selector";
      selector.setAttribute("role", "group");
      selector.setAttribute("aria-label", "Modo visual da câmera inferior");
      const modes = [
        ["real", "REAL"],
        ["line", "LINHA"]
      ];
      modes.forEach(([mode, label]) => {
        const button = document.createElement("button");
        const selected = camera.displayMode === mode;
        button.type = "button";
        button.className = `camera-mode-button${selected ? " active" : ""}`;
        button.dataset.cameraMode = mode;
        button.textContent = label;
        button.setAttribute("aria-pressed", selected ? "true" : "false");
        button.addEventListener("click", () => selectDownwardCameraMode(mode));
        mountedCameraModeButtons.push(button);
        selector.appendChild(button);
      });
      return selector;
    }

    function mountCameraStream(camera, frame, generation) {
      const image = document.createElement("img");
      image.alt = `Imagem direta da ${camera.name.toLowerCase()}`;
      image.onload = () => {
        if (generation !== cameraRenderGeneration) return;
        frame.className = "camera-frame online";
        // Para a câmera inferior, stream MJPEG não substitui a confirmação do IPC.
        if (camera.id !== "downward") setCameraStatus(camera.id, "ONLINE");
      };
      image.onerror = () => {
        if (generation !== cameraRenderGeneration || !cameraIsVisible(camera.id)) return;
        frame.className = "camera-frame offline";
        setCameraFrameMessage(frame, "CÂMERA INDISPONÍVEL", "O stream não respondeu. Uma nova tentativa será feita automaticamente.");
        // A câmera inferior recebe o estado definitivo pelo IPC do serviço.
        // Não a marque como ONLINE nem reutilize o último frame enquanto ele estiver parado.
        if (camera.id === "downward") refreshCameraStatus();
        else setCameraStatus(camera.id, "OFFLINE");
        clearCameraReconnect(camera.id);
        cameraReconnectTimers.set(camera.id, window.setTimeout(
          () => connectCameraImage(camera, image, frame, generation),
          1000
        ));
      };
      mountedCameraImages.set(camera.id, image);
      frame.appendChild(image);
      const message = document.createElement("div");
      message.className = "camera-message";
      const messageTitle = document.createElement("strong");
      messageTitle.textContent = "AGUARDANDO STREAM";
      const messageDetail = document.createElement("span");
      messageDetail.textContent = "Conectando à câmera selecionada.";
      message.append(messageTitle, messageDetail);
      frame.appendChild(message);
      connectCameraImage(camera, image, frame, generation);
    }

    function buildCameraPlaceholder(camera, frame) {
      const placeholder = document.createElement("div");
      placeholder.className = "camera-placeholder";
      const title = document.createElement("strong");
      const isDownward = camera.id === "downward";
      title.textContent = isDownward
        ? (camera.status === "FALHA" ? "FALHA NA CÂMERA INFERIOR" : "CÂMERA INFERIOR")
        : (camera.status === "ERRO" ? "FALHA NA CÂMERA FRONTAL" : "CÂMERA FRONTAL");
      const message = document.createElement("p");
      if (camera.status === "ERRO" || camera.status === "FALHA") message.textContent = camera.error || "Não foi possível abrir a câmera.";
      else if (camera.enabled) message.textContent = isDownward
        ? "Abrindo a câmera inferior e preparando o stream…"
        : "Abrindo a CAM1 e preparando o stream…";
      else message.textContent = "Desligada para economizar processamento.";
      const plannedLabel = document.createElement("span");
      plannedLabel.className = "planned-use";
      plannedLabel.textContent = "Configuração:";
      const plannedUse = document.createElement("p");
      plannedUse.textContent = isDownward
        ? "Saída 480×360 · 30 FPS"
        : "Saída 960×540 · sensor 1920×1080 · 30 FPS";
      placeholder.append(title, message, plannedLabel, plannedUse);
      frame.appendChild(placeholder);
    }

    function updateForwardCameraButtons() {
      document.querySelectorAll("[data-forward-camera-toggle]").forEach(button => {
        const camera = cameras.forward;
        button.disabled = camera.transitioning;
        button.classList.toggle("active", camera.enabled);
        button.setAttribute("aria-pressed", camera.enabled ? "true" : "false");
        button.textContent = camera.transitioning
          ? "AGUARDE"
          : (camera.enabled ? "DESATIVAR" : "ATIVAR");
      });
    }

    function toggleForwardCamera() {
      const camera = cameras.forward;
      if (camera.transitioning) return;
      const enabled = !camera.enabled;
      if (!send({ command: "set_forward_camera", enabled })) {
        camera.error = "O dashboard está sem conexão com o robô.";
        setCameraStatus("forward", "ERRO");
        if (cameraIsVisible("forward")) renderCameraView(activeCameraView);
        return;
      }
      camera.transitioning = true;
      camera.requestedEnabled = enabled;
      camera.transitionDeadlineMs = Date.now() + 5000;
      // A interface abre ou fecha o stream imediatamente. O status da CAM1
      // continua sendo a confirmação final, mas não bloqueia a resposta ao clique.
      camera.enabled = enabled;
      camera.active = enabled;
      camera.error = "";
      setCameraStatus("forward", enabled ? "INICIANDO" : "DESLIGANDO");
      updateForwardCameraButtons();
      if (cameraIsVisible("forward")) renderCameraView(activeCameraView);
    }

    function buildForwardCameraToggle() {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "camera-power-button";
      button.dataset.forwardCameraToggle = "";
      button.addEventListener("click", toggleForwardCamera);
      return button;
    }

    function updateLineCameraButtons() {
      document.querySelectorAll("[data-line-camera-toggle]").forEach(button => {
        const camera = cameras.downward;
        button.disabled = camera.transitioning;
        button.classList.toggle("active", camera.enabled);
        button.setAttribute("aria-pressed", camera.enabled ? "true" : "false");
        button.textContent = camera.transitioning
          ? "AGUARDE"
          : (camera.enabled ? "DESATIVAR" : "ATIVAR");
      });
    }

    function toggleLineCamera() {
      const camera = cameras.downward;
      if (camera.transitioning) return;
      const enabled = !camera.enabled;
      if (!send({ command: "set_line_camera", enabled })) {
        camera.error = "O dashboard está sem conexão com o robô.";
        setCameraStatus("downward", "FALHA");
        if (cameraIsVisible("downward")) renderCameraView(activeCameraView);
        return;
      }
      camera.transitioning = true;
      camera.requestedEnabled = enabled;
      camera.transitionDeadlineMs = Date.now() + 5000;
      camera.enabled = enabled;
      camera.active = enabled;
      camera.error = "";
      setCameraStatus("downward", enabled ? "INICIANDO" : "PARADA");
      updateLineCameraButtons();
      if (cameraIsVisible("downward")) renderCameraView(activeCameraView);
    }

    function buildLineCameraToggle() {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "camera-power-button";
      button.dataset.lineCameraToggle = "";
      button.addEventListener("click", toggleLineCamera);
      return button;
    }

    function buildCameraFeed(cameraId, generation) {
      const camera = cameras[cameraId];
      const feed = document.createElement("article");
      feed.className = "camera-feed-card";
      feed.dataset.cameraId = camera.id;

      const header = document.createElement("header");
      header.className = "camera-feed-header";
      const identity = document.createElement("div");
      identity.className = "camera-feed-identity";
      const name = document.createElement("strong");
      name.textContent = camera.name;
      identity.appendChild(name);
      if (camera.id === "downward") identity.appendChild(buildCameraModeSelector(camera));
      const actions = document.createElement("div");
      actions.className = "camera-feed-actions";
      if (camera.id === "downward") actions.appendChild(buildLineCameraToggle());
      if (camera.id === "forward") actions.appendChild(buildForwardCameraToggle());
      const status = document.createElement("span");
      status.className = `camera-status ${cameraStatusClass(camera.status)}`;
      status.textContent = camera.status;
      mountedCameraStatuses.set(camera.id, status);
      actions.appendChild(status);
      header.append(identity, actions);

      const frame = document.createElement("div");
      const shouldMountStream = camera.enabled && camera.active;
      frame.className = shouldMountStream ? "camera-frame loading" : "camera-frame unconfigured";
      if (shouldMountStream) mountCameraStream(camera, frame, generation);
      else buildCameraPlaceholder(camera, frame);
      if (camera.id === "downward") frame.appendChild(downwardCameraTelemetry);
      feed.append(header, frame);
      updateForwardCameraButtons();
      updateLineCameraButtons();
      return feed;
    }

    function unmountCameraStreams() {
      // Remover o src encerra conexões MJPEG que não pertencem à visualização atual.
      cameraReconnectTimers.forEach(timer => window.clearTimeout(timer));
      cameraReconnectTimers.clear();
      mountedCameraImages.forEach(image => {
        image.onload = null;
        image.onerror = null;
        image.removeAttribute("src");
      });
      mountedCameraImages.clear();
      mountedCameraStatuses.clear();
      mountedCameraModeButtons.length = 0;
      cameraFeeds.replaceChildren();
    }

    function renderCameraView(view) {
      if (!["downward", "forward", "dual"].includes(view)) return;
      cameraRenderGeneration += 1;
      unmountCameraStreams();
      activeCameraView = view;
      cameraFeeds.dataset.view = view;
      cameraViewButtons.forEach(button => {
        const selected = button.dataset.cameraView === view;
        button.classList.toggle("active", selected);
        button.setAttribute("aria-pressed", selected ? "true" : "false");
      });
      const generation = cameraRenderGeneration;
      visibleCameraIds(view).forEach(cameraId => {
        cameraFeeds.appendChild(buildCameraFeed(cameraId, generation));
      });
      downwardCameraTelemetry.hidden = view === "forward";
      renderCameraMetadata();
      if (cameraIsVisible("downward")) refreshCameraStatus();
      if (cameraIsVisible("forward")) refreshForwardCameraStatus();
    }

    function clearCameraDiagnostics() {
      latestCameraDiagnosticData = null;
      renderCurrentCameraDiagnostics();
    }

    function clearOperationCameraDiagnostics() {
      Object.values(operationCameraDiagnosticFields)
        .forEach(field => setTextIfChanged(field, "—"));
      setTextIfChanged(
        operationCameraDiagnosticFields.lineFollowerState,
        "CONTROLE PENDENTE"
      );
    }

    function updateOperationCameraDiagnostics(data) {
  setTextIfChanged(
    operationCameraDiagnosticFields.lineFollowerState,
    data.lineFollowerImplemented === true ? "CONTROLE ATIVO" : "CONTROLE PENDENTE"
  );

  setTextIfChanged(
    operationCameraDiagnosticFields.greenInterpretation,
    String(
      data.greenRawInterpretation ||
      data.greenInterpretation ||
      "SEM_DECISAO"
    ).replaceAll("_", " ")
  );

  setTextIfChanged(
    operationCameraDiagnosticFields.greenConfirmed,
    data.greenConfirmed === true ? "CONFIRMADO" : "NÃO CONFIRMADO"
  );

  setTextIfChanged(
    operationCameraDiagnosticFields.greenPathBlackValid,
    data.greenPathBlackValid === true ? "SIM" : "NÃO"
  );

  setTextIfChanged(
    operationCameraDiagnosticFields.lineSequence,
    Number.isFinite(Number(data.lineSequence))
      ? `SEQ ${Number(data.lineSequence).toFixed(0)}`
      : "—"
  );

  setTextIfChanged(
    operationCameraDiagnosticFields.specularRepair,
    `${formatNumber(data.specularRepairPixels, 0)} px · ${formatNumber(data.specularRepairComponents, 0)} comp`
  );

  setTextIfChanged(
    operationCameraDiagnosticFields.pipelinePrimary,
    `BIN ${formatPipelineTiming(data.binaryMs)} · SPEC ${formatPipelineTiming(data.specularMs)} ms`
  );

  setTextIfChanged(
    operationCameraDiagnosticFields.pipelineSecondary,
    `MORPH ${formatPipelineTiming(data.morphMs)} · CONT ${formatPipelineTiming(data.contoursMs)} ms`
  );

  const silverShadowAvailable = data.silverShadowAvailable === true;
  const silverShadowDetected = data.silverShadowDetected === true;
  const silverMarkerConfirmed = data.courseMarkerConfirmed === true &&
    data.courseMarker === "GRAY";
  const silverConfirmationFrames = Number(data.silverConfirmationFrames) || 0;
  const silverConfirmationRequiredFrames =
    Number(data.silverConfirmationRequiredFrames) || 4;
  setTextIfChanged(
    operationCameraDiagnosticFields.silverShadowState,
    !silverShadowAvailable
      ? "INDISPONÍVEL"
      : silverMarkerConfirmed
        ? "ENTRADA CONFIRMADA"
        : silverShadowDetected
          ? `CONFIRMANDO ${silverConfirmationFrames}/${silverConfirmationRequiredFrames}`
          : String(data.silverShadowLabel || "SEM LEITURA").toUpperCase()
  );
  setTextIfChanged(
    operationCameraDiagnosticFields.silverShadowDetails,
    silverShadowAvailable
      ? `S ${formatNumber(Number(data.silverShadowProbability) * 100, 1)}% · MARGEM ${formatNumber(Number(data.silverShadowMargin) * 100, 1)}% · ${formatPipelineTiming(data.silverShadowInferenceMs)} ms · ${formatNumber(data.silverInferenceTargetFps, 0)} FPS`
      : String(data.silverShadowError || "—")
  );
}

    function updateForwardAssistDiagnostic(data) {
      const state = String(data.forwardPathState || "UNCERTAIN");
      const near = String(data.nearLineState || "UNKNOWN");
      const decision = String(data.gapValidationDecision || "NORMAL");
      setTextIfChanged(
        forwardAssistDiagnostic,
        `NEAR ${near} | FWD ${state} | DECISION ${decision} | SOURCE ${data.lineControlSource || "UNAVAILABLE"} | CONTROLE INFERIOR`
      );
    }

    function renderCurrentCameraDiagnostics() {
      if (latestCameraDiagnosticData) {
        updateOperationCameraDiagnostics(latestCameraDiagnosticData);
      } else {
        clearOperationCameraDiagnostics();
      }
    }

    function updateCameraDiagnostics(data) {
      const numericFields = [
        data.greenConsecutiveSamples,
        data.specularRepairPixels, data.specularRepairComponents,
        data.lineTimestamp, data.lineSequence, data.timestamp
      ];
      const fieldsPresent = typeof data.lineFollowerImplemented === "boolean" &&
        typeof data.greenConfirmed === "boolean" &&
        typeof data.greenPathBlackValid === "boolean" &&
        typeof data.greenInterpretation === "string" &&
        numericFields.every(value => Number.isFinite(Number(value)));
      const statusTimestamp = Number(data.timestamp);
      const lineTimestamp = Number(data.lineTimestamp);
      const nowMs = performance.now();
      if (statusTimestamp !== lastCameraStatusTimestamp) {
        lastCameraStatusTimestamp = statusTimestamp;
        lastCameraStatusChangeAtMs = nowMs;
      }
      const lineAgeAtStatusMs = (statusTimestamp - lineTimestamp) * 1000;
      const statusFresh = fieldsPresent && nowMs - lastCameraStatusChangeAtMs <= 1000 &&
        lineAgeAtStatusMs >= 0 && lineAgeAtStatusMs <= 1000;
      if (!statusFresh) {
        clearCameraDiagnostics();
        return;
      }
      latestCameraDiagnosticData = data;
      renderCurrentCameraDiagnostics();
    }

    async function refreshCameraStatus() {
      if (!cameraIsVisible("downward") && activeDashboardMode !== "diagnostics") return;
      const camera = cameras.downward;
      try {
        const response = await fetch(`${camera.statusUrl}?ts=${Date.now()}`, { cache: "no-store" });
        if (!response.ok) throw new Error("camera status unavailable");
        const data = await response.json();
        updateDatasetCaptureStatus(data);
        let visualState = String(data.state || "FALHA").toUpperCase();
        const statusTimestamp = Number(data.timestamp);
        const nowMs = performance.now();

        if (
          Number.isFinite(statusTimestamp) &&
          statusTimestamp !== lastCameraStatusTimestamp
        ) {
          lastCameraStatusTimestamp = statusTimestamp;
          lastCameraStatusChangeAtMs = nowMs;
        }

        const statusFresh =
          Number.isFinite(statusTimestamp) &&
          lastCameraStatusChangeAtMs > 0 &&
          nowMs - lastCameraStatusChangeAtMs <= 1000;
        if (!statusFresh) {
          // Um JSON antigo não confirma uma câmera em execução após parar o serviço.
          visualState = "FALHA";
        }
        if (!["ONLINE", "PARADA", "INICIANDO", "FALHA"].includes(visualState)) {
          throw new Error("invalid line camera state");
        }
        const backendEnabled = data.enabled === true;
        const backendActive = data.active === true;
        const backendFailed = visualState === "FALHA";
        const requestedStateReady = camera.requestedEnabled
          ? backendEnabled && backendActive
          : !backendEnabled && !backendActive;
        const waitForRequestedState = camera.transitioning &&
          !requestedStateReady && !backendFailed &&
          Date.now() < camera.transitionDeadlineMs;
        if (!waitForRequestedState) {
          camera.enabled = backendEnabled;
          camera.active = backendActive && statusFresh;
        }
        camera.error = data.error || "";
        if (camera.transitioning && (requestedStateReady || backendFailed ||
            Date.now() >= camera.transitionDeadlineMs)) {
          camera.transitioning = false;
        }
        setCameraStatus("downward", visualState);
        if (visualState !== "ONLINE") {
          clearCameraDiagnostics();
        }
        else {
          const width = Number(data.width);
          const height = Number(data.height);
          const sensorMode = data.sensorMode || {};
          const scalerCrop = data.scalerCrop || {};
          camera.metadata.fps = Number(data.fps).toFixed(1);
          camera.metadata.resolution = width > 0 && height > 0 ? `${width.toFixed(0)}×${height.toFixed(0)}` : "--";
          camera.metadata.sensor = Number(sensorMode.width) > 0 && Number(sensorMode.height) > 0 ? `${Number(sensorMode.width).toFixed(0)}×${Number(sensorMode.height).toFixed(0)} ${Number(sensorMode.bitDepth).toFixed(0)}-bit` : "--";
          camera.metadata.crop = Number.isFinite(Number(scalerCrop.x)) && Number.isFinite(Number(scalerCrop.y)) && Number(scalerCrop.width) > 0 && Number(scalerCrop.height) > 0 ? `${Number(scalerCrop.x).toFixed(0)},${Number(scalerCrop.y).toFixed(0)},${Number(scalerCrop.width).toFixed(0)},${Number(scalerCrop.height).toFixed(0)}` : "--";
          camera.metadata.format = data.cameraFormat || "--";
          updateCameraDiagnostics(data);
        }
      } catch {
        // Sem IPC não existe visão válida para o segue-faixa; a missão já bloqueia
        // o movimento por sourceFresh, e o painel expõe a falha sem estado ambíguo.
        setCameraStatus("downward", "FALHA");
        clearCameraDiagnostics();
      }

      updateLineCameraButtons();
      const isStreaming = camera.enabled && camera.active;
      const streamIsMounted = mountedCameraImages.has("downward");
      if (cameraIsVisible("downward") && streamIsMounted !== isStreaming) {
        renderCameraView(activeCameraView);
      }
    }

    async function refreshForwardCameraStatus() {
      const camera = cameras.forward;
      try {
        const response = await fetch(`${camera.statusUrl}?ts=${Date.now()}`, { cache: "no-store" });
        if (!response.ok) throw new Error("forward camera status unavailable");
        const data = await response.json();
        updateDatasetCaptureStatus(data);
        const backendEnabled = data.enabled === true;
        const backendActive = data.active === true;
        const backendFailed = data.state === "error";
        const requestedStateReady = camera.requestedEnabled
          ? backendEnabled && backendActive
          : !backendEnabled && !backendActive;
        const waitForRequestedState = camera.transitioning &&
          !requestedStateReady && !backendFailed &&
          Date.now() < camera.transitionDeadlineMs;
        if (!waitForRequestedState) {
          camera.enabled = backendEnabled;
          camera.active = backendActive;
        }
        camera.error = data.error || "";
        if (camera.transitioning && (requestedStateReady || backendFailed ||
            Date.now() >= camera.transitionDeadlineMs)) {
          camera.transitioning = false;
        }

        const width = Number(data.width);
        const height = Number(data.height);
        const sensorMode = data.sensorMode || {};
        const scalerCrop = data.scalerCrop || {};
        camera.metadata.fps = Number(data.fps || 0).toFixed(1);
        camera.metadata.resolution = width > 0 && height > 0 ? `${width.toFixed(0)}×${height.toFixed(0)}` : "960×540";
        camera.metadata.sensor = Number(sensorMode.width) > 0 && Number(sensorMode.height) > 0 ? `${Number(sensorMode.width).toFixed(0)}×${Number(sensorMode.height).toFixed(0)} ${Number(sensorMode.bitDepth).toFixed(0)}-bit` : "1920×1080 10-bit";
        camera.metadata.crop = Number.isFinite(Number(scalerCrop.x)) && Number.isFinite(Number(scalerCrop.y)) && Number(scalerCrop.width) > 0 && Number(scalerCrop.height) > 0 ? `${Number(scalerCrop.x).toFixed(0)},${Number(scalerCrop.y).toFixed(0)},${Number(scalerCrop.width).toFixed(0)},${Number(scalerCrop.height).toFixed(0)}` : "--";
        camera.metadata.format = data.cameraFormat || "--";
        let status = "INICIANDO";
        if (!camera.enabled) status = "DESLIGADA";
        else if (data.state === "error") status = "ERRO";
        else if (camera.active && Number(data.fps) > 0) status = "ONLINE";
        setCameraStatus("forward", status);
      } catch {
        // A telemetria confirma o estado, mas não controla o stream. Se essa
        // consulta falhar, a imagem deve continuar no estado pedido pelo usuário.
        if (camera.transitioning && Date.now() >= camera.transitionDeadlineMs) {
          camera.transitioning = false;
        }
        camera.error = "A telemetria da câmera frontal não respondeu.";
      }

      updateForwardCameraButtons();
      const isStreaming = camera.enabled && camera.active;
      const streamIsMounted = mountedCameraImages.has("forward");
      // O estado do processo e o elemento exibido podem chegar em ordens
      // diferentes. A comparação com o DOM corrige essa corrida sem reabrir
      // conexões MJPEG que já estejam funcionando.
      if (cameraIsVisible("forward") && streamIsMounted !== isStreaming) {
        renderCameraView(activeCameraView);
      }
    }

    dashboardModeButtons.forEach(button => {
      button.addEventListener("click", () => selectDashboardMode(button.dataset.dashboardMode));
    });
    cameraViewButtons.forEach(button => {
      button.addEventListener("click", () => renderCameraView(button.dataset.cameraView));
    });
    manualDrivePower.addEventListener("input", updateManualPowerSettings);
    manualTurnPower.addEventListener("input", updateManualPowerSettings);
    leftValue.addEventListener("change", updateDriveFromExactInputs);
    rightValue.addEventListener("change", updateDriveFromExactInputs);
    Object.entries(servoControls).forEach(([servoName, control]) => {
      control.slider.addEventListener("input", () => sendServoAngle(servoName, control.slider.value));
      control.slider.addEventListener("change", () => sendServoAngle(servoName, control.slider.value, true));
      control.input.addEventListener("change", () => sendServoAngle(servoName, control.input.value, true));
    });
    element("servoCalibrationBeginButton").addEventListener("click", beginServoCalibration);
    element("servoCalibrationEndButton").addEventListener("click", endServoCalibration);
    element("servoCalibrationApplyButton").addEventListener("click", () => {
      sendServoCalibrationPulse(servoCalibrationPulseInput.value, true);
    });
    element("servoCalibrationDisableButton").addEventListener("click", disableServoCalibrationOutput);
    element("servoCalibrationSaveButton").addEventListener("click", saveServoCalibration);
    element("captureServoZeroButton").addEventListener("click", () => {
      captureServoCalibrationEndpoint(servoPulseAtZeroInput);
    });
    element("captureServo180Button").addEventListener("click", () => {
      captureServoCalibrationEndpoint(servoPulseAt180Input);
    });
    servoCalibrationTarget.addEventListener("change", () => {
      if (servoCalibrationControlAllowed) disableServoCalibrationOutput();
      loadedServoCalibrationProfile = "";
      loadSelectedServoCalibration(true);
      element("servoCalibrationMessage").textContent = "Servo trocado. O pulso anterior foi desligado; use Aplicar para iniciar o novo teste.";
    });
    servoCalibrationPulseSlider.addEventListener("input", () => {
      sendServoCalibrationPulse(servoCalibrationPulseSlider.value);
    });
    servoCalibrationPulseSlider.addEventListener("change", () => {
      if (servoCalibrationOutputEnabled) {
        sendServoCalibrationPulse(servoCalibrationPulseSlider.value, true);
      }
    });
    servoCalibrationPulseInput.addEventListener("change", () => {
      showServoCalibrationPulse(servoCalibrationPulseInput.value);
    });
    servoCalibrationAdjustmentButtons.forEach(button => {
      button.addEventListener("click", () => {
        const nextPulseUs = Number(servoCalibrationPulseInput.value) +
          Number(button.dataset.servoPulseDelta);
        sendServoCalibrationPulse(nextPulseUs, true);
      });
    });
    window.setInterval(() => {
      if (servoCalibrationControlAllowed && servoCalibrationOutputEnabled) {
        sendServoCalibrationPulse(servoCalibrationPulseInput.value, true);
      } else if (servoControlAllowed) {
        Object.entries(servoControls).forEach(([servoName, control]) => {
          if (control.outputEnabled && control.requestedAngleDegrees !== null) {
            sendServoAngle(servoName, control.requestedAngleDegrees, true);
          }
        });
      }
    }, servoKeepAlivePeriodMs);
    document.querySelectorAll("#independentMotorControl [data-side][data-delta]").forEach(button => {
      button.addEventListener("click", () => adjustExactSide(button.dataset.side, Number(button.dataset.delta)));
    });
    oledLayoutButtons.forEach(button => {
      button.addEventListener("click", () => selectOledLayout(button.dataset.oledLayout));
    });
    oledPresetButtons.forEach(button => {
      button.addEventListener("click", () => simulateOledPreset(button));
    });
    autonomousMission.addEventListener("change", selectAutonomousMission);
    rescueZoneTargetColor.addEventListener("change", selectAutonomousMission);
    distanceTargetCm.addEventListener("change", selectAutonomousMission);
    element("servoRoutineConfirmButton").addEventListener(
      "click", confirmServoRoutineAction);
    document.addEventListener("keydown", event => {
      if (!driveKeyCodes.includes(event.code) || event.ctrlKey || event.altKey || event.metaKey) return;
      event.preventDefault();
      if (!manualEnabled) { updateKeyboardIndicators(); return; }
      if (!pressedDriveKeys.has(event.code)) {
        pressedDriveKeys.add(event.code);
        updateKeyboardIndicators();
        applyKeyboardDrive();
      }
    });
    document.addEventListener("keyup", event => {
      if (!driveKeyCodes.includes(event.code)) return;
      event.preventDefault();
      if (pressedDriveKeys.delete(event.code)) {
        updateKeyboardIndicators();
        if (manualEnabled) applyKeyboardDrive();
      }
    });
    window.addEventListener("blur", stopDriveOnFocusLoss);
    document.addEventListener("visibilitychange", () => {
      if (document.hidden) stopDriveOnFocusLoss();
    });
    window.setInterval(sendCurrentDrive, 100);
    window.setInterval(refreshCameraStatus, 200);
    window.setInterval(refreshForwardCameraStatus, 500);
    selectDashboardMode("operation");
    selectOledLayout("standard");
    renderCameraView("downward");
    restoreManualPowerSettings();
    updateKeyboardIndicators();
    connect();
  </script>
</body>
</html>)HTML";
}

void DashboardServer::sendHttpResponse(SocketHandle client, const std::string& content, const std::string& contentType)
{
    std::ostringstream response;
    response << "HTTP/1.1 200 OK\r\n"
             << "Content-Type: " << contentType << "\r\n"
             << "Content-Length: " << content.size() << "\r\n"
             << "Cache-Control: no-store, no-cache, must-revalidate\r\n"
             << "Pragma: no-cache\r\n"
             << "Expires: 0\r\n"
             << "Connection: close\r\n\r\n"
             << content;

    std::string text = response.str();
    send(client, text.c_str(), static_cast<int>(text.size()), 0);
}

bool DashboardServer::sendStaticFile(SocketHandle client, const char* path, const char* contentType)
{
    std::ifstream file(path, std::ios::binary);
    if (!file)
    {
        return false;
    }

    std::ostringstream contentStream;
    contentStream << file.rdbuf();
    const std::string content = contentStream.str();

    // O nome versionado no HTML permite cache longo sem manter uma imagem antiga
    // quando o logotipo for substituído em uma revisão futura.
    std::ostringstream header;
    header << "HTTP/1.1 200 OK\r\n"
           << "Content-Type: " << contentType << "\r\n"
           << "Content-Length: " << content.size() << "\r\n"
           << "Cache-Control: public, max-age=31536000, immutable\r\n"
           << "Connection: close\r\n\r\n";
    const std::string headerText = header.str();
    return sendAll(client, headerText.data(), headerText.size()) &&
           sendAll(client, content.data(), content.size());
}

void DashboardServer::sendHttpNotFound(SocketHandle client)
{
    const std::string content = "Not found";
    std::ostringstream response;
    response << "HTTP/1.1 404 Not Found\r\n"
             << "Content-Type: text/plain; charset=utf-8\r\n"
             << "Content-Length: " << content.size() << "\r\n"
             << "Connection: close\r\n\r\n"
             << content;

    std::string text = response.str();
    send(client, text.c_str(), static_cast<int>(text.size()), 0);
}

bool DashboardServer::sendCameraFrame(SocketHandle client)
{
    // O script da câmera troca o arquivo de forma atômica para evitar JPEG parcial.
    // Se a leitura falhar, a dashboard mantém o aviso de câmera indisponível.
    std::ifstream file(config::kCameraFramePath, std::ios::binary);
    if (!file)
    {
        return false;
    }

    std::ostringstream content;
    content << file.rdbuf();
    std::string frame = content.str();
    if (frame.empty())
    {
        return false;
    }

    std::ostringstream response;
    response << "HTTP/1.1 200 OK\r\n"
             << "Content-Type: image/jpeg\r\n"
             << "Content-Length: " << frame.size() << "\r\n"
             << "Cache-Control: no-store\r\n"
             << "Connection: close\r\n\r\n";

    std::string header = response.str();
    return sendAll(client, header.c_str(), header.size()) && sendAll(client, frame.c_str(), frame.size());
}

bool DashboardServer::sendCameraStreamHead(SocketHandle client, int streamPort)
{
    // O HEAD é usado só para diagnóstico rápido com curl.
    // Ele confirma se o processo Python da câmera está aceitando conexões.
    SocketHandle cameraSocket = socket(AF_INET, SOCK_STREAM, 0);
#ifdef _WIN32
    if (cameraSocket == INVALID_SOCKET)
#else
    if (cameraSocket < 0)
#endif
    {
        return false;
    }

    sockaddr_in address = {};
    address.sin_family = AF_INET;
    address.sin_port = htons(streamPort);
    address.sin_addr.s_addr = htonl(0x7f000001u);

    if (connect(cameraSocket, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
    {
#ifdef _WIN32
        closesocket(cameraSocket);
#else
        close(cameraSocket);
#endif
        return false;
    }

#ifdef _WIN32
    closesocket(cameraSocket);
#else
    close(cameraSocket);
#endif

    const std::string response =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n\r\n";

    return sendAll(client, response.c_str(), response.size());
}

bool DashboardServer::proxyCameraStream(SocketHandle client, const std::string& request,
                                        int streamPort, const char* streamPath,
                                        bool acceptsDisplayMode)
{
    // O vídeo em alta taxa vem do servidor MJPEG do script Python.
    // O proxy mantém o navegador usando a mesma porta do dashboard.
    SocketHandle cameraSocket = socket(AF_INET, SOCK_STREAM, 0);
#ifdef _WIN32
    if (cameraSocket == INVALID_SOCKET)
#else
    if (cameraSocket < 0)
#endif
    {
        return false;
    }

    sockaddr_in address = {};
    address.sin_family = AF_INET;
    address.sin_port = htons(streamPort);
    address.sin_addr.s_addr = htonl(0x7f000001u);

    if (connect(cameraSocket, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
    {
#ifdef _WIN32
        closesocket(cameraSocket);
#else
        close(cameraSocket);
#endif
        return false;
    }

    // Somente o modo visual é encaminhado ao processo da câmera. O parâmetro
    // não entra no IPC de visão e não pode alterar decisões do robô.
    std::string displayMode = "real";
    if (acceptsDisplayMode)
    {
        const std::size_t methodEnd = request.find(' ');
        if (methodEnd != std::string::npos)
        {
            const std::size_t targetStart = methodEnd + 1;
            const std::size_t targetEnd = request.find(' ', targetStart);
            if (targetEnd != std::string::npos)
            {
                const std::string target = request.substr(
                    targetStart, targetEnd - targetStart);
                std::size_t modeStart = target.find("?mode=");
                if (modeStart == std::string::npos)
                {
                    modeStart = target.find("&mode=");
                }
                if (modeStart != std::string::npos)
                {
                    const std::size_t valueStart = modeStart + 6;
                    const std::size_t valueEnd = target.find('&', valueStart);
                    const std::string requestedMode = target.substr(
                        valueStart, valueEnd - valueStart);
                    if (requestedMode == "real" || requestedMode == "line")
                    {
                        displayMode = requestedMode;
                    }
                }
            }
        }
    }

    std::ostringstream cameraRequest;
    cameraRequest << "GET " << streamPath;
    if (acceptsDisplayMode)
    {
        cameraRequest << "?mode=" << displayMode;
    }
    cameraRequest << " HTTP/1.1\r\n"
                  << "Host: 127.0.0.1:" << streamPort << "\r\n"
                  << "Connection: close\r\n\r\n";

    const std::string requestText = cameraRequest.str();
    if (!sendAll(cameraSocket, requestText.c_str(), requestText.size()))
    {
#ifdef _WIN32
        closesocket(cameraSocket);
#else
        close(cameraSocket);
#endif
        return false;
    }

    char buffer[8192] = {};
    while (true)
    {
        SocketResult bytesRead = recv(cameraSocket, buffer, sizeof(buffer), 0);
        if (bytesRead <= 0)
        {
            break;
        }

        if (!sendAll(client, buffer, static_cast<size_t>(bytesRead)))
        {
            break;
        }
    }

#ifdef _WIN32
    closesocket(cameraSocket);
#else
    close(cameraSocket);
#endif

    return true;
}

bool DashboardServer::sendCameraStatus(SocketHandle client, const char* statusPath)
{
    // O status da câmera é gerado pelo script Python em JSON simples.
    // Se ele não existir, a dashboard recebe um estado claro sem afetar o controle do robô.
    std::ifstream file(statusPath, std::ios::binary);
    std::string status;
    if (file)
    {
        std::ostringstream content;
        content << file.rdbuf();
        status = content.str();
    }

    if (status.empty())
    {
        status = "{\"fps\":0.0,\"active\":false}";
    }

    std::ostringstream response;
    response << "HTTP/1.1 200 OK\r\n"
             << "Content-Type: application/json; charset=utf-8\r\n"
             << "Content-Length: " << status.size() << "\r\n"
             << "Cache-Control: no-store\r\n"
             << "Connection: close\r\n\r\n";

    std::string header = response.str();
    return sendAll(client, header.c_str(), header.size()) && sendAll(client, status.c_str(), status.size());
}


bool DashboardServer::setForwardCameraEnabled(bool enabled)
{
    // A troca atômica impede que o processo Python leia um comando incompleto.
    // Este IPC controla apenas a CAM1 e nunca altera o estado ou os motores.
    {
        std::ofstream control(config::kForwardCameraTemporaryControlPath,
                              std::ios::trunc);
        if (!control)
        {
            return false;
        }
        control << (enabled ? "1\n" : "0\n");
        if (!control)
        {
            return false;
        }
    }

    if (std::rename(config::kForwardCameraTemporaryControlPath,
                    config::kForwardCameraControlPath) != 0)
    {
        std::remove(config::kForwardCameraTemporaryControlPath);
        return false;
    }
    return true;
}

bool DashboardServer::setLineCameraEnabled(bool enabled)
{
    // A troca atômica impede que o gerenciador leia um comando incompleto.
    // Este IPC não executa comandos Linux e não tem efeito direto nos motores.
    {
        std::ofstream control(config::kLineCameraTemporaryControlPath,
                              std::ios::trunc);
        if (!control)
        {
            return false;
        }
        control << (enabled ? "1\n" : "0\n");
        if (!control)
        {
            return false;
        }
    }

    if (std::rename(config::kLineCameraTemporaryControlPath,
                    config::kLineCameraControlPath) != 0)
    {
        std::remove(config::kLineCameraTemporaryControlPath);
        return false;
    }
    return true;
}

bool DashboardServer::sendAll(SocketHandle client, const char* data, size_t size)
{
    size_t sent = 0;
    while (sent < size)
    {
        SocketResult result = send(client, data + sent, static_cast<int>(size - sent), 0);
        if (result <= 0)
        {
            return false;
        }
        sent += static_cast<size_t>(result);
    }

    return true;
}

std::string DashboardServer::getHeaderValue(const std::string& request, const std::string& header)
{
    const std::string lowerRequest = lowerCopy(request);
    const std::string lowerHeader = lowerCopy(header);
    size_t start = lowerRequest.find(lowerHeader);
    if (start == std::string::npos)
    {
        return "";
    }

    start += header.size();
    size_t end = request.find("\r\n", start);
    return request.substr(start, end - start);
}

bool DashboardServer::sendWebSocketText(SocketHandle client, const std::string& message)
{
    std::vector<uint8_t> frame;
    frame.push_back(0x81);

    if (message.size() <= 125)
    {
        frame.push_back(static_cast<uint8_t>(message.size()));
    }
    else
    {
        frame.push_back(126);
        frame.push_back(static_cast<uint8_t>((message.size() >> 8) & 0xff));
        frame.push_back(static_cast<uint8_t>(message.size() & 0xff));
    }

    frame.insert(frame.end(), message.begin(), message.end());
    return send(client, reinterpret_cast<const char*>(frame.data()), static_cast<int>(frame.size()), 0) ==
           static_cast<SocketResult>(frame.size());
}

bool DashboardServer::readWebSocketFrame(SocketHandle client, std::string& payload)
{
    uint8_t header[2] = {};
    SocketResult readBytes = recv(client, reinterpret_cast<char*>(header), 2, MSG_WAITALL);
    if (readBytes <= 0)
    {
        return false;
    }

    uint8_t opcode = header[0] & 0x0f;
    if (opcode == 0x8)
    {
        return false;
    }

    bool masked = (header[1] & 0x80) != 0;
    uint64_t length = header[1] & 0x7f;

    if (length == 126)
    {
        uint8_t extended[2] = {};
        if (recv(client, reinterpret_cast<char*>(extended), 2, MSG_WAITALL) <= 0)
        {
            return false;
        }
        length = (extended[0] << 8) | extended[1];
    }
    else if (length == 127)
    {
        return false;
    }

    uint8_t mask[4] = {};
    if (masked && recv(client, reinterpret_cast<char*>(mask), 4, MSG_WAITALL) <= 0)
    {
        return false;
    }

    std::vector<uint8_t> data(length);
    if (length > 0 &&
        recv(client, reinterpret_cast<char*>(data.data()), static_cast<int>(length), MSG_WAITALL) <= 0)
    {
        return false;
    }

    payload.clear();
    for (uint64_t i = 0; i < length; ++i)
    {
        uint8_t byte = masked ? (data[i] ^ mask[i % 4]) : data[i];
        payload.push_back(static_cast<char>(byte));
    }

    return true;
}

bool DashboardServer::getJsonBool(const std::string& json, const std::string& key,
                                  bool& value)
{
    const std::string marker = "\"" + key + "\":";
    size_t start = json.find(marker);
    if (start == std::string::npos)
    {
        return false;
    }
    start += marker.size();
    while (start < json.size() &&
           (json[start] == ' ' || json[start] == '\t'))
    {
        ++start;
    }
    if (json.compare(start, 4, "true") == 0)
    {
        value = true;
        return true;
    }
    if (json.compare(start, 5, "false") == 0)
    {
        value = false;
        return true;
    }
    return false;
}

double DashboardServer::getJsonNumber(const std::string& json, const std::string& key, double fallback)
{
    std::string marker = "\"" + key + "\":";
    size_t start = json.find(marker);
    if (start == std::string::npos)
    {
        return fallback;
    }

    start += marker.size();
    size_t end = json.find_first_of(",}", start);
    try
    {
        return std::stod(json.substr(start, end - start));
    }
    catch (const std::exception&)
    {
        std::cerr << "Dashboard command ignored: invalid numeric value for " << key << "\n";
        return fallback;
    }
}

std::string DashboardServer::getJsonString(const std::string& json,
                                           const std::string& key,
                                           const std::string& fallback)
{
    const std::string marker = "\"" + key + "\":";
    size_t position = json.find(marker);
    if (position == std::string::npos)
    {
        return fallback;
    }

    position += marker.size();
    while (position < json.size() &&
           (json[position] == ' ' || json[position] == '\t'))
    {
        ++position;
    }
    if (position >= json.size() || json[position] != '"')
    {
        return fallback;
    }

    std::string value;
    for (++position; position < json.size(); ++position)
    {
        const char character = json[position];
        if (character == '"')
        {
            return value;
        }
        if (character != '\\')
        {
            value.push_back(character);
            continue;
        }

        if (++position >= json.size())
        {
            return fallback;
        }
        const char escaped = json[position];
        if (escaped == '"' || escaped == '\\' || escaped == '/')
        {
            value.push_back(escaped);
        }
        else if (escaped == 'n' || escaped == 'r' || escaped == 't')
        {
            // Controles não são úteis na OLED e viram espaço antes da
            // sanitização final feita pelo Esp32Bridge.
            value.push_back(' ');
        }
        else
        {
            return fallback;
        }
    }
    return fallback;
}

unsigned int DashboardServer::leftRotate(unsigned int value, int bits)
{
    return (value << bits) | (value >> (32 - bits));
}

std::string DashboardServer::sha1(const std::string& input)
{
    std::vector<uint8_t> data(input.begin(), input.end());
    uint64_t bitLength = static_cast<uint64_t>(data.size()) * 8;

    data.push_back(0x80);
    while ((data.size() % 64) != 56)
    {
        data.push_back(0);
    }

    for (int i = 7; i >= 0; --i)
    {
        data.push_back(static_cast<uint8_t>((bitLength >> (i * 8)) & 0xff));
    }

    uint32_t h0 = 0x67452301;
    uint32_t h1 = 0xefcdab89;
    uint32_t h2 = 0x98badcfe;
    uint32_t h3 = 0x10325476;
    uint32_t h4 = 0xc3d2e1f0;

    for (size_t chunk = 0; chunk < data.size(); chunk += 64)
    {
        uint32_t w[80] = {};

        for (int i = 0; i < 16; ++i)
        {
            size_t j = chunk + i * 4;
            w[i] = (static_cast<uint32_t>(data[j]) << 24) |
                   (static_cast<uint32_t>(data[j + 1]) << 16) |
                   (static_cast<uint32_t>(data[j + 2]) << 8) |
                   static_cast<uint32_t>(data[j + 3]);
        }

        for (int i = 16; i < 80; ++i)
        {
            w[i] = leftRotate(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
        }

        uint32_t a = h0;
        uint32_t b = h1;
        uint32_t c = h2;
        uint32_t d = h3;
        uint32_t e = h4;

        for (int i = 0; i < 80; ++i)
        {
            uint32_t f = 0;
            uint32_t k = 0;

            if (i < 20)
            {
                f = (b & c) | ((~b) & d);
                k = 0x5a827999;
            }
            else if (i < 40)
            {
                f = b ^ c ^ d;
                k = 0x6ed9eba1;
            }
            else if (i < 60)
            {
                f = (b & c) | (b & d) | (c & d);
                k = 0x8f1bbcdc;
            }
            else
            {
                f = b ^ c ^ d;
                k = 0xca62c1d6;
            }

            uint32_t temp = leftRotate(a, 5) + f + e + k + w[i];
            e = d;
            d = c;
            c = leftRotate(b, 30);
            b = a;
            a = temp;
        }

        h0 += a;
        h1 += b;
        h2 += c;
        h3 += d;
        h4 += e;
    }

    std::string digest;
    for (uint32_t h : {h0, h1, h2, h3, h4})
    {
        for (int i = 3; i >= 0; --i)
        {
            digest.push_back(static_cast<char>((h >> (i * 8)) & 0xff));
        }
    }

    return digest;
}

std::string DashboardServer::base64Encode(const std::string& input)
{
    static const char* chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string output;
    int val = 0;
    int valb = -6;

    for (uint8_t c : input)
    {
        val = (val << 8) + c;
        valb += 8;
        while (valb >= 0)
        {
            output.push_back(chars[(val >> valb) & 0x3f]);
            valb -= 6;
        }
    }

    if (valb > -6)
    {
        output.push_back(chars[((val << 8) >> (valb + 8)) & 0x3f]);
    }

    while (output.size() % 4)
    {
        output.push_back('=');
    }

    return output;
}
