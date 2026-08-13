#include "obr/dashboard_server.h"

#include "obr/config.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

namespace
{
std::string lowerCopy(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return text;
}
}

DashboardServer::DashboardServer(RobotState& robotState, Telemetry& telemetry, Esp32Bridge& esp32,
                                 MotorController& motors, StatusLed& readyLed)
    : robotState_(robotState), telemetry_(telemetry), esp32_(esp32),
      motors_(motors), readyLed_(readyLed)
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
    address.sin_addr.s_addr = INADDR_ANY;
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

    while (running_)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(config::kTelemetryPeriodMs));

        TelemetrySample sample = telemetry_.read();
        std::string json = buildTelemetryJson(sample);
        broadcast(json);

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
        if (!proxyCameraStream(client))
        {
            sendHttpNotFound(client);
        }
        closeSocket(client);
        return;
    }

    if (request.find("HEAD /camera-stream.mjpg") == 0)
    {
        if (!sendCameraStreamHead(client))
        {
            sendHttpNotFound(client);
        }
        closeSocket(client);
        return;
    }

    if (request.find("GET /camera-status.json") == 0)
    {
        sendCameraStatus(client);
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

void DashboardServer::handleCommand(const std::string& message)
{
    if (message.find("\"command\":\"estop\"") != std::string::npos)
    {
        robotState_.emergencyStop();
        std::cout << "Emergency stop received\n";
    }
    else if (message.find("\"command\":\"start\"") != std::string::npos)
    {
        esp32_.sendClearEmergencyStop();
        robotState_.start();
        std::cout << "Start received\n";
    }
    else if (message.find("\"command\":\"auto\"") != std::string::npos)
    {
        esp32_.sendClearEmergencyStop();
        robotState_.startAutonomous();
        std::cout << "Autonomous start received\n";
    }
    else if (message.find("\"command\":\"set_autonomous_mission\"") != std::string::npos)
    {
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
        else
        {
            // Missões desconhecidas são ignoradas para nunca executar um
            // comportamento diferente daquele selecionado pelo operador.
            std::cerr << "Invalid autonomous mission ignored\n";
        }
    }
    else if (message.find("\"command\":\"stop\"") != std::string::npos)
    {
        robotState_.stop();
        std::cout << "Stop received\n";
    }
    else if (message.find("\"command\":\"calibrate\"") != std::string::npos)
    {
        // A calibração nunca preserva um comando de movimento anterior.
        // A ESP32 também trava os motores até uma nova partida explícita.
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

std::string DashboardServer::buildTelemetryJson(const TelemetrySample& sample) const
{
    RobotSnapshot state = robotState_.snapshot();
    Esp32TelemetrySnapshot esp32 = esp32_.telemetrySnapshot();
    MotorSynchronizationSnapshot motorSync = motors_.synchronizationSnapshot();

    std::ostringstream json;
    json << std::fixed << std::setprecision(2)
         << "{\"cpu\":" << sample.cpuUsage
         << ",\"temperature\":" << sample.temperature
         << ",\"ram\":" << sample.ramUsage
         << ",\"mode\":\"" << state.mode << "\""
         << ",\"autonomousMission\":\"" << autonomousMissionName(state.autonomousMission) << "\""
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
         << ",\"left\":" << state.left
         << ",\"right\":" << state.right
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
         << ",\"oledOk\":" << (esp32.oledOk ? "true" : "false")
         << ",\"esp32RemoteOledActive\":" << (esp32.remoteOledActive ? "true" : "false")
         << ",\"esp32RaspberrySystemReady\":" << (esp32.raspberrySystemReady ? "true" : "false")
         << ",\"motorSleepPinHigh\":" << (esp32.motorSleepPinHigh ? "true" : "false")
         << ",\"esp32EmergencyStop\":" << (esp32.emergencyStopActive ? "true" : "false")
         << ",\"esp32CalibrationActive\":" << (esp32.calibrationActive ? "true" : "false")
         << ",\"esp32CalibrationStatusKnown\":" << (esp32.calibrationStatusKnown ? "true" : "false")
         << ",\"esp32LastCalibrationSucceeded\":" << (esp32.lastCalibrationSucceeded ? "true" : "false")
         << ",\"esp32UptimeMs\":" << esp32.esp32UptimeMs
         << ",\"raspberryCommandTimeoutMs\":" << config::kCommandTimeoutMs
         << ",\"esp32MotorCommandTimeoutMs\":" << config::kEsp32MotorCommandTimeoutMs
         << ",\"operationalMinimumMotorPower\":" << config::kOperationalMinimumMotorPower
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
  <meta name="theme-color" content="#071016">
  <title>OBR 2026 · Mission Control</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      --bg: #071016;
      --surface: #0d1922;
      --surface-2: #101f2a;
      --line: #203542;
      --line-soft: #172b36;
      --text: #edf8fc;
      --muted: #89a4b2;
      --cyan: #22d3ee;
      --cyan-soft: #103b47;
      --yellow: #fbbf24;
      --green: #34d399;
      --danger: #fb7185;
      --danger-dark: #4b1823;
    }
    * { box-sizing: border-box; }
    body { margin: 0; min-height: 100vh; color: var(--text); background: radial-gradient(circle at 15% -10%, #123343 0, transparent 34%), radial-gradient(circle at 90% 5%, #192f34 0, transparent 26%), var(--bg); }
    button, input { font: inherit; }
    .shell { width: min(1420px, calc(100% - 36px)); margin: 0 auto; padding: 24px 0 42px; }
    .topbar { display: flex; align-items: center; justify-content: space-between; gap: 20px; margin-bottom: 18px; }
    .brand { display: flex; align-items: center; gap: 13px; }
    .brand-mark { width: 52px; height: 52px; display: grid; place-items: center; border: 1px solid #397085; border-radius: 16px; color: var(--cyan); background: linear-gradient(145deg, #102b37, #09151d); box-shadow: inset 0 0 18px #22d3ee12, 0 10px 30px #0005; font-weight: 900; letter-spacing: -.08em; }
    .eyebrow { display: block; margin-bottom: 3px; color: var(--cyan); font-size: .72rem; font-weight: 850; letter-spacing: .18em; text-transform: uppercase; }
    h1 { margin: 0; font-size: clamp(1.5rem, 3vw, 2.35rem); line-height: 1; letter-spacing: -.045em; }
    .status-cluster { display: flex; flex-wrap: wrap; justify-content: flex-end; gap: 8px; }
    .status-pill { min-height: 37px; display: inline-flex; align-items: center; gap: 8px; padding: 8px 12px; border: 1px solid var(--line); border-radius: 999px; background: #0b1720dd; color: var(--muted); font-size: .8rem; font-weight: 800; }
    .status-pill::before { content: ""; width: 7px; height: 7px; border-radius: 50%; background: #526976; box-shadow: 0 0 0 4px #52697618; }
    .status-pill.ok { color: #a7f3d0; border-color: #245d50; background: #0c2825; }
    .status-pill.ok::before { background: var(--green); box-shadow: 0 0 0 4px #34d39918, 0 0 12px #34d39988; }
    .status-pill.warn { color: #fde68a; border-color: #665124; background: #2a2311; }
    .status-pill.warn::before { background: var(--yellow); box-shadow: 0 0 12px #fbbf2488; }
    .status-pill.danger { color: #fecdd3; border-color: #7b3040; background: var(--danger-dark); }
    .status-pill.danger::before { background: var(--danger); box-shadow: 0 0 12px #fb718599; }
    .hero-grid { display: grid; grid-template-columns: 1.05fr 1.45fr 1.15fr 1.35fr; gap: 12px; margin-bottom: 12px; }
    .hero-grid > *, .main-grid > *, .telemetry-grid > * { min-width: 0; }
    .card { position: relative; border: 1px solid var(--line); border-radius: 16px; background: linear-gradient(145deg, #10202a, #0b171f); box-shadow: 0 14px 34px #0003; overflow: hidden; }
    .hero-card { min-height: 140px; padding: 18px; }
    .hero-card::after { content: ""; position: absolute; width: 100px; height: 100px; right: -42px; top: -48px; border-radius: 50%; background: var(--glow, #22d3ee); opacity: .08; filter: blur(3px); }
    .card-label { margin: 0 0 10px; color: var(--muted); font-size: .72rem; font-weight: 850; letter-spacing: .13em; text-transform: uppercase; }
    .hero-value { margin: 0; font-size: clamp(1.8rem, 3.3vw, 2.9rem); line-height: 1; font-weight: 900; letter-spacing: -.05em; }
    .hero-value.yellow { color: var(--yellow); }
    .hero-value.status-good { color: #6ee7b7; text-shadow: 0 0 20px #34d39935; }
    .hero-value.status-warn { color: #fde68a; text-shadow: 0 0 20px #fbbf2430; }
    .hero-value.status-bad { color: #fda4af; text-shadow: 0 0 20px #fb718530; }
    .hero-detail { margin: 10px 0 0; color: var(--muted); font-size: .82rem; }
    .battery-track { height: 8px; margin-top: 14px; padding: 2px; border: 1px solid #705822; border-radius: 99px; background: #241f12; }
    .battery-fill { width: 0; height: 100%; border-radius: inherit; background: linear-gradient(90deg, #f59e0b, #fde047); box-shadow: 0 0 12px #fbbf2466; transition: width .25s ease; }
    .raspberry-card { display: flex; flex-direction: column; }
    .system-metrics { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 7px; }
    .system-metric { --metric: var(--cyan); --metric-surface: #0b3340; --metric-border: #226273; --metric-label: #8be6f2; position: relative; min-width: 0; padding: 9px 9px 8px; border: 1px solid var(--metric-border); border-radius: 11px; background: linear-gradient(145deg, var(--metric-surface), #09161d); box-shadow: inset 0 1px 0 #ffffff0b, 0 7px 18px #0002; overflow: hidden; transition: border-color .2s, background .2s, box-shadow .2s; }
    .system-metric::after { content: ""; position: absolute; width: 42px; height: 42px; right: -19px; top: -21px; border-radius: 50%; background: var(--metric); opacity: .16; filter: blur(1px); }
    .system-metric.cpu { --metric: #22d3ee; --metric-surface: #0b3542; --metric-border: #226273; --metric-label: #8be6f2; }
    .system-metric.ram { --metric: #a78bfa; --metric-surface: #292149; --metric-border: #574d86; --metric-label: #c9bcff; }
    .system-metric.temperature { --metric: #34d399; --metric-surface: #123b32; --metric-border: #286858; --metric-label: #91e9c5; }
    .system-metric[data-level="warn"] { --metric: #fbbf24; --metric-surface: #3a2d0d; --metric-border: #755c1d; --metric-label: #fde68a; }
    .system-metric[data-level="danger"] { --metric: #fb7185; --metric-surface: #481723; --metric-border: #8b3445; --metric-label: #fecdd3; box-shadow: inset 0 1px 0 #ffffff0b, 0 0 18px #fb71852b; }
    .metric-header { position: relative; z-index: 1; display: flex; align-items: center; justify-content: space-between; gap: 5px; color: var(--metric-label); font-size: .61rem; font-weight: 850; letter-spacing: .1em; text-transform: uppercase; }
    .metric-dot { width: 6px; height: 6px; flex: 0 0 auto; border-radius: 50%; background: var(--metric); box-shadow: 0 0 9px var(--metric); }
    .system-metric strong { position: relative; z-index: 1; display: block; margin-top: 5px; color: var(--text); font-size: clamp(1.05rem, 1.6vw, 1.4rem); line-height: 1; letter-spacing: -.035em; white-space: nowrap; font-variant-numeric: tabular-nums; }
    .metric-track { position: relative; z-index: 1; height: 4px; margin-top: 8px; border-radius: 99px; background: #041017b8; overflow: hidden; }
    .metric-fill { width: 0; height: 100%; border-radius: inherit; background: var(--metric); box-shadow: 0 0 9px var(--metric); transition: width .3s ease, background .2s; }
    .raspberry-role { display: flex; align-items: center; gap: 6px; margin: 9px 0 0; color: var(--muted); font-size: .68rem; }
    .raspberry-role::before { content: ""; width: 5px; height: 5px; flex: 0 0 auto; border-radius: 50%; background: #a78bfa; box-shadow: 0 0 8px #a78bfa; }
    .main-grid { display: grid; grid-template-columns: minmax(0, 1.65fr) minmax(360px, .85fr); gap: 12px; margin-bottom: 24px; align-items: start; }
    .side-stack { display: grid; gap: 12px; min-width: 0; }
    .section-card { padding: 16px; }
    .section-header { display: flex; align-items: center; justify-content: space-between; gap: 14px; margin-bottom: 13px; }
    .section-title { display: flex; align-items: center; gap: 9px; margin: 0; font-size: .86rem; letter-spacing: .1em; text-transform: uppercase; }
    .section-title::before { content: ""; width: 3px; height: 16px; border-radius: 4px; background: var(--cyan); box-shadow: 0 0 12px #22d3ee88; }
    .camera-meta { display: flex; flex-wrap: wrap; justify-content: flex-end; gap: 6px; }
    .meta-chip { padding: 5px 8px; border: 1px solid var(--line); border-radius: 7px; background: #09141b; color: var(--muted); font-size: .7rem; }
    .meta-chip strong { color: var(--text); font-variant-numeric: tabular-nums; }
    .camera-frame { position: relative; aspect-ratio: 16 / 9; border: 1px solid #263d49; border-radius: 11px; overflow: hidden; background: linear-gradient(135deg, #081218, #0b1c25); }
    .camera-frame img { display: block; width: 100%; height: 100%; object-fit: contain; }
    .camera-frame.offline img { opacity: 0; }
    .camera-message { position: absolute; inset: 0; display: grid; place-items: center; color: var(--muted); text-align: center; padding: 18px; }
    .camera-frame:not(.offline) .camera-message { display: none; }
    .camera-diagnostics { display: grid; grid-template-columns: repeat(auto-fit, minmax(170px, 1fr)); gap: 8px; margin-top: 10px; }
    .camera-diagnostic-group { --diagnostic-color: var(--cyan); min-width: 0; padding: 10px 11px; border: 1px solid color-mix(in srgb, var(--diagnostic-color) 35%, var(--line)); border-radius: 10px; background: color-mix(in srgb, var(--diagnostic-color) 7%, #09151d); }
    .camera-diagnostic-group.near { --diagnostic-color: #60a5fa; }
    .camera-diagnostic-group.far { --diagnostic-color: #fb923c; }
    .camera-diagnostic-group.control { --diagnostic-color: var(--yellow); }
    .camera-diagnostic-group h3 { margin: 0 0 7px; color: var(--diagnostic-color); font-size: .65rem; letter-spacing: .12em; text-transform: uppercase; }
    .camera-diagnostic-list { display: grid; gap: 4px; margin: 0; }
    .camera-diagnostic-item { display: flex; align-items: baseline; justify-content: space-between; gap: 8px; min-width: 0; }
    .camera-diagnostic-item dt { color: var(--muted); font-size: .62rem; }
    .camera-diagnostic-item dd { margin: 0; color: var(--text); font-size: .69rem; font-weight: 800; font-variant-numeric: tabular-nums; overflow-wrap: anywhere; }
    .mission-state-card { --state-color: var(--cyan); background: radial-gradient(circle at 100% 0, #22d3ee14, transparent 38%), linear-gradient(145deg, #10202a, #0b171f); }
    .mission-state-card[data-tone="active"] { --state-color: var(--green); }
    .mission-state-card[data-tone="warn"] { --state-color: var(--yellow); }
    .mission-state-card[data-tone="danger"] { --state-color: var(--danger); }
    .mission-state-card .section-title::before { background: var(--state-color); box-shadow: 0 0 12px color-mix(in srgb, var(--state-color) 55%, transparent); }
    .machine-badge { display: inline-flex; align-items: center; gap: 7px; max-width: 52%; padding: 6px 9px; border: 1px solid color-mix(in srgb, var(--state-color) 45%, #233945); border-radius: 999px; color: var(--state-color); background: color-mix(in srgb, var(--state-color) 9%, #08141b); font-size: .62rem; font-weight: 900; letter-spacing: .08em; text-transform: uppercase; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
    .machine-badge::before { content: ""; width: 7px; height: 7px; flex: 0 0 auto; border-radius: 50%; background: var(--state-color); box-shadow: 0 0 11px var(--state-color); }
    .machine-action { min-height: 72px; padding: 13px 14px; border: 1px solid color-mix(in srgb, var(--state-color) 30%, var(--line)); border-radius: 12px; background: linear-gradient(135deg, color-mix(in srgb, var(--state-color) 10%, #09151d), #09151d); }
    .machine-action span { display: block; color: var(--muted); font-size: .62rem; font-weight: 850; letter-spacing: .11em; text-transform: uppercase; }
    .machine-action strong { display: block; margin-top: 7px; color: var(--text); font-size: 1.05rem; line-height: 1.25; }
    .machine-progress { margin: 11px 1px 13px; }
    .machine-progress-label { display: flex; justify-content: space-between; gap: 10px; margin-bottom: 6px; color: var(--muted); font-size: .65rem; }
    .machine-progress-label strong { color: var(--state-color); font-variant-numeric: tabular-nums; }
    .machine-progress-track { height: 6px; border-radius: 99px; background: #061118; overflow: hidden; }
    .machine-progress-fill { width: 0; height: 100%; border-radius: inherit; background: var(--state-color); box-shadow: 0 0 10px var(--state-color); transition: width .18s ease; }
    .machine-flow { display: grid; grid-template-columns: repeat(4, 1fr); gap: 5px; margin-bottom: 12px; }
    .machine-step { position: relative; min-width: 0; padding: 8px 3px; border: 1px solid var(--line-soft); border-radius: 8px; color: #668493; background: #08141b; text-align: center; font-size: .55rem; font-weight: 850; letter-spacing: .055em; text-transform: uppercase; transition: .2s; }
    .machine-step.active { color: #061217; border-color: var(--state-color); background: var(--state-color); box-shadow: 0 0 14px color-mix(in srgb, var(--state-color) 35%, transparent); }
    .machine-metrics { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 7px; }
    .machine-metric { min-width: 0; padding: 9px 10px; border: 1px solid var(--line-soft); border-radius: 9px; background: #09151d; }
    .machine-metric span { display: block; color: var(--muted); font-size: .58rem; letter-spacing: .06em; text-transform: uppercase; }
    .machine-metric strong { display: block; margin-top: 5px; color: var(--text); font-size: .82rem; font-variant-numeric: tabular-nums; overflow-wrap: anywhere; }
    .machine-metric strong.accent { color: var(--state-color); }
    .command-card { display: flex; flex-direction: column; gap: 14px; }
    .mission-selector { padding: 12px; border: 1px solid #315464; border-radius: 12px; background: linear-gradient(145deg, #0c2531, #09161d); box-shadow: inset 0 1px 0 #ffffff0a; }
    .mission-selector > label { display: flex; align-items: center; justify-content: space-between; gap: 10px; margin-bottom: 8px; color: #9be8f3; font-size: .68rem; font-weight: 850; letter-spacing: .1em; text-transform: uppercase; }
    .mission-selector > label::after { content: "PADRÃO: PRINCIPAL"; padding: 3px 6px; border: 1px solid #315464; border-radius: 99px; color: var(--muted); background: #07151c; font-size: .55rem; letter-spacing: .06em; }
    .mission-selector select { width: 100%; min-height: 44px; padding: 0 36px 0 12px; border: 1px solid #347085; border-radius: 9px; outline: none; color: var(--text); background: #0b202a; font: inherit; font-size: .78rem; font-weight: 850; letter-spacing: .035em; cursor: pointer; }
    .mission-selector select:focus { border-color: var(--cyan); box-shadow: 0 0 0 3px #22d3ee1c; }
    .mission-selector option { color: var(--text); background: #0b202a; }
    .mission-hint { display: block; margin-top: 7px; color: var(--muted); font-size: .66rem; line-height: 1.35; }
    .oled-editor { border: 1px solid #315464; border-radius: 12px; background: linear-gradient(145deg, #0b202a, #09161d); overflow: hidden; }
    .oled-editor summary { display: flex; align-items: center; justify-content: space-between; gap: 10px; padding: 12px; color: #9be8f3; font-size: .7rem; font-weight: 900; letter-spacing: .09em; text-transform: uppercase; cursor: pointer; list-style: none; }
    .oled-editor summary::-webkit-details-marker { display: none; }
    .oled-editor summary::after { content: "+"; color: var(--cyan); font-size: 1.1rem; line-height: 1; }
    .oled-editor[open] summary::after { content: "−"; }
    .oled-editor-body { display: grid; gap: 9px; padding: 0 12px 12px; border-top: 1px solid var(--line-soft); }
    .oled-editor-state { margin: 10px 0 0; color: var(--muted); font-size: .65rem; line-height: 1.35; }
    .oled-fields { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 8px; }
    .oled-field { min-width: 0; }
    .oled-field.wide { grid-column: 1 / -1; }
    .oled-field label { display: block; margin-bottom: 5px; color: var(--muted); font-size: .59rem; font-weight: 800; letter-spacing: .07em; text-transform: uppercase; }
    .oled-field input { width: 100%; min-width: 0; padding: 9px 10px; border: 1px solid #347085; border-radius: 8px; outline: none; color: var(--text); background: #07171f; font: inherit; font-size: .78rem; }
    .oled-field input:focus { border-color: var(--cyan); box-shadow: 0 0 0 3px #22d3ee1c; }
    .oled-actions { display: grid; grid-template-columns: repeat(2, 1fr); gap: 7px; }
    .oled-actions button { min-height: 40px; font-size: .72rem; }
    .oled-actions .oled-clear { border-color: #52632b; background: #293313; color: #ecfccb; }
    .distance-mission-settings { margin-top: 10px; padding: 10px; border: 1px solid #2b596b; border-radius: 9px; background: #07171f; }
    .distance-mission-settings[hidden] { display: none; }
    .distance-input-label { display: block; margin-bottom: 6px; color: var(--muted); font-size: .62rem; font-weight: 800; letter-spacing: .07em; text-transform: uppercase; }
    .distance-input { display: grid; grid-template-columns: minmax(0, 1fr) auto; align-items: center; gap: 8px; }
    .distance-input input { width: 100%; min-width: 0; padding: 9px 10px; border: 1px solid #347085; border-radius: 8px; outline: none; color: var(--text); background: #0b202a; font: inherit; font-size: 1rem; font-weight: 900; font-variant-numeric: tabular-nums; }
    .distance-input input:focus { border-color: var(--cyan); box-shadow: 0 0 0 3px #22d3ee1c; }
    .distance-input strong { color: var(--cyan); font-size: .75rem; }
    .distance-calibration { display: block; margin-top: 7px; color: #76aab9; font-size: .59rem; line-height: 1.35; }
    .mode-buttons { display: grid; grid-template-columns: repeat(2, 1fr); gap: 8px; }
    button { min-height: 48px; border: 1px solid #285266; border-radius: 10px; color: #dff9ff; background: #123141; font-weight: 850; cursor: pointer; transition: transform .1s, border-color .1s, background .1s; }
    button:hover { transform: translateY(-1px); border-color: var(--cyan); }
    button.active { color: #061217; border-color: var(--cyan); background: var(--cyan); }
    button.stop { border-color: #604d21; background: #332a14; color: #fde68a; }
    button.danger { border-color: #8c3144; background: #501b28; color: #ffe4e6; }
    button.danger:hover { border-color: var(--danger); background: #6d2334; }
    button.calibrate { grid-column: 1 / -1; border-color: #52632b; background: #293313; color: #ecfccb; }
    button.calibrate:hover { border-color: #a3e635; background: #35431a; }
    button:disabled { cursor: not-allowed; opacity: .48; transform: none; }
    .drive-control { padding: 12px; border: 1px solid var(--line-soft); border-radius: 12px; background: #09151d; }
    .drive-control label { display: flex; justify-content: space-between; gap: 12px; margin-bottom: 9px; color: var(--muted); font-size: .76rem; font-weight: 750; text-transform: uppercase; letter-spacing: .07em; }
    .drive-control output { color: var(--text); font-variant-numeric: tabular-nums; }
    input[type="range"] { width: 100%; accent-color: var(--cyan); }
    .manual-speed-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 8px; }
    .manual-speed-grid .drive-control { border-color: #2c596b; background: linear-gradient(145deg, #0a1b24, #09151d); }
    .manual-speed-help { grid-column: 1 / -1; margin: -2px 2px 2px; color: var(--muted); font-size: .65rem; line-height: 1.4; }
    .requested-drive { display: grid; grid-template-columns: repeat(2, 1fr); gap: 8px; }
    .request-value { padding: 11px; border: 1px solid var(--line-soft); border-radius: 10px; background: #0a171e; }
    .request-value span { display: block; color: var(--muted); font-size: .67rem; text-transform: uppercase; }
    .request-adjustment { display: grid; grid-template-columns: 34px minmax(0, 1fr) 34px; gap: 5px; margin-top: 5px; }
    .request-value input { width: 100%; min-width: 0; padding: 5px 4px; border: 1px solid #315464; border-radius: 7px; outline: none; color: var(--text); background: #07151c; font: inherit; font-size: 1.12rem; font-weight: 900; text-align: center; font-variant-numeric: tabular-nums; }
    .request-value input:focus { border-color: var(--cyan); box-shadow: 0 0 0 3px #22d3ee1c; }
    .trim-button { min-height: 34px; padding: 0; border-radius: 7px; font-size: .74rem; }
    .encoder-balance { padding: 12px; border: 1px solid var(--line-soft); border-radius: 12px; background: linear-gradient(145deg, #0a1921, #08141b); }
    .encoder-balance-header { display: flex; align-items: center; justify-content: space-between; gap: 10px; margin-bottom: 10px; }
    .encoder-balance-header strong { font-size: .73rem; letter-spacing: .07em; text-transform: uppercase; }
    .balance-status { padding: 4px 7px; border: 1px solid var(--line); border-radius: 99px; font-size: .59rem; font-weight: 900; letter-spacing: .06em; text-transform: uppercase; }
    .balance-readings { display: grid; grid-template-columns: repeat(2, 1fr); gap: 7px; }
    .balance-reading { padding: 9px; border: 1px solid var(--line-soft); border-radius: 9px; background: #07151c; }
    .balance-reading span { display: block; color: var(--muted); font-size: .59rem; text-transform: uppercase; }
    .balance-reading strong { display: block; margin-top: 4px; font-size: 1rem; font-variant-numeric: tabular-nums; }
    .balance-track { height: 4px; margin-top: 7px; border-radius: 99px; background: #112631; overflow: hidden; }
    .balance-track div { width: 0; height: 100%; border-radius: inherit; background: var(--cyan); transition: width .2s ease; }
    .balance-detail { display: flex; justify-content: space-between; gap: 10px; margin-top: 9px; color: var(--muted); font-size: .65rem; }
    .balance-detail strong { color: var(--text); font-variant-numeric: tabular-nums; }
    .balance-recommendation { margin-top: 9px; padding-top: 9px; border-top: 1px solid var(--line-soft); }
    .balance-recommendation span { color: var(--muted); font-size: .65rem; line-height: 1.35; }
    .keyboard-panel { display: flex; align-items: center; justify-content: space-between; gap: 12px; padding: 12px; border: 1px solid var(--line-soft); border-radius: 12px; background: #09151d; }
    .keyboard-copy strong { display: block; font-size: .78rem; letter-spacing: .06em; text-transform: uppercase; }
    .keyboard-copy span { display: block; max-width: 190px; margin-top: 4px; color: var(--muted); font-size: .7rem; line-height: 1.35; }
    .keys { flex: 0 0 auto; display: grid; grid-template-columns: repeat(3, 30px); grid-template-rows: repeat(2, 30px); gap: 4px; }
    .keycap { display: grid; place-items: center; border: 1px solid #315464; border-radius: 6px; background: #102530; color: #bdeffa; font-size: .72rem; font-weight: 900; box-shadow: inset 0 -2px 0 #071016; }
    .keycap.w { grid-column: 2; }
    .keycap.a { grid-column: 1; grid-row: 2; }
    .keycap.s { grid-column: 2; grid-row: 2; }
    .keycap.d { grid-column: 3; grid-row: 2; }
    .keycap.active { border-color: var(--cyan); background: var(--cyan); color: #061217; box-shadow: 0 0 14px #22d3ee66; }
    .safety-note { margin-top: auto; padding: 11px 12px; border-left: 3px solid var(--yellow); border-radius: 7px; background: #2a2311; color: #e7d9a9; font-size: .75rem; line-height: 1.45; }
    .telemetry-heading { display: flex; align-items: end; justify-content: space-between; gap: 18px; margin: 0 2px 12px; }
    .telemetry-heading h2 { margin: 0; font-size: 1.2rem; }
    .telemetry-heading p { margin: 0; color: var(--muted); font-size: .78rem; }
    .telemetry-grid { display: grid; grid-template-columns: repeat(12, minmax(0, 1fr)); gap: 12px; }
    .telemetry-card { grid-column: span 6; min-height: 230px; padding: 16px; }
    .telemetry-card.wide { grid-column: span 6; }
    .big-pair { display: grid; grid-template-columns: repeat(2, 1fr); gap: 9px; margin-bottom: 12px; }
    .big-reading { padding: 13px; border: 1px solid var(--line-soft); border-radius: 11px; background: #09151c; }
    .big-reading span { display: block; color: var(--muted); font-size: .68rem; text-transform: uppercase; letter-spacing: .06em; }
    .big-reading strong { display: block; margin-top: 6px; font-size: 1.55rem; font-variant-numeric: tabular-nums; }
    .telemetry-list { display: grid; gap: 0; }
    .telemetry-row { min-height: 34px; display: flex; align-items: center; justify-content: space-between; gap: 14px; border-bottom: 1px solid var(--line-soft); font-size: .78rem; }
    .telemetry-row:last-child { border-bottom: 0; }
    .telemetry-row span { color: var(--muted); }
    .telemetry-row strong { min-width: 0; text-align: right; overflow-wrap: anywhere; font-variant-numeric: tabular-nums; }
    .axis { color: var(--cyan); font-weight: 900; }
    .motor-bar { height: 5px; margin-top: 9px; border-radius: 99px; background: #1b303b; overflow: hidden; }
    .motor-bar > div { width: 0; height: 100%; border-radius: inherit; background: linear-gradient(90deg, #0ea5e9, #22d3ee); transition: width .15s; }
    .state-good { color: #6ee7b7; }
    .state-warn { color: #fde68a; }
    .state-bad { color: #fda4af; }
    footer { display: flex; justify-content: space-between; gap: 18px; margin-top: 18px; padding: 14px 2px; color: #668493; font-size: .72rem; }
    @media (max-width: 1050px) {
      .hero-grid { grid-template-columns: repeat(2, 1fr); }
      .main-grid { grid-template-columns: 1fr; }
      .telemetry-card, .telemetry-card.wide { grid-column: span 6; }
    }
    @media (max-width: 680px) {
      html, body { overflow-x: hidden; }
      .shell { width: calc(100% - 20px); max-width: 1420px; padding-top: 14px; }
      .topbar { align-items: flex-start; flex-direction: column; }
      .status-cluster { justify-content: flex-start; }
      .hero-grid { grid-template-columns: 1fr; }
      .hero-card { min-height: 124px; }
      .section-header, .telemetry-heading { align-items: flex-start; flex-direction: column; }
      .camera-meta { justify-content: flex-start; }
      .camera-message, .safety-note { overflow-wrap: anywhere; }
      .keyboard-copy span { max-width: none; }
      .manual-speed-grid { grid-template-columns: 1fr; }
      .manual-speed-help { grid-column: auto; }
      .telemetry-row { align-items: flex-start; flex-wrap: wrap; padding: 8px 0; }
      .telemetry-card, .telemetry-card.wide { grid-column: span 12; }
      footer { flex-direction: column; }
    }
  </style>
</head>
<body>
  <main class="shell">
    <header class="topbar">
      <div class="brand">
        <div class="brand-mark">26</div>
        <div><span class="eyebrow">OBR · Temporada 2026</span><h1>Robot Mission Control</h1></div>
      </div>
      <div class="status-cluster">
        <div id="connection" class="status-pill">PAINEL OFFLINE</div>
        <div id="esp32Link" class="status-pill">ESP32 OFFLINE</div>
        <div id="safetyStatus" class="status-pill warn">ROBÔ PARADO</div>
      </div>
    </header>

    <section class="hero-grid">
      <article class="card hero-card" style="--glow:#22d3ee">
        <p class="card-label">Modo do robô</p>
        <p id="mode" class="hero-value">--</p>
        <p id="modeDetail" class="hero-detail">Aguardando estado da Raspberry</p>
      </article>
      <article class="card hero-card" style="--glow:#fbbf24">
        <p class="card-label">Bateria de níquel · 10,5–14,0 V</p>
        <p id="batteryVoltage" class="hero-value yellow">--.-- V</p>
        <div class="battery-track"><div id="batteryFill" class="battery-fill"></div></div>
        <p class="hero-detail">ADC GPIO36: <strong id="batteryAdc">-- mV</strong></p>
      </article>
      <article class="card hero-card" style="--glow:#34d399">
        <p class="card-label">Link ESP32</p>
        <p id="esp32Status" class="hero-value status-bad">OFFLINE</p>
        <p class="hero-detail">Idade da amostra: <strong id="esp32Age">-- ms</strong></p>
        <p class="hero-detail">Uptime: <strong id="esp32Uptime">--</strong></p>
      </article>
      <article class="card hero-card raspberry-card" style="--glow:#a78bfa">
        <p class="card-label">Raspberry Pi</p>
        <div class="system-metrics">
          <div id="cpuMetric" class="system-metric cpu">
            <div class="metric-header"><span>CPU</span><i class="metric-dot"></i></div>
            <strong id="cpu">--%</strong>
            <div class="metric-track"><div id="cpuFill" class="metric-fill"></div></div>
          </div>
          <div id="ramMetric" class="system-metric ram">
            <div class="metric-header"><span>RAM</span><i class="metric-dot"></i></div>
            <strong id="ram">--%</strong>
            <div class="metric-track"><div id="ramFill" class="metric-fill"></div></div>
          </div>
          <div id="temperatureMetric" class="system-metric temperature">
            <div class="metric-header"><span>Temp.</span><i class="metric-dot"></i></div>
            <strong id="temp">-- °C</strong>
            <div class="metric-track"><div id="temperatureFill" class="metric-fill"></div></div>
          </div>
        </div>
        <p class="raspberry-role">Controle, câmera e estratégia principal</p>
      </article>
    </section>

    <section class="main-grid">
      <section class="card section-card">
        <div class="section-header">
          <h2 id="cameraTitle" class="section-title">Câmera</h2>
          <div class="camera-meta">
            <span class="meta-chip">FPS <strong id="cameraFps">--</strong></span>
            <span class="meta-chip">Papel <strong id="cameraRole">--</strong></span>
            <span class="meta-chip">Stream <strong>DIRETO</strong></span>
            <span class="meta-chip">Resolução <strong id="cameraResolution">--</strong></span>
            <span class="meta-chip">Sensor <strong id="cameraSensorMode">--</strong></span>
            <span class="meta-chip">Crop <strong id="cameraScalerCrop">--</strong></span>
            <span class="meta-chip">Formato <strong id="cameraFormat">--</strong></span>
          </div>
        </div>
        <div id="cameraFrame" class="camera-frame offline">
          <img id="cameraImage" alt="Imagem direta da câmera selecionada">
          <div class="camera-message">Aguardando o stream da câmera</div>
        </div>
        <div class="camera-diagnostics" aria-label="Diagnóstico visual da linha">
          <section class="camera-diagnostic-group near">
            <h3>Near</h3>
            <dl class="camera-diagnostic-list">
              <div class="camera-diagnostic-item"><dt>Válida</dt><dd id="cameraNearValid">—</dd></div>
              <div class="camera-diagnostic-item"><dt>Erro</dt><dd id="cameraNearError">—</dd></div>
              <div class="camera-diagnostic-item"><dt>Área</dt><dd id="cameraNearArea">—</dd></div>
              <div class="camera-diagnostic-item"><dt>Altura</dt><dd id="cameraNearHeight">—</dd></div>
            </dl>
          </section>
          <section class="camera-diagnostic-group far">
            <h3>Far</h3>
            <dl class="camera-diagnostic-list">
              <div class="camera-diagnostic-item"><dt>Válida</dt><dd id="cameraFarValid">—</dd></div>
              <div class="camera-diagnostic-item"><dt>Erro</dt><dd id="cameraFarError">—</dd></div>
              <div class="camera-diagnostic-item"><dt>Área</dt><dd id="cameraFarArea">—</dd></div>
              <div class="camera-diagnostic-item"><dt>Altura</dt><dd id="cameraFarHeight">—</dd></div>
              <div class="camera-diagnostic-item"><dt>Delta de centro</dt><dd id="cameraCenterDelta">—</dd></div>
            </dl>
          </section>
          <section class="camera-diagnostic-group control">
            <h3>Controle</h3>
            <dl class="camera-diagnostic-list">
              <div class="camera-diagnostic-item"><dt>Control error</dt><dd id="cameraControlError">—</dd></div>
              <div class="camera-diagnostic-item"><dt>Correction</dt><dd id="cameraCorrection">—</dd></div>
              <div class="camera-diagnostic-item"><dt>Left preview</dt><dd id="cameraLeftPreview">—</dd></div>
              <div class="camera-diagnostic-item"><dt>Right preview</dt><dd id="cameraRightPreview">—</dd></div>
              <div class="camera-diagnostic-item"><dt>Line sequence</dt><dd id="cameraLineSequence">—</dd></div>
            </dl>
          </section>
        </div>
      </section>

      <div class="side-stack">
      <aside id="missionStateCard" class="card section-card mission-state-card" data-tone="idle">
        <div class="section-header">
          <h2 class="section-title">Máquina de estados</h2>
          <span id="machineStateBadge" class="machine-badge">PARADO</span>
        </div>
        <div class="machine-action">
          <span>Ação atual</span>
          <strong id="machineAction">Missão parada</strong>
        </div>
        <div class="machine-progress">
          <div class="machine-progress-label"><span>Progresso da etapa</span><strong id="machineProgressValue">0%</strong></div>
          <div class="machine-progress-track"><div id="machineProgressFill" class="machine-progress-fill"></div></div>
        </div>
        <div class="machine-flow" aria-label="Fluxo da máquina de estados">
          <span id="machineStepPerception" class="machine-step">Percepção</span>
          <span id="machineStepDecision" class="machine-step">Decisão</span>
          <span id="machineStepMotion" class="machine-step">Movimento</span>
          <span id="machineStepFeedback" class="machine-step">Feedback</span>
        </div>
        <div class="machine-metrics">
          <div class="machine-metric"><span>Missão</span><strong id="machineMission">Principal</strong></div>
          <div class="machine-metric"><span>Comportamento</span><strong id="machineBehavior">NENHUM</strong></div>
          <div class="machine-metric"><span>Comando E / D</span><strong id="machineRequestedSpeed" class="accent">0.00 / 0.00</strong></div>
          <div class="machine-metric"><span>Potência real E / D</span><strong id="machineAppliedSpeed">-- / --</strong></div>
          <div class="machine-metric"><span>Encoder E / D</span><strong id="machineEncoderSpeed">-- / -- cont/s</strong></div>
          <div class="machine-metric"><span>Alvo de distância</span><strong id="machineDistanceTarget">-- cm</strong></div>
          <div class="machine-metric"><span>Distância E / D</span><strong id="machineDistanceSides">-- / -- cm</strong></div>
          <div class="machine-metric"><span>Média percorrida</span><strong id="machineDistanceAverage">-- cm</strong></div>
          <div class="machine-metric"><span>Calibração do encoder</span><strong id="machineEncoderCalibration">-- cont/cm</strong></div>
          <div class="machine-metric"><span>Modo</span><strong id="machineMode">PARADO</strong></div>
        </div>
      </aside>

      <aside class="card section-card command-card">
        <div class="section-header"><h2 class="section-title">Comando</h2></div>
        <div class="mission-selector">
          <label for="autonomousMission">Missão autônoma</label>
          <select id="autonomousMission">
            <option value="main_mission" selected>MISSÃO PRINCIPAL</option>
            <option value="turn_right_90">GIRO 90° À DIREITA</option>
            <option value="drive_distance">PERCORRER DISTÂNCIA</option>
          </select>
          <span id="missionHint" class="mission-hint">Orquestrador da prova; comportamentos ainda não instalados.</span>
          <div id="distanceMissionSettings" class="distance-mission-settings" hidden>
            <span class="distance-input-label">Distância alvo</span>
            <div class="distance-input">
              <input id="distanceTargetCm" type="number" min="1" max="300" step="1" value="20" inputmode="decimal" aria-label="Distância alvo em centímetros">
              <strong>cm</strong>
            </div>
            <small class="distance-calibration">Calibração real: 3600 contagens = 18,7 cm · 192,51 cont/cm.</small>
          </div>
        </div>
        <details class="oled-editor">
          <summary>Personalizar OLED pela Raspberry</summary>
          <div class="oled-editor-body">
            <p id="oledRemoteStatus" class="oled-editor-state">Aguardando telemetria da OLED.</p>
            <div class="oled-fields">
              <div class="oled-field wide"><label for="oledTitle">Título · até 12 caracteres</label><input id="oledTitle" maxlength="12" value="OBR 2026" placeholder="OBR 2026"></div>
              <div class="oled-field"><label for="oledFirstLine">Linha 1 · até 20</label><input id="oledFirstLine" maxlength="20" value="ROBO PRONTO" placeholder="ROBO PRONTO"></div>
              <div class="oled-field"><label for="oledSecondLine">Linha 2 · até 20</label><input id="oledSecondLine" maxlength="20" value="AGUARDANDO" placeholder="AGUARDANDO"></div>
              <div class="oled-field wide"><label for="oledDurationSeconds">Duração · 0,5 a 30 segundos</label><input id="oledDurationSeconds" type="number" min="0.5" max="30" step="0.5" value="10" inputmode="decimal"></div>
            </div>
            <div class="oled-actions">
              <button id="oledShowButton" type="button" onclick="showOledMessage()" disabled>Mostrar na OLED</button>
              <button id="oledClearButton" type="button" class="oled-clear" onclick="clearOledMessage()" disabled>Tela padrão</button>
            </div>
          </div>
        </details>
        <div class="mode-buttons">
          <button id="manualButton" onclick="sendCommand('start')">Manual</button>
          <button id="autoButton" onclick="startAutonomousMission()">Autônomo</button>
          <button id="stopButton" class="stop" onclick="sendCommand('stop')">Parar</button>
          <button id="estopButton" class="danger" onclick="sendCommand('estop')">E-Stop</button>
          <button id="calibrationButton" class="calibrate" onclick="sendCommand('calibrate')" disabled>Resetar e calibrar sensores</button>
        </div>
        <div class="keyboard-panel">
          <div class="keyboard-copy"><strong>Controle WASD</strong><span id="keyboardState">Ative o modo Manual para usar o teclado.</span></div>
          <div class="keys" aria-label="Teclas de movimento">
            <span id="keyW" class="keycap w">W</span><span id="keyA" class="keycap a">A</span><span id="keyS" class="keycap s">S</span><span id="keyD" class="keycap d">D</span>
          </div>
        </div>
        <div class="manual-speed-grid">
          <div class="drive-control">
            <label for="manualDrivePower"><span>Velocidade reta</span><output id="manualDrivePowerValue">0.65</output></label>
            <input id="manualDrivePower" type="range" min="0.65" max="0.97" step="0.01" value="0.65">
          </div>
          <div class="drive-control">
            <label for="manualTurnPower"><span>Velocidade em curva</span><output id="manualTurnPowerValue">0.65</output></label>
            <input id="manualTurnPower" type="range" min="0.65" max="0.97" step="0.01" value="0.65">
          </div>
          <div class="manual-speed-help">Os dois limites começam no menor comando operacional real. A/D gira os dois lados em sentidos opostos e tem prioridade sobre W/S.</div>
        </div>
        <div class="drive-control">
          <label for="throttle"><span>Frente / ré</span><output id="throttleValue">0.00</output></label>
          <input id="throttle" type="range" min="-1" max="1" step="0.01" value="0">
        </div>
        <div class="drive-control">
          <label for="turn"><span>Giro</span><output id="turnValue">0.00</output></label>
          <input id="turn" type="range" min="-1" max="1" step="0.01" value="0">
        </div>
        <div class="requested-drive">
          <div class="request-value"><span>Lado esquerdo · ajuste exato</span><div class="request-adjustment"><button class="trim-button" data-side="left" data-delta="-0.01" aria-label="Reduzir lado esquerdo em 0,01">−.01</button><input id="leftValue" aria-label="Potência exata do lado esquerdo" type="number" min="-1" max="1" step="0.01" value="0.00" inputmode="decimal"><button class="trim-button" data-side="left" data-delta="0.01" aria-label="Aumentar lado esquerdo em 0,01">+.01</button></div></div>
          <div class="request-value"><span>Lado direito · ajuste exato</span><div class="request-adjustment"><button class="trim-button" data-side="right" data-delta="-0.01" aria-label="Reduzir lado direito em 0,01">−.01</button><input id="rightValue" aria-label="Potência exata do lado direito" type="number" min="-1" max="1" step="0.01" value="0.00" inputmode="decimal"><button class="trim-button" data-side="right" data-delta="0.01" aria-label="Aumentar lado direito em 0,01">+.01</button></div></div>
        </div>
        <div class="encoder-balance">
          <div class="encoder-balance-header"><strong>Sincronização automática pelos encoders</strong><span id="balanceStatus" class="balance-status state-warn">AGUARDANDO</span></div>
          <div class="balance-readings">
            <div class="balance-reading"><span>Esquerda</span><strong id="balanceLeftRate">-- cont/s</strong><div class="balance-track"><div id="balanceLeftBar"></div></div></div>
            <div class="balance-reading"><span>Direita</span><strong id="balanceRightRate">-- cont/s</strong><div class="balance-track"><div id="balanceRightBar"></div></div></div>
          </div>
          <div class="balance-detail"><span>Diferença entre os lados</span><strong id="balanceDifference">--%</strong></div>
          <div class="balance-recommendation"><span id="balanceRecommendation">O ajuste automático será ativado quando os dois lados avançarem ou recuarem juntos.</span></div>
        </div>
        <div class="safety-note">Todo comando não nulo respeita o piso real de 0,65. O sincronismo reduz apenas o lado mais rápido sem atravessar esse piso; os campos individuais desativam o sincronismo, mas preservam mínimo, clamp, E-Stop e timeouts.</div>
      </aside>
      </div>
    </section>

    <div class="telemetry-heading">
      <div><span class="eyebrow">Telemetria completa</span><h2>Estado do robô em tempo real</h2></div>
      <p>Dados recebidos da ESP32 pela UART a cada 100 ms</p>
    </div>

    <section class="telemetry-grid">
      <article class="card telemetry-card wide">
        <div class="section-header"><h3 class="section-title">Tração · 4 motores</h3></div>
        <div class="big-pair">
          <div class="big-reading"><span>Esquerda · 2 motores</span><strong id="appliedLeft">0.00</strong><div class="motor-bar"><div id="leftMotorBar"></div></div></div>
          <div class="big-reading"><span>Direita · 2 motores</span><strong id="appliedRight">0.00</strong><div class="motor-bar"><div id="rightMotorBar"></div></div></div>
        </div>
        <div class="telemetry-list">
          <div class="telemetry-row"><span>Solicitado pela Raspberry</span><strong><span id="requestedLeft">0.00</span> / <span id="requestedRight">0.00</span></strong></div>
          <div class="telemetry-row"><span>DRV8833 nSLEEP · GPIO26</span><strong id="sleepState">--</strong></div>
          <div class="telemetry-row"><span>Perfil de potência</span><strong id="motorCommandProfile">--</strong></div>
          <div class="telemetry-row"><span>E-Stop local da ESP32</span><strong id="esp32Estop">--</strong></div>
        </div>
      </article>

      <article class="card telemetry-card wide">
        <div class="section-header"><h3 class="section-title">Encoders</h3></div>
        <div class="big-pair">
          <div class="big-reading"><span>Contagem esquerda</span><strong id="leftEncoderCount">--</strong></div>
          <div class="big-reading"><span>Contagem direita</span><strong id="rightEncoderCount">--</strong></div>
        </div>
        <div class="telemetry-list">
          <div class="telemetry-row"><span>Taxa esquerda</span><strong id="leftEncoderRate">-- cont/s</strong></div>
          <div class="telemetry-row"><span>Taxa direita</span><strong id="rightEncoderRate">-- cont/s</strong></div>
          <div class="telemetry-row"><span>Posição esquerda desde o último reset</span><strong id="leftEncoderPosition">-- cm</strong></div>
          <div class="telemetry-row"><span>Posição direita desde o último reset</span><strong id="rightEncoderPosition">-- cm</strong></div>
          <div class="telemetry-row"><span>Canais</span><strong>GPIO19/21 · GPIO22/23</strong></div>
        </div>
      </article>

      <article class="card telemetry-card wide">
        <div class="section-header"><h3 class="section-title">MPU6050 · orientação</h3></div>
        <div class="big-pair">
          <div class="big-reading"><span>Giro integrado</span><strong id="yawZ">-- °</strong></div>
          <div class="big-reading"><span>Inclinação da rampa</span><strong id="rampAngle">-- °</strong></div>
        </div>
        <div class="telemetry-list">
          <div class="telemetry-row"><span>Giroscópio X / Y / Z</span><strong><b class="axis">X</b> <span id="gyroX">--</span> &nbsp; <b class="axis">Y</b> <span id="gyroY">--</span> &nbsp; <b class="axis">Z</b> <span id="gyroZ">--</span> °/s</strong></div>
          <div class="telemetry-row"><span>Aceleração X / Y / Z</span><strong><b class="axis">X</b> <span id="accelX">--</span> &nbsp; <b class="axis">Y</b> <span id="accelY">--</span> &nbsp; <b class="axis">Z</b> <span id="accelZ">--</span> m/s²</strong></div>
          <div class="telemetry-row"><span>Temperatura do IMU</span><strong id="imuTemperature">-- °C</strong></div>
        </div>
      </article>

      <article class="card telemetry-card">
        <div class="section-header"><h3 class="section-title">Sensores</h3></div>
        <div class="big-reading"><span>Ultrassônico frontal</span><strong id="ultrasonic">-- cm</strong></div>
        <div class="telemetry-list">
          <div class="telemetry-row"><span>Start button · GPIO27</span><strong id="startButtonState">--</strong></div>
          <div class="telemetry-row"><span>Calibração · segure Start por 5 s</span><strong id="calibrationState">--</strong></div>
          <div class="telemetry-row"><span>MPU6050</span><strong id="mpuState">--</strong></div>
          <div class="telemetry-row"><span>Leitura dos sensores</span><strong id="sensorFreshState">--</strong></div>
        </div>
      </article>

      <article class="card telemetry-card">
        <div class="section-header"><h3 class="section-title">I2C e periféricos</h3></div>
        <div class="telemetry-list">
          <div class="telemetry-row"><span>PCA9685 · 0x40</span><strong id="pcaState">--</strong></div>
          <div class="telemetry-row"><span>OLED SSD1306</span><strong id="oledState">--</strong></div>
          <div class="telemetry-row"><span>Inicialização geral na OLED</span><strong id="oledBootState">--</strong></div>
          <div class="telemetry-row"><span>Página enviada pela Raspberry</span><strong id="oledRemoteTelemetry">--</strong></div>
          <div class="telemetry-row"><span>Barramento I2C</span><strong>GPIO14 / GPIO13</strong></div>
          <div class="telemetry-row"><span>UART Raspberry</span><strong>115200 bps</strong></div>
          <div class="telemetry-row"><span>TX / RX ESP32</span><strong>GPIO1 / GPIO3</strong></div>
        </div>
      </article>

      <article class="card telemetry-card">
        <div class="section-header"><h3 class="section-title">Diagnóstico</h3></div>
        <div class="telemetry-list">
          <div class="telemetry-row"><span>Serial aberta</span><strong id="serialState">--</strong></div>
          <div class="telemetry-row"><span>LED de sistema · BCM GPIO26</span><strong id="readyLedState">--</strong></div>
          <div class="telemetry-row"><span>Idade da telemetria</span><strong id="telemetryAge">-- ms</strong></div>
          <div class="telemetry-row"><span>Uptime ESP32</span><strong id="diagnosticUptime">--</strong></div>
          <div class="telemetry-row"><span>Controle independente</span><strong class="state-good">esquerda / direita</strong></div>
          <div class="telemetry-row"><span>Encoders</span><strong class="state-good">sincronismo automático em movimento conjunto</strong></div>
          <div class="telemetry-row"><span>Timeout de comando · Raspberry</span><strong id="raspberryCommandTimeout">-- ms</strong></div>
          <div class="telemetry-row"><span>Watchdog de motor · ESP32</span><strong id="esp32MotorTimeout">-- ms</strong></div>
        </div>
      </article>
    </section>

    <footer><span>OBR 2026 · Plataforma de controle e telemetria</span><span>Raspberry Pi ↔ UART ↔ ESP32</span></footer>
  </main>

  <script>
    let ws;
    let manualEnabled = false;
    let cameraReconnectTimer;
    const driveKeyCodes = ["KeyW", "KeyA", "KeyS", "KeyD"];
    const pressedDriveKeys = new Set();
    const element = id => document.getElementById(id);
    const connection = element("connection");
    const throttle = element("throttle");
    const turn = element("turn");
    const manualDrivePower = element("manualDrivePower");
    const manualTurnPower = element("manualTurnPower");
    const leftValue = element("leftValue");
    const rightValue = element("rightValue");
    const cameraFrame = element("cameraFrame");
    const cameraImage = element("cameraImage");
    const cameraTitle = element("cameraTitle");
    const cameraFps = element("cameraFps");
    const cameraRole = element("cameraRole");
    const cameraResolution = element("cameraResolution");
    const cameraSensorMode = element("cameraSensorMode");
    const cameraScalerCrop = element("cameraScalerCrop");
    const cameraFormat = element("cameraFormat");
    const cameraDiagnosticFields = {
      nearValid: element("cameraNearValid"), nearError: element("cameraNearError"),
      nearArea: element("cameraNearArea"), nearHeight: element("cameraNearHeight"),
      farValid: element("cameraFarValid"), farError: element("cameraFarError"),
      farArea: element("cameraFarArea"), farHeight: element("cameraFarHeight"),
      centerDelta: element("cameraCenterDelta"), controlError: element("cameraControlError"),
      correction: element("cameraCorrection"), leftPreview: element("cameraLeftPreview"),
      rightPreview: element("cameraRightPreview"), lineSequence: element("cameraLineSequence")
    };
    const autonomousMission = element("autonomousMission");
    const distanceTargetCm = element("distanceTargetCm");
    const oledTitle = element("oledTitle");
    const oledFirstLine = element("oledFirstLine");
    const oledSecondLine = element("oledSecondLine");
    const oledDurationSeconds = element("oledDurationSeconds");
    let requestedLeft = 0;
    let requestedRight = 0;
    let rawDiagnosticDrive = false;
    let filteredLeftEncoderRate = null;
    let filteredRightEncoderRate = null;
    let manualMinimumPower = 0.65;
    let manualMaximumPower = 0.97;
    let lastCameraStatusTimestamp = null;
    let lastCameraStatusChangeAtMs = 0;

    function setPill(target, text, state) {
      target.textContent = text;
      target.className = `status-pill ${state || ""}`.trim();
    }

    function setState(id, ok, goodText = "online", badText = "indisponível") {
      const target = element(id);
      target.textContent = ok ? goodText : badText;
      target.className = ok ? "state-good" : "state-bad";
    }

    function formatNumber(value, digits = 1) {
      const number = Number(value);
      return Number.isFinite(number) ? number.toFixed(digits) : "--";
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
      const labels = { manual: "MANUAL", autonomous: "AUTÔNOMO", stopped: "PARADO", emergency: "EMERGÊNCIA" };
      element("mode").textContent = labels[mode] || mode.toUpperCase();
      const selectedMission = String(data.autonomousMission || "main_mission");
      const autonomousDetail = selectedMission === "turn_right_90"
        ? "Executando giro de 90° à direita"
        : selectedMission === "drive_distance"
          ? "Percorrendo a distância selecionada pelos encoders"
          : "Missão principal em execução";
      element("modeDetail").textContent = mode === "manual" ? "Comandos humanos habilitados" : mode === "autonomous" ? autonomousDetail : mode === "emergency" ? "Movimento bloqueado pelo E-Stop" : "Saídas de motor zeradas";
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
      element("missionHint").textContent = mission === "turn_right_90"
        ? "Usa o MPU6050, comando 0,01 com perfil operacional e frenagem preditiva."
        : mission === "drive_distance"
          ? "Avança os dois lados até o alvo medido pelos encoders."
          : "Orquestrador da prova; comportamentos ainda não instalados.";
    }

    function updateStateMachine(data) {
      const phases = {
        stopped: ["PARADO", "idle", "machineStepFeedback"],
        manual: ["CONTROLE MANUAL", "active", "machineStepDecision"],
        ready: ["MISSÃO PRONTA", "idle", "machineStepDecision"],
        starting: ["INICIALIZANDO", "warn", "machineStepPerception"],
        emergency: ["EMERGÊNCIA", "danger", "machineStepFeedback"],
        calibrating: ["CALIBRANDO", "warn", "machineStepFeedback"],
        waiting_esp32: ["ESP32 OFFLINE", "danger", "machineStepFeedback"],
        main_waiting_behaviors: ["ESTRUTURA PRONTA", "idle", "machineStepDecision"],
        waiting_imu: ["AGUARDANDO IMU", "warn", "machineStepPerception"],
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
        distance_correction_failed: ["CORREÇÃO INSUFICIENTE", "danger", "machineStepFeedback"],
        distance_invalid_target: ["ALVO INVÁLIDO", "danger", "machineStepFeedback"]
      };
      let phase = String(data.autonomousPhase || "stopped");
      let action = String(data.autonomousAction || "Aguardando estado da missão");
      if (data.emergency === true || data.esp32EmergencyStop === true) {
        phase = "emergency";
        action = "Movimento bloqueado pela parada de emergência";
      } else if (data.esp32CalibrationActive === true) {
        phase = "calibrating";
        action = "Calibrando sensores: mantenha o robô parado";
      } else if (data.mode === "autonomous" && data.esp32SerialOpen !== true) {
        phase = "waiting_esp32";
        action = "Missão sem comunicação com a ESP32: motores protegidos";
      }
      const phaseInfo = phases[phase] || [phase.replaceAll("_", " ").toUpperCase(), "idle", "machineStepDecision"];
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
        : mission === "drive_distance" ? "Percorrer distância" : "Missão principal";
      element("machineBehavior").textContent = mission === "turn_right_90"
        ? "TESTE DE GIRO"
        : mission === "drive_distance" ? "TESTE DE DISTÂNCIA" : "NENHUM";
      element("machineRequestedSpeed").textContent = `${formatNumber(data.left, 2)} / ${formatNumber(data.right, 2)}`;
      const fresh = data.esp32SensorFresh === true;
      element("machineAppliedSpeed").textContent = fresh
        ? `${formatNumber(data.esp32AppliedLeftPower, 2)} / ${formatNumber(data.esp32AppliedRightPower, 2)}`
        : "-- / --";
      element("machineEncoderSpeed").textContent = fresh
        ? `${formatNumber(data.leftEncoderRate, 0)} / ${formatNumber(data.rightEncoderRate, 0)} cont/s`
        : "-- / -- cont/s";
      const distanceMission = mission === "drive_distance";
      element("machineDistanceTarget").textContent = distanceMission ? `${formatNumber(data.driveDistanceTargetCm, 1)} cm` : "-- cm";
      element("machineDistanceSides").textContent = distanceMission
        ? `${formatNumber(data.autonomousLeftDistanceCm, 1)} / ${formatNumber(data.autonomousRightDistanceCm, 1)} cm`
        : "-- / -- cm";
      element("machineDistanceAverage").textContent = distanceMission ? `${formatNumber(data.autonomousAverageDistanceCm, 1)} cm` : "-- cm";
      element("machineEncoderCalibration").textContent = distanceMission ? `${formatNumber(data.encoderCountsPerCentimeter, 2)} cont/cm` : "-- cont/cm";
      const modeLabels = { manual: "MANUAL", autonomous: "AUTÔNOMO", stopped: "PARADO", emergency: "EMERGÊNCIA" };
      element("machineMode").textContent = modeLabels[data.mode] || String(data.mode || "--").toUpperCase();
    }

    function updateSystemMetric(metricId, fillId, value, warningValue, dangerValue) {
      const safeValue = Number.isFinite(Number(value)) ? Number(value) : 0;
      const metric = element(metricId);
      metric.dataset.level = safeValue >= dangerValue ? "danger" : (safeValue >= warningValue ? "warn" : "normal");
      element(fillId).style.width = `${Math.max(0, Math.min(100, safeValue))}%`;
    }

    function updateMotorBalance(data, fresh) {
      const appliedLeft = Math.abs(Number(data.esp32AppliedLeftPower));
      const appliedRight = Math.abs(Number(data.esp32AppliedRightPower));
      const rawLeftRate = Math.abs(Number(data.leftEncoderRate));
      const rawRightRate = Math.abs(Number(data.rightEncoderRate));
      const movingCommand = appliedLeft > 0.01 && appliedRight > 0.01;
      const valuesValid = fresh && Number.isFinite(rawLeftRate) && Number.isFinite(rawRightRate);
      const status = element("balanceStatus");

      if (!valuesValid || !movingCommand) {
        filteredLeftEncoderRate = null;
        filteredRightEncoderRate = null;
        element("balanceLeftRate").textContent = "-- cont/s";
        element("balanceRightRate").textContent = "-- cont/s";
        element("balanceLeftBar").style.width = "0%";
        element("balanceRightBar").style.width = "0%";
        element("balanceDifference").textContent = "--%";
        element("balanceRecommendation").textContent = "Mantenha os dois lados acionados para acompanhar o sincronismo.";
        status.textContent = fresh ? "ACIONE OS DOIS LADOS" : "SEM TELEMETRIA";
        status.className = "balance-status state-warn";
        return;
      }

      const smoothing = 0.25;
      filteredLeftEncoderRate = filteredLeftEncoderRate === null
        ? rawLeftRate
        : filteredLeftEncoderRate + smoothing * (rawLeftRate - filteredLeftEncoderRate);
      filteredRightEncoderRate = filteredRightEncoderRate === null
        ? rawRightRate
        : filteredRightEncoderRate + smoothing * (rawRightRate - filteredRightEncoderRate);

      const fasterRate = Math.max(filteredLeftEncoderRate, filteredRightEncoderRate);
      const differencePercent = fasterRate > 1
        ? Math.abs(filteredLeftEncoderRate - filteredRightEncoderRate) / fasterRate * 100
        : 0;
      element("balanceLeftRate").textContent = `${filteredLeftEncoderRate.toFixed(0)} cont/s`;
      element("balanceRightRate").textContent = `${filteredRightEncoderRate.toFixed(0)} cont/s`;
      element("balanceLeftBar").style.width = `${fasterRate > 0 ? filteredLeftEncoderRate / fasterRate * 100 : 0}%`;
      element("balanceRightBar").style.width = `${fasterRate > 0 ? filteredRightEncoderRate / fasterRate * 100 : 0}%`;
      element("balanceDifference").textContent = `${differencePercent.toFixed(1)}%`;

      const syncEligible = data.motorSyncEligible === true;
      const syncActive = data.motorSyncActive === true;
      const syncEncoderValid = data.motorSyncEncoderDataValid === true;
      const leftScale = Number(data.motorSyncLeftScale);
      const rightScale = Number(data.motorSyncRightScale);
      const correctedLeft = Number(data.motorSyncCorrectedLeftPower);
      const correctedRight = Number(data.motorSyncCorrectedRightPower);
      const scaleText = Number.isFinite(leftScale) && Number.isFinite(rightScale)
        ? `${leftScale.toFixed(3)} / ${rightScale.toFixed(3)}`
        : "-- / --";
      const correctedText = Number.isFinite(correctedLeft) && Number.isFinite(correctedRight)
        ? `${correctedLeft.toFixed(2)} / ${correctedRight.toFixed(2)}`
        : "-- / --";
      element("balanceRecommendation").textContent = data.rawMotorCommand === true
        ? "Ajuste individual: sincronismo automático desativado; piso de 0,65 preservado."
        : syncEligible
          ? `Escala automática E/D: ${scaleText} · PWM corrigido: ${correctedText}.`
          : "Sincronismo pausado: os lados não estão se movendo juntos no mesmo sentido.";

      if (data.rawMotorCommand === true) {
        status.textContent = "AJUSTE INDIVIDUAL";
        status.className = "balance-status state-warn";
      } else if (!syncEligible) {
        status.textContent = "PAUSADO EM GIRO";
        status.className = "balance-status state-warn";
      } else if (!syncEncoderValid) {
        status.textContent = "AGUARDANDO ENCODERS";
        status.className = "balance-status state-warn";
      } else if (!syncActive) {
        status.textContent = `APRENDENDO ${Number(data.motorSyncValidSamples) || 0}/${Number(data.encoderSyncWarmupSamples) || 3}`;
        status.className = "balance-status state-warn";
      } else if (fasterRate <= 1) {
        status.textContent = "SEM MOVIMENTO";
        status.className = "balance-status state-bad";
      } else if (differencePercent <= 2) {
        status.textContent = "SINCRONIZADO";
        status.className = "balance-status state-good";
      } else if (data.motorSyncCorrectionApplied === true) {
        status.textContent = "CORRIGINDO AUTOMATICAMENTE";
        status.className = "balance-status state-warn";
      } else {
        status.textContent = "MEDINDO DIFERENÇA";
        status.className = "balance-status state-warn";
      }
    }

    function updateEsp32Telemetry(data) {
      const receivedMinimumPower = Number(data.operationalMinimumMotorPower);
      const receivedMaximumPower = Number(data.operationalMaximumReferencePower);
      if (Number.isFinite(receivedMinimumPower) && Number.isFinite(receivedMaximumPower) &&
          receivedMinimumPower > 0 && receivedMaximumPower >= receivedMinimumPower) {
        manualMinimumPower = receivedMinimumPower;
        manualMaximumPower = receivedMaximumPower;
        manualDrivePower.min = manualMinimumPower.toFixed(2);
        manualDrivePower.max = manualMaximumPower.toFixed(2);
        manualTurnPower.min = manualMinimumPower.toFixed(2);
        manualTurnPower.max = manualMaximumPower.toFixed(2);
        manualDrivePower.value = clampManualPower(manualDrivePower.value).toFixed(2);
        manualTurnPower.value = clampManualPower(manualTurnPower.value).toFixed(2);
        element("manualDrivePowerValue").textContent = manualDrivePower.value;
        element("manualTurnPowerValue").textContent = manualTurnPower.value;
      }
      const fresh = data.esp32SensorFresh === true;
      const serialOpen = data.esp32SerialOpen === true;
      const calibrating = data.esp32CalibrationActive === true;
      const mpuOk = fresh && !calibrating && data.mpuOk === true;
      const localEmergency = data.esp32EmergencyStop === true;
      const systemEmergency = data.emergency === true || localEmergency;
      const battery = Number(data.batteryVoltage);
      const distance = Number(data.ultrasonicDistanceCm);
      const age = Number(data.esp32LastSensorAgeMs);

      setPill(element("esp32Link"), calibrating ? "ESP32 CALIBRANDO" : (serialOpen ? (fresh ? "ESP32 SINCRONIZADA" : "ESP32 SEM TELEMETRIA") : "ESP32 OFFLINE"), calibrating ? "warn" : (fresh ? "ok" : (serialOpen ? "warn" : "danger")));
      setPill(element("safetyStatus"), systemEmergency ? "E-STOP ATIVO" : (calibrating ? "CALIBRANDO" : (data.mode === "stopped" ? "ROBÔ PARADO" : "MOVIMENTO AUTORIZADO")), systemEmergency ? "danger" : ((calibrating || data.mode === "stopped") ? "warn" : "ok"));
      const esp32Status = element("esp32Status");
      esp32Status.textContent = calibrating ? "CALIBRANDO" : (fresh ? "ONLINE" : (serialOpen ? "SEM DADOS" : "OFFLINE"));
      esp32Status.className = `hero-value ${calibrating ? "status-warn" : (fresh ? "status-good" : (serialOpen ? "status-warn" : "status-bad"))}`;
      element("esp32Age").textContent = age >= 0 ? `${formatNumber(age, 0)} ms` : "-- ms";
      element("telemetryAge").textContent = age >= 0 ? `${formatNumber(age, 0)} ms` : "-- ms";
      element("sensorFreshState").textContent = fresh ? "atualizada" : "desatualizada";
      element("sensorFreshState").className = fresh ? "state-good" : "state-bad";
      element("serialState").textContent = serialOpen ? "aberta" : "fechada";
      element("serialState").className = serialOpen ? "state-good" : "state-bad";
      element("readyLedState").textContent = data.systemReady === true ? "aceso · sistema pronto" : "apagado · aguardando";
      element("readyLedState").className = data.systemReady === true ? "state-good" : "state-warn";
      element("raspberryCommandTimeout").textContent = `${formatNumber(data.raspberryCommandTimeoutMs, 0)} ms`;
      element("esp32MotorTimeout").textContent = `${formatNumber(data.esp32MotorCommandTimeoutMs, 0)} ms`;

      element("batteryVoltage").textContent = fresh ? `${formatNumber(battery, 2)} V` : "--.-- V";
      element("batteryAdc").textContent = fresh ? `${formatNumber(data.batteryAdcMillivolts, 0)} mV` : "-- mV";
      const batteryPercent = fresh ? Math.max(0, Math.min(1, (battery - 10.5) / 3.5)) : 0;
      element("batteryFill").style.width = `${batteryPercent * 100}%`;

      element("esp32Uptime").textContent = fresh ? formatUptime(data.esp32UptimeMs) : "--";
      element("diagnosticUptime").textContent = fresh ? formatUptime(data.esp32UptimeMs) : "--";
      element("requestedLeft").textContent = formatNumber(data.left, 2);
      element("requestedRight").textContent = formatNumber(data.right, 2);
      const motorCommandProfile = element("motorCommandProfile");
      motorCommandProfile.textContent = data.rawMotorCommand === true
        ? `ajuste individual · mín. ${formatNumber(data.operationalMinimumMotorPower, 2)} · sem sincronismo`
        : `operacional · mín. ${formatNumber(data.operationalMinimumMotorPower, 2)} · sincronismo automático`;
      motorCommandProfile.className = data.rawMotorCommand === true ? "state-warn" : "state-good";
      element("appliedLeft").textContent = fresh ? formatNumber(data.esp32AppliedLeftPower, 2) : "--";
      element("appliedRight").textContent = fresh ? formatNumber(data.esp32AppliedRightPower, 2) : "--";
      element("leftMotorBar").style.width = fresh ? `${Math.min(100, Math.abs(Number(data.esp32AppliedLeftPower)) * 100)}%` : "0%";
      element("rightMotorBar").style.width = fresh ? `${Math.min(100, Math.abs(Number(data.esp32AppliedRightPower)) * 100)}%` : "0%";
      if (fresh) {
        setState("sleepState", data.motorSleepPinHigh === true, "HIGH · habilitado", "LOW · verificar");
        setState("esp32Estop", !localEmergency, "liberado", "ATIVO");
      } else {
        element("sleepState").textContent = "sem dados";
        element("sleepState").className = "state-warn";
        element("esp32Estop").textContent = "sem dados";
        element("esp32Estop").className = "state-warn";
      }

      element("leftEncoderCount").textContent = fresh ? formatNumber(data.leftEncoderCount, 0) : "--";
      element("rightEncoderCount").textContent = fresh ? formatNumber(data.rightEncoderCount, 0) : "--";
      element("leftEncoderRate").textContent = fresh ? `${formatNumber(data.leftEncoderRate, 0)} cont/s` : "-- cont/s";
      element("rightEncoderRate").textContent = fresh ? `${formatNumber(data.rightEncoderRate, 0)} cont/s` : "-- cont/s";
      const countsPerCentimeter = Number(data.encoderCountsPerCentimeter);
      const encoderScaleValid = fresh && Number.isFinite(countsPerCentimeter) && countsPerCentimeter > 0;
      element("leftEncoderPosition").textContent = encoderScaleValid
        ? `${formatNumber(Number(data.leftEncoderCount) / countsPerCentimeter, 1)} cm`
        : "-- cm";
      element("rightEncoderPosition").textContent = encoderScaleValid
        ? `${formatNumber(Number(data.rightEncoderCount) / countsPerCentimeter, 1)} cm`
        : "-- cm";

      element("yawZ").textContent = mpuOk ? `${formatNumber(data.yawZDeg, 1)} °` : "-- °";
      element("rampAngle").textContent = mpuOk ? `${formatNumber(data.rampAngleDeg, 1)} °` : "-- °";
      element("gyroX").textContent = mpuOk ? formatNumber(data.gyroXDegPerSec, 1) : "--";
      element("gyroY").textContent = mpuOk ? formatNumber(data.gyroYDegPerSec, 1) : "--";
      element("gyroZ").textContent = mpuOk ? formatNumber(data.gyroZDegPerSec, 1) : "--";
      element("accelX").textContent = mpuOk ? formatNumber(data.accelX, 1) : "--";
      element("accelY").textContent = mpuOk ? formatNumber(data.accelY, 1) : "--";
      element("accelZ").textContent = mpuOk ? formatNumber(data.accelZ, 1) : "--";
      element("imuTemperature").textContent = mpuOk ? `${formatNumber(data.imuTemperatureCelsius, 1)} °C` : "-- °C";

      element("ultrasonic").textContent = fresh && distance >= 0 ? `${formatNumber(distance, 1)} cm` : "sem eco";
      element("ultrasonic").className = fresh && distance >= 0 ? "state-good" : "state-warn";
      setState("startButtonState", fresh && data.startButtonPressed === true, "pressionado", "solto");
      if (fresh && data.startButtonPressed !== true) element("startButtonState").className = "";
      const calibrationButton = element("calibrationButton");
      calibrationButton.disabled = !fresh || calibrating;
      calibrationButton.textContent = calibrating ? "Calibrando… mantenha o robô parado" : "Resetar e calibrar sensores";
      if (calibrating) {
        element("calibrationState").textContent = "em andamento";
        element("calibrationState").className = "state-warn";
      } else if (data.esp32CalibrationStatusKnown === true) {
        const calibrationOk = data.esp32LastCalibrationSucceeded === true;
        element("calibrationState").textContent = calibrationOk ? "concluída" : "falhou";
        element("calibrationState").className = calibrationOk ? "state-good" : "state-bad";
      } else {
        element("calibrationState").textContent = fresh ? "aguardando" : "sem dados";
        element("calibrationState").className = fresh ? "" : "state-warn";
      }
      setState("mpuState", mpuOk, "online", "indisponível");
      setState("pcaState", fresh && data.pca9685Ok === true, "online", "indisponível");
      setState("oledState", fresh && data.oledOk === true, "online", "indisponível");
      const oledAvailable = fresh && data.oledOk === true;
      const oledBootReady = oledAvailable && data.esp32RaspberrySystemReady === true;
      const remoteOledActive = oledBootReady && data.esp32RemoteOledActive === true;
      element("oledShowButton").disabled = !oledBootReady;
      element("oledClearButton").disabled = !oledAvailable || !remoteOledActive;
      element("oledRemoteStatus").textContent = !oledAvailable
        ? "OLED indisponível ou sem telemetria recente."
        : !oledBootReady
          ? "Inicializando Raspberry, câmera e serviços do robô."
        : remoteOledActive
          ? "Mensagem da Raspberry em exibição; depois do prazo, a tela padrão retorna."
          : "Tela padrão ativa: bateria, giro e inclinação.";
      element("oledRemoteStatus").className = `oled-editor-state ${remoteOledActive ? "state-good" : (oledBootReady ? "" : "state-warn")}`.trim();
      element("oledBootState").textContent = !oledAvailable
        ? "sem dados"
        : oledBootReady ? "concluída · tela liberada" : "em andamento · animação ativa";
      element("oledBootState").className = oledBootReady ? "state-good" : "state-warn";
      element("oledRemoteTelemetry").textContent = !oledAvailable
        ? "sem dados"
        : !oledBootReady ? "bloqueada durante o boot" : remoteOledActive ? "ativa · temporária" : "inativa · tela local";
      element("oledRemoteTelemetry").className = !oledAvailable
        ? "state-warn" : remoteOledActive ? "state-good" : "";
      updateMotorBalance(data, fresh);
    }

    function connect() {
      ws = new WebSocket(`ws://${location.host}/ws`);
      ws.onopen = () => setPill(connection, "PAINEL ONLINE", "ok");
      ws.onclose = () => { manualEnabled = false; resetKeyboardState(); resetDrive(); setPill(connection, "PAINEL OFFLINE", "danger"); window.setTimeout(connect, 1000); };
      ws.onmessage = event => {
        const data = JSON.parse(event.data);
        updateMode(data);
        updateAutonomousMission(data);
        updateStateMachine(data);
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
      if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(payload));
    }

    function sendCommand(command) {
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

    function showOledMessage() {
      const title = sanitizeOledText(oledTitle.value, 12);
      const firstLine = sanitizeOledText(oledFirstLine.value, 20);
      const secondLine = sanitizeOledText(oledSecondLine.value, 20);
      const durationSeconds = Math.max(0.5, Math.min(30, Number(oledDurationSeconds.value) || 10));
      oledTitle.value = title;
      oledFirstLine.value = firstLine;
      oledSecondLine.value = secondLine;
      oledDurationSeconds.value = durationSeconds.toFixed(1);
      if (!title.trim() && !firstLine.trim() && !secondLine.trim()) {
        element("oledRemoteStatus").textContent = "Digite pelo menos um texto antes de enviar.";
        element("oledRemoteStatus").className = "oled-editor-state state-warn";
        return;
      }
      send({ command: "oled_message", title, firstLine, secondLine, durationMs: Math.round(durationSeconds * 1000) });
      element("oledRemoteStatus").textContent = "Mensagem enviada; aguardando confirmação na telemetria.";
      element("oledRemoteStatus").className = "oled-editor-state state-warn";
    }

    function clearOledMessage() {
      send({ command: "oled_clear" });
      element("oledRemoteStatus").textContent = "Retorno à tela padrão solicitado.";
      element("oledRemoteStatus").className = "oled-editor-state state-warn";
    }

    function selectAutonomousMission() {
      manualEnabled = false;
      resetKeyboardState();
      resetDrive();
      const targetCm = Math.max(1, Math.min(300, Number(distanceTargetCm.value) || 20));
      distanceTargetCm.value = formatNumber(targetCm, 1);
      element("distanceMissionSettings").hidden = autonomousMission.value !== "drive_distance";
      send({ command: "set_autonomous_mission", mission: autonomousMission.value, distanceCm: targetCm });
    }

    function startAutonomousMission() {
      // O WebSocket preserva a ordem: primeiro confirma missão e alvo, depois inicia.
      selectAutonomousMission();
      sendCommand("auto");
    }

    function clamp(value) { return Math.max(-1, Math.min(1, value)); }

    function clampManualPower(value) {
      return Math.max(manualMinimumPower, Math.min(manualMaximumPower, Number(value) || manualMinimumPower));
    }

    function scaleOperationalAxis(axis, maximumPower) {
      const safeAxis = clamp(axis);
      if (Math.abs(safeAxis) < 0.0001) return 0;
      const magnitude = manualMinimumPower +
        (clampManualPower(maximumPower) - manualMinimumPower) * Math.abs(safeAxis);
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
      updateDriveFromMixer();
    }

    function restoreManualPowerSettings() {
      try {
        manualDrivePower.value = localStorage.getItem("obrManualDrivePower") || "0.65";
        manualTurnPower.value = localStorage.getItem("obrManualTurnPower") || "0.65";
      } catch {
        manualDrivePower.value = "0.65";
        manualTurnPower.value = "0.65";
      }
      updateManualPowerSettings();
    }

    function sendCurrentDrive() {
      if (manualEnabled) {
        send({ command: rawDiagnosticDrive ? "drive_raw" : "drive", left: requestedLeft, right: requestedRight });
      }
    }

    function updateDriveFromMixer() {
      const throttleValue = clamp(Number(throttle.value));
      const turnValue = clamp(Number(turn.value));
      const selectedPower = Math.abs(turnValue) > 0.0001
        ? clampManualPower(manualTurnPower.value)
        : clampManualPower(manualDrivePower.value);

      // O giro tem prioridade sobre frente e ré. Isso impede que combinações
      // como W+A zerem um lado e façam o robô curvar com metade da tração.
      const turning = Math.abs(turnValue) > 0.0001;
      const leftAxis = turning ? turnValue : throttleValue;
      const rightAxis = turning ? -turnValue : throttleValue;

      requestedLeft = scaleOperationalAxis(leftAxis, selectedPower);
      requestedRight = scaleOperationalAxis(rightAxis, selectedPower);
      rawDiagnosticDrive = false;
      element("throttleValue").textContent = throttleValue.toFixed(2);
      element("turnValue").textContent = turnValue.toFixed(2);
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
      throttle.value = "0";
      turn.value = "0";
      requestedLeft = 0;
      requestedRight = 0;
      rawDiagnosticDrive = false;
      element("throttleValue").textContent = "0.00";
      element("turnValue").textContent = "0.00";
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
      throttle.value = String(forward);
      turn.value = String(turnValue);
      updateDriveFromMixer();
    }

    function stopDriveOnFocusLoss() {
      resetKeyboardState();
      if (manualEnabled) resetDrive();
    }

    function cameraStreamUrl() { return `/camera-stream.mjpg?ts=${Date.now()}`; }
    function startCameraStream() { window.clearTimeout(cameraReconnectTimer); cameraImage.src = cameraStreamUrl(); }

    function clearCameraDiagnostics() {
      Object.values(cameraDiagnosticFields).forEach(field => { field.textContent = "—"; });
    }

    function formatCameraDiagnostic(value, decimals) {
      const number = Number(value);
      return Number.isFinite(number) ? number.toFixed(decimals) : "—";
    }

    function updateCameraDiagnostics(data) {
      const numericFields = [
        data.nearError, data.nearArea, data.nearHeightPx,
        data.controlError, data.correction, data.leftPreview, data.rightPreview,
        data.farError, data.farArea, data.farHeightPx, data.centerDeltaPx,
        data.lineTimestamp, data.lineSequence, data.timestamp
      ];
      const fieldsPresent = typeof data.nearValid === "boolean" &&
        typeof data.farValid === "boolean" &&
        typeof data.centerDeltaValid === "boolean" &&
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

      cameraDiagnosticFields.nearValid.textContent = data.nearValid ? "SIM" : "NÃO";
      cameraDiagnosticFields.nearError.textContent = formatCameraDiagnostic(data.nearError, 3);
      cameraDiagnosticFields.nearArea.textContent = formatCameraDiagnostic(data.nearArea, 1);
      cameraDiagnosticFields.nearHeight.textContent = `${Math.round(Number(data.nearHeightPx))} px`;
      cameraDiagnosticFields.farValid.textContent = data.farValid ? "SIM" : "NÃO";
      cameraDiagnosticFields.farError.textContent = formatCameraDiagnostic(data.farError, 3);
      cameraDiagnosticFields.farArea.textContent = formatCameraDiagnostic(data.farArea, 1);
      cameraDiagnosticFields.farHeight.textContent = `${Math.round(Number(data.farHeightPx))} px`;
      cameraDiagnosticFields.centerDelta.textContent = data.centerDeltaValid
        ? `${formatCameraDiagnostic(data.centerDeltaPx, 1)} px`
        : "INVÁLIDO";
      cameraDiagnosticFields.controlError.textContent = formatCameraDiagnostic(data.controlError, 3);
      cameraDiagnosticFields.correction.textContent = formatCameraDiagnostic(data.correction, 3);
      cameraDiagnosticFields.leftPreview.textContent = formatCameraDiagnostic(data.leftPreview, 3);
      cameraDiagnosticFields.rightPreview.textContent = formatCameraDiagnostic(data.rightPreview, 3);
      cameraDiagnosticFields.lineSequence.textContent = String(Math.trunc(Number(data.lineSequence)));
    }

    async function refreshCameraStatus() {
      try {
        const response = await fetch(`/camera-status.json?ts=${Date.now()}`, { cache: "no-store" });
        if (!response.ok) throw new Error("camera status unavailable");
        const data = await response.json();
        if (data.active !== true || Number(data.fps) <= 0) {
          cameraFps.textContent = "aguardando"; cameraRole.textContent = "--"; cameraResolution.textContent = "--"; cameraSensorMode.textContent = "--"; cameraScalerCrop.textContent = "--"; cameraFormat.textContent = "--"; clearCameraDiagnostics(); return;
        }
        const roleLabel = data.cameraRole === "down" ? "inferior" : data.cameraRole === "forward" ? "frontal" : "--";
        const width = Number(data.width);
        const height = Number(data.height);
        const sensorMode = data.sensorMode || {};
        const scalerCrop = data.scalerCrop || {};
        cameraTitle.textContent = roleLabel === "--" ? "Câmera" : `Câmera ${roleLabel}`;
        cameraImage.alt = roleLabel === "--" ? "Imagem direta da câmera selecionada" : `Imagem direta da câmera ${roleLabel}`;
        cameraFps.textContent = Number(data.fps).toFixed(1);
        cameraRole.textContent = roleLabel;
        cameraResolution.textContent = width > 0 && height > 0 ? `${width.toFixed(0)}×${height.toFixed(0)}` : "--";
        cameraSensorMode.textContent = Number(sensorMode.width) > 0 && Number(sensorMode.height) > 0 ? `${Number(sensorMode.width).toFixed(0)}×${Number(sensorMode.height).toFixed(0)} ${Number(sensorMode.bitDepth).toFixed(0)}-bit` : "--";
        cameraScalerCrop.textContent = Number.isFinite(Number(scalerCrop.x)) && Number.isFinite(Number(scalerCrop.y)) && Number(scalerCrop.width) > 0 && Number(scalerCrop.height) > 0 ? `${Number(scalerCrop.x).toFixed(0)},${Number(scalerCrop.y).toFixed(0)},${Number(scalerCrop.width).toFixed(0)},${Number(scalerCrop.height).toFixed(0)}` : "--";
        if (width > 0 && height > 0) cameraFrame.style.aspectRatio = `${width} / ${height}`;
        cameraFormat.textContent = data.cameraFormat || "--";
        updateCameraDiagnostics(data);
      } catch {
        cameraFps.textContent = "erro"; cameraRole.textContent = "--"; cameraResolution.textContent = "--"; cameraSensorMode.textContent = "--"; cameraScalerCrop.textContent = "--"; cameraFormat.textContent = "--"; clearCameraDiagnostics();
      }
    }

    cameraImage.addEventListener("load", () => cameraFrame.classList.remove("offline"));
    cameraImage.addEventListener("error", () => { cameraFrame.classList.add("offline"); cameraReconnectTimer = window.setTimeout(startCameraStream, 1000); });
    throttle.addEventListener("input", updateDriveFromMixer);
    turn.addEventListener("input", updateDriveFromMixer);
    manualDrivePower.addEventListener("input", updateManualPowerSettings);
    manualTurnPower.addEventListener("input", updateManualPowerSettings);
    leftValue.addEventListener("change", updateDriveFromExactInputs);
    rightValue.addEventListener("change", updateDriveFromExactInputs);
    document.querySelectorAll("[data-side][data-delta]").forEach(button => {
      button.addEventListener("click", () => adjustExactSide(button.dataset.side, Number(button.dataset.delta)));
    });
    autonomousMission.addEventListener("change", selectAutonomousMission);
    distanceTargetCm.addEventListener("change", selectAutonomousMission);
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
    document.addEventListener("visibilitychange", () => { if (document.hidden) stopDriveOnFocusLoss(); });
    window.setInterval(sendCurrentDrive, 100);
    window.setInterval(refreshCameraStatus, 500);
    startCameraStream();
    refreshCameraStatus();
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
             << "Connection: close\r\n\r\n"
             << content;

    std::string text = response.str();
    send(client, text.c_str(), static_cast<int>(text.size()), 0);
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

bool DashboardServer::sendCameraStreamHead(SocketHandle client)
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
    address.sin_port = htons(config::kCameraStreamPort);
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

bool DashboardServer::proxyCameraStream(SocketHandle client)
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
    address.sin_port = htons(config::kCameraStreamPort);
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

    std::ostringstream request;
    request << "GET " << config::kCameraStreamPath << " HTTP/1.1\r\n"
            << "Host: 127.0.0.1:" << config::kCameraStreamPort << "\r\n"
            << "Connection: close\r\n\r\n";

    const std::string requestText = request.str();
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

bool DashboardServer::sendCameraStatus(SocketHandle client)
{
    // O status da câmera é gerado pelo script Python em JSON simples.
    // Se ele não existir, a dashboard recebe um estado claro sem afetar o controle do robô.
    std::ifstream file(config::kCameraStatusPath, std::ios::binary);
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
