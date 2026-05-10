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

DashboardServer::DashboardServer(RobotState& robotState, Telemetry& telemetry, Esp32Bridge& esp32)
    : robotState_(robotState), telemetry_(telemetry), esp32_(esp32)
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
        robotState_.enforceCommandTimeout(std::chrono::milliseconds(config::kCommandTimeoutMs));

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
        robotState_.start();
        std::cout << "Start received\n";
    }
    else if (message.find("\"command\":\"stop\"") != std::string::npos)
    {
        robotState_.stop();
        std::cout << "Stop received\n";
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

    std::ostringstream json;
    json << std::fixed << std::setprecision(2)
         << "{\"cpu\":" << sample.cpuUsage
         << ",\"temperature\":" << sample.temperature
         << ",\"ram\":" << sample.ramUsage
         << ",\"mode\":\"" << state.mode << "\""
         << ",\"left\":" << state.left
         << ",\"right\":" << state.right
         << ",\"emergency\":" << (state.emergencyStop ? "true" : "false")
         << ",\"esp32SerialOpen\":" << (esp32.serialOpen ? "true" : "false")
         << ",\"esp32SensorFresh\":" << (esp32.sensorFresh ? "true" : "false")
         << ",\"esp32LastSensorAgeMs\":" << esp32.lastSensorAgeMs
         << ",\"mpuOk\":" << (esp32.mpuOk ? "true" : "false")
         << ",\"ultrasonicDistanceCm\":" << esp32.ultrasonicDistanceCm
         << ",\"gyroZDegPerSec\":" << esp32.gyroZDegPerSec
         << ",\"yawZDeg\":" << esp32.yawZDeg
         << ",\"accelX\":" << esp32.accelX
         << ",\"accelY\":" << esp32.accelY
         << ",\"accelZ\":" << esp32.accelZ
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
  <title>OBR Robot Dashboard</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: Inter, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #101418;
      color: #edf2f7;
    }
    * { box-sizing: border-box; }
    body { margin: 0; min-height: 100vh; background: #101418; }
    main { width: min(1100px, calc(100% - 32px)); margin: 0 auto; padding: 28px 0; }
    header { display: flex; align-items: center; justify-content: space-between; gap: 16px; margin-bottom: 22px; }
    h1 { margin: 0; font-size: clamp(1.7rem, 5vw, 3.2rem); line-height: 1; }
    .status { min-width: 132px; padding: 10px 12px; border: 1px solid #34404d; border-radius: 8px; text-align: center; font-weight: 700; background: #17202a; }
    .grid { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 12px; }
    .panel { border: 1px solid #2d3742; border-radius: 8px; background: #151b22; padding: 16px; }
    .metric { min-height: 112px; }
    .label { margin: 0 0 10px; color: #a8b3c1; font-size: 0.9rem; }
    .value { margin: 0; font-size: 2.2rem; font-weight: 800; }
    .value.small { font-size: 1.35rem; line-height: 1.25; }
    .controls { grid-column: span 3; display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 10px; }
    button { min-height: 52px; border: 0; border-radius: 8px; color: #071016; background: #4ade80; font-size: 1rem; font-weight: 800; cursor: pointer; }
    button.secondary { background: #60a5fa; }
    button.danger { background: #fb7185; }
    .camera { grid-column: span 3; overflow: hidden; }
    .camera-header { display: flex; align-items: center; justify-content: space-between; gap: 12px; margin-bottom: 10px; }
    .camera-header .label { margin: 0; }
    .camera-info { display: flex; flex-wrap: wrap; justify-content: flex-end; gap: 10px; }
    .camera-fps { color: #edf2f7; font-size: 0.95rem; font-weight: 800; white-space: nowrap; }
    .camera-frame { position: relative; aspect-ratio: 16 / 9; max-width: 960px; margin: 0 auto; background: #0b1117; border-radius: 6px; overflow: hidden; }
    .camera-frame img { display: block; width: 100%; height: 100%; object-fit: contain; }
    .camera-frame.offline img { opacity: 0; }
    .camera-message { position: absolute; inset: 0; display: grid; place-items: center; color: #a8b3c1; text-align: center; padding: 18px; }
    .camera-frame:not(.offline) .camera-message { display: none; }
    .drive { grid-column: span 3; display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 16px; }
    input[type="range"] { width: 100%; accent-color: #facc15; }
    @media (max-width: 760px) {
      header, .grid, .controls, .drive { grid-template-columns: 1fr; }
      header { align-items: stretch; }
      .controls, .camera, .drive { grid-column: span 1; }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <h1>OBR Robot</h1>
      <div id="connection" class="status">offline</div>
    </header>

    <section class="grid">
      <article class="panel metric">
        <p class="label">Modo</p>
        <p id="mode" class="value">-</p>
      </article>
      <article class="panel metric">
        <p class="label">CPU</p>
        <p id="cpu" class="value">-</p>
      </article>
      <article class="panel metric">
        <p class="label">Temperatura</p>
        <p id="temp" class="value">-</p>
      </article>
      <article class="panel metric">
        <p class="label">RAM</p>
        <p id="ram" class="value">-</p>
      </article>
      <article class="panel metric">
        <p class="label">ESP32</p>
        <p id="esp32Status" class="value small">-</p>
      </article>
      <article class="panel metric">
        <p class="label">Ultrassônico</p>
        <p id="ultrasonic" class="value small">-</p>
      </article>
      <article class="panel metric">
        <p class="label">Gyro Z</p>
        <p id="gyroZ" class="value small">-</p>
      </article>
      <article class="panel metric">
        <p class="label">Yaw Z</p>
        <p id="yawZ" class="value small">-</p>
      </article>
      <article class="panel metric">
        <p class="label">Aceleração</p>
        <p id="accel" class="value small">-</p>
      </article>

      <section class="controls">
        <button onclick="sendCommand('start')">Start</button>
        <button class="secondary" onclick="sendCommand('stop')">Stop</button>
        <button class="danger" onclick="sendCommand('estop')">E-Stop</button>
      </section>

      <section class="panel camera">
        <div class="camera-header">
          <p class="label">Câmera</p>
          <div class="camera-info">
            <div class="camera-fps">FPS: <span id="cameraFps">-</span></div>
            <div class="camera-fps">Erro: <span id="cameraLineError">-</span></div>
            <div class="camera-fps">Res: <span id="cameraResolution">-</span></div>
            <div class="camera-fps">Fmt: <span id="cameraFormat">-</span></div>
          </div>
        </div>
        <div id="cameraFrame" class="camera-frame offline">
          <img id="cameraImage" alt="Imagem processada da câmera">
          <div class="camera-message">Aguardando imagem da câmera</div>
        </div>
      </section>

      <section class="panel drive">
        <div>
          <p class="label">Frente/ré: <strong id="throttleValue">0.00</strong></p>
          <input id="throttle" type="range" min="-1" max="1" step="0.05" value="0">
        </div>
        <div>
          <p class="label">Curva: <strong id="turnValue">0.00</strong></p>
          <input id="turn" type="range" min="-1" max="1" step="0.05" value="0">
        </div>
        <div>
          <p class="label">Motor esquerdo: <strong id="leftValue">0.00</strong></p>
        </div>
        <div>
          <p class="label">Motor direito: <strong id="rightValue">0.00</strong></p>
        </div>
      </section>
    </section>
  </main>

  <script>
    let ws;
    let manualEnabled = false;
    const connection = document.getElementById("connection");
    const throttle = document.getElementById("throttle");
    const turn = document.getElementById("turn");
    const cameraFrame = document.getElementById("cameraFrame");
    const cameraImage = document.getElementById("cameraImage");
    const cameraFps = document.getElementById("cameraFps");
    const cameraLineError = document.getElementById("cameraLineError");
    const cameraResolution = document.getElementById("cameraResolution");
    const cameraFormat = document.getElementById("cameraFormat");
    let cameraReconnectTimer;

    function connect() {
      ws = new WebSocket(`ws://${location.host}/ws`);
      ws.onopen = () => { connection.textContent = "online"; };
      ws.onclose = () => { connection.textContent = "offline"; setTimeout(connect, 1000); };
      ws.onmessage = (event) => {
        const data = JSON.parse(event.data);
        document.getElementById("mode").textContent = data.mode;
        document.getElementById("cpu").textContent = `${data.cpu.toFixed(1)}%`;
        document.getElementById("temp").textContent = `${data.temperature.toFixed(1)} C`;
        document.getElementById("ram").textContent = `${data.ram.toFixed(1)}%`;
        updateEsp32Telemetry(data);
      };
    }

    function formatNumber(value, digits) {
      const number = Number(value);
      if (!Number.isFinite(number)) {
        return "-";
      }
      return number.toFixed(digits);
    }

    function updateEsp32Telemetry(data) {
      const sensorFresh = data.esp32SensorFresh === true;
      const mpuOk = sensorFresh && data.mpuOk === true;
      const distanceCm = Number(data.ultrasonicDistanceCm);

      document.getElementById("esp32Status").textContent = data.esp32SerialOpen
        ? (sensorFresh ? "online" : "sem dados")
        : "offline";

      document.getElementById("ultrasonic").textContent = sensorFresh && distanceCm >= 0
        ? `${formatNumber(distanceCm, 1)} cm`
        : "falha";

      document.getElementById("gyroZ").textContent = mpuOk
        ? `${formatNumber(data.gyroZDegPerSec, 1)} deg/s`
        : "sem MPU";

      document.getElementById("yawZ").textContent = mpuOk
        ? `${formatNumber(data.yawZDeg, 1)} deg`
        : "sem MPU";

      document.getElementById("accel").textContent = mpuOk
        ? `x ${formatNumber(data.accelX, 1)}  y ${formatNumber(data.accelY, 1)}  z ${formatNumber(data.accelZ, 1)}`
        : "sem MPU";
    }

    function send(payload) {
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(payload));
      }
    }

    function sendCommand(command) {
      if (command === "start") {
        manualEnabled = true;
      }
      if (command === "stop" || command === "estop") {
        manualEnabled = false;
        resetDrive();
      }
      send({ command });
    }

    function clamp(value) {
      return Math.max(-1, Math.min(1, value));
    }

    function sendDrive() {
      const throttleValue = Number(throttle.value);
      const turnValue = Number(turn.value);
      const leftValue = clamp(throttleValue + turnValue);
      const rightValue = clamp(throttleValue - turnValue);

      document.getElementById("throttleValue").textContent = throttleValue.toFixed(2);
      document.getElementById("turnValue").textContent = turnValue.toFixed(2);
      document.getElementById("leftValue").textContent = leftValue.toFixed(2);
      document.getElementById("rightValue").textContent = rightValue.toFixed(2);

      if (manualEnabled) {
        send({ command: "drive", left: leftValue, right: rightValue });
      }
    }

    function resetDrive() {
      throttle.value = "0";
      turn.value = "0";
      sendDrive();
    }

    function cameraStreamUrl() {
      return `/camera-stream.mjpg?ts=${Date.now()}`;
    }

    function startCameraStream() {
      window.clearTimeout(cameraReconnectTimer);
      cameraImage.src = cameraStreamUrl();
    }

    async function refreshCameraStatus() {
      try {
        const response = await fetch(`/camera-status.json?ts=${Date.now()}`, { cache: "no-store" });
        if (!response.ok) {
          throw new Error("camera status unavailable");
        }
        const data = await response.json();
        if (data.active !== true || Number(data.fps) <= 0) {
          cameraFps.textContent = "aguardando";
          cameraLineError.textContent = "-";
          cameraResolution.textContent = "-";
          cameraFormat.textContent = "-";
          return;
        }
        cameraFps.textContent = Number(data.fps).toFixed(1);
        cameraLineError.textContent = data.lineDetected === true
          ? Number(data.lineError).toFixed(0)
          : "perdida";
        cameraResolution.textContent = Number(data.width) > 0 && Number(data.height) > 0
          ? `${Number(data.width).toFixed(0)}x${Number(data.height).toFixed(0)}`
          : "-";
        cameraFormat.textContent = data.cameraFormat || "-";
      } catch {
        cameraFps.textContent = "erro";
        cameraLineError.textContent = "-";
        cameraResolution.textContent = "-";
        cameraFormat.textContent = "-";
      }
    }

    cameraImage.addEventListener("load", () => {
      cameraFrame.classList.remove("offline");
    });

    cameraImage.addEventListener("error", () => {
      cameraFrame.classList.add("offline");
      cameraReconnectTimer = window.setTimeout(startCameraStream, 1000);
    });

    throttle.addEventListener("input", sendDrive);
    turn.addEventListener("input", sendDrive);
    setInterval(sendDrive, 100);
    setInterval(refreshCameraStatus, 500);
    startCameraStream();
    refreshCameraStatus();
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
