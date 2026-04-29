#include <arpa/inet.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace
{
constexpr int kPort = 8080;

std::atomic<bool> running(true);
std::mutex stateMutex;
std::mutex clientsMutex;
std::vector<int> websocketClients;

struct CpuSample
{
    unsigned long long idle;
    unsigned long long total;
};

struct RobotState
{
    std::string mode = "stopped";
    bool emergencyStop = false;
    double left = 0.0;
    double right = 0.0;
    std::chrono::steady_clock::time_point lastCommand = std::chrono::steady_clock::now();
};

RobotState robotState;

void handleSignal(int)
{
    running = false;
}

uint32_t leftRotate(uint32_t value, int bits)
{
    return (value << bits) | (value >> (32 - bits));
}

std::string sha1(const std::string& input)
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

std::string base64Encode(const std::string& input)
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

CpuSample readCpuSample()
{
    std::ifstream file("/proc/stat");

    std::string cpu;
    unsigned long long user = 0;
    unsigned long long nice = 0;
    unsigned long long system = 0;
    unsigned long long idle = 0;
    unsigned long long iowait = 0;
    unsigned long long irq = 0;
    unsigned long long softirq = 0;
    unsigned long long steal = 0;

    file >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;

    unsigned long long idleAll = idle + iowait;
    unsigned long long total = user + nice + system + idle + iowait + irq + softirq + steal;

    return {idleAll, total};
}

double calculateCpuUsage(const CpuSample& previous, const CpuSample& current)
{
    unsigned long long idleDelta = current.idle - previous.idle;
    unsigned long long totalDelta = current.total - previous.total;

    if (totalDelta == 0)
    {
        return 0.0;
    }

    return 100.0 * (1.0 - static_cast<double>(idleDelta) / totalDelta);
}

double readCpuTemperature()
{
    std::ifstream file("/sys/class/thermal/thermal_zone0/temp");
    int tempMilliCelsius = 0;
    file >> tempMilliCelsius;
    return tempMilliCelsius / 1000.0;
}

std::string dashboardHtml()
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

    body {
      margin: 0;
      min-height: 100vh;
      background: #101418;
    }

    main {
      width: min(1100px, calc(100% - 32px));
      margin: 0 auto;
      padding: 28px 0;
    }

    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      margin-bottom: 22px;
    }

    h1 {
      margin: 0;
      font-size: clamp(1.7rem, 5vw, 3.2rem);
      line-height: 1;
    }

    .status {
      min-width: 132px;
      padding: 10px 12px;
      border: 1px solid #34404d;
      border-radius: 8px;
      text-align: center;
      font-weight: 700;
      background: #17202a;
    }

    .grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 12px;
    }

    .panel {
      border: 1px solid #2d3742;
      border-radius: 8px;
      background: #151b22;
      padding: 16px;
    }

    .metric {
      min-height: 112px;
    }

    .label {
      margin: 0 0 10px;
      color: #a8b3c1;
      font-size: 0.9rem;
    }

    .value {
      margin: 0;
      font-size: 2.2rem;
      font-weight: 800;
    }

    .controls {
      grid-column: span 2;
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 10px;
    }

    button {
      min-height: 52px;
      border: 0;
      border-radius: 8px;
      color: #071016;
      background: #4ade80;
      font-size: 1rem;
      font-weight: 800;
      cursor: pointer;
    }

    button.secondary { background: #60a5fa; }
    button.danger { background: #fb7185; }

    .drive {
      grid-column: span 3;
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 16px;
    }

    input[type="range"] {
      width: 100%;
      accent-color: #facc15;
    }

    @media (max-width: 760px) {
      header, .grid, .controls, .drive {
        grid-template-columns: 1fr;
      }

      header {
        align-items: stretch;
      }

      .controls, .drive {
        grid-column: span 1;
      }
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

      <section class="controls">
        <button onclick="sendCommand('start')">Start</button>
        <button class="secondary" onclick="sendCommand('stop')">Stop</button>
        <button class="danger" onclick="sendCommand('estop')">Emergency Stop</button>
      </section>

      <section class="panel drive">
        <div>
          <p class="label">Motor esquerdo: <strong id="leftValue">0.00</strong></p>
          <input id="left" type="range" min="-1" max="1" step="0.05" value="0">
        </div>
        <div>
          <p class="label">Motor direito: <strong id="rightValue">0.00</strong></p>
          <input id="right" type="range" min="-1" max="1" step="0.05" value="0">
        </div>
      </section>
    </section>
  </main>

  <script>
    let ws;
    const connection = document.getElementById("connection");
    const left = document.getElementById("left");
    const right = document.getElementById("right");

    function connect() {
      ws = new WebSocket(`ws://${location.host}/ws`);

      ws.onopen = () => {
        connection.textContent = "online";
      };

      ws.onclose = () => {
        connection.textContent = "offline";
        setTimeout(connect, 1000);
      };

      ws.onmessage = (event) => {
        const data = JSON.parse(event.data);
        document.getElementById("mode").textContent = data.mode;
        document.getElementById("cpu").textContent = `${data.cpu.toFixed(1)}%`;
        document.getElementById("temp").textContent = `${data.temperature.toFixed(1)} C`;
      };
    }

    function send(payload) {
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(payload));
      }
    }

    function sendCommand(command) {
      send({ command });
    }

    function sendDrive() {
      document.getElementById("leftValue").textContent = Number(left.value).toFixed(2);
      document.getElementById("rightValue").textContent = Number(right.value).toFixed(2);
      send({ command: "drive", left: Number(left.value), right: Number(right.value) });
    }

    left.addEventListener("input", sendDrive);
    right.addEventListener("input", sendDrive);
    connect();
  </script>
</body>
</html>)HTML";
}

void sendHttpResponse(int client, const std::string& content, const std::string& contentType)
{
    std::ostringstream response;
    response << "HTTP/1.1 200 OK\r\n"
             << "Content-Type: " << contentType << "\r\n"
             << "Content-Length: " << content.size() << "\r\n"
             << "Connection: close\r\n\r\n"
             << content;

    std::string text = response.str();
    send(client, text.c_str(), text.size(), 0);
}

std::string getHeaderValue(const std::string& request, const std::string& header)
{
    size_t start = request.find(header);
    if (start == std::string::npos)
    {
        return "";
    }

    start += header.size();
    size_t end = request.find("\r\n", start);
    return request.substr(start, end - start);
}

bool sendWebSocketText(int client, const std::string& message)
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
    return send(client, frame.data(), frame.size(), 0) == static_cast<ssize_t>(frame.size());
}

bool readWebSocketFrame(int client, std::string& payload)
{
    uint8_t header[2] = {};
    ssize_t readBytes = recv(client, header, 2, MSG_WAITALL);
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
        if (recv(client, extended, 2, MSG_WAITALL) <= 0)
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
    if (masked && recv(client, mask, 4, MSG_WAITALL) <= 0)
    {
        return false;
    }

    std::vector<uint8_t> data(length);
    if (length > 0 && recv(client, data.data(), length, MSG_WAITALL) <= 0)
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

double getJsonNumber(const std::string& json, const std::string& key, double fallback)
{
    std::string marker = "\"" + key + "\":";
    size_t start = json.find(marker);
    if (start == std::string::npos)
    {
        return fallback;
    }

    start += marker.size();
    size_t end = json.find_first_of(",}", start);
    return std::stod(json.substr(start, end - start));
}

void handleCommand(const std::string& message)
{
    std::lock_guard<std::mutex> lock(stateMutex);
    robotState.lastCommand = std::chrono::steady_clock::now();

    if (message.find("\"command\":\"estop\"") != std::string::npos)
    {
        robotState.mode = "emergency";
        robotState.emergencyStop = true;
        robotState.left = 0.0;
        robotState.right = 0.0;
        std::cout << "Emergency stop received\n";
    }
    else if (message.find("\"command\":\"start\"") != std::string::npos)
    {
        robotState.emergencyStop = false;
        robotState.mode = "manual";
        std::cout << "Start received\n";
    }
    else if (message.find("\"command\":\"stop\"") != std::string::npos)
    {
        robotState.mode = "stopped";
        robotState.left = 0.0;
        robotState.right = 0.0;
        std::cout << "Stop received\n";
    }
    else if (message.find("\"command\":\"drive\"") != std::string::npos && !robotState.emergencyStop)
    {
        robotState.mode = "manual";
        robotState.left = getJsonNumber(message, "left", robotState.left);
        robotState.right = getJsonNumber(message, "right", robotState.right);
        std::cout << "Drive left=" << robotState.left << " right=" << robotState.right << "\n";
    }
}

void addWebSocketClient(int client)
{
    std::lock_guard<std::mutex> lock(clientsMutex);
    websocketClients.push_back(client);
}

void removeWebSocketClient(int client)
{
    std::lock_guard<std::mutex> lock(clientsMutex);
    websocketClients.erase(
        std::remove(websocketClients.begin(), websocketClients.end(), client),
        websocketClients.end());
}

void handleWebSocket(int client, const std::string& request)
{
    std::string key = getHeaderValue(request, "Sec-WebSocket-Key: ");
    std::string accept = base64Encode(sha1(key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"));

    std::ostringstream response;
    response << "HTTP/1.1 101 Switching Protocols\r\n"
             << "Upgrade: websocket\r\n"
             << "Connection: Upgrade\r\n"
             << "Sec-WebSocket-Accept: " << accept << "\r\n\r\n";

    std::string text = response.str();
    send(client, text.c_str(), text.size(), 0);
    addWebSocketClient(client);

    std::string payload;
    while (running && readWebSocketFrame(client, payload))
    {
        handleCommand(payload);
    }

    removeWebSocketClient(client);
    close(client);
}

std::string buildTelemetryJson(double cpuUsage, double temperature)
{
    RobotState state;
    {
        std::lock_guard<std::mutex> lock(stateMutex);
        state = robotState;
    }

    std::ostringstream json;
    json << std::fixed << std::setprecision(2)
         << "{\"cpu\":" << cpuUsage
         << ",\"temperature\":" << temperature
         << ",\"mode\":\"" << state.mode << "\""
         << ",\"left\":" << state.left
         << ",\"right\":" << state.right
         << ",\"emergency\":" << (state.emergencyStop ? "true" : "false")
         << "}";

    return json.str();
}

void telemetryLoop()
{
    CpuSample previous = readCpuSample();

    while (running)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        CpuSample current = readCpuSample();
        double cpuUsage = calculateCpuUsage(previous, current);
        double temperature = readCpuTemperature();
        previous = current;

        {
            std::lock_guard<std::mutex> lock(stateMutex);
            auto age = std::chrono::steady_clock::now() - robotState.lastCommand;
            if (robotState.mode == "manual" && age > std::chrono::seconds(2))
            {
                robotState.left = 0.0;
                robotState.right = 0.0;
            }
        }

        std::string json = buildTelemetryJson(cpuUsage, temperature);
        std::lock_guard<std::mutex> lock(clientsMutex);

        for (auto it = websocketClients.begin(); it != websocketClients.end();)
        {
            if (sendWebSocketText(*it, json))
            {
                ++it;
            }
            else
            {
                close(*it);
                it = websocketClients.erase(it);
            }
        }

        std::cout << "Telemetry " << json << "\n";
    }
}

void handleClient(int client)
{
    char buffer[4096] = {};
    ssize_t bytesRead = recv(client, buffer, sizeof(buffer) - 1, 0);
    if (bytesRead <= 0)
    {
        close(client);
        return;
    }

    std::string request(buffer, bytesRead);
    if (request.find("Upgrade: websocket") != std::string::npos)
    {
        handleWebSocket(client, request);
        return;
    }

    sendHttpResponse(client, dashboardHtml(), "text/html; charset=utf-8");
    close(client);
}
}

int main()
{
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    int server = socket(AF_INET, SOCK_STREAM, 0);
    if (server < 0)
    {
        std::cerr << "Failed to create socket\n";
        return 1;
    }

    int opt = 1;
    setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in address = {};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(kPort);

    if (bind(server, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
    {
        std::cerr << "Failed to bind port " << kPort << "\n";
        close(server);
        return 1;
    }

    if (listen(server, 8) < 0)
    {
        std::cerr << "Failed to listen on port " << kPort << "\n";
        close(server);
        return 1;
    }

    std::thread telemetry(telemetryLoop);
    telemetry.detach();

    std::cout << "OBR robot dashboard running\n";
    std::cout << "Open http://raspberrypi:" << kPort << " in a browser\n";

    while (running)
    {
        sockaddr_in clientAddress = {};
        socklen_t clientLength = sizeof(clientAddress);
        int client = accept(server, reinterpret_cast<sockaddr*>(&clientAddress), &clientLength);

        if (client >= 0)
        {
            std::thread(handleClient, client).detach();
        }
    }

    close(server);
    return 0;
}
