#pragma once

#include "obr/robot_state.h"
#include "obr/telemetry.h"

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#if defined(_WIN32) && defined(__INTELLISENSE__)
using SocketHandle = unsigned long long;
using SocketResult = int;
using SocketLength = int;

struct sockaddr
{
    unsigned short sa_family;
    char sa_data[14];
};

struct in_addr
{
    unsigned long s_addr;
};

struct sockaddr_in
{
    short sin_family;
    unsigned short sin_port;
    in_addr sin_addr;
    char sin_zero[8];
};

constexpr int AF_INET = 2;
constexpr int SOCK_STREAM = 1;
constexpr int SOL_SOCKET = 0xffff;
constexpr int SO_REUSEADDR = 0x0004;
constexpr int MSG_WAITALL = 0x8;
constexpr unsigned long INADDR_ANY = 0;
constexpr SocketHandle INVALID_SOCKET = static_cast<SocketHandle>(~0ULL);

inline unsigned short htons(unsigned short value)
{
    return value;
}

inline SocketHandle socket(int, int, int)
{
    return 0;
}

inline int setsockopt(SocketHandle, int, int, const char*, int)
{
    return 0;
}

inline int bind(SocketHandle, const sockaddr*, int)
{
    return 0;
}

inline int listen(SocketHandle, int)
{
    return 0;
}

inline SocketHandle accept(SocketHandle, sockaddr*, SocketLength*)
{
    return 0;
}

inline int send(SocketHandle, const char*, int, int)
{
    return 0;
}

inline int recv(SocketHandle, char*, int, int)
{
    return 0;
}
#elif defined(_WIN32)
#include <BaseTsd.h>
#include <winsock2.h>
#include <ws2tcpip.h>
using SocketHandle = SOCKET;
using SocketResult = int;
using SocketLength = int;
#else
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
using SocketHandle = int;
using SocketResult = ssize_t;
using SocketLength = socklen_t;
#endif

// Servidor HTTP/WebSocket usado pelo dashboard de teste e telemetria.
// Ele recebe comandos, mas apenas atualiza o RobotState; GPIO e motores ficam
// sob responsabilidade do controle do robô.
class DashboardServer
{
public:
    DashboardServer(RobotState& robotState, Telemetry& telemetry);
    ~DashboardServer();

    bool start();
    void stop();

private:
    RobotState& robotState_;
    Telemetry& telemetry_;
    std::atomic<bool> running_{false};
    SocketHandle server_{};
    std::thread acceptThread_;
    std::thread telemetryThread_;
    std::mutex clientsMutex_;
    std::vector<SocketHandle> clients_;

    bool initializeSockets();
    void cleanupSockets();
    bool isInvalidSocket(SocketHandle socketHandle) const;
    void closeSocket(SocketHandle socketHandle);
    void shutdownSocket(SocketHandle socketHandle);

    void acceptLoop();
    void telemetryLoop();
    void handleClient(SocketHandle client);
    void handleWebSocket(SocketHandle client, const std::string& request);
    void addWebSocketClient(SocketHandle client);
    void removeWebSocketClient(SocketHandle client);
    void broadcast(const std::string& message);
    void handleCommand(const std::string& message);

    std::string buildTelemetryJson(const TelemetrySample& sample) const;

    static std::string dashboardHtml();
    static void sendHttpResponse(SocketHandle client, const std::string& content, const std::string& contentType);
    static std::string getHeaderValue(const std::string& request, const std::string& header);
    static bool sendWebSocketText(SocketHandle client, const std::string& message);
    static bool readWebSocketFrame(SocketHandle client, std::string& payload);
    static double getJsonNumber(const std::string& json, const std::string& key, double fallback);
    static std::string sha1(const std::string& input);
    static std::string base64Encode(const std::string& input);
    static unsigned int leftRotate(unsigned int value, int bits);
};
