#include "obr/line_follower.h"

#include "obr/config.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

namespace
{
double clampOutput(double value)
{
    if (!std::isfinite(value))
    {
        return 0.0;
    }

    return std::clamp(value, config::kMinMotorOutput, config::kMaxMotorOutput);
}

double currentUnixSeconds()
{
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration<double>(now).count();
}
}

void LineFollower::update(RobotState& robotState, const Esp32TelemetrySnapshot& esp32Telemetry)
{
    const RobotSnapshot snapshot = robotState.snapshot();
    if (snapshot.mode != "autonomous")
    {
        resetMissionState();
        return;
    }

    if (snapshot.autonomousMission == AutonomousMission::TurnRight90)
    {
        updateTurnRight90(robotState, esp32Telemetry);
        return;
    }

    turn90Active_ = false;

    const auto now = std::chrono::steady_clock::now();
    if (phase_ != Phase::Following)
    {
        updateGreenManeuver(robotState, now);
        return;
    }

    const CameraStatus status = readCameraStatus();
    if (!status.valid || !status.active || !isFresh(status))
    {
        // Se a câmera falhar ou o JSON ficar antigo, a ação segura é parar.
        robotState.driveAutonomous(0.0, 0.0);
        return;
    }

    if (isGreenAction(status.greenAction) && now >= greenCooldownUntil_)
    {
        startGreenManeuver(status.greenAction, now);
        updateGreenManeuver(robotState, now);
        return;
    }

    followLine(robotState, status);
}

void LineFollower::updateTurnRight90(
    RobotState& robotState, const Esp32TelemetrySnapshot& esp32Telemetry)
{
    const auto now = std::chrono::steady_clock::now();
    const bool imuReady = esp32Telemetry.sensorFresh && esp32Telemetry.mpuOk &&
                          std::isfinite(esp32Telemetry.yawZDeg);

    if (!turn90Active_)
    {
        if (!imuReady)
        {
            // Sem uma referência angular válida, a missão não pode iniciar.
            robotState.driveAutonomous(0.0, 0.0);
            return;
        }

        turn90Active_ = true;
        turn90StartYawDegrees_ = esp32Telemetry.yawZDeg;
        turn90StartedAt_ = now;
        std::cout << "Turn-right-90 mission started at yaw="
                  << turn90StartYawDegrees_ << " deg\n";
    }

    const auto elapsed = now - turn90StartedAt_;
    if (elapsed > std::chrono::milliseconds(config::kTurn90TimeoutMs))
    {
        // O timeout evita manter os motores ativos se o ângulo parar de mudar.
        robotState.stop();
        turn90Active_ = false;
        std::cout << "Turn-right-90 mission stopped by timeout\n";
        return;
    }

    if (!imuReady)
    {
        // Uma leitura perdida durante o giro para os motores imediatamente.
        robotState.driveAutonomous(0.0, 0.0);
        return;
    }

    const double turnedDegrees = angularDistanceDegrees(
        turn90StartYawDegrees_, esp32Telemetry.yawZDeg);
    const double remainingDegrees = config::kTurn90TargetDegrees - turnedDegrees;
    if (remainingDegrees <= config::kTurn90StopToleranceDegrees)
    {
        robotState.stop();
        turn90Active_ = false;
        std::cout << "Turn-right-90 mission completed at "
                  << turnedDegrees << " deg\n";
        return;
    }

    const double turnPower = remainingDegrees <= config::kTurn90SlowdownDegrees
                                 ? config::kTurn90FinePower
                                 : config::kTurn90Power;
    // Potência positiva à esquerda e negativa à direita gira no mesmo sentido
    // usado pelas manobras de curva à direita já existentes.
    robotState.driveAutonomous(turnPower, -turnPower);
}

void LineFollower::resetMissionState()
{
    phase_ = Phase::Following;
    activeGreenAction_ = "NENHUM";
    lastLineError_ = 0.0;
    turn90Active_ = false;
}

bool LineFollower::cameraReady() const
{
    const CameraStatus status = readCameraStatus();
    return status.valid && status.active && status.fps > 0.0 && isFresh(status);
}

LineFollower::CameraStatus LineFollower::readCameraStatus() const
{
    std::ifstream file(config::kCameraStatusPath);
    CameraStatus status;
    if (!file)
    {
        return status;
    }

    std::ostringstream content;
    content << file.rdbuf();
    const std::string json = content.str();
    if (json.empty())
    {
        return status;
    }

    status.valid = true;
    status.active = getJsonBool(json, "active", true);
    status.lineDetected = getJsonBool(json, "lineDetected", false);
    status.fps = getJsonNumber(json, "fps", 0.0);
    status.lineError = getJsonNumber(json, "lineError", 0.0);
    status.timestampSeconds = getJsonNumber(json, "timestamp", 0.0);
    status.greenAction = getJsonString(json, "greenAction", "NENHUM");
    return status;
}

void LineFollower::followLine(RobotState& robotState, const CameraStatus& status)
{
    if (!status.lineDetected)
    {
        // Em curvas de 90 graus, a linha pode sair da imagem por alguns ciclos.
        // O robô gira para o último lado conhecido, mas ainda para se o JSON da
        // câmera ficar antigo, porque dados antigos não são seguros.
        if (std::abs(lastLineError_) > config::kLineFollowerLostLineDeadbandPixels)
        {
            const double turnPower = config::kLineFollowerLostLineTurnPower;
            if (lastLineError_ > 0.0)
            {
                robotState.driveAutonomous(turnPower, -turnPower);
            }
            else
            {
                robotState.driveAutonomous(-turnPower, turnPower);
            }
            return;
        }

        robotState.driveAutonomous(config::kLineFollowerBasePower * 0.5, config::kLineFollowerBasePower * 0.5);
        return;
    }

    lastLineError_ = status.lineError;

    const double correction = std::clamp(
        status.lineError * config::kLineFollowerTurnGain,
        -config::kLineFollowerMaxTurnCorrection,
        config::kLineFollowerMaxTurnCorrection);

    const double left = clampOutput(config::kLineFollowerBasePower + correction);
    const double right = clampOutput(config::kLineFollowerBasePower - correction);
    robotState.driveAutonomous(left, right);
}

void LineFollower::startGreenManeuver(const std::string& action, std::chrono::steady_clock::time_point now)
{
    activeGreenAction_ = action;
    phase_ = Phase::ApproachingGreen;
    phaseUntil_ = now + std::chrono::milliseconds(config::kGreenApproachMs);
    std::cout << "Green action detected: " << action << "\n";
}

void LineFollower::updateGreenManeuver(RobotState& robotState, std::chrono::steady_clock::time_point now)
{
    if (phase_ == Phase::ApproachingGreen)
    {
        if (now < phaseUntil_)
        {
            // Avança devagar para alinhar o centro do robô com a interseção.
            robotState.driveAutonomous(config::kLineFollowerBasePower, config::kLineFollowerBasePower);
            return;
        }

        phase_ = Phase::TurningGreen;
        const int turnMs = activeGreenAction_ == "MEIA VOLTA" ? config::kGreenUTurnMs : config::kGreenTurnMs;
        phaseUntil_ = now + std::chrono::milliseconds(turnMs);
    }

    if (phase_ == Phase::TurningGreen)
    {
        if (now < phaseUntil_)
        {
            if (activeGreenAction_ == "ESQUERDA")
            {
                robotState.driveAutonomous(-config::kGreenTurnPower, config::kGreenTurnPower);
            }
            else
            {
                // Direita e meia-volta usam o mesmo sentido inicial de giro.
                robotState.driveAutonomous(config::kGreenTurnPower, -config::kGreenTurnPower);
            }
            return;
        }

        phase_ = Phase::Following;
        activeGreenAction_ = "NENHUM";
        greenCooldownUntil_ = now + std::chrono::milliseconds(config::kGreenCooldownMs);
        robotState.driveAutonomous(0.0, 0.0);
    }
}

bool LineFollower::isFresh(const CameraStatus& status)
{
    if (status.timestampSeconds <= 0.0)
    {
        return false;
    }

    const double ageMs = (currentUnixSeconds() - status.timestampSeconds) * 1000.0;
    return ageMs >= 0.0 && ageMs <= config::kCameraStatusTimeoutMs;
}

bool LineFollower::isGreenAction(const std::string& action)
{
    return action == "ESQUERDA" || action == "DIREITA" || action == "MEIA VOLTA";
}

double LineFollower::angularDistanceDegrees(double first, double second)
{
    // Normaliza a diferença para [-180, 180]. Isso mantém a medição correta
    // quando o yaw atravessa a transição entre +180 e -180 graus.
    double difference = std::fmod(second - first + 540.0, 360.0) - 180.0;
    return std::abs(difference);
}

double LineFollower::getJsonNumber(const std::string& json, const std::string& key, double fallback)
{
    const std::string pattern = "\"" + key + "\":";
    size_t start = json.find(pattern);
    if (start == std::string::npos)
    {
        return fallback;
    }

    start += pattern.size();
    const size_t end = json.find_first_of(",}", start);
    if (end == std::string::npos)
    {
        return fallback;
    }

    try
    {
        return std::stod(json.substr(start, end - start));
    }
    catch (const std::exception&)
    {
        return fallback;
    }
}

bool LineFollower::getJsonBool(const std::string& json, const std::string& key, bool fallback)
{
    const std::string pattern = "\"" + key + "\":";
    size_t start = json.find(pattern);
    if (start == std::string::npos)
    {
        return fallback;
    }

    start += pattern.size();
    while (start < json.size() && json[start] == ' ')
    {
        ++start;
    }

    if (json.compare(start, 4, "true") == 0)
    {
        return true;
    }
    if (json.compare(start, 5, "false") == 0)
    {
        return false;
    }
    return fallback;
}

std::string LineFollower::getJsonString(const std::string& json, const std::string& key, const std::string& fallback)
{
    const std::string pattern = "\"" + key + "\":";
    size_t start = json.find(pattern);
    if (start == std::string::npos)
    {
        return fallback;
    }

    start = json.find('"', start + pattern.size());
    if (start == std::string::npos)
    {
        return fallback;
    }

    const size_t end = json.find('"', start + 1);
    if (end == std::string::npos)
    {
        return fallback;
    }

    return json.substr(start + 1, end - start - 1);
}
