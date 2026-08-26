#pragma once

#include <cstdint>
#include <limits>
#include <string>

enum class GreenInterpretation
{
    None,
    FalseMarker,
    Ambiguous,
    Left,
    Right,
    TurnAround180
};

// Transporta o baseline do controle inferior para diagnóstico assíncrono.
// Nenhum destes campos participa das decisões ou dos comandos de motor.
struct CameraCurveDiagnostics
{
    double nearPosition = std::numeric_limits<double>::quiet_NaN();
    double mediumPosition = std::numeric_limits<double>::quiet_NaN();
    double farBandPosition = std::numeric_limits<double>::quiet_NaN();
    double headingAngleDeg = std::numeric_limits<double>::quiet_NaN();
    double finalSteering = std::numeric_limits<double>::quiet_NaN();

    std::string virtualState = "INVALID";
    std::string lineState = "INVALID";
};

// Resultado tipado do IPC visual. A classificação verde é somente percepção
// e não oferece nenhum campo que possa ser convertido em comando de motor.
struct CameraLineSnapshot
{
    bool sourceFresh = false;
    double lineFollowerLeftPower = 0.0;
    double lineFollowerRightPower = 0.0;
    std::string lineControlSource = "unknown";
    bool lineNearDetected = false;

    bool greenPathBlackValid = false;
    std::uint64_t greenCandidateCount = 0;
    bool greenConfirmed = false;
    GreenInterpretation greenInterpretation = GreenInterpretation::None;

    double lineTimestamp = 0.0;
    std::uint64_t lineSequence = 0;
    double ageMs = 0.0;
    CameraCurveDiagnostics curveDiagnostics;
};

// Monitora a saúde da câmera e rejeita IPC ausente, antigo ou inválido.
class CameraMonitor
{
public:
    bool ready() const;
    CameraLineSnapshot lineSnapshot();

private:
    CameraLineSnapshot cachedLineSnapshot_;
    bool hasCachedLineSnapshot_ = false;
};
