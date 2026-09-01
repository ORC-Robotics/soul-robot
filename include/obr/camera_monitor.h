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
// As posições FAR/MEDIUM só participam do controle quando o respectivo gate
// trusted do CameraLineSnapshot confirma que a leitura é válida.
struct CameraCurveDiagnostics
{
    double nearFinePosition = std::numeric_limits<double>::quiet_NaN();
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
    // Posição normalizada da linha no NEAR usada somente para a correção
    // inicial do retorno. NaN impede qualquer giro sem uma leitura válida.
    double lineNearFinePosition = std::numeric_limits<double>::quiet_NaN();

    bool greenPathBlackValid = false;
    std::uint64_t greenCandidateCount = 0;
    bool greenConfirmed = false;
    GreenInterpretation greenInterpretation = GreenInterpretation::None;

    double lineTimestamp = 0.0;
    std::uint64_t lineSequence = 0;
    double ageMs = 0.0;
    bool farTrusted = false;
    bool mediumTrusted = false;
    bool normalSteeringValid = false;
    // Direção lateral calculada pelo recovery inferior exclusivamente com
    // posições trusted. A Raspberry apenas memoriza esta decisão entre frames.
    std::string trustedDirection = "NONE";
    CameraCurveDiagnostics curveDiagnostics;
};

// Leitura leve da câmera frontal e comando produzido pelo mapper NORMAL
// compartilhado com a câmera inferior. Confidence permanece apenas diagnóstico.
struct ForwardLineSnapshot
{
    bool sourceFresh = false;
    bool visible = false;
    double position = std::numeric_limits<double>::quiet_NaN();
    double confidence = 0.0;
    double normalLeftPower = 0.0;
    double normalRightPower = 0.0;
    double timestamp = 0.0;
    std::uint64_t sequence = 0;
    double ageMs = 0.0;

    bool lineObservationValid() const;
    bool normalCommandValid() const;
};

// Monitora a saúde da câmera e rejeita IPC ausente, antigo ou inválido.
class CameraMonitor
{
public:
    bool ready() const;
    CameraLineSnapshot lineSnapshot();
    ForwardLineSnapshot forwardLineSnapshot();

private:
    CameraLineSnapshot cachedLineSnapshot_;
    bool hasCachedLineSnapshot_ = false;
    ForwardLineSnapshot cachedForwardLineSnapshot_;
    bool hasCachedForwardLineSnapshot_ = false;
};
