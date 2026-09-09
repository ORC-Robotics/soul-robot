#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <utility>

enum class GreenInterpretation
{
    None,
    FalseMarker,
    Ambiguous,
    Left,
    Right,
    TurnAround180
};

enum class CourseMarker
{
    None,
    Gray,
    Red
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

// Resultado tipado do IPC visual. Consumidores devem validar a classificação
// verde antes de usá-la em alertas ou em uma decisão segura de movimento.
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

    // Marcadores de transição só têm efeito depois da confirmação temporal
    // feita pela visão. A ausência destes campos preserva o percurso atual.
    bool courseMarkerConfirmed = false;
    CourseMarker courseMarker = CourseMarker::None;
    bool rescueExitConfirmed = false;

    double lineTimestamp = 0.0;
    std::uint64_t lineSequence = 0;
    double ageMs = 0.0;
    bool farTrusted = false;
    bool mediumTrusted = false;
    // Confirma um comando de seguimento LINE aceito pela Raspberry. No Fusion,
    // a roda interna pode ficar negativa sem transformar o comando em search.
    bool normalSteeringValid = false;
    // Direção lateral calculada pelo recovery inferior exclusivamente com
    // posições trusted. A Raspberry apenas memoriza esta decisão entre frames.
    std::string trustedDirection = "NONE";
    // Decisão do Python inferior; não contém steering frontal.
    std::string gapValidationDecision = "NORMAL";
    std::string nearLineState = "UNKNOWN";
    CameraCurveDiagnostics curveDiagnostics;
};

// Trajetória frontal auxiliar. O Python inferior usa sua evidência para GAP;
// nenhum consumidor deve convertê-la em comando de motor.
struct ForwardLineSnapshot
{
    bool sourceFresh = false;
    bool visible = false;
    double position = std::numeric_limits<double>::quiet_NaN();
    double confidence = 0.0;
    std::string pathState = "ABSENT";
    bool present = false;
    bool referenceValid = false;
    // Compatibilidade da estrutura: potências antigas nunca têm autoridade.
    double normalLeftPower = 0.0;
    double normalRightPower = 0.0;
    double timestamp = 0.0;
    std::uint64_t sequence = 0;
    double ageMs = 0.0;

    bool lineObservationValid() const;
    bool normalCommandValid() const;
};

// Leitura da vítima travada publicada pelo processo da câmera frontal.
// tx é negativo à esquerda e positivo à direita do centro da imagem.
struct ForwardBallSnapshot
{
    bool sourceFresh = false;
    bool detected = false;
    std::string type;
    double txDegrees = std::numeric_limits<double>::quiet_NaN();
    double distanceCm = std::numeric_limits<double>::quiet_NaN();
    double radiusPixels = std::numeric_limits<double>::quiet_NaN();
    double visibleAreaPixels = std::numeric_limits<double>::quiet_NaN();
    std::uint64_t targetSequence = 0;
    bool targetLocked = false;
    double timestamp = 0.0;
    double ageMs = 0.0;
};

// Monitora a saúde da câmera e rejeita IPC ausente, antigo ou inválido.
class CameraMonitor
{
public:
    // Um caminho alternativo permite testar o contrato IPC sem câmera nem motores.
    explicit CameraMonitor(std::string forwardLineStatusPath = {})
        : forwardLineStatusPath_(std::move(forwardLineStatusPath)) {}
    bool ready() const;
    CameraLineSnapshot lineSnapshot();
    ForwardLineSnapshot forwardLineSnapshot();
    ForwardBallSnapshot forwardBallSnapshot() const;
    bool setForwardBallDetectionEnabled(bool enabled) const;
    bool requestForwardBallTargetSequence(std::uint64_t sequence) const;

private:
    std::string forwardLineStatusPath_;
    CameraLineSnapshot cachedLineSnapshot_;
    bool hasCachedLineSnapshot_ = false;
    ForwardLineSnapshot cachedForwardLineSnapshot_;
    bool hasCachedForwardLineSnapshot_ = false;
};
