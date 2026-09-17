#pragma once

#include <cstdint>
#include <array>

struct AutonomousStatus;
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
    // Topologia da fita, mantida para diagnóstico; a saída fixa também aceita cruzamentos.
    bool silverClassifierFresh = false;
    std::uint64_t silverSequence = 0;
    bool exitLineUnbranched = false;
    bool sourceFresh = false;
    double lineFollowerLeftPower = 0.0;
    double lineFollowerRightPower = 0.0;
    std::string lineControlSource = "unknown";
    bool lineNearDetected = false;
    // Posição normalizada da linha no NEAR usada somente para a correção
    // inicial do retorno. NaN impede qualquer giro sem uma leitura válida.
    double lineNearFinePosition = std::numeric_limits<double>::quiet_NaN();

    bool greenPathBlackValid = false;
    // Preto válido na ROI superior do marcador atual, independente da
    // classificação lateral. Autoriza espera extra, nunca confirma um lado.
    bool greenFrontRoiValid = false;
    // Verdadeiro somente quando os dois verdes do frame atual possuem altura
    // e orientação compatíveis com a mesma interseção física.
    bool greenPairCompatible = false;
    std::uint64_t greenCandidateCount = 0;
    bool greenConfirmed = false;
    // Evidência geométrica do frame atual antes da confirmação temporal.
    // O coordenador usa este campo apenas para latchear o lado do evento.
    GreenInterpretation greenRawInterpretation = GreenInterpretation::None;
    GreenInterpretation greenInterpretation = GreenInterpretation::None;

    // Marcadores de transição só têm efeito depois da confirmação temporal
    // feita pela visão. A ausência destes campos preserva o percurso atual.
    // Evidência HSV independente de preto, verde e do classificador de prata.
    bool redValid = false;
    double redRatio = 0.0;
    bool redConfirmed = false;
    bool redClearConfirmed = false;
    bool courseMarkerConfirmed = false;
    CourseMarker courseMarker = CourseMarker::None;
    // Expõe a candidata ainda não confirmada para a manobra curta de entrada.
    // Esse campo sozinho nunca autoriza a troca para a área de resgate.
    bool silverCandidateDetected = false;
    bool rescueExitConfirmed = false;

    double lineTimestamp = 0.0;
    std::uint64_t lineSequence = 0;
    double ageMs = 0.0;
    bool farTrusted = false;
    bool mediumTrusted = false;
    // Confirma um comando de seguimento LINE aceito pela Raspberry. No Fusion,
    // a roda interna pode ficar negativa sem transformar o comando em search.
    bool normalSteeringValid = false;
    // Faixa transversal completa em MEDIUM ou FAR; ausência bloqueia apenas
    // a recuperação antecipada durante a curva do obstáculo.
    bool obstacleContinuationBand = false;
    // Direção lateral calculada pelo recovery inferior exclusivamente com
    // posições trusted. A Raspberry apenas memoriza esta decisão entre frames.
    std::string trustedDirection = "NONE";
    // Decisão do Python inferior; não contém steering frontal.
    std::string gapValidationDecision = "NORMAL";
    std::string nearLineState = "UNKNOWN";
    CameraCurveDiagnostics curveDiagnostics;
};

// Evidência de um componente preto por setor; nunca contém comandos de motor.
struct ExitCandidate
{
    bool visible = false;
    double txDegrees = 0.0;
    double score = 0.0;
    int depthBands = 0;
    int nearestBand = 0;
    bool tapeValid = false;
    bool guidanceValid = false;
    double guidanceAngleDegrees = 90.0;
    double entryAngleDegrees = 90.0;
    double entryOffsetNormalized = 0.0;
    double entryDepthNormalized = 0.0;
    double entryX = 0.0;
    double entryY = 0.0;
    bool blockedByColor = false;
    bool grayNoiseLikely = false;
    bool solidBlack = false;
};

// Trajetória frontal auxiliar. O Python inferior usa sua evidência para GAP;
// após a saída, a Raspberry pode usar somente o lado para recuperar a CAM0.
struct ForwardLineSnapshot
{
    // Contrato separado: apenas RescueExitMission usa estas evidências para aproximação.
    bool exitAnalysisActive = false;
    std::uint64_t exitRunSequence = 0;
    bool cameraObscured = false;
    std::array<ExitCandidate, 5> exitCandidates{};
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
    // Evidência simples e independente usada somente durante o scan de obstáculo.
    bool obstacleBlackVisible = false;
    std::uint64_t obstacleBlackPixelCount = 0;
    double obstacleBlackRatio = 0.0;
    std::uint64_t obstacleBlackLargestComponent = 0;
    std::uint64_t obstacleBlackSequence = 0;
    std::uint64_t parabolaLeftBlack = 0;
    std::uint64_t parabolaRightBlack = 0;
    std::uint64_t parabolaSequence = 0;
    std::uint64_t parabolaNearForwardBlack = 0;
    std::uint64_t parabolaNearForwardLargest = 0;
    bool parabolaNearForwardVisible = false;

    bool lineObservationValid() const;
    bool normalCommandValid() const;
};

// Leitura da vítima travada publicada pelo processo da câmera frontal.
// tx é negativo à esquerda e positivo à direita do centro da imagem.
struct ForwardBallSnapshot
{
    bool sourceFresh = false;
    bool detected = false;
    // Indica uma candidata ainda em confirmação temporal. Ela nunca autoriza
    // alinhamento, mas interrompe a busca enquanto o target lock decide.
    bool candidateVisible = false;
    // Direção da candidata do frame atual, antes da confirmação. Serve somente
    // para orientar a busca; nunca autoriza alinhamento ou coleta.
    double candidateTxDegrees = std::numeric_limits<double>::quiet_NaN();
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

enum class RescueZoneGeometryState
{
    NotDetected,
    BoundsUnknown,
    LeftBoundOnly,
    RightBoundOnly,
    FullBounds
};

// Preserva separadamente a observação atual e a confirmação temporal da zona.
// ALIGN_ZONE só pode mover com uma amostra atual e confirmada do mesmo frame.
struct RescueZoneObservation
{
    bool candidateDetected = false;
    bool detected = false;
    RescueZoneGeometryState geometryState =
        RescueZoneGeometryState::NotDetected;
    bool aimValid = false;
    double aimX = std::numeric_limits<double>::quiet_NaN();
    double aimNormalized = std::numeric_limits<double>::quiet_NaN();
    double frameCoverage = 0.0;
};

// Snapshot atômico das duas cores publicado pela câmera frontal.
struct RescueZoneSnapshot
{
    bool sourceFresh = false;
    std::uint64_t sequence = 0;
    double timestamp = 0.0;
    double ageMs = 0.0;
    bool cameraObscured = false;
    RescueZoneObservation green;
    RescueZoneObservation red;
};

// Monitora a saúde da câmera e rejeita IPC ausente, antigo ou inválido.
class CameraMonitor
{
public:
    // Um caminho alternativo permite testar o contrato IPC sem câmera nem motores.
    explicit CameraMonitor(
        std::string forwardLineStatusPath = {},
        std::string rescueZoneStatusPath = {},
        std::string forwardBallStatusPath = {},
        std::string lineStatusPath = {})
        : forwardLineStatusPath_(std::move(forwardLineStatusPath)),
          rescueZoneStatusPath_(std::move(rescueZoneStatusPath)),
          forwardBallStatusPath_(std::move(forwardBallStatusPath)),
          lineStatusPath_(std::move(lineStatusPath)) {}
    bool ready() const;
    CameraLineSnapshot lineSnapshot();
    ForwardLineSnapshot forwardLineSnapshot();
    // Publica o heartbeat e o diagnóstico da busca sem conceder autoridade de motor.
    bool publishExitControl(bool enabled, std::uint64_t runSequence,
                            const AutonomousStatus& status,
                            bool greenYawValid = false,
                            double greenYawDegrees = 0.0,
                            double greenGyroDegreesPerSecond = 0.0,
                            double greenYawAgeMs = 0.0,
                            bool exitOverlayEnabled = false) const;
    ForwardBallSnapshot forwardBallSnapshot() const;
    RescueZoneSnapshot rescueZoneSnapshot() const;
    bool setForwardBallDetectionEnabled(bool enabled) const;
    bool requestForwardBallTarget(
        std::uint64_t sequence,
        const std::string& targetType) const;
    bool publishRescueZoneDetectionInput(
        bool enabled,
        bool ultrasonicFresh,
        bool ultrasonicValid,
        double ultrasonicDistanceCm) const;

private:
    std::string forwardLineStatusPath_;
    std::string rescueZoneStatusPath_;
    std::string forwardBallStatusPath_;
    std::string lineStatusPath_;
    CameraLineSnapshot cachedLineSnapshot_;
    bool hasCachedLineSnapshot_ = false;
    ForwardLineSnapshot cachedForwardLineSnapshot_;
    bool hasCachedForwardLineSnapshot_ = false;
};
