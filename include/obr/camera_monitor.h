#pragma once

#include <cstdint>

enum class GreenTurnDecision
{
    None,
    Approach,
    GuideLeft,
    GuideRight,
    TurnAround180
};

// Resultado tipado da telemetria rápida calculada pelo processo de visão.
// A Missão Principal usa estes valores para seguir e recuperar a linha.
struct CameraLineSnapshot
{
    bool sourceFresh = false;
    bool nearValid = false;
    double nearX = 0.0;
    double nearError = 0.0;
    bool farValid = false;
    double farX = 0.0;
    double farError = 0.0;
    double lateralError = 0.0;
    double headingError = 0.0;
    // O controle principal usa estes campos somente quando o fit visual foi
    // validado. As coordenadas são normalizadas e não representam metros.
    bool trajectoryValid = false;
    double fitA = 0.0;
    double fitB = 0.0;
    double fitC = 0.0;
    double fitQuality = 0.0;
    double fitRmsError = 0.0;
    std::uint64_t fitSampleCount = 0;
    double lookaheadX = 0.0;
    double lookaheadY = 0.0;
    double curvature = 0.0;
    double adaptivePreview = 0.0;
    double previewError = 0.0;
    double pTerm = 0.0;
    double filteredDerivative = 0.0;
    double dTerm = 0.0;
    double controlError = 0.0;
    double previewFactor = 0.0;
    double kControl = 0.0;
    // Mantidos apenas para consumidores antigos da telemetria.
    double kNear = 0.0;
    double kFar = 0.0;
    double correction = 0.0;
    double leftPreview = 0.0;
    double rightPreview = 0.0;
    // A visão confirma a extremidade usando o componente conectado à faixa
    // inferior. A continuação projetada deve ser um componente desconectado.
    bool gapCandidate = false;
    bool gapAlignmentValid = false;
    double gapAlignmentError = 0.0;
    bool gapReturnValid = false;
    double gapReturnError = 0.0;
    // A visão aplica as ROIs verdes às prévias somente depois de reconhecer o
    // marcador. Sem verde, o segue-linha mantém integralmente o controle antigo.
    bool greenNearSeen = false;
    bool greenPathBlackValid = false;
    bool greenConfirmed = false;
    GreenTurnDecision greenTurnDecision = GreenTurnDecision::None;
    double lineTimestamp = 0.0;
    std::uint64_t lineSequence = 0;
    double ageMs = 0.0;
};

// Monitora a saúde da câmera e valida a telemetria visual usada pelo controle.
// Este módulo não interpreta pixels; apenas rejeita dados ausentes ou inválidos.
class CameraMonitor
{
public:
    bool ready() const;
    CameraLineSnapshot lineSnapshot();

private:
    CameraLineSnapshot cachedLineSnapshot_;
    bool hasCachedLineSnapshot_ = false;
};
