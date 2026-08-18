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
    double nearError = 0.0;
    double controlError = 0.0;
    double correction = 0.0;
    double leftPreview = 0.0;
    double rightPreview = 0.0;
    bool farValid = false;
    double farError = 0.0;
    double farArea = 0.0;
    bool centerDeltaValid = false;
    double centerDeltaPx = 0.0;
    // A visão confirma a extremidade e calcula o alinhamento usando somente o
    // componente de fita conectado à NEAR. Uma continuação deve ser desconectada.
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
