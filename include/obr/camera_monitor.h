#pragma once

#include <cstdint>

// Resultado tipado da telemetria rápida calculada pelo processo de visão.
// Estes valores são somente diagnósticos e não participam do controle dos motores.
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
    double lineTimestamp = 0.0;
    std::uint64_t lineSequence = 0;
    double ageMs = 0.0;
};

// Monitora a saúde da câmera e lê a telemetria visual somente para diagnóstico.
// Este módulo não interpreta pixels nem participa de decisões de movimento.
class CameraMonitor
{
public:
    bool ready() const;
    CameraLineSnapshot lineSnapshot();

private:
    CameraLineSnapshot cachedLineSnapshot_;
    bool hasCachedLineSnapshot_ = false;
};
