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

// Resultado tipado do IPC visual. Os campos de apoio à manobra verde ficam
// separados do comando normal para que o novo seguidor não os reutilize.
struct CameraLineSnapshot
{
    bool sourceFresh = false;
    double lineFollowerLeftPower = 0.0;
    double lineFollowerRightPower = 0.0;

    bool greenManeuverNearValid = false;
    bool greenManeuverFarValid = false;
    bool greenManeuverTrajectoryValid = false;
    bool greenManeuverGapCandidate = false;
    double greenManeuverNearError = 0.0;
    double greenManeuverFarError = 0.0;
    double greenManeuverCorrection = 0.0;
    double greenManeuverLeftPower = 0.0;
    double greenManeuverRightPower = 0.0;

    bool greenNearSeen = false;
    bool greenPathBlackValid = false;
    GreenTurnDecision greenCandidateDecision = GreenTurnDecision::None;
    std::uint64_t greenCandidateFrames = 0;
    bool greenConfirmed = false;
    GreenTurnDecision greenTurnDecision = GreenTurnDecision::None;

    double lineTimestamp = 0.0;
    std::uint64_t lineSequence = 0;
    double ageMs = 0.0;
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
