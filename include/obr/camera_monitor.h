#pragma once

#include <cstdint>
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

// Resultado tipado do IPC visual. A classificação verde é somente percepção
// e não oferece nenhum campo que possa ser convertido em comando de motor.
struct CameraLineSnapshot
{
    bool sourceFresh = false;
    double lineFollowerLeftPower = 0.0;
    double lineFollowerRightPower = 0.0;
    std::string lineControlSource = "unknown";

    bool greenPathBlackValid = false;
    std::uint64_t greenCandidateCount = 0;
    bool greenConfirmed = false;
    GreenInterpretation greenInterpretation = GreenInterpretation::None;

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
