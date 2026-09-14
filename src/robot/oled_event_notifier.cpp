#include "obr/oled_event_notifier.h"

#include "obr/config.h"

#include <iostream>

OledEventNotifier::OledEventNotifier(Esp32Bridge& esp32)
    : esp32_(esp32)
{
}

void OledEventNotifier::updateRedFinish(
    bool confirmed, bool missionFinished, bool displayAvailable)
{
    redAlertPending_ = redAlertPending_ || confirmed;
    redAlertPriority_ = missionFinished || redAlertPending_;
    if (redAlertPending_ && displayAvailable &&
        esp32_.sendOledLargeMessage("Vermelho", "CHEGADA", config::kOledNavigationAlertDurationMs))
    {
        redAlertPending_ = false;
        std::cout << "OLED red finish alert\n";
    }
}

void OledEventNotifier::updateLineEvents(
    const CameraLineSnapshot& cameraSnapshot,
    bool displayAvailable)
{
    if (redAlertPriority_ || !cameraSnapshot.sourceFresh)
    {
        // Uma leitura antiga não confirma nem rearma eventos visuais.
        return;
    }

    const bool greenConfirmed = isConfirmedGreen(cameraSnapshot);
    const bool gapConfirmed = isConfirmedGap(cameraSnapshot);
    const bool grayConfirmed = isConfirmedGray(cameraSnapshot);

    if (!greenConfirmed)
    {
        greenAlertLatched_ = false;
        lastGreenInterpretation_ = GreenInterpretation::None;
    }
    if (!gapConfirmed)
    {
        gapAlertLatched_ = false;
    }
    if (!grayConfirmed)
    {
        grayAlertLatched_ = false;
    }
    if (!displayAvailable)
    {
        return;
    }

    // Marcadores de fase têm prioridade sobre manobras e GAP. O retorno evita
    // que dois eventos confirmados no mesmo frame sobrescrevam a OLED.
    if (grayConfirmed)
    {
        if (!grayAlertLatched_ && esp32_.sendOledLargeMessage(
                                      "CINZA",
                                      "CONFIRMADO",
                                      config::kOledNavigationAlertDurationMs))
        {
            grayAlertLatched_ = true;
            std::cout << "OLED gray-marker alert\n";
        }
        return;
    }

    if (greenConfirmed)
    {
        if (greenAlertLatched_ &&
            lastGreenInterpretation_ == cameraSnapshot.greenInterpretation)
        {
            return;
        }
        const char* direction = greenDirectionText(
            cameraSnapshot.greenInterpretation);
        if (direction != nullptr && esp32_.sendOledLargeMessage(
                                        "VERDE",
                                        direction,
                                        config::kOledNavigationAlertDurationMs))
        {
            greenAlertLatched_ = true;
            lastGreenInterpretation_ = cameraSnapshot.greenInterpretation;
            std::cout << "OLED green alert: " << direction << "\n";
        }
        return;
    }

    if (gapConfirmed && !gapAlertLatched_ && esp32_.sendOledLargeMessage(
                                                   "GAP",
                                                   "CONFIRMADO",
                                                   config::kOledNavigationAlertDurationMs))
    {
        gapAlertLatched_ = true;
        std::cout << "OLED gap alert\n";
    }
}

void OledEventNotifier::updateObstacleDetour(
    bool obstacleConfirmed,
    bool displayAvailable)
{
    if (redAlertPriority_) return;
    if (!obstacleConfirmed)
    {
        obstacleAlertLatched_ = false;
        return;
    }
    if (!displayAvailable || obstacleAlertLatched_)
    {
        return;
    }

    // A futura rotina de obstáculo deve chamar este método com true somente
    // depois de validar a detecção e de realmente assumir o desvio.
    if (esp32_.sendOledLargeMessage(
            "DESVIO",
            "",
            config::kOledNavigationAlertDurationMs))
    {
        obstacleAlertLatched_ = true;
        std::cout << "OLED obstacle-detour alert\n";
    }
}

const char* OledEventNotifier::greenDirectionText(
    GreenInterpretation interpretation)
{
    switch (interpretation)
    {
    case GreenInterpretation::Left:
        return "ESQUERDA";
    case GreenInterpretation::Right:
        return "DIREITA";
    case GreenInterpretation::TurnAround180:
        return "180 GRAUS";
    case GreenInterpretation::None:
    case GreenInterpretation::FalseMarker:
    case GreenInterpretation::Ambiguous:
        return nullptr;
    }
    return nullptr;
}

bool OledEventNotifier::isConfirmedGreen(
    const CameraLineSnapshot& cameraSnapshot)
{
    return cameraSnapshot.sourceFresh && cameraSnapshot.greenConfirmed &&
           cameraSnapshot.greenPathBlackValid &&
           greenDirectionText(cameraSnapshot.greenInterpretation) != nullptr;
}

bool OledEventNotifier::isConfirmedGap(
    const CameraLineSnapshot& cameraSnapshot)
{
    return cameraSnapshot.sourceFresh &&
           cameraSnapshot.gapValidationDecision == "GAP";
}

bool OledEventNotifier::isConfirmedGray(
    const CameraLineSnapshot& cameraSnapshot)
{
    return cameraSnapshot.sourceFresh &&
           cameraSnapshot.courseMarkerConfirmed &&
           cameraSnapshot.courseMarker == CourseMarker::Gray;
}
