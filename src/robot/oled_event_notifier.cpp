#include "obr/oled_event_notifier.h"

#include "obr/config.h"

#include <iostream>

OledEventNotifier::OledEventNotifier(Esp32Bridge& esp32)
    : esp32_(esp32)
{
}

void OledEventNotifier::updateGreen(
    const CameraLineSnapshot& cameraSnapshot,
    bool displayAvailable)
{
    if (!cameraSnapshot.sourceFresh)
    {
        // Uma leitura antiga não confirma nem rearma eventos visuais.
        return;
    }

    if (!isConfirmedGreen(cameraSnapshot))
    {
        greenAlertLatched_ = false;
        lastGreenInterpretation_ = GreenInterpretation::None;
        return;
    }

    if (!displayAvailable ||
        (greenAlertLatched_ &&
         lastGreenInterpretation_ == cameraSnapshot.greenInterpretation))
    {
        return;
    }

    const char* direction = greenDirectionText(cameraSnapshot.greenInterpretation);
    if (direction != nullptr && esp32_.sendOledLargeMessage(
                                    "VERDE",
                                    direction,
                                    config::kOledNavigationAlertDurationMs))
    {
        greenAlertLatched_ = true;
        lastGreenInterpretation_ = cameraSnapshot.greenInterpretation;
        std::cout << "OLED green alert: " << direction << "\n";
    }
}

void OledEventNotifier::updateObstacleDetour(
    bool obstacleConfirmed,
    bool displayAvailable)
{
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
