#include "obr/oled_event_notifier.h"

#include <iostream>
#include <string>

namespace
{
bool require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << message << '\n';
        return false;
    }
    return true;
}
}

int main()
{
    bool ok = true;
    ok &= require(
        std::string(OledEventNotifier::greenDirectionText(
            GreenInterpretation::Left)) == "ESQUERDA",
        "Left green should use the expected OLED detail");
    ok &= require(
        std::string(OledEventNotifier::greenDirectionText(
            GreenInterpretation::Right)) == "DIREITA",
        "Right green should use the expected OLED detail");
    ok &= require(
        std::string(OledEventNotifier::greenDirectionText(
            GreenInterpretation::TurnAround180)) == "180 GRAUS",
        "Turn-around green should use the expected OLED detail");
    ok &= require(
        OledEventNotifier::greenDirectionText(
            GreenInterpretation::Ambiguous) == nullptr,
        "Ambiguous green should not produce an OLED alert");

    CameraLineSnapshot snapshot;
    snapshot.sourceFresh = true;
    snapshot.greenConfirmed = true;
    snapshot.greenPathBlackValid = true;
    snapshot.greenInterpretation = GreenInterpretation::Right;
    ok &= require(OledEventNotifier::isConfirmedGreen(snapshot),
                  "A fresh confirmed green should be accepted");

    snapshot.sourceFresh = false;
    ok &= require(!OledEventNotifier::isConfirmedGreen(snapshot),
                  "A stale camera result should be rejected");
    snapshot.sourceFresh = true;
    snapshot.greenPathBlackValid = false;
    ok &= require(!OledEventNotifier::isConfirmedGreen(snapshot),
                  "Green without the associated black path should be rejected");
    return ok ? 0 : 1;
}
