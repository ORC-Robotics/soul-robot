#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"

// Converte eventos confirmados do robô em alertas temporários na OLED.
// O latch evita repetir a mesma mensagem a cada frame da câmera.
class OledEventNotifier
{
public:
    explicit OledEventNotifier(Esp32Bridge& esp32);

    void updateGreen(const CameraLineSnapshot& cameraSnapshot,
                     bool displayAvailable);
    void updateObstacleDetour(bool obstacleConfirmed, bool displayAvailable);

    static const char* greenDirectionText(GreenInterpretation interpretation);
    static bool isConfirmedGreen(const CameraLineSnapshot& cameraSnapshot);

private:
    Esp32Bridge& esp32_;
    bool greenAlertLatched_ = false;
    GreenInterpretation lastGreenInterpretation_ = GreenInterpretation::None;
    bool obstacleAlertLatched_ = false;
};
