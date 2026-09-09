#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"

// Converte eventos confirmados do robô em alertas temporários na OLED.
// O latch evita repetir a mesma mensagem a cada frame da câmera.
class OledEventNotifier
{
public:
    explicit OledEventNotifier(Esp32Bridge& esp32);

    void updateLineEvents(const CameraLineSnapshot& cameraSnapshot,
                          bool displayAvailable);
    void updateObstacleDetour(bool obstacleConfirmed, bool displayAvailable);

    static const char* greenDirectionText(GreenInterpretation interpretation);
    static bool isConfirmedGreen(const CameraLineSnapshot& cameraSnapshot);
    static bool isConfirmedGap(const CameraLineSnapshot& cameraSnapshot);
    static bool isConfirmedGray(const CameraLineSnapshot& cameraSnapshot);

private:
    Esp32Bridge& esp32_;
    bool greenAlertLatched_ = false;
    GreenInterpretation lastGreenInterpretation_ = GreenInterpretation::None;
    bool gapAlertLatched_ = false;
    bool grayAlertLatched_ = false;
    bool obstacleAlertLatched_ = false;
};
