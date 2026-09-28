#pragma once

#include "obr/camera_monitor.h"
#include "obr/esp32_bridge.h"

// Converte eventos confirmados do robô em alertas temporários na OLED.
// O latch evita repetir a mesma mensagem a cada frame da câmera.
class OledEventNotifier
{
public:
    explicit OledEventNotifier(Esp32Bridge& esp32);

    // A chegada tem prioridade e tenta novamente se o display estiver indisponível.
    void updateRedFinish(bool confirmed, bool missionFinished, bool displayAvailable);
    void updateLineEvents(const CameraLineSnapshot& cameraSnapshot,
                          bool displayAvailable);
    void updateObstacleDetour(bool obstacleConfirmed, bool displayAvailable,
                              int waitSecondsRemaining = -1);

    static const char* greenDirectionText(GreenInterpretation interpretation);
    static bool isConfirmedGreen(const CameraLineSnapshot& cameraSnapshot);
    static bool isConfirmedGap(const CameraLineSnapshot& cameraSnapshot);
    static bool isConfirmedGray(const CameraLineSnapshot& cameraSnapshot);

private:
    Esp32Bridge& esp32_;
    bool redAlertPending_ = false;
    bool redAlertPriority_ = false;
    bool greenAlertLatched_ = false;
    GreenInterpretation lastGreenInterpretation_ = GreenInterpretation::None;
    bool gapAlertLatched_ = false;
    bool grayAlertLatched_ = false;
    bool obstacleAlertLatched_ = false;
    bool obstacleCountdownActive_ = false;
    int lastObstacleWaitSeconds_ = -1;
};
