#pragma once

#include "obr/camera_monitor.h"
#include "obr/curve_diagnostics_logger.h"
#include "obr/dashboard_server.h"
#include "obr/esp32_bridge.h"
#include "obr/forward_ball_vision_lifecycle.h"
#include "obr/mission_controller.h"
#include "obr/motor_controller.h"
#include "obr/oled_event_notifier.h"
#include "obr/robot_state.h"
#include "obr/servo_controller.h"
#include "obr/status_led.h"
#include "obr/telemetry.h"

#include <atomic>

// Possui os módulos de runtime e coordena inicialização, loop e desligamento.
// As decisões de movimento continuam delegadas às missões e aos controladores.
class RobotApplication
{
public:
    RobotApplication();
    int run(const std::atomic<bool>& running);

private:
    RobotState robotState_;
    Telemetry telemetry_;
    Esp32Bridge esp32_;
    ServoController servos_;
    CameraMonitor cameraMonitor_;
    ForwardBallVisionLifecycle forwardBallVision_;
    MissionController missionController_;
    MotorController motors_;
    OledEventNotifier oledEvents_;
    CurveDiagnosticsLogger curveDiagnosticsLogger_;
    StatusLed readyLed_;
    DashboardServer dashboard_;
};
