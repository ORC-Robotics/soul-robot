#pragma once

#include "obr/esp32_bridge.h"
#include "obr/robot_state.h"

#include <chrono>

// Oferece comandos normais em graus e concentra o modo restrito de calibração.
// Somente esse modo expõe pulsos brutos ao dashboard; canais continuam internos.
class ServoController
{
public:
    explicit ServoController(Esp32Bridge& esp32);

    bool apply(const RobotSnapshot& state);
    bool disableAll();
    bool beginCalibration();
    bool endCalibration();
    bool disableCalibrationOutput();
    bool setCalibrationPulse(ServoId servo, int pulseUs);
    bool saveCalibration(ServoId servo, int pulseAtZeroUs, int pulseAt180Us);

    static bool isValidAngle(double angleDegrees);
    static bool isValidCalibrationPulse(int pulseUs);
    static bool isValidCalibrationEndpoints(int pulseAtZeroUs,
                                            int pulseAt180Us);
    static double moveAngleToward(double currentDegrees,
                                  double targetDegrees,
                                  double maximumStepDegrees);

private:
    Esp32Bridge& esp32_;
    unsigned long long lastCommandSequence_ = 0;
    bool hasAppliedState_ = false;
    bool outputsDisabled_ = true;
    ServoPose appliedPose_{};
    bool hasAppliedPose_ = false;
    std::chrono::steady_clock::time_point lastMotionUpdate_ =
        std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point wristMotionAllowedAt_{};

    bool setAngle(ServoId servo, double angleDegrees);
    bool setPose(const ServoPose& pose);
};
