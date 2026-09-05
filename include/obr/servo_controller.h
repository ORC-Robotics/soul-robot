#pragma once

#include "obr/esp32_bridge.h"
#include "obr/robot_state.h"

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

private:
    Esp32Bridge& esp32_;
    unsigned long long lastCommandSequence_ = 0;
    bool hasAppliedState_ = false;
    bool outputsDisabled_ = true;

    bool setAngle(ServoId servo, double angleDegrees);
    bool setPose(const ServoPose& pose);
};
