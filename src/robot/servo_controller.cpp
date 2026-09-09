#include "obr/servo_controller.h"

#include "obr/config.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

ServoController::ServoController(Esp32Bridge& esp32)
    : esp32_(esp32)
{
}

bool ServoController::apply(const RobotSnapshot& state)
{
    const auto now = std::chrono::steady_clock::now();

    if (state.servoCalibrationActive && !state.emergencyStop)
    {
        // A calibração usa pulsos brutos enviados de forma explícita pelo
        // operador. O laço normal não pode substituí-los por ângulos nem enviar
        // SERVO_DISABLE_ALL enquanto esse modo dedicado estiver ativo.
        return true;
    }

    const bool operatingMode = state.mode == "manual" || state.mode == "autonomous";
    const bool anyServoRequested = state.armServoRequested ||
                                   state.wristServoRequested ||
                                   state.gripperServoRequested;
    if (!operatingMode || state.emergencyStop || !anyServoRequested)
    {
        return disableAll();
    }

    bool sent = true;
    if (state.armServoRequested && state.wristServoRequested &&
        state.gripperServoRequested)
    {
        if (hasAppliedState_ &&
            state.servoCommandSequence == lastCommandSequence_)
        {
            // Mantém a referência temporal próxima do loop normal para que um
            // comando posterior não comece com um passo maior por tempo ocioso.
            lastMotionUpdate_ = now;
            return true;
        }

        // Cada passo envia novamente a pose completa. Braço e garra permanecem
        // energizados nos alvos memorizados enquanto apenas o pulso avança.
        ServoPose nextPose = state.servoPose;
        const bool reactivatingPose = outputsDisabled_ && hasAppliedPose_;
        if (reactivatingPose)
        {
            // Primeiro reaplica a garra e mantém o pulso no último alvo conhecido.
            // O movimento começa somente após o breve tempo de estabilização.
            nextPose.wristDegrees = appliedPose_.wristDegrees;
            wristMotionAllowedAt_ = now + std::chrono::milliseconds(
                                             config::kServoPoseHoldBeforeWristMotionMs);
        }
        else if (hasAppliedPose_)
        {
            double elapsedSeconds = 0.0;
            if (now >= wristMotionAllowedAt_)
            {
                elapsedSeconds = std::min(
                    std::chrono::duration<double>(now - lastMotionUpdate_).count(),
                    static_cast<double>(config::kServoMotionMaximumElapsedMs) / 1000.0);
            }
            const double maximumStepDegrees =
                config::kWristServoMaximumSpeedDegreesPerSecond *
                std::max(0.0, elapsedSeconds);
            nextPose.wristDegrees = moveAngleToward(
                appliedPose_.wristDegrees,
                state.servoPose.wristDegrees,
                maximumStepDegrees);
        }

        sent = setPose(nextPose);
        if (sent)
        {
            appliedPose_ = nextPose;
            hasAppliedPose_ = true;
            lastMotionUpdate_ = now;

            const bool wristReachedTarget =
                nextPose.wristDegrees == state.servoPose.wristDegrees;
            hasAppliedState_ = wristReachedTarget;
            if (wristReachedTarget)
            {
                lastCommandSequence_ = state.servoCommandSequence;
            }
            outputsDisabled_ = false;
        }
        return sent;
    }
    else
    {
        // No controle manual, cada servo passa a fazer parte do estado quando
        // recebe seu primeiro alvo. Os demais canais continuam sem sinal.
        if (state.armServoRequested)
        {
            sent = setAngle(ServoId::Arm, state.servoPose.armDegrees) && sent;
        }
        if (state.wristServoRequested)
        {
            sent = setAngle(ServoId::Wrist, state.servoPose.wristDegrees) && sent;
        }
        if (state.gripperServoRequested)
        {
            sent = setAngle(ServoId::Gripper, state.servoPose.gripperDegrees) && sent;
        }
    }

    if (sent)
    {
        lastCommandSequence_ = state.servoCommandSequence;
        hasAppliedState_ = true;
        outputsDisabled_ = false;
        hasAppliedPose_ = true;
        if (state.armServoRequested)
        {
            appliedPose_.armDegrees = state.servoPose.armDegrees;
        }
        if (state.wristServoRequested)
        {
            appliedPose_.wristDegrees = state.servoPose.wristDegrees;
        }
        if (state.gripperServoRequested)
        {
            appliedPose_.gripperDegrees = state.servoPose.gripperDegrees;
        }
        lastMotionUpdate_ = now;
    }
    return sent;
}

bool ServoController::setAngle(ServoId servo, double angleDegrees)
{
    if (!isValidAngle(angleDegrees))
    {
        std::cerr << "Servo angle ignored: expected a value from 0 to 180 degrees\n";
        return false;
    }
    return esp32_.sendServoAngle(servo, angleDegrees);
}

bool ServoController::setPose(const ServoPose& pose)
{
    // A pose só é enviada quando os três ângulos são válidos. A ESP32 repete a
    // validação antes de alterar qualquer canal físico do PCA9685.
    if (!isValidAngle(pose.armDegrees) ||
        !isValidAngle(pose.wristDegrees) ||
        !isValidAngle(pose.gripperDegrees))
    {
        std::cerr << "Servo pose ignored: every angle must be from 0 to 180 degrees\n";
        return false;
    }
    return esp32_.sendServoPose(pose);
}

bool ServoController::disableAll()
{
    // Mesmo sem sinal nos servos, o relógio continua acompanhando o loop para
    // que a reativação não use um intervalo ocioso como um salto de movimento.
    lastMotionUpdate_ = std::chrono::steady_clock::now();

    if (outputsDisabled_)
    {
        return true;
    }

    const bool sent = esp32_.sendDisableAllServos();
    if (sent)
    {
        outputsDisabled_ = true;
        hasAppliedState_ = false;
    }
    return sent;
}

double ServoController::moveAngleToward(double currentDegrees,
                                        double targetDegrees,
                                        double maximumStepDegrees)
{
    if (!std::isfinite(currentDegrees) || !std::isfinite(targetDegrees) ||
        !std::isfinite(maximumStepDegrees) || maximumStepDegrees <= 0.0)
    {
        return currentDegrees;
    }

    const double difference = targetDegrees - currentDegrees;
    if (std::abs(difference) <= maximumStepDegrees)
    {
        return targetDegrees;
    }
    return currentDegrees + std::copysign(maximumStepDegrees, difference);
}

bool ServoController::beginCalibration()
{
    return esp32_.sendServoCalibrationBegin();
}

bool ServoController::endCalibration()
{
    outputsDisabled_ = true;
    hasAppliedState_ = false;
    return esp32_.sendServoCalibrationEnd();
}

bool ServoController::disableCalibrationOutput()
{
    return esp32_.sendServoCalibrationDisableOutput();
}

bool ServoController::setCalibrationPulse(ServoId servo, int pulseUs)
{
    if (!isValidCalibrationPulse(pulseUs))
    {
        std::cerr << "Servo calibration pulse ignored: value outside the absolute safe range\n";
        return false;
    }
    return esp32_.sendServoCalibrationPulse(servo, pulseUs);
}

bool ServoController::saveCalibration(ServoId servo, int pulseAtZeroUs,
                                      int pulseAt180Us)
{
    if (!isValidCalibrationEndpoints(pulseAtZeroUs, pulseAt180Us))
    {
        std::cerr << "Servo calibration save ignored: invalid endpoints\n";
        return false;
    }
    return esp32_.sendServoCalibrationSave(
        servo, pulseAtZeroUs, pulseAt180Us);
}

bool ServoController::isValidAngle(double angleDegrees)
{
    return std::isfinite(angleDegrees) &&
           angleDegrees >= config::kServoMinimumAngleDegrees &&
           angleDegrees <= config::kServoMaximumAngleDegrees;
}

bool ServoController::isValidCalibrationPulse(int pulseUs)
{
    return pulseUs >= config::kServoCalibrationAbsoluteMinimumPulseUs &&
           pulseUs <= config::kServoCalibrationAbsoluteMaximumPulseUs;
}

bool ServoController::isValidCalibrationEndpoints(int pulseAtZeroUs,
                                                  int pulseAt180Us)
{
    return isValidCalibrationPulse(pulseAtZeroUs) &&
           isValidCalibrationPulse(pulseAt180Us) &&
           std::abs(pulseAt180Us - pulseAtZeroUs) >=
               config::kServoCalibrationMinimumSpanUs;
}
