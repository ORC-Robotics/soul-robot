#include "obr/silver_entry_maneuver.h"

#include "obr/config.h"

#include <algorithm>
#include <cmath>

namespace
{
AutonomousStatus makeStatus(
    const std::string& phase,
    const std::string& action,
    double progressPercent = 0.0)
{
    AutonomousStatus status;
    status.phase = phase;
    status.action = action;
    status.progressPercent = progressPercent;
    return status;
}

bool encodersReady(const Esp32TelemetrySnapshot& telemetry)
{
    return telemetry.readyForOperation() && telemetry.lastSensorAgeMs >= 0 &&
           telemetry.lastSensorAgeMs <= config::kSilverEntryEncoderFreshnessMs &&
           std::isfinite(telemetry.leftEncoderRate) &&
           std::isfinite(telemetry.rightEncoderRate);
}
}

void SilverEntryManeuver::reset()
{
    active_ = false;
    visionConfirmed_ = false;
    startLeftCount_ = 0;
    startRightCount_ = 0;
    startTime_ = {};
}

SilverEntryOutput SilverEntryManeuver::update(
    const CameraLineSnapshot& vision,
    const Esp32TelemetrySnapshot& telemetry)
{
    SilverEntryOutput output;
    const bool grayConfirmed = vision.sourceFresh &&
                               vision.courseMarkerConfirmed &&
                               vision.courseMarker == CourseMarker::Gray;

    if (!active_)
    {
        if (!vision.sourceFresh || !vision.silverCandidateDetected)
        {
            return output;
        }
        if (!encodersReady(telemetry))
        {
            output.hasControl = true;
            output.status = makeStatus(
                "silver_entry_waiting_encoders",
                "Faixa cinza detectada: aguardando encoders");
            return output;
        }

        active_ = true;
        visionConfirmed_ = grayConfirmed;
        startLeftCount_ = telemetry.leftEncoderCount;
        startRightCount_ = telemetry.rightEncoderCount;
        startTime_ = std::chrono::steady_clock::now();
    }

    output.hasControl = true;
    visionConfirmed_ = visionConfirmed_ || grayConfirmed;

    // Quatro frames válidos encerram a janela imediatamente. Não é necessário
    // percorrer os 5 cm completos quando a câmera já confirmou a faixa prata.
    if (visionConfirmed_)
    {
        output.completed = true;
        output.status = makeStatus(
            "silver_entry_completed",
            "Faixa cinza confirmada durante o avanço",
            100.0);
        return output;
    }

    if (!encodersReady(telemetry))
    {
        output.status = makeStatus(
            "silver_entry_encoder_lost",
            "Avanço cinza pausado: encoders sem dados recentes");
        return output;
    }

    const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - startTime_)
                               .count();
    if (elapsedMs > config::kSilverEntryAdvanceTimeoutMs)
    {
        output.status = makeStatus(
            "silver_entry_timeout",
            "Avanço cinza interrompido: distância não concluída");
        return output;
    }

    const double leftCm = std::abs(
                              telemetry.leftEncoderCount - startLeftCount_) /
                          config::kEncoderCountsPerCentimeter;
    const double rightCm = std::abs(
                               telemetry.rightEncoderCount - startRightCount_) /
                           config::kEncoderCountsPerCentimeter;
    const double averageCm = (leftCm + rightCm) * 0.5;
    const double progress = std::clamp(
        averageCm / config::kSilverEntryAdvanceDistanceCm * 100.0,
        0.0,
        100.0);

    if (averageCm >= config::kSilverEntryAdvanceDistanceCm)
    {
        // Sem quatro positivos dentro da janela, a leitura é descartada e o
        // segue-linha recupera autoridade neste mesmo ciclo de controle.
        reset();
        output.hasControl = false;
        return output;
    }

    output.leftPower = config::kSilverEntryAdvancePower;
    output.rightPower = config::kSilverEntryAdvancePower;
    output.status = makeStatus(
        "silver_entry_advancing",
        "Faixa cinza detectada: validando durante avanço de até 5 cm",
        progress);
    return output;
}
