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

bool alignmentLineVisible(const CameraLineSnapshot& vision)
{
    // A linha usada para a entrada fica abaixo da faixa cinza. O sensor NEAR
    // mede apenas essa região inferior e evita escolher uma linha distante.
    return vision.sourceFresh && vision.lineNearDetected &&
           std::isfinite(vision.lineNearFinePosition) &&
           std::abs(vision.lineNearFinePosition) <= 1.0;
}
}

void SilverEntryManeuver::reset()
{
    active_ = false;
    visionConfirmed_ = false;
    phase_ = Phase::CandidateAdvance;
    startLeftCount_ = 0;
    startRightCount_ = 0;
    startTime_ = {};
    nearCenteredFrames_ = 0;
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
        phase_ = Phase::CandidateAdvance;
        startLeftCount_ = telemetry.leftEncoderCount;
        startRightCount_ = telemetry.rightEncoderCount;
        startTime_ = std::chrono::steady_clock::now();
    }

    output.hasControl = true;
    visionConfirmed_ = visionConfirmed_ || grayConfirmed;

    // Depois da confirmação, qualquer movimento depende de um frame inferior
    // atual. Sem ele, a manobra para em vez de recuar ou girar às cegas.
    if (visionConfirmed_ && !vision.sourceFresh)
    {
        output.status = makeStatus(
            "silver_entry_waiting_line",
            "Aguardando frame atual da linha preta para alinhar a entrada");
        return output;
    }

    if (visionConfirmed_ && phase_ == Phase::CandidateAdvance)
    {
        // A ré acontece sempre antes de procurar a linha inferior. Isso leva
        // a faixa preta para o NEAR sem depender de uma linha distante vista
        // antes da faixa cinza.
        if (!encodersReady(telemetry))
        {
            output.status = makeStatus(
                "silver_entry_waiting_encoders",
                "Faixa cinza confirmada: aguardando encoders para ré");
            return output;
        }
        phase_ = Phase::BackingUpForLine;
        startLeftCount_ = telemetry.leftEncoderCount;
        startRightCount_ = telemetry.rightEncoderCount;
        startTime_ = std::chrono::steady_clock::now();
    }

    if (visionConfirmed_ && phase_ == Phase::BackingUpForLine)
    {
        if (!encodersReady(telemetry))
        {
            output.status = makeStatus(
                "silver_entry_encoder_lost",
                "Ré da faixa cinza pausada: encoders sem dados recentes");
            return output;
        }
        else
        {
            const auto reverseElapsedMs =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - startTime_)
                    .count();
            if (reverseElapsedMs > config::kSilverEntryAdvanceTimeoutMs)
            {
                output.status = makeStatus(
                    "silver_entry_timeout",
                    "Ré da faixa cinza interrompida: distância não concluída");
                return output;
            }
            const double leftCm = std::abs(
                                      telemetry.leftEncoderCount - startLeftCount_) /
                                  config::kEncoderCountsPerCentimeter;
            const double rightCm = std::abs(
                                       telemetry.rightEncoderCount - startRightCount_) /
                                   config::kEncoderCountsPerCentimeter;
            const double averageCm = (leftCm + rightCm) * 0.5;
            if (averageCm < config::kSilverEntryReverseDistanceCm)
            {
                output.leftPower = -config::kSilverEntryReversePower;
                output.rightPower = -config::kSilverEntryReversePower;
                output.status = makeStatus(
                    "silver_entry_backing_up",
                    "Faixa cinza confirmada: recuando para buscar a linha inferior",
                    std::clamp(
                        averageCm / config::kSilverEntryReverseDistanceCm * 100.0,
                        0.0,
                        100.0));
                return output;
            }
            phase_ = Phase::WaitingForLine;
            startTime_ = std::chrono::steady_clock::now();
        }
    }

    if (visionConfirmed_ && phase_ == Phase::WaitingForLine)
    {
        // A câmera continua atual, mas a linha NEAR pode estar ausente.
        // O prazo conta desde o fim da ré e não reabre por uma leitura tardia.
        if (std::chrono::steady_clock::now() - startTime_ >=
            std::chrono::milliseconds(config::kSilverEntryWaitingLineTimeoutMs))
        {
            output.completed = true;
            output.status = makeStatus(
                "silver_entry_line_timeout",
                "Linha NEAR ausente por 2,5 s: entrando no resgate sem alinhamento");
            return output;
        }
        if (!alignmentLineVisible(vision))
        {
            nearCenteredFrames_ = 0;
            output.status = makeStatus(
                "silver_entry_waiting_line",
                "Aguardando a linha preta para alinhar a entrada");
            return output;
        }
        phase_ = Phase::CenteringLine;
        nearCenteredFrames_ = 0;
        startTime_ = std::chrono::steady_clock::now();
    }

    if (visionConfirmed_ && phase_ == Phase::CenteringLine)
    {
        if (!alignmentLineVisible(vision))
        {
            phase_ = Phase::WaitingForLine;
            startTime_ = std::chrono::steady_clock::now();
            nearCenteredFrames_ = 0;
            output.status = makeStatus(
                "silver_entry_waiting_line",
                "Aguardando a linha preta na parte inferior da imagem");
            return output;
        }
        const auto alignmentElapsedMs =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - startTime_)
                .count();
        if (alignmentElapsedMs > config::kSilverEntryAlignmentTimeoutMs)
        {
            phase_ = Phase::AlignmentTimeout;
            output.status = makeStatus(
                "silver_entry_alignment_timeout",
                "Alinhamento da faixa preta expirou: robô parado");
            return output;
        }
        const double nearPosition = vision.lineNearFinePosition;
        if (std::abs(nearPosition) <= config::kSilverEntryNearCenterTolerance)
        {
            ++nearCenteredFrames_;
            if (nearCenteredFrames_ >= config::kSilverEntryNearStableFrames)
            {
                output.completed = true;
                output.status = makeStatus(
                    "silver_entry_completed",
                    "Faixa cinza confirmada e linha preta inferior alinhada",
                    100.0);
                return output;
            }
            output.status = makeStatus(
                "silver_entry_aligning_line",
                "Linha preta inferior centralizada: confirmando");
            return output;
        }

        nearCenteredFrames_ = 0;
        const double turnSign = nearPosition > 0.0 ? 1.0 : -1.0;
        output.leftPower = turnSign * config::kSilverEntryNearCenteringPower;
        output.rightPower = -output.leftPower;
        output.status = makeStatus(
            "silver_entry_aligning_line",
            "Alinhando a entrada pela linha preta inferior");
        return output;
    }

    if (visionConfirmed_ && phase_ == Phase::AlignmentTimeout)
    {
        output.status = makeStatus(
            "silver_entry_alignment_timeout",
            "Alinhamento da faixa preta expirou: robô parado");
        return output;
    }

    const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - startTime_)
                               .count();

    if (!encodersReady(telemetry))
    {
        if (elapsedMs >= config::kSilverEntryCandidateAdvanceTimeoutMs)
        {
            reset();
            output.hasControl = false;
            return output;
        }
        output.status = makeStatus(
            "silver_entry_encoder_lost",
            "Avanço cinza pausado: encoders sem dados recentes");
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

    if (elapsedMs >= config::kSilverEntryCandidateAdvanceTimeoutMs)
    {
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
