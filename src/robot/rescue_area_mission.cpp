#include "obr/rescue_area_mission.h"

#include "obr/config.h"

#include <algorithm>
#include <cmath>
#include <string>

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

bool ballAlignmentFailed(const AutonomousStatus& status)
{
    return status.phase == "ball_alignment_target_lost_timeout" ||
           status.phase == "ball_alignment_camera_stale_timeout" ||
           status.phase == "ball_alignment_motion_timeout" ||
           status.phase == "ball_alignment_esp32_not_ready";
}

AutonomousStatus makeCollectionStatus(
    const std::string& phase,
    const std::string& action,
    double leftDistanceCm,
    double rightDistanceCm,
    double progressPercent)
{
    AutonomousStatus status = makeStatus(phase, action, progressPercent);
    status.targetDistanceCm = config::kVictimCollectionAdvanceDistanceCm;
    status.leftDistanceCm = leftDistanceCm;
    status.rightDistanceCm = rightDistanceCm;
    status.averageDistanceCm = (leftDistanceCm + rightDistanceCm) * 0.5;
    return status;
}

bool collectionEncodersReady(const Esp32TelemetrySnapshot& telemetry)
{
    return telemetry.sensorFresh && telemetry.lastSensorAgeMs >= 0 &&
           telemetry.lastSensorAgeMs <=
               config::kDriveDistanceEncoderFreshnessMs &&
           std::isfinite(telemetry.leftEncoderRate) &&
           std::isfinite(telemetry.rightEncoderRate);
}

bool collectionEncodersStopped(const Esp32TelemetrySnapshot& telemetry)
{
    return collectionEncodersReady(telemetry) &&
           std::abs(telemetry.leftEncoderRate) <=
               config::kBallAlignmentStationaryRateCountsPerSecond &&
           std::abs(telemetry.rightEncoderRate) <=
               config::kBallAlignmentStationaryRateCountsPerSecond;
}

struct CollectionDriveCommand
{
    double left = config::kVictimCollectionAdvancePower;
    double right = config::kVictimCollectionAdvancePower;
};

CollectionDriveCommand collectionDriveCommand(
    double leftDistanceCm,
    double rightDistanceCm)
{
    const double differenceCm = leftDistanceCm - rightDistanceCm;
    const double magnitudeCm = std::abs(differenceCm);
    if (magnitudeCm <= config::kDriveDistanceBalanceDeadbandCm)
    {
        return {};
    }

    const double correction = std::clamp(
        (magnitudeCm - config::kDriveDistanceBalanceDeadbandCm) *
            config::kDriveDistanceBalanceGainPerCm,
        0.0,
        config::kVictimCollectionMaximumBalanceCorrection);
    CollectionDriveCommand command;
    command.left = config::kVictimCollectionAdvancePower -
                   std::copysign(correction, differenceCm);
    command.right = config::kVictimCollectionAdvancePower +
                    std::copysign(correction, differenceCm);
    return command;
}

bool hasLockedVictim(
    const ForwardBallSnapshot& ball,
    std::uint64_t expectedTargetSequence)
{
    // O C++ só recebe a vítima depois da confirmação temporal feita pelo
    // BallTracker. Assim, uma caixa isolada do YOLO nunca interrompe a busca.
    return ball.sourceFresh && ball.detected && ball.targetLocked &&
           ball.targetSequence == expectedTargetSequence &&
           (ball.type == "silver_ball" || ball.type == "black_ball") &&
           std::isfinite(ball.txDegrees);
}
}

RescueAreaOutput RescueAreaMission::update(
    const ForwardBallSnapshot& forwardBallSnapshot,
    const Esp32TelemetrySnapshot& esp32Telemetry,
    std::uint64_t autonomousRunSequence,
    bool rescueExitConfirmed,
    bool finishAfterVictim,
    std::chrono::steady_clock::time_point now)
{
    RescueAreaOutput output;
    if (phase_ == Phase::Failed)
    {
        output.failed = true;
        output.status = failureStatus_;
        return output;
    }

    if (!esp32Telemetry.readyForOperation())
    {
        reset();
        output.failed = true;
        output.status = makeStatus(
            "rescue_esp32_not_ready",
            "Resgate interrompido: ESP32 não está pronta");
        return output;
    }

    if (phase_ == Phase::Searching)
    {
        if (!forwardBallSnapshot.sourceFresh)
        {
            // A câmera pode levar alguns ciclos para publicar o primeiro frame
            // depois que a missão abre o gate do detector. Neste intervalo, o
            // robô fica parado em vez de girar sem visão ou falhar cedo demais.
            searchPhase_ = SearchPhase::WaitingForFrame;
            if (std::isfinite(forwardBallSnapshot.timestamp))
            {
                searchFrameBeforeMotionTimestamp_ =
                    forwardBallSnapshot.timestamp;
            }
            output.status = makeStatus(
                "rescue_waiting_camera",
                "Parado: aguardando leitura atual da câmera frontal");
            return output;
        }

        if (hasLockedVictim(forwardBallSnapshot, autonomousRunSequence))
        {
            ballAlignmentMission_.reset();
            phase_ = Phase::Aligning;
            output.status = makeStatus(
                "rescue_victim_acquired",
                std::string("Vítima ") +
                    (forwardBallSnapshot.type == "silver_ball" ? "prata" : "preta") +
                    " confirmada; parando antes do alinhamento",
                10.0);
            return output;
        }

        if (forwardBallSnapshot.candidateVisible)
        {
            // A candidata ainda não pode comandar alinhamento, mas permanecer
            // parado evita deslocá-la entre os frames exigidos pelo target lock.
            searchPhase_ = SearchPhase::WaitingForFrame;
            searchFrameBeforeMotionTimestamp_ = forwardBallSnapshot.timestamp;
            output.status = makeStatus(
                "rescue_confirming_victim",
                "Motores parados: confirmando a candidata em frames consecutivos");
            return output;
        }

        if (searchPhase_ == SearchPhase::Pivoting)
        {
            if (now - searchPhaseStartedAt_ < std::chrono::milliseconds(
                                                  config::kRescueSearchPulseMs))
            {
                output.leftPower = config::kRescueSearchTurnPower;
                output.rightPower = -config::kRescueSearchTurnPower;
                output.status = makeStatus(
                    "rescue_search_pivot",
                    "Micro-pivô para procurar uma vítima confirmada");
                return output;
            }

            // A referência é registrada no fim do movimento. Somente uma
            // inferência concluída depois deste instante libera outro pulso.
            searchFrameBeforeMotionTimestamp_ = forwardBallSnapshot.timestamp;
            searchPhase_ = SearchPhase::Settling;
            searchPhaseStartedAt_ = now;
            output.status = makeStatus(
                "rescue_search_settling",
                "Motores parados: estabilizando a imagem após o micro-pivô");
            return output;
        }

        if (searchPhase_ == SearchPhase::Settling)
        {
            if (now - searchPhaseStartedAt_ < std::chrono::milliseconds(
                                                  config::kRescueSearchSettlingMs))
            {
                output.status = makeStatus(
                    "rescue_search_settling",
                    "Motores parados: estabilizando a imagem após o micro-pivô");
                return output;
            }
            searchPhase_ = SearchPhase::WaitingForFrame;
        }

        if (searchPhase_ == SearchPhase::WaitingForFrame)
        {
            if (forwardBallSnapshot.timestamp <=
                searchFrameBeforeMotionTimestamp_)
            {
                output.status = makeStatus(
                    "rescue_search_waiting_frame",
                    "Motores parados: aguardando um frame novo da CAM1");
                return output;
            }
            searchPhase_ = SearchPhase::Ready;
        }

        if (searchPhase_ == SearchPhase::Ready)
        {
            // O primeiro ciclo do pulso já aplica a potência. O próximo ciclo
            // controla o tempo configurado sem usar encoder ou alvo da IMU.
            searchPhase_ = SearchPhase::Pivoting;
            searchPhaseStartedAt_ = now;
            output.leftPower = config::kRescueSearchTurnPower;
            output.rightPower = -config::kRescueSearchTurnPower;
            output.status = makeStatus(
                "rescue_search_pivot",
                "Micro-pivô para procurar uma vítima confirmada");
            return output;
        }
    }

    if (phase_ == Phase::Aligning)
    {
        const BallAlignmentOutput alignment = ballAlignmentMission_.update(
            forwardBallSnapshot,
            esp32Telemetry,
            autonomousRunSequence,
            now);
        output.leftPower = alignment.leftPower;
        output.rightPower = alignment.rightPower;
        output.status = alignment.status;
        if (!alignment.finished)
        {
            return output;
        }
        if (alignment.status.phase == "ball_alignment_target_lost_timeout")
        {
            // A perda do alvo durante o giro não deve encerrar o resgate. O
            // robô volta à busca e exige novamente um alvo temporalmente travado.
            ballAlignmentMission_.reset();
            phase_ = Phase::Searching;
            searchPhase_ = SearchPhase::WaitingForFrame;
            searchFrameBeforeMotionTimestamp_ = forwardBallSnapshot.timestamp;
            output.status = makeStatus(
                "rescue_reacquiring_victim",
                "Alvo perdido: retomando a busca visual com os motores parados");
            return output;
        }
        if (ballAlignmentFailed(alignment.status) ||
            alignment.status.phase != "ball_reached")
        {
            phase_ = Phase::Failed;
            failureStatus_ = alignment.status;
            output.failed = true;
            return output;
        }
        phase_ = Phase::PreparingCollectionAdvance;
        collectionPhaseStartedAt_ = now;
        output.status = makeStatus(
            "victim_collection_preparing",
            "Vítima próxima: preparando o avanço final pelos encoders",
            0.0);
        return output;
    }

    if (phase_ == Phase::PreparingCollectionAdvance)
    {
        if (!collectionEncodersStopped(esp32Telemetry))
        {
            if (now - collectionPhaseStartedAt_ >=
                std::chrono::milliseconds(
                    config::kVictimCollectionPreparationTimeoutMs))
            {
                phase_ = Phase::Failed;
                failureStatus_ = makeCollectionStatus(
                    "victim_collection_preparation_timeout",
                    "Avanço final cancelado: encoders indisponíveis ou "
                    "rodas ainda em movimento",
                    0.0, 0.0, 0.0);
                output.failed = true;
                output.status = failureStatus_;
                return output;
            }
            output.status = makeCollectionStatus(
                "victim_collection_preparing",
                "Parado: aguardando encoders e fim da inércia anterior",
                0.0, 0.0, 0.0);
            return output;
        }

        collectionStartLeftCount_ = esp32Telemetry.leftEncoderCount;
        collectionStartRightCount_ = esp32Telemetry.rightEncoderCount;
        collectionLastUptimeMs_ = esp32Telemetry.esp32UptimeMs;
        collectionLastProgressCounts_ = 0.0;
        collectionDifferenceSamples_ = 0;
        collectionPhaseStartedAt_ = now;
        collectionLastProgressAt_ = now;
        phase_ = Phase::AdvancingForCollection;
    }

    if (phase_ == Phase::AdvancingForCollection)
    {
        const double leftCounts = std::abs(static_cast<double>(
            esp32Telemetry.leftEncoderCount - collectionStartLeftCount_));
        const double rightCounts = std::abs(static_cast<double>(
            esp32Telemetry.rightEncoderCount - collectionStartRightCount_));
        const double leftDistanceCm =
            leftCounts / config::kEncoderCountsPerCentimeter;
        const double rightDistanceCm =
            rightCounts / config::kEncoderCountsPerCentimeter;
        const double minimumCounts = std::min(leftCounts, rightCounts);
        const double minimumDistanceCm =
            std::min(leftDistanceCm, rightDistanceCm);
        const double progressPercent = std::clamp(
            minimumDistanceCm /
                config::kVictimCollectionAdvanceDistanceCm * 100.0,
            0.0, 100.0);

        const auto failAdvance = [&](const std::string& phase,
                                     const std::string& action) {
            phase_ = Phase::Failed;
            failureStatus_ = makeCollectionStatus(
                phase, action, leftDistanceCm, rightDistanceCm,
                progressPercent);
            output.failed = true;
            output.status = failureStatus_;
        };

        if (now - collectionPhaseStartedAt_ >=
            std::chrono::milliseconds(config::kVictimCollectionTimeoutMs))
        {
            failAdvance(
                "victim_collection_timeout",
                "Avanço final interrompido pelo tempo limite");
            return output;
        }
        if (!collectionEncodersReady(esp32Telemetry))
        {
            failAdvance(
                "victim_collection_encoder_lost",
                "Avanço final interrompido: encoders sem dados recentes");
            return output;
        }

        const bool newEncoderSample =
            esp32Telemetry.esp32UptimeMs != collectionLastUptimeMs_;
        if (newEncoderSample)
        {
            collectionLastUptimeMs_ = esp32Telemetry.esp32UptimeMs;
            if (std::abs(leftDistanceCm - rightDistanceCm) >
                config::kVictimCollectionMaximumSideDifferenceCm)
            {
                ++collectionDifferenceSamples_;
            }
            else
            {
                collectionDifferenceSamples_ = 0;
            }
        }
        if (collectionDifferenceSamples_ >=
            config::kVictimCollectionDifferenceConfirmationSamples)
        {
            failAdvance(
                "victim_collection_encoder_mismatch",
                "Avanço final interrompido: diferença excessiva entre rodas");
            return output;
        }

        if (minimumCounts >= collectionLastProgressCounts_ +
                                 config::kDriveDistanceMinimumProgressCounts)
        {
            collectionLastProgressCounts_ = minimumCounts;
            collectionLastProgressAt_ = now;
        }
        if (now - collectionLastProgressAt_ >=
            std::chrono::milliseconds(
                config::kVictimCollectionStallTimeoutMs))
        {
            failAdvance(
                "victim_collection_stall",
                "Avanço final interrompido: uma roda não avançou");
            return output;
        }

        // Neste avanço curto, os dois lados precisam medir toda a distância.
        // Não usar previsão evita que a inércia anterior encerre a coleta cedo.
        if (minimumDistanceCm >= config::kVictimCollectionAdvanceDistanceCm)
        {
            phase_ = Phase::SettlingAfterCollectionAdvance;
            collectionPhaseStartedAt_ = now;
            output.status = makeCollectionStatus(
                "victim_collection_settling",
                "PWM zerado: estabilizando após o avanço final",
                leftDistanceCm, rightDistanceCm, progressPercent);
            return output;
        }

        const CollectionDriveCommand command = collectionDriveCommand(
            leftDistanceCm, rightDistanceCm);
        output.leftPower = command.left;
        output.rightPower = command.right;
        output.status = makeCollectionStatus(
            "victim_collection_advancing",
            "Avançando a distância fixa para garantir a coleta",
            leftDistanceCm, rightDistanceCm, progressPercent);
        return output;
    }

    if (phase_ == Phase::SettlingAfterCollectionAdvance)
    {
        const double leftDistanceCm = std::abs(static_cast<double>(
            esp32Telemetry.leftEncoderCount - collectionStartLeftCount_)) /
            config::kEncoderCountsPerCentimeter;
        const double rightDistanceCm = std::abs(static_cast<double>(
            esp32Telemetry.rightEncoderCount - collectionStartRightCount_)) /
            config::kEncoderCountsPerCentimeter;
        if (now - collectionPhaseStartedAt_ <
            std::chrono::milliseconds(config::kVictimCollectionSettleMs))
        {
            output.status = makeCollectionStatus(
                "victim_collection_settling",
                "Motores parados: aguardando o robô estabilizar",
                leftDistanceCm, rightDistanceCm,
                100.0);
            return output;
        }
        phase_ = Phase::VictimReached;
    }

    if (finishAfterVictim || rescueExitConfirmed)
    {
        output.completed = true;
        output.status = makeStatus(
            rescueExitConfirmed ? "rescue_exit_confirmed" : "ball_reached",
            rescueExitConfirmed
                ? "Saída da área de resgate confirmada"
                : "Bola alcançada; teste isolado concluído",
            100.0);
        return output;
    }

    // A parada é intencional: o percurso de procura da saída ainda não possui
    // uma estratégia física validada e não deve ser substituído por movimento cego.
    output.status = makeStatus(
        "rescue_waiting_exit_program",
        "Vítima alcançada: aguardando a estratégia validada de saída");
    return output;
}

void RescueAreaMission::reset()
{
    ballAlignmentMission_.reset();
    phase_ = Phase::Searching;
    searchPhase_ = SearchPhase::Ready;
    searchPhaseStartedAt_ = {};
    collectionPhaseStartedAt_ = {};
    collectionLastProgressAt_ = {};
    searchFrameBeforeMotionTimestamp_ = 0.0;
    collectionLastProgressCounts_ = 0.0;
    collectionStartLeftCount_ = 0;
    collectionStartRightCount_ = 0;
    collectionLastUptimeMs_ = 0;
    collectionDifferenceSamples_ = 0;
    failureStatus_ = {};
}
