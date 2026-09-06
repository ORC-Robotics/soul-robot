#pragma once

#include "obr/config.h"

namespace obstacle_config
{
// Mantém os nomes internos do módulo, mas a fonte de verdade fica no config.h
// central do robô. Assim, missão de distância e desvio compartilham a mesma
// calibração dos encoders e todos os limites de segurança ficam em um só lugar.
constexpr double kEncoderCountsPerCentimeter = config::kEncoderCountsPerCentimeter;
constexpr double kDetectionDistanceCm = config::kObstacleDetectionDistanceCm;
constexpr int kDetectionConfirmationSamples =
    config::kObstacleDetectionConfirmationSamples;
constexpr double kRearmDistanceCm = config::kObstacleRearmDistanceCm;
constexpr int kRearmConfirmationSamples =
    config::kObstacleRearmConfirmationSamples;
constexpr double kFirstRightTurnDegrees = config::kObstacleFirstRightTurnDegrees;
constexpr double kFirstLeftTurnDegrees = config::kObstacleFirstLeftTurnDegrees;
constexpr double kSecondLeftTurnDegrees = config::kObstacleSecondLeftTurnDegrees;
constexpr double kFinalRightTurnDegrees = config::kObstacleFinalRightTurnDegrees;
constexpr double kTurnToleranceDegrees = config::kObstacleTurnToleranceDegrees;
constexpr double kFirstForwardDistanceCm =
    config::kObstacleFirstForwardDistanceCm;
constexpr double kSecondForwardDistanceCm =
    config::kObstacleSecondForwardDistanceCm;
constexpr double kThirdForwardDistanceCm =
    config::kObstacleThirdForwardDistanceCm;
constexpr double kReverseDistanceCm = config::kObstacleReverseDistanceCm;
constexpr double kForwardPower = config::kObstacleForwardPower;
constexpr double kReversePower = config::kObstacleReversePower;
constexpr int kStageSettleMs = config::kObstacleStageSettleMs;
constexpr int kEncoderFreshnessMs = config::kObstacleEncoderFreshnessMs;
constexpr int kDistanceSafetyTimeoutMs =
    config::kObstacleDistanceSafetyTimeoutMs;
constexpr double kBrakePredictionSeconds =
    config::kObstacleBrakePredictionSeconds;
}
