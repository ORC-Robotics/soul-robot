#pragma once

namespace obstacle_config
{
// Calibração dos encoders usada pelos deslocamentos do desvio.
// Atualize estes dois valores ao levar o módulo para outro robô.
constexpr double kEncoderCalibrationCounts = 3600.0;
constexpr double kEncoderCalibrationDistanceCm = 18.7;
constexpr double kEncoderCountsPerCentimeter =
    kEncoderCalibrationCounts / kEncoderCalibrationDistanceCm;

// Distância frontal, em centímetros, que confirma a presença de um obstáculo.
// O valor mantém margem sobre a medição de 6,4 cm observada no teste físico.
constexpr double kDetectionDistanceCm = 8.0;
constexpr int kDetectionConfirmationSamples = 2;

// Histerese usada para não executar duas vezes o desvio do mesmo obstáculo.
constexpr double kRearmDistanceCm = 15.0;
constexpr int kRearmConfirmationSamples = 3;

// Ângulos, em graus, executados na ordem definida pela máquina de estados.
constexpr double kFirstRightTurnDegrees = 45.0;
constexpr double kFirstLeftTurnDegrees = 45.0;
constexpr double kSecondLeftTurnDegrees = 90.0;
constexpr double kFinalRightTurnDegrees = 90.0;
constexpr double kTurnToleranceDegrees = 4.0;

// Distâncias, em centímetros, calibradas no percurso físico atual.
constexpr double kFirstForwardDistanceCm = 25.0;
constexpr double kSecondForwardDistanceCm = 30.0;
constexpr double kThirdForwardDistanceCm = 21.5;
constexpr double kReverseDistanceCm = 5.0;

// Potências normalizadas dos deslocamentos para frente e em ré.
constexpr double kForwardPower = 0.75;
constexpr double kReversePower = 0.75;

// Pausa entre etapas e limites que interrompem falhas sem controlar movimento.
constexpr int kStageSettleMs = 250;
constexpr int kEncoderFreshnessMs = 300;
constexpr int kDistanceSafetyTimeoutMs = 12000;

// Horizonte usado para antecipar as contagens percorridas durante a frenagem.
constexpr double kBrakePredictionSeconds = 0.14;

static_assert(kEncoderCountsPerCentimeter > 0.0,
              "A calibração dos encoders deve ser positiva.");
static_assert(kDetectionDistanceCm > 0.0 &&
                  kRearmDistanceCm > kDetectionDistanceCm &&
                  kDetectionConfirmationSamples > 0 &&
                  kRearmConfirmationSamples > 0,
              "A detecção deve possuir limites e histerese válidos.");
static_assert(kFirstRightTurnDegrees > 0.0 &&
                  kFirstLeftTurnDegrees > 0.0 &&
                  kSecondLeftTurnDegrees > 0.0 &&
                  kSecondLeftTurnDegrees <= 180.0 &&
                  kFinalRightTurnDegrees > 0.0 &&
                  kFinalRightTurnDegrees <= 180.0 &&
                  kTurnToleranceDegrees > 0.0,
              "Os ângulos do desvio devem permanecer válidos.");
static_assert(kFirstForwardDistanceCm > 0.0 &&
                  kSecondForwardDistanceCm > 0.0 &&
                  kThirdForwardDistanceCm > 0.0 &&
                  kReverseDistanceCm > 0.0 &&
                  kForwardPower > 0.0 && kForwardPower <= 1.0 &&
                  kReversePower > 0.0 && kReversePower <= 1.0 &&
                  kStageSettleMs >= 0 && kEncoderFreshnessMs > 0 &&
                  kDistanceSafetyTimeoutMs > 0 &&
                  kBrakePredictionSeconds >= 0.0,
              "Os deslocamentos do desvio devem permanecer seguros.");
}
