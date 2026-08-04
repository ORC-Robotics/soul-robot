#pragma once

#include <Arduino.h>

namespace robot_config
{
// GPIO que gera o pulso de disparo do sensor ultrassônico frontal.
constexpr uint8_t kFrontUltrasonicTriggerPin = 32;

// GPIO que recebe o ECHO do ultrassônico frontal.
// Se o sensor trabalhar em 5 V, um divisor deve limitar este sinal a 3,3 V.
constexpr uint8_t kFrontUltrasonicEchoPin = 33;

// Botão de partida ligado entre o GPIO e o GND. O pull-up interno mantém
// a entrada em nível alto quando o botão não está pressionado.
constexpr uint8_t kStartButtonPin = 27;

// Barramento I2C compartilhado pelo MPU6050, PCA9685 e, futuramente, SSD1306.
constexpr uint8_t kI2cSclPin = 14;
constexpr uint8_t kI2cSdaPin = 13;
constexpr uint32_t kI2cFrequencyHz = 400000;

// Entradas IN1 e IN2 do DRV8833 que comandam os dois motores esquerdos.
constexpr uint8_t kLeftMotorIn1Pin = 5;
constexpr uint8_t kLeftMotorIn2Pin = 18;

// Entradas IN1 e IN2 do DRV8833 que comandam os dois motores direitos.
constexpr uint8_t kRightMotorIn1Pin = 16;
constexpr uint8_t kRightMotorIn2Pin = 17;

// Canais A e B dos encoders. A contagem usa as quatro bordas do quadrature.
constexpr uint8_t kLeftEncoderAPin = 19;
constexpr uint8_t kLeftEncoderBPin = 21;
constexpr uint8_t kRightEncoderAPin = 22;
constexpr uint8_t kRightEncoderBPin = 23;

// UART0 ligada à Raspberry Pi. Esses pinos também são usados para gravar
// a ESP32; desconecte ou mantenha a Raspberry silenciosa durante a gravação.
constexpr uint8_t kRaspberryTxPin = 1;
constexpr uint8_t kRaspberryRxPin = 3;
constexpr uint32_t kRaspberryBaudRate = 115200;

// Entrada ADC1 que mede a bateria através do divisor resistivo de 47 kΩ e 10 kΩ.
// O GPIO36 é somente entrada e continua disponível enquanto o Wi-Fi está ativo.
constexpr uint8_t kBatteryAdcPin = 36;
constexpr float kBatteryUpperResistorOhms = 47000.0f;
constexpr float kBatteryLowerResistorOhms = 10000.0f;
constexpr float kBatteryDividerRatio =
    (kBatteryUpperResistorOhms + kBatteryLowerResistorOhms) /
    kBatteryLowerResistorOhms;

// Endereços I2C padrão. O MPU6050 também é procurado em 0x69 quando AD0 está alto.
constexpr uint8_t kPca9685Address = 0x40;
constexpr uint8_t kMpu6050PrimaryAddress = 0x68;
constexpr uint8_t kMpu6050SecondaryAddress = 0x69;

// Frequência inicial segura do PCA9685 para servos. Todos os 16 canais
// permanecem desligados até que uma função futura defina seus movimentos.
constexpr float kPca9685FrequencyHz = 50.0f;

// PWM do DRV8833. A frequência de 20 kHz fica acima da faixa audível comum.
constexpr uint32_t kMotorPwmFrequencyHz = 20000;
constexpr uint8_t kMotorPwmResolutionBits = 10;
constexpr uint16_t kMotorPwmMaxDuty = (1U << kMotorPwmResolutionBits) - 1U;
constexpr uint8_t kLeftMotorIn1Channel = 0;
constexpr uint8_t kLeftMotorIn2Channel = 1;
constexpr uint8_t kRightMotorIn1Channel = 2;
constexpr uint8_t kRightMotorIn2Channel = 3;

// Inverta somente o lado que girar ao contrário no teste com as rodas suspensas.
constexpr bool kLeftMotorInverted = false;
constexpr bool kRightMotorInverted = false;

// Potência máxima disponível para os motores. O valor 1,0 corresponde a 100%
// do ciclo de trabalho do PWM e só deve ser testado com as rodas suspensas.
constexpr float kMaximumMotorPower = 1.00f;
constexpr float kMotorDeadband = 0.04f;

// Tempo máximo sem comando de movimento antes de zerar os quatro motores.
constexpr uint32_t kMotorCommandTimeoutMs = 500;

// Períodos de leitura e envio. Esses valores evitam sobrecarregar o I2C,
// o navegador e a UART durante o loop principal.
constexpr uint32_t kImuReadIntervalMs = 20;
constexpr uint32_t kTelemetryIntervalMs = 100;
constexpr uint32_t kBatteryReadIntervalMs = 250;
constexpr uint32_t kEncoderRateIntervalMs = 250;
constexpr uint32_t kUltrasonicIntervalMs = 100;

// O ECHO é descartado depois deste tempo para que uma falha do sensor não
// bloqueie o controle dos motores nem o servidor web.
constexpr uint32_t kUltrasonicTimeoutUs = 25000;

// Rede local criada pela ESP32 para configuração e teste de bancada.
// Troque a senha antes de usar o robô em local público.
constexpr const char* kWifiSsid = "OBR2026K-ESP32";
constexpr const char* kWifiPassword = "obr2026k-painel";
}
