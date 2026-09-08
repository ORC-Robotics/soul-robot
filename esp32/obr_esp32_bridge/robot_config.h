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

// Barramento I2C compartilhado pelo MPU6050, PCA9685 e SSD1306.
constexpr uint8_t kI2cSclPin = 14;
constexpr uint8_t kI2cSdaPin = 13;
constexpr uint32_t kI2cFrequencyHz = 400000;

// GPIO ligado ao nSLEEP do DRV8833. LOW desliga as pontes H e tem prioridade
// sobre IN1/IN2; HIGH libera o driver depois que todas as entradas estão em zero.
constexpr uint8_t kMotorSleepPin = 26;

// Entradas IN1 e IN2 do DRV8833 que comandam os dois motores esquerdos.
constexpr uint8_t kLeftMotorIn1Pin = 5;
constexpr uint8_t kLeftMotorIn2Pin = 18;

// Entradas IN1 e IN2 do DRV8833 que comandam os dois motores direitos.
constexpr uint8_t kRightMotorIn1Pin = 16;
constexpr uint8_t kRightMotorIn2Pin = 17;

// Canais A e B dos encoders conforme o lado físico validado na PCB.
// A contagem usa as quatro bordas do quadrature.
constexpr uint8_t kLeftEncoderAPin = 22;
constexpr uint8_t kLeftEncoderBPin = 23;
constexpr uint8_t kRightEncoderAPin = 19;
constexpr uint8_t kRightEncoderBPin = 21;

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

// Endereços I2C padrão. MPU6050 e OLED também são procurados nos endereços
// alternativos usados pelos respectivos módulos.
constexpr uint8_t kPca9685Address = 0x40;
constexpr uint8_t kMpu6050PrimaryAddress = 0x68;
constexpr uint8_t kMpu6050SecondaryAddress = 0x69;
constexpr uint8_t kOledPrimaryAddress = 0x3C;
constexpr uint8_t kOledSecondaryAddress = 0x3D;
constexpr int16_t kOledWidth = 128;
constexpr int16_t kOledHeight = 64;

// Limites do texto remoto recebido da Raspberry Pi. O protocolo usa hexadecimal
// para que vírgulas e outros caracteres não quebrem os campos da UART.
constexpr size_t kRemoteOledTitleMaxLength = 12;
constexpr size_t kRemoteOledLineMaxLength = 20;

// A mensagem remota sempre expira. Assim, uma queda da Raspberry não deixa a
// OLED presa em informação antiga; depois do prazo, bateria e ângulos retornam.
constexpr uint32_t kRemoteOledMinimumDurationMs = 500;
constexpr uint32_t kRemoteOledMaximumDurationMs = 30000;

// O firmware principal exige um heartbeat periódico da Raspberry antes de
// liberar a tela normal. Se o serviço cair, a animação de inicialização retorna.
constexpr uint32_t kRaspberrySystemReadyTimeoutMs = 3000;

// Frequência inicial segura do PCA9685 para servos. Todos os 16 canais
// permanecem desligados até que um comando válido defina seus movimentos.
constexpr float kPca9685FrequencyHz = 50.0f;

// Canais do PCA9685 reservados para os três mecanismos do robô.
// Se a fiação mudar, ajuste somente estes valores e mantenha cada servo em um
// canal exclusivo.
constexpr uint8_t kArmServoChannel = 0;
constexpr uint8_t kWristServoChannel = 1;
constexpr uint8_t kGripperServoChannel = 2;

// Faixa angular exposta à Raspberry e ao dashboard, em graus.
// Os comandos fora dessa faixa são rejeitados antes de chegar ao PCA9685.
constexpr float kServoMinimumAngleDegrees = 0.0f;
constexpr float kServoMaximumAngleDegrees = 180.0f;

// Valores de recuperação confirmados em bancada para os três servos. Perfis
// válidos salvos pela calibração na NVS ainda podem substituir estes padrões.
constexpr uint16_t kArmServoMinimumPulseUs = 500;
constexpr uint16_t kArmServoMaximumPulseUs = 2500;
constexpr uint16_t kWristServoMinimumPulseUs = 500;
constexpr uint16_t kWristServoMaximumPulseUs = 2500;
constexpr uint16_t kGripperServoMinimumPulseUs = 500;
constexpr uint16_t kGripperServoMaximumPulseUs = 2500;

// Limites absolutos aceitos no ajuste de bancada. A interface pode explorar
// essa faixa em passos pequenos, mas o firmware nunca gera pulsos além dela.
constexpr uint16_t kServoCalibrationAbsoluteMinimumPulseUs = 500;
constexpr uint16_t kServoCalibrationAbsoluteMaximumPulseUs = 2500;

// Evita salvar dois extremos quase iguais, o que deixaria a conversão de
// ângulo muito sensível a poucos microssegundos de diferença.
constexpr uint16_t kServoCalibrationMinimumSpanUs = 200;

// O pulso bruto precisa ser renovado pelo dashboard. Se a aba fechar ou a
// conexão cair, o PCA9685 remove o sinal mesmo que a Raspberry continue ligada.
constexpr uint32_t kServoCalibrationCommandTimeoutMs = 2000;

// Namespace e versão usados pela Preferences, que faz parte do core ESP32.
// Uma gravação normal preserva esses dados enquanto "Erase All Flash" não for usado.
constexpr const char* kServoPreferencesNamespace = "obr-servos";
constexpr uint16_t kServoCalibrationStorageVersion = 1;

// Permite inverter individualmente o sentido lógico de cada mecanismo sem
// trocar a convenção de 0° a 180° usada pelo dashboard e pelas missões.
constexpr bool kArmServoInverted = false;
constexpr bool kWristServoInverted = false;
constexpr bool kGripperServoInverted = false;

// Erros nesta configuração devem interromper a compilação antes de gerar um
// firmware capaz de comandar o canal errado ou uma faixa de pulso invertida.
static_assert(kArmServoChannel < 16 && kWristServoChannel < 16 &&
                  kGripperServoChannel < 16,
              "Servo channels must be inside the PCA9685 range");
static_assert(kArmServoChannel != kWristServoChannel &&
                  kArmServoChannel != kGripperServoChannel &&
                  kWristServoChannel != kGripperServoChannel,
              "Every servo must use an exclusive PCA9685 channel");
static_assert(kArmServoMinimumPulseUs < kArmServoMaximumPulseUs &&
                  kWristServoMinimumPulseUs < kWristServoMaximumPulseUs &&
                  kGripperServoMinimumPulseUs < kGripperServoMaximumPulseUs,
              "Servo minimum pulses must be lower than maximum pulses");
static_assert(kServoCalibrationAbsoluteMinimumPulseUs <=
                   kArmServoMinimumPulseUs &&
                  kArmServoMaximumPulseUs <=
                      kServoCalibrationAbsoluteMaximumPulseUs &&
                  kServoCalibrationAbsoluteMinimumPulseUs <=
                      kWristServoMinimumPulseUs &&
                  kWristServoMaximumPulseUs <=
                      kServoCalibrationAbsoluteMaximumPulseUs &&
                  kServoCalibrationAbsoluteMinimumPulseUs <=
                      kGripperServoMinimumPulseUs &&
                  kGripperServoMaximumPulseUs <=
                      kServoCalibrationAbsoluteMaximumPulseUs,
              "Default servo pulses must stay within calibration limits");
static_assert(kServoCalibrationMinimumSpanUs > 0 &&
                  kServoCalibrationMinimumSpanUs <
                      kServoCalibrationAbsoluteMaximumPulseUs -
                          kServoCalibrationAbsoluteMinimumPulseUs &&
                  kServoCalibrationCommandTimeoutMs > 0,
              "Servo calibration safety limits must be valid");

// PWM do DRV8833. A frequência de 20 kHz fica acima da faixa audível comum.
constexpr uint32_t kMotorPwmFrequencyHz = 20000;
constexpr uint8_t kMotorPwmResolutionBits = 10;
constexpr uint16_t kMotorPwmMaxDuty = (1U << kMotorPwmResolutionBits) - 1U;
constexpr uint8_t kLeftMotorIn1Channel = 0;
constexpr uint8_t kLeftMotorIn2Channel = 1;
constexpr uint8_t kRightMotorIn1Channel = 2;
constexpr uint8_t kRightMotorIn2Channel = 3;

// Tempo de estabilização do DRV8833 depois que nSLEEP volta para HIGH.
constexpr uint32_t kMotorDriverWakeDelayUs = 1000;

// Inverta somente o lado que girar ao contrário no teste com as rodas suspensas.
constexpr bool kLeftMotorInverted = false;
constexpr bool kRightMotorInverted = false;

// Extremo da faixa normalizada do protocolo. O valor 1,0 corresponde diretamente
// a 100% do duty; não há outro limite ou perfil aplicado depois desta validação.
constexpr float kMaximumMotorPower = 1.00f;

// Tempo máximo sem comando de movimento antes de zerar os quatro motores.
constexpr uint32_t kMotorCommandTimeoutMs = 500;

// Tempo, em milissegundos, que o botão Start deve permanecer pressionado para
// iniciar a calibração. A espera longa evita resets acidentais durante a prova.
constexpr uint32_t kSensorCalibrationHoldMs = 5000;

// Duração mínima, em milissegundos, para reconhecer um toque curto no Start.
// Pulsos menores são ignorados como ruído ou contato mecânico do botão.
constexpr uint32_t kStartButtonMinimumPressMs = 80;

// Períodos de leitura e envio. Esses valores evitam sobrecarregar o I2C,
// o navegador e a UART durante o loop principal.
constexpr uint32_t kImuReadIntervalMs = 20;
constexpr uint32_t kTelemetryIntervalMs = 100;
constexpr uint32_t kBatteryReadIntervalMs = 250;
// A taxa dos encoders é atualizada junto com cada telemetria. A Raspberry usa
// uma amostra nova por ajuste do sincronismo e filtra o ruído dessa janela.
constexpr uint32_t kEncoderRateIntervalMs = kTelemetryIntervalMs;
constexpr uint32_t kUltrasonicIntervalMs = 100;
constexpr uint32_t kOledRefreshIntervalMs = 100;

// O alerta grande pulsa lentamente sem apagar por completo. O período de 1,6 s
// deixa cada palavra legível, enquanto o contraste mínimo evita um flash seco.
constexpr uint32_t kOledAlertPulsePeriodMs = 1600;
constexpr uint8_t kOledAlertMinimumContrast = 64;
constexpr uint8_t kOledAlertMaximumContrast = 255;
static_assert(kOledAlertPulsePeriodMs > 0,
              "OLED alert pulse period must be greater than zero");
static_assert(kOledAlertMinimumContrast < kOledAlertMaximumContrast,
              "OLED alert contrast limits must be ordered");

// Tempo, em milissegundos, que o resultado da calibração permanece na OLED.
// Depois desse período, a tela volta automaticamente à bateria e aos ângulos.
constexpr uint32_t kOledCalibrationResultDurationMs = 900;

// O ECHO é descartado depois deste tempo para que uma falha do sensor não
// bloqueie o controle dos motores nem o servidor web.
constexpr uint32_t kUltrasonicTimeoutUs = 25000;

// Constante de tempo, em segundos, do filtro complementar da inclinação.
// Valores maiores suavizam mais a leitura, mas tornam a resposta mais lenta.
constexpr float kRampFilterTimeConstantSeconds = 0.50f;

// Permite corrigir o sinal caso o MPU6050 esteja montado com o eixo X invertido.
constexpr float kRampAngleSign = 1.0f;

// Suaviza a velocidade de giro usada para integrar o yaw.
constexpr float kGyroLowPassAlpha = 0.75f;

// Velocidades menores que este limite, em graus por segundo, são tratadas
// como ruído quando o robô está parado.
constexpr float kGyroDeadbandDegreesPerSecond = 0.25f;

// Faixa visual da bateria de níquel de 12 V usada na barra do OLED.
constexpr float kBatteryGaugeMinimumVoltage = 10.50f;
constexpr float kBatteryGaugeMaximumVoltage = 14.00f;

// Rede local criada somente pelo firmware de bancada da ESP32.
constexpr const char* kWifiSsid = "OBR2026K-ESP32";
constexpr const char* kWifiPassword = "obr2026k-painel";
}
