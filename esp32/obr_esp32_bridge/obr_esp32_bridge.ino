#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#ifndef OBR_ESP32_RASPBERRY_MODE
#include <WebServer.h>
#include <WiFi.h>
#endif
#include <Wire.h>
#include <driver/gpio.h>

#ifndef OBR_ESP32_RASPBERRY_MODE
#include "dashboard_page.h"
#endif
#include "robot_config.h"

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

using namespace robot_config;

enum class ControlSource
{
  None,
  Dashboard,
  Raspberry
};

enum class TractionFaultSide : uint8_t
{
  None = 0,
  Left = 1,
  Right = 2
};

void forceMotorPinsLowImmediately()
{
  // Mantém o driver dormindo antes de configurar qualquer entrada de motor.
  // O pull-down físico da PCB garante o mesmo estado antes deste código rodar.
  gpio_set_level(static_cast<gpio_num_t>(kMotorSleepPin), 0);
  // INPUT_OUTPUT permite dirigir nSLEEP e ler no dashboard o nível presente
  // no pad. Em OUTPUT puro, gpio_get_level() pode informar LOW incorretamente.
  gpio_set_direction(static_cast<gpio_num_t>(kMotorSleepPin), GPIO_MODE_INPUT_OUTPUT);

  // Grava LOW antes de habilitar os GPIOs como saída. Assim, nenhuma entrada
  // do DRV8833 recebe um pulso alto causado pela troca de direção do pino.
  gpio_set_level(static_cast<gpio_num_t>(kLeftMotorIn1Pin), 0);
  gpio_set_level(static_cast<gpio_num_t>(kLeftMotorIn2Pin), 0);
  gpio_set_level(static_cast<gpio_num_t>(kRightMotorIn1Pin), 0);
  gpio_set_level(static_cast<gpio_num_t>(kRightMotorIn2Pin), 0);
  gpio_set_direction(static_cast<gpio_num_t>(kLeftMotorIn1Pin), GPIO_MODE_OUTPUT);
  gpio_set_direction(static_cast<gpio_num_t>(kLeftMotorIn2Pin), GPIO_MODE_OUTPUT);
  gpio_set_direction(static_cast<gpio_num_t>(kRightMotorIn1Pin), GPIO_MODE_OUTPUT);
  gpio_set_direction(static_cast<gpio_num_t>(kRightMotorIn2Pin), GPIO_MODE_OUTPUT);
}

// Este inicializador executa antes do setup() e antecipa a parada dos motores.
// Ele existe porque a PCB mantém o DRV8833 habilitado durante o boot da ESP32.
class EarlyMotorSafetyInitializer
{
public:
  EarlyMotorSafetyInitializer()
  {
    forceMotorPinsLowImmediately();
  }
};

EarlyMotorSafetyInitializer earlyMotorSafetyInitializer;

#ifndef OBR_ESP32_RASPBERRY_MODE
// O servidor existe somente no firmware de bancada. O firmware principal
// exclui Wi-Fi e HTTP durante a compilação e obedece apenas à Raspberry.
WebServer server(80);
#endif
Adafruit_MPU6050 mpu;
Adafruit_PWMServoDriver pca9685(kPca9685Address, Wire);
Adafruit_SSD1306 oled(kOledWidth, kOledHeight, &Wire, -1);

struct SensorState
{
  float batteryVoltage = 0.0f;
  uint32_t batteryAdcMillivolts = 0;
  float ultrasonicDistanceCm = -1.0f;
  bool ultrasonicValid = false;
  float accelX = 0.0f;
  float accelY = 0.0f;
  float accelZ = 0.0f;
  float gyroX = 0.0f;
  float gyroY = 0.0f;
  float gyroZ = 0.0f;
  float yawZ = 0.0f;
  float rampAngleDegrees = 0.0f;
  float imuTemperatureCelsius = 0.0f;
  float leftEncoderRate = 0.0f;
  float rightEncoderRate = 0.0f;
};

SensorState sensors;
bool mpuReady = false;
bool pca9685Ready = false;
bool oledReady = false;
bool motorDriverAwake = false;
uint8_t mpuAddress = 0;
uint8_t oledAddress = 0;
bool startButtonPressed = false;
bool calibrationActive = false;
bool calibrationStopLatched = false;
bool dashboardArmed = false;
bool emergencyStopActive = false;
bool tractionFaultActive = false;
TractionFaultSide tractionFaultSide = TractionFaultSide::None;
float currentLeftPower = 0.0f;
float currentRightPower = 0.0f;
float gyroYBias = 0.0f;
float gyroZBias = 0.0f;
float filteredGyroZ = 0.0f;
bool gyroFilterInitialized = false;
bool rampAngleInitialized = false;
ControlSource controlSource = ControlSource::None;

uint32_t lastMotorCommandMs = 0;
uint32_t lastImuReadMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastBatteryReadMs = 0;
uint32_t lastEncoderRateMs = 0;
uint32_t lastUltrasonicTriggerMs = 0;
uint32_t lastButtonChangeMs = 0;
uint32_t startButtonPressedSinceMs = 0;
uint32_t lastMpuIntegrationUs = 0;
uint32_t lastOledRefreshMs = 0;
bool startButtonLongPressHandled = false;

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t leftEncoderCount = 0;
volatile int32_t rightEncoderCount = 0;
volatile uint8_t previousLeftEncoderState = 0;
volatile uint8_t previousRightEncoderState = 0;
int32_t lastLeftEncoderRateCount = 0;
int32_t lastRightEncoderRateCount = 0;
bool tractionMonitorActive = false;
uint32_t tractionMonitorStartMs = 0;
int32_t tractionMonitorLeftStartCount = 0;
int32_t tractionMonitorRightStartCount = 0;
int8_t tractionMonitorLeftDirection = 0;
int8_t tractionMonitorRightDirection = 0;

portMUX_TYPE ultrasonicMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool ultrasonicWaitingForEcho = false;
volatile bool ultrasonicSampleAvailable = false;
volatile uint32_t ultrasonicTriggerUs = 0;
volatile uint32_t ultrasonicEchoStartUs = 0;
volatile uint32_t ultrasonicEchoDurationUs = 0;

constexpr size_t kCommandBufferSize = 128;
char commandBuffer[kCommandBufferSize] = {};
size_t commandLength = 0;

// Tabela de transições do encoder quadrature. Transições inválidas
// contam zero e reduzem erros causados por ruído ou bordas perdidas.
constexpr int8_t kQuadratureTransitions[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0};

void ARDUINO_ISR_ATTR updateLeftEncoder()
{
  const uint8_t currentState =
      (digitalRead(kLeftEncoderAPin) << 1) | digitalRead(kLeftEncoderBPin);
  portENTER_CRITICAL_ISR(&encoderMux);
  leftEncoderCount += kQuadratureTransitions[(previousLeftEncoderState << 2) | currentState];
  previousLeftEncoderState = currentState;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void ARDUINO_ISR_ATTR updateRightEncoder()
{
  const uint8_t currentState =
      (digitalRead(kRightEncoderAPin) << 1) | digitalRead(kRightEncoderBPin);
  portENTER_CRITICAL_ISR(&encoderMux);
  rightEncoderCount += kQuadratureTransitions[(previousRightEncoderState << 2) | currentState];
  previousRightEncoderState = currentState;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void ARDUINO_ISR_ATTR captureUltrasonicEcho()
{
  const uint32_t nowUs = micros();
  const bool echoHigh = digitalRead(kFrontUltrasonicEchoPin) == HIGH;

  portENTER_CRITICAL_ISR(&ultrasonicMux);
  if (ultrasonicWaitingForEcho && echoHigh)
  {
    ultrasonicEchoStartUs = nowUs;
  }
  else if (ultrasonicWaitingForEcho && ultrasonicEchoStartUs != 0)
  {
    ultrasonicEchoDurationUs = nowUs - ultrasonicEchoStartUs;
    ultrasonicSampleAvailable = true;
    ultrasonicWaitingForEcho = false;
    ultrasonicEchoStartUs = 0;
  }
  portEXIT_CRITICAL_ISR(&ultrasonicMux);
}

void attachMotorPwm(uint8_t pin, uint8_t channel)
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, kMotorPwmFrequencyHz, kMotorPwmResolutionBits);
  ledcWrite(pin, 0);
#else
  ledcSetup(channel, kMotorPwmFrequencyHz, kMotorPwmResolutionBits);
  ledcAttachPin(pin, channel);
  ledcWrite(channel, 0);
#endif
}

void writeMotorPwm(uint8_t pin, uint8_t channel, uint16_t duty)
{
  duty = constrain(duty, 0, kMotorPwmMaxDuty);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

float safeMotorPower(float command)
{
  if (!isfinite(command))
  {
    return 0.0f;
  }

  command = constrain(command, -kMaximumMotorPower, kMaximumMotorPower);
  const float magnitude = fabs(command);
  if (magnitude < kMotorCommandDeadband)
  {
    return 0.0f;
  }

  // Elimina a faixa morta física observada nos motores. O comando preserva
  // toda a resolução do slider, mas a saída útil passa a variar de 60% a 100%.
  const float usefulPower = kMinimumMovingMotorPower +
                            magnitude * (kMaximumMotorPower - kMinimumMovingMotorPower);
  return command > 0.0f ? usefulPower : -usefulPower;
}

void zeroMotorPwmOutputs()
{
  writeMotorPwm(kLeftMotorIn1Pin, kLeftMotorIn1Channel, 0);
  writeMotorPwm(kLeftMotorIn2Pin, kLeftMotorIn2Channel, 0);
  writeMotorPwm(kRightMotorIn1Pin, kRightMotorIn1Channel, 0);
  writeMotorPwm(kRightMotorIn2Pin, kRightMotorIn2Channel, 0);
  currentLeftPower = 0.0f;
  currentRightPower = 0.0f;
}

void stopMotorOutputs()
{
  // A parada mantém o DRV8833 habilitado e zera somente IN1/IN2. Isso evita
  // repetir o tempo de inicialização do driver a cada novo movimento.
  zeroMotorPwmOutputs();
}

void resetTractionMonitor()
{
  portENTER_CRITICAL(&encoderMux);
  tractionMonitorLeftStartCount = leftEncoderCount;
  tractionMonitorRightStartCount = rightEncoderCount;
  portEXIT_CRITICAL(&encoderMux);
  tractionMonitorStartMs = millis();
  tractionMonitorActive = false;
  tractionMonitorLeftDirection = 0;
  tractionMonitorRightDirection = 0;
}

void clearTractionFault()
{
  // O rearme exige um comando humano explícito. Apenas parar os motores não
  // apaga a falha, pois comandos antigos poderiam voltar a arrastar um lado.
  tractionFaultActive = false;
  tractionFaultSide = TractionFaultSide::None;
  resetTractionMonitor();
}

void latchTractionFault(TractionFaultSide side)
{
  tractionFaultActive = true;
  tractionFaultSide = side;
  dashboardArmed = false;
  controlSource = ControlSource::None;
  stopMotorOutputs();
  resetTractionMonitor();

  Serial.print("TRACTION_FAULT,");
  Serial.println(side == TractionFaultSide::Left ? "LEFT" : "RIGHT");
}

void keepMotorDriverEnabled()
{
  // Reafirma HIGH sem alternar nSLEEP e sem repetir o tempo de wake. Esta
  // escrita ocorre em todo comando para garantir a habilitação do driver
  // mesmo quando somente um dos lados recebe potência.
  gpio_set_level(static_cast<gpio_num_t>(kMotorSleepPin), 1);
  motorDriverAwake = true;
}

void enableMotorDriver()
{
  // O driver é habilitado uma única vez, depois que os quatro PWMs já estão
  // em zero. O tempo de estabilização ocorre apenas durante o setup().
  zeroMotorPwmOutputs();
  keepMotorDriverEnabled();
  delayMicroseconds(kMotorDriverWakeDelayUs);
}

void setMotorSide(uint8_t in1Pin, uint8_t in1Channel, uint8_t in2Pin,
                  uint8_t in2Channel, float power, bool inverted)
{
  const float directedPower = inverted ? -power : power;
  const uint16_t duty = static_cast<uint16_t>(fabs(directedPower) * kMotorPwmMaxDuty);

  // Zera o sentido oposto antes de aplicar PWM para nunca comandar uma
  // mudança de direção com as duas entradas ativas por engano.
  if (directedPower > 0.0f)
  {
    writeMotorPwm(in2Pin, in2Channel, 0);
    writeMotorPwm(in1Pin, in1Channel, duty);
  }
  else if (directedPower < 0.0f)
  {
    writeMotorPwm(in1Pin, in1Channel, 0);
    writeMotorPwm(in2Pin, in2Channel, duty);
  }
  else
  {
    writeMotorPwm(in1Pin, in1Channel, 0);
    writeMotorPwm(in2Pin, in2Channel, 0);
  }
}

void applyMotorCommand(float leftPower, float rightPower, ControlSource source)
{
  if (emergencyStopActive || tractionFaultActive || calibrationActive || calibrationStopLatched ||
      (source == ControlSource::Dashboard && !dashboardArmed))
  {
    stopMotorOutputs();
    return;
  }

  keepMotorDriverEnabled();

  float safeLeftPower = safeMotorPower(leftPower);
  float safeRightPower = safeMotorPower(rightPower);

  // A elétrica agrupa dois motores em cada lado do robô. Nunca permite que
  // apenas um lado se mova, pois as rodas de borracha travariam e fariam o robô
  // vibrar. Um comando unilateral é convertido em giro com os lados opostos.
  const bool leftSideStopped = safeLeftPower == 0.0f;
  const bool rightSideStopped = safeRightPower == 0.0f;
  if (leftSideStopped != rightSideStopped)
  {
    if (leftSideStopped)
    {
      safeLeftPower = -safeRightPower;
    }
    else
    {
      safeRightPower = -safeLeftPower;
    }
  }

  if (safeLeftPower == 0.0f && safeRightPower == 0.0f)
  {
    stopMotorOutputs();
    resetTractionMonitor();
    controlSource = source;
    lastMotorCommandMs = millis();
    return;
  }

  setMotorSide(kLeftMotorIn1Pin, kLeftMotorIn1Channel, kLeftMotorIn2Pin,
               kLeftMotorIn2Channel, safeLeftPower, kLeftMotorInverted);
  setMotorSide(kRightMotorIn1Pin, kRightMotorIn1Channel, kRightMotorIn2Pin,
               kRightMotorIn2Channel, safeRightPower, kRightMotorInverted);
  currentLeftPower = safeLeftPower;
  currentRightPower = safeRightPower;
  controlSource = source;
  lastMotorCommandMs = millis();
}

void setupMotors()
{
  // Mantém nSLEEP em LOW até que o periférico PWM esteja configurado e com
  // as quatro saídas zeradas. Depois disso, o driver permanece habilitado.
  forceMotorPinsLowImmediately();
  attachMotorPwm(kLeftMotorIn1Pin, kLeftMotorIn1Channel);
  attachMotorPwm(kLeftMotorIn2Pin, kLeftMotorIn2Channel);
  attachMotorPwm(kRightMotorIn1Pin, kRightMotorIn1Channel);
  attachMotorPwm(kRightMotorIn2Pin, kRightMotorIn2Channel);
  enableMotorDriver();
}

bool i2cDeviceResponds(uint8_t address)
{
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void setupOled()
{
  if (i2cDeviceResponds(kOledPrimaryAddress))
  {
    oledAddress = kOledPrimaryAddress;
  }
  else if (i2cDeviceResponds(kOledSecondaryAddress))
  {
    oledAddress = kOledSecondaryAddress;
  }

  if (oledAddress == 0)
  {
    return;
  }

  // O Wire já usa SDA13/SCL14. periphBegin=false impede a biblioteca do OLED
  // de reiniciar o I2C nos pinos padrão da placa e desconectar MPU/PCA9685.
  oledReady = oled.begin(SSD1306_SWITCHCAPVCC, oledAddress, true, false);
  if (!oledReady)
  {
    oledAddress = 0;
    return;
  }

  oled.clearDisplay();
  oled.setTextWrap(false);
  oled.setTextColor(SSD1306_WHITE);
  oled.drawRoundRect(0, 0, kOledWidth, kOledHeight, 4, SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(25, 16);
  oled.print(F("OBR 2026"));
  oled.setCursor(19, 38);
  oled.print(F("INICIALIZANDO"));
  oled.display();
}

void drawCalibrationProgress(int completedSamples, int totalSamples)
{
  if (!oledReady)
  {
    return;
  }

  // Os pontos ao redor do centro formam uma animação circular. A posição
  // preenchida avança junto com as amostras reais coletadas pelo MPU6050.
  constexpr int8_t kOrbitX[] = {0, 6, 8, 6, 0, -6, -8, -6};
  constexpr int8_t kOrbitY[] = {-8, -6, 0, 6, 8, 6, 0, -6};
  constexpr int kOrbitPointCount = sizeof(kOrbitX) / sizeof(kOrbitX[0]);
  const int animationFrame =
      (completedSamples * kOrbitPointCount / totalSamples) % kOrbitPointCount;
  const int progressWidth = constrain(completedSamples * 100 / totalSamples, 0, 100);

  oled.clearDisplay();
  oled.drawRoundRect(0, 0, kOledWidth, kOledHeight, 4, SSD1306_WHITE);
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(34, 4);
  oled.print(F("CALIBRANDO"));

  for (int point = 0; point < kOrbitPointCount; ++point)
  {
    const int16_t pointX = 64 + kOrbitX[point];
    const int16_t pointY = 27 + kOrbitY[point];
    if (point == animationFrame)
    {
      oled.fillCircle(pointX, pointY, 2, SSD1306_WHITE);
    }
    else
    {
      oled.drawPixel(pointX, pointY, SSD1306_WHITE);
    }
  }

  oled.setCursor(40, 39);
  oled.print(F("NAO MOVA"));
  oled.drawRoundRect(12, 52, 104, 8, 3, SSD1306_WHITE);
  if (progressWidth > 0)
  {
    oled.fillRoundRect(14, 54, progressWidth, 4, 1, SSD1306_WHITE);
  }
  oled.display();
}

void drawCalibrationResult(bool succeeded)
{
  if (!oledReady)
  {
    return;
  }

  oled.clearDisplay();
  oled.drawRoundRect(0, 0, kOledWidth, kOledHeight, 4, SSD1306_WHITE);
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(34, 4);
  oled.print(F("CALIBRACAO"));
  oled.drawCircle(23, 36, 12, SSD1306_WHITE);

  if (succeeded)
  {
    // O símbolo de confirmação permanece grande e legível mesmo à distância.
    oled.drawLine(16, 36, 21, 42, SSD1306_WHITE);
    oled.drawLine(21, 42, 31, 30, SSD1306_WHITE);
    oled.setTextSize(2);
    oled.setCursor(43, 24);
    oled.print(F("PRONTO"));
    oled.setTextSize(1);
    oled.setCursor(47, 45);
    oled.print(F("SENSORES OK"));
  }
  else
  {
    oled.drawLine(17, 30, 29, 42, SSD1306_WHITE);
    oled.drawLine(29, 30, 17, 42, SSD1306_WHITE);
    oled.setTextSize(2);
    oled.setCursor(43, 24);
    oled.print(F("FALHOU"));
    oled.setTextSize(1);
    oled.setCursor(47, 45);
    oled.print(F("VERIFIQUE MPU"));
  }
  oled.display();
}

void calibrateGyroscopeBias()
{
  // A calibração ocorre no boot ou após uma solicitação de reset. O robô deve
  // permanecer parado para reduzir a deriva nas leituras de giro e rampa.
  constexpr int kCalibrationSamples = 300;
  constexpr int kOledProgressUpdateSamples = 25;
  float gyroYSum = 0.0f;
  float gyroZSum = 0.0f;
  for (int sample = 0; sample < kCalibrationSamples; ++sample)
  {
    if (sample % kOledProgressUpdateSamples == 0)
    {
      drawCalibrationProgress(sample, kCalibrationSamples);
    }

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temperature;
    mpu.getEvent(&accel, &gyro, &temperature);
    gyroYSum += gyro.gyro.y;
    gyroZSum += gyro.gyro.z;
    delay(3);
  }
  gyroYBias = gyroYSum / kCalibrationSamples;
  gyroZBias = gyroZSum / kCalibrationSamples;
  drawCalibrationProgress(kCalibrationSamples, kCalibrationSamples);
}

void resetEncoderData()
{
  portENTER_CRITICAL(&encoderMux);
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  portEXIT_CRITICAL(&encoderMux);
  lastLeftEncoderRateCount = 0;
  lastRightEncoderRateCount = 0;
  sensors.leftEncoderRate = 0.0f;
  sensors.rightEncoderRate = 0.0f;
  lastEncoderRateMs = millis();
  resetTractionMonitor();
}

void resetSensorMeasurements()
{
  // Mantém a tensão da bateria disponível, mas limpa medidas de navegação que
  // dependem de posição, movimento ou de uma referência inicial do MPU6050.
  sensors.ultrasonicDistanceCm = -1.0f;
  sensors.ultrasonicValid = false;
  sensors.accelX = 0.0f;
  sensors.accelY = 0.0f;
  sensors.accelZ = 0.0f;
  sensors.gyroX = 0.0f;
  sensors.gyroY = 0.0f;
  sensors.gyroZ = 0.0f;
  sensors.yawZ = 0.0f;
  sensors.rampAngleDegrees = 0.0f;
  sensors.imuTemperatureCelsius = 0.0f;
  filteredGyroZ = 0.0f;
  gyroFilterInitialized = false;
  rampAngleInitialized = false;
  lastMpuIntegrationUs = micros();

  portENTER_CRITICAL(&ultrasonicMux);
  ultrasonicWaitingForEcho = false;
  ultrasonicSampleAvailable = false;
  ultrasonicEchoStartUs = 0;
  ultrasonicEchoDurationUs = 0;
  portEXIT_CRITICAL(&ultrasonicMux);

  resetEncoderData();
}

void runSensorCalibration()
{
  // A calibração sempre começa com os motores parados e desabilita o controle
  // de bancada. O E-Stop existente é preservado e nunca é liberado aqui.
  dashboardArmed = false;
  stopMotorOutputs();
  controlSource = ControlSource::None;
  calibrationActive = true;
  calibrationStopLatched = true;
  Serial.println("CALIBRATION,START");

  resetSensorMeasurements();
  if (mpuReady)
  {
    calibrateGyroscopeBias();
  }

  drawCalibrationResult(mpuReady);
  if (oledReady)
  {
    // Mantém o resultado visível sem liberar os motores durante a mensagem.
    delay(kOledCalibrationResultDurationMs);
  }

  lastMotorCommandMs = millis();
  calibrationActive = false;
  Serial.println(mpuReady ? "CALIBRATION,DONE" : "CALIBRATION,FAILED");
}

void setupI2cDevices()
{
  Wire.begin(kI2cSdaPin, kI2cSclPin, kI2cFrequencyHz);
  setupOled();

  if (mpu.begin(kMpu6050PrimaryAddress, &Wire))
  {
    mpuAddress = kMpu6050PrimaryAddress;
    mpuReady = true;
  }
  else if (mpu.begin(kMpu6050SecondaryAddress, &Wire))
  {
    mpuAddress = kMpu6050SecondaryAddress;
    mpuReady = true;
  }

  if (mpuReady)
  {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    calibrateGyroscopeBias();
  }

  drawCalibrationResult(mpuReady);
  if (oledReady)
  {
    delay(kOledCalibrationResultDurationMs);
  }

  pca9685Ready = i2cDeviceResponds(kPca9685Address) && pca9685.begin();
  if (pca9685Ready)
  {
    pca9685.setPWMFreq(kPca9685FrequencyHz);
    for (uint8_t channel = 0; channel < 16; ++channel)
    {
      // Nenhum atuador do PCA9685 foi definido ainda. Manter todos os canais
      // desligados evita movimento de servo inesperado durante a partida.
      pca9685.setPWM(channel, 0, 0);
    }
  }
}

void updateOledIfDue()
{
  const uint32_t nowMs = millis();
  if (!oledReady || nowMs - lastOledRefreshMs < kOledRefreshIntervalMs)
  {
    return;
  }
  lastOledRefreshMs = nowMs;

  char batteryText[10] = "--.--V";
  char yawText[8] = "  --";
  char rampText[8] = "  --";
  if (isfinite(sensors.batteryVoltage))
  {
    snprintf(batteryText, sizeof(batteryText), "%5.2fV", sensors.batteryVoltage);
  }
  if (mpuReady && isfinite(sensors.yawZ))
  {
    snprintf(yawText, sizeof(yawText), "%+4.0f", sensors.yawZ);
  }
  if (mpuReady && isfinite(sensors.rampAngleDegrees))
  {
    snprintf(rampText, sizeof(rampText), "%+4.0f", sensors.rampAngleDegrees);
  }

  const float batteryGauge = constrain(
      (sensors.batteryVoltage - kBatteryGaugeMinimumVoltage) /
          (kBatteryGaugeMaximumVoltage - kBatteryGaugeMinimumVoltage),
      0.0f, 1.0f);
  const int16_t batteryFillWidth = static_cast<int16_t>(batteryGauge * 67.0f);

  oled.clearDisplay();
  oled.drawRoundRect(0, 0, kOledWidth, kOledHeight, 4, SSD1306_WHITE);

  // Nos OLEDs bicolores, as primeiras 16 linhas são fisicamente amarelas.
  // Título e barra ficam nessa faixa para criar contraste sem cortar a tensão.
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(4, 4);
  oled.print(F("BATERIA"));

  oled.drawRoundRect(50, 3, 73, 9, 3, SSD1306_WHITE);
  if (batteryFillWidth > 0)
  {
    oled.fillRoundRect(53, 6, batteryFillWidth, 3, 1, SSD1306_WHITE);
  }

  // A tensão começa abaixo da linha 16 para permanecer completamente azul
  // nos displays SSD1306 que possuem duas cores definidas pelo próprio painel.
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(2);
  int16_t textX;
  int16_t textY;
  uint16_t textWidth;
  uint16_t textHeight;
  oled.getTextBounds(batteryText, 0, 0, &textX, &textY, &textWidth, &textHeight);
  oled.setCursor((kOledWidth - textWidth) / 2, 18);
  oled.print(batteryText);

  // A faixa inferior mantém giro e rampa disponíveis como informações
  // secundárias. Os sinais positivos e negativos facilitam testes de sentido.
  oled.drawLine(3, 39, 124, 39, SSD1306_WHITE);
  oled.drawLine(64, 42, 64, 60, SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(5, 48);
  oled.print(F("GIR"));
  oled.setCursor(25, 48);
  oled.print(yawText);
  oled.drawCircle(57, 48, 1, SSD1306_WHITE);
  oled.setCursor(68, 48);
  oled.print(F("RMP"));
  oled.setCursor(89, 48);
  oled.print(rampText);
  oled.drawCircle(121, 48, 1, SSD1306_WHITE);
  oled.display();
}

void setupEncoders()
{
  pinMode(kLeftEncoderAPin, INPUT_PULLUP);
  pinMode(kLeftEncoderBPin, INPUT_PULLUP);
  pinMode(kRightEncoderAPin, INPUT_PULLUP);
  pinMode(kRightEncoderBPin, INPUT_PULLUP);
  previousLeftEncoderState =
      (digitalRead(kLeftEncoderAPin) << 1) | digitalRead(kLeftEncoderBPin);
  previousRightEncoderState =
      (digitalRead(kRightEncoderAPin) << 1) | digitalRead(kRightEncoderBPin);
  attachInterrupt(digitalPinToInterrupt(kLeftEncoderAPin), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kLeftEncoderBPin), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kRightEncoderAPin), updateRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kRightEncoderBPin), updateRightEncoder, CHANGE);
}

void setupUltrasonic()
{
  pinMode(kFrontUltrasonicTriggerPin, OUTPUT);
  pinMode(kFrontUltrasonicEchoPin, INPUT);
  digitalWrite(kFrontUltrasonicTriggerPin, LOW);
  attachInterrupt(digitalPinToInterrupt(kFrontUltrasonicEchoPin), captureUltrasonicEcho, CHANGE);
}

void triggerUltrasonicIfDue()
{
  const uint32_t nowMs = millis();
  const uint32_t nowUs = micros();

  portENTER_CRITICAL(&ultrasonicMux);
  if (ultrasonicWaitingForEcho &&
      nowUs - ultrasonicTriggerUs > kUltrasonicTimeoutUs)
  {
    ultrasonicWaitingForEcho = false;
    ultrasonicEchoStartUs = 0;
    sensors.ultrasonicValid = false;
  }
  const bool canTrigger = !ultrasonicWaitingForEcho &&
                          nowMs - lastUltrasonicTriggerMs >= kUltrasonicIntervalMs;
  if (canTrigger)
  {
    ultrasonicWaitingForEcho = true;
    ultrasonicTriggerUs = nowUs;
    ultrasonicEchoStartUs = 0;
    lastUltrasonicTriggerMs = nowMs;
  }
  portEXIT_CRITICAL(&ultrasonicMux);

  if (canTrigger)
  {
    digitalWrite(kFrontUltrasonicTriggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(kFrontUltrasonicTriggerPin, LOW);
  }
}

void consumeUltrasonicSample()
{
  uint32_t durationUs = 0;
  portENTER_CRITICAL(&ultrasonicMux);
  if (ultrasonicSampleAvailable)
  {
    durationUs = ultrasonicEchoDurationUs;
    ultrasonicSampleAvailable = false;
  }
  portEXIT_CRITICAL(&ultrasonicMux);

  if (durationUs == 0)
  {
    return;
  }

  const float distanceCm = durationUs * 0.0343f / 2.0f;
  sensors.ultrasonicValid = distanceCm >= 2.0f && distanceCm <= 400.0f;
  sensors.ultrasonicDistanceCm = sensors.ultrasonicValid ? distanceCm : -1.0f;
}

void readBatteryIfDue()
{
  const uint32_t nowMs = millis();
  if (nowMs - lastBatteryReadMs < kBatteryReadIntervalMs)
  {
    return;
  }
  lastBatteryReadMs = nowMs;

  constexpr int kSampleCount = 16;
  uint32_t millivoltSum = 0;
  for (int sample = 0; sample < kSampleCount; ++sample)
  {
    millivoltSum += analogReadMilliVolts(kBatteryAdcPin);
  }
  sensors.batteryAdcMillivolts = millivoltSum / kSampleCount;
  sensors.batteryVoltage =
      (sensors.batteryAdcMillivolts / 1000.0f) * kBatteryDividerRatio;
}

void readImuIfDue()
{
  const uint32_t nowMs = millis();
  if (!mpuReady || nowMs - lastImuReadMs < kImuReadIntervalMs)
  {
    return;
  }
  lastImuReadMs = nowMs;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temperature;
  mpu.getEvent(&accel, &gyro, &temperature);

  const uint32_t nowUs = micros();
  const float deltaSeconds = lastMpuIntegrationUs == 0
                                 ? 0.0f
                                 : (nowUs - lastMpuIntegrationUs) / 1000000.0f;
  lastMpuIntegrationUs = nowUs;

  sensors.accelX = accel.acceleration.x;
  sensors.accelY = accel.acceleration.y;
  sensors.accelZ = accel.acceleration.z;
  sensors.gyroX = gyro.gyro.x * RAD_TO_DEG;
  sensors.gyroY = (gyro.gyro.y - gyroYBias) * RAD_TO_DEG;
  const float correctedGyroZ = (gyro.gyro.z - gyroZBias) * RAD_TO_DEG;

  if (!gyroFilterInitialized)
  {
    filteredGyroZ = correctedGyroZ;
    gyroFilterInitialized = true;
  }
  else
  {
    filteredGyroZ = kGyroLowPassAlpha * filteredGyroZ +
                    (1.0f - kGyroLowPassAlpha) * correctedGyroZ;
  }
  sensors.gyroZ = fabs(filteredGyroZ) < kGyroDeadbandDegreesPerSecond
                      ? 0.0f
                      : filteredGyroZ;
  sensors.imuTemperatureCelsius = temperature.temperature;

  const float accelRampAngle =
      atan2f(-sensors.accelX,
             sqrtf(sensors.accelY * sensors.accelY + sensors.accelZ * sensors.accelZ)) *
      RAD_TO_DEG * kRampAngleSign;
  const float rampGyroRate = sensors.gyroY * kRampAngleSign;
  if (!rampAngleInitialized || deltaSeconds <= 0.0f || deltaSeconds > 0.20f)
  {
    sensors.rampAngleDegrees = accelRampAngle;
    rampAngleInitialized = true;
  }
  else
  {
    const float filterAlpha =
        kRampFilterTimeConstantSeconds / (kRampFilterTimeConstantSeconds + deltaSeconds);
    sensors.rampAngleDegrees =
        filterAlpha * (sensors.rampAngleDegrees + rampGyroRate * deltaSeconds) +
        (1.0f - filterAlpha) * accelRampAngle;
  }

  sensors.yawZ += sensors.gyroZ * deltaSeconds;
  if (sensors.yawZ > 180.0f)
  {
    sensors.yawZ -= 360.0f;
  }
  else if (sensors.yawZ < -180.0f)
  {
    sensors.yawZ += 360.0f;
  }
}

void updateEncoderRatesIfDue()
{
  const uint32_t nowMs = millis();
  const uint32_t elapsedMs = nowMs - lastEncoderRateMs;
  if (elapsedMs < kEncoderRateIntervalMs)
  {
    return;
  }

  int32_t currentLeftCount;
  int32_t currentRightCount;
  portENTER_CRITICAL(&encoderMux);
  currentLeftCount = leftEncoderCount;
  currentRightCount = rightEncoderCount;
  portEXIT_CRITICAL(&encoderMux);

  sensors.leftEncoderRate =
      (currentLeftCount - lastLeftEncoderRateCount) * 1000.0f / elapsedMs;
  sensors.rightEncoderRate =
      (currentRightCount - lastRightEncoderRateCount) * 1000.0f / elapsedMs;
  lastLeftEncoderRateCount = currentLeftCount;
  lastRightEncoderRateCount = currentRightCount;
  lastEncoderRateMs = nowMs;
}

void enforceTractionSafety()
{
  const bool bothSidesCommanded = currentLeftPower != 0.0f && currentRightPower != 0.0f;
  if (tractionFaultActive || !bothSidesCommanded)
  {
    resetTractionMonitor();
    return;
  }

  const int8_t leftDirection = currentLeftPower > 0.0f ? 1 : -1;
  const int8_t rightDirection = currentRightPower > 0.0f ? 1 : -1;

  if (!tractionMonitorActive || leftDirection != tractionMonitorLeftDirection ||
      rightDirection != tractionMonitorRightDirection)
  {
    portENTER_CRITICAL(&encoderMux);
    tractionMonitorLeftStartCount = leftEncoderCount;
    tractionMonitorRightStartCount = rightEncoderCount;
    portEXIT_CRITICAL(&encoderMux);
    tractionMonitorStartMs = millis();
    tractionMonitorActive = true;
    tractionMonitorLeftDirection = leftDirection;
    tractionMonitorRightDirection = rightDirection;
    return;
  }

  const uint32_t elapsedMs = millis() - tractionMonitorStartMs;
  if (elapsedMs < kTractionMonitorWindowMs)
  {
    return;
  }

  int32_t currentLeftCount;
  int32_t currentRightCount;
  portENTER_CRITICAL(&encoderMux);
  currentLeftCount = leftEncoderCount;
  currentRightCount = rightEncoderCount;
  portEXIT_CRITICAL(&encoderMux);

  const int32_t leftDelta = abs(currentLeftCount - tractionMonitorLeftStartCount);
  const int32_t rightDelta = abs(currentRightCount - tractionMonitorRightStartCount);
  const int32_t leadingCounts = max(leftDelta, rightDelta);
  if (leadingCounts < kTractionMinimumLeadingCounts)
  {
    // Preserva a referência para acumular movimento lento. Reiniciar a janela
    // aqui poderia fazer uma falha escapar para sempre com poucos pulsos.
    return;
  }

  // Divide a contagem pela potência aplicada para permitir curvas normais, nas
  // quais os dois lados giram de propósito em velocidades diferentes.
  const float leftProgress = leftDelta / fabs(currentLeftPower);
  const float rightProgress = rightDelta / fabs(currentRightPower);
  const float leadingProgress = max(leftProgress, rightProgress);

  if (leftProgress < leadingProgress * kTractionMinimumProgressRatio)
  {
    latchTractionFault(TractionFaultSide::Left);
    return;
  }
  if (rightProgress < leadingProgress * kTractionMinimumProgressRatio)
  {
    latchTractionFault(TractionFaultSide::Right);
    return;
  }

  tractionMonitorLeftStartCount = currentLeftCount;
  tractionMonitorRightStartCount = currentRightCount;
  tractionMonitorStartMs = millis();
}

void updateStartButton()
{
  constexpr uint32_t kDebounceMs = 30;
  const uint32_t nowMs = millis();
  const bool rawPressed = digitalRead(kStartButtonPin) == LOW;
  if (rawPressed != startButtonPressed && nowMs - lastButtonChangeMs >= kDebounceMs)
  {
    startButtonPressed = rawPressed;
    lastButtonChangeMs = nowMs;
    if (startButtonPressed)
    {
      startButtonPressedSinceMs = nowMs;
      startButtonLongPressHandled = false;
    }
    else
    {
      const uint32_t pressDurationMs = nowMs - startButtonPressedSinceMs;
      if (!startButtonLongPressHandled && startButtonPressedSinceMs != 0 &&
          pressDurationMs >= kStartButtonMinimumPressMs)
      {
        // O evento só é emitido ao soltar para distinguir um toque curto da
        // pressão de 5 segundos reservada à calibração dos sensores.
        Serial.println("START_BUTTON,SHORT");
      }
      startButtonPressedSinceMs = 0;
      startButtonLongPressHandled = false;
    }
  }

  if (startButtonPressed && !startButtonLongPressHandled &&
      nowMs - startButtonPressedSinceMs >= kSensorCalibrationHoldMs)
  {
    startButtonLongPressHandled = true;
    runSensorCalibration();
  }
}

#ifndef OBR_ESP32_RASPBERRY_MODE
const char* controlSourceName()
{
  if (controlSource == ControlSource::Dashboard)
  {
    return "dashboard";
  }
  if (controlSource == ControlSource::Raspberry)
  {
    return "raspberry";
  }
  return "nenhuma";
}

void addJsonFloat(String& json, const char* name, float value, uint8_t digits = 3)
{
  json += ",\"";
  json += name;
  json += "\":";
  json += isfinite(value) ? String(value, static_cast<unsigned int>(digits)) : "0";
}

String telemetryJson()
{
  int32_t leftCount;
  int32_t rightCount;
  portENTER_CRITICAL(&encoderMux);
  leftCount = leftEncoderCount;
  rightCount = rightEncoderCount;
  portEXIT_CRITICAL(&encoderMux);

  String json;
  json.reserve(1100);
  json = '{';
  json += "\"uptimeMs\":" + String(millis());
  json += ",\"wifiClients\":" + String(WiFi.softAPgetStationNum());
  json += ",\"emergencyStop\":" + String(emergencyStopActive ? "true" : "false");
  json += ",\"tractionFaultActive\":" + String(tractionFaultActive ? "true" : "false");
  json += ",\"tractionFaultSide\":" + String(static_cast<uint8_t>(tractionFaultSide));
  json += ",\"dashboardArmed\":" + String(dashboardArmed ? "true" : "false");
  json += ",\"startButtonPressed\":" + String(startButtonPressed ? "true" : "false");
  json += ",\"calibrationActive\":" + String(calibrationActive ? "true" : "false");
  json += ",\"mpuReady\":" + String(mpuReady ? "true" : "false");
  json += ",\"mpuAddress\":" + String(mpuAddress);
  json += ",\"pca9685Ready\":" + String(pca9685Ready ? "true" : "false");
  json += ",\"oledReady\":" + String(oledReady ? "true" : "false");
  json += ",\"oledAddress\":" + String(oledAddress);
  json += ",\"motorDriverAwake\":" + String(motorDriverAwake ? "true" : "false");
  json += ",\"motorSleepPinHigh\":" +
          String(gpio_get_level(static_cast<gpio_num_t>(kMotorSleepPin)) ? "true" : "false");
  json += ",\"ultrasonicValid\":" + String(sensors.ultrasonicValid ? "true" : "false");
  json += ",\"controlSource\":\"" + String(controlSourceName()) + "\"";
  json += ",\"lastCommandAgeMs\":" + String(millis() - lastMotorCommandMs);
  json += ",\"leftEncoderCount\":" + String(leftCount);
  json += ",\"rightEncoderCount\":" + String(rightCount);
  addJsonFloat(json, "batteryVoltage", sensors.batteryVoltage);
  json += ",\"batteryAdcMillivolts\":" + String(sensors.batteryAdcMillivolts);
  addJsonFloat(json, "ultrasonicDistanceCm", sensors.ultrasonicDistanceCm);
  addJsonFloat(json, "leftMotorPower", currentLeftPower);
  addJsonFloat(json, "rightMotorPower", currentRightPower);
  addJsonFloat(json, "leftEncoderRate", sensors.leftEncoderRate);
  addJsonFloat(json, "rightEncoderRate", sensors.rightEncoderRate);
  addJsonFloat(json, "accelX", sensors.accelX);
  addJsonFloat(json, "accelY", sensors.accelY);
  addJsonFloat(json, "accelZ", sensors.accelZ);
  addJsonFloat(json, "gyroX", sensors.gyroX);
  addJsonFloat(json, "gyroY", sensors.gyroY);
  addJsonFloat(json, "gyroZ", sensors.gyroZ);
  addJsonFloat(json, "yawZ", sensors.yawZ);
  addJsonFloat(json, "rampAngleDegrees", sensors.rampAngleDegrees);
  addJsonFloat(json, "imuTemperatureCelsius", sensors.imuTemperatureCelsius);
  json += '}';
  return json;
}

void sendJsonResponse(int statusCode, const String& body)
{
  server.sendHeader("Cache-Control", "no-store");
  server.send(statusCode, "application/json", body);
}

bool readValidatedMotorArgument(const char* name, float& value)
{
  if (!server.hasArg(name))
  {
    return false;
  }
  const String text = server.arg(name);
  char* end = nullptr;
  value = strtof(text.c_str(), &end);
  return end != text.c_str() && *end == '\0' && isfinite(value) &&
         value >= -kMaximumMotorPower && value <= kMaximumMotorPower;
}

void setupDashboardRoutes()
{
  server.on("/", HTTP_GET, []()
            {
              // Impede que o celular continue mostrando uma versão antiga do
              // dashboard depois que um novo firmware é gravado na ESP32.
              server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
              server.sendHeader("Pragma", "no-cache");
              server.send_P(200, "text/html; charset=utf-8", DASHBOARD_HTML); });
  server.on("/api/telemetry", HTTP_GET, []()
            { sendJsonResponse(200, telemetryJson()); });
  server.on("/api/arm", HTTP_POST, []()
            {
              stopMotorOutputs();
              if (emergencyStopActive)
              {
                sendJsonResponse(409, "{\"error\":\"Libere o E-Stop antes de habilitar\"}");
                return;
              }
              clearTractionFault();
              calibrationStopLatched = false;
              dashboardArmed = true;
              controlSource = ControlSource::Dashboard;
              lastMotorCommandMs = millis();
              sendJsonResponse(200, "{\"ok\":true}"); });
  server.on("/api/disarm", HTTP_ANY, []()
            {
              dashboardArmed = false;
              stopMotorOutputs();
              sendJsonResponse(200, "{\"ok\":true}"); });
  server.on("/api/estop", HTTP_POST, []()
            {
              emergencyStopActive = true;
              dashboardArmed = false;
              stopMotorOutputs();
              sendJsonResponse(200, "{\"ok\":true}"); });
  server.on("/api/clear-estop", HTTP_POST, []()
            {
              stopMotorOutputs();
              dashboardArmed = false;
              emergencyStopActive = false;
              sendJsonResponse(200, "{\"ok\":true}"); });
  server.on("/api/motor", HTTP_POST, []()
            {
              float leftPower;
              float rightPower;
              if (!readValidatedMotorArgument("left", leftPower) ||
                  !readValidatedMotorArgument("right", rightPower))
              {
                stopMotorOutputs();
                sendJsonResponse(400, "{\"error\":\"Potencia fora do limite seguro\"}");
                return;
              }
              if (!dashboardArmed || emergencyStopActive)
              {
                stopMotorOutputs();
                sendJsonResponse(409, "{\"error\":\"Motores nao habilitados\"}");
                return;
              }
              applyMotorCommand(leftPower, rightPower, ControlSource::Dashboard);
              sendJsonResponse(200, "{\"ok\":true}"); });
  server.on("/api/reset-encoders", HTTP_POST, []()
            {
              resetEncoderData();
              sendJsonResponse(200, "{\"ok\":true}"); });
  server.on("/api/calibrate-sensors", HTTP_POST, []()
            {
              runSensorCalibration();
              sendJsonResponse(mpuReady ? 200 : 503,
                               mpuReady ? "{\"ok\":true}" : "{\"error\":\"MPU6050 indisponivel\"}"); });
  server.onNotFound([]()
                    { sendJsonResponse(404, "{\"error\":\"Endpoint nao encontrado\"}"); });
  server.begin();
}
#endif

void sendUartError(const char* message)
{
  Serial.print("ERR,");
  Serial.println(message);
}

void handleUartCommand(const char* line)
{
  if (strcmp(line, "PING") == 0)
  {
    Serial.println("PONG");
    return;
  }
  if (strcmp(line, "STOP") == 0)
  {
    stopMotorOutputs();
    controlSource = ControlSource::Raspberry;
    lastMotorCommandMs = millis();
    return;
  }
  if (strcmp(line, "ESTOP") == 0)
  {
    emergencyStopActive = true;
    dashboardArmed = false;
    stopMotorOutputs();
    return;
  }
  if (strcmp(line, "CLEAR_ESTOP") == 0)
  {
    stopMotorOutputs();
    emergencyStopActive = false;
    calibrationStopLatched = false;
    clearTractionFault();
    return;
  }
  if (strcmp(line, "RESET_ENCODERS") == 0)
  {
    resetEncoderData();
    return;
  }
  if (strcmp(line, "CALIBRATE_SENSORS") == 0)
  {
    runSensorCalibration();
    return;
  }

  float leftPower = 0.0f;
  float rightPower = 0.0f;
  int emergencyFlag = 0;
  if (sscanf(line, "MOTOR,%f,%f,%d", &leftPower, &rightPower, &emergencyFlag) == 3)
  {
    if (!isfinite(leftPower) || !isfinite(rightPower) ||
        leftPower < -1.0f || leftPower > 1.0f ||
        rightPower < -1.0f || rightPower > 1.0f)
    {
      stopMotorOutputs();
      sendUartError("invalid_motor_power");
      return;
    }
    if (emergencyFlag != 0)
    {
      emergencyStopActive = true;
      dashboardArmed = false;
      stopMotorOutputs();
      return;
    }
    applyMotorCommand(leftPower, rightPower, ControlSource::Raspberry);
    return;
  }
  sendUartError("bad_command");
}

void readUartCommands()
{
  while (Serial.available() > 0)
  {
    const char character = static_cast<char>(Serial.read());
    if (character == '\n')
    {
      commandBuffer[commandLength] = '\0';
      if (commandLength > 0)
      {
        handleUartCommand(commandBuffer);
      }
      commandLength = 0;
    }
    else if (character != '\r')
    {
      if (commandLength < kCommandBufferSize - 1)
      {
        commandBuffer[commandLength++] = character;
      }
      else
      {
        commandLength = 0;
        sendUartError("command_too_long");
      }
    }
  }
}

void sendUartTelemetryIfDue()
{
  const uint32_t nowMs = millis();
  if (nowMs - lastTelemetryMs < kTelemetryIntervalMs)
  {
    return;
  }
  lastTelemetryMs = nowMs;

  int32_t leftCount;
  int32_t rightCount;
  portENTER_CRITICAL(&encoderMux);
  leftCount = leftEncoderCount;
  rightCount = rightEncoderCount;
  portEXIT_CRITICAL(&encoderMux);

  // Os oito primeiros campos mantêm compatibilidade com o parser atual da
  // Raspberry. Os campos adicionais levam bateria, encoders, botão e PCA9685.
  Serial.print("SENSOR,");
  Serial.print(sensors.ultrasonicValid ? sensors.ultrasonicDistanceCm : -1.0f, 2);
  Serial.print(',');
  Serial.print(sensors.gyroZ, 2);
  Serial.print(',');
  Serial.print(sensors.yawZ, 2);
  Serial.print(',');
  Serial.print(sensors.accelX, 2);
  Serial.print(',');
  Serial.print(sensors.accelY, 2);
  Serial.print(',');
  Serial.print(sensors.accelZ, 2);
  Serial.print(',');
  Serial.print(mpuReady ? 1 : 0);
  Serial.print(',');
  Serial.print(sensors.batteryVoltage, 2);
  Serial.print(',');
  Serial.print(leftCount);
  Serial.print(',');
  Serial.print(rightCount);
  Serial.print(',');
  Serial.print(startButtonPressed ? 1 : 0);
  Serial.print(',');
  Serial.print(pca9685Ready ? 1 : 0);
  Serial.print(',');
  Serial.print(currentLeftPower, 2);
  Serial.print(',');
  Serial.print(currentRightPower, 2);
  Serial.print(',');
  Serial.print(sensors.leftEncoderRate, 1);
  Serial.print(',');
  Serial.print(sensors.rightEncoderRate, 1);
  Serial.print(',');
  Serial.print(sensors.rampAngleDegrees, 2);
  Serial.print(',');
  Serial.print(sensors.gyroX, 2);
  Serial.print(',');
  Serial.print(sensors.gyroY, 2);
  Serial.print(',');
  Serial.print(sensors.imuTemperatureCelsius, 2);
  Serial.print(',');
  Serial.print(oledReady ? 1 : 0);
  Serial.print(',');
  Serial.print(gpio_get_level(static_cast<gpio_num_t>(kMotorSleepPin)) ? 1 : 0);
  Serial.print(',');
  Serial.print(emergencyStopActive ? 1 : 0);
  Serial.print(',');
  Serial.print(sensors.batteryAdcMillivolts);
  Serial.print(',');
  Serial.print(millis());
  Serial.print(',');
  Serial.print(calibrationActive ? 1 : 0);
  Serial.print(',');
  Serial.print(tractionFaultActive ? 1 : 0);
  Serial.print(',');
  Serial.println(static_cast<uint8_t>(tractionFaultSide));
}

void enforceMotorTimeout()
{
  if (millis() - lastMotorCommandMs <= kMotorCommandTimeoutMs)
  {
    return;
  }

  stopMotorOutputs();
  if (controlSource == ControlSource::Dashboard)
  {
    // O navegador deve enviar heartbeat continuamente. Se ele desaparecer,
    // a habilitação de bancada também é removida e exige nova ação humana.
    dashboardArmed = false;
  }
}

void setup()
{
  // Os motores são a primeira parte configurada pelo setup(). Sensores, UART
  // e o modo opcional de bancada só iniciam depois que as entradas estão LOW.
  setupMotors();

  // A UART0 usa exatamente GPIO1/GPIO3, conforme a fiação com a Raspberry.
  // Mensagens de depuração não são enviadas para evitar corromper o protocolo.
  Serial.begin(kRaspberryBaudRate, SERIAL_8N1, kRaspberryRxPin, kRaspberryTxPin);

  pinMode(kStartButtonPin, INPUT_PULLUP);
  setupEncoders();
  setupUltrasonic();

  analogReadResolution(12);
  analogSetPinAttenuation(kBatteryAdcPin, ADC_11db);
  setupI2cDevices();

#ifndef OBR_ESP32_RASPBERRY_MODE
  WiFi.mode(WIFI_AP);
  WiFi.softAP(kWifiSsid, kWifiPassword);
  setupDashboardRoutes();
#endif

  lastMotorCommandMs = millis();
  lastEncoderRateMs = millis();
  Serial.println("READY");
}

void loop()
{
#ifndef OBR_ESP32_RASPBERRY_MODE
  server.handleClient();
#endif
  readUartCommands();
  updateStartButton();
  readImuIfDue();
  readBatteryIfDue();
  updateOledIfDue();
  updateEncoderRatesIfDue();
  enforceTractionSafety();
  triggerUltrasonicIfDue();
  consumeUltrasonicSample();
  enforceMotorTimeout();
  sendUartTelemetryIfDue();
  delay(1);
}
