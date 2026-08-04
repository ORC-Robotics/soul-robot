#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_Sensor.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <driver/gpio.h>

#include "dashboard_page.h"
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

void forceMotorPinsLowImmediately()
{
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

// O dashboard HTTP pertence somente à ESP32 e permite testar o hardware
// mesmo quando a Raspberry Pi ainda não está executando o programa principal.
WebServer server(80);
Adafruit_MPU6050 mpu;
Adafruit_PWMServoDriver pca9685(kPca9685Address, Wire);

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
  float imuTemperatureCelsius = 0.0f;
  float leftEncoderRate = 0.0f;
  float rightEncoderRate = 0.0f;
};

SensorState sensors;
bool mpuReady = false;
bool pca9685Ready = false;
uint8_t mpuAddress = 0;
bool startButtonPressed = false;
bool dashboardArmed = false;
bool emergencyStopActive = false;
float currentLeftPower = 0.0f;
float currentRightPower = 0.0f;
float gyroZBias = 0.0f;
ControlSource controlSource = ControlSource::None;

uint32_t lastMotorCommandMs = 0;
uint32_t lastImuReadMs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastBatteryReadMs = 0;
uint32_t lastEncoderRateMs = 0;
uint32_t lastUltrasonicTriggerMs = 0;
uint32_t lastButtonChangeMs = 0;
uint32_t lastMpuIntegrationUs = 0;

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t leftEncoderCount = 0;
volatile int32_t rightEncoderCount = 0;
volatile uint8_t previousLeftEncoderState = 0;
volatile uint8_t previousRightEncoderState = 0;
int32_t lastLeftEncoderRateCount = 0;
int32_t lastRightEncoderRateCount = 0;

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

float safeMotorPower(float power)
{
  if (!isfinite(power))
  {
    return 0.0f;
  }

  power = constrain(power, -kMaximumMotorPower, kMaximumMotorPower);
  return fabs(power) < kMotorDeadband ? 0.0f : power;
}

void stopMotorOutputs()
{
  // IN1 e IN2 em zero colocam o DRV8833 em coast. Esta função é usada
  // na inicialização, no timeout, no E-Stop e antes de trocar o sentido.
  writeMotorPwm(kLeftMotorIn1Pin, kLeftMotorIn1Channel, 0);
  writeMotorPwm(kLeftMotorIn2Pin, kLeftMotorIn2Channel, 0);
  writeMotorPwm(kRightMotorIn1Pin, kRightMotorIn1Channel, 0);
  writeMotorPwm(kRightMotorIn2Pin, kRightMotorIn2Channel, 0);
  currentLeftPower = 0.0f;
  currentRightPower = 0.0f;
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
  if (emergencyStopActive || (source == ControlSource::Dashboard && !dashboardArmed))
  {
    stopMotorOutputs();
    return;
  }

  currentLeftPower = safeMotorPower(leftPower);
  currentRightPower = safeMotorPower(rightPower);
  setMotorSide(kLeftMotorIn1Pin, kLeftMotorIn1Channel, kLeftMotorIn2Pin,
               kLeftMotorIn2Channel, currentLeftPower, kLeftMotorInverted);
  setMotorSide(kRightMotorIn1Pin, kRightMotorIn1Channel, kRightMotorIn2Pin,
               kRightMotorIn2Channel, currentRightPower, kRightMotorInverted);
  controlSource = source;
  lastMotorCommandMs = millis();
}

void setupMotors()
{
  // Repete a parada imediatamente antes de entregar os pinos ao periférico
  // PWM. A redundância é intencional porque esta etapa afeta movimento real.
  forceMotorPinsLowImmediately();
  attachMotorPwm(kLeftMotorIn1Pin, kLeftMotorIn1Channel);
  attachMotorPwm(kLeftMotorIn2Pin, kLeftMotorIn2Channel);
  attachMotorPwm(kRightMotorIn1Pin, kRightMotorIn1Channel);
  attachMotorPwm(kRightMotorIn2Pin, kRightMotorIn2Channel);
  stopMotorOutputs();
}

bool i2cDeviceResponds(uint8_t address)
{
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void calibrateGyroZ()
{
  // A calibração ocorre apenas no boot. O robô deve permanecer parado
  // durante aproximadamente um segundo para o yaw iniciar com menos deriva.
  constexpr int kCalibrationSamples = 300;
  float sum = 0.0f;
  for (int sample = 0; sample < kCalibrationSamples; ++sample)
  {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temperature;
    mpu.getEvent(&accel, &gyro, &temperature);
    sum += gyro.gyro.z;
    delay(3);
  }
  gyroZBias = sum / kCalibrationSamples;
}

void setupI2cDevices()
{
  Wire.begin(kI2cSdaPin, kI2cSclPin, kI2cFrequencyHz);

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
    calibrateGyroZ();
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
  sensors.gyroY = gyro.gyro.y * RAD_TO_DEG;
  sensors.gyroZ = (gyro.gyro.z - gyroZBias) * RAD_TO_DEG;
  sensors.imuTemperatureCelsius = temperature.temperature;
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

void updateStartButton()
{
  constexpr uint32_t kDebounceMs = 30;
  const bool rawPressed = digitalRead(kStartButtonPin) == LOW;
  if (rawPressed != startButtonPressed && millis() - lastButtonChangeMs >= kDebounceMs)
  {
    startButtonPressed = rawPressed;
    lastButtonChangeMs = millis();
  }
}

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
  json += ",\"dashboardArmed\":" + String(dashboardArmed ? "true" : "false");
  json += ",\"startButtonPressed\":" + String(startButtonPressed ? "true" : "false");
  json += ",\"mpuReady\":" + String(mpuReady ? "true" : "false");
  json += ",\"mpuAddress\":" + String(mpuAddress);
  json += ",\"pca9685Ready\":" + String(pca9685Ready ? "true" : "false");
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
              portENTER_CRITICAL(&encoderMux);
              leftEncoderCount = 0;
              rightEncoderCount = 0;
              portEXIT_CRITICAL(&encoderMux);
              lastLeftEncoderRateCount = 0;
              lastRightEncoderRateCount = 0;
              sendJsonResponse(200, "{\"ok\":true}"); });
  server.onNotFound([]()
                    { sendJsonResponse(404, "{\"error\":\"Endpoint nao encontrado\"}"); });
  server.begin();
}

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
  Serial.println(sensors.rightEncoderRate, 1);
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
  // e Wi-Fi só podem iniciar depois que todas as entradas do driver estão LOW.
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

  WiFi.mode(WIFI_AP);
  WiFi.softAP(kWifiSsid, kWifiPassword);
  setupDashboardRoutes();

  lastMotorCommandMs = millis();
  lastEncoderRateMs = millis();
  Serial.println("READY");
}

void loop()
{
  server.handleClient();
  readUartCommands();
  updateStartButton();
  readImuIfDue();
  readBatteryIfDue();
  updateEncoderRatesIfDue();
  triggerUltrasonicIfDue();
  consumeUltrasonicSample();
  enforceMotorTimeout();
  sendUartTelemetryIfDue();
  delay(1);
}
