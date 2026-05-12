#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// Pinos da UART2 da ESP32 ligada à Raspberry Pi.
// Ligue TX da Raspberry no RX2 da ESP32 e RX da Raspberry no TX2 da ESP32.
constexpr int RASPBERRY_RX_PIN = 16;
constexpr int RASPBERRY_TX_PIN = 17;
constexpr unsigned long UART_BAUD_RATE = 115200;

// Pinos I2C usados pelo MPU6050.
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;

// Pinos do sensor ultrassônico frontal.
// Se o módulo for HC-SR04 de 5 V, use divisor de tensão no ECHO para proteger a ESP32.
constexpr int TRIG_PIN = 25;
constexpr int ECHO_PIN = 35;

// Pinos dos drivers BTS7960.
// Cada motor usa um enable comum e dois sinais PWM: RPWM para um sentido e LPWM para o sentido oposto.
constexpr int LEFT_ENABLE_PIN = 15;
constexpr int LEFT_RPWM_PIN = 14;
constexpr int LEFT_LPWM_PIN = 5;
constexpr int RIGHT_ENABLE_PIN = 2;
constexpr int RIGHT_RPWM_PIN = 4;
constexpr int RIGHT_LPWM_PIN = 33;

// Inverta um motor aqui se ele girar no sentido contrário durante o teste com as rodas suspensas.
constexpr bool LEFT_MOTOR_INVERTED = true;
constexpr bool RIGHT_MOTOR_INVERTED = false;

// Configuração do PWM usado nos pinos RPWM e LPWM do BTS7960.
constexpr int PWM_FREQUENCY_HZ = 1000;
constexpr int PWM_RESOLUTION_BITS = 8;
constexpr int PWM_MAX_DUTY = (1 << PWM_RESOLUTION_BITS) - 1;
constexpr int LEFT_RPWM_CHANNEL = 0;
constexpr int LEFT_LPWM_CHANNEL = 1;
constexpr int RIGHT_RPWM_CHANNEL = 2;
constexpr int RIGHT_LPWM_CHANNEL = 3;

// Tempo máximo, em milissegundos, sem comando da Raspberry antes de zerar os motores.
constexpr unsigned long COMMAND_TIMEOUT_MS = 500;

// Intervalo, em milissegundos, entre linhas de telemetria enviadas para a Raspberry.
constexpr unsigned long SENSOR_INTERVAL_MS = 100;

// Timeout do ultrassônico em microssegundos. Evita travar o loop se não houver eco.
constexpr unsigned long ULTRASONIC_TIMEOUT_US = 25000;

// Zona morta evita PWM pequeno demais para mover o motor de forma previsível.
constexpr float MOTOR_DEADBAND = 0.05f;

// O MPU6050 costuma medir uma rotação pequena mesmo parado.
// A zona morta evita que esse ruído vire deriva constante no yaw da telemetria.
constexpr float GYRO_Z_RATE_DEADBAND_DPS = 0.8f;
constexpr float GYRO_Z_STATIONARY_BIAS_ALPHA = 0.002f;

HardwareSerial RaspberrySerial(2);
Adafruit_MPU6050 mpu;

float gyroZBias = 0.0f;
float yawZ = 0.0f;
float currentLeftPower = 0.0f;
float currentRightPower = 0.0f;
bool mpuReady = false;
bool emergencyStopActive = false;

unsigned long lastCommandTimeMs = 0;
unsigned long lastSensorTimeMs = 0;
unsigned long lastMpuTimeMs = 0;

constexpr size_t COMMAND_BUFFER_SIZE = 96;
char commandBuffer[COMMAND_BUFFER_SIZE] = {};
size_t commandLength = 0;

void attachPwmPin(int pin, int channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
#else
  ledcSetup(channel, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(pin, channel);
#endif
}

void writePwmDuty(int pin, int channel, int duty) {
  duty = constrain(duty, 0, PWM_MAX_DUTY);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

float clampMotorPower(float command) {
  if (!isfinite(command)) {
    return 0.0f;
  }

  command = constrain(command, -1.0f, 1.0f);
  if (fabs(command) < MOTOR_DEADBAND) {
    return 0.0f;
  }

  return command;
}

void stopMotorOutputs() {
  writePwmDuty(LEFT_RPWM_PIN, LEFT_RPWM_CHANNEL, 0);
  writePwmDuty(LEFT_LPWM_PIN, LEFT_LPWM_CHANNEL, 0);
  writePwmDuty(RIGHT_RPWM_PIN, RIGHT_RPWM_CHANNEL, 0);
  writePwmDuty(RIGHT_LPWM_PIN, RIGHT_LPWM_CHANNEL, 0);

  digitalWrite(LEFT_ENABLE_PIN, LOW);
  digitalWrite(RIGHT_ENABLE_PIN, LOW);

  currentLeftPower = 0.0f;
  currentRightPower = 0.0f;
}

void setOneMotor(int enablePin, int rpwmPin, int rpwmChannel, int lpwmPin, int lpwmChannel, float command, bool inverted) {
  if (inverted) {
    command = -command;
  }

  float safeCommand = clampMotorPower(command);
  if (safeCommand == 0.0f) {
    writePwmDuty(rpwmPin, rpwmChannel, 0);
    writePwmDuty(lpwmPin, lpwmChannel, 0);
    digitalWrite(enablePin, LOW);
    return;
  }

  digitalWrite(enablePin, HIGH);

  int duty = static_cast<int>(fabs(safeCommand) * PWM_MAX_DUTY);
  if (safeCommand > 0.0f) {
    writePwmDuty(lpwmPin, lpwmChannel, 0);
    writePwmDuty(rpwmPin, rpwmChannel, duty);
  } else {
    writePwmDuty(rpwmPin, rpwmChannel, 0);
    writePwmDuty(lpwmPin, lpwmChannel, duty);
  }
}

void applyMotorCommand(float leftPower, float rightPower) {
  if (emergencyStopActive) {
    stopMotorOutputs();
    return;
  }

  currentLeftPower = clampMotorPower(leftPower);
  currentRightPower = clampMotorPower(rightPower);

  setOneMotor(LEFT_ENABLE_PIN, LEFT_RPWM_PIN, LEFT_RPWM_CHANNEL, LEFT_LPWM_PIN, LEFT_LPWM_CHANNEL, currentLeftPower, LEFT_MOTOR_INVERTED);
  setOneMotor(RIGHT_ENABLE_PIN, RIGHT_RPWM_PIN, RIGHT_RPWM_CHANNEL, RIGHT_LPWM_PIN, RIGHT_LPWM_CHANNEL, currentRightPower, RIGHT_MOTOR_INVERTED);
}

void setupMotorPins() {
  pinMode(LEFT_ENABLE_PIN, OUTPUT);
  pinMode(RIGHT_ENABLE_PIN, OUTPUT);

  attachPwmPin(LEFT_RPWM_PIN, LEFT_RPWM_CHANNEL);
  attachPwmPin(LEFT_LPWM_PIN, LEFT_LPWM_CHANNEL);
  attachPwmPin(RIGHT_RPWM_PIN, RIGHT_RPWM_CHANNEL);
  attachPwmPin(RIGHT_LPWM_PIN, RIGHT_LPWM_CHANNEL);

  // O robô deve ligar com os motores parados, antes de qualquer comando UART.
  stopMotorOutputs();
}

float readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0) {
    return -1.0f;
  }

  float distanceCm = duration * 0.0343f / 2.0f;
  if (distanceCm < 2.0f || distanceCm > 400.0f) {
    return -1.0f;
  }

  return distanceCm;
}

void calibrateGyroZ() {
  if (!mpuReady) {
    return;
  }

  Serial.println("Calibrando gyro Z. Deixe o MPU6050 parado.");

  constexpr int samples = 800;
  float sum = 0.0f;

  for (int i = 0; i < samples; i++) {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    mpu.getEvent(&accel, &gyro, &temp);
    sum += gyro.gyro.z * 57.2958f;
    delay(5);
  }

  gyroZBias = sum / samples;
  Serial.print("Bias gyro Z: ");
  Serial.print(gyroZBias);
  Serial.println(" deg/s");
}

void setupMpu() {
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!mpu.begin(0x68)) {
    Serial.println("MPU6050 não encontrado em 0x68. Tentando 0x69.");

    if (!mpu.begin(0x69)) {
      // O motor continua seguro sem MPU, mas o dashboard mostrará o sensor como indisponível.
      Serial.println("MPU6050 não encontrado. Confira VCC, GND, SDA, SCL e AD0.");
      mpuReady = false;
      return;
    }
  }

  mpuReady = true;
  Serial.println("MPU6050 encontrado.");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(500);
  calibrateGyroZ();
}

void sendError(const char* message) {
  RaspberrySerial.print("ERR,");
  RaspberrySerial.println(message);
}

void handleCommandLine(const char* line) {
  if (strcmp(line, "PING") == 0) {
    RaspberrySerial.println("PONG");
    return;
  }

  if (strcmp(line, "STOP") == 0) {
    lastCommandTimeMs = millis();
    emergencyStopActive = false;
    stopMotorOutputs();
    return;
  }

  if (strcmp(line, "ESTOP") == 0) {
    lastCommandTimeMs = millis();
    emergencyStopActive = true;
    stopMotorOutputs();
    return;
  }

  float leftPower = 0.0f;
  float rightPower = 0.0f;
  int emergencyFlag = 0;

  if (sscanf(line, "MOTOR,%f,%f,%d", &leftPower, &rightPower, &emergencyFlag) == 3) {
    lastCommandTimeMs = millis();
    emergencyStopActive = emergencyFlag != 0;
    applyMotorCommand(leftPower, rightPower);
    return;
  }

  sendError("bad_command");
}

void readCommandsFromRaspberry() {
  while (RaspberrySerial.available() > 0) {
    char c = static_cast<char>(RaspberrySerial.read());

    if (c == '\n') {
      commandBuffer[commandLength] = '\0';
      handleCommandLine(commandBuffer);
      commandLength = 0;
      continue;
    }

    if (c == '\r') {
      continue;
    }

    if (commandLength < COMMAND_BUFFER_SIZE - 1) {
      commandBuffer[commandLength++] = c;
    } else {
      commandLength = 0;
      sendError("command_too_long");
    }
  }
}

void enforceCommandTimeout() {
  if (millis() - lastCommandTimeMs > COMMAND_TIMEOUT_MS) {
    // Se a Raspberry parar de enviar comandos, a ESP32 zera os drivers dos motores sozinha.
    stopMotorOutputs();
  }
}

void sendSensorTelemetry() {
  float distanceCm = readUltrasonicCm();
  float correctedGyroZ = 0.0f;
  float accelX = 0.0f;
  float accelY = 0.0f;
  float accelZ = 0.0f;

  unsigned long now = millis();
  float dt = (now - lastMpuTimeMs) / 1000.0f;
  lastMpuTimeMs = now;

  if (mpuReady) {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    mpu.getEvent(&accel, &gyro, &temp);

    correctedGyroZ = gyro.gyro.z * 57.2958f - gyroZBias;
    bool robotStopped = fabs(currentLeftPower) < 0.01f && fabs(currentRightPower) < 0.01f;

    if (robotStopped && fabs(correctedGyroZ) < 3.0f) {
      // Quando o robô está parado, ajusta o bias lentamente para compensar deriva térmica.
      gyroZBias = (gyroZBias * (1.0f - GYRO_Z_STATIONARY_BIAS_ALPHA)) +
                  ((gyro.gyro.z * 57.2958f) * GYRO_Z_STATIONARY_BIAS_ALPHA);
      correctedGyroZ = gyro.gyro.z * 57.2958f - gyroZBias;
    }

    if (fabs(correctedGyroZ) < GYRO_Z_RATE_DEADBAND_DPS) {
      correctedGyroZ = 0.0f;
    }

    yawZ += correctedGyroZ * dt;

    if (yawZ > 180.0f) {
      yawZ -= 360.0f;
    }
    if (yawZ < -180.0f) {
      yawZ += 360.0f;
    }

    accelX = accel.acceleration.x;
    accelY = accel.acceleration.y;
    accelZ = accel.acceleration.z;
  }

  RaspberrySerial.print("SENSOR,");
  RaspberrySerial.print(distanceCm, 2);
  RaspberrySerial.print(",");
  RaspberrySerial.print(correctedGyroZ, 2);
  RaspberrySerial.print(",");
  RaspberrySerial.print(yawZ, 2);
  RaspberrySerial.print(",");
  RaspberrySerial.print(accelX, 2);
  RaspberrySerial.print(",");
  RaspberrySerial.print(accelY, 2);
  RaspberrySerial.print(",");
  RaspberrySerial.print(accelZ, 2);
  RaspberrySerial.print(",");
  RaspberrySerial.println(mpuReady ? 1 : 0);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32 OBR bridge: motores, MPU6050 e ultrassônico");

  RaspberrySerial.begin(UART_BAUD_RATE, SERIAL_8N1, RASPBERRY_RX_PIN, RASPBERRY_TX_PIN);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  setupMotorPins();
  setupMpu();

  lastCommandTimeMs = millis();
  lastSensorTimeMs = millis();
  lastMpuTimeMs = millis();

  RaspberrySerial.println("READY");
  Serial.println("Pronto para comandos da Raspberry.");
}

void loop() {
  readCommandsFromRaspberry();
  enforceCommandTimeout();

  unsigned long now = millis();
  if (now - lastSensorTimeMs >= SENSOR_INTERVAL_MS) {
    lastSensorTimeMs = now;
    sendSensorTelemetry();
  }

  delay(1);
}
