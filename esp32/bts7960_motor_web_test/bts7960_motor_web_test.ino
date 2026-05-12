#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// Rede temporária criada pela ESP32 apenas para teste de bancada dos motores.
// Esta senha não é uma credencial real do robô; ela só evita conexões acidentais durante o teste.
constexpr const char* WIFI_SSID = "OBR-Motor-Test";
constexpr const char* WIFI_PASSWORD = "obr2026k";

// Pinos dos drivers BTS7960, iguais ao sketch principal da ESP32.
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

// Frequência e resolução do PWM enviado ao BTS7960.
// A saída final é sempre limitada entre 0 e 255 antes de chegar ao driver.
constexpr int PWM_FREQUENCY_HZ = 1000;
constexpr int PWM_RESOLUTION_BITS = 8;
constexpr int PWM_MAX_DUTY = (1 << PWM_RESOLUTION_BITS) - 1;
constexpr int LEFT_RPWM_CHANNEL = 0;
constexpr int LEFT_LPWM_CHANNEL = 1;
constexpr int RIGHT_RPWM_CHANNEL = 2;
constexpr int RIGHT_LPWM_CHANNEL = 3;

// Pinos I2C usados pelo MPU6050.
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;

// Potência máxima usada por este teste.
// Comece baixo para reduzir o risco de colisão se as rodas estiverem no chão.
constexpr float TEST_MAX_POWER = 1f;

// Correção de eixo usando o gyro Z do MPU6050.
// A correção só atua quando há aceleração/ré e o comando de giro está quase zerado.
constexpr float HEADING_HOLD_KP = 0.025f;
constexpr float HEADING_HOLD_MAX_CORRECTION = 0.25f;
constexpr float HEADING_HOLD_MIN_THROTTLE = 0.12f;
constexpr float HEADING_HOLD_TURN_DEADBAND = 0.06f;
constexpr float HEADING_HOLD_CORRECTION_SIGN = 1.0f;
constexpr int HEADING_HOLD_CALIBRATION_SAMPLES = 800;

// O MPU6050 costuma medir uma rotação pequena mesmo parado.
// A zona morta evita que esse ruído vire deriva constante no yaw.
constexpr float GYRO_Z_RATE_DEADBAND_DPS = 0.8f;
constexpr float GYRO_Z_STATIONARY_BIAS_ALPHA = 0.002f;

// Se a página parar de enviar comandos, a ESP32 desliga os motores sozinha.
constexpr unsigned long COMMAND_TIMEOUT_MS = 500;

WebServer server(80);
Adafruit_MPU6050 mpu;

float currentLeftPower = 0.0f;
float currentRightPower = 0.0f;
float gyroZBias = 0.0f;
float yawZ = 0.0f;
float targetYawZ = 0.0f;
bool mpuReady = false;
bool headingHoldActive = false;
unsigned long lastCommandTimeMs = 0;
unsigned long lastMpuTimeMs = 0;

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="pt-BR">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Teste BTS7960</title>
  <style>
    :root { color-scheme: dark; font-family: Arial, sans-serif; }
    body { margin: 0; background: #101418; color: #edf2f7; }
    main { max-width: 720px; margin: 0 auto; padding: 18px; }
    h1 { font-size: 28px; margin: 8px 0 18px; }
    section { border-top: 1px solid #2d3748; padding: 16px 0; }
    label { display: block; margin: 14px 0 8px; font-weight: 700; }
    input[type="range"] { width: 100%; }
    .row { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 10px; }
    button {
      min-height: 48px;
      border: 0;
      border-radius: 8px;
      background: #2b6cb0;
      color: white;
      font-size: 16px;
      font-weight: 700;
    }
    button.stop { background: #c53030; }
    button.neutral { background: #4a5568; }
    output { font-variant-numeric: tabular-nums; }
    .status { color: #a0aec0; min-height: 22px; }
  </style>
</head>
<body>
  <main>
    <h1>Teste BTS7960</h1>
    <section>
      <p class="status" id="status">Conectando...</p>
      <p class="status" id="gamepadStatus">Controle: desconectado</p>
      <p class="status" id="headingStatus">MPU6050: verificando...</p>
      <button class="stop" id="stop">PARAR</button>
    </section>
    <section>
      <label for="left">Motor esquerdo <output id="leftValue">0.00</output></label>
      <input id="left" type="range" min="-100" max="100" value="0">
      <label for="right">Motor direito <output id="rightValue">0.00</output></label>
      <input id="right" type="range" min="-100" max="100" value="0">
    </section>
    <section class="row">
      <button data-left="0.20" data-right="0.00">Esq +</button>
      <button data-left="-0.20" data-right="0.00">Esq -</button>
      <button data-left="0.00" data-right="0.20">Dir +</button>
      <button data-left="0.00" data-right="-0.20">Dir -</button>
      <button class="neutral" data-left="0.20" data-right="0.20">Frente</button>
      <button class="neutral" data-left="-0.20" data-right="-0.20">Ré</button>
    </section>
  </main>
  <script>
    const left = document.getElementById('left');
    const right = document.getElementById('right');
    const leftValue = document.getElementById('leftValue');
    const rightValue = document.getElementById('rightValue');
    const statusText = document.getElementById('status');
    const gamepadStatus = document.getElementById('gamepadStatus');
    const headingStatus = document.getElementById('headingStatus');
    const maxPower = Number(left.max) / 100;

    function valueFromSlider(slider) {
      return Number(slider.value) / 100;
    }

    function clamp(value, min, max) {
      return Math.min(max, Math.max(min, value));
    }

    function axisWithDeadband(value) {
      return Math.abs(value) < 0.08 ? 0 : value;
    }

    function triggerValue(button) {
      if (!button) {
        return 0;
      }

      return typeof button.value === 'number' ? button.value : (button.pressed ? 1 : 0);
    }

    function firstGamepad() {
      if (!navigator.getGamepads) {
        gamepadStatus.textContent = 'Controle: navegador sem suporte';
        return null;
      }

      return Array.from(navigator.getGamepads()).find((gamepad) => gamepad) || null;
    }

    function updateLabels() {
      leftValue.textContent = valueFromSlider(left).toFixed(2);
      rightValue.textContent = valueFromSlider(right).toFixed(2);
    }

    async function sendMotor(leftPower, rightPower) {
      updateLabels();
      try {
        const response = await fetch(`/motor?left=${leftPower.toFixed(2)}&right=${rightPower.toFixed(2)}`, { cache: 'no-store' });
        statusText.textContent = response.ok ? `Enviado: ${leftPower.toFixed(2)}, ${rightPower.toFixed(2)}` : 'Falha ao enviar comando';
      } catch (error) {
        statusText.textContent = 'Sem resposta da ESP32';
      }
    }

    function sendCurrent() {
      sendMotor(valueFromSlider(left), valueFromSlider(right));
    }

    function setSlidersFromPower(leftPower, rightPower) {
      left.value = Math.round(clamp(leftPower, -maxPower, maxPower) * 100);
      right.value = Math.round(clamp(rightPower, -maxPower, maxPower) * 100);
      sendCurrent();
    }

    function stopMotors() {
      left.value = 0;
      right.value = 0;
      sendCurrent();
    }

    async function updateStatus() {
      try {
        const response = await fetch('/status', { cache: 'no-store' });
        const data = await response.json();
        const hold = data.headingHoldActive ? 'mantendo eixo' : 'livre';
        headingStatus.textContent = data.mpuReady ? `MPU6050: ${hold}, yaw ${data.yawZ.toFixed(1)}°` : 'MPU6050: indisponível';
      } catch (error) {
        headingStatus.textContent = 'MPU6050: sem status';
      }
    }

    function updateFromGamepad() {
      const gamepad = firstGamepad();
      if (!gamepad) {
        sendCurrent();
        return;
      }

      const reverse = triggerValue(gamepad.buttons[6]);
      const forward = triggerValue(gamepad.buttons[7]);
      const throttle = forward - reverse;
      const turn = axisWithDeadband(gamepad.axes[0] || 0);

      let leftPower = throttle + turn;
      let rightPower = throttle - turn;
      const largest = Math.max(1, Math.abs(leftPower), Math.abs(rightPower));

      leftPower = (leftPower / largest) * maxPower;
      rightPower = (rightPower / largest) * maxPower;

      gamepadStatus.textContent = `Controle: ${gamepad.id}`;
      setSlidersFromPower(leftPower, rightPower);
    }

    left.addEventListener('input', sendCurrent);
    right.addEventListener('input', sendCurrent);
    document.getElementById('stop').addEventListener('click', stopMotors);

    window.addEventListener('gamepadconnected', (event) => {
      gamepadStatus.textContent = `Controle: ${event.gamepad.id}`;
    });

    window.addEventListener('gamepaddisconnected', () => {
      gamepadStatus.textContent = 'Controle: desconectado';
      stopMotors();
    });

    document.querySelectorAll('[data-left]').forEach((button) => {
      button.addEventListener('click', () => {
        left.value = Math.round(Number(button.dataset.left) * 100);
        right.value = Math.round(Number(button.dataset.right) * 100);
        sendCurrent();
      });
    });

    updateLabels();
    stopMotors();
    setInterval(updateFromGamepad, 100);
    setInterval(updateStatus, 500);
  </script>
</body>
</html>
)rawliteral";

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

float safeMotorPower(float command) {
  if (!isfinite(command)) {
    return 0.0f;
  }

  return constrain(command, -TEST_MAX_POWER, TEST_MAX_POWER);
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
  headingHoldActive = false;
}

float normalizeAngle(float angleDegrees) {
  while (angleDegrees > 180.0f) {
    angleDegrees -= 360.0f;
  }

  while (angleDegrees < -180.0f) {
    angleDegrees += 360.0f;
  }

  return angleDegrees;
}

void calibrateGyroZ() {
  if (!mpuReady) {
    return;
  }

  Serial.println("Calibrando gyro Z. Deixe o robô parado.");

  float sum = 0.0f;
  for (int i = 0; i < HEADING_HOLD_CALIBRATION_SAMPLES; i++) {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    mpu.getEvent(&accel, &gyro, &temp);
    sum += gyro.gyro.z * 57.2958f;
    delay(5);
  }

  gyroZBias = sum / HEADING_HOLD_CALIBRATION_SAMPLES;
  yawZ = 0.0f;
  targetYawZ = 0.0f;
  lastMpuTimeMs = millis();

  Serial.print("Bias gyro Z: ");
  Serial.print(gyroZBias);
  Serial.println(" deg/s");
}

void setupMpu() {
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!mpu.begin(0x68)) {
    Serial.println("MPU6050 não encontrado em 0x68. Tentando 0x69.");

    if (!mpu.begin(0x69)) {
      Serial.println("MPU6050 não encontrado. O teste continuará sem correção de eixo.");
      mpuReady = false;
      return;
    }
  }

  mpuReady = true;
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(500);
  calibrateGyroZ();
}

void updateYawFromMpu() {
  if (!mpuReady) {
    return;
  }

  unsigned long now = millis();
  float dt = (now - lastMpuTimeMs) / 1000.0f;
  lastMpuTimeMs = now;

  if (dt <= 0.0f || dt > 0.25f) {
    return;
  }

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  mpu.getEvent(&accel, &gyro, &temp);

  float correctedGyroZ = gyro.gyro.z * 57.2958f - gyroZBias;
  bool motorsStopped = fabs(currentLeftPower) < 0.01f && fabs(currentRightPower) < 0.01f;

  if (motorsStopped && fabs(correctedGyroZ) < 3.0f) {
    // Quando o robô está parado, ajusta o bias lentamente para compensar deriva térmica.
    gyroZBias = (gyroZBias * (1.0f - GYRO_Z_STATIONARY_BIAS_ALPHA)) +
                ((gyro.gyro.z * 57.2958f) * GYRO_Z_STATIONARY_BIAS_ALPHA);
    correctedGyroZ = gyro.gyro.z * 57.2958f - gyroZBias;
  }

  if (fabs(correctedGyroZ) < GYRO_Z_RATE_DEADBAND_DPS) {
    correctedGyroZ = 0.0f;
  }

  yawZ = normalizeAngle(yawZ + correctedGyroZ * dt);
}

float headingCorrection(float requestedThrottle, float requestedTurn) {
  if (!mpuReady) {
    return 0.0f;
  }

  if (fabs(requestedThrottle) < HEADING_HOLD_MIN_THROTTLE || fabs(requestedTurn) > HEADING_HOLD_TURN_DEADBAND) {
    headingHoldActive = false;
    return 0.0f;
  }

  if (!headingHoldActive) {
    targetYawZ = yawZ;
    headingHoldActive = true;
  }

  float errorDegrees = normalizeAngle(targetYawZ - yawZ);
  return constrain(errorDegrees * HEADING_HOLD_KP * HEADING_HOLD_CORRECTION_SIGN,
                   -HEADING_HOLD_MAX_CORRECTION,
                   HEADING_HOLD_MAX_CORRECTION);
}

void setOneMotor(int enablePin, int rpwmPin, int rpwmChannel, int lpwmPin, int lpwmChannel, float command, bool inverted) {
  if (inverted) {
    command = -command;
  }

  float safeCommand = safeMotorPower(command);
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
  float requestedLeftPower = safeMotorPower(leftPower);
  float requestedRightPower = safeMotorPower(rightPower);
  float requestedThrottle = (requestedLeftPower + requestedRightPower) * 0.5f;
  float requestedTurn = (requestedLeftPower - requestedRightPower) * 0.5f;
  float correction = headingCorrection(requestedThrottle, requestedTurn);

  currentLeftPower = safeMotorPower(requestedLeftPower + correction);
  currentRightPower = safeMotorPower(requestedRightPower - correction);

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

  // O teste sempre inicia com as saídas zeradas para evitar movimento inesperado ao energizar.
  stopMotorOutputs();
}

void handleIndex() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleMotorCommand() {
  if (!server.hasArg("left") || !server.hasArg("right")) {
    stopMotorOutputs();
    server.send(400, "text/plain", "missing_left_or_right");
    return;
  }

  float leftPower = safeMotorPower(server.arg("left").toFloat());
  float rightPower = safeMotorPower(server.arg("right").toFloat());

  lastCommandTimeMs = millis();
  applyMotorCommand(leftPower, rightPower);

  server.send(200, "text/plain", "ok");
}

void handleStop() {
  lastCommandTimeMs = millis();
  stopMotorOutputs();
  server.send(200, "text/plain", "stopped");
}

void handleStatus() {
  String status = "{";
  status += "\"left\":";
  status += String(currentLeftPower, 2);
  status += ",\"right\":";
  status += String(currentRightPower, 2);
  status += ",\"yawZ\":";
  status += String(yawZ, 2);
  status += ",\"mpuReady\":";
  status += (mpuReady ? "true" : "false");
  status += ",\"headingHoldActive\":";
  status += (headingHoldActive ? "true" : "false");
  status += ",\"timeoutMs\":";
  status += String(COMMAND_TIMEOUT_MS);
  status += "}";

  server.send(200, "application/json", status);
}

void enforceCommandTimeout() {
  if (millis() - lastCommandTimeMs > COMMAND_TIMEOUT_MS) {
    stopMotorOutputs();
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  setupMotorPins();
  setupMpu();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);

  server.on("/", HTTP_GET, handleIndex);
  server.on("/motor", HTTP_GET, handleMotorCommand);
  server.on("/stop", HTTP_GET, handleStop);
  server.on("/status", HTTP_GET, handleStatus);
  server.begin();

  lastCommandTimeMs = millis();

  Serial.println();
  Serial.println("Teste web BTS7960 iniciado.");
  Serial.print("Wi-Fi: ");
  Serial.println(WIFI_SSID);
  Serial.print("Senha: ");
  Serial.println(WIFI_PASSWORD);
  Serial.print("Abra: http://");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  updateYawFromMpu();
  server.handleClient();
  enforceCommandTimeout();
  delay(1);
}
