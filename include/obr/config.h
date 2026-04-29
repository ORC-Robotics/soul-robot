#pragma once

namespace config
{
// Porta HTTP usada pelo dashboard e pelo WebSocket.
// Se mudar este valor, atualize também a URL usada para acessar o robô.
constexpr int kDashboardPort = 8080;

// Intervalo, em milissegundos, entre envios de telemetria para o dashboard.
constexpr int kTelemetryPeriodMs = 500;

// Quantidade de amostras entre logs de telemetria no journal.
// O dashboard continua recebendo todas as amostras; isso só reduz poluição no log.
constexpr int kTelemetryLogEverySamples = 20;

// Tempo máximo, em milissegundos, sem comandos antes de zerar os motores.
// Isso impede que o robô continue andando com um comando antigo.
constexpr int kCommandTimeoutMs = 2000;

// Intervalo, em milissegundos, do loop principal que aplica os comandos aos motores.
constexpr int kMainLoopPeriodMs = 20;

// Período, em milissegundos, do PWM por software usado nos pinos ENA e ENB.
// Valores menores deixam o controle mais suave, mas aumentam o uso de CPU.
constexpr int kMotorPwmPeriodMs = 10;

// Tempo de espera, em milissegundos, após exportar um GPIO no Linux.
// A pasta /sys/class/gpio/gpioN pode levar um instante para aparecer.
constexpr int kGpioExportDelayMs = 100;

// Limites seguros para comandos de motor.
// O dashboard pode enviar valores fora da faixa, então o código limita antes
// de atualizar o estado do robô ou acionar a ponte H.
constexpr double kMinMotorOutput = -1.0;
constexpr double kMaxMotorOutput = 1.0;

// Pinos BCM da Raspberry Pi conectados à ponte H L298N.
// Ajuste estes valores quando a fiação do robô mudar.
// ENA habilita o motor esquerdo e recebe PWM por software.
constexpr int kLeftEnablePin = 18;

// IN1 controla um lado da direção do motor esquerdo.
constexpr int kLeftInput1Pin = 17;

// IN2 controla o outro lado da direção do motor esquerdo.
constexpr int kLeftInput2Pin = 27;

// ENB habilita o motor direito e recebe PWM por software.
constexpr int kRightEnablePin = 13;

// IN3 controla um lado da direção do motor direito.
constexpr int kRightInput1Pin = 22;

// IN4 controla o outro lado da direção do motor direito.
constexpr int kRightInput2Pin = 23;

// Zona morta do motor.
// Comandos com módulo menor que este valor são tratados como parada.
constexpr double kMotorDeadband = 0.05;
}
