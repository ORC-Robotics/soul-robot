#pragma once

namespace config
{
// Porta HTTP usada pelo dashboard e pelo WebSocket.
// Se mudar este valor, atualize também a URL usada para acessar o robô.
constexpr int kDashboardPort = 8080;

// Intervalo, em milissegundos, entre envios de telemetria para o dashboard.
constexpr int kTelemetryPeriodMs = 500;

// Arquivo JPEG atualizado pelo script da câmera.
// O dashboard lê este caminho; se o script não estiver rodando, a imagem fica indisponível.
constexpr const char* kCameraFramePath = "/tmp/obr_camera_frame.jpg";

// Arquivo JSON atualizado pelo script da câmera com dados do processamento.
// O FPS mostrado no dashboard vem deste arquivo para não misturar câmera com telemetria do robô.
constexpr const char* kCameraStatusPath = "/tmp/obr_camera_status.json";

// Porta local do stream MJPEG gerado pelo script Python da câmera.
// O dashboard acessa esse vídeo pelo proxy /camera-stream.mjpg na porta principal.
constexpr int kCameraStreamPort = 8090;

// Caminho HTTP do stream MJPEG dentro do script Python da câmera.
constexpr const char* kCameraStreamPath = "/stream.mjpg";

// Pino físico BOARD 40 usado pelo script da câmera para ligar a iluminação.
// Na Raspberry Pi, esse pino corresponde ao GPIO21; alterar exige revisar a fiação.
constexpr int kCameraLightPinBoard = 40;

// Quantidade de amostras entre logs de telemetria no journal.
// O dashboard continua recebendo todas as amostras; isso só reduz poluição no log.
constexpr int kTelemetryLogEverySamples = 20;

// Tempo máximo, em milissegundos, sem comandos antes de zerar os motores.
// Isso impede que o robô continue andando com um comando antigo.
constexpr int kCommandTimeoutMs = 2000;

// Intervalo, em milissegundos, do loop principal que aplica os comandos aos motores.
constexpr int kMainLoopPeriodMs = 20;

// Tempo máximo, em milissegundos, para aceitar dados da câmera no modo autônomo.
// Se a câmera travar ou parar de atualizar o JSON, o robô deve parar.
constexpr int kCameraStatusTimeoutMs = 400;

// Potência base usada para seguir linha no modo autônomo.
// Comece com valor baixo nos testes para reduzir o risco de colisão.
constexpr double kLineFollowerBasePower = 0.3;

// Ganho proporcional aplicado ao erro horizontal da linha, em pixels.
// Valores altos fazem o robô virar mais forte, mas podem causar oscilação.
constexpr double kLineFollowerTurnGain = 0.0005;

// Potência máxima de correção usada pelo seguidor de linha.
// Esse limite evita comandos bruscos quando a linha aparece perto da borda da imagem.
constexpr double kLineFollowerMaxTurnCorrection = 0.35;

// Potência usada para procurar a linha quando ela some em uma curva fechada.
// Mantenha este valor baixo: ele pode fazer o robô girar no próprio eixo.
constexpr double kLineFollowerLostLineTurnPower = 0.22;

// Erro mínimo, em pixels, para decidir o lado de busca quando a linha some.
// Abaixo deste valor, o robô ainda segue devagar para evitar giro sem direção.
constexpr double kLineFollowerLostLineDeadbandPixels = 35.0;

// Potência usada nas manobras temporizadas ao detectar marcações verdes.
// Teste com as rodas suspensas antes de aumentar esse valor.
constexpr double kGreenTurnPower = 0.35;

// Tempo, em milissegundos, para avançar um pouco antes de girar no verde.
// Isso ajuda o centro do robô a chegar na interseção antes da curva.
constexpr int kGreenApproachMs = 220;

// Tempo, em milissegundos, para curvas de 90 graus acionadas pelo verde.
// Ajuste este valor no robô real conforme velocidade, piso e bateria.
constexpr int kGreenTurnMs = 650;

// Tempo, em milissegundos, para meia-volta quando há verde dos dois lados.
// Deve ser maior que a curva simples, mas ainda precisa ser validado no piso.
constexpr int kGreenUTurnMs = 1200;

// Tempo, em milissegundos, para ignorar o mesmo verde após concluir uma manobra.
// Sem esse bloqueio, o robô pode detectar o mesmo marcador várias vezes.
constexpr int kGreenCooldownMs = 900;

// Tempo de espera, em milissegundos, após exportar um GPIO no Linux.
// A pasta /sys/class/gpio/gpioN pode levar um instante para aparecer.
constexpr int kGpioExportDelayMs = 100;

// Limites seguros para comandos de motor.
// O dashboard pode enviar valores fora da faixa, então o código limita antes
// de atualizar o estado do robô ou acionar os drivers de motor.
constexpr double kMinMotorOutput = -1.0;
constexpr double kMaxMotorOutput = 1.0;

// Zona morta do motor.
// Comandos com módulo menor que este valor são tratados como parada.
constexpr double kMotorDeadband = 0.05;

// Dispositivo UART usado pela Raspberry Pi para falar com a ESP32.
// Em uma Raspberry Pi comum, /dev/serial0 usa GPIO14 como TXD e GPIO15 como RXD.
constexpr const char* kEsp32SerialPort = "/dev/serial0";

// Velocidade da UART entre Raspberry Pi e ESP32, em bits por segundo.
// O sketch da ESP32 deve usar o mesmo valor para evitar comandos corrompidos.
constexpr int kEsp32SerialBaudRate = 115200;

// Tempo máximo, em milissegundos, para considerar recente a telemetria da ESP32.
// Se esse tempo estourar, o dashboard mostra os sensores como desatualizados.
constexpr int kEsp32TelemetryTimeoutMs = 1000;

// Tempo máximo, em milissegundos, que a ESP32 deve aceitar sem novo comando.
// Este valor fica documentado aqui e deve ser mantido igual no sketch da ESP32.
constexpr int kEsp32MotorCommandTimeoutMs = 500;

// Pinos BCM da Raspberry Pi usados pela UART de hardware com a ESP32.
// O TX da Raspberry deve ir ao RX2 da ESP32, e o RX da Raspberry deve vir do TX2.
constexpr int kRaspberryUartTxPin = 14;
constexpr int kRaspberryUartRxPin = 15;

// Pinos da ESP32 usados pela UART2 ligada à Raspberry Pi.
// Ajuste o sketch da ESP32 se a placa usar outros pinos para RX2 e TX2.
constexpr int kEsp32UartRx2Pin = 16;
constexpr int kEsp32UartTx2Pin = 17;

// Pinos da ESP32 conectados aos drivers BTS7960.
// Cada motor usa um enable comum e dois PWM: RPWM para um sentido e LPWM para o sentido oposto.
constexpr int kEsp32LeftEnablePin = 15;
constexpr int kEsp32LeftRpwmPin = 14;
constexpr int kEsp32LeftLpwmPin = 5;
constexpr int kEsp32RightEnablePin = 2;
constexpr int kEsp32RightRpwmPin = 4;
constexpr int kEsp32RightLpwmPin = 33;

// Pinos I2C da ESP32 usados pelo MPU6050.
constexpr int kEsp32MpuSdaPin = 21;
constexpr int kEsp32MpuSclPin = 22;

// Pinos da ESP32 usados pelo sensor ultrassônico frontal.
constexpr int kEsp32UltrasonicTrigPin = 25;
constexpr int kEsp32UltrasonicEchoPin = 35;
}
