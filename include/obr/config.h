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

// Faixa normalizada do protocolo de motor: -1,0 é ré total e 1,0 é frente total.
// Esses limites impedem que comandos inválidos cheguem ao PWM da ESP32.
constexpr double kMinMotorOutput = -1.0;
constexpr double kMaxMotorOutput = 1.0;

// Menor potência operacional usada para mover qualquer lado do robô.
// Zero continua sendo parada real; comandos não nulos menores são elevados a 0,65.
constexpr double kOperationalMinimumMotorPower = 0.65;

// O lado direito precisa de 0,67 para acompanhar o lado esquerdo em 0,65.
// O ganho proporcional preserva essa relação também em outras velocidades e sentidos.
constexpr double kRightMotorCalibrationGain = 0.67 / 0.65;

// Maior referência operacional antes da compensação do lado direito.
// Esse limite mantém a calibração dentro do PWM máximo de 100%.
constexpr double kOperationalMaximumReferencePower =
    kMaxMotorOutput / kRightMotorCalibrationGain;

static_assert(kOperationalMinimumMotorPower > 0.0 &&
                  kOperationalMinimumMotorPower < kOperationalMaximumReferencePower,
              "A potência mínima operacional deve caber na faixa calibrada.");
static_assert(kRightMotorCalibrationGain >= 1.0,
              "O ganho direito deve representar o lado que precisa de mais PWM.");
static_assert(kOperationalMinimumMotorPower * kRightMotorCalibrationGain > 0.6699 &&
                  kOperationalMinimumMotorPower * kRightMotorCalibrationGain < 0.6701,
              "A calibração mínima direita deve resultar em 0,67.");

// Tempo máximo, em milissegundos, para aceitar dados da câmera no modo autônomo.
// Se a câmera travar ou parar de atualizar o JSON, o robô deve parar.
constexpr int kCameraStatusTimeoutMs = 400;

// Menor referência operacional do seguidor de linha.
// Na reta, ela produz 0,65 à esquerda e aproximadamente 0,67 à direita.
constexpr double kLineFollowerBasePower = kOperationalMinimumMotorPower;

// Ganho proporcional aplicado ao erro horizontal da linha, em pixels.
// O filtro e o limite de variação abaixo reduzem oscilações causadas pela câmera.
constexpr double kLineFollowerTurnGain = 0.0008;

// Acréscimo máximo aplicado somente ao lado externo da curva.
// A base nunca é reduzida, e 0,30 mantém a referência abaixo do limite calibrado.
constexpr double kLineFollowerMaxTurnCorrection = 0.30;

// Peso da amostra nova no filtro exponencial do erro da câmera. Valores menores
// deixam a direção mais suave, mas aumentam o atraso para entrar nas curvas.
constexpr double kLineFollowerErrorFilterAlpha = 0.20;

// Erros menores que esta quantidade de pixels não geram correção. A zona morta
// impede que ruído próximo ao centro faça o robô alternar esquerda e direita.
constexpr double kLineFollowerErrorDeadbandPixels = 18.0;

// Variação lógica máxima da correção a cada ciclo de 20 ms. Esse limite evita
// trancos mesmo quando a câmera muda a posição da linha entre dois quadros.
constexpr double kLineFollowerCorrectionSlewPerCycle = 0.015;

// Potência usada para procurar a linha quando ela some em uma curva fechada.
// É igual ao mínimo operacional porque valores menores não movem o conjunto com confiança.
constexpr double kLineFollowerLostLineTurnPower = kOperationalMinimumMotorPower;

// Erro mínimo, em pixels, para decidir o lado de busca quando a linha some.
// Abaixo deste valor, o robô ainda segue devagar para evitar giro sem direção.
constexpr double kLineFollowerLostLineDeadbandPixels = 35.0;

// Potência usada nas manobras temporizadas ao detectar marcações verdes.
// As durações precisam ser validadas novamente porque o giro agora parte de 0,65.
constexpr double kGreenTurnPower = kOperationalMinimumMotorPower;

// Tempo, em milissegundos, para avançar um pouco antes de girar no verde.
// Foi reduzido para compensar a nova base de 0,65 e preservar a distância aproximada.
constexpr int kGreenApproachMs = 170;

// Tempo inicial, em milissegundos, para curvas acionadas pelo verde.
// A proporção 0,35/0,65 preserva aproximadamente o impulso da configuração anterior.
constexpr int kGreenTurnMs = 350;

// Tempo inicial, em milissegundos, para meia-volta quando há verde dos dois lados.
// Foi reduzido junto com a curva simples e ainda precisa ser validado no piso.
constexpr int kGreenUTurnMs = 650;

// Tempo, em milissegundos, para ignorar o mesmo verde após concluir uma manobra.
// Sem esse bloqueio, o robô pode detectar o mesmo marcador várias vezes.
constexpr int kGreenCooldownMs = 900;

// Ângulo-alvo, em graus, da missão de teste que gira o robô para a direita.
constexpr double kTurn90TargetDegrees = 90.0;

// Margem, em graus, usada para parar antes de ultrapassar demais o alvo.
// Ajuste após testar a inércia real das rodas no piso da competição.
constexpr double kTurn90StopToleranceDegrees = 2.0;

// Comando lógico usado durante todo o giro de 90 graus e nas correções.
// O perfil operacional transforma 0,01 em 0,65 à esquerda e 0,67 à direita.
constexpr double kTurn90CommandPower = 0.01;

static_assert(kTurn90CommandPower > 0.0 && kTurn90CommandPower <= kMaxMotorOutput,
              "O comando do giro deve permanecer na faixa normalizada.");

// Tempo de inércia, em segundos, somado à idade da telemetria na projeção angular.
// A projeção corta o PWM antes do alvo para compensar movimento e atraso da UART.
constexpr double kTurn90BrakePredictionSeconds = 0.16;

// Limite, em graus, da antecipação de frenagem. Ele impede que um pico isolado
// do giroscópio faça o robô parar cedo demais.
constexpr double kTurn90MaximumBrakeLeadDegrees = 18.0;

// Tempo, em milissegundos, sem PWM para a inércia terminar antes de conferir o yaw.
constexpr int kTurn90SettleMs = 180;

// Duração, em milissegundos, de cada correção curta para completar
// um giro insuficiente ou reduzir uma ultrapassagem do alvo.
constexpr int kTurn90CorrectionPulseMs = 60;

// Quantidade máxima de pulsos de correção. O limite evita insistir no
// movimento se o yaw não responder como esperado.
constexpr int kTurn90MaximumCorrectionPulses = 3;

// Velocidade angular máxima, em graus por segundo, para considerar o robô estabilizado.
constexpr double kTurn90StationaryRateDegPerSec = 3.0;

// Idade máxima, em milissegundos, da amostra do MPU6050 usada para controlar o giro.
// O painel aceita telemetria mais antiga, mas movimento autônomo exige dado recente.
constexpr int kTurn90ImuFreshnessMs = 200;

// Tempo máximo, em milissegundos, permitido para o giro de teste.
// Inclui frenagem, estabilização e correções; se o MPU6050 falhar ou o robô
// travar, a missão para ao atingir esse limite.
constexpr int kTurn90TimeoutMs = 5000;

// Tempo de espera, em milissegundos, após exportar um GPIO no Linux.
// A pasta /sys/class/gpio/gpioN pode levar um instante para aparecer.
constexpr int kGpioExportDelayMs = 100;

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
// O TX da Raspberry deve ir ao RX da ESP32, e o RX da Raspberry deve vir do TX.
constexpr int kRaspberryUartTxPin = 14;
constexpr int kRaspberryUartRxPin = 15;

// GPIO BCM da Raspberry Pi ligado ao LED de sistema pronto.
// O LED é ativo em HIGH e só acende quando a UART recebe telemetria recente da ESP32.
constexpr int kRaspberryReadyLedPin = 26;

// Pinos da UART0 da ESP32 reservados para a integração futura com a Raspberry.
// Eles também são usados durante a gravação da ESP32 e não devem ser forçados externamente.
constexpr int kEsp32UartRxPin = 3;
constexpr int kEsp32UartTxPin = 1;

// Entradas dos DRV8833. Cada par controla os dois motores do respectivo lado.
constexpr int kEsp32LeftMotorIn1Pin = 5;
constexpr int kEsp32LeftMotorIn2Pin = 18;
constexpr int kEsp32RightMotorIn1Pin = 16;
constexpr int kEsp32RightMotorIn2Pin = 17;

// Pino ligado ao nSLEEP do DRV8833. LOW mantém todas as pontes H desligadas.
constexpr int kEsp32MotorSleepPin = 26;

// Pinos I2C da ESP32 usados pelo MPU6050, PCA9685 e SSD1306.
constexpr int kEsp32I2cSdaPin = 13;
constexpr int kEsp32I2cSclPin = 14;

// Pinos da ESP32 usados pelo sensor ultrassônico frontal.
constexpr int kEsp32UltrasonicTrigPin = 32;
constexpr int kEsp32UltrasonicEchoPin = 33;

// Pinos dos encoders em quadrature conforme o lado físico validado na PCB.
constexpr int kEsp32LeftEncoderAPin = 22;
constexpr int kEsp32LeftEncoderBPin = 23;
constexpr int kEsp32RightEncoderAPin = 19;
constexpr int kEsp32RightEncoderBPin = 21;

// Botão de partida e entrada ADC1 do divisor de tensão da bateria.
constexpr int kEsp32StartButtonPin = 27;
constexpr int kEsp32BatteryAdcPin = 36;
}
