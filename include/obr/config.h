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

// Maior referência operacional aceita antes da correção pelos encoders.
// A margem até 1,0 evita trabalhar continuamente no limite absoluto do PWM.
constexpr double kOperationalMaximumReferencePower = 0.97;

static_assert(kOperationalMinimumMotorPower > 0.0 &&
                  kOperationalMinimumMotorPower < kOperationalMaximumReferencePower,
              "A potência mínima deve caber na faixa operacional.");

// O sincronismo atua somente quando os dois lados avançam ou recuam juntos.
// Ele reduz gradualmente o lado mais rápido, mas o PWM corrigido nunca pode
// ficar abaixo do piso operacional de 0,65 enquanto o comando for diferente de zero.
constexpr int kEncoderSyncTelemetryMaxAgeMs = 250;
constexpr double kEncoderSyncMinimumRateCountsPerSecond = 100.0;
constexpr double kEncoderSyncMinimumAppliedPower = 0.10;
constexpr double kEncoderSyncEfficiencyFilterAlpha = 0.25;
constexpr int kEncoderSyncWarmupSamples = 3;
constexpr double kEncoderSyncMaximumScaleStepPerSample = 0.03;
// Mesmo na referência máxima, esta escala produz exatamente o piso de 0,65.
// Para referências menores, o MotorController calcula um limite ainda maior.
constexpr double kEncoderSyncMinimumScale =
    kOperationalMinimumMotorPower / kOperationalMaximumReferencePower;
constexpr double kEncoderSyncEfficiencyDeadbandRatio = 0.02;

static_assert(kEncoderSyncTelemetryMaxAgeMs > 0 &&
                  kEncoderSyncMinimumRateCountsPerSecond > 0.0 &&
                  kEncoderSyncMinimumAppliedPower > 0.0,
              "O sincronismo exige telemetria recente e movimento mensurável.");
static_assert(kEncoderSyncEfficiencyFilterAlpha > 0.0 &&
                  kEncoderSyncEfficiencyFilterAlpha <= 1.0 &&
                  kEncoderSyncWarmupSamples > 0,
              "O filtro e a aquisição do sincronismo devem ser positivos.");
static_assert(kEncoderSyncMaximumScaleStepPerSample > 0.0 &&
                  kEncoderSyncMinimumScale > 0.0 &&
                  kEncoderSyncMinimumScale <= 1.0 &&
                  kEncoderSyncEfficiencyDeadbandRatio >= 0.0,
              "Os limites da correção automática devem permanecer seguros.");

// Tempo máximo, em milissegundos, para aceitar dados da câmera no modo autônomo.
// Se a câmera travar ou parar de atualizar o JSON, o robô deve parar.
constexpr int kCameraStatusTimeoutMs = 400;

// Menor referência operacional do seguidor de linha. Os dois lados partem de
// 0,65 e o sincronismo reduz automaticamente somente o conjunto mais rápido.
constexpr double kLineFollowerBasePower = kOperationalMinimumMotorPower;

// Ganho aplicado ao erro lateral da linha, em pixels do frame principal.
// Aumentar corrige deslocamentos mais rápido, mas pode ampliar oscilações.
constexpr double kLineFollowerPositionGain = 0.0008;

// Ganho secundário aplicado ao heading local, em graus. A posição lateral deve
// dominar a centralização; um ganho alto aqui pode mandar o robô para o lado
// oposto quando a linha cruza a imagem em perspectiva.
constexpr double kLineFollowerHeadingGain = 0.0002;

// Quando posição e heading discordam, o heading pode cancelar no máximo metade
// da correção lateral. Isso garante que o robô primeiro volte para cima da linha.
constexpr double kLineFollowerOpposingHeadingLimitRatio = 0.50;

static_assert(kLineFollowerOpposingHeadingLimitRatio >= 0.0 &&
                  kLineFollowerOpposingHeadingLimitRatio <= 1.0,
              "O limite relativo do heading deve permanecer entre zero e um.");

// Acréscimo máximo experimental aplicado somente ao lado externo da curva suave.
// O valor inicial de 0,01 mantém o seguidor o mais lento possível para calibrar
// a visão primeiro. Aumentar fortalece a correção e também acelera o lado externo.
constexpr double kLineFollowerMaxTurnCorrection = 0.01;

// Pesos das amostras novas nos filtros de posição e heading. Valores menores
// suavizam ruído, mas aumentam o atraso da resposta do robô.
constexpr double kLineFollowerPositionFilterAlpha = 0.20;
constexpr double kLineFollowerHeadingFilterAlpha = 0.25;

// Erros menores que esta quantidade de pixels não geram correção. Acima do
// limite, apenas o excesso é controlado para a saída crescer sem degrau.
constexpr double kLineFollowerErrorDeadbandPixels = 6.0;

// Headings menores que este valor, em graus, não geram correção. A zona morta
// evita que pequenas variações da regressão local façam o robô tremer na reta.
constexpr double kLineFollowerHeadingDeadbandDegrees = 2.0;

// Variação lógica máxima da correção a cada ciclo de 20 ms. Cinco ciclos são
// necessários para chegar ao diferencial máximo de 0,01, evitando zigue-zague.
constexpr double kLineFollowerCorrectionSlewPerCycle = 0.002;

// Diferencial mínimo usado depois da tolerância inicial sem CurrentPath. Os dois
// lados continuam para frente; somente o lado externo acelera para preservar o
// último ângulo confiável sem transformar a busca em giro no próprio eixo.
constexpr double kLineLostMinimumSteeringCorrection =
    kLineFollowerMaxTurnCorrection;

// Confiança mínima normalizada para aceitar o CurrentPath publicado pela câmera.
// Aumentar rejeita geometrias fracas; diminuir aceita pistas mais degradadas.
constexpr double kLineFollowerMinimumPathConfidence = 0.35;

// Quantidade de status válidos exigida antes do primeiro comando de movimento.
// Isso impede que um único quadro instável arme a busca lateral na partida.
constexpr int kInitialLineAcquireFrames = 5;

// Referências experimentais de velocidade por situação. Nesta primeira versão
// todas preservam 0,65; os valores só devem mudar após ensaio físico controlado.
constexpr double kLineFollowerStraightPower = kLineFollowerBasePower;
constexpr double kLineFollowerApproachPower = kLineFollowerBasePower;
constexpr double kLineFollowerCornerPower = kLineFollowerBasePower;

// Quantidade de status distintos necessária para confirmar um corner e tolerância
// a falhas antes/depois do latch. O status da câmera é publicado a 20 Hz.
constexpr int kEventConfirmFrames = 3;
constexpr int kEventMaxMissFrames = 3;
constexpr int kEventLatchedMaxMissFrames = 8;

// Confiança mínima e proximidades normalizadas do Preview. Zero representa o topo
// da ROI e um representa a base. Estes limites são experimentais, não métricos.
// Aumentar a ActionProximity faz o Corner Anchor chegar mais perto do robô antes
// do giro; diminuir antecipa a execução e exige validação cuidadosa no piso.
constexpr double kEventMinimumConfidence = 0.48;
constexpr double kEventApproachProximity = 0.52;
constexpr double kEventActionProximity = 0.72;
constexpr double kEventExitProximity = 0.45;

// Tempo máximo, em milissegundos, para manter um evento latched sem atualização.
// Reduzir evita eventos antigos; aumentar tolera mais oclusões perto do robô.
constexpr int kEventMaximumAgeMs = 2500;

// Tempo, em milissegundos, para ignorar o corner recém-consumido depois da
// readquisição. Aumentar evita repetição; valores excessivos podem ocultar o próximo corner.
constexpr int kEventCooldownMs = 700;

// Quantidade de status válidos exigida para devolver o controle normal após uma
// curva e limites experimentais aceitos durante a readquisição visual.
constexpr int kReacquireFrames = 3;
constexpr double kReacquireMinimumPathConfidence = 0.45;
constexpr double kReacquireMaxPositionErrorPixels = 85.0;
constexpr double kReacquireMaxHeadingErrorDegrees = 32.0;
constexpr int kReacquireTimeoutMs = 1800;

// Durante dois status inválidos, o robô mantém exatamente o último arco confiável.
// Depois disso, reforça o mesmo sentido de busca. O timeout longo continua sendo
// a proteção contra movimento indefinido quando a pista realmente desaparece.
constexpr int kLineLostGraceFrames = 2;
constexpr int kLineLostSearchTimeoutMs = 3000;

// Limites experimentais do giro visual de um corner. O tempo mínimo impede que
// a linha antiga seja aceita antes de o robô começar a girar; o máximo evita giro infinito.
constexpr int kCornerTurnMinimumMs = 180;
constexpr int kCornerTurnTimeoutMs = 1800;
constexpr double kCornerTurnPower = kLineFollowerCornerPower;

// Distâncias percorridas antes e depois do giro visual de um corner. O avanço
// posiciona o eixo traseiro na curva; a ré reposiciona o robô sobre o novo
// segmento antes de recalcular a trajetória. Ambos usam o menor dos encoders.
constexpr double kCornerAdvanceDistanceCm = 15.0;
constexpr double kCornerReverseDistanceCm = 5.0;

// Tempo, em milissegundos, com PWM zero nas transições da manobra. A pausa antes
// da ré separa a inércia do giro; a pausa final estabiliza a leitura da linha.
constexpr int kCornerReverseStartSettleMs = 200;
constexpr int kCornerTranslationSettleMs = 200;

// Tempo máximo, em milissegundos, para cada deslocamento de 15 cm. O limite é
// independente para frente e ré e impede movimento indefinido se um encoder falhar.
constexpr int kCornerTranslationTimeoutMs = 5000;

// Contracomando do lado interno durante corner, busca e readquisição visual.
// Os dois lados recebem a potência mínima em sentidos opostos, produzindo giro
// no próprio eixo. Reduzir abaixo do mínimo não diminui o PWM por causa do perfil.
constexpr double kLineFollowerCounterTurnPower = kLineFollowerCornerPower;

static_assert(kEventApproachProximity > kEventExitProximity &&
                  kEventActionProximity > kEventApproachProximity &&
                  kEventActionProximity <= 1.0,
              "As zonas normalizadas do evento devem manter ordem e histerese.");
static_assert(kInitialLineAcquireFrames > 0,
              "A aquisição inicial deve exigir pelo menos um status válido.");
static_assert(kCornerAdvanceDistanceCm > 0.0 &&
                  kCornerReverseDistanceCm > 0.0 &&
                  kCornerReverseStartSettleMs >= 0 &&
                  kCornerTranslationSettleMs >= 0 &&
                  kCornerTranslationTimeoutMs >
                      kCornerReverseStartSettleMs + kCornerTranslationSettleMs,
              "Os deslocamentos do corner devem ter distâncias e timeout positivos.");
static_assert(kLineLostSearchTimeoutMs > 0 &&
                  kCornerTurnTimeoutMs > kCornerTurnMinimumMs,
              "Os timeouts visuais devem permitir movimento limitado e seguro.");

// Potência usada no lado externo das manobras com marcações verdes.
// O lado interno recebe o contracomando acima para que os dois lados girem.
constexpr double kGreenTurnPower = kOperationalMinimumMotorPower;

// Confirmação temporal e zonas normalizadas do marcador verde. Os valores são
// experimentais e impedem que um verde distante execute uma manobra imediatamente.
constexpr int kGreenConfirmFrames = 3;
constexpr int kGreenMaxMissFrames = 3;
constexpr int kGreenLatchedMaxMissFrames = 8;
constexpr double kGreenMinimumConfidence = 0.45;
constexpr double kGreenApproachProximity = kEventApproachProximity;
constexpr double kGreenActionProximity = kEventActionProximity;
constexpr int kGreenMaximumAgeMs = 2500;

// Tempo, em milissegundos, para avançar um pouco antes de girar no verde.
// Foi reduzido para compensar a nova base de 0,65 e preservar a distância aproximada.
constexpr int kGreenApproachMs = 170;

// Tempo inicial, em milissegundos, para curvas acionadas pelo verde. O valor foi
// reduzido porque os dois lados agora giram em sentidos opostos.
constexpr int kGreenTurnMs = 350;

// Tempo inicial, em milissegundos, para meia-volta quando há verde dos dois lados.
// O valor também considera a rotação simultânea dos dois lados.
constexpr int kGreenUTurnMs = 650;

static_assert(kGreenApproachMs + kGreenUTurnMs < kCornerTurnTimeoutMs,
              "A meia-volta verde deve terminar antes do timeout do giro visual.");

// Tempo, em milissegundos, para ignorar o mesmo verde após concluir uma manobra.
// Sem esse bloqueio, o robô pode detectar o mesmo marcador várias vezes.
constexpr int kGreenCooldownMs = 900;

// Ângulo-alvo, em graus, da missão de teste que gira o robô para a direita.
constexpr double kTurn90TargetDegrees = 90.0;

// Margem, em graus, usada para parar antes de ultrapassar demais o alvo.
// Ajuste após testar a inércia real das rodas no piso da competição.
constexpr double kTurn90StopToleranceDegrees = 2.0;

// Comando lógico usado durante todo o giro de 90 graus e nas correções.
// O perfil operacional transforma 0,01 em 0,65 nos dois lados; como eles giram
// em sentidos opostos, o sincronismo por encoder permanece desativado.
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

// Calibração empírica informada no teste: 3600 contagens produziram
// 18,7 cm de deslocamento com rodas de 68 mm de diâmetro.
constexpr double kEncoderCalibrationCounts = 3600.0;
constexpr double kEncoderCalibrationDistanceCm = 18.7;
constexpr double kEncoderCountsPerCentimeter =
    kEncoderCalibrationCounts / kEncoderCalibrationDistanceCm;

// Distância inicial e faixa aceitas pelo modo de percurso por encoder.
// O limite evita comandos acidentais excessivamente longos pelo dashboard.
constexpr double kDriveDistanceDefaultTargetCm = 20.0;
constexpr double kDriveDistanceMinimumTargetCm = 1.0;
constexpr double kDriveDistanceMaximumTargetCm = 300.0;

// Comando lógico para andar em linha reta no teste de distância. O perfil parte
// de 0,65 / 0,65 e o sincronismo reduz o lado mecanicamente mais rápido.
constexpr double kDriveDistanceCommandPower = 0.01;

// Horizonte, em segundos, somado à idade da telemetria para prever quantas
// contagens ainda ocorrerão antes de o robô parar por inércia.
constexpr double kDriveDistanceBrakePredictionSeconds = 0.14;

// Tempo sem PWM antes de registrar o resultado final dos dois encoders.
constexpr int kDriveDistanceSettleMs = 300;

// Margem, em centímetros, aceita após a estabilização. Se ainda faltar
// mais que isso, a missão aplica uma correção curta.
constexpr double kDriveDistanceToleranceCm = 0.5;

// Duração e quantidade máxima das correções de distância.
constexpr int kDriveDistanceCorrectionPulseMs = 60;
constexpr int kDriveDistanceMaximumCorrectionPulses = 3;

// Idade máxima da amostra usada para decidir a parada por distância.
constexpr int kDriveDistanceEncoderFreshnessMs = 300;

// Tempo máximo sem avanço dos dois encoders durante um comando de movimento.
// Se um lado não responder, a missão para em vez de percorrer distância indefinida.
constexpr int kDriveDistanceStallTimeoutMs = 1500;
constexpr double kDriveDistanceMinimumProgressCounts = 10.0;

// Tempo máximo da missão. Evita movimento indefinido se um encoder falhar.
constexpr int kDriveDistanceTimeoutMs = 60000;

static_assert(kEncoderCountsPerCentimeter > 0.0,
              "A calibração do encoder deve produzir contagens por centímetro positivas.");
static_assert(kDriveDistanceMinimumTargetCm > 0.0 &&
                  kDriveDistanceMinimumTargetCm < kDriveDistanceMaximumTargetCm,
              "A faixa da missão de distância deve ser válida.");

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

// Limites compartilhados com o firmware da ESP32 para mensagens temporárias na
// OLED. Textos maiores são cortados antes do envio para preservar a linha UART.
constexpr int kRemoteOledTitleMaxLength = 12;
constexpr int kRemoteOledLineMaxLength = 20;
constexpr int kRemoteOledMinimumDurationMs = 500;
constexpr int kRemoteOledMaximumDurationMs = 30000;

// Intervalo do heartbeat que mantém a OLED fora da animação de inicialização.
// A ESP32 tolera três períodos antes de considerar a Raspberry indisponível.
constexpr int kRaspberrySystemStatusHeartbeatMs = 1000;

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
