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

// Arquivo JSON atualizado pelo script da câmera com a saúde da captura.
// O FPS mostrado no dashboard vem deste arquivo para não misturar câmera com telemetria do robô.
constexpr const char* kCameraStatusPath = "/tmp/obr_camera_status.json";

// Arquivo JSON rápido com a última medição visual calculada pelo processo Python.
// A Missão Principal usa esta fonte para seguir e recuperar a linha.
constexpr const char* kCameraLineStatusPath = "/dev/shm/obr_line_status.json";

// CSV temporário com um registro por frame do seguimento normal da linha.
// O arquivo fica fora da RAM compartilhada para continuar disponível após
// encerrar a missão e poder ser copiado diretamente da Raspberry Pi.
constexpr const char* kCurveDiagnosticsPath =
    "/tmp/obr_curve_diagnostics.csv";

// Intervalo máximo, em milissegundos, para a thread auxiliar descarregar o CSV.
// A escrita fora do loop de controle evita atrasar o envio periódico aos motores.
constexpr int kCurveDiagnosticsFlushIntervalMs = 1000;

// Quantidade de frames que antecipa um flush antes do intervalo periódico.
// Um lote pequeno limita perdas em uma queda sem escrever a cada frame.
constexpr int kCurveDiagnosticsFlushFrames = 30;

// O dashboard altera somente este pequeno IPC para solicitar a câmera inferior.
// O gerenciador encerra o processo Python e remove a visão publicada quando
// recebe zero, mantendo a Missão Principal bloqueada com segurança.
constexpr const char* kLineCameraControlPath =
    "/dev/shm/obr_line_camera_enabled";
constexpr const char* kLineCameraTemporaryControlPath =
    "/dev/shm/obr_line_camera_enabled.tmp";

// Idade máxima, em milissegundos, aceita para uma medição visual rápida.
// Amostras mais antigas são marcadas como indisponíveis e têm seus valores zerados.
constexpr int kCameraLineStatusTimeoutMs = 125;

// Porta local do stream MJPEG gerado pelo script Python da câmera.
// O dashboard acessa esse vídeo pelo proxy /camera-stream.mjpg na porta principal.
constexpr int kCameraStreamPort = 8090;

// Caminho HTTP do stream MJPEG dentro do script Python da câmera.
constexpr const char* kCameraStreamPath = "/stream.mjpg";

// O processo frontal captura e processa continuamente, mesmo sem cliente de vídeo.
// Esta porta transmite o overlay 960x540 sem controlar motores ou a câmera inferior.
constexpr int kForwardCameraStreamPort = 8091;
constexpr const char* kForwardCameraStreamPath = "/stream.mjpg";

// O dashboard altera somente este pequeno IPC para disponibilizar o stream da CAM1.
// O valor zero desliga a visualização, mas preserva o processamento frontal a 30 FPS.
constexpr const char* kForwardCameraControlPath =
    "/dev/shm/obr_forward_camera_enabled";
constexpr const char* kForwardCameraTemporaryControlPath =
    "/dev/shm/obr_forward_camera_enabled.tmp";
constexpr const char* kForwardCameraStatusPath =
    "/tmp/obr_forward_camera_status.json";

// Arquivo JSON rápido publicado continuamente pela câmera frontal.
// Este IPC é apenas uma fonte auxiliar e nunca substitui os gates de segurança
// nem o estado da câmera inferior.
constexpr const char* kForwardLineStatusPath =
    "/dev/shm/obr_forward_line_status.json";

// Idade máxima, em milissegundos, aceita para a leitura frontal.
// Uma amostra mais antiga perde autoridade imediatamente e não pode manter
// nem o seguimento frontal nem uma decisão de linha encontrada.
constexpr int kForwardLineStatusTimeoutMs = 125;

// Potência simétrica usada somente na busca frontal com direção já confirmada
// pela câmera inferior. O sinal é aplicado conforme LEFT ou RIGHT.
constexpr double kForwardAssistSearchSpinPower = 0.72;

// Limite angular absoluto, em graus, de uma tentativa completa de busca.
// O valor é um teto de segurança; encontrar a linha encerra o giro antes dele.
constexpr double kForwardAssistMaximumSearchDegrees = 65.0;

// Limiar normalizado usado somente para extrair LEFT/RIGHT das posições
// trusted recebidas pelo Forward Assist. O valor permanece igual ao limiar
// existente na câmera inferior e não altera o detector visual.
constexpr double kForwardAssistDirectionPositionThreshold = 0.20;

// Quantidade de frames MEDIUM novos, consecutivos e no mesmo lado necessária
// para inverter uma direção já latched quando FAR não oferece direção válida.
constexpr int kForwardAssistMediumFlipConfirmationFrames = 2;

// Quantidade de frames inferiores novos e consecutivos exigidos para devolver
// a autoridade depois que a câmera frontal começou a carregar o robô.
constexpr int kForwardAssistBottomStableFrames = 2;

// Quantidade de frames inferiores novos sem trust nem steering NORMAL exigidos
// antes de iniciar SEARCH_SPIN. Dois frames rejeitam uma perda isolada sem
// prolongar excessivamente a transição quando a linha realmente desaparece.
constexpr int kForwardAssistBottomLossFrames = 2;

// Limites do mapper virtual NORMAL existente na câmera inferior. A leitura
// frontal publica o resultado desse mesmo mapper e a Raspberry rejeita qualquer
// comando auxiliar fora desta faixa no FORWARD_FOLLOW.
constexpr double kForwardAssistNormalMinimumPower = 0.66;
constexpr double kForwardAssistNormalMaximumPower = 0.82;

// O Fusion pode reduzir a roda interna até a potência de pivot enquanto mantém
// um target geométrico válido. Estes limites não liberam GREEN, GAP ou recovery.
constexpr double kForwardAssistFusionMinimumPower = -0.72;
constexpr double kForwardAssistFusionMaximumPower = 0.85;

static_assert(kForwardAssistSearchSpinPower > 0.0 &&
                  kForwardAssistSearchSpinPower <= 1.0,
              "A busca frontal deve permanecer na faixa normalizada.");
static_assert(kForwardAssistMaximumSearchDegrees > 0.0 &&
                  kForwardAssistMaximumSearchDegrees <= 180.0 &&
                  kForwardAssistDirectionPositionThreshold > 0.0 &&
                  kForwardAssistDirectionPositionThreshold <= 1.0 &&
                  kForwardAssistMediumFlipConfirmationFrames > 0 &&
                  kForwardAssistBottomStableFrames > 0 &&
                  kForwardAssistBottomLossFrames > 0,
              "Os limites da assistência frontal devem ser positivos.");
static_assert(kForwardAssistNormalMinimumPower > 0.0 &&
                  kForwardAssistNormalMinimumPower <=
                      kForwardAssistNormalMaximumPower &&
                  kForwardAssistNormalMaximumPower <= 1.0,
              "O mapper frontal deve permanecer no intervalo NORMAL.");
static_assert(kForwardAssistFusionMinimumPower >= -1.0 &&
                  kForwardAssistFusionMinimumPower < 0.0 &&
                  kForwardAssistFusionMinimumPower <=
                      kForwardAssistFusionMaximumPower &&
                  kForwardAssistFusionMaximumPower <= 1.0,
              "O mapper Fusion deve permanecer no intervalo normalizado.");

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

// Potência mínima para iniciar uma roda que estava parada.
// Este valor foi validado fisicamente; reduzi-lo pode impedir a partida do motor.
constexpr double kMotorStartMinimumPower = 0.67;

// Potência mínima para manter uma roda que os encoders já confirmaram em movimento.
// Ela permite desacelerar a roda interna nas curvas sem voltar ao piso de partida.
constexpr double kMotorRunMinimumPower = 0.61;

// Taxa mínima, em contagens por segundo, que confirma movimento durante a partida.
// Ela é menor que o limite do sincronismo porque confirmar rotação não exige uma
// medição de eficiência tão precisa quanto corrigir a assimetria entre os lados.
constexpr double kMotorRunConfirmationMinimumRateCountsPerSecond = 20.0;

// Quantidade de amostras novas e válidas dos encoders para trocar STARTING por RUNNING.
// A confirmação evita liberar 0,61 por um pico isolado ou ruído de telemetria.
constexpr int kMotorRunConfirmationSamples = 2;

// Quantidade de amostras inválidas consecutivas que faz uma roda voltar a STARTING.
// A histerese evita alternância rápida, mas volta ao piso de partida se a roda parar.
constexpr int kMotorRunLossSamples = 3;

// Maior referência operacional aceita antes da correção pelos encoders.
// O valor coincide com o limite absoluto do protocolo para liberar todo o PWM.
constexpr double kOperationalMaximumReferencePower = 1.0;

static_assert(kMotorRunMinimumPower > 0.0 &&
                  kMotorRunMinimumPower <= kMotorStartMinimumPower &&
                  kMotorStartMinimumPower < kOperationalMaximumReferencePower,
              "Os pisos de partida e movimento devem caber na faixa operacional.");
static_assert(kMotorRunConfirmationMinimumRateCountsPerSecond > 0.0 &&
                  kMotorRunConfirmationSamples > 0 && kMotorRunLossSamples > 0,
              "A confirmação de movimento pelos encoders deve ser positiva.");

// O sincronismo atua somente quando os dois lados avançam ou recuam juntos.
// Ele reduz gradualmente o lado mais rápido, mas o PWM corrigido nunca pode
// ficar abaixo do piso de execução de 0,61 enquanto o comando for diferente de zero.
constexpr int kEncoderSyncTelemetryMaxAgeMs = 250;
constexpr double kEncoderSyncMinimumRateCountsPerSecond = 100.0;
constexpr double kEncoderSyncMinimumAppliedPower = 0.10;
constexpr double kEncoderSyncEfficiencyFilterAlpha = 0.25;
constexpr int kEncoderSyncWarmupSamples = 3;
constexpr double kEncoderSyncMaximumScaleStepPerSample = 0.03;
// Mesmo na referência máxima, esta escala produz exatamente o piso de 0,61.
// Para referências menores, o MotorController calcula um limite ainda maior.
constexpr double kEncoderSyncMinimumScale =
    kMotorRunMinimumPower / kOperationalMaximumReferencePower;
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

// Tempo máximo, em milissegundos, para considerar a captura da câmera pronta.
// Um status mais antigo apaga o indicador de prontidão do sistema.
constexpr int kCameraStatusTimeoutMs = 400;

// Ângulo-alvo, em graus, da missão de teste que gira o robô para a direita.
constexpr double kTurn90TargetDegrees = 90.0;

// Margem, em graus, usada para parar antes de ultrapassar demais o alvo.
// Ajuste após testar a inércia real das rodas no piso da competição.
constexpr double kTurn90StopToleranceDegrees = 2.0;

// Potência simétrica aplicada aos dois motores durante os giros por IMU.
// O valor 0,75 garante um pivot forte e previsível, sem depender dos pisos
// START/RUN usados no seguimento normal. Afeta a missão de teste de 90°.
constexpr double kTurn90CommandPower = 0.75;

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

// O retorno de 180 graus usa o mesmo controle, mas recebe mais tempo para
// concluir o dobro do deslocamento angular e suas correções.
constexpr int kTurn180TimeoutMs = 8000;

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

// Controle fechado exclusivo do teste autônomo de distância. Ele parte reto em
// 0,70 / 0,70 e corrige pelo erro acumulado entre os encoders, reduzindo o lado
// adiantado e aumentando o atrasado. Isso evita depender de uma compensação fixa
// quando a resposta mecânica muda entre tentativas.
constexpr double kDriveDistanceBaseCommandPower = 0.70;
constexpr double kDriveDistanceMinimumCommandPower = kMotorRunMinimumPower;
constexpr double kDriveDistanceMaximumCommandPower = 0.90;
constexpr double kDriveDistanceBalanceDeadbandCm = 0.25;
constexpr double kDriveDistanceBalanceGainPerCm = 0.06;
constexpr double kDriveDistanceMaximumBalanceCorrection = 0.12;

// Diferença máxima tolerada, em centímetros, entre os dois encoders durante o
// teste de distância. Três amostras novas dão tempo ao controle fechado reagir,
// mas ainda impedem que uma roda continue puxando o robô para um giro grande.
constexpr double kDriveDistanceMaximumSideDifferenceCm = 3.0;
constexpr int kDriveDistanceDifferenceConfirmationSamples = 3;

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
static_assert(kDriveDistanceBaseCommandPower >= kMotorStartMinimumPower &&
                  kDriveDistanceBaseCommandPower <= kMaxMotorOutput &&
                  kDriveDistanceMinimumCommandPower >= kMotorRunMinimumPower &&
                  kDriveDistanceMinimumCommandPower <=
                      kDriveDistanceBaseCommandPower &&
                  kDriveDistanceMaximumCommandPower >=
                      kDriveDistanceBaseCommandPower &&
                  kDriveDistanceMaximumCommandPower <= kMaxMotorOutput &&
                  kDriveDistanceBalanceDeadbandCm >= 0.0 &&
                  kDriveDistanceBalanceGainPerCm > 0.0 &&
                  kDriveDistanceMaximumBalanceCorrection > 0.0 &&
                  kDriveDistanceMaximumSideDifferenceCm > 0.0 &&
                  kDriveDistanceDifferenceConfirmationSamples > 0,
              "O controle fechado da missão de distância deve permanecer seguro.");

// Sequência configurável do retorno sinalizado por dois marcadores verdes.
// Ao reconhecer o retorno, o robô permanece parado antes de começar a alinhar.
// O atraso não bloqueia o loop, mantendo E-Stop e telemetria ativos.
constexpr int kGreenTurnAroundRecognitionDelayMs = 1000;
// Deslocamento normalizado máximo aceito simultaneamente no NEAR e MEDIUM.
// Reduzir este valor exige um alinhamento visual mais preciso antes do avanço.
constexpr double kGreenTurnAroundCenteringTolerance = 0.20;
// Potência normalizada do SPIN usado somente para alinhar NEAR e MEDIUM.
constexpr double kGreenTurnAroundCenteringPower = 0.69;
// Tempo máximo, em milissegundos, da centralização visual.
// O limite impede um giro indefinido se a linha permanecer fora do centro.
constexpr int kGreenTurnAroundCenteringTimeoutMs = 3000;
// Tempo parado, em milissegundos, depois do alinhamento e antes do avanço.
// Essa pausa permite que o robô estabilize sem carregar o SPIN para a sequência.
constexpr int kGreenTurnAroundPostCenteringDelayMs = 1000;
// Distância, em centímetros, percorrida antes de iniciar o giro por IMU.
constexpr double kGreenTurnAroundForwardDistanceCm = 13.0;
// Potência normalizada usada exclusivamente no avanço após reconhecer o
// retorno de 180°.
// O valor 0,69 independe da potência base do segue-linha e não representa cm/s.
constexpr double kGreenTurnAroundForwardPower = 0.69;
// Tempo parado, em milissegundos, entre o avanço e o início do giro.
constexpr int kGreenTurnAroundForwardSettleMs = 250;
// Idade máxima, em milissegundos, aceita para os dados dos encoders.
// O avanço só é interrompido por encoder quando a telemetria fica ausente.
constexpr int kGreenTurnAroundEncoderDataTimeoutMs = 1000;

// Limite absoluto de segurança, em milissegundos, para o avanço configurado.
// Ele não controla a distância; apenas impede movimento indefinido se uma
// contagem congelada continuar chegando como telemetria aparentemente válida.
constexpr int kGreenTurnAroundForwardSafetyTimeoutMs = 10000;
// Ângulo, em graus, controlado pelo MPU6050 antes da busca visual da linha.
constexpr double kGreenTurnAroundImuDegrees = 166.0;
// Erro angular máximo, em graus, aceito para concluir a etapa do IMU.
constexpr double kGreenTurnAroundImuToleranceDegrees = 8.0;
// Define o sentido do retorno: true gira à direita; false gira à esquerda.
constexpr bool kGreenTurnAroundTurnsRight = true;
// Potência normalizada do pivot que continua até o NEAR encontrar a linha.
constexpr double kGreenTurnAroundLineSearchPower = kTurn90CommandPower;
// Giro adicional máximo, em graus, permitido durante a busca visual da linha.
// Com o alvo atual, ele limita o retorno a aproximadamente 195° se a câmera
// não reconhecer o NEAR, em vez de permitir uma volta quase completa.
constexpr double kGreenTurnAroundLineSearchMaximumDegrees = 45.0;
// Quantidade de frames consecutivos com NEAR válido para retomar o seguidor.
constexpr int kGreenTurnAroundLineReacquireFrames = 2;
// Tempo máximo, em milissegundos, da busca visual após o giro pelo IMU.
constexpr int kGreenTurnAroundLineSearchTimeoutMs = 6000;

static_assert(kGreenTurnAroundRecognitionDelayMs > 0 &&
                  kGreenTurnAroundCenteringTolerance > 0.0 &&
                  kGreenTurnAroundCenteringTolerance <= 1.0 &&
                  kGreenTurnAroundCenteringPower >= kMotorStartMinimumPower &&
                  kGreenTurnAroundCenteringPower <= kMaxMotorOutput &&
                  kGreenTurnAroundCenteringTimeoutMs > 0 &&
                  kGreenTurnAroundPostCenteringDelayMs > 0 &&
                  kGreenTurnAroundForwardDistanceCm > 0.0 &&
                  kGreenTurnAroundForwardPower >= kMotorStartMinimumPower &&
                  kGreenTurnAroundForwardPower <= kMaxMotorOutput &&
                  kGreenTurnAroundForwardSettleMs >= 0 &&
                  kGreenTurnAroundEncoderDataTimeoutMs > 0 &&
                  kGreenTurnAroundForwardSafetyTimeoutMs >
                      kGreenTurnAroundEncoderDataTimeoutMs,
              "As etapas iniciais do retorno verde devem permanecer seguras.");
static_assert(kGreenTurnAroundImuDegrees > 0.0 &&
                  kGreenTurnAroundImuDegrees <= 180.0 &&
                  kGreenTurnAroundImuToleranceDegrees > 0.0 &&
                  kGreenTurnAroundImuToleranceDegrees <
                      kGreenTurnAroundImuDegrees &&
                  kGreenTurnAroundLineSearchPower > 0.0 &&
                  kGreenTurnAroundLineSearchPower <= kMaxMotorOutput &&
                  kGreenTurnAroundLineSearchMaximumDegrees > 0.0 &&
                  kGreenTurnAroundLineSearchMaximumDegrees < 180.0 &&
                  kGreenTurnAroundLineReacquireFrames > 0 &&
                  kGreenTurnAroundLineSearchTimeoutMs > 0,
              "O giro e a busca visual do retorno verde devem ser válidos.")
;
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
