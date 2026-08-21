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

// O dashboard altera somente este pequeno IPC para solicitar a câmera inferior.
// O gerenciador encerra o processo Python e remove a visão publicada quando
// recebe zero, mantendo a Missão Principal bloqueada com segurança.
constexpr const char* kLineCameraControlPath =
    "/dev/shm/obr_line_camera_enabled";
constexpr const char* kLineCameraTemporaryControlPath =
    "/dev/shm/obr_line_camera_enabled.tmp";

// O trigger habilita até 300 quadros ou 60 segundos da auditoria de regressão.
// Estes arquivos são diagnósticos e não substituem o IPC normal da visão.
constexpr const char* kLineRegressionTraceRequestPath =
    "/dev/shm/obr_line_trace_request";
constexpr const char* kLineRegressionVisionSamplePath =
    "/dev/shm/obr_line_trace_vision_sample.csv";
constexpr const char* kLineRegressionTracePath =
    "/dev/shm/obr_line_regression_trace.csv";

// Idade máxima, em milissegundos, aceita para uma medição visual rápida.
// Amostras mais antigas são marcadas como indisponíveis e têm seus valores zerados.
constexpr int kCameraLineStatusTimeoutMs = 125;

// Porta local do stream MJPEG gerado pelo script Python da câmera.
// O dashboard acessa esse vídeo pelo proxy /camera-stream.mjpg na porta principal.
constexpr int kCameraStreamPort = 8090;

// Caminho HTTP do stream MJPEG dentro do script Python da câmera.
constexpr const char* kCameraStreamPath = "/stream.mjpg";

// O processo frontal permanece ocioso nesta porta enquanto a câmera está desligada.
// Quando ativado, ele transmite 960x540 sem publicar dados do segue-faixa.
constexpr int kForwardCameraStreamPort = 8091;
constexpr const char* kForwardCameraStreamPath = "/stream.mjpg";

// O dashboard altera somente este pequeno IPC para solicitar a CAM1.
// O valor zero fecha a câmera física e reduz o consumo durante a missão principal.
constexpr const char* kForwardCameraControlPath =
    "/dev/shm/obr_forward_camera_enabled";
constexpr const char* kForwardCameraTemporaryControlPath =
    "/dev/shm/obr_forward_camera_enabled.tmp";
constexpr const char* kForwardCameraStatusPath =
    "/tmp/obr_forward_camera_status.json";

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

// Potência fixa do pivot visual para uma curva geométrica da linha preta.
// A câmera encerra o giro pela nova geometria; reduzir para o piso RUN faria
// o robô perder reatividade justamente quando a curva ainda ocupa a imagem.
constexpr double kCorner90PivotStartPower = 0.70;

// O pivot precisa demonstrar giro físico pelos encoders logo após iniciar.
// O IMU não participa da decisão visual; uma roda sem movimento não pode
// manter contrarrotação indefinidamente.
constexpr int kCorner90MotionConfirmationTimeoutMs = 600;
constexpr double kCorner90MinimumEncoderRateCountsPerSecond = 20.0;

// Sem nova linha por este período, o pivot para os motores. O limite impede
// que uma perda visual deixe o robô contrarrotacionando indefinidamente.
constexpr int kCorner90LineLossTimeoutMs = 1200;

// Uma geometria de linha preta com confiança espacial alta decide o pivot no
// primeiro frame novo. A confiança agrega as leituras brutas da mesma imagem.
constexpr int kCorner90ConfirmationFrames = 1;
constexpr int kCorner90ConfirmationWindowFrames = 3;
constexpr double kBlackLineGeometryMinimumConfidence = 0.2;
// A saída do pivot exige uma trajetória longa e orientada para frente. Isto
// impede que uma parte inclinada da curva, ainda visível na câmera, entregue
// um comando reto ao Pure Pursuit antes de o robô apontar para a nova faixa.
constexpr int kBlackLineGeometryExitMinimumFitSamples = 7;
constexpr double kBlackLineGeometryExitMinimumLookahead = 0.60;
constexpr double kBlackLineGeometryExitMaximumHeadingDegrees = 20.0;
// Uma imagem nova da faixa substituindo o ramo antigo devolve imediatamente o
// controle ao Pure Pursuit. Não há alinhamento angular nem confirmação extra.
constexpr int kCorner90ExitAlignmentFrames = 1;

// Janela, em graus, da mudança de heading bruta que pode pedir pivot. Curvas
// abaixo de 30 graus permanecem no Pure Pursuit; acima de 140 graus exigem
// outra recuperação visual, pois não há direção confiável para contrarrotação.
constexpr double kCorner90MinimumStrongAngleDegrees = 30.0;
constexpr double kCorner90MaximumStrongAngleDegrees = 140.0;

// Erro lateral normalizado máximo aceito na decisão e na saída do cotovelo.
// A margem evita exigir alinhamento perfeito; o Pure Pursuit corrige o restante.
constexpr double kCorner90MaximumCenterError = 0.20;


// Depois que a Missão Principal já parou para confirmar o cotovelo, a mesma
// geometria pode deslocar até 30% do centro. Isso evita perder uma curva curta
// enquanto o robô termina de frear, sem afrouxar a entrada inicial de 20%.
constexpr double kCorner90ConfirmationMaximumCenterError = 0.30;

// Frames visuais novos exigidos para aceitar um marcador verde direcional.
// Durante a confirmação o robô permanece parado para não decidir um verde
// ambíguo ou dois cotovelos muito próximos enquanto ainda está avançando.
constexpr int kGreenTurnConfirmationFrames = 2;

// Giro angular, em graus, executado para um marcador verde direcional.
// O alvo limitado evita pivot sem fim sobre o marcador; a nova faixa é
// confirmada pela câmera somente depois que o giro termina.
constexpr double kGreenDirectionalTurnTargetDegrees = 45.0;

// Depois do giro de 45°, a mesma observação verde não pode iniciar outra
// manobra durante esta janela. Isso deixa a rota nova assumir o controle sem
// repetir o pivot sobre o marcador ainda visível.
constexpr int kGreenTurnVisualHandoffCooldownMs = 1500;

// A rota nova precisa aparecer dentro deste tempo antes da sonda curta. Os
// motores ficam parados enquanto a visão não publicou NEAR/FAR nem uma
// trajetória forte; uma rota NEAR/FAR inicia a reaquisição limitada.
constexpr int kGreenTurnAcquireTimeoutMs = 800;

// Se não houver rota visual, a missão pode fazer uma única sonda curta e
// somente para frente. O maior deslocamento dos encoders limita cada roda a
// 20 mm; não há segunda tentativa, ré ou busca angular livre.
constexpr double kGreenTurnForwardProbeDistanceMm = 20.0;
constexpr double kGreenTurnForwardProbePower = 0.70;
constexpr int kGreenTurnForwardProbeEncoderFreshnessMs = 300;
constexpr int kGreenTurnForwardProbeEncoderStallTimeoutMs = 600;
constexpr double kGreenTurnForwardProbeMinimumProgressCounts = 10.0;

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
static_assert(kCorner90PivotStartPower >= kMotorStartMinimumPower &&
                  kCorner90PivotStartPower <= kOperationalMaximumReferencePower &&
                  kCorner90MotionConfirmationTimeoutMs > 0 &&
                  kCorner90LineLossTimeoutMs >
                      kCorner90MotionConfirmationTimeoutMs &&
                  kCorner90ConfirmationFrames > 0 &&
                  kCorner90ConfirmationFrames <=
                      kCorner90ConfirmationWindowFrames &&
                  kCorner90ExitAlignmentFrames > 0 &&
                  kCorner90MaximumCenterError > 0.0 &&
                  kCorner90MaximumCenterError <=
                      kCorner90ConfirmationMaximumCenterError &&
                  kCorner90ConfirmationMaximumCenterError < 1.0 &&
                  kCorner90MinimumStrongAngleDegrees > 0.0 &&
                  kCorner90MinimumStrongAngleDegrees <
                      kCorner90MaximumStrongAngleDegrees &&
                  kCorner90MaximumStrongAngleDegrees < 180.0,
              "Os limites de segurança do pivot devem ser coerentes.");
static_assert(kBlackLineGeometryMinimumConfidence > 0.0 &&
                  kBlackLineGeometryMinimumConfidence <= 1.0,
              "A confiança mínima da geometria deve estar entre zero e um.");
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

// Ângulo, em graus, do retorno comandado por dois marcadores verdes.
// O IMU encerra o giro; nenhum tempo fixo pode manter os motores ligados.
constexpr double kGreenTurnAroundTargetDegrees = 180.0;

// Margem, em graus, usada para parar antes de ultrapassar demais o alvo.
// Ajuste após testar a inércia real das rodas no piso da competição.
constexpr double kTurn90StopToleranceDegrees = 2.0;

// O giro direcional verde de 45 graus é apenas uma orientação inicial para a
// nova faixa. Aceitar até 12 graus de erro evita pulsos de correção inúteis e
// entrega cedo o controle ao Pure Pursuit, sem afrouxar 90 ou 180 graus.
constexpr double kGreenDirectionalTurnCompletionToleranceDegrees = 12.0;

// Potência simétrica aplicada aos dois motores durante os giros por IMU.
// O valor 0,75 garante um pivot forte e previsível, sem depender dos pisos
// START/RUN usados no seguimento normal. Afeta os giros verdes de 45° e 180°.
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

// Último recurso do segue-faixa: após perder as duas bandas, o robô mantém
// somente a direção visual já comprovada por no máximo 100 mm. Este valor não
// é uma busca livre e deve continuar curto para não atravessar um gap às cegas.
constexpr double kLineRecoveryMemoryMaximumDistanceMm = 100.0;

// A memória deve vir de uma imagem imediatamente anterior à perda. Três
// sequências acomodam o descompasso entre a câmera de 30 FPS e o loop de 20 ms.
constexpr int kLineRecoveryMemoryMaximumSourceSamples = 3;

// A maior roda durante a memória visual recebe no máximo esta potência. A
// normalização mantém o diferencial anterior sem criar uma nova curva sem visão.
constexpr double kLineRecoveryMemoryMaximumPower = 0.70;

// Telemetria dos dois encoders deve estar recente durante a memória visual.
// Este watchdog só detecta ausência total de progresso; não compara os lados,
// pois curvas reais registram naturalmente quantidades diferentes de contagens.
constexpr int kLineRecoveryMemoryEncoderFreshnessMs = 300;
constexpr int kLineRecoveryMemoryEncoderStallTimeoutMs = 600;
constexpr double kLineRecoveryMemoryMinimumProgressCounts = 10.0;

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

// Comando lógico de avanço reto durante o gap. O perfil operacional transforma
// este valor no piso de partida de 0,67 e mantém o sincronismo dos dois lados por encoder.
constexpr double kGapDriveCommandPower = 0.01;

static_assert(kEncoderCountsPerCentimeter > 0.0,
              "A calibração do encoder deve produzir contagens por centímetro positivas.");
static_assert(kLineRecoveryMemoryMaximumDistanceMm > 0.0 &&
                  kLineRecoveryMemoryMaximumPower >= kMotorStartMinimumPower &&
                  kLineRecoveryMemoryMaximumPower <= kMaxMotorOutput &&
                  kLineRecoveryMemoryMaximumSourceSamples > 0 &&
                  kLineRecoveryMemoryEncoderFreshnessMs > 0 &&
                  kLineRecoveryMemoryEncoderStallTimeoutMs > 0 &&
                  kLineRecoveryMemoryMinimumProgressCounts > 0.0,
              "Os limites da memória visual devem permanecer seguros.");
static_assert(kGreenTurnAroundTargetDegrees == 180.0,
              "O retorno verde duplo deve completar 180 graus.");
static_assert(kGreenDirectionalTurnTargetDegrees > 0.0 &&
                  kGreenDirectionalTurnTargetDegrees < kTurn90TargetDegrees &&
                  kGreenTurnVisualHandoffCooldownMs > 0 &&
                  kGreenTurnAcquireTimeoutMs > 0 &&
                  kGreenTurnForwardProbeDistanceMm > 0.0 &&
                  kGreenTurnForwardProbePower >= kMotorStartMinimumPower &&
                  kGreenTurnForwardProbePower <= kMaxMotorOutput &&
                  kGreenTurnForwardProbeEncoderFreshnessMs > 0 &&
                  kGreenTurnForwardProbeEncoderStallTimeoutMs > 0 &&
                  kGreenTurnForwardProbeMinimumProgressCounts > 0.0,
              "Os limites da manobra direcional pelo verde devem permanecer seguros.");
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
static_assert(kGapDriveCommandPower > 0.0 &&
                  kGapDriveCommandPower <= kMaxMotorOutput,
              "O comando da travessia de gap deve permanecer na faixa normalizada.");

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
