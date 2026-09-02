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

// A missão de alinhamento habilita este IPC para executar HSV e Hough.
// O stream frontal pode continuar ativo sem gastar CPU detectando bolas.
constexpr const char* kForwardBallDetectionControlPath =
    "/dev/shm/obr_forward_ball_detection_enabled";
constexpr const char* kForwardBallDetectionTemporaryControlPath =
    "/dev/shm/obr_forward_ball_detection_enabled.tmp";

// Sequência da execução autônoma que deve possuir o alvo visual travado.
// Um novo valor força a visão a descartar qualquer bola da execução anterior.
constexpr const char* kForwardBallTargetSequenceControlPath =
    "/dev/shm/obr_forward_ball_target_sequence";
constexpr const char* kForwardBallTargetSequenceTemporaryControlPath =
    "/dev/shm/obr_forward_ball_target_sequence.tmp";

// IPC rápido, mantido em RAM, com a posição mais recente da bola frontal.
// Separá-lo do status completo permite que o controle receba um tx por frame.
constexpr const char* kForwardBallStatusPath =
    "/dev/shm/obr_forward_ball_status.json";

// Idade máxima, em milissegundos, aceita para a posição da bola frontal.
// Se a visão parar de publicar, o alinhamento interrompe os motores.
constexpr int kForwardBallStatusTimeoutMs = 500;

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

// Erro horizontal, em graus, aceito pela missão isolada de alinhamento.
// Uma zona morta evita alternar rapidamente o sentido perto do centro.
constexpr double kBallAlignmentDeadbandDegrees = 1.0;

// Tempo, em milissegundos, com PWM zerado ao cruzar o centro da imagem.
// A pausa reduz a ultrapassagem antes de permitir uma correção no sentido oposto.
constexpr int kBallAlignmentCrossingBrakeMs = 160;

// Ao entrar nesta faixa, em graus, o giro contínuo é interrompido antes do
// centro. A posição final passa a ser ajustada com pulsos curtos e verificações.
constexpr double kBallAlignmentFineCorrectionThresholdDegrees = 3.0;

// Duração, em milissegundos, de cada correção próxima do centro.
// A potência de partida ainda vence a inércia, mas o pulso limita o avanço.
constexpr int kBallAlignmentFineCorrectionPulseMs = 80;

// Frames novos dentro de ±1° exigidos depois que o robô estiver parado.
// Isso impede concluir por uma única leitura transitória durante a frenagem.
constexpr int kBallAlignmentStableFrames = 3;

// Taxa máxima, em contagens por segundo, para verificar o tx como posição final.
// Os encoders confirmam apenas a parada; não definem o objetivo angular.
constexpr double kBallAlignmentStationaryRateCountsPerSecond = 20.0;

// Tempo máximo, em milissegundos, para o alvo travado reaparecer.
// Durante toda a espera os motores permanecem zerados; ao exceder o limite,
// a execução falha e somente uma nova partida pode selecionar outra bola.
constexpr int kBallAlignmentTargetLossTimeoutMs = 1000;

// Potência normalizada usada para vencer a inércia no início do pivot.
constexpr double kBallAlignmentStartPower = 0.70;

// Limites do controle proporcional depois que os encoders confirmam movimento.
// Perto do centro, o piso de execução reduz a inércia sem parar uma roda.
constexpr double kBallAlignmentMinimumRunPower = kMotorRunMinimumPower;
constexpr double kBallAlignmentMaximumRunPower = 0.68;

// Erro, em graus, a partir do qual o proporcional usa a potência máxima de giro.
// Reduzir este valor torna a aproximação mais agressiva perto do centro.
constexpr double kBallAlignmentFullPowerErrorDegrees = 12.0;

// Distância visual, em centímetros, na qual a aproximação termina.
// Um valor menor aproxima mais o robô da bola e deve ser validado no piso real.
constexpr double kBallApproachStopDistanceCm = 5.0;

// Potência central usada para avançar enquanto o tx corrige a trajetória.
// A correção diferencial acelera um lado e reduz o outro simultaneamente.
constexpr double kBallApproachBasePower = 0.70;

// Maior correção diferencial aplicada durante o avanço.
// O limite mantém a roda interna no piso de movimento e evita virar no lugar.
constexpr double kBallApproachMaximumSteeringCorrection = 0.09;

// Erro de tx, em graus, que aplica a correção diferencial máxima.
// Erros menores produzem correções proporcionais mais suaves.
constexpr double kBallApproachFullSteeringErrorDegrees = 10.0;

static_assert(kBallAlignmentDeadbandDegrees > 0.0 &&
                  kBallAlignmentFullPowerErrorDegrees >
                      kBallAlignmentDeadbandDegrees &&
                  kBallAlignmentStartPower >=
                      kBallAlignmentMaximumRunPower &&
                  kBallAlignmentMaximumRunPower >=
                      kBallAlignmentMinimumRunPower &&
                  kBallAlignmentMinimumRunPower >=
                      kMotorRunMinimumPower &&
                  kBallAlignmentStartPower <= kMaxMotorOutput &&
                  kBallAlignmentCrossingBrakeMs > 0 &&
                  kBallAlignmentFineCorrectionThresholdDegrees >
                      kBallAlignmentDeadbandDegrees &&
                  kBallAlignmentFineCorrectionPulseMs > 0 &&
                  kBallAlignmentStableFrames > 0 &&
                  kBallAlignmentStationaryRateCountsPerSecond >= 0.0 &&
                  kBallAlignmentTargetLossTimeoutMs > 0 &&
                  kBallApproachStopDistanceCm > 0.0 &&
                  kBallApproachBasePower >= kMotorStartMinimumPower &&
                  kBallApproachBasePower +
                          kBallApproachMaximumSteeringCorrection <=
                      kMaxMotorOutput &&
                  kBallApproachBasePower -
                          kBallApproachMaximumSteeringCorrection >=
                      kMotorRunMinimumPower &&
                  kBallApproachMaximumSteeringCorrection > 0.0 &&
                  kBallApproachFullSteeringErrorDegrees >
                      kBallAlignmentDeadbandDegrees,
              "A missão de alinhamento deve preservar limites seguros.");

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
// Primeiro o robô para, avança pelos encoders, gira 125° pelo MPU6050 e
// continua no mesmo sentido até reencontrar a linha ou atingir cerca de 200°.
// Tempo parado, em milissegundos, após reconhecer o retorno.
// Esta espera encerra o comando anterior antes de referenciar os encoders.
constexpr int kGreenTurnAroundRecognitionStopMs = 250;
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
// Ângulo aproximado, em graus, controlado pelo MPU6050 antes da busca visual.
// A câmera passa a decidir o fim da manobra somente depois desta rotação.
constexpr double kGreenTurnAroundImuDegrees = 125.0;
// Erro angular máximo, em graus, aceito para concluir a etapa do IMU.
constexpr double kGreenTurnAroundImuToleranceDegrees = 5.0;
// Define o sentido do retorno: true gira à direita; false gira à esquerda.
constexpr bool kGreenTurnAroundTurnsRight = true;
// Potência normalizada do pivot que continua até o NEAR encontrar a linha.
constexpr double kGreenTurnAroundLineSearchPower = kTurn90CommandPower;
// Potência usada depois que a linha entra pela lateral da faixa NEAR.
// A redução evita atravessar o sensor central entre dois frames da câmera.
constexpr double kGreenTurnAroundLineApproachPower =
    kMotorStartMinimumPower;
// Ângulo total máximo, em graus, permitido para o retorno.
// Ao atingir este valor sem confirmar a linha, o segue-linha reassume o controle.
constexpr double kGreenTurnAroundMaximumDegrees = 200.0;
// Giro adicional máximo, em graus, permitido durante a busca visual da linha.
// Somado aos 125° iniciais, limita a manobra a aproximadamente 200° mesmo
// quando a câmera não consegue confirmar novamente o centro da linha.
constexpr double kGreenTurnAroundLineSearchMaximumDegrees =
    kGreenTurnAroundMaximumDegrees - kGreenTurnAroundImuDegrees;
// Quantidade de frames consecutivos com NEAR válido para retomar o seguidor.
constexpr int kGreenTurnAroundLineReacquireFrames = 2;
// Tempo máximo, em milissegundos, da busca visual após o giro pelo IMU.
constexpr int kGreenTurnAroundLineSearchTimeoutMs = 6000;

static_assert(kGreenTurnAroundRecognitionStopMs > 0 &&
                  kGreenTurnAroundForwardDistanceCm > 0.0 &&
                  kGreenTurnAroundForwardPower >= kMotorStartMinimumPower &&
                  kGreenTurnAroundForwardPower <= kMaxMotorOutput &&
                  kGreenTurnAroundForwardSettleMs >= 0 &&
                  kGreenTurnAroundEncoderDataTimeoutMs > 0 &&
                  kGreenTurnAroundForwardSafetyTimeoutMs >
                      kGreenTurnAroundEncoderDataTimeoutMs,
              "O avanço inicial do retorno verde deve permanecer seguro.");
static_assert(kGreenTurnAroundImuDegrees > 0.0 &&
                  kGreenTurnAroundImuDegrees <= 180.0 &&
                  kGreenTurnAroundImuToleranceDegrees > 0.0 &&
                  kGreenTurnAroundImuToleranceDegrees <
                      kGreenTurnAroundImuDegrees &&
                  kGreenTurnAroundLineSearchPower > 0.0 &&
                  kGreenTurnAroundLineSearchPower <= kMaxMotorOutput &&
                  kGreenTurnAroundLineApproachPower >=
                      kMotorRunMinimumPower &&
                  kGreenTurnAroundLineApproachPower <=
                      kGreenTurnAroundLineSearchPower &&
                  kGreenTurnAroundMaximumDegrees >
                      kGreenTurnAroundImuDegrees &&
                  kGreenTurnAroundMaximumDegrees <= 360.0 &&
                  kGreenTurnAroundLineSearchMaximumDegrees > 0.0 &&
                  kGreenTurnAroundLineSearchMaximumDegrees <= 180.0 &&
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
