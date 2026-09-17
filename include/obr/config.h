#pragma once

#include <cstdint>

namespace config
{
// Fração mínima de vermelho na união dos sensores: 0,08 corresponde a 8%.
// Aumentar reduz a sensibilidade; diminuir pode confirmar manchas pequenas.
constexpr double kRedFinishMinRatio = 0.08;
// Frames consecutivos para confirmar vermelho ou rearmar após sair da faixa.
constexpr int kRedFinishConfirmFrames = 4;

// Porta HTTP usada pelo dashboard e pelo WebSocket.
// Se mudar este valor, atualize também a URL usada para acessar o robô.
constexpr int kDashboardPort = 8080;

// Intervalo, em milissegundos, entre envios de telemetria para o dashboard.
constexpr int kTelemetryPeriodMs = 500;

// Tempo, em milissegundos, durante o qual sensores e visão devem permanecer
// prontos antes de liberar uma nova execução autônoma após o boot ou uma queda.
constexpr int kSystemReadyStableMs = 2000;

// Intervalo mínimo, em milissegundos, entre eventos curtos aceitos do botão.
// Evita que ruído mecânico seja interpretado como dois comandos consecutivos.
constexpr int kStartButtonDuplicateGuardMs = 300;

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
// A inferência de prata pode terminar depois do frame da linha. Esta janela
// mantém a última decisão válida sem afrouxar o timeout geral da CAM0.
constexpr int kSilverClassifierStatusTimeoutMs = 500;
static_assert(kSilverClassifierStatusTimeoutMs > kCameraLineStatusTimeoutMs,
              "A janela da prata deve ser independente da leitura rápida da CAM0.");

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

// O gravador diagnóstico da CAM1 observa este IPC e permanece desligado quando
// o arquivo está ausente ou contém active=false. Ele nunca concede autoridade
// de movimento à câmera frontal.
constexpr const char* kForwardReacquisitionControlPath =
    "/dev/shm/obr_forward_reacquisition_capture.json";

// Cada sessão curta mantém imagens e JSONL dentro do diretório do projeto.
// O serviço usa /home/raspberry/OBR2026K como diretório de trabalho.
constexpr const char* kForwardReacquisitionSessionRoot =
    "logs/forward_reacquisition";

// Gate da percepção das áreas verde e vermelha da sala de resgate.
// Somente o modo isolado de validação deve mantê-lo ativo nesta etapa.
constexpr const char* kRescueZoneDetectionControlPath =
    "/dev/shm/obr_rescue_zone_detection_enabled";
constexpr const char* kRescueZoneDetectionTemporaryControlPath =
    "/dev/shm/obr_rescue_zone_detection_enabled.tmp";

// IPC atômico publicado pela CAM1 com resultados independentes por cor.
// A ausência do arquivo significa que não existe observação atual válida.
constexpr const char* kRescueZoneStatusPath =
    "/dev/shm/obr_rescue_zone_status.json";

// Idade máxima, em milissegundos, aceita pelo ALIGN_ZONE para comandar movimento.
// Um prazo curto impede que a geometria de um frame antigo mova o robô.
constexpr int kRescueZoneStatusTimeoutMs = 250;

// Idade máxima, em milissegundos, da distância exibida junto às áreas.
// Uma leitura mais antiga continua indisponível e nunca deve orientar movimento.
constexpr int kRescueZoneUltrasonicFreshnessMs = 300;

// Intervalo, em milissegundos, do IPC que leva o ultrassônico até a CAM1.
// Cinquenta milissegundos mantêm o overlay atual sem escrever a cada ciclo.
constexpr int kRescueZoneInputPublishIntervalMs = 50;

// Faixa física, em centímetros, aceita para a futura aproximação.
// Valores fora dela são publicados como inválidos e aparecem como "ULTRA --".
constexpr double kRescueZoneUltrasonicMinimumCm = 2.0;
constexpr double kRescueZoneUltrasonicMaximumCm = 400.0;

static_assert(kRescueZoneUltrasonicFreshnessMs > 0 &&
                  kRescueZoneInputPublishIntervalMs > 0 &&
                  kRescueZoneUltrasonicMinimumCm > 0.0 &&
                  kRescueZoneUltrasonicMaximumCm >
                      kRescueZoneUltrasonicMinimumCm,
              "A telemetria ultrassônica das áreas exige limites válidos.");

// Gate do detector YOLO de vítimas da câmera frontal. Fora das etapas que
// precisam procurar ou aproximar uma vítima, a inferência pesada fica desligada.
constexpr const char* kForwardBallDetectionControlPath =
    "/dev/shm/obr_forward_ball_detection_enabled";
constexpr const char* kForwardBallDetectionTemporaryControlPath =
    "/dev/shm/obr_forward_ball_detection_enabled.tmp";

// Identifica a execução autônoma dona do alvo. A troca atômica impede que uma
// vítima rastreada antes de Stop seja reutilizada depois de uma nova partida.
constexpr const char* kForwardBallTargetSequenceControlPath =
    "/dev/shm/obr_forward_ball_target_sequence";
constexpr const char* kForwardBallTargetSequenceTemporaryControlPath =
    "/dev/shm/obr_forward_ball_target_sequence.tmp";

// Contrato de resultado do detector, publicado somente durante o resgate.
constexpr const char* kForwardBallStatusPath =
    "/dev/shm/obr_forward_ball_status.json";

// Idade máxima, em milissegundos, aceita para alinhar com uma vítima.
// A expiração zera os motores no mesmo ciclo do controlador.
constexpr int kForwardBallStatusTimeoutMs = 500;

// Idade máxima, em milissegundos, aceita para a leitura frontal.
// Uma amostra antiga não valida GAP. O prazo correspondente do Python está em
// GAP_VALIDATION_CONFIG["source_timeout"], em segundos, no perfil da câmera.
constexpr int kForwardLineStatusTimeoutMs = 125;

// Recuperação frontal liberada somente depois da saída da sala de resgate.
// O percurso anterior continua usando integralmente o controle inferior.
// Potência simétrica aplicada conforme o último lado válido visto pela CAM1.
constexpr double kForwardAssistSearchSpinPower = 0.72;

// Limite angular absoluto, em graus, de uma tentativa completa de busca.
// O valor é um teto de segurança; encontrar a linha encerra o giro antes dele.
constexpr double kForwardAssistMaximumSearchDegrees = 65.0;
// Tempo máximo, em milissegundos, da recuperação frontal após a saída.
// O limite temporal complementa o limite angular caso a IMU fique indisponível.
constexpr int kForwardAssistRecoveryTimeoutMs = 3000;

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

// Limites ainda usados para validar o mapper NORMAL inferior no CameraMonitor.
// O nome histórico permanece; estes valores não autorizam controle frontal.
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
                  kForwardAssistRecoveryTimeoutMs > 0 &&
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

// Tempo máximo, em milissegundos, sem renovação do painel antes de remover os
// sinais dos servos no modo Manual. Este watchdog é separado dos motores para
// que ajustar braço, pulso ou garra nunca mantenha um comando de tração antigo.
constexpr int kManualServoCommandTimeoutMs = 2000;

// Intervalo, em milissegundos, do loop principal que aplica os comandos aos motores.
constexpr int kMainLoopPeriodMs = 20;

// Faixa normalizada do protocolo de motor: -1,0 é ré total e 1,0 é frente total.
// Esses limites impedem que comandos inválidos cheguem ao PWM da ESP32.
constexpr double kMinMotorOutput = -1.0;
constexpr double kMaxMotorOutput = 1.0;

// Compensa somente o seguimento NORMAL quando a inclinação recente da IMU
// indica subida ou descida. Pela convenção operacional atual, valores positivos
// representam a frente do robô levantada; a faixa intermediária não altera a potência.
constexpr double kLineFollowingUphillThresholdDeg = 4.0;
constexpr double kLineFollowingSteepUphillThresholdDeg = 4.0;
constexpr double kLineFollowingDownhillThresholdDeg = -6.0;
// A partir de 4 graus, a reta nominal passa de 0,75 para 0,85.
// O teto também limita a roda externa nas curvas durante a subida.
constexpr double kLineFollowingUphillPowerOffset = 0.05;
constexpr double kLineFollowingSteepUphillPowerOffset = 0.10;
constexpr double kLineFollowingUphillMaximumPower = 0.80;
constexpr double kLineFollowingSteepUphillMaximumPower = 0.85;
constexpr double kLineFollowingDownhillPowerOffset = -0.05;

static_assert(kLineFollowingUphillThresholdDeg > 0.0 &&
                  kLineFollowingSteepUphillThresholdDeg >=
                      kLineFollowingUphillThresholdDeg &&
                  kLineFollowingDownhillThresholdDeg < 0.0 &&
                  kLineFollowingUphillPowerOffset > 0.0 &&
                  kLineFollowingSteepUphillPowerOffset >
                      kLineFollowingUphillPowerOffset &&
                  kLineFollowingUphillMaximumPower > 0.0 &&
                  kLineFollowingUphillMaximumPower <
                      kLineFollowingSteepUphillMaximumPower &&
                  kLineFollowingSteepUphillMaximumPower <=
                      kMaxMotorOutput &&
                  kLineFollowingDownhillPowerOffset < 0.0,
              "A compensação de rampa deve respeitar os sentidos de subida e descida.");

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

// Avanço reto executado ao encontrar a faixa cinza pela primeira vez.
// A distância é medida pelos encoders e a potência baixa reduz o risco de
// atravessar rapidamente o limite antes da confirmação visual.
constexpr double kSilverEntryAdvanceDistanceCm = 5.0;
constexpr double kSilverEntryAdvancePower = 0.67;
// Tempo máximo, em milissegundos, para concluir o avanço inicial de 5 cm.
// Ao expirar, o segue-linha recupera o controle sem aplicar nova potência.
constexpr int kSilverEntryCandidateAdvanceTimeoutMs = 1500;
// Ré única executada após confirmar o cinza. A distância curta leva a faixa
// preta anterior ao sensor NEAR sem sair da região recém-identificada.
constexpr double kSilverEntryReverseDistanceCm = 3.0;
constexpr double kSilverEntryReversePower = 0.68;
// O alinhamento após a ré usa somente o sensor virtual NEAR, localizado na
// parte inferior da imagem. Duas leituras centrais evitam concluir por ruído.
constexpr double kSilverEntryNearCenterTolerance = 0.12;
constexpr int kSilverEntryNearStableFrames = 2;
constexpr double kSilverEntryNearCenteringPower = 0.69;
constexpr int kSilverEntryAlignmentTimeoutMs = 3000;
constexpr int kSilverEntryEncoderFreshnessMs = 300;
constexpr int kSilverEntryAdvanceTimeoutMs = 2500;

static_assert(kSilverEntryAdvanceDistanceCm > 0.0 &&
                  kSilverEntryAdvancePower >= kMotorStartMinimumPower &&
                  kSilverEntryAdvancePower <= kMaxMotorOutput &&
                  kSilverEntryReverseDistanceCm > 0.0 &&
                  kSilverEntryReversePower >= kMotorStartMinimumPower &&
                  kSilverEntryReversePower <= kMaxMotorOutput &&
                  kSilverEntryNearCenterTolerance > 0.0 &&
                  kSilverEntryNearCenterTolerance <= 1.0 &&
                  kSilverEntryNearStableFrames > 0 &&
                  kSilverEntryNearCenteringPower >= kMotorStartMinimumPower &&
                  kSilverEntryNearCenteringPower <= kMaxMotorOutput &&
                  kSilverEntryAlignmentTimeoutMs > 0 &&
                  kSilverEntryEncoderFreshnessMs > 0 &&
                  kSilverEntryAdvanceTimeoutMs > 0,
              "O avanço da entrada cinza deve permanecer em limites seguros.");

// Faixa central normalizada aceita pelo ALIGN_ZONE. O valor corresponde a
// dez por cento para cada lado do centro publicado pela visão frontal.
constexpr double kRescueZoneAlignDeadbandNormalized = 0.10;

// Dois frames distintos no centro evitam concluir por uma observação isolada.
constexpr int kRescueZoneAlignStableFrames = 2;

// Potência dos micro-pivôs usados tanto para revelar bounds quanto para
// centralizar o aim. O sinal é definido pela direção da correção visual.
// A busca e o alinhamento das zonas usam os mesmos pulsos já usados pelo YOLO.
constexpr double kRescueSearchTurnPower = 0.72;
// Duração, em milissegundos, que permite vencer a inércia após o comando UART.
constexpr int kRescueSearchPulseMs = 130;
// Pausa, em milissegundos, para estabilizar a imagem após cada micro-pivô.
constexpr int kRescueSearchSettlingMs = 100;
constexpr double kRescueZoneAlignTurnPower = kRescueSearchTurnPower;

// Duração do pulso e pausa mecânica fixa, em milissegundos. Nenhuma dessas
// etapas consulta encoder ou usa um pequeno setpoint angular da IMU.
constexpr int kRescueZoneAlignMicroPivotDurationMs = kRescueSearchPulseMs;
constexpr int kRescueZoneAlignSettleMs = kRescueSearchSettlingMs;

// Potência normalizada do pivot contínuo usado exclusivamente pelo SEARCH_ZONE.
// O valor baixo permite observar vários frames da CAM1 durante a varredura.
constexpr double kRescueZoneSearchTurnPower = 0.72;

static_assert(kRescueZoneAlignDeadbandNormalized > 0.0 &&
                  kRescueZoneAlignDeadbandNormalized < 1.0 &&
                  kRescueZoneAlignStableFrames > 0 &&
                  kRescueZoneAlignTurnPower >= kMotorStartMinimumPower &&
                  kRescueZoneAlignTurnPower <= kMaxMotorOutput &&
                  kRescueZoneAlignMicroPivotDurationMs > 0 &&
                  kRescueZoneAlignSettleMs > 0 &&
                  kRescueZoneSearchTurnPower >= kMotorStartMinimumPower &&
                  kRescueZoneSearchTurnPower <= kMaxMotorOutput,
              "Os limites do ALIGN_ZONE devem permanecer seguros.");

// Distâncias frontais, em centímetros, que selecionam as faixas de velocidade
// e concluem a aproximação. Reduzir o limite final aumenta o risco de colisão.
constexpr double kRescueZoneApproachFarDistanceCm = 25.0;
constexpr double kRescueZoneApproachNearDistanceCm = 12.0;
constexpr double kRescueZoneApproachStopDistanceCm = 6.0;

// Potências normalizadas da aproximação. A transição gradual reduz a inércia
// perto da área sem deixar os motores abaixo do piso operacional validado.
constexpr double kRescueZoneApproachFarPower = 0.85;
constexpr double kRescueZoneApproachMidPower = 0.75;
constexpr double kRescueZoneApproachNearPower = 0.70;

// Tempo, em milissegundos, do avanço final após alcançar a distância alvo.
// Esse deslocamento usa potência própria e termina obrigatoriamente com PWM zero.
constexpr int kRescueZoneApproachFinalAdvanceMs = 1500;
// Potência normalizada exclusiva do avanço final. Aumentar este valor aumenta
// o deslocamento e o esforço contra o triângulo durante o intervalo fixo.
constexpr double kRescueZoneApproachFinalAdvancePower = 0.75;

// Cobertura mínima da imagem pela zona alvo para concluir a aproximação.
// A câmera funciona como parada redundante quando o ULTRA perde o eco de perto.
constexpr double kRescueZoneApproachCameraStopCoverage = 0.70;

// Correção diferencial máxima aplicada para conservar o lockedHeading.
// O erro angular de referência aplica a correção completa nos dois lados.
constexpr double kRescueZoneApproachMaximumHeadingCorrection = 0.06;
constexpr double kRescueZoneApproachFullHeadingErrorDegrees = 10.0;

// Tempo máximo, em milissegundos, permitido para uma aproximação isolada.
// O timeout sempre encerra o movimento com PWM zero.
constexpr int kRescueZoneApproachTimeoutMs = 10000;

static_assert(kRescueZoneApproachFarDistanceCm >
                      kRescueZoneApproachNearDistanceCm &&
                  kRescueZoneApproachNearDistanceCm >
                      kRescueZoneApproachStopDistanceCm &&
                  kRescueZoneApproachStopDistanceCm >=
                      kRescueZoneUltrasonicMinimumCm &&
                  kRescueZoneApproachFarPower <= kMaxMotorOutput &&
                  kRescueZoneApproachFarPower >
                      kRescueZoneApproachMidPower &&
                  kRescueZoneApproachMidPower >
                      kRescueZoneApproachNearPower &&
                  kRescueZoneApproachNearPower >=
                      kMotorStartMinimumPower &&
                  kRescueZoneApproachMaximumHeadingCorrection > 0.0 &&
                  kRescueZoneApproachFarPower +
                          kRescueZoneApproachMaximumHeadingCorrection <=
                      kMaxMotorOutput &&
                  kRescueZoneApproachNearPower -
                          kRescueZoneApproachMaximumHeadingCorrection >=
                      kMotorRunMinimumPower &&
                  kRescueZoneApproachFullHeadingErrorDegrees > 0.0 &&
                  kRescueZoneApproachFinalAdvanceMs > 0 &&
                  kRescueZoneApproachFinalAdvancePower >= kMotorStartMinimumPower &&
                  kRescueZoneApproachFinalAdvancePower <= kMaxMotorOutput &&
                  kRescueZoneApproachCameraStopCoverage > 0.0 &&
                  kRescueZoneApproachCameraStopCoverage <= 1.0 &&
                  kRescueZoneApproachTimeoutMs > 0,
              "Os limites do APPROACH_ZONE devem permanecer seguros.");

// Avanço inicial, em centímetros, executado ao entrar na sala de resgate.
// O detector de vítimas permanece ligado durante todo o deslocamento.
constexpr double kRescueEntryAdvanceDistanceCm = 10.0;
constexpr double kRescueEntryAdvancePower = 0.70;

// Limites angulares, em graus, da busca de vítimas em relação ao heading de
// entrada. A segunda varredura amplia a área observada somente quando ±45°
// não encontram uma vítima do tipo solicitado.
constexpr double kRescueVictimFirstSweepDegrees = 45.0;
constexpr double kRescueVictimSecondSweepDegrees = 75.0;
constexpr double kRescueVictimSweepToleranceDegrees = 3.0;
// Mantém o robô parado entre frames de confirmação para não varrer além da vítima.
constexpr int kRescueVictimCandidateHoldMs = 500;
// Limite por tentativa, em milissegundos. A varredura maior recebe mais tempo
// para cruzar a sala; nenhum timeout autoriza insistir no mesmo lado sem limite.
constexpr int kRescueVictimFirstSweepTimeoutMs = 3000;
constexpr int kRescueVictimSecondSweepTimeoutMs = 8000;

// Avanço angular mínimo, em graus, esperado durante a busca contínua.
// Se a IMU permanecer dentro desta faixa pelo tempo abaixo, o robô pode estar
// preso em uma parede e inverte o giro para tentar se liberar.
constexpr double kRescueContinuousSearchMinimumProgressDegrees = 15.0;
constexpr int kRescueContinuousSearchStallTimeoutMs = 2000;

// A verificação das vítimas extras cobre no máximo uma volta completa.
// O tempo limite encerra a busca mesmo se o robô ficar preso e libera a rotina
// da saída, que aponta para os corners usando o heading do último triângulo.
constexpr double kRescueFinalVictimSearchDegrees = 360.0;
constexpr int kRescueFinalVictimSearchTimeoutMs = 20000;

// Ré feita depois de cada coleta para liberar a vítima da parede e criar espaço
// para movimentar o mecanismo. Os encoders limitam o percurso a 15 cm.
constexpr double kRescuePostCollectionReverseDistanceCm = 15.0;
constexpr double kRescuePostCollectionReversePower = 0.80;

// Ré feita depois de cada entrega. Vinte centímetros afastam o robô do
// triângulo antes de iniciar outra busca visual.
constexpr double kRescuePostDepositReverseDistanceCm = 20.0;
constexpr double kRescuePostDepositReversePower = 0.80;

// Ré, em centímetros, feita antes e durante a verificação final.
// Vale após depósito vermelho ou verde para iniciar a saída perto do centro.
constexpr double kRescueFinalDepositReverseDistanceCm = 40.0;

// Busca da saída: limites em graus, frames e milissegundos. Limites maiores
// ampliam as tentativas, mas aumentam o deslocamento antes de uma falha segura.
constexpr double kRescueExitScanDegrees = 30.0;
// Tolerância exclusiva, em graus, para manter a mesma candidata durante a aproximação.
// O valor maior absorve realinhamentos sem liberar direções já rejeitadas.
constexpr double kRescueExitTrackingToleranceDegrees = 30.0;
constexpr double kRescueExitRejectedToleranceDegrees = 15.0;
// A prata confirma um vão de entrada inteiro; este cone impede novas investidas próximas.
constexpr double kRescueExitSilverRejectedToleranceDegrees = 35.0;
// Variação máxima do heading entre frames usados para confirmar uma candidata distante.
constexpr double kRescueExitCandidateHeadingToleranceDegrees = 15.0;
constexpr int kRescueExitCandidateFrames = 3;
// Após ver uma candidata, observa a mesma direção por até 800 ms antes de
// girar novamente. Isso tolera frames perdidos sem aceitar ruído isolado.
constexpr int kRescueExitCandidateReviewMs = 800;
// Quatro frames novos apenas iniciam a validação pelo Fusion inferior.
// A missão continua ativa para dar prioridade à prata antes do handoff.
constexpr int kRescueExitAcquisitionFrames = 4;
// Duração, em milissegundos, de cada pivô contínuo usado para procurar a faixa.
constexpr int kRescueExitLineSearchSideMs = 650;
// Potência normalizada dos pivôs de busca. O valor precisa vencer o atrito do robô.
constexpr double kRescueExitLineSearchTurnPower = 0.75;
// Tempo máximo, em milissegundos, para concluir os pivôs de busca da faixa.
constexpr int kRescueExitLineSearchTimeoutMs = 12000;
// Reta curta, em centímetros, por encoders após o primeiro Fusion de uma faixa
// sem ramificações; evita seguir o braço lateral logo na borda.
constexpr double kRescueExitLineEntryAdvanceCm = 3.0;
// Distância mínima, em centímetros, percorrida com Fusion inferior estável
// antes do handoff. Nesse trecho, qualquer indício de prata para o robô.
constexpr double kRescueExitBottomValidationAdvanceCm = 25.0;
constexpr int kRescueExitSensorTimeoutMs = 2000;
// Tempo máximo, em milissegundos, para o supervisor reiniciar a CAM1 enquanto
// a saída permanece parada. IMU e encoders continuam usando o limite curto.
constexpr int kRescueExitCameraRecoveryTimeoutMs = 15000;
// A CAM1 opera perto de 8 FPS durante a análise completa. Esta janela exclusiva
// evita invalidar a rota entre frames sem afrouxar o timeout global de GAP.
constexpr int kRescueExitForwardStatusTimeoutMs = 400;
// Mantém por pouco tempo a última curva frontal confirmada. Depois deste prazo,
// os motores param antes que a candidata seja rejeitada pelo limite abaixo.
constexpr int kRescueExitGuidanceHoldMs = 300;
// Após parar por perda visual, espera até este prazo por uma imagem nova.
constexpr int kRescueExitReacquisitionWaitMs = 500;
constexpr int kRescueExitApproachTimeoutMs = 20000;
constexpr int kRescueExitTotalTimeoutMs = 120000;
// Potência exclusiva dos giros da busca da saída, na faixa normalizada dos motores.
constexpr double kRescueExitTurnPower = 0.75;
// A aproximação distante é reta; a curva só começa quando a fita chega ao
// limite inferior da CAM1. O valor é uma fração da altura do frame.
constexpr double kRescueExitApproachPower = 0.75;
constexpr double kRescueExitSteeringStartDepth = 0.85;
constexpr double kRescueExitSteeringDeadbandDegrees = 3.5;
// Perto da fita, a autoridade cresce suavemente sem arco fechado nem roda em ré.
constexpr double kRescueExitSteeringFullDegrees = 24.0;
constexpr double kRescueExitSteeringOuterPower = 0.78;
constexpr double kRescueExitSteeringInnerPower = 0.70;
// Exploração ativa usada somente após uma volta completa sem confirmação.
// Cada avanço é limitado por encoder e continua observando as duas câmeras.
constexpr double kRescueExitExplorationPower = 0.75;
constexpr double kRescueExitExplorationAttemptCm = 30.0;
constexpr double kRescueExitExplorationTotalCm = 60.0;
constexpr double kRescueExitExplorationRecoveryCm = 8.0;
constexpr double kRescueExitExplorationOffsetDegrees = 30.0;
constexpr int kRescueExitExplorationMaximumAttempts = 3;
// Yaws de avanço reto medidos a partir do alinhamento do último triângulo.
// O teste isolado usa os mesmos alvos para permitir a calibração na arena.
constexpr double kRescueExitFirstStraightYawDegrees = 58.0;
constexpr double kRescueExitSecondStraightYawDegrees = -100.0;
constexpr double kRescueExitThirdStraightYawDegrees = 58.0;
// A saída fica neste yaw relativo ao último triângulo centralizado.
constexpr double kRescueExitDirectYawDegrees = kRescueExitFirstStraightYawDegrees;
// Tempo parado, em milissegundos, em cada direção do teste isolado de yaw.
// A pausa permite conferir visualmente para qual quina o robô está apontando.
constexpr int kRescueCornerYawHoldMs = 2000;
// Absorve o robô fora do centro e o triângulo fora do centro da imagem.
constexpr double kRescueExitCornerGeometryToleranceDegrees = 24.0;
// A tentativa termina antes deste limite se qualquer Fusion válido aparecer.
constexpr double kRescueExitCornerAdvanceCm = 60.0;
constexpr int kRescueExitCornerVisionTimeoutMs = 2000;
// Recuperação única quando a proteção visual ou os encoders indicam parede no corner.
// A ré cria espaço, o primeiro giro busca o lado visto pela CAM1 e o segundo
// restaura o heading da abertura antes de continuar o avanço.
constexpr double kRescueExitCornerCollisionReverseCm = 15.0;
constexpr double kRescueExitCornerCollisionTurnDegrees = 15.0;
constexpr double kRescueExitCornerCollisionClearanceCm = 10.0;
constexpr double kRescueExitReverseMaximumCm = kRescueFinalDepositReverseDistanceCm;
// O heartbeat expira sem renovar autoridade visual após Stop ou queda da aplicação.
constexpr const char* kRescueExitControlPath = "/dev/shm/obr_rescue_exit_control.json";
constexpr int kRescueExitControlIntervalMs = 100;
constexpr int kRescueExitControlTimeoutMs = 500;
static_assert(kRescueExitControlIntervalMs < kRescueExitControlTimeoutMs &&
              kRescueExitRejectedToleranceDegrees < kRescueExitTrackingToleranceDegrees &&
              kRescueExitSilverRejectedToleranceDegrees > kRescueExitRejectedToleranceDegrees &&
              kRescueExitCandidateHeadingToleranceDegrees > 0.0 &&
              kRescueExitBottomValidationAdvanceCm > 0.0 &&
              kRescueExitBottomValidationAdvanceCm < kRescueExitCornerAdvanceCm &&
              kRescueExitCandidateReviewMs > kRescueExitForwardStatusTimeoutMs &&
              kRescueExitCandidateReviewMs < kRescueExitApproachTimeoutMs &&
              kRescueExitLineSearchSideMs > 0 &&
              kRescueExitLineSearchTurnPower >= kMotorStartMinimumPower &&
              kRescueExitLineSearchTurnPower <= kMaxMotorOutput &&
              kRescueExitLineSearchTimeoutMs > 0 &&
              kRescueExitLineEntryAdvanceCm > 0.0 &&
              kRescueExitLineEntryAdvanceCm < kRescueExitBottomValidationAdvanceCm &&
              kRescueExitCameraRecoveryTimeoutMs > kRescueExitSensorTimeoutMs &&
              kRescueExitForwardStatusTimeoutMs > kForwardLineStatusTimeoutMs &&
              kRescueExitGuidanceHoldMs < kRescueExitReacquisitionWaitMs &&
              kRescueExitReacquisitionWaitMs < kRescueExitForwardStatusTimeoutMs +
                                                        kRescueExitGuidanceHoldMs &&
              kRescueExitReacquisitionWaitMs < kRescueExitApproachTimeoutMs &&
              kRescueExitApproachTimeoutMs < kRescueExitTotalTimeoutMs &&
              kRescueExitTurnPower >= kMotorRunMinimumPower &&
              kRescueExitTurnPower <= kMaxMotorOutput &&
              kRescueExitSteeringStartDepth > 0.0 &&
              kRescueExitSteeringStartDepth < 1.0 &&
              kRescueExitSteeringDeadbandDegrees < kRescueExitSteeringFullDegrees &&
              kRescueExitSteeringInnerPower >= kMotorRunMinimumPower &&
              kRescueExitSteeringInnerPower <= kRescueExitApproachPower &&
              kRescueExitApproachPower <= kRescueExitSteeringOuterPower &&
              kRescueExitSteeringOuterPower <= kMaxMotorOutput &&
              kRescueExitExplorationPower >= kMotorStartMinimumPower &&
              kRescueExitExplorationPower <= kMaxMotorOutput &&
              kRescueExitExplorationAttemptCm > 0.0 &&
              kRescueExitExplorationTotalCm >= kRescueExitExplorationAttemptCm &&
              kRescueExitExplorationRecoveryCm > 0.0 &&
              kRescueExitExplorationRecoveryCm < kRescueExitExplorationAttemptCm &&
              kRescueExitExplorationOffsetDegrees == kRescueExitScanDegrees &&
              kRescueExitExplorationMaximumAttempts == 3 &&
              kRescueExitFirstStraightYawDegrees > -180.0 &&
              kRescueExitFirstStraightYawDegrees <= 180.0 &&
              kRescueExitSecondStraightYawDegrees < 0.0 &&
              kRescueExitSecondStraightYawDegrees > -180.0 &&
              kRescueExitThirdStraightYawDegrees > -180.0 &&
              kRescueExitThirdStraightYawDegrees <= 180.0 &&
              kRescueExitCornerGeometryToleranceDegrees > 0.0 &&
              kRescueExitCornerGeometryToleranceDegrees <
                  kRescueExitScanDegrees &&
              kRescueExitCornerAdvanceCm > 0.0 &&
              kRescueExitCornerCollisionReverseCm > 0.0 &&
              kRescueExitCornerCollisionReverseCm < kRescueExitCornerAdvanceCm &&
              kRescueExitCornerCollisionTurnDegrees > 0.0 &&
              kRescueExitCornerCollisionTurnDegrees < 45.0 &&
              kRescueExitCornerCollisionClearanceCm > 0.0 &&
              kRescueExitCornerCollisionClearanceCm <
                  kRescueExitCornerCollisionReverseCm &&
              kRescueExitCornerVisionTimeoutMs < kRescueExitTotalTimeoutMs,
              "Limites da busca da saída devem preservar as janelas e potências seguras.");


// Tempos máximos, em milissegundos, dos deslocamentos internos do resgate.
// A ausência de progresso ou de telemetria recente interrompe a missão antes.
constexpr int kRescueDistancePreparationTimeoutMs = 1200;
constexpr int kRescueDistanceStallTimeoutMs = 1500;
constexpr int kRescueDistanceTimeoutMs = 12000;
constexpr int kRescueDistanceSettleMs = 250;

// Micropulso aplicado somente no início de movimentos não retos da sala de
// resgate. A potência maior vence a inércia estática e volta ao comando original
// após 80 ms, sem alterar avanços ou recuos perfeitamente retos.
constexpr double kRescueDelicateMotionKickPower = 0.80;
constexpr int kRescueDelicateMotionKickDurationMs = 80;

// Margem angular estrita usada como referência interna da correção fina.
constexpr double kBallAlignmentDeadbandDegrees = 1.0;

// Margem visual, em graus, que permite iniciar a aproximação. O robô não
// precisa terminar toda a centralização parado porque continuará corrigindo
// suavemente o heading enquanto avança.
constexpr double kBallApproachStartToleranceDegrees = 5.0;

// Margem exclusiva da vítima que libera o avanço com correção contínua.
// O limite maior reduz o tempo parado no alinhamento inicial sem afetar a saída.
constexpr double kVictimApproachStartToleranceDegrees = 8.0;

// Metade do campo de visão horizontal de 62° usado para calcular o tx.
// Este limite normaliza o tempo dos pulsos sem alterar o cálculo da câmera.
constexpr double kBallAlignmentMaximumVisualErrorDegrees = 31.0;

// Tempo, em milissegundos, com PWM zerado antes de verificar novamente o tx.
// A pausa evita decidir enquanto a inércia ainda cruza o centro da imagem.
constexpr int kBallAlignmentCrossingBrakeMs = 100;

// Erros até este valor usam pulsos finos. Esta faixa é maior que a tolerância
// de aproximação para evitar voltar ao pulso grosso perto da transição.
constexpr double kBallAlignmentFineCorrectionThresholdDegrees = 12.0;

// Faixa de tempo útil, em milissegundos, do alinhamento grosso. O período do
// controle é de 20 ms; manter pulsos curtos reduz a ultrapassagem sem diminuir
// a potência necessária para vencer o atrito dos motores.
constexpr int kBallAlignmentCoarseMinimumPulseMs = 20;
constexpr int kBallAlignmentCoarseMaximumPulseMs = 40;

// Faixa de tempo útil, em milissegundos, da correção fina. O erro angular
// escolhe proporcionalmente um valor entre estes limites.
constexpr int kBallAlignmentFineMinimumPulseMs = 15;
constexpr int kBallAlignmentFineMaximumPulseMs = 30;

// Fração do erro visual que um único pulso pode percorrer segundo a IMU.
// Os limites impedem tanto pulsos imperceptíveis quanto a passagem pelo alvo.
constexpr double kBallAlignmentCoarseYawFraction = 0.15;
constexpr double kBallAlignmentCoarseMinimumYawDegrees = 0.4;
constexpr double kBallAlignmentCoarseMaximumYawDegrees = 1.3;
constexpr double kBallAlignmentFineYawFraction = 0.20;
constexpr double kBallAlignmentFineMinimumYawDegrees = 0.25;
constexpr double kBallAlignmentFineMaximumYawDegrees = 0.65;

// Tempo máximo, em milissegundos, para a ESP32 confirmar PWM e movimento.
// Se os encoders não mostrarem partida, a missão falha com os motores parados.
constexpr int kBallAlignmentPulseStartTimeoutMs = 300;

// Menor potência aplicada pela ESP32 aceita como confirmação do micro-pivô.
constexpr double kBallAlignmentAppliedPowerMinimum = 0.60;

// Potência usada somente até os encoders confirmarem que o pivô começou.
// Ela iguala o pulso de busca já validado e deixa de ser usada assim que as
// rodas vencem a inércia, limitando o risco de ultrapassar o centro.
constexpr double kBallAlignmentStartPower = 0.72;

// Taxa mínima, em contagens por segundo, que confirma movimento físico nas
// duas rodas. Confirmar apenas o PWM permitia encerrar o pulso com o robô parado.
constexpr double kBallAlignmentMovementMinimumRateCountsPerSecond = 20.0;

// Quantidade de frames novos e alinhados exigida antes da aproximação.
constexpr int kBallAlignmentStableFrames = 3;

// Taxa máxima dos encoders, em contagens por segundo, aceita como parada.
constexpr double kBallAlignmentStationaryRateCountsPerSecond = 20.0;

// Tempo máximo, em milissegundos, para o alvo travado reaparecer.
// Durante toda a perda, a saída dos motores permanece zerada.
// Dois segundos acomodam uma inferência lenta e a janela de reaquisição sem
// autorizar movimento com uma medição antiga.
constexpr int kBallAlignmentTargetLossTimeoutMs = 2000;

// Os pulsos usam somente a margem necessária para vencer o atrito. O fino fica
// ainda mais próximo do piso de partida para reduzir a ultrapassagem do centro.
constexpr double kBallAlignmentCoarsePulsePower = 0.69;
constexpr double kBallAlignmentFinePulsePower = 0.68;

// Distância frontal, em centímetros, que conclui a aproximação da vítima.
constexpr double kBallApproachStopDistanceCm = 5.0;

// Margem angular aceita quando a vítima já atingiu a distância de coleta.
// Muito perto, a caixa ocupa grande parte do frame e pequenos pivôs deixam de
// ser úteis; esta tolerância libera o avanço final sem aceitar um grande desvio.
constexpr double kBallCollectionNearAlignmentToleranceDegrees = 10.0;

// Distância adicional, em centímetros, percorrida depois que a câmera confirma
// a vítima próxima. O avanço por encoder garante contato com o coletor sem
// continuar dependendo de uma caixa que pode sair do campo de visão.
constexpr double kVictimCollectionAdvanceDistanceCm = 5.0;

// Potência e correção diferencial do avanço final. A potência fica próxima do
// piso de partida para reduzir o impacto, e os encoders mantêm o percurso reto.
constexpr double kVictimCollectionAdvancePower = 0.68;
constexpr double kVictimCollectionMaximumBalanceCorrection = 0.05;

// Critérios do avanço final. Diferença persistente ou ausência de progresso
// confirmam contato com a vítima ou parede e zeram o PWM antes da conclusão.
// O tempo total ainda limita a energização dos motores em caso de falha atípica.
constexpr double kVictimCollectionMaximumSideDifferenceCm = 1.5;
constexpr int kVictimCollectionDifferenceConfirmationSamples = 3;
constexpr int kVictimCollectionPreparationTimeoutMs = 1000;
constexpr int kVictimCollectionStallTimeoutMs = 1000;
constexpr int kVictimCollectionTimeoutMs = 3000;
constexpr int kVictimCollectionSettleMs = 200;

// Potência base e correção diferencial usadas durante a aproximação.
constexpr double kBallApproachBasePower = 0.70;
constexpr double kBallApproachMaximumSteeringCorrection = 0.09;

// Erros de heading menores que esta margem não alteram a potência dos motores.
// A margem pequena permite responder ao objetivo visual gradual sem oscilar.
constexpr double kBallApproachHeadingDeadbandDegrees = 0.3;

// Margem visual, em graus, considerada alinhada durante a aproximação.
// Fora dela, somente frames novos atualizam gradualmente o heading desejado.
constexpr double kBallApproachVisualAlignedToleranceDegrees = 2.0;

// Ganho e correção angular máxima, em graus por frame, usados para trazer a
// vítima de volta ao centro sem perseguir integralmente uma inferência atrasada.
constexpr double kBallApproachVisualHeadingGain = 0.40;
constexpr double kBallApproachMaximumHeadingAdjustmentDegrees = 3.0;

// Erro angular, em graus, que aplica a correção diferencial máxima.
constexpr double kBallApproachFullSteeringErrorDegrees = 3.0;

static_assert(kBallAlignmentDeadbandDegrees > 0.0 &&
                  kVictimApproachStartToleranceDegrees >
                      kBallAlignmentDeadbandDegrees &&
                  kBallAlignmentMaximumVisualErrorDegrees >
                      kBallAlignmentFineCorrectionThresholdDegrees &&
                  kBallAlignmentFineCorrectionThresholdDegrees >
                      kVictimApproachStartToleranceDegrees &&
                  kBallAlignmentCoarsePulsePower >=
                      kMotorStartMinimumPower &&
                  kBallAlignmentCoarsePulsePower <= kMaxMotorOutput &&
                  kBallAlignmentFinePulsePower >= kMotorStartMinimumPower &&
                  kBallAlignmentFinePulsePower <= kMaxMotorOutput &&
                  kBallAlignmentCrossingBrakeMs > 0 &&
                  kBallAlignmentFineCorrectionThresholdDegrees >
                      kBallAlignmentDeadbandDegrees &&
                  kBallAlignmentCoarseMinimumPulseMs > 0 &&
                  kBallAlignmentCoarseMaximumPulseMs >=
                      kBallAlignmentCoarseMinimumPulseMs &&
                  kBallAlignmentFineMinimumPulseMs > 0 &&
                  kBallAlignmentFineMaximumPulseMs >=
                      kBallAlignmentFineMinimumPulseMs &&
                  kBallAlignmentFineMaximumPulseMs <
                      kBallAlignmentCoarseMaximumPulseMs &&
                  kBallAlignmentCoarseYawFraction > 0.0 &&
                  kBallAlignmentCoarseMinimumYawDegrees > 0.0 &&
                  kBallAlignmentCoarseMaximumYawDegrees >=
                      kBallAlignmentCoarseMinimumYawDegrees &&
                  kBallAlignmentFineYawFraction > 0.0 &&
                  kBallAlignmentFineMinimumYawDegrees > 0.0 &&
                  kBallAlignmentFineMaximumYawDegrees >=
                      kBallAlignmentFineMinimumYawDegrees &&
                  kBallAlignmentPulseStartTimeoutMs > 0 &&
                  kBallAlignmentAppliedPowerMinimum > 0.0 &&
                  kBallAlignmentAppliedPowerMinimum <=
                      kBallAlignmentCoarsePulsePower &&
                  kBallAlignmentAppliedPowerMinimum <=
                      kBallAlignmentFinePulsePower &&
                  kBallAlignmentStartPower >=
                      kBallAlignmentCoarsePulsePower &&
                  kBallAlignmentStartPower >=
                      kBallAlignmentFinePulsePower &&
                  kBallAlignmentStartPower <= kMaxMotorOutput &&
                  kBallAlignmentMovementMinimumRateCountsPerSecond > 0.0 &&
                  kBallAlignmentStableFrames > 0 &&
                  kBallAlignmentStationaryRateCountsPerSecond >= 0.0 &&
                  kBallAlignmentTargetLossTimeoutMs > 0 &&
                  kBallApproachStopDistanceCm > 0.0 &&
                  kBallCollectionNearAlignmentToleranceDegrees >
                      kVictimApproachStartToleranceDegrees &&
                  kBallCollectionNearAlignmentToleranceDegrees <
                      kBallAlignmentFineCorrectionThresholdDegrees &&
                  kVictimCollectionAdvanceDistanceCm > 0.0 &&
                  kVictimCollectionAdvancePower >= kMotorStartMinimumPower &&
                  kVictimCollectionAdvancePower <= kMaxMotorOutput &&
                  kVictimCollectionMaximumBalanceCorrection > 0.0 &&
                  kVictimCollectionAdvancePower -
                          kVictimCollectionMaximumBalanceCorrection >=
                      kMotorRunMinimumPower &&
                  kVictimCollectionAdvancePower +
                          kVictimCollectionMaximumBalanceCorrection <=
                      kMaxMotorOutput &&
                  kVictimCollectionMaximumSideDifferenceCm > 0.0 &&
                  kVictimCollectionDifferenceConfirmationSamples > 0 &&
                  kVictimCollectionPreparationTimeoutMs > 0 &&
                  kVictimCollectionStallTimeoutMs > 0 &&
                  kVictimCollectionTimeoutMs >
                      kVictimCollectionStallTimeoutMs &&
                  kVictimCollectionSettleMs > 0 &&
                  kBallApproachBasePower >= kMotorStartMinimumPower &&
                  kBallApproachBasePower +
                          kBallApproachMaximumSteeringCorrection <=
                      kMaxMotorOutput &&
                  kBallApproachBasePower -
                          kBallApproachMaximumSteeringCorrection >=
                      kMotorRunMinimumPower &&
                  kBallApproachMaximumSteeringCorrection > 0.0 &&
                  kBallApproachHeadingDeadbandDegrees > 0.0 &&
                  kBallApproachVisualAlignedToleranceDegrees >=
                      kBallApproachHeadingDeadbandDegrees &&
                  kBallApproachVisualAlignedToleranceDegrees <
                      kVictimApproachStartToleranceDegrees &&
                  kBallApproachVisualHeadingGain > 0.0 &&
                  kBallApproachVisualHeadingGain <= 1.0 &&
                  kBallApproachMaximumHeadingAdjustmentDegrees > 0.0 &&
                  kBallApproachFullSteeringErrorDegrees >
                      kBallApproachHeadingDeadbandDegrees,
                "Os limites do alinhamento de vítimas devem permanecer seguros.");

static_assert(kRescueSearchTurnPower >= kMotorStartMinimumPower &&
                  kRescueSearchTurnPower <= kMaxMotorOutput &&
                  kRescueSearchPulseMs > 0 &&
                  kRescueSearchSettlingMs > 0 &&
                  kRescueEntryAdvanceDistanceCm > 0.0 &&
                  kRescueEntryAdvancePower >= kMotorStartMinimumPower &&
                  kRescueEntryAdvancePower <= kMaxMotorOutput &&
                  kRescueVictimFirstSweepDegrees > 0.0 &&
                  kRescueVictimSecondSweepDegrees >
                      kRescueVictimFirstSweepDegrees &&
                  kRescueVictimSecondSweepDegrees < 180.0 &&
                  kRescueVictimSweepToleranceDegrees > 0.0 &&
                  kRescueVictimSweepToleranceDegrees <
                      kRescueVictimFirstSweepDegrees &&
                   kRescueVictimCandidateHoldMs > 0 &&
                   kRescueVictimFirstSweepTimeoutMs > 0 &&
                   kRescueVictimSecondSweepTimeoutMs >= kRescueVictimFirstSweepTimeoutMs &&
                   kRescueContinuousSearchMinimumProgressDegrees > 0.0 &&
                   kRescueContinuousSearchMinimumProgressDegrees < 180.0 &&
                   kRescueContinuousSearchStallTimeoutMs > 0 &&
                   kRescueFinalVictimSearchDegrees == 360.0 &&
                   kRescueFinalVictimSearchTimeoutMs >
                       kRescueContinuousSearchStallTimeoutMs &&
                   kRescuePostCollectionReverseDistanceCm > 0.0 &&
                  kRescuePostCollectionReversePower >=
                      kMotorStartMinimumPower &&
                  kRescuePostCollectionReversePower <= kMaxMotorOutput &&
                  kRescuePostDepositReverseDistanceCm > 0.0 &&
                  kRescuePostDepositReversePower >=
                      kMotorStartMinimumPower &&
                  kRescuePostDepositReversePower <= kMaxMotorOutput &&
                  kRescueFinalDepositReverseDistanceCm >=
                      kRescuePostDepositReverseDistanceCm &&
                  kRescueDistancePreparationTimeoutMs > 0 &&
                  kRescueDistanceStallTimeoutMs > 0 &&
                  kRescueDistanceTimeoutMs > kRescueDistanceStallTimeoutMs &&
                  kRescueDistanceSettleMs > 0 &&
                  kRescueDelicateMotionKickPower > kRescueSearchTurnPower &&
                  kRescueDelicateMotionKickPower <= kMaxMotorOutput &&
                  kRescueDelicateMotionKickDurationMs > 0,
              "Os deslocamentos da sala de resgate devem permanecer seguros.");

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

// Duas leituras ultrassônicas dentro deste limite, em centímetros, confirmam
// um obstáculo. A histerese exige afastamento antes de armar uma nova manobra.
constexpr double kObstacleDetectionDistanceCm = 6.0;
constexpr int kObstacleDetectionConfirmationSamples = 2;
constexpr double kObstacleRearmDistanceCm = 15.0;
constexpr int kObstacleRearmConfirmationSamples = 3;

// A medição gira 60 graus para enxergar a passagem lateral além do obstáculo.
// Depois da escolha, o robô retorna a 40 graus para fechar a aproximação antes
// da reta curta e da curva nominal.
constexpr double kObstacleClearanceScanDegrees = 60.0;
constexpr double kObstacleSideApproachDegrees = 40.0;
// A CAM1 confirma preto apenas nesta janela angular do scan. Antes dela, o
// obstáculo pode preencher a ROI; depois dela, a faixa lateral pode sair do quadro.
constexpr double kObstacleCameraBlackMinimumAngleDegrees = 10.0;
constexpr double kObstacleCameraBlackMaximumAngleDegrees = 40.0;
constexpr int kObstacleCameraBlackConfirmationFrames = 3;
// Os primeiros 25 graus ainda podem manter o obstáculo frontal dentro do cone
// do ultrassônico. A medição final também exige que o robô esteja dentro da
// tolerância do alvo de 60 graus e com velocidade angular estabilizada.
constexpr double kObstacleClearanceIgnoreDegrees = 25.0;

// Depois de alcançar cada lado, o robô permanece parado antes de aceitar ecos.
// Cinco leituras a 100 ms evitam decidir com a primeira medida residual do giro.
constexpr int kObstacleClearanceSettleMs = 200;
constexpr int kObstacleClearanceRequiredSamples = 5;
constexpr int kObstacleClearanceSamplingTimeoutMs = 1500;

// No modo adaptativo, diferenças de até 2 cm são tratadas como empate prático.
// O lado direito preserva uma decisão repetível quando ambas estão livres.
constexpr double kObstacleClearanceTieCm = 2.0;
constexpr bool kObstacleDefaultSideIsRight = true;
// Este obstáculo da arena possui passagem confiável somente pela esquerda.
// Quando ativo, o perfil pula a varredura lateral e inicia o contorno esquerdo
// logo após centralizar, evitando que uma chegada diagonal troque o lado.
constexpr bool kObstacleForceLeftSide = true;

// Idade máxima, em milissegundos, aceita para cada eco da varredura lateral.
// Uma leitura antiga nunca deve influenciar a escolha do lado do obstáculo.
constexpr int kObstacleUltrasonicFreshnessMs = 300;

// Ângulos, em graus, executados na ordem da máquina de desvio.
constexpr double kObstacleFirstRightTurnDegrees = 45.0;
constexpr double kObstacleFirstLeftTurnDegrees = 45.0;
constexpr double kObstacleSecondLeftTurnDegrees = 90.0;
constexpr double kObstacleFinalRightTurnDegrees = 90.0;
// A faixa de ±5 graus aceita um giro real entre 40 e 50 graus no alvo de 45.
constexpr double kObstacleTurnToleranceDegrees = 5.0;
// Potência e micropulsos exclusivos da varredura lateral. O valor 0,73 ainda
// vence o atrito estático, mas reduz a inércia observada com 0,75.
constexpr double kObstacleTurnCommandPower = 0.73;
constexpr int kObstacleTurnCorrectionPulseMs = 30;
// Valor -1 permite quantas correções forem necessárias no desvio. A proteção
// contra giro eterno continua sendo feita pelo timeout total e pela validade da IMU.
constexpr int kObstacleTurnMaximumCorrectionPulses = -1;
constexpr int kObstacleTurnTimeoutMs = 12000;

// Primeira reta experimental após a escolha do lado. O alvo de heading é o
// yaw realmente alcançado no posicionamento lateral, não o yaw base.
constexpr double kObstacleSelectedForwardDistanceCm = 12.0;
constexpr double kObstacleSelectedForwardPower = 0.75;
constexpr double kObstacleSelectedForwardMaximumHeadingCorrection = 0.04;
constexpr double kObstacleSelectedForwardFullHeadingErrorDegrees = 10.0;

// Comprimento calibrável da curva nominal executada após a primeira reta.
constexpr double kObstacleCurveDistanceCm = 20.0;
constexpr double kObstacleCurveEndOffsetDegrees = 45.0;
constexpr double kObstacleCurveBasePower = 0.75;
constexpr double kObstacleCurveMaximumHeadingCorrection = 0.06;
constexpr double kObstacleCurveFullHeadingErrorDegrees = 15.0;
// O modo adaptativo antigo usa este pivot após a curva. O perfil esquerdo fixo
// ignora este valor e segue reto durante a procura temporizada definida abaixo.
constexpr double kObstacleFinalInwardPivotDegrees = 40.0;

// Após a curva, o robô para brevemente e gira 25 graus para a direita. A pausa
// evita que a inércia da curva altere a referência inicial do giro pela IMU.
constexpr int kObstacleExitPivotWaitMs = 250;
constexpr double kObstacleExitPivotRightDegrees = 25.0;
// Depois do giro, o robô segue reto por até 1,3 segundo procurando a faixa.
// O limite impede avanço indefinido caso a visão não encontre a saída.
constexpr int kObstacleExitStraightTimeoutMs = 1300;
// Se a faixa não aparecer na reta, uma busca explícita para a direita também
// possui limite de dois segundos. Ao expirar, os motores são zerados.
constexpr int kObstacleExitSearchRightTimeoutMs = 2000;

// Recuperação antecipada exclusiva da curva de obstáculo. Três amostras novas
// do Fusion cancelam a curva nominal; os encoders medem o avanço curto e a IMU
// limita o pivot de busca para impedir movimento indefinido.
constexpr int kObstacleFusionReacquireConfirmationFrames = 3;
constexpr double kObstacleReacquireForwardDistanceCm = 5.0;
constexpr double kObstacleReacquireSearchMaximumDegrees = 100.0;

// O caso 3 conserva somente o frame lateral mais dominante da parábola. A
// memória passa a ter validade temporal apenas após o primeiro Fusion estável.
constexpr std::uint64_t kObstacleParabolaMinimumBlackPixels = 3000;
constexpr double kObstacleParabolaMinimumDominance = 1.5;
constexpr int kObstacleParabolaGapLostConfirmationFrames = 3;
constexpr int kObstacleParabolaNearValidationFrames = 3;
constexpr int kObstacleParabolaNearRequiredVotes = 2;
constexpr int kObstacleCase3FusionWindowMs = 2000;
// Avanço reto, em centímetros, executado antes do pivot do caso 3. A distância
// cria folga do obstáculo e continua limitada pelos encoders e pelo timeout.
constexpr double kObstacleParabolaReacquireForwardDistanceCm = 5.0;
// Limite angular, em graus, da busca lateral iniciada pelo caso 3.
constexpr double kObstacleParabolaRearBlockDegrees = 65.0;
// Limite menor, em graus, aplicado somente ao Fusion que tenta levar o robô para
// o lado oposto ao melhor lado salvo após o desvio nominal do obstáculo.
constexpr double kObstaclePostObstacleFusionReturnLimitDegrees = 35.0;

// Distâncias, em centímetros, calibradas para contornar o obstáculo atual.
constexpr double kObstacleFirstForwardDistanceCm = 25.0;
constexpr double kObstacleSecondForwardDistanceCm = 30.0;
constexpr double kObstacleThirdForwardDistanceCm = 21.5;
constexpr double kObstacleReverseDistanceCm = 5.0;

// Potências normalizadas dos deslocamentos para frente e em ré.
constexpr double kObstacleForwardPower = 0.75;
constexpr double kObstacleReversePower = 0.75;
// Limite apenas de transição da ré inicial. Se os encoders não responderem, o
// desvio continua pela centralização em vez de encerrar a missão autônoma.
constexpr int kObstacleInitialReverseMaximumMs = 1500;

// Pausa entre etapas e limites de segurança da odometria do desvio.
constexpr int kObstacleStageSettleMs = 250;
constexpr int kObstacleEncoderFreshnessMs = 300;
constexpr int kObstacleDistanceSafetyTimeoutMs = 12000;

// Horizonte, em segundos, usado para antecipar a inércia antes da distância-alvo.
constexpr double kObstacleBrakePredictionSeconds = 0.14;

static_assert(kObstacleDetectionDistanceCm > 0.0 &&
                  kObstacleRearmDistanceCm > kObstacleDetectionDistanceCm &&
                  kObstacleDetectionConfirmationSamples > 0 &&
                  kObstacleRearmConfirmationSamples > 0,
              "A detecção de obstáculo deve possuir histerese válida.");
static_assert(kObstacleClearanceScanDegrees > 0.0 &&
                  kObstacleClearanceScanDegrees <= 90.0 &&
                  kObstacleSideApproachDegrees > 0.0 &&
                  kObstacleSideApproachDegrees <= 90.0 &&
                  kObstacleClearanceScanDegrees >
                      kObstacleSideApproachDegrees &&
                  kObstacleCameraBlackMinimumAngleDegrees >= 0.0 &&
                  kObstacleCameraBlackMaximumAngleDegrees >
                      kObstacleCameraBlackMinimumAngleDegrees &&
                  kObstacleCameraBlackMaximumAngleDegrees <
                      kObstacleClearanceScanDegrees &&
                  kObstacleCameraBlackConfirmationFrames > 0 &&
                  kObstacleClearanceIgnoreDegrees >= 0.0 &&
                  kObstacleClearanceIgnoreDegrees <
                      kObstacleClearanceScanDegrees &&
                  kObstacleClearanceSettleMs >= 0 &&
                  kObstacleClearanceRequiredSamples > 1 &&
                  kObstacleClearanceSamplingTimeoutMs >
                      kObstacleClearanceSettleMs &&
                  kObstacleClearanceTieCm >= 0.0 &&
                  kObstacleUltrasonicFreshnessMs > 0,
              "A varredura lateral do obstáculo deve permanecer válida.");
static_assert(kObstacleFirstRightTurnDegrees > 0.0 &&
                  kObstacleFirstLeftTurnDegrees > 0.0 &&
                  kObstacleSecondLeftTurnDegrees > 0.0 &&
                  kObstacleSecondLeftTurnDegrees <= 180.0 &&
                  kObstacleFinalRightTurnDegrees > 0.0 &&
                  kObstacleFinalRightTurnDegrees <= 180.0 &&
                  kObstacleTurnToleranceDegrees > 0.0 &&
                  kObstacleTurnCommandPower > 0.0 &&
                  kObstacleTurnCommandPower <= kMaxMotorOutput &&
                  kObstacleTurnCorrectionPulseMs > 0 &&
                  (kObstacleTurnMaximumCorrectionPulses == -1 ||
                   kObstacleTurnMaximumCorrectionPulses > 0) &&
                  kObstacleTurnTimeoutMs > 0 &&
                  kObstacleSelectedForwardDistanceCm > 0.0 &&
                  kObstacleSelectedForwardPower > 0.0 &&
                  kObstacleSelectedForwardPower +
                          kObstacleSelectedForwardMaximumHeadingCorrection <=
                      kMaxMotorOutput &&
                  kObstacleSelectedForwardMaximumHeadingCorrection > 0.0 &&
                  kObstacleSelectedForwardFullHeadingErrorDegrees > 0.0 &&
                  kObstacleCurveDistanceCm > 0.0 &&
                  kObstacleCurveEndOffsetDegrees > 0.0 &&
                  kObstacleCurveEndOffsetDegrees <= 90.0 &&
                  kObstacleCurveBasePower > 0.0 &&
                  kObstacleCurveBasePower +
                          kObstacleCurveMaximumHeadingCorrection <=
                      kMaxMotorOutput &&
                  kObstacleCurveMaximumHeadingCorrection > 0.0 &&
                  kObstacleCurveFullHeadingErrorDegrees > 0.0 &&
                  kObstacleFinalInwardPivotDegrees > 0.0 &&
                  kObstacleFinalInwardPivotDegrees <= 45.0 &&
                  kObstacleExitPivotWaitMs >= 0 &&
                  kObstacleExitPivotRightDegrees > 0.0 &&
                  kObstacleExitPivotRightDegrees <= 45.0 &&
                  kObstacleExitStraightTimeoutMs > 0 &&
                  kObstacleExitSearchRightTimeoutMs > 0 &&
                  kObstacleFusionReacquireConfirmationFrames > 0 &&
                  kObstacleReacquireForwardDistanceCm > 0.0 &&
                  kObstacleReacquireSearchMaximumDegrees > 0.0 &&
                  kObstacleReacquireSearchMaximumDegrees < 180.0 &&
                  kObstacleParabolaMinimumBlackPixels > 0 &&
                  kObstacleParabolaMinimumDominance > 1.0 &&
                  kObstacleParabolaGapLostConfirmationFrames > 0 &&
                  kObstacleParabolaNearValidationFrames > 0 &&
                  kObstacleParabolaNearRequiredVotes > 0 &&
                  kObstacleParabolaNearRequiredVotes <=
                      kObstacleParabolaNearValidationFrames &&
                  kObstacleCase3FusionWindowMs > 0 &&
                  kObstacleParabolaReacquireForwardDistanceCm > 0.0 &&
                  kObstacleParabolaRearBlockDegrees > 0.0 &&
                  kObstacleParabolaRearBlockDegrees <
                      kObstacleReacquireSearchMaximumDegrees &&
                  kObstaclePostObstacleFusionReturnLimitDegrees > 0.0 &&
                  kObstaclePostObstacleFusionReturnLimitDegrees <
                      kObstacleParabolaRearBlockDegrees,
              "Os ângulos do desvio devem permanecer válidos.");
static_assert(kObstacleFirstForwardDistanceCm > 0.0 &&
                  kObstacleSecondForwardDistanceCm > 0.0 &&
                  kObstacleThirdForwardDistanceCm > 0.0 &&
                  kObstacleReverseDistanceCm > 0.0 &&
                  kObstacleForwardPower > 0.0 &&
                  kObstacleForwardPower <= kMaxMotorOutput &&
                  kObstacleReversePower > 0.0 &&
                  kObstacleReversePower <= kMaxMotorOutput &&
                  kObstacleInitialReverseMaximumMs > 0 &&
                  kObstacleStageSettleMs >= 0 &&
                  kObstacleEncoderFreshnessMs > 0 &&
                  kObstacleDistanceSafetyTimeoutMs > 0 &&
                  kObstacleBrakePredictionSeconds >= 0.0,
              "Os deslocamentos do desvio devem permanecer seguros.");

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
// Ao reconhecer o retorno, o robô permanece parado por 250 milissegundos antes
// de alinhar. A pausa curta estabiliza a leitura sem atrasar a reação ao verde.
// O atraso não bloqueia o loop, mantendo E-Stop e telemetria ativos.
// O retorno de 180° usa a rotina dedicada somente depois da confirmação
// métrica comum aos verdes. Desative apenas se a prova não usar esse marcador.
constexpr bool kGreenTurnAroundEnabled = true;
// Maior potência base permitida enquanto um candidato verde é confirmado.
// A base não aumenta; o rumo reto é corrigido pela IMU durante esta janela.
constexpr double kGreenConfirmationMaximumBasePower = 0.72;
// Distância máxima, em centímetros, desde a primeira detecção. Ao atingir
// este limite, LEFT/RIGHT acumulado é travado ou o candidato é descartado.
constexpr double kGreenConfirmationMaximumDistanceCm = 3.0;
// Tempo máximo, em milissegundos, para aguardar uma decisão visual depois de
// interromper o avanço nos 30 mm. O timeout libera o seguidor sem converter
// uma observação inconclusiva em verde falso.
constexpr int kGreenConfirmationDecisionWaitMs = 400;
// Espera adicional, em milissegundos, parada nos 30 mm quando ainda há
// candidato com preto válido. Um teto fixo evita espera indefinida.
constexpr int kGreenConfirmationExtraWaitMs = 600;
// Posição lateral normalizada mínima para aceitar uma faixa no lado do verde
// antes dos 30°. Exclui a região central, que pode ser a trajetória antiga.
constexpr double kGreenEarlyLineSideMinimumPosition = 0.20;
// Proteção experimental somente da dispensa dos 30°. Desativar restaura
// a regra anterior, sem desfazer as outras correções dos verdes.
constexpr bool kGreenEarlyBranchGuardEnabled = true;
// Diferença angular mínima, em graus, entre a faixa de entrada e o ramo
// observado, compensando o yaw. É evidência visual, não um giro obrigatório.
// A projeção da câmera é aproximada; validar também com chegadas inclinadas.
constexpr double kGreenEarlyBranchMinimumHeadingChangeDegrees = 20.0;
// Frames novos consecutivos com evidência do ramo para dispensar os 30°.
// Duas imagens evitam liberar a busca por uma única leitura lateral espúria.
constexpr int kGreenEarlyBranchStableFrames = 2;
// Quantidade de frames sem candidato necessária para aceitar um novo marcador.
constexpr int kGreenRearmClearFrames = 3;
// Distância reta, em centímetros, percorrida depois de travar LEFT/RIGHT.
// Os 10 cm afastam o robô do marcador antes do giro lateral.
constexpr double kGreenLateralForwardDistanceCm = 10.0;
// Potência do avanço medido. Ela permanece abaixo do limite da confirmação.
constexpr double kGreenLateralForwardPower = 0.70;
// Ré curta, em centímetros, após reencontrar a faixa do ramo verde.
constexpr double kGreenLateralReverseDistanceCm = 5.0;
// Potência da ré medida; o piso permite partir sem acelerar desnecessariamente.
constexpr double kGreenLateralReversePower = kMotorStartMinimumPower;
// Correção diferencial por grau de desvio do yaw nas retas do verde.
// O limite evita que a IMU transforme um avanço ou uma ré em pivot.
constexpr double kGreenStraightYawGainPerDegree = 0.012;
constexpr double kGreenStraightYawMaximumCorrection = 0.08;
// Giro mínimo de segurança, em graus, quando falta faixa válida no lado
// confirmado. A evidência visual desse ramo pode dispensar o mínimo.
constexpr double kGreenLateralMinimumYawDegrees = 30.0;
// Potência do pivot visual que procura o ramo escolhido pelo marcador.
constexpr double kGreenLateralSearchPower = kTurn90CommandPower;
// Tempo máximo, em milissegundos, para encontrar a nova trajetória.
constexpr int kGreenLateralSearchTimeoutMs = 4500;
// Parâmetros do controle local de centralização. NEAR e MID calculam a
// correção, mas FAR também deve estar confiável para habilitar esta fase.
constexpr double kGreenCenteringBasePower = 0.67;
constexpr double kGreenCenteringPositionGain = 0.34;
constexpr double kGreenCenteringHeadingGain = 0.22;
constexpr double kGreenCenteringMaximumCorrection = 0.18;
constexpr double kGreenCenteringPositionTolerance = 0.18;
constexpr double kGreenCenteringHeadingTolerance = 0.22;
constexpr int kGreenCenteringRequiredFrames = 3;
// O timeout encerra a centralização opcional e inicia a ré medida.
constexpr int kGreenCenteringTimeoutMs = 2000;
// Limite, em graus de yaw, da prioridade do verde lateral visual. Ao alcançar
// este deslocamento, a câmera devolve o controle ao Fusion normal, sem novo giro.
constexpr double kGreenVisualMaximumTurnDegrees = 45.0;
// Antecipação, em segundos, do limite angular para compensar atraso e inércia.
// Um valor maior devolve o controle antes; não comanda um giro pela IMU.
constexpr double kGreenVisualAnglePredictionSeconds = 0.10;
constexpr int kGreenTurnAroundRecognitionDelayMs = 250;
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
constexpr double kGreenTurnAroundForwardDistanceCm = 12.0;
// Potência normalizada usada exclusivamente no avanço após reconhecer o
// retorno de 180°.
// O valor 0,69 independe da potência base do segue-linha e não representa cm/s.
constexpr double kGreenTurnAroundForwardPower = 0.73;
// Tempo parado, em milissegundos, entre o avanço e o início do giro.
constexpr int kGreenTurnAroundForwardSettleMs = 250;
// Idade máxima, em milissegundos, aceita para os dados dos encoders.
// O avanço só é interrompido por encoder quando a telemetria fica ausente.
constexpr int kGreenTurnAroundEncoderDataTimeoutMs = 1000;

// Limite absoluto de segurança, em milissegundos, para o avanço configurado.
// Ele não controla a distância; apenas impede movimento indefinido se uma
// contagem congelada continuar chegando como telemetria aparentemente válida.
constexpr int kGreenTurnAroundForwardSafetyTimeoutMs = 10000;
// Ângulo inicial, em graus, controlado pelo MPU6050. Depois deste trecho, o
// robô continua o giro normalmente e encerra ao reencontrar a faixa pela câmera.
constexpr double kGreenTurnAroundImuDegrees = 140.0;
// Erro angular máximo, em graus, aceito para concluir a etapa do IMU.
constexpr double kGreenTurnAroundImuToleranceDegrees = 8.0;
// Define o sentido do retorno: true gira à direita; false gira à esquerda.
constexpr bool kGreenTurnAroundTurnsRight = true;
// Potência normalizada do pivot que continua até o NEAR encontrar a linha.
constexpr double kGreenTurnAroundLineSearchPower = kTurn90CommandPower;
// Giro adicional máximo, em graus, permitido durante a busca visual da linha.
// Com o alvo atual, ele limita o retorno a aproximadamente 185° se a câmera
// não recuperar a linha pelo NEAR ou Fusion, evitando uma volta quase completa.
constexpr double kGreenTurnAroundLineSearchMaximumDegrees = 45.0;
// Quantidade de confirmações consecutivas com NEAR ou Fusion válido para
// retomar o seguidor depois do giro.
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
static_assert(kGreenConfirmationMaximumBasePower >= kMotorStartMinimumPower &&
                  kGreenConfirmationMaximumBasePower <= kMaxMotorOutput &&
                  kGreenConfirmationMaximumDistanceCm > 0.0 &&
                  kGreenConfirmationDecisionWaitMs > 0 &&
                  kGreenConfirmationExtraWaitMs > 0 &&
                  kGreenEarlyLineSideMinimumPosition > 0.0 &&
                  kGreenEarlyLineSideMinimumPosition < 1.0 &&
                  kGreenEarlyBranchMinimumHeadingChangeDegrees > 0.0 &&
                  kGreenEarlyBranchMinimumHeadingChangeDegrees < 90.0 &&
                  kGreenEarlyBranchStableFrames > 0 &&
                  kGreenRearmClearFrames > 0 &&
                  kGreenLateralForwardDistanceCm > 0.0 &&
                  kGreenLateralForwardPower >= kMotorStartMinimumPower &&
                  kGreenLateralForwardPower <=
                      kGreenConfirmationMaximumBasePower &&
                  kGreenLateralReverseDistanceCm > 0.0 &&
                  kGreenLateralReversePower >= kMotorStartMinimumPower &&
                  kGreenLateralReversePower <= kMaxMotorOutput &&
                  kGreenStraightYawGainPerDegree > 0.0 &&
                  kGreenStraightYawMaximumCorrection > 0.0 &&
                  kGreenStraightYawMaximumCorrection <
                      kGreenLateralReversePower &&
                  kGreenLateralMinimumYawDegrees >= 30.0 &&
                  kGreenLateralMinimumYawDegrees < 180.0 &&
                  kGreenLateralSearchPower >= kMotorStartMinimumPower &&
                  kGreenLateralSearchPower <= kMaxMotorOutput &&
                  kGreenLateralSearchTimeoutMs > 0 &&
                  kGreenCenteringBasePower >= kMotorStartMinimumPower &&
                  kGreenCenteringBasePower <=
                      kGreenConfirmationMaximumBasePower &&
                  kGreenCenteringMaximumCorrection > 0.0 &&
                  kGreenCenteringPositionTolerance > 0.0 &&
                  kGreenCenteringHeadingTolerance > 0.0 &&
                  kGreenCenteringRequiredFrames > 0 &&
                  kGreenCenteringTimeoutMs == 2000,
              "A confirmação e a curva lateral verde devem permanecer seguras.");
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

// Faixa angular aceita para braço, pulso e garra.
// A ESP32 repete esta validação antes de converter o ângulo em pulso do PCA9685.
constexpr double kServoMinimumAngleDegrees = 0.0;
constexpr double kServoMaximumAngleDegrees = 180.0;

// Faixa aceita pela API opcional de movimento gradual da ESP32, em graus por
// segundo. O ServoController não usa essa API automaticamente; a Raspberry
// precisa solicitá-la explicitamente para não alterar as rotinas atuais.
constexpr double kServoSlewMinimumSpeedDegreesPerSecond = 1.0;
constexpr double kServoSlewMaximumSpeedDegreesPerSecond = 720.0;

// Ângulo neutro usado pelo pulso, pela garra e por poses que exigem 0°.
constexpr double kServoInitialAngleDegrees = 0.0;

// Posição do braço, em graus, solicitada ao iniciar o modo Autônomo.
// O modo Manual não aplica esta pose, pois deve conservar a posição atual.
constexpr double kAutonomousInitialArmAngleDegrees = 15.0;

// Velocidade máxima, em graus por segundo, aplicada ao servo do pulso pela
// Raspberry. Reduzir este valor suaviza o movimento e diminui o impulso que
// pode deslocar mecanicamente a garra; aumentar torna o pulso mais rápido.
constexpr double kWristServoMaximumSpeedDegreesPerSecond = 240.0;

// Tempo, em milissegundos, que a pose completa permanece aplicada antes de o
// pulso voltar a se mover após uma reativação. Isso dá à garra tempo para
// recuperar o alvo antes de receber o esforço mecânico do movimento.
constexpr int kServoPoseHoldBeforeWristMotionMs = 200;

// Intervalo máximo, em milissegundos, considerado pela rampa do pulso.
// O limite evita um salto grande depois de uma pausa inesperada do loop.
constexpr int kServoMotionMaximumElapsedMs = 100;

// Tempos conservadores, em milissegundos, reservados para que cada servo
// conclua seu passo antes de o próximo começar. Não existe sensor físico de
// posição, portanto esses valores devem ser validados com o mecanismo real.
constexpr int kServoRoutineArmStepMs = 750;
constexpr int kServoRoutineWristStepMs = 850;
constexpr int kServoRoutineGripperStepMs = 300;

// Tempo, em milissegundos, para estabilizar a pose autônoma 15°/0°/0°.
// Ele cobre o pior deslocamento do pulso pela rampa configurada, inclusive após
// a pausa de reativação, antes de qualquer outro mecanismo começar a mover.
constexpr int kServoRoutineInitialPoseMs = 1300;

// Tempo, em milissegundos, para reativar e confirmar a pose memorizada antes
// do primeiro movimento de uma rotina encadeada. Isso faz a garra recuperar
// sua força de retenção antes de braço ou pulso produzirem inércia.
constexpr int kServoRoutineResumePoseMs = 250;

// Tempo curto, em milissegundos, durante o qual a garra pressiona em 0°.
// Depois desse pulso, somente o sinal da garra é removido para evitar esforço
// contínuo; braço e pulso permanecem energizados.
constexpr int kServoRoutineGripperPressMs = 500;

// Tempo máximo, em milissegundos, para confirmar uma abertura no depósito.
// Se não houver confirmação, a sequência para sem abrir automaticamente.
constexpr int kServoRoutineConfirmationTimeoutMs = 2000;

// Intervalo, em milissegundos, para repetir a solicitação que proíbe FULL_OFF
// durante a sala de resgate até a ESP32 confirmar a trava pela telemetria.
constexpr int kRescueServoHoldRetryMs = 250;

// Tempo máximo, em milissegundos, para a telemetria confirmar os três canais
// ativos depois da primeira pose. A rotina não avança enquanto eles estiverem OFF.
constexpr int kRescueServoEnableConfirmationTimeoutMs = 1000;

// Posições, em graus, usadas pelas rotinas mecânicas predefinidas. Alterar
// qualquer valor muda diretamente os pontos de captura, armazenamento e depósito.
constexpr double kServoRoutineArmHomeDegrees = kAutonomousInitialArmAngleDegrees;
constexpr double kServoRoutineArmPickupDegrees = 103.0;
constexpr double kServoRoutineArmStorageClearanceDegrees = 50.0;
constexpr double kServoRoutineArmStorageTransitionDegrees = 20.0;
constexpr double kServoRoutineArmStoredPickupDegrees = 65.0;
constexpr double kServoRoutineArmStoredCarryDegrees = 25.0;
constexpr double kServoRoutineWristForwardDegrees = 180.0;
constexpr double kServoRoutineWristInternalDegrees = 0.0;
constexpr double kServoRoutineWristStorageClearanceDegrees = 45.0;
constexpr double kServoRoutineWristStoredApproachDegrees = 65.0;
constexpr double kServoRoutineGripperFullyOpenDegrees = 180.0;
constexpr double kServoRoutineGripperDepositDegrees = 90.0;
constexpr double kServoRoutineGripperClosedDegrees = 0.0;
// Após o aperto inicial em 0°, a garra recua para 5° e mantém o PWM
// ativo. Isso conserva a vítima presa sem forçar continuamente o batente.
constexpr double kServoRoutineGripperRetentionDegrees = 5.0;

static_assert(kServoSlewMinimumSpeedDegreesPerSecond > 0.0 &&
                  kServoSlewMaximumSpeedDegreesPerSecond >=
                      kServoSlewMinimumSpeedDegreesPerSecond &&
                  kWristServoMaximumSpeedDegreesPerSecond > 0.0 &&
                  kServoPoseHoldBeforeWristMotionMs >= 0 &&
                  kServoMotionMaximumElapsedMs >= kMainLoopPeriodMs &&
                  kServoRoutineArmStepMs > 0 &&
                  kServoRoutineWristStepMs > 0 &&
                  kServoRoutineGripperStepMs > 0 &&
                  kServoRoutineInitialPoseMs > 0 &&
                  kServoRoutineResumePoseMs >= kServoPoseHoldBeforeWristMotionMs &&
                  kServoRoutineGripperPressMs > 0 &&
                  kServoRoutineConfirmationTimeoutMs > 0 &&
                  kRescueServoHoldRetryMs > 0 &&
                  kRescueServoEnableConfirmationTimeoutMs > 0 &&
                  static_cast<double>(kServoRoutineInitialPoseMs) >=
                      kServoPoseHoldBeforeWristMotionMs +
                          (kServoMaximumAngleDegrees - kServoMinimumAngleDegrees) /
                              kWristServoMaximumSpeedDegreesPerSecond * 1000.0 &&
                  static_cast<double>(kServoRoutineWristStepMs) >=
                      kServoRoutineWristForwardDegrees /
                          kWristServoMaximumSpeedDegreesPerSecond * 1000.0,
              "Os tempos das rotinas de servo devem ser positivos.");
static_assert(
        kServoRoutineArmHomeDegrees >= kServoMinimumAngleDegrees &&
        kServoRoutineArmHomeDegrees <= kServoMaximumAngleDegrees &&
        kServoRoutineArmPickupDegrees >= kServoMinimumAngleDegrees &&
        kServoRoutineArmPickupDegrees <= kServoMaximumAngleDegrees &&
        kServoRoutineArmStorageClearanceDegrees >= kServoMinimumAngleDegrees &&
        kServoRoutineArmStorageClearanceDegrees <= kServoMaximumAngleDegrees &&
        kServoRoutineArmStorageTransitionDegrees >= kServoMinimumAngleDegrees &&
        kServoRoutineArmStorageTransitionDegrees <= kServoMaximumAngleDegrees &&
        kServoRoutineArmStoredPickupDegrees >= kServoMinimumAngleDegrees &&
        kServoRoutineArmStoredPickupDegrees <= kServoMaximumAngleDegrees &&
        kServoRoutineArmStoredCarryDegrees >= kServoMinimumAngleDegrees &&
        kServoRoutineArmStoredCarryDegrees <= kServoMaximumAngleDegrees &&
        kServoRoutineWristForwardDegrees >= kServoMinimumAngleDegrees &&
        kServoRoutineWristForwardDegrees <= kServoMaximumAngleDegrees &&
        kServoRoutineWristInternalDegrees >= kServoMinimumAngleDegrees &&
        kServoRoutineWristInternalDegrees <= kServoMaximumAngleDegrees &&
        kServoRoutineWristStorageClearanceDegrees >= kServoMinimumAngleDegrees &&
        kServoRoutineWristStorageClearanceDegrees <= kServoMaximumAngleDegrees &&
        kServoRoutineWristStoredApproachDegrees >= kServoMinimumAngleDegrees &&
        kServoRoutineWristStoredApproachDegrees <= kServoMaximumAngleDegrees &&
        kServoRoutineGripperFullyOpenDegrees >= kServoMinimumAngleDegrees &&
        kServoRoutineGripperFullyOpenDegrees <= kServoMaximumAngleDegrees &&
        kServoRoutineGripperDepositDegrees >= kServoMinimumAngleDegrees &&
        kServoRoutineGripperDepositDegrees <= kServoMaximumAngleDegrees &&
        kServoRoutineGripperClosedDegrees >= kServoMinimumAngleDegrees &&
        kServoRoutineGripperClosedDegrees <= kServoMaximumAngleDegrees &&
        kServoRoutineGripperRetentionDegrees >= kServoMinimumAngleDegrees &&
        kServoRoutineGripperRetentionDegrees <= kServoMaximumAngleDegrees,
    "As posições das rotinas devem permanecer na faixa angular segura.");

// Faixa absoluta, em microssegundos, permitida somente na calibração de
// bancada. Ela é mais ampla que a faixa operacional e nunca é ultrapassada,
// mesmo que o dashboard envie um valor inválido.
constexpr int kServoCalibrationAbsoluteMinimumPulseUs = 500;
constexpr int kServoCalibrationAbsoluteMaximumPulseUs = 2500;

// Diferença mínima, em microssegundos, entre os pulsos associados a 0° e 180°.
// Uma faixa menor provavelmente representa captura acidental de dois pontos
// quase iguais e tornaria o controle angular excessivamente sensível.
constexpr int kServoCalibrationMinimumSpanUs = 200;

// Tempo máximo, em milissegundos, para considerar recente a telemetria da ESP32.
// Se esse tempo estourar, o dashboard mostra os sensores como desatualizados.
constexpr int kEsp32TelemetryTimeoutMs = 1000;

// Limites compartilhados com o firmware da ESP32 para mensagens temporárias na
// OLED. Textos maiores são cortados antes do envio para preservar a linha UART.
constexpr int kRemoteOledTitleMaxLength = 12;
constexpr int kRemoteOledLineMaxLength = 20;
constexpr int kRemoteOledMinimumDurationMs = 500;
constexpr int kRemoteOledMaximumDurationMs = 30000;

// Tempo, em milissegundos, que alertas confirmados de navegação permanecem na
// OLED. A mensagem é temporária e não bloqueia o laço de controle do robô.
constexpr int kOledNavigationAlertDurationMs = 2500;

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
