# Módulo de desvio de obstáculo

Este módulo contém o código específico, as calibrações e os testes do
desvio de obstáculo usado no OBR2026K.

## Conteúdo

- `include/obr/obstacle_avoidance.h`: interface pública e máquina de estados.
- `src/robot/obstacle_avoidance.cpp`: detecção, giros e deslocamentos.
- `include/obr/config.h`: limites e calibrações centralizados.
- `tests/obstacle_avoidance_test.cpp`: regressões da sequência completa e das falhas.

## Sequência atual

Com `kObstacleIgnoreBlackDuringCurve = true` (padrão atual), a curva de
`kObstacleCurveDistanceCm` (20 cm) ignora o Fusion e as massas pretas da CAM1.
Ela não inicia recuperação antecipada nem salva memória lateral para o caso 3.
A saída nominal volta a procurar Fusion após a curva. Encoders, IMU, parada de
emergência e timeouts continuam ativos. Use `false` para permitir os cenários
de recuperação por linhas laterais descritos abaixo.

O padrão tem `kObstacleForceLeftSide = false`: a escolha do lado e os cenários
da branch `main` foram retomados, com a saída nominal atual espelhada. Forçar
LEFT serve apenas para calibração e não desativa os cenários de recuperação.

1. Confirma o obstáculo a até 6 cm em duas chamadas elegíveis do controle.
   Atualmente, essa contagem não exige dois ecos distintos do ultrassônico.
2. Recua até 2 cm, com encerramento da ré inicial após 1500 ms ou perda de
   atualização dos encoders; nesses casos, passa para a centralização.
3. Centraliza pela câmera inferior e salva `yawBase`.
4. Mede a esquerda a `yawBase - 60°`, retorna ao yaw base e mede a direita a
   `yawBase + 60°`. Em cada lado, espera 200 ms parado e coleta cinco amostras
   novas pela telemetria da ESP32. Uma confirmação exclusiva de preto pela CAM1
   durante o scan prevalece; senão, escolhe a maior distância ultrassônica.
   Diferenças de até 2 cm empatam e escolhem RIGHT por padrão.
5. Posiciona a aproximação em `yawBase - 40°` para LEFT ou `yawBase + 40°`
   para RIGHT e avança 12 cm mantendo o yaw realmente alcançado no giro.
6. Executa uma curva de 20 cm, com yaw alvo progressivo até `yawBase + 45°`
   para LEFT ou `yawBase - 45°` para RIGHT.
   Concluir a distância não garante que esse yaw final tenha sido alcançado.
7. Para por 250 ms e gira 25° para dentro a partir do yaw medido naquele momento:
   direita no contorno LEFT, esquerda no contorno RIGHT.
8. Avança 6 cm com comandos iguais. Os dois encoders precisam alcançar o alvo;
   a compensação de frenagem evita ultrapassar a distância por inércia.
9. Gira mais 10° para dentro enquanto procura o Fusion: direita no LEFT e
   esquerda no RIGHT. Se confirmar três frames, encerra a manobra imediatamente.
10. Sem confirmação, avança reto por até 1500 ms ainda procurando o Fusion.
    A sincronização dos motores pode ajustar as saídas aplicadas.
11. Se necessário, gira para dentro por até 3000 ms: direita no LEFT, esquerda
   no RIGHT. O giro acumulado desde o início da procura após a reta de 6 cm
   inclui os 10° anteriores e vai no máximo até +60°. Nesse extremo,
   inverte o giro sem pausa, com até mais 3000 ms para chegar a -60°.
   O limite negativo é `kObstacleExitSearchOppositeMaximumDegrees`, em
   `include/obr/config.h`: altere `60.0` para mudar o módulo do ângulo negativo.
   Três frames novos e consecutivos com fonte `fusion` e
   `normalSteeringValid` concluem a saída dentro desse setor; sem confirmação,
   para. O contorno RIGHT usa os sinais espelhados.

A entrada aceita Fusion para ambos os lados dentro do setor relativo ao
`yawBase` salvo antes do contorno. A regra anterior de rejeição por potência
foi substituída por limites angulares. Após o reencontro, a proteção permanece
por `kObstacleExitHeadingGuardMs`, sem exigir distância percorrida.

Calibre em `include/obr/config.h`:

- `kObstacleExitHeadingRightLimitDegrees`: limite positivo relativo ao yaw base.
- `kObstacleExitHeadingLeftLimitDegrees`: módulo do limite negativo.
- `kObstacleExitHeadingGuardMs`: duração da proteção após o reencontro.
- `kObstacleExitHeadingReentryMarginDegrees`: margem para dentro do setor antes
  de aceitar novamente a faixa; o padrão é 10 graus.
- `kObstacleExitSearchTimeoutMs`: tempo máximo de cada busca, atualmente 3000 ms.

Os padrões são 60 graus para cada lado e 2000 milissegundos. Ao alcançar um
extremo, o comando é substituído por giro para dentro do setor. A direção da
correção permanece fixa durante a busca. O reencontro exige três frames novos
com Fusion coerente com essa direção e orientação dentro da margem do setor.
Perda de sensores e timeout continuam exigindo parada de segurança.

O cenário físico que motivou essa correção foi testado com sucesso pela equipe.
Outros cenários de continuação ainda precisam de testes. Antes do teste no chão,
confira os giros com rodas suspensas e use potência reduzida.

## Cenários de recuperação

| Cenário | Sequência |
|---|---|
| Faixa inferior durante a curva | Três frames novos e consecutivos com Fusion válido e `obstacleContinuationBand` interrompem a curva; avança 5 cm e gira para o lado escolhido até confirmar três novos frames Fusion |
| Sem faixa durante a curva | Completa a curva, pausa, gira 25°, avança 6 cm, gira mais 10° procurando Fusion, avança procurando por 1500 ms e inicia a busca final por até 1500 ms |
| GAP/LOST após saída nominal | Com memória lateral válida, abre janela de 2000 ms a partir do reencontro; três GAP/LOST novos fazem parar e consultar três frames NEAR da CAM1 |
| Continuação frontal presente | Dois votos NEAR em três devolvem o tratamento ao GAP/LOST normal, sem busca lateral |
| Continuação frontal ausente | Avança 5 cm e gira para o melhor lado salvo durante a curva; em 65° rejeita a faixa e inverte a busca; não aceita Fusion no retorno até cruzar o yaw inicial |
| Fusion vira contra a memória lateral | Dentro da janela, 35° para o lado errado bloqueiam o Fusion e iniciam recuperação para o lado salvo; a confirmação exige steering coerente com esse lado |

O melhor lado do scan define o contorno; o melhor lado salvo pela parábola
define a recuperação do caso 3. São decisões distintas, preservadas da `main`.
A janela do caso 3 só é armada se houver evidência lateral válida da CAM1.
A recuperação antecipada limpa essa memória, como no comportamento anterior.

A saída nominal ainda aceita presença de Fusion sem uma validação completa
do sentido do percurso. A proteção do caso 3 depende da memória lateral e da
janela temporal; não garante, sozinha, que toda faixa traseira será rejeitada.

## Calibração

Edite somente `include/obr/config.h`, na seção `kObstacle...`. Distâncias usam
centímetros; ângulos usam graus; tempos usam milissegundos; potências são
normalizadas. O mesmo módulo de ângulo e a mesma potência atendem aos dois lados.

| Etapa | Constantes principais |
|---|---|
| Detecção e rearmamento | `kObstacleDetectionDistanceCm`, `kObstacleDetectionConfirmationSamples`, `kObstacleRearmDistanceCm`, `kObstacleRearmConfirmationSamples` |
| Ré inicial | `kObstacleReverseDistanceCm`, `kObstacleReversePower`, `kObstacleInitialReverseMaximumMs` |
| Escolha do lado | `kObstacleForceLeftSide`, `kObstacleClearanceScanDegrees`, `kObstacleClearanceSettleMs`, `kObstacleClearanceRequiredSamples`, `kObstacleClearanceTieCm`, `kObstacleDefaultSideIsRight` |
| Aproximação e primeira reta | `kObstacleSideApproachDegrees`, `kObstacleSelectedForwardDistanceCm`, `kObstacleSelectedForwardPower`, `kObstacleSelectedForwardMaximumHeadingCorrection` |
| Curva | `kObstacleCurveDistanceCm`, `kObstacleCurveEndOffsetDegrees`, `kObstacleCurveBasePower`, `kObstacleCurveMaximumHeadingCorrection` |
| Giro nominal de saída | `kObstacleExitPivotWaitMs`, `kObstacleExitPivotDegrees` |
| Primeiro avanço de saída | `kObstacleExitForwardDistanceCm`, `kObstacleExitForwardPower` |
| Giro procurando Fusion | `kObstacleExitFusionTurnDegrees`, `kObstacleExitFusionTurnPower` |
| Segundo avanço procurando Fusion | `kObstacleExitFusionForwardTimeoutMs`, `kObstacleExitForwardPower` |
| Busca nominal | `kObstacleExitSearchTimeoutMs`, `kObstacleExitSearchMaximumDegrees`, `kObstacleExitSearchOppositeMaximumDegrees`, `kObstacleExitSearchPower` |
| Faixa durante a curva | `kObstacleEarlyFusionRecoveryEnabled`, `kObstacleFusionReacquireConfirmationFrames`, `kObstacleReacquireForwardDistanceCm` |
| Potências de recuperação | `kObstacleReacquireForwardPower`, `kObstacleReacquireSearchPower` |
| Memória e caso 3 | `kObstacleCase3RecoveryEnabled`, `kObstacleCase3FusionWindowMs`, `kObstacleParabolaMinimumBlackPixels`, `kObstacleParabolaMinimumDominance` |
| GAP/LOST e avanço do caso 3 | `kObstacleParabolaGapLostConfirmationFrames`, `kObstacleParabolaNearValidationFrames`, `kObstacleParabolaNearRequiredVotes`, `kObstacleParabolaReacquireForwardDistanceCm` |
| Limites angulares de recuperação | `kObstacleParabolaRearBlockDegrees`, `kObstaclePostObstacleFusionReturnLimitDegrees`, `kObstacleReacquireSearchMaximumDegrees` |
| Giros e segurança | `kObstacleTurnCommandPower`, `kObstacleTurnToleranceDegrees`, `kObstacleTurnTimeoutMs`, `kObstacleEncoderFreshnessMs`, `kObstacleDistanceSafetyTimeoutMs`, `kObstacleBrakePredictionSeconds` |

A centralização inicial usa as constantes `kLineCentering...`; a confirmação
de preto do scan usa `kObstacleCameraBlack...`. Os detectores de massa preta da
CAM1 mantêm seus limiares próprios em `scripts/vision/obstacle_black.py` e
`scripts/vision/parabola_black.py`, pois operam em pixels e intensidade de imagem.

E-Stop, Parar e indisponibilidade da ESP32 permanecem acima do módulo. As fases
de giro, avanço e saída possuem verificações de sensores e timeouts próprios.
O verde é processado antes do obstáculo em `LineCourseMission` e pode cancelar
um desvio já iniciado. Portanto, o módulo não possui prioridade sobre o verde.

As constantes sem uso da antiga sequência de 25/30/21,5 cm foram removidas
para não confundir a calibração. `ObstacleAvoidance(true)` força LEFT;
`ObstacleAvoidance(false)` mede os lados. As manobras seguintes são as mesmas.

Consulte `docs/DIAGNOSTICO_DESVIO_OBSTACULO_2026-09-17.md` para os registros
anteriores a esta alteração.

## Build e testes

No Windows, com MinGW disponível no PATH:

```powershell
cmake -S . -B build/obstacle-review -G "MinGW Makefiles" -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build/obstacle-review --target robot_test obstacle_avoidance_test main_mission_test --parallel 4
ctest --test-dir build/obstacle-review -R '^(obstacle_avoidance_test|main_mission_test)$' --output-on-failure --parallel 2
& ./build/obstacle-review/main_mission_test.exe --obstacle-only
```

Na validação desta alteração, o build, a suíte do obstáculo e a integração
`--obstacle-only` passaram. O teste completo `main_mission_test` falhou em
`testExitAcquisitionRestoresFollower`, com “A integração não confirmou a saída”.
A mesma falha foi reproduzida em uma cópia limpa do commit anterior `b273f65`,
sem estas alterações; portanto, o teste geral ainda tem uma pendência prévia
na saída da sala de resgate.

Antes da pista, teste com as rodas suspensas a seleção dos dois lados, E-Stop,
Parar e perda de sensores. Depois, use potência reduzida que ainda vença o
atrito dos motores e grave chegadas retas e diagonais, com e sem faixa durante
a curva. Verifique o reencontro, GAP/LOST verdadeiro e falso, os limites de busca
e a continuação para a frente. O deploy continua em `scripts/deploy.ps1` ou
`scripts/deploy.sh`.

## Dependências ao exportar

O módulo usa interfaces existentes do projeto:

- `Esp32TelemetrySnapshot`, de `obr/esp32_bridge.h`;
- `ImuTurnController`, de `obr/imu_turn_controller.h` e seu arquivo `.cpp`.
- `CameraLineSnapshot` e `ForwardLineSnapshot`, de `obr/camera_monitor.h`;
- `LineCenteringController`, de `obr/line_centering_controller.h` e seu `.cpp`;
- calibrações centralizadas em `obr/config.h`.

No projeto de destino, preserve esses nomes ou crie um adaptador com os campos
de ultrassônico, idade da telemetria, contagens/taxas dos encoders, yaw, giroscópio
e estado do MPU6050.

Adicione `src/robot/obstacle_avoidance.cpp` ao build, inclua `include/` e
chame `ObstacleAvoidance::update()` em cada ciclo. Quando `hasControl` for
verdadeiro, aplique `leftPower` e `rightPower`. Quando `failed` for verdadeiro,
pare o robô. Quando `completed` for verdadeiro, devolva o controle ao seguidor.

O exemplo completo de integração está em `MainMission` deste repositório. Os
nomes de debug exibidos no painel estão mapeados em `DashboardServer`.
