# Rotina autônoma da sala de resgate

Este documento descreve a sequência executada pela `MainMission` depois que a
faixa cinza confirma a entrada da sala. Quando termina a verificação das vítimas
extras, a `MainMission` mantém o robô parado durante a transição e inicia a busca
da saída pelos corners.

## Ordem da missão

1. Liga o YOLO solicitando exclusivamente `silver_ball`.
2. Aguarda o primeiro resultado atual do detector e avança 10 cm por encoders.
3. Se não houver vítima viva confirmada, começa pelo lado da última candidata
   vista na entrada; sem indicação, começa à esquerda. Varre os dois lados em
   `45°` com até 3 segundos por tentativa e depois em `75°` com até 8 segundos
   por tentativa. Cada limite usa o heading real, seguido de parada e frame novo.
   Um timeout zera os motores antes de trocar de tentativa; quatro timeouts
   encerram a busca com `rescue_search_blocked`. Se os quatro headings forem
   concluídos sem encontrar uma vítima obrigatória, o robô continua girando no
   primeiro sentido escolhido até encontrar uma candidata, sem repetir os ângulos.
   Durante essa busca contínua, se o heading permanecer dentro de uma faixa de
   `15°` por 2 segundos apesar do comando de giro, o robô considera que pode
   estar preso em uma parede e inverte imediatamente o sentido da busca.
4. Ao observar uma candidata, interrompe a varredura para estabilizar os frames
   de confirmação. Se ela piscar, gira de volta ao último heading visto em vez
   de continuar a varredura genérica. Ao confirmar uma vítima, faz o
   alinhamento visual; a aproximação pode começar dentro de ±5° e continua
   corrigindo a direção enquanto persegue o alvo, ainda sem movimentar os servos.
5. Depois do alinhamento, leva braço/pulso/garra à posição de coleta com a garra
   aberta e só então libera a aproximação pelo YOLO.
6. Depois que o YOLO conclui a aproximação, executa o fechamento e a retenção
   já validados da garra. Mantém braço em `103°`, pulso em `180°` e garra em `5°`.
7. Recua até 15 cm, com potência 0,80 e medição dos dois encoders. Esse recuo
   acontece antes de levantar o braço em todas as coletas, incluindo vítimas
   extras. Se a ré falhar, o braço permanece baixo e a garra conserva a retenção.
   Depois da ré, levanta para `15°` com armazenamento ou `0°` para depósito direto.
8. Armazena internamente a primeira vítima viva e repete a busca da segunda.
9. Depois de fechar a garra na segunda vítima e recuar 15 cm, encontra, alinha e
   aproxima o triângulo verde. O alinhamento usa os pulsos da busca YOLO:
   potência 0,72 durante 130 ms, seguida de pausa de 100 ms e frame novo.
   Ao atingir 6 cm ou detectar obstrução/cobertura da câmera, executa avanço
   obrigatório por 1.500 ms a 0,75 antes de liberar o depósito. Pausas por perda
   da ESP32 não consomem esse tempo; falhas de sensores e timeout global bloqueiam
   o depósito. Se a zona desaparecer durante o alinhamento, o robô retorna
   automaticamente à busca giratória. O tempo não comprova distância física
   percorrida.
10. Executa o trecho já validado que entrega a vítima carregada, retira a vítima
   armazenada e também a entrega no triângulo verde.
11. Recua 20 cm e passa a solicitar exclusivamente `black_ball`. Essa busca
    obrigatória começa diretamente como giro contínuo para um único lado, sem
    executar antes os headings alternados de 45° e 75°.
12. Coleta a vítima morta, recua 15 cm, encontra o triângulo vermelho, entrega e
    recua 40 cm antes da verificação final.
13. Faz uma única volta de até `360°`, solicitando qualquer tipo de vítima
    extra. Uma vítima encontrada é coletada e entregue na cor correspondente;
    após a entrega, recua 40 cm e inicia uma nova verificação das que ainda
    possam restar. Cada verificação também termina após 20 segundos para impedir
    que o robô permaneça procurando indefinidamente se o giro ficar bloqueado.
14. Ao completar a volta ou atingir o tempo limite sem alvo, zera a tração,
    publica `rescue_room_completed` e entrega o controle à busca da saída, que
    aponta para um corner usando o heading salvo do último triângulo.

Durante essa rotina, cada novo movimento não reto recebe um único micropulso de
0,80 por 80 ms para vencer a inércia. Depois desse intervalo, o comando retorna
automaticamente às potências originais da etapa. Retas, paradas, segue-linha e
busca da saída não recebem esse reforço.

Uma vítima obrigatória ausente após as duas amplitudes não é tratada como missão
concluída: o robô inicia uma busca contínua para o mesmo tipo. Isso evita declarar
sucesso, repetir os mesmos headings ou encerrar a missão por uma detecção perdida.

## Integração dos servos

Os ângulos e passos mecânicos permanecem em `ServoRoutine`. O pulso usa rampa
de 240°/s e cada passo reserva 750 ms para o braço, 850 ms para o pulso e 300 ms
para a garra. Aperto, reativação e inicialização mantêm os tempos anteriores.
Esses valores exigem validação com carga, pois não existe sensor físico de posição.
As etapas da missão são:

- `PrepareCapture`: prefixo da coleta até `15°/180°/180°`, seguido do braço em
  `103°`, mantendo a garra aberta para a aproximação;
- `GripForReverse`: fechamento em `0°` e retenção em `5°`, sem levantar;
- `LiftAfterReverse`: elevação para `15°`, somente após a ré;
- `LiftAfterReverseForDirectDeposit`: elevação para `0°` após a ré;
- `DepositCarriedAndStored`: trecho final da sequência completa que entrega a
  vítima carregada e depois a vítima guardada internamente.

As missões isoladas dos servos (`Capture`, `InternalStorage`, `Deposit`,
`FullSequence` e `FullSequenceTwo`) não tiveram seus ângulos nem sua ordem
alterados.

No armazenamento da primeira vítima, o pulso conclui a ida para `0°` antes de
a garra receber `90°`. Até esse ponto, a retenção permanece em `5°`.

## Segurança

- YOLO ou IPC desatualizado ou pertencente a outra geração: motores em zero.
- Candidata ainda não travada: motores em zero.
- `candidateTxDegrees` é opcional no IPC e orienta somente a busca. Não libera
  alinhamento ou coleta, e frames antigos ou de outra geração não atualizam a pista.
- Troca entre vítimas: uma nova geração limpa o tracker e impede reutilizar o
  alvo anterior.
- Busca angular: depende de IMU atual; dados inválidos pausam o movimento sem
  renovar o orçamento da tentativa.
- Avanços e rés: dependem dos dois encoders, possuem verificação de diferença,
  stall, timeout e estabilização com PWM zero.
- Servos: ao entrar no resgate, a Raspberry arma `SERVO_HOLD`. A ESP32 mantém
  os três canais com PWM mesmo após perda de heartbeat, Stop ou E-Stop; a
  tração continua zerada. Entrar explicitamente em Manual ou Calibração libera
  a trava para devolver o controle completo ao dashboard.
- Intertravamento mecânico: depois que os canais são confirmados ativos, a
  perda de qualquer `servoEnabled` encerra a sequência antes do próximo passo.
  Assim, o pulso não gira se a posição do braço tiver sido invalidada.
- Triângulos: a percepção de cores só fica ativa durante busca, alinhamento e
  aproximação da área de depósito.

## Constantes para validação física

Todos os valores ajustáveis ficam em `include/obr/config.h`:

- `kRescueEntryAdvanceDistanceCm` e `kRescueEntryAdvancePower`;
- `kRescueVictimFirstSweepDegrees` e `kRescueVictimSecondSweepDegrees`;
- `kRescueVictimFirstSweepTimeoutMs` e `kRescueVictimSecondSweepTimeoutMs`;
- `kRescueZoneApproachFinalAdvanceMs` e `kRescueZoneApproachFinalAdvancePower`;
- `kWristServoMaximumSpeedDegreesPerSecond` e tempos `kServoRoutine*StepMs`;
- `kRescuePostCollectionReverseDistanceCm` e potência correspondente;
- `kRescuePostDepositReverseDistanceCm` e potência correspondente;
- `kRescueFinalDepositReverseDistanceCm`, usado antes e durante a busca final;
- timeouts e tolerâncias com prefixo `kRescueDistance`.

## Checklist de teste físico

1. Testar sem vítimas e com as rodas suspensas; conferir a ordem angular e a
   parada em cada limite.
2. Com motores no chão e servos desligados mecanicamente, medir os 10 cm, 15 cm,
   20 cm e 40 cm em pelo menos três tensões de bateria.
3. Com uma vítima prata, confirmar que a garra chega aberta à posição de coleta
   antes de qualquer aproximação e que o braço só levanta depois dos 15 cm de ré.
4. Testar a primeira prata isoladamente e conferir o armazenamento interno.
5. Testar duas pratas e conferir ambas as liberações somente depois que o
   triângulo verde for alcançado.
6. Testar uma vítima preta e confirmar que o triângulo vermelho é selecionado.
7. Acrescentar uma quarta vítima e confirmar que a verificação final a coleta e
   reinicia uma volta para qualquer tipo, respeitando a mesma ré antes da elevação.
8. Sem vítimas extras, confirmar que o giro termina em 360° ou 20 segundos,
   zera os motores e inicia o apontamento para o primeiro corner da saída.
9. Confirmar que cada vítima cai completamente no buraco do triângulo e deixa
   de aparecer para a CAM1 antes da busca seguinte.
10. Em cada fase móvel, acionar Stop e E-Stop; todos os motores devem zerar no
   mesmo ciclo e os servos devem continuar sustentando a última pose.
11. Simular perda do heartbeat sem desconectar a alimentação dos servos;
   confirmar que o braço não cai e que nenhum movimento novo é aceito.
12. Não desconectar o PCA9685 nem a alimentação com o braço carregado: nenhuma
   proteção de software consegue sustentar o mecanismo sem energia física.

## Diagnóstico e testes locais

Os logs do alinhamento registram mudanças de fase, motivo da espera, sequência
da imagem, direção, potências solicitadas/aplicadas e taxas dos encoders. O avanço
final registra motivo, início, pausa, tempo executado e conclusão. No robô,
acompanhe com `journalctl -u obr-robot -f`.

```powershell
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build
ctest --test-dir build --output-on-failure
python -m unittest discover -s tests/python
```

Antes de usar no piso, executar o checklist com rodas suspensas e depois potência
reduzida. Confirmar especialmente os dois lados da varredura, a ré bloqueada,
a velocidade dos servos carregados e o avanço final com a câmera obstruída.
