# Rotina autônoma da sala de resgate

Este documento descreve a sequência executada pela `MainMission` depois que a
faixa cinza confirma a entrada da sala. A busca da saída ainda não faz parte
desta etapa: quando não restarem vítimas, o robô marca a missão como concluída e
permanece parado.

## Ordem da missão

1. Liga o YOLO solicitando exclusivamente `silver_ball`.
2. Aguarda o primeiro resultado atual do detector e avança 10 cm por encoders.
3. Se não houver vítima viva, observa o centro e varre `+45°`, `-45°`, `+75°`
   e `-75°`. Cada limite é seguido por uma parada e por um frame novo.
4. Ao confirmar uma vítima, cancela a varredura e faz primeiro o alinhamento
   visual, ainda sem movimentar os servos.
5. Depois do alinhamento, leva braço/pulso/garra à posição de coleta com a garra
   aberta e só então libera a aproximação pelo YOLO.
6. Depois que o YOLO conclui a aproximação, executa o fechamento e a retenção
   já validados da garra. A pose de `5°` é renovada durante a subida do braço,
   a ré e a procura do triângulo, até o passo explícito de soltura.
7. Recua até 15 cm, com potência 0,80 e medição dos dois encoders. Esse recuo
   acontece depois de todas as coletas, inclusive depois da segunda vítima viva.
8. Armazena internamente a primeira vítima viva e repete a busca da segunda.
9. Depois de fechar a garra na segunda vítima e recuar 15 cm, encontra, alinha e
   aproxima o triângulo verde.
10. Executa o trecho já validado que entrega a vítima carregada, retira a vítima
   armazenada e também a entrega no triângulo verde.
11. Recua 20 cm e passa a solicitar exclusivamente `black_ball`.
12. Coleta a vítima morta, recua 15 cm, encontra o triângulo vermelho, entrega e
    recua 40 cm antes da verificação final.
13. Faz uma verificação final completa por vítimas vivas e depois mortas. Uma
    vítima extra encontrada é coletada e entregue na cor correspondente; após a
    entrega, a verificação recomeça pelas vivas.
14. Quando as duas varreduras finais terminam sem alvo, zera a tração e publica
    `rescue_room_completed`.

Uma vítima obrigatória ausente após as duas amplitudes não é tratada como missão
concluída: o robô reinicia a varredura para o mesmo tipo. Isso evita declarar
sucesso ou encerrar a missão por uma única detecção perdida.

## Integração dos servos

Os ângulos, tempos e ordem mecânica validados permanecem em `ServoRoutine`.
Foram expostos somente pontos de pausa da mesma programação:

- `PrepareCapture`: prefixo da coleta até `15°/180°/180°`, seguido do braço em
  `103°`, mantendo a garra aberta para a aproximação;
- `SecureCapture`: fechamento em `0°`, retenção em `5°` e retorno a `15°`;
- `SecureCaptureForDirectDeposit`: o mesmo fechamento da sequência direta,
  terminando o braço em `0°`;
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
- Troca entre vítimas: uma nova geração limpa o tracker e impede reutilizar o
  alvo anterior.
- Busca angular: depende de IMU atual; uma falha encerra a missão parada.
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
- `kRescuePostCollectionReverseDistanceCm` e potência correspondente;
- `kRescuePostDepositReverseDistanceCm` e potência correspondente;
- `kRescueFinalDeadDepositReverseDistanceCm`, usado antes da busca final;
- timeouts e tolerâncias com prefixo `kRescueDistance`.

## Checklist de teste físico

1. Testar sem vítimas e com as rodas suspensas; conferir a ordem angular e a
   parada em cada limite.
2. Com motores no chão e servos desligados mecanicamente, medir os 10 cm, 15 cm,
   20 cm e 40 cm em pelo menos três tensões de bateria.
3. Com uma vítima prata, confirmar que a garra chega aberta à posição de coleta
   antes de qualquer aproximação.
4. Testar a primeira prata isoladamente e conferir o armazenamento interno.
5. Testar duas pratas e conferir ambas as liberações somente depois que o
   triângulo verde for alcançado.
6. Testar uma vítima preta e confirmar que o triângulo vermelho é selecionado.
7. Acrescentar uma quarta vítima e confirmar que a verificação final a coleta e
   reinicia pelas vítimas vivas.
8. Confirmar que cada vítima cai completamente no buraco do triângulo e deixa
   de aparecer para a CAM1 antes da busca seguinte.
9. Em cada fase móvel, acionar Stop e E-Stop; todos os motores devem zerar no
   mesmo ciclo e os servos devem continuar sustentando a última pose.
10. Simular perda do heartbeat sem desconectar a alimentação dos servos;
   confirmar que o braço não cai e que nenhum movimento novo é aceito.
11. Não desconectar o PCA9685 nem a alimentação com o braço carregado: nenhuma
   proteção de software consegue sustentar o mecanismo sem energia física.
