# Tchauzinho do desafio surpresa

No dashboard, selecione **SERVOS · TCHAUZINHO (BÔNUS)** e inicie o modo Auto para testar o gesto com o robô parado. Ele leva o braço a 105°, move o pulso de 0° a 30° e de volta a 0° três vezes, e termina com braço em 15° e pulso em 0°. A rotina não altera a garra; no modo isolado, a partida Auto coloca a garra em 0°. O botão Parar e a emergência cancelam a rotina.

O tchauzinho no início após calibrar está desativado. A **Missão Principal** segue normalmente ao receber Start. Se `kWaveBonusAtMissionStartEnabled` for reativado, a primeira partida após uma calibração bem-sucedida da ESP32 executará o gesto completo uma vez, com os motores parados, antes de seguir para a missão. Funciona pelo botão físico e pelo Auto do dashboard. Starts posteriores, inclusive após Stop, emergência ou nova calibração, não repetem esse bônus enquanto o programa estiver rodando. A tentativa é consumida ao iniciar: cancelar o gesto também não o rearma. Reiniciar o programa rearma o bônus, que novamente exige calibração confirmada. A calibração sozinha não inicia o gesto; ele aguarda a partida da missão principal.

Os três gatilhos automáticos ficam desligados em `include/obr/config.h`:

```cpp
constexpr bool kWaveBonusAtMissionStartEnabled = false;
constexpr bool kWaveBonusAfterObstacleEnabled = false;
constexpr bool kWaveBonusAfterFinishEnabled = false;
```

Mude para `true` apenas o evento pedido na prova e faça o deploy. Após um desvio na missão principal, o gesto começa quando a faixa de saída foi confirmada; o robô pausa a tração e depois retoma o segue-linha. Na chegada vermelha, os motores permanecem bloqueados pela trava de missão concluída; somente os servos executam o gesto e são desligados ao terminar.

Para inserir o bônus em outra etapa autônoma, chame `robotState.requestWaveBonus()` uma vez na **transição do evento confirmado**, antes de publicar o próximo comando de movimento:

```cpp
if (eventJustCompleted)
{
    robotState.requestWaveBonus();
    return;
}
```

O `MissionController` pausa a missão ativa, renova motor zero, executa o mesmo `ServoRoutineKind::Wave` e devolve o controle à fase que estava em andamento. Não chame a função em cada ciclo do loop, pois isso iniciaria outro gesto após o término. Verifique o retorno `bool` se a etapa precisar registrar que a solicitação foi aceita.

Antes do teste na pista, confira a folga do braço até 105° e do pulso até 30° com os motores sem contato com o chão. Escolha uma etapa em que o mecanismo e qualquer vítima carregada tenham espaço para esse movimento. A ausência de telemetria recente ou do PCA9685 cancela o bônus; o E-Stop tem prioridade e zera os motores imediatamente.
