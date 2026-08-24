# Relatório de Diagnóstico — Perda de Força no Drive em Modo Autônomo

**Projeto:** Soul / OBR2026K  
**Branch de investigação:** `feature/geometric-line-guidance`  
**Data:** 24/08/2026  
**Status:** Arquivado temporariamente — falha não voltou a se repetir após correções físicas

---

## 1. Objetivo

Investigar uma falha intermitente observada durante o modo autônomo em que um dos lados do drive aparentava receber comando normal de potência, porém os motores daquele lado apresentavam força muito abaixo do esperado.

O comportamento observado era semelhante a um PWM extremamente baixo:

- o motor traseiro daquele lado ainda conseguia girar, porém muito fraco;
- o motor dianteiro/omni não conseguia vencer a carga e praticamente não girava;
- o problema parecia ocorrer somente no autônomo;
- em Tuning e Teleoperado os motores conseguiam ser estressados normalmente.

A investigação teve como objetivo separar uma possível falha de software de uma falha elétrica/mecânica.

---

## 2. Arquitetura relevante

O drive é controlado por **lado**, com apenas dois comandos lógicos principais:

- `LEFT`
- `RIGHT`

Cada lado possui dois motores físicos alimentados pelos dois canais de um DRV8833.

Os dois motores do mesmo lado recebem o mesmo comando lógico de potência. Portanto, o software não possui controle independente entre motor dianteiro e traseiro de um mesmo lado.

Fluxo simplificado:

```text
Visão / algoritmo autônomo
        ↓
LEFT / RIGHT
        ↓
MotorController
        ↓
UART
        ↓
ESP32
        ↓
DRV8833 do lado
        ↓
Motor A + Motor B
```

---

## 3. Sintoma observado

Durante certas curvas no modo autônomo:

- o comando exibido pelo sistema permanecia alto;
- o lado afetado ficava fisicamente muito fraco;
- um dos motores conseguia apenas se arrastar;
- o outro não tinha torque suficiente para iniciar movimento;
- o DRV8833 correspondente apresentava aquecimento;
- o problema não era reproduzido de forma consistente em Tuning ou Teleoperado.

Inicialmente foi considerada a hipótese de que o dashboard estivesse escondendo oscilações rápidas de comando devido à diferença entre a taxa de atualização visual e a taxa de controle dos motores.

---

## 4. Instrumentação adicionada

Foi implementada instrumentação temporária `AUTONOMOUS_MOTOR_TRACE` para registrar o caminho completo do comando durante o modo autônomo.

Entre os campos monitorados:

- comando da visão;
- origem do controle (`geometric`, `gap-forward`, `virtual-green`);
- comando solicitado;
- comando final após piso/sincronismo;
- direção;
- PWM final;
- estado da UART;
- telemetria ESP32;
- E-Stop;
- watchdog;
- timeout;
- prioridade/origem do comando.

A instrumentação também confirmou que Autônomo e Teleoperado comum utilizam o mesmo caminho final de potência, enquanto `drive_raw` possui comportamento específico de manutenção/diagnóstico.

---

## 5. Resultado do trace

No trecho capturado durante a investigação, o comando permaneceu estável por dezenas de frames.

Exemplo recorrente:

```text
visionLeft=-0.200
visionRight=0.750

requestedLeft=-0.200
requestedRight=0.750
```

Após o tratamento do controlador:

```text
leftPower=-0.610 / -0.670
rightPower=0.750
```

A ESP32 continuou reportando comandos aplicados equivalentes, sem sinais de falha lógica:

```text
uartOpen=1
driverEnabled=1
esp32Estop=0
watchdogTimedOut=0
uartSource=raspberry
```

Não foram observados no trecho analisado:

- PWM zerando inesperadamente;
- inversões aleatórias de direção;
- timeout de comando;
- atuação do watchdog;
- E-Stop;
- perda da UART;
- troca inesperada da fonte de comando.

Isso enfraqueceu fortemente a hipótese de que o algoritmo autônomo estivesse simplesmente deixando de enviar potência.

---

## 6. Oscilação 0.67 ↔ 0.61

Foi observada uma alternância periódica entre:

- `0.67`: piso de partida;
- `0.61`: piso após movimento confirmado.

Mesmo com o comando visual permanecendo constante, o valor final alternava entre esses dois níveis.

Essa oscilação foi registrada como uma possível anomalia/efeito da máquina de estados de movimento e merece investigação futura caso volte a ter impacto prático.

Entretanto, ela **não explica sozinha** o sintoma principal, pois ambos os valores ainda representam potência significativa e não correspondem a uma condição de PWM quase nulo.

---

## 7. Hipóteses consideradas

Durante a investigação foram consideradas:

1. oscilação rápida do comando autônomo;
2. atuação de watchdog/timeout;
3. E-Stop ou perda de UART;
4. proteção térmica/sobrecorrente do DRV8833;
5. queda de tensão na alimentação do driver;
6. falha de um canal específico do DRV8833;
7. mau contato em motor/cabos/conectores;
8. reversão brusca de direção em pivot;
9. estado incorreto do piso de partida/movimento;
10. necessidade de recuperação automática baseada em encoder.

O trace descartou ou enfraqueceu principalmente as hipóteses 1, 2 e 3 no evento analisado.

---

## 8. Descoberta física posterior

Após a investigação de software, foram encontrados problemas físicos relevantes:

- um motor omni estava girando para trás quando deveria girar para frente;
- esse mesmo conjunto apresentava problema de mau contato.

Após correção/verificação dessas condições, vários testes foram repetidos e a falha original **não voltou a ocorrer**.

Por isso, neste momento não há evidência suficiente para justificar uma alteração permanente no controle autônomo ou uma rotina automática de recuperação.

A causa raiz não é considerada formalmente comprovada, mas o mau contato e a condição incorreta do motor são atualmente os principais suspeitos.

---

## 9. Recuperação automática considerada, mas não implementada

Foi discutida uma possível estratégia baseada em encoder para detectar:

```text
comando alto
+
velocidade real muito abaixo do esperado
=
weak drive / stall parcial
```

A recuperação proposta seria:

1. detectar velocidade anormalmente baixa por um curto período;
2. zerar temporariamente o lado afetado;
3. reaplicar o comando autônomo atual;
4. se necessário, aplicar um único kick curto;
5. verificar recuperação;
6. entrar em cooldown;
7. impedir loops infinitos de reset/kick.

Essa estratégia foi **adiada**.

Motivo: após a correção física, o problema deixou de se repetir. Adicionar agora uma camada de recuperação aumentaria a complexidade sem uma falha reproduzível que justificasse sua validação.

---

## 10. Estado atual

**Status: arquivado temporariamente.**

No momento:

- o robô voltou a operar sem reproduzir a perda de força;
- não será adicionada recuperação automática por encoder;
- não será alterado o algoritmo de visão;
- não será alterado o controle de pivot;
- não será alterada a potência por causa deste incidente;
- a instrumentação diagnóstica pode ser mantida temporariamente caso seja útil em novos testes.

---

## 11. Critério para reabrir esta investigação

Reabrir este diagnóstico caso qualquer uma das seguintes condições volte a ocorrer:

- um lado apresentar força muito abaixo do esperado no autônomo;
- o comando lógico permanecer alto enquanto a velocidade real despenca;
- o DRV8833 aquecer de forma anormal;
- um motor do lado parar enquanto o outro apenas se arrasta;
- o problema voltar a ocorrer somente no autônomo;
- a alternância `0.67 ↔ 0.61` passar a ter efeito perceptível no movimento.

Se a falha voltar, o primeiro passo deve ser:

1. capturar `AUTONOMOUS_MOTOR_TRACE`;
2. registrar simultaneamente velocidade dos encoders;
3. verificar conectores, soldas e alimentação;
4. comparar o comportamento em Autônomo, Teleoperado e `drive_raw`;
5. somente depois considerar uma rotina automática de recuperação.

---

## 12. Conclusão

O trace demonstrou que, no evento analisado, o caminho de software continuava entregando comandos válidos até a ESP32.

A falha física observada não foi acompanhada por evidência de:

- perda de comando;
- timeout;
- watchdog;
- E-Stop;
- queda lógica do PWM.

Posteriormente foi identificado um motor omni com sentido incorreto e problema de mau contato.

Após os ajustes físicos, a falha não voltou a se repetir nos testes realizados.

Por isso, a decisão atual é **não adicionar correções automáticas ao software e arquivar o diagnóstico**, mantendo este relatório como referência caso o comportamento volte a aparecer.
