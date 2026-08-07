# Firmware principal ESP32 + Raspberry Pi

Abra `obr_esp32_main.ino` no Arduino IDE para gravar o firmware usado no robô.
Ele não cria rede Wi-Fi, não inicia servidor HTTP e não hospeda dashboard.

A ESP32 controla GPIO, motores e sensores. A Raspberry Pi controla o movimento
pela UART de 115200 bps e recebe telemetria a cada 100 ms.

## Ligação UART

| Raspberry Pi | ESP32 | Direção |
| --- | --- | --- |
| GPIO14 / TXD | GPIO3 / RX | Raspberry envia comandos |
| GPIO15 / RXD | GPIO1 / TX | ESP32 envia telemetria |
| GND | GND | Referência elétrica comum |

Desative o console serial da Raspberry antes de conectar. Durante a gravação
da ESP32, mantenha a Raspberry desligada ou desconectada da UART0.

## Comandos recebidos

```text
MOTOR,<esquerda>,<direita>,<emergencia>,<retaMin>,<retaMax>,<giroMin>,<giroMax>
STOP
ESTOP
CLEAR_ESTOP
RESET_ENCODERS
CALIBRATE_SENSORS
PING
```

As potências e os limites ficam entre `-1.000` e `1.000`. O perfil padrão usa
55%–60% em reta e 70%–80% em giro. O firmware bloqueia movimento unilateral: se
somente um lado for diferente de zero, o comando é convertido em giro com os
dois lados em sentidos opostos. O formato antigo de `MOTOR` com três campos
continua aceito e preserva os limites já ativos na ESP32.

Se nenhum comando de motor chegar durante 500 ms, os dois lados são zerados.
O E-Stop permanece travado até `CLEAR_ESTOP` e tem prioridade sobre qualquer
comando de movimento.

Enquanto os dois lados recebem potência, a ESP32 também compara o avanço dos
dois encoders em janelas de 250 ms. Duas janelas desequilibradas iniciam uma
recuperação automática: o nSLEEP continua HIGH, os dois PWMs são sincronizados e
recebem um pulso de 100% por 180 ms. O comando original volta automaticamente,
sem travar Manual ou Autônomo. E-Stop, `STOP` e timeout continuam interrompendo
qualquer recuperação imediatamente.

`CALIBRATE_SENSORS` para os motores, zera encoders, ângulos e filtros de
navegação e mede novamente os desvios do giroscópio. O mesmo processo começa ao
manter o botão Start no GPIO27 pressionado continuamente por 5 segundos. O robô
deve permanecer imóvel durante a calibração. Depois dela, a trava local mantém
os motores parados até a Raspberry enviar `CLEAR_ESTOP` como parte de uma nova
habilitação explícita; um comando de movimento antigo não é reaplicado.

## Respostas

```text
READY
PONG
ERR,<motivo>
CALIBRATION,START
CALIBRATION,DONE
CALIBRATION,FAILED
START_BUTTON,SHORT
TRACTION_RECOVERY,LEFT
TRACTION_RECOVERY,RIGHT
SENSOR,<campos CSV...>
```

`START_BUTTON,SHORT` é emitido ao soltar o botão depois de um toque válido. A
Raspberry usa esse evento para iniciar a missão autônoma selecionada. O evento
não é emitido quando a pressão alcança os 5 segundos da calibração, impedindo que
uma tentativa de calibrar também coloque o robô em movimento.

Os primeiros campos de `SENSOR` preservam o protocolo anterior. Os demais
incluem bateria, encoders, botão, PCA9685, potências aplicadas, velocidades dos
encoders, rampa, giroscópio, temperatura, OLED, nSLEEP, E-Stop, uptime e a
recuperação de tração.

Ordem completa dos campos:

```text
SENSOR,
distanciaCm,gyroZ,yawZ,accelX,accelY,accelZ,mpuOk,
bateriaV,encoderEsquerdo,encoderDireito,startButton,pcaOk,
potenciaEsquerda,potenciaDireita,taxaEsquerda,taxaDireita,
rampa,gyroX,gyroY,temperaturaImu,oledOk,nSleepHigh,estop,
bateriaAdcMillivolts,uptimeMs,calibracaoAtiva,recuperacaoTracao,ladoRecuperacao,
retaMin,retaMax,giroMin,giroMax
```

A tensão da bateria não é zerada durante a calibração porque é uma medição
absoluta usada para acompanhar a alimentação do robô.
