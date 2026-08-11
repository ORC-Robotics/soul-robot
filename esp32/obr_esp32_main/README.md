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
MOTOR,<esquerda>,<direita>,<emergencia>
STOP
ESTOP
CLEAR_ESTOP
RESET_ENCODERS
CALIBRATE_SENSORS
SYSTEM_STARTING
SYSTEM_READY
OLED,<duracaoMs>,<tituloHex>,<linha1Hex>,<linha2Hex>
OLED_CLEAR
PING
```

`OLED` mostra uma página temporária enviada pela Raspberry. O título aceita até
12 caracteres ASCII, cada linha aceita até 20 e a duração fica entre 500 e
30.000 ms. Os textos usam hexadecimal para não conflitar com as vírgulas do
protocolo; um campo vazio é enviado como `-`. `OLED_CLEAR` retorna imediatamente
à página local de bateria e ângulos. E-Stop e calibração continuam tendo
prioridade visual sobre qualquer página remota.

`SYSTEM_STARTING` mantém a animação de inicialização ativa. Depois que UART,
câmera e serviços estão prontos, a Raspberry envia `SYSTEM_READY` a cada segundo.
Se esse heartbeat desaparecer por mais de 3 segundos, a ESP32 volta à animação.
Esse estado é apenas visual e não substitui os timeouts e travas dos motores.

As potências ficam entre `-1.000` e `1.000` e são aplicadas diretamente ao PWM.
Os lados esquerdo e direito são independentes: `MOTOR,0.050,0.000,0` aplica 5%
somente ao lado esquerdo. Não existe perfil, mínimo, boost ou recuperação automática.

Se nenhum comando de motor chegar durante 500 ms, os dois lados são zerados.
O E-Stop permanece travado até `CLEAR_ESTOP` e tem prioridade sobre qualquer
comando de movimento.

O nSLEEP permanece HIGH durante a operação. Partidas e inversões aplicam diretamente
a potência recebida. E-Stop, `STOP` e timeout continuam zerando os motores imediatamente.

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
OLED,OK
OLED,CLEARED
SENSOR,<campos CSV...>
```

`START_BUTTON,SHORT` é emitido ao soltar o botão depois de um toque válido. A
Raspberry usa esse evento para iniciar a missão selecionada quando estiver parada.
Em Manual/Autônomo, a telemetria do botão pressionado para o robô imediatamente
e o evento curto posterior é consumido. O evento não é emitido quando a pressão
alcança os 5 segundos da calibração, impedindo que uma tentativa de calibrar
também coloque o robô em movimento.

Os primeiros campos de `SENSOR` preservam o protocolo anterior. Os demais
incluem bateria, encoders, botão, PCA9685, potências aplicadas, velocidades dos
encoders, rampa, giroscópio, temperatura, OLED, nSLEEP, E-Stop e uptime.

Ordem completa dos campos:

```text
SENSOR,
distanciaCm,gyroZ,yawZ,accelX,accelY,accelZ,mpuOk,
bateriaV,encoderEsquerdo,encoderDireito,startButton,pcaOk,
potenciaEsquerda,potenciaDireita,taxaEsquerda,taxaDireita,
rampa,gyroX,gyroY,temperaturaImu,oledOk,nSleepHigh,estop,
bateriaAdcMillivolts,uptimeMs,calibracaoAtiva,oledRemotaAtiva,sistemaRaspberryPronto
```

A tensão da bateria não é zerada durante a calibração porque é uma medição
absoluta usada para acompanhar a alimentação do robô.
