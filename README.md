# OBR2026K

Código do robô OBR 2026. A etapa atual pode ser testada somente com a ESP32: ela
cria a própria rede Wi-Fi, serve um dashboard, controla os DRV8833 e lê MPU6050,
PCA9685, ultrassônico, encoders, botão de partida e tensão da bateria. A ponte
UART e o programa C++ da Raspberry Pi permanecem no projeto para a próxima etapa.

## Visão geral

O dashboard autônomo da ESP32 permite:

- acionar os motores esquerdo e direito em teste de bancada;
- ver distância frontal e tensão da bateria;
- acompanhar aceleração, giro, yaw e temperatura do MPU6050;
- acompanhar contagem e taxa dos dois encoders;
- ver o estado do botão, do PCA9685 e dos clientes Wi-Fi;
- usar parada de emergência e timeout de comando sem depender da Raspberry Pi.

O dashboard da Raspberry Pi continua disponível para câmera, telemetria do
sistema e modos manual/autônomo quando ela for integrada novamente.

## Arquitetura

```txt
src/main.cpp
  Conecta os módulos e executa o loop principal.

src/dashboard/
  Dashboard HTTP/WebSocket e comandos vindos do navegador.

src/robot/
  Estado do robô e controle dos motores.

src/hal/
  Acesso baixo nível aos GPIOs da Raspberry Pi e ponte UART com a ESP32.

src/telemetry/
  Leituras simples de telemetria.

esp32/obr_esp32_bridge/
  Sketch principal da ESP32, dashboard e configuração centralizada de pinos.

include/obr/config.h
  Pinos, portas, limites e constantes do robô.
```

## Teste somente com a ESP32

Abra `esp32/obr_esp32_bridge/obr_esp32_bridge.ino` no Arduino IDE. Selecione uma
placa ESP32 compatível e instale:

- `Adafruit MPU6050`;
- `Adafruit Unified Sensor`;
- `Adafruit PWM Servo Driver Library`.

Depois de gravar o sketch:

1. Conecte o celular ou computador à rede `OBR2026K-ESP32`.
2. Use a senha `obr2026k-painel`.
3. Abra `http://192.168.4.1`.
4. Confira os sensores com os motores sem alimentação.
5. Levante as rodas, energize os drivers e clique em `Habilitar motores`.

Os sliders e os botões de frente e ré permitem controlar cada lado de -100% a
100%. Se o navegador deixar de enviar comandos por 500 ms, a ESP32 zera os
motores e remove a habilitação. O
E-Stop permanece travado até o botão `Liberar E-Stop` ser usado; liberar não
volta a movimentar o robô.

O botão do GPIO27 é lido e aparece na telemetria, mas não inicia movimento
sozinho. A ação de competição desse botão será ligada ao modo autônomo quando a
estratégia correspondente for implementada.

## Pinagem da ESP32

As constantes ficam em `esp32/obr_esp32_bridge/robot_config.h`.

| ESP32 | Ligação | Função |
| --- | --- | --- |
| GPIO32 | Ultrassônico TRIG | Disparo frontal |
| GPIO33 | Ultrassônico ECHO | Leitura frontal, somente até 3,3 V |
| GPIO27 | Start button | Entrada com pull-up, botão para GND |
| GPIO14 / GPIO13 | I2C SCL / SDA | MPU6050, PCA9685 e futuro SSD1306 |
| GPIO5 / GPIO18 | DRV8833 esquerdo IN1 / IN2 | Dois motores do lado esquerdo |
| GPIO16 / GPIO17 | DRV8833 direito IN1 / IN2 | Dois motores do lado direito |
| GPIO19 / GPIO21 | Encoder esquerdo A / B | Contagem quadrature nas quatro bordas |
| GPIO22 / GPIO23 | Encoder direito A / B | Contagem quadrature nas quatro bordas |
| GPIO1 / GPIO3 | UART TX / RX | Reservados para a futura Raspberry Pi |
| GPIO36 | Divisor 47 kΩ / 10 kΩ | Leitura ADC1 da bateria de 12 V |

### Cuidados elétricos

- Todos os módulos devem compartilhar GND.
- O ECHO de um HC-SR04 alimentado em 5 V precisa de divisor ou conversor para
  nunca aplicar 5 V ao GPIO33.
- O divisor da bateria usa 47 kΩ entre bateria e GPIO36 e 10 kΩ entre GPIO36 e
  GND. Um capacitor de 100 nF entre GPIO36 e GND ajuda a estabilizar a leitura.
- Os dois motores de cada lado só podem compartilhar a mesma ponte H se a soma
  das correntes, principalmente a corrente de travamento, estiver dentro do
  limite do DRV8833 e da placa usada. Não ligue duas saídas de pontes H em
  paralelo para tentar aumentar corrente.
- Se a placa DRV8833 expuser `nSLEEP`, mantenha esse pino em nível alto; em nível
  baixo as saídas permanecem desligadas.
- GPIO5 participa da inicialização da ESP32. O driver não deve forçar esse pino
  a um nível incompatível enquanto a placa liga.
- GPIO1 e GPIO3 também são usados na gravação e no monitor serial. Quando a
  Raspberry for instalada, desconecte-a ou mantenha a UART dela silenciosa ao
  gravar a ESP32.

O PCA9685 é detectado em `0x40`, configurado em 50 Hz e inicia com os 16 canais
desligados. Ainda não há movimento de servo porque os atuadores e limites de
pulso não foram informados. O MPU6050 é procurado em `0x68` e `0x69`.

## Comunicação futura com a Raspberry Pi

A comunicação usa UART em `115200` bps. Na Raspberry Pi, o código abre
`/dev/serial0`, que normalmente usa GPIO14 como TXD e GPIO15 como RXD.

| Raspberry Pi | ESP32 | Função |
| --- | --- | --- |
| GPIO14 / TXD | RX / GPIO3 | comandos para a ESP32 |
| GPIO15 / RXD | TX / GPIO1 | telemetria da ESP32 |
| GND | GND | referência elétrica comum |

Ative a UART serial da Raspberry sem console de login antes de integrar:

```sh
sudo raspi-config
```

Use `Interface Options > Serial Port`, desative o shell pela serial e ative a
porta serial de hardware.

## Segurança dos motores

O comportamento esperado é:

- ao ligar, os GPIOs dos motores são forçados para LOW antes do `setup()` e
  permanecem parados até receber um comando válido;
- `Desabilitar` e `Parar` zeram os comandos;
- comandos do dashboard direto só são aceitos depois de `Habilitar motores`;
- se o dashboard parar de enviar comandos, os motores param por timeout;
- se a Raspberry ou a UART pararem de enviar comandos, a ESP32 também para os motores;
- os comandos locais são limitados entre `-1.00` e `1.00`;
- o E-Stop tem prioridade sobre dashboard e UART;
- ao encerrar o programa, o código envia `STOP` para a ESP32.

Durante testes, levante as rodas antes de usar valores altos nos sliders.

## Dependências na Raspberry Pi

Instale uma vez:

```sh
sudo apt update
sudo apt install -y build-essential cmake
```

O serviço roda como usuário `obr`. Se os GPIOs não responderem, confira se o
usuário está no grupo `gpio`:

```sh
groups obr
sudo usermod -aG gpio obr
```

Depois de alterar grupos, reinicie a Raspberry.

Para usar a câmera no dashboard, instale também as dependências da Pi Camera e
do OpenCV para Python:

```sh
sudo apt install -y python3-picamera2 python3-opencv python3-numpy
```

## Deploy pelo Windows

Na pasta do projeto:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/deploy.ps1
```

O deploy copia o código para `/home/obr/OBR2026K`, compila na Raspberry e
reinicia o serviço `obr-robot` quando ele já está instalado.

Se precisar escolher o host manualmente:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/deploy.ps1 -HostName 192.168.0.104
```

Para apenas enviar e compilar, sem iniciar o robô:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/deploy.ps1 -NoRun
```

Para não digitar a senha em todo deploy, crie uma chave SSH no computador de
desenvolvimento e instale a chave pública na Raspberry. A senha da Raspberry será
pedida só nessa configuração inicial:

```powershell
ssh-keygen -t ed25519 -f "$env:USERPROFILE\.ssh\obr_raspberry" -N ""
Get-Content "$env:USERPROFILE\.ssh\obr_raspberry.pub" | ssh obr@raspberrypi.local "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys"
```

## Deploy pelo Linux/macOS

```sh
bash scripts/deploy.sh
```

Para apenas enviar e compilar, sem iniciar o robô:

```sh
bash scripts/deploy.sh --no-run
```

No Linux/macOS, use a mesma chave esperada pelo script:

```sh
ssh-keygen -t ed25519 -f ~/.ssh/obr_raspberry -N ""
ssh-copy-id -i ~/.ssh/obr_raspberry.pub obr@raspberrypi.local
```

## Botão de deploy local

Para abrir um painel local com botão de deploy automático:

```sh
python3 scripts/deploy_panel.py
```

Depois abra:

```txt
http://127.0.0.1:8765
```

O botão executa o deploy em modo serviço, equivalente a `scripts/deploy.sh --service`
no Linux/macOS ou `scripts/deploy.ps1 -Service` no Windows. O painel escuta apenas
em `127.0.0.1`, então ele fica disponível só no computador de desenvolvimento.

## Serviço no boot

Depois que o projeto já tiver sido compilado pelo menos uma vez na Raspberry,
instale o serviço:

No Windows:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/install-service.ps1
```

No Linux/macOS:

```sh
bash scripts/install-service.sh
```

Depois disso, a Raspberry inicia o programa automaticamente no boot. O deploy
normal já reinicia o serviço com a versão nova.

Comandos úteis na Raspberry:

```sh
sudo systemctl status obr-robot
sudo systemctl restart obr-robot
journalctl -u obr-robot -f
```

## Dashboard

Depois do serviço subir, abra:

```txt
http://raspberrypi.local:8080
```

Se o nome não resolver na rede, use o IP atual da Raspberry:

```txt
http://192.168.0.104:8080
```

### Imagem da câmera

O dashboard mostra a imagem processada pelo stream MJPEG na própria porta do
dashboard:

```txt
http://raspberrypi.local:8080/camera-stream.mjpg
```

O script Python mantém um servidor local em `127.0.0.1:8090`, e o C++ faz proxy
para `/camera-stream.mjpg`. O endpoint antigo `/camera.jpg` continua disponível
como snapshot de compatibilidade, lendo `/tmp/obr_camera_frame.jpg`. O FPS atual
da câmera, a resolução, a qualidade JPEG, o erro horizontal da linha e a ação de
verde detectada vêm de `/camera-status.json`.

```sh
cd /home/obr/OBR2026K
python3 scripts/camera_line_frame.py
```

Se estiver em outra pasta, use o caminho completo:

```sh
python3 /home/obr/OBR2026K/scripts/camera_line_frame.py
```

Esse script usa a Pi Camera, detecta a linha preta, desenha o contorno e o erro
horizontal, e transmite vídeo em MJPEG para o dashboard. A configuração padrão
usa `960x540`, JPEG `82` e stream alvo de `30 FPS`. A detecção da linha roda em
uma cópia menor da imagem para preservar FPS sem borrar a visualização do
dashboard. Se o script não estiver rodando ou a câmera falhar, o painel continua
funcionando e mostra o aviso de câmera indisponível.

Quando o serviço `obr-robot` estiver instalado com a versão atual dos scripts,
ele inicia esse script automaticamente junto com o robô. Depois de atualizar o
arquivo de serviço, reinstale uma vez pelo computador de desenvolvimento:

```sh
bash scripts/install-service.sh
```

Se você já estiver no terminal da Raspberry, dentro de `/home/obr/OBR2026K`, use:

```sh
sudo cp scripts/obr-robot.service /etc/systemd/system/obr-robot.service
sudo systemctl daemon-reload
sudo systemctl restart obr-robot
```

## VS Code

Atalhos úteis:

- `Terminal > Run Build Task` para rodar `Deploy Raspberry`;
- `Terminal > Run Task > Deploy only Raspberry` para enviar/compilar sem rodar.
- `Terminal > Run Task > Open Deploy Button` para abrir o painel local com botão
  de deploy automático.

O IntelliSense usa `.vscode/c_cpp_properties.json` para encontrar os headers em
`include/`.

## Commit sugerido

```sh
git add .
git commit -m "Add Raspberry deploy flow and robot dashboard control"
git push
```
