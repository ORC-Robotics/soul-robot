# OBR2026K

Código C++ do robô OBR 2026 rodando em uma Raspberry Pi, com dashboard web para
teste, telemetria simples e controle manual dos motores. A Raspberry conversa
por UART com uma ESP32, que controla drivers BTS7960 e lê o MPU6050 e o sensor
ultrassônico.

## Visão geral

O projeto roda um servidor HTTP/WebSocket na Raspberry Pi. Pelo navegador, o
dashboard permite:

- ver o modo atual do robô;
- ver CPU, temperatura e uso de RAM da Raspberry;
- ver distância do ultrassônico, gyro Z, yaw Z e aceleração vindos da ESP32;
- ver a imagem processada pela câmera, quando o script da câmera estiver rodando;
- iniciar/parar o modo manual;
- controlar frente/ré e curva pelos sliders.

O dashboard é uma ferramenta de desenvolvimento e teste. A lógica de segurança
fica no código do robô, não no navegador.

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
  Sketch Arduino da ESP32 para motores, MPU6050, ultrassônico e UART.

include/obr/config.h
  Pinos, portas, limites e constantes do robô.
```

## Comunicação Raspberry Pi e ESP32

A comunicação usa UART em `115200` bps. Na Raspberry Pi, o código abre
`/dev/serial0`, que normalmente usa GPIO14 como TXD e GPIO15 como RXD.

| Raspberry Pi | ESP32 | Função |
| --- | --- | --- |
| GPIO14 / TXD | RX2 / GPIO16 | comandos para a ESP32 |
| GPIO15 / RXD | TX2 / GPIO17 | telemetria da ESP32 |
| GND | GND | referência elétrica comum |

Ative a UART serial da Raspberry sem console de login antes de testar:

```sh
sudo raspi-config
```

Use `Interface Options > Serial Port`, desative o shell pela serial e ative a
porta serial de hardware.

## Pinos da ESP32

O sketch da ESP32 fica em `esp32/obr_esp32_bridge/obr_esp32_bridge.ino`.
No Arduino IDE, instale o pacote da placa ESP32 e as bibliotecas
`Adafruit MPU6050` e `Adafruit Unified Sensor` antes de gravar.

| ESP32 | BTS7960 | Função |
| --- | --- | --- |
| GPIO15 | EN | enable do driver do motor esquerdo |
| GPIO14 | RPWM | PWM do motor esquerdo em um sentido |
| GPIO5 | LPWM | PWM do motor esquerdo no sentido oposto |
| GPIO2 | EN | enable do driver do motor direito |
| GPIO4 | RPWM | PWM do motor direito em um sentido |
| GPIO33 | LPWM | PWM do motor direito no sentido oposto |
| GND | GND | referência elétrica comum |

| ESP32 | Sensor | Função |
| --- | --- | --- |
| GPIO21 | MPU6050 | SDA |
| GPIO22 | MPU6050 | SCL |
| GPIO25 | Ultrassônico frontal | TRIG |
| GPIO35 | Ultrassônico frontal | ECHO |

Se o ultrassônico for HC-SR04 alimentado com 5 V, reduza o sinal de ECHO para
3,3 V antes de ligar na ESP32.

### Teste direto dos motores pela ESP32

Para testar os BTS7960 sem a Raspberry Pi, grave temporariamente o sketch:

```txt
esp32/bts7960_motor_web_test/bts7960_motor_web_test.ino
```

Esse teste cria uma rede Wi-Fi chamada `OBR-Motor-Test`, com senha `obr2026k`.
Depois de conectar nela, abra:

```txt
http://192.168.4.1
```

Use esse sketch somente para teste de bancada. Ele limita a potência em `0.80`,
zera os motores ao abrir a página e desliga as saídas se parar de receber
comandos por mais de 500 ms. Depois do teste, grave novamente o sketch principal
`esp32/obr_esp32_bridge/obr_esp32_bridge.ino` para voltar à comunicação com a
Raspberry Pi.

O site também aceita controle compatível com navegador. O gatilho direito move
para frente, o gatilho esquerdo dá ré e o analógico esquerdo gira o robô. Se o
controle não aparecer, pressione algum botão com a página aberta para o navegador
liberar o acesso ao dispositivo.

Quando o MPU6050 estiver ligado no I2C da ESP32, o teste tenta manter o eixo do
robô automaticamente enquanto houver aceleração/ré e o analógico de giro estiver
solto. Se a correção piorar o desvio, inverta `HEADING_HOLD_CORRECTION_SIGN` no
sketch de teste.

## Segurança dos motores

O comportamento esperado é:

- ao ligar, os motores começam parados;
- `Stop` zera os comandos e mantém o robô em modo parado;
- comandos de movimento só são aceitos depois de `Start`;
- se o dashboard parar de enviar comandos, os motores param por timeout;
- se a Raspberry ou a UART pararem de enviar comandos, a ESP32 também para os motores;
- todos os comandos de motor são limitados entre `-1.0` e `1.0`;
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
da câmera, a resolução, a qualidade JPEG e o erro horizontal da linha vêm de
`/camera-status.json`.

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

O IntelliSense usa `.vscode/c_cpp_properties.json` para encontrar os headers em
`include/`.

## Commit sugerido

```sh
git add .
git commit -m "Add Raspberry deploy flow and robot dashboard control"
git push
```
