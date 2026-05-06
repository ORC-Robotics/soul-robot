# OBR2026K

Código C++ do robô OBR 2026 rodando em uma Raspberry Pi, com dashboard web para
teste, telemetria simples e controle manual dos motores.

## Visão geral

O projeto roda um servidor HTTP/WebSocket na Raspberry Pi. Pelo navegador, o
dashboard permite:

- ver o modo atual do robô;
- ver CPU, temperatura e uso de RAM da Raspberry;
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
  Acesso baixo nível aos GPIOs da Raspberry Pi.

src/telemetry/
  Leituras simples de telemetria.

include/obr/config.h
  Pinos, portas, limites e constantes do robô.
```

## Pinos da ponte H L298N

Os pinos ficam centralizados em `include/obr/config.h` e usam numeração BCM da
Raspberry Pi.

| Raspberry Pi | L298N | Função |
| --- | --- | --- |
| GPIO17 | IN1 | direção do motor esquerdo |
| GPIO27 | IN2 | direção do motor esquerdo |
| GPIO22 | IN3 | direção do motor direito |
| GPIO23 | IN4 | direção do motor direito |
| GPIO18 | ENA | PWM por software do motor esquerdo |
| GPIO13 | ENB | PWM por software do motor direito |

O controle de potência usa PWM por software em `ENA` e `ENB`. O período atual
está em `config::kMotorPwmPeriodMs`.

## Segurança dos motores

O comportamento esperado é:

- ao ligar, os motores começam parados;
- `Stop` zera os comandos e mantém o robô em modo parado;
- comandos de movimento só são aceitos depois de `Start`;
- se o dashboard parar de enviar comandos, os motores param por timeout;
- todos os comandos de motor são limitados entre `-1.0` e `1.0`;
- ao encerrar o programa, o código tenta zerar os pinos dos motores.

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

## Deploy pelo Linux/macOS

```sh
bash scripts/deploy.sh
```

Para apenas enviar e compilar, sem iniciar o robô:

```sh
bash scripts/deploy.sh --no-run
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

O dashboard lê a imagem processada em `/camera.jpg`. Esse endpoint mostra o
arquivo `/tmp/obr_camera_frame.jpg`, atualizado pelo script. O FPS atual da
câmera aparece ao lado do título da câmera e vem de `/camera-status.json`.

```sh
cd /home/obr/OBR2026K
python3 scripts/camera_line_frame.py
```

Se estiver em outra pasta, use o caminho completo:

```sh
python3 /home/obr/OBR2026K/scripts/camera_line_frame.py
```

Esse script usa a Pi Camera, detecta a linha preta, desenha o contorno, o ângulo
e o erro horizontal, e salva o frame para o dashboard. Se o script não estiver
rodando ou a câmera falhar, o painel continua funcionando e mostra o aviso de
câmera indisponível.

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
