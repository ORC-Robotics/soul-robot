# OBR2026K

## Deploy na Raspberry Pi

Este projeto pode ser enviado pela rede sem usar a extensao Remote SSH do VS Code.
O deploy usa os comandos locais `ssh` e `scp`: ele copia o codigo para a Raspberry,
compila com CMake na propria Raspberry e deixa o binario em `/home/obr/OBR2026K/build/robot_test`.

Na Raspberry, instale as dependencias uma vez:

```sh
sudo apt update
sudo apt install -y build-essential cmake
```

Depois de rodar o deploy, abra o dashboard no navegador:

```txt
http://raspberrypi.local:8080
```

O dashboard usa WebSocket para receber telemetria e enviar comandos basicos de
controle remoto. Por seguranca, o codigo zera os comandos dos motores se ficar
mais de 2 segundos sem receber comando do dashboard.

## Pinos do robo

Os pinos ficam centralizados em `include/obr/config.h`. A configuracao inicial para a
ponte H L298N usa numeracao BCM:

- motor esquerdo: `ENA=12`, `IN1=5`, `IN2=6`
- motor direito: `ENB=13`, `IN3=20`, `IN4=21`

Esta primeira versao usa controle digital para testar os motores: valores
positivos giram para frente, negativos giram para tras, e valores perto de zero
param o motor. Depois podemos trocar os pinos `ENA/ENB` para PWM real.

O servico roda como usuario `obr` e tenta usar o grupo `gpio`. Se os pinos nao
responderem, confira na Raspberry:

```sh
groups obr
sudo usermod -aG gpio obr
```

Depois de alterar grupos, reinicie a Raspberry.

No Windows, rode pelo terminal na pasta do projeto:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/deploy.ps1
```

O deploy reinicia o servico `obr-robot` quando ele ja esta instalado. Se o
servico ainda nao existir, ele roda o programa no terminal via SSH. Para apenas
compilar/enviar sem rodar:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/deploy.ps1 -NoRun
```

No Linux/macOS, o processo e o mesmo, usando Bash:

```sh
bash scripts/deploy.sh
```

Para apenas compilar/enviar sem rodar no Linux/macOS:

```sh
bash scripts/deploy.sh --no-run
```

## Iniciar automaticamente no boot

Depois que o deploy ja tiver compilado o projeto pelo menos uma vez, instale o
servico `systemd` na Raspberry:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/install-service.ps1
```

No Linux/macOS:

```sh
bash scripts/install-service.sh
```

Depois disso, a Raspberry inicia `/home/obr/OBR2026K/build/robot_test`
automaticamente no boot. Para atualizar o codigo e reiniciar o servico:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/deploy.ps1
```

No Linux/macOS:

```sh
bash scripts/deploy.sh
```

Se o usuario, host ou pasta forem diferentes:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/deploy.ps1 -User obr -HostName raspberrypi.local -RemoteDir /home/obr/OBR2026K
```

No VS Code, tambem da para usar:

- `Terminal > Run Build Task` para `Deploy Raspberry`
- `Terminal > Run Task > Deploy only Raspberry`
