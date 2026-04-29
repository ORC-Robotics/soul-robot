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

Para parar de digitar a senha em todo deploy, configure uma chave SSH uma vez:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/setup-ssh-key.ps1
```

Digite a senha da Raspberry quando pedir. Depois disso, o deploy deve entrar sem
pedir senha.

Depois de rodar o deploy, abra o dashboard no navegador:

```txt
http://192.168.0.4:8080
```

O dashboard usa WebSocket para receber telemetria e enviar comandos basicos de
controle remoto. Por seguranca, o codigo zera os comandos dos motores se ficar
mais de 2 segundos sem receber comando do dashboard.

No Windows, rode pelo terminal na pasta do projeto:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/deploy.ps1
```

O deploy ja executa o robo por padrao. Para apenas compilar/enviar sem rodar:

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

Se o usuario, host ou pasta forem diferentes:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/deploy.ps1 -User obr -HostName 192.168.0.4 -RemoteDir /home/obr/OBR2026K
```

No VS Code, tambem da para usar:

- `Terminal > Run Build Task` para `Deploy + Run Raspberry`
- `Terminal > Run Task > Deploy only Raspberry`
