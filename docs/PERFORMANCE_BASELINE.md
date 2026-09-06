# Baseline de performance do dashboard

Esta medição existe para detectar regressões antes que alterações no dashboard
prejudiquem a missão, a visão ou o controle do robô.

## Arquitetura medida

O serviço `obr-robot` executa `scripts/run_robot.sh` e mantém dois processos:

- `build/robot_test`: missão, controle, UART e servidor HTTP/WebSocket do dashboard;
- `scripts/forward_camera_stream.py`: captura frontal e assistente leve de linha;
  a visão pesada de vítimas permanece normalmente ociosa.

O serviço independente `obr-line-camera` executa `scripts/run_line_camera.sh`, que
mantém uma única instância de `scripts/camera_line_frame.py` para captura inferior,
visão, IPC e MJPEG. Isso permite desligar a carga de visão quando o segue-faixa não
está em teste, sem desligar dashboard, ESP32 ou o programa principal.

O dashboard não é um processo separado. O `DashboardServer` usa uma thread de
aceitação, uma thread de telemetria e uma thread curta por conexão HTTP/WebSocket
dentro de `robot_test`. Por isso, a coluna `robotCpuPercent` representa o custo
combinado da missão e do dashboard. `telemetryServerWorkMs` ajuda a isolar o
tempo gasto no ciclo periódico do servidor sem instrumentar o loop de controle.

## Frequências atuais

- loop principal da missão: 20 ms, alvo de 50 Hz;
- telemetria WebSocket: 500 ms, alvo de 2 Hz;
- heartbeat de comando manual no navegador: 100 ms, 10 Hz, somente em modo Manual;
- status inferior e frontal consultados pelo navegador: 500 ms, 2 Hz;
- status publicado pelos processos de câmera: 5 Hz;
- captura inferior: 480×360, alvo de 30 FPS, com FPS real no status da câmera;
- publicação MJPEG inferior: limite de 30 FPS;
- snapshot JPEG em `/tmp`: 2 Hz;
- IPC rápido da linha em `/dev/shm`: uma publicação por frame processado;
- captura frontal: 960×540, alvo de 30 FPS para o assistente de linha;
- HSV/Hough de vítimas: desligado fora de `rescue_area` e limitado a 15 análises
  por segundo quando essa missão está em execução;
- IPC rápido da vítima em `/dev/shm`: publicado somente nos frames analisados;
- reconexão do WebSocket e do MJPEG: tentativa após 1 segundo;
- `requestAnimationFrame`: uma chamada isolada ao trocar a origem do MJPEG, sem loop.

## Como coletar na Raspberry Pi

Com o serviço em execução, faça primeiro uma medição sem navegador aberto:

```bash
cd /home/raspberry/OBR2026K
python3 scripts/performance_baseline.py \
  --duration 60 \
  --output /tmp/obr-baseline-sem-dashboard.csv
```

Depois abra o dashboard e mantenha a câmera visível durante uma segunda medição:

```bash
python3 scripts/performance_baseline.py \
  --duration 60 \
  --output /tmp/obr-baseline-com-dashboard.csv
```

O coletor cria o CSV e um arquivo `.summary.json` com média, p95 e máximo. Ele
faz uma amostra por segundo, não abre o stream e não decodifica frames. Na
segunda medição, o próprio navegador é quem mantém o MJPEG aberto.

O valor `telemetryAgeMs` compara o relógio da Raspberry com o horário de geração
presente no pacote. Quando a coleta for executada na própria Raspberry, essa
idade não depende de sincronização entre computadores.

## Colunas principais

- `systemCpuPercent`, `load1`, `load5`, `load15`;
- `ramUsedMb`, `ramUsedPercent`, `temperatureCelsius`;
- `throttledHex`, obtido por `vcgencmd get_throttled` a cada 5 segundos;
- CPU e RSS de `robot_test`, visão inferior e gerenciador frontal;
- `cameraFps` e `cameraStatusAgeMs`, reutilizados do status existente;
- `telemetryHz`, idade atual, idade no recebimento e mensagens perdidas;
- tamanho do pacote WebSocket recebido;
- intervalo efetivo e tempo de trabalho do ciclo de telemetria no servidor.

As porcentagens de processo seguem a convenção do `top`: 100% corresponde a um
núcleo completamente ocupado. A CPU do sistema inteiro permanece entre 0% e
100%, independentemente da quantidade de núcleos.

Em `throttledHex`, `0x0` significa que nenhum evento de throttling foi registrado.
Outros bits devem ser interpretados conforme a documentação do firmware da
Raspberry Pi. O coletor mantém o valor bruto para não esconder eventos antigos.

## Comparação recomendada

Use a mesma duração, pista, modo do robô, câmera e quantidade de navegadores em
todas as medições. Compare principalmente p95, não apenas a média. Uma alteração
de dashboard deve ser investigada se aumentar de forma repetível:

- CPU ou RSS de `robot_test`;
- CPU ou RSS do processo de visão;
- `telemetryServerWorkMs` ou `telemetryAgeMs`;
- load average, temperatura ou throttling;
- idade do status ou redução do FPS real da câmera.

Os arquivos de baseline são diagnósticos. Eles não devem ser coletados durante
uma prova oficial sem necessidade.

Depois de aprovar a primeira medição na Raspberry, use-a como referência. Como
alertas iniciais, investigue mensagens perdidas diferentes de zero, throttling
diferente de `0x0`, FPS abaixo de 90% da baseline, intervalo de telemetria p95
acima de 600 ms ou aumento repetível maior que 10% no p95 de CPU/RSS. Esses
limites são guardrails de comparação, não substituem o teste do robô na pista.
