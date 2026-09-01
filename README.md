# OBR2026K

> Para conhecer arquitetura, eletrônica, protocolos, operação, segurança,
> limitações e informações que ainda precisam ser preenchidas pela equipe, leia
> o [Guia completo do projeto](docs/GUIA_COMPLETO_DO_PROJETO.md).
>
> A auditoria das implementações antigas removidas está registrada em
> [Remoção de código morto e legado](docs/REMOCAO_CODIGO_MORTO.md).

Código do robô OBR 2026. Existem dois firmwares para a ESP32: um modo de bancada
com Wi-Fi e dashboard local, e o modo principal controlado pela Raspberry Pi via
UART. Os dois compartilham pinos, sensores, filtros e proteções de motor.

## Visão geral

O dashboard autônomo da ESP32 permite:

- acionar os motores esquerdo e direito em teste de bancada;
- ver distância frontal e tensão da bateria;
- acompanhar aceleração, giro, inclinação de rampa e temperatura do MPU6050;
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
  Firmware de bancada da ESP32 com Wi-Fi e dashboard local.

esp32/obr_esp32_main/
  Firmware principal da ESP32 para trabalhar com a Raspberry exclusivamente
  pela UART, sem Wi-Fi, HTTP ou dashboard local.

include/obr/config.h
  Pinos, portas, limites e constantes do robô.
```

## Teste somente com a ESP32

Abra `esp32/obr_esp32_bridge/obr_esp32_bridge.ino` no Arduino IDE. Selecione uma
placa ESP32 compatível e instale:

- `Adafruit MPU6050`;
- `Adafruit Unified Sensor`;
- `Adafruit PWM Servo Driver Library`;
- `Adafruit GFX Library`;
- `Adafruit SSD1306`.

Depois de gravar o sketch:

1. Conecte o celular ou computador à rede `OBR2026K-ESP32`.
2. Use a senha `obr2026k-painel`.
3. Abra `http://192.168.4.1`.
4. Confira os sensores com os motores sem alimentação.
5. Levante as rodas, energize os drivers e clique em `Habilitar motores`.

Os sliders e os botões permitem controlar cada lado diretamente de -100% a 100%.
Um comando de `0.05` produz 5% de PWM, `0.50` produz 50% e `1.00` produz 100%,
sem mínimo, perfil, boost ou remapeamento. O DRV8833 é habilitado uma única vez durante o `setup()` e
permanece ativo. Parar,
desabilitar, acionar o E-Stop ou atingir o timeout apenas zera os quatro PWMs,
sem repetir a inicialização do driver. Se o navegador deixar de enviar comandos
por 500 ms, a ESP32 zera os PWMs e mantém o driver habilitado. O
E-Stop permanece travado até o botão `Liberar E-Stop` ser usado; liberar não
volta a movimentar o robô.

Os botões de curva comandam os lados em sentidos opostos usando o valor solicitado.
Depois de usar `Habilitar motores`, o teclado também pode controlar o robô:
`W` avança, `S` recua, `A` gira para a esquerda e `D` gira para a direita.
Durante um giro, os dois lados se movem em sentidos opostos; A/D têm prioridade
sobre W/S quando duas teclas são pressionadas. Soltar as teclas, trocar de janela
ou ocultar a página zera imediatamente os dois comandos de motor.

Os lados são independentes. As rodas omni dianteiras permitem movimentar apenas
um lado quando a estratégia de software solicitar isso.

Além de validar o comando, a ESP32 envia a contagem dos encoders para diagnóstico.
Partidas e inversões aplicam diretamente o valor solicitado, enquanto o nSLEEP
permanece HIGH. Não existe correção automática baseada nos encoders.

> **Atenção:** a medição de uma bateria de 12 V no GPIO36 não significa que
> essa tensão possa alimentar diretamente o DRV8833. O VM do driver deve ficar
> dentro da faixa recomendada de 2,7 V a 10,8 V. A bateria de níquel pode chegar
> a 14,0 V, portanto o regulador de 8 V dos motores deve permanecer no circuito.

O botão do GPIO27 é lido e aparece na telemetria. Um toque curto, confirmado ao
soltar o botão, inicia na Raspberry a missão autônoma selecionada quando o robô
está parado. Durante controle manual ou autônomo, a borda de pressão para o robô
imediatamente e o evento da soltura é consumido para não reiniciar a missão;
com E-Stop, ele é ignorado e não libera a emergência. Ao mantê-lo
pressionado continuamente por 5 segundos, a ESP32 para os motores, zera
encoders, ângulos e filtros de navegação e recalibra o MPU6050. Durante esse
procedimento, mantenha o robô completamente imóvel. Ao terminar, os motores
continuam bloqueados até um novo comando explícito de Manual ou Auto.

## Pinagem da ESP32

As constantes ficam em `esp32/obr_esp32_bridge/robot_config.h`.

| ESP32 | Ligação | Função |
| --- | --- | --- |
| GPIO32 | Ultrassônico TRIG | Disparo frontal |
| GPIO33 | Ultrassônico ECHO | Leitura frontal, somente até 3,3 V |
| GPIO27 | Start button | Entrada com pull-up, botão para GND |
| GPIO14 / GPIO13 | I2C SCL / SDA | MPU6050, PCA9685 e SSD1306 128×64 |
| GPIO26 | DRV8833 nSLEEP | LOW desliga as pontes; HIGH libera o driver |
| GPIO5 / GPIO18 | DRV8833 esquerdo IN1 / IN2 | Dois motores do lado esquerdo |
| GPIO16 / GPIO17 | DRV8833 direito IN1 / IN2 | Dois motores do lado direito |
| GPIO22 / GPIO23 | Encoder esquerdo A / B | Lado físico validado na PCB; quadrature nas quatro bordas |
| GPIO19 / GPIO21 | Encoder direito A / B | Lado físico validado na PCB; quadrature nas quatro bordas |
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
pulso não foram informados. O MPU6050 é procurado em `0x68` e `0x69`. O OLED
SSD1306 128×64 é procurado em `0x3C` e `0x3D` e mostra tensão da bateria,
ângulo de giro e inclinação frontal da rampa. A inclinação usa um filtro
complementar que combina acelerômetro e giroscópio; o giro usa calibração no
boot, filtro passa-baixas e zona morta para reduzir variações quando parado.
A bateria ocupa a área principal do OLED, com tensão grande e uma barra visual;
giro e rampa permanecem em uma faixa compacta na parte inferior.
A barra usa 10,5 V como vazio e 14,0 V como cheio para a bateria de níquel de
12 V instalada no robô. Ela é uma referência visual e não uma estimativa exata
de capacidade restante sob todas as condições de carga.
Durante a calibração, a OLED substitui temporariamente essa tela por um indicador
animado e uma barra baseada nas amostras reais do MPU6050. Ao terminar, mostra
`PRONTO` ou `FALHOU` por aproximadamente 900 ms e volta automaticamente à tensão
da bateria e aos ângulos.

## Firmware principal com Raspberry Pi

Abra `esp32/obr_esp32_main/obr_esp32_main.ino` no Arduino IDE para gravar a
versão principal. Esse firmware não cria rede Wi-Fi e aceita movimento somente
pela UART da Raspberry, mantendo timeout e E-Stop locais na ESP32.

A comunicação usa UART em `115200` bps. Na Raspberry Pi, o código abre
`/dev/serial0`, que normalmente usa GPIO14 como TXD e GPIO15 como RXD.

As potências são definidas na Raspberry e o protocolo UART apenas transmite os
valores esquerdo e direito. No modo manual de diagnóstico, o duty escolhido é
enviado diretamente. No modo autônomo, o `MotorController` aplica o perfil
START/RUN descrito na seção de segurança antes de enviar a mensagem.

| Raspberry Pi | ESP32 | Função |
| --- | --- | --- |
| GPIO14 / TXD | RX / GPIO3 | comandos para a ESP32 |
| GPIO15 / RXD | TX / GPIO1 | telemetria da ESP32 |
| GPIO26 | — | LED ativo em HIGH que indica sistema pronto e telemetria recente |
| GND | GND | referência elétrica comum |

O LED da Raspberry inicia apagado. Ele só acende quando `/dev/serial0` está
aberta, a Raspberry recebe telemetria recente, o `nSLEEP` está em HIGH, nenhum
E-Stop ou calibração está ativo e a câmera já publicou estado recente com FPS
maior que zero. Se uma dessas condições deixar de ser atendida, o LED apaga
automaticamente. O navegador ainda pode levar uma pequena fração de segundo para
abrir e decodificar o stream MJPEG, mas, quando o LED acende, o processo da câmera
na Raspberry já está produzindo quadros.

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
- o GPIO26 mantém o DRV8833 em sleep somente enquanto os PWMs são configurados no boot;
- depois do `setup()`, o GPIO26 permanece em HIGH e todas as paradas zeram os quatro PWMs;
- no firmware de bancada, comandos do dashboard só são aceitos depois de `Habilitar motores`;
- no firmware principal, somente comandos recebidos da Raspberry pela UART controlam movimento;
- se o dashboard de bancada ou a Raspberry parar de enviar comandos, os motores param por timeout;
- se a Raspberry ou a UART pararem de enviar comandos, a ESP32 também para os motores;
- os comandos locais são limitados entre `-1.00` e `1.00`;
- a ESP32 converte diretamente o comando UART para PWM, sem remapeamento próprio;
- no modo autônomo, cada roda parada recebe no mínimo `0.67`; ela só pode usar
  `0.61` depois de duas amostras recentes do encoder confirmarem movimento no
  mesmo sentido, com taxa de pelo menos `20 cont/s`;
- três amostras recentes inválidas fazem a roda voltar ao estado de partida;
- o controle comum do modo Manual usa `drive`, com o perfil START/RUN e
  sincronismo em avanço reto; somente o ajuste independente usa `drive_raw`
  para testar diretamente valores entre `-1.00` e `1.00` por lado;
- o sincronismo por encoder atua somente em avanço reto, com os dois lados no
  mesmo sentido e praticamente com o mesmo comando; curvas e giros preservam o
  diferencial pedido pela visão;
- o teste autônomo de giro de 90° usa comando lógico `0.01` pelo perfil operacional;
- os lados esquerdo e direito podem ser controlados independentemente;
- o E-Stop tem prioridade sobre dashboard e UART;
- ao encerrar o programa, o código envia `STOP` para a ESP32.

Durante testes, levante as rodas antes de usar valores altos nos sliders.

## Visualizar o dashboard localmente no Windows

Para abrir a mesma interface da Raspberry Pi no computador, sem conectar o robô,
execute na pasta do projeto:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/run-local-dashboard.ps1
```

Depois abra:

```txt
http://127.0.0.1:8080
```

O script configura e compila o projeto em uma pasta de build local e mantém o
servidor no terminal atual. Quando o Ninja está instalado, ele é selecionado
automaticamente. Pressione `Ctrl+C` para encerrar. No Windows, GPIO, UART e motores
ficam inativos; por isso, a ESP32, os sensores e as câmeras aparecem desconectados,
mas o layout e a navegação do dashboard podem ser avaliados.

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

No Windows, o próprio primeiro deploy detecta a ausência de acesso e executa a
preparação automaticamente:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/deploy.ps1 -HostName obr.local
```

A preparação cria uma chave SSH exclusiva para deploy, instala a chave pública e
autoriza sem senha somente os comandos necessários para parar, reiniciar e
consultar os serviços `obr-robot` e `obr-line-camera`. Na primeira execução, ela
pede a senha SSH da Raspberry uma vez e a senha de `sudo` uma vez. Nenhuma senha
ou chave privada é salva no repositório. Os deploys seguintes não fazem perguntas
interativas.

Se quiser executar somente a preparação, use `scripts/install-service.ps1`.

O deploy copia o código para `/home/raspberry/OBR2026K`, para o serviço e compila em
`.build-staging`. O executável em uso só é substituído depois que o novo build
termina e passa pela validação de tamanho e permissão. A troca é atômica: se a
compilação falhar ou for interrompida, um binário parcial ou vazio nunca é
instalado e o robô permanece parado. Depois da troca, o deploy reinicia o serviço
`obr-robot` e falha claramente se ele não ficar ativo.

Se precisar escolher o host manualmente:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/deploy.ps1 -HostName obr.local
```

Para apenas enviar e compilar, sem iniciar o robô:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/deploy.ps1 -NoRun
```

## Deploy pelo Linux/macOS

Também é necessário preparar cada computador uma vez:

```sh
bash scripts/install-service.sh --host obr.local
```

Depois da preparação, o deploy é não interativo:

```sh
bash scripts/deploy.sh
```

Para apenas enviar e compilar, sem iniciar o robô:

```sh
bash scripts/deploy.sh --no-run
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

Os scripts de preparação mostrados acima também instalam e habilitam os serviços.
Eles podem ser executados antes do primeiro deploy: se o binário ainda não existir,
os serviços ficam habilitados e serão iniciados pelo primeiro deploy concluído.
Execute o mesmo comando novamente quando um arquivo `.service` for alterado.

No Windows:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/install-service.ps1
```

No Linux/macOS:

```sh
bash scripts/install-service.sh
```

Depois disso, a Raspberry inicia dois serviços automaticamente no boot:
`obr-robot`, que mantém dashboard, ESP32 e controle principal, e
`obr-line-camera`, que captura e processa exclusivamente a câmera inferior.
O deploy normal reinicia ambos com a versão nova.

Antes de iniciar controle, `run_robot.sh` confirma que `build/robot_test` existe,
não está vazio e possui permissão de execução. A câmera inferior não é iniciada por
esse script: ela pertence somente a `obr-line-camera`, evitando duas instâncias do
processamento de visão. Em caso de falha, o motivo aparece no `journalctl` e o
systemd limita reinicializações rápidas para não permanecer em um ciclo infinito.

Comandos úteis na Raspberry:

```sh
sudo systemctl status obr-robot
sudo systemctl restart obr-robot
journalctl -u obr-robot -f

# Controle dedicado da visão inferior; parar este serviço não para o dashboard.
sudo systemctl status obr-line-camera
sudo systemctl stop obr-line-camera
sudo systemctl start obr-line-camera
sudo systemctl restart obr-line-camera
journalctl -u obr-line-camera -f
```

Ao parar `obr-line-camera`, a Missão Principal não aceita o JSON anterior: o
supervisor remove o IPC de linha antes de iniciar e ao parar, e a missão só se
arma após receber uma publicação nova e fresca. Dashboard, ESP32 e os demais
processos continuam ativos, mas o segue-faixa permanece parado por segurança.

## Dashboard

Depois do serviço subir, abra:

```txt
http://obr.local:8080
```

No modo Manual, o dashboard aceita `W`, `A`, `S` e `D`. `W/S` comandam frente e
ré; `A/D` giram os dois lados em sentidos opostos e têm prioridade sobre `W/S`.
Assim, uma combinação como `W+A` executa o giro completo, sem zerar um lado.
Os limites separados de reta e
curva começam em `0.05`, podem ser ajustados até `1.0` e ficam salvos no navegador.
Eles usam o caminho operacional (`drive`); ao avançar em linha reta, o
`MotorController` sincroniza os lados pelos encoders. Soltar a tecla, trocar de
janela ou ocultar a página zera os comandos. O teclado não movimenta o robô nos
modos Parado, Autônomo ou E-Stop.

Os campos de ajuste exato permitem comandar esquerda e direita separadamente em
passos de `0.01`. O painel de sincronização compara o módulo das taxas dos dois
encoders e mostra a escala aprendida e o PWM corrigido. No controle normal, o
`MotorController` mede a eficiência em `cont/s por PWM`, filtra três amostras e
reduz somente o lado mais rápido em passos máximos de `0.03` por nova telemetria.
O ajuste individual desativa essa malha e também o perfil START/RUN para
diagnóstico. Use-o apenas com rodas suspensas ao testar valores baixos.
A ESP32 recalcula as taxas dos encoders no mesmo período de `100 ms` da UART para
que cada atualização da escala use uma janela de velocidade realmente nova.

Os sliders e o WASD usam o perfil START/RUN: zero permanece parada; uma roda
parada parte em `0.67`; após confirmação individual do encoder ela pode manter
`0.61`. Giros em sentidos opostos não recebem sincronização. O ajuste
independente de Manutenção continua direto. A missão isolada de giro de 90° usa
comando lógico `0.01`, inicialmente convertido em aproximadamente
`0.67 / -0.67`.

### Segue-faixa autônomo

O controlador normal foi removido intencionalmente. A função
`calculate_line_follower_command` recebe a máscara binária de linha já processada
e o resultado da detecção verde, mas publica potência zero até uma nova estratégia
ser implementada. A Missão Principal aplica essa parada somente no estado normal;
as manobras já confirmadas pela máquina de estados verde mantêm prioridade.

O dashboard identifica o segue-faixa como `CONTROLE PENDENTE` e continua mostrando
a saúde da câmera, FPS, resolução, sequência da visão, reparo especular e resultado
verde, além da telemetria geral do robô.

### Desvio autônomo de obstáculo

A Missão Principal confirma um obstáculo frontal com duas leituras consecutivas
de até `8 cm`. A lógica fica isolada na pasta `obstacle_avoidance/` e executa o
percurso medido: direita `45°`, frente `25 cm`, esquerda `45°`, frente `30 cm`,
esquerda `90°`, frente `21,5 cm`, direita `90°` e ré de `5 cm`. Os ângulos usam o
MPU6050 e os deslocamentos usam os dois encoders.

Cada transição interrompe os motores antes de capturar uma nova referência. Perda de
telemetria, encoder ou IMU encerra a missão com potência zero. Todos os valores
de calibração ficam em `obstacle_avoidance/config.h`.

Depois que o ultrassônico confirma o obstáculo, o módulo mantém autoridade até
terminar a ré final. Leituras de linha, decisões verdes e indisponibilidade da
câmera não trocam nem cancelam suas etapas. E-Stop, comando Parar, perda da ESP32,
IMU ou encoders e os timeouts de segurança continuam interrompendo o movimento.

O seletor `Missão autônoma` inicia sempre em `MISSÃO PRINCIPAL` quando o programa
é aberto. A missão escolhida pode ser iniciada pelo botão `Autônomo` do painel ou
por um toque curto no Start físico. Trocar a seleção força o robô para o modo
Parado antes de armar a nova estratégia.

O modo de teste `GIRO 90° À DIREITA` usa yaw e velocidade angular do MPU6050.
Ele mantém comando lógico `0.01`, convertido pelo perfil operacional em potência
suficiente para partir, prevê a inércia antes do alvo e zera o PWM para
estabilizar e pode aplicar até três correções curtas: continua no mesmo sentido
se faltar ângulo ou reverte se ultrapassar. A tolerância real é de ±2° e o
timeout total é de 5 segundos. Se a amostra do MPU6050 ficar inválida ou tiver
mais de 200 ms, a missão é encerrada com os motores zerados. Faça o primeiro teste com as rodas
suspensas e ajuste a projeção de inércia em `include/obr/config.h` se necessário.

O modo `PERCORRER DISTÂNCIA` aceita um alvo de 1 a 300 cm no dashboard. A
calibração empírica atual é `3600 contagens = 18,7 cm`, ou aproximadamente
`192,51 contagens/cm`, medida com rodas de 68 mm. Ao iniciar, a Raspberry guarda
as contagens atuais como referência, move os dois lados para a frente e considera
o menor avanço entre eles; assim, um lado sozinho não conclui o percurso. A
frenagem usa a taxa dos encoders para antecipar a inércia, seguida de estabilização
e até três correções curtas. Telemetria ausente, falta de avanço ou timeout sempre
zeram os motores.

No cartão `Encoders`, `cont/s` é a taxa instantânea e naturalmente volta a zero
quando as rodas param. As contagens e posições em centímetros são acumuladas desde
o último reset ou calibração e não são zeradas pelo botão `Parar`.

O botão `Resetar e calibrar sensores` para o robô, zera encoders e referências
de orientação e recalibra o giroscópio do MPU6050. O mesmo procedimento pode ser
iniciado sem o dashboard ao manter o botão Start da ESP32 pressionado por 5
segundos. Em ambos os casos, deixe o robô imóvel durante a calibração. O painel
mostra quando ela está em andamento e se terminou com sucesso; o movimento só é
liberado novamente por uma ação explícita em Manual ou Auto.

### Imagem da câmera

O dashboard mostra a imagem direta da câmera pelo stream MJPEG na própria porta do
dashboard:

```txt
http://obr.local:8080/camera-stream.mjpg
```

O script Python mantém um servidor local em `127.0.0.1:8090`, e o C++ faz proxy
para `/camera-stream.mjpg`. O endpoint antigo `/camera.jpg` continua disponível
como snapshot de compatibilidade, lendo `/tmp/obr_camera_frame.jpg`. O FPS, a
resolução, o formato e a disponibilidade da captura vêm de
`/camera-status.json`.

A câmera frontal usa a CAM1 e fica fisicamente fechada por padrão. No dashboard,
abra a visualização `Frontal` ou `Dupla` e use o botão `ATIVAR`. Quando ligada, o
gerenciador `forward_camera_stream.py` entrega imagem anotada pelo detector de
bolas em `960x540`, usando o modo físico `1920x1080` de 10 bits, pela rota:

```txt
http://obr.local:8080/forward-camera-stream.mjpg
```

Ao usar `DESATIVAR`, o Picamera2 é encerrado e a CAM1 é liberada; permanece apenas
um gerenciador ocioso que observa o pequeno arquivo de controle. A câmera frontal
não executa segmentação de linha ou verde e não publica no IPC do segue-faixa.
Assim, ativá-la não muda o controle dos motores nem a interpretação da câmera
inferior.

O código também pode solicitar que a frontal já seja aberta no início do serviço:

```sh
OBR_FORWARD_CAMERA_ENABLED=1 bash scripts/run_robot.sh
```

Sem essa variável, ou com valor `0`, cada início do serviço volta ao estado seguro
desligado. O dashboard envia o mesmo controle pelo WebSocket com
`{"command":"set_forward_camera","enabled":true}` ou `false`.

### Detector de bolas na câmera frontal

O detector leve de bolas pretas e pratas fica em `scripts/ball_vision/` e usa somente a
CAM1. Para vê-lo no dashboard, selecione `Frontal` e pressione `ATIVAR`. O mesmo
processo que possui a câmera desenha o círculo no stream e publica no painel
tipo, centro, raio, diâmetro, distância, ângulo, direção e geometria. A bola
preta usa segmentação HSV e contornos; a prata usa Hough com validação da
textura metálica para rejeitar o piso claro.

O comando abaixo é uma alternativa de diagnóstico local. Antes de executá-lo,
desative a câmera frontal no dashboard para liberar o Picamera2; o serviço da
câmera inferior pode continuar ativo normalmente.

```sh
python3 scripts/ball_vision/main.py
```

Ele mostra centro, raio, diâmetro, distância calibrada, ângulo e direção. O guia
de calibração, os ajustes HSV e a saída JSON preparada para a ESP32 estão em
`scripts/ball_vision/README.md`.

### Missão de debug: alinhamento com bola

A opção `ALINHAR COM BOLA MAIS PRÓXIMA` é uma missão autônoma isolada e não
participa da Missão Principal. Ative a câmera frontal, selecione essa missão e
só então use `Autônomo`. O maior raio visual válido representa a bola mais
próxima.

O controle gira pelo `tx`: negativo para a esquerda e positivo para a direita.
O robô considera alinhado somente dentro de `±2°`. Fora dessa zona, o pivot
parte com potência `0,70`. Assim que os dois encoders confirmam movimento no
sentido comandado, entra no controle proporcional: usa `0,68` com erro igual ou
maior que `12°` e reduz linearmente até `0,61` ao se aproximar de `2°`. A posição
usada pelo controle é publicada em RAM a cada frame. Se o `tx` cruzar o centro
entre dois frames, o PWM é zerado por `160 ms` antes de permitir uma correção
oposta. Perda da bola, câmera desligada ou IPC com mais de `500 ms` zera os dois
lados imediatamente.

Para limitar aquecimento, o stream frontal integrado captura a `15 FPS`, limita
o OpenCV a duas threads e executa a etapa Hough da bola prata em meia resolução,
convertendo centro e raio de volta para `960×540`. O desenho e a compressão JPEG
só são executados enquanto existe um cliente acompanhando o dashboard, com o
vídeo de depuração limitado a `12 FPS`. O `tx` de controle continua sendo
publicado em cada frame analisado. As câmeras continuam independentes: ativar
uma delas manualmente não altera o estado solicitado da outra. A política da
missão é explícita: `ALINHAR COM BOLA` liga a frontal, desliga a inferior e
habilita HSV/Hough; fora desse modo, a frontal pode transmitir vídeo cru, mas
não executa nem publica detecção de bolas, e o loop C++ não consulta o IPC de
bolas. Ao selecionar `MISSÃO PRINCIPAL`, a detecção de bolas é desligada e a
câmera inferior é solicitada novamente.

```sh
cd /home/raspberry/OBR2026K
python3 scripts/camera_line_frame.py
```

O papel da câmera conectada pode ser escolhido por argumento:

```sh
python3 scripts/camera_line_frame.py --camera-role forward
python3 scripts/camera_line_frame.py --camera-role down
```

Ou pela variável de ambiente usada pelo serviço:

```sh
OBR_CAMERA_ROLE=down python3 scripts/camera_line_frame.py
```

O perfil `forward` preserva a saída `960x540` e seleciona explicitamente o modo
físico `1920x1080`. O gerenciador frontal reutiliza somente essa configuração de
captura, sem chamar a visão de linha. O perfil `down` usa saída `480x360 @ 30 FPS`
e seleciona explicitamente o modo `1640x1232` de 10 bits, reportado pelo driver
  como full-FOV. A geometria e os kernels preservados da máscara e do detector
  verde são derivados da resolução atual; os valores de referência de `640x480`
não ficam aplicados como pixels fixos. Sem argumento nem variável de ambiente, o
perfil `forward` continua sendo usado por compatibilidade.

Para executar a visão inferior manualmente fora do systemd, use o comando abaixo.
No robô em operação, use `ATIVAR` e `DESATIVAR` no cartão da câmera inferior;
`obr-robot` não inicia outra cópia desse processo.

Se estiver em outra pasta, use o caminho completo:

```sh
python3 /home/raspberry/OBR2026K/scripts/camera_line_frame.py
```

O script publica o comando seguro do ponto de extensão e os resultados necessários
à máquina de estados verde. A configuração padrão `forward` usa `960x540`, JPEG `82` e stream alvo
de `30 FPS`. Como a câmera está montada de cabeça para baixo, o Picamera2 aplica
rotação de 180°. Se o script não estiver rodando ou a câmera falhar, o painel
continua disponível e mostra a câmera como indisponível.

Depois de atualizar os arquivos de serviço, reinstale uma vez pelo computador de
desenvolvimento:

```sh
bash scripts/install-service.sh
```

Se você já estiver no terminal da Raspberry, dentro de `/home/raspberry/OBR2026K`, use:

```sh
sudo cp scripts/obr-robot.service scripts/obr-line-camera.service /etc/systemd/system/
chmod +x scripts/run_robot.sh scripts/run_line_camera.sh
sudo systemctl daemon-reload
sudo systemctl enable --now obr-robot obr-line-camera
```

O cartão da câmera inferior do dashboard mostra os estados `ONLINE`, `PARADA`,
`INICIANDO` e `FALHA`, com o mesmo botão de alternância da câmera frontal:
`ATIVAR` ou `DESATIVAR`. Pelo WebSocket, o dashboard envia apenas
`{"command":"set_line_camera","enabled":true}` ou `false`.

Esse comando grava um único IPC em `/dev/shm`; ele não executa `sudo`,
`systemctl` nem comandos Linux enviados pelo navegador. O gerenciador
`obr-line-camera` permanece ocioso quando desativado, encerra o Python, remove o
IPC visual e publica `PARADA`. Ao ativar, remove qualquer IPC antigo, inicia a
câmera e só informa `ONLINE` após uma publicação atual da visão.

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
