# OBR2026K

> Para conhecer arquitetura, eletrônica, protocolos, operação, segurança,
> limitações e informações que ainda precisam ser preenchidas pela equipe, leia
> o [Guia completo do projeto](docs/GUIA_COMPLETO_DO_PROJETO.md).

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

As potências são definidas diretamente pelo controle manual ou autônomo na
Raspberry. O protocolo UART apenas transmite os valores esquerdo e direito.

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
- a Raspberry aplica no controle normal o mínimo operacional de `0.65` e usa os
  encoders para reduzir gradualmente somente o lado mecanicamente mais rápido;
- os campos exatos do dashboard usam um caminho de diagnóstico direto, sem esse perfil;
- o teste autônomo de giro de 90° usa comando lógico `0.01` pelo perfil operacional;
- os lados esquerdo e direito podem ser controlados independentemente;
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

O deploy copia o código para `/home/raspberry/OBR2026K`, para o serviço e compila em
`.build-staging`. O executável em uso só é substituído depois que o novo build
termina e passa pela validação de tamanho e permissão. A troca é atômica: se a
compilação falhar ou for interrompida, um binário parcial ou vazio nunca é
instalado e o robô permanece parado. Depois da troca, o deploy reinicia o serviço
`obr-robot` e falha claramente se ele não ficar ativo.

Se precisar escolher o host manualmente:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/deploy.ps1 -HostName 192.168.0.102
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
Get-Content "$env:USERPROFILE\.ssh\obr_raspberry.pub" | ssh raspberry@192.168.0.102 "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys"
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
ssh-copy-id -i ~/.ssh/obr_raspberry.pub raspberry@192.168.0.102
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

Antes de iniciar câmera e controle, `run_robot.sh` também confirma que
`build/robot_test` existe, não está vazio e possui permissão de execução. Em caso
de falha, o motivo aparece no `journalctl` e o systemd limita reinicializações
rápidas para não permanecer em um ciclo infinito.

Comandos úteis na Raspberry:

```sh
sudo systemctl status obr-robot
sudo systemctl restart obr-robot
journalctl -u obr-robot -f
```

## Dashboard

Depois do serviço subir, abra:

```txt
http://192.168.0.102:8080
```

Se o nome não resolver na rede, use o IP atual da Raspberry:

```txt
http://192.168.0.104:8080
```

No modo Manual, o dashboard aceita `W`, `A`, `S` e `D`. `W/S` comandam frente e
ré; `A/D` giram os dois lados em sentidos opostos e têm prioridade sobre `W/S`.
Assim, uma combinação como `W+A` executa o giro completo, sem zerar um lado.
Os limites separados de reta e
curva começam em `0.65`, podem ser ajustados até `1.0` e ficam salvos no navegador. Soltar a tecla, trocar
de janela ou ocultar a página zera os comandos. O teclado não movimenta o robô
nos modos Parado, Autônomo ou E-Stop.

Os campos de ajuste exato permitem comandar esquerda e direita separadamente em
passos de `0.01`. O painel de sincronização compara o módulo das taxas dos dois
encoders e mostra a escala aprendida e o PWM corrigido. No controle normal, o
`MotorController` mede a eficiência em `cont/s por PWM`, filtra três amostras e
reduz somente o lado mais rápido em passos máximos de `0.03` por nova telemetria.
O ajuste individual desativa essa malha para diagnóstico, mas todo valor não nulo
continua respeitando o piso operacional de `0,65`.
A ESP32 recalcula as taxas dos encoders no mesmo período de `100 ms` da UART para
que cada atualização da escala use uma janela de velocidade realmente nova.

Os sliders e o WASD usam o perfil operacional: zero permanece parada e qualquer
movimento parte do piso configurado para os motores. A correção aprendida é
reutilizada entre paradas e atualizada conforme bateria, atrito e carga mudam.
Giros em sentidos opostos não recebem sincronização. A missão isolada de giro de
90° usa comando lógico `0.01`, resultando em aproximadamente `0.65 / -0.65`.

A Missão Principal está intencionalmente vazia nesta etapa. Ela funciona como o
ponto de composição dos futuros comportamentos autônomos e mantém os dois motores
zerados enquanto nenhum comportamento estiver instalado. A câmera publica somente
a imagem ao vivo e dados básicos de saúde.

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
http://192.168.0.102:8080/camera-stream.mjpg
```

O script Python mantém um servidor local em `127.0.0.1:8090`, e o C++ faz proxy
para `/camera-stream.mjpg`. O endpoint antigo `/camera.jpg` continua disponível
como snapshot de compatibilidade, lendo `/tmp/obr_camera_frame.jpg`. O FPS, a
resolução, o formato e a disponibilidade da captura vêm de
`/camera-status.json`.

A câmera frontal usa a CAM1 e fica fisicamente fechada por padrão. No dashboard,
abra a visualização `Frontal` ou `Dupla` e use o botão `ATIVAR`. Quando ligada, o
gerenciador `forward_camera_stream.py` entrega imagem bruta em `960x540`, usando o
modo físico `1920x1080` de 10 bits, pela rota:

```txt
http://192.168.0.102:8080/forward-camera-stream.mjpg
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
captura, sem chamar a visão de linha. O perfil `down` usa saída `640x480` e seleciona
explicitamente o modo `1640x1232` de 10 bits, reportado pelo driver como
full-FOV. Sem argumento nem variável de ambiente, o perfil `forward` continua
sendo usado por compatibilidade.

Para selecionar um papel de forma persistente no serviço, crie um override do
systemd com `Environment=OBR_CAMERA_ROLE=down` ou
`Environment=OBR_CAMERA_ROLE=forward` e reinicie `obr-robot`.

Se estiver em outra pasta, use o caminho completo:

```sh
python3 /home/raspberry/OBR2026K/scripts/camera_line_frame.py
```

O script publica a máscara e os diagnósticos da linha usados pelo restante do
projeto. A configuração padrão `forward` usa `960x540`, JPEG `82` e stream alvo
de `30 FPS`. Como a câmera está montada de cabeça para baixo, o Picamera2 aplica
rotação de 180°. Se o script não estiver rodando ou a câmera falhar, o painel
continua disponível e mostra a câmera como indisponível.

Quando o serviço `obr-robot` estiver instalado com a versão atual dos scripts,
ele inicia esse script automaticamente junto com o robô. Depois de atualizar o
arquivo de serviço, reinstale uma vez pelo computador de desenvolvimento:

```sh
bash scripts/install-service.sh
```

Se você já estiver no terminal da Raspberry, dentro de `/home/raspberry/OBR2026K`, use:

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
