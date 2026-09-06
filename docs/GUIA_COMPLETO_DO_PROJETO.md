# Guia completo do projeto OBR2026K

> Documento vivo para apresentar, operar, testar e continuar o desenvolvimento
> do robô. Ele foi escrito a partir do código e das decisões registradas no
> repositório em 6 de agosto de 2026.

## Como usar este documento

Este guia separa informação confirmada pelo código de informação que ainda
precisa ser fornecida pela equipe:

- **Confirmado:** comportamento implementado ou configuração presente no repositório.
- **Em validação:** implementado, mas ainda depende de teste físico representativo.
- **PREENCHER:** informação mecânica, elétrica ou organizacional que não pode ser
  deduzida com segurança pelo software.

Quando uma informação mudar, atualize primeiro a fonte de verdade indicada e
depois revise este documento. Não copie números para vários arquivos sem
necessidade.

## Navegação rápida

- [Identificação e visão geral](#1-ficha-de-identificação)
- [Arquitetura](#3-arquitetura)
- [Hardware e alimentação](#4-hardware-conhecido)
- [Pinagem da ESP32](#5-pinagem-da-esp32)
- [Organização do software](#8-organização-do-repositório)
- [Motores e segurança](#10-controle-dos-motores)
- [Sensores e OLED](#12-sensores-e-periféricos-da-esp32)
- [Protocolo UART](#14-protocolo-uart-raspberry--esp32)
- [Câmera e estrutura autônoma](#16-câmera-ao-vivo)
- [Dashboard e LED de pronto](#18-dashboard-da-raspberry)
- [Build, deploy e serviço](#20-dependências-e-build)
- [Operação e testes](#22-procedimento-recomendado-de-operação)
- [Diagnóstico](#24-diagnóstico-rápido)
- [Limitações e roadmap](#25-limitações-e-dívidas-técnicas-conhecidas)
- [Questionário para completar](#27-questionário-para-a-equipe-preencher)
- [Fontes de verdade](#29-fontes-de-verdade)

## 1. Ficha de identificação

| Campo | Informação atual |
| --- | --- |
| Nome do projeto | OBR2026K |
| Competição | Olimpíada Brasileira de Robótica — temporada 2026 |
| Objetivo geral | Robô móvel com quatro motores, câmera, sensores e controle autônomo/teleoperado |
| Equipe | **PREENCHER: nome oficial da equipe** |
| Escola/instituição | **PREENCHER** |
| Cidade/estado | **PREENCHER** |
| Modalidade e nível da OBR | **PREENCHER** |
| Integrantes e funções | **PREENCHER** |
| Professor/orientador | **PREENCHER** |
| Responsável por eletrônica | **PREENCHER** |
| Responsável por mecânica | **PREENCHER** |
| Responsável por software | **PREENCHER** |
| Repositório remoto | **PREENCHER: URL** |
| Branch principal | **PREENCHER** |
| Licença do projeto | **PREENCHER** |
| Contato da equipe | **PREENCHER** |
| Responsável por manter este guia | **PREENCHER** |
| Data da próxima revisão | **PREENCHER** |

## 2. Resumo do sistema

O robô usa duas unidades de processamento com responsabilidades diferentes:

- A **ESP32** controla diretamente motores, encoders, bateria, ultrassônico,
  botão, MPU6050, PCA9685 e OLED. Ela é a última autoridade de segurança antes
  dos drivers de motor.
- A **Raspberry Pi** executa a estratégia, processa a câmera, hospeda o dashboard,
  lê métricas do Linux e envia comandos de movimento para a ESP32 por UART.

Existem dois firmwares para a ESP32:

1. `obr_esp32_main`: versão principal, sem Wi-Fi, usada com a Raspberry Pi.
2. `obr_esp32_bridge`: versão de bancada, com ponto de acesso Wi-Fi e dashboard
   local para testar o hardware sem a Raspberry.

Os dois firmwares compartilham o mesmo núcleo de sensores, filtros, pinagem e
proteções. Isso reduz a possibilidade de um teste de bancada funcionar de forma
diferente do firmware usado no robô completo.

## 3. Arquitetura

```mermaid
flowchart LR
    Browser[Dashboard no navegador] <-->|HTTP + WebSocket| Pi[Raspberry Pi]
    Camera[Pi Camera V2] --> Capture[Captura e stream MJPEG]
    Capture -->|Status básico + vídeo| Pi
    Pi --> State[RobotState]
    State --> MotorController[MotorController]
    MotorController -->|UART 115200| ESP[ESP32]
    ESP -->|telemetria UART a cada 100 ms| Pi
    ESP --> Drivers[DRV8833 / tração]
    Drivers --> Motors[4 motores: 2 por lado]
    Encoders[Encoder esquerdo + direito] --> ESP
    Sensors[MPU6050 + ultrassônico + bateria + botão] --> ESP
    ESP --> OLED[SSD1306 128x64]
    ESP --> PCA[PCA9685]
    Pi --> ReadyLed[LED de pronto BCM GPIO26]
```

### Fluxo de um comando manual

1. O navegador envia `drive` pelo WebSocket.
2. `DashboardServer` valida o tipo do comando e atualiza `RobotState`.
3. `RobotState` aceita movimento somente no modo manual e limita a faixa.
4. `MotorController` lê uma cópia do estado e envia `MOTOR` pela UART.
5. A ESP32 valida novamente o comando e escreve o mesmo valor nos PWMs dos DRV8833.
6. Os encoders informam no dashboard como cada lado respondeu.
7. Timeout e E-Stop continuam zerando imediatamente os quatro PWMs.

### Fluxo autônomo principal

1. `MissionController` identifica a missão autônoma selecionada.
2. `MainMission` coordena segue-faixa, retorno verde e desvio de obstáculo.
3. A missão isolada `rescue_area` liga o detector de vítimas somente durante sua execução.
4. Cada comportamento atualiza `RobotState`, sem acessar GPIO ou dashboard.
5. `MotorController` aplica os limites antes de enviar comandos à ESP32.
6. E-Stop, timeout e parada continuam tendo prioridade sobre a autonomia.

A Missão Principal preserva o segue-faixa geométrico atual e entrega autoridade
temporária ao desvio quando o ultrassônico confirma um obstáculo. A visão de
vítimas é uma etapa separada e não consome CPU enquanto o robô segue a linha.

## 4. Hardware conhecido

| Componente | Situação conhecida |
| --- | --- |
| Controlador de baixo nível | ESP32, compilada atualmente como `DOIT ESP32 DEVKIT V1` |
| Computador principal | Raspberry Pi; **PREENCHER modelo e revisão** |
| Câmera | Código preparado para Raspberry Pi Camera V2; **confirmar modelo físico** |
| Tração | Quatro motores, agrupados em lado esquerdo e lado direito; dianteiros e traseiros possuem comportamentos/modelos diferentes |
| Drivers | DRV8833 com PWM e nSLEEP |
| IMU | MPU6050 |
| Expansor PWM | PCA9685 em `0x40`, com braço, pulso e garra nos canais 0, 1 e 2 |
| Display | SSD1306 128×64, procurado em `0x3C` e `0x3D` |
| Distância frontal | Sensor ultrassônico; **PREENCHER modelo exato** |
| Encoders | Um conjunto quadrature por lado; **PREENCHER modelo e resolução** |
| Bateria | Níquel, nominal 12 V, referência visual de 10,5 V a 14,0 V |
| Alimentação dos motores | 8 V, conforme montagem atual |
| PCB | Placa própria; **PREENCHER revisão, autor e link do esquemático** |

### Informações mecânicas pendentes

> **PREENCHER:** estas informações afetam controle, odometria e testes.

| Campo | Valor |
| --- | --- |
| Massa total | **PREENCHER em kg** |
| Comprimento × largura × altura | **PREENCHER em mm** |
| Distância entre lados das rodas | **PREENCHER em mm** |
| Distância entre eixos | **PREENCHER em mm** |
| Diâmetro das rodas | **PREENCHER em mm** |
| Material das rodas | Borracha; **PREENCHER modelo/dureza** |
| Modelo dos motores dianteiros | **PREENCHER** |
| Modelo dos motores traseiros | **PREENCHER** |
| Relação da caixa de redução | **PREENCHER** |
| RPM nominal na tensão usada | **PREENCHER** |
| Torque nominal/estol | **PREENCHER** |
| Corrente nominal/estol | **PREENCHER** |
| Tipo de apoio adicional | **PREENCHER: rodízio, esfera, skid etc.** |
| Posição da câmera em relação ao chão | **PREENCHER altura e ângulo** |
| Posição e eixos do MPU6050 | **PREENCHER com foto ou desenho** |

## 5. Pinagem da ESP32

| GPIO ESP32 | Ligação | Função no software |
| --- | --- | --- |
| 32 | Ultrassônico TRIG | Pulso frontal de 10 µs |
| 33 | Ultrassônico ECHO | Captura por interrupção; deve permanecer até 3,3 V |
| 27 | Botão Start | `INPUT_PULLUP`, botão ligado ao GND |
| 14 | I2C SCL | Clock compartilhado a 400 kHz |
| 13 | I2C SDA | Dados compartilhados |
| 26 | DRV8833 nSLEEP | LOW desliga pontes; HIGH habilita após PWMs zerados |
| 5 / 18 | Motor esquerdo IN1 / IN2 | Um comando lógico para os dois motores esquerdos |
| 16 / 17 | Motor direito IN1 / IN2 | Um comando lógico para os dois motores direitos |
| 22 / 23 | Encoder esquerdo A / B | Lado físico validado na PCB; quadrature nas quatro bordas |
| 19 / 21 | Encoder direito A / B | Lado físico validado na PCB; quadrature nas quatro bordas |
| 1 | UART TX | ESP32 envia telemetria à Raspberry |
| 3 | UART RX | ESP32 recebe comandos da Raspberry |
| 36 | ADC da bateria | Divisor de 47 kΩ / 10 kΩ |

### Observações críticas da ESP32

- GPIO1 e GPIO3 também participam da gravação e da serial padrão. A Raspberry
  deve ficar desligada, desconectada ou silenciosa durante a gravação.
- GPIO36 é somente entrada e pertence ao ADC1, podendo funcionar junto com Wi-Fi.
- O ECHO de módulos ultrassônicos alimentados em 5 V precisa de adaptação para
  no máximo 3,3 V.
- A denominação esquerda/direita acima é a lógica final corrigida no software.
- O número 26 aparece também na Raspberry, mas são pinos de placas diferentes:
  ESP32 GPIO26 é nSLEEP; Raspberry BCM GPIO26 é o LED de pronto.

## 6. Pinagem da Raspberry Pi

| GPIO Raspberry (BCM) | Ligação | Função |
| --- | --- | --- |
| 14 / TXD | ESP32 GPIO3 / RX | Envia comandos UART |
| 15 / RXD | ESP32 GPIO1 / TX | Recebe telemetria UART |
| 26 | LED verde | Indica sistema pronto, ativo em HIGH |
| 21 / BOARD 40 | Iluminação da câmera | Ligada enquanto o script da câmera está ativo |
| GND | GND da ESP32 | Referência elétrica comum obrigatória |

> **PREENCHER:** tensão, resistor e modelo do LED verde; transistor ou estágio de
> potência usado na iluminação; diagrama da alimentação compartilhada.

## 7. Alimentação elétrica

### Confirmado

- A bateria é de níquel, nominalmente 12 V.
- O indicador considera 10,5 V como vazio e 14,0 V como cheio.
- Os motores estão sendo alimentados em 8 V.
- A bateria chega ao GPIO36 por um divisor de 47 kΩ e 10 kΩ, relação 5,7:1.
- O firmware faz média de 16 leituras ADC a cada 250 ms.
- A tensão é apenas telemetria: ainda não existe corte automático por bateria baixa.

### PREENCHER — diagrama de potência

- Capacidade da bateria: **____ mAh/Ah**
- Corrente máxima recomendada: **____ A**
- Modelo/revisão do regulador de 8 V: **____**
- Corrente máxima do regulador: **____ A**
- Tensão que alimenta ESP32: **____ V**
- Tensão que alimenta Raspberry Pi: **____ V**
- Fusível principal: **modelo/valor/local ____**
- Chave geral: **modelo/local ____**
- Proteção contra inversão: **____**
- Medição real do divisor contra multímetro: **fator de correção ____**
- Autonomia medida: **____ minutos**

Não ligue a bateria de até 14 V diretamente ao VM do DRV8833. A documentação
atual do projeto considera 10,8 V como limite superior do driver, portanto o
estágio de 8 V deve permanecer no circuito.

## 8. Organização do repositório

```text
CMakeLists.txt
  Build C++17 da aplicação principal da Raspberry.

include/obr/
  Interfaces e configuração central da Raspberry.

src/
  Implementação do dashboard, HAL, estado, estratégia e telemetria.

esp32/obr_esp32_main/
  Entrada do firmware principal sem Wi-Fi.

esp32/obr_esp32_bridge/
  Núcleo compartilhado e firmware de bancada com dashboard local.

scripts/camera_line_frame.py
  Entrypoint e fachada compatível da visão inferior.

scripts/vision/
  Configuração, percepção, guidance, controle, status e lifecycle da câmera inferior.

scripts/forward_camera_stream.py e scripts/ball_vision/
  Captura frontal, assistente leve de linha e detector condicionado de vítimas.

obstacle_avoidance/
  Máquina de estados, calibrações e testes do desvio de obstáculo.

scripts/deploy.ps1 e scripts/deploy.sh
  Entrypoints compatíveis do deploy atômico para a Raspberry.

deployment/
  Implementação do deploy, instalação, unidades systemd e sudoers.

scripts/run_robot.sh
  Supervisor do programa C++ e do gerenciador ocioso da câmera frontal.

scripts/run_line_camera.sh
  Supervisor exclusivo da câmera inferior: remove IPC antigo, publica o estado
  do serviço e inicia uma única instância da visão.

deployment/systemd/obr-robot.service
  Unidade systemd do programa C++, dashboard e ESP32.

deployment/systemd/obr-line-camera.service
  Unidade systemd independente da captura e visão da câmera inferior.

tests/python/
  Regressão Python da visão inferior, câmera frontal e detecção de bolas.

tools/camera/ e tools/diagnostics/
  Diagnósticos manuais de câmera e performance.
```

### Responsabilidade dos módulos C++

| Módulo | Responsabilidade |
| --- | --- |
| `main.cpp` | Liga os módulos, executa o loop de 20 ms e coordena prontidão |
| `RobotState` | Guarda modo, missão, E-Stop, potências e idade do comando |
| `DashboardServer` | HTTP, WebSocket, interface, câmera e telemetria para navegador |
| `Esp32Bridge` | UART, parser de sensores e envio de comandos |
| `MotorController` | Aplica START/RUN por roda no autônomo, sincroniza somente a reta pelos encoders e envia comandos seguros para a ESP32 |
| `MissionController` | Seleciona a missão e reinicia o estado interno a cada nova execução |
| `MainMission` | Coordena o segue-faixa, seus fallbacks e os demais comportamentos autônomos |
| `BallAlignmentMission` | Alinha pelo `tx` da vítima travada e aproxima até a distância segura configurada |
| `ObstacleAvoidance` | Confirma o ultrassônico e executa o desvio completo com IMU e encoders |
| `CameraMonitor` | Valida captura e telemetria visual antes de liberar prévias para a missão |
| `Telemetry` | CPU, temperatura e RAM da Raspberry |
| `GpioPin` | Acesso simples ao GPIO Linux por `/sys/class/gpio` |
| `StatusLed` | LED de sistema pronto |

O binário ainda se chama `robot_test`, apesar de já ser a aplicação principal.
Uma renomeação futura pode melhorar a clareza, mas deve atualizar CMake, scripts,
serviço e documentação em conjunto.

## 9. Firmware da ESP32

### Firmware principal

Arquivo de entrada: `esp32/obr_esp32_main/obr_esp32_main.ino`.

Ele define `OBR_ESP32_RASPBERRY_MODE` e inclui o núcleo compartilhado. Nesse modo:

- não cria ponto de acesso Wi-Fi;
- não inicia servidor HTTP;
- não inclui o HTML do dashboard de bancada;
- recebe movimento somente pela UART da Raspberry;
- continua executando todos os sensores, OLED e mecanismos de segurança.

### Firmware de bancada

Arquivo: `esp32/obr_esp32_bridge/obr_esp32_bridge.ino`.

Ele cria uma rede Wi-Fi local e permite:

- habilitar/desabilitar motores;
- controlar por sliders, botões e WASD;
- observar sensores e periféricos;
- zerar encoders;
- calibrar o MPU6050;
- usar E-Stop sem Raspberry.

A credencial do ponto de acesso está atualmente definida no código. Não copie a
senha para documentos públicos. Antes de publicar o repositório, substitua-a por
configuração local ou confirme que é uma credencial descartável de bancada.

## 10. Controle dos motores

### Modelo lógico

O robô possui quatro motores, mas somente dois comandos de tração:

- `left`: controla os dois motores esquerdos;
- `right`: controla os dois motores direitos.

Os comandos usam a faixa `-1.0` a `1.0`. Zero para; o sinal escolhe o sentido.
A relação física de fios dentro de cada lado precisa permanecer igual para que
os dois motores do mesmo lado girem no mesmo sentido.

> **PREENCHER:** quantos DRV8833 físicos existem, quais canais alimentam cada
> motor e se há canais em paralelo. Anexar esquema da PCB e fotos identificadas.

### PWM

- Frequência: 20 kHz.
- Resolução: 10 bits.
- Duty máximo: 1023.
- Saída máxima: 100%.
- A ESP32 mantém relação direta: `0.05` é 5%, `0.50` é 50% e `1.00` é 100% de duty.
- No modo autônomo, uma roda considerada parada recebe no mínimo `0.67`. Essa
  regra é individual por lado e vale para frente e ré.
- Uma roda só troca de STARTING para RUNNING após duas amostras novas, recentes
  e coerentes do encoder: PWM aplicado no mesmo sentido e taxa de pelo menos
  `20 cont/s`. Em RUNNING, o menor comando não nulo permitido é `0.61`.
- Três amostras recentes inválidas retornam a roda para STARTING. Zero ou troca
  de sentido também reiniciam essa confirmação.
- O `MotorController` mede `cont/s por PWM` e, somente em avanço reto, reduz
  gradualmente o lado mais eficiente. Não existe ganho fixo aplicado ao lado direito.
- Os controles manuais do dashboard usam `drive_raw`: ignoram START/RUN e
  sincronismo para permitir diagnóstico direto entre `0.05` e `1.00`.
- A missão isolada de giro de 90° usa comando lógico `0.01`; o perfil autônomo
  o eleva inicialmente para aproximadamente `0.67 / -0.67`, garantindo a partida.
- Zero permanece exatamente zero em parada, timeout, calibração e E-Stop.

O software de navegação escolhe uma referência operacional. A interface mostra
lado a lado essa referência e o PWM efetivamente aplicado pela ESP32.

### Controle independente dos lados

As rodas omni dianteiras eliminam a antiga preocupação com arraste. A Raspberry
e a ESP32 aceitam um lado em zero e o outro em movimento sem alterar o comando.

### Diagnóstico por encoders

A ESP32 envia contagem e taxa dos dois encoders para o dashboard. A taxa, em
`cont/s`, representa velocidade instantânea e volta a zero quando a roda para; a
contagem acumulada só é zerada por reset ou calibração explícita. O painel também
converte a contagem em posição desde o último reset usando a calibração atual de
`3600 contagens = 18,7 cm`, equivalente a aproximadamente `192,51 contagens/cm`.

Além da missão `drive_distance`, os encoders confirmam individualmente que cada
roda saiu da inércia antes de liberar o piso RUNNING de `0.61`. O sincronismo de
eficiência continua deliberadamente restrito à reta; ele não corrige curvas,
giros nem comandos manuais diretos.

## 11. Camadas de segurança

| Camada | Comportamento |
| --- | --- |
| Inicialização antecipada | GPIOs dos motores e nSLEEP são forçados para LOW antes do `setup()` |
| Startup normal | Driver só é habilitado após os quatro PWMs estarem em zero |
| Estado parado | Raspberry envia `STOP`; ESP32 zera os quatro PWMs |
| Limites | Raspberry e ESP32 limitam comandos a `[-1, 1]` |
| Controle independente | Cada lado mantém sinal, STARTING/RUNNING e confirmação de encoder próprios |
| Feedback dos encoders | Contagens e taxas são exibidas; confirmam START/RUN, sincronizam somente a reta e atendem a missão de distância |
| Timeout Raspberry | Após 2000 ms sem comando válido, `RobotState` zera potências |
| Timeout ESP32 | Após 500 ms sem comando, os PWMs são zerados |
| E-Stop | Tem prioridade sobre manual, autônomo e dashboard |
| Calibração | Para e trava motores até novo rearme explícito |
| Câmera desatualizada | Missão principal para quando o status passa de 400 ms |
| Telemetria ESP32 | Após 1000 ms, dados são considerados desatualizados |
| Encerramento | Serviço sinaliza o C++; programa envia `STOP` antes de sair |
| Deploy | Serviço é parado antes do build e o binário é trocado atomicamente |

O dashboard ajuda o operador, mas não é a autoridade final. A ESP32 deve
continuar segura mesmo se navegador, Wi-Fi, Raspberry ou UART falharem.

## 12. Sensores e periféricos da ESP32

### MPU6050

- Procurado em `0x68` e `0x69`.
- Acelerômetro configurado para ±8 g.
- Giroscópio configurado para ±500 °/s.
- Filtro interno configurado em 21 Hz.
- Leitura a cada 20 ms.
- Calibração usa 300 amostras com intervalo de aproximadamente 3 ms.
- Bias é calculado para os eixos Y e Z.

#### Yaw/giro integrado

- Gyro Z é convertido para graus por segundo.
- Filtro passa-baixas usa `alpha = 0,75`.
- Valores abaixo de `0,25 °/s` são tratados como ruído.
- O resultado é integrado e normalizado entre -180° e +180°.
- Como não há magnetômetro, o yaw pode acumular deriva com o tempo.

#### Inclinação de rampa

- Combina acelerômetro e gyro Y em filtro complementar.
- Constante de tempo: 0,50 s.
- Sinal atual: `+1,0`.
- A leitura depende da orientação física do MPU6050.

> **PREENCHER:** desenhar os eixos X/Y/Z do módulo instalado e registrar qual
> sinal representa subida e descida. Informar também o erro observado após 1,
> 5 e 10 minutos parado.

### Ultrassônico frontal

- Disparo a cada 100 ms.
- Pulso TRIG de 10 µs.
- Timeout de ECHO de 25 ms.
- Faixa aceita pelo software: 2 cm a 400 cm.
- Na Missão Principal, duas leituras de até 8 cm iniciam o desvio de obstáculo.
- Depois do desvio, três leituras a partir de 15 cm rearmam a detecção para um
  novo obstáculo.

> **PREENCHER:** modelo, tensão, divisor do ECHO, posição, ângulo e distância
> entre o sensor e a frente real do robô.

### Encoders

- Leitura quadrature nos canais A e B.
- Interrupção em todas as mudanças dos quatro sinais.
- Tabela de transição ignora mudanças inválidas.
- Contagem e taxa por segundo são enviadas à Raspberry.
- Taxa é atualizada a cada 250 ms.

> **PREENCHER:** pulsos por volta do encoder, contagens por volta após quadrature,
> relação com a roda, sentido positivo e milímetros por contagem. Sem esses dados
> ainda não existe odometria física em metros.

### Bateria

- ADC de 12 bits com atenuação de 11 dB.
- Média de 16 amostras.
- Divisor nominal de 47 kΩ / 10 kΩ.
- Sem fator de calibração específico da placa.
- Sem proteção automática por subtensão.

### PCA9685

- Endereço esperado: `0x40`.
- Frequência inicial: 50 Hz.
- Todos os 16 canais iniciam desligados.
- Canal 0: braço.
- Canal 1: pulso.
- Canal 2: garra.
- Faixa lógica exposta ao software: 0° a 180°.
- Faixa padrão compilada e validada em bancada: 500–2500 µs por servo.
- Faixa absoluta disponível somente na calibração de bancada: 500–2500 µs.
- Os pulsos associados a 0° e 180° são salvos separadamente para cada servo na
  NVS da ESP32; a ordem dos extremos define automaticamente a inversão.
- E-Stop, `STOP`, calibração, reinício ou perda do heartbeat da Raspberry removem
  os sinais dos três canais.
- O dashboard permite ajuste livre somente no modo Manual.
- No modo Manual, a falta de renovação do dashboard por 2 s remove os sinais dos
  servos sem renovar ou interferir no timeout dos motores.

> **CONFIRMAR NA BANCADA:** medir e registrar no painel os extremos finais do
> MG995 do braço e dos dois MG90S. Os padrões de recuperação ficam em
> `esp32/obr_esp32_bridge/robot_config.h`, mas a operação usa primeiro os perfis
> válidos carregados da NVS.

### OLED SSD1306

- Resolução: 128×64.
- Endereços testados: `0x3C` e `0x3D`.
- Atualização a cada 100 ms.
- Área principal: tensão da bateria.
- Barra: 10,5–14,0 V.
- Área inferior: yaw (`GIR`) e inclinação (`RMP`).
- Em OLED bicolor, título/barra ocupam a faixa amarela e a tensão fica abaixo da
  linha 16 para aparecer totalmente azul.
- Durante calibração, mostra animação, progresso, `NÃO MOVA` e resultado.
- No firmware principal, mostra uma animação indeterminada enquanto a Raspberry,
  a câmera e os serviços ainda não concluíram a inicialização. A tela normal só
  é liberada após o heartbeat `SYSTEM_READY`.
- O dashboard da Raspberry pode mostrar temporariamente um título e duas linhas.
  A página remota expira em até 30 segundos e volta automaticamente para bateria
  e ângulos se a Raspberry desconectar ou deixar de renovar a mensagem.
- E-Stop e calibração local têm prioridade sobre o conteúdo enviado pela Raspberry.

## 13. Botão Start físico

O botão está entre GPIO27 e GND com pull-up interno.

### Toque curto

- Debounce: 30 ms.
- Pressões menores que 80 ms não geram `START_BUTTON,SHORT`; se a borda for
  amostrada enquanto houver movimento, ela ainda pode parar o robô por segurança.
- O evento é enviado somente ao soltar.
- Se o robô estiver em Manual ou Autônomo, a borda de pressão recebida pela
  telemetria para o movimento sem aguardar a soltura.
- O evento curto posterior é consumido para não reiniciar a missão.
- Com E-Stop ativo, é ignorado e não libera a emergência.
- Se o robô estiver parado, inicia a missão autônoma selecionada quando:
  - não houver E-Stop;
  - ESP32 e telemetria estiverem prontas;
  - nSLEEP estiver HIGH;
  - não houver calibração;
  - na missão principal, a câmera estiver ativa e recente;
  - no giro de 90°, yaw e giroscópio estiverem válidos e recentes.

### Pressão por cinco segundos

- Para os motores.
- Zera encoders e referências de navegação.
- Zera yaw, inclinação, filtros e ultrassônico.
- Recalibra bias do MPU6050.
- Mantém a bateria disponível.
- Não gera o evento de toque curto.
- Mantém os motores travados até nova ação explícita.

## 14. Protocolo UART Raspberry ↔ ESP32

### Configuração

- Dispositivo Linux: `/dev/serial0`.
- Baud rate: 115200.
- Formato: 8N1.
- TX/RX cruzados e GND compartilhado.
- O console de login serial da Raspberry deve estar desativado.

### Comandos enviados à ESP32

```text
MOTOR,<left>,<right>,<emergency>
STOP
ESTOP
CLEAR_ESTOP
RESET_ENCODERS
CALIBRATE_SENSORS
SYSTEM_STARTING
SYSTEM_READY
SERVO,<ARM|WRIST|GRIPPER>,<angleDegrees>
SERVO_POSE,<armDegrees>,<wristDegrees>,<gripperDegrees>
SERVO_DISABLE_ALL
SERVO_CAL_BEGIN
SERVO_CAL_PULSE,<ARM|WRIST|GRIPPER>,<pulseUs>
SERVO_CAL_DISABLE
SERVO_CAL_SAVE,<ARM|WRIST|GRIPPER>,<pulseAtZeroUs>,<pulseAt180Us>
SERVO_CAL_END
OLED,<durationMs>,<titleHex>,<line1Hex>,<line2Hex>
OLED_BIG,<durationMs>,<primaryTextHex>,<secondaryTextHex>
OLED_CLEAR
PING
```

### Calibração persistente dos servos

O painel da Raspberry oferece a seção **Calibrar pulsos sem regravar**. Entrar
nesse modo zera a tração e não aplica pulso automaticamente. O operador escolhe
um servo, parte de 1500 µs, aproxima-se dos extremos em passos pequenos e usa o
pulso atual como posição lógica 0° ou 180°. Ao salvar, a ESP32 valida uma faixa
de 500–2500 µs, exige pelo menos 200 µs entre os extremos, deduz o sentido e
grava somente o perfil daquele servo com a biblioteca `Preferences` do próprio
core ESP32.

Durante o ajuste, somente um canal pode ficar ativo. O navegador renova o pulso
a cada 500 ms e o firmware o desliga após 2 s sem renovação. O botão **Parar**, o
comando `STOP`, a perda do heartbeat da Raspberry e qualquer trava de segurança
encerram a calibração e removem os três sinais. Uma gravação normal do sketch
preserva a NVS; uma gravação configurada para apagar toda a flash pode remover
os perfis e fazer o firmware voltar aos padrões compilados.

Procedimento recomendado:

1. Deixe o servo desacoplado da mecânica e alimente o PCA9685 corretamente.
2. Selecione o servo e entre na calibração; confirme que nada se move sozinho.
3. Aplique 1500 µs e procure cada extremo aos poucos, recuando ao notar ruído,
   vibração, aquecimento ou batente.
4. Capture o pulso da posição que o software chamará de 0° e depois o de 180°.
5. Salve o perfil e repita nos outros servos.
6. Encerre com **Parar** e confirme no controle normal os comandos 0°, 90° e
   180° antes de acoplar a mecânica.

Ao entrar em Manual ou Autônomo, o `RobotState` solicita a pose inicial
`{0°, 0°, 0°}`. O firmware converte cada ângulo linearmente para a faixa
500–2500 µs. No modo Parado, os três canais permanecem sem pulso.

O comando `OLED` aceita título de até 12 caracteres, duas linhas de até 20
caracteres ASCII e duração entre 500 e 30.000 ms. Os campos de texto são
codificados em hexadecimal e `-` representa texto vazio. `OLED_CLEAR` restaura
a página padrão sem esperar o timeout.

`OLED_BIG` aceita um texto principal e um detalhe de até 20 caracteres. A ESP32
calcula o maior tamanho da fonte que cabe na largura e mantém os dois textos nas
linhas 16 a 63, região fisicamente azul do OLED bicolor. A cor não é selecionável
por software; em um painel monocromático, o mesmo conteúdo aparecerá na cor única
do módulo. O alerta pulsa suavemente entre os contrastes configurados, sem ficar
completamente apagado, e restaura o contraste normal ao terminar.

Depois que o suporte a `OLED_BIG` estiver gravado, a Raspberry pode alterar o
texto, o detalhe, a duração e os gatilhos sem regravar a ESP32. Somente mudanças
no desenho, na animação ou no protocolo exigem uma nova gravação do firmware.

Um verde confirmado, recente e associado à faixa preta gera `VERDE` com o
detalhe `ESQUERDA`, `DIREITA` ou `180 GRAUS`. O evento possui latch para não ser
reenviado a cada frame. O ponto de integração de obstáculo já aceita um booleano
confirmado e produz apenas `DESVIO`, mas ainda não recebe dados nesta branch.

`SYSTEM_READY` é renovado pela Raspberry a cada segundo depois da primeira
inicialização completa. Sem renovação por três segundos, a OLED retorna ao boot.
`SYSTEM_STARTING` força esse estado imediatamente ao iniciar ou encerrar o serviço.

`MOTOR` aceita potências finitas entre -1,000 e 1,000. Os quatro limites também
devem permanecer ordenados e dentro de `[0, 1]`. Valores inválidos param os
motores e geram erro. O formato antigo com somente três campos continua aceito
e usa os padrões armazenados na ESP32.

### Eventos e respostas da ESP32

```text
READY
PONG
ERR,<reason>
CALIBRATION,START
CALIBRATION,DONE
CALIBRATION,FAILED
START_BUTTON,SHORT
OLED,OK
OLED,CLEARED
SENSOR,<campos CSV...>
```

### Ordem da telemetria `SENSOR`

```text
SENSOR,
distanceCm,gyroZ,yawZ,accelX,accelY,accelZ,mpuOk,
batteryV,leftEncoder,rightEncoder,startButton,pcaOk,
appliedLeft,appliedRight,leftRate,rightRate,
ramp,gyroX,gyroY,imuTemperature,oledOk,nSleepHigh,estop,
batteryAdcMillivolts,uptimeMs,calibrationActive
```

Os campos extras são tratados como opcionais pelo parser da Raspberry para
preservar compatibilidade com versões antigas do firmware.

## 15. Estados e modos da Raspberry

| Modo | Significado |
| --- | --- |
| `stopped` | Potências zeradas; estado padrão |
| `manual` | Aceita `drive` do dashboard/WASD |
| `autonomous` | Aceita somente comandos gerados pela estratégia |
| `emergency` | E-Stop travado e potências zeradas |

Trocar a missão autônoma sempre força o modo parado. Comandos de drive não
mudam o modo por conta própria.

### Missões disponíveis

#### `main_mission` — padrão

É o encapsulamento da estratégia normal da prova. Atualmente executa o
segue-faixa por trajetória visual, seus fallbacks seguros, o retorno verde e o
desvio de obstáculo; futuros comportamentos devem ser chamados explicitamente
por este módulo conforme cada situação exigir.

#### `turn_right_90`

- Salva o yaw inicial do MPU6050.
- Comanda esquerda positiva e direita negativa.
- Alvo: 90°.
- Usa comando lógico `0.01 / -0.01` durante todo o giro e nas correções.
- O perfil autônomo aplica inicialmente aproximadamente `0.67 / -0.67` nos motores.
- O sincronismo fica desativado porque os lados giram em sentidos opostos.
- Antecipa o corte do PWM usando velocidade angular, idade da amostra e inércia.
- Aguarda 180 ms e velocidade angular de até 3°/s antes de avaliar o resultado.
- Permite até três pulsos de 60 ms, sempre separados por estabilização.
- Se faltar ângulo, o pulso mantém o sentido; se ultrapassar, o pulso é reverso.
- Tolerância de parada: ±2°.
- Timeout: 5 s.
- Encerra a missão e zera os motores se o MPU ficar indisponível ou a amostra passar de 200 ms.

> **Em validação:** potência, tolerância e inércia precisam ser confirmadas no
> piso real, com bateria em diferentes tensões. A medição angular atual usa o
> módulo da diferença; confirmar também o sinal físico do giro.

#### `drive_distance`

- Aceita alvo de 1 a 300 cm; o padrão é 20 cm.
- Usa a calibração empírica `3600 contagens = 18,7 cm` com rodas de 68 mm.
- Captura as duas contagens no início sem zerar o histórico da ESP32.
- Considera o menor avanço entre esquerda e direita, exigindo que ambos os lados
  percorram o trecho.
- Usa comando lógico `0.01 / 0.01`, convertido pelo perfil operacional.
- Antecipa a parada usando a taxa de cada encoder e a idade da telemetria.
- Aguarda 300 ms com PWM zero e permite até três correções curtas.
- Tolerância atual: 0,5 cm.
- Para se a telemetria passar de 300 ms, se um lado ficar 1,5 s sem avanço ou se
  a missão ultrapassar 60 s.

> **Em validação:** confirme a distância no piso em vários alvos e tensões de
> bateria. Ajuste `kEncoderCalibrationCounts`, `kEncoderCalibrationDistanceCm` e
> a previsão de frenagem em `include/obr/config.h` com novas medições.

#### `rescue_area`

- É uma etapa explícita: selecionar a missão não liga a visão pesada; pressionar
  Auto ou o botão físico abre o gate do detector.
- Detecta vítimas pretas por HSV/contornos e prateadas por Hough, brilho e textura.
- Mantém o assistente frontal a 30 FPS e limita o detector pesado a 15 análises
  por segundo enquanto o gate estiver aberto.
- Escolhe inicialmente a maior área visível, confirma por três frames e mantém o
  mesmo alvo travado durante toda a execução.
- Alinha pelo `tx`, com frenagem antes da confirmação, e aproxima até 5 cm.
- IPC frontal com mais de 500 ms ou medição inválida mantém os motores zerados.
- Depois da primeira aquisição, perder o alvo por 1 s encerra a missão.
- Stop, E-Stop, troca de missão, conclusão e falha desligam o detector e impedem
  reutilizar o alvo de uma execução anterior.

Ainda não existe um classificador automático de entrada na área de resgate. Até
ele ser implementado, a transição para `rescue_area` é explícita pelo dashboard;
um classificador futuro deve apenas selecionar/iniciar esta mesma etapa segura.

## 16. Câmera ao vivo

Arquivo: `scripts/camera_line_frame.py`.

### Captura e publicação

| Parâmetro | Valor atual |
| --- | --- |
| Resolução da câmera inferior | 480×360 |
| FPS alvo da câmera inferior | 30 |
| Stream MJPEG | Porta local 8090, `/stream.mjpg` |
| Qualidade JPEG | 82 |
| Snapshot compatível | 2 FPS em `/tmp/obr_camera_frame.jpg` |
| Status JSON | Saúde da captura em `/tmp/obr_camera_status.json` |
| Rotação | 180°, feita no pipeline com HFlip + VFlip |
| Iluminação | BOARD 40 / BCM21 ativa enquanto o script roda |

O dashboard principal faz proxy do MJPEG pela porta 8080, evitando que o usuário
precise acessar diretamente a porta 8090. O vídeo é exibido sem máscaras,
marcações ou decisões de navegação.

`CameraMonitor` valida a saúde da captura, o comando seguro do ponto de extensão
e os resultados necessários à máquina de estados verde. A interpretação de pixels
permanece isolada no processo Python; toda aplicação de movimento passa por
`RobotState` e `MotorController`.

A câmera inferior pertence ao serviço `obr-line-camera`, separado de
`obr-robot`. Ao parar ou reiniciar esse serviço, o supervisor remove
`/dev/shm/obr_line_status.json`; por isso uma Missão Principal em curso não pode
reaproveitar dados visuais antigos e fica parada até uma publicação nova e fresca.
Máscaras, filtros e detecção verde usam a resolução recebida como base: a referência
validada de 640×480 é escalada para 480×360 sem manter cortes fixos.

O dashboard controla a câmera inferior pelo mesmo IPC simples usado pela câmera
frontal. O botão alterna apenas `{"command":"set_line_camera","enabled":true}`
ou `false` pelo WebSocket; o C++ grava o arquivo de controle atômico em
`/dev/shm`, sem executar `sudo`, `systemctl` ou qualquer comando recebido do
navegador. O gerenciador `obr-line-camera` fica ocioso quando desativado e remove
o IPC de visão. `ONLINE` só aparece depois de uma publicação nova da visão;
`PARADA`, `INICIANDO` e `FALHA` continuam explícitos quando ela não pode ser usada.

## 17. Estrutura autônoma atual

`MissionController` possui uma responsabilidade simples: selecionar a missão e
reiniciar seu estado quando uma nova execução começar. Ele preserva duas missões
isoladas de teste, `turn_right_90` e `drive_distance`, executa `rescue_area` com
`BallAlignmentMission` e delega a estratégia normal para `MainMission`.

`MainMission` mantém a composição dos comportamentos da prova. O desvio de
obstáculo possui prioridade sobre o segue-faixa depois de duas leituras de até
8 cm; uma vez iniciado, mantém autoridade até concluir a ré final ou falhar. A
classificação verde também alimenta a OLED depois de passar pelas validações de atualização,
confirmação e associação com a faixa preta. O retorno de 180° já possui uma
manobra própria; verde à esquerda e à direita ainda não iniciam movimento nesta
branch e continuam disponíveis como percepção, telemetria e alerta visual.

A classificação verde usa somente ROIs locais ao redor dos marcadores para medir
a presença da faixa preta à frente, à esquerda e à direita. Essa percepção
distingue esquerda, direita, retorno, falso e ambíguo sem manter geometrias ou
estados do controlador visual removido.

Cada comportamento novo deve ser integrado de forma deliberada pelo
`MainMission`, com critérios claros de entrada, término e falha.

Regras para os próximos comportamentos:

- cada módulo deve ter uma única responsabilidade;
- decisões de missão pertencem a `MainMission`;
- resultados de sensores entram por estruturas de dados, sem acesso ao dashboard;
- comandos de movimento passam por `RobotState` e `MotorController`;
- cada comportamento deve definir como entra, como termina e como para os motores;
- perda de sensor, E-Stop ou timeout deve resultar no estado mais seguro;
- um novo comportamento visual deve ser isolado e testado antes da integração.

### Recursos ainda não usados pela missão principal

- Inclinação de rampa é apenas telemetria/OLED.
- Encoders não fecham velocidade, mas confirmam START/RUN por roda e sincronizam
  eficiência exclusivamente nos deslocamentos retos.
- PCA9685 aciona braço, pulso e garra, mas a Missão Principal ainda não possui
  uma sequência autônoma definida para esses mecanismos.
- A área de resgate possui detecção, alinhamento e aproximação da vítima, mas
  ainda não possui entrada automática, coleta, entrega nem lógica do kit.

> **PREENCHER:** estratégia completa exigida pela modalidade da equipe e status
> de cada desafio do regulamento 2026.

## 18. Dashboard da Raspberry

### Rede

- Porta: 8080.
- URL típica: `http://raspberrypi.local:8080`.
- IP padrão atual do deploy Windows: `192.168.0.106`.
- WebSocket usa o mesmo host em `/ws`.
- Não há autenticação, TLS ou perfis de usuário; use somente em rede confiável.

### Controles

- `Manual`: libera teleoperação e envia `CLEAR_ESTOP` à ESP32.
- `Autônomo`: inicia a missão selecionada.
- `Parar`: zera movimento sem travar E-Stop.
- `E-Stop`: trava emergência.
- `Resetar e calibrar sensores`: para e calibra a ESP32.
- Seletor de missão: `MISSÃO PRINCIPAL`, `GIRO 90° À DIREITA` ou
  `PERCORRER DISTÂNCIA`.
- Missão de distância: campo de 1 a 300 cm e progresso independente dos dois lados.
- WASD: W/S para frente/ré; A/D gira os dois lados em sentidos opostos e tem
  prioridade sobre W/S, impedindo que combinações de teclas zerem um lado.
- Sliders: frente/ré e giro.
- Limites manuais separados: reta e curva, de `0.05` a `1.0`, persistidos no navegador.
- Sliders, WASD e ajuste individual enviam `drive_raw`: são comandos diretos,
  sem START/RUN e sem sincronismo, destinados ao diagnóstico consciente.
- Perfil autônomo: roda parada recebe no mínimo `0.67`; após duas confirmações
  recentes do encoder, pode manter `0.61`.
- Giro autônomo de 90°: comando `0.01` elevado inicialmente pelo perfil de partida.
- Sincronização: calcula a eficiência de cada lado em `cont/s por PWM`, aguarda
  três amostras válidas e reduz somente o lado mais rápido em passos de até 0,03.
  A escala respeita o estado de cada roda: não rebaixa STARTING abaixo de `0.67`
  nem RUNNING abaixo de `0.61`. Giros opostos, curvas diferenciais e ajustes
  individuais não entram nessa malha.
- A ESP32 atualiza a taxa dos encoders a cada 100 ms, sincronizada com o período
  da telemetria UART; a Raspberry filtra essas amostras antes de ajustar a escala.

Perder foco, ocultar a aba, soltar teclas ou perder WebSocket zera comandos do
navegador. A segurança não depende apenas disso: os timeouts inferiores continuam
ativos.

### Telemetria exibida

- modo e missão;
- bateria e ADC;
- link, uptime e idade da ESP32;
- CPU, RAM e temperatura da Raspberry;
- câmera, FPS, resolução e formato;
- potência solicitada pela Raspberry, comando final e PWM aplicado pela ESP32;
- estado STARTING/RUNNING e amostras de confirmação de cada lado;
- contagens e taxas dos encoders;
- estado pendente do segue-faixa, sequência da visão, reparo especular e decisão verde;
- ultrassônico;
- MPU6050, yaw, rampa, gyro, aceleração e temperatura;
- PCA9685 e OLED;
- nSLEEP, E-Stop e calibração;
- LED de pronto, UART e timeouts.

## 19. LED de sistema pronto

O LED verde da Raspberry no BCM GPIO26 acende apenas quando:

- UART está aberta;
- telemetria da ESP32 está recente;
- nSLEEP está HIGH;
- não há E-Stop local da ESP32;
- não há calibração ativa;
- a câmera está ativa, com FPS maior que zero e status recente;
- o `RobotState` não está em emergência.

O LED não indica que a pista foi reconhecida ou que a bateria está carregada. Ele
indica que a infraestrutura necessária para iniciar está presente.

## 20. Dependências e build

### Raspberry Pi

- Compilador C++ com suporte a C++17.
- CMake 3.16 ou superior.
- pthread.
- Python 3.
- `python3-picamera2`.
- `python3-opencv`.
- `python3-numpy`.
- `RPi.GPIO`.

Build manual:

```sh
cmake -S . -B build
cmake --build build
```

No Windows o C++ também compila para verificação, mas a UART `/dev/serial0`, os
GPIOs e a Pi Camera só funcionam na Raspberry.

### ESP32 / Arduino IDE

Placa atualmente usada na compilação:

```text
DOIT ESP32 DEVKIT V1
FQBN: esp32:esp32:esp32doit-devkit-v1
```

Bibliotecas Arduino necessárias:

- Adafruit MPU6050;
- Adafruit Unified Sensor;
- Adafruit PWM Servo Driver Library;
- Adafruit GFX Library;
- Adafruit SSD1306.

`Preferences`, usada para salvar a calibração dos servos na NVS, já acompanha o
core **esp32 by Espressif Systems** e não exige instalação separada.

Firmware principal para o robô:

```text
esp32/obr_esp32_main/obr_esp32_main.ino
```

Firmware de teste sem Raspberry:

```text
esp32/obr_esp32_bridge/obr_esp32_bridge.ino
```

### Uso de memória observado em 06/08/2026

| Alvo | Flash | RAM global |
| --- | --- | --- |
| ESP32 principal | 380.763 bytes (29%) | 24.848 bytes (7%) |
| ESP32 de bancada | 1.059.767 bytes (80%) | 49.684 bytes (15%) |

Esses números variam com a versão do ESP32 Arduino Core e das bibliotecas. O
firmware de bancada é bem maior porque inclui Wi-Fi, servidor HTTP e HTML.

## 21. Deploy e inicialização automática

### Windows

```powershell
.\scripts\deploy.ps1 -HostName 192.168.0.106 -Service
```

### Linux/macOS

```sh
bash scripts/deploy.sh --host raspberrypi.local --service
```

### Comportamento seguro do deploy

1. Para `obr-robot` e `obr-line-camera` antes de alterar código/binário.
2. Copia fontes e scripts.
3. Compila em `/home/obr/OBR2026K/.build-staging`.
4. Confirma que o novo `robot_test` não está vazio.
5. Instala como `robot_test.new`.
6. Faz `mv` atômico para o caminho final.
7. Reinstala as duas unidades systemd.
8. Habilita os dois serviços no boot.
9. Reinicia e exige estado `active` dos dois serviços.

Se o build falhar, o executável anterior não é substituído e o robô permanece
parado.

### Serviço systemd

| Item | Valor |
| --- | --- |
| Nome | `obr-robot.service` e `obr-line-camera.service` |
| Usuário | `raspberry` |
| Grupo suplementar | `gpio` |
| Diretório | `/home/obr/OBR2026K` |
| Entrada | `scripts/run_robot.sh` e `scripts/run_line_camera.sh` |
| Reinício | `obr-robot`: sempre; `obr-line-camera`: apenas em falha, após 2 s |
| Limite | 5 partidas em 30 s |
| Boot | `multi-user.target` |

No deploy validado em 06/08/2026, o binário principal da Raspberry ficou com
303.600 bytes, formato ELF ARM64/AArch64. Esse tamanho é apenas referência; a
validação do deploy verifica que o arquivo não está vazio, não exige um tamanho
fixo.

`run_robot.sh` recusa binário ausente, vazio ou sem permissão de execução antes
de iniciar o programa principal. Ele não inicia a câmera inferior; isso evita duas
instâncias e deixa a visão desligável sem afetar dashboard ou ESP32.

## 22. Procedimento recomendado de operação

### Antes de energizar

- [ ] Robô apoiado de forma segura ou rodas suspensas para primeiro teste.
- [ ] Bateria sem dano, inchaço, aquecimento ou conectores soltos.
- [ ] Polaridade e fusível conferidos.
- [ ] Nenhum cabo próximo às rodas.
- [ ] GND comum entre Raspberry, ESP32 e eletrônica de potência.
- [ ] UART conectada corretamente.
- [ ] ECHO do ultrassônico limitado a 3,3 V.
- [ ] nSLEEP conectado ao ESP32 GPIO26.
- [ ] Motores de cada lado giram no mesmo sentido elétrico.

### Ao ligar

1. Energize o robô com rodas suspensas após qualquer alteração de motor/PCB.
2. Observe que não há movimento durante o boot.
3. Aguarde o OLED terminar a calibração sem mover o robô.
4. Aguarde o LED verde da Raspberry.
5. Abra o dashboard.
6. Confirme bateria, ESP32 online, câmera ativa e ausência de falhas.
7. Faça um comando curto em baixa demanda lógica.
8. Só então coloque no piso.

### Para iniciar a missão

- Selecione a missão no dashboard; `MISSÃO PRINCIPAL` é o padrão.
- Use `Autônomo` ou toque curto no Start físico.
- O Start físico é ignorado se o sistema não estiver pronto.
- Durante Manual ou Autônomo, pressionar Start para o robô sem aguardar a soltura.

### Para calibrar

- Deixe o robô completamente imóvel.
- Segure Start por cinco segundos ou use o botão no dashboard.
- Aguarde `PRONTO` no OLED.
- Faça novo rearme Manual/Autônomo.

### Para desligar

1. Pare o robô no dashboard.
2. Aguarde potências aplicadas em zero.
3. Encerre/desligue a Raspberry de forma limpa quando possível.
4. Desligue a alimentação de potência.

> **PREENCHER:** ordem física exata das chaves e conectores do robô.

## 23. Plano de testes

### Ordem segura

1. Build local.
2. Teste sem drivers/motores conectados.
3. Teste com rodas suspensas.
4. Teste com potência limitada.
5. Teste curto no piso.
6. Teste com bateria cheia e baixa.
7. Teste em piso semelhante ao da competição.

### Matriz de validação

| Teste | Status | Data/responsável/evidência |
| --- | --- | --- |
| C++ compila no Windows | Confirmado | **PREENCHER última execução registrada** |
| Firmware ESP32 principal compila | Confirmado | **PREENCHER versão Arduino/Core** |
| Firmware ESP32 bancada compila | Confirmado | **PREENCHER versão Arduino/Core** |
| Serviço inicia sozinho após reboot | Confirmado em 06/08/2026 | Raspberry em `192.168.0.106` |
| Dashboard responde na porta 8080 | Confirmado em 06/08/2026 | HTTP 200 |
| Câmera publica aproximadamente 30 FPS | Confirmado em 06/08/2026 | 31,45 FPS após aquecimento |
| LED pronto acompanha câmera + ESP32 | Confirmado em bancada | **PREENCHER vídeo/foto** |
| E-Stop zera os quatro motores | **PREENCHER teste físico** | |
| Timeout Raspberry zera motores | **PREENCHER** | |
| Timeout ESP32 zera motores | **PREENCHER** | |
| Diagnóstico raw 0,05 aplica aproximadamente 5% de PWM | **PREENCHER** | |
| Sincronismo reduz o lado mais rápido sem acelerar o lento | **PREENCHER** | Testar em reta no piso |
| Cada lado pode ser acionado independentemente | **PREENCHER** | Rodas omni dianteiras |
| Ambos os lados giram em W/S | **PREENCHER** | |
| A/D gira com os dois lados em sentidos opostos | **PREENCHER** | Rodas suspensas |
| Giro de 90° para no alvo | **PREENCHER** | |
| Stream da câmera não possui sobreposições | **PREENCHER** | Conferir imagem direta no dashboard |
| Missão Principal mantém os motores zerados | **PREENCHER** | Testar com as rodas suspensas |
| Rampa indica sinal correto | **PREENCHER** | |
| Bateria confere com multímetro | **PREENCHER** | |
| Autonomia completa de prova | **PREENCHER** | |

Não existem testes automatizados no CMake atualmente. `ctest` não encontra casos
cadastrados.

## 24. Diagnóstico rápido

### Sistema não inicia no boot

```sh
systemctl is-enabled obr-robot
systemctl is-active obr-robot
sudo systemctl status obr-robot --no-pager
journalctl -b -u obr-robot --no-pager
```

Verifique o binário:

```sh
stat /home/obr/OBR2026K/build/robot_test
file /home/obr/OBR2026K/build/robot_test
```

Um incidente anterior deixou `robot_test` com zero bytes. O deploy atual usa
staging, validação e troca atômica para impedir recorrência.

### Dashboard não abre

```sh
curl -I http://127.0.0.1:8080/
ss -ltnp | grep 8080
```

### Câmera não aparece

```sh
curl http://127.0.0.1:8080/camera-status.json
pgrep -af camera_line_frame.py
journalctl -u obr-robot -f
```

Confira `active`, `fps` e `error` no JSON.

### ESP32 aparece offline

```sh
ls -l /dev/serial0
fuser /dev/serial0
```

Confirme:

- console serial desativado;
- TX/RX cruzados;
- GND comum;
- baud rate 115200;
- firmware `obr_esp32_main` gravado;
- nenhuma outra aplicação usando a UART.

### Um lado não gira

1. Pare imediatamente.
2. Leia no dashboard qual potência foi solicitada e aplicada.
3. Observe contagem/taxa dos dois encoders.
4. Compare a potência aplicada com o limite físico em que os quatro motores começam a girar.
5. Desligue a potência antes de trocar cabos.
6. Teste motor, saída do driver, conector e entrada da PCB separadamente.
7. Após corrigir, teste novamente com as rodas suspensas.

### OLED não aparece

- Verifique 3,3 V/GND e SDA13/SCL14.
- Procure `0x3C` e `0x3D` em scanner I2C.
- Confirme bibliotecas Adafruit GFX e SSD1306.
- Confirme que o construtor é 128×64.

### Bateria não confere

- Meça a bateria e o ponto ADC com multímetro.
- Confirme resistores de 47 kΩ e 10 kΩ.
- Confirme GND comum.
- Calcule e registre um fator de correção real.
- Nunca aplique a bateria diretamente no GPIO36.

## 25. Limitações e dívidas técnicas conhecidas

- Dashboard não possui autenticação ou HTTPS.
- Credencial do Wi-Fi de bancada está no firmware.
- `robot_test` é um nome provisório para o binário principal.
- Parser JSON do C++ é manual e simples, sem biblioteca dedicada.
- O desvio de obstáculo e a missão de resgate possuem testes automatizados, mas
  ainda precisam de validação física completa no robô e na iluminação da arena.
- Bateria baixa não gera parada automática.
- Não há detecção explícita de travamento simultâneo dos dois lados.
- Não há controle fechado de velocidade por encoder.
- A distância física usa uma calibração empírica única; ainda não existe odometria
  2D nem calibração separada por lado.
- Yaw do MPU6050 deriva por não usar referência absoluta.
- Braço, pulso e garra já possuem controle manual e protocolo, mas as poses da
  estratégia final ainda precisam ser definidas pela equipe.
- Rampas, entrada automática no resgate, coleta, kit e outras situações da prova
  ainda não possuem estratégia completa.
- Modelo mecânico e elétrico completo não está versionado neste repositório.
- O acesso GPIO da Raspberry usa `/sys/class/gpio`, interface considerada legada
  em kernels Linux recentes; funciona na configuração atual, mas deve ser
  acompanhado em futuras atualizações do sistema.

## 26. Roadmap sugerido

Esta lista é uma sugestão técnica, não uma decisão automática da equipe:

- [ ] Preencher toda a ficha mecânica e elétrica.
- [ ] Versionar esquemático, PCB e diagrama de potência.
- [ ] Registrar modelo e CPR dos encoders.
- [ ] Calibrar bateria contra multímetro.
- [ ] Criar testes automatizados para `RobotState`, protocolo e missões.
- [ ] Especificar e testar cada novo comportamento antes de integrá-lo à Missão Principal.
- [ ] Implementar detecção de travamento dos dois lados.
- [ ] Definir limite de bateria baixa e política segura.
- [x] Integrar o ultrassônico ao desvio de obstáculo da Missão Principal.
- [ ] Validar fisicamente as distâncias e os ângulos do desvio de obstáculo.
- [ ] Classificar automaticamente a entrada da área de resgate.
- [ ] Calibrar e validar vítimas pretas e prateadas na iluminação da arena.
- [ ] Confirmar no robô os canais e pulsos finais dos três servos do PCA9685.
- [ ] Documentar estratégia completa da modalidade OBR.
- [ ] Renomear `robot_test` quando a equipe decidir o nome final.
- [ ] Remover credencial do firmware antes de tornar o repositório público.

## 27. Questionário para a equipe preencher

### Mecânica

1. Qual é o modelo exato dos motores?
2. Qual é a relação de redução e o RPM em 8 V?
3. Qual é o diâmetro real das rodas sob carga?
4. Qual é a distância entre os centros dos lados esquerdo e direito?
5. Qual é a massa final e a distribuição aproximada de peso?
6. Onde ficam câmera, IMU, ultrassônico e centro de rotação?

### Eletrônica

1. Quantos DRV8833 físicos existem e como os quatro motores estão distribuídos?
2. O GPIO26 controla nSLEEP de todos os drivers ou somente de um?
3. Qual é a revisão da PCB?
4. Qual regulador gera 8 V e qual a corrente máxima?
5. Existem fusível, proteção de inversão e medição de corrente?
6. Qual é a capacidade e o modelo exato da bateria?
7. Como Raspberry e ESP32 são alimentadas?

### Sensores

1. Qual é o CPR efetivo de cada encoder após quadrature e redução?
2. Qual é o modelo do ultrassônico?
3. Como o MPU6050 está orientado fisicamente?
4. O OLED é realmente bicolor amarelo/azul em todas as unidades?
5. Quais são os pulsos finais, as posições seguras e as poses de prova do braço,
   do pulso e da garra?

### Software e prova

1. Qual modalidade/nível e quais desafios do regulamento precisam ser atendidos?
2. O que exatamente compõe a “Missão Principal” final?
3. Qual é o comportamento esperado em obstáculo, rampa, gap, verde e resgate?
4. Quais velocidades são seguras em cada trecho?
5. Quais testes já foram feitos e onde estão os vídeos/dados?
6. Quem pode alterar constantes de motor e segurança antes da prova?

## 28. Referência rápida

### URLs

```text
Dashboard: http://raspberrypi.local:8080
Dashboard pelo IP atual: http://192.168.0.106:8080
Status da câmera: http://raspberrypi.local:8080/camera-status.json
Stream pelo proxy: http://raspberrypi.local:8080/camera-stream.mjpg
```

### Comandos úteis no Windows

```powershell
# Deploy, build e restart seguro
.\scripts\deploy.ps1 -HostName 192.168.0.106 -Service

# Somente compilar C++ localmente
cmake -S . -B build
cmake --build build --config Release

# Ver estado remoto
ssh -i "$env:USERPROFILE\.ssh\obr_raspberry" obr@192.168.0.106 "systemctl status obr-robot --no-pager"
```

### Comandos úteis na Raspberry

```sh
sudo systemctl status obr-robot
sudo systemctl restart obr-robot
journalctl -u obr-robot -f
curl http://127.0.0.1:8080/camera-status.json
```

## 29. Fontes de verdade

| Assunto | Arquivo principal |
| --- | --- |
| Pinos e constantes da Raspberry | `include/obr/config.h` |
| Pinos e constantes da ESP32 | `esp32/obr_esp32_bridge/robot_config.h` |
| Segurança e sensores ESP32 | `esp32/obr_esp32_bridge/obr_esp32_bridge.ino` |
| Protocolo UART na Raspberry | `src/hal/esp32_bridge.cpp` |
| Estados e modos | `src/robot/robot_state.cpp` |
| Seleção de missões | `src/robot/mission_controller.cpp` |
| Orquestração da Missão Principal | `src/robot/main_mission.cpp` |
| Desvio de obstáculo | `obstacle_avoidance/` |
| Visão e alinhamento de vítimas | `scripts/ball_vision/` / `src/robot/ball_alignment_mission.cpp` |
| Dashboard | `src/dashboard/dashboard_server.cpp` |
| Captura das câmeras | `scripts/camera_line_frame.py` / `scripts/forward_camera_stream.py` |
| Deploy Windows/Linux | `scripts/deploy.ps1` / `scripts/deploy.sh`; implementação em `deployment/` |
| Startup | `scripts/run_robot.sh` / `deployment/systemd/obr-robot.service` |

## 30. Regra de manutenção deste guia

Atualize este documento quando ocorrer qualquer uma destas mudanças:

- troca de pino, driver, sensor, motor, bateria ou regulador;
- mudança de protocolo UART;
- novo modo ou missão;
- alteração de timeout ou limite de segurança;
- mudança de deploy, serviço ou endereço de rede;
- calibração relevante de visão, motor, encoder ou bateria;
- conclusão de um item marcado como `PREENCHER`.

Uma pessoa nova deve conseguir responder, somente com este guia e o repositório:

1. O que cada placa faz?
2. Como o robô é ligado sem movimento inesperado?
3. Como gravar, compilar e fazer deploy?
4. Como iniciar, parar, calibrar e usar E-Stop?
5. Onde alterar cada constante?
6. Como diagnosticar câmera, UART, sensor e motor?
7. Quais partes estão prontas e quais ainda dependem da equipe?
