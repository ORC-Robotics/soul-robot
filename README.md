# Soul — OBR2026K

Software do Soul, robô autônomo da ORC Robotics para a Olimpíada Brasileira de Robótica (OBR). Este repositório reúne a versão final utilizada em competição: controle e navegação na Raspberry Pi, firmware da ESP32, percepção visual e painel de operação.

O sistema percorre a pista seguindo a linha, interpreta marcações, trata interrupções do trajeto e obstáculos, executa a coleta e o depósito de vítimas na área de resgate e busca a linha para continuar o percurso.

## Arquitetura

A **Raspberry Pi** executa a estratégia em C++ e a visão em processos Python. A **ESP32** recebe comandos pela UART, aciona motores e servos e devolve as leituras dos sensores. O dashboard da Raspberry permite operação manual, seleção de missões, visualização das câmeras e acompanhamento da telemetria.

```text
Câmera inferior (CAM0) ── percepção da linha e marcações ─┐
Câmera frontal (CAM1) ── linha, vítimas e zonas ──────────┤
                                                       ▼
Dashboard HTTP/WebSocket ↔ Raspberry Pi: missões e RobotState
                                     ↕ UART — 115200 bps
                                   ESP32
                     ┌───────────────┼─────────────────┐
                 Tração          Sensores       PCA9685 / servos
```

`RobotApplication` organiza o ciclo de execução. `MissionController` seleciona a rotina, e `MainMission` coordena percurso, entrada no resgate, sala de resgate e saída. Os módulos de percepção publicam resultados por arquivos de IPC em `/dev/shm`; as missões atualizam `RobotState`, e os controladores de motores e servos enviam os comandos à ESP32. A percepção não aciona GPIO diretamente.

A comunicação serial usa comandos textuais e telemetria CSV, incluindo encoders, IMU, distância frontal, bateria, botão, atuadores e estados de segurança. O firmware principal é `obr_esp32_main`; `obr_esp32_bridge` mantém uma alternativa de bancada com Wi-Fi e painel local.

## Hardware principal

| Componente | Função |
| --- | --- |
| Raspberry Pi | Estratégia, visão computacional e dashboard |
| ESP32 | Controle dos periféricos e proteção local da tração |
| Câmeras inferior e frontal | Percepção do percurso e da área de resgate |
| Quatro motores e drivers DRV8833 | Tração diferencial, agrupada por lado |
| Encoders esquerdo e direito | Medição de deslocamento e acompanhamento dos motores |
| MPU6050 | Orientação dos giros e inclinação nas rampas |
| Ultrassônico frontal | Distância para obstáculos e aproximações |
| PCA9685 e servos de braço, pulso e garra | Coleta, armazenamento e depósito |
| OLED SSD1306, botão e leitura de bateria | Estado local, partida/parada e diagnóstico |

As referências de hardware e controle da Raspberry ficam em [`include/obr/config.h`](include/obr/config.h). A configuração compartilhada pelos firmwares fica em [`esp32/obr_esp32_bridge/robot_config.h`](esp32/obr_esp32_bridge/robot_config.h). A ligação serial e o protocolo estão no [README do firmware principal](esp32/obr_esp32_main/README.md).

## Software e capacidades

O controle usa **C++17 e CMake**. A percepção usa **Python 3, Picamera2, OpenCV e NumPy**, com **ONNX Runtime** para o detector YOLO de vítimas e **LiteRT** para o classificador TFLite da faixa prata. O dashboard usa HTTP/WebSocket, e os processos do robô são supervisionados por **systemd** no Linux.

- **Seguimento de linha:** segmentação da faixa pela CAM0, orientação geométrica e cálculo das potências diferenciais. A CAM1 fornece assistência frontal.
- **Verdes:** associação das marcações à faixa, confirmação temporal e manobras à esquerda, à direita ou de retorno, com apoio da IMU.
- **Gaps e perda de linha:** validação da continuidade e uso da visão frontal para recuperação, com controle da validade das observações.
- **Obstáculos:** detecção ultrassônica, recuo, espera com contagem na OLED, contorno por IMU/encoders e reencontro da faixa. A configuração final usa o contorno pela esquerda; o módulo também contém seleção automática de lado.
- **Rampas:** ajuste da potência do seguidor conforme a inclinação informada pelo MPU6050.
- **Entrada no resgate:** reconhecimento da faixa prata e transição coordenada da navegação por linha para a missão da sala.
- **Vítimas:** detecção de bolas pratas e pretas pelo modelo ONNX, confirmação temporal, alinhamento, aproximação e coleta. A missão coordena armazenamento interno e depósito nas zonas verde e vermelha usando braço, pulso e garra.
- **Localização e saída:** reconhecimento visual das zonas, referências de yaw e deslocamentos por encoders. Após o resgate, uma sequência de travessia, reposicionamento junto à parede e busca visual devolve o controle à CAM0. Há uma rota específica para reentrada com o resgate já concluído.
- **Chegada:** detecção de vermelho e encerramento da missão com tração parada.
- **Telemetria e dashboard:** câmeras, fases da missão, comandos, sensores, bateria e métricas da Raspberry, além de modos isolados de diagnóstico.

## Segurança operacional

A tração inicia parada, as potências são limitadas e a parada de emergência tem prioridade sobre comandos manuais e autônomos. Comandos manuais vencem na Raspberry; a ESP32 zera os motores se deixar de receber comandos de motor por 500 ms. O encerramento normal solicita a parada dos motores.

As missões verificam a validade das entradas conforme cada etapa. Na saída normal do resgate, há movimentos limitados por tempo que podem prosseguir sem IMU, encoders ou câmera válidos; imagens antigas não fornecem correção e a busca sem confirmação termina parada. Essa particularidade está descrita na [sequência de saída](docs/RESCUE_EXIT_SEQUENCE.md).

Durante o resgate, `SERVO_HOLD` pode manter os pulsos dos servos mesmo em Stop, E-Stop ou perda de comunicação, para sustentar mecanismos sujeitos à gravidade. A parada da tração não implica desenergização do braço. Os detalhes constam no [protocolo da ESP32](esp32/obr_esp32_main/README.md).

## Organização do repositório

| Caminho | Conteúdo |
| --- | --- |
| `src/application/`, `src/robot/` | Aplicação, missões e controle |
| `src/hal/`, `src/telemetry/`, `src/dashboard/` | Hardware, comunicação, telemetria e painel |
| `include/obr/` | Interfaces e configuração C++ |
| `esp32/` | Firmware principal e alternativa de bancada |
| `scripts/vision/`, `scripts/ball_vision/` | Percepção da pista, zonas e vítimas |
| `assets/` | Recursos do painel e modelos de inferência |
| `scripts/`, `deployment/` | Execução, deploy e serviços |
| `tests/` | Testes C++ e Python |
| `docs/`, `calibration/`, `training/` | Documentação complementar, calibração e treinamento |

## Compilação e execução

Na Raspberry Pi com Linux, instale as ferramentas e bibliotecas de captura:

```sh
sudo apt install -y build-essential cmake python3-venv python3-pip python3-picamera2 python3-opencv python3-numpy
python3 -m venv --system-site-packages .venv
.venv/bin/python3 -m pip install onnxruntime==1.30.0
python3 -m venv --system-site-packages .venv-ai
.venv-ai/bin/python3 -m pip install ai-edge-litert
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure
```

O executável se chama `robot_test`, apesar de ser a aplicação principal. Os modelos usados estão em `assets/models/ball_detector.onnx` e `assets/models/silver_down.tflite`. A ESP32 deve estar gravada com o firmware principal, e a UART da Raspberry deve estar habilitada sem console serial. Consulte as dependências e a ligação no [firmware principal](esp32/obr_esp32_main/README.md) e no [guia de hardware](docs/GUIA_COMPLETO_DO_PROJETO.md).

Para uma execução manual na Raspberry, sem outra instância dos serviços, execute `bash scripts/run_robot.sh` e, em outro terminal, `bash scripts/run_line_camera.sh`. O painel fica em `http://obr.local:8080`; use os controles das câmeras para conferir a percepção antes de iniciar a missão. A operação física requer a montagem e as calibrações do robô.

Para testes Python sem câmeras, com NumPy e OpenCV disponíveis:

```sh
python -m unittest discover -s tests/python
```

No Windows, o C++ também pode ser compilado com MinGW usando `cmake -S . -B build -G "MinGW Makefiles" -DBUILD_TESTING=ON`, seguido dos comandos de build e CTest acima. Essa compilação não substitui a validação do hardware e dos serviços Linux. O resultado da revisão final está em [Validação da versão de competição](docs/COMPETITION_RELEASE_VALIDATION.md).

## Deploy

O deploy usa SSH por chave, copia o projeto, compila na Raspberry e reinicia os serviços. Os padrões são `raspberry@obr.local`, diretório `/home/raspberry/OBR2026K` e chave `~/.ssh/obr_raspberry`.

Na preparação inicial, use `deployment/install-service.ps1` no Windows ou `bash deployment/install-service.sh` no Linux/macOS para instalar os serviços `obr-robot` e `obr-line-camera`. Depois, execute na raiz do projeto:

```powershell
.\scripts\deploy.ps1
```

```sh
bash scripts/deploy.sh
```

O serviço `obr-robot` executa `scripts/run_robot.sh` como usuário `raspberry`; `obr-line-camera` gerencia a percepção inferior. Consulte os logs com `journalctl -u obr-robot -u obr-line-camera -f`.

## Documentação complementar

- [Firmware principal e protocolo UART](esp32/obr_esp32_main/README.md).
- [Desvio de obstáculos](docs/OBSTACLE_AVOIDANCE.md).
- [Rotina da sala de resgate](docs/ROTINA_SALA_RESGATE.md) e [sequência final de saída](docs/RESCUE_EXIT_SEQUENCE.md).
- [Visão de vítimas](scripts/ball_vision/README.md).
- [Gaps e validação frontal](docs/FORWARD_GAP_VALIDATION.md).
- [Detecção da chegada](docs/red-finish.md).
- [Calibração da linha](calibration/README.md) e [dataset e classificador da faixa prata](docs/SILVER_DATASET.md).
- [Guia de hardware, operação e diagnóstico](docs/GUIA_COMPLETO_DO_PROJETO.md).

Os documentos de calibração, diagnóstico e desenvolvimento preservam contexto de suas respectivas etapas e podem conter parâmetros históricos. Para o comportamento final, prevalecem o código e as constantes atuais; a saída normal é descrita em `docs/RESCUE_EXIT_SEQUENCE.md`.
