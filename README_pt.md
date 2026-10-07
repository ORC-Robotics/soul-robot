# Soul

[English](README.md) | Português

<!-- Foto do robô: adicione docs/images/soul.jpg e descomente.
<p align="center">
  <img src="docs/images/soul.jpg" alt="Soul, robô autônomo de resgate" width="600"/>
</p>
-->

## Sobre o Soul

O Soul é um robô autônomo de resgate projetado e construído pela ORC Robotics. Sozinho, ele segue a linha da pista, interpreta as marcações de curva e passa por gaps, rampas e obstáculos. Na área de resgate, encontra as vítimas, recolhe cada uma, deposita na zona certa e segue até a saída.

### Compacto e completo

Apesar do tamanho compacto, o Soul reúne um sistema robótico completo. Uma **Raspberry Pi** executa a estratégia das missões em C++ e a visão computacional em Python, enquanto uma **ESP32** aciona motores e servos e devolve as leituras dos sensores pela UART. Duas câmeras, encoders, uma IMU e um sensor ultrassônico permitem ler a pista e o ambiente, e um painel web mostra tudo em tempo real.

## Principais recursos

### Navegação
- **Seguimento de linha** — segmentação da pista pela câmera inferior, com apoio da câmera frontal
- **Marcações de curva** — verdes associados à linha e confirmados ao longo do tempo, com curvas guiadas pela IMU
- **Gaps e recuperação da linha** — validação da continuidade e reaquisição pela câmera frontal
- **Obstáculos** — detecção ultrassônica e contorno guiado por IMU e encoders
- **Rampas** — velocidade ajustada à inclinação medida pela IMU

### Resgate
- **Detecção de vítimas** — modelo YOLO no ONNX Runtime para encontrar bolas pratas e pretas
- **Coleta e depósito** — braço, pulso e garra para armazenar as vítimas e depositá-las nas zonas verde e vermelha
- **Entrada e saída do resgate** — classificador da faixa prata (TFLite) na entrada, reconhecimento visual das zonas e odometria na saída

### Operação e segurança
- **Painel em tempo real** — câmeras, fases da missão, comandos e telemetria via HTTP/WebSocket
- **Tração segura** — a parada de emergência tem prioridade máxima, e a ESP32 para os motores se os comandos deixarem de chegar por 500 ms

## Hardware

- **Computador principal**: Raspberry Pi
- **Microcontrolador**: ESP32, ligada por UART a 115200 bps
- **Visão**: câmeras inferior e frontal
- **Tração**: quatro motores DC com drivers DRV8833 e encoders
- **Sensores**: IMU MPU6050 e ultrassônico frontal
- **Manipulador**: servos de braço, pulso e garra em um PCA9685
- **Interface**: OLED SSD1306, botão de partida e leitura de bateria

## Software

- **Controle**: C++17 e CMake
- **Visão**: Python 3, Picamera2, OpenCV e NumPy
- **Inferência**: ONNX Runtime (YOLO) e LiteRT (TFLite)
- **Firmware**: ESP32 (Arduino)
- **Execução**: serviços systemd no Linux, com deploy via SSH

## Estrutura do projeto

```
soul-robot/
├── src/             # Aplicação, missões, controle, camada de hardware, telemetria e painel
├── include/obr/     # Interfaces e configuração C++
├── esp32/           # Firmware da ESP32 (principal e ponte de bancada)
├── scripts/         # Processos de visão, execução e deploy
├── assets/          # Recursos do painel e modelos de inferência
├── deployment/      # Instalação dos serviços systemd
├── calibration/     # Dados e relatórios de calibração da linha
├── training/        # Treinamento do classificador da faixa prata
├── tests/           # Testes C++ e Python
└── docs/            # Documentação técnica
```

## Primeiros passos

Na Raspberry Pi:

```sh
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure
```

Dependências, ligações, calibração e deploy estão na [visão técnica](docs/VISAO_TECNICA.md) e no [guia do firmware da ESP32](esp32/obr_esp32_main/README.md).

## Documentação

- [Visão técnica](docs/VISAO_TECNICA.md) — arquitetura, segurança, compilação e deploy
- [Guia de hardware e operação](docs/GUIA_COMPLETO_DO_PROJETO.md)
- [Firmware da ESP32 e protocolo UART](esp32/obr_esp32_main/README.md)
- [Rotina da sala de resgate](docs/ROTINA_SALA_RESGATE.md) e [sequência de saída](docs/RESCUE_EXIT_SEQUENCE.md)
- [Desvio de obstáculos](docs/OBSTACLE_AVOIDANCE.md)
- [Visão de vítimas](scripts/ball_vision/README.md)

---

Desenvolvido pela [ORC Robotics](https://github.com/ORC-Robotics) no SENAI ORC, Brasil.
