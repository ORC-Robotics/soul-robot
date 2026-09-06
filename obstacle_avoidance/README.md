# Módulo de desvio de obstáculo

Esta pasta contém todo o código específico, as calibrações e os testes do
desvio de obstáculo usado no OBR2026K.

## Conteúdo

- `obstacle_avoidance.h`: interface pública e máquina de estados.
- `obstacle_avoidance.cpp`: detecção, giros e deslocamentos.
- `config.h`: aliases internos para os limites centralizados em `include/obr/config.h`.
- `obstacle_avoidance_test.cpp`: regressões da sequência completa e das falhas.

## Sequência atual

1. Confirma duas leituras ultrassônicas de até 8 cm.
2. Gira 45° à direita.
3. Avança 25 cm.
4. Gira 45° à esquerda.
5. Avança 30 cm.
6. Gira 90° à esquerda.
7. Avança 21,5 cm.
8. Gira 90° à direita.
9. Recua 5 cm e devolve a autoridade ao segue-linha.

Depois de iniciado, linha, verde e câmera não alteram a sequência. E-Stop,
comando Parar, perda da ESP32, IMU ou encoders e timeouts permanecem acima do
módulo e devem zerar os motores.

## Dependências ao exportar

O módulo usa duas interfaces existentes do projeto:

- `Esp32TelemetrySnapshot`, de `obr/esp32_bridge.h`;
- `ImuTurnController`, de `obr/imu_turn_controller.h` e seu arquivo `.cpp`.

No projeto de destino, preserve esses nomes ou crie um adaptador com os campos
de ultrassônico, idade da telemetria, contagens/taxas dos encoders, yaw, giroscópio
e estado do MPU6050.

Adicione `obstacle_avoidance.cpp` ao build, inclua a raiz que contém esta pasta e
chame `ObstacleAvoidance::update()` em cada ciclo. Quando `hasControl` for
verdadeiro, aplique `leftPower` e `rightPower`. Quando `failed` for verdadeiro,
pare o robô. Quando `completed` for verdadeiro, devolva o controle ao seguidor.

O exemplo completo de integração está em `MainMission` deste repositório. Os
nomes de debug exibidos no painel estão mapeados em `DashboardServer`.
