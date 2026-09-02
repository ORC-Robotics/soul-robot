# Remoção de código morto e legado

Data da auditoria: 17 de agosto de 2026.

Este documento registra a remoção de arquivos que permaneciam no repositório,
mas não participavam do executável, do serviço da Raspberry Pi, do firmware da
ESP32 nem das ferramentas de teste atuais.

## Critérios usados

Um arquivo foi removido somente quando todos estes pontos foram confirmados:

1. Não estava listado no `CMakeLists.txt` do executável `robot_test`.
2. Não era iniciado por `scripts/run_robot.sh` ou pelo serviço `obr-robot`.
3. Não era incluído ou instanciado pelos módulos compilados atualmente.
4. Não era importado pelos scripts Python executados no robô.
5. Existia uma implementação atual com responsabilidade equivalente, ou o
   hardware correspondente já não fazia parte do robô.
6. O build e os testes continuavam funcionando sem o arquivo.

Não foi usado apenas o nome ou a idade do arquivo como critério. Ferramentas
manuais e módulos usados somente por testes foram mantidos.

## BNO055 removido

Arquivos removidos:

- `include/obr/bno055.h`
- `src/hal/bno055.cpp`

O BNO055 não está instalado no robô atual. A orientação usada pela Raspberry
chega pela telemetria UART da ESP32, que lê o MPU6050. Nenhuma instância de
`Bno055Sensor` era criada, o arquivo não aparecia no `CMakeLists.txt` e suas
constantes antigas já não existiam na configuração ativa.

Manter apenas o `.cpp` depois da remoção do header também deixaria um arquivo
que não poderia ser compilado isoladamente. Por isso, o par completo foi
removido.

Substituição atual:

- Leitura física do MPU6050: `esp32/obr_esp32_bridge/obr_esp32_bridge.ino`.
- Transporte da telemetria: `src/hal/esp32_bridge.cpp`.
- Consumo dos dados: missões, dashboard e telemetria da Raspberry Pi.

## Arquitetura antiga duplicada em `src/`

Arquivos removidos:

- `src/config.h`
- `src/dashboard_server.cpp`
- `src/dashboard_server.h`
- `src/gpio.cpp`
- `src/gpio.h`
- `src/motor_controller.cpp`
- `src/motor_controller.h`
- `src/robot_state.cpp`
- `src/robot_state.h`
- `src/telemetry.cpp`
- `src/telemetry.h`

Esses arquivos formavam um protótipo anterior concentrado na raiz de `src/`.
Ele controlava uma ponte L298N diretamente pelos GPIOs da Raspberry e possuía
versões próprias de dashboard, estado e telemetria. O robô atual não usa essa
ligação: os comandos de motor são validados pela Raspberry e enviados pela UART
para a ESP32, que controla os drivers e mantém seu próprio timeout de segurança.

As classes antigas tinham os mesmos nomes das classes atuais. Isso dificultava
buscas, navegação no editor e manutenção, além de permitir que alguém alterasse
por engano um arquivo que nunca entraria no binário.

Substituições atuais:

| Responsabilidade | Implementação atual |
| --- | --- |
| Configuração da Raspberry | `include/obr/config.h` |
| Dashboard HTTP e WebSocket | `include/obr/dashboard_server.h` e `src/dashboard/dashboard_server.cpp` |
| GPIO da Raspberry | `include/obr/gpio.h` e `src/hal/gpio.cpp` |
| Controle e envio dos motores | `include/obr/motor_controller.h` e `src/robot/motor_controller.cpp` |
| Estado do robô | `include/obr/robot_state.h` e `src/robot/robot_state.cpp` |
| Telemetria da Raspberry | `include/obr/telemetry.h` e `src/telemetry/telemetry.cpp` |

## Seguidores de linha antigos

Arquivos removidos:

- `include/obr/line_follower.h`
- `src/robot/line_follower.cpp`
- `include/obr/behaviors/line_follower.h`
- `src/robot/behaviors/line_follower.cpp`

Existiam duas classes diferentes chamadas `LineFollower`. Uma implementava uma
máquina de estados antiga lendo o status da câmera; a outra calculava comandos a
partir de `CameraSnapshot`. Nenhuma das duas aparecia no `CMakeLists.txt` e cada
header era incluído somente pelo seu próprio `.cpp`.

O comportamento realmente executado está em `MainMission`. Ele recebe
`CameraLineSnapshot`, usa sequências novas da câmera, valida a idade das fontes e
para o robô quando câmera ou ESP32 ficam indisponíveis.

Substituição atual:

- Interface: `include/obr/main_mission.h`.
- Implementação: `src/robot/main_mission.cpp`.
- Seleção e ciclo da missão: `include/obr/mission_controller.h` e
  `src/robot/mission_controller.cpp`.

## Ferramentas antigas removidas no reset do segue-faixa

Uma auditoria posterior removeu também estas implementações e diagnósticos do
controlador normal, depois que todos os consumidores foram eliminados:

- `scripts/analyze_line_regression_trace.py`;
- `scripts/simple_line_vision.py` e seu teste;
- `scripts/vision_path.py` e seu teste;
- `include/obr/line_regression_trace.h` e
  `src/diagnostics/line_regression_trace.cpp`.

As evidências históricas continuam disponíveis em `artifacts/`, mas não descrevem
o ponto de extensão atual, que publica potência zero.

## Arquivos avaliados e mantidos

Os itens abaixo não entram diretamente no executável principal, mas possuem uma
função válida e, portanto, não foram classificados como código morto:

- `camera_fov_test/direct_picamera2_mode_test.py`: diagnóstico direto de modos
  da câmera.
- `deployment/deploy_panel.py`: interface local opcional para o deploy documentado.
- `deployment/systemd/obr-robot.service.d/camera-role.conf.example`: exemplo de override do
  serviço.
- `esp32/obr_esp32_main` e `esp32/obr_esp32_bridge`: o firmware principal inclui
  a implementação compartilhada do bridge com o dashboard Wi-Fi desabilitado.

As constantes de pinagem da ESP32 mantidas em `include/obr/config.h` também não
foram removidas nesta limpeza. Elas servem como mapa da integração, embora os
valores efetivamente compilados pela ESP32 estejam em
`esp32/obr_esp32_bridge/robot_config.h`. Uma futura consolidação deve evitar que
essas duas representações divirjam.

## Efeito esperado

A remoção não altera o comportamento do robô porque nenhum dos arquivos estava
no grafo de build ou no fluxo do serviço. O objetivo é:

- impedir manutenção acidental de implementações inativas;
- deixar clara a arquitetura realmente executada;
- evitar conflitos entre classes com o mesmo nome;
- impedir que builds futuros por curingas tentem compilar o BNO055 incompleto;
- reduzir o volume de código que a equipe precisa compreender durante a prova.

O controle de motores, E-Stop, timeout de comandos, comunicação UART, câmeras e
missões atuais não foi removido nem substituído nesta limpeza.

## Recuperação pelo histórico

Os arquivos continuam disponíveis no histórico do Git. Antes do commit desta
limpeza, eles podem ser consultados no checkpoint `19beff9`:

```sh
git show 19beff9:src/hal/bno055.cpp
git show 19beff9:src/robot/line_follower.cpp
git show 19beff9:src/dashboard_server.cpp
```

Se alguma implementação precisar ser estudada no futuro, prefira consultar o
histórico em vez de recolocar código inativo no caminho principal do projeto.

## Limite do deploy incremental

Os scripts de deploy atuais copiam `src/` e `include/` com `scp -r`, que não
apaga arquivos ausentes na origem. Por isso, uma Raspberry Pi que já recebeu
uma versão anterior pode conservar cópias órfãs dos arquivos removidos. Essas
cópias não entram no executável, pois o `CMakeLists.txt` lista explicitamente
somente as implementações atuais.

Esta auditoria não executou uma remoção destrutiva no diretório remoto. Se a
equipe quiser que a árvore de fontes da Raspberry também fique fisicamente
limpa, deve fazer uma limpeza pontual dos caminhos documentados acima ou adotar
posteriormente uma sincronização com exclusões controladas.

## Validação após a remoção

Em 17 de agosto de 2026, foram executadas estas verificações:

- build CMake totalmente novo em diretório temporário, com Ninja e MinGW
  15.2.0: `robot_test` compilado e vinculado, 13 de 13 etapas concluídas;
- `python -m unittest discover -s tests/vision -p "test_*.py"`: testes de visão
  concluídos, com resultado `OK`; dez testes foram ignorados porque o OpenCV
  não está instalado no ambiente Windows usado na auditoria;
- `git diff --check`: nenhuma falha de espaços ou formatação no diff;
- busca pós-remoção: os nomes BNO055 e `LineFollower` aparecem somente neste
  documento histórico.

O teste físico, o build nativo na Raspberry Pi e o reinício do serviço não
foram executados nesta auditoria local. Para repetir as verificações de software:

```sh
cmake -S . -B build
cmake --build build
python3 -m unittest discover -s tests/vision -p "test_*.py"
```

Na Raspberry Pi, valide o fluxo completo com:

```powershell
.\deployment\deploy.ps1 -Service
```
