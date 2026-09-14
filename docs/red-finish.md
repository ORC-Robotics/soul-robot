# Chegada vermelha pela CAM0

A câmera inferior mede vermelho na união das áreas reais dos sensores FAR,
FAR BAND, MEDIUM (incluindo extensões) e NEAR. A área auxiliar `near.position`
não entra no cálculo. Sobreposições contam uma vez; os espaços entre sensores
não entram no denominador.

## Calibração

- `include/obr/config.h`: `kRedFinishMinRatio = 0.10` (10%) e
  `kRedFinishConfirmFrames = 4` (presença e ausência consecutivas).
- `scripts/vision/camera_config.py`: H entre 0–10 ou 170–179, S mínimo 100 e
  V mínimo 70, na escala HSV do OpenCV.
- O IPC de controle existente leva os parâmetros C++ à CAM0 mesmo com a busca
  da saída desativada. Sem configuração recente, aparece `UNAVAILABLE` e a visão
  não confirma nem rearma chegada. A tolerância entre frames reutiliza
  `kCameraLineStatusTimeoutMs` (125 ms).

O vídeo mostra `RED: X%` e destaca os pixels vermelhos válidos. Após quatro
frames acima do limiar, mostra também `RED CONFIRMED`. A análise usa a imagem
original; os desenhos não participam da detecção. A confirmação visual pode
continuar aparecendo enquanto o robô sai da mesma faixa após um novo START.

## Parada e nova execução

A chegada é válida em qualquer modo. Ela zera os motores, cancela as solicitações
de servos e marca `missionFinished`, mantendo o modo `stopped`. A proteção
existente da ESP32 para sustentação dos servos permanece em vigor. O OLED mostra
`Vermelho` / `CHEGADA` com prioridade sobre os alertas de navegação.

START físico, início manual/autônomo e seleção de missão/modo limpam a conclusão.
Stop, timeout e reconexão não retomam a execução. Um novo START permite sair da
mesma faixa: o detector só rearma após quatro frames válidos abaixo de 10%.
A emergência mantém a prioridade e o botão físico não a libera.

A telemetria inclui `missionFinished`, `redRatio` (0 a 1) e `redValid`.
Se `redValid` for falso, a proporção anterior não é uma observação atual.

## Validação rápida

```powershell
cmake --build build --target robot_test main_mission_test oled_event_notifier_test -j 4
.\build\main_mission_test.exe --red-only
ctest --test-dir build -R "^(main_mission_test|oled_event_notifier_test)$" --output-on-failure
python -m unittest discover -s tests/python -p test_red_detection.py
```

1. Sem motores conectados, comparar `RED: X%` em preto, branco, verde, possíveis
   falsos positivos e na faixa vermelha real. Vermelho fora dos sensores não conta.
2. Com rodas suspensas e potência reduzida, apresentar vermelho: confirmar STOP,
   `missionFinished` e OLED. Aguardar e verificar que nenhuma rotina retoma.
3. Acionar START ainda sobre vermelho: permitir sair; retirar a faixa por pelo
   menos quatro frames e apresentá-la novamente para verificar outra parada.
4. Repetir a liberação pelo painel e conferir E-Stop antes de testar no chão.

Os testes locais não substituem a calibração de iluminação e a medição da parada
no hardware. Não é necessário reiniciar o serviço para liberar uma chegada.

## Resultado da validação local

- Compilação de `robot_test`, `main_mission_test` e `oled_event_notifier_test`: passou.
- Dois testes novos essenciais (Python e C++ com `--red-only`): passaram.
- Teste existente do OLED, oito testes Python de saída e teste da telemetria
  de confiança FAR/MEDIUM: passaram.
- A execução completa de `main_mission_test` parou na asserção de centralização
  do desvio com câmera indisponível, depois de passar pelo teste de vermelho.
- Os testes existentes de geometria verde esperam uma região superior diferente
  da configuração atual. O teste de chaves exatas do IPC não espera os campos
  `silverTimestamp`, `silverClassifierAvailable`, `silverSequence` e
  `exitLineUnbranched`, já presentes no trabalho local anterior ao vermelho.
  Esses comportamentos não foram alterados nesta implementação.
- Hardware, distância física de parada e legibilidade do OLED ainda precisam
  da verificação em bancada descrita acima.

Safety check (revisão de código; hardware não verificado):
- Motors stop on emergency stop: yes.
- Motors stop on command timeout: yes.
- Motor output clamped: yes.
- Pins centralized in config.h: yes; nenhum pino foi adicionado.

Comment quality check:
- Comments are in Brazilian Portuguese: yes.
- Spelling and accents were reviewed: yes.
- Comments explain purpose, effect, or safety risk: yes.
- Comments avoid obvious noise: yes.
