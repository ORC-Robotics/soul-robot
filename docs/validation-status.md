# Status de validação do robô

**STATUS: EXPERIMENTAL / NÃO VALIDADO**

Este checkpoint preserva o estado atual do algoritmo para instalação e ensaios
sem movimento na Raspberry Pi 5. Ele não representa uma versão estável para
competição e não autoriza testes autônomos no chão.

## Mapeamento físico da Raspberry Pi 5

- CAM0 / índice `0`: câmera inferior (`down`), responsável pelo segue-faixa,
  detecção de verde e publicação de `obr_line_status.json`.
- CAM1 / índice `1`: câmera frontal (`forward`), reservada para visão de resgate.
  Enquanto não houver pipeline e stream frontal validados, o dashboard deve
  manter o placeholder `NÃO CONFIGURADA`.

Os papéis são definidos pelos índices configurados, não pelo modelo do sensor.
Uma câmera ausente não pode fazer a outra assumir seu papel automaticamente.

## Problemas conhecidos ainda abertos

- Falsos candidatos verdes podem aparecer no chão branco.
- O verde real ainda pode aparecer como ambíguo em situações físicas.
- O HSV precisa ser calibrado usando frames RGB reais da pista.
- Pode ocorrer salto da contrarrotação para o avanço mínimo `0.69`.
- A câmera frontal ainda não possui pipeline de resgate validado.
- A visualização RGB e as máscaras do dashboard continuam em evolução.

## Resultados da validação de 17 de agosto de 2026

- A validação histórica usou `base_speed_preview=0.66` e piso `0.65`. Esses
  valores foram substituídos porque um dos motores não gira nessa potência. O
  perfil atual usa base `0.70` e piso `0.69`, produzindo diferencial entre
  `0.69` e `0.71`; esta nova calibração ainda exige teste físico suspenso.
- A captura da câmera inferior voltou a operar próxima de 30 FPS. O
  [status registrado](../artifacts/pi5-camera-validation-20260817/camera-status.json)
  mostra 30,07 FPS, e o
  [traço de 300 amostras](../artifacts/pi5-camera-validation-20260817/line-regression-trace-300.csv)
  registra a frequência e o tempo de processamento de cada quadro.
- CAM0 e CAM1 foram registradas separadamente nas evidências
  [cam0.jpg](../artifacts/pi5-camera-validation-20260817/cam0.jpg) e
  [cam1.jpg](../artifacts/pi5-camera-validation-20260817/cam1.jpg).
- A proposta de aplicar um boost ao sair de `TurningNear` foi descartada. Ela
  adicionaria outra transição de potência sem resolver a causa da instabilidade.

## Validação ainda obrigatória antes de novos testes no chão

1. Manter o robô suspenso e o E-Stop disponível no primeiro ensaio após deploy.
2. Confirmar startup com os motores parados e sem loop de reinicialização do
   serviço `obr-robot`.
3. Confirmar que somente CAM0 atualiza `obr_line_status.json` durante a execução.
4. Calibrar o HSV com imagens reais da pista e repetir os casos de verde.
5. Validar a saída de `TurningNear` e observar se ainda ocorre salto da
   contrarrotação para o avanço mínimo `0.69`.
