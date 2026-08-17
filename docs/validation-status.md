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
- O seguidor de linha perdeu estabilidade recentemente.
- Há indicação de queda aproximada de 30 FPS para 14–17 FPS.
- O boost ao sair do alinhamento/`TurningAhead` ainda precisa de validação.
- Pode ocorrer salto da contrarrotação para o avanço mínimo `0.65`.
- A câmera frontal ainda não possui pipeline de resgate validado.
- A visualização RGB e as máscaras do dashboard continuam em evolução.

## Validação obrigatória na Pi 5

Antes de qualquer teste no chão:

1. Manter o robô suspenso e o E-Stop disponível.
2. Confirmar visualmente que CAM0 mostra o chão e CAM1 mostra a frente.
3. Confirmar que somente CAM0 atualiza `obr_line_status.json`.
4. Verificar startup sem comandos de motor e sem loop de reinicialização.
5. Medir pelo menos 300 frames, incluindo FPS, frequência de `lineSequence`,
   idade do frame, CPU, RAM, temperatura e throttling.
