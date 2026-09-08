# Resultado final — calibração da câmera inferior

**Configuração aplicada no computador e no robô: razão relativa 61%, limiar
mínimo 40 e exclusão do verde antes dos filtros de componentes pretos.**
AE, ganho automático, AWB e os filtros morfológicos originais foram mantidos.
A coleta foi encerrada a pedido do operador, sem repetir o ciclo de cenários.

Foram preservados **3.120 frames reais em 35 datasets** e comparados **158
reprocessamentos**. Dez condições físicas orientaram os ajustes; outros 180
frames, adquiridos após congelar os parâmetros, avaliaram a retirada da sombra
na mesma reta. Esses reprocessamentos não são 158 capturas independentes.
Não foi realizada uma run com movimento.

## Configuração recomendada e instalada

| Parâmetro | Original | Final |
|---|---|---|
| Câmera / imagem processada | IMX219 / 480×360 / 30 FPS | Mantido |
| Sensor / orientação | 1640×1232, 10 bits / 180° | Mantido |
| AE / AnalogueGain / AWB | Automáticos | Mantidos automáticos |
| EV / contraste / nitidez / saturação | +0,4 / 1,05 / 1,2 / 1,0 | Mantidos |
| Brightness | Não solicitado; default da API 0 | Mantido |
| Razão relativa ao fundo | 70% | **61%** |
| Limite inferior do threshold, cinza de 8 bits | 0, desativado | **40** |
| Exclusão HSV verde antes dos componentes pretos | Desativada | **Ativada** |
| Limite superior do threshold | 190 | Mantido |
| Fundo / abertura / fechamento efetivos | 151×151 / elipse 13×13 / retângulo 9×9 | Mantidos |
| ROIs, área, espessura e interior dos componentes | Originais | Mantidos |

Os valores permanentes estão em
[`scripts/vision/camera_config.py`](../../scripts/vision/camera_config.py),
no perfil `CAMERA_PROFILES["down"]["vision"]`. Não estão em `include/obr/config.h`.
O manifesto [candidate_v1.json](../candidate_v1.json) preserva a configuração
congelada antes da validação final. Sua igualdade com o perfil instalado foi
verificada, assim como os hashes dos arquivos.

Pipeline final: BGR → cinza → fechamento grande estima o fundo → binarização
`gray <= clip(floor(0.61 * background), 40, 190)` → abertura/fechamento originais
→ recorte estrutural → retirar o HSV verde de uma cópia → filtros originais de
componentes → máscara e contornos aceitos para o Fusion/sensores virtuais.
O detector verde continua recebendo a máscara estrutural, antes dessa exclusão.
Seu HSV, associação, confirmação e comportamento de manobra não foram editados.

## Diagnóstico original

O [diagnóstico anterior às alterações](../README.md) registra inicialização,
resolução, controles, ROIs, filtros, stream e formas originais de captura.
Os problemas observados foram:

- O limiar de 70% aceitava sombras, tanto isoladas quanto ligadas à fita.
  Contar apenas componentes desconectados escondia parte desse erro.
- Verde escuro entrava na máscara preta e podia desviar a trajetória escolhida.
- Fita escura ocupando a borda contaminava a estimativa do próprio fundo;
  reduzir apenas a razão para 61% agravava a perda nesse caso.
- Um limiar muito restritivo, como 55%, apagava detalhes reais sob reflexos
  dos LEDs na curva de 90°.
- Fixar exposição e branco não eliminou esses problemas. A exposição foi
  constante dentro das primeiras sequências automáticas; não apareceu evidência
  de que sua oscilação fosse a principal causa do ruído.

## Experimentos e decisões

Os experimentos mudaram uma família por vez. O histórico e as configurações
intermediárias estão em [EXPERIMENT_LOG.md](EXPERIMENT_LOG.md) e na
[tabela completa](comparison.md), com [indicadores individuais](comparison.json).

| Hipótese / alteração | Observação quantitativa | Conclusão |
|---|---|---|
| Reduzir razão relativa, explorando 55–65% e refinando a faixa | 55% limpou as primeiras poses, mas perdeu cerca de 3% da fita sob reflexo no 90°. Em 60%, novos frames chegaram a recall mínimo 98,24%. 61% preservou a fita anotada e aceitou menos ruído que 62%. | Adotar 61%. |
| Fixar exposição e ganho: 18.808 µs solicitados, 18.790 µs aplicados, ganho 1,497076× | Jitter do centro com máscara original: 0,3063 px, contra 0,1141 px no automático. Com razão 55%, ambos perto de 0,0035 px. | Sem vantagem consistente; manter automático. |
| Fixar AWB em ganhos 1,89 / 1,294 | Jitter 0,0021 px com razão 55%, em uma única pose. | Diferença subpixel insuficiente para justificar bloquear AWB. |
| Reduzir exposição à metade | Separação robusta fita/piso caiu de 37,4 para 24 níveis de cinza; FP da máscara original subiu de 3.260 para 13.768 px/frame. | Descartar nesta iluminação estática. |
| Fixar exposição, ganho e branco juntos, após testes separados | Ainda houve 5.758,78 px falsos/frame com razão 70%; a mudança de segmentação resolveu muito mais que o bloqueio da câmera. | Não manter controles fixos. |
| Limiar global 60/80/100 | 60 e 80 perderam 14,47% e 5,75% da fita; 100 ainda reteve falsos positivos. | Manter o método relativo existente; não adicionar outro adaptive threshold. |
| Reduzir abertura ou fechamento | Sem ganho consistente. No ensaio escuro, abertura menor elevou FP de 17,63 para 71,63 px/frame. | Preservar morfologia. |
| Excluir verde da máscara preta | Após os componentes ainda sobravam cerca de 147 px e fragmentos; excluir antes dos filtros produziu zero FP no cenário anotado e preservou a fita. | Excluir antes dos componentes, preservando o detector verde. |
| Aumentar janela de fundo para recuperar fita na borda | Referências 301 e 401 recuperaram fita, mas geraram cerca de 3.966 e 11.315 px falsos/frame. | Descartar. |
| Impor limiar mínimo 40 ou 45 | Ambos recuperaram a borda; 40 também preservou as demais condições e é menos permissivo ao ruído. | Adotar 40. |

No ensaio controlado de verde, 90 frames com exclusão e 90 sem exclusão
mantiveram `DIREITA`, confirmação verdadeira, associação preta válida e um
candidato verde, sem rejeições. Evidência: [green_live_validation.json](green_live_validation.json).
Isso verifica essa marcação estática, não todas as manobras verdes possíveis.

## Comparação por cenário

Cada linha compara as duas configurações **nos mesmos 90 RGB**. Valores são
médias por frame. FP inclui erro ligado à linha e componentes isolados. Recall
refere-se ao interior anotado da fita, excluindo a margem incerta de 7 px.
Os dados desta tabela participaram do desenvolvimento, mesmo quando seus
identificadores históricos contêm `validation`.

| Cenário / dataset | FP original → final, px/frame | Componentes falsos original → final | Fita original → final | Jitter do centro original → final, px |
|---|---:|---:|---:|---:|
| Dobra forte / `validation_same_pose_01` | 4.714,42 → 6,58 | 3,856 → 0,022 | 100% → 100% | 0,0599 → 0,0063 |
| Reta central / `validation_straight_01` | 10.526,70 → 502,04 | 3,989 → 0,933 | 100% → 100% | 0,4262 → 0,0042 |
| Curva suave / `validation_gentle_curve_01` | 7.449,40 → 301,32 | 3,000 → 0,944 | 100% → 100% | 0,1685 → 0,0050 |
| Gap / `validation_gap_01` | 10.334,40 → 1.158,12 | 3,000 → 1,000 | 100% → 100% | 0,4993 → 0,0133 |
| Interseção / `validation_intersection_01` | 1.347,02 → 0 | 1,000 → 0 | 100% → 100% | 0,0085 → 0,0106 |
| Curva 90° / `validation_turn90_01` | 3.569,77 → 0 | 2,756 → 0 | 100% → 100% | 1,3126 → 0,0002 |
| Perto do verde / `validation_green_01` | 5.739,13 → 0 | 0 → 0 | 100% → 100% | 0,0301 → 0,0362 |
| Piso sem fita / `validation_white_floor_01` | 9.278,32 → 0 | 2,644 → 0 | Não se aplica | Não se aplica |
| Borda com fio / `validation_border_wire_01` | 4.990,58 → 3,10 | 2,000 → 0,011 | 79,97% → 100% | 1,8047 → 0,0059 |
| Sombra adicional / `baseline_external_shadow_01` | 13.404,93 → 2.372,14 | 3,933 → 3,000 | 100% → 100% | 0,3665 → 0,0026 |

Os números completos, largura, continuidade, área, IoU, confiança e trajetória
estão em [candidate_development_comparison.json](candidate_development_comparison.json).
Na borda, a cobertura das linhas horizontais passou de 89,91% para 100%; o
jitter médio do alvo Fusion passou de 14,04 px para zero. No gap, o alvo deixou
a sombra e passou à fita, sem unir artificialmente as duas pontas do gap.
No piso vazio, desapareceu a falsa trajetória. O verde ligado à linha mostra
por que zero componentes isolados não significa ausência de falsos positivos.

Nem todo indicador melhorou: o jitter subpixel do centro aumentou ligeiramente
na interseção e perto do verde. Isso está preservado na comparação; a decisão
considerou a forte redução de FP e a conservação da geometria e da trajetória.

## Validação após congelar a configuração

Foram capturados **180 novos frames**, em duas sequências de 90, após o operador
retirar a sombra mantendo a mesma reta. Não foram usados para novos ajustes.
Os perfis, horários, fontes, segurança e reprodução das máscaras efetivamente
capturadas passaram nas verificações de [holdout.json](holdout.json).
O operador dispensou repetir outras curvas, gap e interseção nessa fase final.

| Indicador na sequência final ao vivo | Original, replay dos mesmos RGB | Final, máscara capturada |
|---|---:|---:|
| Componentes falsos/frame | 3,00 | 0,90 |
| Pixels falsos/frame | 8.591,84 | 322,69 |
| Fita anotada preservada, inclusive pior frame | 100% | 100% |
| Cobertura das linhas horizontais | 100% | 100% |
| Jitter do centro | 0,2410 px | 0,0100 px |
| Jitter da largura | 0,3164 px | 0,0199 px |
| Erro relativo de largura | 22,02% | 3,12% |
| IoU temporal | 99,3746% | 99,7460% |
| Jitter do alvo Fusion | 0 px | 0 px |
| Score v3 | 77,98 | 96,88 |

Nessa sequência, FP caiu **96,24%** e jitter do centro **95,87%**. Na outra
sequência final, FP passou de 9.054,74 para 580,77 px/frame e jitter de 0,3110
para 0,0050 px, também com 100% da fita preservada. A diferença entre sequências
é mantida no relatório, sem escolher apenas o melhor momento.

## Sombra, reflexo e score

Na captura ao vivo com sombra adicional, comparar sobre os mesmos RGB reduziu
FP de **13.713,07 para 2.510,77 px/frame** e jitter do centro de **0,3071 para
0,0143 px**. Restaram três componentes falsos por frame. A fita ficou preservada
e o jitter do alvo Fusion passou de 0,4256 px para zero.

Comparando a pose com e sem a sombra, o excesso médio de FP associado à condição
sombreada foi 5.121,22 px na configuração original e 2.188,08 px na final,
redução descritiva de 57,27% desse excesso. As sequências foram adquiridas em
momentos diferentes com automações ativas; não há medição de lux nem inferência
estatística causal. As regiões de reflexo anotadas preservaram a fita com 61%;
55% e 60% falharam em detalhes reais do 90°.

O **score v3 é `100 × mínimo(indicadores normalizados)`**; a comparação entre
cenários usa novamente o pior caso. Não há pesos ajustados para favorecer uma
configuração, nem uma média que compense perda de fita por fundo limpo. Recall
e continuidade por frame são verificados também fora da nota. Os componentes
isolados são contados e medidos em área, mas não penalizados novamente além de
seus pixels falsos. Gaps anotados têm trechos verdadeiros separados.

O pior score da configuração final nos dados disponíveis foi **96,13**.
Seus componentes nesse ensaio (`baseline_external_shadow_01`) foram:

| Componente | Valor normalizado |
|---|---:|
| Recall / cobertura horizontal / conectividade no pior frame | 1,0000 / 1,0000 / 1,0000 |
| Especificidade do fundo | 0,9763 |
| IoU temporal | 0,9972 |
| Alvo dentro da anotação com sua incerteza | 1,0000 |
| AUC do contraste fita/piso | 0,99994 |
| Fidelidade da largura | 0,9690 |
| Confiança temporal / estabilidade do alvo Fusion | 1,0000 / 1,0000 |
| Recall / especificidade na região de sombra, pior frame | 1,0000 / 0,9750 |
| Recall / especificidade na região de reflexo, pior frame | 1,0000 / 1,0000 |
| Especificidade na região anotada com partículas, pior frame | **0,9613** |

A última região contém também sombra; seu nome histórico `natural_particles`
não prova que todo erro ali seja causado por partículas. É a contribuição que
limita a nota. Fidelidade de largura é `max(0, 1 − erro relativo à geometria
anotada)`. AUC mede a probabilidade de a fita ser mais escura que o piso; também
registramos `P05(piso) − P95(fita)`. Comparações de segmentação sobre o mesmo RGB
têm exatamente o mesmo contraste original.

A estabilidade do alvo usa P95 de deslocamento dividido pela tolerância existente
do Fusion, 0,12 da largura (57,6 px), não uma tolerância nova inventada para os
experimentos. Todos os valores em pixels continuam publicados.

No score v3, a margem quadrada de 7 px das anotações vale também para o alvo
Fusion. Na saída lateral, ele pode cair nessa faixa incerta e fora do polígono
estrito. A versão v2 zerava esse ensaio por esse motivo. A correção foi aplicada
a todas as configurações; o indicador estrito e [comparison_v2.json](comparison_v2.json)
foram preservados. A nota é exploratória e não representa probabilidade de
completar uma run.

## Exemplos reais antes/depois

Os diagnósticos mostram RGB, máscara pura, fita aceita, FP, componentes rejeitados,
scanlines, largura, centro e alvo Fusion. Cada par abaixo usa o mesmo RGB.

| Condição | Original | Final |
|---|---|---|
| Reta final sem sombra adicional | [Antes](final_straight_before.png) | [Depois](final_straight_after.png) |
| Reta com sombra adicional | [Antes](final_shadow_before.png) | [Depois](final_shadow_after.png) |

Também estão disponíveis a [curva 90° final](final_turn90_after.png),
a [borda recuperada](border_live_candidate.png) e o
[verde excluído da máscara preta](green_live_61_exclusion.png).
Os RGB sem desenhos e máscaras correspondentes ficam em `captures/*/raw` e
`captures/*/mask`; os reprocessamentos ficam em `experiments/*/*/mask`.

## Instrumentação, arquivos e operação

Foram alterados apenas `vision/camera_config.py`, `vision/line_masks.py` e
`vision/application.py`, acrescentando parâmetros, exclusão sobre cópia e captura
pareada antes dos desenhos. As ferramentas adicionadas são:

- `vision/calibration_capture.py` e `request_line_calibration.py`: pedidos de
  30–100 frames, RGB/máscara/metadados do mesmo request, parâmetros e fontes
  ativos, telemetria de segurança, restauração dos parâmetros ao finalizar.
- `line_calibration.py` e `compare_line_calibration.py`: análise offline,
  métricas por frame, scores individuais e comparação pelo pior cenário.
- `render_line_calibration.py`: imagens de diagnóstico a partir dos dados reais.
- `audit_line_calibration.py` e `report_line_calibration_holdout.py`: hashes,
  pares, reprodução da máscara, separação temporal e verificação da validação.
- `test_line_calibration.py`: testes de captura, segurança, restauração e métricas.

Não foram adicionadas dependências. PNGs são gravados fora do loop visual;
o armazenamento em memória é limitado a 100 frames. A auditoria reproduziu
**todos os 3.120 pares**, pixel a pixel, usando os parâmetros salvos. Os primeiros
60 têm máscaras reconstruídas pelo código original; os 3.060 seguintes têm a
própria máscara efetivamente usada, capturada junto do RGB. Esse detalhe está
explícito no baseline, sem tratar uma reconstrução como captura original.

Exemplo de uso futuro no Raspberry, sem iniciar movimento (não foi executado
como nova coleta no encerramento):

```sh
cd /home/raspberry/OBR2026K
python3 scripts/request_line_calibration.py new_final_01 --scenario straight_center --frames 90
# Reproduz temporariamente todos os parâmetros originais da segmentação.
python3 scripts/request_line_calibration.py new_original_01 --scenario straight_center --frames 90 --line-ratio 70 --min-threshold 0 --no-exclude-green
```

Cada identificador precisa ser novo. Controles temporários de câmera podem ser
passados por `--controls-json`; não podem ser combinados com ajustes de
segmentação na mesma captura. Os pedidos exigem E-Stop confirmado nas duas
placas, telemetria recente, potências e encoders zero. Perder essas condições
aborta a captura. A ferramenta não envia comandos de movimento.

O deploy externo do colega interrompeu uma tentativa anterior, que expirou e
não entrou no conjunto de dados. Os fontes anteriores e posteriores foram
preservados e comparados. Na instalação final, foi alterado apenas o perfil da
câmera; as adições do colega na aplicação remota foram preservadas.
**A aplicação remota contém adições que não estão na branch local; integre essa
divergência antes de um deploy geral.** Os detalhes e cópias estão na seção
correspondente do [histórico](EXPERIMENT_LOG.md) e em
[deploy_interruption_inventory.json](deploy_interruption_inventory.json).

Foi reiniciado somente `obr-line-camera`. `obr-robot` manteve PID 153890;
ambas as emergências permaneceram ativas, com potências e encoders zero.
A câmera estabilizou em aproximadamente 30 FPS (status 30,3 FPS). O perfil
anterior permanece no Raspberry em `calibration/camera_config_before_final.py`.
[final_status.json](final_status.json) registra fontes, parâmetros e segurança.
PID de controle, motores, GPIO, estados, verde, gap e curvas não foram editados.
Não houve commit automático nem deploy geral.

Verificações após alterar os valores permanentes: **12 testes de calibração,
55 de perfis e 184 de seguimento passaram**; 251 no total. Comandos:

```powershell
python scripts/test_line_calibration.py
python -m unittest discover -s tests/python -p test_camera_profiles.py
python -m unittest discover -s tests/python -p test_camera_line_virtual_turn.py
python scripts/audit_line_calibration.py
python scripts/report_line_calibration_holdout.py
python scripts/compare_line_calibration.py calibration/experiments --regions calibration/scoring_regions.json --output calibration/report
git diff --check
```

## Limitações remanescentes

Ainda existem manchas sob sombra forte e algum ruído no gap/reta. A tentativa
de eliminar tudo com limiar mais restritivo prejudicou fita real sob reflexo;
a configuração final prioriza preservação e estabilidade da trajetória.
Isso é uma limitação explícita, não uma máscara visualmente perfeita.

A validação após congelamento abrangeu uma condição de iluminação em uma reta,
sem repetir as demais geometrias, conforme decisão do operador. Os cenários de
desenvolvimento foram variados, mas não substituem uma validação independente
completa. Movimento, vibração, borramento, outras marcações verdes e execução
das máquinas de estados de gap/curvas não foram avaliados nesta calibração visual.

As anotações manuais têm margem incerta de 7 px e excluem o chassi. Os frames
consecutivos não são condições independentes. A câmera entrega RGB processado,
não Bayer; somente os primeiros 60 usam timestamps de publicação e amostragem
espaçada. Os seguintes preservam SensorTimestamp, FrameDuration e metadados.
Os dados volumosos estão ignorados pelo Git, mas preservados no computador e
no Raspberry; relatórios, anotações e ferramentas são versionáveis.
