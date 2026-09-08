# Histórico dos experimentos — calibração visual real

Este arquivo preserva as conclusões intermediárias, inclusive candidatas
descartadas e tarefas que ainda estavam pendentes quando foram registradas.
As referências a parâmetros temporários e próximas coletas são históricas.
O resultado encerrado e a configuração aplicada estão em [RESULTS.md](RESULTS.md).

**A candidata atual é razão relativa de 61%, limiar mínimo de 40 em cinza de
8 bits e exclusão da cor verde antes do filtro de componentes preto, com AE/AWB originais. A configuração permanente
continua em 70%, sem essa exclusão; as alterações ao vivo são temporárias.
Ainda não há configuração final aprovada para uma run.**

Continuação: dobra forte, reta, curva suave, gap, interseção, curva de 90°,
marcação verde, piso sem fita, borda com fio e sombra adicional já foram avaliados.
A comparação retirando somente a sombra está aguardando o operador.
As capturas denominadas `validation` que passaram
a orientar ajustes são desenvolvimento; será necessária nova validação com
os parâmetros congelados.

O total atualizado de frames reais e as verificações de integridade estão em
[integrity.json](integrity.json). A tabela inclui reprocessamentos das mesmas
capturas, que não equivalem a aquisições independentes. Nenhuma imagem sintética entrou na
calibração. Os testes unitários usam dados sintéticos apenas para verificar as
ferramentas.

O [diagnóstico original](../README.md), a [tabela completa](comparison.md), os
[indicadores individuais](comparison.json) e a [comparação visual](comparison.png)
permitem revisar os resultados. As máscaras de cada experimento e os JSON por
frame permanecem em `calibration/experiments/`; os RGB ficam nos datasets originais.

## Observação → hipótese → teste → conclusão

1. **Piso sombreado aceito como preto.** A máscara original contém ilhas e uma
   região falsa ligada à fita. Hipótese: a razão de 70% é permissiva para o
   contraste produzido pelos LEDs. Foram testadas razões 55/60/65%, sem alterar
   exposição, morfologia, componentes ou trajetória. Em 55%, os falsos positivos
   anotados desapareceram nas capturas em iluminação normal. Em 60%, houve
   resultado semelhante sob luz normal, mas maior sensibilidade ao ensaio escuro.
2. **Limiar global mais simples.** Foram testados 60/80/100 em cinza de 8 bits,
   sobre os mesmos 60 RGB. Os limiares 60 e 80 perderam, respectivamente, 14,47%
   e 5,75% da fita anotada; o limiar 100 reteve falsos positivos. Não adotados.
   O método relativo já existente foi preservado; não se introduziu outro adaptive
   threshold.
3. **Automação causando oscilação.** Na primeira sequência, ExposureTime foi
   exatamente 19.243 µs e AnalogueGain 1,497076 em todas as amostras. ColourGains
   variou pouco. Na sequência contínua original seguinte, exposição foi 18.808 µs,
   também constante. A mudança entre sequências existe, mas não demonstrou
   oscilação de exposição dentro delas.
4. **Fixar exposição/ganho.** Solicitados 18.808 µs e 1,497076×, com AWB ativo;
   o sensor aplicou 18.790 µs. Com a máscara original, o jitter do centro foi
   0,3063 px, contra 0,1141 px na sequência automática de referência. Com razão
   55%, ambos ficaram próximos de 0,0035 px. Não demonstrou vantagem consistente.
5. **Fixar AWB.** Ganhos vermelho/azul solicitados 1,89/1,294 e confirmados nos
   metadados. Com razão 55%, jitter do centro 0,0021 px. A diferença em relação ao
   automático é subpixel e foi medida em uma única posição; insuficiente para
   recomendar o bloqueio do branco. A câmera afeta também os dados do verde,
   que ainda não foram validados nessa calibração.
6. **Reduzir exposição.** Solicitados 9.404 µs, aplicados 9.395 µs, ganho 1,497076×.
   A separação robusta entre fita e piso caiu de aproximadamente 37,4 para 24 níveis
   de cinza. Falsos positivos originais subiram de 3.260 para 13.768 px/frame.
   Em razão 55%, restaram 17,63 px/frame; em 60%, 2.039,90 px/frame. Hipótese
   descartada para esta iluminação. Isso não mede borramento com robô em movimento.
7. **Fixar todos os controles.** Exposição/ganho e branco foram fixados juntos
   após os testes isolados. A razão original ainda produziu 5.758,78 px falsos por
   frame e jitter de centro 3,4458 px. A razão 55% produziu zero pixels falsos
   anotados e jitter 0,0032 px. Fixar tudo não eliminou a causa da segmentação ruim.
8. **Frames novos após escolher a candidata.** A aquisição
   `validation_same_pose_01` usou 90 novos frames, sem reutilizar RGB do ajuste.
   O resultado abaixo é validação temporal na mesma posição física. Não substitui
   a validação de reta, gap, interseção ou outras curvas.

## Comparação quantitativa

Cada coluna usa o mesmo RGB para as duas segmentações. Os percentuais de fita
consideram apenas o interior anotado; a faixa incerta de 7 px nas bordas é excluída.

| Medida | Baseline original, 60 amostras | Candidata 55%, mesmas amostras | Original, 90 frames novos | Candidata 55%, mesmos frames novos |
|---|---:|---:|---:|---:|
| Componentes falsos/frame, média | 3,6167 | 0 | 3,8556 | 0 |
| Área falsa no fundo, px/frame | 4.619,33 | 0 | 4.714,42 | 0 |
| Fita anotada preservada | 100% | 100% | 100% | 100% |
| Cobertura das linhas horizontais esperadas | 100% | 100% | 100% | 100% |
| Jitter médio do centro, px | 1,0407 | 0,0046 | 0,0599 | 0,0014 |
| Jitter médio do alvo Fusion, px | ver JSON | ver JSON | 0 | 0,1907 |
| IoU temporal da máscara, frames novos | — | — | 0,996643 | 0,999980 |
| Score v2, frames novos | — | — | 86,61 | 97,54 |

**Trade-off real:** o alvo Fusion original ficou constante nos 90 frames novos.
Com 55%, houve jitter médio de 0,1907 px e P95 de 1,4142 px. O desvio padrão do
ângulo foi 0,0352°. Os alvos permaneceram sobre a fita anotada, porém a melhora
da máscara não demonstrou melhora da navegação. Esse ponto deve ser observado
nas próximas geometrias.

O número de componentes sozinho é insuficiente: em `original_live_01`, a contagem
falsa foi zero, mas havia 3.260 px falsos ligados ao componente verdadeiro. A medida
de fundo anotado detectou esse problema.

## Score e métricas

O score atual é **v3**. A margem quadrada de 7 px da anotação passou a valer
também para o alvo Fusion. O indicador estrito de alvo dentro do polígono continua
em cada `metrics.json`; não foi apagado. Na saída pela borda, o alvo pode cair
alguns pixels além do contorno traçado e ainda estar dentro dessa faixa incerta.
O score v2 zerava todo esse ensaio apesar de a trajetória terminar na saída
correta. A correção vale para todas as configurações, incluindo o baseline.
O snapshot anterior permanece em [comparison_v2.json](comparison_v2.json).
Os scores v2 das tabelas históricas abaixo mantêm seus rótulos; consulte a tabela
gerada para a comparação uniforme atual.

`score = 100 × mínimo(indicadores normalizados)`. Uma máscara vazia não ganha:
ela zera recall e continuidade quando há fita anotada. Não existem pesos ajustados
a estes dados. Cada contribuição está publicada em `comparison.json`.

Os indicadores incluem recall da fita, especificidade do fundo, cobertura das
linhas horizontais, IoU temporal, alvo sobre a fita, AUC do contraste, conectividade
no pior frame, fidelidade da largura, confiança temporal do próprio Fusion e
estabilidade de seu alvo. Nas regiões de sombra/reflexo, usa-se o pior frame de
recall/especificidade para não diluir a falha local pela área de piso fácil.

- AUC é a probabilidade empírica de a fita ser mais escura que o fundo; a margem
  `P05(piso) − P95(fita)` também é registrada para revelar perda de separação.
- Fidelidade de largura = `max(0, 1 − erro relativo médio)` contra a geometria
  desenhada no RGB. A largura horizontal de uma curva não é tratada como uma
  largura de fita constante. A incerteza da anotação limita essa medida.
- Estabilidade do alvo usa o P95 de seu deslocamento, normalizado pela tolerância
  já presente no Fusion: 0,12 da largura, ou 57,6 px nesta resolução. A medida em
  pixels permanece explícita; isso não é uma tolerância física validada em pista.
- Cada trecho conectado da anotação é avaliado separadamente. Assim, um gap real
  com dois trechos anotados não é confundido com fragmentação dentro de um trecho.
- Componentes isolados são relatados, mas não recebem penalidade duplicada por
  contagem: seus pixels falsos já afetam o indicador de fundo.
- O score v1 foi calculado durante a exploração e preservado nos ensaios. O v2
  acrescenta largura, contraste, regiões difíceis e métricas do Fusion. Os pesos
  não foram ajustados. A decisão precisa considerar os indicadores, não só a nota.

Nas primeiras poses, 55% e 60% ficaram praticamente empatados; diferenças de
centésimos de ponto no score não superam a incerteza da anotação. No ensaio de
exposição reduzida, 55% reteve menos falsos positivos. Isso justifica continuar
investigando 55% naquela etapa. Os casos posteriores de reflexo em 90° refutaram
a escolha de 55% como configuração geral, conforme os resultados abaixo.

## Continuação: reta e morfologia

Na reta centralizada, o baseline de 90 frames apresentou 10.253,29 px falsos por
frame, contra zero em 55% e 9,48 px em 60%. A fita anotada permaneceu preservada.
Nos 90 frames novos de validação da reta, a comparação foi:

| Medida | Original | Razão 55% |
|---|---:|---:|
| Falsos componentes/frame | 3,9889 | 0 |
| Pixels falsos/frame | 10.526,70 | 0 |
| Fita anotada preservada | 100% | 100% |
| Jitter do centro, px | 0,4262 | 0,0020 |
| Jitter do alvo Fusion, px | 0 | 0 |
| Erro relativo de largura | 22,33% | 2,35% |

Os [diagnósticos da reta original](straight_original.png) e
[da candidata](straight_candidate_55.png) mostram contornos anotados, pixels
falsos, scanlines, centros, larguras e o alvo Fusion. As scanlines de diagnóstico
não alteram os sensores virtuais ou o seguidor.

Foram feitos cinco reprocessamentos adicionais para investigar morfologia,
mantendo a razão 55%: abertura de referência 13 e 9, e fechamento 7. Na curva
com luz normal, o resultado foi praticamente equivalente ao candidato inicial.
No ensaio escuro, reduzir a abertura para 9 aumentou os falsos positivos de
17,63 para 71,63 px/frame; reduzir o fechamento levou a 17,40 px/frame, diferença
pequena demais para concluir melhora. Nenhuma dessas mudanças foi mantida.
Não se reutilizou a validação da curva para escolher a morfologia.

O teto de cinza 190 é redundante com razão 70%, cujo limiar máximo é
`floor(255 × 0,70) = 178`; variar o teto acima de 178 não muda a máscara original.

As duas razões foram comparadas também nas mesmas validações da curva e reta.
Com 60%, o jitter do alvo Fusion foi zero em ambas; com 55%, foi zero na reta e
0,1907 px na curva. Nenhuma produziu pixels falsos anotados nessas duas validações.
Portanto, 60% também segue candidata: sua maior sensibilidade ao ruído nos
baselines e no ensaio escuro precisa ser ponderada frente à estabilidade do alvo.
O score agregado ainda não resolve esse trade-off para as geometrias pendentes.

## Curva suave e gap: validação e aplicação ao vivo

Cada linha compara configurações sobre os mesmos 90 RGB da respectiva captura.
As anotações foram traçadas na imagem original antes de examinar as candidatas.

| Captura | Razão | Componentes falsos/frame | FP px/frame | Jitter centro px | Alvo Fusion sobre fita | Score v2 |
|---|---:|---:|---:|---:|---:|---:|
| Curva suave, validação | 70% | 3 | 7.449,40 | 0,1685 | 100% | 78,70 |
| Curva suave, validação | 60% | 0 | 0 | 0,0057 | 100% | 98,63 |
| Curva suave, validação | 55% | 0 | 0 | 0,0015 | 100% | 98,53 |
| Curva suave, teste ao vivo | 70%, replay | 2,6444 | 6.980,98 | 0,3301 | 100% | 78,31 |
| Curva suave, teste ao vivo | 55%, máscara real | 0 | 0 | 0,0006 | 100% | 98,52 |
| Gap, validação | 70% | 3 | 10.334,40 | 0,4993 | 0% | 0 |
| Gap, validação | 60% | 1 | 867,96 | 0,0033 | 100% | 97,86 |
| Gap, validação | 55% | 0 | 0 | 0,0037 | 100% | 99,59 |
| Gap, teste ao vivo | 70%, replay | 3 | 10.251,66 | 0,4237 | 0% | 0 |
| Gap, teste ao vivo | 55%, máscara real | 0 | 0 | 0,0036 | 100% | 99,57 |

Todas essas configurações preservaram 100% do interior anotado da fita e das
linhas horizontais esperadas. No gap, os dois componentes verdadeiros são
intencionalmente separados: não se premia fechar o espaço branco. Com 70%, uma
sombra ligada à ponta inferior desviou o Fusion para a lateral; em 55%, o alvo
ficou na ponta inferior verdadeira. Isso demonstra melhora da entrada e do alvo
normal nesse cenário, sem demonstrar a travessia autônoma do gap.

Em cada teste `captured_55`, o replay coincidiu pixel a pixel com as 90 máscaras
efetivamente usadas pelo aplicativo. `relative_70` é o contrafactual calculado
sobre esses mesmos RGB, não outra captura ao vivo. Os dois testes restauraram
70% ao terminar. Exemplos: [curva suave original](gentle_curve_original.png),
[curva suave com 55% ao vivo](gentle_curve_live_55.png),
[gap original](gap_original.png), [gap com 55% ao vivo](gap_live_55.png).

Nessa etapa, o score favoreceu 60% ligeiramente na curva suave pela largura anotada, enquanto
55% elimina o componente falso que permanece no gap em 60%. Essa é uma diferença
de comportamento relevante; a pequena diferença de largura não justifica ignorar
um componente falso em outro cenário. Os casos seguintes também mostraram perda
de fita com 55%; os indicadores individuais permanecem no JSON.

## Interseção e reflexo na curva de 90°: hipótese de 55% descartada

Na interseção, 55% perdeu 0,128% da fita anotada na primeira sequência; 60%
preservou toda a fita. Os intermediários 56/57/58% mostraram o compromisso entre
ruído no gap e preservação sob reflexo. Em 58%, a interseção ficou preservada,
mas o gap manteve cerca de 534 px falsos por frame.

A curva de 90° foi decisiva: 55% perdeu 3,00% da fita e 58% perdeu 2,26% na
primeira sequência. Em 60%, essa sequência ficou preservada, mas novas capturas
revelaram uma falha intermitente: recall mínimo de 98,24% e médio de 99,85%.
No teste de 60% ao vivo, o recall médio foi 99,79%; o alvo Fusion continuou
estável e correto. Isso não torna a falha visual aceitável por definição.

Uma busca limitada a 61/62% preservou toda a fita nos novos frames. 61% deixou
menos ruído no gap: 1.108,91 contra 1.517,57 px/frame em 62%. Assim, 61% passou
a ser a candidata para as próximas condições, mantendo os filtros originais.
As diferenças de um ponto percentual indicam margem pequena nesse reflexo;
não se deve declarar robustez de iluminação a partir de uma única posição.

Exemplos: [falha em 55%](turn90_rejected_55.png),
[teste ao vivo de 60%](turn90_live_60.png). As máscaras de 61% e os valores por
frame estão nos experimentos. Nenhum filtro de área ou largura foi aumentado
para esconder os componentes residuais; pontas curtas de fita podem depender deles.

## Marcação verde: correção na entrada preta, detector preservado

Com razão 61%, a marcação verde produziu aproximadamente 5.650 px falsos na
máscara preta. O detector HSV já conhecia essa região; o filtro de contornos
preto aceitava a marcação ligada à fita. Excluir a máscara HSV depois dos
componentes reduziu a área falsa para 147,19 px/frame, mas deixou pequenas ilhas.

Excluir a mesma máscara HSV antes de construir os componentes pretos reduziu
a área falsa e os componentes falsos anotados a zero, preservando 100% da fita.
Os limites HSV, filtros do verde, associação com preto e regras de manobra não
foram editados. A exclusão usa uma cópia da máscara estrutural, preservando a
original para a associação verde. Os contornos entregues ao Fusion são
recalculados a partir da máscara corrigida.

O teste ao vivo comparou 61% sem exclusão e 61% com exclusão, mantendo câmera e
demais filtros. Foram 90 frames de cada condição. Com exclusão, houve zero área
falsa anotada e recall 100%; o replay coincidiu pixel a pixel com a máscara real.
O detector manteve `DIREITA`, confirmado, preto válido, um candidato e zero
rejeitados em todos os 180 frames. Veja [registros](green_live_validation.json)
e [diagnóstico real](green_live_61_exclusion.png). Isso valida essa marcação
estática; não valida a execução da manobra.

Para medir contraste com piso branco, a anotação exclui o verde apenas da
distribuição de intensidade do piso. O verde permanece classe negativa para a
máscara preta e continua penalizado como falso positivo.

## Controle negativo: piso sem fita

No baseline, as sombras produziram 10.601,69 px falsos/frame e uma trajetória
falsa. Na sequência seguinte foram 9.278,32 px falsos/frame. A candidata 61%
com exclusão de verde produziu máscara vazia e ausência de trajetória nas duas
sequências e no teste ao vivo de 90 frames. Não há recall de fita nem contraste
fita/piso definido nesse cenário: esses indicadores são ausentes, não 100%.

## Borda e fio: falha da estimativa de fundo corrigida

O operador colocou uma curva à esquerda no canto inferior e um fio preto solto
no piso. A fita foi anotada separadamente do fio. O fio foi rejeitado pelos filtros
originais, mas a estimativa de fundo de 151 px falhou onde a fita ocupava o canto.
Com 70%, apenas 81,13% da fita anotada apareceu; com 61%, foram 58,56%.

Aumentar somente a janela de referência de 201 para 301/401 (225/301 px efetivos)
recuperou toda a fita, mas acrescentou 3.966/11.315 px falsos por frame. Reduzir a
razão com janela maior voltou a perder fita sob reflexo na curva de 90°. Essa
família de mudança foi descartada.

No primeiro RGB da borda, a fita anotada apresentou cinza de 12 a 45 e o piso,
excluindo o fio, mínimo 50. Testaram-se limites mínimos de limiar 40 e 45,
mantendo 61%, janela original e exclusão verde. Ambos recuperaram toda a fita,
sem pixels falsos anotados no baseline. **40 foi preferido por ser o menor valor
testado que recuperou a fita, com maior margem até o piso escuro.** Isso não
significa que todo pixel da fita esteja abaixo de 40; a regra relativa e o
preenchimento de contornos originais continuam operando.

A regra candidata é `cinza <= clip(floor(0,61 * fundo_local), 40, 190)`.
O limite mínimo impede que uma região de fita muito escura seja usada como seu
próprio fundo e depois rejeitada. Ele não amplia ROIs nem muda a trajetória.

| Captura da borda | Configuração | Fita preservada | FP px/frame | Jitter centro px | Jitter alvo Fusion px |
|---|---|---:|---:|---:|---:|
| Novos 90 frames | Original 70% | 79,97% | 4.990,58 | 1,8047 | 14,0425 |
| Mesmos frames | 61%, mínimo 40, exclusão verde | 100% | 3,10 | 0,0059 | 0 |
| Teste ao vivo, 90 frames | Original, replay | 81,75% | 4.646,52 | 1,4451 | 10,2573 |
| Mesmos RGB e máscara real | Candidata completa | 100% | 0 | 0,0073 | 0 |

O alvo da candidata ficou dentro da incerteza da anotação na saída lateral.
Seu indicador estrito foi zero, pois o alvo está poucos pixels fora do polígono;
essa distinção é explícita no score v3. Exemplos:
[falha da candidata anterior](border_failed_61.png),
[janela maior com mais sombra](border_background_301.png),
[candidata completa ao vivo](border_live_candidate.png).

Reprocessar as outras oito condições com o mínimo 40 preservou seus resultados
anteriores. Nenhum tamanho de abertura, fechamento ou componente foi alterado.

## Sombra externa adicional

No teste ao vivo, 70% em replay produziu 13.713,07 px falsos/frame; a candidata
completa produziu 2.510,77 px/frame, com recall 100% em ambos. O jitter do centro
caiu de 0,3071 para 0,0143 px. Ainda permaneceram três componentes de sombra;
não se afirma eliminação completa do ruído. A retirada da sombra na mesma pose
será usada para separar sensibilidade à iluminação de diferença de geometria.

## Interrupção por deploy externo

Em 08/09/2026, às 15:09:10 no horário local, um deploy de outro computador
reiniciou `obr-robot` e `obr-line-camera`. A integração `CalibrationCapture` foi
substituída por uma versão com coleta opcional de dataset de prata. A primeira
tentativa de capturar o gap expirou sem criar um dataset; o pedido pendente foi
arquivado e cancelado antes da reintegração. Nenhum frame dessa tentativa entrou
nas métricas.

Os 1.050 frames anteriores já estavam preservados localmente. A comparação do
conteúdo, normalizando finais de linha, confirmou câmera, perfil, segmentação,
sensores virtuais e Fusion iguais. Alguns módulos antigos adicionais estavam
presentes no deploy; presença adicional não significa mudança no caminho ativo.
As mudanças efetivas no aplicativo e publicador incluíam o gravador de prata.

Foi preservado um arquivo do código recebido em
`calibration/captures/deploy_interruption_source.tar.gz`, com inventário em
[deploy_interruption_inventory.json](deploy_interruption_inventory.json).
Apenas a integração de calibração foi reinserida no aplicativo remoto,
preservando as adições do colega. O arquivo resultante está em
`calibration/captures/post_deploy_application.py`; a aplicação local mantém o
patch de calibração sobre a branch local, sem importar as outras funcionalidades.
Um deploy geral a partir desta branch exige integrar previamente essas diferenças.

O E-Stop no Raspberry foi reativado e confirmado junto ao da ESP32; potências e
encoders ficaram zerados. A recuperação reiniciou somente `obr-line-camera` e
verificou que o PID de `obr-robot` permaneceu igual. A coleta opcional de prata
estava desativada. O gap foi então capturado novamente, com hashes dos fontes
e parâmetros salvos, e o replay das máscaras originais coincidiu exatamente.

## Recomendações atuais e parâmetros

**Câmera:** manter provisoriamente a configuração original (AE/AWB ligados,
EV +0,4, contraste 1,05, nitidez 1,2 e saturação 1,0). Os controles fixos foram
restaurados ao final de cada sequência. Os valores manuais testados estão nos
`config.json`; não são uma recomendação definitiva.

**Segmentação candidata para avaliação:** mudar somente
`line_max_background_ratio_percent` de 70 para 61 e habilitar `line_exclude_green`
para excluir o HSV verde antes de construir componentes pretos. Usar
`line_min_threshold = 40`. Manter fundo 201 de referência
(151 efetivo), máximo de cinza 190, abertura 17 de referência (13 efetivo),
fechamento 11 de referência (9 efetivo) e filtros de componentes originais.
As opções foram testadas separadamente e depois juntas no pipeline real. Os
parâmetros voltam automaticamente ao perfil anterior ao terminar cada captura.
O arquivo `scripts/vision/camera_config.py` permanece original; a opção de exclusão
é falsa por padrão quando ausente.

O sensor reportou suporte a ExposureTime, AnalogueGain, AE, AWB, ColourGains,
Brightness e Contrast. Brightness não é solicitado no pipeline original; o
default reportado pela API é 0, não uma leitura por frame. Não houve evidência
para investigar brilho, contraste ou morfologia antes dos próximos cenários.

## Código, execução e segurança

O caminho normal recebeu a integração em `vision/application.py` e uma opção
de máscara verde em `create_line_candidate_mask`, desligada por padrão.
`vision/calibration_capture.py` guarda no máximo 100 frames em memória e grava
PNG em outra thread. A captura usa o mesmo request para RGB, metadados e máscara
efetivamente usada, antes de qualquer desenho. Todos os 90 pares da primeira
sequência contínua coincidiram pixel a pixel com o replay, apesar das versões
diferentes de OpenCV nas máquinas.

Pedidos de controles temporários exigem telemetria recente de E-Stop no Raspberry
e na ESP32, potências aplicadas zero e encoders zerados. Perda dessa condição
aborta a captura e solicita os controles originais. A ferramenta não envia
comandos de movimento. Identificadores, quantidade de frames e limites dos
controles são validados. Erros e status ficam em
`/dev/shm/obr_line_calibration_result.json`; a conclusão também é preservada em
`result.json` junto da captura, pois `/dev/shm` é volátil.

Somente `obr-line-camera` foi reiniciado para carregar a instrumentação. O serviço
continua executando `run_line_camera.sh` como `raspberry`; logs:
`journalctl -u obr-line-camera`. Os arquivos systemd e o deploy geral não mudaram.
PID, motores, GPIO, lógica de estados, verde, gap e curvas não foram editados.

Para capturar no Raspberry, depois de posicionar manualmente o robô e confirmar
E-Stop no dashboard:

```sh
cd /home/raspberry/OBR2026K
python3 scripts/request_line_calibration.py baseline_straight_01 --scenario straight_center --frames 90
```

O resultado vai para `calibration/captures/baseline_straight_01`. Use um
identificador novo em cada captura. Para controles temporários, passe
`--controls-json caminho.json`; o JSON aceita apenas os cinco controles de
exposição/ganho/AE/AWB/ganhos de cor, e o serviço restaura os originais.
Para testar a candidata atual na máscara efetivamente usada e no stream,
adicione `--line-ratio 61 --exclude-green --min-threshold 40`. O ensaio de cor deve ser comparado a 61%
sem exclusão para isolar o efeito da cor. Não é permitido combinar essas opções com controles de
câmera na mesma captura. A referência compartilhada do perfil é restaurada em
conclusão, aborto e encerramento.

Para repetir a análise local, use um diretório de saída novo:

```powershell
python scripts/line_calibration.py calibration/baseline/current_pose_01 --annotations calibration/baseline/current_pose_01/annotations.json --output calibration/experiments/review_55 --ratio 55
python scripts/compare_line_calibration.py calibration/experiments --regions calibration/scoring_regions.json --output calibration/report
python scripts/test_line_calibration.py
python -m unittest discover -s tests/python -p test_camera_profiles.py
python -m unittest discover -s tests/python -p test_camera_line_virtual_turn.py
```

Verificação: 12 testes novos, 55 testes de perfis e 184 de seguimento passaram.
Foram verificadas captura real, cadência, controles aplicados, restauração e
igualdade de máscaras. Comentários novos estão em português brasileiro, com
acentos revisados e explicação de intenção/segurança.

A análise recusa RGB cujo SHA-256 não coincide com o manifesto e recusa o baseline
se o replay não reproduzir a máscara salva. O relatório também agrupa as mesmas
configurações de câmera/segmentação e apresenta o pior cenário, sem favorecer
uma condição apenas porque ela teve mais capturas.

## O que falta para concluir

Foram avaliadas dez condições físicas: dobra forte, reta centralizada, curva
suave, gap, interseção, curva de 90°, marcação verde, piso vazio, borda com fio
e reta com sombra externa. Também há reflexos dos LEDs e partículas naturais.
A candidata foi congelada em [candidate_v1.json](../candidate_v1.json).
Faltam a retirada da sombra na mesma pose e novas capturas de validação com
esses parâmetros inalterados, sem usar o resultado para novos ajustes.

Em cada nova posição: capturar primeiro os parâmetros originais; anotar o RGB
independentemente da máscara; comparar original e candidata congelada; preservar gap, detalhes finos
e saídas laterais; capturar novos frames para validação depois da seleção. Não
aplicar uma candidata que só ganhe no score agregado e perca fita ou trajetória
em um cenário crítico.

As primeiras regiões de sombra/reflexo eram subdivisões de imagens reais.
O ensaio de sombra externa foi preparado fisicamente pelo operador; sua
intensidade não foi medida por instrumento. Não houve ensaio com movimento
para verificar borramento, vibração ou mudanças rápidas de iluminação. A análise
de outras marcações verdes e a passagem dinâmica pelo gap permanecem pendentes. A segmentação das
duas pontas do gap foi avaliada; a máquina de estados do gap não foi executada.

Os RGB não são Bayer. A primeira coleta tem timestamps de publicação e amostragem
espaçada; as seguintes preservam SensorTimestamp e FrameDuration. As capturas
apresentam dependência temporal: 90 frames não equivalem a 90 condições independentes.
Não se afirma significância estatística entre sequências únicas.

Os dados volumosos estão ignorados pelo Git, mas preservados no computador e no
Raspberry. O relatório, as anotações e as ferramentas são versionáveis. Não houve
commit automático, nem alteração definitiva da calibração do robô.
