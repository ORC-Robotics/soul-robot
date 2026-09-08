# Como retomar a calibração, se necessário

A calibração está encerrada por enquanto. Este roteiro registra oportunidades
futuras; não solicita novas coletas nem indica alterações a aplicar agora.
Podemos trabalhar em outro algoritmo mantendo o perfil visual atual.

## Ponto de partida

- Referência de código: commit `81a2aca`, incorporado à `main`.
- Perfil inferior: razão **61%**, limiar mínimo **40**, exclusão do verde
  **ativada**, teto 190; AE/ganho/AWB automáticos e morfologia original.
- Evidência: 3.120 frames reais, 35 datasets e 158 reprocessamentos.
- Configuração congelada: [candidate_v1.json](candidate_v1.json).
  Resultados e limites: [report/RESULTS.md](report/RESULTS.md).
- Validação posterior ao congelamento: 180 frames da mesma reta sem sombra
  adicional. Não houve teste com movimento nem repetição independente de todas
  as geometrias nessa etapa final.

Numa retomada, comparar principalmente com **a configuração calibrada de 61%**,
para provar melhora sobre o que já funciona. O original de 70% continua como
referência histórica. Preservar datasets, anotações, manifesto e relatórios
desta sessão; novos experimentos precisam de outros identificadores.

## Onde ainda pode melhorar

As sugestões são hipóteses, não melhorias comprovadas. Começar pelo problema
que aparecer no uso real, sem reiniciar automaticamente toda a coleta.

| Sinal para retomar | Evidência / onde observar | Próximo experimento útil | Critério para manter a mudança |
|---|---|---|---|
| Sombras viram manchas relevantes | Com sombra adicional restaram 3 componentes e 2.510,77 px falsos/frame; ver `live_external_shadow_candidate_01` e [diagnóstico](report/final_shadow_after.png). O alvo Fusion ficou estável nessa pose. | Separar nas anotações sombra, sujeira e possível obstrução. Inspecionar cinza e estimativa de fundo na região falsa. Só depois comparar uma alteração pequena na estimação do fundo ou no filtro de componentes, primeiro offline. | Reduzir FP local sem apagar fita no 90°, na borda ou nas pontas do gap; preservar alvo e confiança. |
| Ruído residual na reta ou gap interfere na trajetória | Na reta final: 322,69 px falsos/frame. No gap de desenvolvimento: 1.158,12 px, mantendo ambas as pontas. | Comparar área, espessura e distância à fita dos componentes falsos e verdadeiros. Testar filtro adicional somente se houver separação útil entre eles. | Rejeitar ruído sem rejeitar ponta legítima, ramo de interseção ou saída lateral. Uma máscara mais limpa não basta. |
| Reflexo apaga fita real | 55% perdeu cerca de 3% da fita no 90°; 60% teve recall mínimo 98,24%. O perfil final preservou os trechos anotados. | Se houver falha nova, verificar se a informação ainda existe no RGB ou se há saturação. Se existir, investigar correção local limitada. A função `repair_small_specular_holes` existe, mas não está ativa nem validada como solução. | Preservar gaps reais e não transformar verde ou piso em preto. Não habilitar reparo de reflexos sem comparação real. |
| Fita desaparece na extremidade do FOV | Na borda anotada, original 79,97% de fita preservada e final 100%. O mínimo 40 resolveu a contaminação da estimativa de fundo pela própria fita. | Se reaparecer com outra posição ou fita, medir cinza e separação do piso nessa região. Investigar o limite mínimo ou a estimativa de fundo isoladamente. | Preservar a borda sem aceitar fio/sujeira. Aumentar o mínimo sem medir a margem pode admitir ruído escuro. |
| Segmentação oscila quando muda a iluminação | As primeiras sequências automáticas tiveram exposição constante. Não demonstraram vantagem consistente de exposição/ganho/WB fixos; transições rápidas não foram avaliadas. | Com robô parado, observar uma transição de iluminação e relacionar ExposureTime, AnalogueGain, DigitalGain e ColourGains com centro, largura, área e máscara. Comparar automação e bloqueio se aparecer correlação com o erro. | Melhorar estabilidade sem perder contraste no escuro, saturar a fita ou prejudicar a entrada do detector verde. |
| Linha fica borrada somente em movimento | Movimento, vibração e borramento não foram medidos. Exposição pela metade piorou o contraste no ensaio estático. | Tratar como avaliação própria, com movimento conhecido, captura apropriada e procedimento operacional seguro. A ferramenta atual exige imobilidade e não atende esse ensaio. | Melhorar os frames em movimento e conservar desempenho estático. Não retirar o `StationaryGuard` nem alterar PID/motores para esconder defeito visual. |
| Exclusão do verde prejudica fita em outra marcação | Uma marcação direita passou em 90 frames com e 90 sem exclusão, mantendo interpretação e confirmação. Outras marcações não foram cobertas. | Na marcação que falhar, comparar HSV, preto estrutural e preto entregue ao Fusion sobre os mesmos RGB. Separar erro de cor de erro de associação. | Preservar fita e interpretação verde. Não alterar HSV ou manobras automaticamente ao ajustar a máscara preta. |

O maior resíduo observado foi sombra, mas isso não obriga nova calibração se
o algoritmo real continuar confiável. Pequenos fragmentos podem permanecer;
importam sua influência na trajetória e a preservação da linha verdadeira.

## Tentativas descartadas: não repetir sem evidência nova

- Reduzir globalmente para 55% só para remover bolinhas: já apagou fita sob reflexo.
- Aumentar indiscriminadamente a janela de fundo: referências 301/401 recuperaram
  a borda, mas geraram aproximadamente 3.966/11.315 px falsos/frame.
- Alterar morfologia apenas pela aparência: não houve ganho consistente; uma
  abertura menor piorou o ruído no ensaio escuro. Gaps e interseções exigem cuidado.
- Fixar controles apenas porque há LEDs: os testes não demonstraram vantagem
  consistente. É preciso uma falha nova que justifique a hipótese.
- Introduzir outro adaptive threshold ou reparo de reflexos sem provar melhora
  nos mesmos RGB e preservar casos difíceis já conhecidos.

## Onde trabalhar no código

| Responsabilidade | Arquivo |
|---|---|
| Perfil permanente, razão, mínimo, exclusão, kernels e câmera | [`camera_config.py`](../scripts/vision/camera_config.py), `CAMERA_PROFILES["down"]` |
| Inicialização e controles pedidos à câmera | [`camera.py`](../scripts/vision/camera.py) |
| Fundo, limiar, morfologia e componentes | [`line_masks.py`](../scripts/vision/line_masks.py) |
| Ordem de processamento, integração verde e máscara entregue ao algoritmo | [`application.py`](../scripts/vision/application.py) |
| Captura pareada, metadados e exigência de imobilidade | [`calibration_capture.py`](../scripts/vision/calibration_capture.py) e [`request_line_calibration.py`](../scripts/request_line_calibration.py) |
| Reprocessamento e métricas por frame | [`line_calibration.py`](../scripts/line_calibration.py) |
| Score, pior cenário e regiões difíceis | [`compare_line_calibration.py`](../scripts/compare_line_calibration.py) e [scoring_regions.json](scoring_regions.json) |
| Anotações e diagnósticos | [annotations](annotations), [`render_line_calibration.py`](../scripts/render_line_calibration.py) e [report](report) |

Os parâmetros desta calibração não ficam em `include/obr/config.h`.
Não esconder defeitos da máscara alterando tolerâncias do Fusion, scanlines,
PID, estados, verde, gap ou curvas. Se o próximo trabalho mudar o algoritmo
de trajetória, avaliar seus resultados separadamente dos ajustes da câmera.

## Sequência mínima de retomada

1. Identificar um erro concreto no RGB, máscara e trajetória; conferir código e
   parâmetros realmente instalados. O status salvo desta sessão é histórico.
   Comparar adições remotas antes de deploy geral: houve divergência nesta sessão.
2. Usar primeiro os frames existentes do cenário correspondente. Capturar novos
   30–100 frames somente se os antigos não representarem a falha. Para ensaios
   estáticos, manter robô parado e RGB/máscara/metadados pareados; anotar no RGB.
3. Formular uma hipótese e mudar uma família. Experimentos de componentes devem
   começar offline antes de entrar no pipeline ao vivo.
4. Comparar com 61% nos mesmos RGB: recall mínimo, conectividade por trecho, FP
   ligado e isolado, largura, jitter, IoU e alvo/confiança do Fusion. Verificar
   as regiões difíceis; uma nota agregada maior não justifica perder fita.
5. Reprocessar os casos que protegem contra regressão: 90° com reflexo, gap,
   borda com fio, interseção, verde e piso vazio. Isso reaproveita dados existentes
   e não exige montar de novo todos os cenários.
6. Se houver ganho mensurável, congelar outra candidata e validar com frames
   novos focados na falha e no risco do ajuste. Registrar o alcance da validação;
   replay não é aquisição independente. Descartar hipóteses que falharem.

Captura estática futura no Raspberry, com LEDs ligados e E-Stop confirmado nas
duas placas, usando um identificador ainda inexistente:

```sh
cd /home/raspberry/OBR2026K
python3 scripts/request_line_calibration.py resume_shadow_reference_01 --scenario external_shadow --frames 90 --line-ratio 61 --min-threshold 40 --exclude-green
```

O cenário não cria anotações automaticamente. Se a pose mudou, desenhar novos
polígonos no RGB. Para exposição/ganho/WB, usar `--controls-json` sem opções de
segmentação no mesmo pedido. Brilho e contraste não fazem parte da lista atual
de controles temporários da ferramenta.

Exemplo de replay local da referência preservada, na raiz do repositório e com
saída nova:

```powershell
python scripts/line_calibration.py calibration/captures/live_external_shadow_candidate_01 --annotations calibration/annotations/external_shadow_01.json --output calibration/experiments/resume_review/reference_61 --ratio 61 --min-threshold 40 --exclude-green
python scripts/render_line_calibration.py calibration/experiments/resume_review/reference_61 --output calibration/experiments/resume_review/reference_61/diagnostic_review.png
```

A alternativa deve usar o mesmo dataset/anotação e outro diretório de saída.
Ao consolidar uma sessão futura, apontar `--output` do comparador para um novo
diretório, preservando os relatórios históricos.

## Ao voltar de outro algoritmo

Os dados completos estão em `calibration/captures`, `calibration/experiments`
e `calibration/baseline`; as pastas volumosas são ignoradas pelo Git. Um clone
novo não contém esses frames. Eles foram preservados neste computador e no
Raspberry; copiar antes de limpar qualquer workspace.

Score e replay de trajetória também dependem do Fusion, sensores virtuais e
constantes. Se outro trabalho mudar esses módulos, não atribuir a diferença
automaticamente à segmentação. Registrar versões e comparar máscaras com o
mesmo algoritmo de trajetória, ou separar explicitamente os experimentos.
Uma falha de reprodução após mudanças futuras exige investigar a versão;
não substituir as máscaras antigas para fazer a auditoria passar.

Manter explícita a margem manual de 7 px. Alvo estrito e alvo dentro da margem
são indicadores distintos; não alargar a margem para favorecer uma candidata.
A região histórica `natural_particles` também contém sombra: separar essas
anotações numa nova investigação para atribuir corretamente o erro.

O verificador `report_line_calibration_holdout.py` está vinculado à candidata v1
e aos seus perfis/fontes de referência. Para outra candidata, criar novo manifesto
e adaptar a verificação explicitamente, preservando a validação v1.
Nenhum ensaio ou alteração de parâmetros foi executado ao criar este roteiro.
