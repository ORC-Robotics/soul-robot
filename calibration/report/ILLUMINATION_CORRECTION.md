# Compensação das sombras fixas da câmera inferior

A segmentação preta agora pode corrigir o padrão fixo de iluminação do
robô antes do threshold relativo. A correção atua somente na imagem cinza
interna usada para a linha. RGB, verde, prata, GAP, Fusion, sensores virtuais,
PID e steering não foram alterados.

## Referência e algoritmo

O artefato
[`down_line_illumination_480x360.png`](../../scripts/vision/calibration/down_line_illumination_480x360.png)
foi gerado a partir dos 90 PNGs lossless de
`calibration/captures/validation_white_floor_01`. A captura comprova E-Stop nas
duas placas, potências e encoders zerados, IMX219, 480×360 e controles
automáticos. A máscara antiga não participou da construção.

O gerador calcula a mediana temporal, aplica `GaussianBlur` com sigma 15 px,
usa o percentil 85 da região útil como alvo e limita o ganho entre 1× e 2×.
O [manifesto](../../scripts/vision/calibration/down_line_illumination_480x360.json)
preserva os hashes dos 90 frames, configuração, segurança e parâmetros do
algoritmo. A reprodução gerou novamente o mesmo SHA-256
`21db98d4f1af631b1648fcf0dbabdb5b4663f2671878dfc7a7b48dd2d46e1608`.

O cinza corrigido segue pelo pipeline existente 61%/40/190 e pela mesma
morfologia. Pixels originalmente `<= 40` continuam elegíveis, mesmo depois da
compensação. Isso preserva a função do limiar mínimo quando a fita real ocupa
a borda e contamina a estimativa do fundo local.

## Resultado do replay

Foram comparados 10 cenários, com 90 frames cada, sobre os mesmos RGB. O
[relatório completo](illumination_validation.json) contém os indicadores por
cenário.

| Cenário | FP atual → corrigido, px/frame | Recall mínimo | Continuidade mínima |
|---|---:|---:|---:|
| Reta | 502,04 → 0 | 100% | 100% |
| Curva suave | 301,32 → 0 | 100% | 100% |
| GAP | 1.158,12 → 0 | 100% | 100% |
| Interseção | 0 → 0 | 100% | 100% |
| Curva de 90° | 0 → 0 | 100% | 100% |
| Verde | 0 → 0,07 | 100% | 100% |
| Piso branco | 0 → 0 | N/A | N/A |
| Borda com fio | 3,10 → 0 | 100% | 100% |
| Sombra adicional | 2.372,14 → 0 | 100% | 100% |
| Holdout sem sombra adicional | 322,69 → 0 | 100% | 100% |

Nos 900 frames, os falsos positivos anotados caíram de 419.348 para 6
pixels. A fita anotada atravessou a zona compensada nos dois lados em nove
datasets; o recall do pior frame foi 100% à esquerda e à direita. O piso
branco permaneceu sem trajetória aceita em todos os frames.

O benchmark local mediu acréscimo médio de 0,067 ms no pipeline completo e
aproximadamente 0,040 ms na multiplicação isolada. Depois de substituir a
composição inicial por operações OpenCV mascaradas, o overlay mediu 0,205 ms
em RGB. Depois de acrescentar a comparação vermelha completa, o overlay binário
mediu 0,472 ms; a segunda segmentação usada apenas nesse diagnóstico acrescentou
2,60 ms no computador de desenvolvimento. Esse custo não existe no stream RGB
normal.

Na Raspberry, a mesma multiplicação OpenCV de um frame 480×360 pelo mapa teve
média de 0,187 ms, P95 de 0,293 ms, P99 de 0,398 ms e máximo de 0,516 ms em
3.000 iterações. O ensaio usou arquivos temporários em `/tmp`, sem acessar a
câmera, motores, GPIO ou serviços. O FPS completo ao vivo continua pendente,
pois a alteração não foi instalada no robô nesta etapa.

## Overlay e operação

No stream, ciano delimita a região cujo ganho é pelo menos 1,10, acrescida de
8 px de margem. Em RGB, verde mostra candidatos pretos preservados dentro dessa
região. Na visualização binária, branco continua sendo exatamente a máscara
entregue ao Fusion; seu contorno fica verde dentro da zona compensada, e
vermelho mostra os candidatos que a pipeline sem compensação entregaria e a
nova rejeitou. Essa comparação executa a segunda morfologia somente no modo
binário de diagnóstico. O texto `ILLUM ON` mostra hash abreviado, ganho máximo
e tempo do frame.

Os diagnósticos offline mostram também em vermelho o que a configuração
anterior aceitava e a nova rejeitou:

- [piso branco](illumination_white_floor.png);
- [fita na borda](illumination_border_wire.png).

Se o PNG estiver ausente, tiver hash incorreto ou resolução diferente de
480×360, a câmera registra o motivo, publica `illuminationCorrectionActive=false`
e executa o pipeline 61/40 original. O mapa nunca é redimensionado
silenciosamente.

Para reconstruir a referência:

```powershell
python scripts/build_line_illumination_reference.py calibration/captures/validation_white_floor_01
```

Para comparar um dataset sem acessar o robô:

```powershell
python scripts/line_calibration.py calibration/captures/validation_gap_01 `
  --annotations calibration/annotations/gap_01.json `
  --output build/gap-with-illumination --ratio 61 --min-threshold 40 `
  --exclude-green --illumination-correction
```

Depois de um deploy autorizado, a validação parada deve capturar nomes novos:

```sh
python3 scripts/request_line_calibration.py white_floor_illumination_live_01 \
  --scenario white_floor --frames 90
python3 scripts/request_line_calibration.py white_floor_original_live_01 \
  --scenario white_floor --frames 90 --no-illumination-correction
```

O acesso com a chave dedicada permitiu consultar a Raspberry, mas o capturador
seguro recusou a nova coleta porque os dois sinais de E-Stop estavam
desativados. O operador decidiu continuar sem essa captura. Portanto, a
referência versionada permanece baseada na sequência anterior de 90 PNGs
lossless com segurança comprovada. O frame do stream confirmou visualmente a
simetria das sombras, mas não foi tratado como raw nem usado para ajustar o
mapa. A validação lossless na pose atual, o FPS completo depois de um deploy
autorizado e as runs para contar GAPs falsos permanecem como etapas
operacionais.
