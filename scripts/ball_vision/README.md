# Visão de bolas pela câmera frontal

Esta ferramenta detecta bolas pretas e pratas com OpenCV e abre exclusivamente a câmera
frontal. Ela não envia comandos aos motores e não acessa a câmera inferior do
segue-faixa.

## Uso como módulo independente

Toda a detecção está contida nesta pasta. Para reutilizá-la em outro projeto,
copie o diretório `ball_vision` e importe somente a interface pública:

```python
from ball_vision import BallVisionPipeline

pipeline = BallVisionPipeline()
result = pipeline.analyze(frame_bgr)

if result.observation is not None:
    ball = result.observation
    print(ball.candidate.ball_type, ball.distance.distance_cm, ball.angle_degrees)
```

Use `pipeline.reset()` quando uma nova missão começar. O chamador fornece um
frame BGR do OpenCV; o módulo não depende do dashboard, do WebSocket, do IPC nem
do controle dos motores.

Arquivos internos:

- `pipeline.py`: interface pública e composição do detector.
- `ball_detector.py`: candidatos pretos e prateados.
- `ball_tracker.py`: estabilidade temporal e aquisição inicial.
- `distance_calibration.py`: conversão de raio para distância.
- `camera.py`: captura opcional da câmera frontal.
- `main.py`: execução local pela linha de comando.

O detector também está integrado ao stream frontal normal. No dashboard,
selecione `Frontal` e pressione `ATIVAR` para ver o vídeo anotado e o HUD com os
dados da bola. HSV e Hough só são executados quando a missão autônoma selecionada
é `ÁREA DE RESGATE · VÍTIMA MAIS PRÓXIMA` e está em execução; fora dela, o stream permanece disponível como
vídeo cru e o IPC não publica uma bola antiga.
O stream mantém o assistente frontal a 30 FPS e limita a visão pesada a 15 análises
por segundo.

## Dependências

Na Raspberry Pi:

```sh
sudo apt install -y python3-picamera2 python3-opencv python3-numpy
```

O stream frontal do dashboard e este programa não podem possuir a CAM1 ao mesmo
tempo. Antes de executar, use `DESATIVAR` no cartão da câmera frontal. O serviço
`obr-line-camera`, responsável pela câmera inferior, pode continuar ativo.

## Execução

A partir da raiz do projeto:

```sh
python3 scripts/ball_vision/main.py
```

Pressione `Q` ou `Esc` para fechar. Para imprimir o JSON preparado para uma
integração futura com a ESP32, limitado a cinco mensagens por segundo:

```sh
python3 scripts/ball_vision/main.py --print-json
```

Exemplo:

```json
{"type":"black_ball","distance_cm":42,"angle":15}
```

Quando o alvo selecionado é prateado, o mesmo contrato usa o tipo
`silver_ball`:

```json
{"type":"silver_ball","distance_cm":42,"angle":15}
```

O modo sem janela serve para diagnósticos remotos:

```sh
python3 scripts/ball_vision/main.py --headless --print-json
```

O índice padrão da câmera frontal é `1`. A mesma configuração usada pelo
restante do projeto pode alterá-lo:

```sh
OBR_FORWARD_CAMERA_INDEX=1 python3 scripts/ball_vision/main.py
```

## Como funciona

1. `camera.py` configura a segunda câmera em 960×540 a 30 FPS e garante seu
   fechamento mesmo quando ocorre uma falha.
2. Para a bola preta, `ball_detector.py` suaviza o frame, converte BGR para HSV e
   seleciona pixels cujo brilho `V` é baixo. Abertura remove pontos isolados e
   fechamento preenche pequenos reflexos na superfície da bola.
3. Contornos pequenos, excessivamente grandes, pouco circulares ou que preencham
   mal seu círculo envolvente são rejeitados. Isso reduz falsos positivos causados
   por sombras, retângulos e partes escuras da pista.
   Quando a bola toca a borda superior, o detector usa limites específicos para
   o arco visível e rejeita barras escuras pela proporção do contorno.
4. Para a bola prata, o detector realça o contraste local, procura circunferências
   com Hough e valida o brilho, a variação de intensidade, a textura interna e
   a cobertura da circunferência. A textura amassada do alumínio é necessária
   para não confundir o piso claro com a bola. No stream integrado, a etapa
   Hough usa meia resolução e converte a geometria encontrada de volta para a
   escala original, reduzindo CPU sem invalidar a calibração em 960×540.
5. `distance_calibration.py` interpola os cinco pontos medidos. Fora da faixa,
   usa uma relação inversa ancorada no ponto extremo para permanecer contínua e
   positiva.
6. `main.py` combina os dois detectores, calcula o ângulo pelo FOV horizontal e
   desenha todos os dados. Os dois tipos produzem o mesmo `BallCandidate`, com a
   área visível em pixels para permitir uma seleção inicial comum.
7. `ball_tracker.py` associa a mesma bola entre frames e aplica uma mediana curta
   seguida de suavização exponencial ao centro e ao raio. O filtro reduz saltos
   do Hough sem publicar uma posição antiga quando o frame atual não tem bola.
   Antes de travar o alvo, três frames de aquisição mantêm os motores parados e
   confirmam espacialmente a bola de maior área visível. A bola preta usa a área
   real do contorno; a prata conta somente os pixels de seu círculo que cabem no
   frame. Durante o giro, variações de raio e novas bolas maiores não trocam o
   alvo. Se ele desaparecer, o rastreador publica ausência sem liberar outro
   objeto; somente `reset()` no começo de outra execução permite nova escolha.
   O IPC publica `targetSequence`, `targetLocked` e `visibleAreaPixels` para
   impedir que o C++ use uma observação herdada de uma execução anterior.

## Ajustes de campo

Os valores iniciais precisam ser validados com a iluminação da arena:

```sh
python3 scripts/ball_vision/main.py \
  --maximum-value 100 \
  --minimum-circularity 0.72 \
  --horizontal-fov 62 \
  --center-angle 5
```

- Aumentar `--maximum-value` aceita objetos menos escuros, mas também mais sombras.
- Aumentar `--minimum-circularity` reduz falsos positivos, mas pode rejeitar uma
  bola parcialmente cortada pela borda.
- `--horizontal-fov` deve ser recalibrado com alvos em ângulos conhecidos.
- `--center-angle` controla a zona classificada como `centro`.
- Os limites da prata ficam em `SilverBallDetectorConfig`. Aumentar
  `minimum_edge_density` reduz falsos positivos no piso, mas pode perder uma bola
  muito distante ou com poucas dobras visíveis. Reduzir
  `hough_accumulator_threshold` aceita círculos menos definidos, com maior risco
  de falsos positivos.
- Os parâmetros temporais ficam em `BallTrackerConfig`. O histórico padrão de
  cinco frames prioriza estabilidade com pouco atraso; aumentar esse valor deixa
  a leitura mais estável, mas faz o centro responder mais lentamente ao movimento.

A calibração de distância vale para 960×540 e para o mesmo enquadramento usado nas
medições originais. Nesta versão, ela é compartilhada entre preta e prata sob
a premissa de que ambas têm o mesmo diâmetro real. Se os diâmetros forem
diferentes, a bola prata precisará de pontos próprios. Alterar resolução ou crop
também exige medir novamente os raios.

## Testes

Em um computador com OpenCV e NumPy:

```sh
python3 scripts/test_ball_vision.py
```
