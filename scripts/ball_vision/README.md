# Visão de bolas pela câmera frontal

Esta ferramenta usa o YOLO ONNX treinado para detectar bolas pretas e pratas na
câmera frontal. Ela não envia comandos aos motores e não acessa a câmera
inferior do segue-faixa.

O arquivo implantado é `assets/models/ball_detector.onnx`, exportado do treino
completo `ball_detector_combined_stable`. O processo falha de forma segura se o
modelo estiver ausente; ele não volta para o detector geométrico antigo.

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
- `yolo_detector.py`: inferência ONNX, decodificação das caixas e NMS.
- `ball_detector.py`: detector geométrico legado, mantido apenas para diagnósticos.
- `ball_tracker.py`: estabilidade temporal e aquisição inicial.
- `distance_calibration.py`: conversão de raio para distância.
- `camera.py`: captura opcional da câmera frontal.
- `main.py`: execução local pela linha de comando.

O detector também está integrado ao stream frontal normal. No dashboard,
selecione `Frontal` e pressione `ATIVAR` para ver o vídeo anotado e o HUD com os
dados da bola. A inferência YOLO só é executada quando a missão autônoma selecionada
é `ÁREA DE RESGATE · VÍTIMA MAIS PRÓXIMA` e está em execução; fora dela, o stream permanece disponível como
vídeo cru e o IPC não publica uma bola antiga.
O stream mantém o assistente frontal e limita a visão pesada a 30 análises
por segundo.

## Dependências

Na Raspberry Pi:

```sh
sudo apt install -y python3-picamera2 python3-opencv python3-numpy
```

O deploy instala `onnxruntime==1.30.0` no ambiente virtual do robô. Ele executa
o ONNX do YOLO11, pois a OpenCV 4.6 da Raspberry é usada apenas para captura e
desenho do overlay.

No robô, o modelo é exportado em 384×384 e a inferência é limitada a 4 FPS em
uma thread separada. Se uma análise ultrapassar 250 ms, a próxima espera a
anterior terminar, sem acumular frames antigos. A câmera e o stream continuam
em 30 FPS; alterar essa resolução ou o limite exige medir novamente o
desempenho em campo. As caixas retornam às coordenadas de 960×540 antes do
cálculo, portanto a calibração de distância existente permanece inalterada.

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
2. `yolo_detector.py` preserva a proporção do frame com letterbox, executa o
   modelo ONNX pelo OpenCV DNN, aplica NMS e converte cada caixa em um
   `BallCandidate` compatível com o restante da visão.
3. O raio usado pela calibração é metade da média entre largura e altura da
   caixa YOLO. A fórmula e os pontos existentes de distância não foram alterados.
5. `distance_calibration.py` interpola os cinco pontos medidos. Fora da faixa,
   usa uma relação inversa ancorada no ponto extremo para permanecer contínua e
   positiva.
5. `main.py` calcula o ângulo pelo FOV horizontal e desenha todos os dados. Os
   dois tipos produzem o mesmo `BallCandidate`, com a área da caixa em pixels
   para permitir uma seleção inicial comum.
7. `ball_tracker.py` associa a mesma bola entre frames e aplica uma mediana curta
   seguida de suavização exponencial ao centro e ao raio. O filtro reduz saltos
   do Hough sem publicar uma posição antiga quando o frame atual não tem bola.
   Antes de travar o alvo, três frames de aquisição mantêm os motores parados e
   confirmam espacialmente uma vítima. A prata possui prioridade sobre a preta
   na aquisição; durante o giro, uma troca ainda exige quatro frames, melhoria de
   score e um salto fisicamente plausível. Se ele desaparecer, o rastreador
   publica ausência sem liberar outro objeto; somente `reset()` no começo de
   outra execução permite nova escolha.
   O IPC publica `targetSequence`, `targetLocked` e `visibleAreaPixels` para
   impedir que o C++ use uma observação herdada de uma execução anterior.

## Ajustes de campo

Os valores iniciais do YOLO precisam ser validados com a iluminação da arena:

```sh
python3 scripts/ball_vision/main.py --horizontal-fov 62 --center-angle 5
```

- Os limites do YOLO ficam em `YoloBallDetectorConfig`. Aumentar
  `confidence_threshold` reduz falsos positivos, mas pode perder vítimas distantes.
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
- O gerenciamento de alvo usa os estados `SEARCHING`, `TRACKING` e `REACQUIRE`.
  Uma candidata diferente precisa persistir por `switch_confirmation_frames`,
  superar `minimum_confidence_improvement` e melhorar a distância aparente ou o
  alinhamento pelos limites `minimum_distance_improvement_ratio` e
  `minimum_position_improvement_pixels`. Durante a confirmação, o alvo atual é
  mantido. Depois de `reacquire_after_missing_frames` sem o alvo, o gerenciador
  entra em reaquisição; após `search_after_missing_frames`, reinicia a busca.

Na Área de Resgate, o C++ gira somente enquanto o tracker não publicar uma
vítima travada. Ao receber a confirmação, zera os motores em um ciclo, chama o
alinhamento existente e usa a mesma distância calibrada para parar em
`kBallApproachStopDistanceCm`. A velocidade da busca fica em
`config::kRescueSearchTurnPower`.

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
