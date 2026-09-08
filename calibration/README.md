# Calibração da câmera inferior — sessão de 08/09/2026

**Sessão encerrada: [resultado final e configuração aplicada](report/RESULTS.md).**
Foram auditados 3.120 frames reais. O perfil inferior agora usa razão relativa
61%, limiar mínimo 40 e exclusão do verde da máscara preta; AE/AWB e morfologia
foram mantidos. As seções abaixo preservam o diagnóstico feito antes dos ajustes.

Para voltar a este assunto depois: [roteiro de retomada e melhorias possíveis](RESUME.md).

## Diagnóstico antes dos experimentos

O código do Raspberry é a referência do baseline. Após a troca de branch feita
pelo operador, os 16 arquivos Python locais do pipeline coincidem com a cópia
instalada, desconsiderando CRLF/LF. A cópia original está em `baseline/provenance`.

- Câmera IMX219, índice 0, sensor 1640×1232/10 bits, main 480×360 RGB888
  (array BGR do OpenCV), transformação de 180°, alvo 30 FPS; status inicial 30,23 FPS.
- `obr-line-camera` executa `scripts/run_line_camera.sh`, que inicia
  `camera_line_frame.py` → `vision/application.py` → `vision/camera.py`.
- AE e AWB habilitados; EV +0,4, contraste 1,05, nitidez 1,2, saturação 1,0.
  Brightness não é explicitamente configurado. ExposureTime e AnalogueGain são
  escolhidos automaticamente; os valores efetivos precisam vir dos metadados.
- BGR → cinza → fechamento retangular 151×151 estima o fundo → pixel branco
  se cinza ≤ min(floor(0,70 × fundo), 190) → abertura elíptica 13×13 → fechamento
  retangular 9×9 → recorte estrutural (y < 47 excluído) → filtros por contorno.
- Contornos: área ≥120 px², lado curto ≥15 px, pelo menos 15% do interior com
  raio de distância ≥5,5 px. Área máxima =100% da imagem. Contornos externos
  aceitos são preenchidos: buracos internos desaparecem nessa etapa.
- Existe função de reparo de reflexos, mas ela não é chamada pela segmentação.
- FAR, FAR BAND, MEDIUM e NEAR usam a máscara candidata. Fusion seleciona o
  contorno no envelope físico e o alvo; o extrator legado de scanlines fica
  desativado por padrão. A máscara inteira não equivale ao envelope Fusion.
- Verde usa HSV e ROI até y=300, além de associação com preto estrutural.
  Mudanças na imagem ou no preto podem afetá-lo mesmo sem editar o detector verde.
- MJPEG local: `127.0.0.1:8090/stream.mjpg?mode=real|line`, exposto pelo dashboard
  em `http://obr.local:8080/camera-stream.mjpg`. Ambos recebem desenhos; o modo
  linha também contém cinza e verde. Snapshot JPEG em `/tmp/obr_camera_frame.jpg`
  (2 FPS), status em `/tmp/obr_camera_status.json` (5 Hz), IPC de linha em
  `/dev/shm/obr_line_status.json` a cada processamento.
- Sem recompilar/reiniciar: seleção de visualização e pedido diagnóstico verde.
  Habilitar/desabilitar a câmera encerra/recria a captura. Os ajustes de câmera
  e segmentação não tinham interface de alteração durante a execução.

## Riscos identificados

O limiar relativo pode aceitar sombras abruptas; o kernel grande pode estimar o
fundo usando iluminação de outra região. Abertura forte pode apagar extremidades
finas. Fechamento pode unir frestas. Preencher contornos pode eliminar espaços
reais. O filtro de área máxima está praticamente desabilitado. Vários comentários
não correspondem aos valores (32% versus 30%; fechamento 7 versus 11; 30% versus
100%; iluminação descrita como ausente, embora LEDs estejam instalados).

## Baseline original preservado

`baseline/current_pose_01`: 60 RGB PNG sem perdas e respectivas máscaras
reconstruídas pelas funções instaladas, sem alterar parâmetros nem reiniciar.
Cada amostra inclui timestamp de publicação e ExposureTime, AnalogueGain,
ColourGains do mesmo request. O mecanismo original não preserva SensorTimestamp,
DigitalGain ou FrameDuration. Não confundir timestamp de publicação com exposição.
São amostras espaçadas (~11 Hz), e não 60 frames consecutivos a 30 FPS.
`raw` significa imagem colorida antes da segmentação, não Bayer do sensor.

O operador confirmou imobilidade/LEDs/disponibilidade, mas não nomeou o trecho.
O RGB mostra uma dobra forte aproximadamente de 90°, reflexos e sombra inferior.
O E-Stop foi confirmado no Raspberry e na ESP32, com potências e encoders zerados.
O registro está em `baseline/provenance/safety.json`.

## Método de comparação

Anotações são feitas no RGB, nunca inferidas da máscara vencedora. Regiões
incertas, bordas da fita e partes do chassi ficam sem rótulo. Componentes
desconectados são apenas candidatos a ruído: em gaps e interseções podem ser
legítimos. Falsos positivos são medidos contra fundo explicitamente anotado,
inclusive quando a sombra se conecta à trajetória.

Não se somam pesos ajustados para favorecer um resultado. O score conservador
usa o menor dos indicadores normalizados e mostra todos separadamente. Uma
perda de linha não pode ser compensada por uma imagem estável e vazia. Comparar
o pior cenário e impor ausência de regressão nas regiões anotadas precede
qualquer recomendação. Com somente um cenário, o resultado é exploratório.

Exposição fixa será avaliada separadamente de AWB e dos limiares. A documentação
oficial descreve ExposureTime em microssegundos, ColourGains vermelho/azul e
DigitalGain como metadado a observar:
https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf

Essa era a situação antes dos experimentos. Os resultados, a validação final
limitada a uma reta e as restrições de uso estão em [report/RESULTS.md](report/RESULTS.md).
