# Dataset e treinamento YOLO de bolas

Classes do modelo:

- `0`: `black_ball`
- `1`: `silver_ball`

## Estrutura oficial

```text
yolo_ball/datasets/
├── data.yaml
├── images/raw/<camera>/<sessão>/<black|silver|other>/
├── annotated/<sessão>/images/
├── annotated/<sessão>/labels/
├── train/images/
├── train/labels/
├── valid/images/
└── valid/labels/
```

- `images/raw`: frames originais recebidos do robô. As pastas `black`, `silver`
  e `other` servem somente para organizar a coleta; não são anotações YOLO.
- `annotated`: sessões selecionadas e concluídas na ferramenta de anotação.
- `train` e `valid`: conjuntos finais consumidos pelo treinamento.

O arquivo `datasets/data.yaml` é a única configuração de dataset usada pelo
`train.py`.

## Fluxo completo

### 1. Capturar no robô

Na dashboard, informe uma sessão, selecione a câmera, escolha o FPS e use o modo
de uma foto ou contínuo. Pause a captura ao terminar cada cenário.

Para trazer as imagens ao computador uma vez:

```powershell
.\tools\camera\sync_yolo_dataset.ps1 -Once
```

### 2. Selecionar e anotar

Escolha frames variados, evitando muitos quadros consecutivos quase iguais.
Exporte cada sessão anotada desta forma:

```text
datasets/annotated/sessao_preta_01/images/<arquivo>.jpg
datasets/annotated/sessao_preta_01/labels/<arquivo>.txt
```

Imagem e label devem possuir o mesmo caminho relativo e o mesmo nome-base.
Cada imagem precisa de um `.txt`: para uma imagem sem bola, crie um `.txt` vazio.
Essa exigência impede que uma anotação esquecida seja interpretada como negativo.

Cada linha não vazia do label possui:

```text
class_id center_x center_y width height
```

As coordenadas são normalizadas entre `0` e `1`.

### 3. Separar em train e valid

Prepare pelo menos duas sessões anotadas e execute:

```powershell
py -3 yolo_ball/prepare_dataset.py
```

O script:

- valida classes, coordenadas e pares imagem/label;
- mantém cada sessão inteira somente em `train` ou `valid`;
- usa 80% das sessões para treino e 20% para validação;
- prefixa os arquivos com o nome da sessão para evitar colisões;
- preserva o dataset existente e não sobrescreve conteúdo diferente sem
  `--overwrite`;
- grava a decisão em `datasets/split_manifest.json`.

Para alterar a proporção:

```powershell
py -3 yolo_ball/prepare_dataset.py --validation-ratio 0.25
```

### 4. Treinar na GPU

```powershell
py -3 yolo_ball/train.py --epochs 100 --image-size 640 --batch 8 --device 0
```

O melhor peso será salvo em:

```text
yolo_ball/runs/ball_detector/weights/best.pt
```

### 5. Exportar

```powershell
py -3 yolo_ball/export.py --format onnx
```
