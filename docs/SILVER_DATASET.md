# Coleta e treinamento da faixa prata

A coleta usa as câmeras já abertas pela aplicação. Na inferior,
`vision.application.DownwardCameraApplication` entrega o frame original ao
`SilverDatasetRecorder`; na frontal, o frame passa pela orientação física antes
da coleta, ainda sem overlays. `vision.status_publisher.LineStatusPublisher`
e o status frontal incluem a mesma telemetria de dataset no dashboard.

O recorder e o classificador já são classes com responsabilidades específicas.
O treinamento permanece offline, separado do ciclo do robô. TensorFlow não é
uma dependência das câmeras. O classificador distingue `black`, `other` e
`silver`; ele não substitui o seguidor geométrico nem calcula comandos dos
motores. Quando quatro inferências consecutivas confirmam `silver`, porém, a
câmera publica `courseMarkerConfirmed=GRAY` e a missão principal entra na área
de resgate. Por isso, o modelo precisa ser validado na pista antes da operação.

## Estado operacional temporário

O detector e a confirmação da entrada no resgate ficam ativos por padrão.
No primeiro positivo válido, o robô interrompe o segue-faixa e inicia uma janela
de avanço reto de até 5 cm em baixa velocidade, medida pelos encoders. Quatro
positivos consecutivos publicam `GRAY`, confirmam a entrada e param o robô. A
busca de vítimas não é iniciada automaticamente. Se o avanço terminar sem
confirmação, a candidata é descartada e o segue-linha volta.
`SILVER_DETECTION_ENABLED=0` desliga a inferência. O modo shadow está fixado
como falso em `vision.camera_config` enquanto esta integração é validada.

O desligamento foi motivado por falsos positivos observados em pista sobre uma
faixa preta sem cinza aparente. O modelo chegou a 85,1% de probabilidade de
`silver`, com margem de 71,4 pontos percentuais, e sustentou quatro positivos
até acionar a entrada na área de resgate. Aumentar somente a confirmação
temporal não corrige esse caso, pois o erro persistiu entre frames.

O modelo atual recebe o frame inteiro reduzido para 128×128. Ele não confirma
cor neutra, brilho, formato transversal, largura nem posição do marcador em
relação à trajetória. O dataset inferior disponível durante o diagnóstico
possuía apenas três sessões físicas, todas coletadas no mesmo dia. Isso permite
que o classificador aprenda iluminação e contexto da cena em vez do material
prateado. Também não há resultado versionado de uma validação independente em
runs completas.

Antes de reativar o marcador na missão principal:

1. grave como exemplos `black` os falsos positivos reais, além de sombras,
   reflexos, curvas, interseções e diferentes trechos de fita;
2. faça novas sessões em dias, posições e iluminações diferentes, separando
   treino, validação e teste por sessão física;
3. detecte primeiro uma faixa transversal plausível e classifique apenas essa
   ROI, exigindo brilho e neutralidade compatíveis com prata;
4. relacione o candidato à trajetória preta e rejeite regiões sem geometria de
   marcador;
5. meça falsos acionamentos por run e recall de prata em sessões que não
   participaram do ajuste;
6. calibre confiança, margem e confirmação temporal somente depois dessas
   medições.

Elevar apenas o limiar de 70% não é uma correção comprovada: já houve falso
positivo acima de 85%, e ainda não foi medida a distribuição dos positivos
reais fora do treino.

## Fotografar na Raspberry Pi

Com os serviços normais de câmera em execução e o robô parado, abra uma sessão SSH:

```bash
cd /home/raspberry/OBR2026K
python3 scripts/capture_silver_dataset.py --camera down --session session_001 --fps 4
```

Use `b`, `o` ou `s` e Enter para selecionar a classe e capturar; `p` pausa,
`r` consulta a contagem e `q` encerra pausando. A coleta começa pausada.
Para a frontal, substitua `down` por `forward`. Use um único controlador de
coleta por vez: o arquivo de controle é compartilhado entre as câmeras.

Os JPEGs ficam em `dataset/raw/<camera>/<session>/<classe>/`. A gravação ocorre
na thread do recorder com fila de dois frames. Falhas da coleta não interrompem
a visão. Ao abrir uma sessão, novos frames aguardam a contagem inicial sem
serem contabilizados como descartes. O dashboard mostra coleta ativa, pausada
ou sem atualização (4 segundos). Na seção das câmeras, cada clique ou toque
em `B` (preta), `P` (prata) ou `O` (outros) salva exatamente um frame YOLO.
Essa coleta fica separada em
`yolo_ball/datasets/images/raw/<camera>/<sessão>/<classe>/`; os frames precisam
ser anotados antes de serem movidos para `images/train` ou `images/val`.

Para receber automaticamente no computador cada imagem capturada pela dashboard,
deixe este comando aberto no PowerShell, na raiz do projeto:

```powershell
powershell -ExecutionPolicy Bypass -File tools/camera/sync_yolo_dataset.ps1
```

O sincronizador consulta a Raspberry a cada dois segundos e copia somente JPEGs
novos para `yolo_ball/datasets/images/raw/`. Ele usa a mesma chave SSH do deploy,
nunca remove arquivos locais ou remotos e pode ser encerrado com `Ctrl+C`.

Faça pelo menos três sessões independentes **por câmera**, cada uma com as três
classes. Varie posição e iluminação entre sessões. O treino separa sessões
inteiras, evitando que frames quase iguais caiam no treino e no teste.
Confira espaço livre e examine alguns JPEGs antes de fazer uma coleta longa.

## Conferir e treinar no computador

Copie `dataset/raw/` da Raspberry para `dataset/raw/` neste repositório. As
pastas de dataset e resultados continuam ignoradas pelo Git.

```bash
python3 training/training_silver_classifier.py --camera down --validate-only
```

A conferência exige NumPy, mas não TensorFlow. Ela verifica sessões, distribuição
das classes e separação dos conjuntos; a qualidade visual precisa ser revisada.

Para executar a inferência TFLite no serviço da câmera inferior, prepare o
ambiente opcional que o launcher reconhece somente quando todas as dependências
necessárias podem ser importadas:

```bash
cd /home/raspberry/OBR2026K
python3 -m venv --system-site-packages .venv-ai
.venv-ai/bin/python3 -m pip install ai-edge-litert
```

Se esse ambiente estiver ausente ou incompleto, o serviço continua usando
`.venv` ou o Python do sistema. Nesse caso, a visão de linha permanece ativa e
o detector Silver informa indisponibilidade na telemetria, sem confirmar cinza.

Em um ambiente de treinamento com NumPy e TensorFlow instalados:

```bash
python3 training/training_silver_classifier.py --camera down
python3 training/training_silver_classifier.py --camera forward
```

O treinamento usa MobileNetV3Small com pesos ImageNet (o primeiro uso precisa
baixá-los). A fase inicial permite até 60 épocas e o fine-tuning até 40, mas o
EarlyStopping encerra antes quando a validação deixa de melhorar. `--epochs` e
`--fine-tune-epochs` alteram esses limites. Por padrão, ele aguarda 12 épocas
sem melhora; `--early-stopping-patience` altera essa janela e `--lr-patience`
controla quando reduzir o learning rate. `--fine-tune` habilita a segunda fase.
`--roi LEFT TOP RIGHT BOTTOM`
define um recorte normalizado; se utilizado, a inferência futura deverá aplicar
o mesmo recorte no `SilverLineDetector`. O padrão é a imagem inteira. Os modelos
existentes de teste em `assets/models/` não demonstram qualidade de classificação
na pista.

Saídas: `training_outputs/silver_<camera>/` contém divisão das sessões, modelo
Keras, métricas, histórico e falsos positivos; `assets/models/silver_<camera>.tflite`
recebe o modelo exportado. Reexecutar o treinamento substitui essas saídas da
mesma câmera. Preserve uma cópia quando quiser comparar experimentos.

Depois do treinamento, `vision.silver_detection.SilverLineDetector` carrega o
modelo correspondente com `from_camera_model("down")` ou
`from_camera_model("forward")`. `detect(frame)` retorna `detected`, a classe
vencedora, as probabilidades e a margem da faixa prata sobre a concorrente mais
forte. Os limites iniciais são 70% de confiança e 15 pontos percentuais de
margem. A busca roda a 6 FPS; após o primeiro positivo, a confirmação roda a
15 FPS e exige quatro positivos consecutivos. Calibre esses valores com as
métricas e falsos positivos do dataset real antes de operar a missão principal.

## Validação antes de operar

```bash
python3 -m unittest discover -s tests/python
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

Após o deploy autorizado, valide primeiro com rodas suspensas: inicialização
parada, parada de emergência e perda de comandos. Confira ambas as imagens,
as três classes, pausa/retomada, troca de sessão e telemetria antiga com o
processo da câmera parado. Só então teste movimentação em potência reduzida.
Os entrypoints de deploy continuam `scripts/deploy.sh` e `scripts/deploy.ps1`,
delegando para `deployment/`. Nenhum serviço precisa mudar para a coleta.
