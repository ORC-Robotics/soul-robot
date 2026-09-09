# GAP: NEAR-C virtual e validação frontal

## Resultado e diagnóstico

O gate atual usa a presença do sensor virtual central `NEAR-C`, independente
da validade global do Fusion. A frontal responde
`PRESENT / UNCERTAIN / ABSENT`; a decisão inferior é
`NORMAL / CHECKING / GAP / LOST`. A frontal não calcula steering nem comanda
motores. O operador instalou a revisão no robô em 9 de setembro; os hashes de
`line_control.py` e `gap_validation.py` foram conferidos antes de analisar as
três runs descritas abaixo. Esta tarefa não alterou systemd.

Antes da correção mais recente, um detector geométrico cobria quase toda a base
da imagem inferior. Uma sombra lateral podia satisfazer esse detector e publicar
`nearLinePresent=true`, mesmo com `nearFinePosition=null` e o `NEAR-C` vazio.
O Fusion permanecia válido pelo mesmo ruído e mantinha `SOURCE: fusion`, sem
abrir CHECKING. Agora somente a ocupação do corredor central pode manter a
presença local. O código C++ `normalSteeringValid` representa validade do
comando, não existência física da fita; ele continua validando as potências
NORMAL sem votar na entrada de GAP.

A primeira versão local da frontal exigia concordância com a extrapolação
inferior. Essa exigência foi substituída conforme o relato da equipe: fita
lateral, inclinada ou deslocada após uma curva pode confirmar GAP. A referência
inferior agora serve apenas ao desenho ciano e ao desempate de qual candidato
mostrar como principal. Não pode rejeitar presença frontal.

Os ganhos e a seleção do Fusion em LINE normal, PID, sensores virtuais,
segmentação calibrada, máquinas de curva/verde e potências de recovery foram
preservados. Durante um GAP já reconhecido, uma seleção `deepestFallback`
estável pode receber autoridade Fusion antes de alcançar o NEAR. Essa exceção
é transitória e não muda os critérios do seguimento normal. O bloqueio
`local_line_lost` continua ativo exclusivamente após LOST confirmado: pixels
residuais não podem encerrar prematuramente a busca.

## Presença próxima: `nearLinePresent`

`nearLinePresent` agora reflete diretamente o sensor virtual `nearCenter`. Na
resolução inferior de 480×360, ele ocupa x=38,5–61,5% e y=82–100%. A ocupação
média da máscara nessa faixa precisa alcançar 10% para publicar `PRESENT`.
Esses limites são os mesmos usados pelo seguidor virtual preservado.

Sombras, sujeira e fragmentos restritos às laterais ainda podem formar um target
Fusion, mas não votam como linha sob o centro do robô. Isso permite iniciar GAP
quando a faixa central some, mesmo que haja ruído preto nas asas da imagem. A
frontal e o FAR inferior continuam sendo evidências de continuação; nenhum deles
rearma o `NEAR-C`.

Uma sombra grande que atravesse o corredor central ainda pode ativar o sensor.
Se isso aparecer em novas runs, a sequência raw/máscara deve orientar um ajuste
da ocupação ou dos limites da faixa. O caso observado em 9 de setembro era
lateral e já é separado pela geometria existente, portanto não justificou mudar
o threshold calibrado da câmera.

A medição de ocupação é por frame. A memória temporal só aceita sequência e
timestamp crescentes, com idade de no máximo 125 ms. Arma após dois frames de
presença, lembra essa presença por 300 ms e exige dois frames novos de ausência
para abrir CHECKING. Frame repetido, antigo ou futuro não confirma perda.
Uma imagem vazia isolada não abre GAP.

## Decisão e autoridade

| Condição | Decisão / efeito |
|---|---|
| `NEAR-C` ativo | NORMAL; controle inferior original |
| Presença local recente + duas ausências novas | CHECKING; tentativa limitada de travessia inferior |
| CHECKING + qualquer posição FAR inferior confiável nas mesmas duas ausências | GAP; confirma imediatamente um GAP curto |
| CHECKING + duas observações frontais novas PRESENT | GAP; mantém travessia inferior |
| GAP + FAR trusted | `virtual-gap-far`; FAR BAND, ou o conjunto FAR quando ela está vazia, guia com o mapper NORMAL |
| GAP + target Fusion inferior apoiado pelo mesmo componente conectado em FAR e MEDIUM, estável por dois frames | Mantém decisão GAP e transfere o controle para `fusion-gap-reacquire` |
| Sem confirmação frontal ou inferior em 500 ms | LOST; inicia recovery inferior existente |
| GAP perde toda evidência de continuação por 200 ms | LOST |
| GAP chega a 1,5 s sem continuação inferior no frame atual | LOST; a frontal sozinha não prolonga esse teto |
| Duas observações novas de fita inferior reaparecida | Libera NORMAL e encerra busca/travessia |
| Verde, recuperação por sensores ou tracker especial já ativo | Preserva prioridade existente; desarma a memória local |

CHECKING não é parada: usa provisoriamente o avanço GAP original. O contador
legado de 45 frames não recebe autoridade para iniciar busca enquanto o gate
estiver em CHECKING/GAP. A evidência inferior do frame atual é consumida antes
dos prazos. Evidência frontal atrasada não reabre uma janela vencida.

`SOURCE: gap-forward` significa avanço reto enquanto nenhuma posição FAR trusted
está disponível. `SOURCE: virtual-gap-far` significa que a posição FAR guia a
travessia. Ela usa a mesma correção contínua limitada do mapper NORMAL: ambas as
rodas permanecem positivas e os ramos PIVOT, SPIN e parada de uma roda ficam
bloqueados. `DECISION: GAP` significa continuação confirmada pelo FAR inferior,
pela frontal ou pelo target Fusion inferior reobservado. Quando FAR e MEDIUM
trusted sustentam juntos o mesmo componente conectado e o target Fusion pertence
a esse componente por dois frames novos,
`SOURCE: fusion-gap-reacquire` conserva a direção Fusion, mas limita as potências
ao mapper NORMAL enquanto a fita não chega ao NEAR; nenhuma roda recua. Ao chegar
a LOST, a mesma ré de cinco frames e a mesma varredura do
`VirtualLineSearchTracker` assumem, no lado lembrado pela inferior. Dois frames
FAR trusted podem cancelar essa busca e retomar GAP sem esperar pelo NEAR. Não
foi criado recovery novo nem alteradas suas potências.

A espera original por um candidato verde ainda pode preservar o último comando
Fusion por até dois frames. Essa prioridade breve foi mantida; por isso DECISION
e SOURCE são publicados separadamente e podem divergir durante essa espera.

Para um GAP curto, o FAR inferior é evidência melhor e mais rápida que esperar o
IPC frontal: vem do mesmo frame em que a ausência local foi medida. Duas leituras
trusted confirmam continuação se `farBandPosition` ou `farPosition` for finita.
A FAR BAND é preferida por enxergar mais longe; `farPosition` reúne os sensores
FAR laterais e central e assume quando a banda ainda está vazia. Uma única leitura
trusted já pode produzir steering limitado durante CHECKING, mas não confirma GAP
sozinha. MEDIUM isolado não guia a travessia e não libera o Fusion. A reaquisição
antecipada exige FAR e MEDIUM trusted simultâneos, target Fusion finito,
conectividade física na máscara e estabilidade temporal. Dois trechos distintos
não podem somar seus trusts para liberar o Fusion. Uma curva forte ainda passa
quando forma um componente contínuo; não existe limite arbitrário entre suas
posições laterais. O FAR não substitui `nearLinePresent`, não rearma presença
local e não interfere em LINE normal. Da mesma forma, um target Fusion sem esse
suporte conjunto não pode vetar a perda do `NEAR-C` nem assumir durante GAP.

Essa restrição veio de evidência da run real de 8 de setembro. Em uma execução de
44 segundos, o CSV registrou 24 entradas em GAP e 121 frames nesse estado. Em
100 deles, `gap-sensor-recovery` parou uma roda: houve 52 comandos `0,75/0,00`
e 48 comandos `0,00/0,75`. O lado alternou conforme FAR/MEDIUM mudavam, a linha
era reencontrada brevemente e logo perdida de novo. A decisão GAP estava correta;
o giro comandado dentro dela causou o círculo. O arquivo bruto foi preservado
localmente em `build/gap-last-run/obr_curve_diagnostics.csv` e permanece fora do Git.

O comportamento lateral foi introduzido em `0d1daf0` (`perto da estabilidade`,
25 de agosto de 2026). Antes desse commit, o ramo GAP aplicava direção zero com
fonte `gap-forward`. O commit passou a dar prioridade à direção observada em
FAR/MEDIUM e a usar `gap-sensor-recovery`, que para uma das rodas. Os refactors
posteriores apenas moveram esse ramo entre arquivos. A primeira correção removeu
o comando de uma roda parada. A revisão atual recupera o steering FAR sem
reintroduzir esse giro: usa o mapper NORMAL, sempre com as duas rodas avançando.
Quando FAR e MEDIUM confirmam uma trajetória Fusion estável, o próprio Fusion
reassume com ambas as rodas para frente; até esse ponto, MEDIUM não comanda a
travessia.

### Três runs reais após o primeiro deploy

O CSV de 9 de setembro registrou as três tentativas nos frames 95–201, 300–394
e 475–615. Nas duas falhas, a sequência foi praticamente igual: um candidato
distante ativou GAP/Fusion, o `NEAR-C` reapareceu por poucos frames, o Fusion
normal chegou a saturar em pivot e houve uma segunda perda. O operador confirmou
depois que essa primeira reaquisição pertencia a outro trecho desconectado da
continuação correta. Essa segunda entrada teve somente oito frames de
`gap-forward`, aproximadamente 0,27 s, antes de LOST.

Na primeira falha, FAR BAND reapareceu durante a busca por 28 frames; na segunda,
por nove frames. O estado LOST antigo ignorou ambos. Na terceira tentativa, FAR
BAND reapareceu por 45 frames e MEDIUM por 30, mas a busca continuou até a ajuda
manual levar a fita ao `NEAR-C`. A janela curta agravou as falhas, mas não foi a
causa inicial: o gate aceitava trust FAR e MEDIUM calculado separadamente, sem
provar que o target Fusion pertencia à mesma fita. Havia ainda duas falhas:
LOST era terminal sem NEAR e o Fusion de reaquisição podia mandar uma roda para
trás. A terceira tentativa não
terminou limpa no registro: o último frame ainda estava em GAP, sem NEAR.

A revisão seguinte aumentou `confirmation_seconds` de 0,25 para 0,50 s, permite
LOST → GAP após dois frames FAR trusted e limita `fusion-gap-reacquire` ao mapper
NORMAL. O gate agora exige que o label escolhido pelo Fusion cruze FAR e MEDIUM;
trechos desconectados não somam evidência. O seguimento LINE normal conserva sua
curva de potência e seus pivots.
O CSV bruto foi preservado localmente em
`build/gap-three-runs-20260909-073732/obr_curve_diagnostics.csv` e fica fora do Git.

Em LOST, o argumento do mapper impede que Fusion ou pixels soltos cancelem a
busca. As exceções são o reencontro local confirmado e dois frames FAR trusted;
este último retoma GAP com potência limitada. Na recuperação normal, o gate
libera o tracker e o mapper original volta a decidir, inclusive com fita lateral.

Sem histórico recente de fita local, não se inventa um GAP no startup. Nesse
caso a lógica normal/recovery já existente continua decidindo. Um robô colocado
parado diretamente num vazio não fornece a transição presença → ausência.

`gap_entry_is_required` recebe a presença temporal já confirmada do `NEAR-C`.
A assinatura antiga foi mantida para consumidores/testes legados; o caminho
legado não é chamado pelo gate atual e não decide sobre o novo IPC.

## O que a frontal confirma

A ROI frontal existente continua x=5–95%, y=55–100%, aproximadamente 7–16 cm à
frente na montagem informada. Pelo menos duas bandas devem aparecer nos 60%
mais próximos dessa ROI. Fita observada apenas no trecho distante fica UNCERTAIN.

PRESENT significa que há pelo menos um componente com espessura, extensão,
alongamento e bandas compatíveis com fita. Mais de uma fita plausível não reduz
presença a zero: os candidatos são mantidos separados. O candidato principal é
o mais próximo; a posição inferior recente ajuda somente no desempate lateral.
Não há média de centroides globais.

Não existe limite de erro contra heading, curvatura ou posição da inferior para
confirmar presença. Mudança de direção após curva e desalinhamento entre câmeras
não invalidam fita. O desenho previsto usa coordenadas normalizadas com x=0 no
eixo do robô, sem comparar pixels de resoluções diferentes. A projeção não é
uma transformação métrica calibrada e não decide GAP.

A nota publicada é uma medida de suporte geométrico, não uma probabilidade:
`min(1, bandas/mínimo, extensão/mínimo, alongamento/mínimo)`. Largura e ligação
são verificadas antes. PRESENT exige suporte completo e bandas próximas. Uma
nota 1 com suporte só distante continua UNCERTAIN; consumir apenas o número é
incorreto. O gate lê estado, booleano de presença, versão e frescor juntos.

Essa filosofia não distingue semanticamente todas as linhas da pista. Uma faixa
lateral real ou um caminho serrilhado com espessura/extensão suficientes pode
contar como presença. Isso não lhe dá autoridade de steering. Uma sombra alongada
com formato de fita ainda pode enganar a medição; a qualidade da segmentação
continua importante. Não há promessa de eliminar todo falso positivo.

## Configuração

Parâmetros em [camera_config.py](../scripts/vision/camera_config.py). Os dicionários
do GAP reutiliza `VIRTUAL_CENTER_X0/X1`, `VIRTUAL_NEAR_Y0/Y1` e
`VIRTUAL_ROW_MIN_ACTIVATION`; alterar esses valores também muda o sensor virtual
usado pelo seguidor. Não há hot reload; uma instalação futura deve atualizar os
processos correspondentes juntos.

| Parâmetro do gate inferior | Valor |
|---|---:|
| Faixa horizontal do `NEAR-C` | 0,385–0,615 |
| Faixa vertical do `NEAR-C` | 0,82–1,00 |
| Ocupação mínima | 0,10 |

O analisador geométrico por componentes permanece na câmera frontal:

| Parâmetro frontal | Valor |
|---|---:|
| `bands` / `min_bands` | 7 / 3 |
| `min_width` / `max_width`, fração da largura | 0,008 / 0,15 |
| Espessura equivalente na resolução atual | 7,68–144 px em 960 |
| `min_extent`, fração da largura | 0,07 (67,2 px) |
| `min_elongation` / `clipped_elongation` | 1,4 / 0,75 |
| `near_fraction` / `near_bands` | 0,60 / 2 |
| `max_slope`, px transversais por px longitudinal | 3 |
| `max_missing_bands` / `max_components` | 1 / 16 |
| `uncertain_threshold` | 0,60 |

Os limites frontais de espessura e extensão são proporcionais à resolução.
Reduzi-los aceita fragmentos menores; aumentá-los perde pontas legítimas e fita
lateral. Os filtros de alongamento ajudam contra blobs, mas precisam considerar
bordas.

`GAP_VALIDATION_CONFIG`: `near_present_frames=2` arma; `near_loss_frames=2`
confirma perda; `bottom_far_present_frames=2` confirma qualquer posição FAR
trusted; `bottom_fusion_reacquire_frames=2` libera o target Fusion distante
somente com FAR e MEDIUM trusted no mesmo componente;
`bottom_fusion_target_radius_px=3` associa o ponto Fusion ao label da fita sem
unir componentes;
`near_history_seconds=0.30`; `forward_present_frames=2` confirma
frontal; `source_timeout=0.125`; `confirmation_seconds=0.50`;
`evidence_grace_seconds=0.20`; `max_gap_seconds=1.5`. Tempos em segundos.
A continuação inferior observada no frame atual mantém GAP além desse último
teto; sem ela, o teto impede que apenas a frontal sustente avanço indefinido.
A reaquisição mantém `GEOMETRIC_GAP_REACQUIRE_FRAMES=2`. O prazo C++ frontal
continua `kForwardLineStatusTimeoutMs=125`, em milissegundos.

`FORWARD_PATH_CONFIG` contém apenas as escalas da previsão de debug. A referência
expira em `reference_timeout=2 s`; expirar não bloqueia presença frontal.

## IPC e debug

Inferior: `/dev/shm/obr_line_status.json` publica `nearLinePresent`,
`nearLineState`, `nearLineMissingFrames`, `bottomFarLinePresent`,
`bottomFarPresentFrames`, `bottomFusionReacquireCandidate`,
`bottomFusionReacquireFrames`, `bottomFusionReacquireReady`, `forwardPresenceState`,
`gapValidationDecision`, `gapValidationReason` e referência opcional.
`lineControlSource` continua informando qual ramo inferior gerou os comandos.

Frontal: `/dev/shm/obr_forward_line_status.json`, agora `forwardPathVersion=2`,
com `forwardLinePresent` e `forwardPathState=PRESENT/UNCERTAIN/ABSENT`.
Campos antigos de potência frontal permanecem `null`; `normalCommandValid()`
no C++ continua sempre falso. Schema anterior não confirma presença no gate novo.
Escrita atômica e rejeição de dados vencidos/duplicados foram preservadas.

Os dois streams mostram NEAR, FWD e DECISION; SOURCE aparece no overlay inferior
existente e no painel frontal. Na inferior, o painel de presença aparece mesmo
com `LEGACY_LINE_DEBUG_ENABLED=False`. Verde marca fita aceita; amarelo indica
suporte insuficiente; vermelho mantém candidatos rejeitados. Ciano é apenas
previsão opcional. UNKNOWN/UNAVAILABLE indicam diagnóstico inferior não disponível.
O dashboard também publica a decisão e a fonte inferior; seu resumo de missão
é atualizado durante a execução da missão.

## Evidência física e dados preservados

Na run relatada em 9 de setembro, a leitura ao vivo do IPC inferior mostrou no
mesmo frame `nearFinePosition=null`, `farTrusted=false`, `mediumTrusted=false`,
`nearLinePresent=true`, Fusion válido em aproximadamente 10,6° e
`SOURCE: fusion`. Isso confirmou que o gate largo, e não o `NEAR-C`, sustentava
NORMAL. A máscara exibida continha uma mancha lateral; sem uma captura raw/máscara
pareada dessa run, esse diagnóstico comprova a causa lógica da decisão, mas não
atribui toda a mancha a uma causa óptica específica.

A equipe identificou a posição como um gap real da pista. Foram lidos os streams
existentes, sem abrir outra câmera, alterar configuração ou enviar movimento:
[inferior](gap-presence-evidence/down.jpg) e [frontal](gap-presence-evidence/forward.jpg).
São JPEGs com overlays, não RGB/raw nem máscaras pareadas.

Na imagem inferior ainda existe uma porção larga de fita que chega à base da
imagem; na frontal aparece a interrupção e fita mais à frente. A leitura de
[60 IPCs distintos](gap-presence-evidence/ipc.json) confirmou SOURCE fusion em
60/60, sem trust FAR/MEDIUM. Essa posição sozinha **não comprova Fusion sustentado
apenas por partículas** e não deve ser usada para forçar GAP antes da perda local.
O código remoto ainda publica o schema anterior; nenhuma implementação nova foi
instalada nesta tarefa. Hashes e horários estão preservados na evidência.

A captura pareada raw/máscara foi recusada pelo `StationaryGuard` já existente:
as emergências estavam desativadas, embora potências aplicadas e encoders
estivessem zerados. Aguardou-se confirmação do operador; a coleta pareada não
foi executada. O pedido não foi publicado e nenhum controle foi alterado.

Foram reprocessadas **900 máscaras reais já salvas**, em dez sequências de 90:

| Sequência | PRESENT | ABSENT / UNCERTAIN |
|---|---:|---:|
| Reta central | 90 | 0 / 0 |
| Curva suave | 90 | 0 / 0 |
| Dobra forte | 90 | 0 / 0 |
| Curva 90° | 90 | 0 / 0 |
| Interseção | 90 | 0 / 0 |
| Próximo do verde | 90 | 0 / 0 |
| Gap com ponta de fita ainda próxima | 90 | 0 / 0 |
| Borda com fio | 90 | 0 / 0 |
| Reta com sombra | 90 | 0 / 0 |
| Piso branco sem fita | 0 | 90 / 0 |

[Resumo do replay do NEAR-C, configurações e hashes](gap-presence-evidence/replay-near-center-summary.json).
São dados estáticos da calibração anterior, incluindo reprocessamentos com o
perfil 61/min40/verde; não são novas travessias nem validação independente da
transição GAP/LOST. Nove sequências ainda contêm fita local verdadeira. Confirmar
presença nelas significa apenas que a ocupação central ultrapassou o limite.
O [replay geométrico anterior](gap-presence-evidence/replay.json) foi preservado
como evidência histórica e não representa mais o gate inferior ativo.

## Testes e performance

A suíte local passou: **350 testes Python, 12 testes de calibração e os sete
alvos CTest**, além do build completo. Testes novos executam o mesmo
`GapValidator.process_frame` usado pela aplicação, seguido pelo controlador
inferior real: reta e curvas com comandos idênticos; `NEAR-C` central;
micropartículas; Fusion válido por sombra lateral; perda em dois frames novos;
frontal inclinada/deslocada; ausência prolongada; retorno da linha;
FAR inferior central/lateral num GAP curto; alternância rápida dos lados do FAR;
frames repetidos/antigos/futuros;
prazo absoluto; verde e trackers especiais.

No benchmark local (Windows 11, CPU Intel, Python 3.12.10, OpenCV 5.0.0, uma
thread, 960×540, dez aquecimentos e 100 frames por caso), segmentação e análise
frontal tiveram:

| Caso | Mediana, ms | P95, ms |
|---|---:|---:|
| Reta | 1,719 | 2,760 |
| Curva forte | 1,931 | 2,303 |
| Dois caminhos | 1,999 | 2,499 |
| Interseção | 1,606 | 1,975 |
| Lateral | 1,700 | 2,172 |
| Inclinada | 1,361 | 1,816 |
| Partículas | 0,757 | 1,176 |
| Dez caminhos | 4,645 | 5,306 |

Como a aplicação já calcula os sensores virtuais, ler o `NEAR-C` e atualizar o
gate acrescentou mediana de **0,0016 ms**, P95 de 0,0027 ms. A referência
opcional acrescentou mediana de 0,194 ms. O tempo continua incluído em
`lineProcessingMs`. [Benchmark completo](gap-presence-evidence/benchmark.json).
O processamento geométrico por componentes e bandas permanece apenas na frontal,
sem ML ou novas dependências. Não há previsão de runtime garantido no Raspberry:
captura, IPC, JPEG, streaming e detector de bolas não estão nessa medição.

Comandos locais, sem iniciar câmera ou motores:

```sh
cmake -S . -B build
cmake --build build -j 2
ctest --test-dir build --output-on-failure
python -m unittest discover -s tests/python
python scripts/test_line_calibration.py
python tools/camera/benchmark_forward_path.py --frames 100
python tools/camera/evaluate_near_presence.py calibration/experiments/validation_gap_01/candidate_61_min40_green
git diff --check
```

## Arquivos e validação futura

- `vision/line_presence.py`: medição física e diagnóstico inferior.
- `vision/gap_validation.py`: memória fresca, gate e reutilização da travessia.
- `vision/forward_path.py`, `forward_camera_stream.py`: presença frontal, IPC v2 e overlay.
- `vision/line_control.py`: entrada explícita de GAP, proteção de LOST contra fragmentos; mapper normal preservado.
- `vision/application.py`, `status_publisher.py`, `camera_config.py`: integração, publicação e parâmetros.
- `camera_monitor.h/.cpp`, `forward_line_assist.h/.cpp`, `robot_state.h`, `dashboard_server.cpp`: contratos e diagnóstico, sem autoridade frontal de motor.
- `include/obr/config.h`: comentários de compatibilidade, valores existentes mantidos.
- Testes em `tests/python/test_forward_path_validation.py`, `test_forward_camera_stream.py` e `tests/main_mission_test.cpp`.
- Ferramentas em `tools/camera/benchmark_forward_path.py` e `evaluate_near_presence.py`; esta documentação e seus dados em `docs/gap-presence-evidence`.

Antes de instalar futuramente, reconciliar a divergência remota registrada em
[calibration/RESUME.md](../calibration/RESUME.md). Python inferior, frontal e C++
precisam da mesma versão. Não executar deploy geral com versões divergentes.

1. Com emergência ativa e robô parado, verificar presença local na reta e em
   curvas laterais; registrar raw/máscara pela ferramenta existente.
2. Mover manualmente a pista sob rodas suspensas para observar a transição de
   fita real para partículas: dois frames de perda devem abrir CHECKING mesmo
   com Fusion válido. Conferir timestamps, fonte e motivo.
3. Mostrar fita frontal inclinada/lateral: confirmar GAP sem exigir a linha
   ciana coincidente. Um frame frontal vazio isolado não deve declarar LOST.
4. Retirar evidência frontal até o prazo: verificar LOST e a busca original.
   Reapresentar FAR trusted por dois frames e conferir retorno a GAP; levar a
   fita ao NEAR-C e conferir a devolução ao normal.
5. Somente após os ensaios estáticos, verificar movimento com o procedimento
   existente de potência reduzida, emergência acessível e novos trechos reais.
   Medir falsos GAP, perdas corretamente encaminhadas, atraso e FPS no Raspberry.

Segurança no código: emergência para motores **sim**; timeout de comandos
preservado **sim**; clamp preservado **sim**; pinos centralizados e inalterados
**sim**. Comentários novos em português, acentos e propósito revisados **sim**.
Nenhum serviço, GPIO ou comando de movimento foi alterado/executado nesta tarefa.
