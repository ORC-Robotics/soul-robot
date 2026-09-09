# GAP: presença física inferior e validação frontal

## Resultado e diagnóstico

O gate atual usa presença física de fita próxima, independente do Fusion.
A frontal responde `PRESENT / UNCERTAIN / ABSENT`; a decisão inferior é
`NORMAL / CHECKING / GAP / LOST`. A frontal não calcula steering nem comanda
motores. Esta implementação é local: **nenhum deploy ou alteração de systemd**.

Antes da correção, um fragmento podia manter o Fusion válido e impedir GAP
por dois caminhos: `fusion_near_connected` vetava a entrada, e a validade Fusion
participava de `normal_visible`, que encerrava a confirmação. O código C++
`normalSteeringValid` também representa validade do comando, não existência
física da fita. Ele continua validando a faixa de potências NORMAL sem votar
na presença local.

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

`line_presence.py` examina a máscara já segmentada, sem modificá-la. Na inferior,
o contexto cobre x=2–98% e y=60–100%. A confirmação local exige pelo menos três
bandas abaixo de y=76%, cobrindo o NEAR existente (82–100%) e suas laterais.
O contexto superior ajuda a avaliar a forma sem exigir que toda a fita caiba
na estreita região próxima.

Cada componente é examinado separadamente em até sete bandas horizontais e sete
verticais. A segunda orientação preserva curvas quase horizontais e saídas
laterais. Para ser fita plausível precisa combinar:

- espessura dentro da faixa configurada;
- extensão mínima entre bandas;
- pelo menos três bandas coerentes do mesmo componente;
- deslocamento transversal limitado entre bandas;
- extensão compatível com a espessura, com regra explícita para corte pelo FOV;
- suporte suficiente na região próxima.

Área só descarta partículas óbvias antes desse trabalho. Partículas separadas
não somam bandas entre si. Um blob pequeno não tem extensão; uma mancha compacta
não satisfaz o alongamento. O detector não exige estar no NEAR-C nem concordar
com um target Fusion.

Uma fita cortada pela base ou pela borda lateral pode continuar fora da imagem.
Nesses casos a exigência de alongamento é menor, mas largura, extensão mínima e
bandas próximas continuam obrigatórias. Isso evita declarar perda por uma ponta
real curta. A amostragem inclui as extremidades observáveis do componente;
intervalos estreitos demais continuam rejeitados.

A medição geométrica é por frame. A memória temporal só aceita sequência e
timestamp crescentes, com idade de no máximo 125 ms. Arma após dois frames de
presença, lembra essa presença por 300 ms e exige dois frames novos de ausência
para abrir CHECKING. Frame repetido, antigo ou futuro não confirma perda.
Uma imagem vazia isolada não abre GAP.

## Decisão e autoridade

| Condição | Decisão / efeito |
|---|---|
| Fita local plausível | NORMAL; controle inferior original |
| Presença local recente + duas ausências novas | CHECKING; tentativa limitada de travessia inferior |
| CHECKING + FAR inferior confiável nas mesmas duas ausências | GAP; confirma imediatamente um GAP curto |
| CHECKING + duas observações frontais novas PRESENT | GAP; mantém travessia inferior |
| GAP + target Fusion inferior distante, trusted e estável por dois frames | Mantém decisão GAP e transfere o controle para `fusion-gap-reacquire` |
| Sem confirmação frontal em 250 ms | LOST; inicia recovery inferior existente |
| GAP perde evidência frontal por 200 ms | LOST |
| GAP chega a 1,5 s desde CHECKING | LOST, mesmo com fita frontal visível |
| Duas observações novas de fita inferior reaparecida | Libera NORMAL e encerra busca/travessia |
| Verde, recuperação por sensores ou tracker especial já ativo | Preserva prioridade existente; desarma a memória local |

CHECKING não é parada: usa provisoriamente o avanço GAP original. O contador
legado de 45 frames não recebe autoridade para iniciar busca enquanto o gate
estiver em CHECKING/GAP. Os prazos são verificados antes de consumir nova
evidência; um ciclo atrasado não reabre uma janela vencida.

`SOURCE: gap-forward` significa execução de avanço, inclusive em CHECKING.
`DECISION: GAP` significa continuação confirmada pelo FAR inferior, pela frontal
ou pelo target Fusion inferior reobservado. Enquanto só existe evidência,
FAR/MEDIUM não produzem steering. Quando o mesmo contorno Fusion forma um target
estável em dois frames novos, `SOURCE: fusion-gap-reacquire` aplica o mapper
Fusion existente antes de a fita chegar ao NEAR. Ao chegar a LOST, a mesma ré de
cinco frames e a mesma varredura do `VirtualLineSearchTracker` assumem, no lado
lembrado pela inferior. Não foi criado recovery novo nem alteradas suas potências.

A espera original por um candidato verde ainda pode preservar o último comando
Fusion por até dois frames. Essa prioridade breve foi mantida; por isso DECISION
e SOURCE são publicados separadamente e podem divergir durante essa espera.

Para um GAP curto, o FAR inferior é evidência melhor e mais rápida que esperar o
IPC frontal: vem do mesmo frame em que a ausência local foi medida. Duas leituras
FAR trusted, com posição de banda finita, confirmam continuação. Uma leitura FAR
ou MEDIUM isolada mantém `gap-forward` e não possui autoridade de steering. Ela
só participa da reaquisição antecipada quando o Fusion seleciona um contorno
com target finito e estabilidade temporal. Sua última direção continua disponível
para o recovery caso a decisão chegue a LOST. O FAR não substitui
`nearLinePresent`, não rearma presença local e não interfere em LINE normal.

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
posteriores apenas moveram esse ramo entre arquivos. A correção atual restaura o
avanço reto enquanto existe apenas evidência. Quando há uma trajetória Fusion
inferior estável, o próprio Fusion reassume; FAR/MEDIUM nunca comandam uma roda
diretamente e continuam como evidência e memória para LOST.

Em LOST, o novo argumento do mapper impede que Fusion ou pixels soltos cancelem
a busca antes do reencontro local confirmado. Na recuperação, o gate libera o
tracker e o mapper original volta a decidir, inclusive com fita lateral.

Sem histórico recente de fita local, não se inventa um GAP no startup. Nesse
caso a lógica normal/recovery já existente continua decidindo. Um robô colocado
parado diretamente num vazio não fornece a transição presença → ausência.

`gap_entry_is_required` possui parâmetros explícitos de presença física usados
pela aplicação. A assinatura antiga foi mantida para consumidores/testes legados;
o caminho legado não é chamado pelo gate atual e não decide sobre o novo IPC.

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
são independentes dos sensores virtuais e do PID. Não há hot reload; uma instalação
futura deve atualizar os processos correspondentes juntos.

| Parâmetro | Inferior | Frontal |
|---|---:|---:|
| `bands` / `min_bands` | 7 / 3 | 7 / 3 |
| `min_width` / `max_width`, fração da largura | 0,025 / 0,24 | 0,008 / 0,15 |
| Espessura equivalente na resolução atual | 12–115 px em 480 | 7,68–144 px em 960 |
| `min_extent`, fração da largura | 0,10 (48 px) | 0,07 (67,2 px) |
| `min_elongation` | 1,3 | 1,4 |
| `clipped_elongation` | 0,50 | 0,75 |
| `near_fraction` / `near_bands` | 0,60 / 3 | 0,60 / 2 |
| `max_slope`, px transversais por px longitudinal | 3 | 3 |
| `max_missing_bands` / `max_components` | 1 / 16 | 1 / 16 |
| `uncertain_threshold` | 0,60 | 0,60 |

Os limites de espessura e extensão são proporcionais à resolução. Reduzi-los
aceita fragmentos menores; aumentá-los perde pontas legítimas e fita lateral.
Os filtros de alongamento ajudam contra blobs, mas precisam considerar bordas.
A largura inferior inicial de 0,18 rejeitou fita real larga das capturas salvas;
0,24 preservou esses casos. A amostragem inicialmente interna ao componente
oscilou entre 48 UNCERTAIN e 42 PRESENT num gap com ponta cortada. Incluir os
extremos observáveis e tratar o corte pelo FOV produziu PRESENT em 90/90, sem
passar as partículas sintéticas. Esses dados participaram do desenvolvimento.

`GAP_VALIDATION_CONFIG`: `near_present_frames=2` arma; `near_loss_frames=2`
confirma perda; `bottom_far_present_frames=2` confirma GAP curto;
`bottom_fusion_reacquire_frames=2` libera o target Fusion distante estável;
`near_history_seconds=0.30`; `forward_present_frames=2` confirma
frontal; `source_timeout=0.125`; `confirmation_seconds=0.25`;
`evidence_grace_seconds=0.20`; `max_gap_seconds=1.5`. Tempos em segundos.
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

[Resultados por frame, configurações e hashes](gap-presence-evidence/replay.json).
São dados estáticos da calibração anterior, incluindo reprocessamentos com o
perfil 61/min40/verde; não são novas travessias nem validação independente da
transição GAP/LOST. Nove sequências ainda contêm fita local verdadeira. Confirmar
presença nelas não significa que cada componente aceito seja fita.

## Testes e performance

A suíte local passou: **324 testes Python, 12 testes de calibração e os seis
alvos CTest**, além do build completo. Testes novos executam o mesmo
`GapValidator.process_frame` usado pela aplicação, seguido pelo controlador
inferior real: reta e curvas com comandos idênticos; fita lateral/horizontal;
micropartículas; Fusion válido por fragmento; perda em dois frames novos;
frontal inclinada/deslocada; ausência prolongada; retorno da linha;
FAR inferior central/lateral num GAP curto; alternância rápida dos lados do FAR;
frames repetidos/antigos/futuros;
prazo absoluto; verde e trackers especiais.

No benchmark local (Windows 11, CPU Intel, Python 3.12.10, OpenCV 5.0.0, uma
thread, 960×540, dez aquecimentos e 100 frames por caso), segmentação e análise
frontal tiveram:

| Caso | Mediana, ms | P95, ms |
|---|---:|---:|
| Reta | 1,878 | 2,387 |
| Curva forte | 1,866 | 2,418 |
| Dois caminhos | 2,188 | 3,114 |
| Interseção | 1,773 | 2,690 |
| Lateral | 1,200 | 1,706 |
| Inclinada | 1,484 | 1,974 |
| Partículas | 0,778 | 1,053 |
| Dez caminhos | 5,059 | 7,750 |

O detector local inferior acrescentou mediana de **0,507 ms**, P95 de 0,734 ms.
A referência opcional acrescentou mediana de 0,218 ms. O tempo da presença está
incluído em `lineProcessingMs`. [Benchmark completo](gap-presence-evidence/benchmark.json).
O processo usa componentes e sete bandas em duas orientações, sem ML ou novas
dependências. Não há previsão de runtime garantido no Raspberry: captura, IPC,
JPEG, streaming e detector de bolas não estão nessa medição.

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
   Reapresentar fita inferior por dois frames e conferir a devolução ao normal.
5. Somente após os ensaios estáticos, verificar movimento com o procedimento
   existente de potência reduzida, emergência acessível e novos trechos reais.
   Medir falsos GAP, perdas corretamente encaminhadas, atraso e FPS no Raspberry.

Segurança no código: emergência para motores **sim**; timeout de comandos
preservado **sim**; clamp preservado **sim**; pinos centralizados e inalterados
**sim**. Comentários novos em português, acentos e propósito revisados **sim**.
Nenhum serviço, GPIO ou comando de movimento foi alterado/executado nesta tarefa.
