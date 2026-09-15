# Busca da saída

A missão principal inicia a busca após `RescueRoomMission` concluir a varredura
final. Não executa outra ré nessa transição. A ré após a entrega continua sob
controle da missão de resgate, com as distâncias já configuradas.

## Busca geométrica pelos corners

O heading travado na centralização do último triângulo, vermelho ou verde,
define o yaw relativo 0°. Na missão principal, ele permanece salvo mesmo se a
verificação final de vítimas girar o robô após a ré. No modo isolado, o yaw no
instante da partida assume essa referência. A busca aponta para +100°, +170°
e −96° relativos a ela. Se confirmar um triângulo vermelho ou verde em qualquer
alvo, descarta aquela direção e passa à próxima. Após três imagens novas sem
triângulo confirmado, gira para o mesmo yaw e inicia a reta, sem acréscimo
angular. Headings já rejeitados são pulados; o retorno restaura o yaw 0° relativo.

Cada tentativa retorna pela distância realmente medida nos encoders e restaura
o yaw da origem antes de apontar para o segundo corner. Após três imagens novas
sem triângulo confirmado, o robô avança reto naquele corner mesmo que ainda não
exista Fusion. A CAM1 pode confirmar visualmente a faixa antes desse avanço, mas
não corrige os motores durante o trajeto. Quando a CAM0 vê preto próximo sem
`Fusion`, o robô pausa a reta e executa dois pivôs contínuos com potência 0,75.
O yaw do corner escolhe o primeiro sentido: em +170° (quina direita), começa
girando à esquerda; em +100° ou -96° (lado esquerdo), começa à direita. Após
650 ms, inverte imediatamente por mais 650 ms. A CAM0 procura `Fusion` novo durante
todo o giro; se não houver, recua e tenta outro yaw.
O `Fusion` só conta quando a CAM0 também confirma uma faixa única, sem T, X ou Y.
No primeiro `Fusion` válido, o robô avança 3 cm reto, medidos pelos dois
encoders. Depois volta aos pivôs; se a CAM0 ainda vê a faixa única, ela
passa a guiar as rodas. Quatro frames novos iniciam a validação da saída;
se a faixa aparecer como T ou o controle passar a `fusion-gap-reacquire`
durante o cruzamento, isso não cancela a validação, mantendo a prioridade da prata.
Enquanto avança no corner, a análise pesada
de candidatas frontais também fica suspensa; permanece apenas a proteção visual.
Isso evita oscilações entre headings frontais e reduz o uso de CPU.
Se a geometria não puder ser confirmada, se os três yaws falharem ou se o
retorno perder a referência, o robô volta ao yaw central salvo e reinicia a
própria sequência geométrica. A varredura angular anterior não é mais ativada.
Bloqueios temporários são liberados; entrada prata e triângulos confirmados
continuam proibidos. Após confirmar prata, a missão permite concluir somente
o recuo e os giros de reposicionamento; um novo avanço continua bloqueado se
a prata permanecer sob a CAM0.

Na missão completa, os alvos são testados na ordem +100°, +170° e −96° desde
o último triângulo centralizado. A entrada prata não muda essa ordem.

A CAM1 procura preto por contraste local no frame inteiro e organiza as
continuações em cinco setores e três faixas de profundidade. Componentes muito
pequenos, frestas com pouco suporte por altura e traços quase horizontais são
descartados. Estruturas que nascem no topo e permanecem presas à lateral também
são tratadas como moldura ou parede. Quando há mais de uma rota válida, a
continuidade, a estabilidade da largura e uma preferência moderada pela direção
frontal evitam trocar a fita por uma quina distante; uma saída lateral continua
válida quando for a única opção. Cada candidata publica um
ponto `near` no centro inferior, um ponto `entry` no começo próximo da faixa e
um ponto `far` na parte superior robusta do traçado. A altura normalizada de
`entry` indica quando a fita chega perto: antes de 0,85 da altura da imagem, o
robô avança reto; depois, corrige suavemente pela direção da continuação.
A mesma convenção angular do Fusion transforma os pontos em direção:
90° segue reto, abaixo de 90° corrige à esquerda e acima corrige à direita.

A primeira imagem válida interrompe imediatamente o giro de busca. Três imagens
novas consistentes confirmam a candidata, e o terceiro frame já inicia o avanço
frontal, sem pivô de alinhamento. Oscilações de score não reiniciam a confirmação.
Após ver uma candidata, a busca fica parada por até 800 ms para reobservar o
mesmo heading (±15°), mesmo que alguns frames não mostrem a fita. Outra rota
não toma seu lugar nesse intervalo. Se ela não reaparecer, a varredura só
recomeça após novas imagens vazias; um único frame não autoriza aproximação.
A estratégia geométrica mantém as pistas Fusion e os setores vetados somente
como diagnóstico da tentativa atual; ela não transfere o controle para a antiga
exploração angular. Nos corners geométricos, o avanço permanece reto até quatro
frames novos do Fusion inferior iniciarem a validação pela CAM0. Depois do deslocamento, o mapa
visual dependente da posição é reconstruído e a entrada prata continua bloqueada.
A CAM0 começa a guiar após a reta de 3 cm, com um novo frame inferior de fonte
`fusion`, direção normal válida, faixa sem ramificações e classificador de
prata recente. A missão de saída só
entra na validação após quatro frames e só termina depois de mais 25 cm
medidos pelos encoders; nesse trecho, qualquer
indício de prata para imediatamente e a confirmação rejeita o corner.
A CAM1 pode rejeitar preto refletivo antes do primeiro Fusion único. Depois que
a reta de 3 cm começa, somente o classificador de prata da CAM0 pode rejeitar
essa saída, evitando que ruído frontal desfaça uma confirmação inferior válida.
Depois da transferência, dois frames inferiores em `LOST` permitem à CAM1 girar
para o último lado frontal válido. Dois frames normais novos devolvem novamente
a autoridade à CAM0; essa assistência só existe no percurso posterior à saída.
Prata tem prioridade: um indício pausa o avanço e a confirmação rejeita
permanentemente uma faixa de ±35° em torno do heading acompanhado. A topologia
da fita impede que o Fusion de um T, X ou Y inicie a aquisição da saída.

## Operação

Para conferir somente a geometria, selecione **SAÍDA · TESTAR YAW DAS QUINAS**
no painel. Coloque o robô parado no ponto de referência do triângulo vermelho
ou verde e aponte-o para esse triângulo. Ao iniciar o modo Autônomo, o yaw atual
vira a referência: o robô gira para +100°, +170° e −96° relativos a ela, nessa
ordem, e fica parado por 2 segundos em cada direção. Depois da última pausa,
encerra parado. Usa exatamente os headings do avanço reto da saída.
O teste não avança, não procura fita ou triângulos e não move
servos. O yaw medido aparece no indicador **Giro integrado**; o estado da missão
mostra qual dos três alvos está sendo conferido. Stop e E-Stop interrompem o
teste. O comando de seleção é
`{"command":"set_autonomous_mission","mission":"rescue_corner_yaw_test"}`.
O tempo de pausa fica em `kRescueCornerYawHoldMs`, em `include/obr/config.h`.

No painel, selecione **SAÍDA · BUSCAR E RETOMAR PERCURSO** com o robô parado e
posicionado na sala. Inicie pelo controle autônomo existente. O comando de
seleção usa `{"command":"set_autonomous_mission","mission":"rescue_exit"}`; a seleção
não inicia motores. Ao adquirir a saída, o teste também continua seguindo a
linha. Use Stop ou a parada de emergência para encerrar.

A detecção da faixa vermelha final ainda não está implementada. Depois da
saída, uma nova leitura de prata não reinicia o resgate.

O painel e o overlay frontal mostram setor, confiança, heading em graus,
rodada, rejeições, distância avançada/recuada em centímetros e última falha.
O log registra as transições com o prefixo `Rescue exit:`. No Raspberry Pi:

```sh
journalctl -u obr-robot -f
```

## Controle e configuração

- `RescueExitMission` toma decisões e retorna potências. `MainMission` coordena
  a transferência ao `LineCourseMission`; `RobotState` mantém a autoridade
  sobre modo, emergência e timeout de comandos.
- `EncoderDistanceController` compartilha a preparação, o deslocamento, a
  estabilização e as proteções por encoders com `RescueRoomMission`.
- A visão reutiliza o detector calibrado `analyze_rescue_zones`, mas possui
  segmentação preta exclusiva. Setores ocupados por triângulos verdes ou
  vermelhos são vetados antes da escolha da rota. A conferência é repetida
  ao mudar de setor ou heading. O detector colorido separado é desligado durante
  o avanço de um corner já classificado e reativado ao retornar para classificar
  outro corner, reduzindo o custo de processamento da CAM1.
- A textura cinza refletiva é avaliada por continuidade, solidez transversal e
  fragmentação. Um preto muito ruidoso no yaw inspecionado é tratado como
  indício de prata: antes da reta, o yaw é descartado; durante o avanço, o
  robô para, recua a distância medida e tenta o próximo yaw. Isso também vale
  antes de o Fusion da CAM0 concluir a saída. Uma linha preta contínua não
  aciona esse veto. O overlay REAL e LINHA mostra as três medidas e o estado
  `COLOR CHECK/CLEARED/REARMED`.
- Para diagnóstico frontal, a orientação da candidata reutiliza
  `calculate_fusion_style_angle`. A fita
  transversal da entrada não produz orientação; o alvo vem da continuação que
  avança para o fundo da imagem. Enquanto `entryDepthNormalized` estiver abaixo
  de 0,85, as duas rodas recebem 0,75. Depois, a correção é limitada a
  0,78 / 0,70. Em um corner geométrico escolhido, essas curvas não comandam os
  motores: o avanço usa 0,75 / 0,75 até a CAM0 assumir. Nenhuma roda para ou
  entra em ré durante a perseguição.
- `cameraObscured` mantém a proteção de 70% de pixels escuros (luminância até
  60). Durante a busca, baixa variação de brilho sozinha não bloqueia a CAM1
  quando a imagem é clara e neutra, como o piso: ela precisa ser escura ou
  fortemente saturada. Na aproximação e na exploração, o critério conservador
  original continua valendo para parar diante de uma parede clara. A proteção
  dos triângulos não muda. Obstrução confirmada zera os motores imediatamente.
  Sem CAM1 nem Fusion inferior, a última curva dura no
  máximo 300 ms; depois o robô espera parado até 500 ms por uma leitura nova.
  Sem recuperação, recua somente a distância realmente avançada, limitada a
  40 cm, e tenta reencontrar a mesma faixa uma vez.
- Durante o avanço geométrico, obstrução ou falta de progresso para
  imediatamente. O robô recua a distância avançada, volta à referência e
  tenta o próximo yaw. Não há ultrassom nessa decisão.
- A leitura normal da CAM0 vence em 125 ms, mas a última classificação de prata
  possui janela própria de 500 ms. A indisponibilidade real da CAM0 em NEAR
  para os motores imediatamente e encerra a tentativa após 2 s.
- No modo de saída, dois frames positivos novos confirmam a prata/cinza. As
  demais missões preservam a confirmação original de quatro frames.
- A CAM1 tem janela exclusiva de 400 ms na saída; não altera o prazo global de
  125 ms. A última curva confirmada permanece por no máximo 300 ms sem frame
  correspondente; depois os motores param e, antes de NEAR, a tentativa é
  rejeitada após 500 ms sem recuperar a candidata.
- A aplicação publica um heartbeat atômico em
  `/dev/shm/obr_rescue_exit_control.json`. A visão exige idade até 500 ms e
  inclui a sequência da execução no IPC frontal. Dados ausentes ou inválidos
  deixam a saída sem autorização visual. Os contratos normais continuam válidos
  quando o produtor não possui os campos da saída.

Os parâmetros de movimento ficam em `include/obr/config.h`, no bloco
`kRescueExit*`. Valores iniciais:

| Parâmetro | Valor |
| --- | --- |
| Passo de busca | 30° à direita |
| Potência de giro | 0,75, exclusiva da busca da saída |
| Potência reta de aproximação | 0,75 / 0,75 |
| Início da correção | `entry` a partir de 85% da altura da CAM1 |
| Curva de aproximação próxima | Externa até 0,78; interna até 0,70 |
| Curva suave | Deadband 3,5°; autoridade máxima em 24° |
| Limite de recuo | Menor avanço dos dois encoders, limitado a 40 cm |
| Tolerância de acompanhamento | ±30° durante a tentativa |
| Tolerância da confirmação inicial | ±15° entre frames novos |
| Reobservação de candidata | Até 800 ms parada antes de voltar a girar |
| Yaws de busca desde o triângulo | +100°, +170° e −96°, sem acréscimo antes da reta |
| Pivôs de busca após preto próximo | 650 ms no sentido oposto ao lado definido pelo yaw e 650 ms contínuos no outro sentido, com potência 0,75 |
| Reta antes do controle pelo Fusion | 3 cm por encoders após a primeira faixa única válida |
| Tentativa por corner | 60 cm sem visão; com Fusion frontal recente, continua a 0,75 até a CAM0 |
| Parede ou travamento no corner | Para, recua o avanço medido e tenta o próximo yaw |
| Exploração por corner | Até 60 cm sem Fusion frontal recente |
| Tolerância de direção rejeitada | ±15° comum; ±35° após prata confirmada |
| Perda da candidata antes de NEAR | Mantém a última curva por até 300 ms, para e tenta reaquisição aos 500 ms |
| Perda dupla após NEAR | Para após 300 ms; espera até 500 ms e recua se não recuperar |
| Validade da prata | 500 ms, independente dos 125 ms da leitura normal |
| Validade da CAM1 durante a saída | 400 ms, sem alterar o timeout global de 125 ms |
| Recovery após a saída | CAM0 `LOST` por 2 frames; CAM1 gira até 65° ou 3 s; CAM0 reassume em 2 frames normais |
| Falta de sensores necessários | Para imediatamente; falha após 2 s |
| Tempo de tentativa / busca total | 20 s / 120 s |
| Estratégia de busca | Somente geometria entre os corners; sem fallback angular |

As constantes específicas de análise de imagem ficam junto ao algoritmo em
`scripts/vision/rescue_exit.py`, seguindo os módulos Python de visão existentes.
`depthBands` e `tapeValid` permanecem disponíveis para diagnóstico; a topologia
da CAM0 veta o Fusion ramificado na aquisição da saída. A associação angular de 30° evita
trocar a candidata durante a perseguição; os 15° de headings rejeitados continuam
independentes. Heading identifica uma direção aproximada, não uma posição no
mapa. No fluxo geométrico, o recuo usa o avanço medido para retornar
aproximadamente à origem salva.

Nenhum pino, serviço ou script de deploy foi alterado. Não há dependência nova
de execução. O ultrassônico não participa da busca ou confirmação da saída;
o seguidor normal retoma seu tratamento habitual de obstáculos após a aquisição.

## Testes automatizados

Compilação local usada no Windows:

```powershell
cmake -S . -B build/exit-validation -G "MinGW Makefiles"
cmake --build build/exit-validation -j 4
ctest --test-dir build/exit-validation --output-on-failure
```

No Linux, use o gerador padrão com `cmake -S . -B build`. Para executar somente
o novo controle e sua integração:

```sh
ctest --test-dir build/exit-validation --output-on-failure -R rescue_exit
python -m unittest discover -s tests/python -p test_rescue_exit.py
python -m unittest discover -s tests/python -p test_forward_camera_stream.py
python -m unittest discover -s tests/python -p test_forward_path_validation.py
python -m unittest discover -s tests/python -p test_silver_detection.py
python -m unittest discover -s tests/python -p test_rescue_zone.py
```

Os testes da câmera frontal substituem somente a construção do detector YOLO;
segmentação e geometria usam NumPy/OpenCV reais. Esses testes não validam a
inferência do modelo de vítimas.

Nesta máquina, o CMake está disponível, mas o compilador MinGW indicado no
cache não está instalado. Compile os testes C++ antes do ensaio com motores.
Os testes Python focados foram executados offline.

## Checklist de bancada

1. Sem motores conectados, observar os setores e conferir o sinal dos ângulos:
   esquerda negativa e direita positiva. Confirmar que uma candidata atrás
   aparece durante a varredura.
2. Com rodas suspensas, conferir giro a 0,75, avanço e recuo. Cobrir a lente
   frontal antes de NEAR: deve parar após 300 ms e rejeitar aos 500 ms. Depois
   de NEAR: deve seguir a evidência da
   CAM1 e, na perda simultânea da CAM1 e do Fusion, avançar reto por no máximo
   1 s. Sem recuperação, deve rejeitar e executar o recuo limitado.
3. Apresentar prata à CAM0 junto de uma faixa preta. O robô deve frear no indício,
   confirmar a prata e bloquear aquela direção. Com `fusion` e direção normal
   válidos, quatro frames novos devem iniciar a validação. A saída só deve ser
   entregue ao percurso após 25 cm adicionais sem indício de prata.
4. Interromper CAM0, CAM1 ou telemetria da ESP32. Os motores devem parar. Acionar
   emergência durante busca, giro, aproximação e ré; nenhuma etapa pode retomar
   sozinha após Stop.
5. No piso, validar primeiro em área livre e com supervisão. Ajustar potências
   somente respeitando os pisos operacionais já calibrados. Conferir a distância
   real da ré em tentativas curtas e longas, incluindo a inércia de frenagem.
6. Ensaiar sala com entrada prata, saída atrás, falso candidato perto da parede
   e câmera coberta. Colocar os triângulos verde e vermelho no campo de visão e
   confirmar que seus setores não viram candidatas de saída. Confirmar apenas
   uma rodada adicional de rejeições temporárias e parada após esgotamento ou
   tempo total.
7. Executar o resgate completo e conferir que a saída começa após a varredura
   final sem nova ré de entrada. Confirmar continuidade estável no seguidor.
8. No journal, confirmar que a primeira candidata interrompe `rescue_exit_turning`,
   que não aparece `rescue_exit_aligning` e que não há alternância rápida entre
   `rescue_exit_searching` e `rescue_exit_waiting_sensors`.

O aceite físico depende desses ensaios; imagens sintéticas não medem aderência,
inércia, iluminação real ou a taxa de processamento no Raspberry Pi.

### Safety check

- Motors stop on emergency stop: yes, preservado em RobotState/ESP32 e nos gates.
- Motors stop on command timeout: yes, fluxo existente preservado.
- Motor output clamped: yes, na saída e no deslocamento compartilhado.
- Pins centralized in config.h: yes, sem novos pinos.

### Comment quality check

- Comments are in Brazilian Portuguese: yes.
- Spelling and accents were reviewed: yes.
- Comments explain purpose, effect, or safety risk: yes.
- Comments avoid obvious noise: yes.
