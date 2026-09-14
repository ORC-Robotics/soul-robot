# Busca da saída

A missão principal inicia a busca após `RescueRoomMission` concluir a varredura
final. Não executa outra ré nessa transição. A ré após a entrega continua sob
controle da missão de resgate, com as distâncias já configuradas.

A CAM1 procura preto em cinco setores e três faixas de profundidade. Após três
imagens novas consistentes, o robô se orienta e aproxima. A CAM0 confirma a prata
ou uma fita contínua sem ramificações. Quatro imagens inferiores válidas, com
novas inferências de prata, permitem retomar o seguidor normal. Prata tem
prioridade sobre a fita. Cruzamentos, bifurcações e caminhos concorrentes não
confirmam a saída; curvas com um caminho único são permitidas.

## Operação

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
- A visão reutiliza a captura, a segmentação preta, o classificador de prata,
  a conectividade FAR/MID e a análise existente de fita. A verificação adicional
  de ramificações afina uma máscara reduzida usando apenas NumPy e OpenCV.
- `cameraObscured` reutiliza exatamente a função e os limites da aproximação
  dos triângulos: fração escura de 70% com luminância até 60, ou desvio padrão
  de luminância até 8. Na saída, esse sinal interrompe e rejeita a tentativa;
  não executa o avanço final de entrega dos triângulos.
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
| Potência de giro | Referência à busca de vítimas: 0,72 |
| Potência de aproximação | Referência ao MID dos triângulos: 0,75 |
| Correção diferencial máxima | Referência aos triângulos: 0,06 |
| Limite de recuo | Menor avanço dos dois encoders, limitado a 40 cm |
| Tolerância de direção rejeitada | ±15°, incluindo 0°/360° |
| Perda da candidata | Para imediatamente; rejeita após 500 ms |
| Falta de sensores necessários | Para imediatamente; falha após 2 s |
| Tempo de tentativa / busca total | 10 s / 120 s |
| Rodadas | Duas; somente rejeições temporárias são liberadas uma vez |

As constantes específicas de análise de imagem ficam junto ao algoritmo em
`scripts/vision/rescue_exit.py`, seguindo os módulos Python de visão existentes.
A exigência visual cresce ao alcançar MID e NEAR e não diminui novamente na
mesma tentativa. A associação angular evita selecionar novamente o maior blob
durante a aproximação. Heading identifica uma direção aproximada, não uma
posição no mapa; o recuo não promete retornar ao centro geométrico da sala.

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

Na validação local desta implementação, a compilação, os testes de saída e
129 testes Python passaram. A suíte C++ geral encontra uma falha anterior em
`testObstaclePausesIfBottomCameraBecomesUnavailable`, reproduzida também no
executável anterior à alteração: o teste espera uma pausa na centralização,
mas recebe a parada `camera_not_ready`. O comportamento do desvio não foi
alterado nesta entrega.

Treze testes Python que usam diretórios temporários não puderam ser repetidos
na sandbox do Windows. A revisão automática bloqueou a repetição fora da
sandbox por limite de uso; execute os comandos acima em um terminal com acesso
aos diretórios temporários para completar essa verificação.

## Checklist de bancada

1. Sem motores conectados, observar os setores e conferir o sinal dos ângulos:
   esquerda negativa e direita positiva. Confirmar que uma candidata atrás
   aparece durante a varredura.
2. Com rodas suspensas, conferir giro a 0,72, avanço e recuo. Cobrir a lente
   frontal durante a aproximação: deve parar e iniciar o recuo limitado.
3. Apresentar prata à CAM0 junto de uma faixa preta. O robô deve frear no indício,
   confirmar a prata e bloquear aquela direção. Uma faixa preta em T/X/Y não
   deve confirmar saída.
4. Interromper CAM0, CAM1 ou telemetria da ESP32. Os motores devem parar. Acionar
   emergência durante busca, giro, aproximação e ré; nenhuma etapa pode retomar
   sozinha após Stop.
5. No piso, validar primeiro em área livre e com supervisão. Ajustar potências
   somente respeitando os pisos operacionais já calibrados. Conferir a distância
   real da ré em tentativas curtas e longas, incluindo a inércia de frenagem.
6. Ensaiar sala com entrada prata, saída atrás, falso candidato perto da parede
   e câmera coberta. Confirmar apenas uma rodada adicional de rejeições
   temporárias e parada após esgotamento ou tempo total.
7. Executar o resgate completo e conferir que a saída começa após a varredura
   final sem nova ré de entrada. Confirmar continuidade estável no seguidor.

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
