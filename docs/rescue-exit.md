# Saída fixa e retomada do percurso

A missão principal inicia a saída após a varredura final do resgate. Não executa outra ré nessa transição. O modo **SAÍDA · BUSCAR E RETOMAR PERCURSO** usa a mesma rotina e também continua pelo percurso após adquirir a linha.

## Sequência atual

1. Girar para `kRescueExitDirectYawDegrees`, relativo ao heading salvo do último triângulo centralizado. No teste isolado, usar o heading salvo pelo alinhamento ou, sem ele, o primeiro yaw válido da partida.
2. Avançar `kRescueExitFrontGuidanceStartCm`, inicialmente **30 cm**, com as duas potências iguais. A menor distância acumulada das rodas determina o avanço; giro e deslocamentos anteriores não entram nessa medida.
3. Após os 30 cm, corrigir pelo ângulo Fusion atual da CAM1, inclusive quando a fita estiver distante. Não exigir profundidade de 85% na imagem. A convenção é: 90° reto, menor que 90° esquerda, maior que 90° direita. A correção continua leve, com potência base 0,75 e limites atuais de 0,78 na roda externa e 0,70 na interna.
4. Sem Fusion frontal válido, avançar reto e continuar verificando as duas câmeras. Uma nova imagem sem alvo cancela a correção anterior. Apenas preto visível, fita transversal, cor ou textura cinza ruidosa não autorizam uma correção.
5. Retomar a CAM0 após quatro imagens novas consecutivas com fonte `fusion`, comando normal válido e potências finitas. T, X ou Y não impedem essa entrega: a saída fixa já define a rota. Leitura vencida ou comando inválido reiniciam a confirmação; IPC repetido não conta como imagem nova.

A saída não executa pivôs de busca, uma reta adicional de 3 cm, varreduras por setores ou tentativas em outros yaws. Os testes específicos de calibração de yaw, incluindo ré de 40 cm + testar yaws, permanecem separados.

## Overlay frontal

No modo isolado, o stream da CAM1 mostra o overlay tanto durante a saída quanto após a CAM0 assumir. Em **REAL**, ele aparece sobre a imagem; em **LINHA**, mostra a máscara preta exclusiva da saída. A linha magenta liga o centro inferior ao alvo Fusion; a linha laranja mostra a continuação preta observada.

- `CONTROL CAM1`: o Fusion frontal corrige o robô após a reta inicial.
- `CONTROL INITIAL_STRAIGHT`: a distância inicial ainda não terminou.
- `CONTROL STRAIGHT_NO_FUSION`: avanço reto sem alvo Fusion frontal válido.
- `CAM0 CONFIRMING frames n/4`: confirmação inferior em andamento.
- `CAM0 STALE`, `INVALID_COMMAND` ou `SOURCE_gap-forward`: motivo pelo qual a CAM0 ainda não pode assumir. A fonte real aparece também ao lado de `CONTROL`.
- `CONTROL CAM0 / COURSE` e `HANDED_OFF`: a CAM0 já assumiu o percurso. A máscara frontal continua sendo diagnóstico, sem comandar motores nem suspender o tracker normal de GAP.
- `CONTROL STOPPED`: falha da saída. A causa aparece na última linha do overlay.
- `NO FUSION n/60 cm`: distância adicional sem Fusion válido de nenhuma das câmeras.

O heartbeat separa `enabled` (análise de controle da saída) de `exitOverlayEnabled` (visualização). Ambos exigem timestamp atual e execução válida. O overlay isolado permanece após uma falha, mas E-Stop e Stop normal encerram essa visualização.

## Segurança e configuração

As constantes ficam em `include/obr/config.h`. `kRescueExitFallbackMaximumAdvanceCm`, inicialmente **60 cm**, limita o avanço adicional sem Fusion das duas câmeras após a reta inicial. Um Fusion atual reinicia esse limite. Ao atingir o limite, o robô para com falha, sem procurar outra saída. O timeout total permanece em 120 segundos.

Imagem vencida ou indisponível zera as potências enquanto a câmera tenta recuperar. IMU, encoders ou ESP32 indisponíveis também param o movimento. Obstrução visual, rodas sem progresso e reinício do ESP32 encerram a saída. `RobotState` e o controle de motores preservam E-Stop, timeout de comandos e clamp.

Os 30 cm são medidos por encoder e dependem da calibração e do escorregamento das rodas.

## Verificação

No Windows, com CMake e MinGW disponíveis:

```powershell
cmake -S . -B build/rescue-fixed-mingw -G "MinGW Makefiles" -DBUILD_TESTING=ON
cmake --build build/rescue-fixed-mingw --target robot_test rescue_exit_mission_test main_mission_test -j 4
ctest --test-dir build/rescue-fixed-mingw -R "^(rescue_exit_mission_test|rescue_exit_integration_test|main_mission_test)$" --output-on-failure
python -m unittest discover -s tests/python -p test_rescue_exit.py
python -m unittest discover -s tests/python -p test_forward_camera_stream.py
```

Teste manual, primeiro com rodas suspensas e depois com potência reduzida no chão:

- Confirmar que o giro não conta nos 30 cm e que nenhum Fusion assume antes dessa distância.
- Usar uma continuação diagonal distante, com transversal ou cruzamento: conferir alvo magenta e `CONTROL CAM1` após os 30 cm.
- Confirmar `frames 1/4` até `4/4` e entrega à CAM0 mesmo em um T.
- Retirar a linha: conferir avanço reto, diagnóstico de bloqueio e parada no limite de fallback.
- Testar perda de câmera, E-Stop e Stop seguido de nova partida; a distância deve reiniciar.

Para atualizar o Raspberry Pi, usar o deploy existente: `./scripts/deploy.ps1`. Ver os logs com `journalctl -u obr-robot -f`.
