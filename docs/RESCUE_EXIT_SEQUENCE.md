# Sequência da saída normal do resgate

Depois do depósito final, a saída normal executa:

1. Alinhamento ao yaw registrado na entrada pelo prata + 90°. O alinhamento usa o menor giro até esse alvo absoluto; não soma 90° à orientação do último depósito.
2. Avanço de 60 cm, medido pelo menor avanço dos dois encoders. Sem medição ou progresso, encerra a etapa após o prazo configurado, atualmente 4 segundos.
3. Ré por 400 ms e novo avanço por 2 segundos, somente por tempo, sem exigir câmera, IMU ou progresso dos encoders. Ambos usam `kRescueExitExplorationPower`, atualmente 0,90, com sinal negativo na ré. Um ciclo com potência zero separa a inversão de sentido.
4. Parada de 300 ms e zero local de yaw na orientação estabilizada junto à parede. O segundo giro busca o alvo local negativo configurado, atualmente −85°. O yaw global da ESP32 permanece igual; cada leitura é convertida para o referencial local desta etapa.
5. Avanço com a busca Fusion existente: a CAM1 corrige com evidências atuais e a CAM0 assume após quatro imagens novas consecutivas válidas.

A reentrada na sala com resgate já concluído mantém sua entrada curta, seu giro e sua reta anteriores.

## Ajustes em `include/obr/config.h`

| Constante | Valor | Uso |
| --- | --- | --- |
| `kRescueExitFromEntryYawDegrees` | 90° | Deslocamento à direita da referência do prata |
| `kRescueExitFromEntryYawToleranceDegrees` | 5° | Tolerância dos dois giros normais |
| `kRescueExitCrossingCm` | 60 cm | Travessia antes do giro à esquerda |
| `kRescueExitWallReverseMs` | 400 ms | Ré após a travessia ou seu timeout |
| `kRescueExitWallAdvanceMs` | 2000 ms | Novo avanço antes de registrar o zero local |
| `kRescueExitYawZeroSettleMs` | 300 ms | Pausa parada antes de registrar o zero local na parede |
| `kRescueExitLeftTurnDegrees` | 85° | Módulo do alvo negativo no referencial local da parede |
| `kRescueExitNormalTurnTimeoutMs` | 10000 ms | Prazo máximo por giro; expirar libera a próxima etapa |
| `kRescueExitTurnCorrectionMaximumMs` | 4000 ms | Prazo da correção fina já existente |
| `kRescueExitTimedQuarterTurnMs` | 1500 ms | Estimativa para 90° sem IMU; calibrar no robô |
| `kRescueExitCrossingTimeoutMs` | 4000 ms | Prazo da travessia sem distância confirmada |
| `kRescueExitNormalSearchTimeoutMs` | 12000 ms | Prazo da busca em movimento |

Falhas de IMU, encoder ou câmera não marcam a saída normal como falha. Giros incompletos liberam a próxima etapa ao atingir o prazo. A perda da IMU não reinicia o relógio do giro. Sem referência de entrada, usa o yaw atual quando disponível; sem IMU, usa tempo. Câmera obstruída ou vencida não fornece correção.

Sem sensores, os 90° e os 60 cm são estimativas por tempo e dependem de potência, bateria e piso. Sem linha confirmada, a busca para após 12 segundos ou após o limite de avanço sem Fusion. A missão permanece ativa e aceita uma confirmação posterior da CAM0. Comunicação com a ESP32, driver habilitado e ausência de emergência continuam necessários para movimento real.

## Compilar, testar e aplicar

No Windows, usando o diretório de build MinGW existente:

```powershell
cmake --build build/rescue-fixed-mingw --target robot_test rescue_exit_mission_test main_mission_test -j 4
ctest --test-dir build/rescue-fixed-mingw -R '^rescue_exit_(mission|integration)_test$' --output-on-failure
.\scripts\deploy.ps1
```

O deploy precisa ser executado para aplicar a alteração na Raspberry Pi.

## Teste manual

- Com as rodas suspensas, conferir o sentido dos dois giros e acionar E-Stop durante cada etapa.
- Em potência reduzida, conferir o alvo relativo ao prata e medir a travessia no chão.
- Conferir a ré de 400 ms, o novo avanço de 2 segundos, a parada de 300 ms na parede e o segundo alvo local configurado, inclusive com yaw global próximo de ±180°.
- Acionar E-Stop durante a ré e durante o novo avanço e conferir a parada dos dois motores.
- Indisponibilizar a medição da IMU e dos encoders e conferir a progressão pelos prazos, sem falha da missão.
- Cobrir a câmera durante a travessia; após o giro à esquerda, conferir a busca temporizada e a parada sem confirmação de linha.
- Recuperar a CAM0 e conferir o retorno ao seguidor após quatro imagens novas válidas.
- Repetir a reentrada na sala concluída e conferir que a rota anterior permanece igual.

Safety check:
- Motors stop on emergency stop: yes.
- Motors stop on command timeout: yes; watchdog e timeout dos comandos preservados.
- Motor output clamped: yes.
- Pins centralized in config.h: yes; nenhum pino alterado.

Comment quality check:
- Comments are in Brazilian Portuguese: yes.
- Spelling and accents were reviewed: yes.
- Comments explain purpose, effect, or safety risk: yes.
- Comments avoid obvious noise: yes.
