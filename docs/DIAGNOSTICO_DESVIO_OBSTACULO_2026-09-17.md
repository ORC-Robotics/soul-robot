# Diagnóstico do desvio em 17/09/2026

Este relatório descreve o estado e os logs anteriores à restauração da escolha
automática do lado. A sequência implementada depois deste diagnóstico está em
`docs/OBSTACLE_AVOIDANCE.md`; os valores abaixo foram preservados como evidência
histórica.

## Estado verificado

A consulta foi feita por SSH, sem iniciar movimento, reiniciar serviços ou
alterar arquivos da Raspberry. `obr-robot` e `obr-line-camera` estavam ativos,
com o robô em modo parado. O processo C++ usava `build/robot_test`, modificado
às 09:51:53, e o serviço iniciou às 09:51:56, no horário de Fortaleza.

Os SHA-256 de `config.h`, `obstacle_avoidance.cpp`, `line_course_mission.cpp`,
`application.py` e `camera_config.py` coincidiram entre o computador e a
Raspberry. Os eventos do serviço também correspondem ao perfil fixo atual.
A coincidência dos fontes não constitui, sozinha, uma prova do conteúdo do binário.

| Parte | Comportamento atual |
|---|---|
| Detecção | Até 6 cm; duas chamadas elegíveis; rearmamento após três chamadas a pelo menos 15 cm |
| Ré inicial | Alvo de 5 cm, potência 0,75, máximo de 1500 ms; transição para centralização se encoders ficarem antigos |
| Referência | Centralização pela CAM0; IMU salva `yawBase` |
| Lado | LEFT forçado por configuração; sem medir os lados |
| Aproximação | Alvo `yawBase - 40°`; avanço de 12 cm a 0,75, correção de heading até 0,04 |
| Contorno | 20 cm medidos pela menor distância das rodas; yaw alvo interpolado até `yawBase + 45°`; correção até 0,06 |
| Saída | Pausa de 250 ms; giro relativo de 25° à direita; reta por até 1300 ms |
| Busca | Giro à direita a `+0,73 / -0,73` por até 2000 ms; sem limite angular específico nessa fase |
| Confirmação | Três frames novos da CAM0 com `fusion` e `normalSteeringValid` |
| Segurança | Validade dos sensores, limites de potência, E-Stop, Parar e watchdogs continuam necessários |

As câmeras inferior e frontal estavam processando perto de 30 FPS. A CAM1
funciona como fonte auxiliar geral, mas o perfil fixo não consulta sua geometria
frontal para confirmar a saída. Durante o controle do obstáculo, o Forward Assist
é reiniciado e seus comandos não são aplicados aos motores.

O modo adaptativo antigo contém medição lateral, recuperação antecipada,
memória da parábola e proteção pós-desvio do caso 3. Com o perfil fixo, a curva
não coleta essa memória; a conclusão da saída a apaga. Assim, os limites antigos
de 35°, 65° e 100° não protegem a busca final do perfil fixo.

`obstacleContinuationBand` também não participa dessa saída. Esse campo indica
cobertura das três colunas pela união de MEDIUM e FAR; não identifica sozinho
o sentido correto de percurso.

## Execução mais recente com reencontro declarado

Horário local: 17/09/2026, 10:20:04; sequência autônoma 11. Esta é a última
tentativa encontrada hoje que declarou reencontro após a busca à direita e,
imediatamente depois, inverteu o sentido de giro dos comandos.

| Horário | Evento confirmado pelo registro |
|---|---|
| 10:20:04,452 | `obstacle_detected`; comandos zerados |
| 10:20:05,333 | LEFT selecionado após ré e centralização |
| 10:20:06,342 | Avanço inicial |
| 10:20:07,080 | Curva do contorno |
| 10:20:08,515 | Fim da curva; pausa |
| 10:20:08,803 | Giro de saída à direita |
| 10:20:09,748 | Avanço de saída |
| 10:20:11,061 | Busca à direita: L=+0,730; R=-0,730 |
| 10:20:12,229 | Fonte visual passa a `fusion`; busca ainda gira à direita |
| 10:20:12,311 | `obstacle_exit_reacquired`; módulo zera seus comandos |
| 10:20:12,331 | `line_following`: L=-0,156; R=+0,814; giro à esquerda |
| 10:20:13,724 | Modo parado; journal registra acionamento físico do botão Start |

No primeiro comando do segue-linha, o trace registra saída esquerda de -0,670.
O mínimo de partida do motor aumenta a magnitude do comando visual de -0,156.
O CSV confirma steering -0,844 no frame 46056, seguido de correções negativas
fortes. Depois, os comandos voltam a ser positivos nas duas rodas.

Isso prova a inversão dos comandos e uma aceitação de Fusion com correção
forte. Não prova uma meia-volta física completa, o ângulo percorrido ou qual
trecho da faixa foi visto. Esses pontos exigem vídeo e yaw sincronizados.

## Tentativa seguinte: a última execução encontrada

Iniciou às 10:20:28,742, sequência 12. Completou ré, posicionamento, reta e
curva. O giro de saída teve várias correções para a esquerda entre pulsos de
estabilização. Começou o avanço de saída às 10:20:37,810 e a busca à direita
às 10:20:39,125. Parou às 10:20:40,439, antes de completar dois segundos de busca;
o journal registra o botão físico Start. Não declarou reencontro da faixa.

As sessões de vídeo mais recentes em `logs/forward_reacquisition` são de
11/09/2026, anteriores a essas tentativas. Não há gravação desse contorno de
17/09 nesse diretório. O CSV de curvas só grava `line_following`, não todas as
fases do desvio; seu timestamp também tem precisão insuficiente para substituir
o trace por ciclo. A telemetria no journal é espaçada e não fornece yaw contínuo.

## Causa provável e limites do diagnóstico

O contorno termina pela distância, sem exigir erro final de yaw pequeno. Em
seguida, acrescenta 25° ao yaw atual, avança por tempo e procura sempre à
direita. Essa busca pode atravessar a direção frontal e chegar a setores onde
uma faixa anterior fica visível, pois só possui limite de tempo.

A confirmação aceita qualquer Fusion válido por três frames novos. Não exige
alinhamento com a direção de saída, concordância frontal da CAM1 ou rejeição
de uma faixa traseira. A autoridade então passa imediatamente ao segue-linha,
que pode aplicar um pivot forte. Essa fragilidade explica o comportamento
observado nos comandos; identificar visualmente a faixa errada continua sendo
uma hipótese, não uma conclusão comprovada.

## Mudanças propostas, em ordem

1. **Registrar a saída inteira.** Incluir fase, yaw base/atual/alvo, erro angular,
   distância, fonte visual, steering, votos, motivos de rejeição e comandos
   solicitados/aplicados. Associar imagens CAM0/CAM1 aos eventos. O gravador
   atual da CAM1 já permite iniciar a coleta, mas seus dados de controle
   precisam ser ampliados para explicar a aceitação da CAM0.
2. **Limitar a busca por ângulo e tempo.** Salvar a referência de saída e um
   setor frontal admissível. Parar antes de entrar no setor traseiro. Calibrar
   o setor com yaw real e vídeo; não copiar automaticamente os limites do modo
   adaptativo antigo, que usam outras referências.
3. **Confirmar direção, além de presença.** Manter três frames novos, exigir
   geometria compatível com a continuação e heading admissível. Avaliar apoio
   da CAM1 quando seus dados estiverem recentes e confiáveis; massa preta
   isolada não identifica uma linha nem seu sentido de percurso.
4. **Alinhar antes de devolver o controle.** Após um candidato válido, parar,
   alinhar e confirmar novamente. Evitar entregar os motores enquanto o Fusion
   pede pivot forte, como no frame 46056. Manter uma proteção temporária de
   heading após a transferência, com parada caso ultrapasse o setor permitido.
5. **Recalibrar a geometria somente com essas evidências.** Conferir yaw real
   no fim dos 20 cm, posição do robô e utilidade do giro relativo de +25°.
   Preferir saída limitada por distância e heading, mantendo timeout como
   proteção. Não aumentar tempos nem inverter o lado por tentativa.

As primeiras alterações comportamentais devem ficar em `ObstacleAvoidance`,
com constantes em `config.h`. A integração de transferência de autoridade fica
em `LineCourseMission`; mudanças de visão só entram se os registros mostrarem
que os campos existentes não bastam. Não é necessária uma biblioteca nova.

## Validação das próximas alterações

- Regressões sem hardware: rejeitar Fusion fora do setor frontal; parar no
  limite angular mesmo antes do timeout; rejeitar dados antigos; confirmar
  frames distintos; impedir transferência com pivot forte.
- Compilar localmente e executar os testes do obstáculo e da missão principal.
- Testar E-Stop, Parar, perda da ESP32/visão/IMU/encoders e ausência da faixa.
- Testar com rodas suspensas antes da pista; verificar uma potência reduzida
  que ainda vença o atrito dos motores.
- Na pista, variar a chegada diagonal e repetir o mesmo obstáculo gravando a
  saída. Confirmar continuação para a frente e ausência de retorno ao trecho anterior.

Nesta investigação, apenas a documentação foi atualizada. A lógica de movimento,
as calibrações e o serviço da Raspberry não foram alterados.

Evidências locais preservadas em `logs/obstacle_audit/journal_analysis.json`,
`logs/obstacle_audit/curve_diagnostics.csv` e `logs/obstacle_audit/timeline.csv`.
O diretório `logs/` é ignorado pelo Git; esses arquivos não entram em commits.
