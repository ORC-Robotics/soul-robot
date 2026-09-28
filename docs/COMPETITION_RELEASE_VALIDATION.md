# Validação da versão de competição

Revisão de encerramento realizada em 28 de setembro de 2026. O estado funcional
encontrado foi preservado: esta revisão altera documentação e consolida as
modificações de código e testes que já estavam pendentes.

## Escopo consolidado

- Configurações finais do desvio e da saída do resgate.
- Espera após a ré do obstáculo, contagem na OLED e telemetria correspondente.
- Reaquisição da linha após o contorno e sequência de saída junto à parede.
- Suporte ao gesto de partida, mantido desativado na configuração final.
- Limiar de confiança da vítima prata em 0,50.
- Testes pendentes das missões e documentação dessas rotinas.
- README reescrito e referências documentais corrigidas, sem mudanças funcionais
  adicionais para acomodar resultados de testes.

Não havia arquivos removidos. Builds, logs, caches, ambientes virtuais, datasets
e saídas de treinamento ficaram fora do commit. Os modelos de inferência já
versionados em `assets/models/` são recursos utilizados pelo robô.

## Validação local

Ambiente: Windows, MinGW/GCC 16.1.0, Python 3.14.0, OpenCV 5.0.0 e NumPy 2.5.2.

```powershell
cmake -S . -B build/final-review -G "MinGW Makefiles" -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build/final-review --parallel 4
ctest --test-dir build/final-review --output-on-failure
python -m unittest discover -s tests/python
git diff --check
```

| Verificação | Resultado |
| --- | --- |
| Configuração e compilação C++ completa | Sucesso, incluindo `robot_test` e os executáveis de teste |
| CTest | 16 de 17 testes passaram |
| Python unittest | 545 testes executados; 15 falhas e 1 erro reportados, incluindo subtestes |
| Links e caminhos locais do README | Conferidos |
| Revisão do diff | Sem descarte das alterações funcionais pendentes |

O teste C++ `obstacle_avoidance_test` interrompeu com a mensagem
“O giro adicional deve ser espelhado para dentro do contorno.” A suíte completa
desse executável, portanto, não foi validada. Os testes da missão principal,
saída e integração do resgate, servos e OLED passaram.

Na suíte Python, as divergências envolvem rastreamento de vítimas, limiar YOLO,
campos publicados no IPC, geometria/confirmação de verdes e rejeição de amostras
de cor na zona de resgate. O teste do limiar ainda espera 0,55, enquanto a
configuração final usa 0,50. O erro é um `NameError` para
`analyze_green_marker_contours` em `test_camera_profiles.py`. Não se atribui
automaticamente todas as falhas a testes obsoletos: permanecem registradas sem
alterar o comportamento de competição.

A primeira execução Python sofreu restrições de acesso a diretórios temporários
do sandbox. A execução repetida sem essa restrição produziu os resultados acima.
Os logs locais estão em `build/final-ctest.log` e
`build/final-python-tests-unrestricted.log`, deliberadamente não versionados.

Não foram executados deploy, gravação da ESP32, testes físicos, captura real ou
inferência dos modelos nas câmeras da Raspberry. A validação local não equivale
a uma nova homologação da montagem de competição.

## Segurança preservada

Conferência do código, sem novo ensaio físico:

```text
Safety check:
- Motors stop on emergency stop: yes
- Motors stop on command timeout: yes
- Motor output clamped: yes
- Pins centralized in config.h: yes; Raspberry em include/obr/config.h e firmware em robot_config.h
```

Esses itens se referem à tração. `SERVO_HOLD` mantém sustentação dos mecanismos
quando ativo. A saída normal contém movimentos temporizados que toleram perda
de medições, conforme [sua documentação](RESCUE_EXIT_SEQUENCE.md).

Para eventual reprodução física, seguir o checklist já existente na
[sequência de saída](RESCUE_EXIT_SEQUENCE.md): começar com rodas suspensas,
conferir E-Stop e timeout, depois testar deslocamentos e retomada da linha em
potência reduzida. Nenhum desses ensaios foi executado nesta revisão.
