# Gravação diagnóstica da CAM1

O gravador fica desligado por padrão e não participa de decisões de movimento.
Ele salva imagens da CAM1 a 5 FPS e registra o comando final de movimento a cada
ciclo do processo C++ enquanto a sessão estiver ativa.

Na Raspberry Pi, execute os comandos a partir de
`/home/raspberry/OBR2026K`:

```bash
./.venv/bin/python3 scripts/forward_reacquisition_recorder.py start obstacle_exit_left
./.venv/bin/python3 scripts/forward_reacquisition_recorder.py status
./.venv/bin/python3 scripts/forward_reacquisition_recorder.py stop
```

Se o ambiente virtual não existir, use `python3`. O label aceita letras,
números, `_` e `-`. A frequência de imagem pode ser ajustada entre 0,5 e 10 FPS:

```bash
python3 scripts/forward_reacquisition_recorder.py start rescue_exit --fps 5
```

Cada execução cria `logs/forward_reacquisition/<timestamp>_<label>/` com
`session.json`, `vision.jsonl`, `control.jsonl`, `frames/`, `masks/` e
`overlays/`. O `stop` registra o horário final em `session.json`; ele não para o
robô, as câmeras ou o serviço.
