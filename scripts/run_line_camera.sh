#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="${OBR_APP_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"
PYTHON_BIN=""
AI_PYTHON_BIN="$APP_DIR/.venv-ai/bin/python3"
DEFAULT_PYTHON_BIN="$APP_DIR/.venv/bin/python3"
CAMERA_SCRIPT="$APP_DIR/scripts/camera_line_frame.py"
STATUS_WRITER="$APP_DIR/scripts/write_line_camera_status.py"
LINE_STATUS_PATH="/dev/shm/obr_line_status.json"
CONTROL_PATH="/dev/shm/obr_line_camera_enabled"
CONTROL_TEMP_PATH="/dev/shm/obr_line_camera_enabled.tmp"
IDLE_POLL_SECONDS="0.10"
RETRY_DELAY_SECONDS="1"
CAMERA_PID=""

if [[ -x "$AI_PYTHON_BIN" ]] &&
   "$AI_PYTHON_BIN" -c 'import cv2, numpy; from picamera2 import Picamera2; from ai_edge_litert.interpreter import Interpreter' >/dev/null 2>&1; then
  # Usa o ambiente de inferência somente quando ele também possui todas as
  # dependências da câmera. Um ambiente incompleto não pode derrubar o serviço.
  PYTHON_BIN="$AI_PYTHON_BIN"
elif [[ -x "$DEFAULT_PYTHON_BIN" ]]; then
  PYTHON_BIN="$DEFAULT_PYTHON_BIN"
else
  PYTHON_BIN="$(command -v python3)"
fi

publish_state() {
  "$PYTHON_BIN" "$STATUS_WRITER" "$1" --enabled "$2" "${@:3}" || true
}

write_enabled() {
  printf '%s\n' "$1" > "$CONTROL_TEMP_PATH"
  mv -f "$CONTROL_TEMP_PATH" "$CONTROL_PATH"
}

camera_requested() {
  [[ -r "$CONTROL_PATH" ]] && [[ "$(tr -d '[:space:]' < "$CONTROL_PATH")" == "1" ]]
}

stop_camera() {
  if [[ -n "$CAMERA_PID" ]] && kill -0 "$CAMERA_PID" >/dev/null 2>&1; then
    kill "$CAMERA_PID" >/dev/null 2>&1 || true
    wait "$CAMERA_PID" >/dev/null 2>&1 || true
  fi
  CAMERA_PID=""
  rm -f "$LINE_STATUS_PATH"
}

start_camera() {
  # Nunca aceite a telemetria visual de uma execução anterior ao reativar.
  rm -f "$LINE_STATUS_PATH"
  publish_state INICIANDO 1
  OBR_CAMERA_ROLE=down "$PYTHON_BIN" -u "$CAMERA_SCRIPT" &
  CAMERA_PID="$!"
}

shutdown_manager() {
  stop_camera
  publish_state PARADA 0
}

trap 'shutdown_manager; exit 0' INT TERM EXIT

if [[ ! -f "$CAMERA_SCRIPT" ]]; then
  publish_state FALHA 1 --error "camera_line_frame.py ausente"
  trap - EXIT
  exit 1
fi

# A câmera inferior inicia ligada na primeira instalação. Depois, o valor é
# preservado para que uma desativação feita pelo dashboard sobreviva ao restart.
if [[ ! -e "$CONTROL_PATH" ]]; then
  write_enabled "${OBR_LINE_CAMERA_ENABLED:-1}"
fi

# O status e o IPC visual anteriores nunca são válidos depois do restart do serviço.
stop_camera
publish_state INICIANDO 1

while true; do
  # /dev/shm pode ser limpo externamente durante a operação. A ausência do
  # controle equivale ao primeiro boot: restaura o padrão habilitado. A opção
  # explícita de desativar continua preservada pelo valor "0" existente.
  if [[ ! -e "$CONTROL_PATH" ]]; then
    write_enabled "${OBR_LINE_CAMERA_ENABLED:-1}"
  fi

  if camera_requested; then
    if [[ -z "$CAMERA_PID" ]]; then
      start_camera
    elif ! kill -0 "$CAMERA_PID" >/dev/null 2>&1; then
      set +e
      wait "$CAMERA_PID"
      camera_exit_code="$?"
      set -e
      CAMERA_PID=""
      rm -f "$LINE_STATUS_PATH"
      publish_state FALHA 1 --error "camera_line_frame.py encerrou com código $camera_exit_code"
      sleep "$RETRY_DELAY_SECONDS"
    fi
  else
    if [[ -n "$CAMERA_PID" ]]; then
      stop_camera
    else
      rm -f "$LINE_STATUS_PATH"
    fi
    publish_state PARADA 0
  fi
  sleep "$IDLE_POLL_SECONDS"
done
