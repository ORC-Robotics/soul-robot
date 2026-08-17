#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="${OBR_APP_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"
ROBOT_BIN="$APP_DIR/build/robot_test"
CAMERA_SCRIPT="$APP_DIR/scripts/camera_line_frame.py"
CAMERA_PATTERN="$APP_DIR/scripts/[c]amera_line_frame.py"
FORWARD_CAMERA_SCRIPT="$APP_DIR/scripts/forward_camera_stream.py"
FORWARD_CAMERA_PATTERN="$APP_DIR/scripts/[f]orward_camera_stream.py"
PYTHON_BIN="$APP_DIR/.venv/bin/python3"
CAMERA_PID=""
FORWARD_CAMERA_PID=""
ROBOT_PID=""

if [[ ! -x "$PYTHON_BIN" ]]; then
  # O ambiente virtual é preferido, mas uma instalação sem venv pode usar o
  # Python do sistema quando todas as dependências já estiverem disponíveis.
  PYTHON_BIN="$(command -v python3)"
fi

stop_camera() {
  if [[ -n "$CAMERA_PID" ]] && kill -0 "$CAMERA_PID" >/dev/null 2>&1; then
    # Para a câmera antes de sair para desligar a iluminação e liberar o hardware.
    kill "$CAMERA_PID"
    wait "$CAMERA_PID" >/dev/null 2>&1 || true
  fi
}

stop_forward_camera() {
  if [[ -n "$FORWARD_CAMERA_PID" ]] && kill -0 "$FORWARD_CAMERA_PID" >/dev/null 2>&1; then
    # O gerenciador fecha a CAM1 antes de sair para liberar o conector e o ISP.
    kill "$FORWARD_CAMERA_PID"
    wait "$FORWARD_CAMERA_PID" >/dev/null 2>&1 || true
  fi
}

stop_robot() {
  if [[ -n "$ROBOT_PID" ]] && kill -0 "$ROBOT_PID" >/dev/null 2>&1; then
    # Encerra o processo principal para permitir que ele zere os motores ao sair.
    kill "$ROBOT_PID"
    wait "$ROBOT_PID" >/dev/null 2>&1 || true
  fi
}

cleanup() {
  stop_robot
  stop_forward_camera
  stop_camera
}

trap cleanup EXIT INT TERM

cd "$APP_DIR"

if [[ ! -s "$ROBOT_BIN" ]]; then
  # Um executável vazio pode ser aceito pelo shell como um script sem comandos.
  # Falhar explicitamente mantém o LED apagado e deixa a causa visível no journal.
  echo "Robot binary is missing or empty: $ROBOT_BIN" >&2
  exit 1
fi

if [[ ! -x "$ROBOT_BIN" ]]; then
  # O serviço nunca deve iniciar câmera ou motores sem um binário executável.
  echo "Robot binary is not executable: $ROBOT_BIN" >&2
  exit 1
fi

if [[ -f "$CAMERA_SCRIPT" ]]; then
  # Remove uma instância antiga do mesmo script, caso uma reinicialização anterior
  # tenha deixado a porta do stream ocupada.
  pkill -f "$CAMERA_PATTERN" >/dev/null 2>&1 || true

  # A câmera é uma ajuda para o dashboard, mas não deve impedir o robô de iniciar.
  OBR_CAMERA_ROLE="${OBR_CAMERA_ROLE:-down}" "$PYTHON_BIN" -u "$CAMERA_SCRIPT" &
  CAMERA_PID="$!"
else
  echo "Camera script not found: $CAMERA_SCRIPT"
fi

if [[ -f "$FORWARD_CAMERA_SCRIPT" ]]; then
  # O processo frontal fica ocioso e não abre a CAM1 até receber uma ativação.
  # Defina OBR_FORWARD_CAMERA_ENABLED=1 apenas quando o boot já deve iniciar o stream.
  pkill -f "$FORWARD_CAMERA_PATTERN" >/dev/null 2>&1 || true
  OBR_FORWARD_CAMERA_ENABLED="${OBR_FORWARD_CAMERA_ENABLED:-0}" \
    "$PYTHON_BIN" -u "$FORWARD_CAMERA_SCRIPT" &
  FORWARD_CAMERA_PID="$!"
else
  echo "Forward camera script not found: $FORWARD_CAMERA_SCRIPT"
fi

"$ROBOT_BIN" &
ROBOT_PID="$!"

set +e
wait "$ROBOT_PID"
ROBOT_STATUS="$?"
set -e

exit "$ROBOT_STATUS"
