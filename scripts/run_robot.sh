#!/usr/bin/env bash
set -euo pipefail

APP_DIR="/home/obr/OBR2026K"
ROBOT_BIN="$APP_DIR/build/robot_test"
CAMERA_SCRIPT="$APP_DIR/scripts/camera_line_frame.py"
CAMERA_PATTERN="$APP_DIR/scripts/[c]amera_line_frame.py"
CAMERA_PID=""
ROBOT_PID=""

stop_camera() {
  if [[ -n "$CAMERA_PID" ]] && kill -0 "$CAMERA_PID" >/dev/null 2>&1; then
    # Para a câmera antes de sair para desligar a iluminação e liberar o hardware.
    kill "$CAMERA_PID"
    wait "$CAMERA_PID" >/dev/null 2>&1 || true
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
  stop_camera
}

trap cleanup EXIT INT TERM

cd "$APP_DIR"

if [[ -f "$CAMERA_SCRIPT" ]]; then
  # Remove uma instância antiga do mesmo script, caso uma reinicialização anterior
  # tenha deixado a porta do stream ocupada.
  pkill -f "$CAMERA_PATTERN" >/dev/null 2>&1 || true

  # A câmera é uma ajuda para o dashboard, mas não deve impedir o robô de iniciar.
  python3 -u "$CAMERA_SCRIPT" &
  CAMERA_PID="$!"
else
  echo "Camera script not found: $CAMERA_SCRIPT"
fi

"$ROBOT_BIN" &
ROBOT_PID="$!"

set +e
wait "$ROBOT_PID"
ROBOT_STATUS="$?"
set -e

exit "$ROBOT_STATUS"
