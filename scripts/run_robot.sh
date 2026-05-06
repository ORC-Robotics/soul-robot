#!/usr/bin/env bash
set -euo pipefail

APP_DIR="/home/obr/OBR2026K"
ROBOT_BIN="$APP_DIR/build/robot_test"
CAMERA_SCRIPT="$APP_DIR/scripts/camera_line_frame.py"
CAMERA_PID=""

stop_camera() {
  if [[ -n "$CAMERA_PID" ]] && kill -0 "$CAMERA_PID" >/dev/null 2>&1; then
    # Para a câmera antes de sair para desligar a iluminação e liberar o hardware.
    kill "$CAMERA_PID"
    wait "$CAMERA_PID" >/dev/null 2>&1 || true
  fi
}

trap stop_camera EXIT INT TERM

cd "$APP_DIR"

if [[ -f "$CAMERA_SCRIPT" ]]; then
  # A câmera é uma ajuda para o dashboard, mas não deve impedir o robô de iniciar.
  python3 "$CAMERA_SCRIPT" &
  CAMERA_PID="$!"
else
  echo "Camera script not found: $CAMERA_SCRIPT"
fi

exec "$ROBOT_BIN"
