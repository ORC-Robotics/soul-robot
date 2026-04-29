#!/usr/bin/env bash
set -euo pipefail

HOST_NAME="${HOST_NAME:-raspberrypi.local}"
USER_NAME="${USER_NAME:-obr}"
REMOTE_DIR="${REMOTE_DIR:-/home/obr/OBR2026K}"
TARGET="${TARGET:-robot_test}"
RUN_ROBOT="${RUN_ROBOT:-1}"
KEY_PATH="${KEY_PATH:-$HOME/.ssh/obr_raspberry}"
SERVICE_NAME="${SERVICE_NAME:-obr-robot}"
RESTART_SERVICE="${RESTART_SERVICE:-0}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host)
      HOST_NAME="$2"
      shift 2
      ;;
    --user)
      USER_NAME="$2"
      shift 2
      ;;
    --remote-dir)
      REMOTE_DIR="$2"
      shift 2
      ;;
    --target)
      TARGET="$2"
      shift 2
      ;;
    --key)
      KEY_PATH="$2"
      shift 2
      ;;
    --service)
      RESTART_SERVICE="1"
      shift
      ;;
    --service-name)
      SERVICE_NAME="$2"
      shift 2
      ;;
    --no-run)
      RUN_ROBOT="0"
      shift
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"
REMOTE="${USER_NAME}@${HOST_NAME}"
SSH_ARGS=()
SCP_ARGS=()

if [[ -f "$KEY_PATH" ]]; then
  SSH_ARGS=(-i "$KEY_PATH")
  SCP_ARGS=(-i "$KEY_PATH")
fi

echo "Deploying to ${REMOTE}:${REMOTE_DIR}"

ssh "${SSH_ARGS[@]}" "$REMOTE" "mkdir -p '$REMOTE_DIR' '$REMOTE_DIR/build'"
scp "${SCP_ARGS[@]}" "$WORKSPACE/CMakeLists.txt" "${REMOTE}:${REMOTE_DIR}/CMakeLists.txt"
scp "${SCP_ARGS[@]}" -r "$WORKSPACE/src" "${REMOTE}:${REMOTE_DIR}/"
scp "${SCP_ARGS[@]}" -r "$WORKSPACE/include" "${REMOTE}:${REMOTE_DIR}/"
ssh "${SSH_ARGS[@]}" "$REMOTE" "cd '$REMOTE_DIR' && cmake -S . -B build && cmake --build build"

echo "Deploy complete: ${REMOTE}:${REMOTE_DIR}/build/${TARGET}"

if [[ "$RUN_ROBOT" == "0" ]]; then
  echo "Robot was deployed but is not running."
  echo "Dashboard URL after starting: http://${HOST_NAME}:8080"
elif [[ "$RESTART_SERVICE" == "1" ]]; then
  ssh "${SSH_ARGS[@]}" "$REMOTE" "sudo systemctl restart ${SERVICE_NAME}.service && sudo systemctl status ${SERVICE_NAME}.service --no-pager"
  echo "Service restarted. Dashboard URL: http://${HOST_NAME}:8080"
else
  ssh "${SSH_ARGS[@]}" "$REMOTE" "if systemctl cat ${SERVICE_NAME}.service >/dev/null 2>&1; then sudo systemctl restart ${SERVICE_NAME}.service && sudo systemctl status ${SERVICE_NAME}.service --no-pager; else cd '$REMOTE_DIR' && ./build/$TARGET; fi"
  echo "Dashboard URL: http://${HOST_NAME}:8080"
fi
