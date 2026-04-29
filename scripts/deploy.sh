#!/usr/bin/env bash
set -euo pipefail

HOST_NAME="${HOST_NAME:-192.168.0.104}"
USER_NAME="${USER_NAME:-obr}"
REMOTE_DIR="${REMOTE_DIR:-/home/obr/OBR2026K}"
TARGET="${TARGET:-robot_test}"
RUN_ROBOT="${RUN_ROBOT:-1}"
KEY_PATH="${KEY_PATH:-$HOME/.ssh/obr_raspberry}"

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
ssh "${SSH_ARGS[@]}" "$REMOTE" "cd '$REMOTE_DIR' && cmake -S . -B build && cmake --build build"

echo "Deploy complete: ${REMOTE}:${REMOTE_DIR}/build/${TARGET}"

if [[ "$RUN_ROBOT" == "1" ]]; then
  ssh "${SSH_ARGS[@]}" "$REMOTE" "cd '$REMOTE_DIR' && ./build/$TARGET"
else
  echo "Robot was deployed but is not running."
  echo "Dashboard URL after starting: http://${HOST_NAME}:8080"
fi
