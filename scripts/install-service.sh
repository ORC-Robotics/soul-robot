#!/usr/bin/env bash
set -euo pipefail

HOST_NAME="${HOST_NAME:-192.168.0.105}"
USER_NAME="${USER_NAME:-obr}"
SERVICE_NAME="${SERVICE_NAME:-obr-robot}"
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
    --service)
      SERVICE_NAME="$2"
      shift 2
      ;;
    --key)
      KEY_PATH="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REMOTE="${USER_NAME}@${HOST_NAME}"
SERVICE_FILE="$SCRIPT_DIR/${SERVICE_NAME}.service"
SSH_ARGS=()
SCP_ARGS=()

if [[ -f "$KEY_PATH" ]]; then
  SSH_ARGS=(-i "$KEY_PATH")
  SCP_ARGS=(-i "$KEY_PATH")
fi

echo "Installing $SERVICE_NAME service on $REMOTE"
scp "${SCP_ARGS[@]}" "$SERVICE_FILE" "${REMOTE}:/tmp/${SERVICE_NAME}.service"
ssh "${SSH_ARGS[@]}" "$REMOTE" "sudo mv /tmp/${SERVICE_NAME}.service /etc/systemd/system/${SERVICE_NAME}.service && sudo systemctl daemon-reload && sudo systemctl enable --now ${SERVICE_NAME}.service && sudo systemctl status ${SERVICE_NAME}.service --no-pager"

echo "Service installed. Dashboard should start automatically on boot."
