#!/usr/bin/env bash
set -euo pipefail

HOST_NAME="${HOST_NAME:-192.168.0.104}"
USER_NAME="${USER_NAME:-raspberry}"
SERVICE_NAME="${SERVICE_NAME:-obr-robot}"
LINE_CAMERA_SERVICE_NAME="obr-line-camera"
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
LINE_CAMERA_SERVICE_FILE="$SCRIPT_DIR/${LINE_CAMERA_SERVICE_NAME}.service"
SSH_ARGS=()
SCP_ARGS=()

if [[ -f "$KEY_PATH" ]]; then
  SSH_ARGS=(-i "$KEY_PATH")
  SCP_ARGS=(-i "$KEY_PATH")
else
  echo "SSH key not found at $KEY_PATH. SSH may ask for the Raspberry password."
fi

echo "Installing $SERVICE_NAME and $LINE_CAMERA_SERVICE_NAME on $REMOTE"
scp "${SCP_ARGS[@]}" "$SERVICE_FILE" "$LINE_CAMERA_SERVICE_FILE" "${REMOTE}:/tmp/"
ssh "${SSH_ARGS[@]}" "$REMOTE" "chmod +x /home/${USER_NAME}/OBR2026K/scripts/run_robot.sh /home/${USER_NAME}/OBR2026K/scripts/run_line_camera.sh && sudo mv /tmp/${SERVICE_NAME}.service /tmp/${LINE_CAMERA_SERVICE_NAME}.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable --now ${SERVICE_NAME}.service ${LINE_CAMERA_SERVICE_NAME}.service && sudo systemctl status ${SERVICE_NAME}.service ${LINE_CAMERA_SERVICE_NAME}.service --no-pager"

echo "Service installed. Dashboard should start automatically on boot."
