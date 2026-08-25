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

if [[ ! "$USER_NAME" =~ ^[a-z_][a-z0-9_-]*$ ]]; then
  echo "Invalid Raspberry user: $USER_NAME" >&2
  exit 1
fi

if [[ ! "$SERVICE_NAME" =~ ^[a-zA-Z0-9_.@-]+$ ]]; then
  echo "Invalid systemd service name: $SERVICE_NAME" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REMOTE="${USER_NAME}@${HOST_NAME}"
SERVICE_FILE="$SCRIPT_DIR/${SERVICE_NAME}.service"
LINE_CAMERA_SERVICE_FILE="$SCRIPT_DIR/${LINE_CAMERA_SERVICE_NAME}.service"
SUDOERS_TEMPLATE_FILE="$SCRIPT_DIR/obr-deploy.sudoers"
REMOTE_INSTALLER_FILE="$SCRIPT_DIR/install-service-remote.sh"

if [[ ! -f "$KEY_PATH" ]]; then
  mkdir -p "$(dirname "$KEY_PATH")"
  chmod 700 "$(dirname "$KEY_PATH")"
  echo "Creating the deploy SSH key at $KEY_PATH"
  ssh-keygen -t ed25519 -f "$KEY_PATH" -N "" -C "obr2026k-deploy"
fi

SSH_KEY_ARGS=(-i "$KEY_PATH" -o IdentitiesOnly=yes -o StrictHostKeyChecking=accept-new)

if ! ssh "${SSH_KEY_ARGS[@]}" -o BatchMode=yes -o ConnectTimeout=5 "$REMOTE" true 2>/dev/null; then
  public_key="$(ssh-keygen -y -f "$KEY_PATH")"
  read -r key_type key_data _ <<< "$public_key"

  if [[ ! "$key_type" =~ ^(ssh-(ed25519|rsa)|ecdsa-sha2-nistp(256|384|521))$ || ! "$key_data" =~ ^[A-Za-z0-9+/=]+$ ]]; then
    echo "Invalid public key generated from $KEY_PATH" >&2
    exit 1
  fi

  public_key_line="$key_type $key_data obr2026k-deploy"
  install_key_command="umask 077; mkdir -p ~/.ssh; touch ~/.ssh/authorized_keys; grep -qF '$key_type $key_data' ~/.ssh/authorized_keys || printf '%s\\n' '$public_key_line' >> ~/.ssh/authorized_keys; chmod 700 ~/.ssh; chmod 600 ~/.ssh/authorized_keys"

  echo "Enter the Raspberry SSH password once to install the deploy key."
  ssh "${SSH_KEY_ARGS[@]}" "$REMOTE" "$install_key_command"
fi

if ! ssh "${SSH_KEY_ARGS[@]}" -o BatchMode=yes -o ConnectTimeout=5 "$REMOTE" true; then
  echo "The SSH key was not accepted by $REMOTE" >&2
  exit 1
fi

SSH_ARGS=("${SSH_KEY_ARGS[@]}" -o BatchMode=yes)
SCP_ARGS=("${SSH_KEY_ARGS[@]}" -o BatchMode=yes)

echo "Installing $SERVICE_NAME and $LINE_CAMERA_SERVICE_NAME on $REMOTE"
scp "${SCP_ARGS[@]}" \
  "$SERVICE_FILE" \
  "$LINE_CAMERA_SERVICE_FILE" \
  "$SUDOERS_TEMPLATE_FILE" \
  "$REMOTE_INSTALLER_FILE" \
  "${REMOTE}:/tmp/"

echo "Enter the Raspberry sudo password once to finish the service setup."
normalize_files_command="sed -i 's/\r$//' /tmp/install-service-remote.sh /tmp/obr-deploy.sudoers '/tmp/$SERVICE_NAME.service' '/tmp/$LINE_CAMERA_SERVICE_NAME.service'"
remote_install_command="$normalize_files_command && chmod +x /tmp/install-service-remote.sh && /tmp/install-service-remote.sh '$USER_NAME' '$SERVICE_NAME' '$LINE_CAMERA_SERVICE_NAME'"
ssh "${SSH_ARGS[@]}" -t "$REMOTE" "$remote_install_command"

echo "Setup complete. Future deploys will not ask for SSH or sudo passwords."
