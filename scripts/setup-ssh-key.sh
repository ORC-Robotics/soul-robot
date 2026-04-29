#!/usr/bin/env bash
set -euo pipefail

HOST_NAME="${HOST_NAME:-192.168.0.104}"
USER_NAME="${USER_NAME:-obr}"
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

REMOTE="${USER_NAME}@${HOST_NAME}"
mkdir -p "$(dirname "$KEY_PATH")"

if [[ ! -f "$KEY_PATH" ]]; then
  ssh-keygen -t ed25519 -f "$KEY_PATH" -N "" -C "obr-deploy"
fi

echo "Sending public key to $REMOTE"
echo "Type the Raspberry password once when prompted."

cat "$KEY_PATH.pub" | ssh "$REMOTE" "mkdir -p ~/.ssh && touch ~/.ssh/authorized_keys && cat >> ~/.ssh/authorized_keys && awk '!seen[\$0]++' ~/.ssh/authorized_keys > ~/.ssh/authorized_keys.tmp && mv ~/.ssh/authorized_keys.tmp ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys"

echo "SSH key configured. Future deploys should not ask for the Raspberry password."
