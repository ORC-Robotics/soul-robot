#!/usr/bin/env bash
set -euo pipefail

DEPLOY_USER="${1:-}"
SERVICE_NAME="${2:-}"
LINE_CAMERA_SERVICE_NAME="${3:-}"
SUDOERS_TEMPLATE="/tmp/obr-deploy.sudoers"
SUDOERS_CANDIDATE="/tmp/obr-deploy.sudoers.generated"
SUDOERS_TARGET="/etc/sudoers.d/obr-deploy"
PROJECT_DIR="/home/$DEPLOY_USER/OBR2026K"

if [[ ! "$DEPLOY_USER" =~ ^[a-z_][a-z0-9_-]*$ ]]; then
  echo "Invalid deploy user: $DEPLOY_USER" >&2
  exit 1
fi

if [[ ! "$SERVICE_NAME" =~ ^[a-zA-Z0-9_.@-]+$ || ! "$LINE_CAMERA_SERVICE_NAME" =~ ^[a-zA-Z0-9_.@-]+$ ]]; then
  echo "Invalid systemd service name." >&2
  exit 1
fi

SYSTEMCTL_PATH="$(command -v systemctl)"
if [[ -z "$SYSTEMCTL_PATH" || ! -x "$SYSTEMCTL_PATH" ]]; then
  echo "systemctl was not found on the Raspberry Pi." >&2
  exit 1
fi

# Gera a regra com o usuário e o caminho reais da Raspberry. A regra libera
# somente parar, reiniciar e consultar os dois serviços usados pelo robô.
sed \
  -e "s|@DEPLOY_USER@|$DEPLOY_USER|g" \
  -e "s|@SYSTEMCTL_PATH@|$SYSTEMCTL_PATH|g" \
  -e "s|@SERVICE_NAME@|$SERVICE_NAME|g" \
  -e "s|@LINE_CAMERA_SERVICE_NAME@|$LINE_CAMERA_SERVICE_NAME|g" \
  "$SUDOERS_TEMPLATE" > "$SUDOERS_CANDIDATE"

# Solicita a senha administrativa uma única vez durante a configuração inicial.
# Os comandos de deploy posteriores usam apenas a regra limitada acima.
sudo -v
sudo visudo -cf "$SUDOERS_CANDIDATE"
sudo install -o root -g root -m 0440 "$SUDOERS_CANDIDATE" "$SUDOERS_TARGET"
sudo visudo -cf "$SUDOERS_TARGET"

sudo install -o root -g root -m 0644 "/tmp/$SERVICE_NAME.service" "/etc/systemd/system/$SERVICE_NAME.service"
sudo install -o root -g root -m 0644 "/tmp/$LINE_CAMERA_SERVICE_NAME.service" "/etc/systemd/system/$LINE_CAMERA_SERVICE_NAME.service"
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME.service" "$LINE_CAMERA_SERVICE_NAME.service"

# Em uma instalação nova, o binário ainda pode não existir. Nesse caso, os
# serviços ficam habilitados e o primeiro deploy inicia os dois normalmente.
if [[ -f "$PROJECT_DIR/scripts/run_robot.sh" && -f "$PROJECT_DIR/scripts/run_line_camera.sh" ]]; then
  chmod +x "$PROJECT_DIR/scripts/run_robot.sh" "$PROJECT_DIR/scripts/run_line_camera.sh"
fi

if [[ -x "$PROJECT_DIR/scripts/run_robot.sh" &&
      -x "$PROJECT_DIR/scripts/run_line_camera.sh" &&
      -s "$PROJECT_DIR/build/robot_test" ]]; then
  sudo systemctl restart "$SERVICE_NAME.service" "$LINE_CAMERA_SERVICE_NAME.service"
  sudo systemctl status "$SERVICE_NAME.service" "$LINE_CAMERA_SERVICE_NAME.service" --no-pager
else
  echo "Services enabled. They will start after the first successful deploy."
fi

rm -f "$SUDOERS_CANDIDATE"
