#!/usr/bin/env bash
set -euo pipefail

HOST_NAME="${HOST_NAME:-192.168.0.104}"
USER_NAME="${USER_NAME:-raspberry}"
REMOTE_DIR="${REMOTE_DIR:-/home/raspberry/OBR2026K}"
TARGET="${TARGET:-robot_test}"
RUN_ROBOT="${RUN_ROBOT:-1}"
KEY_PATH="${KEY_PATH:-$HOME/.ssh/obr_raspberry}"
SERVICE_NAME="${SERVICE_NAME:-obr-robot}"
RESTART_SERVICE="${RESTART_SERVICE:-0}"
RUN_MODE="${RUN_MODE:-auto}"

if [[ "$RUN_ROBOT" == "0" ]]; then
  RUN_MODE="no-run"
elif [[ "$RESTART_SERVICE" == "1" ]]; then
  RUN_MODE="service"
fi

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
      RUN_MODE="service"
      shift
      ;;
    --run)
      RUN_MODE="run"
      shift
      ;;
    --service-name)
      SERVICE_NAME="$2"
      shift 2
      ;;
    --no-run)
      RUN_MODE="no-run"
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
REMOTE_BUILD="${REMOTE_DIR}/build"
REMOTE_STAGING_BUILD="${REMOTE_DIR}/.build-staging"
REMOTE_CAMERA_PATTERN="${REMOTE_DIR}/scripts/[c]amera_line_frame.py"
REMOTE_FORWARD_CAMERA_PATTERN="${REMOTE_DIR}/scripts/[f]orward_camera_stream.py"
REMOTE_RUN_SCRIPT="${REMOTE_DIR}/scripts/run_robot.sh"
SSH_ARGS=()
SCP_ARGS=()

if [[ -f "$KEY_PATH" ]]; then
  SSH_ARGS=(-i "$KEY_PATH")
  SCP_ARGS=(-i "$KEY_PATH")
else
  echo "SSH key not found at $KEY_PATH. SSH may ask for the Raspberry password."
fi

echo "Deploying to ${REMOTE}:${REMOTE_DIR}"

ssh "${SSH_ARGS[@]}" "$REMOTE" "mkdir -p '$REMOTE_DIR' '$REMOTE_BUILD'"

# Para o serviço antes de trocar código ou binário. Se o build falhar, o robô
# permanece parado e o executável válido anterior não é substituído.
ssh "${SSH_ARGS[@]}" "$REMOTE" "sudo systemctl stop '${SERVICE_NAME}.service' >/dev/null 2>&1 || true"
scp "${SCP_ARGS[@]}" "$WORKSPACE/CMakeLists.txt" "${REMOTE}:${REMOTE_DIR}/CMakeLists.txt"
scp "${SCP_ARGS[@]}" -r "$WORKSPACE/src" "${REMOTE}:${REMOTE_DIR}/"
scp "${SCP_ARGS[@]}" -r "$WORKSPACE/include" "${REMOTE}:${REMOTE_DIR}/"
scp "${SCP_ARGS[@]}" -r "$WORKSPACE/scripts" "${REMOTE}:${REMOTE_DIR}/"

# O deploy copia apenas os arquivos necessários para executar o robô. Os testes
# continuam ativos no build local, mas não podem exigir a pasta tests na Raspberry.
atomic_build_command="cd '$REMOTE_DIR' && find scripts -type d -name '__pycache__' -prune -exec rm -rf {} + && cmake -S . -B '$REMOTE_STAGING_BUILD' -DBUILD_TESTING=OFF && cmake --build '$REMOTE_STAGING_BUILD' --target '$TARGET' && test -s '$REMOTE_STAGING_BUILD/$TARGET' && install -m 755 '$REMOTE_STAGING_BUILD/$TARGET' '$REMOTE_BUILD/$TARGET.new' && mv -f '$REMOTE_BUILD/$TARGET.new' '$REMOTE_BUILD/$TARGET' && test -s '$REMOTE_BUILD/$TARGET'"
ssh "${SSH_ARGS[@]}" "$REMOTE" "$atomic_build_command"

echo "Deploy complete: ${REMOTE}:${REMOTE_BUILD}/${TARGET}"

stop_old_camera_command="pkill -f '$REMOTE_CAMERA_PATTERN' >/dev/null 2>&1 || true; pkill -f '$REMOTE_FORWARD_CAMERA_PATTERN' >/dev/null 2>&1 || true"
prepare_scripts_command="cd '$REMOTE_DIR' && find scripts -type f \( -name '*.sh' -o -name '*.service' \) -exec sed -i 's/\r$//' {} + && chmod +x '$REMOTE_RUN_SCRIPT'"
install_service_command="$prepare_scripts_command && sudo cp '$REMOTE_DIR/scripts/${SERVICE_NAME}.service' '/etc/systemd/system/${SERVICE_NAME}.service' && sudo systemctl daemon-reload && sudo systemctl enable '${SERVICE_NAME}.service'"
restart_service_command="$install_service_command && sudo systemctl restart '${SERVICE_NAME}.service'"
status_service_command="sudo systemctl is-active --quiet '${SERVICE_NAME}.service' && sudo systemctl status '${SERVICE_NAME}.service' --no-pager"

if [[ "$RUN_MODE" == "no-run" ]]; then
  echo "Robot was deployed but is not running."
  echo "Dashboard URL after starting: http://${HOST_NAME}:8080"
elif [[ "$RUN_MODE" == "service" ]]; then
  ssh "${SSH_ARGS[@]}" "$REMOTE" "$stop_old_camera_command; $restart_service_command"
  ssh "${SSH_ARGS[@]}" "$REMOTE" "$status_service_command"
  echo "Service restarted. Dashboard URL: http://${HOST_NAME}:8080"
elif [[ "$RUN_MODE" == "run" ]]; then
  ssh "${SSH_ARGS[@]}" "$REMOTE" "$stop_old_camera_command; $prepare_scripts_command; cd '$REMOTE_DIR' && ./build/$TARGET"
else
  start_command="$stop_old_camera_command; if [ -f '$REMOTE_DIR/scripts/${SERVICE_NAME}.service' ]; then $restart_service_command; else $prepare_scripts_command; cd '$REMOTE_DIR' && ./build/$TARGET; fi"
  ssh "${SSH_ARGS[@]}" "$REMOTE" "$start_command"
  ssh "${SSH_ARGS[@]}" "$REMOTE" "$status_service_command"
  echo "Dashboard URL: http://${HOST_NAME}:8080"
fi
