#!/usr/bin/env bash
set -euo pipefail

HOST_NAME="${HOST_NAME:-obr.local}"
USER_NAME="${USER_NAME:-raspberry}"
REMOTE_DIR="${REMOTE_DIR:-/home/raspberry/OBR2026K}"
TARGET="${TARGET:-robot_test}"
RUN_ROBOT="${RUN_ROBOT:-1}"
KEY_PATH="${KEY_PATH:-$HOME/.ssh/obr_raspberry}"
SERVICE_NAME="${SERVICE_NAME:-obr-robot}"
RESTART_SERVICE="${RESTART_SERVICE:-0}"
RUN_MODE="${RUN_MODE:-auto}"
FULL_SYNC=0

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
    --full-sync)
      FULL_SYNC=1
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
SETUP_SCRIPT="$SCRIPT_DIR/install-service.sh"

REMOTE="${USER_NAME}@${HOST_NAME}"
REMOTE_BUILD="${REMOTE_DIR}/build"
REMOTE_STAGING_BUILD="${REMOTE_DIR}/.build-staging"
REMOTE_CAMERA_PATTERN="${REMOTE_DIR}/scripts/[c]amera_line_frame.py"
REMOTE_FORWARD_CAMERA_PATTERN="${REMOTE_DIR}/scripts/[f]orward_camera_stream.py"
REMOTE_RUN_SCRIPT="${REMOTE_DIR}/scripts/run_robot.sh"
REMOTE_LINE_CAMERA_RUN_SCRIPT="${REMOTE_DIR}/scripts/run_line_camera.sh"

LINE_CAMERA_SERVICE_NAME="obr-line-camera"

setup_hint="bash scripts/install-service.sh --host $HOST_NAME"

initialize_deploy_access() {
  if [[ ! -f "$SETUP_SCRIPT" ]]; then
    echo "Setup script not found: $SETUP_SCRIPT" >&2
    exit 1
  fi

  echo "Preparing SSH and limited sudo access for the first deploy."

  bash "$SETUP_SCRIPT" \
    --host "$HOST_NAME" \
    --user "$USER_NAME" \
    --service "$SERVICE_NAME" \
    --key "$KEY_PATH"
}

required_dashboard_assets=(
  "$WORKSPACE/assets/dashboard-logo.png"
  "$WORKSPACE/assets/soul-sync-favicon.png"
  "$WORKSPACE/assets/models/ball_detector.onnx"
)

for asset_path in "${required_dashboard_assets[@]}"; do
  if [[ ! -s "$asset_path" ]]; then
    echo "Dashboard asset not found or empty: $asset_path" >&2
    exit 1
  fi
done

if ! command -v rsync >/dev/null 2>&1; then
  echo "rsync is not installed locally. Install it with: sudo apt install rsync" >&2
  exit 1
fi

if [[ ! -f "$KEY_PATH" ]]; then
  initialize_deploy_access
fi

SSH_ARGS=(
  -i "$KEY_PATH"
  -o IdentitiesOnly=yes
  -o BatchMode=yes
  -o StrictHostKeyChecking=accept-new
)

if ! ssh "${SSH_ARGS[@]}" "$REMOTE" true; then
  initialize_deploy_access

  if ! ssh "${SSH_ARGS[@]}" "$REMOTE" true; then
    echo "SSH key access could not be configured for $REMOTE." >&2
    echo "Retry manually with: $setup_hint" >&2
    exit 1
  fi
fi

if ! ssh "${SSH_ARGS[@]}" "$REMOTE" "command -v rsync >/dev/null 2>&1"; then
  echo "rsync is not installed on the Raspberry." >&2
  echo "Install it once with: sudo apt install rsync" >&2
  exit 1
fi

echo "Deploying to ${REMOTE}:${REMOTE_DIR}"

ssh "${SSH_ARGS[@]}" "$REMOTE" \
  "mkdir -p '$REMOTE_DIR' '$REMOTE_BUILD'"

# Para os serviços antes de substituir código/binário.
ssh "${SSH_ARGS[@]}" "$REMOTE" \
  "sudo -n systemctl stop '${SERVICE_NAME}.service' '${LINE_CAMERA_SERVICE_NAME}.service' || { echo 'Deploy access is not configured. Run: $setup_hint' >&2; exit 1; }"

if [[ "$FULL_SYNC" == "1" ]]; then
  echo "Synchronizing all deploy files and rebuilding from scratch..."
else
  echo "Synchronizing changed files with rsync..."
fi

RSYNC_SSH="ssh -i \"$KEY_PATH\" -o IdentitiesOnly=yes -o BatchMode=yes -o StrictHostKeyChecking=accept-new"
RSYNC_ARGS=(-az)
if [[ "$FULL_SYNC" == "1" ]]; then
  RSYNC_ARGS+=(--ignore-times)
fi

(
  cd "$WORKSPACE"

  rsync "${RSYNC_ARGS[@]}" \
    -e "$RSYNC_SSH" \
    CMakeLists.txt \
    src \
    include \
    scripts \
    assets \
    "${REMOTE}:${REMOTE_DIR}/"
)

# O modo completo remove somente o build temporário. Código, dataset, ambiente
# virtual e o último binário válido permanecem intactos até o novo build passar.
if [[ "$FULL_SYNC" == "1" ]]; then
  ssh "${SSH_ARGS[@]}" "$REMOTE" "rm -rf '$REMOTE_STAGING_BUILD'"
fi

# Build incremental em diretório persistente.
# Pi 5 possui 4 cores, então usa quatro jobs em paralelo.
atomic_build_command="cd '$REMOTE_DIR' && \
test -s assets/dashboard-logo.png && \
test -s assets/soul-sync-favicon.png && \
test -s assets/models/ball_detector.onnx && \
find scripts -type d -name '__pycache__' -prune -exec rm -rf {} + && \
cmake -S . -B '$REMOTE_STAGING_BUILD' -DBUILD_TESTING=OFF && \
cmake --build '$REMOTE_STAGING_BUILD' --target '$TARGET' --parallel 4 && \
test -s '$REMOTE_STAGING_BUILD/$TARGET' && \
install -m 755 '$REMOTE_STAGING_BUILD/$TARGET' '$REMOTE_BUILD/$TARGET.new' && \
mv -f '$REMOTE_BUILD/$TARGET.new' '$REMOTE_BUILD/$TARGET' && \
test -s '$REMOTE_BUILD/$TARGET'"

ssh "${SSH_ARGS[@]}" "$REMOTE" "$atomic_build_command"

# Usa exatamente o mesmo Python escolhido pelo run_robot.sh.
vision_dependencies_command="if [ -x '$REMOTE_DIR/.venv/bin/python3' ]; then \
python_bin='$REMOTE_DIR/.venv/bin/python3'; \
else \
python_bin=\$(command -v python3); \
fi; \
test -n \"\$python_bin\" || { echo 'Python 3 was not found for the forward camera.' >&2; exit 1; }; \
if ! \"\$python_bin\" -c 'import onnxruntime; raise SystemExit(0 if onnxruntime.__version__ == \"1.30.0\" else 1)' 2>/dev/null; then \
\"\$python_bin\" -m pip install --disable-pip-version-check onnxruntime==1.30.0; \
fi; \
\"\$python_bin\" -c 'import onnxruntime; raise SystemExit(0 if onnxruntime.__version__ == \"1.30.0\" else 1)'"

ssh "${SSH_ARGS[@]}" "$REMOTE" "$vision_dependencies_command"

echo "Deploy complete: ${REMOTE}:${REMOTE_BUILD}/${TARGET}"

stop_old_camera_command="pkill -f '$REMOTE_CAMERA_PATTERN' >/dev/null 2>&1 || true; pkill -f '$REMOTE_FORWARD_CAMERA_PATTERN' >/dev/null 2>&1 || true"

prepare_scripts_command="cd '$REMOTE_DIR' && \
find scripts -type f \( -name '*.sh' -o -name '*.service' \) -exec sed -i 's/\r$//' {} + && \
chmod +x '$REMOTE_RUN_SCRIPT' '$REMOTE_LINE_CAMERA_RUN_SCRIPT'"

restart_service_command="$prepare_scripts_command && \
sudo -n systemctl restart '${SERVICE_NAME}.service' '${LINE_CAMERA_SERVICE_NAME}.service'"

status_service_command="sudo -n systemctl is-active --quiet '${SERVICE_NAME}.service' && \
sudo -n systemctl is-active --quiet '${LINE_CAMERA_SERVICE_NAME}.service' && \
sudo -n systemctl status '${SERVICE_NAME}.service' '${LINE_CAMERA_SERVICE_NAME}.service' --no-pager"

if [[ "$RUN_MODE" == "no-run" ]]; then
  echo "Robot was deployed but is not running."
  echo "Dashboard URL after starting: http://${HOST_NAME}:8080"

elif [[ "$RUN_MODE" == "service" ]]; then
  ssh "${SSH_ARGS[@]}" "$REMOTE" "$stop_old_camera_command; $restart_service_command"
  ssh "${SSH_ARGS[@]}" "$REMOTE" "$status_service_command"
  echo "Service restarted. Dashboard URL: http://${HOST_NAME}:8080"

elif [[ "$RUN_MODE" == "run" ]]; then
  ssh "${SSH_ARGS[@]}" "$REMOTE" \
    "$stop_old_camera_command; $prepare_scripts_command; cd '$REMOTE_DIR' && ./build/$TARGET"

else
  ssh "${SSH_ARGS[@]}" "$REMOTE" "$stop_old_camera_command; $restart_service_command"
  ssh "${SSH_ARGS[@]}" "$REMOTE" "$status_service_command"
  echo "Dashboard URL: http://${HOST_NAME}:8080"
fi
