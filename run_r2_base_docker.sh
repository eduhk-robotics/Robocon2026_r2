#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="robocon2026-r2:jazzy"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Dockerfile is at repository root (same level as this script)
REPO_ROOT="${SCRIPT_DIR}"

if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
  docker build -t "${IMAGE_NAME}" "${REPO_ROOT}"
fi

DEVICE_BY_ID="/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00"
DEVICE_TTY="/dev/ttyACM0"

DEVICE_ARGS=()
if [ -e "${DEVICE_BY_ID}" ]; then
  DEVICE_ARGS+=(--device "${DEVICE_BY_ID}")
elif [ -e "${DEVICE_TTY}" ]; then
  DEVICE_ARGS+=(--device "${DEVICE_TTY}")
else
  echo "No damiao serial device found. Plug in the device and retry." >&2
  exit 1
fi

exec docker run --rm -it \
  --network host \
  --ipc host \
  --env ROS_LOG_DIR=/workspace/2026R2_ws/.ros_log \
  "${DEVICE_ARGS[@]}" \
  --group-add dialout \
  -v "${REPO_ROOT}/venv_r2:/workspace/venv_r2" \
  "${IMAGE_NAME}" \
  bash -c "source /opt/ros/jazzy/setup.bash && source /workspace/2026R2_ws/install/setup.bash && exec bash"
