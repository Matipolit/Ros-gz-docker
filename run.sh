#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST_DATA_DIR="${HOST_DATA_DIR:-${SCRIPT_DIR}/data}"
IMAGE_NAME="${ISAAC_SIM_IMAGE_NAME:-isaac-sim-runner:jazzy}"

# --- Configuration Variables ---
HOST_USER_ID=$(id -u)
HOST_GROUP_ID=$(id -g)
HOST_XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR}"
HOST_WAYLAND_DISPLAY="${WAYLAND_DISPLAY}"

# --- Dynamic GPU Permission Fix (The "Double-Tap") ---
# We need to find the groups for BOTH the render node (computation)
# AND the card node (display). 
# 1. Get Group ID for Render Node (usually /dev/dri/renderD128)
if [ -e /dev/dri/renderD128 ]; then
    RENDER_GID=$(stat -c '%g' /dev/dri/renderD128)
else
    RENDER_GID=$(stat -c '%g' /dev/dri/card0)
fi

# 2. Get Group ID for Video Card (usually /dev/dri/card0 or card1)
# We grep for 'card' to find the primary display device
VIDEO_GID=$(stat -c '%g' $(find /dev/dri -name 'card*' | head -n 1))

echo "Detected GPU Render GID: $RENDER_GID"
echo "Detected GPU Video GID:  $VIDEO_GID"

mkdir -p "${HOST_DATA_DIR}"
mkdir -p "${HOST_DATA_DIR}/rosbags"
mkdir -p "${HOST_DATA_DIR}/reports"

# --- X11 Authorization Fix (Just in case) ---
# Even if using Wayland, some libraries check X11 auth. 
# We explicitly allow the local user to connect to X11 (safe-ish for dev).
xhost +local:root > /dev/null 2>&1

# --- Docker Run Command ---
docker run -it --rm \
    --name ros-jazzy-container \
    --network host \
    --ipc=host \
    --env="DISPLAY" \
    --env="WAYLAND_DISPLAY=${HOST_WAYLAND_DISPLAY}" \
    --env="XDG_RUNTIME_DIR=${HOST_XDG_RUNTIME_DIR}" \
    --env="QT_QPA_PLATFORM=wayland" \
    --env="GDK_BACKEND=wayland" \
    --env="XCURSOR_SIZE=24" \
    --volume="${HOST_XDG_RUNTIME_DIR}/${HOST_WAYLAND_DISPLAY}:${HOST_XDG_RUNTIME_DIR}/${HOST_WAYLAND_DISPLAY}:rw" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device=/dev/dri:/dev/dri \
    --group-add ${RENDER_GID} \
    --group-add ${VIDEO_GID} \
    --user="${HOST_USER_ID}:${HOST_GROUP_ID}" \
    --volume="${HOST_DATA_DIR}:/home/ubuntu/shared:rw" \
    --workdir="/home/ubuntu/shared" \
    "${IMAGE_NAME}"
