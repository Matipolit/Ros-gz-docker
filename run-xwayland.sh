#!/bin/bash

# --- Configuration Variables ---
HOST_USER_ID=$(id -u)
HOST_GROUP_ID=$(id -g)
HOST_XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR}"
HOST_WAYLAND_DISPLAY="${WAYLAND_DISPLAY}"

# --- Choose QT_QPA_PLATFORM ---
# Option 1: Wayland preferred, XCB fallback
# CURRENT_QT_PLATFORM="wayland;xcb"
# Option 2: Force Wayland
# CURRENT_QT_PLATFORM="wayland"
# Option 3: Force XCB (XWayland) - LET'S TRY THIS
CURRENT_QT_PLATFORM="xcb"

echo "Attempting to run with QT_QPA_PLATFORM=${CURRENT_QT_PLATFORM}"

# --- Docker Run Command ---
docker run -it --rm \
    --env="DISPLAY" \
    --env="WAYLAND_DISPLAY=${HOST_WAYLAND_DISPLAY}" \
    --env="XDG_RUNTIME_DIR=${HOST_XDG_RUNTIME_DIR}" \
    --env="QT_QPA_PLATFORM=${CURRENT_QT_PLATFORM}" \
    --env="GDK_BACKEND=wayland,x11" \
    --env="XCURSOR_SIZE=24" \
    --env="LIBGL_DEBUG=verbose" \
    --env="XAUTHORITY=/home/ubuntu/.Xauthority" \
    --volume="${HOST_XDG_RUNTIME_DIR}/${HOST_WAYLAND_DISPLAY}:${HOST_XDG_RUNTIME_DIR}/${HOST_WAYLAND_DISPLAY}:rw" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/$(whoami)/.Xauthority:/home/ubuntu/.Xauthority:rw" \
    --volume="/home/matip/ros-gz-docker/data:/home/ubuntu/shared:rw" \
    --device=/dev/dri:/dev/dri \
    --user="${HOST_USER_ID}:${HOST_GROUP_ID}" \
    ros-jazzy-gz-wayland \
    bash # Start bash to allow manual command execution
