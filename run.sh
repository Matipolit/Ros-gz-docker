#!/bin/bash

# --- Configuration Variables ---
HOST_USER_ID=$(id -u)
HOST_GROUP_ID=$(id -g)
HOST_XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR}"
HOST_WAYLAND_DISPLAY="${WAYLAND_DISPLAY}"

# --- Docker Run Command ---

docker run -it --rm \
    --env="DISPLAY" \
    --env="WAYLAND_DISPLAY=${HOST_WAYLAND_DISPLAY}" \
    --env="XDG_RUNTIME_DIR=${HOST_XDG_RUNTIME_DIR}" \
    --env="QT_QPA_PLATFORM=wayland;xcb" \
    --env="GDK_BACKEND=wayland,x11" \
    --env="XCURSOR_SIZE=24" \
    --volume="${HOST_XDG_RUNTIME_DIR}/${HOST_WAYLAND_DISPLAY}:${HOST_XDG_RUNTIME_DIR}/${HOST_WAYLAND_DISPLAY}:rw" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device=/dev/dri:/dev/dri \
    --user="${HOST_USER_ID}:${HOST_GROUP_ID}" \
    --volume="/home/matip/ros-gz-docker/data:/home/ubuntu/shared:rw" \
    ros-jazzy-gz-wayland
