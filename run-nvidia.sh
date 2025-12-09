#!/bin/bash

# --- Configuration Variables ---
HOST_USER_ID=$(id -u)
HOST_GROUP_ID=$(id -g)
HOST_XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR}"
HOST_WAYLAND_DISPLAY="${WAYLAND_DISPLAY}"

# --- X11 Authorization ---
# "Lower the drawbridge" for Xwayland connections
xhost + > /dev/null 2>&1

# --- Docker Run Command (NVIDIA Mode) ---
# --runtime=nvidia: The key to unlocking the 5070 Ti
# --gpus all: Explicitly request all GPUs
# NVIDIA_DRIVER_CAPABILITIES=all: Ensures Compute (CUDA) + Graphics (OpenGL) + Display

docker run -it --rm \
    --name ros-jazzy-container \
    --network host \
    --ipc=host \
    --runtime=nvidia \
    --gpus all \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY=${DISPLAY}" \
    --env="QT_QPA_PLATFORM=xcb" \
    --env="XDG_RUNTIME_DIR=${HOST_XDG_RUNTIME_DIR}" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${HOST_XDG_RUNTIME_DIR}:${HOST_XDG_RUNTIME_DIR}:rw" \
    --user="${HOST_USER_ID}:${HOST_GROUP_ID}" \
    --volume="/home/matip/ros-gz-docker/data:/home/ubuntu/shared:rw" \
    ros-jazzy-gz-wayland
