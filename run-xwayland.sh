#!/bin/bash

# --- Configuration Variables ---
HOST_USER_ID=$(id -u)
HOST_GROUP_ID=$(id -g)
HOST_XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR}"
HOST_WAYLAND_DISPLAY="${WAYLAND_DISPLAY}"

# --- GPU Groups ---
if [ -e /dev/dri/renderD128 ]; then
    RENDER_GID=$(stat -c '%g' /dev/dri/renderD128)
else
    RENDER_GID=$(stat -c '%g' /dev/dri/card0)
fi
VIDEO_GID=$(stat -c '%g' $(find /dev/dri -name 'card*' | head -n 1))

# --- The "Nuclear" Authorization Fix ---
# Allow all local connections. This is the only 100% reliable way 
# to stop "Authorization required" errors in complex Docker setups.
xhost +

# --- Docker Run Command ---
docker run -it --rm \
    --name ros-jazzy-container \
    --network host \
    --ipc=host \
    --env="DISPLAY=${DISPLAY}" \
    --env="QT_QPA_PLATFORM=xcb" \
    --env="XDG_RUNTIME_DIR=${HOST_XDG_RUNTIME_DIR}" \
    --env="ROS_DISTRO=jazzy" \
    --env="ROS_DOMAIN_ID=0" \
    --env="ROS_LOCALHOST_ONLY=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${HOST_XDG_RUNTIME_DIR}:${HOST_XDG_RUNTIME_DIR}:rw" \
    --device=/dev/dri:/dev/dri \
    --group-add ${RENDER_GID} \
    --group-add ${VIDEO_GID} \
    --user="${HOST_USER_ID}:${HOST_GROUP_ID}" \
    --volume="/home/matip/ros-gz-docker/data:/home/ubuntu/shared:rw" \
    ros-jazzy-gz-wayland
