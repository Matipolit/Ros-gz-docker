#!/bin/bash

# --- Configuration Variables ---
HOST_USER_ID=$(id -u)
HOST_GROUP_ID=$(id -g)
HOST_XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR}"
HOST_WAYLAND_DISPLAY="${WAYLAND_DISPLAY}"

# --- X11 Authorization ---
xhost + > /dev/null 2>&1

mkdir -p /home/matip/ros-gz-docker/data/isaac-sim-cache/ov-data
mkdir -p /home/matip/ros-gz-docker/data/isaac-sim-cache/ov-cache
mkdir -p /home/matip/ros-gz-docker/data/isaac-sim-cache/warp
mkdir -p /home/matip/ros-gz-docker/data/isaac-sim-cache/nvidia-omniverse

# --- Docker Run Command (NVIDIA Mode) ---

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
    --env="ROS_DISTRO=jazzy" \
    --env="ROS_DOMAIN_ID=0" \
    --env="ROS_LOCALHOST_ONLY=1" \
    --env="RMW_IMPLEMENTATION=rmw_fastrtps_cpp" \
    --env="LD_LIBRARY_PATH=/opt/isaacsim/_build/linux-x86_64/release/exts/isaacsim.ros2.bridge/jazzy/lib" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${HOST_XDG_RUNTIME_DIR}:${HOST_XDG_RUNTIME_DIR}:rw" \
    --user="${HOST_USER_ID}:${HOST_GROUP_ID}" \
    --volume="/home/matip/ros-gz-docker/data:/home/ubuntu/shared:rw" \
    --volume="/home/matip/ros-gz-docker/data/isaac-sim-cache/ov-data:/home/ubuntu/.local/share/ov:rw" \
    --volume="/home/matip/ros-gz-docker/data/isaac-sim-cache/ov-cache:/home/ubuntu/.cache/ov:rw" \
    --volume="/home/matip/ros-gz-docker/data/isaac-sim-cache/warp:/home/ubuntu/.cache/warp:rw" \
    --volume="/home/matip/ros-gz-docker/data/isaac-sim-cache/nvidia-omniverse:/home/ubuntu/.nvidia-omniverse:rw" \
    --workdir="/home/ubuntu" \
    ros-jazzy-gz-wayland
