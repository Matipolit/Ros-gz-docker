#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST_DATA_DIR="${HOST_DATA_DIR:-${SCRIPT_DIR}/data}"
IMAGE_NAME="${SLAM_RUNTIME_IMAGE_NAME:-slam-runtime:jazzy}"
CONTAINER_NAME="${SLAM_RUNTIME_CONTAINER_NAME:-slam-runtime-container}"

# Keep ROS DDS discovery simple across containers on Linux.
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

export DISPLAY=$DISPLAY
export XDG_RUNTIME_DIR=/tmp
export QT_X11_NO_MITSHM=1

mkdir -p "${HOST_DATA_DIR}/rosbags"
mkdir -p "${HOST_DATA_DIR}/reports"
mkdir -p "${HOST_DATA_DIR}/worlds"
mkdir -p "${HOST_DATA_DIR}/rtabmap"
mkdir -p "${HOST_DATA_DIR}/slam_ws"

# --- X11 Authorization ---
xhost +local:root > /dev/null 2>&1

if [ "$#" -eq 0 ]; then
    set -- bash
fi

docker run -it --rm \
    --name "${CONTAINER_NAME}" \
    --network host \
    --privileged \
    --ipc host \
    --runtime=nvidia \
    --gpus all \
    --env "NVIDIA_VISIBLE_DEVICES=all" \
    --env "NVIDIA_DRIVER_CAPABILITIES=all" \
    --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
    --env "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}" \
    --env "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" \
    --env "DISPLAY=${DISPLAY:-}" \
    --env "QT_QPA_PLATFORM=xcb" \
    --env "XDG_RUNTIME_DIR=/tmp" \
    --env "QT_X11_NO_MITSHM=1" \
    --env "MESA_NO_MITSHM=1" \
    --env "LIBGL_MIT_SHM=0" \
    --env "LD_LIBRARY_PATH=/opt/slam_ws/src/ORB_SLAM3/lib" \
    --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume "${HOST_DATA_DIR}:/data:rw" \
    --volume "${HOST_DATA_DIR}:/workspace/data:rw" \
    --volume "${HOST_DATA_DIR}/rtabmap:/root/.ros:rw" \
    --volume "${HOST_DATA_DIR}/slam_ws:/opt/slam_ws:rw" \
    --volume "${SCRIPT_DIR}/slam_evaluator:/workspace/slam_evaluator:rw" \
    --workdir /workspace \
    "${IMAGE_NAME}" "$@"
