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

mkdir -p "${HOST_DATA_DIR}/rosbags"
mkdir -p "${HOST_DATA_DIR}/reports"
mkdir -p "${HOST_DATA_DIR}/worlds"
mkdir -p "${HOST_DATA_DIR}/rtabmap"

if [ "$#" -eq 0 ]; then
    set -- bash
fi

docker run -it --rm \
    --name "${CONTAINER_NAME}" \
    --network host \
    --ipc host \
    --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
    --env "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}" \
    --env "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" \
    --volume "${HOST_DATA_DIR}:/data:rw" \
    --volume "${HOST_DATA_DIR}:/workspace/data:rw" \
    --volume "${HOST_DATA_DIR}/rtabmap:/root/.ros:rw" \
    --volume "${SCRIPT_DIR}/slam_evaluator:/workspace/slam_evaluator:rw" \
    --workdir /workspace \
    "${IMAGE_NAME}" "$@"
