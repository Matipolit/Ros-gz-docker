#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST_DATA_DIR="${HOST_DATA_DIR:-${SCRIPT_DIR}/data}"
HOST_SCRIPTS_DIR="${HOST_SCRIPTS_DIR:-${SCRIPT_DIR}/slam_evaluator}"
IMAGE_NAME="${SLAM_EVALUATOR_IMAGE_NAME:-slam-evaluator:py312}"

mkdir -p "${HOST_DATA_DIR}/rosbags"
mkdir -p "${HOST_DATA_DIR}/reports"
mkdir -p "${HOST_SCRIPTS_DIR}"

if [ "$#" -eq 0 ]; then
    set -- python3 -m pip list
fi

DOCKER_ARGS=()
if command -v nvidia-smi >/dev/null 2>&1; then
    DOCKER_ARGS+=(
        --runtime=nvidia
        --gpus all
        --env="NVIDIA_VISIBLE_DEVICES=all"
        --env="NVIDIA_DRIVER_CAPABILITIES=all"
    )
fi

docker run -it --rm \
    --name slam-evaluator-container \
    "${DOCKER_ARGS[@]}" \
    --volume="${HOST_DATA_DIR}:/data:rw" \
    --volume="${HOST_SCRIPTS_DIR}:/workspace/slam_evaluator:rw" \
    --workdir="/workspace/slam_evaluator" \
    "${IMAGE_NAME}" "$@"
