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
    set -- -m pip list
fi

docker run -it --rm \
    --name slam-evaluator-container \
    --volume="${HOST_DATA_DIR}:/data:rw" \
    --volume="${HOST_SCRIPTS_DIR}:/workspace/slam_evaluator:rw" \
    --workdir="/workspace/slam_evaluator" \
    "${IMAGE_NAME}" "$@"
