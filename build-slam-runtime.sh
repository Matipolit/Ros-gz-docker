#!/bin/bash
set -euo pipefail

if docker buildx version >/dev/null 2>&1; then
    docker buildx build \
        --load \
        --file Dockerfile.slam_runtime \
        --tag slam-runtime:jazzy \
        .
else
    docker build \
        --file Dockerfile.slam_runtime \
        --tag slam-runtime:jazzy \
        .
fi
