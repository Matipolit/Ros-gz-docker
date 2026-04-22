#!/bin/bash
set -euo pipefail

docker build \
    --file Dockerfile \
    --tag isaac-sim-runner:jazzy \
    .
