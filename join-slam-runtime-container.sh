#!/bin/bash
set -euo pipefail

CONTAINER_NAME="${SLAM_RUNTIME_CONTAINER_NAME:-slam-runtime-container}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

docker exec \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
    -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
    -it "${CONTAINER_NAME}" /bin/bash -lc 'source /opt/ros/jazzy/setup.bash; if [ -f /opt/slam_ws/install/setup.bash ]; then source /opt/slam_ws/install/setup.bash; fi; exec bash'
