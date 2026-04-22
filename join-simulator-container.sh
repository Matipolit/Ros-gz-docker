#!/bin/bash
set -euo pipefail

CONTAINER_NAME="${SIM_CONTAINER_NAME:-ros-jazzy-container}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

docker exec \
	-e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
	-e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
	-e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION}" \
	-it "${CONTAINER_NAME}" /bin/bash -lc 'source /opt/ros/jazzy/setup.bash; exec bash'