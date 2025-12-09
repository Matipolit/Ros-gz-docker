#!/bin/bash
set -e

# Get the UID and GID the container is running as (passed by --user)
TARGET_UID=$(id -u)
TARGET_GID=$(id -g)
RUNTIME_DIR_BASE="/run/user"
TARGET_XDG_RUNTIME_DIR="${RUNTIME_DIR_BASE}/${TARGET_UID}"

echo "Entrypoint: Ensuring ${TARGET_XDG_RUNTIME_DIR} exists and has correct ownership for UID ${TARGET_UID} GID ${TARGET_GID}."

# Ensure the base for XDG_RUNTIME_DIR exists
if [ ! -d "${TARGET_XDG_RUNTIME_DIR}" ]; then
    echo "Entrypoint: ${TARGET_XDG_RUNTIME_DIR} does not exist, creating."
    mkdir -p "${TARGET_XDG_RUNTIME_DIR}"
    # Chown and chmod after creation
    chown "${TARGET_UID}:${TARGET_GID}" "${TARGET_XDG_RUNTIME_DIR}" || echo "Entrypoint Warning: Could not chown ${TARGET_XDG_RUNTIME_DIR}"
    chmod 0700 "${TARGET_XDG_RUNTIME_DIR}" || echo "Entrypoint Warning: Could not chmod ${TARGET_XDG_RUNTIME_DIR}"
else
    echo "Entrypoint: ${TARGET_XDG_RUNTIME_DIR} already exists."
    # Still attempt to chown/chmod in case it exists but with wrong perms from a lower layer
    chown "${TARGET_UID}:${TARGET_GID}" "${TARGET_XDG_RUNTIME_DIR}" || echo "Entrypoint Warning: Could not chown existing ${TARGET_XDG_RUNTIME_DIR}"
    chmod 0700 "${TARGET_XDG_RUNTIME_DIR}" || echo "Entrypoint Warning: Could not chmod existing ${TARGET_XDG_RUNTIME_DIR}"
fi
ls -ld "${TARGET_XDG_RUNTIME_DIR}" # Print permissions for verification

# Source ROS 2 setup
source "/opt/ros/${ROS_DISTRO}/setup.bash"

export PATH=$PATH:/root/.cargo/bin/

export ROS_DOMAIN_ID=50

echo "Entrypoint: Executing command: $@"
exec "$@"
