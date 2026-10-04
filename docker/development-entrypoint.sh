#!/bin/bash
set -e

EMBR_REPO="${EMBR_REPO:-/home/${USER:-embr}/EMBR-AutoBot}"
EMBR_PHYS_WS="${EMBR_PHYS_WS:-${EMBR_REPO}/embr_phys/ros2_ws}"
ROS_HOME="${ROS_HOME:-/tmp/ros}"
export ROS_HOME
mkdir -p "${ROS_HOME}"

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

# Build the physical workspace if this bind-mounted checkout has no artifacts.
if [ ! -f "${EMBR_PHYS_WS}/install/setup.bash" ]; then
    echo "Building EMBR physical workspace..."
    cd "${EMBR_PHYS_WS}"
    colcon build --symlink-install
fi

source "${EMBR_PHYS_WS}/install/setup.bash"

echo "ROS 2 ${ROS_DISTRO:-humble} development environment is ready."
echo "Physical workspace: ${EMBR_PHYS_WS}"

exec "$@"
