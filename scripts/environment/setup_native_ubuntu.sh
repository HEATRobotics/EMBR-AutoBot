#!/usr/bin/env bash
# Install EMBR's Ubuntu host tools and ROS package dependencies.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
if [[ "${ROS_DISTRO}" != "humble" ]]; then
    echo "This project requires ROS 2 Humble; ROS_DISTRO is '${ROS_DISTRO}'. Use a Humble terminal or unset ROS_DISTRO." >&2
    exit 1
fi
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ "$(. /etc/os-release && echo "${ID}:${VERSION_ID}")" != "ubuntu:22.04" ]]; then
    echo "This setup script supports Ubuntu 22.04 (Jammy); install ROS 2 Humble first." >&2
    exit 1
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "ROS 2 ${ROS_DISTRO} was not found at ${ROS_SETUP}. Install ROS 2 Humble first." >&2
    exit 1
fi

sudo apt-get update
grep -Ev '^[[:space:]]*(#|$)' "${SCRIPT_DIR}/ubuntu-packages.txt" \
    | xargs -r sudo apt-get install -y

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update

# ROS setup scripts may read unset environment variables. Keep failure handling
# enabled, but disable nounset while loading the ROS environment.
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u
rosdep install --from-paths \
    "${REPO_ROOT}/embr_phys/ros2_ws/src" \
    --ignore-src --rosdistro "${ROS_DISTRO}" -r -y

rosdep check --from-paths \
    "${REPO_ROOT}/embr_phys/ros2_ws/src" \
    --ignore-src --rosdistro "${ROS_DISTRO}"

echo "Ubuntu dependencies are installed and verified. Rebuild the workspace with colcon as documented."
