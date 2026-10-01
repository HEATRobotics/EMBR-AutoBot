#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
COMPOSE_FILE="${REPO_ROOT}/compose.linux.yaml"
SERVICE="${GAZEBO_STATUS_SERVICE:-canopen-sim}"

info() {
    printf '[INFO] %s\n' "$1"
}

success() {
    printf '[SUCCESS] %s\n' "$1"
}

error() {
    printf '[ERROR] %s\n' "$1" >&2
}

if ! command -v docker >/dev/null 2>&1; then
    error "docker is not installed or not available on PATH."
    exit 1
fi

if ! docker compose version >/dev/null 2>&1; then
    error "docker compose is not available."
    exit 1
fi

export USER="${USER:-$(id -un)}"

if [ -z "${XAUTHORITY:-}" ]; then
    export XAUTHORITY="/tmp/embr-gazebo-status.xauthority"
    touch "$XAUTHORITY"
fi

COMPOSE_ARGS=(-f "$COMPOSE_FILE")

# Hosts without a GPU (hosted CI runners, Docker Desktop's Linux VM) have no
# /dev/dri, and compose refuses to start a container whose device mapping is
# missing. Nothing checked here needs the GPU, so drop the mapping there.
if [ ! -e /dev/dri ]; then
    info "No /dev/dri on this host; starting ${SERVICE} without GPU devices."
    NO_GPU_OVERRIDE="$(mktemp --suffix=.yaml)"
    trap 'rm -f "$NO_GPU_OVERRIDE"' EXIT
    printf 'services:\n  %s:\n    devices: !reset []\n' "$SERVICE" >"$NO_GPU_OVERRIDE"
    COMPOSE_ARGS+=(-f "$NO_GPU_OVERRIDE")
fi

info "Starting ${SERVICE} container and checking Gazebo Fortress..."

# The image gets Fortress through ros-humble-ros-gz (Fortress is Gazebo Sim 6),
# installed without recommends, so the ignition-fortress metapackage and the
# ign CLI may be absent. Check the pieces the image actually relies on.
docker compose "${COMPOSE_ARGS[@]}" run --rm --no-deps \
    --entrypoint /bin/bash \
    "$SERVICE" \
    -lc '
# ROS setup scripts reference unset variables, so source before set -u.
source /opt/ros/humble/setup.bash
set -euo pipefail

installed() {
    dpkg-query -W -f="\${db:Status-Abbrev} \${Package} \${Version}\n" "$@" 2>/dev/null | grep "^ii"
}

echo "[INFO] Checking ROS 2 <-> Gazebo bridge packages..."
installed ros-humble-ros-gz-sim
installed ros-humble-ros-gz-bridge

echo "[INFO] Checking Gazebo Sim 6 (Fortress) libraries..."
installed "libignition-gazebo6*"

echo "[INFO] Checking ros_gz_sim is visible to ROS 2..."
ros2 pkg prefix ros_gz_sim

if command -v ign >/dev/null 2>&1; then
    echo "[INFO] Checking ign gazebo command..."
    if ign gazebo --version >/tmp/ign-gazebo-version.txt 2>&1; then
        cat /tmp/ign-gazebo-version.txt
    else
        ign gazebo --help >/dev/null
        echo "ign gazebo responded to --help"
    fi
else
    echo "[INFO] ign CLI not installed in this image; library checks above cover Fortress."
fi
'

success "Gazebo Fortress status check passed."
