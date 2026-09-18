#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SERVICE="${GAZEBO_STATUS_SERVICE:-control-systems-dev}"

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

info "Starting ${SERVICE} container and checking Gazebo Fortress..."

docker compose -f "${REPO_ROOT}/compose.linux.yaml" run --rm --no-deps \
    --entrypoint /bin/bash \
    "$SERVICE" \
    -lc '
set -euo pipefail

echo "[INFO] Checking ign CLI..."
command -v ign

echo "[INFO] Checking ignition-fortress package..."
dpkg-query -W -f=\${Package}\ \${Version}\\n ignition-fortress

echo "[INFO] Checking ign gazebo command..."
if ign gazebo --version >/tmp/ign-gazebo-version.txt 2>&1; then
    cat /tmp/ign-gazebo-version.txt
else
    ign gazebo --help >/dev/null
    echo "ign gazebo responded to --help"
fi
'

success "Gazebo Fortress status check passed."
