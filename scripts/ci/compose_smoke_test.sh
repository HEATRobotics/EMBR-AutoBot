#!/usr/bin/env bash
# Builds and starts every active service in a compose file, confirms each
# container reaches the running state, then tears everything down.
#
# This intentionally does not exercise any GUI (RViz/Gazebo) behavior - CI
# runners have no X server. It only proves the image builds and the service
# can be started, per the compose-sync CI requirement.
#
# Usage: compose_smoke_test.sh <compose-file>
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

info() { printf '[INFO] %s\n' "$1"; }
success() { printf '[SUCCESS] %s\n' "$1"; }
warning() { printf '[WARNING] %s\n' "$1"; }
error() { printf '[ERROR] %s\n' "$1" >&2; }

# Hosts without a GPU (most hosted CI runners, and Docker Desktop's Linux VM
# on Windows/macOS) have no /dev/dri render node at all, so `docker compose
# up` fails before the container even starts - this is an environment
# limitation, not a regression in the compose file. Anything else that keeps
# `up` from succeeding is a real failure.
is_missing_device_error() {
    grep -q 'error gathering device information' "$1" && grep -q '/dev/dri' "$1"
}

COMPOSE_FILE="${1:?Usage: $0 <compose-file>}"

if [ ! -f "${COMPOSE_FILE}" ]; then
    error "No such compose file: ${COMPOSE_FILE}"
    exit 1
fi

# See scripts/ci/check_compose_parity.sh for why this is needed.
if [ -z "${XAUTHORITY:-}" ]; then
    export XAUTHORITY="/tmp/embr-compose-smoke.xauthority"
    touch "${XAUTHORITY}"
fi

cleanup() {
    info "Tearing down ${COMPOSE_FILE}..."
    docker compose -f "${COMPOSE_FILE}" down --remove-orphans -v || true
}
trap cleanup EXIT

mapfile -t SERVICES < <(docker compose -f "${COMPOSE_FILE}" config --services)

if [ "${#SERVICES[@]}" -eq 0 ]; then
    info "No active services in ${COMPOSE_FILE}; nothing to smoke test."
    exit 0
fi

status=0

for service in "${SERVICES[@]}"; do
    info "Building ${service} (${COMPOSE_FILE})..."
    if ! docker compose -f "${COMPOSE_FILE}" build "${service}"; then
        error "${service}: build failed."
        status=1
        continue
    fi

    info "Starting ${service} (${COMPOSE_FILE})..."
    up_log="$(mktemp)"
    if ! docker compose -f "${COMPOSE_FILE}" up -d --force-recreate "${service}" >"${up_log}" 2>&1; then
        cat "${up_log}"
        if is_missing_device_error "${up_log}"; then
            warning "${service}: skipped - this host has no /dev/dri (no GPU), which this service requires. This is an environment limitation, not a code issue; verify on a host with a GPU device."
            rm -f "${up_log}"
            continue
        fi
        error "${service}: failed to start."
        docker compose -f "${COMPOSE_FILE}" logs "${service}" || true
        rm -f "${up_log}"
        status=1
        continue
    fi
    rm -f "${up_log}"

    # Give the entrypoint (which may run a first-time colcon build) a window
    # to finish and settle, then confirm the container is still running.
    running="false"
    for _ in $(seq 1 30); do
        state="$(docker compose -f "${COMPOSE_FILE}" ps --format '{{.State}}' "${service}" 2>/dev/null || true)"
        if [ "${state}" = "running" ]; then
            running="true"
            break
        fi
        sleep 2
    done

    if [ "${running}" = "true" ]; then
        success "${service}: running."
    else
        error "${service}: did not reach the running state."
        docker compose -f "${COMPOSE_FILE}" logs "${service}" || true
        status=1
    fi

    docker compose -f "${COMPOSE_FILE}" stop "${service}" || true
done

exit "${status}"
