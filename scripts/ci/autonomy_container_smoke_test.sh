#!/usr/bin/env bash
# Reuse the prebuilt autonomy image to start the service with each desktop
# compose file, then verify its Python and ROS dependencies in the container.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

COMPOSE_FILES=(compose.windows.yaml compose.macos.yaml)
CURRENT_COMPOSE_FILE=""

cleanup() {
    if [[ -n "${CURRENT_COMPOSE_FILE}" ]]; then
        docker compose --profile autonomy -f "${CURRENT_COMPOSE_FILE}" down --remove-orphans -v || true
    fi
}
trap cleanup EXIT

for compose_file in "${COMPOSE_FILES[@]}"; do
    CURRENT_COMPOSE_FILE="${compose_file}"
    echo "### ${compose_file}"
    docker compose --profile autonomy -f "${compose_file}" up -d --no-build autonomy

    running=false
    for _ in $(seq 1 30); do
        state="$(docker compose --profile autonomy -f "${compose_file}" ps --format '{{.State}}' autonomy 2>/dev/null || true)"
        if [[ "${state}" == "running" ]]; then
            running=true
            break
        fi
        sleep 2
    done

    if [[ "${running}" != true ]]; then
        docker compose --profile autonomy -f "${compose_file}" logs autonomy || true
        echo "[FAIL] autonomy did not reach running state with ${compose_file}" >&2
        exit 1
    fi
    echo "[PASS] autonomy service is running with ${compose_file}"

    if [[ "${compose_file}" == "${COMPOSE_FILES[0]}" ]]; then
        docker compose --profile autonomy -f "${compose_file}" exec -T autonomy \
            rlpy scripts/ci/check_autonomy_imports.py docker/requirements-rl.txt
        docker compose --profile autonomy -f "${compose_file}" exec -T autonomy \
            bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 pkg prefix nav2_bringup'
        echo "[PASS] nav2_bringup is installed in the autonomy image"
    fi

    docker compose --profile autonomy -f "${compose_file}" down --remove-orphans -v
    CURRENT_COMPOSE_FILE=""
done