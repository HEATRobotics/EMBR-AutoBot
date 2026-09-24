#!/usr/bin/env bash
# Fails if the compose.*.yaml files have drifted out of sync with each other.
#
# Two checks run:
#   1. Section parity  - every "# MARK: <tag>" section (active or still
#      commented-out as a placeholder) must appear in every compose file.
#   2. Service parity  - every *active* (uncommented) service must appear in
#      every compose file, so an OS-specific file cannot be updated without
#      the others following.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

info() { printf '[INFO] %s\n' "$1"; }
success() { printf '[SUCCESS] %s\n' "$1"; }
error() { printf '[ERROR] %s\n' "$1" >&2; }

mapfile -t COMPOSE_FILES < <(find . -maxdepth 1 -iname 'compose.*.yaml' -printf '%P\n' | sort)

if [ "${#COMPOSE_FILES[@]}" -lt 2 ]; then
    error "Expected at least two compose.*.yaml files, found: ${COMPOSE_FILES[*]:-none}"
    exit 1
fi

info "Checking parity across: ${COMPOSE_FILES[*]}"

# compose.linux.yaml binds ${XAUTHORITY} into the container; give it a
# placeholder so `docker compose config` can resolve on hosts (and CI
# runners) that have no X11 session, matching the convention already used
# in scripts/environment/status_gazebo_fortress.sh.
if [ -z "${XAUTHORITY:-}" ]; then
    export XAUTHORITY="/tmp/embr-compose-parity.xauthority"
    touch "${XAUTHORITY}"
fi

# ---------------------------------------------------------------
# 1. Section (MARK tag) parity
# ---------------------------------------------------------------

status=0
reference_file="${COMPOSE_FILES[0]}"
reference_marks="$(grep -oE '# MARK:.*' "${reference_file}" | sed -E 's/^#+ *//' | sort -u)"

for file in "${COMPOSE_FILES[@]:1}"; do
    marks="$(grep -oE '# MARK:.*' "${file}" | sed -E 's/^#+ *//' | sort -u)"
    if [ "${marks}" != "${reference_marks}" ]; then
        error "Section mismatch between ${reference_file} and ${file}:"
        diff <(printf '%s\n' "${reference_marks}") <(printf '%s\n' "${marks}") >&2 || true
        status=1
    fi
done

if [ "${status}" -eq 0 ]; then
    success "Compose sections (MARK tags) are in sync across all compose files."
fi

# ---------------------------------------------------------------
# 2. Active service parity
# ---------------------------------------------------------------

reference_services="$(docker compose -f "${reference_file}" config --services | sort -u)"

for file in "${COMPOSE_FILES[@]:1}"; do
    services="$(docker compose -f "${file}" config --services | sort -u)"
    if [ "${services}" != "${reference_services}" ]; then
        error "Active service mismatch between ${reference_file} and ${file}:"
        diff <(printf '%s\n' "${reference_services}") <(printf '%s\n' "${services}") >&2 || true
        status=1
    fi
done

if [ "${status}" -eq 0 ]; then
    success "Active services are in sync across all compose files: ${reference_services//$'\n'/, }"
else
    error "Compose files are out of sync. Add/remove the matching service (or placeholder MARK section) in every compose.*.yaml file."
fi

exit "${status}"
