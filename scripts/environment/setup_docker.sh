#!/bin/bash

# ---------------------------------------------------------------
# Version comparison helpers
# ---------------------------------------------------------------

version_ge() {
    # Returns true if $1 >= $2
    local current="$1"
    local minimum="$2"

    local current_major current_minor current_patch
    local minimum_major minimum_minor minimum_patch

    IFS=. read -r current_major current_minor current_patch <<< "$current"
    IFS=. read -r minimum_major minimum_minor minimum_patch <<< "$minimum"

    current_minor=${current_minor:-0}
    current_patch=${current_patch:-0}
    minimum_minor=${minimum_minor:-0}
    minimum_patch=${minimum_patch:-0}

    if (( current_major > minimum_major )); then
        return 0
    elif (( current_major < minimum_major )); then
        return 1
    fi

    if (( current_minor > minimum_minor )); then
        return 0
    elif (( current_minor < minimum_minor )); then
        return 1
    fi

    if (( current_patch >= minimum_patch )); then
        return 0
    else
        return 1
    fi
}

version_le() {
    # Returns true if $1 <= $2
    local current="$1"
    local maximum="$2"

    local current_major current_minor current_patch
    local maximum_major maximum_minor maximum_patch

    IFS=. read -r current_major current_minor current_patch <<< "$current"
    IFS=. read -r maximum_major maximum_minor maximum_patch <<< "$maximum"

    current_minor=${current_minor:-0}
    current_patch=${current_patch:-0}
    maximum_minor=${maximum_minor:-0}
    maximum_patch=${maximum_patch:-0}

    if (( current_major < maximum_major )); then
        return 0
    elif (( current_major > maximum_major )); then
        return 1
    fi

    if (( current_minor < maximum_minor )); then
        return 0
    elif (( current_minor > maximum_minor )); then
        return 1
    fi

    if (( current_patch <= maximum_patch )); then
        return 0
    else
        return 1
    fi
}

# ---------------------------------------------------------------
# Docker check
# ---------------------------------------------------------------

check_docker() {
    bar
    info "DOCKER CHECK"

    local docker_version_min="29.3"
    local docker_install_recommended="29.4"

    local docker_api_min="1.40"
    local docker_api_max="1.54"

    local docker_client_api_version_current
    local docker_server_api_version_current
    local docker_server_version_current
    local docker_info_error

    # -----------------------------------------------------------
    # Check if Docker command exists
    # -----------------------------------------------------------

    if ! command -v docker >/dev/null 2>&1; then
        echo ""
        error "Docker is not installed or is not available in PATH."
        info "Please install Docker version $docker_install_recommended, then run this script again."
        bar
        return 1
    fi

    # -----------------------------------------------------------
    # Check if Docker client can communicate with Docker daemon
    # -----------------------------------------------------------

    if ! docker_info_error=$(docker info 2>&1 >/dev/null); then
        echo ""
        error "Docker Client Can NOT Communicate With Docker Server/Daemon"
        error "Docker returned the following error:"
        echo "$docker_info_error" >&2
        echo ""
        info "Make sure Docker Desktop or the Docker daemon is running, then try again."
        bar
        return 1
    fi

    success "Docker Client Can Communicate With Docker Server/Daemon"

    # -----------------------------------------------------------
    # Get Docker versions
    # -----------------------------------------------------------

    docker_client_api_version_current=$(docker version --format '{{.Client.APIVersion}}')
    docker_server_api_version_current=$(docker version --format '{{.Server.APIVersion}}')
    docker_server_version_current=$(docker version --format '{{.Server.Version}}')

    echo ""
    info "Docker Server Version: $docker_server_version_current"
    info "Docker Client API Version: $docker_client_api_version_current"
    info "Docker Server API Version: $docker_server_api_version_current"

    # -----------------------------------------------------------
    # Check Docker Server version
    # -----------------------------------------------------------

    # GitHub-hosted runners ship an older Docker than we require locally and
    # it can't be pinned, so only warn there. The API range check below still
    # applies.
    if version_ge "$docker_server_version_current" "$docker_version_min"; then
        success "Docker Server version is adequate."
    elif [ "${GITHUB_ACTIONS:-}" = "true" ]; then
        warning "Docker Server $docker_server_version_current is below the required $docker_version_min; continuing because this is a CI runner."
    else
        echo ""
        error "Docker Server version is not adequate."
        info "Minimum required Docker version: $docker_version_min"
        info "Detected Docker Server version: $docker_server_version_current"
        info "You may need to update Docker."
        info "Recommended Docker version to install: $docker_install_recommended"
        bar
        return 1
    fi

    # -----------------------------------------------------------
    # Check Docker Server API minimum version
    # -----------------------------------------------------------

    if ! version_ge "$docker_server_api_version_current" "$docker_api_min"; then
        echo ""
        error "Docker Server API version is too old."
        info "Minimum required Docker API version: $docker_api_min"
        info "Detected Docker Server API version: $docker_server_api_version_current"
        info "You may need to update Docker."
        info "Recommended Docker version to install: $docker_install_recommended"
        bar
        return 1
    fi

    # -----------------------------------------------------------
    # Check Docker Server API maximum version
    # -----------------------------------------------------------

    if ! version_le "$docker_server_api_version_current" "$docker_api_max"; then
        echo ""
        error "Docker Server API version is too new for this project."
        info "Maximum supported Docker API version: $docker_api_max"
        info "Detected Docker Server API version: $docker_server_api_version_current"
        info "This project may need compatibility testing with your Docker version."
        bar
        return 1
    fi

    success "Docker Server API version is within supported range."

    echo ""
    success "Docker environment check passed."
    bar
}

check_docker