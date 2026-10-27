<<<<<<< HEAD
# ROS 2 Humble autonomy environment for EMBR (MuJoCo, Gymnasium, SB3, W&B, ONNX, Nav2).
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    MUJOCO_GL=osmesa
=======
FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy
>>>>>>> ab96607 (Added optional autonomy Docker environment. Added an opt-in ROS 2 Jazzy container with CPU-based RL, MuJoCo, headless rendering, Nav2, and experiment-tracking dependencies. Keep the default development container unchanged and document autonomy setup for Windows and macOS.)

SHELL ["/bin/bash", "-c"]

ARG USERNAME=embr
ARG USER_UID=1000
ARG USER_GID=1000

ENV HOME=/home/${USERNAME} \
    EMBR_REPO=/home/${USERNAME}/EMBR-AutoBot \
<<<<<<< HEAD
    ROS_HOME=/tmp/ros

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-bringup \
        libegl1 libgl1 libglfw3 libosmesa6 \
    && rm -rf /var/lib/apt/lists/*

# CPU-only torch first (large layer, rarely changes)
RUN pip3 install torch==2.14.1 --index-url https://download.pytorch.org/whl/cpu

# Everything else, pinned
COPY docker/autonomy-requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt

RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home \
        --shell /bin/bash "${USERNAME}" \
=======
    EMBR_PHYS_WS=/home/${USERNAME}/EMBR-AutoBot/embr_phys/ros2_ws \
    ROS_HOME=/tmp/ros \
    MUJOCO_GL=osmesa \
    PYOPENGL_PLATFORM=osmesa \
    MPLBACKEND=Agg \
    PATH="/opt/embr-venv/bin:${PATH}"

# Noble replaces libgl1-mesa-glx with libgl1-mesa-dri.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ffmpeg \
        git \
        libegl1 \
        libgl1 \
        libgl1-mesa-dri \
        libglfw3 \
        libglib2.0-0 \
        libosmesa6-dev \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-venv \
        ros-jazzy-ament-cmake \
        ros-jazzy-ament-cmake-python \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

COPY docker/requirements-rl.txt /tmp/requirements-rl.txt
RUN python3 -m venv --system-site-packages /opt/embr-venv \
    && /opt/embr-venv/bin/pip install --no-cache-dir -r /tmp/requirements-rl.txt \
    && rm /tmp/requirements-rl.txt

RUN existing_user="$(getent passwd "${USER_UID}" | cut -d: -f1 || true)" \
    && if [ -n "${existing_user}" ]; then \
        if [ "${existing_user}" != "${USERNAME}" ]; then \
            usermod --login "${USERNAME}" --home "/home/${USERNAME}" --move-home "${existing_user}"; \
        fi; \
    else \
        useradd --uid "${USER_UID}" --create-home --shell /bin/bash "${USERNAME}"; \
    fi \
    && existing_group="$(getent group "${USER_GID}" | cut -d: -f1 || true)" \
    && if [ -n "${existing_group}" ]; then \
        if [ "${existing_group}" != "${USERNAME}" ]; then \
            groupmod --new-name "${USERNAME}" "${existing_group}"; \
        fi; \
    else \
        groupadd --gid "${USER_GID}" "${USERNAME}"; \
    fi \
    && usermod --gid "${USERNAME}" "${USERNAME}" \
>>>>>>> ab96607 (Added optional autonomy Docker environment. Added an opt-in ROS 2 Jazzy container with CPU-based RL, MuJoCo, headless rendering, Nav2, and experiment-tracking dependencies. Keep the default development container unchanged and document autonomy setup for Windows and macOS.)
    && mkdir -p /tmp/ros \
    && chmod 1777 /tmp/ros

WORKDIR ${EMBR_REPO}

<<<<<<< HEAD
RUN printf '\n# Load ROS 2 in interactive shells.\nsource /opt/ros/${ROS_DISTRO}/setup.bash\n' >> /home/${USERNAME}/.bashrc \
=======
COPY --chown=${USER_UID}:${USER_GID} embr_phys/ros2_ws/src/ ${EMBR_PHYS_WS}/src/
RUN apt-get update \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update \
    && rosdep install --from-paths ${EMBR_PHYS_WS}/src --ignore-src --skip-keys ament_python -r -y \
    && cd ${EMBR_PHYS_WS} \
    && colcon build --symlink-install \
    && chown -R "${USER_UID}:${USER_GID}" ${EMBR_PHYS_WS} \
    && rm -rf /var/lib/apt/lists/*

RUN printf '\nsource /opt/ros/${ROS_DISTRO}/setup.bash\nif [ -f "${EMBR_PHYS_WS}/install/setup.bash" ]; then\n    source "${EMBR_PHYS_WS}/install/setup.bash"\nfi\n' >> /home/${USERNAME}/.bashrc \
>>>>>>> ab96607 (Added optional autonomy Docker environment. Added an opt-in ROS 2 Jazzy container with CPU-based RL, MuJoCo, headless rendering, Nav2, and experiment-tracking dependencies. Keep the default development container unchanged and document autonomy setup for Windows and macOS.)
    && chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.bashrc

USER ${USERNAME}
CMD ["bash"]
<<<<<<< HEAD

LABEL maintainer="Anvi" \
      description="EMBR autonomy environment" \
      version="1.0"
=======
>>>>>>> ab96607 (Added optional autonomy Docker environment. Added an opt-in ROS 2 Jazzy container with CPU-based RL, MuJoCo, headless rendering, Nav2, and experiment-tracking dependencies. Keep the default development container unchanged and document autonomy setup for Windows and macOS.)
