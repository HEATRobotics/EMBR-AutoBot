FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy

SHELL ["/bin/bash", "-c"]

ARG USERNAME=embr
ARG USER_UID=1000
ARG USER_GID=1000

ENV HOME=/home/${USERNAME} \
    EMBR_REPO=/home/${USERNAME}/EMBR-AutoBot \
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
RUN python3 -m venv /opt/embr-venv \
    && /opt/embr-venv/bin/pip install --no-cache-dir -r /tmp/requirements-rl.txt \
    && rm /tmp/requirements-rl.txt

RUN printf '#!/bin/sh\nexec env -u PYTHONPATH /opt/embr-venv/bin/python "$@"\n' > /usr/local/bin/rlpy \
    && chmod +x /usr/local/bin/rlpy \
    && rlpy -c "import torch, mujoco, lerobot" \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && python3 -c "from sensor_msgs.msg import Imu; Imu()"

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
    && mkdir -p /tmp/ros \
    && chmod 1777 /tmp/ros

WORKDIR ${EMBR_REPO}

RUN printf '\nsource /opt/ros/${ROS_DISTRO}/setup.bash\n' >> /home/${USERNAME}/.bashrc \
    && chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.bashrc

USER ${USERNAME}
CMD ["bash"]
