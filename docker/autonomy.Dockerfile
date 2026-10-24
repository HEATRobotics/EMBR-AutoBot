# ROS 2 Humble autonomy environment for EMBR (MuJoCo, Gymnasium, SB3, W&B, ONNX, Nav2).
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    MUJOCO_GL=osmesa

SHELL ["/bin/bash", "-c"]

ARG USERNAME=embr
ARG USER_UID=1000
ARG USER_GID=1000

ENV HOME=/home/${USERNAME} \
    EMBR_REPO=/home/${USERNAME}/EMBR-AutoBot \
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
    && mkdir -p /tmp/ros \
    && chmod 1777 /tmp/ros

WORKDIR ${EMBR_REPO}

RUN printf '\n# Load ROS 2 in interactive shells.\nsource /opt/ros/${ROS_DISTRO}/setup.bash\n' >> /home/${USERNAME}/.bashrc \
    && chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.bashrc

USER ${USERNAME}
CMD ["bash"]

LABEL maintainer="Anvi" \
      description="EMBR autonomy environment" \
      version="1.0"