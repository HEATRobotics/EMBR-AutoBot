# ROS 2 Humble development environment for the EMBR physical workspace.
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

ARG USERNAME=embr
ARG USER_UID=1000
ARG USER_GID=1000

ENV HOME=/home/${USERNAME} \
    EMBR_REPO=/home/${USERNAME}/EMBR-AutoBot \
    EMBR_PHYS_WS=/home/${USERNAME}/EMBR-AutoBot/embr_phys/ros2_ws \
    ROS_HOME=/tmp/ros

COPY scripts/environment/ubuntu-packages.txt /tmp/ubuntu-packages.txt
RUN apt-get update \
    && grep -Ev '^[[:space:]]*(#|$)' /tmp/ubuntu-packages.txt \
        | xargs -r apt-get install -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/* /tmp/ubuntu-packages.txt

RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home \
        --shell /bin/bash "${USERNAME}" \
    && mkdir -p /tmp/ros \
    && chmod 1777 /tmp/ros

WORKDIR ${EMBR_REPO}

# Cache dependency installation and validate an initial build. Copy only source
# packages so host-generated build/install/log artifacts never enter the image.
COPY --chown=${USER_UID}:${USER_GID} embr_phys/ros2_ws/src/ ${EMBR_PHYS_WS}/src/
RUN apt-get update \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update \
    && rosdep install --from-paths ${EMBR_PHYS_WS}/src --ignore-src -r -y \
    && cd ${EMBR_PHYS_WS} \
    && colcon build --symlink-install \
    && chown -R "${USER_UID}:${USER_GID}" ${EMBR_PHYS_WS}

COPY docker/development-entrypoint.sh /development-entrypoint.sh
RUN chmod +x /development-entrypoint.sh \
    && printf '\n# Load ROS 2 and the EMBR physical workspace in interactive shells.\nsource /opt/ros/${ROS_DISTRO}/setup.bash\nif [ -f "${EMBR_PHYS_WS}/install/setup.bash" ]; then\n    source "${EMBR_PHYS_WS}/install/setup.bash"\nfi\n' >> /home/${USERNAME}/.bashrc \
    && chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.bashrc

USER ${USERNAME}
ENTRYPOINT ["/development-entrypoint.sh"]
CMD ["bash"]

LABEL maintainer="Maison Gulyas" \
      description="EMBR physical workspace development environment" \
      version="1.0"
