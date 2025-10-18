FROM --platform=$BUILDPLATFORM ghcr.io/high-flyers/ros-core:latest

ARG USERNAME=hf
ARG ROS_DISTRO=humble

USER ${USERNAME}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV ROS_WS=/home/${USERNAME}/ws
WORKDIR ${ROS_WS}/src
RUN git clone "https://github.com/PX4/px4_msgs" --branch "release/1.15"
RUN git clone "https://github.com/eProsima/Micro-XRCE-DDS-Agent" --branch v2.4.1

RUN sudo apt-get update && sudo apt-get -y --quiet --no-install-recommends install \
    ros-${ROS_DISTRO}-rviz2 \
    && sudo rm -rf /var/lib/apt/lists/*

RUN sudo pip3 install -U numpy numpy-quaternion

RUN usermod -aG dialout ${USERNAME}

WORKDIR ${ROS_WS}
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    colcon build

WORKDIR ${ROS_WS}/src
COPY --chown=${USERNAME} . imav

WORKDIR ${ROS_WS}/src/imav
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    scripts/build.sh

RUN echo "source \"/opt/ros/${ROS_DISTRO}/setup.bash\"" >> "/home/${USERNAME}/.bashrc" && \
    echo "source \"${ROS_WS}/install/setup.bash\"" >> "/home/${USERNAME}/.bashrc" && \
    echo "source \"${ROS_WS}/src/imav/install/setup.bash\"" >> "/home/${USERNAME}/.bashrc"

CMD ["/bin/bash"]
