FROM ros:humble-ros-base

ARG USERNAME=hf
ARG ROS_DISTRO=humble
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
    build-essential \
    cmake \
    curl \
    ros-dev-tools \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV ROS_WS=/home/${USERNAME}/ws
WORKDIR ${ROS_WS}/src
RUN git clone "https://github.com/PX4/px4_msgs" --branch "release/1.15"

RUN git clone "https://github.com/eProsima/Micro-XRCE-DDS-Agent" --branch v3.0.1
WORKDIR ${ROS_WS}/src/Micro-XRCE-DDS-Agent/build
RUN cmake .. && make && sudo make install && sudo ldconfig /usr/local/lib

RUN sudo apt-get update && sudo apt-get -y --quiet --no-install-recommends install \
    ros-${ROS_DISTRO}-rviz2 \
    && sudo rm -rf /var/lib/apt/lists/*

WORKDIR ${ROS_WS}
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    colcon build

WORKDIR ${ROS_WS}/src
COPY . imav
RUN sudo chmod +x imav/scripts/build.sh

WORKDIR ${ROS_WS}
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    src/imav/scripts/build.sh

RUN echo "source \"/opt/ros/${ROS_DISTRO}/setup.bash\"" >> "/home/${USERNAME}/.bashrc" && \
    echo "source \"${ROS_WORKSPACE}/install/setup.bash\"" >> "/home/${USERNAME}/.bashrc"

RUN sudo sed -i '$i source $ROS_WORKSPACE/install/setup.bash' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
