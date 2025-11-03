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
    libclang-dev \
    python3-vcstool \
    && sudo rm -rf /var/lib/apt/lists/*

RUN curl https://sh.rustup.rs -sSf | bash -s -- -y
RUN source "~/.bashrc" && cargo install cargo-ament-build

RUN sudo pip3 install -U numpy numpy-quaternion
RUN sudo pip3 install git+https://github.com/colcon/colcon-cargo.git
RUN sudo pip3 install git+https://github.com/colcon/colcon-ros-cargo.git

RUN sudo usermod -aG dialout ${USERNAME}

WORKDIR ${ROS_WS}
RUN git clone https://github.com/ros2-rust/ros2_rust.git src/ros2_rust
RUN vcs import src < src/ros2_rust/ros2_rust_humble.repos

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
