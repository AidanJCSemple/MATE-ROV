FROM osrf/ros:jazzy-desktop

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    ros-jazzy-foxglove-bridge \
    python3-colcon-common-extensions \
    git \
 && rm -rf /var/lib/apt/lists/*

ENV GZ_VERSION=harmonic
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc
WORKDIR /workspace
