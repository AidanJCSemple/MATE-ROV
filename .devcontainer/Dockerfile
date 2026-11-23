FROM osrf/ros:jazzy-desktop

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-rmw-zenoh-cpp \
    python3-colcon-common-extensions \
    python3-pytest \
    git \
 && rm -rf /var/lib/apt/lists/*

ENV GZ_VERSION=harmonic
# Same middleware as the team's watod containers, so the sim can join their ROS network.
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc
WORKDIR /workspace
