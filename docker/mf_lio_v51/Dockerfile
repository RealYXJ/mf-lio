FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libpcl-dev \
    python3-colcon-common-extensions \
    ros-jazzy-pcl-conversions \
    ros-jazzy-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
RUN mkdir -p src

# Livox-SDK2 is the C++ SDK livox_ros_driver2 links against
# (provides liblivox_lidar_sdk_shared.so) -- must be built+installed first.
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 && \
    cd /tmp/Livox-SDK2 && mkdir build && cd build && \
    cmake .. && make -j"$(nproc)" && make install && \
    ldconfig && \
    rm -rf /tmp/Livox-SDK2

# livox_ros_driver2 provides the CustomMsg type mf_lio_v51 depends on.
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git src/livox_ros_driver2

COPY ros2_ws/src/mf_lio_v51 src/mf_lio_v51

# livox_ros_driver2's own build script wants to run from its own directory
# and needs ROS_DISTRO/humble-or-later detection; build it first, standalone,
# then build mf_lio_v51 against it.
RUN source /opt/ros/jazzy/setup.bash && \
    cd src/livox_ros_driver2 && \
    ./build.sh jazzy

RUN source /opt/ros/jazzy/setup.bash && \
    source /ws/install/setup.bash 2>/dev/null || true && \
    colcon build --packages-select mf_lio_v51 --cmake-args -DCMAKE_BUILD_TYPE=Release

CMD ["/bin/bash"]
