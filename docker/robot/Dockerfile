# expects context to be the root of the repository, i.e. AirStack/
FROM osrf/ros:humble-desktop-full

WORKDIR /root/AirStack

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
ENV QT_X11_NO_MITSHM=1
ENV EDITOR=nano
ENV XDG_RUNTIME_DIR=/tmp

COPY docker/robot/entrypoint.bash /entrypoint.bash


RUN apt update
# Install dev tools
RUN apt install -y \
    vim nano wget curl tree \
    cmake build-essential \
    less htop jq \
    python3-pip

# Install ROS2 packages

# Package dependencies
# perform ROS dependency installation for our workspace.
COPY ros_ws/src /tmp/ros_ws/src
RUN rosdep init && rosdep update && \
    rosdep install --from-paths /tmp/ros_ws/src --ignore-src -r -y && \
    rm -rf /tmp/ros_ws || echo "Some ROS dependencies installation failed"

# Install any additional ROS2 packages
RUN apt install -y \
    ros-dev-tools \
    ros-humble-mavros \ 
    ros-humble-tf2*

RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh


# Install Python dependencies
RUN pip3 install \
    empy \
    future \
    lxml \
    matplotlib \
    numpy \
    pkgconfig \
    psutil \
    pygments \
    wheel \
    pymavlink \
    pyyaml \
    requests \
    setuptools \
    six \
    toml \
    scipy


# Override install newer openvdb 8.2.0 for compatibility with Ubuntu 22.04  https://bugs.launchpad.net/bugs/1970108
RUN apt remove -y libopenvdb*; \
    git clone --recurse --branch v8.2.0-debian https://github.com/wyca-robotics/openvdb.git /opt/openvdb && \
    mkdir /opt/openvdb/build && cd /opt/openvdb/build && \
    cmake .. && \
    make -j8 && make install && \
    cd ..; rm -rf /opt/openvdb/build

# Cleanup
RUN apt autoremove -y \
    && apt clean -y \
    && rm -rf /var/lib/apt/lists/*
