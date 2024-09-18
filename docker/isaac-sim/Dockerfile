# Assumes build context to be the repository root
FROM nvcr.io/nvidia/isaac-sim:4.1.0

WORKDIR /isaac-sim

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    unzip \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN set -eux; \
       key='C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'; \
       export GNUPGHOME="$(mktemp -d)"; \
       gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
       mkdir -p /usr/share/keyrings; \
       gpg --batch --export "$key" > /usr/share/keyrings/ros2-latest-archive-keyring.gpg; \
       gpgconf --kill all; \
       rm -rf "$GNUPGHOME"

# setup sources.list
RUN echo "deb [ signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg ] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=humble

# Install ROS2 packages
RUN apt update && apt install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [ signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg ] http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2-latest.list
RUN apt update && \
    apt install -y \
    cmake \
    git \
    ros-humble-desktop \
    ros-dev-tools \
    python3-pip \
    python3-rosdep \
    ros-humble-tf2*

RUN pip install git+https://github.com/dronekit/dronekit-python#egg=dronekit
RUN pip install PyYAML mavproxy tmuxp scipy

# Install Isaac Sim ROS2 workspace so that we can run the isaacsim ROS package
RUN cd /tmp/ && \
    curl -L -O https://github.com/isaac-sim/IsaacSim-ros_workspaces/archive/main.zip && \
    unzip main.zip && \
    mv IsaacSim-ros_workspaces-main/humble_ws /humble_ws && \
    cd /humble_ws && \
    . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install && \
    rm -rf /tmp/IsaacSim-ros_workspaces-main

# copy over the AscentAeroSystemsSITLPackage to the drag and drop folder but also need to copy extras folder for this
ADD docker/extras /extras
COPY docker/isaac-sim/AscentAeroSystemsSITLPackage /extras/drag_and_drop/

COPY ros_ws/fastdds.xml /isaac-sim/fastdds.xml

# Cleanup. Prevent people accidentally doing git commits as root in Docker
RUN apt purge -y git &&  apt autoremove -y \
    && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

