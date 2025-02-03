# Base Image Selection
ARG BASE_IMAGE
FROM ${BASE_IMAGE:-ubuntu:22.04}

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

#######################
# 1. Base System Setup
#######################

# Install language
RUN apt-get update && apt-get install -y \
    locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get update \
    && apt-get install -y tzdata \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

# System upgrade
RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    wget \
    && rm -rf /var/lib/apt/lists/*

#######################
# 2. ROS2 Core Setup
#######################

# Add universe repository and install prerequisites
RUN apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository universe

# Install ROS2 repository and packages
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# Set ROS2 Environment Variables
ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/humble
ENV COLCON_PREFIX_PATH=/opt/ros/humble
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib
ENV PATH=/opt/ros/humble/bin:$PATH
ENV PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ENV DEBIAN_FRONTEND=

WORKDIR /root/ros_ws

#######################
# 3. Development Tools
#######################

# Install dev tools
RUN apt update && apt install -y \
    vim nano wget curl tree \
    cmake build-essential \
    less htop jq \
    python3-pip \
    tmux \
    gdb

#######################
# 4. ROS2 Additional Packages
#######################

# Install OpenCV Python bindings
RUN apt update -y && apt install -y \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 tools and packages individually to better handle dependencies
RUN apt update -y && apt install -y ros-dev-tools
RUN apt update -y && apt install -y ros-humble-mavros
RUN apt update -y && apt install -y ros-humble-tf2-msgs ros-humble-tf2-py ros-humble-tf2-ros
RUN apt update -y && apt install -y ros-humble-topic-tools
# RUN apt update -y && apt install -y ros-humble-grid-map
RUN apt update -y && apt install -y ros-humble-domain-bridge
RUN apt update -y && apt install -y libcgal-dev
RUN apt update -y && apt install -y python3-colcon-common-extensions

# Install image-processing related packages separately
RUN apt update -y && apt install -y \
    ros-humble-vision-opencv \
    ros-humble-stereo-image-proc \
    ros-humble-image-view

# Install geographiclib datasets for MAVROS
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

#######################
# 5. Python Dependencies
#######################
    
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

#######################
# 6. Additional Software
#######################

# Install OpenVDB 9.1.0
RUN apt remove -y libopenvdb*; \
    git clone --recurse --branch v9.1.0 https://github.com/wyca-robotics/openvdb.git /opt/openvdb && \
    mkdir /opt/openvdb/build && cd /opt/openvdb/build && \
    cmake .. && \
    make -j8 && make install && \
    cd ..; rm -rf /opt/openvdb/build

#######################
# 7. System Configuration
#######################

# Setup SSH Access
RUN apt-get update && apt-get install -y openssh-server
RUN mkdir /var/run/sshd

# Password is airstack
RUN echo 'root:airstack' | chpasswd
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

EXPOSE 22

#######################
# 8. Conditional Setup
#######################

ARG REAL_ROBOT=True
RUN if [ "$REAL_ROBOT"  = "true" ]; then \
  # Put commands here that should run for the real robot but not the sim
  echo "REAL_ROBOT is true"; \
  apt-get update && apt-get install -y libimath-dev; \
else \
  # Put commands here that should be run for the sim but not the real robot
  echo "REAL_ROBOT is false"; \
fi

#######################
# 9. Final Cleanup
#######################

# Cleanup
RUN apt purge git -y \
    && apt autoremove -y \
    && apt clean -y \
    && rm -rf /var/lib/apt/lists/*