# either ubuntu:22.04 or l4t. ubuntu:22.04 is default
ARG BASE_IMAGE
FROM ${BASE_IMAGE:-ubuntu:22.04}

# from https://github.com/athackst/dockerfiles/blob/main/ros2/humble.Dockerfile
ENV DEBIAN_FRONTEND=noninteractive

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

# Install ROS2
RUN sudo add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*

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
# ========================

WORKDIR /root/ros_ws

# Install dev tools
RUN apt update && apt install -y \
    vim nano wget curl tree \
    cmake build-essential \
    less htop jq \
    python3-pip \
    tmux \
    gdb

# Install any additional ROS2 packages
RUN apt update -y && apt install -y \
    ros-dev-tools \
    ros-humble-mavros \ 
    ros-humble-tf2* \
    ros-humble-stereo-image-proc \
    ros-humble-image-view \
    ros-humble-topic-tools \
    ros-humble-grid-map \
    ros-humble-domain-bridge \
    libcgal-dev \
    python3-colcon-common-extensions
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

# Add ability to SSH
RUN apt-get update && apt-get install -y openssh-server
RUN mkdir /var/run/sshd

# Password is airstack
RUN echo 'root:airstack' | chpasswd
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

EXPOSE 22
    

# Cleanup. Prevent people accidentally doing git commits as root in Docker
RUN apt purge git -y \
    && apt autoremove -y \
    && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

