# either ubuntu:22.04 or l4t
ARG BASE_IMAGE
FROM ${BASE_IMAGE}

# ========================
# install ros2 humble https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
RUN apt update && apt install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN apt install -y software-properties-common
RUN add-apt-repository universe

RUN apt update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && apt install -y ros-humble-desktop

# ========================

WORKDIR /root/ros_ws

# Install dev tools
RUN apt install -y \
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
    libcgal-dev 
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

