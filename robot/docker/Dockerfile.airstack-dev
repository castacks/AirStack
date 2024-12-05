FROM osrf/ros:humble-desktop-full

WORKDIR /root/ros_ws

RUN apt update
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

