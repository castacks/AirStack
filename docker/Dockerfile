FROM nvcr.io/nvidia/isaac-sim:4.1.0

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
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO humble

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop ros-dev-tools emacs nano vim curl net-tools wget vim tmux less htop \
    ca-certificates gnupg lsb-core sudo wget \
    astyle build-essential cmake cppcheck file g++ gcc gdb git lcov libfuse2 libxml2-dev libxml2-utils \
    make ninja-build python3 python3-dev python3-pip python3-setuptools python3-wheel rsync shellcheck unzip zip \
    automake binutils-dev bison build-essential flex g++-multilib gcc-multilib gdb-multiarch genromfs gettext gperf \
    libelf-dev libexpat-dev libgmp-dev libisl-dev libmpc-dev libmpfr-dev libncurses5 libncurses5-dev libncursesw5-dev \
    libtool pkg-config screen texinfo u-boot-tools util-linux vim-common bc openjdk-11-jre openjdk-11-jdk libvecmath-java \
    dmidecode gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly gstreamer1.0-libav libeigen3-dev libgstreamer-plugins-base1.0-dev libgstreamer-gl1.0-0 \
    libimage-exiftool-perl libopencv-dev libxml2-utils pkg-config protobuf-compiler lsb-release libasio-dev ros-humble-mavros \
    kmod lsof libfuse-dev iproute2 tcpdump xterm \
    python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame \
    && rm -rf /var/lib/apt/lists/*
# Package dependencies
# perform ROS dependency installation for our workspace.
COPY ros_ws/src /tmp/ros_ws/src
RUN rosdep init && rosdep update && rosdep install --from-paths /tmp/ros_ws/src --ignore-src -r -y && rm -rf /tmp/ros_ws || echo "Some ROS dependencies installation failed"

# Install Gazebo Harmonic
RUN apt-get update && apt-get install -y lsb-release gnupg
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install -y gz-harmonic

# package vdb_mapping
# Build and install patched version of OpenVDB (see https://github.com/SteveMacenski/spatio_temporal_voxel_layer/issues/232). Hopefully this gets fixed in the future.
RUN apt-get remove -y libopenvdb*; apt-get update && apt-get install -y libboost-system-dev libboost-iostreams-dev libtbb-dev libblosc-dev; \
    git clone --recurse --branch v8.2.0-debian https://github.com/wyca-robotics/openvdb.git /opt/openvdb && \
    mkdir /opt/openvdb/build && cd /opt/openvdb/build && \
    cmake .. && \
    make -j8 && make install && \
    cd ..; rm -rf /opt/openvdb/build

RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
# Install ROS2 packages
RUN apt install ros-humble-tf2* ros-humble-perception-pcl libpcl-dev -y

RUN  wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt update
RUN apt-get update && apt-get install -y --no-install-recommends gz-garden \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install argcomplete argparse>=1.2 cerberus coverage empy==3.3.4 future jinja2>=2.8 jsonschema kconfiglib lxml matplotlib>=3.0.* numpy>=1.13 nunavut>=1.1.0 packaging pandas>=0.21 pkgconfig psutil pygments wheel>=0.31.1 pymavlink pyros-genmsg pyserial pyulog>=0.5.0 pyyaml requests setuptools>=39.2.0 six>=1.12.0 toml>=0.9 sympy>=1.10.1 scipy mkdocs-material

#ADD dronekit-python /docker_build/dronekit-python
#RUN /isaac-sim/kit/python.sh -m pip install /docker_build/dronekit-python

RUN /isaac-sim/python.sh -m pip install git+https://github.com/dronekit/dronekit-python#egg=dronekit

RUN pip3 install PyYAML mavproxy tmuxp

#RUN echo 'root:passme24' | chpasswd

#RUN useradd -G video -ms /bin/bash uav
#RUN usermod -a -G dialout uav
#RUN usermod -a -G sudo uav
#RUN echo 'uav:passme24' | chpasswd
#USER uav
#WORKDIR /home/uav