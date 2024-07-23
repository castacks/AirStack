# Note the build context should be the root of the repository (AirStack/)

FROM nvcr.io/nvidia/isaac-sim:4.0.0

# Trick to get apt-get to not prompt for timezone in tzdata
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=America/New_York
################
# DEPENDENCIES #
################

# Install ROS2 Humble

RUN apt update && apt install locales -y
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN apt install software-properties-common -y
RUN add-apt-repository universe
RUN apt update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update
RUN apt upgrade -y
RUN apt install ros-humble-desktop ros-dev-tools -y
RUN rosdep init && rosdep update

# Package dependencies
# perform ROS dependency installation for our workspace.
COPY ros_ws/src /tmp/ros_ws/src
RUN rosdep install --from-paths /tmp/ros_ws/src --ignore-src -r -y && rm -rf /tmp/ros_ws

# vdb_mapping
# Build and install patched version of OpenVDB (see https://github.com/SteveMacenski/spatio_temporal_voxel_layer/issues/232).
RUN apt-get remove -y libopenvdb*; apt-get update && apt-get install -y libboost-system-dev libboost-iostreams-dev libtbb-dev libblosc-dev; \
    git clone --recurse --branch v8.2.0-debian https://github.com/wyca-robotics/openvdb.git /opt/openvdb && \
    mkdir /opt/openvdb/build && cd /opt/openvdb/build && \
    cmake .. && \
    make -j8 && make install && \
    cd ..; rm -rf /opt/openvdb/build

# Install MAVROS
RUN apt install ros-humble-mavros -y
RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh && chmod +x install_geographiclib_datasets.sh && ./install_geographiclib_datasets.sh


# Install isaac sim python dependencies. we might not need all of these
RUN /isaac-sim/python.sh -m pip install scipy
RUN /isaac-sim/python.sh -m pip install git+https://github.com/dronekit/dronekit-python#egg=dronekit
# Install python dependencies for mavlink:
RUN apt install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame -y
RUN pip install PyYAML mavproxy
# install docs dependencies
RUN pip install mkdocs-material

RUN apt install gstreamer1.0-gl -y


##############
# DEV TOOLS #
##############
RUN apt install vim nano -y

# bashrc
COPY docker/.bashrc /root/.bashrc

WORKDIR /root/AirStack