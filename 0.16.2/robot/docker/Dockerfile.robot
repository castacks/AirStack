# either ubuntu:22.04 or l4t. ubuntu:22.04 is default
ARG BASE_IMAGE
FROM ${BASE_IMAGE:-ubuntu:22.04}

ARG SKIP_MACVO=false

ARG UPDATE_FLAGS="-o Acquire::AllowInsecureRepositories=true -o Acquire::AllowDowngradeToInsecureRepositories=true"
ARG INSTALL_FLAGS="-o APT::Get::AllowUnauthenticated=true"

# from https://github.com/athackst/dockerfiles/blob/main/ros2/humble.Dockerfile
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# Install language
RUN apt-get ${UPDATE_FLAGS} update && apt-get ${INSTALL_FLAGS} install -y \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get ${UPDATE_FLAGS} update \
  && apt-get ${INSTALL_FLAGS} install -y tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get ${UPDATE_FLAGS} update \
    && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get ${UPDATE_FLAGS} update && apt-get ${INSTALL_FLAGS} install -y --no-install-recommends \
    emacs \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    wget \
    iputils-ping \
    net-tools \
  bind9-host \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2
RUN sudo add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get ${UPDATE_FLAGS} update -y && apt-get ${INSTALL_FLAGS} install -y --no-install-recommends \
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

# Install dev tools
RUN apt update && apt install -y \
  vim nano emacs wget curl tree \
  cmake build-essential \
  less htop jq \
  python3-pip \
  python3-rosdep \
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
  ros-humble-rosbag2-storage-mcap \
  ros-humble-xacro \
  libcgal-dev \
  python3-colcon-common-extensions

RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Install TensorRT
RUN if [ "$SKIP_MACVO" != "true" ]; then \
  wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu$(lsb_release -rs | tr -d .)/x86_64/cuda-keyring_1.1-1_all.deb && \ 
  dpkg -i cuda-keyring_1.1-1_all.deb && \
  apt update -y && \
  apt install -y \
    libnvinfer8 libnvinfer-dev libnvinfer-plugin8 \
    python3-libnvinfer python3-libnvinfer-dev; \
  fi

# Install Python dependencies
RUN pip3 install \
  empy \
  future \
  lxml \
  matplotlib==3.8.4 \
  numpy==1.24.0 \
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
  scipy \
  torch \
  torchvision \
  pypose \
  rich \
  tqdm \
  pillow \ 
  flow_vis \
  h5py \
  evo \
  tabulate \
  einops \
  timm==0.9.12 \
  rerun-sdk==0.22.0 \
  yacs \
  wandb \ 
  loguru \
  jaxtyping \
  kornia \
  typeguard==2.13.3 \
  onnx \
  tensorrt

# Override install newer openvdb 9.1.0 for compatibility with Ubuntu 22.04  https://bugs.launchpad.net/bugs/1970108
RUN apt remove -y libopenvdb*; \
  git clone --recurse --branch v9.1.0 https://github.com/wyca-robotics/openvdb.git /opt/openvdb && \
  mkdir /opt/openvdb/build && cd /opt/openvdb/build && \
  cmake .. && \
  make -j8 && make install && \
  cd ..; rm -rf /opt/openvdb/build

# Add ability to SSH
RUN apt-get ${UPDATE_FLAGS} update && apt-get ${INSTALL_FLAGS} install -y openssh-server libglfw3-dev libglm-dev
RUN mkdir /var/run/sshd

# Password is airstack
RUN echo 'root:airstack' | chpasswd
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

EXPOSE 22

# Install colcon, seems to be getting removed
RUN pip install -U colcon-common-extensions

# Downloading model weights for MACVO
WORKDIR /model_weights
RUN if [ "$SKIP_MACVO" != "true" ]; then \
    wget -r "https://github.com/MAC-VO/MAC-VO/releases/download/model/MACVO_FrontendCov.pth" && \ 
    wget -r "https://github.com/MAC-VO/MAC-VO/releases/download/model/MACVO_posenet.pkl" && \
    pwd && ls -R && \
    mv /model_weights/github.com/MAC-VO/MAC-VO/releases/download/model/MACVO_FrontendCov.pth /model_weights/MACVO_FrontendCov.pth && \
    mv /model_weights/github.com/MAC-VO/MAC-VO/releases/download/model/MACVO_posenet.pkl /model_weights/MACVO_posenet.pkl && \
    rm -rf /model_weights/github.com; \
  fi


# Fixes for MACVO Integration
RUN if [ "$SKIP_MACVO" != "true" ]; then pip install huggingface_hub; fi
RUN if [ "$SKIP_MACVO" != "true" ]; then pip uninstall matplotlib -y; fi

# Temporary fix for UFM
#WORKDIR /root/model_weights
#RUN if [ "$SKIP_MACVO" != "true" ]; then \
#    wget -r "https://github.com/castacks/MAC-VO-ROS2/releases/download/dsta-efficient-v0/UFM_Env2.zip" && \
#    apt update && apt install -y unzip && \
#    mv /root/model_weights/github.com/castacks/MAC-VO-ROS2/releases/download/dsta-efficient-v0/UFM_Env2.zip /root/model_weights/UFM_Env2.zip && \
#    unzip UFM_Env2.zip && \
#    rm UFM_Env2.zip; \
#    fi

#WORKDIR /root/model_weights/UFM
#RUN if [ "$SKIP_MACVO" != "true" ]; then pip install -e .; fi

#WORKDIR /root/model_weights/UFM/UniCeption
#RUN if [ "$SKIP_MACVO" != "true" ]; then pip install -e .; fi

#WORKDIR /root/model_weights/UFM/benchmarks
#RUN if [ "$SKIP_MACVO" != "true" ]; then pip install -e .; fi


# TMux config
RUN git clone https://github.com/tmux-plugins/tpm /root/.tmux/plugins/tpm

WORKDIR /root/AirStack/robot/ros_ws

# Cleanup
RUN apt autoremove -y \
  && apt clean -y \
  && rm -rf /var/lib/apt/lists/*

