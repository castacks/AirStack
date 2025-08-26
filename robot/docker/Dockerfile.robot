# either ubuntu:22.04 or l4t. ubuntu:22.04 is default
ARG BASE_IMAGE
FROM ${BASE_IMAGE:-ubuntu:22.04}

# from https://github.com/athackst/dockerfiles/blob/main/ros2/humble.Dockerfile
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

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

RUN apt-get update \
  && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
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
  && apt-get update -y && apt-get install -y --no-install-recommends \
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
  libcgal-dev \
  python3-colcon-common-extensions

RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Install TensorRT
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu$(lsb_release -rs | tr -d .)/x86_64/cuda-keyring_1.1-1_all.deb && \ 
  dpkg -i cuda-keyring_1.1-1_all.deb && \
  apt update -y && \
  apt install -y \
  libnvinfer8 libnvinfer-dev libnvinfer-plugin8 \
  python3-libnvinfer python3-libnvinfer-dev

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
RUN apt-get update && apt-get install -y openssh-server
RUN mkdir /var/run/sshd

# Password is airstack
RUN echo 'root:airstack' | chpasswd
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

EXPOSE 22

ARG REAL_ROBOT=false
RUN if [ "$REAL_ROBOT"  = "true" ]; then \
  # Put commands here that should run for the real robot but not the sim
  echo "REAL_ROBOT is true"; \
  apt-get update && apt-get install -y libimath-dev; \
  else \
  # Put commands here that should be run for the sim but not the real robot
  echo "REAL_ROBOT is false"; \
  fi

# Downloading model weights for MACVO
WORKDIR /model_weights
RUN wget -r "https://github.com/MAC-VO/MAC-VO/releases/download/model/MACVO_FrontendCov.pth" && \ 
  wget -r "https://github.com/MAC-VO/MAC-VO/releases/download/model/MACVO_posenet.pkl" && \
  wget -r "https://github.com/castacks/MAC-VO-ROS2/releases/download/dsta-efficient-v0/dsta_efficient.ckpt" && \ 
  mv /model_weights/github.com/MAC-VO/MAC-VO/releases/download/model/MACVO_FrontendCov.pth /model_weights/MACVO_FrontendCov.pth && \
  mv /model_weights/github.com/MAC-VO/MAC-VO/releases/download/model/MACVO_posenet.pkl /model_weights/MACVO_posenet.pkl && \
  mv /model_weights/github.com/castacks/MAC-VO-ROS2/releases/download/dsta-efficient-v0/dsta_efficient.ckpt /model_weights/dsta_efficient.ckpt && \
  rm -rf /model_weights/github.com

# Install colcon, seems to be getting removed
RUN pip install -U colcon-common-extensions

# Fixes for MACVO Integration
RUN pip install huggingface_hub
RUN pip uninstall matplotlib -y

# Temporary fix for UFM
WORKDIR /model_weights
RUN wget -r "https://github.com/castacks/MAC-VO-ROS2/releases/download/dsta-efficient-v0/UFM_Env2.zip" && \
  apt update && apt install -y unzip && \
  mv /model_weights/github.com/castacks/MAC-VO-ROS2/releases/download/dsta-efficient-v0/UFM_Env2.zip /model_weights/UFM_Env2.zip && \
  unzip UFM_Env2.zip && \
  rm UFM_Env2.zip

WORKDIR /model_weights/UFM
RUN pip install -e .

WORKDIR /model_weights/UFM/UniCeption
RUN pip install -e .

WORKDIR /model_weights/UFM/benchmarks
RUN pip install -e .

# TMux config
RUN git clone https://github.com/tmux-plugins/tpm /home/robot/.tmux/plugins/tpm

WORKDIR /home/robot/ros_ws

# Make it so that files created within the container reflect the user's UID/GID so they don't have to change file permissions from root. See https://github.com/boxboat/fixuid
# need to give access to docker to access container name
# creates user "robot" with UID 1000, home directory /home/robot, and shell /bin/sh
# creates group "robot" with GID 1000
RUN addgroup --gid 1000 robot && \
  adduser --uid 1000 --ingroup robot --home /home/robot --shell /bin/sh --disabled-password --gecos "" robot && \
  chown -R robot:robot /home/robot

RUN USER=robot && \
  GROUP=robot && \
  curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.6.0/fixuid-0.6.0-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
  chown root:root /usr/local/bin/fixuid && \
  chmod 4755 /usr/local/bin/fixuid && \
  mkdir -p /etc/fixuid && \
  printf "user: $USER\ngroup: $GROUP\n" > /etc/fixuid/config.yml

# Cleanup. Prevent people accidentally doing git commits from within Docker
RUN apt purge git -y \
  && apt autoremove -y \
  && apt clean -y \
  && rm -rf /var/lib/apt/lists/*

USER robot:robot
ENTRYPOINT ["fixuid"]

