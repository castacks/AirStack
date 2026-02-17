# either ubuntu:22.04 or l4t. ubuntu:22.04 is default
ARG BASE_IMAGE
# ============================================================
# Stage 1 — builder: compile/download everything
# ============================================================
FROM ${BASE_IMAGE:-ubuntu:22.04} AS builder

# Re-declare ARGs (Docker ARGs do not persist across FROM)
ARG BASE_IMAGE
ARG REAL_ROBOT
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

# Install dev tools (includes build-time tools: cmake, build-essential)
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

# Install TensorRT (NVIDIA/L4T images only)
RUN if echo "$BASE_IMAGE" | grep -qE "(nvidia|l4t)"; then \
  wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu$(lsb_release -rs | tr -d .)/x86_64/cuda-keyring_1.1-1_all.deb && \
  dpkg -i cuda-keyring_1.1-1_all.deb && \
  apt update -y && \
  apt install -y \
    libnvinfer8 libnvinfer-dev libnvinfer-plugin8 \
    python3-libnvinfer python3-libnvinfer-dev; \
  fi

# Install Python dependencies (unconditional)
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
  typeguard==2.13.3

# Install NVIDIA-only Python dependencies (NVIDIA/L4T images only)
RUN if echo "$BASE_IMAGE" | grep -qE "(nvidia|l4t)"; then \
  pip3 install \
    torch \
    torchvision \
    onnx \
    tensorrt; \
  fi

# Override install newer openvdb 9.1.0 for compatibility with Ubuntu 22.04  https://bugs.launchpad.net/bugs/1970108
RUN apt remove -y libopenvdb*; \
  git clone --recurse --branch v9.1.0 https://github.com/wyca-robotics/openvdb.git /opt/openvdb && \
  mkdir /opt/openvdb/build && cd /opt/openvdb/build && \
  cmake .. && \
  make -j8 && make install && \
  cd ..; rm -rf /opt/openvdb/build

# Install colcon, seems to be getting removed
RUN pip install -U colcon-common-extensions

# Downloading model weights for MACVO (NVIDIA/L4T images only)
WORKDIR /model_weights
RUN if echo "$BASE_IMAGE" | grep -qE "(nvidia|l4t)"; then \
    wget -r "https://github.com/MAC-VO/MAC-VO/releases/download/model/MACVO_FrontendCov.pth" && \
    wget -r "https://github.com/MAC-VO/MAC-VO/releases/download/model/MACVO_posenet.pkl" && \
    pwd && ls -R && \
    mv /model_weights/github.com/MAC-VO/MAC-VO/releases/download/model/MACVO_FrontendCov.pth /model_weights/MACVO_FrontendCov.pth && \
    mv /model_weights/github.com/MAC-VO/MAC-VO/releases/download/model/MACVO_posenet.pkl /model_weights/MACVO_posenet.pkl && \
    rm -rf /model_weights/github.com; \
  fi

# Fixes for MACVO Integration (NVIDIA/L4T images only)
RUN if echo "$BASE_IMAGE" | grep -qE "(nvidia|l4t)"; then pip install huggingface_hub; fi
RUN if echo "$BASE_IMAGE" | grep -qE "(nvidia|l4t)"; then pip uninstall matplotlib -y; fi

# TMux config
RUN git clone https://github.com/tmux-plugins/tpm /root/.tmux/plugins/tpm

# Cleanup
RUN apt autoremove -y \
  && apt clean -y \
  && rm -rf /var/lib/apt/lists/*


# ============================================================
# Stage 2 — runtime: lean final image
# ============================================================
FROM ${BASE_IMAGE:-ubuntu:22.04} AS runtime

# Re-declare ARGs (Docker ARGs do not persist across FROM)
ARG BASE_IMAGE
ARG REAL_ROBOT
ARG UPDATE_FLAGS="-o Acquire::AllowInsecureRepositories=true -o Acquire::AllowDowngradeToInsecureRepositories=true"
ARG INSTALL_FLAGS="-o APT::Get::AllowUnauthenticated=true"

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

# Carry over all ROS2 ENV vars from the builder stage
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

# Install runtime dev tools (no cmake or build-essential)
RUN apt update && apt install -y \
  vim nano emacs wget curl tree \
  less htop jq \
  python3-pip \
  python3-rosdep \
  tmux \
  gdb \
  && rm -rf /var/lib/apt/lists/*

# Install runtime ROS2 packages (no libcgal-dev)
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
  python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/*

RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Install NVIDIA runtime apt packages (no -dev counterparts; NVIDIA/L4T images only)
RUN if echo "$BASE_IMAGE" | grep -qE "(nvidia|l4t)"; then \
  wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu$(lsb_release -rs | tr -d .)/x86_64/cuda-keyring_1.1-1_all.deb && \
  dpkg -i cuda-keyring_1.1-1_all.deb && \
  apt update -y && \
  apt install -y \
    libnvinfer8 libnvinfer-plugin8 \
    python3-libnvinfer \
  && rm -rf /var/lib/apt/lists/*; \
  fi

# Add ability to SSH (libglfw3-dev and libglm-dev kept per spec)
RUN apt-get ${UPDATE_FLAGS} update && apt-get ${INSTALL_FLAGS} install -y \
  openssh-server libglfw3-dev libglm-dev \
  && rm -rf /var/lib/apt/lists/*
RUN mkdir /var/run/sshd

# Copy build artifacts from the builder stage
COPY --from=builder /opt/ros/humble          /opt/ros/humble
COPY --from=builder /usr/local/lib/python3.10 /usr/local/lib/python3.10
COPY --from=builder /usr/local/bin            /usr/local/bin
COPY --from=builder /usr/local/include        /usr/local/include
COPY --from=builder /usr/local/lib            /usr/local/lib
COPY --from=builder /model_weights            /model_weights
COPY --from=builder /root/.tmux               /root/.tmux

# Password is airstack
RUN echo 'root:airstack' | chpasswd
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

EXPOSE 22

WORKDIR /root/AirStack/robot/ros_ws

