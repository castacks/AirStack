# either ubuntu:24.04 or l4t. ubuntu:24.04 is default
ARG BASE_IMAGE
ARG ENABLE_RAYFRONTS=false
ARG FINAL_STAGE=runtime
# ============================================================
# Stage 1 — builder: compile/download everything
# ============================================================
FROM ${BASE_IMAGE:-ubuntu:24.04} AS builder

# Re-declare ARGs (Docker ARGs do not persist across FROM)
ARG BASE_IMAGE
ARG REAL_ROBOT
ARG UPDATE_FLAGS="-o Acquire::AllowInsecureRepositories=true -o Acquire::AllowDowngradeToInsecureRepositories=true"
ARG INSTALL_FLAGS="-o APT::Get::AllowUnauthenticated=true"
ARG SKIP_MACVO=false
ARG SKIP_TENSORRT=false
ARG ENABLE_RAYFRONTS

# from https://github.com/athackst/dockerfiles/blob/main/ros2/jazzy.Dockerfile
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
  ros-jazzy-desktop \
  python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=jazzy
ENV AMENT_PREFIX_PATH=/opt/ros/jazzy
ENV COLCON_PREFIX_PATH=/opt/ros/jazzy
ENV LD_LIBRARY_PATH=/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/lib
ENV PATH=/opt/ros/jazzy/bin:$PATH
ENV PYTHONPATH=/opt/ros/jazzy/local/lib/python3.12/dist-packages:/opt/ros/jazzy/lib/python3.12/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ENV DEBIAN_FRONTEND=
# ========================

# Install dev tools (includes build-time tools: cmake, build-essential)
RUN apt update && apt install -y --no-install-recommends \
  vim nano tree \
  cmake build-essential \
  less htop jq \
  python3-pip \
  python3-rosdep \
  tmux \
  gdb \
  && rm -rf /var/lib/apt/lists/*

# Install any additional ROS2 packages
RUN apt update -y && apt install -y --no-install-recommends \
  ros-dev-tools \
  ros-jazzy-mavros \
  ros-jazzy-tf2* \
  ros-jazzy-stereo-image-proc \
  ros-jazzy-image-view \
  ros-jazzy-topic-tools \
  ros-jazzy-grid-map \
  ros-jazzy-domain-bridge \
  ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-xacro \
  ros-jazzy-foxglove-bridge \
  libcgal-dev \
  python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/*

RUN /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

# Install TensorRT (NVIDIA/L4T images only, unless SKIP_TENSORRT=true)
# Note: TensorRT 8 packages may not be available for Ubuntu 24.04, so this is optional
RUN if echo "$BASE_IMAGE" | grep -qE "(nvidia|l4t)" && [ "${SKIP_TENSORRT}" != "true" ]; then \
  if [ ! -f /etc/apt/sources.list.d/cuda*.list ]; then \
  wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu$(lsb_release -rs | tr -d .)/x86_64/cuda-keyring_1.1-1_all.deb && \
  dpkg -i cuda-keyring_1.1-1_all.deb || true; \
  fi && \
  apt update -y && \
  apt install -y --no-install-recommends \
  libnvinfer10 libnvinfer-dev libnvinfer-plugin10 \
  python3-libnvinfer python3-libnvinfer-dev; \
  fi

# Install Python dependencies (unconditional)
# Note: numpy>=1.26 required for Python 3.12 compatibility
# Using --ignore-installed to avoid conflicts with system packages
RUN pip3 install --break-system-packages --ignore-installed \
  empy==3.3.4 \
  future \
  lxml \
  matplotlib==3.8.4 \
  # numpy must be <2.0 for MACVO
  numpy~=1.26.4 \
  pkgconfig \
  psutil \
  pygments \
  wheel \
  pymavlink \
  pyyaml \
  requests \
  # setup tools must be <80 for Jazzy https://github.com/ros2/ros2/issues/1702#issuecomment-3007929996
  setuptools==79.0.1 \
  six \
  toml \
  scipy \
  scikit-learn \
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

# Install MACVO Python dependencies (skipped if SKIP_MACVO=true)
RUN if [ "${SKIP_MACVO}" != "true" ]; then \
    if [ "${ENABLE_RAYFRONTS}" = "true" ]; then \
      pip3 install --break-system-packages \
        --index-url https://download.pytorch.org/whl/cu130 \
        torch==2.9.1 torchvision torchaudio && \
      pip3 install --break-system-packages onnx tensorrt; \
    else \
      pip3 install --break-system-packages \
        torch torchvision onnx tensorrt; \
    fi; \
  fi

# Downloading model weights for MACVO (skipped if SKIP_MACVO=true)
WORKDIR /model_weights
RUN if [ "${SKIP_MACVO}" != "true" ]; then \
  wget -r "https://github.com/MAC-VO/MAC-VO/releases/download/model/MACVO_FrontendCov.pth" && \
  wget -r "https://github.com/MAC-VO/MAC-VO/releases/download/model/MACVO_posenet.pkl" && \
  pwd && ls -R && \
  mv /model_weights/github.com/MAC-VO/MAC-VO/releases/download/model/MACVO_FrontendCov.pth /model_weights/MACVO_FrontendCov.pth && \
  mv /model_weights/github.com/MAC-VO/MAC-VO/releases/download/model/MACVO_posenet.pkl /model_weights/MACVO_posenet.pkl && \
  rm -rf /model_weights/github.com; \
  fi

# Fixes for MACVO Integration (skipped if SKIP_MACVO=true)
RUN if [ "${SKIP_MACVO}" != "true" ]; then \
  pip install --break-system-packages huggingface_hub; \
  fi

# TMux config
RUN git clone --depth 1 https://github.com/tmux-plugins/tpm /root/.tmux/plugins/tpm

# Install eProsima DDS Router
# System library dependencies (Asio, TinyXML2, OpenSSL, yaml-cpp)
RUN apt update && apt install -y --no-install-recommends \
  libasio-dev libtinyxml2-dev libssl-dev libyaml-cpp-dev \
  && rm -rf /var/lib/apt/lists/*

# Clone sources and build + install globally into /usr/local. Pinned to v3.4.0
RUN mkdir -p /tmp/DDS-Router/src \
  && cd /tmp/DDS-Router \
  && wget https://raw.githubusercontent.com/eProsima/DDS-Router/v3.4.0/ddsrouter.repos \
  && vcs import src < ddsrouter.repos \
  && colcon build --merge-install --install-base /usr/local \
  && rm -rf /tmp/DDS-Router

# RayFronts deps (builder-stage, gated on ENABLE_RAYFRONTS=true)
RUN if [ "${ENABLE_RAYFRONTS}" = "true" ]; then \
    pip3 install --break-system-packages \
      hydra-core open_clip_torch "transformers<5" \
      git+https://github.com/facebookresearch/segment-anything.git \
      ftfy regex nanobind pandas protobuf \
      "scipy==1.15.2" "scikit-image" "numpy<2" && \
    pip3 install --break-system-packages \
      torch-scatter==2.1.2 && \
    pip3 install --break-system-packages --force-reinstall --no-deps \
      setuptools==79.0.1; \
  fi

# Re-pin empy to 3.x. The RayFronts block above transitively upgrades it via
# transformers/bloom/colcon-core to 4.x, which is API-incompatible with
# rosidl_generator_rs and breaks the workspace build with
# "TransientParseError: not enough data to read" on rmw.rs.em.
RUN pip3 install --break-system-packages --force-reinstall empy==3.3.4

# Patched OpenVDB (OasisArtisan fork) — exposes Int8Grid to Python bindings
RUN if [ "${ENABLE_RAYFRONTS}" = "true" ]; then \
    apt-get ${UPDATE_FLAGS} update && apt-get ${INSTALL_FLAGS} install -y --no-install-recommends \
      libboost-iostreams-dev libtbb-dev libblosc-dev python3-dev && \
    git clone --depth 1 https://github.com/OasisArtisan/openvdb /tmp/openvdb && \
    cmake -S /tmp/openvdb -B /tmp/openvdb/build \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DOPENVDB_BUILD_PYTHON_MODULE=ON \
      -DOPENVDB_PYTHON_WRAP_ALL_GRID_TYPES=ON \
      -DUSE_NUMPY=ON \
      -Dnanobind_DIR=$(python3 -c "import nanobind,os;print(os.path.join(os.path.dirname(nanobind.__file__),'cmake'))") && \
    cmake --build /tmp/openvdb/build -j4 && \
    cmake --install /tmp/openvdb/build && \
    rm -rf /tmp/openvdb && \
    rm -rf /var/lib/apt/lists/*; \
  fi

# Cleanup
RUN apt autoremove -y \
  && apt clean -y \
  && rm -rf /var/lib/apt/lists/*


# ============================================================
# Stage 2 — runtime: lean final image
# ============================================================
FROM ${BASE_IMAGE:-ubuntu:24.04} AS runtime

# Re-declare ARGs (Docker ARGs do not persist across FROM)
ARG BASE_IMAGE
ARG REAL_ROBOT
ARG UPDATE_FLAGS="-o Acquire::AllowInsecureRepositories=true -o Acquire::AllowDowngradeToInsecureRepositories=true"
ARG INSTALL_FLAGS="-o APT::Get::AllowUnauthenticated=true"
ARG SKIP_MACVO=false
ARG SKIP_TENSORRT=false

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
  ros-jazzy-desktop \
  python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*

# Carry over all ROS2 ENV vars from the builder stage
ENV ROS_DISTRO=jazzy
ENV AMENT_PREFIX_PATH=/opt/ros/jazzy
ENV COLCON_PREFIX_PATH=/opt/ros/jazzy
ENV LD_LIBRARY_PATH=/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/lib
ENV PATH=/opt/ros/jazzy/bin:$PATH
ENV PYTHONPATH=/opt/ros/jazzy/local/lib/python3.12/dist-packages:/opt/ros/jazzy/lib/python3.12/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ENV DEBIAN_FRONTEND=
# ========================

# Install runtime dev tools (no cmake or build-essential)
RUN apt update && apt install -y --no-install-recommends \
  vim nano tree \
  less htop jq \
  python3-pip \
  python3-rosdep \
  tmux \
  && rm -rf /var/lib/apt/lists/*

# Install runtime ROS2 packages (no libcgal-dev)
RUN apt update -y && apt install -y --no-install-recommends \
  ros-dev-tools \
  ros-jazzy-mavros \
  ros-jazzy-tf2* \
  ros-jazzy-stereo-image-proc \
  ros-jazzy-image-view \
  ros-jazzy-topic-tools \
  ros-jazzy-grid-map \
  ros-jazzy-domain-bridge \
  ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-xacro \
  ros-jazzy-foxglove-bridge \
  python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/*

# Install emoji font support and refresh font cache
RUN apt-get update && apt-get install -y --no-install-recommends \
  fonts-noto-color-emoji \
  fontconfig \
  && fc-cache -f -v \
  && rm -rf /var/lib/apt/lists/*

RUN /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

# Install DDS Router runtime library dependencies + OpenVDB
RUN apt update && apt install -y --no-install-recommends \
  libtinyxml2-dev libssl-dev libyaml-cpp-dev \
  libopenvdb-dev \
  && rm -rf /var/lib/apt/lists/*

# Install NVIDIA runtime apt packages (no -dev counterparts; NVIDIA/L4T images only, unless SKIP_TENSORRT=true)
# Note: TensorRT 8 packages may not be available for Ubuntu 24.04, so this is optional
RUN if echo "$BASE_IMAGE" | grep -qE "(nvidia|l4t)" && [ "${SKIP_TENSORRT}" != "true" ]; then \
  if [ ! -f /etc/apt/sources.list.d/cuda*.list ]; then \
  wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu$(lsb_release -rs | tr -d .)/x86_64/cuda-keyring_1.1-1_all.deb && \
  dpkg -i cuda-keyring_1.1-1_all.deb || true; \
  fi && \
  apt update -y && \
  apt install -y \
  libnvinfer10 libnvinfer-plugin10 \
  python3-libnvinfer \
  && rm -rf /var/lib/apt/lists/*; \
  fi

# Install Foxglove Studio desktop app
RUN wget -q https://get.foxglove.dev/desktop/latest/foxglove-studio-latest-linux-amd64.deb -O /tmp/foxglove-studio.deb \
  && apt-get ${UPDATE_FLAGS} update \
  && apt-get ${INSTALL_FLAGS} install -y --no-install-recommends /tmp/foxglove-studio.deb \
  && rm /tmp/foxglove-studio.deb \
  && rm -rf /var/lib/apt/lists/*

# Add ability to SSH (libglfw3-dev and libglm-dev kept per spec)
RUN apt-get ${UPDATE_FLAGS} update && apt-get ${INSTALL_FLAGS} install -y --no-install-recommends \
  openssh-server libglfw3-dev libglm-dev \
  && rm -rf /var/lib/apt/lists/*
RUN mkdir /var/run/sshd

# Copy build artifacts from the builder stage
# /opt/ros/jazzy is NOT copied — runtime installs the same packages via apt (including foxglove-bridge)
# /usr/local/lib/python3.12 is NOT copied separately — it is covered by /usr/local/lib below
# /usr/local/include is copied to provide OpenVDB (and DDS Router) headers for in-container colcon builds
COPY --from=builder /usr/local/bin            /usr/local/bin
COPY --from=builder /usr/local/lib            /usr/local/lib
COPY --from=builder /usr/local/include        /usr/local/include
COPY --from=builder /model_weights            /model_weights
COPY --from=builder /root/.tmux               /root/.tmux

# Password is airstack
RUN echo 'root:airstack' | chpasswd \
  && sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config \
  && sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config \
  && sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

EXPOSE 22

WORKDIR /root/AirStack/robot/ros_ws


# runtime + compiled RayFronts (only reached when FINAL_STAGE=runtime-rayfronts)
FROM runtime AS runtime-rayfronts
COPY ./common/rayfronts          /opt/rayfronts
COPY ./common/rayfronts_configs/ /opt/rayfronts/rayfronts/configs/
RUN apt-get update && apt-get install -y --no-install-recommends \
      cmake build-essential python3-dev \
 && cd /opt/rayfronts && CMAKE_INSTALL_PREFIX=/usr/local ./compile.sh \
 && rm -rf /var/lib/apt/lists/*

FROM ${FINAL_STAGE} AS final
