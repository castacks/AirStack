# either ubuntu:24.04 / nvidia CUDA, or Jetson intermediary from Dockerfile.l4t-stack-base (see robot-l4t in compose).
ARG BASE_IMAGE
ARG SKIP_RAYFRONTS=true
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
ARG TARGET_ARCH=x86_64

ARG PIP_VERSION=24.0
ARG PYTHON_VERSION=3.12

ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=${ROS_DISTRO}
ARG SKIP_RAYFRONTS

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
  ros-${ROS_DISTRO}-desktop \
  python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*

ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib/${TARGET_ARCH}-linux-gnu:/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/local/lib/python${PYTHON_VERSION}/dist-packages:/opt/ros/${ROS_DISTRO}/lib/python${PYTHON_VERSION}/site-packages
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
  xvfb \
  && rm -rf /var/lib/apt/lists/*

# Freeze pip and setuptools versions (ignore-installed: apt-shipped wheel/setuptools have no pip RECORD).
RUN python3 -m pip install --no-cache-dir --break-system-packages --ignore-installed --upgrade \
  "pip==${PIP_VERSION}" \
  "setuptools==79.0.1" \
  wheel

# Install any additional ROS2 packages
RUN apt update -y && apt install -y --no-install-recommends \
  ros-dev-tools \
  ros-${ROS_DISTRO}-mavros \
  ros-${ROS_DISTRO}-mavros-extras \
  ros-${ROS_DISTRO}-tf2* \
  ros-${ROS_DISTRO}-stereo-image-proc \
  ros-${ROS_DISTRO}-image-view \
  ros-${ROS_DISTRO}-topic-tools \
  ros-${ROS_DISTRO}-grid-map \
  ros-${ROS_DISTRO}-domain-bridge \
  ros-${ROS_DISTRO}-rosbag2-storage-mcap \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-ament-package \
  python3-colcon-common-extensions \
  # asm_mighty (MIGHTY local planner + acl-mapping world model): PCL for the
  # voxel map, nlohmann for the solver config loader — module.yaml deps
  ros-jazzy-pcl-ros \
  ros-jazzy-pcl-conversions \
  ros-jazzy-tf2-sensor-msgs \
  ros-jazzy-tf2-eigen \
  libpcl-dev \
  nlohmann-json3-dev \
  && rm -rf /var/lib/apt/lists/*

RUN /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

# Install Python dependencies (unconditional)
# Note: numpy>=1.26 required for Python 3.12 compatibility
# Using --ignore-installed to avoid conflicts with system packages
RUN pip3 install --break-system-packages --ignore-installed \
  "pytest==7.4.*" \
  empy==3.3.4 \
  future \
  # matplotlib stays: dev-script consumer + published-image behavior
  # (its resolver also pulls pillow transitively)
  matplotlib==3.8.4 \
  # kept <2.0 conservatively; audit consumers before relaxing
  numpy~=1.26.4 \
  wheel \
  pyyaml \
  requests \
  # setup tools must be <80 for Jazzy https://github.com/ros2/ros2/issues/1702#issuecomment-3007929996
  setuptools==79.0.1 \
  # scipy stays: position_setpoint_pub
  scipy \
  scikit-learn

# Keep pytest < 8.1. ROS Jazzy launch_testing still implements
# pytest_pycollect_makemodule(path=...), which pluggy rejects after pytest 8.1
# removed the py.path hook argument (PluginValidationError on colcon test).
RUN python3 -m pip install --no-cache-dir --break-system-packages \
  "pytest>=7.4,<8.1"

# torch for RayFronts: cu130-pinned wheels match its torch-scatter / OpenVDB ABI
RUN if [ "${SKIP_RAYFRONTS}" != "true" ]; then \
    pip3 install --break-system-packages \
      --index-url https://download.pytorch.org/whl/cu130 \
      torch==2.9.1 torchvision torchaudio; \
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

# RayFronts deps (builder-stage, gated on SKIP_RAYFRONTS!=true)
RUN if [ "${SKIP_RAYFRONTS}" != "true" ]; then \
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

# Patched OpenVDB (OasisArtisan fork) — exposes Int8Grid to Python bindings
RUN if [ "${SKIP_RAYFRONTS}" != "true" ]; then \
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

# rayfronts/__init__ imports its rerun visualizer unconditionally and the RADIO
# encoder imports einops at model load; without them the mapper cannot start. Separate layer so a
# change here never invalidates the OpenVDB build above.
RUN if [ "${SKIP_RAYFRONTS}" != "true" ]; then \
    pip3 install --break-system-packages rerun-sdk==0.22.0 einops; \
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
ARG TARGET_ARCH=x86_64

ARG PIP_VERSION=24.0
ARG PYTHON_VERSION=3.12

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

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
  ros-${ROS_DISTRO}-desktop \
  python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*

ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib/${TARGET_ARCH}-linux-gnu:/opt/ros/${ROS_DISTRO}/lib
ENV PATH=/opt/ros/${ROS_DISTRO}/bin:$PATH
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/local/lib/python${PYTHON_VERSION}/dist-packages:/opt/ros/${ROS_DISTRO}/lib/python${PYTHON_VERSION}/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ENV DEBIAN_FRONTEND=
# ========================

# Install runtime dev tools. (The compile toolchain — cmake, build-essential —
# still arrives below via ros-dev-tools: the runtime image keeps it because
# `bws` builds the ROS workspace inside this container.)
RUN apt update && apt install -y --no-install-recommends \
  vim nano tree \
  less htop jq \
  python3-pip \
  python3-yaml \
  python3-empy \
  python3-rosdep \
  tmux \
  xvfb \
  && rm -rf /var/lib/apt/lists/*

# Freeze pip and setuptools versions (ignore-installed: apt-shipped wheel/setuptools have no pip RECORD).
RUN python3 -m pip install --no-cache-dir --break-system-packages --ignore-installed --upgrade \
  "pip==${PIP_VERSION}" \
  "setuptools==79.0.1" \
  wheel

# Install runtime ROS2 packages. ros-dev-tools pulls in the compile toolchain
# (cmake, build-essential) — deliberate, since `bws` builds in-container.
# (ros-*-grid-map pulls CGAL in as a transitive dependency where it needs it.)
# foxglove-bridge was dropped from the robot image (the GCS image installs its
# own); per-robot Foxglove returns later as an opt-in stack include.
RUN apt update -y && apt install -y --no-install-recommends \
  ros-dev-tools \
  ros-${ROS_DISTRO}-mavros \
  ros-${ROS_DISTRO}-mavros-extras \
  ros-${ROS_DISTRO}-tf2* \
  ros-${ROS_DISTRO}-stereo-image-proc \
  ros-${ROS_DISTRO}-image-view \
  ros-${ROS_DISTRO}-topic-tools \
  ros-${ROS_DISTRO}-grid-map \
  ros-${ROS_DISTRO}-domain-bridge \
  ros-${ROS_DISTRO}-rosbag2-storage-mcap \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-ament-package \
  python3-colcon-common-extensions \
  # asm_mighty (MIGHTY local planner + acl-mapping world model): PCL for the
  # voxel map, nlohmann for the solver config loader — module.yaml deps
  ros-jazzy-pcl-ros \
  ros-jazzy-pcl-conversions \
  ros-jazzy-tf2-sensor-msgs \
  ros-jazzy-tf2-eigen \
  libpcl-dev \
  nlohmann-json3-dev \
  && rm -rf /var/lib/apt/lists/*

# Install emoji font support and refresh font cache
RUN apt-get update && apt-get install -y --no-install-recommends \
  fonts-noto-color-emoji \
  fontconfig \
  && fc-cache -f -v \
  && rm -rf /var/lib/apt/lists/*

# MAVROS requires geographiclib-tools to be installed for any offboard control to work.
RUN apt-get update \
  && apt-get install -y --no-install-recommends geographiclib-tools \
  && /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh \
  && test -f /usr/share/GeographicLib/geoids/egm96-5.pgm \
  && rm -rf /var/lib/apt/lists/*

# Install DDS Router runtime library dependencies + OpenVDB
RUN apt update && apt install -y --no-install-recommends \
  libtinyxml2-dev libssl-dev libyaml-cpp-dev \
  libopenvdb-dev \
  && rm -rf /var/lib/apt/lists/*

# Install Foxglove Studio desktop app only for non-real-robot images
RUN if [ "${REAL_ROBOT}" != "true" ] && [ "$(dpkg --print-architecture)" = "amd64" ]; then \
      wget -q https://get.foxglove.dev/desktop/latest/foxglove-studio-latest-linux-amd64.deb -O /tmp/foxglove-studio.deb && \
      apt-get ${UPDATE_FLAGS} update && \
      apt-get ${INSTALL_FLAGS} install -y --no-install-recommends /tmp/foxglove-studio.deb && \
      rm /tmp/foxglove-studio.deb; \
    else \
      echo "Skipping Foxglove Studio install (REAL_ROBOT=${REAL_ROBOT}, arch=$(dpkg --print-architecture))"; \
    fi && \
    rm -rf /var/lib/apt/lists/*

# Add ability to SSH
RUN apt-get ${UPDATE_FLAGS} update && apt-get ${INSTALL_FLAGS} install -y --no-install-recommends \
  openssh-server \
  && rm -rf /var/lib/apt/lists/*
RUN mkdir /var/run/sshd

# Default local planner dep: the asm_mighty module (MIGHTY, pinned by the
# full_default / lite_default / lite_offload_global stacks) reads its solver
# config with the header-only nlohmann json library. It is installed in the
# base image so the default stack builds with no module dependency layer (the
# module declares no deps of its own). Other modules' deps enter the image via
# their own tier-1/2 layers (airstack module lock --build) — e.g. asm_droan's
# assimp/EGL/GLFW/GLM link deps for droan_gl.
RUN apt-get ${UPDATE_FLAGS} update && apt-get ${INSTALL_FLAGS} install -y --no-install-recommends \
  nlohmann-json3-dev \
  && rm -rf /var/lib/apt/lists/*

# Copy build artifacts from the builder stage
# /opt/ros/jazzy is NOT copied — runtime installs the same packages via apt
# /usr/local/lib/python3.12 is NOT copied separately — it is covered by /usr/local/lib below
# /usr/local/include is copied to provide OpenVDB (and DDS Router) headers for in-container colcon builds
COPY --from=builder /usr/local/bin            /usr/local/bin
COPY --from=builder /usr/local/lib            /usr/local/lib
COPY --from=builder /usr/local/include        /usr/local/include
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
      cmake build-essential python3-dev python3-venv \
 && cd /opt/rayfronts && CMAKE_INSTALL_PREFIX=/usr/local ./compile.sh \
 && rm -rf /var/lib/apt/lists/*

# FPV+LVLM baseline runs in an isolated venv. --system-site-packages reuses the
# system torch / numpy(1.26) / rclpy / cv_bridge, so only the LVLM-specific deps
# (transformers/bitsandbytes/accelerate) live in the venv and can't perturb
# rayfronts' system transformers. numpy stays system 1.26 (cv_bridge needs <2);
# the venv's transformers shadows the system's only for the venv's python.
# semantic_search_task launches lvlm_baseline_node with /opt/lvlm-venv/bin/python.
RUN python3 -m venv --system-site-packages /opt/lvlm-venv \
 && /opt/lvlm-venv/bin/pip install --no-cache-dir \
      "transformers==4.57.6" bitsandbytes accelerate "numpy<2"

# Co-NavGPT2 baseline. SYSTEM python, not a venv: the node is an ordinary ROS
# package that must import rclpy and cv_bridge, and every pin below was measured
# against this image rather than guessed. Three separate installs, because each
# flag is load-bearing:
#
# 1. open3d --no-deps. Its declared deps drag in pandas + scikit-learn, and
#    modern pandas requires numpy>=2 -- which silently upgraded numpy to 2.5.2
#    and broke both scipy and cv_bridge when tried. Only core geometry is used
#    (PointCloud, voxel_down_sample, transform, KDTreeFlann, cluster_dbscan),
#    so its dep tree is dead weight anyway.
# 2. The import-time deps open3d actually needs, pinned. `import open3d` reaches
#    visualization/draw_plotly and dies without `dash`; dash needs no numpy, so
#    it cannot drag numpy 2 in. --ignore-installed blinker is required because
#    Debian's python3-blinker ships no RECORD and pip refuses to replace it.
#    "setuptools<80" is NOT optional: dash's tree pulls setuptools 84, which
#    violates colcon-core's <80 and breaks `colcon build`.
# 3. ultralytics/supervision --no-deps: both declare opencv-python, and a pip
#    cv2 shadows the ROS cv2 at /usr/lib/python3/dist-packages that cv_bridge is
#    built against. Their other deps are already in system site-packages; the
#    ones that were not -- py-cpuinfo and defusedxml (supervision imports it for
#    pascal_voc) -- are installed by name above. ultralytics-thop is in THIS
#    group rather than that one because it declares an unpinned torch: resolved
#    normally it upgrades torch 2.9.1 -> 2.13.0 and orphans torchvision
#    0.24.1+cu130, whose compiled ops then fail to register, so `import
#    torchvision` dies with "operator torchvision::nms does not exist" and takes
#    YOLO-World's NMS with it. It needs only the torch already in the image.
# 4. ultralytics' CLIP fork, --no-deps for the same opencv reason. YOLO-World is
#    open-vocabulary, so set_classes() encodes the prompt vocabulary with CLIP —
#    and upstream Co-NavGPT2 calls set_classes in the DETECTOR'S CONSTRUCTOR, so
#    without this the search_planner node dies at startup with
#    "ModuleNotFoundError: No module named 'clip'". ultralytics ships a lazy
#    check_requirements("git+.../CLIP.git") fallback for exactly this case, but
#    it shells out to pip WITHOUT --break-system-packages, so under PEP 668 it
#    fails and the retry `import clip` raises anyway.
#
# The trailing check fails the BUILD rather than letting a shadowed cv2, an
# upgraded numpy, or a colcon-breaking setuptools reach a run.
RUN pip install --no-cache-dir --break-system-packages --no-deps "open3d>=0.19" \
 && pip install --no-cache-dir --break-system-packages --ignore-installed blinker \
      "numpy<2" "setuptools<80" dash configargparse addict nbformat pyquaternion \
      scikit-fmm openai py-cpuinfo defusedxml \
 && pip install --no-cache-dir --break-system-packages --no-deps \
      ultralytics ultralytics-thop "supervision==0.19.0" \
 && pip install --no-cache-dir --break-system-packages --no-deps \
      "clip @ git+https://github.com/ultralytics/CLIP.git" \
 && python3 -c "import numpy, cv2, clip, colcon_core, cv_bridge, open3d, skfmm, ultralytics, supervision, openai; \
assert numpy.__version__.startswith('1.'), numpy.__version__; \
assert cv2.__file__.startswith('/usr/lib/python3'), cv2.__file__; \
import torch, torchvision; \
torchvision.ops.nms(torch.tensor([[0.,0.,1.,1.]]), torch.tensor([0.9]), 0.5); \
print('conavgpt2 deps ok: numpy', numpy.__version__, 'cv2', cv2.__version__, 'open3d', open3d.__version__, 'torch', torch.__version__, 'tv', torchvision.__version__, 'clip models', len(clip.available_models()))"

# The VLM Co-NavGPT2 queries is served over an OpenAI-compatible endpoint. The
# server reuses lvlm-venv (transformers/bitsandbytes/accelerate already there);
# these are only the HTTP layer. NOT qwen-vl-utils: nothing imports it — the
# server does its own PIL decode and hands min_pixels/max_pixels straight to
# AutoProcessor.
RUN /opt/lvlm-venv/bin/pip install --no-cache-dir \
      fastapi uvicorn

FROM ${FINAL_STAGE} AS final

