FROM osrf/ros:jazzy-desktop-full

WORKDIR /root

# System deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip tmux vim wget curl unzip \
    ros-jazzy-mavros ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs \
    libvulkan1 \
    && rm -rf /var/lib/apt/lists/*

# PX4 SITL
ARG PX4_VERSION="v1.16.1"
RUN git clone --branch ${PX4_VERSION} --recursive https://github.com/PX4/PX4-Autopilot.git
RUN cd PX4-Autopilot && Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
RUN cd PX4-Autopilot && make px4_sitl_default

# Microsoft AirSim (legacy) Python client with msgpack fix
# See [AirSim #3333](https://github.com/microsoft/AirSim/issues/3333#issuecomment-827894198) for details.
RUN pip3 install --break-system-packages jinja2 msgpack backports.ssl-match-hostname && \
    pip3 install --break-system-packages \
      git+https://github.com/tbelhalfaoui/msgpack-rpc-python.git@fix-msgpack-dep && \
    pip3 install --break-system-packages airsim --ignore-installed numpy

# Patch AirSim client to remove encoding params that break with C msgpack
RUN sed -i "s/pack_encoding='utf-8', //" \
      /usr/local/lib/python3.*/dist-packages/airsim/client.py 2>/dev/null; \
    sed -i "s/unpack_encoding='utf-8'//" \
      /usr/local/lib/python3.*/dist-packages/airsim/client.py 2>/dev/null; \
    true

# Create non-root user (UE4 refuses to run as root)
RUN useradd -m -s /bin/bash -G sudo ms-airsim && \
    echo 'ms-airsim ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers && \
    mkdir -p /home/ms-airsim/Documents/AirSim && \
    echo "source /opt/ros/jazzy/setup.bash" >> /home/ms-airsim/.bashrc

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

COPY entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

WORKDIR /root
