# airstack-osmo-workspace
#
# Image used by the OSMO airstack-dev workflow. Boots into a Docker-in-Docker
# (DinD) pod with sshd on :22 so VS Code / Cursor Remote-SSH can attach. The
# inner dockerd then runs the regular AirStack docker-compose stack (Isaac
# Sim, robot-desktop, GCS) on the GPU forwarded into the pod.
#
# Built and pushed by the lab admin (see osmo/README.md):
#   docker build -t airlab-docker.andrew.cmu.edu/airstack/airstack-osmo-workspace:latest .
#   docker push   airlab-docker.andrew.cmu.edu/airstack/airstack-osmo-workspace:latest
#
# Students never build this image.

FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Base utilities + sshd + dev ergonomics. python3 is here so the airstack.sh
# CLI's helper scripts can shell out to python3 without extra installs.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    git \
    git-lfs \
    gnupg \
    iproute2 \
    iptables \
    jq \
    less \
    locales \
    lsb-release \
    openssh-server \
    procps \
    python3 \
    python3-pip \
    rsync \
    sudo \
    tmux \
    tzdata \
    vim-tiny \
    && locale-gen C.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Docker CE + Compose plugin (required for `airstack up` to work inside the
# pod). The same install procedure as get.docker.com but pinned to apt repos
# so we can upgrade explicitly.
RUN install -m 0755 -d /etc/apt/keyrings \
    && curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
       | gpg --dearmor -o /etc/apt/keyrings/docker.gpg \
    && chmod a+r /etc/apt/keyrings/docker.gpg \
    && echo \
       "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
        https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" \
       > /etc/apt/sources.list.d/docker.list \
    && apt-get update && apt-get install -y --no-install-recommends \
       docker-ce \
       docker-ce-cli \
       containerd.io \
       docker-buildx-plugin \
       docker-compose-plugin \
       fuse-overlayfs \
    && rm -rf /var/lib/apt/lists/*

# Note: dockerd inside an OSMO/k8s pod is running on top of an overlayfs
# rootfs (the pod's own /). Docker refuses overlay2-on-overlayfs, so without
# fuse-overlayfs the entrypoint's storage-driver fallback chain lands on
# vfs, which has no copy-on-write and bloats the AirStack image set ~3x
# (airstack-dev-10, 2026-05-14 burned 270Gi for 2 of 3 inner images).
# fuse-overlayfs gives proper CoW for DinD over overlayfs.

# NVIDIA Container Toolkit, configured to register the `nvidia` runtime with
# dockerd. This is what lets the inner Isaac Sim container see the GPU
# forwarded into the workspace pod. The pool's nodes still need the host-side
# NVIDIA driver + nvidia-container-toolkit; this just configures the inner
# dockerd.
RUN curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
      | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
    && curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
      | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
      > /etc/apt/sources.list.d/nvidia-container-toolkit.list \
    && apt-get update && apt-get install -y --no-install-recommends \
       nvidia-container-toolkit \
    && rm -rf /var/lib/apt/lists/*

# sshd: pubkey-only, no password auth ever. Host keys generated at runtime in
# entrypoint.sh so each pod has unique keys (good practice; harmless cost).
COPY sshd_config /etc/ssh/sshd_config
RUN chmod 644 /etc/ssh/sshd_config && mkdir -p /var/run/sshd

# entrypoint: starts sshd, dockerd, clones AirStack, materializes secrets,
# runs `airstack up`, then sleeps so port-forwards keep working.
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod 0755 /usr/local/bin/entrypoint.sh

# Symlink airstack.sh from the cloned repo into PATH on first run; for now
# expose a placeholder so command lookups don't fail before the clone.
WORKDIR /root

# Default to a long-running entrypoint. The OSMO workflow overrides command
# and args to invoke /tmp/entry.sh which sources this image's entrypoint.sh
# logic. Either path works.
CMD ["/usr/local/bin/entrypoint.sh"]
