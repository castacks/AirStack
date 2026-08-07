# Prebaked image for AirStack CI ephemeral runners on OSMO.
#
# This bakes in what the old cloud-init.yaml.j2 installed on the OpenStack VM
# (Docker CE + compose plugin, NVIDIA container toolkit, the GitHub Actions
# runner) so pod start is fast and the single-use JIT token can't expire during
# a slow apt/bootstrap. Build it and push to a registry your OSMO pool can pull:
#
#   docker build -f runner.Dockerfile \
#       --build-arg RUNNER_VERSION=2.336.0 \
#       -t <registry>/airstack-ci-runner:2.336.0 .
#   docker push <registry>/airstack-ci-runner:2.336.0
#
# Then set `runner_image: <registry>/airstack-ci-runner:2.336.0` in config.yaml.
# Keep RUNNER_VERSION in sync with the actions/runner release you want.
#
# GPU-in-Docker-in-Docker: the OSMO task must run privileged (see
# `privileged: true` in runner-workflow.yaml.j2) on a platform with
# "Privileged Mode Allowed" + GPUs. The inner dockerd uses the NVIDIA container
# toolkit installed here to expose the node GPU to the `airstack up` containers.
# The image is linux/amd64 (x86_64 runner tarball); rebuild for arm64 if needed.
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# Docker CE (+ compose/buildx plugins), NVIDIA container toolkit, and the tools
# the AirStack test harness / GH runner need (git, jq, python venv, ...).
# e2fsprogs + fuse-overlayfs back the storage-backend selection in
# runner-entrypoint.sh: the pod rootfs is overlayfs, which the kernel rejects as
# an overlay upperdir, so dockerd's data-root has to live on a loopback ext4
# image (mkfs.ext4) or, failing that, use the fuse-overlayfs driver.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg jq git sudo iproute2 \
      e2fsprogs fuse-overlayfs kmod mount \
      python3 python3-venv python3-pip \
  && install -m 0755 -d /etc/apt/keyrings \
  && curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
      -o /etc/apt/keyrings/docker.asc \
  && chmod a+r /etc/apt/keyrings/docker.asc \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \
https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
      > /etc/apt/sources.list.d/docker.list \
  && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
      | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
      | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
      > /etc/apt/sources.list.d/nvidia-container-toolkit.list \
  && apt-get update && apt-get install -y --no-install-recommends \
      docker-ce docker-ce-cli containerd.io \
      docker-buildx-plugin docker-compose-plugin \
      nvidia-container-toolkit \
  && nvidia-ctk runtime configure --runtime=docker \
  && rm -rf /var/lib/apt/lists/*

# GitHub Actions runner (self-contained; version pinned at build time).
ARG RUNNER_VERSION=2.336.0
RUN mkdir -p /home/runner/actions-runner \
  && cd /home/runner/actions-runner \
  && curl -fsSL -o runner.tar.gz \
      "https://github.com/actions/runner/releases/download/v${RUNNER_VERSION}/actions-runner-linux-x64-${RUNNER_VERSION}.tar.gz" \
  && tar xzf runner.tar.gz \
  && rm runner.tar.gz \
  && ./bin/installdependencies.sh

COPY runner-entrypoint.sh /usr/local/bin/run-ephemeral-runner.sh
RUN chmod +x /usr/local/bin/run-ephemeral-runner.sh

WORKDIR /home/runner/actions-runner
