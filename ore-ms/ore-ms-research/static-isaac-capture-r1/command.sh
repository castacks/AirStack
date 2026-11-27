#!/bin/bash
set -euo pipefail
p="$(cd "$(dirname "$0")" && pwd)"
docker run --rm --name ore-isolated-geometry-r1 --pull never --network none \
  --gpus '"device=0"' --shm-size 1g \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=N -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
  --mount "type=bind,src=$p/input,dst=/capture-input,readonly" \
  --mount "type=bind,src=$p/output,dst=/capture-output" \
  --entrypoint /bin/bash \
  airlab-docker.andrew.cmu.edu/airstack/airstack@sha256:ba2402ec4f389fad6552d79e87871131c587d090963d1d894539016904f3e697 \
  /capture-input/run.sh
