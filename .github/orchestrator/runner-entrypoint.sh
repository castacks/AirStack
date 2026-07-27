#!/usr/bin/env bash
# Entry point for the AirStack CI ephemeral-runner container (an OSMO task).
#
# Starts an inner Docker daemon — the AirStack test harness runs `airstack up`
# (docker compose) inside this container — waits for it, then runs exactly ONE
# ephemeral GitHub Actions job via the single-use JIT config. When run.sh exits,
# the OSMO task completes and the pod is destroyed: one job per pod, same as the
# old OpenStack VM.
#
# Requires a privileged pod (dockerd) scheduled on a GPU platform; the NVIDIA
# container toolkit (baked into the image) lets the inner dockerd pass the node
# GPU through to the compose containers.
set -euxo pipefail

: "${ENCODED_JIT_CONFIG:?ENCODED_JIT_CONFIG must be set by the workflow}"

# Start dockerd in the background (needs privileged).
dockerd >/var/log/dockerd.log 2>&1 &

# Wait for the daemon to accept connections (~60s budget).
for _ in $(seq 1 60); do
  if docker info >/dev/null 2>&1; then
    break
  fi
  sleep 1
done
if ! docker info >/dev/null 2>&1; then
  echo "ERROR: dockerd did not become ready" >&2
  cat /var/log/dockerd.log >&2 || true
  exit 1
fi

# Non-fatal GPU sanity check — surfaces GPU/privileged/toolkit misconfig early.
nvidia-smi || echo "WARN: nvidia-smi unavailable (check GPU + privileged + toolkit)"

cd /home/runner/actions-runner
# The JIT config makes this runner single-use + ephemeral; run.sh returns after
# one job, which completes the task and lets OSMO reap the pod.
exec ./run.sh --jitconfig "${ENCODED_JIT_CONFIG}"
