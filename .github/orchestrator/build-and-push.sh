#!/usr/bin/env bash
# Build & push the AirStack CI ephemeral-runner image to AirLab Harbor.
# Run on a linux/amd64 machine (or buildx --platform linux/amd64) with:
#   docker login airlab-docker.andrew.cmu.edu
#
# Usage:
#   ./build-and-push.sh
#   RUNNER_VERSION=2.334.0 ./build-and-push.sh

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REGISTRY="${REGISTRY:-airlab-docker.andrew.cmu.edu/airstack}"
RUNNER_VERSION="${RUNNER_VERSION:-2.334.0}"
IMAGE="${REGISTRY}/airstack-ci-runner:${RUNNER_VERSION}"

echo "==> Building ${IMAGE}"
docker build \
  -f "${ROOT}/runner.Dockerfile" \
  --build-arg "RUNNER_VERSION=${RUNNER_VERSION}" \
  -t "${IMAGE}" \
  "${ROOT}"

echo "==> Pushing ${IMAGE}"
docker push "${IMAGE}"

echo "==> Done. Set in /etc/airstack-orchestrator/config.yaml:"
echo "    runner_image: \"${IMAGE}\""
