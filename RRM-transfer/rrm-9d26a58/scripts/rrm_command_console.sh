#!/bin/bash
# Localhost task console; dependencies reused from this session's robot container.
set -euo pipefail
[[ $# -ge 1 && $# -le 2 ]] || {
  echo 'Usage: bash scripts/rrm_command_console.sh <verified-bundle-directory> [port]' >&2; exit 2;
}
rrm_source=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
rrm_bundle=$(cd "$1" && pwd)
rrm_port="${2:-8787}"
rrm_deps=$(mktemp -d /tmp/rrm-console-deps.XXXXXX)
docker cp airstack-robot-desktop-1:/tmp/rrm-canonical-deps/. "$rrm_deps/"
rrm_capture=$(docker exec airstack-robot-desktop-1 mktemp -d /tmp/rrm-console-capture.XXXXXX)
docker cp "$rrm_source/scripts/airstack_capture_image.py" \
  "airstack-robot-desktop-1:$rrm_capture/capture.py"
PYTHONPATH="$rrm_deps:$rrm_source:$rrm_source/scripts" exec python3 \
  "$rrm_source/scripts/rrm_command_console.py" --bundle "$rrm_bundle" \
  --output-dir "$rrm_source/../../.rrm-artifacts/command-requests" \
  --camera-script "$rrm_capture/capture.py" --port "$rrm_port"
