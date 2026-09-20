#!/bin/bash
# Localhost task console; dependencies reused from this session's robot container.
set -euo pipefail
[[ $# -le 3 ]] || {
  cat >&2 <<EOF
Usage: bash scripts/rrm_command_console.sh [verified-bundle-directory] [port]

With no bundle, the console starts in live-only mode from the checked-in Office
context and scene manifest. It can capture/save live requests and use the private
Cosmos worker, but it has no historical proposal or flight-dispatch panel.

Pass a verified historical bundle only when reference review is wanted:
  bash scripts/rrm_office_fetch_import.sh <psc-job-id>
EOF
  exit 2
}
rrm_source=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
rrm_console_args=()
if [[ $# -ge 1 && -n "${1:-}" ]]; then
  rrm_bundle=$(cd "$1" && pwd)
  rrm_console_args+=(--bundle "$rrm_bundle")
fi
rrm_port="${2:-8787}"
if [[ $# -eq 0 ]]; then
  rrm_console_args+=(--context-template "$rrm_source/examples/office_visual_eval/navigation_context.json"
                     --scene-manifest "$rrm_source/examples/office_visual_eval/scene_manifest.json")
fi
rrm_deps=$(mktemp -d /tmp/rrm-console-deps.XXXXXX)
rrm_container_deps=/tmp/rrm-canonical-deps
# The robot container may have been recreated while switching Isaac profiles.
# Rebuild this session-local dependency cache without changing the image/ROS workspace.
if ! docker exec airstack-robot-desktop-1 env PYTHONPATH="$rrm_container_deps" \
  python3 -c 'import pydantic; assert int(pydantic.__version__.split(".")[0]) == 2' \
  >/dev/null 2>&1; then
  echo "Preparing isolated RRM console dependencies..." >&2
  docker exec airstack-robot-desktop-1 bash -c \
    'python3 -m pip install --disable-pip-version-check --target "$1" "pydantic>=2.0"' \
    _ "$rrm_container_deps"
fi
docker cp airstack-robot-desktop-1:/tmp/rrm-canonical-deps/. "$rrm_deps/"
rrm_psc_args=()
if [[ "${RRM_PSC_BRIDGE:-0}" == "1" ]]; then
  # The bridge is non-interactive and uses the operator's already configured PSC
  # key/agent. It never accepts a password or token through the browser.
  rrm_psc_args=(--psc-bridge "$rrm_source/scripts/rrm_psc_bridge.sh")
fi
PYTHONPATH="$rrm_deps:$rrm_source:$rrm_source/scripts" exec python3 \
  "$rrm_source/scripts/rrm_command_console.py" "${rrm_console_args[@]}" \
  --output-dir "$rrm_source/../../.rrm-artifacts/command-requests" \
  --camera-script "$rrm_source/scripts/airstack_capture_image.py" --port "$rrm_port" "${rrm_psc_args[@]}"
