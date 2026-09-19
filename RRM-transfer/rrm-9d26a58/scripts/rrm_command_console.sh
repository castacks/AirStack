#!/bin/bash
# Localhost task console; dependencies reused from this session's robot container.
set -euo pipefail
rrm_historical_psc_job_id="${RRM_HISTORICAL_PSC_JOB_ID:-}"
[[ $# -ge 1 && $# -le 2 ]] || {
  cat >&2 <<EOF
Usage: bash scripts/rrm_command_console.sh <verified-bundle-directory> [port]

No verified bundle is bundled with a recreated OSMO workspace. A PSC Slurm job ID
is not a VS Code folder. Fetch the job you intend to use in an MFA-capable terminal,
then pass the printed "Verified bundle:" path:
  bash scripts/rrm_office_fetch_import.sh <psc-job-id>
EOF
  if [[ -n "$rrm_historical_psc_job_id" ]]; then
    echo "Configured historical reference: $rrm_historical_psc_job_id" >&2
  else
    echo "Set RRM_HISTORICAL_PSC_JOB_ID to display your current historical reference here." >&2
  fi
  exit 2
}
rrm_source=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
rrm_bundle=$(cd "$1" && pwd)
rrm_port="${2:-8787}"
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
  "$rrm_source/scripts/rrm_command_console.py" --bundle "$rrm_bundle" \
  --output-dir "$rrm_source/../../.rrm-artifacts/command-requests" \
  --camera-script "$rrm_source/scripts/airstack_capture_image.py" --port "$rrm_port" "${rrm_psc_args[@]}"
