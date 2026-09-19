#!/usr/bin/env bash
set -euo pipefail
[[ $# -eq 2 ]] || { echo "usage: $0 <immutable-request-dir> <psc-job-id>" >&2; exit 2; }
request_dir=$(cd "$1" && pwd); run_id=$(basename "$request_dir"); job_id="$2"
[[ "$run_id" =~ ^[0-9a-f]{32}$ && "$job_id" =~ ^[0-9]+$ ]] || exit 2
: "${RRM_PSC_ROOT:=/ocean/projects/eng260004p/oabolade/physical-ai}"
: "${RRM_PSC_HOST:=bridges2.psc.edu}"
: "${RRM_PSC_USER:?Set RRM_PSC_USER}"
bundle="$request_dir/psc-bridge-bundle-$job_id"
if [[ -e "$bundle" ]]; then
  [[ -f "$bundle/result.json" ]] || { echo "existing bundle is incomplete: $bundle" >&2; exit 1; }
  echo "Reusing already downloaded bundle: $bundle"
else
  stage=$(mktemp -d "$request_dir/.psc-fetch-$job_id.XXXXXX")
  trap 'rm -rf "$stage"' EXIT
  # PSC's login node permits shell commands but does not provide rsync. Stream the
  # completed bundle once over SSH, then atomically retain it only after validation.
  ssh -o StrictHostKeyChecking=accept-new "$RRM_PSC_USER@$RRM_PSC_HOST" \
    "tar -C '$RRM_PSC_ROOT/runs/rrm/office/$job_id' -czf - ." | tar -xzf - -C "$stage"
  [[ -f "$stage/result.json" ]] || { echo "PSC result bundle is not ready" >&2; exit 1; }
  mv "$stage" "$bundle"
  trap - EXIT
fi
payload=$(printf '{"run_id":"%s","job_id":"%s","bundle_dir":"%s"}\n' "$run_id" "$job_id" "$bundle")
status=$(curl -fsS -X POST -H 'Content-Type: application/json' -d "$payload" http://127.0.0.1:8787/api/requests/manual-import)
echo "PSC job $job_id recorded in GUI history: $status"
