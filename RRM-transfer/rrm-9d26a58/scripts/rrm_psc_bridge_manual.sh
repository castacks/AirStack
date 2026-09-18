#!/usr/bin/env bash
# Non-interactive OSMO -> PSC bridge for exactly one immutable RRM request.
# It is invoked by rrm_command_console.py in a background worker, never by browser JS.
# PSC credentials must already be available to ssh-agent/key management; passwords and
# tokens are deliberately unsupported here.
set -euo pipefail

[[ $# -eq 1 ]] || { echo "usage: $0 <immutable-request-dir>" >&2; exit 2; }
request_dir=$(cd "$1" && pwd)
run_id=$(basename "$request_dir")
[[ "$run_id" =~ ^[0-9a-f]{32}$ ]] || { echo "invalid request ID" >&2; exit 2; }
for required in input.json input.png request.json observation.json; do
  [[ -f "$request_dir/$required" ]] || { echo "missing request evidence: $required" >&2; exit 2; }
done

: "${RRM_PSC_PROJECT:=eng260004p}"
: "${RRM_PSC_ROOT:=/ocean/projects/eng260004p/oabolade/physical-ai}"
: "${RRM_PSC_HOST:=data.bridges2.psc.edu}"
: "${RRM_PSC_USER:?Set RRM_PSC_USER; use an approved non-interactive PSC SSH key.}"
: "${RRM_PSC_WAIT_S:=3600}"
[[ "$RRM_PSC_WAIT_S" =~ ^[1-9][0-9]*$ ]] || { echo "RRM_PSC_WAIT_S must be positive" >&2; exit 2; }

psc_target="$RRM_PSC_USER@$RRM_PSC_HOST"
remote_request="$RRM_PSC_ROOT/incoming/rrm/$run_id"
remote_source="$RRM_PSC_ROOT/src/rrm-submissions/$run_id"
local_source=$(cd "$(dirname "$0")/.." && pwd)
ssh_args=( -o StrictHostKeyChecking=accept-new -o ControlMaster=auto -o ControlPath=/tmp/ssh-%r@%h:%p -o ControlPersist=10m "$psc_target")
rsync_ssh='ssh -o StrictHostKeyChecking=accept-new -o ControlMaster=auto -o ControlPath=/tmp/ssh-%r@%h:%p'

# Establish the master connection once so the user only types their password/Duo once.
ssh -o StrictHostKeyChecking=accept-new -o ControlMaster=yes -o ControlPath=/tmp/ssh-%r@%h:%p -o ControlPersist=10m -fN "$psc_target"
trap 'ssh -O exit -o ControlPath=/tmp/ssh-%r@%h:%p "$psc_target" 2>/dev/null || true' EXIT

ssh "${ssh_args[@]}" "mkdir -p $remote_request $remote_source"
rsync -a --delete -e "$rsync_ssh" -- "$request_dir/" "$psc_target:$remote_request/"
rsync -a --delete --exclude .git --exclude __pycache__ -e "$rsync_ssh" -- \
  "$local_source/" "$psc_target:$remote_source/"

job_id=$(ssh "${ssh_args[@]}" "PROJECT=$RRM_PSC_PROJECT RRM_PERSIST_ROOT=$RRM_PSC_ROOT sbatch -A $RRM_PSC_PROJECT $remote_source/scripts/psc_rrm_office.sbatch $remote_request $remote_source | awk '/Submitted batch job/{print \$4}'")
[[ "$job_id" =~ ^[0-9]+$ ]] || { echo "PSC submission did not return a Slurm job ID (got: $job_id)" >&2; exit 1; }

deadline=$((SECONDS + RRM_PSC_WAIT_S))
while ssh "${ssh_args[@]}" squeue -h -j "$job_id" 2>/dev/null | grep -q .; do
  (( SECONDS < deadline )) || { echo "PSC job $job_id did not finish before RRM_PSC_WAIT_S" >&2; exit 1; }
  sleep 10
done

local_bundle="$request_dir/psc-bridge-bundle-$job_id"
[[ ! -e "$local_bundle" ]] || { echo "local PSC bundle already exists" >&2; exit 1; }
rsync -a -e "$rsync_ssh" -- "$psc_target:$RRM_PSC_ROOT/runs/rrm/office/$job_id/" "$local_bundle/"
[[ -f "$local_bundle/result.json" ]] || { echo "PSC job $job_id produced no result bundle" >&2; exit 1; }
json_result=$(printf '{"run_id":"%s","job_id":"%s","bundle_dir":"%s"}\n' "$run_id" "$job_id" "$local_bundle")
echo "$json_result"
curl -s -X POST -H "Content-Type: application/json" -d "$json_result" http://127.0.0.1:8787/api/requests/manual-import || echo "Warning: could not notify GUI."
