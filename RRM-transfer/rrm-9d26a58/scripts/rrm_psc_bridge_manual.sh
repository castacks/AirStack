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
: "${RRM_PSC_HOST:=bridges2.psc.edu}"
: "${RRM_PSC_USER:?Set RRM_PSC_USER; use an approved non-interactive PSC SSH key.}"
: "${RRM_PSC_GRES:=gpu:h100-80:1}"
[[ "$RRM_PSC_GRES" =~ ^gpu:(h100-80|l40s-48):1$ ]] || {
  echo "RRM_PSC_GRES must be gpu:h100-80:1 or gpu:l40s-48:1" >&2; exit 2;
}

psc_target="$RRM_PSC_USER@$RRM_PSC_HOST"
remote_request="$RRM_PSC_ROOT/incoming/rrm/$run_id"
remote_source="$RRM_PSC_ROOT/src/rrm-submissions/$run_id"
local_source=$(cd "$(dirname "$0")/.." && pwd)
ssh_args=( -o StrictHostKeyChecking=accept-new "$psc_target")
rsync_ssh='ssh -o StrictHostKeyChecking=accept-new'

echo "Packaging and submitting to PSC (you will be prompted for your password once)..."
job_id=$(tar -czf - -C "$request_dir/.." "$run_id" -C "$local_source/.." "$(basename "$local_source")" | ssh "${ssh_args[@]}" "
  set -e
  stage=\$(mktemp -d \"$RRM_PSC_ROOT/.rrm-stage.XXXXXX\")
  trap 'rm -rf \"\$stage\"' EXIT
  # One tar stream must be extracted once. The previous two extraction commands
  # consumed stdin on the first pass, leaving the source snapshot with EOF.
  tar -xzf - -C \"\$stage\"
  test -d \"\$stage/$run_id\"
  test -d \"\$stage/$(basename "$local_source")\"
  mkdir -p \"$RRM_PSC_ROOT/incoming/rrm\" \"$RRM_PSC_ROOT/src/rrm-submissions\"
  # This retry path replaces only the exact immutable request/source staging
  # directories for this run ID; no completed PSC result directory is touched.
  rm -rf \"$remote_request\" \"$remote_source\"
  mv \"\$stage/$run_id\" \"$remote_request\"
  mv \"\$stage/$(basename "$local_source")\" \"$remote_source\"
  
  export PROJECT=\"$RRM_PSC_PROJECT\"
  export RRM_PERSIST_ROOT=\"$RRM_PSC_ROOT\"
  sbatch -A \"$RRM_PSC_PROJECT\" --gres=\"$RRM_PSC_GRES\" \"$remote_source/scripts/psc_rrm_office.sbatch\" \"$remote_request\" \"$remote_source\" | awk '/Submitted batch job/{print \$4}'
")

[[ "$job_id" =~ ^[0-9]+$ ]] || { echo "PSC submission failed or did not return a Slurm job ID (got: $job_id)" >&2; exit 1; }
echo "Job submitted to PSC successfully! Slurm Job ID: $job_id"
curl -fsS -X POST -H 'Content-Type: application/json' \
  -d "$(printf '{\"run_id\":\"%s\",\"job_id\":\"%s\"}' "$run_id" "$job_id")" \
  http://127.0.0.1:8787/api/requests/manual-submitted || echo "Warning: GUI did not record the submitted job."
echo "Watch it with: squeue -j $job_id"
echo "After it completes, fetch/import it (do not submit again):"
echo "  RRM_PSC_USER=$RRM_PSC_USER bash $(cd "$(dirname "$0")" && pwd)/rrm_psc_fetch_result.sh $request_dir $job_id"
