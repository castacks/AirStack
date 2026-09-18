#!/bin/bash
# Run on the OSMO host; reuse a prior PSC image for an inference-only retry.
set -euo pipefail
[[ $# -le 1 ]] || { echo 'Usage: bash scripts/rrm_office_retry_transfer.sh [previous-job-id]' >&2; exit 2; }
rrm_previous_job="${1:-46280177}"
[[ "$rrm_previous_job" =~ ^[0-9]+$ ]] || { echo 'Previous job ID must be numeric' >&2; exit 2; }
rrm_source=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
rrm_psc=/ocean/projects/eng260004p/oabolade/physical-ai
rrm_tag="rrm-office-ids-$(date -u +%Y%m%dT%H%M%S)-$$"
rrm_remote="$rrm_psc/src/$rrm_tag"
rrm_ssh='ssh -l oabolade -o HostName=192.231.243.28 -o HostKeyAlias=data.bridges2.psc.edu'
rsync -avP --exclude='__pycache__' --exclude='.venv' --exclude='.git' \
  -e "$rrm_ssh" "$rrm_source/" "data.bridges2.psc.edu:$rrm_remote/"
echo 'Source transfer finished. In your PSC login terminal, run:'
# Pass source explicitly: it must survive the scheduler's environment policy.
printf 'sbatch -A eng260004p --time=00:30:00 %q %q %q\n' \
  "$rrm_remote/scripts/psc_rrm_office.sbatch" \
  "$rrm_psc/runs/rrm/office/$rrm_previous_job/input.png" "$rrm_remote"
