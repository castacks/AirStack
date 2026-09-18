#!/bin/bash
# Run on the OSMO host, not PSC. Uses interactive PSC authentication if needed.
# Copies into a separate source snapshot; never replaces existing PSC RRM work.
set -euo pipefail
rrm_source=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
rrm_image="${1:?usage: bash scripts/rrm_office_transfer.sh /absolute/path/to/frozen.png}"
[[ -f "$rrm_image" ]] || { echo 'Image not found' >&2; exit 2; }
rrm_psc=/ocean/projects/eng260004p/oabolade/physical-ai
rrm_tag="rrm-office-demo-$(date -u +%Y%m%dT%H%M%SZ)"
rrm_ssh='ssh -l oabolade -o HostName=192.231.243.28 -o HostKeyAlias=data.bridges2.psc.edu'
rsync -avP --exclude='__pycache__' --exclude='.venv' --exclude='.git' \
  -e "$rrm_ssh" "$rrm_source/" "data.bridges2.psc.edu:$rrm_psc/src/$rrm_tag/"
rsync -avP -e "$rrm_ssh" "$rrm_image" \
  "data.bridges2.psc.edu:$rrm_psc/src/$rrm_tag/office-input.png"
if [[ -f "$rrm_image.json" ]]; then
  rsync -avP -e "$rrm_ssh" "$rrm_image.json" \
    "data.bridges2.psc.edu:$rrm_psc/src/$rrm_tag/office-input.png.json"
fi
rrm_scene="$rrm_source/../../simulation/isaac-sim/launch_scripts/rrm_office_visual_eval.py"
if [[ -f "$rrm_scene" ]]; then
  rsync -avP -e "$rrm_ssh" "$rrm_scene" \
    "data.bridges2.psc.edu:$rrm_psc/src/$rrm_tag/rrm_office_visual_eval.py"
fi
echo 'Transfer finished. On the PSC login terminal, run:'
printf 'sbatch -A eng260004p %q %q %q\n' \
  "$rrm_psc/src/$rrm_tag/scripts/psc_rrm_office.sbatch" \
  "$rrm_psc/src/$rrm_tag/office-input.png" "$rrm_psc/src/$rrm_tag"
