#!/bin/bash
# Run on the OSMO host. Downloads and imports a PSC bundle; never dispatches.
set -euo pipefail
[[ $# == 1 && "$1" =~ ^[0-9]+$ ]] || {
  echo 'Usage: bash scripts/rrm_office_fetch_import.sh <job-id>' >&2; exit 2;
}
rrm_job="$1"
rrm_source=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
rrm_artifacts="$rrm_source/../../.rrm-artifacts"
rrm_container=airstack-robot-desktop-1
rrm_deps=/tmp/rrm-canonical-deps
# Reuse the isolated dependencies prepared in this session; fail before transfer
# if the container or dependencies have disappeared.
docker exec "$rrm_container" env PYTHONPATH="$rrm_deps" \
  python3 -c 'import pydantic; assert int(pydantic.__version__.split(".")[0]) == 2'
mkdir -p "$rrm_artifacts"
rrm_dest=$(mktemp -d "$rrm_artifacts/psc-office-$rrm_job.XXXXXX")
rrm_dest=$(cd "$rrm_dest" && pwd)
echo "Preserving bundle and imported proposal under: $rrm_dest"
rrm_ssh='ssh -l oabolade -o HostName=192.231.243.28 -o HostKeyAlias=data.bridges2.psc.edu'
rsync -avP -e "$rrm_ssh" \
  "data.bridges2.psc.edu:/ocean/projects/eng260004p/oabolade/physical-ai/runs/rrm/office/$rrm_job/" \
  "$rrm_dest/bundle/"
(
  cd "$rrm_dest/bundle"
  sha256sum --check SHA256SUMS
)
rrm_work=$(docker exec "$rrm_container" mktemp -d /tmp/rrm-office-import.XXXXXX)
docker exec "$rrm_container" mkdir "$rrm_work/source" "$rrm_work/bundle"
docker cp "$rrm_source/." "$rrm_container:$rrm_work/source/"
docker cp "$rrm_dest/bundle/." "$rrm_container:$rrm_work/bundle/"
docker exec "$rrm_container" env PYTHONPATH="$rrm_deps:$rrm_work/source" \
  python3 "$rrm_work/source/scripts/rrm_import_office.py" \
  --bundle "$rrm_work/bundle" --output-dir "$rrm_work/imported"
docker cp "$rrm_container:$rrm_work/imported" "$rrm_dest/imported"
echo "Verified bundle: $rrm_dest/bundle"
echo "Unexecuted proposal: $rrm_dest/imported/proposal.json"
echo 'Original evidence remains on PSC. This OSMO copy is ephemeral.'
