#!/usr/bin/env bash
# osmo_provision.sh — take a submitted OSMO dev workflow all the way to
# "ready to render", unattended.
#
# The pod can sit in SCHEDULING for a long time waiting for a GPU slot, then
# needs ~10 min of image pulls, and only THEN can anything be copied onto it.
# That is too much dead time to babysit, and an agent asked to "wait for the
# pod" just gives up. So: one script, run it in the background, come back.
#
#   scene_gen/tools/osmo_provision.sh <workflow-id>
#
# Steps, each idempotent:
#   1. wait for the workflow to reach RUNNING
#   2. wait for the inner Docker stack (isaac-sim-livestream) to be up
#   3. open the ssh port-forward and leave it running
#   4. verify the pod's clone matches local HEAD
#   5. sync the assets the pod's clone is missing (see build-scenes-on-osmo)
#   6. sync the uncommitted working-tree code the scene needs
#   7. verify all of it, loudly
#
# Everything it does is described in .agents/skills/build-scenes-on-osmo.
set -uo pipefail

WF="${1:-$(cat "${HOME}/.airstack/osmo-state" 2>/dev/null)}"
[ -n "$WF" ] || { echo "usage: $0 <workflow-id>"; exit 2; }
cd "$(dirname "${BASH_SOURCE[0]}")/../.." || exit 1
say() { echo "[provision $(date +%H:%M:%S)] $*"; }

# ---- 1. RUNNING ----------------------------------------------------------
say "waiting for $WF to reach RUNNING"
for i in $(seq 1 480); do            # up to 4h of queueing
    st=$(timeout 60 osmo workflow query "$WF" 2>/dev/null \
         | awk -F': +' '/^Status/ {print $2; exit}' | tr -d ' \r\n')
    case "$st" in
        RUNNING) say "workflow RUNNING"; break ;;
        PENDING|"") sleep 30 ;;
        *) say "workflow is $st — giving up"; exit 1 ;;
    esac
done

# ---- 2/3. tunnel + inner stack -------------------------------------------
# The tunnel has to come up before we can see the inner Docker at all, so
# these two interleave: forward first, then poll the container through it.
if ! nc -z localhost 2200 2>/dev/null; then
    say "opening ssh port-forward"
    nohup ./airstack.sh osmo:ide --no-open > /tmp/osmo_ide_$WF.log 2>&1 &
    for i in $(seq 1 60); do nc -z localhost 2200 2>/dev/null && break; sleep 5; done
fi
nc -z localhost 2200 2>/dev/null || { say "tunnel never came up"; exit 1; }
say "tunnel up on localhost:2200"

say "waiting for the inner stack (isaac-sim-livestream)"
for i in $(seq 1 180); do
    out=$(./scene_gen/tools/osmo_isaac.sh ssh \
          'docker ps --format "{{.Names}}" 2>/dev/null' 2>/dev/null)
    echo "$out" | grep -q isaac-sim-livestream && { say "sim container up"; break; }
    sleep 20
done

# ---- 4. clone matches? ---------------------------------------------------
LOCAL=$(git rev-parse HEAD)
POD=$(./scene_gen/tools/osmo_isaac.sh ssh 'git -C /root/AirStack rev-parse HEAD' 2>/dev/null | tr -d ' \r\n')
say "local HEAD $LOCAL"
say "pod   HEAD $POD"
[ "$LOCAL" = "$POD" ] && say "clone MATCHES" || say "WARNING: clone differs from local HEAD"

# ---- 5. assets: NOTHING TO DO, they come from Nucleus --------------------
#
# This step used to rsync ~4.4 GB onto every pod — AEC packs, objaverse props,
# material textures — because the clone does not carry them (`.gitignore`
# excludes `scene_gen/**/*.usda`, and the packs are untracked entirely).
#
# It was five minutes a pod and it was never necessary. `airstack-dev.yaml`
# now sets AIRSTACK_ASSET_ROOT to the Nucleus mirror, exactly as every OSMO
# MISSION yaml already did, so `airstack://` resolves there instead of against
# the clone. Nucleus is on the same LAN as the worker; the same library
# uploads in four seconds from the pod.
#
# Keep `SYNC_ASSETS=1` for the case where a material has been added locally
# and not yet pushed to Nucleus — but the right fix then is
# `scene_gen/tools/upload_materials.py`, not a per-pod copy.
if [ "${SYNC_ASSETS:-0}" != "1" ]; then
    say "assets come from Nucleus (AIRSTACK_ASSET_ROOT); skipping the 4.4 GB copy"
    say "  set SYNC_ASSETS=1 to force it, or run tools/upload_materials.py instead"
else
say "syncing material .usda wrappers"
./scene_gen/tools/osmo_sync.sh $(find scene_gen -name "*.usda") >/dev/null 2>&1 \
    && say "  usda wrappers OK" || say "  usda wrappers FAILED"

for d in scene_gen/assets/materials/megascans \
         scene_gen/assets/aec/tower/Assets/Vegetation \
         scene_gen/assets/aec/brownstone/Assets/Vegetation \
         scene_gen/assets/aec/tower/Assets/Bollard_01 \
         scene_gen/assets/aec/brownstone/Props \
         scene_gen/assets/aec/brownstone/Assets/Create_Brownstone02 \
         scene_gen/assets/objaverse \
         scene_gen/assets/aec/brownstone/Materials ; do
    [ -d "$d" ] || continue
    say "syncing $d ($(du -sh "$d" 2>/dev/null | cut -f1))"
    ./scene_gen/tools/osmo_sync.sh --dir "$d" >/dev/null 2>&1 \
        && say "  done" || say "  FAILED: $d"
done

fi

# ---- 6. the uncommitted code the scene needs -----------------------------
say "syncing hurricane code (all uncommitted, so absent from the clone)"
./scene_gen/tools/osmo_sync.sh \
    scene_gen/disaster/hurricane.py \
    scene_gen/disaster/hurricane_flow.py \
    scene_gen/disaster/surge.py \
    scene_gen/disaster/washaway.py \
    scene_gen/compile_disaster.py \
    scene_gen/config/presets/suburb_hurricane_500_l2.yaml \
    scene_gen/config/presets/suburb_hurricane_500_l3.yaml \
    simulation/isaac-sim/launch_scripts/suburb_hurricane_launch_script.py \
    simulation/isaac-sim/launch_scripts/bake_hurricane_archetypes_launch_script.py \
    simulation/isaac-sim/utils/snapshots_rp.py \
    scene_gen/tools/osmo_sync.sh scene_gen/tools/osmo_pull.sh \
    scene_gen/tools/osmo_isaac.sh >/dev/null 2>&1 \
    && say "  code OK" || say "  code FAILED"

# ---- 7. verify, do not assume -------------------------------------------
say "verifying on the pod:"
./scene_gen/tools/osmo_isaac.sh exec '
  R=/isaac-sim/AirStack
  echo "  megascans .usda : $(ls $R/scene_gen/assets/materials/megascans/*.usda 2>/dev/null | wc -l)"
  echo "  Swamp_Water     : $(ls $R/scene_gen/assets/materials/megascans/Swamp_Water/ 2>/dev/null | wc -l) file(s)"
  echo "  Soil_Mud        : $(ls $R/scene_gen/assets/materials/megascans/Soil_Mud/ 2>/dev/null | wc -l) file(s)"
  echo "  tower veg       : $(du -sh $R/scene_gen/assets/aec/tower/Assets/Vegetation 2>/dev/null | cut -f1)"
  echo "  objaverse       : $(du -sh $R/scene_gen/assets/objaverse 2>/dev/null | cut -f1)"
  n=0; for f in disaster/hurricane.py disaster/hurricane_flow.py disaster/surge.py disaster/washaway.py \
                config/presets/suburb_hurricane_500_l2.yaml config/presets/suburb_hurricane_500_l3.yaml; do
    [ -f "$R/scene_gen/$f" ] && n=$((n+1)); done
  for f in launch_scripts/suburb_hurricane_launch_script.py launch_scripts/bake_hurricane_archetypes_launch_script.py utils/snapshots_rp.py; do
    [ -f "$R/simulation/isaac-sim/$f" ] && n=$((n+1)); done
  echo "  code files      : $n of 9"
'
say "PROVISION_DONE for $WF"
