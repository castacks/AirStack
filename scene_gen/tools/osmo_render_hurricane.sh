#!/usr/bin/env bash
# Render one hurricane preset on the pod and pull its frames.
#   osmo_render_hurricane.sh <preset> <snapname> [EXTRA=env ...]
# Resolves the sim container rather than assuming its name (see the
# build-scenes-on-osmo skill's trap list).
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.." || exit 1
P="${1:?preset}"; N="${2:?snapname}"; shift 2
CT=$(./scene_gen/tools/osmo_isaac.sh ssh 'docker ps --format "{{.Names}}" | grep -m1 isaac-sim')
[ -n "$CT" ] || { echo "no isaac-sim container"; exit 1; }
ENVS="PYTHONUNBUFFERED=1 ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes_hurricane SCENE_CONFIG=$P HUR_SEED=11 HUR_HEADLESS=1 SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/$N $*"
./scene_gen/tools/osmo_isaac.sh exec 'pkill -9 -f suburb_hurricane_launch_script 2>/dev/null; sleep 2' >/dev/null 2>&1
./scene_gen/tools/osmo_isaac.sh ssh "docker exec -d $CT bash -lc '$ENVS /isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/suburb_hurricane_launch_script.py --ext-folder /isaac-sim/.local/share/ov/data/documents/Kit/shared/exts > /isaac-sim/.nvidia-omniverse/logs/$N.log 2>&1'"
echo "[$N] launched $(date +%H:%M:%S)"
until ./scene_gen/tools/osmo_isaac.sh exec "grep -c SCENE_DONE /isaac-sim/.nvidia-omniverse/logs/$N.log 2>/dev/null || echo 0" 2>/dev/null | tr -d ' \r' | grep -q '^1$'; do
    ./scene_gen/tools/osmo_isaac.sh exec 'pgrep -cf suburb_hurricane_launch_script' 2>/dev/null | tr -d ' \r' | grep -q '^0$' && { echo "[$N] DIED"; break; }
    sleep 30
done
./scene_gen/tools/osmo_isaac.sh exec "grep -E '^\[hurricane\]|GAVE UP|implausible' /isaac-sim/.nvidia-omniverse/logs/$N.log | tail -14"
./scene_gen/tools/osmo_pull.sh "$N" "$HOME/hurricane_previews/$N" 2>&1 | tail -1
echo "[$N] frames: $(ls "$HOME/hurricane_previews/$N"/*.png 2>/dev/null | wc -l)"
