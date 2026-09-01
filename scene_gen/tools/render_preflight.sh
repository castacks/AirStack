#!/usr/bin/env bash
# render_preflight.sh — REFUSE to render against stale files.
#
# Born 2026-08-31: FINAL4 rendered with stale tree archetypes on the pod while
# every count-based check passed ("34 files" says nothing about WHICH bytes).
# This compares CONTENT HASHES of every render-critical file between the local
# working tree and the render host, and exits non-zero on any mismatch.
#
#   OSMO_SSH_LOCAL_PORT=2203 scene_gen/tools/render_preflight.sh            # pod
#   RP_LOCAL_CONTAINER=isaac-sim scene_gen/tools/render_preflight.sh        # local container
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.." || exit 1
PORT="${OSMO_SSH_LOCAL_PORT:-2203}"
LOCAL_CTNR="${RP_LOCAL_CONTAINER:-}"
LIST=$( (ls scene_gen/disaster/{washaway,surge,hurricane,hurricane_flow,wind_flow,ground,planks,vegetation,bake}.py \
            scene_gen/suburb_scene.py scene_gen/scene_generator.py scene_gen/detail/modular_house.py \
            simulation/isaac-sim/launch_scripts/suburb_hurricane_launch_script.py \
            simulation/isaac-sim/utils/snapshots_rp.py \
            scene_gen/config/presets/suburb_hurricane_500_l2.yaml scene_gen/config/presets/suburb_hurricane_500_l3.yaml \
            scene_gen/tools/hurricane_cameras_png.py scene_gen/compile_disaster.py \
            scene_gen/assets/materials/megascans/Wet_Destroyed_Asphalt.usda scene_gen/assets/materials/megascans/Soil_Mud_Wet.usda; \
         ls scene_gen/assets/archetypes_hurricane/*.usd scene_gen/assets/archetypes_tornado/*.usd \
            scene_gen/assets/materials/megascans/Soil_Mud_Wet/*.png) 2>/dev/null )
md5sum $LIST | sort -k2 > /tmp/rp_local.md5
if [ -n "$LOCAL_CTNR" ]; then
    docker exec "$LOCAL_CTNR" bash -c "cd /isaac-sim/AirStack && md5sum $(echo $LIST | tr '\n' ' ') 2>/dev/null" | sort -k2 > /tmp/rp_remote.md5
else
    ssh -p "$PORT" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR root@localhost \
        "cd /root/AirStack && md5sum $(echo $LIST | tr '\n' ' ') 2>/dev/null" | sort -k2 > /tmp/rp_remote.md5
fi
if diff -q /tmp/rp_local.md5 /tmp/rp_remote.md5 >/dev/null; then
    echo "PREFLIGHT OK: $(wc -l < /tmp/rp_local.md5) files content-identical"
else
    echo "PREFLIGHT FAILED — stale or missing on render host:"
    diff /tmp/rp_local.md5 /tmp/rp_remote.md5 | grep -E '^[<>]' | awk '{print $1, $3}' | sort | uniq -c | sort -rn | head -20
    exit 1
fi
