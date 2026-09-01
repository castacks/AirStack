#!/usr/bin/env bash
# render_preflight_fire.sh — REFUSE to render the urban-fire city against
# stale files on the render host.
#
# Sibling of render_preflight.sh (which covers the HURRICANE file set).
# Born 2026-09-01 while prepping airstack-dev-183 for the urban-fire
# baseline: the hurricane list does not cover fire, and a hand count of
# files says nothing about WHICH bytes are there. This compares CONTENT
# HASHES of every module simulation/isaac-sim/launch_scripts/
# urban_fire_city_launch_script.py imports at runtime -- traced by hand,
# transitively, through fire_assembly_lib / fire_bake / gac_fire /
# fire_collapse / fire_people / urban_fire / urban_fire_city / the shared
# quake_flow+fracture+damage+settle helpers, generate_scene's city-layout
# chain (city_detail/districts/city_layout/gac_props/road_markings/parks),
# and the detail/ kit-slice + kit-bake modules gac_slice/gac_storey_slice/
# kit_bake/urban_building -- plus the harvested JSON data files districts.py
# and urban_building.py read directly, and kits.json (the sliced GAC/
# downtowncity kit-piece manifest).
#
#   OSMO_SSH_LOCAL_PORT=2204 scene_gen/tools/render_preflight_fire.sh    # pod
#   RP_LOCAL_CONTAINER=isaac-sim-livestream scene_gen/tools/render_preflight_fire.sh
#
# If you add a NEW `from . import X` / `from detail import X` reachable
# from the launch script's import chain, add it to LIST below -- this script
# only knows what it is told, same caveat as render_preflight.sh.
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.." || exit 1
PORT="${OSMO_SSH_LOCAL_PORT:-2204}"
LOCAL_CTNR="${RP_LOCAL_CONTAINER:-}"

LIST=$(cat <<'FILES'
scene_gen/scene_generator.py
scene_gen/generate_scene.py
scene_gen/compile_disaster.py
scene_gen/compile_locale.py
scene_gen/layout/__init__.py
scene_gen/layout/city_layout.py
scene_gen/detail/city_detail.py
scene_gen/detail/districts.py
scene_gen/detail/gac_props.py
scene_gen/detail/road_markings.py
scene_gen/detail/parks.py
scene_gen/detail/gac_slice.py
scene_gen/detail/gac_storey_slice.py
scene_gen/detail/kit_bake.py
scene_gen/detail/urban_building.py
scene_gen/detail/modular_house.py
scene_gen/detail/vehicles.py
scene_gen/detail/suburb_parcel.py
scene_gen/disaster/fire.py
scene_gen/disaster/fire_assembly_lib.py
scene_gen/disaster/fire_bake.py
scene_gen/disaster/fire_collapse.py
scene_gen/disaster/fire_people.py
scene_gen/disaster/gac_fire.py
scene_gen/disaster/kit_substitute.py
scene_gen/disaster/soot_bake.py
scene_gen/disaster/soot_plume.py
scene_gen/disaster/urban_fire.py
scene_gen/disaster/urban_fire_city.py
scene_gen/disaster/urban_fire_spread.py
scene_gen/disaster/quake_flow.py
scene_gen/disaster/quake_sliced.py
scene_gen/disaster/quake_collapse.py
scene_gen/disaster/quake_rubble.py
scene_gen/disaster/quake_rubble_usd.py
scene_gen/disaster/fracture.py
scene_gen/disaster/damage.py
scene_gen/disaster/scorch.py
scene_gen/disaster/vegetation.py
scene_gen/disaster/people.py
scene_gen/disaster/ground_class.py
scene_gen/disaster/settle.py
scene_gen/disaster/tornado_people.py
simulation/isaac-sim/utils/scene_prep.py
simulation/isaac-sim/utils/sky_presets.py
simulation/isaac-sim/utils/snapshots_rp.py
simulation/isaac-sim/utils/snapshots.py
simulation/isaac-sim/launch_scripts/urban_fire_city_launch_script.py
scene_gen/config/presets/downtown_fire_500.yaml
scene_gen/config/harvested/burnability_table.json
scene_gen/config/harvested/downtown_west_blocks.json
scene_gen/config/harvested/standalone_buildings.json
scene_gen/assets/kits/kits.json
scene_gen/disaster/baseline_captures.py
scene_gen/tools/crop_window.py
scene_gen/tools/fc_dump_crop.py
scene_gen/config/presets/downtown_fire_1500.yaml
scene_gen/config/presets/downtown_fire_1500_lvl2.yaml
scene_gen/config/presets/downtown_fire_1500_lvl3.yaml
FILES
)

md5sum $LIST | sort -k2 > /tmp/rpf_local.md5
if [ -n "$LOCAL_CTNR" ]; then
    docker exec "$LOCAL_CTNR" bash -c "cd /isaac-sim/AirStack && md5sum $(echo $LIST | tr '\n' ' ') 2>/dev/null" | sort -k2 > /tmp/rpf_remote.md5
else
    ssh -p "$PORT" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR root@localhost \
        "cd /root/AirStack && md5sum $(echo $LIST | tr '\n' ' ') 2>/dev/null" | sort -k2 > /tmp/rpf_remote.md5
fi
if diff -q /tmp/rpf_local.md5 /tmp/rpf_remote.md5 >/dev/null; then
    echo "FIRE PREFLIGHT OK: $(wc -l < /tmp/rpf_local.md5) files content-identical"
else
    echo "FIRE PREFLIGHT FAILED — stale or missing on render host:"
    diff /tmp/rpf_local.md5 /tmp/rpf_remote.md5 | grep -E '^[<>]' | awk '{print $1, $3}' | sort | uniq -c | sort -rn | head -30
    exit 1
fi
