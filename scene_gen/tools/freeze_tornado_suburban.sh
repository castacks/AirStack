#!/usr/bin/env bash
# Freeze Tornado/Suburban levels 1-3, variant 1 only.
#
#   bash /isaac-sim/AirStack/scene_gen/tools/freeze_tornado_suburban.sh
#
# ONE SCENE PER LEVEL, deliberately: the geometry is reviewed before the four
# extra people placements are spent, because each is a full ~265 MB export and
# regenerating them after a scene change is pure waste.
#
# The tornado assembly is NOT `scene_api.build_scene` — different damage model,
# different debris, its own casualty planner — so the freeze steps live in
# `suburb_tornado_launch_script.py` rather than in the wildfire freeze
# launcher. Same env knobs and the same output contract either way.
#
# The ladder (see the three presets, and freeze-disaster-dataset):
#   level  seed  severity  width_m  core  ignition            swept
#   1      13    0.55       95      0.18  (-350, 380) hdg 320   0
#   2      10    0.82      155      0.22  (-180,-300) hdg  38   4
#   3      19    0.95      240      0.32  ( 380,-350) hdg 130  20
set -u
BASE=/isaac-sim/final_disaster_dataset/Tornado/Suburban
LAUNCH=/isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/suburb_tornado_launch_script.py
EXTS=~/.local/share/ov/data/documents/Kit/shared/exts
for L in 1 2 3; do
  OUT="$BASE/level_$L/1"
  echo "=================================================================="
  echo "=== TORNADO LEVEL $L  VARIANT 1"
  echo "=================================================================="
  mkdir -p "$OUT"
  ARCH_DIR=/isaac-sim/AirStack/scene_gen/assets/archetypes_tornado \
  SCENE_CONFIG="suburb_tornado_1000_l$L" \
  TOR_SEED=11 \
  PEOPLE_SNAPS=6 \
  SNAP_DIR="$OUT/snaps" \
  FREEZE_OUT="$OUT" \
  FREEZE_EXPORT=1 \
  FREEZE_EXIT=1 \
  PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \
  /isaac-sim/python.sh "$LAUNCH" --ext-folder "$EXTS"
  echo "=== TORNADO LEVEL $L exited $?"
done
echo "ALL_TORNADO_LEVELS_DONE"
