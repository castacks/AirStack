#!/usr/bin/env bash
# Build and freeze every Fire/Suburban cell: 3 intensity levels x 5 people
# placements = 15 scenes.
#
# Run it INSIDE the container, from the tmux pane:
#   bash /isaac-sim/AirStack/scene_gen/tools/freeze_fire_suburban.sh
#
# WHY A SCRIPT AND NOT A `for` LOOP TYPED INTO THE PANE. Fifteen invocations
# with nine env knobs each is far past what survives being sent through
# `tmux send-keys` as one line — and a knob that silently arrives empty is not
# a syntax error, it is a scene built at the wrong intensity that looks
# plausible. The launcher reads every knob through its own `env()` helper
# because the image exports them all as "", so the failure mode is quiet.
#
# THE LADDER IS THE POINT, and it is recorded in
# `.agents/skills/freeze-disaster-dataset/SKILL.md`:
#
#   level  seed  severity  burn_frac  ignition            reads as
#   1      23    0.45      0.25       (-460, 255) hdg 0   fire at the edge of town
#   2      10    0.60      0.45       (-250,-250) hdg 45  the tuned middle
#   3      19    0.80      0.75       ( 255,-460) hdg 90  most of the plat black
#
# A LEVEL IS A DIFFERENT LAYOUT, not the same plat burnt harder — three
# intensities of one layout would score as three views of one scene. And the
# IGNITION MOVES with the level, along a different EDGE each time: the park is
# central on most seeds, so any corner-to-corner diagonal runs through it and
# the scene reads as "the fire started in the park".
#
# PEOPLE VARIANTS ARE THE SAME GEOMETRY. `PEOPLE_VARIANT=k` offsets
# `people.seed` and nothing else, so layout, fire and every damage level come
# out bit-identical across the five — that is the axis. If two variants of one
# level disagree on house counts, something has re-seeded the layout.

set -u

ARCH=/isaac-sim/AirStack/scene_gen/assets/archetypes
BASE=/isaac-sim/final_disaster_dataset/Fire/Suburban
LAUNCH=/isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/freeze_dataset_launch_script.py
EXTS=~/.local/share/ov/data/documents/Kit/shared/exts

# level : seed : severity : burn_frac : epicenter : heading
LEVELS=(
  "1:23:0.45:0.25:-460,255:0"
  "2:10:0.60:0.45:-250,-250:45"
  "3:19:0.80:0.75:255,-460:90"
)

for spec in "${LEVELS[@]}"; do
  IFS=: read -r L SEED SEV BURN EPI HDG <<< "$spec"
  for V in 1 2 3 4 5; do
    OUT="$BASE/level_$L/$V"
    # Snapshots only on the first variant of each level: they cost ~50 s and
    # the geometry is identical across the five, so four more sets of the same
    # pictures buy nothing.
    SNAPS=0; [ "$V" = "1" ] && SNAPS=1
    echo "=================================================================="
    echo "=== LEVEL $L  VARIANT $V   seed=$SEED sev=$SEV burn=$BURN epi=$EPI hdg=$HDG"
    echo "=================================================================="
    mkdir -p "$OUT"
    SCENE_CONFIG=suburb_wildfire_1000 \
    ARCH_DIR="$ARCH" \
    FREEZE_LAYOUT_SEED="$SEED" \
    FREEZE_SEVERITY="$SEV" \
    FREEZE_EPICENTER="$EPI" \
    FREEZE_HEADING="$HDG" \
    MINI_SEED=11 \
    MINI_BURN_FRAC="$BURN" \
    MINI_ELAPSED=0 \
    PEOPLE_VARIANT="$V" \
    FREEZE_SNAPS="$SNAPS" \
    FREEZE_EXPORT=1 \
    FREEZE_COLLECT=0 \
    FREEZE_EXIT=1 \
    FREEZE_HEADLESS=1 \
    FREEZE_DISASTER=wildfire \
    FREEZE_OUT="$OUT" \
    PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \
    /isaac-sim/python.sh "$LAUNCH" --ext-folder "$EXTS"
    echo "=== LEVEL $L VARIANT $V exited $?"
  done
done
echo "ALL_FIRE_SUBURBAN_CELLS_DONE"
