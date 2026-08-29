#!/usr/bin/env bash
# bake_quake_by_style.sh — bake the earthquake archetypes ONE STYLE PER PROCESS.
#
#     scene_gen/tools/bake_quake_by_style.sh [style ...]
#
# Drives `bake_quake_archetypes_launch_script.py` through the isaac-sim tmux
# pane once per style, waiting for each run's banner before sending the
# next. The manifest is MERGED per run, so a partial library is never lost
# and any style can be re-baked alone.
#
# WHY ONE PROCESS PER STYLE. The first full bake lost three of nine rows to
# fractures that came back empty after ~25 min in one process (see the
# earthquake skill); a fresh process fractured the same styles normally.
# Until that state is understood, isolating each style costs ~30 s of app
# start-up per style and buys a bake that cannot degrade.
#
# Assumes: container `isaac-sim` up (any launcher), fracture deps installed
# (`fracture.ensure_deps` handles it, but a FRESH container should be primed
# once — see the wildfire skill's Environment section).

set -u
STYLES=("$@")
if [ ${#STYLES[@]} -eq 0 ]; then
  STYLES=(apartment office brownstone commercial tower office_wide office_plain
          apartment_tall apartment_long walkup brownstone_row commercial_mid
          department_store dw_terrace civic_offices highrise_04)
fi
ARCH_DIR=${ARCH_DIR:-/isaac-sim/AirStack/omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/archetype}
ARCH_GRADES=${ARCH_GRADES:-DG0,DG1,DG2,DG3,DG4,DG5,SETTLE,TILT,OV}
LOG=${LOG:-/tmp/quake_bake_by_style.log}   # NOT under ~/docker/isaac-sim/logs: that dir is root-owned
KITLOG='D="/isaac-sim/kit/logs/Kit/Isaac-Sim Python/5.1"; F="$D/$(ls -t "$D" | head -1)"; echo "$F"'

pane() { docker exec isaac-sim tmux "$@"; }

for st in "${STYLES[@]}"; do
  echo "$(date +%H:%M:%S) === $st ===" | tee -a "$LOG"
  pane send-keys -t isaac C-c
  sleep 12
  PREV=$(docker exec isaac-sim bash -c "$KITLOG")
  pane clear-history -t isaac
  pane send-keys -t isaac "clear; SCENE_CONFIG=downtown ARCH_DIR=$ARCH_DIR ARCH_STYLES=$st ARCH_GRADES=$ARCH_GRADES PYTHONPATH=\"\$ISAAC_SIM_PYTHONPATH\" /isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/bake_quake_archetypes_launch_script.py --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts" ENTER
  # PIN THE LOG FILE (tornado skill): the newest kit log is the PREVIOUS
  # run's until the new process creates its own, and a fixed sleep is not
  # enough — a quick previous run left its banner in the newest file and two
  # styles were reported "done in 0 s". Wait until the newest log differs
  # from the one that was newest BEFORE the launch.
  F=""
  for _ in $(seq 1 30); do
    sleep 5
    F=$(docker exec isaac-sim bash -c "$KITLOG")
    [ "$F" != "$PREV" ] && break
  done
  if [ "$F" = "$PREV" ]; then
    echo "  no new kit log after 150 s — skipping $st" | tee -a "$LOG"
    continue
  fi
  echo "  kit log: $F" | tee -a "$LOG"
  t0=$(date +%s)
  while true; do
    if docker exec isaac-sim grep -q "QUAKE ARCHETYPE BAKE" "$F" 2>/dev/null; then
      echo "  done in $(( $(date +%s) - t0 )) s" | tee -a "$LOG"
      docker exec isaac-sim grep "py stdout" "$F" | grep "\[qarch\]\|fracture\] EMPTY" | sed 's/.*py stdout\]: //' | tee -a "$LOG"
      break
    fi
    # SPECIFIC PATTERNS: a bare "Segmentation" matched Kit's own
    # `InstanceSegmentationLegacy` node listing and aborted every style at
    # start-up on the first run of this driver.
    if docker exec isaac-sim grep -q "Traceback (most recent\|Segmentation fault\|Aborted (core" "$F" 2>/dev/null; then
      echo "  FAILED — see $F" | tee -a "$LOG"
      docker exec isaac-sim grep -A 12 "Traceback (most recent\|Segmentation fault" "$F" | tail -20 | tee -a "$LOG"
      break
    fi
    if [ $(( $(date +%s) - t0 )) -gt 2400 ]; then
      echo "  TIMEOUT after 40 min" | tee -a "$LOG"
      break
    fi
    sleep 20
  done
done
echo "$(date +%H:%M:%S) all styles attempted; manifest: $ARCH_DIR/archetypes.json" | tee -a "$LOG"
