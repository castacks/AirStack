#!/usr/bin/env bash
# bake_quake_headless.sh — bake the earthquake archetypes headless, one style
# per Isaac process, TWO processes at a time through eq_bench.sh's GPU slots.
#
#     scene_gen/tools/bake_quake_headless.sh [style ...]
#     ARCH_DIR=... ARCH_GRADES=DG0,DG3 scene_gen/tools/bake_quake_headless.sh tower office
#
# Successor to bake_quake_by_style.sh (tmux pane, one at a time, GUI). Each
# style's stdout is ~/docker/isaac-sim/logs/bake_<style>.log; the manifest is
# merged per run under a file lock, so two styles finishing together cannot
# lose each other's records.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
STYLES=("$@")
if [ ${#STYLES[@]} -eq 0 ]; then
  STYLES=(apartment office brownstone commercial tower office_wide office_plain
          apartment_tall apartment_long walkup brownstone_row commercial_mid
          department_store dw_terrace civic_offices highrise_04)
fi
ARCH_DIR=${ARCH_DIR:-/isaac-sim/AirStack/omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/archetype}
ARCH_GRADES=${ARCH_GRADES:-DG0,DG1,DG2,DG3,DG4,DG5,SETTLE,TILT,OV}
ARCH_SEED=${ARCH_SEED:-4}
SETTLE_STEPS=${SETTLE_STEPS:-2200}
# _o_ merge: on|off|both. `both` also writes the UNMERGED twin to
# $ARCH_DIR/_raw/ from the same settled stage, which is the only honest A/B.
BAKE_MERGE=${BAKE_MERGE:-on}
LOG=${LOG:-/tmp/quake_bake_headless.log}
# EXTRA_ENV: further VAR=value pairs handed to the launcher (docker exec does
# not inherit the host environment, so `EQ_SOLID_N=0.85` or `EQ_RUBBLE=v1`
# exported on the host never reached the bake before this — round 3 was
# baked at the code defaults). Example:
#     EXTRA_ENV="EQ_SOLID_N=0.85 EQ_RUBBLE=v2" scene_gen/tools/bake_quake_headless.sh
EXTRA_ENV=${EXTRA_ENV:-}
echo "$(date +%H:%M:%S) bake start: ${STYLES[*]} -> $ARCH_DIR grades $ARCH_GRADES" | tee -a "$LOG"
t0=$(date +%s)
pids=()
for st in "${STYLES[@]}"; do
  ( LAUNCHER=bake_quake_archetypes_launch_script.py TIMEOUT_S=3000 \
    "$HERE/eq_bench.sh" "bake_$st" ARCH_DIR="$ARCH_DIR" ARCH_STYLES="$st" ARCH_GRADES="$ARCH_GRADES" \
      ARCH_SEED="$ARCH_SEED" SETTLE_STEPS="$SETTLE_STEPS" SETTLE_CULL_LEDGES=1 \
      BAKE_MERGE="$BAKE_MERGE" $EXTRA_ENV \
      > "/tmp/bake_$st.out" 2>&1
    echo "$(date +%H:%M:%S) $st: $(grep -o 'DONE in [0-9]* s\|FAILED in [0-9]* s\|TIMEOUT' /tmp/bake_$st.out | tail -1)" | tee -a "$LOG"
    grep "\[qarch\]\|EMPTY\|Traceback" "/tmp/bake_$st.out" | tail -12 >> "$LOG" ) &
  pids+=($!)
  sleep 20      # stagger the slot grabs so two runs never race the same kit log
done
wait "${pids[@]}"
echo "$(date +%H:%M:%S) all styles attempted in $(( ($(date +%s) - t0) / 60 )) min; manifest: $ARCH_DIR/archetypes.json" | tee -a "$LOG"
