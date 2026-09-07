#!/usr/bin/env bash
# Fast-loop iteration on mesh damage: edit, run this, look at the picture.
#
#     scene_gen/tools/damage_loop.sh                       # the cheap default
#     scene_gen/tools/damage_loop.sh old_brick_shop pancaked
#     scene_gen/tools/damage_loop.sh midrise_14_0204a soft_storey
#     scene_gen/tools/damage_loop.sh a,b,c pancaked        # a spread, one row each
#
# Re-sends the launcher to the tmux pane of a container that is ALREADY UP, so
# the loop costs a Kit restart rather than a container recreate. The damage is
# re-run from source every time — nothing is read from or written to the
# archetype library, so what you see is the code as it is right now.
#
# Prints the launcher's own lines and the PNG path, then exits; the scene is
# left loaded in the pane for a human to look at.
set -u

ASSET="${1:-midrise_14_0204a}"
LEVEL="${2:-pancaked}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"      # AirStack/
SNAP="${SNAP_DIR:-/isaac-sim/.nvidia-omniverse/logs/damage_loop}"
HOST_SNAP="$HOME/docker/isaac-sim/logs/$(basename "$SNAP")"
C="${ISAAC_CONTAINER:-isaac-sim}"
KLOG='D="/isaac-sim/kit/logs/Kit/Isaac-Sim Python/5.1"; F="$D/$(ls -t "$D" | head -1)"'

docker exec "$C" tmux send-keys -t isaac C-c 2>/dev/null
sleep 4
docker exec "$C" tmux clear-history -t isaac 2>/dev/null
rm -f "$HOST_SNAP/${ASSET}_${LEVEL}.png"

# PASS THROUGH EVERY OTHER `DAMAGE_*` KNOB the caller happens to have set —
# DAMAGE_STAGES, DAMAGE_CUTAWAY, DAMAGE_SEED and the rest. tmux send-keys
# builds a fresh command line in the pane, so nothing is inherited: a var that
# is not named here is simply not set, and the launcher silently uses its
# default. That looked exactly like the knob having no effect.
EXTRA=""
for v in $(compgen -v | grep '^DAMAGE_' | grep -vE '^DAMAGE_(ASSET|LEVEL)$'); do
    EXTRA="$EXTRA $v=${!v}"
done

echo "=== $ASSET / $LEVEL$EXTRA  $(date +%H:%M:%S)"
docker exec "$C" tmux send-keys -t isaac \
  "clear; SNAP_DIR=$SNAP DAMAGE_ASSET=$ASSET DAMAGE_LEVEL=$LEVEL$EXTRA PYTHONPATH=\"\$ISAAC_SIM_PYTHONPATH\" /isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/damage_loop_launch_script.py --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts" ENTER

# WAIT ON THE KIT LOG, NOT THE PANE. A startup failure never reaches the pane —
# it prints to `py stderr` and the pane just returns to a prompt.
for _ in $(seq 1 "${LOOP_TIMEOUT_TICKS:-120}"); do
    sleep 10
    if docker exec "$C" bash -c "$KLOG; grep -aq '\[loop\] DONE' \"\$F\""; then
        break
    fi
    if docker exec "$C" bash -c "$KLOG; grep -aq 'py stderr\]: Traceback' \"\$F\""; then
        echo "--- FAILED ---"
        docker exec "$C" bash -c "$KLOG; grep -a 'py stderr' \"\$F\" | sed 's/.*py stderr\]: //' | tail -12"
        exit 1
    fi
done

docker exec "$C" bash -c "$KLOG; grep -a 'py stdout' \"\$F\" | sed 's/.*py stdout\]: //' | grep -a '^\[loop\]\|^\[settle\]   [0-9]'"
echo "--- host path ---"
ls -la "$HOST_SNAP/${ASSET}_${LEVEL}.png" 2>/dev/null || echo "  no PNG at $HOST_SNAP"
