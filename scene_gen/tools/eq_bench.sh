#!/usr/bin/env bash
# eq_bench.sh — run ONE earthquake launcher headless in the isaac-sim
# container, serialised across agents, and wait for its banner.
#
#     scene_gen/tools/eq_bench.sh <snap_name> [VAR=value ...]
#
#     scene_gen/tools/eq_bench.sh tiltA EQ_STYLE=commercial EQ_RECIPES=tilt_sink,DG3 EQ_SEED=4
#     LAUNCHER=downtown_quake_launch_script.py scene_gen/tools/eq_bench.sh city10 SEVERITY=0.6
#
# * Runs `LAUNCHER` (default eq_building_bench_launch_script.py) via a plain
#   `docker exec` — NOT the tmux pane — with ISAAC_SIM_HEADLESS=true, so
#   several agents can queue runs without touching each other's pane.
# * At most GPU_SLOTS (default 2) Isaac processes run at once: the run takes
#   the first free slot lock under /tmp/eq_bench_locks/ and blocks until one
#   frees. One 16 GB card holds two fracturing benches; three do not fit.
# * Captures land in ~/docker/isaac-sim/logs/<snap_name>/ on the host
#   (= /isaac-sim/.nvidia-omniverse/logs/<snap_name> in the container).
# * The launcher's own stdout goes to ~/docker/isaac-sim/logs/<snap_name>.log
#   (host) — read THAT, not `docker logs`, which is empty for this container.
# * Exit code: 0 on the DONE banner, 1 on a Traceback / segfault / timeout.
#   The last 40 lines of the log are printed either way.
#
# Env for the driver itself (everything else is passed to the launcher):
#   LAUNCHER     launch script file name           (default eq_building_bench_launch_script.py)
#   GPU_SLOTS    concurrent Isaac processes        (default 2)
#   TIMEOUT_S    per-run ceiling                   (default 2400)
#   DONE_RE      regex that marks success          (default 'EQ BENCH DONE|TIMING|QUAKE ARCHETYPE BAKE')
set -u
NAME=${1:?snap name}; shift
LAUNCHER=${LAUNCHER:-eq_building_bench_launch_script.py}
GPU_SLOTS=${GPU_SLOTS:-2}
TIMEOUT_S=${TIMEOUT_S:-2400}
DONE_RE=${DONE_RE:-'EQ BENCH DONE|QUAKE ARCHETYPE BAKE|snapshots -> |^EXIT 0'}   # NOT the TIMING banner: the city prints it BEFORE its captures
HOSTLOG="$HOME/docker/isaac-sim/logs/$NAME.log"
CLOG="/isaac-sim/.nvidia-omniverse/logs/$NAME.log"
SNAP="/isaac-sim/.nvidia-omniverse/logs/$NAME"
mkdir -p /tmp/eq_bench_locks
if ! docker ps --format '{{.Names}}' | grep -qx isaac-sim; then
  echo "eq_bench: container isaac-sim is not running" >&2; exit 1
fi
ENVS="ISAAC_SIM_HEADLESS=true PYTHONUNBUFFERED=1 SNAP_DIR=$SNAP"
# .env leaks EMPTY strings for every var compose forwards (SETTLE_STEPS, ...)
# and SCENE_CONFIG=suburb; give the launcher sane values unless the caller set them
case " $* " in *" SETTLE_STEPS="*) ;; *) ENVS="$ENVS SETTLE_STEPS=2200";; esac
case "$LAUNCHER" in
  downtown_quake*|bake_quake*)
    case " $* " in *" SCENE_CONFIG="*) ;; *) ENVS="$ENVS SCENE_CONFIG=$( [ "${LAUNCHER#downtown}" != "$LAUNCHER" ] && echo downtown_earthquake || echo downtown )";; esac ;;
esac   # unbuffered: Kit hard-exits on close and block-buffered prints never reach the log
for kv in "$@"; do ENVS="$ENVS $kv"; done
# take a slot (blocks until one frees)
exec 9>/dev/null
slot=""
while [ -z "$slot" ]; do
  for i in $(seq 1 "$GPU_SLOTS"); do
    exec 9>"/tmp/eq_bench_locks/slot$i"
    if flock -n 9; then slot=$i; break; fi
  done
  [ -z "$slot" ] && sleep 15
done
echo "eq_bench[$NAME]: slot $slot, $(date +%H:%M:%S), $ENVS"
docker exec isaac-sim bash -c "rm -rf '$SNAP'; mkdir -p '$SNAP'; : > '$CLOG'"
docker exec isaac-sim bash -c "cd /isaac-sim && env $ENVS PYTHONPATH=\"\$ISAAC_SIM_PYTHONPATH\" timeout ${TIMEOUT_S}s /isaac-sim/python.sh /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/$LAUNCHER --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts --no-window > '$CLOG' 2>&1; echo \"EXIT \$?\" >> '$CLOG'" &
PID=$!
t0=$(date +%s)
rc=1
while true; do
  if grep -qE "$DONE_RE" "$HOSTLOG" 2>/dev/null; then rc=0; break; fi
  if grep -qE "Traceback \(most recent|Segmentation fault|Aborted \(core|^EXIT [1-9]" "$HOSTLOG" 2>/dev/null; then rc=1; break; fi
  if ! kill -0 $PID 2>/dev/null; then grep -qE "$DONE_RE" "$HOSTLOG" 2>/dev/null && rc=0; break; fi
  if [ $(( $(date +%s) - t0 )) -gt "$TIMEOUT_S" ]; then echo "eq_bench[$NAME]: TIMEOUT"; break; fi
  sleep 10
done
# the launcher keeps the app open for review; headless it just has to die
docker exec isaac-sim bash -c "pkill -f 'SNAP_DIR=$SNAP ' >/dev/null 2>&1; pkill -f 'logs/$NAME ' >/dev/null 2>&1; true"
sleep 3
docker exec isaac-sim bash -c "for p in \$(pgrep -f 'launch_scripts/$LAUNCHER'); do tr '\\0' ' ' < /proc/\$p/environ 2>/dev/null | grep -q 'SNAP_DIR=$SNAP ' && kill \$p; done; true" 2>/dev/null
flock -u 9
echo "eq_bench[$NAME]: $([ $rc = 0 ] && echo DONE || echo FAILED) in $(( $(date +%s) - t0 )) s; captures: $HOME/docker/isaac-sim/logs/$NAME/"
grep -E "\[(eq|quake|qarch|settle|fracture|ground)\]|EMPTY|TIMING|Traceback|Error" "$HOSTLOG" | tail -40
exit $rc
