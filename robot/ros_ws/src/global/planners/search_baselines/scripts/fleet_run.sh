#!/usr/bin/env bash
# fleet_run.sh — take off every robot container and launch ONE search method on
# each, the way the benchmark skill §2 does by hand for a single robot.
#
#   scripts/fleet_run.sh lawnmower suburb_mini          # NUM_ROBOTS containers
#   scripts/fleet_run.sh conavgpt2 suburb_mini 1        # just robot_1
#   SKIP_TAKEOFF=1 scripts/fleet_run.sh lawnmower suburb_mini   # relaunch planners only
#
# Per container: wait for MAVROS `connected: true`, TakeoffTask to
# $TAKEOFF_ALT_M (12 m), then `ros2 launch search_baselines <method>.launch.xml
# scene_params_file:=<scene>.yaml` in a tmux window `search` of the bringup
# session, tee'd to /tmp/conavgpt2/live.log inside that container. Every
# planner cuts the search area into NUM_ROBOTS sectors and flies its own —
# see README "A fleet". The shared model servers must already be healthy on
# offboard-compute (`./airstack.sh up offboard-compute`).
set -u
METHOD="${1:?method (lawnmower|conavgpt2|vlfm|frontier|nearest)}"
SCENE="${2:?scene yaml stem, e.g. suburb_mini}"
N="${3:-${NUM_ROBOTS:-$(grep -E '^NUM_ROBOTS=' "$(dirname "$0")/../../../../../../../.env" 2>/dev/null | sed 's/.*="\([0-9]*\)".*/\1/')}}"
N="${N:-1}"
ALT="${TAKEOFF_ALT_M:-12.0}"
PREFIX="${CONTAINER_PREFIX:-disaster-dataset-robot-desktop-}"

for i in $(seq 1 "$N"); do
  C="${PREFIX}${i}"
  echo "== $C (robot_$i)"
  [ "${SKIP_TAKEOFF:-0}" = "1" ] && { echo "   (SKIP_TAKEOFF=1: already airborne)"; continue; }
  docker exec "$C" bash -lc 'sws >/dev/null; for k in $(seq 1 30); do
      if timeout 6 ros2 topic echo /'"robot_$i"'/interface/mavros/state --once 2>/dev/null | grep -q "connected: true"; then echo "   mavros connected"; exit 0; fi; sleep 1; done
      echo "   mavros NOT connected"; exit 1' || continue
  docker exec "$C" bash -lc 'sws >/dev/null; ros2 action send_goal /'"robot_$i"'/tasks/takeoff task_msgs/action/TakeoffTask "{target_altitude_m: '"$ALT"', velocity_m_s: 2.0}" 2>&1 | grep -E "accepted|success|rejected" | sed "s/^/   /"' &
done
wait
for i in $(seq 1 "$N"); do
  C="${PREFIX}${i}"
  docker exec "$C" bash -c "mkdir -p /tmp/conavgpt2; tmux kill-window -t bringup:search 2>/dev/null; tmux new-window -t bringup -n search; tmux send-keys -t bringup:search 'sws && ros2 launch search_baselines ${METHOD}.launch.xml scene_params_file:=\$(ros2 pkg prefix search_baselines)/share/search_baselines/config/${SCENE}.yaml 2>&1 | tee /tmp/conavgpt2/live.log' ENTER" \
    && echo "== $C: launched $METHOD on $SCENE (log /tmp/conavgpt2/live.log)"
done
