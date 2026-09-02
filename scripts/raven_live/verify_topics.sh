#!/usr/bin/env bash
# PREFLIGHT GATE — prove every topic we depend on is actually CARRYING DATA
# before a run is allowed to continue.
#
# WHY THIS EXISTS
# Three separate runs on 2026-09-02 were flown, recorded and analysed before
# anyone noticed a topic was silent, and each cost a full bring-up (~4 min) plus
# the flight. Every one of them was a topic that EXISTED — `ros2 topic list`
# showed it, Foxglove offered it in the dropdown — but never published:
#
#   * rayfronts gates every publish on get_subscription_count() > 0
#     (messaging_services/ros.py, visualizers/ros.py). No subscriber, no data,
#     no warning. The DDS router's bare reader does NOT count as a subscriber.
#   * the dds_router allowlist had the rayfronts entries commented out, so the
#     GCS saw the topic name (advertised) and nothing else.
#   * a QoS mismatch (BEST_EFFORT publisher vs RELIABLE subscriber) drops every
#     message and logs one line, once, at discovery.
#
# `ros2 topic list` and `ros2 topic info` CANNOT catch any of these. Only an
# actual message can. So this script waits for a real message on every topic,
# on the domain that matters, and exits non-zero if any is silent.
#
# IMPORTANT: use a LONG per-topic timeout. voxels_sim/all republishes about
# every 50 s (querying.period 10 frames x dataset.frame_skip 10 at ~2 fps); a
# 10 s check reports it dead when it is healthy. That false negative cost an
# afternoon.
#
#   scripts/raven_live/verify_topics.sh [timeout_s]     # default 75
#
# Exit 0 = every required topic delivered a message. Exit 1 = at least one
# silent; the run should be aborted and its bag discarded rather than flown.
set -uo pipefail

T="${1:-75}"
ROBOT="${ROBOT:-1}"
RC="${ROBOT_CONTAINER:-disaster-dataset-robot-desktop-1}"
GC="${GCS_CONTAINER:-disaster-dataset-gcs-1}"
FAIL=0
PASS=0

chk() {  # chk <container> <domain> <topic> <required|optional>
  local c="$1" d="$2" t="$3" req="${4:-required}"
  local out
  out=$(docker exec "$c" bash -c "
      source /opt/ros/jazzy/setup.bash
      export ROS_DOMAIN_ID=$d
      timeout $T ros2 topic echo '$t' --once >/dev/null 2>&1 && echo FLOW || echo DEAD
    " 2>/dev/null | tail -1)
  if [ "$out" = "FLOW" ]; then
    printf '  \033[32mFLOW\033[0m  d%-2s %s\n' "$d" "$t"; PASS=$((PASS+1))
  elif [ "$req" = "optional" ]; then
    printf '  \033[33mSKIP\033[0m  d%-2s %s  (optional, silent >%ss)\n' "$d" "$t" "$T"
  else
    printf '  \033[31mDEAD\033[0m  d%-2s %s  (no message in %ss)\n' "$d" "$t" "$T"
    FAIL=$((FAIL+1))
  fi
}

echo "=============================================================="
echo " PREFLIGHT  robot_$ROBOT   per-topic timeout ${T}s"
echo "=============================================================="

echo ""
echo "-- sensors feeding rayfronts (robot domain $ROBOT) --"
chk "$RC" "$ROBOT" "/robot_$ROBOT/sensors/front_stereo/left/image_rect"
chk "$RC" "$ROBOT" "/robot_$ROBOT/sensors/front_stereo/left/depth_ground_truth"
chk "$RC" "$ROBOT" "/robot_$ROBOT/odometry_conversion/odometry"

echo ""
echo "-- rayfronts map output (the detection path) --"
chk "$RC" "$ROBOT" "/robot_$ROBOT/rayfronts/status"
chk "$RC" "$ROBOT" "/robot_$ROBOT/rayfronts/msg_serv/voxels_sim/all"
chk "$RC" "$ROBOT" "/robot_$ROBOT/rayfronts/msg_serv/rays_sim/all"
chk "$RC" "$ROBOT" "/robot_$ROBOT/rayfronts/voxel_rgb"

echo ""
echo "-- raven output --"
chk "$RC" "$ROBOT" "/robot_$ROBOT/global_plan"
chk "$RC" "$ROBOT" "/robot_$ROBOT/navigation_mode"
chk "$RC" "$ROBOT" "/robot_$ROBOT/debug/voxel_table"
chk "$RC" "$ROBOT" "/robot_$ROBOT/debug/frontier_table"
chk "$RC" "$ROBOT" "/robot_$ROBOT/filtered_rays"
chk "$RC" "$ROBOT" "/robot_$ROBOT/filtered_frontiers"

echo ""
echo "-- what the GCS actually renders (domain 0) --"
chk "$GC" 0 "/robot_$ROBOT/rayfronts/msg_serv/voxels_sim/all"
chk "$GC" 0 "/robot_$ROBOT/rayfronts/voxel_rgb"
chk "$GC" 0 "/gcs/payload/robot_$ROBOT/filtered_rays"
chk "$GC" 0 "/rayfronts_debug/robot_$ROBOT/voxels_sim/all" optional
chk "$GC" 0 "/rayfronts_debug/robot_$ROBOT/voxel_rgb" optional

echo ""
echo "-- query index map (sim_K -> label; sim_0 must be the TARGET) --"
docker exec "$RC" bash -c "
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=$ROBOT
    timeout 20 ros2 topic echo /robot_$ROBOT/rayfronts/status --once --full-length 2>/dev/null | head -1
  " 2>/dev/null | python3 -c "
import sys, json, re
raw = sys.stdin.read()
m = re.search(r\"'(\{.*\})'\", raw, re.S) or re.search(r'(\{.*\})', raw, re.S)
if not m:
    print('  (status topic gave nothing — cannot map indices)'); sys.exit(0)
try:
    s = json.loads(m.group(1))
except Exception as e:
    print(f'  (unparsable status: {e})'); sys.exit(0)
q = s.get('queries', [])
print(f\"  anchored={s.get('anchored')} frames={s.get('frames_robot')} \"
      f\"vox={s.get('vox_count')} rays={s.get('ray_count')} queries={len(q)}\")
for i, lbl in enumerate(q):
    mark = '   <-- TARGET, colour by this' if i == 0 else ''
    print(f'    sim_{i:<2} {lbl}{mark}')
if q and q[0] != 'person':
    print(f\"  !! sim_0 is '{q[0]}', NOT 'person' — querying.query_file must be \"
          f'null so arrival order wins, else the target lands on the LAST index.')
"

echo ""
echo "=============================================================="
if [ "$FAIL" -gt 0 ]; then
  echo " RESULT: FAIL — $FAIL required topic(s) silent, $PASS flowing."
  echo " Do NOT let the run continue; abort and discard its bag."
  exit 1
fi
echo " RESULT: PASS — all $PASS required topics flowing."
exit 0
