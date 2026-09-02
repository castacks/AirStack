#!/usr/bin/env bash
# Log WHAT RAVEN IS FOLLOWING, live, to one host file.
#
# WHY: /global_plan tells you where the drone was sent, but not what the
# frontier tier was choosing between. When the drone flies a small box instead
# of exploring (observed 2026-09-02: a ~20 m loop, out to y=-20.7 and back,
# four times over 10 minutes) the two candidate explanations look identical
# from odometry alone:
#
#   MAP problem      the frontier tier only ever had 1-2 candidates, because at
#                    12-13 m AGL with ZED_PITCH_DEG=0 and depth_limit 20 the
#                    visible ground is a 3.6 m annulus -> almost no frontier
#                    cloud -> DBSCAN yields one blob next to the drone.
#   SCORING problem  it had plenty of candidates and picked badly
#                    (frontier_behavior MOMENTUM_WEIGHT=5.0 pulls toward the
#                    current heading; UNLOCK_RADIUS_M=5.0 releases the lock
#                    early, so it can ping-pong between two viewpoints).
#
# `frontiers raw=N kept(lo<z<hi)=K viewpoints=V` + the ranked viewpoint table
# separates them: raw≈0 or kept≈0 is the MAP; raw large but viewpoints=1-2 is
# the altitude band or DBSCAN; many viewpoints with the choice flipping is
# SCORING.
#
# Reads only. Never starts, stops or touches a compose service.
#
#   scripts/raven_live/watch_raven.sh [out_file] [period_s]
set -uo pipefail

OUT="${1:-$HOME/raven_previews/raven_watch_$(date +%Y%m%d_%H%M%S).log}"
PERIOD="${2:-20}"
ROBOT="${ROBOT:-1}"
CONTAINER="${CONTAINER:-disaster-dataset-robot-desktop-1}"
mkdir -p "$(dirname "$OUT")"

echo "watch_raven -> $OUT  (robot_$ROBOT, every ${PERIOD}s, Ctrl-C to stop)"
echo "# raven watch  robot_$ROBOT  started $(date -Is)" > "$OUT"

# raven's own stdout carries the tier line ([Frontier-based] / [Voxel-based] /
# [Ray-based] / [LVLM-guided]) and the [coverage] percentage. It only exists
# once a SemanticSearchTask goal has started raven on this robot.
docker exec "$CONTAINER" bash -c \
  "tail -F -n +1 /tmp/raven_robot_${ROBOT}.log 2>/dev/null" \
  | sed -u 's/^/[raven] /' >> "$OUT" &
TAIL_PID=$!
trap 'kill $TAIL_PID 2>/dev/null; echo; echo "stopped -> $OUT"; exit 0' INT TERM

i=0
while true; do
  i=$((i+1))
  {
    echo ""
    echo "===== poll $i  $(date +%H:%M:%S) ====="
    docker exec "$CONTAINER" bash -c "
      source /opt/ros/jazzy/setup.bash
      export ROS_DOMAIN_ID=$ROBOT
      echo -n '[odom]      '
      timeout 6 ros2 topic echo /robot_$ROBOT/odometry_conversion/odometry \
        --once --field pose.pose.position 2>/dev/null | tr '\n' ' '
      echo
      echo -n '[mode]      '
      timeout 6 ros2 topic echo /robot_$ROBOT/navigation_mode --once 2>/dev/null | head -1
      echo -n '[frontier]  '
      timeout 6 ros2 topic echo /robot_$ROBOT/debug/frontier_table --once 2>/dev/null | head -8
      echo -n '[voxels]    '
      timeout 6 ros2 topic echo /robot_$ROBOT/debug/voxel_table --once 2>/dev/null | head -1
      echo -n '[rays]      '
      timeout 6 ros2 topic echo /robot_$ROBOT/debug/ray_table --once 2>/dev/null | head -1
      echo -n '[target]    '
      timeout 6 ros2 topic echo /robot_$ROBOT/current_target --once 2>/dev/null | tr '\n' ' '
      echo
      echo -n '[status]    '
      timeout 6 ros2 topic echo /robot_$ROBOT/rayfronts/status --once --full-length 2>/dev/null \
        | head -1 | cut -c1-260
    " 2>&1
  } >> "$OUT"
  sleep "$PERIOD"
done
