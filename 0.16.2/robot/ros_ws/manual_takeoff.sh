#!/usr/bin/env bash
set -euo pipefail

# Pair 1 (run in parallel)
ros2 service call /robot_1/trajectory_controller/set_trajectory_mode \
  airstack_msgs/srv/TrajectoryMode "{mode: 2}" & p1=$!

ros2 service call /robot_1/takeoff_landing_planner/set_takeoff_landing_command \
  airstack_msgs/srv/TakeoffLandingCommand "{command: 0}" & p2=$!

wait $p1 $p2
echo "takeoff command sent"
