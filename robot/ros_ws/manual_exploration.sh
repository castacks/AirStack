#!/usr/bin/env bash
set -euo pipefail

# Pair 2 (run in parallel)
ros2 service call /robot_1/behavior/global_plan_toggle \
  std_srvs/srv/Trigger "{}" & p1=$!

# ros2 service call /robot_1/trajectory_controller/set_trajectory_mode \
#   airstack_msgs/srv/TrajectoryMode "{mode: 3}" & p2=$!

wait $p1 
# wait $p2
echo "exploration command sent"
