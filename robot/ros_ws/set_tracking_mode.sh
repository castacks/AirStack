#!/usr/bin/env bash
set -euo pipefail

ros2 service call /robot_1/trajectory_controller/set_trajectory_mode \
  airstack_msgs/srv/TrajectoryMode "{mode: 3}" & p2=$!

wait $p2
echo "exploration command sent"
