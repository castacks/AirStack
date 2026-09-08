#!/bin/bash
S=/tmp/claude-1000/-home-andrew-Development-AirStack/97a43d6a-e0e4-4bf0-b482-4b0ff71e8bfd/scratchpad
$S/restart_isaac.sh > $S/restart_isaac2.log 2>&1
grep -q ODOM_READY $S/restart_isaac2.log || { echo RESTART_FAILED; tail -5 $S/restart_isaac2.log; exit 1; }
sleep 15
mkdir -p $S/shots/flight2; rm -f $S/shots/flight2/*
docker exec ws-robot-desktop-1 bash -lc "timeout 90 ros2 action send_goal /robot_1/tasks/takeoff task_msgs/action/TakeoffTask '{target_altitude_m: 10.0, velocity_m_s: 1.0}'" 2>&1 | grep -i "success\|error\|rejected" | head -2
for i in $(seq -w 1 45); do
  python3 $S/xcap.py "Isaac Sim Python" $S/shots/flight2/isaac_$i.png >/dev/null 2>&1
  docker exec ws-robot-desktop-1 bash -lc "timeout 5 ros2 topic echo /robot_1/odometry_conversion/odometry --once --field pose.pose.position" 2>/dev/null | grep -A2 "^x:" | tr '\n' ' ' > $S/shots/flight2/pos_$i.txt
  sleep 2
done
echo CAPTURE2_DONE
