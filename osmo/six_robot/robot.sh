#!/usr/bin/env bash
set -e
# The pinned v0.19 image uses MAVROS before the 2.15 plugin namespace change.
export MAVROS_NAMESPACE=mavros
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/AirStack/common/fastdds.xml
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /root/AirStack/robot/ros_ws/install/setup.bash
export RAYFRONTS_PER_ROBOT=false RULES_LOCAL_PLANNER=false RULES_PLANNER=false RULES_EXECUTOR=true RULES_DASHBOARD=false
export RULES_FLEET_FILE=/root/AirStack/osmo/six_robot/fleet.yaml
export RULES_AUTO_TAKEOFF=true RULES_TAKEOFF_ALTITUDE_M=3.5
export RULES_MAX_ALTITUDE_M=12
if [ "$ROBOT_NAME" = "robot_1" ] || [ "$ROBOT_NAME" = "robot_2" ]; then export RULES_MAX_ALTITUDE_M=25; fi
export RULES_REACH_RADIUS_M=1.0 RULES_REACH_ALTITUDE_M=0.6
export AIRSTACK_STACK_DIR=/root/AirStack/stacks/rules_planner
exec ros2 launch desktop_bringup robot.launch.xml
