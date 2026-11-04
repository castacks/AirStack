#!/usr/bin/env bash
# Explicit ROS environment; does not read a user's shell profile or credentials.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /opt/ws2/AirStack/robot/ros_ws/install/setup.bash
export ROBOT_NAME=robot_1 ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
function sws() { :; }
export -f sws
exec bash --noprofile --norc -c "$1" ws2 "${@:2}"
