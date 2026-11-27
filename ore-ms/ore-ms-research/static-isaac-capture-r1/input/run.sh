#!/bin/bash
set -euo pipefail
bridge=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy
export LD_LIBRARY_PATH="$bridge/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$bridge/rclpy"
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=91
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
exec /isaac-sim/python.sh /capture-input/capture.py
