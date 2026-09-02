#!/usr/bin/env bash
# Publish the search vocabulary to the shared RayFronts mapper ATOMICALLY —
# one JSON array in one String message on the robot's msg_serv topic, so the
# whole ordered list arrives in a single delivery and `person` is q0/sim_0
# (multi_robot_mapping_server._on_new_text_query parses JSON arrays;
# add_queries preserves order).
#
# Run INSIDE a robot container (or any container with ROS + DDS on the robot
# network):   pub_vocab.sh [robot_index=1]
#
# The topic has msg_serv in it — /robot_N/rayfronts/new_text_query (without)
# is advertised by SST but has NO mapper subscriber (cost a mission
# 2026-09-02 15:47).
set -euo pipefail
N="${1:-1}"
# ROS setup scripts read unset AMENT_* vars — same set -u trap as
# offboard_compute.sh.
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID="$N"
# json.dumps twice: inner = the vocabulary array as a string, outer = the
# String message YAML ({"data": "..."} — JSON is a YAML subset). No manual
# quote-escaping.
MSG=$(python3 -c 'import json; v = ["person", "road", "grass", "tree", "house", "wood debris", "sky"]; print(json.dumps({"data": json.dumps(v)}))')
exec ros2 topic pub --once "/robot_${N}/rayfronts/msg_serv/new_text_query" \
  std_msgs/msg/String "$MSG"
