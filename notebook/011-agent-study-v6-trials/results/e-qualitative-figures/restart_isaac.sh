#!/bin/bash
cd /tmp/claude-1000/-home-andrew-Development-AirStack/97a43d6a-e0e4-4bf0-b482-4b0ff71e8bfd/scratchpad/ws
export AUTOLAUNCH=true NUM_ROBOTS=1 COMPOSE_PROFILES=desktop,isaac-sim
export ISAAC_SIM_HEADLESS=false DISPLAY=:1
export ISAAC_SIM_SCENE=/isaac-sim/AirStack/simulation/isaac-sim/assets/scenes/obstacles_practice.usda
export ISAAC_SIM_USE_STANDALONE=true ISAAC_SIM_SCRIPT_NAME=example_multi_px4_pegasus_launch_script.py
export PLAY_SIM_ON_START=true ENABLE_LIDAR=true
export ISAAC_SIM_FOLLOW_CAM=1 ISAAC_SIM_FOLLOW_CAM_OFFSET=-6,-5,2.5
export ISAAC_SIM_DOME_LIGHT="3500,0"
./airstack.sh down
./airstack.sh up
echo UP_EXIT=$?
sleep 20
docker stop ws-gcs-1
docker exec ws-robot-desktop-1 bash -c "apt-get update -qq >/dev/null 2>&1; apt-get install -y -qq nlohmann-json3-dev 2>&1 | tail -1"
LP=$(docker exec ws-robot-desktop-1 printenv LAUNCH_PACKAGE); echo "LAUNCH_PACKAGE=$LP"
docker exec ws-robot-desktop-1 tmux send-keys -t bringup:0.0 C-c
sleep 3
docker exec ws-robot-desktop-1 tmux send-keys -t bringup:0.0 "autolaunch $LP robot.launch.xml" ENTER
t0=$(date +%s)
until docker exec ws-robot-desktop-1 bash -c "source /opt/ros/jazzy/setup.bash; timeout 8 ros2 topic echo /robot_1/odometry_conversion/odometry --once --field pose.pose.position" >/dev/null 2>&1; do
  if [ $(( $(date +%s) - t0 )) -gt 900 ]; then echo ODOM_TIMEOUT; exit 1; fi; sleep 10; done
echo "ODOM_READY after $(( $(date +%s) - t0 ))s"
