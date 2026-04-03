#!/bin/bash
set -e

NUM_ROBOTS="${NUM_ROBOTS:-1}"
AIRSIM_BINARY_PATH="${AIRSIM_BINARY_PATH:-/airsim_env/LinuxNoEditor/Blocks.sh}"

# Generate settings.json from template
python3 /home/airsim/Documents/AirSim/generate_settings.py

# Start tmux session
tmux new -d -s airsim

# Launch AirSim UE4 binary
tmux send-keys -t airsim "sudo -u airsim $AIRSIM_BINARY_PATH" ENTER

# Wait for AirSim API to be ready
python3 -c "
import airsim, time
while True:
    try:
        c = airsim.MultirotorClient()
        c.confirmConnection()
        break
    except:
        time.sleep(1)
print('AirSim ready')
"

# Launch PX4 SITL instances
for i in $(seq 1 "$NUM_ROBOTS"); do
    mkdir -p "/root/px4_instance_$i"
    tmux new-window -t airsim
    tmux send-keys -t airsim \
        "cd /root/px4_instance_$i && PX4_SIM_MODEL=none_iris /root/PX4-Autopilot/build/px4_sitl_default/bin/px4 /root/PX4-Autopilot/ROMFS/px4fmu_common -s /root/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS -i $i" ENTER
done

# Build ROS workspace
cd /root/ros_ws && colcon build --symlink-install

# Launch bridge nodes (one per robot, each on its own ROS domain)
for i in $(seq 1 "$NUM_ROBOTS"); do
    tmux new-window -t airsim
    tmux send-keys -t airsim \
        "source /root/ros_ws/install/setup.bash && ROS_DOMAIN_ID=$i ros2 run airsim_ros_bridge bridge_node --ros-args -p robot_name:=robot_$i" ENTER
done

sleep infinity
