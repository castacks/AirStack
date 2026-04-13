#!/bin/bash
set -e

NUM_ROBOTS="${NUM_ROBOTS:-1}"
MS_AIRSIM_BINARY_PATH="${MS_AIRSIM_BINARY_PATH:-/ms-airsim-env/LinuxNoEditor/Blocks.sh}"
MS_AIRSIM_HEADLESS="${MS_AIRSIM_HEADLESS:-false}"

# Generate settings.json from template
python3 /home/ms-airsim/Documents/AirSim/generate_settings.py

# Start tmux session
tmux new -d -s ms-airsim

# Launch Microsoft AirSim (legacy) UE4 binary (optionally headless via -RenderOffScreen)
UE4_FLAGS=""
if [ "$MS_AIRSIM_HEADLESS" = "true" ]; then
    UE4_FLAGS="-RenderOffScreen -nosound"
fi
tmux send-keys -t ms-airsim "sudo -u ms-airsim $MS_AIRSIM_BINARY_PATH $UE4_FLAGS" ENTER

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
    tmux new-window -t ms-airsim
    tmux send-keys -t ms-airsim \
        "cd /root/px4_instance_$i && PX4_SIM_MODEL=none_iris /root/PX4-Autopilot/build/px4_sitl_default/bin/px4 /root/PX4-Autopilot/ROMFS/px4fmu_common -s /root/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS -i $i" ENTER
done

# Build ROS workspace
cd /root/ros_ws && colcon build --symlink-install

# Launch bridge nodes (one per robot, each on its own ROS domain)
for i in $(seq 1 "$NUM_ROBOTS"); do
    tmux new-window -t ms-airsim
    tmux send-keys -t ms-airsim \
        "source /root/ros_ws/install/setup.bash && ROS_DOMAIN_ID=$i ros2 run ms_airsim_ros_bridge bridge_node --ros-args -p robot_name:=robot_$i" ENTER
done

sleep infinity
