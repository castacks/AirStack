#!/bin/bash
set -e

NUM_ROBOTS="${NUM_ROBOTS:-1}"
MS_AIRSIM_BINARY_PATH="${MS_AIRSIM_BINARY_PATH:-/ms-airsim-env/LinuxNoEditor/Blocks.sh}"
MS_AIRSIM_HEADLESS="${MS_AIRSIM_HEADLESS:-false}"
# Seconds to let AirSim sensors settle before PX4 starts its EKF.
# Too short → PX4 snapshots a bad local origin (altitude offset).
MS_AIRSIM_PX4_START_DELAY="${MS_AIRSIM_PX4_START_DELAY:-3}"

# Generate settings.json from template
python3 /home/ms-airsim/Documents/AirSim/generate_settings.py

# Start tmux session
tmux new -d -s ms-airsim -n airsim

# Drop the keyboard-tips status-right from common/.tmux.conf so the centered
# window list (airsim, robot_<i>_px4, robot_<i>_bridge) has room to breathe.
tmux set-option -t ms-airsim status-right ''

# Launch Microsoft AirSim (legacy) UE4 binary (optionally headless via -RenderOffScreen)
UE4_FLAGS=""
if [ "$MS_AIRSIM_HEADLESS" = "true" ]; then
    UE4_FLAGS="-RenderOffScreen -nosound"
fi
tmux send-keys -t ms-airsim:airsim \
    "sudo -u ms-airsim $MS_AIRSIM_BINARY_PATH $UE4_FLAGS" ENTER

# Build ROS workspace
cd /root/ros_ws && colcon build --symlink-install

# Launch bridge nodes — one window per robot, named robot_<i>_bridge
for i in $(seq 1 "$NUM_ROBOTS"); do
    window="robot_${i}_bridge"
    tmux new-window -t ms-airsim -n "$window"
    tmux send-keys -t "ms-airsim:$window" \
        "source /root/ros_ws/install/setup.bash && ROS_DOMAIN_ID=$i ros2 run ms_airsim_ros_bridge bridge_node --ros-args -p robot_name:=robot_$i" ENTER
done

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

echo "Waiting ${MS_AIRSIM_PX4_START_DELAY}s for AirSim sensors to settle..."
sleep "$MS_AIRSIM_PX4_START_DELAY"

# Launch PX4 SITL instances — one window per robot, named robot_<i>_px4
for i in $(seq 1 "$NUM_ROBOTS"); do
    mkdir -p "/root/px4_instance_$i"
    window="robot_${i}_px4"
    tmux new-window -t ms-airsim -n "$window"
    tmux send-keys -t "ms-airsim:$window" \
        "cd /root/px4_instance_$i && PX4_SIM_MODEL=none_iris /root/PX4-Autopilot/build/px4_sitl_default/bin/px4 /root/PX4-Autopilot/ROMFS/px4fmu_common -s /root/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS -i $i" ENTER
done



sleep infinity
