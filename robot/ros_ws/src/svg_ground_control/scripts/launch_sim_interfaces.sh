#!/bin/bash
# Launch one MAVROS interface stack per simulated drone, all on the current
# ROS_DOMAIN_ID, namespaced /drone_1 ... /drone_N.
#
# Pairs with simulation/isaac-sim/launch_scripts/svg_multi_drone_single_domain.py,
# which spawns PX4 SITL instance i with MAVLink offboard port 14540+i /
# onboard port 14580+i and MAVLink system id 1+i.
#
# Usage (inside the robot container, workspace sourced):
#   ./launch_sim_interfaces.sh <num_drones> [sim_ip]
set -euo pipefail

NUM_DRONES=${1:?usage: launch_sim_interfaces.sh <num_drones> [sim_ip]}
SIM_IP=${2:-172.31.0.200}

pids=()
cleanup() {
    echo "Stopping ${#pids[@]} interface stacks..."
    kill "${pids[@]}" 2>/dev/null || true
    wait || true
}
trap cleanup INT TERM

for i in $(seq 1 "$NUM_DRONES"); do
    name="drone_${i}"
    echo "Launching interface for ${name} (fcu udp://:$((14540 + i))@${SIM_IP}:$((14580 + i)), sysid $((1 + i)))"
    ROBOT_NAME="$name" \
    FCU_URL="udp://:$((14540 + i))@${SIM_IP}:$((14580 + i))" \
    TGT_SYSTEM="$((1 + i))" \
    ros2 launch svg_ground_control sim_drone_interface.launch.xml \
        drone_name:="$name" &
    pids+=($!)
    sleep 1
done

echo "All ${NUM_DRONES} interface stacks running. Ctrl-C to stop."
wait
