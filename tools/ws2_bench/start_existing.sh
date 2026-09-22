#!/usr/bin/env bash
# Start this demo in the existing local v0.18 containers. Does not arm.
set -euo pipefail
task_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
for container in isaac-sim airstack-robot-desktop-1; do
  if [[ $(docker inspect --format '{{.State.Running}}' "$container") == true ]]; then
    echo "Stop the previous demo in $container before using this cold-start helper." >&2
    exit 1
  fi
done
runtime="$task_dir/../../robot/ros_ws/ws2_runtime"
control_mode=${WS2_CONTROL_MODE:-demo}
patch_texture=${WS2_PATCH_TEXTURE:-/isaac-sim/AirStack/tools/ws2_bench/assets/learned_patch.png}
patch_kind=${WS2_PATCH_KIND:-Rui learned FCRN patch}
if [[ -z ${WS2_PATCH_TEXTURE:-} && ! -f "$task_dir/assets/learned_patch.png" ]]; then
  echo 'Install Rui patch first: python3 tools/ws2_bench/import_patch.py /path/to/learned_patch.png' >&2
  exit 1
fi
[[ "$control_mode" == demo || "$control_mode" == bench ]] || { echo 'Unknown WS2_CONTROL_MODE' >&2; exit 1; }
if [[ -e "$runtime/flight_guard.json" ]]; then
  echo 'Review and archive the prior flight_guard.json before starting.' >&2
  exit 1
fi
docker start isaac-sim airstack-robot-desktop-1
docker cp "$task_dir/ground_truth.py" airstack-robot-desktop-1:/tmp/ws2_ground_truth.py
docker cp "$task_dir/gt_pid.yaml" airstack-robot-desktop-1:/tmp/ws2_gt_pid.yaml
docker exec -d airstack-robot-desktop-1 bash -lc 'sws && ros2 launch desktop_bringup robot.launch.xml role:=full > /root/AirStack/robot/ros_ws/ws2_runtime/robot_gt.log 2>&1'
docker exec -d airstack-robot-desktop-1 bash -lc 'sws && python3 /tmp/ws2_ground_truth.py > /root/AirStack/robot/ros_ws/ws2_runtime/ground_truth.log 2>&1'
docker exec -d -e WS2_EXECUTE="$([[ "$control_mode" == bench ]] && echo true || echo false)" airstack-robot-desktop-1 bash -lc 'sws && ros2 launch mononav_bridge vision_planner_bridge.launch.xml vision_planner_name:=ws2_bench vision_planner_max_frame_rate:=3.0 vision_planner_execute_commands:="$WS2_EXECUTE" > /root/AirStack/robot/ros_ws/ws2_runtime/bridge_gt.log 2>&1'
if [[ "$control_mode" == demo ]]; then
for attempt in {1..30}; do
  if docker exec airstack-robot-desktop-1 pgrep -f '^/root/AirStack/robot/ros_ws/install/pid_controller/lib/pid_controller/pid_controller .*__ns:=/robot_1/control' >/dev/null; then break; fi
  sleep 1
done
docker exec airstack-robot-desktop-1 pkill -TERM -f '^/root/AirStack/robot/ros_ws/install/pid_controller/lib/pid_controller/pid_controller .*__ns:=/robot_1/control'
docker exec -d airstack-robot-desktop-1 bash -lc 'sws && ros2 run pid_controller pid_controller --ros-args -r __ns:=/ws2/control --params-file /tmp/ws2_gt_pid.yaml -r odometry:=/ws2/control/odometry -r tracking_point:=/ws2/control/reference -r command:=/robot_1/interface/cmd_roll_pitch_yawrate_thrust > /root/AirStack/robot/ros_ws/ws2_runtime/gt_pid.log 2>&1'
fi
docker exec -d -e WS2_HEADLESS="${WS2_HEADLESS:-1}" -e WS2_EPISODE_CONFIG="${WS2_EPISODE_CONFIG:-}" -e WS2_PATCH_TEXTURE="$patch_texture" -e WS2_PATCH_KIND="$patch_kind" -e WS2_OFFICE_USD="${WS2_OFFICE_USD:-/tmp/ws2_assets/Isaac/4.5/Isaac/Environments/Office/office.usd}" isaac-sim bash -lc 'PYTHONPATH="$ISAAC_SIM_PYTHONPATH" /isaac-sim/python.sh -u /isaac-sim/AirStack/tools/ws2_bench/launch_office.py --ext-folder /isaac-sim/.local/share/ov/data/documents/Kit/shared/exts > /isaac-sim/AirStack/robot/ros_ws/ws2_runtime/simulation_gt.log 2>&1'
echo 'Started; wait for fresh scene_status.json and camera readiness. Vehicle is not armed.'
