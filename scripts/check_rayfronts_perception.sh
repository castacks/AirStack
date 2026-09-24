#!/usr/bin/env bash
# Verify every shared-RayFronts robot has live stereo disparity and a point-cloud
# publisher.  Probe all DDS domains concurrently: with sparse camera time
# slicing, checking them serially adds one complete camera rotation per robot.

num_robots="${1:-8}"
probe_s="${RAYFRONTS_PERCEPTION_PROBE_S:-180}"
probe_dir="$(mktemp -d /tmp/rayfronts-perception.XXXXXX)"

cleanup() {
    rm -rf "${probe_dir:?}"
}
trap cleanup EXIT

source /opt/ros/jazzy/setup.bash
if [[ -f /root/AirStack/robot/ros_ws/install/setup.bash ]]; then
    source /root/AirStack/robot/ros_ws/install/setup.bash
fi
set -u

pids=()
for robot_id in $(seq 1 "$num_robots"); do
    (
        export ROS_DOMAIN_ID="$robot_id"
        disparity="/robot_${robot_id}/perception/stereo_image_proc/disparity"
        point_cloud="/robot_${robot_id}/perception/stereo_image_proc/point_cloud"

        if ! timeout "$probe_s" ros2 topic hz "$disparity" 2>/dev/null \
            | grep -m1 -q "average rate"; then
            echo "PERCEPTION_DEAD robot_${robot_id}: no disparity in ${probe_s}s"
            exit 1
        fi

        # A fresh CLI process needs a short discovery window after the topic-hz
        # subscriber exits.  This only reads graph metadata; it never subscribes
        # to the high-bandwidth point cloud.
        publishers=0
        for _ in $(seq 1 10); do
            publishers="$(ros2 topic info "$point_cloud" 2>/dev/null \
                | awk '/Publisher count:/ {print $3}')"
            publishers="${publishers:-0}"
            [[ "$publishers" -gt 0 ]] && break
            sleep 1
        done
        if [[ "$publishers" -le 0 ]]; then
            echo "PERCEPTION_DEAD robot_${robot_id}: point-cloud publishers=0"
            exit 1
        fi
        echo "PERCEPTION_OK robot_${robot_id}: disparity streaming, point-cloud publishers=${publishers}"
    ) >"$probe_dir/robot_${robot_id}.log" 2>&1 &
    pids+=("$!")
done

status=0
for pid in "${pids[@]}"; do
    wait "$pid" || status=1
done
cat "$probe_dir"/robot_*.log
exit "$status"
