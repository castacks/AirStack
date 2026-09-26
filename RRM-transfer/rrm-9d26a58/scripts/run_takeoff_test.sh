#!/bin/bash
# Automated takeoff test with diagnostic capture.
# 1. Copies the diagnostic script into the robot container
# 2. Starts the diagnostic recorder (25s capture)
# 3. Waits 3s for baseline data
# 4. Sends a takeoff action goal
# 5. Waits for results
# 6. Copies trace data back to host

set -e

TRACE_DIR="/root/AirStack/.rrm-artifacts/takeoff-diagnostics/test-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$TRACE_DIR"

echo "=== Copying diagnostic script to container ==="
docker cp /root/AirStack/RRM-transfer/rrm-9d26a58/scripts/takeoff_diagnostic.py \
  airstack-robot-desktop-1:/tmp/takeoff_diagnostic.py

echo "=== Starting diagnostic recorder (25s) ==="
docker exec -d airstack-robot-desktop-1 bash -lc \
  'source /root/AirStack/robot/ros_ws/install/setup.bash && \
   python3 /tmp/takeoff_diagnostic.py --output /tmp/takeoff-trace.jsonl --duration 25'

echo "=== Waiting 3s for baseline ==="
sleep 3

echo "=== Sending takeoff goal to 1.0m ==="
docker exec airstack-robot-desktop-1 bash -lc \
  'source /root/AirStack/robot/ros_ws/install/setup.bash && \
   ros2 action send_goal /robot_1/tasks/takeoff task_msgs/action/TakeoffTask \
   "{target_altitude_m: 1.0}" --feedback' 2>&1 | tee "$TRACE_DIR/takeoff-action.log"

echo "=== Waiting for diagnostic to finish ==="
sleep 5

echo "=== Copying trace data ==="
docker cp airstack-robot-desktop-1:/tmp/takeoff-trace.jsonl "$TRACE_DIR/trace.jsonl" 2>/dev/null || true
docker cp airstack-robot-desktop-1:/tmp/diag.log "$TRACE_DIR/diag.log" 2>/dev/null || true

echo "=== Results in $TRACE_DIR ==="
wc -l "$TRACE_DIR/trace.jsonl" 2>/dev/null || echo "No trace file"
echo "=== Done ==="
