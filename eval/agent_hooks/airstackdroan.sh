#!/usr/bin/env bash
# Hook script: start AirStack Docker containers for the DROAN local planner,
# then launch the ROS bridge sidecar that connects to the SAFE benchmark.
#
# Usage (called automatically by the benchmark or manually for testing):
#   AIRSTACK_ROOT=/path/to/AirStack bash eval/agent_hooks/airstackdroan.sh
#
# Environment variables (with defaults):
#   AIRSTACK_ROOT        — repo root (default: two dirs above this script)
#   AIRSTACK_DROAN_PORT  — TCP port the sidecar listens on (default 8780)
#   AUTOLAUNCH           — set to "false" to prevent AirStack from launching
#                          the autonomy stack automatically (recommended here)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AIRSTACK_ROOT="${AIRSTACK_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
SIDECAR_PORT="${AIRSTACK_DROAN_PORT:-8780}"
SIDECAR_SCRIPT="$SCRIPT_DIR/airstack_ros_sidecar.py"

echo "[airstackdroan.sh] AIRSTACK_ROOT=$AIRSTACK_ROOT"
echo "[airstackdroan.sh] Sidecar port=$SIDECAR_PORT"

# ── 1. Bring up the robot container with DROAN planner, no autolaunch ─────────
export AUTOLAUNCH=false
cd "$AIRSTACK_ROOT"
docker compose up -d robot-desktop

# Wait for the container to be healthy / running
echo "[airstackdroan.sh] Waiting for airstack-robot-desktop-1 to be ready..."
for i in $(seq 1 30); do
    STATUS=$(docker inspect --format '{{.State.Status}}' airstack-robot-desktop-1 2>/dev/null || echo "missing")
    if [ "$STATUS" = "running" ]; then
        echo "[airstackdroan.sh] Container running."
        break
    fi
    sleep 2
done

# ── 2. Build & source the DROAN planner inside the container ──────────────────
docker exec airstack-robot-desktop-1 bash --login -c \
    "bws --packages-select droan_local_planner && sws && \
     ros2 launch droan_local_planner droan_local_planner.launch.xml &"

# Give the planner time to advertise its topics
sleep 5

# ── 3. Launch the ROS bridge sidecar ─────────────────────────────────────────
# The sidecar runs inside the container so it can import rclpy from the ROS 2 Jazzy env.
docker exec -d airstack-robot-desktop-1 bash --login -c \
    "sws && AIRSTACK_DROAN_PORT=$SIDECAR_PORT python3 /AirStack/eval/agent_hooks/airstack_ros_sidecar.py --planner droan"

echo "[airstackdroan.sh] Sidecar launched on port $SIDECAR_PORT"
