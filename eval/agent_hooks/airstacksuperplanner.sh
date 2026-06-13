#!/usr/bin/env bash
# Hook script: start AirStack Docker containers for the Super Planner,
# then launch the ROS bridge sidecar.
#
# Usage:
#   AIRSTACK_ROOT=/path/to/AirStack bash eval/agent_hooks/airstacksuperplanner.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AIRSTACK_ROOT="${AIRSTACK_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
SIDECAR_PORT="${AIRSTACK_SUPER_PORT:-8781}"

echo "[airstacksuperplanner.sh] AIRSTACK_ROOT=$AIRSTACK_ROOT"
echo "[airstacksuperplanner.sh] Sidecar port=$SIDECAR_PORT"

export AUTOLAUNCH=false
cd "$AIRSTACK_ROOT"
docker compose up -d robot-desktop

echo "[airstacksuperplanner.sh] Waiting for airstack-robot-desktop-1 to be ready..."
for i in $(seq 1 30); do
    STATUS=$(docker inspect --format '{{.State.Status}}' airstack-robot-desktop-1 2>/dev/null || echo "missing")
    if [ "$STATUS" = "running" ]; then
        echo "[airstacksuperplanner.sh] Container running."
        break
    fi
    sleep 2
done

docker exec airstack-robot-desktop-1 bash --login -c \
    "bws --packages-select super_planner && sws && \
     ros2 launch super_planner super_planner.launch.xml &"

sleep 5

docker exec -d airstack-robot-desktop-1 bash --login -c \
    "sws && AIRSTACK_SUPER_PORT=$SIDECAR_PORT python3 /AirStack/eval/agent_hooks/airstack_ros_sidecar.py --planner super"

echo "[airstacksuperplanner.sh] Sidecar launched on port $SIDECAR_PORT"
