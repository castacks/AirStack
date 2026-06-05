#!/usr/bin/env bash
# PropSpinTest for px4_interface — verifies full command path through the interface.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AIRSTACK_ROOT="$(cd "$SCRIPT_DIR/../../../../../.." && pwd)"
AIRSTACK="$AIRSTACK_ROOT/airstack.sh"

ROBOT_NAME="${ROBOT_NAME:-robot_1}"
ROBOT_CONTAINER="airstack-robot-desktop-1"
# PX4 SITL + MicroXRCEAgent bridge to DDS domain 0
PX4_DOMAIN=0
PX4_NS="px4_1"

echo "=== Killing any existing containers ==="
docker rm -f isaac-sim 2>/dev/null || true
docker rm -f "$ROBOT_CONTAINER" 2>/dev/null || true

echo "=== Starting robot container ==="
AUTOLAUNCH=false "$AIRSTACK" up robot-desktop

echo "=== Building px4_interface ==="
docker exec "$ROBOT_CONTAINER" bash -c "source ~/.bashrc && bws --packages-select px4_interface"

echo "=== Starting Isaac Sim ==="
ISAAC_SIM_USE_STANDALONE=true \
ISAAC_SIM_SCRIPT_NAME=px4_prop_spin_test.py \
AUTOLAUNCH=true \
"$AIRSTACK" up isaac-sim

echo "=== Ensuring MicroXRCE-DDS agent is available ==="
docker exec isaac-sim bash -c "
if ! which MicroXRCEAgent > /dev/null 2>&1; then
  echo 'Building MicroXRCEAgent from source (one-time, ~5 min)...'
  cd /tmp &&
  git clone --depth=1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git &&
  cd Micro-XRCE-DDS-Agent && mkdir build && cd build &&
  cmake .. -DCMAKE_BUILD_TYPE=Release &&
  make -j\$(nproc) &&
  cp MicroXRCEAgent /usr/local/bin/ &&
  echo 'Build done.'
else
  echo 'MicroXRCEAgent already available.'
fi
"

echo "=== Starting MicroXRCE-DDS agent ==="
docker exec -d isaac-sim bash -c "MicroXRCEAgent udp4 -p 8888 > /tmp/uxrce.log 2>&1"

echo "=== Waiting 15s for Isaac Sim + PX4 to boot... ==="
sleep 15

echo "=== Launching px4_interface (domain $PX4_DOMAIN) ==="
docker exec -d "$ROBOT_CONTAINER" bash -c "
export ROBOT_NAME=$ROBOT_NAME &&
export ROS_DOMAIN_ID=$PX4_DOMAIN &&
source /root/AirStack/robot/ros_ws/install/setup.bash &&
ros2 launch px4_interface px4_interface.launch.xml > /tmp/px4_interface.log 2>&1"

echo "    Interface log: docker exec $ROBOT_CONTAINER bash -c 'tail -f /tmp/px4_interface.log'"

echo "=== Running PropSpinTest through px4_interface ==="
PROP_SPIN_SCRIPT="/root/AirStack/robot/ros_ws/src/interface/px4_interface/scripts/prop_spin_test.py"
OUTPUT=$(docker exec "$ROBOT_CONTAINER" bash -c \
    "source /root/AirStack/robot/ros_ws/install/setup.bash && \
     ROS_DOMAIN_ID=$PX4_DOMAIN PX4_NAMESPACE=$PX4_NS python3 $PROP_SPIN_SCRIPT 15 2>&1")

echo "$OUTPUT" | sed 's/^/  [robot] /'

if echo "$OUTPUT" | grep -q "interface verified"; then
    echo ""
    echo "=== PASS: px4_interface verified — props are spinning in Isaac Sim! ==="
    exit 0
else
    echo ""
    echo "=== FAIL: props did not spin ==="
    echo "Interface log:"
    docker exec "$ROBOT_CONTAINER" bash -c "tail -20 /tmp/px4_interface.log 2>/dev/null"
    echo "MicroXRCE log:"
    docker exec isaac-sim bash -c "tail -20 /tmp/uxrce.log 2>/dev/null"
    exit 1
fi
