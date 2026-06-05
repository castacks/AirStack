#!/usr/bin/env bash
# PropSpinTest for modalai_interface.
# Spins props in sim via uXRCE-DDS to verify the full ModalAI interface path.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AIRSTACK_ROOT="$(cd "$SCRIPT_DIR/../../../../../.." && pwd)"
AIRSTACK="$AIRSTACK_ROOT/airstack.sh"

ROBOT_NAME="${ROBOT_NAME:-robot_1}"
ROBOT_CONTAINER="airstack-robot-desktop-1"
# Must match ROS_DOMAIN_ID inside the robot container (container name → robot_1 → domain 1)
ROS_DOMAIN="${ROS_DOMAIN_ID:-1}"

echo "=== Killing any existing containers ==="
docker rm -f isaac-sim 2>/dev/null || true
docker rm -f "$ROBOT_CONTAINER" 2>/dev/null || true

echo "=== Starting robot container ==="
AUTOLAUNCH=false "$AIRSTACK" up robot-desktop

echo "=== Building modalai_interface ==="
docker exec "$ROBOT_CONTAINER" bash -c "source ~/.bashrc && bws --packages-select modalai_interface"

echo "=== Launching modalai_interface (backgrounded) ==="
docker exec -d "$ROBOT_CONTAINER" bash -c \
    "export ROBOT_NAME=$ROBOT_NAME && \
     source /root/AirStack/robot/ros_ws/install/setup.bash && \
     ros2 launch modalai_interface modalai_sim.launch.xml \
     > /tmp/modalai_interface.log 2>&1"

echo "    Logs: docker exec $ROBOT_CONTAINER bash -c 'tail -f /tmp/modalai_interface.log'"

echo "=== Starting Isaac Sim ==="
ISAAC_SIM_USE_STANDALONE=true \
ISAAC_SIM_SCRIPT_NAME=modalai_voxl2_pegasus_launch_script.py \
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

echo "=== Starting MicroXRCE-DDS agent (domain $ROS_DOMAIN) ==="
docker exec -d isaac-sim bash -c "
ROS_DOMAIN_ID=${ROS_DOMAIN} MicroXRCEAgent udp4 -p 8888 -d ${ROS_DOMAIN} > /tmp/uxrce.log 2>&1"

echo "=== Waiting 10s for Isaac Sim + PX4 to come up... ==="
sleep 10

echo "=== Running PropSpinTest in robot container ==="
PROP_SPIN_SCRIPT="/root/AirStack/robot/ros_ws/src/interface/modalai_interface/scripts/prop_spin_test.py"
docker exec "$ROBOT_CONTAINER" bash -c \
    "source /root/AirStack/robot/ros_ws/install/setup.bash && \
     ROS_DOMAIN_ID=${ROS_DOMAIN} python3 ${PROP_SPIN_SCRIPT} 15 2>&1" | \
while IFS= read -r line; do
    echo "  [robot] $line"
    if echo "$line" | grep -q "interface verified"; then
        echo ""
        echo "=== PASS: ModalAI interface verified — props are spinning! ==="
        exit 0
    fi
    if echo "$line" | grep -q "TIMEOUT\|Error\|error"; then
        echo ""
        echo "=== FAIL: $line ==="
        exit 1
    fi
done

RESULT=${PIPESTATUS[0]}
if [ "$RESULT" -eq 0 ]; then
    echo "=== PASS: ModalAI interface verified — props are spinning! ==="
    exit 0
else
    echo "=== FAIL: PropSpinTest exited with code $RESULT ==="
    echo "Interface log:"
    docker exec "$ROBOT_CONTAINER" bash -c "tail -20 /tmp/modalai_interface.log 2>/dev/null"
    echo "MicroXRCE log:"
    docker exec isaac-sim bash -c "tail -20 /tmp/uxrce.log 2>/dev/null"
    exit 1
fi
