#!/usr/bin/env bash
# PropSpinTest for px4_interface — verifies full command path through the interface.
#
# Prerequisites:
#   - px4_msgs synced to PX4 release/1.16 (see README.md)
#   - DISPLAY forwarded for Isaac Sim GUI
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AIRSTACK_ROOT="$(cd "$SCRIPT_DIR/../../../../../.." && pwd)"
AIRSTACK="$AIRSTACK_ROOT/airstack.sh"

ROBOT_NAME="${ROBOT_NAME:-robot_1}"
ROBOT_CONTAINER="airstack-robot-desktop-1"
ISAAC_LAUNCH_SCRIPT="/isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/px4_prop_spin_test.py"
ISAAC_EXT_FOLDER="~/.local/share/ov/data/documents/Kit/shared/exts"

# PX4 SITL + MicroXRCEAgent bridge to DDS domain 0
PX4_DOMAIN=0
PX4_NS="px4_1"

echo "=== Killing any existing containers ==="
docker rm -f isaac-sim 2>/dev/null || true
docker rm -f "$ROBOT_CONTAINER" 2>/dev/null || true

echo "=== Starting robot container ==="
AUTOLAUNCH=false "$AIRSTACK" up robot-desktop

echo "=== Building px4_msgs + px4_interface ==="
docker exec "$ROBOT_CONTAINER" bash -c \
    "source ~/.bashrc && bws --packages-select px4_msgs px4_interface"

echo "=== Starting Isaac Sim container (no autolaunch, for airframe patch) ==="
AUTOLAUNCH=false "$AIRSTACK" up isaac-sim

echo "=== Patching PX4 airframe (allow arm without RC in offboard) ==="
docker exec isaac-sim bash -c "
  grep -q 'COM_RCL_EXCEPT' /isaac-sim/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10015_gazebo-classic_iris || \
  echo 'param set-default COM_RCL_EXCEPT 4' >> /isaac-sim/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10015_gazebo-classic_iris
"

echo "=== Ensuring MicroXRCE-DDS agent is available ==="
docker exec isaac-sim bash -c "
if ! which MicroXRCEAgent > /dev/null 2>&1; then
  echo 'Building MicroXRCEAgent from source (one-time, ~5 min)...'
  cd /tmp &&
  git clone --depth=1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git &&
  cd Micro-XRCE-DDS-Agent && mkdir build && cd build &&
  cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local &&
  make -j\$(nproc) &&
  make install &&
  ldconfig &&
  echo 'Build done.'
else
  echo 'MicroXRCEAgent already available.'
fi
"

echo "=== Starting MicroXRCE-DDS agent (domain $PX4_DOMAIN) ==="
docker exec isaac-sim bash -c "rm -f /tmp/uxrce.log"
docker exec -d isaac-sim bash -c \
    "ROS_DOMAIN_ID=$PX4_DOMAIN MicroXRCEAgent udp4 -p 8888 -d $PX4_DOMAIN > /tmp/uxrce.log 2>&1"

echo "=== Launching Isaac Sim + PX4 GUI (domain $PX4_DOMAIN) ==="
docker exec isaac-sim bash -c \
    "tmux send-keys -t isaac 'source /isaac-sim/.bashrc && ROS_DOMAIN_ID=$PX4_DOMAIN run_isaac_python $ISAAC_LAUNCH_SCRIPT --ext-folder $ISAAC_EXT_FOLDER' ENTER"

echo "=== Waiting for PX4 to connect to MicroXRCEAgent (up to 8 min)... ==="
DEADLINE=$((SECONDS + 480))
until docker exec isaac-sim bash -c "grep -q 'establish_session' /tmp/uxrce.log 2>/dev/null"; do
    if [ $SECONDS -ge $DEADLINE ]; then
        echo "TIMEOUT: PX4 never connected to MicroXRCEAgent. Is Isaac Sim running?"
        docker exec isaac-sim bash -c "tail -20 /tmp/uxrce.log 2>/dev/null"
        exit 1
    fi
    sleep 3
    echo "  still waiting... (${SECONDS}s)"
done
echo "  PX4 connected to MicroXRCEAgent."
echo "  Waiting 15s for sim to fully initialise..."
sleep 15

echo "=== Setting up FastDDS UDP-only config in robot container ==="
docker exec "$ROBOT_CONTAINER" bash -c "
mkdir -p /root/.ros &&
printf '%s\n' \
  '<?xml version=\"1.0\" encoding=\"UTF-8\" ?>' \
  '<profiles xmlns=\"http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles\">' \
  '    <transport_descriptors>' \
  '        <transport_descriptor>' \
  '            <transport_id>UdpTransport</transport_id>' \
  '            <type>UDPv4</type>' \
  '        </transport_descriptor>' \
  '    </transport_descriptors>' \
  '    <participant profile_name=\"udp_transport_profile\" is_default_profile=\"true\">' \
  '        <rtps>' \
  '            <userTransports>' \
  '                <transport_id>UdpTransport</transport_id>' \
  '            </userTransports>' \
  '            <useBuiltinTransports>false</useBuiltinTransports>' \
  '        </rtps>' \
  '    </participant>' \
  '</profiles>' \
  > /root/.ros/fastdds.xml"

echo "=== Launching px4_interface (domain $PX4_DOMAIN) ==="
docker exec "$ROBOT_CONTAINER" bash -c "pkill -f px4_interface.launch 2>/dev/null || true"
docker exec -d "$ROBOT_CONTAINER" bash -c "
export ROBOT_NAME=$ROBOT_NAME &&
export ROS_DOMAIN_ID=$PX4_DOMAIN &&
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml &&
source /root/AirStack/robot/ros_ws/install/setup.bash &&
ros2 launch px4_interface px4_interface.launch.xml > /tmp/px4_interface.log 2>&1"

echo "    Interface log: docker exec $ROBOT_CONTAINER bash -c 'tail -f /tmp/px4_interface.log'"
echo "=== Waiting 10s for px4_interface to receive PX4 odometry... ==="
sleep 10

echo "=== Checking interface odometry ==="
if ! docker exec "$ROBOT_CONTAINER" bash -c \
    "source /root/AirStack/robot/ros_ws/install/setup.bash && \
     ROS_DOMAIN_ID=$PX4_DOMAIN timeout 5 ros2 topic echo /$ROBOT_NAME/interface/odometry --once" \
    > /dev/null 2>&1; then
    echo ""
    echo "=== FAIL: /$ROBOT_NAME/interface/odometry not flowing ==="
    echo "    Likely cause: px4_msgs version mismatch with PX4 v1.16."
    echo "    Fix: sync robot/ros_ws/src/local/controls/px4_msgs/msg from PX4/px4_msgs release/1.16"
    echo "         then rebuild: bws --packages-select px4_msgs px4_interface"
    echo ""
    echo "--- Interface log (last 20 lines) ---"
    docker exec "$ROBOT_CONTAINER" bash -c "tail -20 /tmp/px4_interface.log 2>/dev/null"
    echo ""
    echo "--- ROS out/ topics on domain $PX4_DOMAIN ---"
    docker exec "$ROBOT_CONTAINER" bash -c \
        "source /root/AirStack/robot/ros_ws/install/setup.bash && \
         ROS_DOMAIN_ID=$PX4_DOMAIN ros2 topic list 2>/dev/null | grep 'out/' | head -20"
    exit 1
fi
echo "  Interface odometry: OK"

echo "=== Running PropSpinTest through px4_interface ==="
PROP_SPIN_SCRIPT="/root/AirStack/robot/ros_ws/src/interface/px4_interface/scripts/prop_spin_test.py"
OUTPUT=$(docker exec "$ROBOT_CONTAINER" bash -c \
    "source /root/AirStack/robot/ros_ws/install/setup.bash && \
     FASTRTPS_DEFAULT_PROFILES_FILE=/root/.ros/fastdds.xml \
     ROS_DOMAIN_ID=$PX4_DOMAIN PX4_NAMESPACE=$PX4_NS python3 $PROP_SPIN_SCRIPT 15 2>&1") || true

echo "$OUTPUT" | sed 's/^/  [robot] /'

if echo "$OUTPUT" | grep -q "interface verified"; then
    echo ""
    echo "=== PASS: px4_interface verified — props are spinning in Isaac Sim! ==="
    exit 0
else
    echo ""
    echo "=== FAIL: props did not spin ==="
    echo ""
    echo "--- Interface log (last 30 lines) ---"
    docker exec "$ROBOT_CONTAINER" bash -c "tail -30 /tmp/px4_interface.log 2>/dev/null"
    echo ""
    echo "--- MicroXRCE log (last 30 lines) ---"
    docker exec isaac-sim bash -c "tail -30 /tmp/uxrce.log 2>/dev/null"
    echo ""
    echo "--- PX4 SITL output (arm/prearm messages) ---"
    docker logs isaac-sim 2>&1 | grep -iE "arm|preflight|pre_arm|rcl|except|reject|deny|offboard|manual_control" | tail -30
    echo ""
    echo "--- ROS topics visible on domain $PX4_DOMAIN from robot container ---"
    docker exec "$ROBOT_CONTAINER" bash -c \
        "source /root/AirStack/robot/ros_ws/install/setup.bash && \
         ROS_DOMAIN_ID=$PX4_DOMAIN ros2 topic list 2>/dev/null | grep -E 'px4|fmu|vehicle' | head -30"
    exit 1
fi
