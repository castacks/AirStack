#!/usr/bin/env bash
# run_hw_test.sh — bring up modalai_interface against real VOXL2 hardware
#
# Usage:
#   ./run_hw_test.sh                        # USB NCM: full setup + prop spin (10s)
#   ./run_hw_test.sh --wifi                 # WiFi: skip USB setup, use SSH instead
#   ./run_hw_test.sh --up-only              # bring up and leave running (no prop spin)
#   ./run_hw_test.sh --duration 5           # prop spin for N seconds
#   ./run_hw_test.sh --wifi --voxl-ip <ip>  # override VOXL2 WiFi IP (default: 192.168.123.167)
#
# WiFi mode requires sshpass: sudo apt install sshpass
# (or set up passwordless SSH keys to the VOXL2 and remove the sshpass calls)

set -euo pipefail

# ── Config ────────────────────────────────────────────────────────────────────
VOXL_USB_IP=192.168.123.2
VOXL_WIFI_IP=192.168.123.167
VOXL_SSH_PASS=oelinux123
HOST_IP=192.168.123.1
CONTAINER_NAME=modalai-hw-test
AIRSTACK_DIR="$(cd "$(dirname "$0")/../../../../../.." && pwd)"
IMAGE=airlab-docker.andrew.cmu.edu/airstack/airstack:v0.18.0_robot-x86-64_dev
ROBOT_NAME=robot_1
ROS_DOMAIN_ID=0
SPIN_DURATION=10

# ── Arg parsing ───────────────────────────────────────────────────────────────
UP_ONLY=false
WIFI=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --up-only)          UP_ONLY=true; shift ;;
        --wifi)             WIFI=true; shift ;;
        --voxl-ip)          VOXL_WIFI_IP="$2"; shift 2 ;;
        --duration)         SPIN_DURATION="$2"; shift 2 ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

# ── Helpers ───────────────────────────────────────────────────────────────────
info()  { echo "[hw-test] $*"; }
die()   { echo "[hw-test] ERROR: $*" >&2; exit 1; }
wait_adb() { info "Waiting for ADB..."; adb wait-for-device; info "ADB ready."; }
voxl_cmd() {
    if [[ "$WIFI" == true ]]; then
        sshpass -p "$VOXL_SSH_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 root@"$VOXL_WIFI_IP" "$@"
    else
        adb shell "$@"
    fi
}

# ── WiFi path ─────────────────────────────────────────────────────────────────
if [[ "$WIFI" == true ]]; then
    command -v sshpass >/dev/null || die "sshpass not found. Install with: sudo apt install sshpass"

    info "Checking SSH connectivity to $VOXL_WIFI_IP..."
    voxl_cmd "echo ok" > /dev/null || die "Cannot SSH to $VOXL_WIFI_IP — is the drone on WiFi?"
    info "VOXL2 reachable."

    info "Starting VOXL2 services..."
    voxl_cmd "systemctl start voxl-px4.service voxl-mpa-ros2.service voxl-microdds-agent.service" 2>/dev/null || true
    sleep 5

    VOXL_IP="$VOXL_WIFI_IP"

# ── USB NCM path ──────────────────────────────────────────────────────────────
else
    info "Checking ADB..."
    adb devices | grep -q "device$" || die "No ADB device found. Is the VOXL2 plugged in?"

    info "Starting usb-ncm.service on VOXL2..."
    adb shell "systemctl start usb-ncm.service" 2>/dev/null || true
    sleep 10; wait_adb

    info "Starting usb-rebind.service on VOXL2..."
    adb shell "systemctl start usb-rebind.service" 2>/dev/null || true
    sleep 12; wait_adb

    ENX_IFACE=$(ip link show | grep -o 'enx[0-9a-f]*' | head -1)
    [[ -n "$ENX_IFACE" ]] || die "No enx* interface found — USB NCM didn't come up."

    info "Assigning $HOST_IP/24 to $ENX_IFACE..."
    if ip addr show "$ENX_IFACE" | grep -q "$HOST_IP"; then
        info "(already assigned, skipping)"
    else
        sudo ip addr add "$HOST_IP/24" dev "$ENX_IFACE"
    fi

    info "Bringing wlan0 down on VOXL2..."
    adb shell "ip link set wlan0 down" 2>/dev/null || true

    info "Restarting voxl-microdds-agent..."
    adb shell "systemctl restart voxl-microdds-agent.service"
    sleep 10; wait_adb

    info "Starting VOXL2 services..."
    adb shell "systemctl start voxl-px4.service voxl-mpa-ros2.service" 2>/dev/null || true
    sleep 5

    VOXL_IP="$VOXL_USB_IP"
fi

# ── Shared: verify network ────────────────────────────────────────────────────
info "Verifying ping to $VOXL_IP..."
for i in 1 2 3 4 5; do
    ping -c 1 -W 3 "$VOXL_IP" > /dev/null 2>&1 && break
    [[ $i -eq 5 ]] && die "Cannot reach $VOXL_IP after 5 attempts."
    info "  ping attempt $i failed, retrying..."
    sleep 3
done
info "Network OK."

# ── Start container ───────────────────────────────────────────────────────────
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    info "Removing existing $CONTAINER_NAME container..."
    docker rm -f "$CONTAINER_NAME"
fi

info "Starting $CONTAINER_NAME (--network host)..."
docker run -d --network host --name "$CONTAINER_NAME" \
    -v "$AIRSTACK_DIR:/root/AirStack" \
    -e ROBOT_NAME="$ROBOT_NAME" \
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
    "$IMAGE" \
    bash -c "source /root/AirStack/robot/ros_ws/install/local_setup.bash && \
             ros2 launch modalai_interface modalai_interface.launch.xml \
             voxl_qvio_topic:=/vvhub_body_wrt_fixed/pose > /tmp/modalai.log 2>&1"

info "Waiting 8s for modalai_interface to initialize..."
sleep 8

# ── Up-only mode ──────────────────────────────────────────────────────────────
if [[ "$UP_ONLY" == true ]]; then
    info "------------------------------------------------------------"
    info "Stack is up. modalai_interface running in: $CONTAINER_NAME"
    info ""
    info "To run tests manually:"
    info "  docker exec $CONTAINER_NAME bash -c \\"
    info "    \"source /root/AirStack/robot/ros_ws/install/local_setup.bash && \\"
    info "     export ROBOT_NAME=$ROBOT_NAME ROS_DOMAIN_ID=$ROS_DOMAIN_ID && \\"
    info "     python3 /root/AirStack/robot/ros_ws/src/interface/modalai_interface/scripts/prop_spin_test.py 10 --force\""
    info ""
    info "To tear down: docker rm -f $CONTAINER_NAME"
    info "------------------------------------------------------------"
    exit 0
fi

# ── Run prop_spin_test ────────────────────────────────────────────────────────
info "Running prop_spin_test (${SPIN_DURATION}s)..."
docker exec "$CONTAINER_NAME" bash -c "
    source /root/AirStack/robot/ros_ws/install/local_setup.bash
    export ROBOT_NAME=$ROBOT_NAME ROS_DOMAIN_ID=$ROS_DOMAIN_ID
    python3 /root/AirStack/robot/ros_ws/src/interface/modalai_interface/scripts/prop_spin_test.py $SPIN_DURATION --force
"
EXIT_CODE=$?

# ── Cleanup ───────────────────────────────────────────────────────────────────
info "Tearing down $CONTAINER_NAME..."
docker rm -f "$CONTAINER_NAME" > /dev/null

if [[ $EXIT_CODE -eq 0 ]]; then
    info "SUCCESS — interface verified."
else
    die "prop_spin_test failed (exit $EXIT_CODE)."
fi
