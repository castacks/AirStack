#!/usr/bin/env bash
# Mode C preflight checks — run BEFORE ~/takeoff (does not arm motors).
#
# Usage (robot container, after diffaero_real.launch.py + MicroXRCEAgent):
#   export ROS_DOMAIN_ID=1
#   source install/setup.bash   # or: sws
#   ./src/svg_ground_control/scripts/preflight_diffaero_mode_c.sh drone_1
#
# Optional: CHECK_TOF=1 to require /drone_1/perception/tof streaming.

set -euo pipefail

DRONE="${1:-drone_1}"
CHECK_TOF="${CHECK_TOF:-0}"
HZ_TIMEOUT="${HZ_TIMEOUT:-5}"
FAIL=0

pass() { echo "  [OK]  $*"; }
fail() { echo "  [FAIL] $*"; FAIL=1; }
warn() { echo "  [WARN] $*"; }
section() { echo; echo "=== $* ==="; }

check_hz() {
    local topic="$1"
    local min_hz="$2"
    local qos_extra="${3:-}"
    local out
    if ! out=$(timeout "${HZ_TIMEOUT}" ros2 topic hz "${topic}" ${qos_extra} 2>&1); then
        fail "${topic} — no data within ${HZ_TIMEOUT}s"
        return
    fi
    if echo "${out}" | grep -q "average rate"; then
        pass "${topic} publishing"
        echo "       $(echo "${out}" | grep "average rate" | tail -1)"
    else
        fail "${topic} — hz did not stabilize"
    fi
}

section "Environment"
if [[ "${ROS_DOMAIN_ID:-}" != "1" ]]; then
    warn "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset} (expected 1 for Mode C)"
else
    pass "ROS_DOMAIN_ID=1"
fi

section "MicroXRCE / PX4 bridge"
if ros2 topic list 2>/dev/null | grep -q "/${DRONE}/fmu/out/vehicle_status"; then
    pass "/${DRONE}/fmu/out/vehicle_status exists"
else
    fail "/${DRONE}/fmu/out/vehicle_status missing — is MicroXRCEAgent running? VOXL connected?"
fi

ARMED=$(timeout 3 ros2 topic echo "/${DRONE}/fmu/out/vehicle_status" --once \
    --qos-reliability best_effort 2>/dev/null | grep -E "^\s*armed:" | awk '{print $2}' || true)
if [[ "${ARMED}" == "true" ]]; then
    fail "Vehicle is ARMED — disarm before preflight (RC or QGC)"
elif [[ "${ARMED}" == "false" ]]; then
    pass "Vehicle disarmed"
else
    warn "Could not read armed state (agent or px4_interface down?)"
fi

section "Mocap (NatNet)"
check_hz "/${DRONE}/pose" 30

section "External vision → PX4 EKF2"
check_hz "/${DRONE}/fmu/in/vehicle_visual_odometry" 30 \
    "--qos-reliability best_effort"
check_hz "/${DRONE}/fmu/out/vehicle_odometry" 10 \
    "--qos-reliability best_effort"

section "Ground odometry (commander input)"
check_hz "/${DRONE}/odometry_conversion/odometry" 20

section "DiffAero commander"
if ros2 node list 2>/dev/null | grep -q diffaero_velocity_commander; then
    pass "diffaero_velocity_commander node running"
else
    fail "diffaero_velocity_commander not running"
fi

if [[ "${CHECK_TOF}" == "1" ]]; then
    section "ToF perception (optional)"
    check_hz "/${DRONE}/perception/tof" 5
fi

section "Hand-carry reminder"
echo "  Carry the drone ~1 m and confirm:"
echo "    ros2 topic echo /${DRONE}/odometry_conversion/odometry --once"
echo "    RViz (/svg/viz/markers or /svg/${DRONE}/tof_image) tracks motion."
echo "  North carry → PX4 out/vehicle_odometry position[0] increases (B4b in experiment.md)."

echo
if [[ "${FAIL}" -eq 0 ]]; then
    echo "Preflight PASSED for ${DRONE}. RC kill switch ready — then ~/takeoff (hover only first)."
    exit 0
else
    echo "Preflight FAILED — fix items above before ros2 service call .../takeoff"
    exit 1
fi
