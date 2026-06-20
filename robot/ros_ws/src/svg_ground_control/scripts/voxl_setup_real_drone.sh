#!/bin/bash
# =============================================================================
# voxl_setup_real_drone.sh — one-shot VOXL2 (ModalAI / PX4) comms provisioning
# for AirStack SVG ground control.
#
# Run this ON THE DRONE (VOXL2 adb shell), NOT in the AirStack container.
# It makes a fresh VOXL2 talk to a remote XRCE-DDS agent on the ground PC so the
# drone's PX4 topics show up as /<robot_name>/fmu/... on the ground stack.
#
# What it fixes (the whole bring-up arc, idempotently):
#   1. Points PX4's microdds_client / uxrce_dds_client at the ground PC
#      (-h <host_ip> -p <port>) and namespaces it (-n <robot_name>) so topics
#      land at /<robot_name>/fmu/... — editing /usr/bin/voxl-px4-start, the
#      authoritative, persistent startup script.
#   2. Pins the DDS domain to <domain_id> via the PX4 param (UXRCE_DDS_DOM_ID or
#      the older XRCE_DDS_DOM_ID — whichever this build has) AND as a boot-time
#      `param set` in voxl-px4-start so it survives a flash param reset.
#   3. Disables the onboard voxl-microdds-agent (you use the REMOTE agent on the
#      ground PC instead; the local one would just fight for the link).
#   4. Warns if a stray microdds_client is injected via /etc/modalai/voxl-px4.conf
#      EXTRA_STEPS (a second client on 127.0.0.1 can win and undo step 1).
#   5. Restarts voxl-px4 from the edited script and verifies the session.
#
# NOT in scope: flight/EKF tuning (EKF2_EV_CTRL for mocap fusion, failsafes, RC
# kill switch). This script only wires up COMMS. Do flight config separately.
#
# Usage (as root on the VOXL):
#   ./voxl_setup_real_drone.sh <robot_name> <ground_pc_ip> [domain_id] [port]
# Example:
#   ./voxl_setup_real_drone.sh drone_1 192.168.123.134 1 8888
#
# After it finishes, ON THE GROUND PC (ROS_DOMAIN_ID=<domain_id>):
#   MicroXRCEAgent udp4 -p <port> -v4         # must be running to bridge
#   ros2 topic list | grep <robot_name>/fmu   # topics appear HERE, not on the VOXL
# =============================================================================
set -euo pipefail

# ----- args ------------------------------------------------------------------
if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ] || [ $# -lt 2 ]; then
    grep -E '^#( |$)' "$0" | sed -E 's/^# ?//'
    exit "$([ $# -lt 2 ] && echo 1 || echo 0)"
fi

ROBOT_NAME="$1"
HOST_IP="$2"
DOMAIN_ID="${3:-1}"
PORT="${4:-8888}"

START="/usr/bin/voxl-px4-start"
CONF="/etc/modalai/voxl-px4.conf"
AGENT_SVC="voxl-microdds-agent"

log()  { printf '\033[1;32m[setup]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[warn ]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[error]\033[0m %s\n' "$*" >&2; exit 1; }

# ----- preflight -------------------------------------------------------------
[ "$(id -u)" -eq 0 ] || die "Must run as root (you are $(id -un)). Re-run with sudo."

# robot name: lowercase letters/digits/underscore (valid ROS namespace token)
echo "$ROBOT_NAME" | grep -qE '^[a-zA-Z][a-zA-Z0-9_]*$' \
    || die "robot_name '$ROBOT_NAME' is not a valid ROS namespace token."
# host ip: loose IPv4 check
echo "$HOST_IP" | grep -qE '^([0-9]{1,3}\.){3}[0-9]{1,3}$' \
    || die "ground_pc_ip '$HOST_IP' does not look like an IPv4 address."
echo "$DOMAIN_ID" | grep -qE '^[0-9]+$' || die "domain_id '$DOMAIN_ID' must be an integer."

[ -f "$START" ] || die "$START not found — is this a ModalAI VOXL2 with voxl-px4 installed?"
[ -w "$START" ] || die "$START is not writable. Try:  mount -o remount,rw /  then re-run."

log "robot_name=$ROBOT_NAME  ground_pc_ip=$HOST_IP  domain_id=$DOMAIN_ID  port=$PORT"

# ----- detect the PX4 module name + param command ----------------------------
# ModalAI ships the older 'microdds_client'; upstream/newer is 'uxrce_dds_client'.
MODULE="$(grep -oE '(microdds_client|uxrce_dds_client)' "$START" 2>/dev/null | head -n1 || true)"
if [ -z "$MODULE" ]; then
    for m in microdds_client uxrce_dds_client; do
        command -v "px4-$m" >/dev/null 2>&1 && MODULE="$m" && break
    done
fi
[ -z "$MODULE" ] && MODULE="microdds_client" && warn "Could not detect module name; defaulting to microdds_client."
log "PX4 XRCE module: $MODULE"

PARAM_CMD=""
command -v px4-param >/dev/null 2>&1 && PARAM_CMD="px4-param"

# Wait for the running PX4 to answer (px4- wrappers need the daemon up).
wait_px4() {
    local i
    for i in $(seq 1 40); do
        if [ -n "$PARAM_CMD" ] && $PARAM_CMD show SYS_AUTOSTART >/dev/null 2>&1; then
            return 0
        fi
        sleep 1
    done
    return 1
}

# Find which DDS-domain param this build actually has (query the LIVE px4 now).
DOMAIN_PARAM=""
if [ -n "$PARAM_CMD" ] && wait_px4; then
    for cand in UXRCE_DDS_DOM_ID XRCE_DDS_DOM_ID; do
        out="$($PARAM_CMD show "$cand" 2>&1 || true)"
        if echo "$out" | grep -qi 'not found'; then
            continue                      # param absent on this build
        fi
        if echo "$out" | grep -q "$cand"; then
            DOMAIN_PARAM="$cand"
            break
        fi
    done
fi
if [ -n "$DOMAIN_PARAM" ]; then
    log "DDS-domain param on this build: $DOMAIN_PARAM"
else
    warn "No client-side DDS-domain param detected (or PX4 not responding)."
    warn "Will inject best-effort param-set lines; if neither exists, set the domain on the"
    warn "ground-PC agent instead (run it under ROS_DOMAIN_ID=$DOMAIN_ID)."
fi

# ----- 1+2. rewrite the client start line + boot-time domain param -----------
BAK="$START.bak.$(date +%Y%m%d-%H%M%S)"
cp -p "$START" "$BAK"
log "Backed up $START -> $BAK"

# Idempotent: strip any prior provisioning block + the original client line,
# then insert a fresh block (domain param-set BEFORE the client start).
awk -v host="$HOST_IP" -v port="$PORT" -v name="$ROBOT_NAME" -v dom="$DOMAIN_ID" \
    -v mod="$MODULE" -v dparam="$DOMAIN_PARAM" '
    function emit() {
        print "# >>> svg-real-drone provisioning (voxl_setup_real_drone.sh) >>>"
        if (dparam != "") {
            print "param set " dparam " " dom
        } else {
            print "param set UXRCE_DDS_DOM_ID " dom
            print "param set XRCE_DDS_DOM_ID " dom
        }
        print mod " start -t udp -h " host " -p " port " -n " name
        print "# <<< svg-real-drone provisioning <<<"
    }
    # Match by FIELDS, not POSIX-class regex: mawk (Ubuntu 18.04 default, common
    # on VOXL2) does NOT support [[:space:]] — field matching is portable across
    # mawk/busybox/gawk. awk default FS splits on whitespace and ignores leading
    # indentation, so $1 is the first token even if the line is indented.
    $1 == "#" && $2 == ">>>" && $3 == "svg-real-drone" { if (!done) { emit(); done=1 } skip=1; next }
    $1 == "#" && $2 == "<<<" && $3 == "svg-real-drone" { skip=0; next }
    skip == 1 { next }
    ($1 == "microdds_client" || $1 == "uxrce_dds_client") && $2 == "start" {
        if (!done) { emit(); done=1 }
        next
    }
    { print }
    END { if (!done) emit() }
' "$START" > "$START.tmp"
chmod --reference="$START" "$START.tmp" 2>/dev/null || chmod +x "$START.tmp"
mv "$START.tmp" "$START"

# Verify the edit actually landed — never silently no-op. If the new host/name
# aren't present, the rewrite failed (e.g. awk incompatibility); restore + abort.
client_lines="$(awk '($1=="microdds_client"||$1=="uxrce_dds_client")&&$2=="start"{c++} END{print c+0}' "$START")"
if ! grep -q -- "-h $HOST_IP" "$START" || ! grep -q -- "-n $ROBOT_NAME" "$START"; then
    cp -p "$BAK" "$START"
    die "Rewrite verification FAILED (host/name not found in $START). Restored from $BAK. \
Check 'awk' on this device (mawk vs gawk) and report the client-start line in $START."
fi
[ "$client_lines" = "1" ] || warn "Expected exactly 1 client-start line in $START, found $client_lines — inspect it."
log "Rewrote $MODULE start line -> -h $HOST_IP -p $PORT -n $ROBOT_NAME (domain $DOMAIN_ID)"

# ----- 3. disable the onboard agent (we use the remote one) ------------------
if systemctl list-unit-files 2>/dev/null | grep -q "^${AGENT_SVC}"; then
    systemctl disable --now "$AGENT_SVC" >/dev/null 2>&1 || true
    log "Disabled onboard $AGENT_SVC (using remote agent on $HOST_IP)."
else
    warn "$AGENT_SVC unit not found; skipping (nothing to disable)."
fi

# ----- 4. warn about a stray localhost client in the conf --------------------
if [ -f "$CONF" ] && grep -nE '(microdds_client|uxrce_dds_client).*(127\.0\.0\.1|start)' "$CONF" >/dev/null 2>&1; then
    warn "Found a microdds/uxrce client reference in $CONF (EXTRA_STEPS?):"
    grep -nE '(microdds_client|uxrce_dds_client)' "$CONF" | sed 's/^/        /' >&2
    warn "If it starts a SECOND client on 127.0.0.1 it can override step 1 — remove/fix it."
fi

# ----- 5. restart + verify ---------------------------------------------------
log "Restarting voxl-px4 to apply the edited start script..."
systemctl restart voxl-px4

if ! wait_px4; then
    warn "PX4 did not respond within timeout after restart; check 'journalctl -u voxl-px4'."
fi

# Persist the domain param to flash too (boot-time line already set it in order).
if [ -n "$PARAM_CMD" ] && [ -n "$DOMAIN_PARAM" ]; then
    $PARAM_CMD set "$DOMAIN_PARAM" "$DOMAIN_ID" >/dev/null 2>&1 || true
    $PARAM_CMD save >/dev/null 2>&1 || true
fi

STATUS_CMD="px4-$MODULE"
echo
log "Verification ($STATUS_CMD status):"
if command -v "$STATUS_CMD" >/dev/null 2>&1; then
    if "$STATUS_CMD" status 2>&1 | tee /tmp/.svg_xrce_status | sed 's/^/        /'; then :; fi
    if grep -q 'connected' /tmp/.svg_xrce_status 2>/dev/null \
       && grep -q "Agent IP: *$HOST_IP" /tmp/.svg_xrce_status 2>/dev/null; then
        log "OK: client connected to $HOST_IP."
    else
        warn "Client not yet 'connected' to $HOST_IP."
        warn "Make sure the ground-PC agent is running:  MicroXRCEAgent udp4 -p $PORT -v4"
        warn "and that UDP $PORT is reachable from this drone (ping $HOST_IP)."
    fi
    rm -f /tmp/.svg_xrce_status
else
    warn "$STATUS_CMD not found; check status manually."
fi

echo
log "Done. Next, ON THE GROUND PC (ROS_DOMAIN_ID=$DOMAIN_ID):"
echo "        MicroXRCEAgent udp4 -p $PORT -v4        # if not already running"
echo "        ros2 topic list | grep $ROBOT_NAME/fmu  # /$ROBOT_NAME/fmu/... appears HERE"
echo "        ros2 launch svg_ground_control real_interfaces.launch.py drones:=$ROBOT_NAME"
