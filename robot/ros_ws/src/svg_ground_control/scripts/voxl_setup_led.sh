#!/bin/bash
# voxl_setup_led.sh — install the SVG onboard LED daemon as a systemd service ON THE VOXL2.
#
#   usage (as root, in the adb shell):
#     voxl_setup_led.sh <robot_name> [ground_pc_ip] [num_leds=11]
#     e.g.  voxl_setup_led.sh drone_1              # ground IP read from /usr/bin/voxl-px4-start
#           voxl_setup_led.sh drone_2 192.168.50.6 11
#
#   extra daemon flags (e.g. an RGB strip, other brightness):
#     LED_EXTRA_ARGS="--rgb --brightness 60" voxl_setup_led.sh drone_1
#
# Getting the files onto the drone (from the ground PC, package dir):
#     adb push scripts/svg_led_daemon.py scripts/voxl_setup_led.sh /usr/bin/
#
# What it does (idempotent — re-run to change name/IP/args):
#   1. copies svg_led_daemon.py to /usr/bin (looked up next to this script, then /usr/bin)
#   2. writes /etc/systemd/system/svg-led.service (After=voxl-px4, Restart=always)
#   3. daemon-reload + enable + RESTART (restart, not `enable --now`: a running unit
#      would otherwise keep the old arguments — same lesson as the microdds watchdog)
# The strip should show the default color (green) within a few seconds of voxl-px4
# being up. See experiment.md B1(d).
set -u

ROBOT_NAME="${1:-}"
GROUND_IP="${2:-}"
NUM_LEDS="${3:-11}"
EXTRA_ARGS="${LED_EXTRA_ARGS:-}"

DAEMON_NAME="svg_led_daemon.py"
DAEMON_DST="/usr/bin/$DAEMON_NAME"
SVC="svg-led"
UNIT="/etc/systemd/system/${SVC}.service"
PX4_START="/usr/bin/voxl-px4-start"
SINK="/run/mpa/modal_io_bridge"

log()  { printf '\033[1;32m[led-setup]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[warn     ]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[error    ]\033[0m %s\n' "$*" >&2; exit 1; }

[ -n "$ROBOT_NAME" ] || die "usage: $0 <robot_name> [ground_pc_ip] [num_leds]"
[ "$(id -u)" -eq 0 ] || die "Must run as root (you are $(id -un))."
echo "$ROBOT_NAME" | grep -qE '^[a-zA-Z][a-zA-Z0-9_]*$' \
    || die "robot_name '$ROBOT_NAME' is not a valid name."
echo "$NUM_LEDS" | grep -qE '^[0-9]+$' || die "num_leds '$NUM_LEDS' must be an integer."

PY="$(command -v python3 || true)"
[ -n "$PY" ] || die "python3 not found on this VOXL — the daemon needs it (stdlib only)."

# ----- ground IP: given, else the -h flag of the uXRCE client line ------------
if [ -z "$GROUND_IP" ]; then
    GROUND_IP="$(grep -oE -- '-h[[:space:]]+([0-9]{1,3}\.){3}[0-9]{1,3}' "$PX4_START" 2>/dev/null \
                 | head -n1 | awk '{print $2}')"
    [ -n "$GROUND_IP" ] \
        || die "No ground IP given and none found in $PX4_START. Run voxl_setup_real_drone.sh first, or pass the IP."
    log "ground IP $GROUND_IP (from $PX4_START)"
fi
echo "$GROUND_IP" | grep -qE '^([0-9]{1,3}\.){3}[0-9]{1,3}$' \
    || die "ground_pc_ip '$GROUND_IP' does not look like an IPv4 address."

# ----- daemon file ------------------------------------------------------------
SRC=""
for cand in "$(dirname "$0")/$DAEMON_NAME" "/usr/bin/$DAEMON_NAME" "/data/$DAEMON_NAME" "/home/root/$DAEMON_NAME"; do
    if [ -f "$cand" ]; then SRC="$cand"; break; fi
done
[ -n "$SRC" ] || die "$DAEMON_NAME not found next to this script, in /usr/bin, /data or /home/root. adb push it first."
if [ "$SRC" != "$DAEMON_DST" ]; then
    cp -f "$SRC" "$DAEMON_DST" || die "cannot copy $SRC -> $DAEMON_DST (try: mount -o remount,rw /)"
fi
chmod +x "$DAEMON_DST"
"$PY" -m py_compile "$DAEMON_DST" || die "$DAEMON_DST does not compile with $PY"
log "daemon at $DAEMON_DST"

# ----- LED path: FIFO, or MAVLink tunnel through voxl-mavlink-server ----------
# voxl-px4 builds without the modal_io_bridge FIFO (Starling 2 Max SDK) take the
# same packet as a MAVLink TUNNEL message. The tunnel's payload-type enum is a
# ModalAI dialect value — read it from the installed mavlink headers if present
# so the daemon does not rely on its built-in default.
TUNNEL_ARGS=""
if [ ! -e "$SINK" ]; then
    HDR_TYPE="$(grep -rhoE 'MAV_TUNNEL_PAYLOAD_TYPE_MODALAI_ESC_UART_PASSTHRU *= *[0-9]+' \
                /usr/include /usr/local/include /opt 2>/dev/null | head -n1 | grep -oE '[0-9]+$')"
    HDR_CRC="$(grep -rhoE 'MAVLINK_MSG_ID_TUNNEL_CRC +[0-9]+' \
               /usr/include /usr/local/include /opt 2>/dev/null | head -n1 | grep -oE '[0-9]+$')"
    [ -n "$HDR_TYPE" ] && TUNNEL_ARGS="$TUNNEL_ARGS --tunnel-payload-type $HDR_TYPE" \
        && log "mavlink headers: MODALAI_ESC_UART_PASSTHRU=$HDR_TYPE"
    [ -n "$HDR_CRC" ]  && TUNNEL_ARGS="$TUNNEL_ARGS --tunnel-crc-extra $HDR_CRC" \
        && log "mavlink headers: TUNNEL crc_extra=$HDR_CRC"
    [ -n "$HDR_TYPE" ] || warn "mavlink headers not found — using the daemon's built-in tunnel payload type (201). If the LEDs stay dark, check this value against your SDK's mavlink dialect."
fi

# ----- systemd unit -----------------------------------------------------------
cat > "$UNIT" <<UEOF
[Unit]
Description=SVG onboard LED daemon ($ROBOT_NAME -> ground $GROUND_IP, $NUM_LEDS leds)
After=voxl-px4.service network-online.target
Wants=voxl-px4.service

[Service]
ExecStart=$PY $DAEMON_DST --name $ROBOT_NAME --ground-ip $GROUND_IP --num-leds $NUM_LEDS$TUNNEL_ARGS $EXTRA_ARGS
Restart=always
RestartSec=3
KillSignal=SIGTERM
TimeoutStopSec=5

[Install]
WantedBy=multi-user.target
UEOF
systemctl daemon-reload
systemctl enable "$SVC" >/dev/null 2>&1 || true
systemctl restart "$SVC" || die "systemctl restart $SVC failed"
log "installed + (re)started $SVC"

sleep 2
LIB="$(ls /usr/lib64/libmodalio.so /usr/lib/libmodalio.so 2>/dev/null | head -n1)"
if [ -e "$SINK" ]; then
    log "LED sink $SINK present."
else
    if [ -n "$LIB" ]; then
        log "$SINK absent; daemon uses $LIB (its own MAVLink-tunnel fallback)."
    else
        log "$SINK absent; daemon sends the packet as a MAVLink TUNNEL via voxl-mavlink-server."
    fi
    systemctl is-active --quiet voxl-mavlink-server || warn "voxl-mavlink-server is NOT active — the tunnel path needs it: systemctl start voxl-mavlink-server"
    [ -e /run/mpa/mavlink_onboard/control ] || warn "/run/mpa/mavlink_onboard/control missing — voxl-mavlink-server not (yet) up"
fi
systemctl --no-pager --lines=8 status "$SVC" 2>/dev/null | sed 's/^/    /'
log "verify: strip is $( [ -n "$EXTRA_ARGS" ] && echo "at its default color" || echo green ); logs: journalctl -u $SVC -f"
