#!/bin/bash
# voxl_push_led.sh — install/update the onboard LED daemon on a drone over Wi-Fi (scp + ssh).
#
#   usage:  scripts/voxl_push_led.sh <robot_name> <drone_ip> [ground_pc_ip] [num_leds]
#   e.g.    scripts/voxl_push_led.sh drone_2 192.168.50.12
#
# Copies svg_led_daemon.py + voxl_setup_led.sh to /usr/bin on the VOXL and runs the
# installer there (root password on a stock VOXL2: oelinux123; use ssh-copy-id once to
# skip the prompts). Extra daemon flags pass through: LED_EXTRA_ARGS="--rgb" ...
# USB alternative:  adb push scripts/svg_led_daemon.py scripts/voxl_setup_led.sh /usr/bin/
set -eu
NAME="${1:?usage: $0 <robot_name> <drone_ip> [ground_pc_ip] [num_leds]}"
IP="${2:?usage: $0 <robot_name> <drone_ip> [ground_pc_ip] [num_leds]}"
GROUND="${3:-}"; NLEDS="${4:-}"
HERE="$(cd "$(dirname "$0")" && pwd)"
SSH_OPTS="-o StrictHostKeyChecking=accept-new -o ConnectTimeout=8"

echo "[push-led] $NAME @ $IP"
scp $SSH_OPTS "$HERE/svg_led_daemon.py" "$HERE/voxl_setup_led.sh" "root@$IP:/usr/bin/"
ssh $SSH_OPTS "root@$IP" "chmod +x /usr/bin/voxl_setup_led.sh /usr/bin/svg_led_daemon.py && \
    LED_EXTRA_ARGS='${LED_EXTRA_ARGS:-}' voxl_setup_led.sh $NAME $GROUND $NLEDS && \
    sleep 12 && journalctl -u svg-led -n 4 --no-pager"
echo "[push-led] done — strip should be steady green; expect 'PX4 ESC LED bits muted' above."
