#!/usr/bin/env bash
# osmo_isaac.sh — drive the Isaac Sim tmux pane inside the OSMO dev pod.
#
# The pod runs the AirStack stack on an INNER dockerd, so every command is
# two hops: ssh to the pod over the `airstack osmo:ide` port-forward, then
# `docker exec` into the sim container. The container is named
# `isaac-sim-livestream` (not `isaac-sim`) whenever the workflow sets
# ISAAC_SIM_LIVESTREAM=true, which airstack-dev.yaml does.
#
# `docker logs` is EMPTY for this container — the process lives in a tmux
# pane. `pane` is how you actually read what a launcher printed. Same rule
# as the run-isaac-sim-launcher skill states for a local container.
#
#   osmo_isaac.sh pane [n]            tail the pane (default 40 lines)
#   osmo_isaac.sh stop                Ctrl-C whatever is running in it
#   osmo_isaac.sh run <script> [K=V…] relaunch with env
#   osmo_isaac.sh exec  <shell cmd>   run a bash command in the container
#   osmo_isaac.sh ssh   <shell cmd>   run a bash command on the POD itself
set -euo pipefail

PORT="${OSMO_SSH_LOCAL_PORT:-2200}"
REMOTE="${OSMO_SSH_REMOTE:-root@localhost}"
# THE SIM CONTAINER'S NAME IS NOT FIXED, so ask the pod rather than assume.
#
# It is `isaac-sim-livestream` under COMPOSE_PROFILES=...,isaac-sim-livestream
# and plain `isaac-sim` under ...,isaac-sim — and which one you get is decided
# by the COMMITTED `.env`, which any session can change. Hardcoding the
# livestream name cost a bake here: `.env` moved to the plain profile between
# one pod and the next and every `docker exec` came back "No such container".
#
# Resolve once per invocation, prefer an exact match, fall back to whatever
# running container has "isaac-sim" in its name. `OSMO_SIM_CTNR` still wins.
_resolve_ctnr() {
    [ -n "${OSMO_SIM_CTNR:-}" ] && { echo "$OSMO_SIM_CTNR"; return; }
    local names
    names=$(ssh "${SSH_OPTS[@]}" "$REMOTE" \
            'docker ps --format "{{.Names}}"' 2>/dev/null)
    for want in isaac-sim-livestream isaac-sim; do
        echo "$names" | grep -qx "$want" && { echo "$want"; return; }
    done
    echo "$names" | grep -m1 isaac-sim || echo isaac-sim
}
LAUNCH_DIR=/isaac-sim/AirStack/simulation/isaac-sim/launch_scripts
EXTS='~/.local/share/ov/data/documents/Kit/shared/exts'

SSH_OPTS=(-p "$PORT" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null
          -o LogLevel=ERROR -o ConnectTimeout=15)
pod() { ssh "${SSH_OPTS[@]}" "$REMOTE" "$@"; }
CTNR="$(_resolve_ctnr)"
ctnr() { pod "docker exec ${CTNR} bash -c $(printf '%q' "$1")"; }

cmd="${1:-pane}"; shift || true

case "$cmd" in
  pane)
    # -J joins wrapped lines; without `resize-window` the pane is 80 cols and
    # every long line arrives pre-broken, which makes grepping useless.
    pod "docker exec ${CTNR} tmux resize-window -t isaac -x 220 -y 50" 2>/dev/null || true
    pod "docker exec ${CTNR} tmux capture-pane -p -J -t isaac -S -${1:-3000}" \
      | grep -v '^$' | tail -"${1:-40}"
    ;;
  stop)
    pod "docker exec ${CTNR} tmux send-keys -t isaac C-c" || true
    sleep 8
    pod "docker exec ${CTNR} bash -c 'pgrep -af isaac-sim/python.sh || echo NO_SIM_PROC'" || true
    ;;
  run)
    script="$1"; shift
    env_str=""
    for kv in "$@"; do env_str+="${kv} "; done
    # `clear` first so the next `pane` read starts at this run, not the last.
    pod "docker exec ${CTNR} tmux clear-history -t isaac" || true
    pod "docker exec ${CTNR} tmux send-keys -t isaac $(printf '%q' \
      "clear; ${env_str}PYTHONPATH=\"\$ISAAC_SIM_PYTHONPATH\" /isaac-sim/python.sh ${LAUNCH_DIR}/${script} --ext-folder ${EXTS}") ENTER"
    echo "launched ${script} with: ${env_str}"
    echo "read it with:  $0 pane 60"
    ;;
  exec) ctnr "$*" ;;
  ssh)  pod  "$*" ;;
  *) echo "usage: $0 {pane|stop|run|exec|ssh} …" >&2; exit 2 ;;
esac
