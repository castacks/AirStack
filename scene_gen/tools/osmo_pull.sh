#!/usr/bin/env bash
# osmo_pull.sh — bring rendered PNGs (or anything else) back from the pod.
#
# The counterpart to osmo_sync.sh. Isaac writes viewport captures to SNAP_DIR,
# which MUST sit under a path the container has mounted from the pod — the
# log mount is the one that always exists:
#
#     container  /isaac-sim/.nvidia-omniverse/logs/<name>
#     pod        /root/docker/isaac-sim/logs/<name>
#
# so `osmo_pull.sh <name> <local-dir>` fetches that directory. Pass an
# absolute pod path as <name> to fetch something else.
set -euo pipefail

PORT="${OSMO_SSH_LOCAL_PORT:-2200}"
REMOTE="${OSMO_SSH_REMOTE:-root@localhost}"
POD_LOGS="${OSMO_POD_LOGS:-/root/docker/isaac-sim/logs}"

SSH_OPTS=(-p "$PORT" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null
          -o LogLevel=ERROR -o ConnectTimeout=15)

[ $# -ge 1 ] || { echo "usage: $0 <snap-name|/abs/pod/path> [local-dir]" >&2; exit 2; }

src="$1"
case "$src" in /*) ;; *) src="${POD_LOGS}/${src}" ;; esac
dest="${2:-$HOME/hurricane_previews}"

mkdir -p "$dest"
rsync -az --info=stats1,progress2 -e "ssh ${SSH_OPTS[*]}" \
      "${REMOTE}:${src}/" "${dest}/"
echo
echo "pulled -> ${dest}"
ls -lh "$dest" | tail -20
