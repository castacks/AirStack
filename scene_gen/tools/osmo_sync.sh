#!/usr/bin/env bash
# osmo_sync.sh — push named working-tree files to the OSMO dev pod.
#
# WHY THIS EXISTS AND NOT `git push`. The pod's entrypoint clones AirStack
# fresh from GitHub, so the only code it ever sees is what is COMMITTED and
# PUSHED. That is normally the right contract — but it forces a commit for
# every iteration, and when several agents are editing this working tree at
# once, committing to share one file to the pod drags in everyone else's
# in-flight edits and creates a branch that diverges from what any of them
# is testing.
#
# So: sync an EXPLICIT FILE LIST over the ssh port-forward that `airstack
# osmo:ide` already establishes. Only the files named on the command line
# move. Nothing is committed, nothing is pushed, and the other agents'
# uncommitted work stays out of the pod.
#
#   scene_gen/tools/osmo_sync.sh scene_gen/disaster/hurricane.py ...
#   scene_gen/tools/osmo_sync.sh --dir scene_gen/assets/archetypes_hurricane
#
# Paths are repo-relative and land at the same repo-relative path under
# $REMOTE_ROOT on the pod.
#
# Requires: `airstack osmo:ide --no-open` running in another shell (or any
# other listener on $PORT forwarding to the pod's sshd).

set -euo pipefail

PORT="${OSMO_SSH_LOCAL_PORT:-2200}"
REMOTE="${OSMO_SSH_REMOTE:-root@localhost}"
REMOTE_ROOT="${OSMO_REMOTE_ROOT:-/root/AirStack}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

SSH_OPTS=(-p "$PORT" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null
          -o LogLevel=ERROR -o ConnectTimeout=15)

if ! nc -z localhost "$PORT" 2>/dev/null; then
    echo "ERROR: nothing listening on localhost:${PORT}." >&2
    echo "  Start the tunnel first:  airstack osmo:ide --no-open" >&2
    exit 1
fi

[ $# -gt 0 ] || { echo "usage: $0 [--dir] <repo-relative-path>..." >&2; exit 2; }

DIR_MODE=false
if [ "$1" = "--dir" ]; then DIR_MODE=true; shift; fi

cd "$REPO_ROOT"

for rel in "$@"; do
    [ -e "$rel" ] || { echo "ERROR: no such path: $rel" >&2; exit 1; }
    remote_dir="${REMOTE_ROOT}/$(dirname "$rel")"
    ssh "${SSH_OPTS[@]}" "$REMOTE" "mkdir -p '${remote_dir}'"
    if [ "$DIR_MODE" = true ] || [ -d "$rel" ]; then
        # Trailing slash on the source: copy the CONTENTS into the
        # like-named remote dir rather than nesting it one deeper.
        rsync -az --info=stats1 -e "ssh ${SSH_OPTS[*]}" \
              "${rel}/" "${REMOTE}:${REMOTE_ROOT}/${rel}/"
    else
        rsync -az --info=stats1 -e "ssh ${SSH_OPTS[*]}" \
              "$rel" "${REMOTE}:${REMOTE_ROOT}/${rel}"
    fi
    echo "  -> ${REMOTE_ROOT}/${rel}"
done

echo "synced $# path(s) to ${REMOTE}:${REMOTE_ROOT}"
