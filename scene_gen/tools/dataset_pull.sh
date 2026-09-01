#!/usr/bin/env bash
# dataset_pull.sh — bring finished `final_disaster_dataset/` cells back to
# ~/SEI-COA/final_disaster_dataset on THIS machine.
#
# TWO INDEPENDENT SOURCES, pick one (or run both — they are idempotent and
# additive, and a cell frozen on a pod can be re-pulled from Nucleus later
# to pick up a re-upload without re-touching the pod):
#
#   --from-pod       tar over the OSMO ssh tunnel (`osmo_sync.sh`/
#                    `osmo_pull.sh`'s own pattern, generalised) straight
#                    from the pod's OWN `final_disaster_dataset` mount —
#                    `/root/SEI-COA/final_disaster_dataset` by convention
#                    (`.env`'s `FINAL_DATASET_DIR` default is
#                    `${HOME}/SEI-COA/final_disaster_dataset`, and HOME is
#                    /root on every pod this repo's tooling reaches over
#                    ssh). TAR, not rsync: `osmo_pull.sh` already covers the
#                    rsync case for the LOGS mount, and a cell directory can
#                    hold thousands of small `snaps/*.png` files where one
#                    `tar -C ... -cf - . | ssh ... tar -xf -` pipeline beats
#                    rsync's per-file round trips over the same tunnel, and
#                    needs nothing on the remote side beyond `tar` and `ssh`
#                    -- no rsync install to depend on.
#   --from-nucleus   `dataset_upload.py --download` run inside a container
#                    that can reach Nucleus (default: the LOCAL `isaac-sim`
#                    container, if one is running — its own
#                    `/isaac-sim/final_disaster_dataset` is ALREADY the same
#                    bind mount this script's `--dest` defaults to, so a
#                    Nucleus pull through it lands on the host with no
#                    extra copy step at all).
#
# THE USER WANTS FULL LOCAL COPIES INCLUDING snaps/. Nucleus does not carry
# snaps/ (571 MB/cell of human-review captures nothing at run time reads —
# see `dataset_upload.py`'s own module docstring), so --from-nucleus alone
# will never bring them down; --from-pod does, because the pod's own
# `final_disaster_dataset` mount has them (they are written locally by the
# freeze launcher and never pruned before landing there). Run --from-pod at
# least once per cell if snaps/ matters.
#
# USAGE
#   dataset_pull.sh --from-pod    [SUBPATH]   # e.g. Fire/Urban/level_1/1
#   dataset_pull.sh --from-nucleus [SUBPATH]
#   dataset_pull.sh --from-pod --from-nucleus [SUBPATH]
#
#   SUBPATH restricts the pull to one cell/locale/disaster subtree, same
#   convention as `dataset_upload.py --only`. Omit it to pull everything.
#
# ENV (all have the same names/defaults the sibling osmo_*.sh tools use,
# where an equivalent exists):
#   OSMO_SSH_LOCAL_PORT   ssh tunnel local port (default 2200; the
#                         `build-scenes-on-osmo` skill's live pods use 2204
#                         — export it before calling this script, exactly
#                         as every other osmo_*.sh tool expects)
#   OSMO_SSH_REMOTE       ssh target (default root@localhost)
#   POD_DATASET_DIR       the pod-side final_disaster_dataset root
#                         (default /root/SEI-COA/final_disaster_dataset)
#   DEST                  local destination root
#                         (default ${FINAL_DATASET_DIR:-$HOME/SEI-COA/final_disaster_dataset},
#                         the exact host path `.env`/docker-compose already
#                         bind-mount into every isaac-sim container)
#   NUCLEUS_CONTAINER     container to run the omni.client download in
#                         (default isaac-sim)
set -euo pipefail

PORT="${OSMO_SSH_LOCAL_PORT:-2200}"
REMOTE="${OSMO_SSH_REMOTE:-root@localhost}"
POD_DATASET_DIR="${POD_DATASET_DIR:-/root/SEI-COA/final_disaster_dataset}"
DEST="${DEST:-${FINAL_DATASET_DIR:-$HOME/SEI-COA/final_disaster_dataset}}"
NUCLEUS_CONTAINER="${NUCLEUS_CONTAINER:-isaac-sim}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

SSH_OPTS=(-p "$PORT" -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null
          -o LogLevel=ERROR -o ConnectTimeout=15)

FROM_POD=false
FROM_NUCLEUS=false
SUBPATH=""
for arg in "$@"; do
    case "$arg" in
        --from-pod) FROM_POD=true ;;
        --from-nucleus) FROM_NUCLEUS=true ;;
        --*) echo "ERROR: unknown flag $arg" >&2; exit 2 ;;
        *) SUBPATH="$arg" ;;
    esac
done
if ! $FROM_POD && ! $FROM_NUCLEUS; then
    echo "usage: $0 [--from-pod] [--from-nucleus] [SUBPATH]" >&2
    echo "  (pass at least one of --from-pod / --from-nucleus)" >&2
    exit 2
fi

mkdir -p "$DEST"
echo "dataset_pull: destination $DEST"
[ -n "$SUBPATH" ] && echo "dataset_pull: subpath $SUBPATH"

# ---------------------------------------------------------------------------
# --from-pod: tar over the ssh tunnel
# ---------------------------------------------------------------------------
if $FROM_POD; then
    if ! nc -z localhost "$PORT" 2>/dev/null; then
        echo "ERROR: nothing listening on localhost:${PORT}." >&2
        echo "  Start the tunnel first (see build-scenes-on-osmo / DEV183_ACCESS.md):" >&2
        echo "    nohup osmo workflow port-forward <workflow> workspace --port ${PORT}:22 \\" >&2
        echo "        --connect-timeout 86400 > /tmp/osmo_ide.log 2>&1 &" >&2
        exit 1
    fi
    src_dir="${POD_DATASET_DIR}"
    [ -n "$SUBPATH" ] && src_dir="${POD_DATASET_DIR}/${SUBPATH}"
    echo "dataset_pull: --from-pod  ${REMOTE}:${src_dir}  (port ${PORT})"
    # Verify the remote dir exists before starting a tar that would
    # otherwise fail silently mid-stream with an empty/garbled archive.
    if ! ssh "${SSH_OPTS[@]}" "$REMOTE" "test -d '${src_dir}'"; then
        echo "ERROR: no such directory on the pod: ${src_dir}" >&2
        echo "  (check POD_DATASET_DIR and SUBPATH)" >&2
        exit 1
    fi
    local_dir="$DEST"
    [ -n "$SUBPATH" ] && local_dir="${DEST}/${SUBPATH}"
    mkdir -p "$local_dir"
    # `-C <dir> .` on the SENDER so the archive's own paths are relative —
    # extracting with `-C <local_dir>` then lands the SAME subtree shape
    # locally with no path surgery on either side. `-z` keeps the pipe
    # small over a tunnel that may be a shared, rate-limited hop.
    ssh "${SSH_OPTS[@]}" "$REMOTE" \
        "tar -C '${src_dir}' -czf - ." \
        | tar -C "$local_dir" -xzf -
    echo "dataset_pull: --from-pod done -> ${local_dir}"
    du -sh "$local_dir" 2>/dev/null || true
fi

# ---------------------------------------------------------------------------
# --from-nucleus: dataset_upload.py --download, run inside a Kit container
# ---------------------------------------------------------------------------
if $FROM_NUCLEUS; then
    if ! docker ps --format '{{.Names}}' | grep -qx "$NUCLEUS_CONTAINER"; then
        echo "ERROR: container '$NUCLEUS_CONTAINER' is not running (docker ps)." >&2
        echo "  airstack up isaac-sim   # or set NUCLEUS_CONTAINER to the right name" >&2
        exit 1
    fi
    # Built as ONE string, not spliced from a bash ARRAY into a single-quoted
    # heredoc: `bash -c 'script' "${arr[@]}"` treats every array element past
    # the first as a POSITIONAL PARAMETER ($0, $1, ...) to the script, not as
    # more script text — `--only <subpath>` silently never reached
    # `dataset_upload.py`'s own argv that way (measured 2026-09-01: `docker
    # exec ... bash -c '...' "${only_arg[@]}"` with a 2-element array left
    # `--download` with no `--only` at all, argparse then choked on the
    # STRAY word as if it were `--only`'s value with nothing after it).
    remote_cmd='EXT=/isaac-sim/kit/extscore/omni.client.lib
        LIB=$(ls -d /isaac-sim/extscache/omni.client-*/bin | head -1)
        LD_LIBRARY_PATH="/isaac-sim/kit:$LIB:$LD_LIBRARY_PATH" PYTHONPATH="$EXT" \
        /isaac-sim/kit/python/bin/python3 -u \
        /isaac-sim/AirStack/scene_gen/tools/dataset_upload.py --download'
    if [ -n "$SUBPATH" ]; then
        remote_cmd="${remote_cmd} --only $(printf '%q' "$SUBPATH")"
    fi
    echo "dataset_pull: --from-nucleus via container '$NUCLEUS_CONTAINER'"
    docker exec "$NUCLEUS_CONTAINER" bash -c "$remote_cmd"
    # `/isaac-sim/final_disaster_dataset` in the container IS `$DEST` on the
    # host (the same bind mount `.env`'s FINAL_DATASET_DIR sets up) UNLESS
    # the caller overrode DEST to point somewhere else — flag that rather
    # than silently disagreeing with what was just printed above.
    if [ "$DEST" != "${FINAL_DATASET_DIR:-$HOME/SEI-COA/final_disaster_dataset}" ]; then
        echo "dataset_pull: NOTE — the Nucleus pull landed in the container's" >&2
        echo "  bind-mounted host path (\$FINAL_DATASET_DIR / ~/SEI-COA/final_disaster_dataset)," >&2
        echo "  which is NOT the DEST=${DEST} this run was given. Copy/symlink" >&2
        echo "  manually if you need it under a different local path." >&2
    else
        echo "dataset_pull: --from-nucleus done -> ${DEST} (via the container's bind mount)"
    fi
fi

echo "dataset_pull: complete."
