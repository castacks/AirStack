#!/usr/bin/env bash
# Stage A earthquake bake over the urban_v2 pack, newest assets first.
#
# ONE bake_cli RUN PER SOURCE FAMILY, in the order the families were added to
# the pack. `bake_cli` takes the plan in pack order, and an unattended bake is
# always going to be cut short by something — the disk floor, the morning — so
# the order decides WHICH assets have a complete set of rungs when it stops.
# Front-loading the newly-added assets is what makes a partial bake useful.
#
# Every run carries --skip-existing, so this script is also its own resume:
# re-running it picks up wherever the last one stopped, at the cost of one Kit
# boot (~40 s) per family.
#
#     scene_gen/tools/bake_overnight.sh          # from AirStack/
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"   # AirStack/
OUT="${BAKE_OUT:-$HERE/scene_gen/assets/archetypes_urban_v2}"
# NOT `GROUPS`: that is a bash special variable holding the caller's group
# ids, so assigning it is ignored and "$GROUPS" expands to a gid — which
# read as `1006: No such file or directory` on the redirect.
GROUP_FILE="${BAKE_GROUPS:-$HERE/scene_gen/_bakelab/groups.txt}"
LOG="${BAKE_LOG:-$HERE/scene_gen/_bakelab/bake.log}"
CONFIG="${BAKE_CONFIG:-urban_v2}"

set -a; . "$HERE/.env.host"; set +a
export BAKE_DISK_FLOOR_GB="${BAKE_DISK_FLOOR_GB:-12}"

mkdir -p "$OUT" "$(dirname "$LOG")"
echo "=== bake_overnight start $(date -Is) out=$OUT floor=${BAKE_DISK_FLOOR_GB}GB" | tee -a "$LOG"

# THE GROUP LIST IS READ UP FRONT, not streamed by `done < "$GROUP_FILE"`.
# Kit is a long-running child that inherits this loop's stdin, and a child that
# reads it consumes the group list out from under the loop — so the list is
# pulled into an array first and every child gets </dev/null.
mapfile -t GROUP_LINES < "$GROUP_FILE"

for line in "${GROUP_LINES[@]}"; do
    name="${line%%$'\t'*}"
    types="${line#*$'\t'}"
    [ -z "$types" ] && continue
    [ "$name" = "$types" ] && continue
    free=$(df -B1 --output=avail "$OUT" | tail -1 | tr -d ' ')
    floor=$(( ${BAKE_DISK_FLOOR_GB%%.*} * 1000000000 ))
    if [ "${free:-0}" -lt "$floor" ]; then
        echo "=== STOP before '$name': $((free/1000000000)) GB free" | tee -a "$LOG"
        break
    fi
    echo "=== group '$name' $(date -Is)" | tee -a "$LOG"
    "$HERE/.venv/bin/python" "$HERE/scene_gen/archetypes/bake_cli.py" \
        --config "$CONFIG" --disaster earthquake \
        --only "$types" --skip-existing --out "$OUT" \
        </dev/null >>"$LOG" 2>&1
    echo "=== group '$name' exit=$? $(date -Is)" | tee -a "$LOG"
done

echo "=== bake_overnight done $(date -Is)" | tee -a "$LOG"
