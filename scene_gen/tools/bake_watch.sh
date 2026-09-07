#!/usr/bin/env bash
# Keep the gallery and the report current while `bake_overnight.sh` runs.
#
# Both consumers are incremental — `render_archetypes.py` skips any rung that
# already has a PNG, `bake_report.py` re-reads the append-only trace — so this
# is just the two of them on a timer. The point is that a bake stopped at 4 a.m.
# by the disk floor is still readable at 8 a.m. without anyone running anything.
#
# THE PERIOD IS NOT SHORT ON PURPOSE. Cycles and PhysX are on the same GPU, and
# rendering a 1.4 GB archetype is not free in RAM either; a tight loop would be
# taking the night away from the thing it is documenting.
#
#     scene_gen/tools/bake_watch.sh &          # from AirStack/
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"      # AirStack/
LIB="${WATCH_LIB:-$HERE/scene_gen/assets/archetypes_urban_v2/earthquake}"
GAL="${WATCH_GAL:-$HERE/scene_gen/galleries/archetypes_urban_v2}"
LOG="${WATCH_LOG:-$HERE/scene_gen/_bakelab/watch.log}"
PERIOD="${WATCH_PERIOD:-1200}"
BPY="${BPY_PYTHON:-/home/myan2/coasei/scenegen/.venv/bin/python}"

mkdir -p "$GAL" "$(dirname "$LOG")"
while true; do
    if [ -f "$LIB/manifest.json" ]; then
        echo "=== pass $(date -Is)" >>"$LOG"
        # SIDECARS FIRST. `render_archetypes.py` renders `<name>.local.usda`
        # when one exists, and without it Blender drops every `omniverse://`
        # texture and the whole gallery comes out grey. Both steps skip what
        # they have already done, so a pass over an unchanged library is cheap.
        ( set -a; . "$HERE/.env.host"; set +a
          "$HERE/.venv/bin/python" \
            "$HERE/scene_gen/tools/localize_archetype_textures.py" "$LIB" \
            </dev/null >>"$LOG" 2>&1 )
        "$BPY" "$HERE/scene_gen/tools/render_archetypes.py" "$LIB" -o "$GAL" \
            </dev/null >>"$LOG" 2>&1
        python3 "$HERE/scene_gen/tools/bake_report.py" --library "$LIB" \
            </dev/null >>"$LOG" 2>&1
    fi
    sleep "$PERIOD"
done
