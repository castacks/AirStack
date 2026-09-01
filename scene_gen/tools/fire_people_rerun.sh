#!/usr/bin/env bash
# fire_people_rerun.sh — the ONE COMMAND to re-solve `fire_people` once the
# FINAL manifest and the FINAL bake sidecars land.
#
#     scene_gen/tools/fire_people_rerun.sh MANIFEST DUMP SIDECAR_DIR
#
# Runs `tools/fire_people_dry_run.py` against those three inputs, writes
# `_plans/fire_people_final.json` + `.png` (the exact pair `FC_PEOPLE_JSON`
# and a reviewer's eye both want), prints the whole report — the manifest
# <-> dump match, the sidecar-completeness table, the census, the converter
# preview, the rule-check table — and exits NON-ZERO if anything in that
# report says the run cannot be trusted, so the lead can run this BLIND and
# read the exit code rather than the pane.
#
# HOST-SIDE ONLY. No Isaac, no docker, no Nucleus — `fire_people_dry_run.py`
# is `python3 -m pxr/usd-core` end to end, and this script is a thin
# wrapper: it does not build anything the tool itself does not already do.
#
# WHAT "TRUSTED" MEANS HERE, so a green run is worth something:
#   - every manifest record is re-verified against the dump BY GEOMETRY,
#     not just by `i` (`_manifest_matches_dump`) — a record that used to
#     name a building that has since moved or resized is SKIPPED and
#     counted, never placed against geometry it no longer owns;
#   - a manifest that skips down to ZERO surviving buildings FAILS the run
#     rather than reporting a vacuous PASS on 0 records checked;
#   - every rule `check_rules` claims (footprint keepouts, standoffs, group
#     sizes, window/roof eligibility, burial bands, the drone ceiling, the
#     `max_wall_dist_m` cap that is ALSO the guard under the launcher's own
#     `FC_PEOPLE_MAX_DIST_M` cull) must hold.
#
# A missing SIDECAR_DIR is not a failure — `fire_people.load_sidecars`
# degrades to an empty sidecar set and every 3-D class falls back to its
# documented "derived"/"estimated" path, flagged `needs_bench`. Pass "" (or
# any nonexistent path) to run without sidecars on purpose.
set -u

usage() {
  echo "usage: $(basename "$0") MANIFEST DUMP SIDECAR_DIR [-- extra dry-run args]" >&2
  echo "  MANIFEST     the fire manifest (tools/fire_city_dry_run.py output," >&2
  echo "               e.g. _plans/fire_city_<seed>.json)" >&2
  echo "  DUMP         the FC city placements dump (fc_dump_*.json)" >&2
  echo "  SIDECAR_DIR  directory of fire_bake .json sidecars; \"\" if none yet" >&2
  exit 2
}

if [ "$#" -lt 3 ]; then
  usage
fi

MANIFEST="$1"; DUMP="$2"; SIDECAR_DIR="$3"; shift 3

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCENE_GEN_DIR="$(dirname "$SCRIPT_DIR")"
OUT_JSON="$SCENE_GEN_DIR/_plans/fire_people_final.json"
OUT_PNG="$SCENE_GEN_DIR/_plans/fire_people_final.png"

for f in "$MANIFEST" "$DUMP"; do
  if [ ! -f "$f" ]; then
    echo "fire_people_rerun.sh: no file at '$f'" >&2
    exit 2
  fi
done
if [ -n "$SIDECAR_DIR" ] && [ ! -d "$SIDECAR_DIR" ]; then
  echo "fire_people_rerun.sh: WARNING — no directory at '$SIDECAR_DIR';" \
       "running with zero sidecars (every 3-D class falls back to its" \
       "documented default and carries needs_bench)." >&2
fi

echo "fire_people_rerun.sh: manifest=$MANIFEST"
echo "fire_people_rerun.sh: dump=$DUMP"
echo "fire_people_rerun.sh: sidecar_dir=${SIDECAR_DIR:-<none>}"
echo "fire_people_rerun.sh: writing $OUT_JSON / $OUT_PNG"
echo ""

python3 "$SCRIPT_DIR/fire_people_dry_run.py" \
  --manifest "$MANIFEST" \
  --dump "$DUMP" \
  --sidecar-dir "$SIDECAR_DIR" \
  --json "$OUT_JSON" \
  --out "$OUT_PNG" \
  "$@"
STATUS=$?

echo ""
if [ "$STATUS" -eq 0 ]; then
  echo "fire_people_rerun.sh: PASS — $OUT_JSON is ready for FC_PEOPLE_JSON."
else
  echo "fire_people_rerun.sh: FAIL (exit $STATUS) — see the RULE CHECKS /" \
       "MANIFEST <-> DUMP MATCH tables above. Do NOT point FC_PEOPLE_JSON" \
       "at $OUT_JSON until this is green." >&2
fi
exit "$STATUS"
