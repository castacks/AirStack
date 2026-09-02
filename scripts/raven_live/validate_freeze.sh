#!/usr/bin/env bash
# validate_freeze.sh — gate between Stage 1 (build + freeze) and Stage 2 (fly).
#
#   scripts/raven_live/validate_freeze.sh
#
# Two halves:
#
#  A. THE CELL ON DISK. Does FREEZE_OUT hold what `example_multi_drone_scene_
#     import.py` will need, in the shape `frozen_annotations.resolve_cell`
#     accepts? A bad answer here is the failure that costs a whole run: a
#     path that does not resolve reaches `AddReference` as a SILENT NO-OP —
#     the prim composes empty, nothing errors, and the first sign is a black
#     viewport with drones hovering over nothing.
#
#  B. THE TWO CASUALTIES. Re-pick them against the REAL people file and
#     regenerate SPAWN_CONFIGS (see spawn_from_real_people.py's header for
#     the deck_points caveat and the substitution rule).
#
# exit 0  cell is good AND the two casualties are where the runbook says
# exit 2  cell is good but a casualty had to be SUBSTITUTED — usable, but the
#         runbook's / mission file's hardcoded SPAWN_CONFIGS are now wrong
# exit 1  the cell is not flyable
set -uo pipefail

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$REPO"

# Container path -> host path. The repo is bind-mounted at /isaac-sim/AirStack.
FREEZE_OUT_C="${FREEZE_OUT:-/isaac-sim/AirStack/_test_freeze/raven_suburb_tornado_250}"
CELL="${CELL:-${REPO}/${FREEZE_OUT_C#/isaac-sim/AirStack/}}"
FREEZE_NAME="${FREEZE_NAME:-RavenSuburbTornado250}"
TOR_SEED="${TOR_SEED:-10}"
SCENE_CONFIG="${SCENE_CONFIG:-suburb_tornado_250}"
OUT_DIR="${OUT_DIR:-${REPO}/scripts/raven_live/out}"
UV="${UV:-uv}"

FAIL=0
ok()   { printf '  \033[32m✓\033[0m %s\n' "$*"; }
bad()  { printf '  \033[31m✗\033[0m %s\n' "$*"; FAIL=1; }
warn() { printf '  \033[33m!\033[0m %s\n' "$*"; }

echo "══════════════════════════════════════════════════════════════════════"
echo " A. the frozen cell"
echo "    container : ${FREEZE_OUT_C}"
echo "    host      : ${CELL}"
echo "══════════════════════════════════════════════════════════════════════"

if [ ! -d "$CELL" ]; then
  bad "the cell directory does not exist. Stage 1 either never ran or never
      reached the freeze block. Check:  scripts/raven_live/watch_stage1.sh"
  exit 1
fi
ok "cell directory exists"

# ── the .usd, and the resolve_cell contract ────────────────────────────────
# frozen_annotations.resolve_cell() on a DIRECTORY that is not shaped like
# `<Disaster>/<Locale>/level_<n>/<k>` (ours is not) falls through to
# "exactly one .usd in this folder". Two .usd files -> ValueError -> SystemExit
# at launch. One .usd and it can be named anything.
mapfile -t USDS < <(find "$CELL" -maxdepth 1 -name '*.usd' -o -maxdepth 1 -name '*.usdc' | sort)
case "${#USDS[@]}" in
  0) bad "no .usd at the top level of the cell. The export did not land.
      Read ${CELL}/freeze_report.json (the launcher writes the portability
      gate's own verify() result there even when the export was refused)." ;;
  1) sz=$(du -h "${USDS[0]}" | cut -f1)
     ok "exactly one .usd: $(basename "${USDS[0]}") (${sz}) — resolve_cell will find it"
     if [ "$(basename "${USDS[0]}" .usd)" != "$FREEZE_NAME" ]; then
       warn "…but it is not named '${FREEZE_NAME}.usd'. Harmless for
      resolve_cell (single-file fallback), worth knowing: FREEZE_NAME was
      probably not passed, and the launcher's auto-derivation falls back to
      the literal string 'scene'."
     fi ;;
  *) bad "${#USDS[@]} .usd files at the top level:
      $(printf '%s ' "${USDS[@]##*/}")
      resolve_cell raises 'holds N .usd files; name one' and the launcher
      exits. Delete the stray one, or point FROZEN_SCENE at the file rather
      than the directory." ;;
esac

# ── the ground-truth JSONs ─────────────────────────────────────────────────
check_json() {  # path, label, required(0/1)
  local p="$1" label="$2" req="$3"
  if [ ! -f "$p" ]; then
    if [ "$req" = "1" ]; then bad "missing ${label}: ${p}"; else warn "missing ${label} (optional): ${p}"; fi
    return
  fi
  if ! python3 -c "import json,sys;json.load(open(sys.argv[1]))" "$p" 2>/dev/null; then
    bad "${label} is not valid JSON: ${p}"; return
  fi
  ok "${label}  $(python3 -c "
import json,sys
d=json.load(open(sys.argv[1]))
if isinstance(d,dict):
    n=len(d.get('people') or d.get('records') or d.get('objects') or d)
    print('%d top-level entr(ies)' % n)
else:
    print('%d record(s)' % len(d))" "$p")"
}

check_json "${CELL}/GT_hints.json"     "GT_hints.json"     1
check_json "${CELL}/GT_people.json"    "GT_people.json"    1
check_json "${CELL}/humans_${TOR_SEED}.json" "humans_${TOR_SEED}.json (PEOPLE_JSON)" 0
check_json "${CELL}/freeze_report.json" "freeze_report.json" 1

# ── the portability verdict ────────────────────────────────────────────────
# suburb_tornado_launch_script.py calls freeze.export_scene(..., collect=False)
# with the DEFAULT enforce_portable=True, and — unlike freeze_dataset_launch_
# script.py — reads no FREEZE_WAIVE_VEGETATION / FREEZE_WAIVE_MIRRORED knob.
# So the gate here is UNWAIVABLE from this launcher: if it fires, nothing
# ships and freeze_report.json is the diagnosis, not a warning.
if [ -f "${CELL}/freeze_report.json" ]; then
  python3 - "$CELL/freeze_report.json" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
print("    portable_ok    : {0}".format(d.get("portable_ok")))
print("    ok             : {0}".format(d.get("ok")))
print("    prims/meshes   : {0} / {1}".format(d.get("prims"), d.get("meshes")))
print("    prototypes     : {0}".format(d.get("prototypes")))
print("    size_mb        : {0}".format(d.get("size_mb")))
print("    deps scan      : {0}".format(d.get("deps_scan_method")))
bl = d.get("build_local") or []
un = d.get("unresolved")
if bl:
    print("    build_local    : {0} path(s) — first 5:".format(len(bl)))
    for p in bl[:5]:
        print("        {0}".format(p))
if un:
    print("    UNRESOLVED     : {0} path(s) — first 5:".format(len(un)))
    for p in un[:5]:
        print("        {0}".format(p))
PY
fi

# ── snapshots, if Stage 1 took them ────────────────────────────────────────
SNAPS="${HOME}/docker/isaac-sim/logs/raven_t250"
if [ -d "$SNAPS" ]; then
  n=$(find "$SNAPS" -name '*.png' | wc -l)
  ok "review PNGs: ${n} in ${SNAPS}  (look at overview.png and p00_* first —
      p00 is the WORST-covered casualty in the scene)"
else
  warn "no review PNGs at ${SNAPS} (SNAP_DIR was empty, or the captures failed)"
fi

if [ "$FAIL" -ne 0 ]; then
  echo
  echo "══════════════════════════════════════════════════════════════════════"
  echo " ❌ THE CELL IS NOT FLYABLE. Fix Stage 1 before touching .env."
  echo "══════════════════════════════════════════════════════════════════════"
  exit 1
fi

# ── B. the casualties + SPAWN_CONFIGS ──────────────────────────────────────
echo
echo "══════════════════════════════════════════════════════════════════════"
echo " B. re-validate the two casualties against the REAL people file"
echo "══════════════════════════════════════════════════════════════════════"

PEOPLE="${CELL}/GT_people.json"
[ -f "$PEOPLE" ] || PEOPLE="${CELL}/humans_${TOR_SEED}.json"

if ! command -v "$UV" >/dev/null 2>&1; then
  bad "\`uv\` not on PATH. The spawn search needs numpy/pyyaml/shapely/
      scikit-learn/matplotlib on top of the system python3; \`uv run --with\`
      is how every other host-side scene_gen tool in this repo gets them.
      Install uv, or run spawn_from_real_people.py under an interpreter that
      already has them."
  exit 1
fi

set +e
"$UV" run --with numpy --with matplotlib --with pyyaml --with shapely \
      --with scikit-learn python3 \
      "${REPO}/scripts/raven_live/spawn_from_real_people.py" \
      --people "$PEOPLE" --config "$SCENE_CONFIG" --out-dir "$OUT_DIR"
RC=$?
set -e

echo
case "$RC" in
  0) echo "══════════════════════════════════════════════════════════════════════"
     echo " ✅ CELL GOOD, CASUALTIES UNCHANGED. Next:"
     echo "      scripts/raven_live/stage2_env_1robot.sh   (or _2robot)"
     echo "══════════════════════════════════════════════════════════════════════" ;;
  2) echo "══════════════════════════════════════════════════════════════════════"
     echo " ⚠  CELL GOOD, BUT A CASUALTY MOVED — a substitution was made."
     echo "    The SPAWN_CONFIGS in _plans/raven_test_scene_runbook.md §3 and in"
     echo "    osmo/missions/raven_single_shared_test.yaml are STALE. Use only"
     echo "    ${OUT_DIR}/spawns_{1,2}robot.json from here on; stage2_env_*.sh"
     echo "    reads those files, so the rest of the checklist is unchanged."
     echo "══════════════════════════════════════════════════════════════════════" ;;
  *) echo "══════════════════════════════════════════════════════════════════════"
     echo " ❌ could not produce a spawn pair — read the error above."
     echo "══════════════════════════════════════════════════════════════════════" ;;
esac
exit "$RC"
