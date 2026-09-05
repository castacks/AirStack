#!/usr/bin/env bash
# Experimental, non-destructive urban-earthquake dataset runner.
#
# It consumes the reviewed quake archetype library from Nucleus, builds one
# live stage per level, writes complete GT/review evidence, freezes, uploads,
# and cold-opens the published Nucleus cell.  The production launch path is
# unchanged; all behaviour here is opt-in via FREEZE_* / QUAKE_* variables.
set -uo pipefail

REPO="${REPO:-/root/AirStack}"
CONTAINER="${CONTAINER:-isaac-sim}"
LEVELS="${LEVELS:-${*:-1 2 3}}"
LOG_DIR="${LOG_DIR:-/root/docker/isaac-sim/logs/urban_quake_fast}"
FREEZE_ROOT="${FREEZE_ROOT:-/isaac-sim/final_disaster_dataset/Earthquake/Urban_fast}"
NUCLEUS_ROOT="${NUCLEUS_ROOT:-omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA}"
NUCLEUS_DATASET="${NUCLEUS_DATASET:-${NUCLEUS_ROOT}/final_disaster_dataset}"
ARCH_DIR="${ARCH_DIR:-${NUCLEUS_ROOT}/scene_gen/assets/archetype_r15}"
GAC_ARCH_DIR="${GAC_ARCH_DIR:-${NUCLEUS_ROOT}/scene_gen/assets/gac_quake}"
ASSET_SET="${ASSET_SET:-urban_quake_v5}"
REGION_M="${REGION_M:-1000x1000}"
SCENE_CONFIG="${SCENE_CONFIG:-downtown_earthquake}"
ACTIVE_GPU="${ISAAC_SIM_ACTIVE_GPU:-2}"

say() { printf '\n== %s ==\n' "$*"; }
die() { printf 'FAILED: %s\n' "$*" >&2; exit 1; }
now() { date +%s; }
dex() {
  docker exec -e PYTHONHASHSEED=0 -e PYTHONUNBUFFERED=1 \
    -e ISAAC_SIM_ACTIVE_GPU="$ACTIVE_GPU" "$CONTAINER" "$@"
}
magnitude_for() {
  case "$1" in
    1) printf '5.5' ;;
    2) printf '6.5' ;;
    3) printf '7.5' ;;
    *) die "unsupported level $1 (expected 1, 2, or 3)" ;;
  esac
}
seed_for() {
  case "$1" in
    1) printf '31' ;;
    2) printf '41' ;;
    3) printf '51' ;;
  esac
}

mkdir -p "$LOG_DIR"
RUN_START=$(now)

for L in $LEVELS; do
  MAG=$(magnitude_for "$L")
  SEED=$(seed_for "$L")
  CELL="${FREEZE_ROOT}/level_${L}/1"
  SNAPS="${CELL}/snaps"
  REMOTE_CELL="${NUCLEUS_DATASET}/Earthquake/Urban_fast/level_${L}/1"
  RUN_LOG="${LOG_DIR}/l${L}_build_freeze.log"
  TIMING="${LOG_DIR}/l${L}_timings.json"
  LEVEL_START=$(now)

  say "earthquake L${L}: M${MAG}, seed ${SEED}"
  # Preserve an incomplete prior attempt for diagnosis; never overwrite a
  # potentially useful failed stage in place.
  if dex bash -lc "test -d '$CELL' && python3 - '$CELL' <<'PY'
import json, os, sys
p = os.path.join(sys.argv[1], 'freeze_report.json')
try:
    r = json.load(open(p))
except Exception:
    raise SystemExit(1)
raise SystemExit(0 if r.get('portable_ok') else 1)
PY"; then
    say "local portable cell already exists; skipping rebuild"
  else
    dex bash -lc "if [ -d '$CELL' ]; then mv '$CELL' '${CELL}.failed_$(date +%Y%m%d_%H%M%S)'; fi; mkdir -p '$CELL'" \
      || die "prepare L${L} output"
    BUILD_START=$(now)
    dex bash -lc "cd '/isaac-sim/AirStack' && \
      REPO='/isaac-sim/AirStack' \
      AIRSTACK_ASSET_ROOT='$NUCLEUS_ROOT' \
      OBJAVERSE_ASSET_ROOT='${NUCLEUS_ROOT}/scene_gen/assets/objaverse' \
      SCENE_CONFIG='$SCENE_CONFIG' ASSET_SET='$ASSET_SET' \
      REGION_M='$REGION_M' MAGNITUDE='$MAG' QUAKE_SEED='$SEED' \
      ARCH_DIR='$ARCH_DIR' GAC_ARCH_DIR='$GAC_ARCH_DIR' \
      QUAKE_EXTENSIVE_REVIEW=1 QUAKE_REVIEW_MAJOR_MAX='${QUAKE_REVIEW_MAJOR_MAX:-48}' \
      FREEZE_OUT='$CELL' FREEZE_EXPORT=1 FREEZE_COLLECT=0 \
      FREEZE_SNAPS=1 SNAP_DIR='$SNAPS' FREEZE_EXIT=1 \
      PEOPLE_VARIANT='${PEOPLE_VARIANT:-1}' \
      ISAAC_SIM_HEADLESS=true \
      timeout '${QUAKE_TIMEOUT_S:-21600}s' /isaac-sim/python.sh \
      simulation/isaac-sim/launch_scripts/experimental_fast_freeze_urban_quake_city_launch_script.py \
      --no-window" 2>&1 | tee "$RUN_LOG"
    [ "${PIPESTATUS[0]}" = 0 ] || die "L${L} build/freeze"
    BUILD_S=$(( $(now) - BUILD_START ))
    grep -q 'REVIEW GATE OK' "$RUN_LOG" || die "L${L} review gate"
    grep -q 'FREEZE DONE' "$RUN_LOG" || \
      grep -q 'durable report observed; hard batch exit (portable_ok=True)' "$RUN_LOG" \
      || die "L${L} freeze completion marker"
  fi

  GATE_START=$(now)
  dex python3 - "$CELL" <<'PY' || die "L${L} local cell gate"
import glob, json, os, sys
cell = sys.argv[1]
need = ('GT_people.json', 'GT_hints.json', 'build_stats.json',
        'freeze_report.json', 'review_manifest.json')
missing = [q for q in need if not os.path.isfile(os.path.join(cell, q))]
if missing:
    raise SystemExit('missing sidecars: ' + ', '.join(missing))
usds = glob.glob(os.path.join(cell, '*.usd')) + glob.glob(os.path.join(cell, '*.usdc'))
if len(usds) != 1:
    raise SystemExit('expected exactly one USD, found %d' % len(usds))
freeze = json.load(open(os.path.join(cell, 'freeze_report.json')))
review = json.load(open(os.path.join(cell, 'review_manifest.json')))
people = json.load(open(os.path.join(cell, 'GT_people.json')))
if not freeze.get('portable_ok'):
    raise SystemExit('portable_ok is false')
if not review.get('ok'):
    raise SystemExit('review gate is false')
if review.get('successful_views') != review.get('expected_views'):
    raise SystemExit('review views incomplete')
if people.get('standing_or_walking') != 0 or people.get('count', 0) <= 0:
    raise SystemExit('casualty-only population gate failed')
print('local quake cell gate OK:', usds[0])
PY
  GATE_S=$(( $(now) - GATE_START ))

  say "render exact 2-D damage/casualty plan"
  dex bash -lc "cd '/isaac-sim/AirStack' && /isaac-sim/python.sh \
    scene_gen/tools/quake_damage_map.py \
    --records '$SNAPS/quake_buildings.json' \
    --people '$SNAPS/quake_people.json' --region 1000 --map-only \
    --title 'Urban earthquake L${L} (M${MAG}) — exact damage assignment' \
    --out '$SNAPS/damage_plan.png'" \
    || die "L${L} damage map"
  dex test -s "$SNAPS/damage_plan.png" || die "L${L} damage map is empty"

  PUBLISH_START=$(now)
  say "publish complete L${L} cell to Nucleus"
  dex bash -lc "cd '/isaac-sim/AirStack' && bash scene_gen/tools/usd_python.sh \
    scene_gen/tools/dataset_upload.py --local '$CELL' \
    --remote '$REMOTE_CELL' --include-snaps --native-tree-copy \
    --workers '${DATASET_UPLOAD_WORKERS:-4}'" \
    || die "L${L} Nucleus upload/size gate"
  dex bash -lc "cd '/isaac-sim/AirStack' && bash scene_gen/tools/usd_python.sh \
    scene_gen/tools/verify_nucleus_cell.py '$REMOTE_CELL'" \
    || die "L${L} Nucleus cold-open gate"
  PUBLISH_S=$(( $(now) - PUBLISH_START ))
  TOTAL_S=$(( $(now) - LEVEL_START ))
  BUILD_S=${BUILD_S:-0}
  python3 - "$TIMING" "$BUILD_S" "$GATE_S" "$PUBLISH_S" "$TOTAL_S" <<'PY'
import json, sys
keys = ('build_review_freeze_s', 'local_gate_map_s',
        'nucleus_publish_verify_s', 'total_s')
vals = [int(v) for v in sys.argv[2:]]
with open(sys.argv[1], 'w') as fh:
    json.dump(dict(zip(keys, vals)), fh, indent=2)
print('timings:', dict(zip(keys, vals)))
PY
  unset BUILD_S
done

say "all requested earthquake levels complete in $(( $(now) - RUN_START )) s"
