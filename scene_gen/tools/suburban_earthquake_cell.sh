#!/usr/bin/env bash
# Build, review, freeze, publish and cold-verify Earthquake/Suburban L1-L3.
# Run on an Isaac pod host; one Kit process is used at a time.
set -uo pipefail

REPO="${REPO:-/root/AirStack}"
CONTAINER="${CONTAINER:-isaac-sim}"
LEVELS="${LEVELS:-${*:-1 2 3}}"
ACTIVE_GPU="${ISAAC_SIM_ACTIVE_GPU:-2}"
LOG_DIR="${LOG_DIR:-/root/docker/isaac-sim/logs/suburban_quake_dataset}"
FREEZE_ROOT="${FREEZE_ROOT:-/isaac-sim/final_disaster_dataset/Earthquake/Suburban}"
WORK_ROOT="${WORK_ROOT:-/isaac-sim/.cache/suburban_earthquake/work}"
CACHE="${QUAKE_HOUSE_CACHE:-/isaac-sim/.cache/suburban_earthquake/fractured_bearing_v5}"
NUCLEUS_ROOT="${NUCLEUS_ROOT:-omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA}"
NUCLEUS_DATASET="${NUCLEUS_DATASET:-${NUCLEUS_ROOT}/final_disaster_dataset}"
CACHE_REMOTE="${CACHE_REMOTE:-${NUCLEUS_ROOT}/scene_gen/assets/suburban_quake_cache/fractured_bearing_v5}"

say() { printf '\n== %s ==\n' "$*"; }
die() { printf 'FAILED: %s\n' "$*" >&2; exit 1; }
now() { date +%s; }
dex() {
  docker exec -i -e PYTHONHASHSEED=0 -e PYTHONUNBUFFERED=1 \
    -e ISAAC_SIM_ACTIVE_GPU="$ACTIVE_GPU" "$CONTAINER" "$@"
}
config_for() { printf 'suburb_earthquake_1000_l%s' "$1"; }
magnitude_for() {
  case "$1" in 1) printf '5.5';; 2) printf '6.5';; 3) printf '7.5';;
    *) die "unsupported level $1";; esac
}
seed_for() { case "$1" in 1) printf '31';; 2) printf '41';; 3) printf '51';; esac; }
people_for() { case "$1" in 1) printf '12';; 2) printf '50';; 3) printf '70';; esac; }

mkdir -p "$LOG_DIR"
RUN_START=$(now)
dex mkdir -p "$CACHE" "$WORK_ROOT" || die "create cache/work roots"

say "restore shared suburban-earthquake cache from Nucleus"
# A first-ever cache has no remote tree yet. dataset_upload reports that and
# returns with an empty download; the prepare pass below then creates it.
dex bash -lc "cd '/isaac-sim/AirStack' && bash scene_gen/tools/usd_python.sh \
  scene_gen/tools/dataset_upload.py --download --local '$CACHE' \
  --remote '$CACHE_REMOTE'" || die "download Nucleus house cache"

for L in $LEVELS; do
  CONFIG=$(config_for "$L")
  MAG=$(magnitude_for "$L")
  SEED=$(seed_for "$L")
  PEOPLE=$(people_for "$L")
  CELL="${FREEZE_ROOT}/level_${L}/1"
  REMOTE_CELL="${NUCLEUS_DATASET}/Earthquake/Suburban/level_${L}/1"
  WORK="${WORK_ROOT}/level_${L}"
  PLAN="${WORK}/damage_plan"
  MANIFEST="${WORK}/bake_work.json"
  SCENE="${WORK}/scene"
  NAME="earthquake_suburban_lvl${L}_1"
  RUN_LOG="${LOG_DIR}/l${L}.log"
  LEVEL_START=$(now)

  say "suburban earthquake L${L}: M${MAG}, seed ${SEED}"
  if dex python3 - "$CELL" <<'PY'
import glob,json,os,sys
cell=sys.argv[1]
try: report=json.load(open(os.path.join(cell,'freeze_report.json')))
except Exception: raise SystemExit(1)
usds=glob.glob(os.path.join(cell,'*.usd'))+glob.glob(os.path.join(cell,'*.usdc'))
raise SystemExit(0 if report.get('portable_ok') and len(usds)==1 else 1)
PY
  then
    say "local portable L${L} already exists; skipping build"
  else
    dex bash -lc "stamp=\$(date +%Y%m%d_%H%M%S); \
      if [ -d '$CELL' ]; then mv '$CELL' '${CELL}.failed_'\"\$stamp\"; fi; \
      if [ -d '$WORK' ]; then mv '$WORK' '${WORK}.failed_'\"\$stamp\"; fi; \
      mkdir -p '$WORK' '$CELL/snaps'" || die "prepare L${L} directories"

    say "write exact 2-D damage plan"
    dex bash -lc "cd '/isaac-sim/AirStack' && /isaac-sim/python.sh \
      scene_gen/tools/suburban_quake_plan.py --config '$CONFIG' --out '$PLAN'" \
      2>&1 | tee "$RUN_LOG"
    [ "${PIPESTATUS[0]}" = 0 ] || die "L${L} plan"

    say "prepare and deduplicate required house caches"
    dex bash -lc "cd '/isaac-sim/AirStack' && bash scene_gen/tools/suburban_quake_cpu.sh \
      scene_gen/tools/suburban_quake_prepare.py --report '${PLAN}.json' \
      --cache '$CACHE' --manifest '$MANIFEST'" 2>&1 | tee -a "$RUN_LOG"
    [ "${PIPESTATUS[0]}" = 0 ] || die "L${L} cache prepare"
    PENDING=$(dex python3 - "$MANIFEST" <<'PY'
import json,sys
print(sum(not row.get('ready') for row in json.load(open(sys.argv[1]))))
PY
    ) || die "count pending L${L} caches"
    if [ "$PENDING" -gt 0 ]; then
      say "settle ${PENDING} new cache entries on assigned GPU"
      dex bash -lc "cd '/isaac-sim/AirStack' && \
        QUAKE_BAKE_WORK='$MANIFEST' QUAKE_SETTLE_GPU=1 \
        ISAAC_SIM_ACTIVE_GPU='$ACTIVE_GPU' ISAAC_SIM_HEADLESS=true \
        timeout '${QUAKE_BAKE_TIMEOUT_S:-21600}s' /isaac-sim/python.sh \
        simulation/isaac-sim/launch_scripts/suburb_earthquake_bake_launch_script.py \
        --no-window" 2>&1 | tee -a "$RUN_LOG"
      [ "${PIPESTATUS[0]}" = 0 ] || die "L${L} isolated cache bake"
    else
      say "all L${L} house caches were hits"
    fi
    dex python3 - "$MANIFEST" <<'PY' || die "L${L} cache readiness gate"
import json,sys
rows=json.load(open(sys.argv[1]))
bad=[]
for row in rows:
    try: meta=json.load(open(row['sidecar']))
    except Exception as exc: bad.append((row['id'],str(exc))); continue
    if not meta.get('physics_ready'): bad.append((row['id'],'not physics_ready'))
if bad: raise SystemExit('unready cache entries: '+repr(bad[:10]))
print('cache readiness gate OK:',len(rows),'unique entries')
PY

    say "publish reusable house cache to Nucleus"
    dex bash -lc "cd '/isaac-sim/AirStack' && bash scene_gen/tools/usd_python.sh \
      scene_gen/tools/dataset_upload.py --local '$CACHE' \
      --remote '$CACHE_REMOTE' --workers '${DATASET_UPLOAD_WORKERS:-4}'" \
      2>&1 | tee -a "$RUN_LOG"
    [ "${PIPESTATUS[0]}" = 0 ] || die "L${L} cache upload"

    say "assemble binary USD from settled cache"
    dex bash -lc "cd '/isaac-sim/AirStack' && bash scene_gen/tools/suburban_quake_cpu.sh \
      scene_gen/tools/suburban_quake_build.py --out '$SCENE' \
      --config '$CONFIG' --cache '$CACHE' --scene-format usdc" \
      2>&1 | tee -a "$RUN_LOG"
    [ "${PIPESTATUS[0]}" = 0 ] || die "L${L} CPU assembly"
    dex bash -lc "cd '/isaac-sim/AirStack' && bash scene_gen/tools/suburban_quake_cpu.sh \
      scene_gen/tools/suburban_quake_audit.py '$SCENE' --cache '$CACHE' \
      --expect-people '$PEOPLE'" 2>&1 | tee -a "$RUN_LOG"
    [ "${PIPESTATUS[0]}" = 0 ] || die "L${L} cold CPU audit"
    dex cp "${PLAN}.png" "$CELL/snaps/damage_plan.png" || die "copy L${L} map"

    say "render extensive review and freeze portable cell"
    dex bash -lc "cd '/isaac-sim/AirStack' && \
      REVIEW_INPUT='$SCENE' FREEZE_OUT='$CELL' FREEZE_NAME='$NAME' \
      SNAP_DIR='$CELL/snaps' SCENE_CONFIG='$CONFIG' MAGNITUDE='$MAG' \
      QUAKE_SEED='$SEED' SNAP_BLANK_RETRIES='${SNAP_BLANK_RETRIES:-12}' \
      SUBURB_QUAKE_REVIEW_BUILDINGS='${SUBURB_QUAKE_REVIEW_BUILDINGS:-24}' \
      ISAAC_SIM_ACTIVE_GPU='$ACTIVE_GPU' ISAAC_SIM_HEADLESS=true \
      timeout '${QUAKE_FREEZE_TIMEOUT_S:-21600}s' /isaac-sim/python.sh \
      simulation/isaac-sim/launch_scripts/suburb_earthquake_freeze_launch_script.py \
      --no-window" 2>&1 | tee -a "$RUN_LOG"
    [ "${PIPESTATUS[0]}" = 0 ] || die "L${L} review/freeze"
  fi

  dex python3 - "$CELL" "$PEOPLE" <<'PY' || die "L${L} local publication gate"
import glob,json,os,sys
cell,want=sys.argv[1],int(sys.argv[2])
required=('GT_people.json','GT_hints.json','build_stats.json',
          'freeze_report.json','review_manifest.json')
missing=[p for p in required if not os.path.isfile(os.path.join(cell,p))]
usds=glob.glob(os.path.join(cell,'*.usd'))+glob.glob(os.path.join(cell,'*.usdc'))
if missing or len(usds)!=1: raise SystemExit('missing=%r usds=%r'%(missing,usds))
people=json.load(open(os.path.join(cell,'GT_people.json')))
review=json.load(open(os.path.join(cell,'review_manifest.json')))
freeze=json.load(open(os.path.join(cell,'freeze_report.json')))
if people.get('count')!=want or people.get('standing_or_walking')!=0:
    raise SystemExit('casualty gate failed: '+repr(people.get('count')))
if not review.get('ok') or review.get('successful_views')!=review.get('expected_views'):
    raise SystemExit('review gate failed')
if not freeze.get('portable_ok'): raise SystemExit('portable gate failed')
print('local publication gate OK:',usds[0])
PY

  say "upload and cold-open canonical Nucleus cell"
  dex bash -lc "cd '/isaac-sim/AirStack' && bash scene_gen/tools/usd_python.sh \
    scene_gen/tools/dataset_upload.py --local '$CELL' --remote '$REMOTE_CELL' \
    --include-snaps --native-tree-copy --workers '${DATASET_UPLOAD_WORKERS:-4}'" \
    2>&1 | tee -a "$RUN_LOG"
  [ "${PIPESTATUS[0]}" = 0 ] || die "L${L} cell upload"
  dex bash -lc "cd '/isaac-sim/AirStack' && bash scene_gen/tools/usd_python.sh \
    scene_gen/tools/verify_nucleus_cell.py '$REMOTE_CELL'" \
    2>&1 | tee -a "$RUN_LOG"
  [ "${PIPESTATUS[0]}" = 0 ] || die "L${L} Nucleus cold-open gate"
  printf '{"level":%s,"total_s":%s}\n' "$L" "$(( $(now)-LEVEL_START ))" \
    > "${LOG_DIR}/l${L}_timing.json"
done

say "all requested suburban-earthquake levels complete in $(( $(now)-RUN_START )) s"
