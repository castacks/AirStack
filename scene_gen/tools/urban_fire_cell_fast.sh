#!/usr/bin/env bash
# Experimental quality-equivalent urban-fire cell runner.
#
# Unlike urban_fire_cell.sh this does not run the standalone assembly pass:
# freeze_urban_fire_city_launch_script.py performs the same FireCityApp build,
# composition, prop and human placement before exporting.  Running both is a
# duplicate full-city build.  The trusted production runner is not modified.
set -uo pipefail

REPO="${REPO:-/isaac-sim/AirStack}"
CONTAINER="${CONTAINER:-$(docker ps --format '{{.Names}}' | grep -m1 isaac-sim || echo isaac-sim)}"
LEVELS="${LEVELS:-${1:-1}}"
FB_OUT="${FB_OUT:-/isaac-sim/.cache/fire_bakes/urban_1km}"
FREEZE_OUT="${FREEZE_OUT:-/isaac-sim/final_disaster_dataset/Fire/Urban_fast}"
NUCLEUS_DATASET="${NUCLEUS_DATASET:-omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/final_disaster_dataset}"
NUCLEUS_ASSET_ROOT="${NUCLEUS_ASSET_ROOT:-omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA}"
NUCLEUS_OBJAVERSE_ROOT="${NUCLEUS_OBJAVERSE_ROOT:-${NUCLEUS_ASSET_ROOT%/}/scene_gen/assets/objaverse}"
LOG_DIR="${LOG_DIR:-/isaac-sim/logs/urban_fire_fast}"
ISAAC_SIM_PYTHONPATH="${ISAAC_SIM_PYTHONPATH:-/isaac-sim/kit/python/lib/python3.11/site-packages}"

say() { printf '\n\033[1m== %s ==\033[0m\n' "$*"; }
die() { printf '\033[31mFAILED: %s\033[0m\n' "$*" >&2; exit 1; }
now() { date +%s; }
dex() {
  docker exec -e PYTHONHASHSEED=0 -e SG_INSTANCE_PLACEMENTS=1 \
    -e PYTHONUNBUFFERED=1 -e ISAAC_SIM_PYTHONPATH="$ISAAC_SIM_PYTHONPATH" "$@"
}

mkdir -p "$LOG_DIR"
RUN_START=$(now)

# Objaverse is a separate pseudo-scheme from airstack:// and therefore has a
# separate root override. Its source USDs use relative texture paths; opening
# those USDs locally makes Kit resolve the paths to /isaac-sim/... while
# flattening, even when every other asset already comes from Nucleus. Keep the
# cache mirrored one-for-one, then compose the source USDs from Nucleus so
# their relative textures resolve to Nucleus before the final flatten.
if [ "${SYNC_OBJAVERSE_TO_NUCLEUS:-1}" = "1" ]; then
  say "verify Objaverse source cache on Nucleus"
  dex "$CONTAINER" bash -lc "cd '$REPO' && bash scene_gen/tools/usd_python.sh \
    scene_gen/tools/dataset_upload.py \
    --local '$REPO/scene_gen/assets/objaverse' \
    --remote '$NUCLEUS_OBJAVERSE_ROOT'" \
    || die "Nucleus Objaverse source-cache sync"
fi

for L in $LEVELS; do
  LEVEL_START=$(now)
  PRESET="downtown_urban_fire_1000_l${L}"
  DUMP="$REPO/scene_gen/_plans/fc_dump_1km_l${L}.json"
  MANIFEST="$REPO/scene_gen/_plans/fire_corr_l${L}.json"
  CELL="${FREEZE_OUT}/level_${L}/1"
  FLOG="${LOG_DIR}/l${L}_freeze.log"
  TLOG="${LOG_DIR}/l${L}_timings.json"
  [ -f "$MANIFEST" ] || die "no manifest $MANIFEST"
  [ -f "$DUMP" ] || die "no placement dump $DUMP"
  CITY_SEED=$(python3 "$REPO/scene_gen/tools/fire_city_manifest.py" \
    "$MANIFEST" --print-seed) || die "could not read city seed"
  BAKES="${FB_OUT}/city_${CITY_SEED}"

  say "LEVEL $L fast path (seed $CITY_SEED)"

  # Cheap failures are rejected before consuming a pod GPU.  The proof is tied
  # to the dump, manifest, preset and every layout-affecting source/config file.
  VERIFY_START=$(now)
  KITDUMP="$REPO/scene_gen/_plans/kit_dump_l${L}.json"
  STAMP="$REPO/scene_gen/_plans/.dump_verified_l${L}_s${CITY_SEED}.json"
  python3 "$REPO/scene_gen/tools/urban_fire_fast_preflight.py" \
    --repo "$REPO" --dump "$DUMP" --manifest "$MANIFEST" || die "CPU preflight"
  if ! python3 "$REPO/scene_gen/tools/urban_fire_fast_preflight.py" \
      --repo "$REPO" --dump "$DUMP" --manifest "$MANIFEST" \
      --stamp "$STAMP" --check-stamp; then
    dex "$CONTAINER" bash -lc "cd '$REPO' && \
      ISAAC_SIM_HEADLESS=true SCENE_CONFIG='$PRESET' \
      FC_INTACT_ONLY=1 FC_DUMP='$KITDUMP' \
      /isaac-sim/python.sh '$REPO/simulation/isaac-sim/launch_scripts/urban_fire_city_launch_script.py' --no-window" \
      >"${LOG_DIR}/l${L}_kitdump.log" 2>&1 || die "Kit layout proof"
    if ! python3 "$REPO/scene_gen/tools/verify_dump_matches_kit.py" \
      --offline "$DUMP" --kit "$KITDUMP"; then
      say "offline reconstruction differs; promote authoritative Kit layout"
      DIMCAT="$REPO/scene_gen/config/harvested/urban_building_dimensions.json"
      python3 "$REPO/scene_gen/tools/kit_dump_to_dimension_catalog.py" \
        "$KITDUMP" --out "$DIMCAT" --merge-existing "$DIMCAT" \
        || die "canonical dimension catalog"
      cp "$DUMP" "${DUMP%.json}.offline.json" \
        || die "preserve offline layout candidate"
      cp "$KITDUMP" "$DUMP" || die "promote Kit layout"
      read -r FRAC EPOCH COLLAPSE F6 <<EOF
$(python3 - "$REPO/scene_gen/config/presets/${PRESET}.yaml" <<'PY'
import sys, yaml
d = yaml.safe_load(open(sys.argv[1]))
print(d["fire_target_frac"],
      float(d["duration_s"]) * float(d["start_offset_frac"]),
      d["fire_collapse"], d["fire_f6"])
PY
)
EOF
      python3 "$REPO/scene_gen/tools/fire_corridor_manifest.py" --dump "$DUMP" \
        --target-frac "$FRAC" --epoch-s "$EPOCH" --collapse "$COLLAPSE" \
        --f6 "$F6" --seed "$CITY_SEED" --out "$MANIFEST" \
        --bake-list "$REPO/scene_gen/_plans/bake_l${L}.json" \
        || die "regenerate fire plan from authoritative Kit layout"
      python3 "$REPO/scene_gen/tools/verify_dump_matches_kit.py" \
        --offline "$DUMP" --kit "$KITDUMP" \
        || die "promoted Kit layout failed its identity gate"
    fi
    python3 "$REPO/scene_gen/tools/urban_fire_fast_preflight.py" \
      --repo "$REPO" --dump "$DUMP" --manifest "$MANIFEST" \
      --stamp "$STAMP" --write-stamp || die "write layout proof"
  fi
  VERIFY_S=$(( $(now) - VERIFY_START ))

  BAKE_START=$(now)
  if [ "${FB_NUCLEUS_KITS:-1}" = "1" ]; then
    say "pull current sliced-kit cache from Nucleus"
    dex "$CONTAINER" bash -lc "cd '$REPO' && bash scene_gen/tools/usd_python.sh \
      scene_gen/tools/sync_sliced_kits.py --pull" \
      || die "Nucleus sliced-kit pull"
  fi
  if [ "${FB_CANONICAL_KITS:-1}" = "1" ] && [[ " ${BAKE_ARGS:-} " != *" --dry-run "* ]]; then
    GAC_ASSETS=$(python3 - "$MANIFEST" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
rows = d.get("records") or d.get("manifest") or d.get("buildings") or []
print(",".join(sorted({str(r["asset"]) for r in rows
                       if r.get("kind") == "gac" and r.get("asset")})))
PY
    )
    if [ -n "$GAC_ASSETS" ]; then
      say "warm canonical disaster-independent GAC kits"
      dex "$CONTAINER" bash -lc "cd '$REPO' && bash scene_gen/tools/usd_python.sh \
        scene_gen/tools/experimental_warm_canonical_kits.py --assets '$GAC_ASSETS'" \
        || die "canonical sliced-kit warmup"
    fi
  fi
  FB_OUT="$FB_OUT" CONTAINER="$CONTAINER" REPO="$REPO" SETTLE_FABRIC=1 \
    SETTLE_REST_V2="${SETTLE_REST_V2:-1}" \
    python3 "$REPO/scene_gen/tools/fire_city_bake_fast.py" "$MANIFEST" ${BAKE_ARGS:-} \
    || die "bake"
  if [[ " ${BAKE_ARGS:-} " != *" --dry-run "* ]]; then
    say "normalize reusable bake layers to Nucleus asset paths"
    python3 "$REPO/scene_gen/tools/normalize_fire_bake_cache.py" "$BAKES" \
      --backup-dir "$BAKES/pre_portable_backup" \
      || die "portable bake-cache normalization"
  fi
  if [ "${FB_NUCLEUS_KITS:-1}" = "1" ] && [[ " ${BAKE_ARGS:-} " != *" --dry-run "* ]]; then
    say "push newly sliced kits to Nucleus"
    dex "$CONTAINER" bash -lc "cd '$REPO' && bash scene_gen/tools/usd_python.sh \
      scene_gen/tools/sync_sliced_kits.py --push" \
      || die "Nucleus sliced-kit push"
  fi
  BAKE_S=$(( $(now) - BAKE_START ))
  N_BAKES=$(dex "$CONTAINER" bash -lc "ls -1 '$BAKES'/*.usd 2>/dev/null | wc -l" | tr -d ' \r')
  [ "${N_BAKES:-0}" != "0" ] || die "no bakes in $BAKES"

  # Pure CPU correctness/review gate.  Freeze independently solves the chosen
  # PEOPLE_VARIANT before Kit starts, so there is no second placement process.
  PEOPLE_START=$(now)
  bash "$REPO/scene_gen/tools/fire_people_rerun.sh" "$MANIFEST" "$DUMP" "$BAKES" \
    || die "people planning gate"
  PEOPLE_S=$(( $(now) - PEOPLE_START ))

  # The former standalone assembly stage is intentionally absent.  This one
  # Kit process assembles, composes damage, places props/people, captures and
  # exports.  That is exactly what the freeze launcher already does internally.
  FREEZE_START=$(now)
  dex "$CONTAINER" bash -lc "cd '$REPO' && \
    REPO='$REPO' AIRSTACK_ASSET_ROOT='$NUCLEUS_ASSET_ROOT' \
    OBJAVERSE_ASSET_ROOT='$NUCLEUS_OBJAVERSE_ROOT' \
    SCENE_CONFIG='$PRESET' FC_MANIFEST='$MANIFEST' FC_DUMP='$DUMP' FC_BAKES='$BAKES' \
    FC_LAYOUT_STAMP='$STAMP' \
    FREEZE_OUT='$CELL' FREEZE_HEADLESS=1 FREEZE_EXPORT=1 \
    FREEZE_SNAPS='${FREEZE_SNAPS:-1}' PEOPLE_VARIANT='${PEOPLE_VARIANT:-1}' \
    /isaac-sim/python.sh '$REPO/simulation/isaac-sim/launch_scripts/experimental_fast_freeze_urban_fire_city_launch_script.py' --no-window" \
    2>&1 | tee "$FLOG"
  [ "${PIPESTATUS[0]}" = 0 ] || die "combined assemble/freeze"
  FREEZE_S=$(( $(now) - FREEZE_START ))

  grep -q 'Flow OOM check CLEAN' "$FLOG" || die "Flow OOM check did not pass"
  grep -q 'FLOW STARVED' "$FLOG" && die "Flow starved"
  grep -q 'solved ZERO burning buildings' "$FLOG" && die "zero burning buildings in people solve"

  GATE_START=$(now)
  python3 - "$CELL" <<'PY' || die "frozen output gate"
import glob, json, os, sys
cell = sys.argv[1]
usds = glob.glob(os.path.join(cell, "*.usd"))
if not usds:
    raise SystemExit("no frozen USD")
p = os.path.join(cell, "freeze_report.json")
if not os.path.isfile(p):
    raise SystemExit("no freeze_report.json")
r = json.load(open(p))
if not r.get("portable_ok"):
    raise SystemExit("portable_ok=False")
if not r.get("sky_lights"):
    raise SystemExit("no sky light")
print("fast cell gate OK:", usds[0])
PY
  GATE_S=$(( $(now) - GATE_START ))

  # Local output is disposable build scratch. Publish the COMPLETE cell,
  # including review snapshots and every JSON sidecar, then open the USD from
  # its Nucleus URL and resolve dependencies there. A local-only success is not
  # considered a completed level.
  PUBLISH_START=$(now)
  REMOTE_CELL="$NUCLEUS_DATASET/Fire/Urban_fast/level_${L}/1"
  say "publish complete cell to Nucleus"
  dex "$CONTAINER" bash -lc "cd '$REPO' && bash scene_gen/tools/usd_python.sh \
    scene_gen/tools/dataset_upload.py --local '$CELL' --remote '$REMOTE_CELL' \
    --include-snaps --native-tree-copy \
    --workers '${DATASET_UPLOAD_WORKERS:-4}'" \
    || die "Nucleus cell upload/size verification"
  dex "$CONTAINER" bash -lc "cd '$REPO' && bash scene_gen/tools/usd_python.sh \
    scene_gen/tools/verify_nucleus_cell.py '$REMOTE_CELL'" \
    || die "cold-open published Nucleus cell"
  PUBLISH_S=$(( $(now) - PUBLISH_START ))
  TOTAL_S=$(( $(now) - LEVEL_START ))

  python3 - "$TLOG" "$VERIFY_S" "$BAKE_S" "$PEOPLE_S" "$FREEZE_S" "$GATE_S" "$PUBLISH_S" "$TOTAL_S" <<'PY'
import json, sys
keys = ("layout_verify_s", "bake_s", "people_gate_s", "combined_assemble_freeze_s", "gate_s", "nucleus_publish_verify_s", "total_s")
vals = [int(x) for x in sys.argv[2:]]
with open(sys.argv[1], "w") as f:
    json.dump(dict(zip(keys, vals)), f, indent=2)
print("timings:", dict(zip(keys, vals)))
PY
done

say "all requested levels complete in $(( $(now) - RUN_START )) s"
