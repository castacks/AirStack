#!/usr/bin/env bash
# urban_fire_cell.sh — build ONE urban-fire cell end to end, on a pod.
#
# Stages 1-3 are HOST work (CPU, no GPU, no Kit) and belong to
# `make_cell_plan.sh`; this is the POD half — clear, bake, assemble, people,
# freeze, verify. Run on an OSMO dev pod with the isaac-sim container up.
#
#   bash scene_gen/tools/urban_fire_cell.sh 1                      # level 1
#   BAKE_ARGS=--dry-run bash scene_gen/tools/urban_fire_cell.sh 1   # preview
#   LEVELS="1 2 3" bash scene_gen/tools/urban_fire_cell.sh          # all three
#
# EVERY GATE HERE IS A GATE BECAUSE SOMETHING SHIPPED WITHOUT IT. The recorded
# failure mode of this pipeline is not a crash — it is a run that prints every
# count correctly, exits 0, and produces a cell with no fire, no smoke, no
# bakes composed, or no `.usd` at all. So each stage below asserts on its
# OUTPUT rather than on its exit code, because the launchers mostly return 0
# regardless:
#
#   * `FREEZE_EXPORT` defaults to "0" (freeze script:259) -- without it the
#     freeze captures snaps, writes GT, and never writes a USD.
#   * `fire_city_bake.sh` writes to `$FB_OUT/city_<seed>` (:258) but the
#     assembly globs `FC_BAKES/*.usd` NON-recursively (launcher:908), so
#     FC_BAKES pointed at the parent composes ZERO bakes and builds the
#     intact city.
#   * the portability gate raises `PortabilityError`, is caught, printed, and
#     the process still exits 0 (freeze script:884-903).
#   * Flow OOM "does not crash and does not raise. It renders a city with NO
#     smoke in it" -- it only prints.
#   * `city_layout_audit.py` has no `sys.exit`; `|| die` on it is decorative.
#   * `fire_people.py` has no argparse and no __main__ -- calling it as a
#     module is a silent no-op. `fire_people_rerun.sh` is the real gate.
#
# WHAT IT ASSUMES EXISTS (written by the host half):
#   scene_gen/_plans/fc_dump_1km_l<N>.json    the frozen layout
#   scene_gen/_plans/fire_corr_l<N>.json      the corridor manifest
#   scene_gen/_plans/bake_l<N>.json           the bake worklist (a report)
set -uo pipefail

REPO="${REPO:-/isaac-sim/AirStack}"
# The pod's container is `isaac-sim` on some images and `isaac-sim-livestream`
# on others, decided by the COMMITTED .env rather than the workflow -- an
# unattended chain has failed on the very first step over exactly this.
CONTAINER="${CONTAINER:-$(docker ps --format '{{.Names}}' | grep -m1 isaac-sim || echo isaac-sim)}"
LEVELS="${LEVELS:-${1:-1}}"
FB_OUT="${FB_OUT:-/isaac-sim/.cache/fire_bakes/urban_1km}"
FREEZE_OUT="${FREEZE_OUT:-/isaac-sim/final_disaster_dataset/Fire/Urban}"
SNAP_ROOT="${SNAP_ROOT:-/isaac-sim/snaps/urban_fire}"
LOG_DIR="${LOG_DIR:-/isaac-sim/logs/urban_fire}"
STALE_BEFORE="${STALE_BEFORE:-2026-09-02}"

# ---- DETERMINISM. Every stage, not just the bake driver. --------------------
# `SG_INSTANCE_PLACEMENTS` is read in exactly ONE place
# (`generate_scene.py:374`) and nothing in the fire tree reads it, which is
# why it gets omitted on one stage and silently desyncs the layout.
export PYTHONHASHSEED=0
export SG_INSTANCE_PLACEMENTS=1
# `gac_storey_slice.clip()` imports vtk lazily INSIDE a live SimulationApp and
# can raise ModuleNotFoundError even though vtk is installed. One line, zero
# cost, removes a first-boot failure.
export ISAAC_SIM_PYTHONPATH="${ISAAC_SIM_PYTHONPATH:-/isaac-sim/kit/python/lib/python3.11/site-packages}"

say () { printf '\n\033[1m== %s ==\033[0m\n' "$*"; }
die () { printf '\033[31mFAILED: %s\033[0m\n' "$*" >&2; exit 1; }

# NOTE: `final_disaster_dataset/Fire/Urban` was emptied earlier pending an
# empty-block investigation. That finding did not reproduce (105 of 106 blocks
# filled; 24/24 swept seeds had zero empty blocks -- see the
# build-urban-fire-city skill, "Still open"), and generating 1 km directly
# dissolves it either way, so the run is no longer gated on it. Set
# FC_REFUSE_OVERWRITE=1 if you want the old refuse-if-present behaviour.
if [ "${FC_REFUSE_OVERWRITE:-0}" = "1" ] && [ -d "$FREEZE_OUT" ]; then
  die "$FREEZE_OUT exists and FC_REFUSE_OVERWRITE=1"
fi

mkdir -p "$LOG_DIR"

dex () {
  # PYTHONUNBUFFERED is not optional once we tee to a log: a bake has written
  # all 72 archetypes and printed nothing at all without it.
  docker exec -e PYTHONHASHSEED=0 -e SG_INSTANCE_PLACEMENTS=1 \
              -e PYTHONUNBUFFERED=1 \
              -e ISAAC_SIM_PYTHONPATH="$ISAAC_SIM_PYTHONPATH" "$@"
}

# ---- Stage 0: POD PRE-FLIGHT + CACHE HYGIENE -------------------------------
say "0/9 pre-flight"
SHM=$(docker exec "$CONTAINER" df -B1 --output=size /dev/shm 2>/dev/null | tail -1 | tr -d ' ')
if [ -n "${SHM:-}" ] && [ "$SHM" -lt 1000000000 ]; then
  die "/dev/shm is $((SHM/1024/1024)) MB — Kit fills it and dies during
     extension startup with 'Bus error (core dumped)'. Remount 16G."
fi
# A hard-killed Kit leaves segments that segfault the NEXT launch 1-2 s in.
dex "$CONTAINER" bash -lc 'rm -f /dev/shm/carb-RStringInternals-* /dev/shm/sem.* 2>/dev/null; true'

# STALE BAKES ARE INVISIBLE TO THE CACHE. `fire_city_bake.sh` keys on
# `<stem>.usd` + `<stem>.json` EXISTING, so a bake carrying the mirrored-atlas
# soot or roof-piercing columns (both fixed 2026-09-02) classifies as HAVE and
# is skipped forever. A settle does not reproduce, so the remedy is delete,
# never "re-run to get the same pile".
if [ "${FC_CLEAR_STALE:-1}" = "1" ]; then
  N_STALE=$(dex "$CONTAINER" bash -lc \
    "find '$FB_OUT' -name '*.usd' ! -newermt '$STALE_BEFORE' 2>/dev/null | wc -l" | tr -d ' \r')
  if [ "${N_STALE:-0}" != "0" ]; then
    say "  clearing $N_STALE bake(s) older than $STALE_BEFORE"
    dex "$CONTAINER" bash -lc \
      "find '$FB_OUT' \\( -name '*.usd' -o -name '*.json' \\) ! -newermt '$STALE_BEFORE' -delete" \
      || die "could not clear stale bakes"
  fi
fi

for L in $LEVELS; do
  PRESET="downtown_urban_fire_1000_l${L}"
  DUMP="$REPO/scene_gen/_plans/fc_dump_1km_l${L}.json"
  MANIFEST="$REPO/scene_gen/_plans/fire_corr_l${L}.json"
  WORKLIST="$REPO/scene_gen/_plans/bake_l${L}.json"
  SNAPS="${SNAP_ROOT}/l${L}"
  CELL="${FREEZE_OUT}/level_${L}/1"
  ALOG="${LOG_DIR}/l${L}_assemble.log"
  FLOG="${LOG_DIR}/l${L}_freeze.log"

  [ -f "$MANIFEST" ] || die "no manifest $MANIFEST — run make_cell_plan.sh first"

  # THE BAKE DIR THE ASSEMBLY MUST BE POINTED AT. `fire_city_bake.sh` appends
  # `city_<seed>`; the assembly globs `FC_BAKES/*.usd` non-recursively, so the
  # parent composes zero bakes and silently builds the INTACT city.
  CITY_SEED=$(python3 "$REPO/scene_gen/tools/fire_city_manifest.py" \
                "$MANIFEST" --print-seed) || die "could not read seed from $MANIFEST"
  BAKES="${FB_OUT}/city_${CITY_SEED}"

  say "LEVEL ${L} — ${PRESET}  (city seed ${CITY_SEED}, bakes -> ${BAKES})"

  # ---- Stage 3b: PROVE THE OFFLINE DUMP IS THE CITY KIT BUILDS -------------
  # The offline dump is a CPU reimplementation of `dump_city_placements`. If
  # the two disagree the manifest names buildings the assembled city does not
  # have in those positions and the bakes land on the wrong cells — visible
  # three stages later as `manifest/city match: 4/34`, long after the cheap
  # place to catch it. FC_INTACT_ONLY=1 dumps placements WITHOUT composing any
  # bake, so this is cheap and needs no bakes to exist yet.
  #
  # Runs once per (preset, seed): the result is cached beside the dump, and
  # FC_SKIP_DUMP_DIFF=1 skips it entirely once you trust a level.
  KITDUMP="$REPO/scene_gen/_plans/kit_dump_l${L}.json"
  STAMP="$REPO/scene_gen/_plans/.dump_verified_l${L}_s${CITY_SEED}"
  if [ "${FC_SKIP_DUMP_DIFF:-0}" != "1" ] && [ ! -f "$STAMP" ]; then
    say "3b/9 verify the offline dump against a real Kit build"
    dex "$CONTAINER" bash -lc "
        cd $REPO &&
        ISAAC_SIM_HEADLESS=true SCENE_CONFIG='$PRESET' \
        FC_INTACT_ONLY=1 FC_DUMP='$KITDUMP' \
        ./python.sh simulation/isaac-sim/launch_scripts/urban_fire_city_launch_script.py --no-window
    " > "${LOG_DIR}/l${L}_kitdump.log" 2>&1 || die "Kit intact-only dump (level $L)"
    if ! python3 "$REPO/scene_gen/tools/verify_dump_matches_kit.py" \
        --offline "$DUMP" --kit "$KITDUMP"; then
      say "  offline reconstruction differs; promote authoritative Kit layout"
      DIMCAT="$REPO/scene_gen/config/harvested/urban_building_dimensions.json"
      python3 "$REPO/scene_gen/tools/kit_dump_to_dimension_catalog.py" \
        "$KITDUMP" --out "$DIMCAT" --merge-existing "$DIMCAT" \
        || die "canonical dimension catalog (level $L)"
      cp "$DUMP" "${DUMP%.json}.offline.json" \
        || die "preserve offline layout candidate (level $L)"
      cp "$KITDUMP" "$DUMP" || die "promote Kit layout (level $L)"
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
        --target-frac '$FRAC' --epoch-s '$EPOCH' --collapse '$COLLAPSE' \
        --f6 '$F6' --seed '$CITY_SEED' --out '$MANIFEST' \
        --bake-list "$WORKLIST" \
        || die "regenerate fire plan from authoritative Kit layout (level $L)"
      python3 "$REPO/scene_gen/tools/verify_dump_matches_kit.py" \
        --offline "$DUMP" --kit "$KITDUMP" \
        || die "promoted Kit layout failed its identity gate"
    fi
    touch "$STAMP"
  fi

  # ---- Stage 4: BAKE -------------------------------------------------------
  # Host-side driver: it issues its own `docker exec` per building. It is
  # already the lazy, cache-checked bake, so the manifest IS the worklist.
  # FB_REST_STRICT=1 because "NOT AT REST is data loss, not a warning" —
  # bodies still moving at bake time are DELETED from the export and the cell
  # ships missing debris, while the driver exits 0.
  say "4/9 bake"
  [ -f "$WORKLIST" ] && python3 - "$WORKLIST" <<'PY'
import sys, json
d = json.load(open(sys.argv[1]))
print("  worklist: %d assets / %d instances lack a bake"
      % (d["n_assets"], d["n_instances"]))
PY
  FB_OUT="$FB_OUT" CONTAINER="$CONTAINER" REPO="$REPO" FB_REST_STRICT=1 \
    bash "$REPO/scene_gen/tools/fire_city_bake.sh" "$MANIFEST" ${BAKE_ARGS:-} \
      || die "bake (level $L)"
  FB_OUT="$FB_OUT" CONTAINER="$CONTAINER" REPO="$REPO" \
    bash "$REPO/scene_gen/tools/fire_city_bake.sh" "$MANIFEST" --verify-only \
      || die "bake verify (level $L)"
  N_BAKES=$(dex "$CONTAINER" bash -lc "ls -1 '$BAKES'/*.usd 2>/dev/null | wc -l" | tr -d ' \r')
  [ "${N_BAKES:-0}" != "0" ] || die "no *.usd in $BAKES — the assembly would
     compose zero bakes and build the intact city while exiting 0"
  say "  $N_BAKES bake(s) present"

  # ---- Stage 5: ASSEMBLE ---------------------------------------------------
  # `--no-window` is REQUIRED or the app segfaults in carb.eventdispatcher /
  # omni.appwindow seconds after start on a headless pod.
  say "5/9 assemble  (log: $ALOG)"
  dex "$CONTAINER" bash -lc "
      cd $REPO &&
      SCENE_CONFIG='$PRESET' \
      FC_MANIFEST='$MANIFEST' FC_DUMP='$DUMP' FC_BAKES='$BAKES' \
      FC_INTACT_ONLY=0 SNAP_DIR='$SNAPS' \
      ./python.sh simulation/isaac-sim/launch_scripts/urban_fire_city_launch_script.py --no-window
  " 2>&1 | tee "$ALOG"
  [ "${PIPESTATUS[0]}" = "0" ] || die "assemble (level $L)"

  # THE THREE THINGS THE ASSEMBLY ONLY PRINTS.
  grep -q 'Flow OOM check CLEAN' "$ALOG" \
    || die "no 'Flow OOM check CLEAN' in $ALOG — a Vulkan OOM renders a city
     with NO smoke while every count reports correct. 'Kit log not found' is
     neither a pass nor a fail and is treated as a failure here."
  grep -q 'FLOW STARVED' "$ALOG" && die "FLOW STARVED in $ALOG"
  # The offline dump is a CPU reimplementation of what the launcher builds in
  # Kit; nothing else proves the two agree, and a mismatched pair reads as
  # 'manifest/city match: 4/34' and composes bakes onto the wrong buildings.
  MATCH=$(grep -o 'manifest/city match: [0-9]*/[0-9]*' "$ALOG" | tail -1)
  if [ -n "$MATCH" ]; then
    say "  $MATCH"
    echo "$MATCH" | awk -F'[ /]' '{exit ($(NF-1) == $NF) ? 0 : 1}' \
      || die "$MATCH — the offline dump and the in-Kit city disagree"
  else
    echo "  (no 'manifest/city match' line — check $ALOG by hand)"
  fi

  # ---- Stage 6: PEOPLE -----------------------------------------------------
  # `fire_people.py` has no argparse and no __main__; calling it as a module
  # writes nothing and cannot fail. `fire_people_rerun.sh` is the one-command
  # gate that exits non-zero if any of it cannot be trusted.
  say "6/9 people"
  bash "$REPO/scene_gen/tools/fire_people_rerun.sh" "$MANIFEST" "$DUMP" "$BAKES" \
    || die "fire_people_rerun (level $L)"

  # ---- Stage 7: FREEZE -----------------------------------------------------
  # FREEZE_EXPORT=1 IS THE WHOLE POINT. It defaults to 0, in which case this
  # stage captures snaps, writes GT_people.json, exits 0 — and never writes a
  # USD. SNAP_DIR is deliberately NOT set here: the freeze writes into
  # <FREEZE_OUT>/snaps/ itself, and setting it runs FireCityApp's slower bench
  # capture pass on top.
  say "7/9 freeze -> $CELL  (log: $FLOG)"
  dex "$CONTAINER" bash -lc "
      cd $REPO &&
      SCENE_CONFIG='$PRESET' \
      FC_MANIFEST='$MANIFEST' FC_DUMP='$DUMP' FC_BAKES='$BAKES' \
      FREEZE_OUT='$CELL' FREEZE_HEADLESS=1 FREEZE_EXPORT=1 FREEZE_SNAPS=1 \
      PEOPLE_VARIANT=1 \
      ./python.sh simulation/isaac-sim/launch_scripts/freeze_urban_fire_city_launch_script.py --no-window
  " 2>&1 | tee "$FLOG"
  [ "${PIPESTATUS[0]}" = "0" ] || die "freeze (level $L)"
  grep -q 'solved ZERO burning buildings' "$FLOG" \
    && die "fire_people solved ZERO burning buildings (a print, not an exit)"

  # ---- Stage 8: GATE THE ARTEFACT ------------------------------------------
  # The portability gate raises, is caught, printed, and the process exits 0.
  # So read the verdict here. `build_local > 0` is NOT waived automatically:
  # the waiver only covers paths with a stat-verified Nucleus twin, and the
  # fire cells legitimately carry container-local soot PNGs, so a human has to
  # look before FREEZE_WAIVE_MIRRORED goes on.
  say "8/9 gate the frozen cell"
  python3 - "$CELL" "$L" <<'PY' || die "frozen cell failed its gate (level $L)"
import glob, json, os, sys
cell, lvl = sys.argv[1], sys.argv[2]
usds = glob.glob(os.path.join(cell, "*.usd"))
if not usds:
    sys.exit("no .usd in %s — FREEZE_EXPORT was not honoured" % cell)
rp = os.path.join(cell, "freeze_report.json")
if not os.path.exists(rp):
    sys.exit("no freeze_report.json in %s" % cell)
r = json.load(open(rp))
bad = []
if not r.get("portable_ok"):
    bad.append("portable_ok=False (build_local=%s cross_scope=%s unresolved=%s)"
               % (len(r.get("build_local") or []),
                  len(r.get("cross_scope_bindings") or []),
                  len(r.get("unresolved") or [])))
if not (r.get("sky_lights") or []):
    bad.append("sky_lights=0 — the cell ships unlit")
bb = r.get("bbox_m") or []
if len(bb) == 6:
    span = max(bb[3] - bb[0], bb[4] - bb[1])
    if span > 1200:
        bad.append("bbox span %.0f m — expected ~1 km; wrong preset?" % span)
if bad:
    sys.exit("; ".join(bad))
print("  ok: %s | %.0f MB | sky_lights=%d | portable_ok"
      % (os.path.basename(usds[0]), r.get("total_mb") or 0,
         len(r.get("sky_lights") or [])))
PY

  # ---- Stage 9: COLD RE-VERIFY + LAYOUT AUDIT ------------------------------
  # The in-process gate verifies a file the same process just wrote. Re-open
  # it in a FRESH process. The ComputeAllDependencies `_UnpackValue` fallback
  # is normal here, not a defect.
  say "9/9 cold verify + layout audit"
  dex "$CONTAINER" bash -lc "
      cd $REPO && python3 -c \"
from disaster import freeze
import glob, sys
u = sorted(glob.glob('$CELL/*.usd'))[0]
r = freeze.verify(u, expect_self_contained=False)
freeze.report(r)
sys.exit(0 if r.get('portable_ok') else 1)
\"" || die "cold verify (level $L)"

  # `city_layout_audit.py` has no sys.exit, so gate on its numbers, not its
  # exit code. Run it AFTER the freeze so it can read the cell's own GT.
  python3 "$REPO/scene_gen/tools/city_layout_audit.py" \
      --dump "$DUMP" --manifest "$MANIFEST" \
      --gt "$CELL/GT_hints.json" 2>&1 | tee "${LOG_DIR}/l${L}_audit.log"
  python3 - "${LOG_DIR}/l${L}_audit.log" <<'PY' || die "layout audit (level $L)"
import re, sys
txt = open(sys.argv[1]).read()
m = re.search(r"record_xy[^\n]*?max\s+([0-9.]+)\s*m", txt)
if m and float(m.group(1)) > 0.5:
    sys.exit("record_xy max %s m — a bake was composed against the wrong "
             "coordinate (pass condition is 0.00 m)" % m.group(1))
for pat, what in ((r"(\d+)\s+on road", "buildings on road"),
                  (r"(\d+)\s+empty block", "empty blocks")):
    m = re.search(pat, txt)
    if m and int(m.group(1)) > 0:
        sys.exit("%s: %s" % (what, m.group(1)))
print("  layout audit clean")
PY

  say "level ${L} done -> ${CELL}"
done

say "ALL LEVELS DONE"
