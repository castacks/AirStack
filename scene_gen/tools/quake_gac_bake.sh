#!/usr/bin/env bash
# quake_gac_bake.sh — bake the earthquake-damaged GAC/downtowncity row ONE
# BUILDING PER KIT PROCESS, sequentially. The earthquake sibling of
# `fire_bake.sh`, same shape: a manifest of entries, one `docker exec` per
# building, a success gate on the launcher's own banner, an explicit file
# list at the end.
#
#     scene_gen/tools/quake_gac_bake.sh [--dry-run] [--verify-only] [entry ...]
#
#     # the task's own three rows
#     scene_gen/tools/quake_gac_bake.sh \
#       SM_Building_02:DG5 SM_Building_12:DG4 SM_Building_23:DG3
#
#     # just print what would run
#     scene_gen/tools/quake_gac_bake.sh --dry-run
#
# ENTRY FORMAT  `name:grade[:seed]`, empty seed = the base-seed convention
#   SM_Building_02:DG5           a GreatAmericanCity asset (bare name)
#   dtc:Amar_Tower:DG3           a downtowncity block — `gac_fire.split_kind`'s
#                                `kind:name` prefix, unchanged by this script
#   SM_Building_09:DG4:19        seed pinned to 19
#
# GRADE is `quake_sliced.LEVELS` — DG0..DG5, SETTLE, TILT, OV. DG0 is the
# pristine no-op export; the interesting grades are DG1 and up.
#
# WHY ONE PROCESS PER BUILDING, SEQUENTIAL
# ------------------------------------------
# Same reasons as `fire_bake.sh`: `gac_storey_slice.slice_lock` is a
# MACHINE-WIDE `flock` (a live slice in the GUI tmux pane blocks a headless
# bake here and vice versa), and one GPU does not hold two Kit processes.
# Do not add a `&`.
#
# Env (all optional):
#   QB_OUT           container output dir  (default
#                    /isaac-sim/AirStack/scene_gen/assets/gac_quake — INSIDE
#                    the repo, unlike fire's container-only cache: these
#                    bakes are meant to ship)
#   QB_SEED          base seed; building i gets QB_SEED + 31*i, matching
#                    `fire_bake.sh`'s own column-seed convention
#                                              (default 7)
#   SETTLE_STEPS     step target per building  (default 2400)
#   SETTLE_QUIET     quiet-phase steps         (default 400)
#   SETTLE_DECOMP_M  convex-decomposition threshold, m (default 0.8)
#   SETTLE_FABRIC    1 keeps PhysX transforms in Fabric during the settle
#                                              (default 0)
#   RUBBLE_ASSET_ROOT  where `quake_rubble_usd.py` finds its debris asset
#                    library                  (default
#                    /isaac-sim/AirStack/scene_gen/assets)
#   EQ_LADDER        `quake_flow`'s ladder-table switch (`qc` default,
#                    `legacy` for the old table) — passed through so a
#                    future change to which ladder a sliced building's
#                    fit-out/roof dressing reads takes effect with no edit
#                    here; `quake_sliced.LADDER_S` itself does not consult
#                    this var today
#   EQ_RUBBLE        `v2` (default) or `v1` — same passthrough; `quake_
#                    sliced._author_pile` always calls the v2 planner today
#                    (there is no v1 branch in that module), so this only
#                    matters once one is added
#   EXTRA_ENV        further `VAR=value` pairs handed to the launcher, same
#                    mechanism `bake_quake_headless.sh` uses (`docker exec`
#                    does not inherit the host environment):
#                        EXTRA_ENV="KEEP_PHYSICS=1" scene_gen/tools/quake_gac_bake.sh
#   TIMEOUT_S        per-building ceiling      (default 5400)
#   CONTAINER        container name            (default isaac-sim)
#   REPO             repo path in container    (default /isaac-sim/AirStack)

set -u

CONTAINER=${CONTAINER:-isaac-sim}
REPO=${REPO:-/isaac-sim/AirStack}
QB_OUT=${QB_OUT:-/isaac-sim/AirStack/scene_gen/assets/gac_quake}
QB_SEED=${QB_SEED:-7}
SETTLE_STEPS=${SETTLE_STEPS:-2400}
SETTLE_QUIET=${SETTLE_QUIET:-400}
SETTLE_DECOMP_M=${SETTLE_DECOMP_M:-0.8}
SETTLE_FABRIC=${SETTLE_FABRIC:-0}
RUBBLE_ASSET_ROOT=${RUBBLE_ASSET_ROOT:-/isaac-sim/AirStack/scene_gen/assets}
EQ_LADDER=${EQ_LADDER:-qc}
EQ_RUBBLE=${EQ_RUBBLE:-v2}
EXTRA_ENV=${EXTRA_ENV:-}
TIMEOUT_S=${TIMEOUT_S:-5400}
LAUNCHER="$REPO/simulation/isaac-sim/launch_scripts/quake_gac_bake_launch_script.py"
HOSTLOGDIR="$HOME/docker/isaac-sim/logs"
CLOGDIR="/isaac-sim/.nvidia-omniverse/logs"

DRY=0
VERIFY_ONLY=0
ENTRIES=()
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=1 ;;
    --verify-only) VERIFY_ONLY=1 ;;
    -h|--help) sed -n '2,60p' "$0"; exit 0 ;;
    -*) echo "quake_gac_bake.sh: unknown flag $1" >&2; exit 2 ;;
    *) ENTRIES+=("$1") ;;
  esac
  shift
done

# THE DEFAULT ROW is the task's own three buildings, at their asked-for
# grades — a masonry total collapse, a concrete storey collapse and a mid-
# severity concrete row, one of each ladder's harder end.
if [ ${#ENTRIES[@]} -eq 0 ]; then
  ENTRIES=(SM_Building_02:DG5
           SM_Building_12:DG4
           SM_Building_23:DG3)
fi

if [ "$DRY" = 0 ] && ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "quake_gac_bake.sh: container $CONTAINER is not running" >&2
  exit 1
fi
mkdir -p "$HOSTLOGDIR" 2>/dev/null || true

# ---------------------------------------------------------------------------
# --verify-only: reopen every bake COLD (no Kit, no GPU) and check its
# materials resolve, no prim still points into the dropped source subtree, no
# physics API survived and no Flow prim shipped — `fire_bake.verify_export`,
# the same disaster-agnostic check `fire_bake.sh --verify-only` runs, via
# `test_quake_gac_bake.py --verify <dir>` (the same `--verify` CLI shape
# `test_fire_bake.py` already has).
# ---------------------------------------------------------------------------
if [ "$VERIFY_ONLY" = 1 ]; then
  CMD="$REPO/scene_gen/tools/usd_python.sh $REPO/scene_gen/tests/test_quake_gac_bake.py --verify $QB_OUT"
  echo "+ docker exec $CONTAINER bash -c \"$CMD\""
  [ "$DRY" = 1 ] && exit 0
  docker exec "$CONTAINER" bash -c "$CMD"
  exit $?
fi

echo "quake_gac_bake.sh: ${#ENTRIES[@]} building(s) -> $CONTAINER:$QB_OUT"
echo "                   seed base $QB_SEED, settle ${SETTLE_STEPS}+${SETTLE_QUIET}, decomp ${SETTLE_DECOMP_M} m"
echo

t_all=$(date +%s)
i=0
ok=0
fail=0
declare -a SUMMARY=()
declare -a USDS=()
for ent in "${ENTRIES[@]}"; do
  IFS=':' read -r NAME GRADE SEED <<EOF
$ent
EOF
  GRADE=${GRADE:-DG3}
  if [ -z "${SEED:-}" ]; then SEED=$(( QB_SEED + 31 * i )); fi
  # `fire_bake.out_stem`'s format, with kind fixed to "gac" — see the
  # launcher's own module docstring for why.
  STEM="gac_${NAME}_${GRADE}_s${SEED}"
  CLOG="$CLOGDIR/$STEM.log"
  HLOG="$HOSTLOGDIR/$STEM.log"

  ENVS="ISAAC_SIM_HEADLESS=true PYTHONUNBUFFERED=1"
  ENVS="$ENVS QB_NAME=$NAME QB_GRADE=$GRADE QB_SEED=$SEED QB_INDEX=$i QB_OUT=$QB_OUT QB_VERIFY=1"
  ENVS="$ENVS SETTLE_STEPS=$SETTLE_STEPS SETTLE_QUIET=$SETTLE_QUIET"
  ENVS="$ENVS SETTLE_DECOMP_M=$SETTLE_DECOMP_M SETTLE_FABRIC=$SETTLE_FABRIC"
  ENVS="$ENVS RUBBLE_ASSET_ROOT=$RUBBLE_ASSET_ROOT EQ_LADDER=$EQ_LADDER EQ_RUBBLE=$EQ_RUBBLE"
  ENVS="$ENVS $EXTRA_ENV"
  RUN="mkdir -p '$CLOGDIR' '$QB_OUT'; : > '$CLOG'; cd /isaac-sim && env $ENVS PYTHONPATH=\"\$ISAAC_SIM_PYTHONPATH\" timeout ${TIMEOUT_S}s /isaac-sim/python.sh $LAUNCHER --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts --no-window > '$CLOG' 2>&1; echo \"EXIT \$?\" >> '$CLOG'"

  echo "=== [$i] $ent -> $STEM"
  echo "+ docker exec $CONTAINER bash -c \"$RUN\""
  if [ "$DRY" = 1 ]; then USDS+=("$QB_OUT/$STEM.usd"); i=$((i+1)); continue; fi

  t0=$(date +%s)
  docker exec "$CONTAINER" bash -c "$RUN"
  dt=$(( $(date +%s) - t0 ))

  if docker exec "$CONTAINER" grep -q "QUAKE GAC BAKE DONE" "$CLOG" 2>/dev/null; then
    SZ=$(docker exec "$CONTAINER" bash -c "stat -c %s '$QB_OUT/$STEM.usd' 2>/dev/null || echo 0")
    MB=$(awk -v b="$SZ" 'BEGIN{printf "%.1f", b/1e6}')
    VOK=$(docker exec "$CONTAINER" grep -c "BAKE VERIFY OK" "$CLOG" 2>/dev/null || echo 0)
    STILL=$(docker exec "$CONTAINER" grep -o "still moving [0-9]*" "$CLOG" 2>/dev/null | tail -1)
    echo "    DONE in ${dt}s, ${MB} MB, verify=$([ "$VOK" -gt 0 ] && echo OK || echo PROBLEM) ${STILL:-}"
    SUMMARY+=("$(printf '%-2s %-34s %-6s %7ss %8s MB  %s' "$i" "$STEM" "$GRADE" "$dt" "$MB" "$([ "$VOK" -gt 0 ] && echo verify-OK || echo VERIFY-PROBLEM)")")
    USDS+=("$QB_OUT/$STEM.usd")
    ok=$((ok+1))
  else
    echo "    FAILED after ${dt}s — see $HLOG"
    docker exec "$CONTAINER" grep -E "Traceback|Error|QUAKE GAC BAKE FAILED|EXIT [1-9]" "$CLOG" 2>/dev/null | tail -12
    SUMMARY+=("$(printf '%-2s %-34s %-6s %7ss %8s     FAILED' "$i" "$STEM" "$GRADE" "$dt" "-")")
    fail=$((fail+1))
  fi
  i=$((i+1))
done

[ "$DRY" = 1 ] && { echo; echo "(dry run — nothing was executed)"; }

echo
echo "========================================================================"
echo "QUAKE GAC BAKE ROW   $ok ok, $fail failed, $(( $(date +%s) - t_all )) s total"
if [ ${#SUMMARY[@]} -gt 0 ]; then printf '%s\n' "${SUMMARY[@]}"; fi
echo "  bakes:    $CONTAINER:$QB_OUT"
echo "  manifest: $CONTAINER:$QB_OUT/gac_quake.json (keyed by name, grade)"
echo "  logs:     $HOSTLOGDIR/<stem>.log"
echo "------------------------------------------------------------------------"
# NAME THE FILES, NOT THE DIRECTORY — same reason `fire_bake.sh` gives: a
# re-bake at a different QB_SEED leaves the old stems in place, and an
# explicit list is what this run actually produced.
echo "Files baked this run:"
for u in "${USDS[@]-}"; do
  [ -n "$u" ] && echo "  $u"
done
echo "========================================================================"
[ "$fail" = 0 ]
